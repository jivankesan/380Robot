/**
 * main.cpp – entry point.
 *
 * Starts the vision.py subprocess, creates the Unix domain socket that
 * vision.py writes to, then launches three threads and waits.
 *
 * Thread layout:
 *   vision_socket_thread  – receives UDP datagrams from vision.py, updates SharedState
 *   serial_thread         – sends motor/claw commands to Arduino, reads telemetry
 *   control_thread        – PD line follow + speed profile → motor PWM
 *   fsm_thread            – task state machine
 *
 * Usage:
 *   ./robot [camera_source]
 *
 *   camera_source: integer (e.g. 0 for /dev/video0) or MJPEG URL.
 *                  Passed to vision.py as its first argument.
 *                  Defaults to 0 if omitted.
 */

#include "shared_state.h"
#include "serial.h"
#include "control.h"
#include "fsm.h"
#include "../config.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ── Global shutdown flag (set by SIGINT handler) ─────────────────────────────
static SharedState* g_state_ptr = nullptr;

static void sigint_handler(int) {
    std::cout << "\n[main] SIGINT – shutting down\n";
    if (g_state_ptr) g_state_ptr->shutdown.store(true);
}

// ── Vision socket thread ─────────────────────────────────────────────────────
// Receives lines from vision.py via a Unix domain SOCK_DGRAM socket.
// Protocol:
//   LINE,<valid>,<lateral_m>,<heading_rad>,<curvature_1pm>
//   DET,<cx>,<cy>,<w>,<h>,<score>
//   LOCKED

static void parse_vision_message(const char* buf, SharedState& state) {
    std::string line(buf);

    // Split by ','
    std::vector<std::string> tok;
    std::stringstream ss(line);
    std::string t;
    while (std::getline(ss, t, ',')) tok.push_back(t);
    if (tok.empty()) return;

    if (tok[0] == "LINE" && tok.size() >= 5) {
        std::lock_guard<std::mutex> lk(state.mtx);
        state.line_obs.valid             = (tok[1] == "1");
        state.line_obs.lateral_error_m   = std::stof(tok[2]);
        state.line_obs.heading_error_rad = std::stof(tok[3]);
        state.line_obs.curvature_1pm     = std::stof(tok[4]);
        state.line_obs.timestamp         = Clock::now();
    } else if (tok[0] == "DET" && tok.size() >= 6) {
        if (!state.object_detect_enabled.load()) return;
        std::lock_guard<std::mutex> lk(state.mtx);
        state.detection.valid     = true;
        state.detection.cx        = std::stof(tok[1]);
        state.detection.cy        = std::stof(tok[2]);
        state.detection.w         = std::stof(tok[3]);
        state.detection.h         = std::stof(tok[4]);
        state.detection.score     = std::stof(tok[5]);
        state.detection.timestamp = Clock::now();
    } else if (tok[0] == "LOCKED") {
        if (!state.object_detect_enabled.load()) return;
        std::lock_guard<std::mutex> lk(state.mtx);
        state.target_locked = true;
    } else if (tok[0] == "NODET") {
        if (!state.object_detect_enabled.load()) return;
        std::lock_guard<std::mutex> lk(state.mtx);
        state.detection.valid = false;
    } else if (tok[0] == "GREEN" && tok.size() >= 6) {
        if (!state.green_detect_enabled.load()) return;
        std::lock_guard<std::mutex> lk(state.mtx);
        state.green_detection.valid     = true;
        state.green_detection.cx        = std::stof(tok[1]);
        state.green_detection.cy        = std::stof(tok[2]);
        state.green_detection.w         = std::stof(tok[3]);
        state.green_detection.h         = std::stof(tok[4]);
        state.green_detection.score     = std::stof(tok[5]);
        state.green_detection.timestamp = Clock::now();
    } else if (tok[0] == "NOGREEN") {
        if (!state.green_detect_enabled.load()) return;
        std::lock_guard<std::mutex> lk(state.mtx);
        state.green_detection.valid = false;
    }
}

static void vision_socket_thread(SharedState& state) {
    // Remove stale socket file
    unlink(VISION_SOCKET_PATH);

    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[vision_sock] failed to create socket\n";
        return;
    }

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VISION_SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[vision_sock] bind failed\n";
        close(fd);
        return;
    }

    // Set receive timeout so we can check shutdown flag
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 50000;  // 50 ms
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::cout << "[vision_sock] listening on " << VISION_SOCKET_PATH << "\n";
    char buf[512];

    while (!state.shutdown.load()) {
        ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            buf[n] = '\0';
            // Strip trailing newline/whitespace
            while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r' || buf[n-1] == ' '))
                buf[--n] = '\0';
            try {
                parse_vision_message(buf, state);
            } catch (...) {
                std::cerr << "[vision_sock] parse error: " << buf << "\n";
            }
        }
    }

    close(fd);
    unlink(VISION_SOCKET_PATH);
    std::cout << "[vision_sock] thread exited\n";
}

// ── Launch vision.py as subprocess ────────────────────────────────────────────

static pid_t launch_vision(const std::string& camera_src) {
    // Resolve the directory containing the robot binary at runtime using
    // /proc/self/exe — avoids relying on __FILE__ which is the compile-time
    // path on the build machine and wrong when deployed to the Pi.
    char exe_buf[4096] = {};
    ssize_t len = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
    if (len < 0) {
        perror("[main] readlink /proc/self/exe failed");
        return -1;
    }
    std::string exe_path(exe_buf, len);

    // exe is at .../raw/robot  →  go up one level to .../raw/
    size_t pos = exe_path.rfind('/');
    std::string base_dir = (pos != std::string::npos)
                           ? exe_path.substr(0, pos)
                           : ".";
    std::string script_path = base_dir + "/vision/vision.py";

    pid_t pid = fork();
    if (pid == 0) {
        execl("/usr/bin/python3", "python3", script_path.c_str(),
              camera_src.c_str(), nullptr);
        perror("[main] execl vision.py failed");
        _exit(1);
    }
    return pid;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::string camera_src = "0";
    if (argc >= 2) camera_src = argv[1];

    SharedState state;
    g_state_ptr = &state;

    // Register signal handler
    struct sigaction sa{};
    sa.sa_handler = sigint_handler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Start vision socket thread first so the socket exists before vision.py connects
    std::thread t_sock(vision_socket_thread, std::ref(state));
    std::this_thread::sleep_for(100ms);  // give socket thread time to bind

    // Launch vision.py
    pid_t vision_pid = launch_vision(camera_src);
    if (vision_pid < 0) {
        std::cerr << "[main] fork failed – continuing without vision\n";
    } else {
        std::cout << "[main] vision.py launched (pid=" << vision_pid << ")\n";
    }

    // Start C++ threads
    std::thread t_serial (serial_thread,  std::ref(state));
    std::thread t_control(control_thread, std::ref(state));
    std::thread t_fsm    (fsm_thread,     std::ref(state));

    std::cout << "[main] all threads running – press Ctrl+C to stop\n";

    // Wait for shutdown
    while (!state.shutdown.load()) {
        std::this_thread::sleep_for(100ms);
    }

    // Kill vision subprocess
    if (vision_pid > 0) {
        kill(vision_pid, SIGTERM);
        waitpid(vision_pid, nullptr, 0);
    }

    // Join threads
    t_fsm.join();
    t_control.join();
    t_serial.join();
    t_sock.join();

    std::cout << "[main] clean shutdown\n";
    return 0;
}
