#include "serial.h"
#include "../config.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;

// ── helpers ───────────────────────────────────────────────────────────────────

static int open_serial(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    speed_t speed = B115200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                     ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

static void serial_write(int fd, const std::string& s) {
    if (fd < 0) return;
    ssize_t n = write(fd, s.c_str(), s.size());
    (void)n;
}

static void send_motor(int fd, int left, int right) {
    serial_write(fd, "M," + std::to_string(left) + "," + std::to_string(right) + "\n");
}

static void send_servo(int fd, int servo_num, int angle) {
    serial_write(fd, "C," + std::to_string(servo_num) + "," + std::to_string(angle) + "\n");
}

static void apply_claw(int fd, ClawMode mode) {
    switch (mode) {
        case ClawMode::OPEN:
            send_servo(fd, 2, SERVO2_OPEN);
            send_servo(fd, 1, SERVO1_HOME);
            break;
        case ClawMode::CLOSE:
            send_servo(fd, 2, SERVO2_CLOSED);
            break;
        case ClawMode::ROTATE:
            send_servo(fd, 1, SERVO1_CARRY);
            break;
        case ClawMode::HOLD:
        default:
            break;
    }
}

// Parse one ASCII line from Arduino telemetry.
static void parse_line(const std::string& line, SharedState& state) {
    if (line.empty()) return;

    // Split by ','
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) parts.push_back(tok);

    if (parts.empty()) return;

    if (parts[0] == "T" && parts.size() >= 5) {
        // T,<battery_mv>,<left_enc>,<right_enc>,<estop>
        try {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.hw_status.battery_v  = std::stof(parts[1]) / 1000.0f;
            state.hw_status.left_enc   = std::stoi(parts[2]);
            state.hw_status.right_enc  = std::stoi(parts[3]);
            state.hw_status.estop      = (parts[4][0] == '1');
            state.hw_status.timestamp  = Clock::now();
            state.hw_ready = true;
        } catch (...) {
            std::cerr << "[serial] bad telemetry: " << line << "\n";
        }
    } else if (parts[0] == "E" && parts.size() >= 3) {
        std::cerr << "[serial] Arduino error: " << parts[2] << "\n";
    }
}

// ── main thread function ───────────────────────────────────────────────────────

void serial_thread(SharedState& state) {
    int fd = open_serial(SERIAL_PORT, SERIAL_BAUD);
    if (fd < 0) {
        std::cerr << "[serial] WARN: could not open " << SERIAL_PORT
                  << " – running without hardware\n";
    } else {
        std::cout << "[serial] opened " << SERIAL_PORT << "\n";
        // Reset claw to open/home on startup
        send_servo(fd, 2, SERVO2_OPEN);
        send_servo(fd, 1, SERVO1_HOME);
    }

    const auto cmd_period  = duration_cast<nanoseconds>(duration<double>(1.0 / CMD_RATE_HZ));
    auto next_cmd_tick     = Clock::now() + cmd_period;

    std::string read_buf;
    char raw[256];

    // Pure-spin alternation state (mirrors SerialBridgeNode)
    bool spin_toggle    = false;
    int  spin_tick_cnt  = 0;

    while (!state.shutdown.load()) {
        // ── read incoming telemetry ──────────────────────────────────────
        if (fd >= 0) {
            ssize_t n = read(fd, raw, sizeof(raw) - 1);
            if (n > 0) {
                raw[n] = '\0';
                read_buf += raw;
                size_t pos;
                while ((pos = read_buf.find('\n')) != std::string::npos) {
                    parse_line(read_buf.substr(0, pos), state);
                    read_buf.erase(0, pos + 1);
                }
            }
        }

        // ── send motor command at CMD_RATE_HZ ────────────────────────────
        auto now = Clock::now();
        if (now >= next_cmd_tick) {
            next_cmd_tick += cmd_period;

            int  pwm_l, pwm_r;
            ClawMode claw;
            bool     claw_pending;
            double   cmd_v, cmd_omega;

            {
                std::lock_guard<std::mutex> lk(state.mtx);
                pwm_l        = state.motor_pwm_left;
                pwm_r        = state.motor_pwm_right;
                claw         = state.claw_mode;
                claw_pending = state.claw_cmd_pending;
                cmd_v        = state.manual_cmd_v;
                cmd_omega    = state.manual_cmd_omega;
                if (claw_pending) state.claw_cmd_pending = false;
            }

            // Handle claw
            if (claw_pending) apply_claw(fd, claw);

            // Handle pure-spin alternation (same logic as SerialBridgeNode)
            // pwm_l == 0 && pwm_r == 0 but the ControlThread encodes a spin
            // request as a special convention: when ControlMode::MANUAL and
            // linear≈0, angular≠0 the ControlThread sets motor_pwm_* to the
            // raw spin values with alternation flags encoded in sign—but
            // actually it's cleaner to re-derive here from cmd_v/cmd_omega
            // since the serial thread owns the spin toggle state.
            //
            // Approach: ControlThread writes motor_pwm_* = INT32_MIN as a
            // sentinel for "pure spin; derive here". But that's fragile.
            // Instead: ControlThread always writes the final pwm values.
            // The spin alternation logic lives in ControlThread (see control.cpp).
            // Here we just send whatever was computed.

            send_motor(fd, pwm_l, pwm_r);
        }

        // Short sleep to avoid busy-loop on the read side
        std::this_thread::sleep_for(1ms);
    }

    // Graceful shutdown
    if (fd >= 0) {
        send_motor(fd, 0, 0);
        close(fd);
    }
    std::cout << "[serial] thread exited\n";
}
