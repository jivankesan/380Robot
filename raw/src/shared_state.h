/**
 * shared_state.h – all data shared between threads.
 *
 * Single mutex protects everything. Lock times are microseconds at
 * these rates (100 Hz control, 50 Hz serial, 20 Hz FSM), so a
 * coarse-grained lock is fine and avoids deadlocks.
 */
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

using Clock    = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// ── Vision data (written by VisionSocketThread) ──────────────────────────────

struct LineObs {
    bool  valid            = false;
    float lateral_error_m  = 0.0f;
    float heading_error_rad = 0.0f;
    float curvature_1pm    = 0.0f;
    TimePoint timestamp    = Clock::now();
};

struct Detection {
    bool  valid     = false;
    float cx        = 0.5f;
    float cy        = 0.5f;
    float w         = 0.0f;
    float h         = 0.0f;
    float score     = 0.0f;
    // class_name always "blue_circle" for this project, omitted for simplicity
    TimePoint timestamp = Clock::now();
};

// ── Hardware status (written by SerialThread) ─────────────────────────────────

struct HwStatus {
    float   battery_v  = 12.0f;
    int32_t left_enc   = 0;
    int32_t right_enc  = 0;
    bool    estop      = false;
    TimePoint timestamp = Clock::now();
};

// ── Control modes ─────────────────────────────────────────────────────────────

enum class ControlMode {
    LINE_FOLLOW,  // PD controller uses vision line data
    MANUAL        // FSM writes manual_cmd_v / manual_cmd_omega directly
};

enum class ClawMode {
    OPEN     = 0,
    CLOSE    = 1,
    HOLD     = 2,
    ROTATE   = 3,
    UNROTATE = 4   // rotate arm back to HOME position
};

// ── Shared state ──────────────────────────────────────────────────────────────

struct SharedState {
    std::mutex mtx;

    // Vision
    LineObs   line_obs;
    Detection detection;        // blue
    Detection green_detection;  // green drop zone
    bool      target_locked = false;

    // Hardware
    HwStatus hw_status;
    bool     hw_ready = false;

    // Control
    ControlMode control_mode     = ControlMode::LINE_FOLLOW;
    double      manual_cmd_v     = 0.0;
    double      manual_cmd_omega = 0.0;

    // Final motor commands written by ControlThread, read by SerialThread
    int motor_pwm_left  = 0;
    int motor_pwm_right = 0;

    // Claw command written by FSM, read and cleared by SerialThread
    ClawMode claw_mode         = ClawMode::OPEN;
    bool     claw_cmd_pending  = false;

    // Safety estop (set by ControlThread, triggers FSM failsafe)
    bool estop = false;

    // FSM state string (for console logging only)
    std::string fsm_state = "INIT";

    // When false, vision socket thread drops DET/NODET/LOCKED messages.
    std::atomic<bool> object_detect_enabled{true};
    // When false, vision socket thread drops GREEN/NOGREEN messages.
    std::atomic<bool> green_detect_enabled{false};

    // Shutdown signal
    std::atomic<bool> shutdown{false};
};
