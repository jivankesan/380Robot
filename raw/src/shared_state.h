// One mutex covers everything. Contention at 100/50/20 Hz is negligible.
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

using Clock    = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

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
    // class_name omitted – always "blue_circle" in this project
    TimePoint timestamp = Clock::now();
};

struct HwStatus {
    float   battery_v  = 12.0f;
    int32_t left_enc   = 0;
    int32_t right_enc  = 0;
    bool    estop      = false;
    TimePoint timestamp = Clock::now();
};

enum class ControlMode {
    LINE_FOLLOW,  // PD controller uses vision line data
    MANUAL,       // FSM writes manual_cmd_v / manual_cmd_omega directly
    DIRECT_PWM    // FSM writes raw PWM directly, bypasses all logic
};

enum class ClawMode {
    OPEN     = 0,
    CLOSE    = 1,
    HOLD     = 2,
    ROTATE   = 3,
    UNROTATE = 4   // rotate arm back to HOME position
};

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
    int         direct_pwm_left  = 0;   // used when control_mode == DIRECT_PWM
    int         direct_pwm_right = 0;

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

    // Set true by FSM when drop completes; switches control to safe params
    std::atomic<bool> post_drop_mode{false};

    // Set true by main thread when operator presses W; gates FSM start
    std::atomic<bool> start_requested{false};
};
