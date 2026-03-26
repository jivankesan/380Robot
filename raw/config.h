/**
 * config.h – all tunable parameters in one place.
 * Replaces ROS2 YAML parameter files. Edit here, recompile.
 */
#pragma once

// ── Serial ───────────────────────────────────────────────────────────────────
static constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
static constexpr int SERIAL_BAUD = 115200;
static constexpr int WATCHDOG_MS = 250;
static constexpr double CMD_RATE_HZ = 50.0;
static constexpr double TELEMETRY_RATE_HZ = 20.0;

// Robot geometry
static constexpr double WHEEL_BASE_M = 0.15;
static constexpr double WHEEL_RADIUS_M = 0.048;  // 96 mm diameter
// static constexpr double MOTOR_MAX_RPM  = 210.0;
// Derived: max wheel omega = MOTOR_MAX_RPM * 2π / 60 ≈ 21.99 rad/s
// Derived: max linear speed = max_wheel_omega * WHEEL_RADIUS_M ≈ 1.056 m/s

// PWM limits
static constexpr int MAX_PWM = 255;
static constexpr int MIN_PWM = 30;       // deadband override
static constexpr int SPIN_PWM = 50;      // full-spin PWM
static constexpr int SPIN_MIN_PWM = 20;  // minimum spin PWM

// Motor calibration – reduce the gain of the faster wheel until both match
static constexpr double LEFT_MOTOR_GAIN = 1.0;
static constexpr double RIGHT_MOTOR_GAIN = 1.0;
static constexpr bool LEFT_REVERSED = false;
static constexpr bool RIGHT_REVERSED = false;

// Servo angles (sent as C,<servo_num>,<angle>\n to Arduino)
static constexpr int SERVO1_HOME = 35;     // rotation arm: horizontal/resting
static constexpr int SERVO1_CARRY = 90;    // rotation arm: rotated carry
static constexpr int SERVO2_OPEN = 50;     // gripper: open
static constexpr int SERVO2_CLOSED = 130;  // gripper: closed

// ── Line-follow PD controller ────────────────────────────────────────────────
// ROI is now a lookahead window (y: 0.10–0.60) so errors are larger and
// arrive earlier. Gains raised accordingly; KD raised for damping to prevent
// overshoot from acting on predicted-future error rather than current error.
static constexpr double CONTROL_RATE_HZ = 100.0;
static constexpr double KP_LATERAL = 1.1;
static constexpr double KD_LATERAL = 3.0;
static constexpr double KP_HEADING = 0.9;
static constexpr double KD_HEADING = 3.5;
static constexpr double BASE_SPEED_MPS = 0.35;
static constexpr double MAX_LIN_VEL_MPS = 0.5;  // MOTOR_MAX_RPM * 2π/60 * WHEEL_RADIUS_M
static constexpr double MIN_LIN_VEL_MPS = 0.08;
static constexpr double MAX_ANG_VEL_RPS = 2.0;
static constexpr double HEADING_BRAKE_GAIN = 2.0;  // was 1.5  – brake harder on curves
static constexpr double TURN_SPEED_GAIN = 5.0;     // was 4.0
static constexpr double MIN_TURN_SPEED_MPS = 0.12;
static constexpr double TURN_OMEGA_DEADBAND = 0.15;
static constexpr double LOST_LINE_TIMEOUT_S = 0.2;

// ── Speed profiler ───────────────────────────────────────────────────────────
static constexpr double SP_V_MAX = 0.5;  // MOTOR_MAX_RPM * 2π/60 * WHEEL_RADIUS_M
static constexpr double SP_V_MIN = 0.1;
static constexpr double SP_A_MAX_ACCEL = 3.0;
static constexpr double SP_A_MAX_DECEL = 8.0;  // was 6.0  – brake faster into turns
static constexpr double SP_ALPHA_MAX = 6.0;    // was 4.0  – angular rate can change faster
static constexpr double SP_K_CURVATURE = 0.4;  // was 0.3
static constexpr double SP_K_ERROR = 0.4;      // was 0.3
static constexpr double SP_K_HEADING = 0.4;    // was 0.3

// ── Safety ───────────────────────────────────────────────────────────────────
static constexpr double SAFETY_CMD_TIMEOUT_S = 0.5;
static constexpr double SAFETY_HW_TIMEOUT_S = 1.0;
static constexpr double SAFETY_MIN_BATTERY_V = 0.0;  // 0 = disabled

// ── Green drop-zone detector ─────────────────────────────────────────────────
static constexpr int GREEN_H_MIN = 82;  // bright cyan-teal (as seen in image)
static constexpr int GREEN_H_MAX = 97;
static constexpr int GREEN_S_MIN = 150;  // very saturated only
static constexpr int GREEN_S_MAX = 255;
static constexpr int GREEN_V_MIN = 120;  // bright only
static constexpr int GREEN_V_MAX = 255;
static constexpr int GREEN_MIN_AREA_PX =
  15000;  // only trigger when box fills large portion of frame
static constexpr double GREEN_ROI_Y_END = 0.70;  // top 70% – box spans past halfway when close

// ── FSM ──────────────────────────────────────────────────────────────────────
static constexpr double FSM_RATE_HZ = 20.0;
static constexpr double PICKUP_DRIVE_TIME_S = 0.28;     // drive forward after blue seen
static constexpr double PICKUP_DRIVE_SPEED_MPS = 0.15;  // slow creep toward target
static constexpr double PICKUP_CLOSE_TIME_S = 1.0;
static constexpr double PICKUP_ROTATE_TIME_S = 1.0;
static constexpr double PICKUP_SPIN_TIME_S = 1.1;
static constexpr double PICKUP_SPIN_OMEGA_RPS = 1.5;
static constexpr double LINE_LOSS_TIMEOUT_S = 3.0;

// Drop zone approach: turn right 30deg, then drive forward into box
// Drop zone: stop, turn hard right, then drop
static constexpr double DROP_ZONE_TURN_OMEGA_RPS = -PICKUP_SPIN_OMEGA_RPS;  // opposite of 180 spin
static constexpr double DROP_ZONE_TURN_TIME_S = 0.55;                       // tune to adjust angle

// Drop sequence
static constexpr double DROP_UNROTATE_TIME_S = 1.0;  // time to rotate arm back to HOME
static constexpr double DROP_OPEN_TIME_S = 1.0;      // time to open gripper

// Find line after drop (reverse briefly, then turn left until red line seen)
static constexpr double FIND_LINE_REVERSE_TIME_S = 0.8;  // reverse to clear dropped package
static constexpr double FIND_LINE_OMEGA_RPS = 1.2;       // positive = left turn
static constexpr double FIND_LINE_TIMEOUT_S = 6.0;       // failsafe if line never found

// Approach (vision-based drive toward blue circle – unused in current flow, kept for reference)
static constexpr double APPROACH_SPEED_MPS = 0.15;
static constexpr double APPROACH_KP_ANGULAR = 1.5;
static constexpr double APPROACH_TOP_TOL = 0.15;
static constexpr double APPROACH_CENTER_TOL_X = 0.20;
static constexpr double APPROACH_ALIGN_GATE_X = 0.15;
static constexpr double APPROACH_DET_TIMEOUT_S = 0.5;
static constexpr double APPROACH_MAX_ANG_VEL = 2.0;
static constexpr double APPROACH_MIN_CIRCLE_H = 0.40;

// ── IPC with vision.py ───────────────────────────────────────────────────────
// vision.py sends UDP datagrams to C++ on this Unix domain socket path.
static constexpr const char* VISION_SOCKET_PATH = "/tmp/robot_vision.sock";
