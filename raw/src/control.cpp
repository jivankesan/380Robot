#include "control.h"
#include "../config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;

static int clamp_pwm(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

void control_thread(SharedState& state) {
    const double dt = 1.0 / CONTROL_RATE_HZ;
    const auto period = duration_cast<nanoseconds>(duration<double>(dt));
    auto next_tick = Clock::now() + period;

    double last_lateral  = 0.0;
    double last_heading  = 0.0;
    TimePoint last_valid_line_time = Clock::now();

    double v_cmd     = 0.0;
    double omega_cmd = 0.0;

    TimePoint last_cmd_received   = Clock::now();
    TimePoint last_hw_status_time = Clock::now();

    bool spin_toggle   = false;
    int  spin_tick_cnt = 0;

    while (!state.shutdown.load()) {
        std::this_thread::sleep_until(next_tick);
        next_tick += period;

        LineObs   line;
        HwStatus  hw;
        ControlMode mode;
        double manual_v, manual_omega;
        bool hw_ready;

        int direct_l = 0, direct_r = 0;
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            line         = state.line_obs;
            hw           = state.hw_status;
            mode         = state.control_mode;
            manual_v     = state.manual_cmd_v;
            manual_omega = state.manual_cmd_omega;
            hw_ready     = state.hw_ready;
            direct_l     = state.direct_pwm_left;
            direct_r     = state.direct_pwm_right;
        }

        const bool safe = state.post_drop_mode.load();
        const double p_base_speed      = safe ? BASE_SPEED_MPS_SAFE      : BASE_SPEED_MPS;
        const double p_heading_brake   = safe ? HEADING_BRAKE_GAIN_SAFE  : HEADING_BRAKE_GAIN;
        const double p_turn_speed_gain = safe ? TURN_SPEED_GAIN_SAFE     : TURN_SPEED_GAIN;
        const double p_turn_deadband   = safe ? TURN_OMEGA_DEADBAND_SAFE : TURN_OMEGA_DEADBAND;
        const double p_right_gain      = safe ? RIGHT_MOTOR_GAIN_SAFE    : RIGHT_MOTOR_GAIN;
        const double p_accel           = safe ? SP_A_MAX_ACCEL_SAFE      : SP_A_MAX_ACCEL;
        const double p_decel           = safe ? SP_A_MAX_DECEL_SAFE      : SP_A_MAX_DECEL;
        const double p_alpha           = safe ? SP_ALPHA_MAX_SAFE        : SP_ALPHA_MAX;
        const double p_k_curv          = safe ? SP_K_CURVATURE_SAFE      : SP_K_CURVATURE;

        bool hw_timeout  = duration_cast<duration<double>>(Clock::now() - hw.timestamp).count()
                           > SAFETY_HW_TIMEOUT_S;
        bool hw_estop    = hw.estop;
        bool batt_low    = (SAFETY_MIN_BATTERY_V > 0.0 && hw.battery_v > 0.0
                            && hw.battery_v < SAFETY_MIN_BATTERY_V);
        bool estop_active = hw_timeout || hw_estop || batt_low;

        if (estop_active) {
            // Write zeros and signal FSM
            std::lock_guard<std::mutex> lk(state.mtx);
            state.motor_pwm_left  = 0;
            state.motor_pwm_right = 0;
            state.estop = true;
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.estop = false;
        }

        if (mode == ControlMode::DIRECT_PWM) {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.motor_pwm_left  = direct_l;
            state.motor_pwm_right = direct_r;
            continue;
        }

        double raw_v     = 0.0;
        double raw_omega = 0.0;

        if (mode == ControlMode::LINE_FOLLOW) {
            // Track line-valid time
            if (line.valid) last_valid_line_time = Clock::now();
            double time_since_valid = duration_cast<duration<double>>(
                Clock::now() - last_valid_line_time).count();

            if (!line.valid || time_since_valid > LOST_LINE_TIMEOUT_S) {
                // Lost line – stop, reset derivative state
                last_lateral = 0.0;
                last_heading = 0.0;
                raw_v = 0.0;
                raw_omega = 0.0;
            } else {
                double lat = line.lateral_error_m;
                double hdg = line.heading_error_rad;
                double d_lat = (lat - last_lateral) / dt;
                double d_hdg = (hdg - last_heading) / dt;

                raw_omega = KP_LATERAL * lat + KD_LATERAL * d_lat
                          + KP_HEADING * hdg + KD_HEADING * d_hdg;
                raw_omega = std::clamp(raw_omega, -MAX_ANG_VEL_RPS, MAX_ANG_VEL_RPS);

                double excess = std::max(0.0, std::abs(raw_omega) - p_turn_deadband);
                double brake  = p_heading_brake * std::abs(hdg);
                raw_v = p_base_speed - brake - p_turn_speed_gain * excess;
                raw_v = std::clamp(raw_v, MIN_TURN_SPEED_MPS, MAX_LIN_VEL_MPS);

                last_lateral = lat;
                last_heading = hdg;
            }
        } else {
            // MANUAL – FSM drives directly
            raw_v     = manual_v;
            raw_omega = manual_omega;
            // Reset PD derivative state so re-enable is smooth
            last_lateral = 0.0;
            last_heading = 0.0;
            last_valid_line_time = Clock::now();
        }

        // Reverse bypasses the profiler completely – apply directly
        if (raw_v < 0.0) {
            v_cmd     = raw_v;
            omega_cmd = raw_omega;
        } else {
            double v_target = SP_V_MAX;

            if (line.valid) {
                double curv = std::abs(line.curvature_1pm);
                double lerr = std::abs(line.lateral_error_m);
                double herr = std::abs(line.heading_error_rad);
                v_target = SP_V_MAX
                         - p_k_curv     * curv
                         - SP_K_ERROR   * lerr
                         - SP_K_HEADING * herr;
            }

            if (raw_v < v_target) v_target = raw_v;
            v_target = std::clamp(v_target, SP_V_MIN, SP_V_MAX);

            double a_lim = (v_target < v_cmd) ? p_decel : p_accel;
            double dv    = std::clamp(v_target - v_cmd, -a_lim * dt, a_lim * dt);
            v_cmd = std::clamp(v_cmd + dv, 0.0, SP_V_MAX);

            double domega = std::clamp(raw_omega - omega_cmd,
                                       -p_alpha * dt, p_alpha * dt);
            omega_cmd += domega;

            if (raw_v == 0.0) v_cmd = 0.0;
        }

        int pwm_l = 0, pwm_r = 0;

        if (std::abs(v_cmd) < 0.01 && std::abs(omega_cmd) > 0.1) {
            // Pure-spin mode: alternate left/right each SPIN_TICKS ticks
            // (hardware cannot drive both wheels in opposite directions simultaneously)
            int dir = (omega_cmd > 0) ? 1 : -1;
            int eff_pwm = std::clamp(
                (int)(std::abs(omega_cmd) / 1.5 * SPIN_PWM),
                SPIN_MIN_PWM, SPIN_PWM);

            int left_p  = LEFT_REVERSED  ? (dir * eff_pwm) : (-dir * eff_pwm);
            int right_p = RIGHT_REVERSED ? (dir * eff_pwm) : (-dir * eff_pwm);

            if (spin_toggle) {
                pwm_l = left_p;
                pwm_r = 0;
            } else {
                pwm_l = 0;
                pwm_r = right_p;
            }

            spin_tick_cnt++;
            if (spin_tick_cnt >= SPIN_TICKS) {
                spin_toggle = !spin_toggle;
                spin_tick_cnt = 0;
            }
        } else {
            spin_toggle   = false;
            spin_tick_cnt = 0;

            double v_left  = v_cmd - omega_cmd * WHEEL_BASE_M / 2.0;
            double v_right = v_cmd + omega_cmd * WHEEL_BASE_M / 2.0;

            double omega_left  = v_left  / WHEEL_RADIUS_M;
            double omega_right = v_right / WHEEL_RADIUS_M;

            const double max_wheel_omega = 10.0;  // ~95 RPM
            double norm_l = std::clamp(omega_left  / max_wheel_omega, -1.0, 1.0) * LEFT_MOTOR_GAIN;
            double norm_r = std::clamp(omega_right / max_wheel_omega, -1.0, 1.0) * p_right_gain;
            if (LEFT_REVERSED)  norm_l = -norm_l;
            if (RIGHT_REVERSED) norm_r = -norm_r;

            pwm_l = static_cast<int>(norm_l * MAX_PWM);
            pwm_r = static_cast<int>(norm_r * MAX_PWM);

            // During forward motion keep both wheels forward
            if (v_cmd > 0.0) {
                pwm_l = std::max(0, pwm_l);
                pwm_r = std::max(0, pwm_r);
            }

            // Enforce minimum PWM to overcome static friction
            auto apply_min = [](int p) -> int {
                if (p > 0 && p < MIN_PWM)  return MIN_PWM;
                if (p < 0 && p > -MIN_PWM) return -MIN_PWM;
                return p;
            };
            pwm_l = apply_min(pwm_l);
            pwm_r = apply_min(pwm_r);
        }

        // Clamp to hardware limits
        pwm_l = clamp_pwm(pwm_l, -MAX_PWM, MAX_PWM);
        pwm_r = clamp_pwm(pwm_r, -MAX_PWM, MAX_PWM);

        {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.motor_pwm_left  = pwm_l;
            state.motor_pwm_right = pwm_r;
        }
    }

    // Zero motors on exit
    {
        std::lock_guard<std::mutex> lk(state.mtx);
        state.motor_pwm_left  = 0;
        state.motor_pwm_right = 0;
    }
    std::cout << "[control] thread exited\n";
}
