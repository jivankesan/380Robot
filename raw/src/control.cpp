#include "control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "../config.h"
#include "encoder.h"

using namespace std::chrono;
using namespace std::chrono_literals;

static int clamp_pwm(int v, int lo, int hi) {
  return std::max(lo, std::min(hi, v));
}

void control_thread(SharedState& state) {
  const double dt = 1.0 / CONTROL_RATE_HZ;
  const auto period = duration_cast<nanoseconds>(duration<double>(dt));
  auto next_tick = Clock::now() + period;

  // ── Line-follow PD state ─────────────────────────────────────────────────
  double last_lateral = 0.0;
  double last_heading = 0.0;
  TimePoint last_valid_line_time = Clock::now();

  // ── Speed profiler state ─────────────────────────────────────────────────
  double v_cmd = 0.0;
  double omega_cmd = 0.0;

  // ── Safety state ─────────────────────────────────────────────────────────
  TimePoint last_cmd_received = Clock::now();  // updated any time we have a non-zero cmd
  TimePoint last_hw_status_time = Clock::now();

  // ── Spin alternation state (pure-spin mode) ───────────────────────────────
  bool spin_toggle = false;
  int spin_tick_cnt = 0;

  // ── Wheel velocity PID state ──────────────────────────────────────────────
  int32_t prev_enc_l = encoder_left();
  int32_t prev_enc_r = encoder_right();
  TimePoint prev_enc_time = Clock::now();
  double pid_int_l = 0.0;
  double pid_int_r = 0.0;
  double pid_err_l = 0.0;
  double pid_err_r = 0.0;

  while (!state.shutdown.load()) {
    std::this_thread::sleep_until(next_tick);
    next_tick += period;

    // ── Read shared state ────────────────────────────────────────────────
    LineObs line;
    HwStatus hw;
    ControlMode mode;
    double manual_v, manual_omega;
    bool hw_ready;

    int direct_l = 0, direct_r = 0;
    {
      std::lock_guard<std::mutex> lk(state.mtx);
      line = state.line_obs;
      hw = state.hw_status;
      mode = state.control_mode;
      manual_v = state.manual_cmd_v;
      manual_omega = state.manual_cmd_omega;
      hw_ready = state.hw_ready;
      direct_l = state.direct_pwm_left;
      direct_r = state.direct_pwm_right;
    }

    // ── Safety checks ────────────────────────────────────────────────────
    bool hw_timeout =
      duration_cast<duration<double>>(Clock::now() - hw.timestamp).count() > SAFETY_HW_TIMEOUT_S;
    bool hw_estop = hw.estop;
    bool batt_low =
      (SAFETY_MIN_BATTERY_V > 0.0 && hw.battery_v > 0.0 && hw.battery_v < SAFETY_MIN_BATTERY_V);
    bool estop_active = hw_timeout || hw_estop || batt_low;

    if (estop_active) {
      // Write zeros and signal FSM
      std::lock_guard<std::mutex> lk(state.mtx);
      state.motor_pwm_left = 0;
      state.motor_pwm_right = 0;
      state.estop = true;
      continue;
    }

    {
      std::lock_guard<std::mutex> lk(state.mtx);
      state.estop = false;
    }

    // ── Direct PWM passthrough ───────────────────────────────────────────
    if (mode == ControlMode::DIRECT_PWM) {
      std::lock_guard<std::mutex> lk(state.mtx);
      state.motor_pwm_left = direct_l;
      state.motor_pwm_right = direct_r;
      continue;
    }

    // ── Compute raw (v, omega) ───────────────────────────────────────────
    double raw_v = 0.0;
    double raw_omega = 0.0;

    if (mode == ControlMode::LINE_FOLLOW) {
      // Track line-valid time
      if (line.valid)
        last_valid_line_time = Clock::now();
      double time_since_valid =
        duration_cast<duration<double>>(Clock::now() - last_valid_line_time).count();

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

        raw_omega = KP_LATERAL * lat + KD_LATERAL * d_lat + KP_HEADING * hdg + KD_HEADING * d_hdg;
        raw_omega = std::clamp(raw_omega, -MAX_ANG_VEL_RPS, MAX_ANG_VEL_RPS);

        double excess = std::max(0.0, std::abs(raw_omega) - TURN_OMEGA_DEADBAND);
        double brake = HEADING_BRAKE_GAIN * std::abs(hdg);
        raw_v = BASE_SPEED_MPS - brake - TURN_SPEED_GAIN * excess;
        raw_v = std::clamp(raw_v, MIN_TURN_SPEED_MPS, MAX_LIN_VEL_MPS);

        last_lateral = lat;
        last_heading = hdg;
      }
    } else {
      // MANUAL – FSM drives directly
      raw_v = manual_v;
      raw_omega = manual_omega;
      // Reset PD derivative state so re-enable is smooth
      last_lateral = 0.0;
      last_heading = 0.0;
      last_valid_line_time = Clock::now();
    }

    // ── Speed profiler ───────────────────────────────────────────────────
    // Reverse bypasses the profiler completely – apply directly
    if (raw_v < 0.0) {
      v_cmd = raw_v;
      omega_cmd = raw_omega;
    } else {
      double v_target = SP_V_MAX;

      if (line.valid) {
        double curv = std::abs(line.curvature_1pm);
        double lerr = std::abs(line.lateral_error_m);
        double herr = std::abs(line.heading_error_rad);
        v_target = SP_V_MAX - SP_K_CURVATURE * curv - SP_K_ERROR * lerr - SP_K_HEADING * herr;
      }

      if (raw_v < v_target)
        v_target = raw_v;
      v_target = std::clamp(v_target, SP_V_MIN, SP_V_MAX);

      double a_lim = (v_target < v_cmd) ? SP_A_MAX_DECEL : SP_A_MAX_ACCEL;
      double dv = std::clamp(v_target - v_cmd, -a_lim * dt, a_lim * dt);
      v_cmd = std::clamp(v_cmd + dv, 0.0, SP_V_MAX);

      double domega = std::clamp(raw_omega - omega_cmd, -SP_ALPHA_MAX * dt, SP_ALPHA_MAX * dt);
      omega_cmd += domega;

      if (raw_v == 0.0)
        v_cmd = 0.0;
    }

    // ── Twist → differential wheel PWM ──────────────────────────────────
    int pwm_l = 0, pwm_r = 0;

    if (std::abs(v_cmd) < 0.01 && std::abs(omega_cmd) > 0.1) {
      // Pure-spin mode: alternate left/right each SPIN_TICKS ticks
      // (hardware cannot drive both wheels in opposite directions simultaneously)
      int dir = (omega_cmd > 0) ? 1 : -1;
      int eff_pwm = std::clamp((int)(std::abs(omega_cmd) / 1.5 * SPIN_PWM), SPIN_MIN_PWM, SPIN_PWM);

      int left_p = LEFT_REVERSED ? (dir * eff_pwm) : (-dir * eff_pwm);
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
      spin_toggle = false;
      spin_tick_cnt = 0;

      // Target wheel angular velocities (rad/s)
      double tgt_omega_l = (v_cmd - omega_cmd * WHEEL_BASE_M / 2.0) / WHEEL_RADIUS_M;
      double tgt_omega_r = (v_cmd + omega_cmd * WHEEL_BASE_M / 2.0) / WHEEL_RADIUS_M;

      // Measure actual wheel velocities from Pi GPIO encoders
      int32_t enc_l = encoder_left();
      int32_t enc_r = encoder_right();
      auto enc_now = Clock::now();
      double enc_dt = duration_cast<duration<double>>(enc_now - prev_enc_time).count();

      double meas_omega_l = 0.0, meas_omega_r = 0.0;
      if (enc_dt > 0.002) {
        const double ticks_per_rad = ENCODER_TICKS_PER_REV / (2.0 * M_PI);
        meas_omega_l = (enc_l - prev_enc_l) / enc_dt / ticks_per_rad;
        meas_omega_r = (enc_r - prev_enc_r) / enc_dt / ticks_per_rad;
        prev_enc_l = enc_l;
        prev_enc_r = enc_r;
        prev_enc_time = enc_now;
      }

      // PID error
      double err_l = tgt_omega_l - meas_omega_l;
      double err_r = tgt_omega_r - meas_omega_r;

      // Reset integral when stopped
      if (v_cmd == 0.0 && omega_cmd == 0.0) {
        pid_int_l = 0.0;
        pid_int_r = 0.0;
      }

      pid_int_l = std::clamp(pid_int_l + err_l * dt, -MOTOR_I_CLAMP, MOTOR_I_CLAMP);
      pid_int_r = std::clamp(pid_int_r + err_r * dt, -MOTOR_I_CLAMP, MOTOR_I_CLAMP);

      double d_err_l = (err_l - pid_err_l) / dt;
      double d_err_r = (err_r - pid_err_r) / dt;
      pid_err_l = err_l;
      pid_err_r = err_r;

      double pid_out_l = MOTOR_KP * err_l + MOTOR_KI * pid_int_l + MOTOR_KD * d_err_l;
      double pid_out_r = MOTOR_KP * err_r + MOTOR_KI * pid_int_r + MOTOR_KD * d_err_r;

      // Feedforward: map target omega to PWM via empirical motor max
      const double max_wheel_omega = 22.0;  // 210 RPM * 2π/60 = 21.99 rad/s
      double ff_l =
        std::clamp(tgt_omega_l / max_wheel_omega, -1.0, 1.0) * LEFT_MOTOR_GAIN * MAX_PWM;
      double ff_r =
        std::clamp(tgt_omega_r / max_wheel_omega, -1.0, 1.0) * RIGHT_MOTOR_GAIN * MAX_PWM;

      double raw_l = ff_l + pid_out_l;
      double raw_r = ff_r + pid_out_r;

      if (LEFT_REVERSED)
        raw_l = -raw_l;
      if (RIGHT_REVERSED)
        raw_r = -raw_r;

      pwm_l = static_cast<int>(std::clamp(raw_l, -(double)MAX_PWM, (double)MAX_PWM));
      pwm_r = static_cast<int>(std::clamp(raw_r, -(double)MAX_PWM, (double)MAX_PWM));

      // During forward motion keep both wheels forward
      if (v_cmd > 0.0) {
        pwm_l = std::max(0, pwm_l);
        pwm_r = std::max(0, pwm_r);
      }

      // Enforce minimum PWM to overcome static friction
      auto apply_min = [](int p) -> int {
        if (p > 0 && p < MIN_PWM)
          return MIN_PWM;
        if (p < 0 && p > -MIN_PWM)
          return -MIN_PWM;
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
      state.motor_pwm_left = pwm_l;
      state.motor_pwm_right = pwm_r;
    }
  }

  // Zero motors on exit
  {
    std::lock_guard<std::mutex> lk(state.mtx);
    state.motor_pwm_left = 0;
    state.motor_pwm_right = 0;
  }
  std::cout << "[control] thread exited\n";
}
