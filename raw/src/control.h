#pragma once
#include "shared_state.h"

/**
 * Runs the control thread. Blocking – call from a std::thread.
 *
 * At CONTROL_RATE_HZ this thread:
 *   1. Reads SharedState.line_obs and control_mode
 *   2. If LINE_FOLLOW: runs PD controller on lateral+heading error → raw (v, omega)
 *      If MANUAL:      uses manual_cmd_v / manual_cmd_omega directly
 *   3. Applies trapezoidal speed profiler (rate-limiting + curvature slowdown)
 *   4. Checks safety watchdogs; zeros command on fault
 *   5. Converts (v, omega) → differential wheel PWM
 *   6. Writes motor_pwm_left / motor_pwm_right to SharedState
 */
void control_thread(SharedState& state);
