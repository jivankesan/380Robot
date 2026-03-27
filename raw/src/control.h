#pragma once
#include "shared_state.h"

// PD line-follower + speed profiler → differential wheel PWM.
// Runs at CONTROL_RATE_HZ; writes motor_pwm_left/right to SharedState.
// Safety watchdogs zero the command if hw telemetry times out or estop fires.
void control_thread(SharedState& state);
