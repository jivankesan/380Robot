#pragma once
#include "shared_state.h"

// Mission state machine. Runs at FSM_RATE_HZ.
//
// INIT → FOLLOW_LINE_SEARCH → PICKUP → SPIN_180
//      → RETURN_FOLLOW_LINE → APPROACH_DROP_ZONE → DROP
//      → FIND_LINE → FINAL_FOLLOW → DONE
//
// Any state → FAILSAFE_STOP on estop or line loss.
// Controls the robot via SharedState.control_mode and claw_cmd_pending.
void fsm_thread(SharedState& state);
