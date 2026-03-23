#pragma once
#include "shared_state.h"

/**
 * Runs the FSM thread. Blocking – call from a std::thread.
 *
 * At FSM_RATE_HZ implements the same state machine as task_fsm_node.cpp:
 *   INIT → FOLLOW_LINE_SEARCH → APPROACH_TARGET → PICKUP
 *        → SPIN_180 → RETURN_FOLLOW_LINE → DROP → DONE
 *   Any state → FAILSAFE_STOP on estop or line loss
 *
 * The FSM controls the robot by setting SharedState.control_mode and
 * writing SharedState.manual_cmd_v / manual_cmd_omega (when MANUAL),
 * and SharedState.claw_mode / claw_cmd_pending.
 */
void fsm_thread(SharedState& state);
