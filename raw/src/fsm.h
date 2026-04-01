#pragma once
#include "shared_state.h"

// States: INIT → FOLLOW_LINE_SEARCH → PICKUP → SPIN_180
//       → RETURN_FOLLOW_LINE → APPROACH_DROP_ZONE → DROP
//       → FIND_LINE → FINAL_FOLLOW → DONE
// Any state → FAILSAFE_STOP on estop or sustained line loss.
void fsm_thread(SharedState& state);
