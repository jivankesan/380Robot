#pragma once
#include "shared_state.h"

// Arduino serial I/O. Runs at CMD_RATE_HZ for motor commands; also drains
// incoming telemetry (T,...) into SharedState.hw_status and sends claw
// servo angles whenever claw_cmd_pending is set. Zeroes motors on shutdown.
void serial_thread(SharedState& state);
