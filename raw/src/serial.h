#pragma once
#include "shared_state.h"

/**
 * Runs the serial I/O thread. Blocking – call from a std::thread.
 *
 * Responsibilities:
 *  - Open serial port to Arduino
 *  - At CMD_RATE_HZ: send M,<left_pwm>,<right_pwm>\n  (reads SharedState.motor_pwm_*)
 *  - When claw_cmd_pending: send C,<servo>,<angle>\n   (reads SharedState.claw_mode)
 *  - Read incoming T,... lines and update SharedState.hw_status
 *  - On shutdown: send M,0,0 and close fd
 */
void serial_thread(SharedState& state);
