#pragma once
#include <cstdint>

/**
 * encoder.h – Pi-side quadrature encoder reader via pigpio GPIO interrupts.
 *
 * Call encoder_init() once after gpioInitialise().
 * Call encoder_cleanup() before gpioTerminate().
 * encoder_left() / encoder_right() return cumulative signed tick counts.
 */

void    encoder_init();
void    encoder_cleanup();
int32_t encoder_left();
int32_t encoder_right();
