#include "encoder.h"
#include "../config.h"

#include <pigpio.h>
#include <atomic>
#include <iostream>

// ── Quadrature decode lookup table ───────────────────────────────────────────
//
// Index = (prev_AB << 2) | new_AB  where AB = (A<<1)|B
// Forward (CW) Gray-code sequence: 00→01→11→10→00
//
static const int8_t QEM[16] = {
     0,  1, -1,  0,   // 00→{00,01,10,11}
    -1,  0,  0,  1,   // 01→{00,01,10,11}
     1,  0,  0, -1,   // 10→{00,01,10,11}
     0, -1,  1,  0    // 11→{00,01,10,11}
};

// ── Per-encoder state ─────────────────────────────────────────────────────────
struct EncState {
    std::atomic<int32_t> count{0};
    int prev_ab = 0;  // only written from pigpio callback thread
};

static EncState g_left, g_right;

// ── Callbacks ─────────────────────────────────────────────────────────────────
//
// pigpio calls these on any edge of the registered GPIO.
// 'level' is the new state of the triggering pin; we gpioRead the partner pin.
// All callbacks for one encoder fire on the same pigpio internal thread,
// so prev_ab is safe without a mutex.

static void left_cb(int gpio, int level, uint32_t /*tick*/) {
    int a = (gpio == ENC_LEFT_A) ? level : gpioRead(ENC_LEFT_A);
    int b = (gpio == ENC_LEFT_B) ? level : gpioRead(ENC_LEFT_B);
    int new_ab = (a << 1) | b;
    g_left.count.fetch_add(QEM[(g_left.prev_ab << 2) | new_ab],
                           std::memory_order_relaxed);
    g_left.prev_ab = new_ab;
}

static void right_cb(int gpio, int level, uint32_t /*tick*/) {
    int a = (gpio == ENC_RIGHT_A) ? level : gpioRead(ENC_RIGHT_A);
    int b = (gpio == ENC_RIGHT_B) ? level : gpioRead(ENC_RIGHT_B);
    int new_ab = (a << 1) | b;
    g_right.count.fetch_add(QEM[(g_right.prev_ab << 2) | new_ab],
                            std::memory_order_relaxed);
    g_right.prev_ab = new_ab;
}

// ── Public API ────────────────────────────────────────────────────────────────

void encoder_init() {
    for (int pin : {ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B}) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDownResistor(pin, PI_PUD_UP);
    }

    // Seed prev_ab from current pin state so first transition is correct
    g_left.prev_ab  = (gpioRead(ENC_LEFT_A)  << 1) | gpioRead(ENC_LEFT_B);
    g_right.prev_ab = (gpioRead(ENC_RIGHT_A) << 1) | gpioRead(ENC_RIGHT_B);

    gpioSetAlertFunc(ENC_LEFT_A,  left_cb);
    gpioSetAlertFunc(ENC_LEFT_B,  left_cb);
    gpioSetAlertFunc(ENC_RIGHT_A, right_cb);
    gpioSetAlertFunc(ENC_RIGHT_B, right_cb);

    std::cout << "[encoder] init – left A=GPIO" << ENC_LEFT_A
              << " B=GPIO" << ENC_LEFT_B
              << "  right A=GPIO" << ENC_RIGHT_A
              << " B=GPIO" << ENC_RIGHT_B << "\n";
}

void encoder_cleanup() {
    gpioSetAlertFunc(ENC_LEFT_A,  nullptr);
    gpioSetAlertFunc(ENC_LEFT_B,  nullptr);
    gpioSetAlertFunc(ENC_RIGHT_A, nullptr);
    gpioSetAlertFunc(ENC_RIGHT_B, nullptr);
}

int32_t encoder_left()  { return g_left.count.load(std::memory_order_relaxed); }
int32_t encoder_right() { return g_right.count.load(std::memory_order_relaxed); }
