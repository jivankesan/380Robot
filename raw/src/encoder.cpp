#include "encoder.h"
#include "../config.h"

#include <lgpio.h>
#include <atomic>
#include <iostream>

// ── Quadrature decode lookup table ───────────────────────────────────────────
// Index = (prev_AB << 2) | new_AB  where AB = (A<<1)|B
static const int8_t QEM[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

struct EncState {
    std::atomic<int32_t> count{0};
    int prev_ab = 0;
};

static EncState g_left, g_right;
static int      g_handle = -1;

// ── Callbacks ─────────────────────────────────────────────────────────────────
// lgpio delivers alerts in batches; process each in order.

static void left_alerts(int num_alerts, lgGpioAlert_p alerts, void* /*userdata*/) {
    for (int i = 0; i < num_alerts; i++) {
        int gpio  = alerts[i].report.gpio;
        int level = alerts[i].report.level;
        if (level > 1) continue;  // ignore watchdog events
        int a = (gpio == ENC_LEFT_A) ? level : lgGpioRead(g_handle, ENC_LEFT_A);
        int b = (gpio == ENC_LEFT_B) ? level : lgGpioRead(g_handle, ENC_LEFT_B);
        int new_ab = (a << 1) | b;
        g_left.count.fetch_add(QEM[(g_left.prev_ab << 2) | new_ab],
                               std::memory_order_relaxed);
        g_left.prev_ab = new_ab;
    }
}

static void right_alerts(int num_alerts, lgGpioAlert_p alerts, void* /*userdata*/) {
    for (int i = 0; i < num_alerts; i++) {
        int gpio  = alerts[i].report.gpio;
        int level = alerts[i].report.level;
        if (level > 1) continue;
        int a = (gpio == ENC_RIGHT_A) ? level : lgGpioRead(g_handle, ENC_RIGHT_A);
        int b = (gpio == ENC_RIGHT_B) ? level : lgGpioRead(g_handle, ENC_RIGHT_B);
        int new_ab = (a << 1) | b;
        g_right.count.fetch_add(QEM[(g_right.prev_ab << 2) | new_ab],
                                std::memory_order_relaxed);
        g_right.prev_ab = new_ab;
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void encoder_init() {
    g_handle = lgGpiochipOpen(GPIOCHIP);
    if (g_handle < 0) {
        std::cerr << "[encoder] lgGpiochipOpen(" << GPIOCHIP
                  << ") failed: " << lguErrorText(g_handle) << "\n";
        return;
    }

    // Claim each pin for both-edge alerts with internal pull-up
    for (int pin : {ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B}) {
        int rc = lgGpioClaimAlert(g_handle, LG_SET_PULL_UP, LG_BOTH_EDGES, pin, -1);
        if (rc < 0)
            std::cerr << "[encoder] claim pin " << pin
                      << " failed: " << lguErrorText(rc) << "\n";
    }

    // Seed initial state
    g_left.prev_ab  = (lgGpioRead(g_handle, ENC_LEFT_A)  << 1) | lgGpioRead(g_handle, ENC_LEFT_B);
    g_right.prev_ab = (lgGpioRead(g_handle, ENC_RIGHT_A) << 1) | lgGpioRead(g_handle, ENC_RIGHT_B);

    lgGpioSetAlertsFunc(g_handle, ENC_LEFT_A,  left_alerts,  nullptr);
    lgGpioSetAlertsFunc(g_handle, ENC_LEFT_B,  left_alerts,  nullptr);
    lgGpioSetAlertsFunc(g_handle, ENC_RIGHT_A, right_alerts, nullptr);
    lgGpioSetAlertsFunc(g_handle, ENC_RIGHT_B, right_alerts, nullptr);

    std::cout << "[encoder] init on gpiochip" << GPIOCHIP
              << " – left A=" << ENC_LEFT_A << " B=" << ENC_LEFT_B
              << "  right A=" << ENC_RIGHT_A << " B=" << ENC_RIGHT_B << "\n";
}

void encoder_cleanup() {
    if (g_handle < 0) return;
    for (int pin : {ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B})
        lgGpioFree(g_handle, pin);
    lgGpiochipClose(g_handle);
    g_handle = -1;
}

int32_t encoder_left()  { return g_left.count.load(std::memory_order_relaxed); }
int32_t encoder_right() { return g_right.count.load(std::memory_order_relaxed); }
