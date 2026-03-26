#include "encoder.h"
#include "../config.h"

#include <lgpio.h>
#include <atomic>
#include <iostream>

static std::atomic<int32_t> g_left{0};
static std::atomic<int32_t> g_right{0};
static int g_handle = -1;

// lgpio batch callback: receives an array of alert events
static void left_alerts(int num_alerts, lgGpioAlert_p alerts, void* /*userdata*/) {
    for (int i = 0; i < num_alerts; i++)
        if (alerts[i].report.level < 2)  // ignore watchdog events (level==2)
            g_left.fetch_add(1, std::memory_order_relaxed);
}

static void right_alerts(int num_alerts, lgGpioAlert_p alerts, void* /*userdata*/) {
    for (int i = 0; i < num_alerts; i++)
        if (alerts[i].report.level < 2)
            g_right.fetch_add(1, std::memory_order_relaxed);
}

void encoder_init() {
    g_handle = lgGpiochipOpen(GPIOCHIP);
    if (g_handle < 0) {
        std::cerr << "[encoder] lgGpiochipOpen(" << GPIOCHIP
                  << ") failed: " << lguErrorText(g_handle) << "\n";
        return;
    }

    for (int pin : {ENC_LEFT, ENC_RIGHT}) {
        int rc = lgGpioClaimAlert(g_handle, LG_SET_PULL_UP, LG_BOTH_EDGES, pin, -1);
        if (rc < 0)
            std::cerr << "[encoder] claim pin " << pin
                      << " failed: " << lguErrorText(rc) << "\n";
    }

    lgGpioSetAlertsFunc(g_handle, ENC_LEFT,  left_alerts,  nullptr);
    lgGpioSetAlertsFunc(g_handle, ENC_RIGHT, right_alerts, nullptr);

    std::cout << "[encoder] single-channel init – left=GPIO" << ENC_LEFT
              << "  right=GPIO" << ENC_RIGHT << "\n";
}

void encoder_cleanup() {
    if (g_handle < 0) return;
    lgGpioFree(g_handle, ENC_LEFT);
    lgGpioFree(g_handle, ENC_RIGHT);
    lgGpiochipClose(g_handle);
    g_handle = -1;
}

int32_t encoder_left()  { return g_left.load(std::memory_order_relaxed); }
int32_t encoder_right() { return g_right.load(std::memory_order_relaxed); }
