#include "fsm.h"
#include "../config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;

// ── State enum ────────────────────────────────────────────────────────────────

enum class State {
    INIT,
    FOLLOW_LINE_SEARCH,
    APPROACH_TARGET,
    PICKUP,
    SPIN_180,
    RETURN_FOLLOW_LINE,
    DROP,
    DONE,
    FAILSAFE_STOP
};

static std::string to_str(State s) {
    switch (s) {
        case State::INIT:               return "INIT";
        case State::FOLLOW_LINE_SEARCH: return "FOLLOW_LINE_SEARCH";
        case State::APPROACH_TARGET:    return "APPROACH_TARGET";
        case State::PICKUP:             return "PICKUP";
        case State::SPIN_180:           return "SPIN_180";
        case State::RETURN_FOLLOW_LINE: return "RETURN_FOLLOW_LINE";
        case State::DROP:               return "DROP";
        case State::DONE:               return "DONE";
        case State::FAILSAFE_STOP:      return "FAILSAFE_STOP";
        default:                        return "UNKNOWN";
    }
}

// ── FSM context (local to this function) ──────────────────────────────────────

struct FsmCtx {
    State     current  = State::INIT;
    TimePoint state_start = Clock::now();

    // Cached copies read from SharedState each tick
    LineObs   line;
    Detection det;
    bool      target_locked = false;
    bool      hw_ready      = false;
    bool      estop         = false;
    TimePoint last_line_valid_time    = Clock::now();
    TimePoint last_detection_time     = Clock::now();
    // Set to true the first time a valid LINE is seen in FOLLOW_LINE_SEARCH.
    // The line-loss check is suppressed until then so vision startup delay
    // (rpicam-vid init, first frame decode) doesn't trigger a false failsafe.
    bool line_ever_seen = false;

    void transition(State next) {
        if (next == current) return;
        std::cout << "[fsm] " << to_str(current) << " -> " << to_str(next) << "\n";
        // Reset line-loss timer whenever entering a state that checks it,
        // so startup/spin delays don't immediately trigger failsafe.
        if (next == State::FOLLOW_LINE_SEARCH ||
            next == State::RETURN_FOLLOW_LINE ||
            next == State::SPIN_180) {
            last_line_valid_time = Clock::now();
        }
        // Reset first-seen flag when re-entering search so a second pass
        // also waits for the first valid line before loss-checking.
        if (next == State::FOLLOW_LINE_SEARCH) {
            line_ever_seen = false;
        }
        current     = next;
        state_start = Clock::now();
    }

    double state_elapsed() const {
        return duration_cast<duration<double>>(Clock::now() - state_start).count();
    }
};

// ── Helpers that write to SharedState ─────────────────────────────────────────

static void set_line_follow(SharedState& s) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.control_mode = ControlMode::LINE_FOLLOW;
}

static void set_manual(SharedState& s, double v, double omega) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.control_mode     = ControlMode::MANUAL;
    s.manual_cmd_v     = v;
    s.manual_cmd_omega = omega;
}

static void stop(SharedState& s) {
    set_manual(s, 0.0, 0.0);
}

static void send_claw(SharedState& s, ClawMode mode) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.claw_mode        = mode;
    s.claw_cmd_pending = true;
}

// ── State handlers ────────────────────────────────────────────────────────────

static void handle_init(FsmCtx& ctx, SharedState& state) {
    stop(state);
    bool line_ok = ctx.line.valid;
    if ((line_ok && ctx.hw_ready) || ctx.state_elapsed() > 10.0) {
        std::cout << "[fsm] systems ready, starting mission\n";
        ctx.transition(State::FOLLOW_LINE_SEARCH);
    }
}

static void handle_follow_line_search(FsmCtx& ctx, SharedState& state) {
    set_line_follow(state);
    // Ensure detection is enabled when searching (in case of re-entry)
    if (!state.object_detect_enabled.load()) {
        state.object_detect_enabled.store(true);
        std::cout << "[fsm] object detection re-enabled\n";
    }
    // Any blue blob seen → stop and pick up immediately.
    if (ctx.det.valid) {
        std::cout << "[fsm] blue detected (cx=" << ctx.det.cx
                  << " cy=" << ctx.det.cy << " h=" << ctx.det.h
                  << ") – driving forward for " << PICKUP_DRIVE_TIME_S << "s then closing\n";
        state.object_detect_enabled.store(false);
        std::cout << "[fsm] object detection disabled\n";
        ctx.transition(State::PICKUP);
    }
}

static void handle_approach_target(FsmCtx& ctx, SharedState& state) {
    // Suppress line follower; FSM drives manually
    const Detection& d = ctx.det;

    if (!d.valid) {
        double lost_for = duration_cast<duration<double>>(
            Clock::now() - ctx.last_detection_time).count();
        if (lost_for > APPROACH_DET_TIMEOUT_S) {
            stop(state);
        }
        return;
    }

    double top_y   = d.cy - d.h / 2.0;
    double error_x = d.cx - 0.5;

    bool circle_close_enough = d.h >= APPROACH_MIN_CIRCLE_H;
    bool vertically_close    = std::abs(top_y - 0.5) < APPROACH_TOP_TOL;
    bool horizontally_ok     = std::abs(error_x) < APPROACH_CENTER_TOL_X;
    bool overshot            = top_y > (0.5 + APPROACH_TOP_TOL);

    if (overshot || (circle_close_enough && vertically_close && horizontally_ok)) {
        std::cout << "[fsm] approach complete (top_y=" << top_y
                  << " h=" << d.h << ") – picking up\n";
        stop(state);
        ctx.transition(State::PICKUP);
        return;
    }

    bool aligned = std::abs(error_x) < APPROACH_ALIGN_GATE_X;
    double linear  = (aligned && top_y < 0.5) ? APPROACH_SPEED_MPS : 0.0;
    double angular = std::clamp(-APPROACH_KP_ANGULAR * error_x,
                                -APPROACH_MAX_ANG_VEL, APPROACH_MAX_ANG_VEL);
    set_manual(state, linear, angular);
}

static void handle_pickup(FsmCtx& ctx, SharedState& state) {
    double t        = ctx.state_elapsed();
    double t_close  = PICKUP_DRIVE_TIME_S;
    double t_rotate = PICKUP_DRIVE_TIME_S + PICKUP_CLOSE_TIME_S;
    double t_done   = PICKUP_DRIVE_TIME_S + PICKUP_CLOSE_TIME_S + PICKUP_ROTATE_TIME_S;

    static double last_log = -1.0;
    if (t - last_log >= 0.5) {
        last_log = t;
        if (t < t_close)
            std::cout << "[fsm] PICKUP phase 1/3: driving forward (t=" << t << "s / " << t_close << "s)\n";
        else if (t < t_rotate)
            std::cout << "[fsm] PICKUP phase 2/3: closing gripper (t=" << t << "s / " << t_rotate << "s)\n";
        else if (t < t_done)
            std::cout << "[fsm] PICKUP phase 3/3: rotating arm (t=" << t << "s / " << t_done << "s)\n";
    }

    if (t < t_close) {
        set_manual(state, PICKUP_DRIVE_SPEED_MPS, 0.0);
    } else if (t < t_rotate) {
        stop(state);
        send_claw(state, ClawMode::CLOSE);
    } else if (t < t_done) {
        send_claw(state, ClawMode::ROTATE);
    } else {
        last_log = -1.0;
        std::cout << "[fsm] claw secured – spinning 180\n";
        ctx.transition(State::SPIN_180);
    }
}

static void handle_spin180(FsmCtx& ctx, SharedState& state) {
    double t = ctx.state_elapsed();
    if (t < PICKUP_SPIN_TIME_S) {
        set_manual(state, 0.0, PICKUP_SPIN_OMEGA_RPS);
        static double last_spin_log = -1.0;
        if (t - last_spin_log >= 0.5) {
            last_spin_log = t;
            std::cout << "[fsm] SPIN_180: t=" << t << "s / " << PICKUP_SPIN_TIME_S << "s\n";
        }
    } else {
        stop(state);
        std::cout << "[fsm] spin complete – returning to line\n";
        ctx.transition(State::RETURN_FOLLOW_LINE);
    }
}

static void handle_return_follow_line(FsmCtx& ctx, SharedState& state) {
    set_line_follow(state);
    send_claw(state, ClawMode::HOLD);
    double t = ctx.state_elapsed();
    static double last_ret_log = -1.0;
    if (t - last_ret_log >= 2.0) {
        last_ret_log = t;
        std::cout << "[fsm] RETURN: t=" << t << "s / " << RETURN_TIME_S
                  << "s  line_valid=" << ctx.line.valid << "\n";
    }
    if (t > RETURN_TIME_S) {
        last_ret_log = -1.0;
        std::cout << "[fsm] return time elapsed – dropping\n";
        ctx.transition(State::DROP);
    }
}

static void handle_drop(FsmCtx& ctx, SharedState& state) {
    stop(state);
    send_claw(state, ClawMode::OPEN);
    if (ctx.state_elapsed() > DROP_OPEN_TIME_S) {
        std::cout << "[fsm] mission complete!\n";
        ctx.transition(State::DONE);
    }
}

static void handle_done(FsmCtx& /*ctx*/, SharedState& state) {
    stop(state);
}

static void handle_failsafe(FsmCtx& /*ctx*/, SharedState& state) {
    stop(state);
    send_claw(state, ClawMode::OPEN);
}

// ── Main thread function ───────────────────────────────────────────────────────

void fsm_thread(SharedState& state) {
    const double dt = 1.0 / FSM_RATE_HZ;
    const auto period = duration_cast<nanoseconds>(duration<double>(dt));
    auto next_tick = Clock::now() + period;

    FsmCtx ctx;

    while (!state.shutdown.load()) {
        std::this_thread::sleep_until(next_tick);
        next_tick += period;

        // ── Read shared state ────────────────────────────────────────────────
        bool target_locked_snapshot;
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            ctx.line          = state.line_obs;
            ctx.det           = state.detection;
            ctx.hw_ready      = state.hw_ready;
            ctx.estop         = state.estop;
            target_locked_snapshot = state.target_locked;
            // Consume target_locked (one-shot)
            if (state.target_locked) state.target_locked = false;
        }

        // Update cached times
        if (ctx.line.valid)  ctx.last_line_valid_time  = Clock::now();
        if (ctx.det.valid)   ctx.last_detection_time   = Clock::now();
        ctx.target_locked = target_locked_snapshot;

        // Update FSM state string for logging
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.fsm_state = to_str(ctx.current);
        }

        // ── Global failsafe checks ───────────────────────────────────────────
        if (ctx.estop && ctx.current != State::FAILSAFE_STOP) {
            ctx.transition(State::FAILSAFE_STOP);
        }

        // Line loss check. In FOLLOW_LINE_SEARCH, suppress until the first
        // valid line is received — vision startup can take a few seconds.
        if (ctx.line.valid) ctx.line_ever_seen = true;

        bool check_line = (ctx.current == State::FOLLOW_LINE_SEARCH ||
                           ctx.current == State::RETURN_FOLLOW_LINE);
        if (check_line && (ctx.line_ever_seen || ctx.current == State::RETURN_FOLLOW_LINE)) {
            double line_loss = duration_cast<duration<double>>(
                Clock::now() - ctx.last_line_valid_time).count();
            if (line_loss > LINE_LOSS_TIMEOUT_S) {
                std::cerr << "[fsm] line lost too long – failsafe\n";
                ctx.transition(State::FAILSAFE_STOP);
            }
        }

        // ── Dispatch ────────────────────────────────────────────────────────
        switch (ctx.current) {
            case State::INIT:               handle_init(ctx, state);               break;
            case State::FOLLOW_LINE_SEARCH: handle_follow_line_search(ctx, state); break;
            case State::APPROACH_TARGET:    handle_approach_target(ctx, state);    break;
            case State::PICKUP:             handle_pickup(ctx, state);             break;
            case State::SPIN_180:           handle_spin180(ctx, state);            break;
            case State::RETURN_FOLLOW_LINE: handle_return_follow_line(ctx, state); break;
            case State::DROP:               handle_drop(ctx, state);               break;
            case State::DONE:               handle_done(ctx, state);               break;
            case State::FAILSAFE_STOP:      handle_failsafe(ctx, state);           break;
        }
    }

    std::cout << "[fsm] thread exited\n";
}
