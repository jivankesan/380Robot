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
    PICKUP,
    SPIN_180,
    RETURN_FOLLOW_LINE,
    APPROACH_DROP_ZONE,
    DROP,
    FIND_LINE,
    FINAL_FOLLOW,
    DONE,
    FAILSAFE_STOP
};

static std::string to_str(State s) {
    switch (s) {
        case State::INIT:               return "INIT";
        case State::FOLLOW_LINE_SEARCH: return "FOLLOW_LINE_SEARCH";
        case State::PICKUP:             return "PICKUP";
        case State::SPIN_180:           return "SPIN_180";
        case State::RETURN_FOLLOW_LINE: return "RETURN_FOLLOW_LINE";
        case State::APPROACH_DROP_ZONE: return "APPROACH_DROP_ZONE";
        case State::DROP:               return "DROP";
        case State::FIND_LINE:          return "FIND_LINE";
        case State::FINAL_FOLLOW:       return "FINAL_FOLLOW";
        case State::DONE:               return "DONE";
        case State::FAILSAFE_STOP:      return "FAILSAFE_STOP";
        default:                        return "UNKNOWN";
    }
}

// ── FSM context ───────────────────────────────────────────────────────────────

struct FsmCtx {
    State     current     = State::INIT;
    TimePoint state_start = Clock::now();

    LineObs   line;
    Detection det;          // blue
    Detection green_det;    // green drop zone
    bool      hw_ready  = false;
    bool      estop     = false;

    TimePoint last_line_valid_time = Clock::now();
    bool      line_ever_seen       = false;

    // For FINAL_FOLLOW: track when line was last valid to detect line end
    TimePoint last_line_seen_final = Clock::now();

    void transition(State next) {
        if (next == current) return;
        std::cout << "[fsm] " << to_str(current) << " -> " << to_str(next) << "\n";
        if (next == State::FOLLOW_LINE_SEARCH ||
            next == State::RETURN_FOLLOW_LINE ||
            next == State::SPIN_180) {
            last_line_valid_time = Clock::now();
        }
        if (next == State::FOLLOW_LINE_SEARCH) {
            line_ever_seen = false;
        }
        if (next == State::FINAL_FOLLOW) {
            last_line_seen_final = Clock::now();
        }
        current     = next;
        state_start = Clock::now();
    }

    double state_elapsed() const {
        return duration_cast<duration<double>>(Clock::now() - state_start).count();
    }
};

// ── Helpers ───────────────────────────────────────────────────────────────────

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

static void stop(SharedState& s) { set_manual(s, 0.0, 0.0); }

static void set_direct_pwm(SharedState& s, int left, int right) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.control_mode    = ControlMode::DIRECT_PWM;
    s.direct_pwm_left  = left;
    s.direct_pwm_right = right;
}

static void send_claw(SharedState& s, ClawMode mode) {
    std::lock_guard<std::mutex> lk(s.mtx);
    s.claw_mode        = mode;
    s.claw_cmd_pending = true;
}

// ── State handlers ────────────────────────────────────────────────────────────

static void handle_init(FsmCtx& ctx, SharedState& state) {
    stop(state);
    if ((ctx.line.valid && ctx.hw_ready) || ctx.state_elapsed() > 10.0) {
        std::cout << "[fsm] systems ready, starting mission\n";
        ctx.transition(State::FOLLOW_LINE_SEARCH);
    }
}

static void handle_follow_line_search(FsmCtx& ctx, SharedState& state) {
    set_line_follow(state);
    if (!state.object_detect_enabled.load()) {
        state.object_detect_enabled.store(true);
        std::cout << "[fsm] object detection re-enabled\n";
    }
    if (ctx.det.valid) {
        std::cout << "[fsm] blue detected (cx=" << ctx.det.cx
                  << " cy=" << ctx.det.cy << " h=" << ctx.det.h
                  << ") – driving forward " << PICKUP_DRIVE_TIME_S << "s then closing\n";
        state.object_detect_enabled.store(false);
        ctx.transition(State::PICKUP);
    }
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
            std::cout << "[fsm] PICKUP 1/3: driving forward (t=" << t << "s / " << t_close << "s)\n";
        else if (t < t_rotate)
            std::cout << "[fsm] PICKUP 2/3: closing gripper (t=" << t << "s / " << t_rotate << "s)\n";
        else
            std::cout << "[fsm] PICKUP 3/3: rotating arm (t=" << t << "s / " << t_done << "s)\n";
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

    // Enable green detection the moment we enter this state
    if (!state.green_detect_enabled.load()) {
        state.green_detect_enabled.store(true);
        std::cout << "[fsm] green detection enabled\n";
    }

    static double last_ret_log = -1.0;
    double t = ctx.state_elapsed();
    if (t - last_ret_log >= 2.0) {
        last_ret_log = t;
        std::cout << "[fsm] RETURN: t=" << t << "s  line_valid=" << ctx.line.valid
                  << "  green_valid=" << ctx.green_det.valid << "\n";
    }

    if (ctx.green_det.valid) {
        last_ret_log = -1.0;
        std::cout << "[fsm] green drop zone seen (cx=" << ctx.green_det.cx
                  << " cy=" << ctx.green_det.cy << ") – approaching\n";
        ctx.transition(State::APPROACH_DROP_ZONE);
    }
}

static void handle_approach_drop_zone(FsmCtx& ctx, SharedState& state) {
    double t = ctx.state_elapsed();

    if (t < DROP_ZONE_TURN_TIME_S) {
        // Right wheel forward only, left wheel stopped
        set_direct_pwm(state, 0, 50);
        static double last_log = -1.0;
        if (t - last_log >= 0.2) {
            last_log = t;
            std::cout << "[fsm] APPROACH_DROP turning right (t=" << t
                      << "s / " << DROP_ZONE_TURN_TIME_S << "s)\n";
        }
    } else {
        stop(state);
        state.green_detect_enabled.store(false);
        std::cout << "[fsm] turn complete – dropping\n";
        ctx.transition(State::DROP);
    }
}

static void handle_drop(FsmCtx& ctx, SharedState& state) {
    stop(state);
    double t          = ctx.state_elapsed();
    double t_open     = DROP_UNROTATE_TIME_S;
    double t_done     = DROP_UNROTATE_TIME_S + DROP_OPEN_TIME_S;

    static double last_log = -1.0;
    if (t - last_log >= 0.5) {
        last_log = t;
        if (t < t_open)
            std::cout << "[fsm] DROP 1/2: rotating arm to home (t=" << t << "s)\n";
        else
            std::cout << "[fsm] DROP 2/2: opening gripper (t=" << t << "s)\n";
    }

    if (t < t_open) {
        send_claw(state, ClawMode::UNROTATE);
    } else if (t < t_done) {
        send_claw(state, ClawMode::OPEN);
    } else {
        last_log = -1.0;
        std::cout << "[fsm] drop complete – finding line\n";
        ctx.transition(State::FIND_LINE);
    }
}

static void handle_find_line(FsmCtx& ctx, SharedState& state) {
    double t = ctx.state_elapsed();

    // Mirror the drop turn: right wheel backward, left stopped
    if (t < FIND_LINE_REVERSE_TIME_S) {
        set_direct_pwm(state, 0, -100);
        std::cout << "[fsm] FIND_LINE: reversing (t=" << t << "s / "
                  << FIND_LINE_REVERSE_TIME_S << "s)\n";
        return;
    }

    std::cout << "[fsm] reverse complete – resuming line follow\n";
    ctx.transition(State::FINAL_FOLLOW);
}

static void handle_final_follow(FsmCtx& ctx, SharedState& state) {
    set_line_follow(state);

    if (ctx.line.valid) {
        ctx.last_line_seen_final = Clock::now();
    }

    double line_loss = duration_cast<duration<double>>(
        Clock::now() - ctx.last_line_seen_final).count();

    if (line_loss > LINE_LOSS_TIMEOUT_S) {
        stop(state);
        std::cout << "[fsm] line ended – mission complete!\n";
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
    const double dt     = 1.0 / FSM_RATE_HZ;
    const auto   period = duration_cast<nanoseconds>(duration<double>(dt));
    auto next_tick      = Clock::now() + period;

    FsmCtx ctx;

    while (!state.shutdown.load()) {
        std::this_thread::sleep_until(next_tick);
        next_tick += period;

        // ── Read shared state ────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            ctx.line      = state.line_obs;
            ctx.det       = state.detection;
            ctx.green_det = state.green_detection;
            ctx.hw_ready  = state.hw_ready;
            ctx.estop     = state.estop;
        }

        if (ctx.line.valid) ctx.last_line_valid_time = Clock::now();

        // Update FSM state string for logging
        {
            std::lock_guard<std::mutex> lk(state.mtx);
            state.fsm_state = to_str(ctx.current);
        }

        // ── Global failsafe ──────────────────────────────────────────────────
        if (ctx.estop && ctx.current != State::FAILSAFE_STOP) {
            ctx.transition(State::FAILSAFE_STOP);
        }

        // Line loss check (only during line-following states)
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
            case State::PICKUP:             handle_pickup(ctx, state);             break;
            case State::SPIN_180:           handle_spin180(ctx, state);            break;
            case State::RETURN_FOLLOW_LINE: handle_return_follow_line(ctx, state); break;
            case State::APPROACH_DROP_ZONE: handle_approach_drop_zone(ctx, state); break;
            case State::DROP:               handle_drop(ctx, state);               break;
            case State::FIND_LINE:          handle_find_line(ctx, state);          break;
            case State::FINAL_FOLLOW:       handle_final_follow(ctx, state);       break;
            case State::DONE:               handle_done(ctx, state);               break;
            case State::FAILSAFE_STOP:      handle_failsafe(ctx, state);           break;
        }
    }

    std::cout << "[fsm] thread exited\n";
}
