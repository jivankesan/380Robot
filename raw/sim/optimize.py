"""
optimize.py - Differential-evolution optimizer for the line-follow controller.

Runs the simulation with varying parameter sets, saves runs that reach past
50% of the track, and uses scipy differential_evolution to search the space.

Usage:
    python3 optimize.py
"""

import os
import sys
import json
import datetime
import numpy as np
from scipy.optimize import differential_evolution

sys.path.insert(0, os.path.dirname(__file__))
from simulate import simulate, build_course, DEFAULT_PARAMS, plot_run

PARAM_NAMES = [
    'KP_LAT',
    'KD_LAT',
    'KP_HDG',
    'KD_HDG',
    'BASE_SPEED',
    'HEADING_BRAKE_GAIN',
    'TURN_SPEED_GAIN',
    'MAX_ANG_VEL',
    'SP_A_MAX_DECEL',
    'SP_ALPHA_MAX',
    'MIN_TURN_SPEED',    # floor speed through turns — primary lever for lap time
    'SP_K_HEADING',      # profiler deceleration sensitivity to heading error
    'SP_K_ERROR',        # profiler deceleration sensitivity to lateral error
]

BOUNDS = [
    (0.5,  12.0),   # KP_LAT
    (1.0,  30.0),   # KD_LAT
    (0.5,  10.0),   # KP_HDG
    (1.0,  30.0),   # KD_HDG
    (0.30,  1.056), # BASE_SPEED
    (0.0,   4.0),   # HEADING_BRAKE_GAIN
    (0.0,   8.0),   # TURN_SPEED_GAIN
    (3.0,  12.0),   # MAX_ANG_VEL
    (2.0,  15.0),   # SP_A_MAX_DECEL
    (3.0,  15.0),   # SP_ALPHA_MAX
    (0.10,  0.50),  # MIN_TURN_SPEED
    (0.0,   1.5),   # SP_K_HEADING
    (0.0,   1.5),   # SP_K_ERROR
]

GOOD_PARAMS_FILE = os.path.join(os.path.dirname(__file__), 'good_params.json')

good_runs   = []
run_counter = [0]   # list so cost_fn can mutate it

# Two separate best-run trackers: fastest complete lap, and lowest-cost DNF.
# best_run.png always shows the complete run once one exists.
best_complete_time   = [float('inf')]
best_complete_x      = [None]
best_complete_hist   = [None]
best_complete_params = [None]
best_complete_diag   = [None]

best_dnf_cost   = [float('inf')]
best_dnf_x      = [None]
best_dnf_hist   = [None]
best_dnf_params = [None]
best_dnf_diag   = [None]

best_cost       = [float('inf')]   # scalar used only for DE convergence reporting

total_track_pts = None
HALFWAY         = None

BEST_PLOT_PATH = os.path.join(os.path.dirname(__file__), 'best_run.png')

MAX_TIME = 30.0   # simulation timeout (s)


def evaluate_run(history, lap_time, max_prog):
    """
    Compute a weighted scalar cost and diagnostics dict for one simulation run.

    Cost components:
      time_cost        - lap time, or extrapolated completion time for DNF.
      tracking_cost    - RMS lateral/heading error, ITAE, and mean turn-phase e_lat.
                         ITAE (integral of time-weighted absolute error) penalises
                         persistent drift more than brief transients.
      smoothness_cost  - angular-rate jerk + line-loss event count.
      speed_cost       - penalty for low mean speed (0 if avg >= 0.5 m/s).
      completion_penalty - sqrt-shaped DNF penalty; any complete run beats any DNF.
    """
    progress = max_prog / total_track_pts

    t     = np.array(history['t'],     dtype=float)
    e_lat = np.array(history['e_lat'], dtype=float)
    e_hdg = np.array(history['e_hdg'], dtype=float)
    omega = np.array(history['omega'], dtype=float)
    v     = np.array(history['v'],     dtype=float)
    valid = np.array(history['valid'],  dtype=bool)
    n     = len(t)

    if n == 0:
        return 1000.0, {}

    dt = 0.01

    # Time cost: actual lap time, or extrapolated (capped at 3× timeout for DNF)
    if lap_time is not None:
        time_cost = float(lap_time)
    else:
        if progress > 0.05:
            time_cost = min(t[-1] / progress, 3.0 * MAX_TIME)
        else:
            time_cost = 3.0 * MAX_TIME

    # Tracking cost
    rms_lat = float(np.sqrt(np.mean(e_lat ** 2))) * 100        # cm
    rms_hdg = float(np.sqrt(np.mean(np.degrees(e_hdg) ** 2)))  # degrees

    # ITAE: ∫ t·|e_lat| dt, normalised so 1 cm steady-state error for 30 s → ~1.0
    itae = float(np.sum(t * np.abs(e_lat)) * dt) / (0.01 * MAX_TIME ** 2)

    # Turn-specific error (|e_hdg| > 0.1 rad ≈ 5.7°); falls back to rms_lat if sparse
    in_turn = np.abs(e_hdg) > 0.10
    if in_turn.sum() > 10:
        turn_lat = float(np.mean(np.abs(e_lat[in_turn]))) * 100
    else:
        turn_lat = rms_lat

    tracking_cost = (0.25 * rms_lat +
                     0.15 * rms_hdg +
                     0.40 * itae +
                     0.40 * turn_lat)

    # Smoothness cost
    if len(omega) > 1:
        jerk      = float(np.mean(np.diff(omega) ** 2) / dt)
        jerk_norm = min(jerk / 50.0, 5.0)
    else:
        jerk_norm = 0.0

    loss_events = int(np.sum(np.diff(valid.astype(np.int8)) < 0)) if n > 1 else 0
    smoothness_cost = jerk_norm + 1.5 * loss_events

    # Speed cost
    mean_speed = float(np.mean(v))
    speed_cost = max(0.0, 0.5 - mean_speed) * 8.0

    # Completion penalty: sqrt shape keeps gradient toward finish non-zero at all progress
    if lap_time is not None:
        completion_penalty = 0.0
    else:
        completion_penalty = 30.0 * (1.0 - progress) ** 0.5

    zeta_lat = zeta_hdg = 0.0   # filled by caller; placeholder for diagnostics

    cost = (2.0 * time_cost +
            0.5 * tracking_cost +
            0.3 * smoothness_cost +
            1.5 * speed_cost +
            completion_penalty)

    diag = {
        'progress':          round(progress,    3),
        'time_cost':         round(time_cost,   2),
        'rms_lat_cm':        round(rms_lat,     2),
        'rms_hdg_deg':       round(rms_hdg,     2),
        'itae':              round(itae,         3),
        'turn_lat_cm':       round(turn_lat,     2),
        'jerk_norm':         round(jerk_norm,    3),
        'loss_events':       loss_events,
        'mean_speed':        round(mean_speed,   3),
        'tracking_cost':     round(tracking_cost,2),
        'smoothness_cost':   round(smoothness_cost,2),
        'completion_penalty':round(completion_penalty,2),
    }

    return cost, diag


def cost_fn(x):
    run_counter[0] += 1
    run_num = run_counter[0]

    params = DEFAULT_PARAMS.copy()
    for name, val in zip(PARAM_NAMES, x):
        params[name] = float(val)

    lap_time, history, max_prog = simulate(params, max_time=MAX_TIME)

    cost, diag = evaluate_run(history, lap_time, max_prog)

    # Damping ratios for lat and heading loops
    zeta_lat = params['KD_LAT'] / (2.0 * np.sqrt(max(params['KP_LAT'], 1e-6)))
    zeta_hdg = params['KD_HDG'] / (2.0 * np.sqrt(max(params['KP_HDG'], 1e-6)))

    # Soft regulariser toward well-damped region (ζ ∈ [0.5, 2.5])
    zeta_penalty = (max(0.0, 0.5 - zeta_lat) ** 2 + max(0.0, zeta_lat - 2.5) ** 2 +
                    max(0.0, 0.5 - zeta_hdg) ** 2 + max(0.0, zeta_hdg - 2.5) ** 2)
    cost += 0.3 * zeta_penalty

    progress = diag['progress']
    if lap_time is not None:
        diagnosis = "COMPLETE"
    elif diag['rms_hdg_deg'] > 15:
        diagnosis = "UNDER-TURN"
    elif diag['jerk_norm'] > 1.5 or diag['loss_events'] > 3:
        diagnosis = "OSCILLATING"
    elif progress < 0.3:
        diagnosis = "EARLY-LOSS"
    else:
        diagnosis = "DNF"

    best_cost[0] = min(best_cost[0], cost)

    if lap_time is not None:
        if lap_time < best_complete_time[0]:
            best_complete_time[0]   = lap_time
            best_complete_x[0]      = x.copy()
            best_complete_hist[0]   = history
            best_complete_params[0] = params.copy()
            best_complete_diag[0]   = diag
            plot_run(
                history,
                title=(f"BEST COMPLETE  lap={lap_time:.2f}s  "
                       f"lat={diag['rms_lat_cm']:.1f}cm  turn={diag['turn_lat_cm']:.1f}cm"),
                save_path=BEST_PLOT_PATH,
                show=False,
            )
            flag = "✓ SUB-14!" if lap_time < 14.0 else "★"
            print(f"[opt] {flag} New best COMPLETE  lap={lap_time:.2f}s → best_run.png")
    else:
        if cost < best_dnf_cost[0]:
            best_dnf_cost[0]   = cost
            best_dnf_x[0]      = x.copy()
            best_dnf_hist[0]   = history
            best_dnf_params[0] = params.copy()
            best_dnf_diag[0]   = diag
            if best_complete_time[0] == float('inf'):
                plot_run(
                    history,
                    title=(f"BEST DNF  cost={cost:.2f}  prog={progress:.0%}  "
                           f"lat={diag['rms_lat_cm']:.1f}cm"),
                    save_path=BEST_PLOT_PATH,
                    show=False,
                )
                print(f"[opt] ↑ New best DNF  cost={cost:.2f}  prog={progress:.0%} → best_run.png")

    if progress >= 0.5:
        entry = {
            'run':       run_num,
            'timestamp': datetime.datetime.now().isoformat(timespec='seconds'),
            'params':    {n: float(v) for n, v in zip(PARAM_NAMES, x)},
            'lap_time':  float(lap_time) if lap_time is not None else None,
            'cost':      round(cost, 4),
            'diagnosis': diagnosis,
            'zeta_lat':  round(zeta_lat, 3),
            'zeta_hdg':  round(zeta_hdg, 3),
            **diag,
        }
        good_runs.append(entry)
        good_runs.sort(key=lambda r: (r['lap_time'] if r['lap_time'] is not None else 999.0,
                                      r['cost']))
        try:
            with open(GOOD_PARAMS_FILE, 'w') as f:
                json.dump(good_runs, f, indent=2)
        except OSError as e:
            print(f"[opt] WARNING: could not write {GOOD_PARAMS_FILE}: {e}")

    print(
        f"Run {run_num:4d} | prog={progress*100:3.0f}% | cost={cost:6.2f} | "
        f"lat={diag['rms_lat_cm']:4.1f}cm | turn={diag['turn_lat_cm']:4.1f}cm | "
        f"itae={diag['itae']:.2f} | jerk={diag['jerk_norm']:.2f} | "
        f"loss={diag['loss_events']} | spd={diag['mean_speed']:.2f}m/s | "
        f"\u03b6=({zeta_lat:.2f},{zeta_hdg:.2f}) | "
        f"{diagnosis:<11} \u03c9={params['MAX_ANG_VEL']:.1f}"
    )

    return cost


gen_counter = [0]

def de_callback(xk, convergence):
    gen_counter[0] += 1
    if gen_counter[0] % 10 == 0:
        print(
            f"\n{'─'*70}\n"
            f"  Generation {gen_counter[0]:4d}  |  best cost so far = {best_cost[0]:.4f}\n"
            f"{'─'*70}"
        )
    return False   # returning True stops DE early


if __name__ == '__main__':
    track_pts, _, _ = build_course()
    total_track_pts = len(track_pts)
    HALFWAY         = int(total_track_pts * 0.5)

    print("=" * 70)
    print("  Line-follow controller optimizer  (scipy differential_evolution)")
    print("=" * 70)
    print(f"  Track points : {total_track_pts}")
    print(f"  Halfway      : {HALFWAY}  ({HALFWAY/total_track_pts*100:.0f}% of track)")
    print(f"  Good-params  : progress >= 50% saved to {GOOD_PARAMS_FILE}")
    print(f"  Target       : lap_time < 14 s")
    print()
    print(f"  {'Parameter':<22}  {'Low':>6}  {'High':>6}")
    print(f"  {'-'*38}")
    for name, (lo, hi) in zip(PARAM_NAMES, BOUNDS):
        print(f"  {name:<22}  {lo:6.2f}  {hi:6.2f}")
    print("=" * 70)
    print()

    result = None
    try:
        result = differential_evolution(
            cost_fn,
            bounds=BOUNDS,
            maxiter=3000,
            popsize=15,       # 15 × 13 params = 195 individuals per generation
            mutation=(0.5, 1.5),
            recombination=0.9,
            seed=42,
            init='latinhypercube',
            workers=1,
            tol=0.001,
            polish=False,
            callback=de_callback,
        )
    except KeyboardInterrupt:
        print("\n\n[opt] Interrupted by user.")

    # Prefer fastest complete run; fall back to best DNF
    bx     = best_complete_x[0] or best_dnf_x[0]
    b_diag = best_complete_diag[0] or best_dnf_diag[0]
    if bx is None and result is not None:
        bx = result.x

    print("\n" + "=" * 70)
    if bx is not None:
        is_complete = best_complete_x[0] is not None
        if is_complete:
            print(f"  Best COMPLETE lap  : {best_complete_time[0]:.3f} s"
                  + ("  ✓ SUB-14!" if best_complete_time[0] < 14.0 else ""))
        else:
            print(f"  Best DNF cost      : {best_dnf_cost[0]:.4f}  (no complete run yet)")
        print(f"  Total runs         : {run_counter[0]}")
        print(f"  Good runs (>=50%)  : {len(good_runs)}")
        print(f"  Complete runs      : {sum(1 for r in good_runs if r['lap_time'] is not None)}")
        print()
        print("  Paste into config.h:")
        print("  " + "-" * 50)
        best_p = dict(zip(PARAM_NAMES, bx))
        for name, val in best_p.items():
            print(f"  static constexpr double {name:<22} = {val:.4f};")
        zl = best_p['KD_LAT'] / (2.0 * np.sqrt(max(best_p['KP_LAT'], 1e-6)))
        zh = best_p['KD_HDG'] / (2.0 * np.sqrt(max(best_p['KP_HDG'], 1e-6)))
        print("  " + "-" * 50)
        print(f"  Damping ratios:  ζ_lat={zl:.3f}  ζ_hdg={zh:.3f}  (ideal ≈ 0.7)")
        print(f"  Best run plot saved → {BEST_PLOT_PATH}")
    else:
        print("  No runs recorded.")
    print("=" * 70)
