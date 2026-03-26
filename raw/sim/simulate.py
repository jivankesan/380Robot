"""
Robot line-follow simulation — Python translation of control.cpp.

Camera ROI covers 10–20 cm ahead of the wheel centre (top half of frame).
e_lat: mean lateral offset of track centroids in the ROI, + = line to robot's right.
e_hdg: arctan2(far_centroid – near_centroid, LOOK_SPAN/2), + = track turning right.
Dynamics: unicycle model, Euler integration at 100 Hz.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# Robot physical parameters
#   mass   = 0.500 kg
#   length = 0.20 m  (caster front, LiPo rear)
#   width  = 0.15 m  (wheel-to-wheel)
#   CoM    ≈ at wheel axle
#
# Yaw inertia through wheel axle:
#   LiPo  0.12 kg × (0.05 m)²          = 0.000300
#   body  0.18 kg × (0.06 m)²          = 0.000648
#   caster 0.08 kg × (0.15 m)²         = 0.001800
#   misc  0.04 kg × (0.05 m)²          = 0.000100
#   wheels 2×0.04 kg × (0.075 m)²      = 0.000450
#   Total I_yaw ≈ 0.003 kg·m²
ROBOT_MASS  = 0.500   # kg
ROBOT_I_YAW = 0.003   # kg·m²
MOTOR_F_MAX = 6.0     # N per wheel
# ^ calibrated for 210 RPM motors on 7.4V 2S LiPo; stall torque ≈ 0.3 N·m → 6 N at r=0.048 m

# Track geometry
R_TURN = 0.15   # turn arc radius (m)
L_TILE = 0.30   # tile side length (m)
N_PTS  = 100    # centerline sample-points per tile

# Camera ROI bounds
LOOK_NEAR = 0.10
LOOK_FAR  = 0.20
LOOK_MID  = (LOOK_NEAR + LOOK_FAR) / 2
LOOK_SPAN = LOOK_FAR - LOOK_NEAR

# Heading error denominator — must match vision.py's pixel-space computation.
# vision.py: heading = arctan2(cx_top - cx_bot, mid_y)
#   ROI y: 10–60% of 480px frame → roi_h = 240 px, mid_y = 120 px
#   LINE_PIXELS_TO_M = 0.001 m/px → physical depth = 120 * 0.001 = 0.12 m
# Using LOOK_SPAN/2 = 0.05 m would make heading errors 2.4× too large.
HEADING_DENOM = 0.12  # m  (matches vision.py mid_y in physical units)

# Detection fails if the centroid lateral offset exceeds this (~80% of half-tile-width)
DETECTION_LAT_LIMIT = 0.12   # m

# Arc-distance index steps corresponding to the ROI bounds
_SPM     = N_PTS / L_TILE
IDX_NEAR = round(LOOK_NEAR * _SPM)
IDX_MID  = round(LOOK_MID  * _SPM)
IDX_FAR  = round(LOOK_FAR  * _SPM)

DEFAULT_PARAMS = {
    # ── PD controller ── (matches config.h exactly)
    'KP_LAT': 2.5,   'KD_LAT': 6.0,
    'KP_HDG': 1.8,   'KD_HDG': 8.0,        # config.h: KD_HEADING = 8.0
    'BASE_SPEED':         0.50,              # config.h: BASE_SPEED_MPS = 0.50
    'MAX_LIN_VEL':        1.056,
    'MIN_TURN_SPEED':     0.12,              # config.h: MIN_TURN_SPEED_MPS = 0.12
    'MAX_ANG_VEL':        2.5,              # config.h: MAX_ANG_VEL_RPS = 2.5
    'HEADING_BRAKE_GAIN': 2.0,
    'TURN_SPEED_GAIN':    5.0,
    'TURN_OMEGA_DEADBAND':0.15,
    # ── Speed profiler ── (matches config.h exactly)
    'SP_V_MAX':       1.056,
    'SP_V_MIN':       0.10,                 # config.h: SP_V_MIN = 0.1
    'SP_A_MAX_ACCEL': 3.0,                  # config.h: SP_A_MAX_ACCEL = 3.0
    'SP_A_MAX_DECEL': 8.0,                  # config.h: SP_A_MAX_DECEL = 8.0
    'SP_ALPHA_MAX':   6.0,                  # config.h: SP_ALPHA_MAX = 6.0
    'SP_K_CURVATURE': 0.4,
    'SP_K_ERROR':     0.4,
    'SP_K_HEADING':   0.4,
    'LOST_LINE_TIMEOUT_S': 0.2,
}


class TrackBuilder:
    def __init__(self):
        self.pts        = []
        self.joints     = [(0.0, 0.0, 0.0)]
        self.tile_types = []
        self.x = self.y = self.h = 0.0

    def S(self, kind='S'):
        for i in range(1, N_PTS + 1):
            t = i / N_PTS
            self.pts.append((self.x + t * L_TILE * np.cos(self.h),
                             self.y + t * L_TILE * np.sin(self.h)))
        self.x += L_TILE * np.cos(self.h)
        self.y += L_TILE * np.sin(self.h)
        self.joints.append((self.x, self.y, self.h))
        self.tile_types.append(kind)
        return self

    def T(self, d):
        cx = self.x + R_TURN * np.cos(self.h + d * np.pi / 2)
        cy = self.y + R_TURN * np.sin(self.h + d * np.pi / 2)
        a0 = np.arctan2(self.y - cy, self.x - cx)
        for i in range(1, N_PTS + 1):
            t = i / N_PTS
            a = a0 + d * t * np.pi / 2
            self.pts.append((cx + R_TURN * np.cos(a), cy + R_TURN * np.sin(a)))
        a1 = a0 + d * np.pi / 2
        self.x = cx + R_TURN * np.cos(a1)
        self.y = cy + R_TURN * np.sin(a1)
        self.h += d * np.pi / 2
        self.joints.append((self.x, self.y, self.h))
        self.tile_types.append('R' if d == -1 else 'L')
        return self

    def R(self):      return self.T(-1)
    def Lf(self):     return self.T(+1)
    def Target(self): return self.S(kind='T')

    def build(self):
        return np.array(self.pts), self.joints, self.tile_types


def build_course():
    tb = TrackBuilder()
    (tb
     .S().S().S().S().S()
     .R()
     .S().S().S().S()
     .R().R()
     .Lf().Lf()
     .R()
     .S().S()
     .R()
     .S()
     .R()
     .S()
     .Lf()
     .Target()
    )
    return tb.build()


def precompute_curvature(track_pts):
    """
    Compute signed curvature (1/m) at every track point.
    κ = dθ/ds, estimated via finite differences of the heading angle.
    Positive = turning left (counter-clockwise), matching vision.py convention.
    """
    dx = np.gradient(track_pts[:, 0])
    dy = np.gradient(track_pts[:, 1])
    ds = np.sqrt(dx**2 + dy**2)
    headings = np.arctan2(dy, dx)
    # unwrap to avoid jumps at ±π
    headings = np.unwrap(headings)
    dh = np.gradient(headings)
    curvature = dh / np.where(ds > 1e-9, ds, 1e-9)
    return curvature


def compute_errors(rx, ry, rh, track_pts, track_curv, prog_idx):
    """
    Simulate camera line-detection output.

    Uses arc distance along the track rather than straight-line projection to
    avoid the corner-peeking artifact (heading projection exposes the upcoming
    arc before the robot physically reaches the tile seam).

    Mirrors the two-half centroid logic in vision.py.
    Returns (e_lat [m], e_hdg [rad], curvature [1/m], valid).
    """
    i_near = prog_idx + IDX_NEAR
    i_mid  = prog_idx + IDX_MID
    i_far  = prog_idx + IDX_FAR

    if i_far >= len(track_pts):
        return 0.0, 0.0, 0.0, False

    right     = np.array([ np.sin(rh), -np.cos(rh)])
    robot_pos = np.array([rx, ry])

    lat_roi  = (track_pts[i_near:i_far] - robot_pos) @ right
    lat_near = (track_pts[i_near:i_mid] - robot_pos) @ right
    lat_far  = (track_pts[i_mid:i_far]  - robot_pos) @ right

    e_lat = float(np.mean(lat_roi))

    if abs(e_lat) > DETECTION_LAT_LIMIT:
        return 0.0, 0.0, 0.0, False

    e_hdg = float(np.arctan2(np.mean(lat_far) - np.mean(lat_near), HEADING_DENOM))

    # vision.py always sends curvature = 0.0 (see vision.py line 192: "...0.0")
    # The real speed profiler therefore never uses SP_K_CURVATURE.
    # track_curv is kept for diagnostics/visualization only.
    curv = 0.0

    return e_lat, e_hdg, curv, True


class PDController:
    def __init__(self):
        self.last_lat = 0.0
        self.last_hdg = 0.0

    def reset(self):
        self.last_lat = self.last_hdg = 0.0

    def step(self, e_lat, e_hdg, p, dt):
        d_lat = (e_lat - self.last_lat) / dt
        d_hdg = (e_hdg - self.last_hdg) / dt

        omega = (p['KP_LAT'] * e_lat + p['KD_LAT'] * d_lat +
                 p['KP_HDG'] * e_hdg + p['KD_HDG'] * d_hdg)
        omega = float(np.clip(omega, -p['MAX_ANG_VEL'], p['MAX_ANG_VEL']))

        excess = max(0.0, abs(omega) - p['TURN_OMEGA_DEADBAND'])
        brake  = p['HEADING_BRAKE_GAIN'] * abs(e_hdg)
        v = p['BASE_SPEED'] - brake - p['TURN_SPEED_GAIN'] * excess
        v = float(np.clip(v, p['MIN_TURN_SPEED'], p['MAX_LIN_VEL']))

        self.last_lat = e_lat
        self.last_hdg = e_hdg
        return v, omega


class SpeedProfiler:
    def __init__(self):
        self.v_cmd     = 0.0
        self.omega_cmd = 0.0

    def reset(self):
        self.v_cmd = self.omega_cmd = 0.0

    def step(self, v_raw, omega_raw, e_lat, e_hdg, curv, p, dt):
        v_target = (p['SP_V_MAX']
                    - p['SP_K_CURVATURE'] * abs(curv)
                    - p['SP_K_ERROR']     * abs(e_lat)
                    - p['SP_K_HEADING']   * abs(e_hdg))
        v_target = min(v_target, v_raw)
        v_target = float(np.clip(v_target, p['SP_V_MIN'], p['SP_V_MAX']))

        a_lim = p['SP_A_MAX_DECEL'] if v_target < self.v_cmd else p['SP_A_MAX_ACCEL']
        dv = float(np.clip(v_target - self.v_cmd, -a_lim * dt, a_lim * dt))
        self.v_cmd = float(np.clip(self.v_cmd + dv, 0.0, p['SP_V_MAX']))

        domega = float(np.clip(omega_raw - self.omega_cmd,
                               -p['SP_ALPHA_MAX'] * dt, p['SP_ALPHA_MAX'] * dt))
        self.omega_cmd += domega

        if v_raw == 0.0:
            self.v_cmd = 0.0

        return self.v_cmd, self.omega_cmd


class RobotDynamics:
    """
    Differential-drive plant with mass and yaw inertia.

    Converts desired (v_cmd, omega_cmd) to actual (v, omega) by computing the
    required wheel forces, clamping to motor limits, and integrating F=ma / τ=Iα.

        F_R = m·Δv/(2·dt) + I·Δω/(L·dt)
        F_L = m·Δv/(2·dt) − I·Δω/(L·dt)

    The shared force budget means hard braking into a turn limits available angular
    acceleration, matching real-hardware behaviour that a pure kinematic model misses.
    """

    def __init__(self):
        self.v     = 0.0
        self.omega = 0.0

    def reset(self):
        self.v = self.omega = 0.0

    def step(self, v_cmd, omega_cmd, dt):
        dv     = (v_cmd     - self.v)     / dt
        domega = (omega_cmd - self.omega) / dt

        L = 0.15  # wheel base (m)

        F_R = ROBOT_MASS * dv / 2.0 + ROBOT_I_YAW * domega / L
        F_L = ROBOT_MASS * dv / 2.0 - ROBOT_I_YAW * domega / L

        F_R = float(np.clip(F_R, -MOTOR_F_MAX, MOTOR_F_MAX))
        F_L = float(np.clip(F_L, -MOTOR_F_MAX, MOTOR_F_MAX))

        a_v   = (F_L + F_R) / ROBOT_MASS
        alpha = (F_R - F_L) * L / 2.0 / ROBOT_I_YAW

        self.v     += a_v   * dt
        self.omega += alpha * dt

        return self.v, self.omega


def find_progress(rx, ry, track_pts, last_idx, search_ahead=200):
    """Local nearest-point search, constrained to always advance along the track."""
    start = max(0, last_idx - 5)
    end   = min(len(track_pts), last_idx + search_ahead)
    seg   = track_pts[start:end]
    dists = np.linalg.norm(seg - np.array([rx, ry]), axis=1)
    local = int(np.argmin(dists))
    return start + local, float(dists[local])


def simulate(params=None, max_time=30.0, dt=0.01, crash_limit=0.13):
    """
    Run one lap with the given parameter dict.
    Returns (lap_time, history, max_prog_idx). lap_time is None on crash or timeout.
    """
    p = DEFAULT_PARAMS.copy()
    if params:
        p.update(params)

    track_pts, joints, tile_types = build_course()
    track_curv = precompute_curvature(track_pts)

    tx0, ty0, th_end = joints[-2]
    target = np.array([tx0 + 0.15 * np.cos(th_end),
                        ty0 + 0.15 * np.sin(th_end)])

    rx, ry, rh = joints[0]
    prog_idx     = 0
    max_prog_idx = 0

    pd    = PDController()
    prof  = SpeedProfiler()
    plant = RobotDynamics()
    lost_line_since = None

    hist = {k: [] for k in ('t', 'x', 'y', 'h', 'v', 'omega', 'e_lat', 'e_hdg', 'valid')}

    n_steps = int(max_time / dt)

    for step_i in range(n_steps):
        t = step_i * dt

        prog_idx, nearest_dist = find_progress(rx, ry, track_pts, prog_idx)
        max_prog_idx = max(max_prog_idx, prog_idx)

        if nearest_dist > crash_limit:
            print(f"[sim] CRASH at t={t:.2f}s  dist_to_line={nearest_dist*100:.1f} cm")
            return None, hist, max_prog_idx

        dist_to_target = float(np.linalg.norm(np.array([rx, ry]) - target))
        if prog_idx > len(track_pts) * 0.85 and dist_to_target < 0.06:
            print(f"[sim] COMPLETE  lap_time={t:.3f}s")
            return t, hist, max_prog_idx

        e_lat, e_hdg, curv, valid = compute_errors(rx, ry, rh, track_pts, track_curv, prog_idx)

        if valid:
            lost_line_since = None
        else:
            if lost_line_since is None:
                lost_line_since = t
            if (t - lost_line_since) > p['LOST_LINE_TIMEOUT_S']:
                pd.reset()
                prof.reset()
                plant.reset()
                hist['t'].append(t); hist['x'].append(rx); hist['y'].append(ry)
                hist['h'].append(rh); hist['v'].append(0.0); hist['omega'].append(0.0)
                hist['e_lat'].append(0.0); hist['e_hdg'].append(0.0); hist['valid'].append(False)
                continue

        v_raw,  omega_raw  = pd.step(e_lat, e_hdg, p, dt)
        v_prof, omega_prof = prof.step(v_raw, omega_raw, e_lat, e_hdg, curv, p, dt)
        v_act,  omega_act  = plant.step(v_prof, omega_prof, dt)

        hist['t'].append(t);         hist['x'].append(rx);      hist['y'].append(ry)
        hist['h'].append(rh);        hist['v'].append(v_act);   hist['omega'].append(omega_act)
        hist['e_lat'].append(e_lat); hist['e_hdg'].append(e_hdg); hist['valid'].append(valid)

        # Positive omega → right wheel faster → CW rotation → heading decreases.
        rx += v_act * np.cos(rh) * dt
        ry += v_act * np.sin(rh) * dt
        rh -= omega_act * dt

    print(f"[sim] TIMEOUT  (did not finish in {max_time:.0f}s)")
    return None, hist, max_prog_idx


def plot_run(history, title="Simulation run", save_path='sim_result.png', show=True, params=None):
    track_pts, joints, tile_types = build_course()

    tx0, ty0, th_end = joints[-2]
    target_x = tx0 + 0.15 * np.cos(th_end)
    target_y  = ty0 + 0.15 * np.sin(th_end)

    t     = np.array(history['t'])
    x_arr = np.array(history['x'])
    y_arr = np.array(history['y'])
    v_arr = np.array(history['v'])
    om_arr= np.array(history['omega'])
    el_arr= np.array(history['e_lat'])
    eh_arr= np.array(history['e_hdg'])

    fig = plt.figure(figsize=(18, 10))
    gs  = GridSpec(3, 2, figure=fig, width_ratios=[1.6, 1], hspace=0.45, wspace=0.35)

    ax_map = fig.add_subplot(gs[:, 0])

    face = {'S': '#d4a96a', 'R': '#c8974f', 'L': '#c8974f', 'T': '#b8c8a0'}
    for i, tt in enumerate(tile_types):
        ex, ey, eh = joints[i]
        fx, fy = np.cos(eh), np.sin(eh)
        nx, ny = -fy, fx
        half = L_TILE / 2
        corners = np.array([
            [ex + half*nx,          ey + half*ny],
            [ex - half*nx,          ey - half*ny],
            [ex + L_TILE*fx - half*nx, ey + L_TILE*fy - half*ny],
            [ex + L_TILE*fx + half*nx, ey + L_TILE*fy + half*ny],
        ])
        ax_map.add_patch(plt.Polygon(corners, fc=face[tt], ec='#7a5c2e', lw=1.0, zorder=1))

    ax_map.plot(track_pts[:, 0], track_pts[:, 1], 'r-', lw=1.8, zorder=2, alpha=0.6)

    speed_color = v_arr / max(v_arr.max(), 0.01)
    ax_map.scatter(x_arr, y_arr, c=speed_color, cmap='plasma',
                   s=4, zorder=3, alpha=0.8)
    ax_map.plot(*[x_arr[0], y_arr[0]], 'go', ms=10, zorder=5, label='Start')
    if len(x_arr) > 1:
        ax_map.annotate('', xy=(x_arr[-1], y_arr[-1]),
                        xytext=(x_arr[-2], y_arr[-2]),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=2), zorder=5)

    for r, c in zip([0.075, 0.060, 0.045, 0.030, 0.010],
                    ['#1144EE', 'white', 'red', 'white', '#111111']):
        ax_map.add_patch(plt.Circle((target_x, target_y), r,
                                    fc=c, ec='black', lw=0.6, zorder=4))

    sm = plt.cm.ScalarMappable(cmap='plasma',
                               norm=plt.Normalize(0, v_arr.max() if len(v_arr) else 1))
    sm.set_array([])
    fig.colorbar(sm, ax=ax_map, label='Speed (m/s)', shrink=0.6)

    ax_map.set_aspect('equal')
    ax_map.grid(True, alpha=0.2)
    ax_map.set_xlabel('X (m)'); ax_map.set_ylabel('Y (m)')
    ax_map.set_title(title, fontweight='bold')

    ax_v  = fig.add_subplot(gs[0, 1])
    ax_e  = fig.add_subplot(gs[1, 1])
    ax_om = fig.add_subplot(gs[2, 1])

    ax_v.plot(t, v_arr, color='#e05c00', lw=1.5)
    ax_v.set_ylabel('Speed (m/s)'); ax_v.set_ylim(0, 1.1)
    _p = DEFAULT_PARAMS.copy()
    if params:
        _p.update(params)
    ax_v.axhline(_p['BASE_SPEED'], ls='--', color='grey', lw=0.8, label='base')
    ax_v.legend(fontsize=8); ax_v.grid(True, alpha=0.3)

    ax_e.plot(t, np.array(el_arr) * 100, color='red',  lw=1.2, label='e_lat (cm)')
    ax_e.plot(t, np.degrees(eh_arr),     color='blue', lw=1.2, label='e_hdg (°)')
    ax_e.axhline(0, ls='--', color='grey', lw=0.6)
    ax_e.set_ylabel('Error'); ax_e.legend(fontsize=8); ax_e.grid(True, alpha=0.3)

    ax_om.plot(t, om_arr, color='#008855', lw=1.5)
    ax_om.set_ylabel('ω (rad/s)'); ax_om.set_xlabel('Time (s)')
    ax_om.axhline(0, ls='--', color='grey', lw=0.6); ax_om.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'[sim] saved → {save_path}')
    if show:
        plt.show()
    plt.close('all')


if __name__ == '__main__':
    import sys, json, os

    use_best = '--best' in sys.argv

    params = None
    if use_best:
        good_file = os.path.join(os.path.dirname(__file__), 'good_params.json')
        with open(good_file) as f:
            runs = json.load(f)
        complete = [r for r in runs if r['lap_time'] is not None]
        if not complete:
            print("No complete runs in good_params.json yet.")
            sys.exit(1)
        complete.sort(key=lambda r: r['lap_time'])
        best = complete[0]
        params = best['params']
        print("=" * 55)
        print(f"  Best run: {best['lap_time']:.2f}s  (run #{best['run']})")
        print(f"  Mean speed: {best['mean_speed']:.3f} m/s")
    else:
        print("=" * 55)
        print("  Robot line-follow simulation  (default params)")

    print(f"  Camera ROI: {LOOK_NEAR*100:.0f}–{LOOK_FAR*100:.0f} cm ahead of wheel centre")
    print("=" * 55)

    lap_time, history, _ = simulate(params, max_time=30.0)

    if lap_time is not None:
        v_arr = np.array(history['v'])
        print(f"\n  Lap time  : {lap_time:.3f} s")
        print(f"  Avg speed : {v_arr.mean():.3f} m/s")
        print(f"  Max speed : {v_arr.max():.3f} m/s")
        print(f"  Target    : < 14 s   {'✓ PASS' if lap_time < 14 else '✗ FAIL'}")
    else:
        print("\n  Did not complete the course.")

    plot_run(history, title=f"Line-follow sim  –  lap = "
             + (f"{lap_time:.2f} s" if lap_time else "DNF"),
             params=params)
