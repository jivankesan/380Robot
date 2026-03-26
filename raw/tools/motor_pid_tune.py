#!/usr/bin/env python3
"""
motor_pid_tune.py – Wheel velocity PID step-response tuner.

Runs standalone on the Pi (do NOT run alongside the main robot binary).
Commands a target wheel speed, reads encoders via lgpio, runs the same
feedforward+PID as control.cpp, and plots the full step response.

Requirements:
    sudo apt install python3-lgpio python3-matplotlib python3-numpy python3-serial

Usage:
    python3 motor_pid_tune.py [options]

Examples:
    # Default test (5 rad/s, 3 s, starting gains from config.h)
    python3 motor_pid_tune.py

    # Tune gains without touching C++ code
    python3 motor_pid_tune.py --target 6 --kp 25 --ki 4 --kd 0.2

    # Save data for later comparison
    python3 motor_pid_tune.py --save run1.csv
"""

import argparse
import math
import time
import sys
import csv

import lgpio
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Constants – must match config.h ──────────────────────────────────────────
WHEEL_RADIUS_M        = 0.048
ENCODER_TICKS_PER_REV = 341.2
TICKS_PER_RAD         = ENCODER_TICKS_PER_REV / (2.0 * math.pi)
MAX_PWM               = 255
MIN_PWM               = 30
MAX_WHEEL_OMEGA       = 22.0   # 210 RPM * 2π/60 = 21.99 rad/s

# Encoder GPIO pins (BCM numbering) and gpiochip (Pi 5 = 4, older Pi = 0)
GPIOCHIP    = 4
ENC_LEFT_A  = 27
ENC_LEFT_B  = 17
ENC_RIGHT_A = 24
ENC_RIGHT_B = 23

# Quadrature decode lookup: index = (prev_AB << 2) | new_AB
QEM = [
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
]

# ── Encoder ───────────────────────────────────────────────────────────────────

class QuadratureEncoder:
    def __init__(self, handle, pin_a, pin_b):
        self.h     = handle
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0

        lgpio.gpio_claim_alert(handle, pin_a, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_alert(handle, pin_b, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)

        self.prev_ab = (lgpio.gpio_read(handle, pin_a) << 1) | lgpio.gpio_read(handle, pin_b)
        self._cb_a = lgpio.callback(handle, pin_a, lgpio.BOTH_EDGES, self._cb)
        self._cb_b = lgpio.callback(handle, pin_b, lgpio.BOTH_EDGES, self._cb)

    def _cb(self, chip, gpio, level, tick):
        a = level if gpio == self.pin_a else lgpio.gpio_read(self.h, self.pin_a)
        b = level if gpio == self.pin_b else lgpio.gpio_read(self.h, self.pin_b)
        new_ab = (a << 1) | b
        self.count += QEM[(self.prev_ab << 2) | new_ab]
        self.prev_ab = new_ab

    def get(self):
        return self.count

    def cancel(self):
        self._cb_a.cancel()
        self._cb_b.cancel()
        lgpio.gpio_free(self.h, self.pin_a)
        lgpio.gpio_free(self.h, self.pin_b)

# ── PID controller (mirrors WheelPID in control.cpp) ─────────────────────────

class WheelPID:
    def __init__(self, kp, ki, kd, i_clamp):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.i_clamp = i_clamp
        self.integral  = 0.0
        self.prev_err  = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, target, measured, dt):
        err           = target - measured
        self.integral = max(-self.i_clamp, min(self.i_clamp, self.integral + err * dt))
        d_err         = (err - self.prev_err) / dt
        self.prev_err = err

        ff      = max(-1.0, min(1.0, target / MAX_WHEEL_OMEGA)) * MAX_PWM
        pid_out = self.kp * err + self.ki * self.integral + self.kd * d_err
        raw_pwm = ff + pid_out

        # Apply deadband and clamp (matches apply_min in control.cpp)
        pwm = int(max(-MAX_PWM, min(MAX_PWM, raw_pwm)))
        if   0 < pwm < MIN_PWM:  pwm =  MIN_PWM
        elif -MIN_PWM < pwm < 0: pwm = -MIN_PWM

        return pwm, err, ff, pid_out

# ── Step response test ────────────────────────────────────────────────────────

def run_test(args):
    h = lgpio.gpiochip_open(GPIOCHIP)
    if h < 0:
        sys.exit(f"ERROR: cannot open /dev/gpiochip{GPIOCHIP} (lgpio error {h})")

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except serial.SerialException as e:
        lgpio.gpiochip_close(h)
        sys.exit(f"ERROR: cannot open {args.port}: {e}")

    time.sleep(0.1)  # let serial settle

    enc_l = QuadratureEncoder(h, ENC_LEFT_A, ENC_LEFT_B)
    enc_r = QuadratureEncoder(h, ENC_RIGHT_A, ENC_RIGHT_B)
    pid_l = WheelPID(args.kp, args.ki, args.kd, args.i_clamp)
    pid_r = WheelPID(args.kp, args.ki, args.kd, args.i_clamp)

    dt          = 1.0 / args.rate
    target      = args.target   # rad/s

    # Pre-allocate log arrays
    n_samples   = int(args.duration * args.rate) + 10
    t_log       = np.zeros(n_samples)
    meas_l_log  = np.zeros(n_samples)
    meas_r_log  = np.zeros(n_samples)
    pwm_l_log   = np.zeros(n_samples, dtype=int)
    pwm_r_log   = np.zeros(n_samples, dtype=int)
    err_l_log   = np.zeros(n_samples)
    err_r_log   = np.zeros(n_samples)
    ff_log      = np.zeros(n_samples)
    pid_out_log = np.zeros(n_samples)
    dist_th_log = np.zeros(n_samples)
    dist_l_log  = np.zeros(n_samples)
    dist_r_log  = np.zeros(n_samples)

    prev_enc_l  = enc_l.get()
    prev_enc_r  = enc_r.get()
    prev_t      = time.monotonic()
    dist_l      = 0.0
    dist_r      = 0.0
    idx         = 0

    print(f"\nStep test:  target={target:.2f} rad/s  ({target * WHEEL_RADIUS_M * 1000:.1f} mm/s)")
    print(f"Gains:      Kp={args.kp}  Ki={args.ki}  Kd={args.kd}  i_clamp={args.i_clamp}")
    print(f"Duration:   {args.duration}s  at {args.rate} Hz")
    print("─" * 55)
    print(f"{'t':>6}  {'tgt':>7}  {'meas_L':>8}  {'meas_R':>8}  {'pwm_L':>6}  {'pwm_R':>6}")

    try:
        step_start = time.monotonic()

        while True:
            loop_start = time.monotonic()
            t_elapsed  = loop_start - step_start
            if t_elapsed > args.duration:
                break

            # ── Measure ───────────────────────────────────────────────────────
            now_l  = enc_l.get()
            now_r  = enc_r.get()
            now_t  = time.monotonic()
            meas_dt = now_t - prev_t

            if meas_dt > 0.001:
                omega_l  = (now_l - prev_enc_l) / meas_dt / TICKS_PER_RAD
                omega_r  = (now_r - prev_enc_r) / meas_dt / TICKS_PER_RAD
                dist_l  += abs(now_l - prev_enc_l) / TICKS_PER_RAD * WHEEL_RADIUS_M
                dist_r  += abs(now_r - prev_enc_r) / TICKS_PER_RAD * WHEEL_RADIUS_M
                prev_enc_l = now_l
                prev_enc_r = now_r
                prev_t     = now_t
            else:
                omega_l = meas_l_log[idx - 1] if idx > 0 else 0.0
                omega_r = meas_r_log[idx - 1] if idx > 0 else 0.0

            # ── PID ───────────────────────────────────────────────────────────
            pwm_l, err_l, ff, pid_out = pid_l.update(target, omega_l, dt)
            pwm_r, err_r, _,  _       = pid_r.update(target, omega_r, dt)

            ser.write(f"M,{pwm_l},{pwm_r}\n".encode())

            # ── Log ───────────────────────────────────────────────────────────
            if idx < n_samples:
                t_log[idx]       = t_elapsed
                meas_l_log[idx]  = omega_l
                meas_r_log[idx]  = omega_r
                pwm_l_log[idx]   = pwm_l
                pwm_r_log[idx]   = pwm_r
                err_l_log[idx]   = err_l
                err_r_log[idx]   = err_r
                ff_log[idx]      = ff
                pid_out_log[idx] = pid_out
                dist_th_log[idx] = target * WHEEL_RADIUS_M * t_elapsed
                dist_l_log[idx]  = dist_l
                dist_r_log[idx]  = dist_r

            if idx % int(args.rate // 5) == 0:
                print(f"{t_elapsed:6.2f}  {target:7.2f}  {omega_l:8.2f}  {omega_r:8.2f}  "
                      f"{pwm_l:6d}  {pwm_r:6d}")
            idx += 1

            # Sleep to maintain loop rate
            sleep_t = dt - (time.monotonic() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

    finally:
        ser.write(b"M,0,0\n")
        time.sleep(0.05)
        enc_l.cancel()
        enc_r.cancel()
        lgpio.gpiochip_close(h)
        ser.close()

    n = min(idx, n_samples)
    return {
        't':          t_log[:n],
        'target':     np.full(n, target),
        'meas_l':     meas_l_log[:n],
        'meas_r':     meas_r_log[:n],
        'pwm_l':      pwm_l_log[:n],
        'pwm_r':      pwm_r_log[:n],
        'err_l':      err_l_log[:n],
        'err_r':      err_r_log[:n],
        'ff':         ff_log[:n],
        'pid_out':    pid_out_log[:n],
        'dist_th':    dist_th_log[:n],
        'dist_l':     dist_l_log[:n],
        'dist_r':     dist_r_log[:n],
    }

# ── Metrics ───────────────────────────────────────────────────────────────────

def step_metrics(t, target_val, measured):
    if len(measured) < 10:
        return {}

    steady = float(np.mean(measured[-20:])) if len(measured) >= 20 else float(measured[-1])

    # Rise time: 10% → 90% of target
    t10 = next((t[i] for i, v in enumerate(measured) if v >= 0.10 * target_val), None)
    t90 = next((t[i] for i, v in enumerate(measured) if v >= 0.90 * target_val), None)
    rise_time = float(t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot
    peak = float(np.max(measured))
    overshoot = max(0.0, (peak - target_val) / target_val * 100.0) if target_val > 0 else 0.0

    # Settling time (last time outside ±2% band)
    tol = 0.02 * target_val
    settled_idx = len(measured)
    for i in range(len(measured) - 1, -1, -1):
        if abs(measured[i] - steady) > tol:
            settled_idx = i + 1
            break
    settling_time = float(t[settled_idx]) if settled_idx < len(t) else float(t[-1])

    return {
        'rise_time_ms':   rise_time * 1000 if rise_time else None,
        'overshoot_pct':  overshoot,
        'settling_ms':    settling_time * 1000,
        'sse':            abs(target_val - steady),
        'steady':         steady,
    }

# ── Plot ──────────────────────────────────────────────────────────────────────

def plot(data, args):
    t      = data['t']
    target = data['target'][0]

    m_l = step_metrics(t, target, data['meas_l'])
    m_r = step_metrics(t, target, data['meas_r'])

    print("\n── Step Response Metrics ──────────────────────────────────────────")
    for m, label in [(m_l, 'LEFT '), (m_r, 'RIGHT')]:
        rt = f"{m['rise_time_ms']:.1f} ms" if m.get('rise_time_ms') else "N/A"
        print(f"  {label}  rise={rt:>10}  "
              f"overshoot={m['overshoot_pct']:5.1f}%  "
              f"settle={m['settling_ms']:6.0f} ms  "
              f"SSE={m['sse']:.4f} rad/s  "
              f"steady={m['steady']:.3f} rad/s")

    dist_err_l = abs(data['dist_th'][-1] - data['dist_l'][-1])
    dist_err_r = abs(data['dist_th'][-1] - data['dist_r'][-1])
    print(f"\n  Distance error at t={t[-1]:.1f}s:")
    print(f"    Left:  ideal={data['dist_th'][-1]:.3f} m  "
          f"actual={data['dist_l'][-1]:.3f} m  error={dist_err_l*100:.1f} cm")
    print(f"    Right: ideal={data['dist_th'][-1]:.3f} m  "
          f"actual={data['dist_r'][-1]:.3f} m  error={dist_err_r*100:.1f} cm")
    print("───────────────────────────────────────────────────────────────────")
    print(f"\n  Suggested next step:")
    _suggest(m_l, m_r, args)

    fig = plt.figure(figsize=(15, 11))
    fig.suptitle(
        f"Motor PID Step Response  |  target={target:.2f} rad/s  |  "
        f"Kp={args.kp}  Ki={args.ki}  Kd={args.kd}",
        fontsize=13, fontweight='bold'
    )
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.5, wspace=0.38)

    # ── 1. Velocity step response (full width) ────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, data['target'], 'k--', lw=1.5, label='Target', zorder=3)
    ax1.plot(t, data['meas_l'], color='royalblue', lw=1.5, label='Left  wheel')
    ax1.plot(t, data['meas_r'], color='tomato',    lw=1.5, label='Right wheel')
    ax1.axhline(0.9 * target, color='gray', lw=0.7, ls=':')
    ax1.axhline(1.02 * target, color='green', lw=0.7, ls=':', alpha=0.6)
    ax1.axhline(0.98 * target, color='green', lw=0.7, ls=':', alpha=0.6)
    ax1.fill_between(t, 0.98 * target, 1.02 * target, alpha=0.07, color='green',
                     label='±2% band')
    ax1.set_ylabel('Wheel speed (rad/s)')
    ax1.set_title('Velocity Step Response')
    ax1.legend(loc='lower right', fontsize=8)
    ax1.grid(True, alpha=0.3)

    info = (f"LEFT   rise={m_l.get('rise_time_ms', 0) or 0:.0f} ms  "
            f"OS={m_l['overshoot_pct']:.1f}%  SSE={m_l['sse']:.3f}\n"
            f"RIGHT  rise={m_r.get('rise_time_ms', 0) or 0:.0f} ms  "
            f"OS={m_r['overshoot_pct']:.1f}%  SSE={m_r['sse']:.3f}")
    ax1.text(0.01, 0.06, info, transform=ax1.transAxes, fontsize=8,
             va='bottom', bbox=dict(boxstyle='round', fc='white', alpha=0.85))

    # ── 2. PWM output ─────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, data['pwm_l'], color='royalblue', lw=1.2, label='Left')
    ax2.plot(t, data['pwm_r'], color='tomato',    lw=1.2, label='Right')
    ax2.plot(t, data['ff'],    color='gray',       lw=1.0, ls='--', label='Feedforward')
    ax2.axhline(MAX_PWM, color='red', lw=0.8, ls='--', alpha=0.5, label='Max PWM')
    ax2.set_ylabel('PWM'); ax2.set_title('PWM Output (FF + PID)')
    ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

    # ── 3. Velocity error ─────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, data['err_l'], color='royalblue', lw=1.2, label='Left  error')
    ax3.plot(t, data['err_r'], color='tomato',    lw=1.2, label='Right error')
    ax3.axhline(0, color='k', lw=0.8)
    ax3.fill_between(t, -0.05 * target, 0.05 * target, alpha=0.1, color='green',
                     label='±5% band')
    ax3.set_ylabel('Error (rad/s)'); ax3.set_title('Velocity Error')
    ax3.legend(fontsize=7); ax3.grid(True, alpha=0.3)

    # ── 4. Distance comparison (full width) ───────────────────────────────────
    ax4 = fig.add_subplot(gs[2, :])
    ax4.plot(t, data['dist_th'], 'k--', lw=1.5, label='Ideal (perfect tracking)')
    ax4.plot(t, data['dist_l'],  color='royalblue', lw=1.5, label='Left  actual')
    ax4.plot(t, data['dist_r'],  color='tomato',    lw=1.5, label='Right actual')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Distance (m)')
    ax4.set_title('Distance: Theoretical vs Actual')
    ax4.legend(fontsize=8); ax4.grid(True, alpha=0.3)

    dist_note = (f"Distance error at end:  "
                 f"Left={dist_err_l*100:.1f} cm   Right={dist_err_r*100:.1f} cm")
    ax4.text(0.01, 0.05, dist_note, transform=ax4.transAxes, fontsize=8,
             va='bottom', bbox=dict(boxstyle='round', fc='white', alpha=0.85))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


def _suggest(m_l, m_r, args):
    os_l = m_l.get('overshoot_pct', 0)
    os_r = m_r.get('overshoot_pct', 0)
    sse_l = m_l.get('sse', 0)
    sse_r = m_r.get('sse', 0)
    rt_l  = m_l.get('rise_time_ms')
    rt_r  = m_r.get('rise_time_ms')

    overshoot = max(os_l, os_r)
    sse       = max(sse_l, sse_r)
    rise_slow = (rt_l and rt_l > 200) or (rt_r and rt_r > 200)

    if overshoot > 10:
        print(f"    Overshoot={overshoot:.1f}% — reduce Kp (try {args.kp * 0.75:.1f})"
              f"  or increase Kd (try {args.kd * 1.5:.2f})")
    elif overshoot > 5:
        print(f"    Slight overshoot={overshoot:.1f}% — increase Kd slightly (try {args.kd * 1.25:.2f})")
    elif sse > 0.2:
        print(f"    SSE={sse:.3f} rad/s — increase Ki (try {args.ki * 1.5:.1f})")
    elif rise_slow:
        print(f"    Rise time slow — increase Kp (try {args.kp * 1.25:.1f})")
    else:
        print("    Looks good! Copy gains to config.h:")
        print(f"      MOTOR_KP={args.kp}  MOTOR_KI={args.ki}  MOTOR_KD={args.kd}")

# ── CSV save ──────────────────────────────────────────────────────────────────

def save_csv(data, path, args):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# motor_pid_tune.py',
                    f'kp={args.kp}', f'ki={args.ki}', f'kd={args.kd}',
                    f'target={args.target}'])
        w.writerow(['t', 'target', 'meas_l', 'meas_r', 'pwm_l', 'pwm_r',
                    'err_l', 'err_r', 'ff', 'pid_out', 'dist_th', 'dist_l', 'dist_r'])
        for i in range(len(data['t'])):
            w.writerow([
                f"{data['t'][i]:.4f}", f"{data['target'][i]:.4f}",
                f"{data['meas_l'][i]:.4f}", f"{data['meas_r'][i]:.4f}",
                data['pwm_l'][i], data['pwm_r'][i],
                f"{data['err_l'][i]:.4f}", f"{data['err_r'][i]:.4f}",
                f"{data['ff'][i]:.2f}", f"{data['pid_out'][i]:.2f}",
                f"{data['dist_th'][i]:.4f}", f"{data['dist_l'][i]:.4f}",
                f"{data['dist_r'][i]:.4f}",
            ])
    print(f"\n  Data saved to {path}")

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Motor PID step-response tuner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--port',     default='/dev/ttyUSB0',
                        help='Arduino serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--target',   type=float, default=5.0,
                        help='Target wheel speed in rad/s (default: 5.0 ≈ 0.24 m/s)')
    parser.add_argument('--kp',       type=float, default=20.0)
    parser.add_argument('--ki',       type=float, default=5.0)
    parser.add_argument('--kd',       type=float, default=0.1)
    parser.add_argument('--i_clamp',  type=float, default=4.0,
                        help='Integral clamp in rad (default: 4.0)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Test duration in seconds (default: 3.0)')
    parser.add_argument('--rate',     type=float, default=100.0,
                        help='Control loop rate in Hz (default: 100)')
    parser.add_argument('--save',     default=None,
                        help='Save raw data to CSV file')
    args = parser.parse_args()

    data = run_test(args)

    if args.save:
        save_csv(data, args.save, args)

    plot(data, args)


if __name__ == '__main__':
    main()
