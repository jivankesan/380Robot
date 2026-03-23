#!/usr/bin/env python3
"""
test_spin180.py — Find the right motor commands for a 180-degree spin.

For a consistent spin, BOTH alternating commands must rotate the robot
in the same direction. Test each mode to find which one works:

  Mode A: M(-pwm, 0) / M(0, +pwm)  — original (left back / right fwd)
  Mode B: M(+pwm, 0) / M(0, +pwm)  — left fwd  / right fwd  (try if A waggles)
  Mode C: M(-pwm, 0) / M(0, -pwm)  — left back / right back
  Mode D: M(0, +pwm) only           — right wheel only
  Mode E: M(-pwm, 0) only           — left wheel only

Usage:
    python3 scripts/test_spin180.py --mode B --time 2.0
"""

import argparse
import serial
import time

BAUD = 115200

MODES = {
    "A": ("M(-pwm,0) / M(0,+pwm)", lambda p: (-p, 0), lambda p: (0,  p)),
    "B": ("M(+pwm,0) / M(0,+pwm)", lambda p: ( p, 0), lambda p: (0,  p)),
    "C": ("M(-pwm,0) / M(0,-pwm)", lambda p: (-p, 0), lambda p: (0, -p)),
    "D": ("M(0,+pwm) only",         lambda p: ( 0,  p), lambda p: (0,  p)),
    "E": ("M(-pwm,0) only",         lambda p: (-p,  0), lambda p: (-p, 0)),
}


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.005)


def spin(ser, spin_time_s, pwm, ticks_per_side, cmd_a, cmd_b):
    tick_period = 0.02
    total_ticks = int(spin_time_s / tick_period)
    toggle = False
    tick_count = 0

    for _ in range(total_ticks):
        t_start = time.monotonic()
        l, r = cmd_a(pwm) if toggle else cmd_b(pwm)
        send(ser, f"M,{l},{r}")
        tick_count += 1
        if tick_count >= ticks_per_side:
            toggle = not toggle
            tick_count = 0
        elapsed = time.monotonic() - t_start
        remaining = tick_period - elapsed
        if remaining > 0:
            time.sleep(remaining)

    send(ser, "M,0,0")
    print("Done — motors stopped.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",  default="/dev/ttyUSB0")
    parser.add_argument("--mode",  default="A", choices=list(MODES.keys()),
                        help="Which command pair to test (default: A)")
    parser.add_argument("--time",  type=float, default=2.0,
                        help="Spin duration in seconds")
    parser.add_argument("--pwm",   type=int,   default=150)
    parser.add_argument("--ticks", type=int,   default=5,
                        help="Ticks per side before switching (5 = 100ms per side at 50Hz)")
    args = parser.parse_args()

    label, cmd_a, cmd_b = MODES[args.mode]
    print(f"Mode {args.mode}: {label}")
    print(f"  time={args.time}s  pwm={args.pwm}  ticks_per_side={args.ticks}")

    with serial.Serial(args.port, BAUD, timeout=1) as ser:
        time.sleep(2.0)
        while ser.in_waiting:
            print("Arduino:", ser.readline().decode().strip())

        input("\nPlace robot on floor. Press Enter to spin...")
        spin(ser, args.time, args.pwm, args.ticks, cmd_a, cmd_b)

    print("\nTry all modes until one spins cleanly:")
    for m, (desc, _, _) in MODES.items():
        print(f"  --mode {m}  {desc}")


if __name__ == "__main__":
    main()
