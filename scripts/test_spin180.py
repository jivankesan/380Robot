#!/usr/bin/env python3
"""
test_spin180.py — Test the 180-degree pickup spin via direct serial commands.

Sends alternating M(-150,0) / M(0,150) to spin the robot, then stops.
Tune SPIN_TIME_S until the robot lands ~180 degrees from its start heading.

Usage:
    python3 scripts/test_spin180.py [--port /dev/ttyUSB0] [--time 2.0] [--pwm 150] [--ticks 5]
"""

import argparse
import serial
import time

BAUD = 115200


def send(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode())
    time.sleep(0.005)


def stop(ser: serial.Serial):
    send(ser, "M,0,0")


def spin_180(ser: serial.Serial, spin_time_s: float, pwm: int, ticks_per_side: int, direction: int = 1):
    """
    Alternate M(-pwm,0) / M(0,pwm) at 50 Hz for spin_time_s seconds.
    direction: +1 = CCW, -1 = CW
    ticks_per_side: how many 20ms ticks each wheel holds before switching.
    """
    tick_period = 0.02  # 50 Hz
    left_pwm  = -direction * pwm
    right_pwm =  direction * pwm

    total_ticks = int(spin_time_s / tick_period)
    toggle = False
    tick_count = 0

    print(f"Spinning {'CCW' if direction > 0 else 'CW'} for {spin_time_s}s "
          f"(pwm={pwm}, ticks_per_side={ticks_per_side})")

    for i in range(total_ticks):
        t_start = time.monotonic()

        if toggle:
            send(ser, f"M,{left_pwm},0")
        else:
            send(ser, f"M,0,{right_pwm}")

        tick_count += 1
        if tick_count >= ticks_per_side:
            toggle = not toggle
            tick_count = 0

        # Sleep for remainder of tick
        elapsed = time.monotonic() - t_start
        remaining = tick_period - elapsed
        if remaining > 0:
            time.sleep(remaining)

    stop(ser)
    print("Spin complete — motors stopped.")


def main():
    parser = argparse.ArgumentParser(description="Test 180-degree spin")
    parser.add_argument("--port",      default="/dev/ttyUSB0")
    parser.add_argument("--time",      type=float, default=2.0,
                        help="Spin duration in seconds (tune until ~180 deg)")
    parser.add_argument("--pwm",       type=int,   default=150,
                        help="Spin PWM (should match spin_pwm in hw.yaml)")
    parser.add_argument("--ticks",     type=int,   default=5,
                        help="Ticks per side before switching wheel (5 = 100ms at 50Hz)")
    parser.add_argument("--direction", type=int,   default=1, choices=[1, -1],
                        help="+1 = CCW, -1 = CW")
    args = parser.parse_args()

    print(f"Opening {args.port} at {BAUD} baud...")
    with serial.Serial(args.port, BAUD, timeout=1) as ser:
        time.sleep(2.0)  # Wait for Arduino reset

        # Drain startup message
        while ser.in_waiting:
            print("Arduino:", ser.readline().decode().strip())

        input("Press Enter to start spin (robot should be on the floor, clear of obstacles)...")
        spin_180(ser, args.time, args.pwm, args.ticks, args.direction)

        print("\nTuning guide:")
        print("  Spun < 180 deg  →  increase --time")
        print("  Spun > 180 deg  →  decrease --time")
        print("  Wrong direction →  use --direction -1")


if __name__ == "__main__":
    main()
