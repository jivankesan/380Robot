#!/usr/bin/env python3
"""
Standalone pickup test — no ROS required.
Talks directly to the Arduino over serial.

Usage (on the Pi):
  python3 /workspaces/380Robot/scripts/test_pickup.py [/dev/ttyUSB0]
"""
import sys
import time
import serial

PORT     = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD     = 115200
FWD_PWM  = 80   # both wheels forward (max_pwm=100)
FWD_TIME = 0.5  # seconds to drive forward

# Servo angles from hw.yaml
SERVO2_OPEN   = 70    # gripper open
SERVO2_CLOSED = 150   # gripper closed
SERVO1_HOME   = 90    # rotation home (level)
SERVO1_CARRY  = 135   # rotation carry (after pickup)

WATCHDOG_MS = 250  # Arduino cuts motors if no command within this interval

def send(ser, cmd: str):
    line = cmd.strip() + "\n"
    ser.write(line.encode())
    print(f"  >> {cmd}")

def drive_for(ser, left_pwm, right_pwm, duration_s):
    """Keep sending motor command at watchdog rate for duration_s."""
    interval = (WATCHDOG_MS / 1000.0) / 2.0  # send at 2x watchdog rate
    end = time.time() + duration_s
    while time.time() < end:
        send(ser, f"M,{left_pwm},{right_pwm}")
        time.sleep(interval)

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)  # let Arduino boot/reset after serial open

        print("Resetting to initial position...")
        send(ser, "M,0,0")
        send(ser, f"C,2,{SERVO2_OPEN}")
        send(ser, f"C,1,{SERVO1_HOME}")
        time.sleep(1.0)

        print(f"Driving forward for {FWD_TIME}s...")
        drive_for(ser, FWD_PWM, FWD_PWM, FWD_TIME)

        print("Stopping motors...")
        send(ser, "M,0,0")
        time.sleep(0.4)

        print("Closing gripper (servo 2)...")
        send(ser, f"C,2,{SERVO2_CLOSED}")
        time.sleep(1.0)

        print("Rotating claw to carry (servo 1)...")
        send(ser, f"C,1,{SERVO1_CARRY}")
        time.sleep(0.5)

        print("Done.")

if __name__ == "__main__":
    main()
