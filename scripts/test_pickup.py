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
FWD_PWM  = 80   # both wheels forward (above min_pwm=60)
FWD_TIME = 0.5  # seconds to drive forward

def send(ser, cmd: str):
    line = cmd.strip() + "\n"
    ser.write(line.encode())
    print(f"  >> {cmd}")

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)  # let Arduino boot/reset after serial open

        print("Driving forward...")
        send(ser, f"M,{FWD_PWM},{FWD_PWM}")
        time.sleep(FWD_TIME)

        print("Stopping motors...")
        send(ser, "M,0,0")
        time.sleep(0.4)

        print("Closing gripper (servo 2)...")
        send(ser, "C,2,50")   # 50 = closed angle
        time.sleep(1.0)

        print("Rotating claw to carry (servo 1)...")
        send(ser, "C,1,90")   # 90 = carry angle
        time.sleep(0.5)

        print("Done.")

if __name__ == "__main__":
    main()
