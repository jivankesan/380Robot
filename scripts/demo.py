#!/usr/bin/env python3
"""Autonomous demo script for 380Robot assessment.

Talks directly to the Arduino over serial — no ROS serial bridge needed.

Sequence:
  1. Forward  (FORWARD_DURATION seconds)
  2. Stop + pause
  3. Backward (BACKWARD_DURATION seconds)
  4. Stop + pause
  5. Claw open   (servo2 = 90)
  6. Claw close  (servo2 = 135)
  7. Tilt        (servo1 = 135)
  8. Level       (servo1 = 150)
  9. Claw open   (servo2 = 90)

Run via:  ./scripts/run_demo.sh
"""

import os
import termios
import time

# ── Tune these ───────────────────────────────────────────────────────────────
SERIAL_PORT      = '/dev/ttyUSB0'
BAUD_RATE        = 115200

FORWARD_PWM      =  100   # out of 255  (~0.3 m/s)
BACKWARD_PWM     = -100

FORWARD_DURATION  = 2.0   # seconds
BACKWARD_DURATION = 2.0
PAUSE_DURATION    = 1.0
CLAW_STEP_DELAY   = 1.5   # seconds between claw steps
# ─────────────────────────────────────────────────────────────────────────────


def open_serial(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty = termios.tcgetattr(fd)

    # Raw mode, 8N1
    tty[0] = 0   # iflag
    tty[1] = 0   # oflag
    tty[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    tty[3] = 0   # lflag
    tty[4] = termios.B115200
    tty[5] = termios.B115200
    tty[6][termios.VMIN]  = 0
    tty[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, tty)
    return fd


def send(fd, cmd: str):
    msg = (cmd + '\n').encode()
    os.write(fd, msg)
    print(f'  >> {cmd}')


def drive(fd, pwm_left, pwm_right, duration):
    direction = 'FORWARD' if pwm_left > 0 else 'BACKWARD'
    print(f'\n>>> {direction}: PWM={pwm_left} for {duration:.1f}s')
    end = time.time() + duration
    tick = 0
    while time.time() < end:
        send(fd, f'M,{pwm_left},{pwm_right}')
        remaining = end - time.time()
        if tick % 4 == 0:
            print(f'    {remaining:.1f}s remaining')
        tick += 1
        time.sleep(0.05)  # 20 Hz

    send(fd, 'M,0,0')
    print('    stopped')


def servo(fd, num, angle, label=''):
    print(f'\n>>> SERVO {num} -> {angle}  {label}')
    send(fd, f'S,{num},{angle}')


def main():
    print('\n======== 380Robot Demo Start ========')
    print(f'  Port:     {SERIAL_PORT} @ {BAUD_RATE}')
    print(f'  Forward:  PWM={FORWARD_PWM} x {FORWARD_DURATION:.1f}s')
    print(f'  Backward: PWM={BACKWARD_PWM} x {BACKWARD_DURATION:.1f}s')
    print('=====================================\n')

    fd = open_serial(SERIAL_PORT, BAUD_RATE)
    print('Serial port open. Waiting 2s for Arduino to boot...')
    time.sleep(2.0)

    try:
        # ── Reset servos to start position (open + level) ───────────────────
        print('\n>>> Resetting servos to start position...')
        servo(fd, 1, 135, 'parallel')
        time.sleep(0.5)
        servo(fd, 2, 90,  'open')
        time.sleep(1.0)

        # ── Mobility demo ───────────────────────────────────────────────────
        drive(fd, FORWARD_PWM, FORWARD_PWM, FORWARD_DURATION)
        print(f'Pausing {PAUSE_DURATION:.1f}s...')
        time.sleep(PAUSE_DURATION)

        drive(fd, BACKWARD_PWM, BACKWARD_PWM, BACKWARD_DURATION)
        print(f'Pausing {PAUSE_DURATION:.1f}s...')
        time.sleep(PAUSE_DURATION)

        # ── Claw demo ───────────────────────────────────────────────────────
        servo(fd, 2, 90,  'open')
        time.sleep(CLAW_STEP_DELAY)

        servo(fd, 2, 150, 'close')
        time.sleep(CLAW_STEP_DELAY)

        servo(fd, 1, 90,  'tilt down')
        time.sleep(CLAW_STEP_DELAY)

        servo(fd, 1, 135, 'tilt back / parallel')
        time.sleep(CLAW_STEP_DELAY)

        servo(fd, 2, 90,  'open')
        time.sleep(CLAW_STEP_DELAY)

    finally:
        send(fd, 'M,0,0')
        os.close(fd)

    print('\n======== Demo Complete ========\n')


if __name__ == '__main__':
    main()
