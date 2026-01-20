# Arduino Mega 2560 Firmware

Firmware for motor and claw control on the 380Robot.

## Hardware Setup

### Pin Assignments

| Function | Pin | Notes |
|----------|-----|-------|
| Left Motor Enable | 5 | PWM |
| Left Motor IN1 | 22 | Direction |
| Left Motor IN2 | 23 | Direction |
| Right Motor Enable | 6 | PWM |
| Right Motor IN1 | 24 | Direction |
| Right Motor IN2 | 25 | Direction |
| Claw Servo | 9 | PWM |
| Left Encoder A | 2 | Interrupt |
| Left Encoder B | 3 | |
| Right Encoder A | 18 | Interrupt |
| Right Encoder B | 19 | |
| Battery Sense | A0 | Voltage divider |
| E-Stop Button | 52 | Active LOW |

### Motor Driver

Designed for L298N or similar H-bridge driver:
- EN pin controls speed via PWM
- IN1/IN2 control direction

### Claw Servo

Standard hobby servo:
- Open position: 30 degrees
- Closed position: 120 degrees

### Battery Monitoring

Uses a voltage divider (10k/10k) to scale battery voltage to 0-5V range.

## Serial Protocol

See `protocol.md` for detailed protocol specification.

**Quick Reference:**

Commands (ROS → Arduino):
```
M,<left_pwm>,<right_pwm>\n    Motor command
C,<mode>,<pos>\n              Claw command
```

Telemetry (Arduino → ROS):
```
T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
E,<code>,<message>\n
```

## Building and Uploading

1. Open `src/main.ino` in Arduino IDE
2. Select Board: "Arduino Mega or Mega 2560"
3. Select Port: `/dev/ttyACM0` (Linux) or appropriate COM port
4. Click Upload

## Safety Features

- **Watchdog Timer**: Motors stop after 250ms without commands
- **E-Stop**: Physical button immediately stops motors
- **PWM Clamping**: Values constrained to [-255, 255]
