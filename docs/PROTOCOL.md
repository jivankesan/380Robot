# Serial Communication Protocol

This document describes the serial protocol between the ROS 2 serial bridge node and the Arduino Mega 2560.

## Connection Settings

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Message Format

All messages are ASCII text, terminated with newline (`\n`).
Fields are separated by commas (`,`).

## Commands (ROS → Arduino)

### Motor Command

Sets motor PWM values for differential drive.

```
M,<left_pwm>,<right_pwm>\n
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `left_pwm` | int | -255 to 255 | Left motor PWM (negative = reverse) |
| `right_pwm` | int | -255 to 255 | Right motor PWM (negative = reverse) |

**Example:**
```
M,150,-150\n    # Spin in place (left forward, right reverse)
M,200,200\n     # Drive forward
M,0,0\n         # Stop
```

### Claw Command

Controls the claw servo.

```
C,<mode>,<position>\n
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `mode` | int | 0-2 | 0=open, 1=close, 2=hold (use position) |
| `position` | int | 0-1000 | Position scale (0=open, 1000=closed) |

**Example:**
```
C,0,0\n       # Open claw
C,1,1000\n    # Close claw
C,2,500\n     # Hold at 50% position
```

## Telemetry (Arduino → ROS)

### Status Telemetry

Sent at ~20 Hz.

```
T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n
```

| Field | Type | Description |
|-------|------|-------------|
| `battery_mv` | int | Battery voltage in millivolts |
| `left_enc` | long | Left encoder count (cumulative) |
| `right_enc` | long | Right encoder count (cumulative) |
| `estop` | int | E-stop state (0=normal, 1=active) |

**Example:**
```
T,12450,1523,-1489,0\n   # 12.45V, encoders, no estop
T,11200,2000,2000,1\n    # Low battery, estop active
```

### Error Message

Sent when an error occurs.

```
E,<code>,<message>\n
```

| Field | Type | Description |
|-------|------|-------------|
| `code` | int | Error code |
| `message` | string | Human-readable message |

**Error Codes:**

| Code | Meaning |
|------|---------|
| 0 | Info/Status (e.g., "Arduino ready") |
| 1 | Unknown command |
| 2 | Invalid motor command |
| 3 | Invalid claw command |
| 4 | Watchdog timeout |
| 5 | Hardware fault |

**Example:**
```
E,0,Arduino ready\n
E,2,Invalid motor command\n
```

## Safety Features

### Watchdog Timer

If no motor command is received within 250ms, motors are automatically stopped.
This prevents runaway behavior if communication is lost.

### E-Stop

Physical e-stop button immediately stops motors when pressed.
E-stop state is reported in telemetry.

### PWM Limits

All PWM values are clamped to [-255, 255] range.

## Timing

| Parameter | Value |
|-----------|-------|
| Command rate (ROS) | 50 Hz |
| Telemetry rate (Arduino) | 20 Hz |
| Watchdog timeout | 250 ms |

## Example Session

```
# Arduino boots
E,0,Arduino ready

# ROS sends motor commands
M,100,100
M,100,100

# Arduino sends telemetry
T,12300,45,47,0
T,12290,92,94,0

# ROS opens claw
C,0,0

# ROS stops motors
M,0,0

# Arduino sends telemetry
T,12280,100,102,0
```

## Implementation Notes

### ROS Side (`serial_bridge_node`)

1. Parse incoming telemetry by splitting on commas
2. Handle partial messages (buffer until newline)
3. Implement watchdog on ROS side as backup
4. Log errors received from Arduino

### Arduino Side

1. Use non-blocking serial reads
2. Buffer incoming characters until newline
3. Send telemetry at fixed rate using millis()
4. Implement watchdog using millis() comparison
