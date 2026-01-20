# 380Robot Architecture

## System Overview

The 380Robot is a ROS 2-based autonomous robot designed for line following and pick-and-place tasks. The system uses a camera as its primary sensor and communicates with an Arduino Mega 2560 for motor and claw control.

## Node Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PERCEPTION                                      │
│                                                                             │
│  ┌──────────────┐     ┌───────────────────┐     ┌───────────────────┐      │
│  │  v4l2_camera │────▶│ line_detector_node│────▶│/vision/line_obs   │      │
│  │              │     │    (Python)       │     └───────────────────┘      │
│  │              │     └───────────────────┘                                │
│  │              │                                                          │
│  │              │     ┌───────────────────┐     ┌───────────────────┐      │
│  │              │────▶│object_detector_   │────▶│/vision/detections │      │
│  │              │     │     node (Python) │     └───────────────────┘      │
│  └──────────────┘     └───────────────────┘                                │
│         │                                                                   │
│         ▼                                                                   │
│  /camera/image_raw                                                          │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                               CONTROL                                        │
│                                                                             │
│  ┌───────────────────┐     ┌───────────────────┐     ┌─────────────────┐   │
│  │line_follow_       │────▶│  speed_profile_   │────▶│  safety_stop_   │   │
│  │controller (C++)   │     │    node (C++)     │     │   node (C++)    │   │
│  └───────────────────┘     └───────────────────┘     └─────────────────┘   │
│         │                          │                         │              │
│         ▼                          ▼                         ▼              │
│  /control/cmd_vel         /control/cmd_vel_limited    /hw/cmd_vel          │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            TASK MANAGEMENT                                   │
│                                                                             │
│                      ┌───────────────────────┐                              │
│                      │   task_fsm_node       │                              │
│                      │       (C++)           │                              │
│                      └───────────────────────┘                              │
│                               │                                             │
│                      /claw/cmd │ /control/enable                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            HARDWARE                                          │
│                                                                             │
│                      ┌───────────────────────┐                              │
│                      │  serial_bridge_node   │                              │
│                      │       (C++)           │                              │
│                      └───────────────────────┘                              │
│                               │                                             │
│                        Serial │ /dev/ttyACM0                                │
│                               ▼                                             │
│                      ┌───────────────────────┐                              │
│                      │   Arduino Mega 2560   │                              │
│                      │   - Motor Driver      │                              │
│                      │   - Claw Servo        │                              │
│                      │   - Encoders          │                              │
│                      └───────────────────────┘                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Vision Pipeline

1. **Camera** captures images at 30 FPS
2. **Line Detector** processes images:
   - Crops to ROI (bottom half)
   - Converts to HSV, thresholds for line
   - Finds contours, computes centroid
   - Outputs lateral error, heading error, curvature
3. **Object Detector** runs in parallel:
   - Heuristic color detection or YOLO
   - Outputs bounding boxes with class and confidence

### Control Pipeline

1. **Line Follow Controller** receives `LineObservation`:
   - PD control on lateral and heading error
   - Outputs raw `cmd_vel` (v, omega)
2. **Speed Profile** applies rate limiting:
   - Trapezoidal acceleration profile
   - Curvature-based speed reduction
   - Outputs `cmd_vel_limited`
3. **Safety Stop** gates output:
   - Watchdog for timeouts
   - E-stop monitoring
   - Battery voltage check

### Task State Machine

States:
- `INIT` → Wait for sensors ready
- `FOLLOW_LINE_SEARCH` → Follow line, look for target
- `APPROACH_TARGET` → Move toward detected object
- `PICKUP` → Stop, close claw
- `RETURN_FOLLOW_LINE` → Follow line back
- `DROP` → Open claw
- `DONE` / `FAILSAFE_STOP`

## TF Tree

```
map (optional)
└── odom (optional)
    └── base_footprint
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_link
            ├── camera_link
            │   └── camera_optical_frame
            └── claw_link
```

## Message Types

| Message | Package | Description |
|---------|---------|-------------|
| `LineObservation` | robot_interfaces | Line detection result |
| `Detection2D` | robot_interfaces | Single object detection |
| `Detections2D` | robot_interfaces | Array of detections |
| `HwStatus` | robot_interfaces | Hardware telemetry |
| `ClawCommand` | robot_interfaces | Claw actuator command |
| `MotorCommand` | robot_interfaces | Raw motor command |

## Key Topics

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | camera | vision nodes |
| `/vision/line_observation` | LineObservation | line_detector | controller, fsm |
| `/vision/detections` | Detections2D | object_detector | fsm |
| `/control/cmd_vel` | Twist | controller | speed_profile |
| `/control/cmd_vel_limited` | Twist | speed_profile | safety, hw |
| `/hw/status` | HwStatus | serial_bridge | safety, fsm |
| `/claw/cmd` | ClawCommand | fsm | serial_bridge |

## Configuration Files

Located in `ros2_ws/src/robot_bringup/config/`:

- `camera.yaml` - Camera device and resolution
- `vision.yaml` - ROI, thresholds, detector settings
- `control.yaml` - PD gains, speed limits
- `fsm.yaml` - State machine thresholds
- `hw.yaml` - Serial port, motor calibration
