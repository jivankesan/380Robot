# 380Robot - ROS 2 Line-Following Robot

A ROS 2 Jazzy-based autonomous robot system for line following, object detection, and pick-and-place tasks. Designed for Raspberry Pi 5 deployment with Arduino Mega 2560 for motor/claw control.

## Features

- **Line Following**: Camera-based line detection and tracking
- **Object Detection**: LEGO person (or configurable target) detection using color heuristics
- **Pick and Place**: Claw-based grasping and releasing
- **Safety Systems**: Watchdog timers and fail-safe behaviors

## Architecture

- **Vision (Python)**: Line detection, object detection via OpenCV
- **Control (C++)**: PD controller with trapezoidal speed profiles
- **FSM (C++)**: Task-level state machine
- **Hardware Interface (C++)**: Serial bridge to Arduino

## Prerequisites

- Docker Desktop (macOS/Windows) or Docker Engine (Linux)
- `docker compose` (v2+)

## Quick Start

### 1. Clone and Setup

```bash
git clone <repo-url> 380Robot
cd 380Robot

# Copy environment template
cp .env.example .env
```

### 2. Build and Start Docker Container

```bash
docker compose build
docker compose up -d
docker compose exec robot-dev bash
```

### 3. Build ROS Workspace (inside container)

```bash
cd /workspaces/380Robot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Important**: You must run `source install/setup.bash` after every build and in every new terminal session.

### 4. Run the Robot

**Development mode (no hardware - uses simulated camera):**
```bash
ros2 launch robot_bringup bringup_dev.launch.py
```

**Real robot (with camera and Arduino connected):**
```bash
ros2 launch robot_bringup bringup_real.launch.py
```

**Teleop mode (manual keyboard control):**
```bash
ros2 launch robot_bringup teleop.launch.py
```

## Quick Reference Commands

```bash
# Enter the development container
docker compose exec robot-dev bash

# Rebuild after code changes (inside container)
cd /workspaces/380Robot/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Clean rebuild (if you have issues)
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# View available topics
ros2 topic list

# Monitor line detection
ros2 topic echo /vision/line_observation

# Monitor FSM state
ros2 topic echo /control/fsm_state

# Stop the container (from host)
docker compose down
```

## Directory Structure

```
380Robot/
├── docker/                 # Docker configuration
│   ├── Dockerfile
│   └── entrypoint.sh
├── compose/
│   └── docker-compose.yml
├── ros2_ws/               # ROS 2 workspace
│   └── src/
│       ├── robot_interfaces/    # Custom messages/services
│       ├── robot_bringup/       # Launch files and configs
│       ├── robot_description/   # URDF and TF
│       ├── robot_vision_py/     # Python vision nodes
│       ├── robot_control_cpp/   # C++ control nodes
│       ├── robot_fsm_cpp/       # C++ state machine
│       └── robot_hw_cpp/        # C++ hardware interface
├── firmware/              # Arduino code
│   └── arduino_mega/
├── scripts/               # Helper scripts
├── docs/                  # Documentation
└── README.md
```

## Configuration

Configuration files are in `ros2_ws/src/robot_bringup/config/`:

| File | Description |
|------|-------------|
| `camera.yaml` | Camera device and resolution settings |
| `vision.yaml` | Line detection thresholds, ROI, object detection |
| `control.yaml` | PD gains, speed limits, acceleration |
| `fsm.yaml` | State machine thresholds and timing |
| `hw.yaml` | Serial port, motor calibration |

## Hardware Setup

### Required Components

- Raspberry Pi 5 (or development machine)
- Arduino Mega 2560
- USB webcam or Pi Camera
- Differential drive chassis with motors
- Servo-driven claw
- L298N or similar motor driver

### Arduino Setup

1. Open `firmware/arduino_mega/src/main.ino` in Arduino IDE
2. Adjust pin definitions to match your wiring
3. Upload to Arduino Mega 2560

### Serial Connection (Linux)

```bash
# Add user to dialout group for serial access
sudo usermod -aG dialout $USER
# Log out and back in

# Verify Arduino is detected
ls /dev/ttyACM*
```

## Troubleshooting

### Container Issues

```bash
# Rebuild container from scratch
docker compose build --no-cache
docker compose up -d
```

### Build Issues

```bash
# Clean rebuild
cd /workspaces/380Robot/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

### "Package not found" Error

Always source the workspace after building:
```bash
source /workspaces/380Robot/ros2_ws/install/setup.bash
```

### Camera Not Working

On Linux, uncomment device passthrough in `compose/docker-compose.yml`:
```yaml
devices:
  - /dev/video0:/dev/video0
```

## Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Serial Protocol](docs/PROTOCOL.md)
- [Tuning Guide](docs/TUNING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## License

MIT License
