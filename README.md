# 380Robot - ROS 2 Line-Following Robot

A ROS 2-based autonomous robot system for line following, object detection, and pick-and-place tasks. Designed for Raspberry Pi 5 deployment with Arduino Mega 2560 for motor/claw control.

## Features

- **Line Following**: Camera-based line detection and tracking
- **Object Detection**: LEGO person (or configurable target) detection
- **Pick and Place**: Claw-based grasping and releasing
- **Safety Systems**: Watchdog timers and fail-safe behaviors

## Architecture

- **Vision (Python)**: Line detection, object detection via OpenCV/YOLO
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

# Edit .env to match your setup (UID/GID, ROS_DISTRO, etc.)
```

### 2. Build Docker Image

```bash
docker compose build
```

### 3. Start Development Container

```bash
docker compose up -d
```

### 4. Enter Development Shell

```bash
./scripts/bootstrap.sh
# Or manually:
docker compose exec robot-dev bash
```

### 5. Build ROS Workspace (inside container)

```bash
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 6. Run the Robot

**Development mode (no hardware):**

```bash
ros2 launch robot_bringup bringup_dev.launch.py
```

**Real robot:**

```bash
ros2 launch robot_bringup bringup_real.launch.py
```

**Teleop mode:**

```bash
ros2 launch robot_bringup teleop.launch.py
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

- `camera.yaml` - Camera settings
- `vision.yaml` - Vision pipeline parameters
- `control.yaml` - Controller gains and limits
- `fsm.yaml` - State machine thresholds
- `hw.yaml` - Hardware interface settings

## Documentation

- [Architecture Overview](docs/ARCHITECTURE.md)
- [Serial Protocol](docs/PROTOCOL.md)
- [Tuning Guide](docs/TUNING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## Hardware Setup

### Required Components

- Raspberry Pi 5 (or development machine)
- Arduino Mega 2560
- USB webcam or Pi Camera
- Differential drive chassis with motors
- Servo-driven claw

### Wiring

See `docs/ARCHITECTURE.md` for detailed wiring diagrams.

## License

MIT License
