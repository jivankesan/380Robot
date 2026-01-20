# 380Robot Engineering Spec (ROS 2 + Docker)

**Repo root:** `380Robot/` (already created).

This specification is designed to be handed to an automation/coding agent to scaffold the full repository and implement a working ROS 2 system.

The system:
- Runs a ROS 2 stack (development in Docker; deploy on Raspberry Pi).
- Uses **Python** for vision and **C++** for control/state/motion.
- Interfaces with an **Arduino Mega 2560** for motor + claw actuation.
- Uses a single **camera** as the only sensor.

---

## 1. System goals

### 1.1 Functional goals
1. Follow a high-contrast line/path on the floor using only a camera.
2. Detect a LEGO person (or configurable target class) in the forward camera view.
3. Approach the person, close a front claw to pick it up, and confirm grasp (best-effort).
4. Return along the line to a drop zone and release the object.
5. Provide a safe stop and recovery behavior on detection loss or hardware faults.

### 1.2 Non-goals (explicit)
- No SLAM, no LiDAR, no IMU-based localization. (Odometry is optional/placeholder.)
- No complex multi-object tracking; simple detection is sufficient.
- No full autonomy stack (Nav2) required.

### 1.3 Primary constraints
- **Camera-only** sensing.
- Controls/state in **C++**, vision in **Python**.
- Cross-platform development (macOS/Windows/Linux) using Docker.

---

## 2. High-level architecture

### 2.1 ROS graph overview
The system is decomposed into:
- **Perception**: camera driver + vision pipeline producing line geometry and object detections.
- **Control**: line-follow controller, trapezoidal speed profiler, command outputs.
- **Task logic**: finite-state machine (FSM) coordinating follow → detect → approach → pickup → return → drop.
- **Hardware interface**: serial bridge to Arduino for motor PWM + claw servo.

### 2.2 Core ROS concepts
- Standard message types where possible: `sensor_msgs/Image`, `geometry_msgs/Twist`.
- Custom messages/services for detections and low-level commands.
- TF frames: `map` (optional), `odom` (optional), `base_link`, `camera_link`.

---

## 3. Repository layout

All ROS code lives in `ros2_ws/`.

```
380Robot/
  README.md
  380Robot_ENGINEERING_SPEC.md
  .gitignore
  .editorconfig
  .clang-format
  .pre-commit-config.yaml

  docker/
    Dockerfile
    entrypoint.sh
    rosdep-skip-keys.txt

  compose/
    docker-compose.yml

  ros2_ws/
    src/
      robot_interfaces/          # msgs/srvs/actions
      robot_bringup/             # launch files, param sets
      robot_description/         # URDF, meshes, RViz config
      robot_vision_py/           # Python perception nodes
      robot_control_cpp/         # C++ control nodes (line + speed)
      robot_fsm_cpp/             # C++ task state machine
      robot_hw_cpp/              # C++ serial bridge to Arduino
      robot_tools/               # scripts (calibration, bagging, utils)

  firmware/
    arduino_mega/
      README.md
      src/
        main.ino
        protocol.h
        motors.h
        claw.h

  scripts/
    bootstrap.sh
    build.sh
    lint.sh
    test.sh

  docs/
    ARCHITECTURE.md
    PROTOCOL.md
    TUNING.md
    TROUBLESHOOTING.md
```

### 3.1 Naming conventions
- ROS packages: `robot_<domain>_<lang>` suffix (`_py`, `_cpp`) when mixed language.
- Topics are namespaced by domain: `/camera/*`, `/vision/*`, `/control/*`, `/hw/*`.
- Parameters use `snake_case` and grouped namespaces.

---

## 4. ROS 2 distribution strategy

### 4.1 Development default
- Default to **ROS 2 Jazzy** if targeting Ubuntu 24.04.

### 4.2 Deployment fallback
- If the Raspberry Pi OS/Ubuntu version cannot support Jazzy, support **ROS 2 Humble** (Ubuntu 22.04) as a fallback.

### 4.3 Build-time toggle
- Dockerfile supports `ARG ROS_DISTRO=jazzy` (or `humble`).
- Compose can pass `ROS_DISTRO` via build args/environment.

---

## 5. Docker-based dev environment

### 5.1 Requirements
- Docker Desktop (macOS/Windows) or Docker Engine (Linux).
- `docker compose` available.

### 5.2 Dockerfile responsibilities
The Docker image must:
- Install ROS 2 base + required dependencies.
- Set up colcon tooling.
- Run `rosdep` for workspace dependencies.
- Provide an entrypoint that sources ROS and workspace overlays.

### 5.3 `docker/Dockerfile` (spec)
**Key behaviors**:
- Base image: Ubuntu matching distro.
- Install ROS 2 (`ros-base` or `desktop` depending on GUI needs).
- Install developer tools: `build-essential`, `cmake`, `git`, `python3-pip`, `colcon`, `rosdep`, `vcstool`.
- Install Python deps for vision (`opencv-python`, `numpy`, optional `onnxruntime`).
- Create non-root user (recommended) for file ownership matching host.

### 5.4 `compose/docker-compose.yml` (spec)
One main service: `robot-dev`.
- Bind-mount repo to `/workspaces/380Robot`.
- Working directory: `/workspaces/380Robot/ros2_ws`.
- Host networking on Linux; on macOS/Windows use default bridge (document port needs if any).
- Devices:
  - Serial pass-through when on Linux: `/dev/ttyACM0` mapped.
  - On macOS/Windows, serial pass-through may require manual config (documented in `docs/TROUBLESHOOTING.md`).

### 5.5 Entrypoint
`docker/entrypoint.sh`:
- Source `/opt/ros/$ROS_DISTRO/setup.bash`.
- If `/workspaces/380Robot/ros2_ws/install/setup.bash` exists, source it.
- Exec passed command.

### 5.6 Developer workflows
Inside container:
- `rosdep install --from-paths src --ignore-src -r -y`
- `colcon build --symlink-install`
- `source install/setup.bash`

---

## 6. ROS 2 workspace & packages

### 6.1 `robot_interfaces`
Defines custom message/service types.

**Messages**:
1. `robot_interfaces/msg/LineObservation.msg`
   - `float32 lateral_error_m`  # +right
   - `float32 heading_error_rad` # +ccw
   - `float32 curvature_1pm`     # signed
   - `bool valid`
   - `builtin_interfaces/Time stamp`

2. `robot_interfaces/msg/Detection2D.msg`
   - `string class_name`
   - `float32 score`
   - `float32 cx`  # normalized 0..1
   - `float32 cy`
   - `float32 w`
   - `float32 h`

3. `robot_interfaces/msg/Detections2D.msg`
   - `builtin_interfaces/Time stamp`
   - `string frame_id`
   - `Detection2D[] detections`

4. `robot_interfaces/msg/HwStatus.msg`
   - `builtin_interfaces/Time stamp`
   - `float32 battery_v`
   - `int32 left_enc`
   - `int32 right_enc`
   - `bool estop`
   - `string last_error`

5. `robot_interfaces/msg/ClawCommand.msg`
   - `uint8 mode` # 0=open, 1=close, 2=hold
   - `float32 position` # 0..1 optional

**Services**:
- `robot_interfaces/srv/SetMode.srv`
  - Request: `string mode`
  - Response: `bool ok`, `string message`

### 6.2 `robot_bringup`
- Launch files for dev, real robot, and simulation.
- Parameter YAML files.

**Launch files**:
- `launch/bringup_real.launch.py`
- `launch/bringup_dev.launch.py` (no hardware; uses dummy drivers)
- `launch/teleop.launch.py`

**Config**:
- `config/camera.yaml`
- `config/vision.yaml`
- `config/control.yaml`
- `config/fsm.yaml`
- `config/hw.yaml`

### 6.3 `robot_description`
- `urdf/robot.urdf.xacro`
- `rviz/robot.rviz`
- Frames:
  - `base_link`
  - `base_footprint`
  - `camera_link`
  - `claw_link`

### 6.4 `robot_vision_py`
Python perception nodes.

**Nodes**:
1. `camera_node` (choose one)
   - Option A (real): `v4l2_camera` or `libcamera` wrapper.
   - Option B (dev): image publisher from file/video.

2. `line_detector_node`
   - Sub: `/camera/image_raw` (`sensor_msgs/Image`)
   - Pub: `/vision/line_observation` (`LineObservation`)
   - Steps (baseline):
     - Resize + crop ROI (lower half of image).
     - Convert to HSV/gray.
     - Threshold to isolate line.
     - Morphology (open/close).
     - Find largest contour / compute centroid.
     - Fit line or polynomial for heading + curvature estimate.
     - Output errors in robot frame approximation.

3. `object_detector_node`
   - Sub: `/camera/image_raw`
   - Pub: `/vision/detections` (`Detections2D`)
   - MVP: simple color/shape heuristic OR lightweight ML model.
   - Config params for class name, confidence threshold.

**Optional**:
- `debug_image_node` publishing annotated images to `/vision/debug_image`.

### 6.5 `robot_control_cpp`
C++ control nodes.

**Nodes**:
1. `line_follow_controller_node`
   - Sub: `/vision/line_observation` (`LineObservation`)
   - Pub: `/control/cmd_vel` (`geometry_msgs/Twist`)
   - Parameters:
     - `kp_lateral`, `kp_heading`, `kd_lateral`, `kd_heading`
     - `base_speed_mps` (nominal)
     - `max_ang_vel_rps`, `max_lin_vel_mps`
     - `lost_line_timeout_s`
   - Control law (example):
     - `omega = clamp(kp_lateral*e + kp_heading*theta + kd*de/dt, ±max_ang)`
     - `v` provided by speed profiler (below) or fixed base speed.

2. `speed_profile_node` (trapezoidal profile)
   - Sub: `/vision/line_observation` OR `/control/curvature` (if separated)
   - Sub: `/control/raw_cmd_vel` (optional)
   - Pub: `/control/cmd_vel_limited` (`geometry_msgs/Twist`)
   - Parameters:
     - `v_max`, `v_min`
     - `a_max` (linear accel), `alpha_max` (ang accel)
     - `jerk_max` (optional)
     - `curvature_slowdown_gain`, `error_slowdown_gain`
   - Behavior:
     - Compute **target v** based on curvature/error:
       - `v_target = clamp(v_max - k1*abs(curvature) - k2*abs(lateral_error), v_min, v_max)`
     - Apply trapezoidal rate limiting:
       - `v = clamp(v_prev + clamp(v_target - v_prev, -a_max*dt, a_max*dt), v_min, v_max)`
     - Optionally rate-limit `omega` similarly.

3. `safety_stop_node`
   - Sub: `/hw/status` and internal watchdog timers.
   - Pub: `/control/estop` (bool) and/or publishes zero velocity to `/control/cmd_vel_limited` when triggered.

### 6.6 `robot_fsm_cpp`
Task-level finite state machine.

**Node**: `task_fsm_node`
- Inputs:
  - `/vision/line_observation`
  - `/vision/detections`
  - `/hw/status`
- Outputs:
  - `/control/mode` (via `SetMode` service OR topic)
  - `/control/fsm_state` (`std_msgs/String`)
  - `/claw/cmd` (`ClawCommand`)
  - May gate velocity by publishing `/control/behavior_cmd_vel`.

**States (minimum viable)**:
1. `INIT`
   - Wait for camera + hardware ready.
2. `FOLLOW_LINE_SEARCH`
   - Follow line at nominal speed.
3. `APPROACH_TARGET`
   - When detection score above threshold and centered enough, slow down and align.
4. `PICKUP`
   - Stop, close claw, wait `pickup_hold_time_s`.
5. `RETURN_FOLLOW_LINE`
   - Follow line back; optionally use a marker condition for drop zone.
6. `DROP`
   - Stop, open claw.
7. `DONE`
8. `FAILSAFE_STOP`
   - Stop on loss of line too long, estop, low battery, serial errors.

**Transitions** (examples):
- `FOLLOW_LINE_SEARCH → APPROACH_TARGET` if detection present, `class_name==target`, score≥thr.
- `APPROACH_TARGET → PICKUP` if bbox center within tolerance and bbox size indicates close.
- `PICKUP → RETURN_FOLLOW_LINE` after closing and hold.
- `RETURN_FOLLOW_LINE → DROP` when drop zone detected (line marker) or after time/distance heuristic.
- Any → `FAILSAFE_STOP` if `/hw/status.estop` or timeouts.

### 6.7 `robot_hw_cpp`
Low-level hardware interface to Arduino.

**Node**: `serial_bridge_node`
- Sub: `/control/cmd_vel_limited` (`Twist`)
- Sub: `/claw/cmd` (`ClawCommand`)
- Pub: `/hw/status` (`HwStatus`)
- Parameters:
  - `serial_port` (e.g., `/dev/ttyACM0`)
  - `baudrate` (e.g., 115200)
  - `wheel_base_m`
  - `wheel_radius_m` (if converting to wheel speeds)
  - `max_pwm` (0..255)
  - `cmd_rate_hz`

**Responsibilities**:
- Convert `Twist (v, omega)` to left/right wheel commands:
  - `v_left = v - omega*wheel_base/2`
  - `v_right = v + omega*wheel_base/2`
  - Map to PWM with calibration gains.
- Send framed commands to Arduino.
- Parse telemetry frames from Arduino.
- Maintain watchdog: if no serial ACK/telemetry for N ms, publish error and stop.

### 6.8 `robot_tools`
Utilities:
- Camera calibration tool.
- Bag record/play scripts.
- Dataset capture for detections.

---

## 7. Topics, services, and TF

### 7.1 Topic list
| Topic | Type | Pub | Sub | Notes |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera | vision | Primary image stream |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera | vision | Calibration |
| `/vision/line_observation` | `LineObservation` | line_detector | control,fsm | Lateral/heading errors |
| `/vision/detections` | `Detections2D` | object_detector | fsm | Target detection |
| `/vision/debug_image` | `sensor_msgs/Image` | vision | RViz | Optional |
| `/control/raw_cmd_vel` | `geometry_msgs/Twist` | line_follow | speed_profile | Optional split |
| `/control/cmd_vel_limited` | `geometry_msgs/Twist` | speed_profile | hw | Final gated command |
| `/claw/cmd` | `ClawCommand` | fsm | hw | Claw actuation |
| `/hw/status` | `HwStatus` | hw | all | Telemetry + errors |
| `/control/fsm_state` | `std_msgs/String` | fsm | debug | State visibility |

### 7.2 Services
- `/control/set_mode` (`SetMode`) (optional): used to switch between `AUTO`, `TELEOP`, `STOP`.

### 7.3 TF frames
- `base_link` is the robot body frame.
- `camera_link` fixed transform from `base_link`.
- `claw_link` fixed transform.

---

## 8. Arduino firmware & serial protocol

### 8.1 Firmware placement
`firmware/arduino_mega/` contains Arduino code and protocol docs.

### 8.2 Command protocol (ASCII, line-based)
Use simple line frames to reduce parsing complexity.

**Motor command**:
- `M,<left_pwm>,<right_pwm>\n`
  - `left_pwm`, `right_pwm` in `[-255, 255]` (sign indicates direction).

**Claw command**:
- `C,<mode>,<pos>\n`
  - `mode`: `0=open`, `1=close`, `2=hold`
  - `pos`: `0..1000` optional servo position scale.

**Telemetry**:
- `T,<battery_mv>,<left_enc>,<right_enc>,<estop>\n`

**Error**:
- `E,<code>,<msg>\n`

### 8.3 Firmware behaviors
- Apply motor PWM outputs with direction.
- Optional encoder counting.
- Watchdog: if no motor command for `>250ms`, stop motors.
- Respond with telemetry at `20–50Hz`.

---

## 9. Control: trapezoidal speed profile

### 9.1 Motivation
We want the robot to speed up on straights, decelerate before turns, then accelerate after turns, with bounded acceleration.

### 9.2 Inputs
From `LineObservation`:
- `lateral_error_m` (`e`)
- `heading_error_rad` (`theta`)
- `curvature_1pm` (`kappa`)

### 9.3 Target speed function
Compute a speed target:
- `v_target = v_max - k1*abs(kappa) - k2*abs(e) - k3*abs(theta)`
- Clamp to `[v_min, v_max]`.

### 9.4 Trapezoidal limiter
Maintain `v_prev` and apply:
- `dv = clamp(v_target - v_prev, -a_max*dt, a_max*dt)`
- `v = v_prev + dv`

Optionally apply a similar limiter to angular velocity:
- `omega = clamp(omega_prev + clamp(omega_target - omega_prev, -alpha_max*dt, alpha_max*dt), -omega_max, omega_max)`

### 9.5 Failsafes
- If `LineObservation.valid == false` for longer than `lost_line_timeout_s`, command `v=0, omega=0` and trigger FSM fallback.

---

## 10. Build, lint, and test requirements

### 10.1 Build
- `colcon build --symlink-install`

### 10.2 Lint
- C++: `ament_clang_format`, `ament_cpplint`.
- Python: `ruff` (or `flake8`) + `black`.
- Launch files: basic syntax checks.

### 10.3 Tests
- Unit tests for:
  - Speed profile limiter.
  - Twist-to-wheel conversion.
  - FSM transition logic (pure functions where possible).

---

## 11. Documentation requirements

Create these docs:
- `docs/ARCHITECTURE.md`: ROS graph and dataflow.
- `docs/PROTOCOL.md`: serial protocol + examples.
- `docs/TUNING.md`: how to tune thresholds, ROI, gains.
- `docs/TROUBLESHOOTING.md`: camera/serial/Docker issues.

---

## 12. Minimum viable bringup plan

### 12.1 Phase 1: Dev without hardware
- Use image/video playback for `/camera/image_raw`.
- Validate line detector produces stable errors.
- Validate controller outputs sensible `/cmd_vel`.

### 12.2 Phase 2: Hardware-in-loop
- Connect Arduino, verify serial commands.
- Drive motors with teleop.

### 12.3 Phase 3: Full autonomy
- Enable FSM and object detector.
- Tune approach thresholds.

---

## 13. Acceptance criteria

1. `docker compose up` brings up a container that can build the workspace with one command.
2. `bringup_dev.launch.py` runs with a sample video and publishes all perception/control topics.
3. On hardware, `teleop.launch.py` can command motors and claw through the serial bridge.
4. On a marked track, robot completes: follow → detect → pickup → return → drop, at least 3/5 runs.

---

## 14. File-by-file implementation checklist (what the agent must generate)

### 14.1 Root
- `README.md` with quickstart: build, run bringup, run teleop.
- `.gitignore`, `.editorconfig`, `.clang-format`.

### 14.2 Docker
- `docker/Dockerfile`
- `docker/entrypoint.sh`
- `compose/docker-compose.yml`

### 14.3 ROS packages
- Full package manifests (`package.xml`, `CMakeLists.txt`, `setup.py` where needed).
- Node implementations:
  - `robot_vision_py`: `line_detector_node.py`, `object_detector_node.py`
  - `robot_control_cpp`: `line_follow_controller_node.cpp`, `speed_profile_node.cpp`, `safety_stop_node.cpp`
  - `robot_fsm_cpp`: `task_fsm_node.cpp`
  - `robot_hw_cpp`: `serial_bridge_node.cpp`
  - `robot_interfaces`: msg/srv definitions

### 14.4 Launch & params
- `robot_bringup/launch/*.launch.py`
- `robot_bringup/config/*.yaml`

### 14.5 Firmware
- Arduino code implementing the serial protocol and motor/claw control.

560** for motor/claw actuation via a simple serial protocol.

---

## 2. Assumptions

### 2.1 Hardware
- Compute: **Raspberry Pi 5** (primary), dev machines: macOS and Windows.
- Actuation: 4-wheel drivetrain (differential drive assumed), claw (servo or motor-driven).
- Controller: **Arduino Mega 2560** to drive motors/servos and read basic telemetry if available.
- Sensor: 1 camera (USB webcam or Pi Camera).

### 2.2 ROS 2 distro
- **Default (dev): ROS 2 Jazzy** on Ubuntu 24.04 container.
- **Fallback (deployment): ROS 2 Humble** if the target OS/device constraints require it.

> Repo is parameterized to support either distro via build args.

---

## 3. High-level architecture

### 3.1 Node graph (conceptual)
- `camera_node` (driver) publishes images.
- `vision_node_py` (Python) publishes:
  - line/track information (offset/heading/curvature)
  - target detections (LEGO person bounding box + confidence)
  - optional drop-zone detection
- `controller_node_cpp` computes velocity commands with trapezoidal/rate-limited profiles.
- `fsm_node_cpp` sequences behaviors (FOLLOW_LINE → APPROACH_TARGET → PICKUP → RETURN → DROP).
- `arduino_bridge_cpp` sends motor/claw commands over serial and optionally receives telemetry.

### 3.2 Data flow
Camera → Vision (Python) → Controller/FSM (C++) → Arduino Bridge → Motors/Claw

---

## 4. Repository layout

All code lives under a ROS 2 workspace `ros2_ws/`.

```
380Robot/
  README.md
  380Robot_ENGINEERING_SPEC.md
  .gitignore
  .editorconfig
  .env.example

  docker/
    ros/
      Dockerfile
      entrypoint.sh
      rosdep-sources.list.d/
        20-custom.list
    scripts/
      dev-shell.sh
      colcon-build.sh
      colcon-test.sh

  docker-compose.yml

  ros2_ws/
    src/
      robot_interfaces/
      robot_bringup/
      robot_description/
      robot_vision_py/
      robot_control_cpp/
      robot_fsm_cpp/
      robot_hw_cpp/

    params/
      camera.yaml
      vision.yaml
      control.yaml
      fsm.yaml
      hw.yaml

    launch/
      bringup.launch.py
      sim.launch.py

    tools/
      bag_record.sh
      bag_play.sh
      calibrate_camera.md

  firmware/
    arduino_mega/
      README.md
      src/
        main.ino
      protocol.md

  docs/
    architecture.md
    wiring.md
    tuning.md
    troubleshooting.md

  ci/
    precommit/
      .pre-commit-config.yaml

  .github/
    workflows/
      ci.yml
```

---

## 5. Docker development environment

### 5.1 Goals
- Same dev environment on macOS and Windows (Docker Desktop).
- ROS tooling preinstalled: `colcon`, `rosdep`, `vcstool`, build essentials.
- Hot-reload style workflow using bind mounts for `ros2_ws`.

### 5.2 `docker/ros/Dockerfile`
Requirements:
- Base image: `ubuntu:24.04` (Jazzy) or `ubuntu:22.04` (Humble) selected by build args.
- Install:
  - ROS 2 base + common packages (`ros-<distro>-ros-base`, `image-transport`, `cv-bridge`, `vision-msgs`, `tf2`, `rclcpp`, `rclpy`, `diagnostic-updater`)
  - Dev tools: `build-essential`, `cmake`, `ninja-build`, `git`, `curl`, `python3-pip`, `clang-format`, `cppcheck`
  - Python deps: `opencv-python`, `numpy`, optional `ultralytics` or `onnxruntime` (depending on chosen detector)
- Create non-root user `ros` with UID/GID defaults that can be overridden (for file permissions on bind mounts).
- Copy `entrypoint.sh` that sources ROS and workspace overlays.

### 5.3 `docker-compose.yml`
Provide one main service: `rosdev`.

**Mounts:**
- `./ros2_ws:/work/ros2_ws`
- `./firmware:/work/firmware`

**Devices:**
- Dev on macOS/Windows: camera passthrough varies; provide optional `--device` sections for Linux, and document Mac/Windows limitations.
- Serial passthrough (Linux): map `/dev/ttyACM0` or `/dev/ttyUSB0`.

**Networking:**
- Use `network_mode: host` on Linux for easy ROS discovery.
- For macOS/Windows (no host mode), use FastDDS simple discovery + explicit `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` settings; keep everything in one container for simplicity.

**Environment variables:**
- `ROS_DOMAIN_ID=0` (configurable)
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `DISPLAY` optional (for GUI on Linux)

### 5.4 Dev scripts
- `docker/scripts/dev-shell.sh`: `docker compose exec rosdev bash`
- `docker/scripts/colcon-build.sh`: builds with `--symlink-install`
- `docker/scripts/colcon-test.sh`: runs tests + prints failures.

---

## 6. ROS 2 workspace

### 6.1 Build system conventions
- C++ packages: `ament_cmake`, C++17.
- Python packages: `ament_python`.
- Use `colcon` with `--symlink-install` for fast iteration.
- All nodes should:
  - Expose parameters with sane defaults.
  - Log via `RCLCPP_*` or `rclpy.logging`.
  - Publish/subscribe with explicit QoS.

### 6.2 Packages

#### 6.2.1 `robot_interfaces`
Defines custom messages and services.

**Messages (minimum set):**
1. `LineState.msg`
   - `float32 lateral_error_m` (left negative / right positive)
   - `float32 heading_error_rad`
   - `float32 curvature` (signed, approx 1/r)
   - `bool valid`

2. `Detection2D.msg`
   - `string class_name`
   - `float32 confidence`
   - `float32 cx` (0..1 normalized)
   - `float32 cy` (0..1 normalized)
   - `float32 w`  (0..1 normalized)
   - `float32 h`  (0..1 normalized)

3. `Detection2DArray.msg`
   - `builtin_interfaces/Time stamp`
   - `Detection2D[] detections`

4. `ClawCommand.msg`
   - `uint8 mode` (0=open, 1=close, 2=hold)
   - `float32 position` (0..1 optional)

5. `MotorCommand.msg`
   - `float32 left`  (normalized -1..1)
   - `float32 right` (normalized -1..1)

**Services (optional but helpful):**
- `SetBool`-style service for enabling/disabling control output.

#### 6.2.2 `robot_vision_py`
Python perception pipeline.

**Nodes:**
- `vision_node`:
  - Subscribes: `/camera/image_raw`, `/camera/camera_info`.
  - Publishes: `/perception/line_state` (`robot_interfaces/LineState`), `/perception/detections` (`Detection2DArray`).

**Line detection algorithm (baseline):**
- ROI crop bottom portion of image.
- Convert to HSV/gray, threshold for line color.
- Morphological clean-up.
- Fit line with contours or Hough.
- Output lateral error (pixel offset → normalized → meters with calibration scalar) and heading error.
- Curvature estimate from polynomial fit across multiple scanlines.

**Object detection options:**
- Baseline: simple color/shape heuristic if LEGO person has distinct color.
- Robust: YOLOv8n / YOLOv5n with `ultralytics` OR ONNX model with `onnxruntime`.

**Parameters:**
- `line_roi_y_start`, `line_threshold_*`, `line_morph_kernel`, `pixels_to_m_scale`.
- `detector_type` (`heuristic|yolo|onnx`), `model_path`, `conf_threshold`, `target_class`.

#### 6.2.3 `robot_control_cpp`
C++ control node turning perception into motion.

**Node:** `controller_node`
- Subscribes: `/perception/line_state`, `/perception/detections`.
- Publishes: `/cmd_vel` (`geometry_msgs/Twist`) and/or `/hw/motor_cmd` (`robot_interfaces/MotorCommand`).

**Control approach:**
- Line following using a PD controller on lateral + heading error:
  - `omega = k_e * e + k_theta * theta`
  - `v_target` from speed policy (below)
- Convert `(v, omega)` to wheel commands for differential drive:
  - `v_l = v - omega * (wheel_base/2)`
  - `v_r = v + omega * (wheel_base/2)`
  - normalize to `[-1, 1]` for `MotorCommand`.

**Trapezoidal / rate-limited speed profile (required):**
- Maintain internal state `v_cmd` updated at control frequency.
- Given target `v_target`, apply acceleration limits:
  - `dv = clamp(v_target - v_cmd, -a_max*dt, a_max*dt)`
  - `v_cmd += dv`
- Optional jerk limit:
  - Maintain `a_cmd`, limit `da` similarly with `j_max`.

**Speed policy:**
- Straight segments allow higher speed; turns require slowdown.
- Define a *turn severity* metric:
  - `severity = w1*abs(curvature) + w2*abs(heading_error) + w3*abs(lateral_error)`
- Compute:
  - `v_target = clamp(v_max - k_s * severity, v_min, v_max)`

**Parameters:**
- `control_rate_hz`, `wheel_base_m`, `v_min`, `v_max`, `a_max`, `j_max` (optional).
- `k_e`, `k_theta`.
- `motor_output_topic` choose `cmd_vel` vs `motor_cmd`.

#### 6.2.4 `robot_fsm_cpp`
C++ mission state machine.

**Node:** `fsm_node`
- Subscribes: `/perception/line_state`, `/perception/detections`, optional `/hw/telemetry`.
- Publishes:
  - `/fsm/state` (`std_msgs/String`)
  - `/fsm/goal` (optional)
  - `/hw/claw_cmd` (`robot_interfaces/ClawCommand`)
  - `/control/enable` (`std_msgs/Bool`) to enable/disable controller output

**States (minimum):**
1. `FOLLOW_LINE_OUTBOUND`
2. `APPROACH_TARGET`
3. `PICKUP`
4. `FOLLOW_LINE_RETURN`
5. `APPROACH_DROPZONE`
6. `DROP`
7. `IDLE/ERROR`

**Transitions (baseline):**
- FOLLOW_LINE_OUTBOUND → APPROACH_TARGET when target detection is stable for `N` frames and within a horizontal window.
- APPROACH_TARGET → PICKUP when detection box size exceeds threshold (close enough) OR line state invalid but detection centered.
- PICKUP: stop motors, close claw, wait `t_grasp`, optionally verify with a simple heuristic (e.g., change in motor current/servo position if available), then return.
- RETURN: follow line until dropzone condition is met.

**Dropzone detection options:**
- Heuristic: special marker on ground (color patch/AprilTag).
- If using AprilTag, add a small `apriltag_ros` node and subscribe to its detections.

**Parameters:**
- `target_class`, `detection_stability_frames`, `target_center_tolerance`, `target_size_close_threshold`.
- `pickup_close_time_s`, `drop_open_time_s`.

#### 6.2.5 `robot_hw_cpp`
Hardware interface to Arduino.

**Node:** `arduino_bridge`
- Subscribes: `/hw/motor_cmd`, `/hw/claw_cmd`.
- Publishes: `/hw/telemetry` (custom msg optional) and `/diagnostics`.

**Serial protocol (minimum viable):**
- 115200 baud.
- ASCII lines terminated by `\n`.
- Commands from ROS → Arduino:
  - Motor: `M,<left_norm>,<right_norm>\n` where values are `-1.0..1.0`
  - Claw: `C,<mode>,<position>\n` where `mode` is `0/1/2` and `position` is `0..1`
- Telemetry Arduino → ROS (optional):
  - `T,<battery_v>,<status>\n`

**Safety:**
- Command watchdog: if no motor command in `timeout_ms`, stop motors.
- Clamp all outputs.

#### 6.2.6 `robot_bringup`
Launch files, parameter loading, and runtime composition.

- Contains `launch/` and references `ros2_ws/params/*.yaml`.
- Exposes a single entry launch: `bringup.launch.py`.

#### 6.2.7 `robot_description` (optional but recommended)
URDF, TF frames:
- `base_link`, `base_footprint`, `camera_link`, `claw_link`.
- Static transforms via `robot_state_publisher`.

---

## 7. ROS topics, frames, and QoS

### 7.1 Topics
| Topic | Type | Publisher | Subscribers | Notes |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera | vision | Use `SensorDataQoS` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera | vision | calibration |
| `/perception/line_state` | `robot_interfaces/LineState` | vision | controller, fsm | best effort ok |
| `/perception/detections` | `robot_interfaces/Detection2DArray` | vision | controller, fsm | reliable recommended |
| `/cmd_vel` | `geometry_msgs/Twist` | controller | (optional) bridge | if using twist pipeline |
| `/hw/motor_cmd` | `robot_interfaces/MotorCommand` | controller | arduino_bridge | normalized outputs |
| `/hw/claw_cmd` | `robot_interfaces/ClawCommand` | fsm | arduino_bridge | open/close |
| `/hw/telemetry` | custom/`std_msgs/String` | arduino_bridge | fsm, logger | optional |
| `/fsm/state` | `std_msgs/String` | fsm | logger/ui | debugging |
| `/control/enable` | `std_msgs/Bool` | fsm | controller | gating |

### 7.2 TF frames
- `map` (optional), `odom` (optional), `base_footprint`, `base_link`, `camera_link`, `claw_link`.
- For MVP, static TF between `base_link` and `camera_link` is sufficient.

---

## 8. Launch and runtime modes

### 8.1 Real robot bringup (`bringup.launch.py`)
Launch order:
1. camera driver (choose one):
   - `v4l2_camera` for USB webcam
   - `libcamera` stack for Pi camera
2. `robot_vision_py/vision_node`
3. `robot_control_cpp/controller_node`
4. `robot_fsm_cpp/fsm_node`
5. `robot_hw_cpp/arduino_bridge`
6. `robot_state_publisher` (optional)

All nodes load YAML params from `ros2_ws/params/`.

### 8.2 Simulation bringup (`sim.launch.py`) (optional)
- If you add Gazebo later, keep a placeholder launch that can spawn the robot and feed synthetic images.

---

## 9. Parameter files

Store in `ros2_ws/params/`:
- `camera.yaml`: device index, resolution, fps.
- `vision.yaml`: ROI, thresholds, detector configs.
- `control.yaml`: gains, wheelbase, speed limits, accel limits.
- `fsm.yaml`: thresholds, timings, class names.
- `hw.yaml`: serial port, baudrate, watchdog.

---

## 10. Firmware (Arduino Mega)

Directory: `firmware/arduino_mega/`

Artifacts:
- `protocol.md` describing exact serial protocol.
- `src/main.ino` implementing:
  - Serial parse loop
  - Motor outputs (PWM pins) and direction pins
  - Servo control for claw
  - Watchdog stop

Minimum deliverable: Arduino accepts `M` and `C` commands and actuates accordingly.

---

## 11. Testing, linting, and CI

### 11.1 C++
- Add `ament_lint_auto`.
- `clang-format` configuration in repo root (`.clang-format`).
- Unit tests with `gtest` for:
  - trapezoid limiter
  - wheel mixing
  - state transition logic (pure functions)

### 11.2 Python
- `pytest` for:
  - line detector on saved sample images
  - detection message formatting

### 11.3 CI (`.github/workflows/ci.yml`)
- Build in Docker (Ubuntu) using the same Dockerfile.
- Run `colcon build` + `colcon test`.

---

## 12. Developer workflow

### 12.1 First-time setup
1. Copy `.env.example` → `.env` and set:
   - `ROS_DISTRO=jazzy` (or `humble`)
   - `ROS_DOMAIN_ID=0`
2. Build container:
   - `docker compose build`
3. Start:
   - `docker compose up -d`
4. Enter shell:
   - `./docker/scripts/dev-shell.sh`

### 12.2 Build and run
Inside container:
- Install deps (first time): `rosdep update && rosdep install --from-paths /work/ros2_ws/src -i -y`
- Build: `colcon build --symlink-install`
- Source: `source /work/ros2_ws/install/setup.bash`
- Launch: `ros2 launch robot_bringup bringup.launch.py`

---

## 13. Acceptance criteria (MVP)

1. **Line follow**: robot follows a test loop for ≥ 2 minutes without losing the line under stable lighting.
2. **Target detection**: LEGO person detection triggers approach within 1–2 seconds when in view.
3. **Pickup**: robot stops and closes claw for configured duration and can transport the LEGO person.
4. **Return + drop**: robot reaches drop zone marker and releases claw.
5. **Safety**: loss of control messages or serial timeout stops motors.

---

## 14. Implementation order (recommended)

1. Scaffold repo + Docker + compose + workspace build pipeline.
2. Implement `robot_interfaces`.
3. Implement camera bringup (USB webcam via `v4l2_camera` as easiest).
4. Implement `robot_vision_py` line detector and publish `LineState`.
5. Implement `robot_control_cpp` with PD + trapezoid limiter; drive robot in place.
6. Implement Arduino protocol + `robot_hw_cpp` bridge; validate motor/claw commands.
7. Implement `robot_fsm_cpp` with minimal states and detection stubs.
8. Add object detection (heuristic → YOLO/ONNX).
9. Add drop-zone detection marker.
10. Add tests + CI.

---

## 15. File-by-file deliverables checklist

### Root
- `README.md`: quickstart + dev commands.
- `.env.example`: env vars.

### Docker
- `docker/ros/Dockerfile`
- `docker/ros/entrypoint.sh`
- `docker-compose.yml`

### ROS packages
- `ros2_ws/src/robot_interfaces/...`
- `ros2_ws/src/robot_vision_py/...`
- `ros2_ws/src/robot_control_cpp/...`
- `ros2_ws/src/robot_fsm_cpp/...`
- `ros2_ws/src/robot_hw_cpp/...`
- `ros2_ws/src/robot_bringup/...`

### Params + launch
- `ros2_ws/params/*.yaml`
- `ros2_ws/launch/*.launch.py`

### Firmware
- `firmware/arduino_mega/src/main.ino`
- `firmware/arduino_mega/protocol.md`

---

## 16. Notes on portability (macOS/Windows)

- Prefer running the full ROS graph inside a **single container** during development.
- For camera/serial passthrough:
  - macOS/Windows: you may need to run camera locally or use a network stream; document this in `docs/troubleshooting.md`.
  - Linux: direct `/dev/video0` and `/dev/ttyACM0` passthrough works well.

---

End of spec.
