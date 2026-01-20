# Tuning Guide

This guide explains how to tune the 380Robot for optimal performance.

## Vision Tuning

### Line Detection

Configuration file: `ros2_ws/src/robot_bringup/config/vision.yaml`

#### ROI (Region of Interest)

Adjust these to focus on the relevant part of the image:

```yaml
roi_y_start: 0.5    # Start from middle (0.0 = top, 1.0 = bottom)
roi_y_end: 1.0      # End at bottom
roi_x_start: 0.1    # Ignore left 10%
roi_x_end: 0.9      # Ignore right 10%
```

**Tuning Tips:**
- Smaller ROI = faster processing, less noise
- Make sure the line is visible in the ROI at all expected distances
- Use `/vision/debug_image` to visualize

#### Color Thresholds (HSV)

For detecting a **dark line on light floor**:

```yaml
line_h_min: 0       # Hue min (0-180)
line_h_max: 180     # Hue max
line_s_min: 0       # Saturation min (0-255)
line_s_max: 50      # Saturation max (low = grayscale)
line_v_min: 0       # Value min (0-255)
line_v_max: 80      # Value max (low = dark)
```

**Tuning Process:**
1. Capture an image with `ros2 run image_view image_view image:=/camera/image_raw`
2. Use a tool like GIMP or OpenCV to find HSV values of the line
3. Set min/max with some margin for lighting variation
4. Test under different lighting conditions

#### Minimum Contour Area

```yaml
min_contour_area: 500   # Pixels
```

Increase if detecting false positives from noise.
Decrease if missing valid line detections.

### Object Detection

#### Heuristic Detector

For detecting colored objects (e.g., yellow LEGO person):

```yaml
heuristic_h_min: 15     # Yellow hue range
heuristic_h_max: 35
heuristic_s_min: 100    # High saturation (vibrant color)
heuristic_s_max: 255
heuristic_v_min: 100    # Medium to high brightness
heuristic_v_max: 255
heuristic_min_area: 200
heuristic_max_area: 50000
```

## Control Tuning

Configuration file: `ros2_ws/src/robot_bringup/config/control.yaml`

### PD Gains

```yaml
# Lateral error (side-to-side)
kp_lateral: 2.0    # Proportional gain
kd_lateral: 0.1    # Derivative gain

# Heading error (angle)
kp_heading: 1.5
kd_heading: 0.05
```

**Tuning Process:**

1. Start with P gains only (set D gains to 0)
2. Increase `kp_lateral` until the robot tracks the line center
3. If oscillating, add `kd_lateral` to dampen
4. Repeat for heading gains
5. Test on curves - may need different gains for sharp turns

**Signs of Mis-tuning:**
- **Oscillation**: Gains too high, add derivative or reduce proportional
- **Slow response**: Gains too low, increase proportional
- **Overshoots corners**: Heading gains need adjustment

### Speed Profile

```yaml
v_max: 0.5              # Maximum speed (m/s)
v_min: 0.1              # Minimum speed (m/s)
a_max: 0.5              # Max acceleration (m/s²)

# Slowdown gains
curvature_slowdown_gain: 0.3
error_slowdown_gain: 0.5
heading_slowdown_gain: 0.2
```

**Tuning Tips:**
- Lower `v_max` for tighter tracks or testing
- Increase slowdown gains if robot overshoots curves
- Decrease slowdown gains for faster overall speed

## FSM Tuning

Configuration file: `ros2_ws/src/robot_bringup/config/fsm.yaml`

### Detection Stability

```yaml
detection_stability_frames: 5    # Frames before transition
```

Higher values = more stable but slower response.

### Approach Thresholds

```yaml
target_center_tolerance_x: 0.15  # How centered (0 = dead center)
target_center_tolerance_y: 0.2
target_size_close_threshold: 0.25  # How big = close enough
```

**Tuning Process:**
1. Place object at desired pickup distance
2. Note the bounding box size in `/vision/detections`
3. Set `target_size_close_threshold` to that value

### Timing

```yaml
pickup_close_time_s: 1.0   # Time to close claw
pickup_hold_time_s: 0.5    # Time to hold before moving
drop_open_time_s: 1.0      # Time claw stays open at drop
return_time_s: 10.0        # Time-based return (if used)
```

## Hardware Tuning

Configuration file: `ros2_ws/src/robot_bringup/config/hw.yaml`

### Motor Calibration

If motors spin at different speeds for same PWM:

```yaml
left_motor_gain: 1.0     # Multiply left motor command
right_motor_gain: 1.05   # Slightly faster right motor
```

### Direction Reversal

If a motor spins the wrong direction:

```yaml
left_motor_reversed: false
right_motor_reversed: true
```

## Testing Workflow

1. **Static Test**: Place robot on blocks, test motor directions
2. **Straight Line**: Tune line detection and basic tracking
3. **Curves**: Tune gains for smooth curve following
4. **Speed Test**: Increase speed, retune if needed
5. **Full Mission**: Test complete pickup/drop sequence

## Diagnostic Tools

```bash
# View debug image
ros2 run rqt_image_view rqt_image_view /vision/debug_image

# Monitor line observation
ros2 topic echo /vision/line_observation

# Monitor detections
ros2 topic echo /vision/detections

# Check FSM state
ros2 topic echo /control/fsm_state

# Plot topics
ros2 run rqt_plot rqt_plot
```
