# Troubleshooting Guide

Common issues and solutions for the 380Robot.

## Docker Issues

### Container Won't Build

**Symptom**: `docker compose build` fails

**Solutions**:
1. Check internet connection (needs to download packages)
2. Clear Docker cache: `docker builder prune`
3. Check available disk space
4. Try pulling base image manually: `docker pull ubuntu:24.04`

### Can't Access Camera in Container

**Symptom**: Camera node fails to open `/dev/video0`

**Solutions (Linux)**:
```bash
# Uncomment device passthrough in docker-compose.yml:
devices:
  - /dev/video0:/dev/video0

# Add user to video group
sudo usermod -aG video $USER
# Log out and back in

# Check camera is detected
ls -la /dev/video*
```

**Solutions (macOS/Windows)**:
- Camera passthrough is not natively supported
- Use a network camera stream, or
- Run vision nodes on host, other nodes in container

### Can't Access Serial Port in Container

**Symptom**: Serial bridge can't open `/dev/ttyACM0`

**Solutions (Linux)**:
```bash
# Uncomment in docker-compose.yml:
devices:
  - /dev/ttyACM0:/dev/ttyACM0

# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and back in

# Check permissions
ls -la /dev/ttyACM0
```

**Solutions (macOS/Windows)**:
- Serial passthrough requires extra configuration
- Consider running serial bridge on host

## Camera Issues

### No Image Published

**Symptom**: `/camera/image_raw` has no messages

**Check**:
```bash
# List video devices
v4l2-ctl --list-devices

# Test camera
v4l2-ctl -d /dev/video0 --all

# Check node is running
ros2 node list

# Check for errors
ros2 topic echo /rosout
```

**Solutions**:
- Verify correct device in `camera.yaml`
- Try different pixel format (YUYV, MJPG)
- Reduce resolution/framerate

### Image Quality Poor

**Symptom**: Image is dark, blurry, or noisy

**Solutions**:
- Adjust camera exposure: `v4l2-ctl -d /dev/video0 -c exposure_auto=1`
- Improve lighting conditions
- Clean camera lens
- Try different resolution

## Vision Issues

### Line Not Detected

**Symptom**: `LineObservation.valid = false`

**Debug**:
```bash
# View debug image
ros2 run rqt_image_view rqt_image_view /vision/debug_image
```

**Solutions**:
1. Check ROI is correct (line should be visible)
2. Adjust HSV thresholds for line color
3. Increase/decrease `min_contour_area`
4. Check lighting conditions

### False Line Detections

**Symptom**: Robot tracks wrong features

**Solutions**:
- Narrow ROI to exclude distractions
- Tighten HSV thresholds
- Increase `min_contour_area`
- Add physical masking (tape around edges)

### Object Not Detected

**Symptom**: No detections when object is visible

**Solutions**:
- Check HSV thresholds match object color
- Adjust `heuristic_min_area` / `heuristic_max_area`
- Lower `confidence_threshold`
- Ensure good lighting on object

## Control Issues

### Robot Oscillates

**Symptom**: Robot weaves side to side

**Solutions**:
1. Reduce `kp_lateral` gain
2. Increase `kd_lateral` gain
3. Reduce maximum speed
4. Check for sensor noise/latency

### Robot Doesn't Follow Line

**Symptom**: Robot drives straight ignoring line

**Check**:
```bash
ros2 topic echo /vision/line_observation
ros2 topic echo /control/cmd_vel
```

**Solutions**:
- Verify line detection is working
- Check controller is receiving observations
- Increase PD gains
- Check FSM state (must be in FOLLOW mode)

### Robot Moves Erratically

**Symptom**: Jerky or unpredictable motion

**Solutions**:
- Check for dropped messages (network issues)
- Verify motor wiring (loose connections)
- Check encoder feedback
- Reduce acceleration limits

## Hardware Issues

### Motors Don't Spin

**Symptom**: Commands sent but no motor movement

**Check**:
```bash
# Check serial communication
ros2 topic echo /hw/status

# Check for Arduino errors
# (Look at serial monitor in Arduino IDE)
```

**Solutions**:
- Verify Arduino is programmed and running
- Check motor driver power supply
- Verify wiring matches pin definitions
- Test motors directly with Arduino serial monitor

### Motors Spin Wrong Direction

**Solution**:
```yaml
# In hw.yaml:
left_motor_reversed: true   # Toggle as needed
right_motor_reversed: false
```

Or swap motor wires on driver.

### Claw Doesn't Move

**Check**:
- Servo power supply (separate from motor power)
- Servo signal wire connection
- Servo position limits in Arduino code

### Arduino Not Responding

**Symptoms**: No telemetry, commands ignored

**Solutions**:
1. Check USB cable and connection
2. Verify correct serial port in `hw.yaml`
3. Reset Arduino
4. Re-upload firmware
5. Check for serial buffer overflow

## FSM Issues

### Robot Stuck in INIT State

**Symptom**: Never transitions to FOLLOW_LINE

**Check**:
```bash
ros2 topic echo /control/fsm_state
ros2 topic echo /vision/line_observation
ros2 topic echo /hw/status
```

**Solutions**:
- Wait for line detection to become valid
- Check hardware status is being published
- Manually trigger state (if testing)

### Robot Never Picks Up Object

**Symptom**: Approaches but doesn't transition to PICKUP

**Check**:
```bash
ros2 topic echo /vision/detections
```

**Solutions**:
- Adjust `target_size_close_threshold`
- Adjust center tolerances
- Verify detection is stable (check `detection_stability_frames`)

## Network/ROS Issues

### Nodes Can't Communicate

**Symptom**: Topics published but not received

**Solutions**:
```bash
# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID

# Check RMW implementation
echo $RMW_IMPLEMENTATION

# List all nodes
ros2 node list

# Check topic connections
ros2 topic info /camera/image_raw -v
```

### High Latency

**Symptom**: Delayed response to commands

**Solutions**:
- Use BEST_EFFORT QoS for camera
- Reduce image resolution
- Check network congestion
- Profile node performance

## General Debugging Commands

```bash
# System overview
ros2 node list
ros2 topic list
ros2 topic hz /camera/image_raw

# Detailed topic info
ros2 topic info /vision/line_observation -v
ros2 topic echo /vision/line_observation

# Node info
ros2 node info /line_detector_node

# Parameter inspection
ros2 param list /line_detector_node
ros2 param get /line_detector_node kp_lateral

# TF tree
ros2 run tf2_tools view_frames

# Record data for analysis
ros2 bag record -a
```
