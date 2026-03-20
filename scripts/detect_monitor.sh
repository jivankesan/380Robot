#!/bin/bash
# Run camera + object detector and print /vision/detections live.
# Usage: bash scripts/detect_monitor.sh

SETUP="source /opt/ros/jazzy/setup.bash && source /workspaces/380Robot/ros2_ws/install/setup.bash"
PARAMS="--ros-args --params-file /workspaces/380Robot/ros2_ws/install/robot_bringup/share/robot_bringup/config/vision.yaml"

cleanup() {
  echo "Stopping..."
  kill 0
  wait
}
trap cleanup EXIT INT TERM

docker exec -d 380robot-dev bash -c "$SETUP && ros2 run robot_vision_py mjpeg_camera_node $PARAMS"
docker exec -d 380robot-dev bash -c "$SETUP && ros2 run robot_vision_py object_detector_node $PARAMS"

echo "Waiting for nodes to start..."
sleep 3

echo "--- Detections (Ctrl+C to stop) ---"
docker exec -it 380robot-dev bash -c "$SETUP && ros2 topic echo /vision/detections"
