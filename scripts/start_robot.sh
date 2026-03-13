#!/bin/bash
# Start the robot: brings up Docker and launches ROS2 nodes
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}380Robot Startup${NC}"
echo "================"

# Fix device permissions
echo -e "${YELLOW}Setting device permissions...${NC}"
sudo chmod 666 /dev/ttyUSB0  2>/dev/null || echo "Warning: /dev/ttyUSB0 not found"
sudo chmod 666 /dev/video0   2>/dev/null || echo "Warning: /dev/video0 not found"
sudo chmod 666 /dev/media0   2>/dev/null || echo "Warning: /dev/media0 not found"
sudo chmod 666 /dev/media2   2>/dev/null || echo "Warning: /dev/media2 not found"

# Launch camera_ros natively on the Pi (libcamera is Pi-specific and won't work
# inside the Ubuntu Docker container)
echo -e "${YELLOW}Starting Pi Camera natively...${NC}"
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
# Kill any existing camera_node process
pkill -f "camera_node" 2>/dev/null || true
ros2 run camera_ros camera_node \
  --ros-args \
  -p camera:=0 \
  -p width:=640 \
  -p height:=480 \
  -p fps:=58.92 \
  -r "~/image_raw:=/camera/image_raw" \
  -r "~/camera_info:=/camera/camera_info" &
CAMERA_PID=$!
echo "Camera node PID: $CAMERA_PID"
sleep 2

# Start the container if not already running
echo -e "${YELLOW}Starting Docker container...${NC}"
cd "$REPO_DIR/compose"
if [ "$(docker inspect -f '{{.State.Running}}' 380robot-dev 2>/dev/null)" != "true" ]; then
  docker compose up -d
else
  echo "Container already running."
fi

# Build and launch inside the container (skip camera_node — it runs natively)
echo -e "${YELLOW}Building ROS workspace and launching robot...${NC}"
docker exec -it 380robot-dev bash -c "
  source /opt/ros/jazzy/setup.bash && \
  cd /workspaces/380Robot/ros2_ws && \
  sudo colcon build --symlink-install && \
  source install/setup.bash && \
  ros2 launch robot_bringup bringup_real.launch.py
"

# Clean up camera node when done
kill $CAMERA_PID 2>/dev/null || true
