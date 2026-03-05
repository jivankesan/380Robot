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
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || echo "Warning: /dev/ttyUSB0 not found"
sudo chmod 666 /dev/video0  2>/dev/null || echo "Warning: /dev/video0 not found"

# Start or restart the container
echo -e "${YELLOW}Starting Docker container...${NC}"
cd "$REPO_DIR/compose"
docker rm -f 380robot-dev 2>/dev/null || true
docker compose up -d

# Build the ROS workspace inside the container
echo -e "${YELLOW}Building ROS workspace...${NC}"
docker exec 380robot-dev bash -c "
  cd /workspaces/380Robot/ros2_ws && \
  colcon build --symlink-install 2>&1 | tail -5
"

# Launch the robot
echo -e "${GREEN}Launching robot...${NC}"
docker exec -it 380robot-dev bash -c "
  source /workspaces/380Robot/ros2_ws/install/setup.bash && \
  ros2 launch robot_bringup bringup_real.launch.py
"
