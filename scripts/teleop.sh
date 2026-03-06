#!/bin/bash
# Keyboard teleop for 380Robot - run this over SSH from your Mac.
# Arrow keys (and w/a/s/d) control the robot.
#
# Usage:  ./scripts/teleop.sh
#
# What it does:
#   1. Starts the serial bridge (Arduino comms) in the background inside Docker
#   2. Runs the teleop node in THIS terminal so your keyboard works
#   3. Kills the serial bridge and stops the motors when you quit (q or Ctrl-C)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

CONTAINER="380robot-dev"
ROS_SETUP="source /opt/ros/jazzy/setup.bash && source /workspaces/380Robot/ros2_ws/install/setup.bash"
HW_PARAMS="/workspaces/380Robot/ros2_ws/install/robot_bringup/share/robot_bringup/config/hw.yaml"

# --- Check container is running ---
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo -e "${RED}Container '${CONTAINER}' is not running.${NC}"
    echo "Start it first with:  ./scripts/start_robot.sh"
    exit 1
fi

# --- Fix serial port permissions ---
echo -e "${YELLOW}Setting serial port permissions...${NC}"
docker exec "$CONTAINER" bash -c "sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true"

# --- Start serial bridge in background ---
echo -e "${YELLOW}Starting serial bridge...${NC}"
docker exec -d "$CONTAINER" bash -c "
    $ROS_SETUP && \
    ros2 run robot_hw_cpp serial_bridge_node \
        --ros-args --params-file $HW_PARAMS
"

# Give the serial bridge a moment to open the port
sleep 2

# --- On exit: kill serial bridge and stop motors ---
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping...${NC}"
    docker exec "$CONTAINER" bash -c "pkill -f serial_bridge_node 2>/dev/null || true"
}
trap cleanup EXIT

# --- Run teleop in THIS terminal (stdin = your keyboard) ---
echo -e "${GREEN}Teleop ready. Use arrow keys or w/a/s/d. Press q to quit.${NC}"
echo ""
docker exec -it "$CONTAINER" bash -c "
    $ROS_SETUP && \
    ros2 run robot_vision_py teleop_node
"
