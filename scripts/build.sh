#!/bin/bash
# Build script for 380Robot ROS 2 workspace
# Run this inside the Docker container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$REPO_DIR/ros2_ws"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

cd "$WS_DIR"

echo -e "${GREEN}Building 380Robot ROS 2 Workspace${NC}"
echo "==================================="

# Source ROS
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    echo "ROS 2 not found. Are you inside the Docker container?"
    exit 1
fi

# Install dependencies
echo -e "${YELLOW}Installing dependencies...${NC}"
rosdep update --rosdistro=${ROS_DISTRO} || true
rosdep install --from-paths src --ignore-src -r -y

# Build
echo -e "${YELLOW}Building workspace...${NC}"
colcon build --symlink-install "$@"

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

echo ""
echo -e "${GREEN}Build complete!${NC}"
echo ""
echo "To use the workspace, run:"
echo "  source $WS_DIR/install/setup.bash"
