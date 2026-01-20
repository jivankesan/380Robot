#!/bin/bash
# Test script for 380Robot
# Runs colcon test and reports results

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$REPO_DIR/ros2_ws"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$WS_DIR"

echo -e "${GREEN}Testing 380Robot ROS 2 Packages${NC}"
echo "================================="

# Source ROS
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    echo "ROS 2 not found. Are you inside the Docker container?"
    exit 1
fi

# Source workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo -e "${YELLOW}Workspace not built. Building first...${NC}"
    colcon build --symlink-install
    source install/setup.bash
fi

# Run tests
echo -e "${YELLOW}Running tests...${NC}"
colcon test "$@"

# Show results
echo ""
echo -e "${YELLOW}Test Results:${NC}"
colcon test-result --verbose

echo ""
echo -e "${GREEN}Test run complete${NC}"
