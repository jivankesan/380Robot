#!/bin/bash
# Run the assessment demo inside the Docker container.
#
# Usage:
#   Terminal 1 (line following):  ./scripts/start_robot.sh
#   Terminal 2 (mobility demo):   ./scripts/run_demo.sh
#
# The demo does:
#   - Forward movement (autonomous)
#   - Backward movement (autonomous)
#   - Claw open / close
#
# Line following + 90° turn is demonstrated by start_robot.sh (bringup_real).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}380Robot Demo${NC}"
echo "============="

# Fix device permissions
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || echo "Warning: /dev/ttyUSB0 not found"

# Restart container so devices are freshly mounted
echo -e "${YELLOW}Starting Docker container...${NC}"
cd "$REPO_DIR/compose"
docker compose down 2>/dev/null || true
docker compose up -d
sleep 3

echo -e "${YELLOW}Building ROS workspace...${NC}"
docker exec 380robot-dev bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /workspaces/380Robot/ros2_ws && \
    sudo colcon build --symlink-install
"

echo -e "${YELLOW}Running autonomous demo (forward / backward / claw)...${NC}"
docker exec -it 380robot-dev bash -c "
    python3 /workspaces/380Robot/scripts/demo.py
"

echo -e "${GREEN}Demo finished. Bringup (line following) is still running in background.${NC}"
echo -e "To stop everything: docker stop 380robot-dev"
