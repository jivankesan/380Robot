#!/bin/bash
# Bootstrap script for 380Robot development environment
# This script sets up the development environment and enters the container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

cd "$REPO_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}380Robot Development Environment Bootstrap${NC}"
echo "============================================"

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed${NC}"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check for docker compose
if ! docker compose version &> /dev/null; then
    echo -e "${RED}Error: docker compose is not available${NC}"
    echo "Please install Docker Compose V2"
    exit 1
fi

# Create .env file if it doesn't exist
if [ ! -f .env ]; then
    echo -e "${YELLOW}Creating .env file from template...${NC}"
    cp .env.example .env

    # Update UID/GID for Linux
    if [[ "$(uname)" == "Linux" ]]; then
        sed -i "s/HOST_UID=1000/HOST_UID=$(id -u)/" .env
        sed -i "s/HOST_GID=1000/HOST_GID=$(id -g)/" .env
    fi
    echo -e "${GREEN}Created .env file. Edit it if needed.${NC}"
fi

# Build the container if needed
echo -e "${YELLOW}Building Docker image...${NC}"
docker compose build

# Start the container
echo -e "${YELLOW}Starting container...${NC}"
docker compose up -d

# Enter the container
echo -e "${GREEN}Entering development container...${NC}"
echo ""
echo "Inside the container, run:"
echo "  rosdep update"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
docker compose exec robot-dev bash
