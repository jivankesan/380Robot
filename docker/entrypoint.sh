#!/bin/bash
set -e

# Source ROS 2 installation
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace overlay if it exists
if [ -f /workspaces/380Robot/ros2_ws/install/setup.bash ]; then
    source /workspaces/380Robot/ros2_ws/install/setup.bash
fi

# Set RMW implementation
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

# Execute the command passed to the container
exec "$@"
