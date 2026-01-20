# 380Robot ROS 2 Development Dockerfile
# Supports both Jazzy (Ubuntu 24.04) and Humble (Ubuntu 22.04)

ARG ROS_DISTRO=jazzy
ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION}

ARG ROS_DISTRO
ARG HOST_UID=1000
ARG HOST_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install base dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 and development tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \
    build-essential \
    cmake \
    git \
    clang-format \
    cppcheck \
    wget \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Pin setuptools to a version compatible with ROS 2 colcon build system
# Versions 81+ break editable installs for ament_python packages
RUN pip3 install --break-system-packages --no-cache-dir "setuptools<81"

# Optional: ML dependencies for YOLO detection (heuristic detector works without these)
# Commenting out to avoid dependency conflicts - uncomment if you need YOLO
# RUN pip3 install --break-system-packages --no-cache-dir ultralytics onnxruntime

# Create non-root user with matching UID/GID
# Handle case where GID/UID might already exist in base image
RUN if ! getent group ros > /dev/null 2>&1; then \
        groupadd --gid ${HOST_GID} ros 2>/dev/null || groupadd ros; \
    fi \
    && if ! getent passwd ros > /dev/null 2>&1; then \
        useradd --uid ${HOST_UID} --gid ros -m -s /bin/bash ros 2>/dev/null || useradd --gid ros -m -s /bin/bash ros; \
    fi \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Setup user environment
USER ros
WORKDIR /home/ros

# Update rosdep as user
RUN rosdep update

# Copy entrypoint
COPY --chown=ros:ros entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh

# Add ROS setup to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspaces/380Robot/ros2_ws/install/setup.bash ]; then source /workspaces/380Robot/ros2_ws/install/setup.bash; fi" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Default workdir - will be overridden by docker-compose
WORKDIR /home/ros

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
