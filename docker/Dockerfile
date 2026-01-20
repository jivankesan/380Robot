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

# Install Python dependencies for vision
RUN pip3 install --break-system-packages --no-cache-dir \
    opencv-python-headless \
    numpy \
    ultralytics \
    onnxruntime

# Create non-root user with matching UID/GID
RUN groupadd --gid ${HOST_GID} ros \
    && useradd --uid ${HOST_UID} --gid ${HOST_GID} -m ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros

# Setup user environment
USER ros
WORKDIR /home/ros

# Update rosdep as user
RUN rosdep update

# Create workspace directory
RUN mkdir -p /workspaces/380Robot/ros2_ws/src

# Copy entrypoint
COPY --chown=ros:ros entrypoint.sh /home/ros/entrypoint.sh
RUN chmod +x /home/ros/entrypoint.sh

# Add ROS setup to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspaces/380Robot/ros2_ws/install/setup.bash ]; then source /workspaces/380Robot/ros2_ws/install/setup.bash; fi" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

WORKDIR /workspaces/380Robot/ros2_ws

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
