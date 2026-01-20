#!/usr/bin/env python3
"""Launch file for teleop mode - manual control via keyboard or joystick."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('robot_bringup')
    description_dir = get_package_share_directory('robot_description')

    # Config file paths
    camera_config = os.path.join(bringup_dir, 'config', 'camera.yaml')
    hw_config = os.path.join(bringup_dir, 'config', 'hw.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot state publisher (URDF/TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Camera node (optional for teleop, but useful for monitoring)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            parameters=[camera_config],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),

        # Hardware: Serial bridge
        Node(
            package='robot_hw_cpp',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[hw_config],
        ),

        # Teleop keyboard node
        # Note: This requires a terminal for keyboard input
        # Run with: ros2 run teleop_twist_keyboard teleop_twist_keyboard
        # Or use the teleop node from robot_tools
        Node(
            package='robot_vision_py',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'linear_speed': 0.3,
                'angular_speed': 1.0,
                'linear_step': 0.05,
                'angular_step': 0.1,
            }],
            prefix='xterm -e',  # Opens in new terminal on Linux with X11
        ),
    ])
