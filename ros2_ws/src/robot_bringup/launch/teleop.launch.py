#!/usr/bin/env python3
"""Launch file for manual keyboard teleop (fallback control).

Starts the serial bridge and teleop node only — no vision or autonomy.
Arrow keys and w/a/s/d move the robot; run this in a terminal you can type in.

Usage:
    ros2 launch robot_bringup teleop.launch.py
    ros2 launch robot_bringup teleop.launch.py linear_speed:=0.4
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    hw_config = os.path.join(bringup_dir, 'config', 'hw.yaml')

    linear_speed = LaunchConfiguration('linear_speed', default='0.35')
    angular_speed = LaunchConfiguration('angular_speed', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.35',
            description='Forward/backward speed (m/s)'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='1.0',
            description='Turn speed (rad/s)'
        ),

        # Hardware: serial bridge to Arduino
        Node(
            package='robot_hw_cpp',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[hw_config],
            output='screen',
        ),

        # Keyboard teleop — runs in this terminal (arrow keys + w/a/s/d)
        Node(
            package='robot_vision_py',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'linear_step': 0.05,
                'angular_step': 0.1,
            }],
            output='screen',
        ),
    ])
