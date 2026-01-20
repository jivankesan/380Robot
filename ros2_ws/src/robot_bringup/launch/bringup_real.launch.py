#!/usr/bin/env python3
"""Launch file for real robot operation."""

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
    vision_config = os.path.join(bringup_dir, 'config', 'vision.yaml')
    control_config = os.path.join(bringup_dir, 'config', 'control.yaml')
    fsm_config = os.path.join(bringup_dir, 'config', 'fsm.yaml')
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

        # Camera node (v4l2_camera)
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

        # Vision: Line detector
        Node(
            package='robot_vision_py',
            executable='line_detector_node',
            name='line_detector_node',
            parameters=[vision_config],
        ),

        # Vision: Object detector
        Node(
            package='robot_vision_py',
            executable='object_detector_node',
            name='object_detector_node',
            parameters=[vision_config],
        ),

        # Control: Line follow controller
        Node(
            package='robot_control_cpp',
            executable='line_follow_controller_node',
            name='line_follow_controller_node',
            parameters=[control_config],
        ),

        # Control: Speed profile
        Node(
            package='robot_control_cpp',
            executable='speed_profile_node',
            name='speed_profile_node',
            parameters=[control_config],
        ),

        # Control: Safety stop
        Node(
            package='robot_control_cpp',
            executable='safety_stop_node',
            name='safety_stop_node',
            parameters=[control_config],
        ),

        # FSM: Task state machine
        Node(
            package='robot_fsm_cpp',
            executable='task_fsm_node',
            name='task_fsm_node',
            parameters=[fsm_config],
        ),

        # Hardware: Serial bridge
        Node(
            package='robot_hw_cpp',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[hw_config],
        ),
    ])
