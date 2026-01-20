#!/usr/bin/env python3
"""Launch file for development mode (no hardware, uses video/image playback)."""

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
    vision_config = os.path.join(bringup_dir, 'config', 'vision.yaml')
    control_config = os.path.join(bringup_dir, 'config', 'control.yaml')
    fsm_config = os.path.join(bringup_dir, 'config', 'fsm.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    video_file = LaunchConfiguration('video_file', default='')
    image_file = LaunchConfiguration('image_file', default='')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'video_file',
            default_value='',
            description='Path to video file for playback (leave empty for live camera)'
        ),
        DeclareLaunchArgument(
            'image_file',
            default_value='',
            description='Path to image file for static testing'
        ),

        # Robot state publisher (URDF/TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Dummy camera/image publisher for development
        Node(
            package='robot_vision_py',
            executable='dummy_camera_node',
            name='dummy_camera_node',
            parameters=[{
                'video_file': video_file,
                'image_file': image_file,
                'frame_rate': 30.0,
                'loop': True,
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
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

        # FSM: Task state machine (optional in dev mode)
        Node(
            package='robot_fsm_cpp',
            executable='task_fsm_node',
            name='task_fsm_node',
            parameters=[fsm_config],
        ),

        # Dummy hardware bridge (prints commands instead of sending to Arduino)
        Node(
            package='robot_hw_cpp',
            executable='dummy_hw_node',
            name='dummy_hw_node',
            parameters=[{
                'print_commands': True,
            }],
        ),
    ])
