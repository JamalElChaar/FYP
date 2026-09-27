#!/usr/bin/env python3
"""Replay an image/calibration pair through detection, plane projection, and IK."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Common launch arguments (image_path, calibration_path, orientation, etc.)
    # remain exposed by the included launch. Only the input source is fixed here.
    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='image', choices=['image']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('computer_vision_model'), 'launch', 'fruit_plane_pipeline.launch.py'])),
            launch_arguments={'input_mode': 'image'}.items())
    ])
