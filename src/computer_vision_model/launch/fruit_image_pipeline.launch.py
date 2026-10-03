#!/usr/bin/env python3
"""Run the full pipeline on a saved image instead of the live camera.

Everything except the camera driver behaves exactly as a live run: the camera
TF is published from the configured extrinsics, the transform is looked up
from TF, and detection, plane projection and IK are unchanged. The only thing
swapped out is where the image comes from.

    ros2 launch computer_vision_model fruit_image_pipeline.launch.py

Defaults to input_images/banana_scene.jpg with its calibration discovered
alongside it (banana_scene.yaml). Point it at another image with:

    ros2 launch computer_vision_model fruit_image_pipeline.launch.py \\
        image_path:=/path/to/photo.jpg calibration_path:=/path/to/photo.yaml

replay_as_live defaults to true. Setting it false reverts to the older
behaviour, where the camera-to-base transform is read from the calibration
file rather than TF -- useful for reproducing an old result exactly, but it
ignores any change to the configured camera position.

Every argument of fruit_plane_pipeline.launch.py remains available here,
including minimum_confidence, position_only and model_id.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='image', choices=['image']),
        DeclareLaunchArgument(
            'replay_as_live', default_value='true', choices=['true', 'false'],
            description='Publish the camera TF and look the transform up from '
                        'TF, as a live run does, instead of using the one '
                        'recorded in the calibration file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('computer_vision_model'), 'launch',
                'fruit_plane_pipeline.launch.py'])),
            launch_arguments={
                'input_mode': 'image',
                'replay_as_live': LaunchConfiguration('replay_as_live'),
            }.items())
    ])
