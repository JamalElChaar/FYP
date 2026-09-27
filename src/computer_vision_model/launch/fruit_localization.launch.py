#!/usr/bin/env python3
"""Detect a fruit, locate it in base_link, and publish an arm target pose.

This starts the Astra Pro driver, the RGB-D capture node, the Roboflow
detection node, and the localization node that converts a bounding box into a
base_link pose.

The camera extrinsic is NOT known automatically. Measure where the camera sits
relative to the robot base and pass it in, for example:

    ros2 launch computer_vision_model fruit_localization.launch.py \
      cam_x:=0.30 cam_y:=0.00 cam_z:=0.45 \
      cam_roll:=0.0 cam_pitch:=0.60 cam_yaw:=3.14159

Set publish_camera_tf:=false once the camera is modelled in the robot URDF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    detection_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("computer_vision_model"),
                "launch",
                "fruit_detection.launch.py",
            ])
        ),
        launch_arguments={
            "output_directory": LaunchConfiguration("output_directory"),
            "run_on_start": LaunchConfiguration("run_on_start"),
        }.items(),
    )

    # Measured base_link -> camera transform. Replace the defaults with real
    # measurements; they are placeholders, not a calibration.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_static_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        arguments=[
            "--x", LaunchConfiguration("cam_x"),
            "--y", LaunchConfiguration("cam_y"),
            "--z", LaunchConfiguration("cam_z"),
            "--roll", LaunchConfiguration("cam_roll"),
            "--pitch", LaunchConfiguration("cam_pitch"),
            "--yaw", LaunchConfiguration("cam_yaw"),
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
        ],
    )

    localization_node = Node(
        package="computer_vision_model",
        executable="fruit_localization_node",
        name="fruit_localization_node",
        output="screen",
        parameters=[{
            "base_frame": LaunchConfiguration("base_frame"),
            "target_label": LaunchConfiguration("target_label"),
            "minimum_confidence": _float_parameter("minimum_confidence"),
            "standoff_m": _float_parameter("standoff_m"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_directory", default_value="/tmp/computer_vision_captures"),
        DeclareLaunchArgument(
            "run_on_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "camera_frame", default_value="camera_link",
            description="Root camera frame published by the Astra driver"),
        DeclareLaunchArgument(
            "publish_camera_tf", default_value="true", choices=["true", "false"],
            description="Set false when the camera is modelled in the URDF"),
        DeclareLaunchArgument("cam_x", default_value="0.0"),
        DeclareLaunchArgument("cam_y", default_value="0.0"),
        DeclareLaunchArgument("cam_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("target_label", default_value=""),
        DeclareLaunchArgument("minimum_confidence", default_value="0.40"),
        DeclareLaunchArgument(
            "standoff_m", default_value="0.03",
            description="Metres above the fruit for the commanded target"),
        detection_stack,
        camera_tf,
        localization_node,
    ])
