#!/usr/bin/env python3
"""Stream the OMPL path and Pilz descent directly to the ESP32 at 5 Hz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    moveit_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("control_arm"),
                "launch",
                "moveit_hardware.launch.py",
            ])
        ),
        launch_arguments={
            "test_mode": "false",
            "start_agent": "false",
        }.items(),
    )

    direct_command_node = Node(
        package="control_arm",
        executable="direct_esp32_moveit_node",
        name="slow_moveit_trajectory",
        output="screen",
        parameters=[{
            "mode": "slow_full_path",
            "command_interval_seconds": _float_parameter(
                "command_interval_seconds"
            ),
            "pre_final_offset": _float_parameter("pre_final_offset"),
            "x": _float_parameter("x"),
            "y": _float_parameter("y"),
            "z": _float_parameter("z"),
            "ox": _float_parameter("ox"),
            "oy": _float_parameter("oy"),
            "oz": _float_parameter("oz"),
            "ow": _float_parameter("ow"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("command_interval_seconds", default_value="0.2"),
        DeclareLaunchArgument("pre_final_offset", default_value="0.03"),
        DeclareLaunchArgument("startup_delay_seconds", default_value="6.0"),
        DeclareLaunchArgument("x", default_value="0.166"),
        DeclareLaunchArgument("y", default_value="0.132"),
        DeclareLaunchArgument("z", default_value="0.0508"),
        DeclareLaunchArgument("ox", default_value="-0.548"),
        DeclareLaunchArgument("oy", default_value="0.447"),
        DeclareLaunchArgument("oz", default_value="0.006"),
        DeclareLaunchArgument("ow", default_value="0.707"),
        moveit_hardware,
        TimerAction(
            period=LaunchConfiguration("startup_delay_seconds"),
            actions=[direct_command_node],
        ),
    ])
