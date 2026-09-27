#!/usr/bin/env python3
"""Pure-simulation joint-target test.

Starts MoveIt with mock hardware (no ESP32, no micro-ROS Agent) and plans
+ executes directly to a joint-space target given in servo degrees
(default: 90 for every joint, i.e. ROS 0 rad with the standard
offset=90/direction=+1 convention -- the arm's calibrated home pose).
Unlike direct_esp32_moveit_node, this executes the trajectory through
MoveIt's own arm_controller, so the simulated robot state actually moves
and RViz shows the result.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            "test_mode": "true",
            "start_agent": "false",
        }.items(),
    )

    joint_target_node = Node(
        package="control_arm",
        executable="joint_target_node",
        name="joint_target_test",
        output="screen",
        parameters=[{
            # 90 deg servo for every joint == 0 rad ROS with the default
            # offset=90/direction=+1 convention. Edit here to try other
            # joint-space targets.
            "servo_degrees": [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("startup_delay_seconds", default_value="6.0"),
        moveit_hardware,
        TimerAction(
            period=LaunchConfiguration("startup_delay_seconds"),
            actions=[joint_target_node],
        ),
    ])
