#!/usr/bin/env python3
"""Forward kinematics helper.

Given a joint configuration, print the end-effector pose it produces. Loads
only the robot model -- no RViz, no controllers, no ESP32 -- so it runs in a
second and needs no hardware.

Angles are given in servo degrees by default (the same 0-180 convention as
/esp32/joint_commands), comma separated:

  ros2 launch control_arm fk_pose.launch.py servo_degrees:="90,45,90,90,90,90"

To give ROS joint degrees instead (the numbers RViz's Joints tab shows):

  ros2 launch control_arm fk_pose.launch.py ros_degrees:="0,45,-25,90,-110,0"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def _parse_angles(text):
    values = [float(v) for v in text.replace(" ", "").split(",") if v != ""]
    if len(values) != 6:
        raise RuntimeError(
            f"expected 6 comma-separated angles, got {len(values)}: {text!r}")
    return values


def launch_setup(context, *args, **kwargs):
    servo_text = LaunchConfiguration("servo_degrees").perform(context)
    ros_text = LaunchConfiguration("ros_degrees").perform(context)

    # ros_degrees, when given, overrides servo_degrees.
    if ros_text.strip():
        angles = _parse_angles(ros_text)
        input_is_ros_degrees = True
    else:
        angles = _parse_angles(servo_text)
        input_is_ros_degrees = False

    description_pkg = get_package_share_directory("robot_arm_description")
    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="robot_arm_movit_config")
        .robot_description(
            file_path=os.path.join(
                description_pkg, "urdf", "robot", "arm.urdf.xacro"
            ),
            mappings={
                "use_gazebo": "false",
                "use_camera": "false",
                "use_simulation": "true",
            },
        )
        .to_moveit_configs()
    )

    return [
        Node(
            package="control_arm",
            executable="fk_pose_node",
            name="fk_pose",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                {
                    "servo_degrees": angles,
                    "input_is_ros_degrees": input_is_ros_degrees,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "servo_degrees", default_value="90,90,90,90,90,90",
            description="Six comma-separated servo angles in degrees (0-180)"),
        DeclareLaunchArgument(
            "ros_degrees", default_value="",
            description="Six comma-separated ROS joint angles in degrees; "
                        "overrides servo_degrees when set"),
        OpaqueFunction(function=launch_setup),
    ])
