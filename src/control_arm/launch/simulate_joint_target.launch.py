#!/usr/bin/env python3
"""Pure-simulation joint-target test.

Starts MoveIt with mock hardware (no ESP32, no micro-ROS Agent, no control
path to the servos) and plans + executes to a joint configuration, so the
simulated arm actually moves and RViz shows it.

Angles may be given either way:

  # ROS joint degrees -- this is what the vision pipeline reports as
  # joint_angles_degrees, and what RViz's Joints tab displays
  ros2 launch control_arm simulate_joint_target.launch.py \\
      ros_degrees:="0,65,-84,4,-112,0"

  # servo degrees (0-180), the /esp32/joint_commands convention
  ros2 launch control_arm simulate_joint_target.launch.py \\
      servo_degrees:="90,110,31,4,92,90"

With neither given it goes to servo 90 on every joint, the ESP32 firmware's
boot pose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Must match custom_hardware.cpp and direct_esp32_moveit_node:
#   servo_deg = offset + direction * ros_deg
SERVO_OFFSETS = [90.0, 103.6364, 111.3713, 0.0, 90.0, 90.0]
SERVO_DIRECTIONS = [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]


def _parse(text):
    values = [float(v) for v in text.replace(" ", "").split(",") if v != ""]
    if values and len(values) != 6:
        raise RuntimeError(f"expected 6 comma-separated angles, got {len(values)}")
    return values


def launch_setup(context, *args, **kwargs):
    ros_text = LaunchConfiguration("ros_degrees").perform(context)
    servo_text = LaunchConfiguration("servo_degrees").perform(context)

    ros_values = _parse(ros_text)
    if ros_values:
        # Convert the pipeline's ROS joint degrees into the servo convention
        # the node expects.
        servo = [SERVO_OFFSETS[i] + SERVO_DIRECTIONS[i] * ros_values[i] for i in range(6)]
    else:
        servo = _parse(servo_text) or [90.0] * 6

    return [
        TimerAction(
            period=LaunchConfiguration("startup_delay_seconds"),
            actions=[Node(
                package="control_arm",
                executable="joint_target_node",
                name="joint_target_test",
                output="screen",
                parameters=[{"servo_degrees": servo}],
            )],
        )
    ]


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

    return LaunchDescription([
        DeclareLaunchArgument("startup_delay_seconds", default_value="6.0"),
        DeclareLaunchArgument(
            "ros_degrees", default_value="",
            description="Six comma-separated ROS joint angles in degrees "
                        "(the vision pipeline's joint_angles_degrees)"),
        DeclareLaunchArgument(
            "servo_degrees", default_value="",
            description="Six comma-separated servo angles in degrees (0-180); "
                        "ignored when ros_degrees is given"),
        moveit_hardware,
        OpaqueFunction(function=launch_setup),
    ])
