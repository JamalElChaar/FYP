#!/usr/bin/env python3
"""Plan to a pose and command the final joint angles one at a time, retrying
until the plan lands on the intended arm configuration.

Pose 2 was originally the pose from final_angles_sequential.launch.py mirrored
180 degrees about the base Z axis. That target proved unreachable once real
joint limits were applied: MoveIt needed joint_1 at servo 259 deg, well past
its 0-180 range, so planning correctly refused it.

The pose below is derived the other way round, by forward kinematics from a
joint configuration that is known to be reachable (ROS degrees
0, 65, -84, 4, -112, 0 -- servo 90, 110, 31, 4, 92, 90). A pose obtained this
way always has at least one IK solution: the configuration it came from.

A pose usually has several IK solutions though, and OMPL is randomised, so a
plan can reach the same pose through a quite different arm configuration. The
node therefore replans up to max_plan_attempts times until every joint lands
within angle_tolerance_deg of desired_servo_degrees.
"""

import datetime
import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _int_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _resolve_log_dir():
    """Find src/custom_hardware/logs, from either the source tree or a
    symlink-installed share directory (colcon build --symlink-install)."""
    launch_file_path = os.path.realpath(__file__)
    # .../launch -> .../control_arm -> .../src
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(launch_file_path)))
    custom_hardware_dir = os.path.join(src_dir, "custom_hardware")
    if os.path.isdir(custom_hardware_dir):
        return os.path.join(custom_hardware_dir, "logs")
    # Fallback if this ever runs from a non-symlink install: keep logs
    # next to the launch file rather than failing.
    return os.path.join(os.path.dirname(launch_file_path), "logs")


def _start_file_logging():
    """Mirror everything printed to the terminal into a text file under
    src/custom_hardware/logs, one file per launch run.

    launch keeps its own loggers/handlers and does not propagate records up to
    Python's root logger, so a logging.FileHandler on root never sees
    anything. Every line that reaches the screen is ultimately written to this
    process's real stdout/stderr file descriptor, including from processes
    started later, so this redirects fd 1/2 through `tee`.
    """
    log_dir = _resolve_log_dir()
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = os.path.join(
        log_dir, f"final_angles_sequential_pose2_{timestamp}.txt")

    sys.stdout.flush()
    sys.stderr.flush()
    tee = subprocess.Popen(["tee", "-a", log_file_path], stdin=subprocess.PIPE)
    os.dup2(tee.stdin.fileno(), 1)
    os.dup2(tee.stdin.fileno(), 2)

    print(f"[final_angles_sequential_pose2] Writing this run's full output to {log_file_path}")
    return log_file_path


def _parse_angles(text):
    values = [float(v) for v in text.replace(" ", "").split(",") if v != ""]
    if values and len(values) != 6:
        raise RuntimeError(
            f"desired_servo_degrees needs 6 comma-separated values, got {len(values)}")
    return values


def launch_setup(context, *args, **kwargs):
    desired = _parse_angles(
        LaunchConfiguration("desired_servo_degrees").perform(context))

    parameters = {
        "mode": "sequential_final",
        "joint_delay_seconds": _float_parameter("joint_delay_seconds"),
        "x": _float_parameter("x"),
        "y": _float_parameter("y"),
        "z": _float_parameter("z"),
        "ox": _float_parameter("ox"),
        "oy": _float_parameter("oy"),
        "oz": _float_parameter("oz"),
        "ow": _float_parameter("ow"),
        "freeze_joint_1": _bool_parameter("freeze_joint_1"),
        "angle_tolerance_deg": _float_parameter("angle_tolerance_deg"),
        "max_plan_attempts": _int_parameter("max_plan_attempts"),
        "use_joint_target": _bool_parameter("use_joint_target"),
    }
    # Only set desired_servo_degrees when angles were actually given: launch
    # cannot infer a type for an empty array parameter and rejects it. Leaving
    # it out falls back to the node's own empty default, which means "accept
    # whatever IK solution MoveIt finds".
    if desired:
        parameters["desired_servo_degrees"] = desired

    direct_command_node = Node(
        package="control_arm",
        executable="direct_esp32_moveit_node",
        name="final_angles_sequential_pose2",
        output="screen",
        parameters=[parameters],
    )

    return [
        TimerAction(
            period=LaunchConfiguration("startup_delay_seconds"),
            actions=[direct_command_node],
        )
    ]


def generate_launch_description():
    _start_file_logging()

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

    return LaunchDescription([
        DeclareLaunchArgument("joint_delay_seconds", default_value="1.0"),
        DeclareLaunchArgument("startup_delay_seconds", default_value="6.0"),
        # Forward kinematics of ROS degrees (0, 65, -84, 4, -112, 0), i.e.
        # servo (90, 110, 31, 4, 92, 90). Reachable by construction.
        DeclareLaunchArgument("x", default_value="-0.0883"),
        DeclareLaunchArgument("y", default_value="-0.1345"),
        DeclareLaunchArgument("z", default_value="0.1034"),
        DeclareLaunchArgument("ox", default_value="-0.6101"),
        DeclareLaunchArgument("oy", default_value="0.4544"),
        DeclareLaunchArgument("oz", default_value="-0.0190"),
        DeclareLaunchArgument("ow", default_value="0.6488"),
        # The configuration the plan is expected to land on, in servo degrees.
        # Set to "" to accept whatever IK solution MoveIt finds first.
        DeclareLaunchArgument(
            "desired_servo_degrees", default_value="90,110,31,4,92,90",
            description="Six comma-separated servo angles the plan should land on"),
        DeclareLaunchArgument(
            "angle_tolerance_deg", default_value="15.0",
            description="Per-joint tolerance against desired_servo_degrees"),
        DeclareLaunchArgument(
            "max_plan_attempts", default_value="10",
            description="How many times to replan before giving up"),
        # Plan to desired_servo_degrees as a joint-space goal instead of
        # running IK on the Cartesian pose. Deterministic -- the final
        # configuration is exactly the one requested, and because the pose was
        # derived from these angles by FK, the end-effector lands in the same
        # place either way. Retrying becomes unnecessary with this set.
        DeclareLaunchArgument(
            "use_joint_target", default_value="false",
            description="Command the joint configuration directly, bypassing IK"),
        # If true, joint_1 is locked to its current value for the whole plan.
        DeclareLaunchArgument("freeze_joint_1", default_value="false"),
        moveit_hardware,
        OpaqueFunction(function=launch_setup),
    ])
