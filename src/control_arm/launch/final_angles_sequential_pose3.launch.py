#!/usr/bin/env python3
"""Pose 3: reach an object sitting at base level (z = 0).

Same x, y and orientation as final_angles_sequential_pose3.launch.py, with
z dropped from 0.1034 to 0.02.

Unlike pose 2, this target is NOT derived from a known joint configuration, so
there is no guarantee IK can reach it -- if it cannot, MoveIt reports a
planning failure rather than producing something the servos cannot execute.
desired_servo_degrees is therefore left empty: any solution MoveIt finds is
accepted, and the joint limits in the URDF already guarantee every joint of
that solution is inside the servo range.
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
        log_dir, f"final_angles_sequential_pose3_{timestamp}.txt")

    sys.stdout.flush()
    sys.stderr.flush()
    tee = subprocess.Popen(["tee", "-a", log_file_path], stdin=subprocess.PIPE)
    os.dup2(tee.stdin.fileno(), 1)
    os.dup2(tee.stdin.fileno(), 2)

    print(f"[final_angles_sequential_pose3] Writing this run's full output to {log_file_path}")
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
        "position_only": _bool_parameter("position_only"),
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
        name="final_angles_sequential_pose3",
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
        # Object position relative to base_link. z = 0 is base level.
        # Best reachability is at a radius of 0.22-0.28 m from the base
        # axis; sqrt(x^2 + y^2) here is 0.161 m, which is comfortable.
        DeclareLaunchArgument("x", default_value="-0.0883"),
        DeclareLaunchArgument("y", default_value="-0.1345"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("ox", default_value="-0.6101"),
        DeclareLaunchArgument("oy", default_value="0.4544"),
        DeclareLaunchArgument("oz", default_value="-0.0190"),
        DeclareLaunchArgument("ow", default_value="0.6488"),
        # Empty: accept whatever IK solution MoveIt finds. The URDF joint
        # limits already constrain it to the servo range. Set six comma-
        # separated servo angles here to hold out for a specific configuration.
        DeclareLaunchArgument(
            "desired_servo_degrees", default_value="",
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
        # Reach the position, let the planner pick the wrist orientation.
        DeclareLaunchArgument(
            "position_only", default_value="true",
            description="Ignore the target orientation and match position only"),
        DeclareLaunchArgument(
            "use_joint_target", default_value="false",
            description="Command the joint configuration directly, bypassing IK"),
        # If true, joint_1 is locked to its current value for the whole plan.
        DeclareLaunchArgument("freeze_joint_1", default_value="false"),
        moveit_hardware,
        OpaqueFunction(function=launch_setup),
    ])
