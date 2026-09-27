#!/usr/bin/env python3
"""Plan directly to the final pose and command joints one at a time."""

import datetime
import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


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

    launch keeps its own loggers/handlers and does not propagate records
    up to Python's root logger, so a logging.FileHandler on root never
    sees anything (this is why an earlier attempt produced an empty
    file). Every line that reaches the screen -- launch's own messages
    and every child process's re-prefixed output, e.g. "[move_group-2]
    ..." -- is ultimately written to this process's real stdout/stderr
    file descriptor, including from processes started later. So this
    redirects fd 1/2 through `tee`, which duplicates everything to both
    the terminal and the log file.
    """
    log_dir = _resolve_log_dir()
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = os.path.join(
        log_dir, f"final_angles_sequential_{timestamp}.txt")

    sys.stdout.flush()
    sys.stderr.flush()
    tee = subprocess.Popen(["tee", "-a", log_file_path], stdin=subprocess.PIPE)
    os.dup2(tee.stdin.fileno(), 1)
    os.dup2(tee.stdin.fileno(), 2)

    print(f"[final_angles_sequential] Writing this run's full output to {log_file_path}")
    return log_file_path


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

    direct_command_node = Node(
        package="control_arm",
        executable="direct_esp32_moveit_node",
        name="final_angles_sequential",
        output="screen",
        parameters=[{
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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_delay_seconds", default_value="1.0"),
        DeclareLaunchArgument("startup_delay_seconds", default_value="6.0"),
        DeclareLaunchArgument("x", default_value="0.166"),
        DeclareLaunchArgument("y", default_value="0.132"),
        DeclareLaunchArgument("z", default_value="0.0508"),
        DeclareLaunchArgument("ox", default_value="-0.548"),
        DeclareLaunchArgument("oy", default_value="0.447"),
        DeclareLaunchArgument("oz", default_value="0.006"),
        DeclareLaunchArgument("ow", default_value="0.707"),
        # If true, joint_1 (the base rotator) is locked to its current value
        # for the whole plan via a MoveIt path constraint, so only joints 2-6
        # may move.
        DeclareLaunchArgument("freeze_joint_1", default_value="false"),
        moveit_hardware,
        TimerAction(
            period=LaunchConfiguration("startup_delay_seconds"),
            actions=[direct_command_node],
        ),
    ])
