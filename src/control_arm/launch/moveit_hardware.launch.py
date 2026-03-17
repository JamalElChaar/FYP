#!/usr/bin/env python3
"""
Launch MoveIt2 with real hardware (CustomHardwareInterface → ESP32).

Uses the exact same generate_demo_launch() as the working demo.launch.py,
but swaps the URDF to use CustomHardwareInterface instead of mock hardware.

Usage:
  Real hardware:  ros2 launch control_arm moveit_hardware.launch.py
  Test mode:      ros2 launch control_arm moveit_hardware.launch.py test_mode:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    test_mode = LaunchConfiguration('test_mode').perform(context)
    use_simulation = 'true' if test_mode.lower() == 'true' else 'false'

    description_pkg = get_package_share_directory('robot_arm_description')

    # Use the SAME MoveItConfigsBuilder as demo.launch.py, but override
    # robot_description to point at the real-hardware URDF instead of mock.
    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="robot_arm_movit_config")
        .robot_description(
            file_path=os.path.join(
                description_pkg, 'urdf', 'robot', 'arm.urdf.xacro'
            ),
            mappings={
                'use_gazebo': 'false',
                'use_camera': 'false',
                'use_simulation': use_simulation,
            },
        )
        .to_moveit_configs()
    )

    # generate_demo_launch gives us the exact same working stack as demo.launch.py
    demo_ld = generate_demo_launch(moveit_config)
    actions = list(demo_ld.entities)

    # Add micro-ROS agent when using real hardware (not in test mode)
    if use_simulation == 'false':
        agent_port = LaunchConfiguration('agent_port').perform(context)
        micro_ros_agent = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'udp4', '--port', agent_port,
            ],
            output='screen',
        )
        actions.insert(0, micro_ros_agent)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'test_mode', default_value='false',
            description='Test without ESP32: simulation mode, no micro-ROS agent'),
        DeclareLaunchArgument(
            'agent_port', default_value='8888',
            description='UDP port for micro-ROS agent'),
        OpaqueFunction(function=launch_setup),
    ])
