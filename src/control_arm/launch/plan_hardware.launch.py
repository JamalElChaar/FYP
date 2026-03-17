#!/usr/bin/env python3
"""
Launch the interactive MoveIt hardware planning node.

Prerequisites:
    moveit_hardware.launch.py must already be running.

Usage:
    ros2 launch control_arm plan_hardware.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')

    moveit_hardware_node = Node(
        package='control_arm',
        executable='moveit_hardware_node',
        name='moveit_hardware_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        # Run in a terminal so std::cin works for the interactive menu
        prefix='xterm -e' if False else '',
    )

    return LaunchDescription([
        use_sim_time_arg,
        moveit_hardware_node,
    ])
