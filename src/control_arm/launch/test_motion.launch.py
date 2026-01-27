#!/usr/bin/env python3
"""
Launch file for test_motion_node.

This launch file starts the test motion planning node which demonstrates
basic MoveIt 2 motion planning with the robot_arm.

Prerequisites:
    - MoveIt 2 must be running (use demo.launch.py or full_robot_moveit.launch.py)
    
Usage:
    ros2 launch control_arm test_motion.launch.py

:author: Jamal
:date: January 27, 2026
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Test motion node
    test_motion_node = Node(
        package='control_arm',
        executable='test_motion_node',
        name='test_motion_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        test_motion_node,
    ])
