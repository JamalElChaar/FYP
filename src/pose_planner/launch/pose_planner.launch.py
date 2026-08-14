#!/usr/bin/env python3
"""
Launch the pose planner node.

Prerequisites:
    MoveIt 2 + RViz must already be running, e.g.:
        ros2 launch robot_arm_movit_config demo.launch.py

Usage:
    # Default target pose
    ros2 launch pose_planner pose_planner.launch.py

    # Custom target pose (position only – keeps default orientation)
    ros2 launch pose_planner pose_planner.launch.py x:=0.3 y:=0.0 z:=0.3

    # Custom position + orientation (quaternion)
    ros2 launch pose_planner pose_planner.launch.py x:=0.3 y:=0.1 z:=0.25 ox:=0.0 oy:=0.707 oz:=0.0 ow:=0.707
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Target position ──
        DeclareLaunchArgument('x',  default_value='0.166', description='Target X position (metres)'),
        DeclareLaunchArgument('y',  default_value='0.132', description='Target Y position (metres)'),
        DeclareLaunchArgument('z',  default_value='0.0508', description='Target Z position (metres)'),

        # ── Target orientation (quaternion) ──
        DeclareLaunchArgument('ox', default_value='-0.548', description='Target orientation X'),
        DeclareLaunchArgument('oy', default_value='0.447',  description='Target orientation Y'),
        DeclareLaunchArgument('oz', default_value='0.006',  description='Target orientation Z'),
        DeclareLaunchArgument('ow', default_value='0.707',  description='Target orientation W'),

        # ── Node ──
        Node(
            package='pose_planner',
            executable='pose_planner_node',
            name='pose_planner_node',
            output='screen',
            parameters=[{
                'x':  LaunchConfiguration('x'),
                'y':  LaunchConfiguration('y'),
                'z':  LaunchConfiguration('z'),
                'ox': LaunchConfiguration('ox'),
                'oy': LaunchConfiguration('oy'),
                'oz': LaunchConfiguration('oz'),
                'ow': LaunchConfiguration('ow'),
            }],
        ),
    ])
