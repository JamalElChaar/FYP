#!/usr/bin/env python3
"""
Launch real robot with custom hardware interface.

This launch file sets up the ROS 2 control environment for the real robot_arm
using the custom hardware interface for communication with physical hardware.

:author: Jamal
:date: February 2026
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the real robot.

    Returns:
        LaunchDescription: A complete launch description for the real robot
    """
    # Package names
    package_name_description = 'robot_arm_description'
    package_name_moveit = 'robot_arm_movit_config'

    # Find package shares
    pkg_share_description = FindPackageShare(
        package=package_name_description).find(package_name_description)
    pkg_share_moveit = FindPackageShare(
        package=package_name_moveit).find(package_name_moveit)

    # Paths to files
    default_urdf_model_path = os.path.join(
        pkg_share_description, 'urdf', 'robot', 'arm.urdf.xacro')
    default_rviz_config_path = os.path.join(
        pkg_share_description, 'rviz', 'view_robot.rviz')
    default_controllers_file = os.path.join(
        pkg_share_moveit, 'config', 'ros2_controllers.yaml')

    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    controllers_file = LaunchConfiguration('controllers_file')

    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot URDF file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file')

    declare_controllers_file_cmd = DeclareLaunchArgument(
        name='controllers_file',
        default_value=default_controllers_file,
        description='Absolute path to ros2_controllers.yaml file')

    # Get robot description from xacro
    robot_description_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_model,
        ' use_gazebo:=false',  # Use custom hardware interface, not Gazebo
        ' use_camera:=false',
    ]), value_type=str)

    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    # Arm Controller Spawner (delayed to wait for controller manager)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    # Delay arm controller spawner after joint state broadcaster
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[arm_controller_spawner],
                ),
            ],
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_controllers_file_cmd)

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delayed_arm_controller_spawner)
    ld.add_action(rviz_node)

    return ld
