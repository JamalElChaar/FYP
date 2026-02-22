#!/usr/bin/env python3
"""
Launch manual hardware control with micro-ROS and slider GUI.

This launch file starts:
1) micro-ROS agent (UDP)
2) robot_state_publisher
3) joint_state_publisher_gui (manual sliders)
4) bridge node: /joint_states -> /esp32/joint_commands
5) RViz
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
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
    package_name_description = 'robot_arm_description'
    pkg_share_description = FindPackageShare(
        package=package_name_description).find(package_name_description)

    default_urdf_model_path = os.path.join(
        pkg_share_description, 'urdf', 'robot', 'arm.urdf.xacro')
    default_rviz_config_path = os.path.join(
        pkg_share_description, 'rviz', 'robot_arm_description.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    agent_port = LaunchConfiguration('agent_port')
    start_agent = LaunchConfiguration('start_agent')
    start_slider_bridge = LaunchConfiguration('start_slider_bridge')
    start_jsp_gui = LaunchConfiguration('start_jsp_gui')
    auto_return_to_neutral = LaunchConfiguration('auto_return_to_neutral')
    neutral_delay_sec = LaunchConfiguration('neutral_delay_sec')

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

    declare_agent_port_cmd = DeclareLaunchArgument(
        name='agent_port',
        default_value='8888',
        description='UDP port for micro-ROS agent')

    declare_start_agent_cmd = DeclareLaunchArgument(
        name='start_agent',
        default_value='true',
        description='Whether to start micro-ROS agent from this launch file')

    declare_start_slider_bridge_cmd = DeclareLaunchArgument(
        name='start_slider_bridge',
        default_value='true',
        description='Whether to start slider-to-ESP32 bridge node')

    declare_start_jsp_gui_cmd = DeclareLaunchArgument(
        name='start_jsp_gui',
        default_value='true',
        description='Whether to start joint_state_publisher_gui')

    declare_auto_return_to_neutral_cmd = DeclareLaunchArgument(
        name='auto_return_to_neutral',
        default_value='true',
        description='Auto-return ESP32 command to neutral after delay')

    declare_neutral_delay_sec_cmd = DeclareLaunchArgument(
        name='neutral_delay_sec',
        default_value='2.0',
        description='Delay before neutral auto-return command (seconds)')

    robot_description_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_model,
        ' use_gazebo:=false',
        ' use_camera:=false',
    ]), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'udp4', '--port', agent_port
        ],
        output='screen',
        shell=False,
        condition=IfCondition(start_agent),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_jsp_gui),
    )

    slider_bridge_node = Node(
        package='custom_hardware',
        executable='joint_state_to_esp32_bridge',
        name='joint_state_to_esp32_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_return_to_neutral': auto_return_to_neutral,
            'neutral_delay_sec': neutral_delay_sec,
            'neutral_command_deg': [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],
        }],
        condition=IfCondition(start_slider_bridge),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    delayed_robot_state_publisher = TimerAction(
        period=1.0,
        actions=[robot_state_publisher_node],
    )
    delayed_joint_state_publisher_gui = TimerAction(
        period=1.2,
        actions=[joint_state_publisher_gui_node],
    )
    delayed_slider_bridge = TimerAction(
        period=2.0,
        actions=[slider_bridge_node],
    )
    delayed_rviz = TimerAction(
        period=2.0,
        actions=[rviz_node],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_agent_port_cmd)
    ld.add_action(declare_start_agent_cmd)
    ld.add_action(declare_start_slider_bridge_cmd)
    ld.add_action(declare_start_jsp_gui_cmd)
    ld.add_action(declare_auto_return_to_neutral_cmd)
    ld.add_action(declare_neutral_delay_sec_cmd)

    ld.add_action(micro_ros_agent)
    ld.add_action(delayed_robot_state_publisher)
    ld.add_action(delayed_joint_state_publisher_gui)
    ld.add_action(delayed_slider_bridge)
    ld.add_action(delayed_rviz)

    return ld
