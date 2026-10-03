#!/usr/bin/env python3
"""Arm in RViz with joint sliders, for calibrating a servo against reality.

Model only: no MoveIt, no ros2_control, no ESP32. Nothing here can command a
servo, so it is safe to run alongside a powered arm.

Procedure for one joint (joint_1 shown; substitute the index for others):

  1. Command that servo to a known value, leaving the rest held. NaN means
     "keep your current position", so only the chosen joint moves:

       ros2 topic pub --once /esp32/joint_commands \\
         std_msgs/msg/Float64MultiArray "{data: [90.0, .nan, .nan, .nan, .nan, .nan]}"

  2. In the slider window, drag that joint until the on-screen arm matches
     the physical one. Read the angle off the slider -- that is the ROS angle
     corresponding to the servo value you commanded.

  3. Repeat at a second servo value, far enough away to measure accurately
     (45 deg of separation is plenty).

Two (servo, ros) pairs give the calibration exactly:

    direction = (servo_b - servo_a) / (ros_b - ros_a)      # should be +1 or -1
    offset    = servo_a - direction * ros_a

Send both pairs over and the constants get updated in all four places that
hold them, along with the joint limits, which are derived from the offset.

The tray and camera are hidden so nothing obscures the arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_tray", default_value="false",
            description="Show the printed tray mesh (off: it hides the arm)"),
        DeclareLaunchArgument(
            "use_camera", default_value="false",
            description="Show the camera and mast"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("robot_arm_description"), "launch",
                "robot_state_publisher.launch.py"])),
            launch_arguments={
                "use_rviz": "true",
                # jsp_gui gives the sliders; plain use_jsp must stay off or a
                # second publisher fights it for /joint_states.
                "jsp_gui": "true",
                "use_jsp": "false",
                "use_tray": LaunchConfiguration("use_tray"),
                "use_camera": LaunchConfiguration("use_camera"),
                "use_gazebo": "false",
                "use_sim_time": "false",
            }.items(),
        ),
    ])
