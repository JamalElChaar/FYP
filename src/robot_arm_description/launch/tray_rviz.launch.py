"""Display the arm seated in its printed tray with the Astra Pro in its cradle."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_arm_description'), 'launch',
                'robot_state_publisher.launch.py',
            ])),
            launch_arguments={
                'use_camera': 'true',
                'use_tray': 'true',
                'use_gazebo': 'false',
                'use_sim_time': 'false',
                'use_jsp': 'true',
                'jsp_gui': 'false',
                'use_rviz': 'true',
                # The printed cradle is generated at this fixed pose.
                'camera_x': '0.22',
                'camera_y': '-0.30',
                'camera_z': '0.50',
                'camera_pitch': '0.872664626',
            }.items(),
        ),
    ])
