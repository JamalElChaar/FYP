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
                # Measured from the ground, x=y=0 at the base centre:
                # (-0.6625, -0.3475, 0.25); base_link is 0.006 m up -> z 0.244.
                'camera_x': '-0.2469',
                'camera_y': '-0.5069',
                'camera_z': '0.228',
                'camera_pitch': '0.314159265',
                'camera_yaw': '0.785398163',
            }.items(),
        ),
    ])
