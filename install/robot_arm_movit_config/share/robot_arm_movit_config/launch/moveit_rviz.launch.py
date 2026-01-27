from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="robot_arm_movit_config").to_moveit_configs()
    
    # Get the moveit_rviz launch description
    ld = generate_moveit_rviz_launch(moveit_config)
    
    # Add joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )
    
    ld.add_action(joint_state_publisher_node)
    
    return ld
