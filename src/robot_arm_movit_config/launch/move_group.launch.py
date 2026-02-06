from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock"
    )
    
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="robot_arm_movit_config").to_moveit_configs()
    
    # Get the base launch description
    ld = generate_move_group_launch(moveit_config)
    
    # Create new launch description with use_sim_time argument first
    new_ld = LaunchDescription()
    new_ld.add_action(use_sim_time_arg)
    
    # Add all original entities
    for entity in ld.entities:
        new_ld.add_action(entity)
    
    return new_ld
