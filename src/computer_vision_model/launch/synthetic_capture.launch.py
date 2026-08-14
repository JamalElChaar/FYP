from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_directory = LaunchConfiguration("output_directory")
    exit_after_capture = LaunchConfiguration("exit_after_capture")
    synthetic_max_frames = LaunchConfiguration("synthetic_max_frames")

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_directory",
            default_value="/tmp/computer_vision_captures",
            description="Directory for latest_rgb.jpg and matching depth files",
        ),
        DeclareLaunchArgument(
            "exit_after_capture",
            default_value="false",
            choices=["true", "false"],
            description="Stop the capture node after its startup capture",
        ),
        DeclareLaunchArgument(
            "synthetic_max_frames",
            default_value="0",
            description="Stop the synthetic publisher after N frames; 0 runs forever",
        ),
        Node(
            package="computer_vision_model",
            executable="synthetic_rgbd_publisher",
            name="synthetic_rgbd_publisher",
            output="screen",
            parameters=[{"max_frames": synthetic_max_frames}],
        ),
        Node(
            package="computer_vision_model",
            executable="rgbd_capture_node",
            name="rgbd_capture_node",
            output="screen",
            parameters=[{
                "output_directory": output_directory,
                "capture_on_start": True,
                "exit_after_capture": exit_after_capture,
            }],
        ),
    ])
