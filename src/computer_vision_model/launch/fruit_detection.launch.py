"""Launch Astra Pro capture followed by one Roboflow detection request."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    output_directory = LaunchConfiguration("output_directory")
    run_on_start = LaunchConfiguration("run_on_start")
    minimum_confidence = LaunchConfiguration("minimum_confidence")

    astra_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("astra_camera"),
                "launch",
                "astra_pro.launch.xml",
            ])
        ),
        launch_arguments={
            "camera_name": "camera",
            "depth_registration": "true",
            "color_depth_synchronization": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_ir": "false",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "publish_tf": "true",
            "use_uvc_camera": "true",
        }.items(),
    )

    capture_node = Node(
        package="computer_vision_model",
        executable="rgbd_capture_node",
        name="rgbd_capture_node",
        output="screen",
        parameters=[{
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "output_directory": output_directory,
            "capture_on_start": False,
            "require_matching_dimensions": True,
            "maximum_timestamp_delta_ms": 100.0,
            "depth_unit_m_per_value": 0.001,
        }],
    )

    detection_node = Node(
        package="computer_vision_model",
        executable="fruit_detection_node",
        name="fruit_detection_node",
        output="screen",
        parameters=[{
            "model_id": "sliced-fruits-and-vegetables-rnw8f/1",
            "api_url": "https://serverless.roboflow.com",
            "run_on_start": run_on_start,
            "startup_delay_seconds": 2.0,
            "minimum_confidence": minimum_confidence,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_directory",
            default_value="/tmp/computer_vision_captures",
            description="Directory for matching RGB, depth, and detections",
        ),
        DeclareLaunchArgument(
            "run_on_start",
            default_value="true",
            choices=["true", "false"],
            description="Capture and detect once after the nodes start",
        ),
        DeclareLaunchArgument(
            "minimum_confidence",
            default_value="0.25",
            description="Discard detections below this confidence",
        ),
        astra_driver,
        capture_node,
        detection_node,
    ])
