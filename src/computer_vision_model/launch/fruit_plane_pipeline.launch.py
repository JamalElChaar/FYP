#!/usr/bin/env python3
"""One RGB snapshot -> Roboflow -> known-plane projection -> MoveIt IK, with logs."""
import datetime
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def boolean(name):
        return value(name).lower() == 'true'
    run_dir = Path(value('logs_directory')).expanduser().resolve() / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run_dir.mkdir(parents=True)
    captures = run_dir / 'captures'
    phase_path = run_dir / 'phases.txt'
    phase_path.write_text('VISION PIPELINE PHASE REPORT\nAll Cartesian values are metres; joint solutions are ROS angles.\n')
    # Mirror the launch process's stdout/stderr, including prefixed child output.
    # A root Python logging handler misses ROS launch loggers with propagation disabled.
    sys.stdout.flush()
    sys.stderr.flush()
    tee = subprocess.Popen(['tee', '-a', str(run_dir / 'launch.log')],
                           stdin=subprocess.PIPE, start_new_session=True)
    os.dup2(tee.stdin.fileno(), 1)
    os.dup2(tee.stdin.fileno(), 2)
    tee.stdin.close()  # fd 1/2 retain the pipe; tee exits at EOF when launch exits.

    def note(phase, detail):
        with phase_path.open('a') as stream:
            stream.write(f'\n[{datetime.datetime.now().astimezone().isoformat()}] {phase}\n{detail}\n')

    shutting_down = [False]

    def process_exit(event, _context):
        if event.returncode and not shutting_down[0]:
            note('PROCESS EXIT', f'{event.action.name}: exit code {event.returncode}; see launch.log for driver/process error details.')
        return []

    def shutdown(event, _context):
        shutting_down[0] = True
        note('LAUNCH SHUTDOWN', str(event.reason))
        return []

    note('CONFIGURATION', f'Camera body reference relative to base: ({value("cam_x")}, {value("cam_y")}, {value("cam_z")}) m.\n'
         f'Camera RPY: ({value("cam_roll")}, {value("cam_pitch")}, {value("cam_yaw")}) rad.\n'
         f'Fruit plane Z: {value("plane_z")} m. Camera extrinsics are configured values, not automatically calibrated.\n'
         'Only joint-angle calculation is enabled. No motor or trajectory commands are sent.')
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir / 'ros')),
               SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
               LogInfo(msg=f'Vision run logs: {run_dir}'),
               RegisterEventHandler(OnProcessExit(on_exit=process_exit)),
               RegisterEventHandler(OnShutdown(on_shutdown=shutdown))]
    sim = boolean('use_sim_time')
    image_mode = value('input_mode') == 'image'
    if image_mode:
        note('IMAGE MODE', f'Image: {value("image_path")}\nCalibration: {value("calibration_path") or "auto-discover beside image"}\n'
             'Using the recorded optical-to-base transform from the calibration YAML; live camera mount arguments are ignored.')
    if boolean('start_moveit'):
        config = MoveItConfigsBuilder('robot_arm', package_name='robot_arm_movit_config').to_moveit_configs()
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[config.robot_description, {'use_sim_time': sim}]),
            Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=[config.to_dict(), {'use_sim_time': sim,
                    'allow_trajectory_execution': False,
                    'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
                    'publish_robot_description': True,
                    'publish_robot_description_semantic': True}])]
    if boolean('publish_camera_tf') and not image_mode:
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='vision_camera_mount_tf', output='screen', parameters=[{'use_sim_time': sim}],
            arguments=['--x', value('cam_x'), '--y', value('cam_y'), '--z', value('cam_z'),
                       '--roll', value('cam_roll'), '--pitch', value('cam_pitch'), '--yaw', value('cam_yaw'),
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link']))
    if boolean('start_camera') and not image_mode:
        driver_launch = Path(get_package_share_directory('astra_camera')) / 'launch/astra_pro.launch.xml'
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(str(driver_launch)), launch_arguments={
            'camera_name': 'camera', 'enable_color': 'true', 'enable_depth': 'false',
            'enable_ir': 'false', 'enable_point_cloud': 'false', 'enable_colored_point_cloud': 'false',
            'depth_registration': 'false', 'color_depth_synchronization': 'false',
            'publish_tf': 'true', 'use_uvc_camera': 'true'}.items()))
    if image_mode:
        actions.append(Node(package='computer_vision_model', executable='saved_image_capture_node',
            output='screen', parameters=[{
                'image_path': value('image_path'), 'calibration_path': value('calibration_path'),
                'output_directory': str(captures), 'phase_log_path': str(phase_path), 'use_sim_time': sim}]))
    else:
        actions.append(Node(package='computer_vision_model', executable='rgbd_capture_node',
            output='screen', parameters=[{
                'rgb_topic': value('rgb_topic'), 'camera_info_topic': value('camera_info_topic'),
                'require_depth': False, 'maximum_rgb_age_ms': 2000.0, 'capture_on_start': False,
                'output_directory': str(captures), 'use_sim_time': sim}]))
    actions += [
        Node(package='computer_vision_model', executable='fruit_detection_node', output='screen', parameters=[{
            'run_on_start': boolean('run_on_start'), 'one_shot': True,
            'startup_delay_seconds': 2.0, 'minimum_confidence': float(value('minimum_confidence')),
            'model_id': value('model_id'), 'api_url': value('api_url'),
            'capture_timeout': float(value('stage_timeout')), 'inference_timeout': float(value('stage_timeout')), 'phase_log_path': str(phase_path), 'use_sim_time': sim}]),
        Node(package='computer_vision_model', executable='camera_to_base_node', output='screen', parameters=[{
            'transform_source': 'recorded' if image_mode else 'tf',
            'plane_z': float(value('plane_z')), 'target_offset_z': float(value('target_offset_z')),
            'stage_timeout': float(value('stage_timeout')), 'ik_timeout': float(value('ik_timeout')),
            'ox': float(value('ox')), 'oy': float(value('oy')), 'oz': float(value('oz')), 'ow': float(value('ow')),
            'tip_link': value('tip_link'), 'avoid_collisions': True,
            'output_directory': str(captures), 'phase_log_path': str(phase_path), 'use_sim_time': sim}])]
    return actions


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    defaults = {
        'logs_directory': str(package_root / 'logs'),
        'input_mode': 'camera', 'image_path': str(package_root / 'input_images' / 'fruit.jpg'),
        'calibration_path': '',
        'start_camera': 'true', 'start_moveit': 'true', 'publish_camera_tf': 'true',
        'run_on_start': 'true', 'use_sim_time': 'false',
        'rgb_topic': '/camera/color/image_raw', 'camera_info_topic': '/camera/color/camera_info',
        'cam_x': '0.22', 'cam_y': '-0.29', 'cam_z': '0.434',
        'cam_roll': '0.0', 'cam_pitch': '0.872664626', 'cam_yaw': '1.570796327',
        'plane_z': '-0.006', 'target_offset_z': '0.0',
        'ox': '-0.548', 'oy': '0.447', 'oz': '0.006', 'ow': '0.707', 'tip_link': 'link_6',
        'minimum_confidence': '0.25', 'stage_timeout': '60.0', 'ik_timeout': '2.0',
        'model_id': 'sliced-fruits-and-vegetables-rnw8f/1', 'api_url': 'https://serverless.roboflow.com'}
    flags = {'start_camera', 'start_moveit', 'publish_camera_tf', 'run_on_start', 'use_sim_time'}
    arguments = [DeclareLaunchArgument(k, default_value=v, **({'choices': ['true', 'false']} if k in flags else
                    {'choices': ['camera', 'image']} if k == 'input_mode' else {}))
                 for k, v in defaults.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
