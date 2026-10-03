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
         + ('Joint angles will be PUBLISHED to /esp32/joint_commands; the arm will move.\n'
            if value('execute_on_hardware').lower() == 'true'
            else 'Only joint-angle calculation is enabled. No motor or trajectory commands are sent.\n'))
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir / 'ros')),
               SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
               LogInfo(msg=f'Vision run logs: {run_dir}'),
               RegisterEventHandler(OnProcessExit(on_exit=process_exit)),
               RegisterEventHandler(OnShutdown(on_shutdown=shutdown))]
    sim = boolean('use_sim_time')
    image_mode = value('input_mode') == 'image'
    # Joints with no working servo; shared by the IK node (which sends NaN for
    # them) and the RViz joint-state node (which holds them at the seed).
    disabled_joints = [n for n in value('disabled_joints').replace(' ', '').split(',') if n]
    # Replay a saved image through the LIVE path: the only thing swapped out is
    # the camera driver. The camera TF is still published and the transform is
    # still looked up from TF, so the configured extrinsics apply exactly as
    # they would with a real capture -- unlike plain image mode, which reads a
    # transform recorded inside the calibration file and ignores the config.
    replay_as_live = image_mode and boolean('replay_as_live')
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
        # Nothing else in this launch publishes /joint_states -- the pipeline
        # solves IK but never executes -- so without this RViz would show the
        # arm collapsed at zeros. This shows the SOLVED POSE, not the motion:
        # the arm snaps from the seed to the solution, uninterpolated.
        actions.append(Node(
            package='computer_vision_model', executable='solution_joint_state_node',
            name='solution_joint_states', output='screen',
            parameters=[dict({'use_sim_time': sim},
                              **({'held_joints': disabled_joints} if disabled_joints else {}))]))
        if boolean('start_rviz'):
            actions.append(Node(
                package='rviz2', executable='rviz2', name='vision_rviz', output='screen',
                arguments=['-d', str(Path(get_package_share_directory(
                    'robot_arm_movit_config')) / 'config' / 'moveit.rviz')],
                parameters=[config.robot_description,
                            config.robot_description_semantic,
                            config.robot_description_kinematics,
                            {'use_sim_time': sim}]))
    if boolean('publish_camera_tf') and (not image_mode or replay_as_live):
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='vision_camera_mount_tf', output='screen', parameters=[{'use_sim_time': sim}],
            arguments=['--x', value('cam_x'), '--y', value('cam_y'), '--z', value('cam_z'),
                       '--roll', value('cam_roll'), '--pitch', value('cam_pitch'), '--yaw', value('cam_yaw'),
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link']))
        # camera_link -> camera_color_optical_frame, the standard optical
        # rotation (-90, 0, -90). Published in BOTH modes, not just replay.
        #
        # The projection looks up base_link -> the capture's rgb_frame_id,
        # which is camera_color_optical_frame. The mount transform above only
        # reaches camera_link, so without this edge the chain is broken and
        # can_transform() never becomes true -- the pipeline stalls silently at
        # TRANSFORM until stage_timeout. The Astra driver was relied on to
        # supply it in live mode and did not. Publishing it here also makes the
        # live and replay frame trees identical by construction.
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='vision_camera_optical_tf', output='screen',
            parameters=[{'use_sim_time': sim}],
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '-1.570796327', '--pitch', '0', '--yaw', '-1.570796327',
                       '--frame-id', 'camera_link',
                       '--child-frame-id', 'camera_color_optical_frame']))
    if boolean('start_camera') and not image_mode:
        driver_launch = Path(get_package_share_directory('astra_camera')) / 'launch/astra_pro.launch.xml'
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(str(driver_launch)), launch_arguments={
            'camera_name': 'camera', 'enable_color': 'true', 'enable_depth': 'false',
            'enable_ir': 'false', 'enable_point_cloud': 'false', 'enable_colored_point_cloud': 'false',
            'depth_registration': 'false', 'color_depth_synchronization': 'false',
            # false: the optical-frame edge is published above, and two
            # publishers on one edge make the lookup nondeterministic.
            'publish_tf': 'false', 'use_uvc_camera': 'true',
            # The Astra Pro's UVC colour sensor supports MJPG 1280x720 @30 fps.
            # The driver derives its default intrinsics from the configured
            # resolution (getDefaultCameraInfo(width, height, ...)), so fx/fy
            # and cx/cy scale automatically and the projection stays valid.
            'color_width': value('color_width'), 'color_height': value('color_height'),
            'color_fps': value('color_fps')}.items()))
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
    # Same omit-when-empty rule as the class filters below.
    joint_parameters = {'disabled_joints': disabled_joints} if disabled_joints else {}
    detection_parameters = {
        'run_on_start': boolean('run_on_start'), 'one_shot': True,
        'startup_delay_seconds': float(value('startup_delay_seconds')),
        'minimum_confidence': float(value('minimum_confidence')),
        'model_id': value('model_id'), 'api_url': value('api_url'),
        'capture_timeout': float(value('stage_timeout')),
        'inference_timeout': float(value('stage_timeout')),
        'phase_log_path': str(phase_path), 'use_sim_time': sim}
    # An empty list reaches launch_ros as an empty tuple, which it rejects
    # ("Expected 'value' to be one of [float, int, str, bool, bytes], but got
    # '()'"), so an unset filter is omitted entirely and the node's own default
    # (no filtering) applies.
    for name in ('excluded_classes', 'allowed_classes', 'class_aliases'):
        labels = [n for n in value(name).replace(' ', '').split(',') if n]
        if labels:
            detection_parameters[name] = labels
    actions += [
        Node(package='computer_vision_model', executable='fruit_detection_node',
             output='screen', parameters=[detection_parameters]),
        Node(package='computer_vision_model', executable='camera_to_base_node', output='screen', parameters=[dict(joint_parameters, **{
            'transform_source': 'recorded' if (image_mode and not replay_as_live) else 'tf',
            'plane_z': float(value('plane_z')), 'target_offset_z': float(value('target_offset_z')),
            'stage_timeout': float(value('stage_timeout')), 'ik_timeout': float(value('ik_timeout')),
            'ox': float(value('ox')), 'oy': float(value('oy')), 'oz': float(value('oz')), 'ow': float(value('ow')),
            'tip_link': value('tip_link'),
            'avoid_collisions': boolean('avoid_collisions'),
            'position_only': boolean('position_only'),
            'execute_on_hardware': boolean('execute_on_hardware'),
            'home_on_start': boolean('home_on_start'),
            'home_settle_seconds': float(value('home_settle_seconds')),
            'stab_enabled': boolean('stab_enabled'),
            'stab_distance_m': float(value('stab_distance_m')),
            'stab_pipeline_id': value('stab_pipeline_id'),
            'stab_planner_id': value('stab_planner_id'),
            'stab_lin_attempts': int(value('stab_lin_attempts')),
            'stab_fallback_enabled': boolean('stab_fallback_enabled'),
            'stab_fallback_pipeline_id': value('stab_fallback_pipeline_id'),
            'stab_fallback_planner_id': value('stab_fallback_planner_id'),
            'stab_planning_time': float(value('stab_planning_time')),
            'stab_duration_seconds': float(value('stab_duration_seconds')),
            'override_point_base': value('override_point_base'),
            'command_settle_seconds': float(value('command_settle_seconds')),
            'orientation_attempts': int(value('orientation_attempts')),
            'output_directory': str(captures), 'phase_log_path': str(phase_path), 'use_sim_time': sim})])]
    return actions


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    defaults = {
        'logs_directory': str(package_root / 'logs'),
        # 'camera' = live Astra capture; 'image' = replay a saved photo.
        # fruit_image_pipeline.launch.py is just this launch with input_mode
        # forced to 'image'.
        'input_mode': 'camera',
        # Comma-separated class labels, case-insensitive. excluded_classes drops
        # those labels; allowed_classes, when set, keeps ONLY those. Filtering
        # happens at detection, so a rejected class can never reach projection
        # even if it is the most confident thing in frame.
        # cucumber is NOT excluded: the model labels the banana "Cucumber",
        # and excluding it would drop the only real target. It is renamed
        # below instead.
        'excluded_classes': 'carrot', 'allowed_classes': '',
        # Display renaming, "from:to" pairs. Applied AFTER filtering, so the
        # filters above still match what the model actually returned; the raw
        # label is kept on every detection as original_label.
        'class_aliases': 'apple:banana,cucumber:banana',
        # Bench override: "x,y,z" in metres, base_link frame. Replaces the
        # projected detection point; everything else runs for real. Empty = off.
        'override_point_base': '',
        'image_path': str(package_root / 'input_images' / 'banana_scene.jpg'),
        # Replay a saved image through the live TF path rather than the
        # transform recorded in its calibration file.
        'replay_as_live': 'true',
        'calibration_path': '',
        'start_camera': 'true', 'start_moveit': 'true', 'publish_camera_tf': 'true',
        # Show the solved pose in RViz. Requires start_moveit, which provides
        # the robot description and the /joint_states source.
        'start_rviz': 'true',
        'run_on_start': 'true', 'use_sim_time': 'false',
        'rgb_topic': '/camera/color/image_raw', 'camera_info_topic': '/camera/color/camera_info',
        # 640x480 (4:3), not the sensor's 1280x720 maximum. The driver's
        # default intrinsics hardcode a 4:3 assumption
        # (utils.cpp: cy = width * 3/8 - 0.5), so at 16:9 it reports
        # cy = 479.5 instead of the correct 359.5 -- a 120 px error, about
        # 8 degrees, which corrupts the projection. 720p is only safe once a
        # real colour calibration is supplied via color_info_url.
        'color_width': '640', 'color_height': '480', 'color_fps': '30',
        # Fitted from the arm base visible in a capture; see the xacro for details.
        'cam_x': '-0.2469', 'cam_y': '-0.5069', 'cam_z': '0.228',
        'cam_roll': '0.0', 'cam_pitch': '0.314159265', 'cam_yaw': '0.785398163',
        'plane_z': '-0.006',
        # No approach height: the first pose is the fruit plane itself. A
        # hover above it was tried at 10 and 15 cm and IK found no solution --
        # the fruit sits only ~19 cm from the base, too close to reach over.
        'target_offset_z': '0.0',
        'ox': '-0.548', 'oy': '0.447', 'oz': '0.006', 'ow': '0.707', 'tip_link': 'link_6',
        # Reaching an object is a position task. Pinning the wrist makes KDL
        # reject positions the arm can plainly reach, so by default the
        # configured orientation is tried first and then relaxed.
        'position_only': 'true', 'orientation_attempts': '40',
        # Publish the IK solution to /esp32/joint_commands and move the real
        # arm. Off by default: the input is a vision detection, and a wrong
        # one would drive the servos somewhere real.
        'execute_on_hardware': 'false', 'command_settle_seconds': '1.0',
        # Joints we do not drive. Still planned for -- the arm model needs them
        # to reach the target -- but NaN (the firmware's "hold" value) goes in
        # their slot and their servo angle is exempt from the 0-180 range
        # check, so neither can block joints 2-5 from moving.
        #   joint_1: no servo pin in the firmware at all.
        #   joint_6: unbounded in the URDF, so IK freely returns wrist-roll
        #            angles its 0-180 servo cannot reach; we do not care where
        #            the wrist ends up, so it is held rather than limited.
        'disabled_joints': 'joint_1,joint_6',
        # Stab: a second, OMPL-planned motion straight down from the hover
        # pose. Needs execute_on_hardware -- it is a real movement.
        # Drive every servo to 90 and let it settle before the pipeline runs,
        # so each run starts from the firmware's boot pose rather than wherever
        # the previous run left the arm. Needs execute_on_hardware.
        'home_on_start': 'true', 'home_settle_seconds': '3.0',
        'stab_enabled': 'true', 'stab_distance_m': '0.05',
        # Pilz LIN: a straight line in Cartesian space, so the fork descends
        # vertically. OMPL plans in joint space and only pins the endpoints,
        # which let the tip curve between them -- differently on every run.
        'stab_pipeline_id': 'pilz_industrial_motion_planner',
        'stab_planner_id': 'LIN',
        # If LIN cannot hold the line, fall back to joint-space planning after
        # this many attempts. LIN is deterministic, so retries repeat the same
        # result -- stab_lin_attempts:=1 falls back immediately.
        'stab_lin_attempts': '10', 'stab_fallback_enabled': 'true',
        'stab_fallback_pipeline_id': 'ompl',
        'stab_fallback_planner_id': 'RRTConnectkConfigDefault',
        'stab_planning_time': '5.0', 'stab_duration_seconds': '2.0',
        'avoid_collisions': 'true',
        # 0.3 to catch weaker live-camera detections. NOTE: 0.5 was chosen to
        # drop a 0.490 false positive on the arm itself; at 0.3 that comes back,
        # and since selection takes the highest-confidence detection it can win.
        # Use allowed_classes to keep the filtering reliable at this threshold.
        'minimum_confidence': '0.3', 'stage_timeout': '60.0', 'ik_timeout': '2.0',
        # Give TF time to settle before the snapshot. The transform is looked
        # up at the capture timestamp, and tf2 only keeps ~10 s of history, so
        # capturing before the frames are flowing makes the lookup unsatisfiable.
        'startup_delay_seconds': '10.0',
        'model_id': 'sliced-fruits-and-vegetables-rnw8f/1', 'api_url': 'https://serverless.roboflow.com'}
    flags = {'start_camera', 'start_moveit', 'publish_camera_tf', 'run_on_start', 'start_rviz',
             'use_sim_time', 'position_only', 'avoid_collisions', 'replay_as_live',
             'execute_on_hardware', 'stab_enabled', 'home_on_start', 'stab_fallback_enabled'}
    arguments = [DeclareLaunchArgument(k, default_value=v, **({'choices': ['true', 'false']} if k in flags else
                    {'choices': ['camera', 'image']} if k == 'input_mode' else {}))
                 for k, v in defaults.items()]
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
