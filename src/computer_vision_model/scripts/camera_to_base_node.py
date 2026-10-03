#!/usr/bin/env python3
"""Project the most confident detection onto a base-frame plane and request IK.

No trajectory execution or motor command publishers are created by this node.
"""
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, RobotState, WorkspaceParameters)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import tf2_ros
import yaml

from vision_pipeline_support import phase_log, project_box_to_plane


class CameraToBaseNode(Node):
    def __init__(self):
        super().__init__('camera_to_base_node')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.base_frame = param('base_frame', 'base_link')
        self.transform_source = param('transform_source', 'tf')
        if self.transform_source not in ('tf', 'recorded'):
            raise ValueError('transform_source must be tf or recorded')
        self.plane_z = float(param('plane_z', -0.006))
        self.target_offset_z = float(param('target_offset_z', 0.0))
        self.group = param('planning_group', 'arm')
        self.tip = param('tip_link', 'link_6')
        self.log_path = param('phase_log_path', '')
        self.output_directory = Path(param('output_directory', '/tmp/computer_vision_captures')).resolve()
        self.timeout = float(param('stage_timeout', 45.0))
        self.ik_timeout = float(param('ik_timeout', 2.0))
        self.avoid_collisions = bool(param('avoid_collisions', True))
        self.joint_names = list(param('joint_names', [f'joint_{i}' for i in range(1, 7)]))
        # SRDF "home" state = servo 90 on every joint under the 2026-09-29
# calibration; used only as an IK seed if live state is unavailable.
        self.seed = list(param('seed_positions', [0.0, -0.238, -0.373, 1.570796, 0.0, 0.0]))
        self.orientation = np.array([float(param(k, v)) for k, v in
                                     zip(('ox', 'oy', 'oz', 'ow'), (-0.548, 0.447, 0.006, 0.707))])
        # Reaching an object is a position task: which way the wrist happens to
        # be turned rarely matters. KDL solves the full 6-DOF pose, so a pinned
        # orientation is a hard constraint that discards most solutions and
        # commonly yields NO_IK_SOLUTION at a position the arm can clearly
        # reach. With position_only, the configured orientation is tried first
        # and then relaxed outward until the position is achievable.
        # Optional hardware execution. Off by default: this node's input is a
        # vision detection, which in practice is sometimes a pillow or the arm
        # itself, and a bad detection would drive the servos somewhere real.
        self.execute_on_hardware = bool(param('execute_on_hardware', False))
        self.servo_offsets = list(param('servo_offsets_deg', [90.0, 103.6364, 111.3713, 0.0, 90.0, 90.0]))
        self.servo_directions = list(param('servo_directions', [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]))
        self.servo_min_deg = float(param('servo_min_deg', 0.0))
        self.servo_max_deg = float(param('servo_max_deg', 180.0))
        # Joints with no working servo on the hardware. They are still planned
        # for and still appear in the IK solution -- the arm model needs them
        # to reach the target -- but NaN is sent in their slot, which the
        # firmware treats as "hold", and their servo angle is exempt from the
        # range check so one dead joint cannot block the other five.
        self.disabled_joints = [str(n) for n in param('disabled_joints', ['joint_1'])]
        unknown = [n for n in self.disabled_joints if n not in self.joint_names]
        if unknown:
            raise ValueError(f'disabled_joints names no such joint: {unknown}')
        # Joints whose IK answer must map into the servo range even though the
        # URDF leaves them unbounded. joint_6 is continuous in the model, so KDL
        # happily returns wrist angles no servo can produce (-152 deg -> servo
        # -62); those solutions are geometrically valid and physically useless,
        # so they are rejected here and the search retries.
        # joint_1 is deliberately NOT listed: it keeps its full 360 deg of
        # freedom, since it has no servo pin at all and is held via NaN.
        # This is an IK-acceptance rule ONLY: it does not change the URDF, the
        # hold/NaN behaviour, or what is published.
        self.ik_servo_range_joints = [str(n) for n in
                                      param('ik_servo_range_joints', ['joint_6'])]
        unknown = [n for n in self.ik_servo_range_joints if n not in self.joint_names]
        if unknown:
            raise ValueError(f'ik_servo_range_joints names no such joint: {unknown}')
        # Bench override: use a fixed base-frame point instead of the one
        # projected from the detection. Everything else still runs for real --
        # capture, inference, selection, IK, planning, execution -- so the arm
        # path can be exercised while the camera extrinsics are still wrong.
        # Format "x,y,z" in metres; empty disables it.
        self.override_point_base = [
            float(n) for n in
            str(param('override_point_base', '')).replace(' ', '').split(',') if n]
        if self.override_point_base and len(self.override_point_base) != 3:
            raise ValueError('override_point_base must be "x,y,z" in metres')
        self.command_settle_seconds = float(param('command_settle_seconds', 1.0))
        # Home the arm before doing anything else, so every run starts from the
        # same known pose. The ESP32 never reports its position, so without this
        # the IK seed is a guess about where the arm was left by the last run.
        # Servo 90 on every joint is the firmware's own boot pose.
        self.home_on_start = bool(param('home_on_start', True))
        self.home_servo_degrees = [float(v) for v in
                                   param('home_servo_degrees', [90.0] * 6)]
        self.home_settle_seconds = float(param('home_settle_seconds', 3.0))
        self.home_wait_seconds = float(param('home_wait_seconds', 10.0))
        if len(self.home_servo_degrees) < len(self.joint_names):
            raise ValueError('home_servo_degrees needs one value per joint')
        self.home_sent_at = None
        self.home_started_at = None
        # Stab: after reaching the hover pose (target_offset_z above the fruit),
        # plan a second motion straight down with OMPL and stream it to the
        # servos. Planning goes through /plan_kinematic_path, a separate
        # capability from trajectory execution -- move_group runs with
        # allow_trajectory_execution false, so MoveIt plans but never drives the
        # arm; this node still owns every servo command that is sent.
        self.stab_enabled = bool(param('stab_enabled', True))
        self.stab_distance_m = float(param('stab_distance_m', 0.08))
        # Pilz LIN: a straight line in CARTESIAN space. OMPL plans in joint
        # space and only pins the endpoints, so the fork tip traced an
        # uncontrolled curve between them -- and a different one each run,
        # since RRTConnect is randomised. A stab has to go straight down.
        self.stab_pipeline_id = param('stab_pipeline_id', 'pilz_industrial_motion_planner')
        self.stab_planner_id = param('stab_planner_id', 'LIN')
        # Fallback. LIN refuses any path it cannot hold straight; OMPL will
        # route around the same obstacle with a curve. Falling back trades the
        # straight descent for getting there at all.
        # NOTE: Pilz LIN is analytic and deterministic -- an identical request
        # returns an identical failure -- so retries beyond the first only
        # burn planning calls. stab_lin_attempts:=1 falls back immediately.
        self.stab_lin_attempts = max(1, int(param('stab_lin_attempts', 10)))
        self.stab_fallback_enabled = bool(param('stab_fallback_enabled', True))
        self.stab_fallback_pipeline_id = param('stab_fallback_pipeline_id', 'ompl')
        self.stab_fallback_planner_id = param(
            'stab_fallback_planner_id', 'RRTConnectkConfigDefault')
        self.stab_planning_time = float(param('stab_planning_time', 5.0))
        # Playback pacing: the trajectory's own time_from_start is scaled for a
        # controller running at hundreds of hertz, but the ESP32 gets one
        # datagram per waypoint, so the descent is stretched over this instead.
        self.stab_duration_seconds = float(param('stab_duration_seconds', 2.0))
        if self.stab_distance_m < 0:
            raise ValueError('stab_distance_m must not be negative')
        if self.stab_duration_seconds <= 0 or self.stab_planning_time <= 0:
            raise ValueError('stab_duration_seconds and stab_planning_time must be positive')
        if (len(self.servo_offsets) != len(self.joint_names) or
                len(self.servo_directions) != len(self.joint_names)):
            raise ValueError('servo_offsets_deg and servo_directions need one value per joint')

        self.position_only = bool(param('position_only', False))
        self.orientation_attempts = max(1, int(param('orientation_attempts', 40)))
        self.ik_attempt = 0
        if not np.isfinite(self.orientation).all() or np.linalg.norm(self.orientation) < 1e-9:
            raise ValueError('Target orientation must be a finite nonzero quaternion')
        self.orientation /= np.linalg.norm(self.orientation)
        if len(self.seed) != len(self.joint_names) or not np.isfinite(self.seed).all():
            raise ValueError('IK seed must contain one finite angle for each joint')
        if not np.isfinite([self.plane_z, self.target_offset_z, self.timeout, self.ik_timeout]).all():
            raise ValueError('Plane, offset, and timeouts must be finite')
        if min(self.timeout, self.ik_timeout) <= 0:
            raise ValueError('Timeouts must be positive')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(PoseStamped, '/computer_vision/fruit_pose', latched)
        self.solution_pub = self.create_publisher(String, '/computer_vision/ik_solution', latched)
        self.status_pub = self.create_publisher(String, '/computer_vision/pipeline_result', latched)
        self.command_pub = self.create_publisher(Float64MultiArray, '/esp32/joint_commands', 10)
        self.create_subscription(String, '/computer_vision/detections', self.on_detections, latched)
        self.create_subscription(String, '/computer_vision/detection_status', self.on_detection_status, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, rclpy.qos.qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # OMPL planning for the stab descent. MoveGroupPlanService is not in the
        # launch's disable_capabilities list, so this is reachable even though
        # the execution capabilities are switched off.
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.motion_phase = 'HOVER'
        self.hover_radians = None
        self.stab_radians = {}
        self.stab_points = []
        self.stab_index = 0
        self.stab_interval = 0.0
        self.stab_next_time = 0.0
        # Which planner the next request uses; swapped on fallback.
        self.stab_active_pipeline = self.stab_pipeline_id
        self.stab_active_planner = self.stab_planner_id
        self.stab_plan_attempt = 0
        self.stab_using_fallback = False
        self.live_joints = {}
        self.stage = 'WAITING_FOR_DETECTION'
        self.started = time.monotonic()
        self.waiting_phase = 'CAPTURE'
        self.result = {}
        self.future = None
        self.home_pending = self.home_on_start and self.execute_on_hardware
        self.timer = self.create_timer(0.1, self.tick)
        self.record('STARTUP', 'READY', 'Waiting for one capture and its detections.', {
            'transform_source': self.transform_source,
            'plane_frame': self.base_frame, 'plane_z_m': self.plane_z,
            'target_offset_z_m': self.target_offset_z, 'target_link': self.tip,
            'target_quaternion_xyzw': self.orientation.tolist(), 'execution_enabled': False})

    def record(self, phase, status, detail, data=None):
        phase_log(self.log_path, phase, status, detail, data)
        if status == 'ERROR':
            self.get_logger().error(f'{phase}: {status}: {detail}')
        else:
            self.get_logger().info(f'{phase}: {status}: {detail}')

    def publish_result(self, status, detail=''):
        report = dict(self.result, status=status, stage=self.stage, detail=detail)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        target = self.output_directory / 'pipeline_result.json'
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temporary.replace(target)
        self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))

    def fail(self, detail):
        phase = self.waiting_phase if self.stage == 'WAITING_FOR_DETECTION' else self.stage
        self.record(phase, 'ERROR', detail)
        self.publish_result('error', detail)
        self.stage = 'DONE'

    def on_detection_status(self, message):
        if self.stage != 'WAITING_FOR_DETECTION':
            return
        try:
            event = json.loads(message.data)
        except ValueError:
            return
        if event.get('status') == 'inferencing':
            self.waiting_phase = 'DETECTION'
            self.started = time.monotonic()
        if event.get('status') == 'error':
            self.fail('Capture/detection failed: ' + event.get('detail', 'Unknown error'))

    def on_joints(self, message):
        now = time.monotonic()
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self.live_joints[name] = (float(position), now)

    def on_detections(self, message):
        if self.stage != 'WAITING_FOR_DETECTION':
            return
        self.stage = 'SELECTION'
        try:
            result = json.loads(message.data)
            image_path = Path(result['image_path']).resolve()
            # Reject a latched result from another launch or an older capture directory.
            if image_path.parent != self.output_directory:
                self.stage = 'WAITING_FOR_DETECTION'
                return
            detections = result['detections']
            if not detections:
                raise ValueError('No detections passed the confidence threshold')
            if any(not math.isfinite(float(d['confidence'])) for d in detections):
                raise ValueError('Detection confidence is not finite')
            self.detection = max(detections, key=lambda d: float(d['confidence']))
            self.metadata_path = image_path.parent / 'latest_capture.yaml'
            self.metadata = yaml.safe_load(self.metadata_path.read_text())
            size = result.get('image', {})
            for dim in ('width', 'height'):
                if int(size.get(dim, 0)) != int(self.metadata['rgb'][dim]):
                    raise ValueError('Detection dimensions do not match the captured RGB image')
            self.camera_frame = self.metadata['capture']['rgb_frame_id']
            if not self.camera_frame:
                raise ValueError('Capture has no optical frame ID')
            stamp = self.metadata['capture']['rgb_stamp']
            self.capture_time = rclpy.time.Time(seconds=int(stamp[0]), nanoseconds=int(stamp[1]))
            self.result = {'selected_detection': self.detection, 'plane_z_m': self.plane_z,
                           'base_frame': self.base_frame, 'camera_frame': self.camera_frame,
                           'image_path': str(image_path)}
            self.record('SELECTION', 'SUCCESS', 'Highest-confidence detection; passing box centre to projection.', self.detection)
            self.stage = 'TRANSFORM'
            self.started = time.monotonic()
        except Exception as error:
            self.fail(str(error))

    def home_arm(self):
        """Drive every servo to the home pose once, before the pipeline runs.

        Returns True when homing is finished (or was skipped), False while it
        is still in progress. Nothing downstream advances until it returns True.
        """
        if self.home_started_at is None:
            self.home_started_at = time.monotonic()
        if self.home_sent_at is None:
            if self.command_pub.get_subscription_count() == 0:
                if time.monotonic() - self.home_started_at < self.home_wait_seconds:
                    return False
                self.record('HOME', 'ERROR',
                            f'No subscriber on /esp32/joint_commands after '
                            f'{self.home_wait_seconds:.0f} s; the arm was NOT homed. '
                            'Continuing, but it starts from wherever it was left.')
                self.home_pending = False
                self.started = time.monotonic()
                return True
            message = Float64MultiArray(
                data=[float(v) for v in self.home_servo_degrees[:len(self.joint_names)]])
            self.command_pub.publish(message)
            # Republish once: a single UDP datagram to the ESP32 can be lost.
            self.command_pub.publish(message)
            self.home_sent_at = time.monotonic()
            self.record('HOME', 'SENT',
                        f'Homing: servo angles published to /esp32/joint_commands, '
                        f'settling for {self.home_settle_seconds:.0f} s before the '
                        'pipeline runs. The ESP32 echoes commands rather than '
                        'measuring them, so this is what was sent, not confirmation '
                        'the arm arrived.',
                        {'servo_degrees': dict(zip(
                            self.joint_names, self.home_servo_degrees))})
            return False
        if time.monotonic() - self.home_sent_at < self.home_settle_seconds:
            return False
        self.home_pending = False
        # Restart the stage clock: the homing wait must not count against
        # whatever stage the pipeline is in when it resumes.
        self.started = time.monotonic()
        self.record('HOME', 'SUCCESS', 'Arm commanded home; starting the pipeline.')
        return True

    def tick(self):
        if self.stage == 'DONE':
            return
        # Before anything else, and before the stage timeout is judged.
        if self.home_pending:
            if not self.home_arm():
                return
        if time.monotonic() - self.started > self.timeout:
            if self.future is not None:
                self.future.cancel()
            self.fail('Timed out in this phase; check camera/API, TF, or /compute_ik availability')
            return
        if self.stage == 'TRANSFORM':
            try:
                if self.transform_source == 'recorded':
                    saved = self.metadata['transform_camera_to_base']
                    if (saved.get('parent_frame') != self.base_frame or
                            saved.get('child_frame') != self.camera_frame):
                        raise ValueError('Recorded transform frame names do not match this image and base')
                    translation, quaternion = saved['translation_m'], saved['quaternion_xyzw']
                else:
                    # Nonblocking: listener must keep receiving TF while waiting.
                    if not self.tf_buffer.can_transform(self.base_frame, self.camera_frame, self.capture_time):
                        # Say so once, rather than spinning silently until the
                        # stage timeout: a missing frame and a slow one look
                        # identical on any single tick.
                        if not getattr(self, '_tf_wait_logged', False) and \
                                time.monotonic() - self.started > 3.0:
                            self._tf_wait_logged = True
                            self.record('TRANSFORM', 'WAITING',
                                        f'No transform {self.base_frame} -> '
                                        f'{self.camera_frame} at the capture stamp yet. '
                                        'Check that both camera TF publishers are '
                                        'running; giving up at stage_timeout.')
                        return
                    tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, self.capture_time)
                    t, q = tf.transform.translation, tf.transform.rotation
                    translation, quaternion = [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
                geometry = project_box_to_plane(self.detection, self.metadata, translation, quaternion, self.plane_z)
                self.result.update(geometry)
                self.result['transform_camera_to_base'] = {
                    'parent_frame': self.base_frame, 'child_frame': self.camera_frame,
                    'translation_m': translation, 'quaternion_xyzw': quaternion}
                self.result['transform_source'] = self.transform_source
                # A live capture becomes a self-contained replayable image/calibration pair.
                self.metadata['transform_camera_to_base'] = self.result['transform_camera_to_base']
                temporary = self.metadata_path.with_suffix('.yaml.tmp')
                temporary.write_text(yaml.safe_dump(self.metadata, sort_keys=False), encoding='utf-8')
                temporary.replace(self.metadata_path)
                self.record('TRANSFORM', 'SUCCESS',
                            'Camera optical point projected onto the known plane, then transformed into the arm base (metres).',
                            dict(geometry, transform=self.result['transform_camera_to_base'], intrinsics=self.metadata['camera_intrinsics']))
                xyz = geometry['point_base_m']
                if self.override_point_base:
                    projected = list(xyz)
                    xyz = list(self.override_point_base)
                    self.result['point_base_m'] = xyz
                    self.result['point_base_m_projected'] = projected
                    self.result['point_base_m_source'] = 'override_point_base'
                    self.record('TRANSFORM', 'OVERRIDE',
                                'Target point replaced by override_point_base; the '
                                'projected point below was NOT used. Detection and '
                                'everything downstream ran normally.',
                                {'used_point_base_m': xyz,
                                 'projected_point_base_m': projected})
                self.pose = PoseStamped()
                self.pose.header.frame_id = self.base_frame
                self.pose.header.stamp = self.get_clock().now().to_msg()
                self.pose.pose.position.x, self.pose.pose.position.y = xyz[:2]
                self.pose.pose.position.z = xyz[2] + self.target_offset_z
                (self.pose.pose.orientation.x, self.pose.pose.orientation.y,
                 self.pose.pose.orientation.z, self.pose.pose.orientation.w) = self.orientation.tolist()
                self.result['target_pose'] = {'frame': self.base_frame, 'link': self.tip,
                    'position_m': [xyz[0], xyz[1], self.pose.pose.position.z],
                    'quaternion_xyzw': self.orientation.tolist()}
                self.pose_pub.publish(self.pose)
                self.record('POSE', 'SUCCESS', 'Published /computer_vision/fruit_pose; passing pose to MoveIt IK.', self.result['target_pose'])
                self.publish_result('pose_ready')
                self.stage = 'IK_SERVICE'
                self.started = time.monotonic()
            except Exception as error:
                self.fail(str(error))
        elif self.stage == 'IK_SERVICE' and self.ik_client.service_is_ready():
            self.request_ik()
        elif self.stage == 'IK_RESULT' and self.future.done():
            self.finish_ik()
        elif self.stage == 'STAB_PLAN_SERVICE' and self.plan_client.service_is_ready():
            try:
                self.request_plan()
            except Exception as error:
                self.fail(str(error))
        elif self.stage == 'STAB_PLAN_RESULT' and self.future.done():
            try:
                self.finish_plan()
            except Exception as error:
                self.fail(str(error))
        elif self.stage == 'STAB_STREAM':
            try:
                self.stream_stab()
            except Exception as error:
                self.fail(str(error))

    @staticmethod
    def _quaternion_multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array([
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz])

    def _candidate_orientation(self, attempt):
        """Orientation to try on this attempt, as xyzw.

        Attempt 0 is always the configured orientation, so an explicitly
        requested wrist angle is honoured whenever it is reachable. After
        that, orientations are sampled on shells of increasing angular
        distance from it: several directions at each radius before the radius
        grows. Sweeping directions at a small radius matters -- the reachable
        wrist angles are usually only ten or twenty degrees away, and a search
        that simply widens monotonically walks straight past them.
        """
        base = np.array(self.orientation, dtype=float)
        base = base / np.linalg.norm(base)
        if attempt == 0 or not self.position_only:
            return base

        directions = 8
        index = attempt - 1
        shell, direction = divmod(index, directions)
        radii_deg = (15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0, 180.0)
        angle = math.radians(radii_deg[min(shell, len(radii_deg) - 1)])

        # Spread the directions of each shell over a sphere.
        z = 1.0 - 2.0 * (direction + 0.5) / directions
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        phi = (direction + shell * 0.5) * math.pi * (3.0 - math.sqrt(5.0))
        axis = np.array([radius * math.cos(phi), radius * math.sin(phi), z])
        axis /= np.linalg.norm(axis)

        half = angle / 2.0
        delta = np.array([*(axis * math.sin(half)), math.cos(half)])
        result = self._quaternion_multiply(base, delta)
        return result / np.linalg.norm(result)

    def request_ik(self):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group
        ik.ik_link_name = self.tip
        pose = copy.deepcopy(self.pose)
        candidate = self._candidate_orientation(self.ik_attempt)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = [float(v) for v in candidate]
        ik.pose_stamped = pose
        ik.avoid_collisions = self.avoid_collisions
        ik.timeout.sec = int(self.ik_timeout)
        ik.timeout.nanosec = int((self.ik_timeout % 1) * 1e9)
        live = all(name in self.live_joints and time.monotonic() - self.live_joints[name][1] < 2.0
                   for name in self.joint_names)
        seed = [self.live_joints[n][0] for n in self.joint_names] if live else self.seed
        ik.robot_state.joint_state.name = self.joint_names
        ik.robot_state.joint_state.position = seed
        ik.robot_state.is_diff = True

        # Tell the solver about the servo range up front, as well as checking
        # its answer afterwards. Expressed as centre +/- half-span, which is
        # what JointConstraint takes.
        limits = {}
        constraints = Constraints()
        for index, name in enumerate(self.joint_names):
            if name not in self.ik_servo_range_joints:
                continue
            direction = self.servo_directions[index]
            edges = sorted(((self.servo_min_deg - self.servo_offsets[index]) / direction,
                            (self.servo_max_deg - self.servo_offsets[index]) / direction))
            centre, half = (edges[0] + edges[1]) / 2.0, (edges[1] - edges[0]) / 2.0
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = math.radians(centre)
            constraint.tolerance_above = constraint.tolerance_below = math.radians(half)
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
            limits[name] = f'{edges[0]:+.2f} .. {edges[1]:+.2f} deg '\
                           f'(servo {self.servo_min_deg:.0f}-{self.servo_max_deg:.0f})'
        if constraints.joint_constraints:
            ik.constraints = constraints

        self.record('IK_REQUEST', 'STARTED', 'Requesting joint angles only; no trajectory execution.', {
            'group': self.group, 'link': self.tip, 'avoid_collisions': self.avoid_collisions,
            'position_only': self.position_only,
            'servo_range_constrained': limits,
            'orientation_attempt': f'{self.ik_attempt + 1} of '
                                   f'{self.orientation_attempts if self.position_only else 1}',
            'orientation_xyzw': [float(v) for v in self._candidate_orientation(self.ik_attempt)],
            'seed_source': 'recent joint_states' if live else 'configured SRDF home seed (not measured)',
            'seed_radians': dict(zip(self.joint_names, seed)), 'pose': self.result['target_pose']})
        self.future = self.ik_client.call_async(request)
        self.stage = 'IK_RESULT'
        self.started = time.monotonic()

    def send_to_hardware(self, radians_by_joint, quiet=False):
        """Convert the IK solution to servo degrees and publish it.

        Returns the servo angles on success, or None if anything was out of
        range -- in which case nothing at all is published, rather than
        sending a partial pose that would leave the arm in a shape no one
        asked for. Joints listed in disabled_joints are exempt: they get NaN
        (the firmware's "hold" value) and are not range-checked.
        """
        servo = []
        for index, name in enumerate(self.joint_names):
            if name in self.disabled_joints:
                servo.append(float('nan'))
                continue
            degrees = math.degrees(radians_by_joint[name])
            value = self.servo_offsets[index] + self.servo_directions[index] * degrees
            servo.append(value)

        unsafe = {self.joint_names[i]: round(v, 2) for i, v in enumerate(servo)
                  if self.joint_names[i] not in self.disabled_joints and
                  (not math.isfinite(v) or not self.servo_min_deg <= v <= self.servo_max_deg)}
        if unsafe:
            self.record('EXECUTE', 'ERROR',
                        f'Refusing to send: servo angles outside '
                        f'{self.servo_min_deg}-{self.servo_max_deg} deg.', unsafe)
            return None

        if self.command_pub.get_subscription_count() == 0:
            self.record('EXECUTE', 'ERROR',
                        'No subscriber on /esp32/joint_commands; the ESP32 is not '
                        'connected, so nothing was sent.')
            return None

        message = Float64MultiArray(data=[float(v) for v in servo])
        self.command_pub.publish(message)
        # Republish once: a single UDP datagram to the ESP32 can be lost.
        self.command_pub.publish(message)
        held = ', '.join(self.disabled_joints) or 'none'
        if quiet:
            return servo
        self.record('EXECUTE', 'SENT',
                    'Servo angles published to /esp32/joint_commands. The ESP32 echoes '
                    'commands rather than measuring them, so this is what was sent, not '
                    'confirmation the arm arrived. '
                    f'Sent NaN (hold, no servo) for: {held}.',
                    # null rather than NaN: the phase log is strict JSON.
                    {'servo_degrees': dict(zip(
                        self.joint_names,
                        [None if not math.isfinite(v) else round(v, 2) for v in servo])),
                     'held_joints': self.disabled_joints})
        return servo

    def servo_range_violations(self, radians_by_joint):
        """Joints in ik_servo_range_joints whose angle leaves the servo range.

        joint_6 is continuous in the URDF, so KDL returns wrist angles no servo
        can produce (-152 deg -> servo -62). Those solutions are geometrically
        valid and physically unbuildable, so finish_ik rejects them.
        """
        bad = {}
        for index, name in enumerate(self.joint_names):
            if name not in self.ik_servo_range_joints:
                continue
            degrees = math.degrees(radians_by_joint[name])
            servo = self.servo_offsets[index] + self.servo_directions[index] * degrees
            if not math.isfinite(servo) or not self.servo_min_deg <= servo <= self.servo_max_deg:
                bad[name] = {'joint_degrees': round(degrees, 3),
                             'servo_degrees': round(servo, 2) if math.isfinite(servo) else None}
        return bad

    # ------------------------------------------------------------ stab motion
    def publish_for_rviz(self, radians_by_joint):
        """Push one configuration to /computer_vision/ik_solution.

        solution_joint_state_node drives /joint_states from this topic. Without
        a publish per phase it would only ever see the end-of-run result, so
        RViz would jump straight to the final pose while the hardware was
        actually doing hover -> settle -> descend.
        """
        self.solution_pub.publish(String(data=json.dumps(
            {'joint_angles_radians': {n: float(v) for n, v in radians_by_joint.items()}})))

    def begin_stab(self):
        """Re-aim IK at a pose stab_distance_m below the hover pose."""
        # The stab solve overwrites joint_angles_*, so snapshot the hover
        # answer under its own keys before it is lost.
        self.result['hover_joint_angles_radians'] = dict(self.hover_radians)
        self.result['hover_joint_angles_degrees'] = {
            n: math.degrees(v) for n, v in self.hover_radians.items()}
        self.result['hover_servo_degrees_sent'] = self.result.get('servo_degrees_sent')
        self.motion_phase = 'STAB'
        self.ik_attempt = 0
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.z -= self.stab_distance_m
        self.result['stab_target_pose'] = {
            'frame': self.base_frame, 'link': self.tip,
            'position_m': [self.pose.pose.position.x, self.pose.pose.position.y,
                           self.pose.pose.position.z],
            'quaternion_xyzw': self.orientation.tolist()}
        self.pose_pub.publish(self.pose)
        self.record('STAB_POSE', 'SUCCESS',
                    f'Descending {self.stab_distance_m * 100:.1f} cm from the hover pose; '
                    'solving IK for the stab target.', self.result['stab_target_pose'])
        self.stage = 'IK_SERVICE'
        self.started = time.monotonic()

    def request_plan(self):
        """Ask OMPL for a path from the hover configuration to the stab one."""
        request = GetMotionPlan.Request()
        plan = MotionPlanRequest()
        plan.group_name = self.group
        plan.pipeline_id = self.stab_active_pipeline
        plan.planner_id = self.stab_active_planner
        # Pilz LIN is deterministic: one attempt either works or it does not.
        # A sampling planner benefits from several.
        plan.num_planning_attempts = 10 if self.stab_using_fallback else 1
        plan.allowed_planning_time = self.stab_planning_time
        plan.max_velocity_scaling_factor = 0.1
        plan.max_acceleration_scaling_factor = 0.1

        workspace = WorkspaceParameters()
        workspace.header.frame_id = self.base_frame
        workspace.min_corner.x = workspace.min_corner.y = workspace.min_corner.z = -1.0
        workspace.max_corner.x = workspace.max_corner.y = workspace.max_corner.z = 1.0
        plan.workspace_parameters = workspace

        # Start from the hover solution rather than /joint_states: the ESP32
        # never reports back, so the only thing known about the arm is the
        # command just sent to it.
        start = RobotState()
        start.joint_state.name = list(self.joint_names)
        start.joint_state.position = [self.hover_radians[n] for n in self.joint_names]
        start.is_diff = False
        plan.start_state = start

        goal = Constraints()
        for name in self.joint_names:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(self.stab_radians[name])
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        plan.goal_constraints = [goal]

        request.motion_plan_request = plan
        self.record('STAB_PLAN', 'STARTED',
                    f'Planning the descent with {self.stab_active_pipeline}/'
                    f'{self.stab_active_planner} via /plan_kinematic_path '
                    f'(attempt {self.stab_plan_attempt + 1}'
                    f'{"" if self.stab_using_fallback else f" of {self.stab_lin_attempts}"}). '
                    + ('FALLBACK: joint-space planning, so the descent is NOT '
                       'guaranteed straight.' if self.stab_using_fallback else
                       'LIN is a straight line in Cartesian space, so the fork tip '
                       'descends vertically rather than following a joint-space curve.')
                    + ' Planning only; MoveIt does not execute.',
                    {'group': self.group, 'pipeline_id': self.stab_active_pipeline,
                     'planner_id': self.stab_active_planner,
                     'is_fallback': self.stab_using_fallback,
                     'allowed_planning_time_s': self.stab_planning_time,
                     'start_radians': {n: self.hover_radians[n] for n in self.joint_names},
                     'goal_radians': {n: self.stab_radians[n] for n in self.joint_names}})
        self.future = self.plan_client.call_async(request)
        self.stage = 'STAB_PLAN_RESULT'
        self.started = time.monotonic()

    def finish_plan(self):
        response = self.future.result().motion_plan_response
        code = response.error_code.val
        name = next((k for k in dir(MoveItErrorCodes)
                     if k.isupper() and getattr(MoveItErrorCodes, k) == code), str(code))
        if code != MoveItErrorCodes.SUCCESS:
            self.stab_plan_attempt += 1
            planner = f'{self.stab_active_pipeline}/{self.stab_active_planner}'

            # Still have attempts left with the current planner.
            if self.stab_using_fallback or self.stab_plan_attempt < self.stab_lin_attempts:
                if not self.stab_using_fallback:
                    self.record('STAB_PLAN', 'RETRY',
                                f'{planner} failed: {name} ({code}). Retrying '
                                f'({self.stab_plan_attempt + 1} of '
                                f'{self.stab_lin_attempts}). Note LIN is deterministic, '
                                'so an identical request returns an identical result.')
                    self.stage = 'STAB_PLAN_SERVICE'
                    self.started = time.monotonic()
                    return

            # LIN budget exhausted: switch to the joint-space fallback once.
            if not self.stab_using_fallback and self.stab_fallback_enabled:
                self.stab_using_fallback = True
                self.stab_active_pipeline = self.stab_fallback_pipeline_id
                self.stab_active_planner = self.stab_fallback_planner_id
                self.stab_plan_attempt = 0
                self.result['stab_fallback_used'] = True
                self.record('STAB_PLAN', 'FALLBACK',
                            f'{planner} failed {self.stab_lin_attempts} time(s): '
                            f'{name} ({code}). Falling back to '
                            f'{self.stab_active_pipeline}/{self.stab_active_planner}. '
                            'WARNING: that plans in joint space, so the descent will '
                            'NOT be a straight line -- the fork tip may curve between '
                            'the two poses.')
                self.stage = 'STAB_PLAN_SERVICE'
                self.started = time.monotonic()
                return

            raise RuntimeError(
                f'Stab planning failed: {name} ({code}) with {planner}'
                + ('' if self.stab_using_fallback else
                   ' and no fallback is enabled')
                + '. LIN refuses a path it cannot hold straight -- a singularity or '
                  'an unreachable point on the line -- rather than detouring around it.')

        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError('OMPL returned an empty trajectory for the stab')

        # Reorder into this node's joint order; the planner may use its own.
        order = [trajectory.joint_names.index(n) for n in self.joint_names]
        self.stab_points = [{n: float(point.positions[order[i]])
                             for i, n in enumerate(self.joint_names)}
                            for point in trajectory.points]
        self.stab_index = 0
        self.stab_interval = self.stab_duration_seconds / max(1, len(self.stab_points) - 1)
        self.stab_next_time = time.monotonic()
        self.result['stab_waypoints'] = len(self.stab_points)
        self.record('STAB_PLAN', 'SUCCESS',
                    f'{len(self.stab_points)} waypoints; streaming them over '
                    f'{self.stab_duration_seconds:.1f} s.',
                    {'waypoints': len(self.stab_points),
                     'pipeline_id': self.stab_active_pipeline,
                     'planner_id': self.stab_active_planner,
                     'is_fallback': self.stab_using_fallback,
                     'interval_s': round(self.stab_interval, 4)})
        self.stage = 'STAB_STREAM'
        self.started = time.monotonic()

    def stream_stab(self):
        """Publish one trajectory waypoint per interval."""
        if time.monotonic() < self.stab_next_time:
            return
        waypoint = self.stab_points[self.stab_index]
        servo = self.send_to_hardware(waypoint, quiet=True)
        if servo is None:
            raise RuntimeError(
                f'Stab waypoint {self.stab_index + 1} of {len(self.stab_points)} was '
                'outside the servo range; descent stopped part-way')
        self.publish_for_rviz(waypoint)
        self.stab_index += 1
        self.stab_next_time = time.monotonic() + self.stab_interval
        if self.stab_index >= len(self.stab_points):
            final = self.stab_points[-1]
            self.result['stab_joint_angles_radians'] = final
            self.result['stab_joint_angles_degrees'] = {
                n: math.degrees(v) for n, v in final.items()}
            self.result['stab_servo_degrees_sent'] = dict(zip(
                self.joint_names,
                [None if not math.isfinite(v) else v for v in servo]))
            self.record('STAB_EXECUTE', 'SENT',
                        f'Descent complete: {len(self.stab_points)} waypoints sent to '
                        '/esp32/joint_commands. The ESP32 echoes commands rather than '
                        'measuring them, so this is what was sent, not confirmation '
                        'the fork arrived.',
                        {'final_servo_degrees': self.result['stab_servo_degrees_sent'],
                         'final_joint_degrees': self.result['stab_joint_angles_degrees']})
            self.finish_pipeline('Hover reached and stab descent streamed to the arm.')

    def finish_pipeline(self, detail):
        self.solution_pub.publish(String(data=json.dumps(self.result)))
        self.publish_result('success')
        self.record('PIPELINE', 'COMPLETE', detail)
        self.stage = 'DONE'

    def finish_ik(self):
        try:
            response = self.future.result()
            code = response.error_code.val
            name = next((k for k in dir(MoveItErrorCodes) if k.isupper() and getattr(MoveItErrorCodes, k) == code), str(code))
            self.result['moveit_error_code'] = code
            if code != MoveItErrorCodes.SUCCESS:
                # With position_only, a rejected orientation is not a failure
                # yet: widen the wrist and try again before giving up.
                if self.position_only and self.ik_attempt + 1 < self.orientation_attempts:
                    self.ik_attempt += 1
                    self.record('IK_RESULT', 'RETRY',
                                f'{name} ({code}) with this wrist orientation; '
                                f'relaxing it and retrying '
                                f'({self.ik_attempt + 1} of {self.orientation_attempts}).')
                    self.stage = 'IK_SERVICE'
                    self.started = time.monotonic()
                    return
                raise RuntimeError(f'MoveIt IK returned {name} ({code}); target/orientation may be unreachable or in collision')
            state = response.solution.joint_state
            values = dict(zip(state.name, state.position))
            if any(n not in values or not math.isfinite(values[n]) for n in self.joint_names):
                raise ValueError('MoveIt returned an incomplete or non-finite joint solution')

            # Hard servo-range rule on the unbounded joints. MoveIt reported
            # SUCCESS, but a solution needing servo angles outside 0-180 cannot
            # be built on this hardware, so it counts as a failed solve and
            # takes the same retry path as NO_IK_SOLUTION.
            violations = self.servo_range_violations(values)
            if violations:
                summary = ', '.join(
                    f"{n} {v['joint_degrees']:+.2f} deg -> servo {v['servo_degrees']}"
                    for n, v in violations.items())
                if self.position_only and self.ik_attempt + 1 < self.orientation_attempts:
                    self.ik_attempt += 1
                    self.record('IK_RESULT', 'RETRY',
                                f'Solution rejected: {summary} is outside '
                                f'{self.servo_min_deg}-{self.servo_max_deg}; relaxing the '
                                f'wrist and retrying '
                                f'({self.ik_attempt + 1} of {self.orientation_attempts}).',
                                violations)
                    self.stage = 'IK_SERVICE'
                    self.started = time.monotonic()
                    return
                self.result['servo_range_violations'] = violations
                raise RuntimeError(
                    f'IK solution rejected: {summary} is outside the servo range '
                    f'{self.servo_min_deg}-{self.servo_max_deg} deg. Joints '
                    f'{", ".join(self.ik_servo_range_joints)} are unbounded in the URDF '
                    'but their servos are not, so this pose is not buildable.')
            self.result['joint_angles_radians'] = {n: values[n] for n in self.joint_names}
            self.result['joint_angles_degrees'] = {n: math.degrees(values[n]) for n in self.joint_names}
            used = self._candidate_orientation(self.ik_attempt)
            self.result['ik_orientation_xyzw'] = [float(v) for v in used]
            self.result['ik_orientation_attempts'] = self.ik_attempt + 1
            self.record('IK_RESULT', 'SUCCESS', 'MoveIt joint solution (ROS joint angles, not servo calibration values).', {
                'radians': self.result['joint_angles_radians'], 'degrees': self.result['joint_angles_degrees'],
                'orientation_xyzw': self.result['ik_orientation_xyzw'],
                'orientation_attempts_used': self.ik_attempt + 1,
                'note': ('wrist orientation was relaxed from the configured one'
                         if self.position_only and self.ik_attempt else
                         'configured wrist orientation was achievable')})
            if self.motion_phase == 'STAB':
                # Goal configuration for the descent. Nothing is sent yet: OMPL
                # plans the path from the hover pose to here first.
                self.stab_radians = dict(self.result['joint_angles_radians'])
                self.result['stab_ik_joint_angles_degrees'] = dict(
                    self.result['joint_angles_degrees'])
                self.stage = 'STAB_PLAN_SERVICE'
                self.started = time.monotonic()
                return

            sent = None
            if self.execute_on_hardware:
                servo = self.send_to_hardware(self.result['joint_angles_radians'])
                sent = servo is not None
                # null rather than NaN for held joints: strict JSON downstream.
                self.result['servo_degrees_sent'] = (
                    dict(zip(self.joint_names,
                             [None if not math.isfinite(v) else v for v in servo]))
                    if servo is not None else None)
                self.result['held_joints'] = self.disabled_joints

            if sent:
                self.hover_radians = dict(self.result['joint_angles_radians'])
                self.publish_for_rviz(self.hover_radians)
                if self.stab_enabled and self.stab_distance_m > 0:
                    # Let the arm settle at the hover pose before descending.
                    time.sleep(self.command_settle_seconds)
                    self.begin_stab()
                    return
                self.finish_pipeline('Servo angles sent to the arm.')
                return

            if sent is None:
                detail = 'Pose and joint solution saved and published. No motion requested.'
                if self.stab_enabled:
                    detail += (' Stab skipped: it needs execute_on_hardware, since the '
                               'descent is a second real motion.')
            else:
                # Do not claim success here: send_to_hardware already recorded
                # the specific EXECUTE error, and nothing was published.
                detail = ('Pose and joint solution saved, but NOTHING was sent to the '
                          'arm -- see the EXECUTE error above. Stab skipped.')
            self.finish_pipeline(detail)
        except Exception as error:
            self.fail(str(error))


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.stage != 'DONE':
            phase_log(node.log_path, node.stage, 'INTERRUPTED', 'Launch stopped before this phase completed.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
