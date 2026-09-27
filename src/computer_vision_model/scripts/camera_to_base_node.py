#!/usr/bin/env python3
"""Project the most confident detection onto a base-frame plane and request IK.

No trajectory execution or motor command publishers are created by this node.
"""
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
        # Existing SRDF home state; used only as an IK seed if live state is unavailable.
        self.seed = list(param('seed_positions', [0.0, 0.785398, -0.436332, 1.570796, -1.919862, 0.0]))
        self.orientation = np.array([float(param(k, v)) for k, v in
                                     zip(('ox', 'oy', 'oz', 'ow'), (-0.548, 0.447, 0.006, 0.707))])
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
        self.create_subscription(String, '/computer_vision/detections', self.on_detections, latched)
        self.create_subscription(String, '/computer_vision/detection_status', self.on_detection_status, latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, rclpy.qos.qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.live_joints = {}
        self.stage = 'WAITING_FOR_DETECTION'
        self.started = time.monotonic()
        self.waiting_phase = 'CAPTURE'
        self.result = {}
        self.future = None
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

    def tick(self):
        if self.stage == 'DONE':
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
                self.pose = PoseStamped()
                self.pose.header.frame_id = self.base_frame
                self.pose.header.stamp = self.get_clock().now().to_msg()
                xyz = geometry['point_base_m']
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

    def request_ik(self):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group
        ik.ik_link_name = self.tip
        ik.pose_stamped = self.pose
        ik.avoid_collisions = self.avoid_collisions
        ik.timeout.sec = int(self.ik_timeout)
        ik.timeout.nanosec = int((self.ik_timeout % 1) * 1e9)
        live = all(name in self.live_joints and time.monotonic() - self.live_joints[name][1] < 2.0
                   for name in self.joint_names)
        seed = [self.live_joints[n][0] for n in self.joint_names] if live else self.seed
        ik.robot_state.joint_state.name = self.joint_names
        ik.robot_state.joint_state.position = seed
        ik.robot_state.is_diff = True
        self.record('IK_REQUEST', 'STARTED', 'Requesting joint angles only; no trajectory execution.', {
            'group': self.group, 'link': self.tip, 'avoid_collisions': self.avoid_collisions,
            'seed_source': 'recent joint_states' if live else 'configured SRDF home seed (not measured)',
            'seed_radians': dict(zip(self.joint_names, seed)), 'pose': self.result['target_pose']})
        self.future = self.ik_client.call_async(request)
        self.stage = 'IK_RESULT'
        self.started = time.monotonic()

    def finish_ik(self):
        try:
            response = self.future.result()
            code = response.error_code.val
            name = next((k for k in dir(MoveItErrorCodes) if k.isupper() and getattr(MoveItErrorCodes, k) == code), str(code))
            self.result['moveit_error_code'] = code
            if code != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f'MoveIt IK returned {name} ({code}); target/orientation may be unreachable or in collision')
            state = response.solution.joint_state
            values = dict(zip(state.name, state.position))
            if any(n not in values or not math.isfinite(values[n]) for n in self.joint_names):
                raise ValueError('MoveIt returned an incomplete or non-finite joint solution')
            self.result['joint_angles_radians'] = {n: values[n] for n in self.joint_names}
            self.result['joint_angles_degrees'] = {n: math.degrees(values[n]) for n in self.joint_names}
            self.record('IK_RESULT', 'SUCCESS', 'MoveIt joint solution (ROS joint angles, not servo calibration values).', {
                'radians': self.result['joint_angles_radians'], 'degrees': self.result['joint_angles_degrees']})
            self.solution_pub.publish(String(data=json.dumps(self.result)))
            self.publish_result('success')
            self.record('PIPELINE', 'COMPLETE', 'Pose and joint solution saved and published. No motion requested.')
            self.stage = 'DONE'
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
