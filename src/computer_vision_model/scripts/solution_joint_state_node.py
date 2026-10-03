#!/usr/bin/env python3
"""Drive /joint_states from the vision pipeline's IK solution, for RViz.

The pipeline deliberately does not execute trajectories -- it solves IK and
publishes the answer as JSON on /computer_vision/ik_solution. Nothing in that
launch publishes /joint_states, so robot_state_publisher has no joint angles
and RViz shows the arm collapsed at zeros.

This node fills that gap. It publishes /joint_states continuously, starting at
the configured seed (the SRDF "home" pose, servo 90 on every joint) and
switching to the IK solution when one arrives.

IMPORTANT: this is a VISUALISATION of the solved pose, not a simulation of the
motion. There is no trajectory, no interpolation and no collision checking --
the arm snaps from the seed to the solution in one step. It also says nothing
about where the real arm is: the ESP32 has never published joint feedback, so
matching RViz is not evidence the hardware arrived.

Joints listed in held_joints (those with no working servo) are shown at their
seed value rather than the solved value, so RViz matches what the hardware can
actually do.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class SolutionJointStateNode(Node):
    def __init__(self):
        super().__init__('solution_joint_state_node')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.joint_names = [str(n) for n in param(
            'joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])]
        # Servo 90 on every joint, through the calibrated offsets -- the same
        # seed camera_to_base_node uses and the SRDF "home" state.
        self.positions = [float(v) for v in param(
            'seed_radians', [0.0, -0.238, -0.373, 1.570796, 0.0, 0.0])]
        # Shown at the seed value even when the solution moves them, matching
        # the firmware, which ignores joints with no servo pin.
        #
        # The default is [''], not []: rclpy infers a parameter's type from its
        # default value, an empty list infers BYTE_ARRAY, and that then rejects
        # the STRING_ARRAY the launch passes (InvalidParameterTypeException).
        # A ParameterDescriptor does NOT override the inference. The empty
        # placeholder is filtered out below.
        self.held = [str(n) for n in (param('held_joints', ['']) or [])
                     if str(n).strip()]
        rate = float(param('publish_rate_hz', 20.0))

        if len(self.positions) != len(self.joint_names):
            raise ValueError('seed_radians needs one angle per joint')
        unknown = [n for n in self.held if n not in self.joint_names]
        if unknown:
            raise ValueError(f'held_joints names no such joint: {unknown}')

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        # The solution is latched, so a late subscriber still receives it.
        self.create_subscription(
            String, '/computer_vision/ik_solution', self.on_solution,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            'Publishing /joint_states at the seed pose; will switch to the IK '
            'solution when one is published. Held joints: '
            f'{", ".join(self.held) or "none"}')

    def on_solution(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().warning(f'Ignoring unparseable IK solution: {error}')
            return

        radians = payload.get('joint_angles_radians')
        if not isinstance(radians, dict):
            self.get_logger().warning(
                'IK solution carried no joint_angles_radians; leaving RViz at the seed')
            return

        updated, skipped = [], []
        for index, name in enumerate(self.joint_names):
            if name not in radians:
                continue
            if name in self.held:
                skipped.append(name)
                continue
            try:
                self.positions[index] = float(radians[name])
            except (TypeError, ValueError):
                self.get_logger().warning(f'Ignoring non-numeric angle for {name}')
                continue
            updated.append(name)

        self.get_logger().info(
            f'RViz moved to the IK solution for: {", ".join(updated) or "no joints"}'
            + (f' (held at seed: {", ".join(skipped)})' if skipped else ''))

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.joint_names
        message.position = self.positions
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SolutionJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
