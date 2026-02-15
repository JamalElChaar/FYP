#!/usr/bin/env python3
"""
Simple test script to move joints 1 and 2 of the robot arm.
Publishes to the JointTrajectory controller.

Usage:
    1. Source your workspace: source install/setup.bash
    2. Launch the robot: ros2 launch robot_arm_bringup real_robot.launch.py
    3. Run this script: python3 test_joint_movement.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover_test')
        
        # Action client for the arm controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Joint names (must match your URDF)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        self.get_logger().info('JointMover initialized')
        self.get_logger().info('Waiting for action server...')
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            raise RuntimeError('Action server not available')
        
        self.get_logger().info('Action server connected!')

    def send_joint_positions(self, positions, duration_sec=2.0):
        """
        Send joint positions to the arm.
        
        Args:
            positions: List of 6 joint angles in radians
            duration_sec: Time to reach the position
        """
        if len(positions) != 6:
            self.get_logger().error(f'Expected 6 positions, got {len(positions)}')
            return False
        
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.get_logger().info(f'Sending positions: {[f"{p:.2f}" for p in positions]}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Goal failed with status: {result.status}')
            return False


def degrees_to_radians(degrees):
    return degrees * math.pi / 180.0


def main():
    rclpy.init()
    
    try:
        mover = JointMover()
        
        print("\n" + "="*50)
        print("Robot Arm Joint Movement Test")
        print("="*50)
        print("\nThis will move joints 1 and 2.")
        print("Other joints will stay at their home position (0 rad).")
        print("\nPress Enter to start, or Ctrl+C to cancel...")
        input()
        
        # Test sequence - all angles in radians
        # Home position is 0 radians (corresponds to 90° on servos)
        
        print("\n--- Step 1: Go to home position (all joints at 0) ---")
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(home_position, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 2: Move joint 1 to +30 degrees ---")
        # Joint 1 moves, others stay at home
        pos_j1_plus = [degrees_to_radians(30), 0.0, 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(pos_j1_plus, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 3: Move joint 1 to -30 degrees ---")
        pos_j1_minus = [degrees_to_radians(-30), 0.0, 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(pos_j1_minus, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 4: Joint 1 to 0, Joint 2 to +30 degrees ---")
        pos_j2_plus = [0.0, degrees_to_radians(30), 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(pos_j2_plus, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 5: Joint 2 to -30 degrees ---")
        pos_j2_minus = [0.0, degrees_to_radians(-30), 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(pos_j2_minus, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 6: Move both joints 1 and 2 together ---")
        pos_both = [degrees_to_radians(20), degrees_to_radians(20), 0.0, 0.0, 0.0, 0.0]
        mover.send_joint_positions(pos_both, duration_sec=2.0)
        time.sleep(1)
        
        print("\n--- Step 7: Return to home position ---")
        mover.send_joint_positions(home_position, duration_sec=2.0)
        
        print("\n" + "="*50)
        print("Test completed successfully!")
        print("="*50 + "\n")
        
    except KeyboardInterrupt:
        print("\nTest cancelled by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
