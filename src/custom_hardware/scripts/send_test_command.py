#!/usr/bin/env python3
"""
Simple test command sender for the hardware interface.

This script provides easy commands to test the hardware interface
without needing to use complex ROS 2 CLI commands.

Usage:
    python3 send_test_command.py home      # Move to home position
    python3 send_test_command.py wave      # Perform a wave motion
    python3 send_test_command.py test      # Small test movement
    python3 send_test_command.py pos 0.5 0.3 0.2 0.1 0.0 0.0  # Custom positions

:author: Jamal
:date: February 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import sys
import time
import math


class CommandSender(Node):
    """Simple command sender for testing."""

    def __init__(self):
        super().__init__('command_sender')
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Command sender initialized')
    
    def send_trajectory(self, waypoints: list, durations: list):
        """Send a multi-point trajectory."""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        total_time = 0.0
        for positions, duration in zip(waypoints, durations):
            total_time += duration
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(seconds=total_time).to_msg()
            goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f'Sending trajectory with {len(waypoints)} waypoints...')
        
        future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, executing...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=total_time + 10.0)
        
        self.get_logger().info('Trajectory complete!')
        return True
    
    def go_home(self):
        """Move to home position (all zeros)."""
        self.get_logger().info('Moving to HOME position...')
        waypoints = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        durations = [3.0]
        return self.send_trajectory(waypoints, durations)
    
    def test_movement(self):
        """Small test movement."""
        self.get_logger().info('Performing TEST movement...')
        waypoints = [
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],   # Move joint 1
            [0.2, 0.2, 0.0, 0.0, 0.0, 0.0],   # Move joint 2
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # Return home
        ]
        durations = [2.0, 2.0, 2.0]
        return self.send_trajectory(waypoints, durations)
    
    def wave_motion(self):
        """Perform a wave motion."""
        self.get_logger().info('Performing WAVE motion...')
        waypoints = []
        durations = []
        
        # Generate wave pattern
        for i in range(5):
            angle = 0.3 * math.sin(i * math.pi / 2)
            waypoints.append([0.0, 0.0, 0.0, 0.0, angle, -angle])
            durations.append(0.5)
        
        # Return to home
        waypoints.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        durations.append(1.0)
        
        return self.send_trajectory(waypoints, durations)
    
    def custom_position(self, positions: list):
        """Move to custom position."""
        self.get_logger().info(f'Moving to custom position: {positions}')
        waypoints = [positions]
        durations = [3.0]
        return self.send_trajectory(waypoints, durations)


def print_usage():
    """Print usage information."""
    print("""
╔════════════════════════════════════════════════════════════════╗
║          CUSTOM HARDWARE INTERFACE TEST COMMANDS               ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  Usage: python3 send_test_command.py <command> [args]          ║
║                                                                ║
║  Commands:                                                     ║
║    home    - Move all joints to zero position                  ║
║    test    - Perform small test movement                       ║
║    wave    - Perform a wave motion pattern                     ║
║    pos <j1> <j2> <j3> <j4> <j5> <j6>                          ║
║            - Move to custom position (in radians)              ║
║                                                                ║
║  Examples:                                                     ║
║    python3 send_test_command.py home                           ║
║    python3 send_test_command.py test                           ║
║    python3 send_test_command.py pos 0.5 0.3 0.2 0.1 0.0 0.0   ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
""")


def main():
    if len(sys.argv) < 2:
        print_usage()
        return
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    sender = CommandSender()
    
    try:
        time.sleep(1.0)  # Wait for connections
        
        if command == 'home':
            sender.go_home()
        elif command == 'test':
            sender.test_movement()
        elif command == 'wave':
            sender.wave_motion()
        elif command == 'pos':
            if len(sys.argv) < 8:
                print("Error: 'pos' command requires 6 joint values")
                print("Example: python3 send_test_command.py pos 0.5 0.3 0.2 0.1 0.0 0.0")
                return
            positions = [float(sys.argv[i]) for i in range(2, 8)]
            sender.custom_position(positions)
        else:
            print(f"Unknown command: {command}")
            print_usage()
    
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
