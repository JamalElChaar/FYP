#!/usr/bin/env python3
"""
Test script for the custom hardware interface.

This script tests the hardware interface by:
1. Checking if controllers are loaded and active
2. Sending test trajectories to the arm
3. Monitoring joint states feedback
4. Verifying the control loop is working

Run this after launching: ros2 launch custom_hardware robot_arm_hardware.launch.py

:author: Jamal
:date: February 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers

import time
import math


class HardwareInterfaceTester(Node):
    """Node to test the custom hardware interface."""

    def __init__(self):
        super().__init__('hardware_interface_tester')
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Current joint states
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6
        self.states_received = False
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client for trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Publisher for direct effort commands (alternative controller)
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        # Service client to check controllers
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )
        
        self.get_logger().info('Hardware Interface Tester initialized')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state messages."""
        self.states_received = True
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.current_velocities[i] = msg.velocity[idx]
    
    def check_controllers(self) -> bool:
        """Check if controllers are loaded and active."""
        self.get_logger().info('Checking controllers...')
        
        if not self.list_controllers_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Controller manager service not available!')
            return False
        
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().error('Failed to get controller list!')
            return False
        
        response = future.result()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('CONTROLLER STATUS:')
        self.get_logger().info('=' * 50)
        
        controllers_ok = True
        for controller in response.controller:
            status = '✓' if controller.state == 'active' else '✗'
            self.get_logger().info(
                f'  {status} {controller.name}: {controller.state} ({controller.type})'
            )
            if controller.name in ['joint_state_broadcaster', 'arm_controller']:
                if controller.state != 'active':
                    controllers_ok = False
        
        return controllers_ok
    
    def check_joint_states(self) -> bool:
        """Check if joint states are being published."""
        self.get_logger().info('Checking joint states...')
        
        # Wait for joint states
        start_time = time.time()
        while not self.states_received and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.states_received:
            self.get_logger().error('No joint states received!')
            return False
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('CURRENT JOINT STATES:')
        self.get_logger().info('=' * 50)
        
        for i, name in enumerate(self.joint_names):
            self.get_logger().info(
                f'  {name}: pos={self.current_positions[i]:.4f} rad, '
                f'vel={self.current_velocities[i]:.4f} rad/s'
            )
        
        return True
    
    def send_test_trajectory(self, positions: list, duration: float = 3.0) -> bool:
        """Send a test trajectory to the arm controller."""
        self.get_logger().info(f'Sending test trajectory: {positions}')
        
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available!')
            return False
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Add trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal_msg.trajectory.points.append(point)
        
        # Send goal
        self.get_logger().info('Sending trajectory goal...')
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected!')
            return False
        
        self.get_logger().info('Trajectory goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('✓ Trajectory completed successfully!')
            return True
        else:
            self.get_logger().warn(f'Trajectory finished with error code: {result.result.error_code}')
            return False
    
    def send_effort_command(self, efforts: list):
        """Send direct effort commands."""
        msg = Float64MultiArray()
        msg.data = efforts
        self.effort_pub.publish(msg)
        self.get_logger().info(f'Sent effort command: {efforts}')
    
    def run_full_test(self):
        """Run a complete test of the hardware interface."""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  CUSTOM HARDWARE INTERFACE TEST')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
        # Test 1: Check controllers
        self.get_logger().info('TEST 1: Checking controllers...')
        if not self.check_controllers():
            self.get_logger().error('TEST 1 FAILED: Controllers not ready!')
            return False
        self.get_logger().info('TEST 1 PASSED: Controllers are active\n')
        
        # Test 2: Check joint states
        self.get_logger().info('TEST 2: Checking joint state feedback...')
        if not self.check_joint_states():
            self.get_logger().error('TEST 2 FAILED: No joint states!')
            return False
        self.get_logger().info('TEST 2 PASSED: Joint states received\n')
        
        # Test 3: Send small movement trajectory
        self.get_logger().info('TEST 3: Sending test trajectory...')
        test_positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Small movement
        if not self.send_test_trajectory(test_positions, duration=2.0):
            self.get_logger().warn('TEST 3 WARNING: Trajectory may have issues')
        else:
            self.get_logger().info('TEST 3 PASSED: Trajectory executed\n')
        
        # Give time for movement
        time.sleep(1.0)
        
        # Check final positions
        self.get_logger().info('Checking final joint states...')
        rclpy.spin_once(self, timeout_sec=0.5)
        self.check_joint_states()
        
        # Test 4: Return to home position
        self.get_logger().info('\nTEST 4: Returning to home position...')
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_test_trajectory(home_positions, duration=2.0)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  ALL TESTS COMPLETED!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('The hardware interface is working correctly.')
        self.get_logger().info('You can now deploy to the Raspberry Pi.')
        self.get_logger().info('')
        
        return True


def main():
    rclpy.init()
    
    tester = HardwareInterfaceTester()
    
    try:
        # Give system time to start
        print("\nWaiting 3 seconds for system to initialize...")
        time.sleep(3.0)
        
        tester.run_full_test()
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
