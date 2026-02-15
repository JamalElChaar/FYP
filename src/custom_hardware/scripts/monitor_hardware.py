#!/usr/bin/env python3
"""
Real-time hardware interface monitor.

This script provides a live view of what the hardware interface is doing,
showing commands being sent and states being read - exactly what would
happen on real hardware.

Run this in a separate terminal while testing.

:author: Jamal
:date: February 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import curses
import threading


class HardwareMonitor(Node):
    """Real-time monitor for hardware interface."""

    def __init__(self):
        super().__init__('hardware_monitor')
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # State storage
        self.positions = [0.0] * 6
        self.velocities = [0.0] * 6
        self.efforts = [0.0] * 6
        self.commands = [0.0] * 6
        
        self.state_count = 0
        self.command_count = 0
        self.last_state_time = None
        self.hz = 0.0
        
        # Subscribe to joint states (what hardware interface READS)
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10
        )
        
        # Subscribe to controller commands (what would be WRITTEN to hardware)
        # Note: The actual commands go through the hardware interface
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/arm_controller/joint_trajectory_controller/command',
            self.command_callback,
            10
        )
        
    def state_callback(self, msg: JointState):
        """Process joint state updates."""
        self.state_count += 1
        
        # Calculate update rate
        current_time = self.get_clock().now()
        if self.last_state_time is not None:
            dt = (current_time - self.last_state_time).nanoseconds / 1e9
            if dt > 0:
                self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt)  # Smoothed
        self.last_state_time = current_time
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.velocities[i] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self.efforts[i] = msg.effort[idx]
    
    def command_callback(self, msg: Float64MultiArray):
        """Process command updates."""
        self.command_count += 1
        for i, val in enumerate(msg.data[:6]):
            self.commands[i] = val


def run_monitor(stdscr, node):
    """Run the curses-based monitor display."""
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking input
    
    # Colors
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLUE)
    
    while True:
        # Check for quit
        try:
            key = stdscr.getch()
            if key == ord('q'):
                break
        except:
            pass
        
        # Spin ROS once
        rclpy.spin_once(node, timeout_sec=0.01)
        
        # Clear and draw
        stdscr.clear()
        
        # Header
        stdscr.attron(curses.color_pair(4))
        stdscr.addstr(0, 0, " " * 70)
        stdscr.addstr(0, 2, "CUSTOM HARDWARE INTERFACE MONITOR")
        stdscr.addstr(0, 50, f"Rate: {node.hz:.1f} Hz")
        stdscr.attroff(curses.color_pair(4))
        
        # Status line
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(2, 2, f"States received: {node.state_count}  |  Commands sent: {node.command_count}")
        stdscr.attroff(curses.color_pair(3))
        
        # Table header
        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(4, 2, "Joint      Position (rad)  Velocity (rad/s)  Effort (Nm)")
        stdscr.addstr(5, 2, "-" * 60)
        stdscr.attroff(curses.color_pair(2))
        
        # Joint data
        for i, name in enumerate(node.joint_names):
            row = 6 + i
            stdscr.addstr(row, 2, f"{name:10}")
            
            # Position (green if moving)
            color = curses.color_pair(1) if abs(node.velocities[i]) > 0.01 else 0
            stdscr.attron(color)
            stdscr.addstr(row, 13, f"{node.positions[i]:+8.4f}")
            stdscr.attroff(color)
            
            stdscr.addstr(row, 29, f"{node.velocities[i]:+8.4f}")
            stdscr.addstr(row, 47, f"{node.efforts[i]:+8.4f}")
        
        # Simulated hardware representation
        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(14, 2, "SIMULATED HARDWARE OUTPUT (what motors would receive):")
        stdscr.addstr(15, 2, "-" * 60)
        stdscr.attroff(curses.color_pair(2))
        
        # Visual representation of motor positions
        for i, name in enumerate(node.joint_names):
            row = 16 + i
            pos_deg = node.positions[i] * 180.0 / 3.14159
            bar_len = int(20 + pos_deg / 9.0)  # Scale to fit
            bar_len = max(0, min(40, bar_len))
            
            stdscr.addstr(row, 2, f"{name}: ")
            stdscr.addstr(row, 12, "[")
            stdscr.attron(curses.color_pair(1))
            stdscr.addstr(row, 13, "=" * bar_len)
            stdscr.attroff(curses.color_pair(1))
            stdscr.addstr(row, 13 + bar_len, "|")
            stdscr.addstr(row, 54, "]")
            stdscr.addstr(row, 56, f"{pos_deg:+6.1f}°")
        
        # Footer
        stdscr.attron(curses.color_pair(3))
        stdscr.addstr(24, 2, "Press 'q' to quit")
        stdscr.attroff(curses.color_pair(3))
        
        stdscr.refresh()


def main():
    rclpy.init()
    node = HardwareMonitor()
    
    try:
        curses.wrapper(lambda stdscr: run_monitor(stdscr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
