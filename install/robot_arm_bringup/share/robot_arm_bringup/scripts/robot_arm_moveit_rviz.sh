#!/bin/bash
# Script to launch robot_arm with MoveIt 2 in RViz only (no Gazebo)
# This is useful for motion planning visualization and testing

cleanup() {
    echo "Cleaning up..."
    sleep 2.0
    pkill -9 -f "ros2|rviz2|robot_state_publisher|joint_state_publisher|moveit|move_group"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching robot_arm with MoveIt 2 (RViz only)..."
ros2 launch robot_arm_movit_config moveit_rviz.launch.py \
    robot_name:=robot_arm \
    use_rviz:=true \
    use_jsp_gui:=false

# Keep the script running until Ctrl+C
wait
