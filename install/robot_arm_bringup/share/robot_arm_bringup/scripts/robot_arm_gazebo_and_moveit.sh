#!/bin/bash
# Single script to launch the robot_arm with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|joint_state_publisher|moveit|move_group"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo Ignition simulation with robot_arm..."
ros2 launch robot_arm_gazebo robot_arm.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_camera:=false \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.03 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

echo "Waiting for Gazebo to initialize..."
sleep 15

echo "Launching MoveIt 2 motion planning..."
ros2 launch robot_arm_movit_config move_group.launch.py &

# Keep the script running until Ctrl+C
wait