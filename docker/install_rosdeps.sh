#!/bin/bash
set -e

echo "---- Installing ROS 2 dependencies ----"

apt-get install -y --no-install-recommends \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-ros-warehouse \
  ros-humble-moveit-planners \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-msgs

apt-get install -y --no-install-recommends \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-effort-controllers \
  ros-humble-gripper-controllers

apt-get install -y --no-install-recommends \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control

apt-get install -y --no-install-recommends \
  ros-humble-xacro \
  ros-humble-urdf-tutorial \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui

apt-get install -y --no-install-recommends \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-rqt-joint-trajectory-controller

apt-get install -y --no-install-recommends \
  ros-humble-tf2-ros \
  ros-humble-control-msgs \
  ros-humble-controller-manager-msgs \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  python3-numpy

echo "---- ROS 2 dependencies installed successfully ----"
