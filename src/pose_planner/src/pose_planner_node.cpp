/**
 * @file pose_planner_node.cpp
 * @brief Plans and executes a motion to a target pose using MoveIt 2.
 *
 * The target pose (position + orientation) is set via ROS parameters so you
 * can change it from the launch file or the command line without recompiling.
 *
 * Usage (after MoveIt demo is already running):
 *   ros2 launch pose_planner pose_planner.launch.py
 *   ros2 launch pose_planner pose_planner.launch.py x:=0.3 y:=0.1 z:=0.25
 *
 * @author Jamal
 */

#include <chrono>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create the node with parameters
  auto node = std::make_shared<rclcpp::Node>(
      "pose_planner_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a logger
  auto logger = node->get_logger();

  // ── Target pose parameters (easy to change from launch / CLI) ──
  double px = node->get_parameter_or<double>("x", 0.166);
  double py = node->get_parameter_or<double>("y", 0.132);
  double pz = node->get_parameter_or<double>("z", 0.0508);
  double ox = node->get_parameter_or<double>("ox", -0.548);
  double oy = node->get_parameter_or<double>("oy", 0.447);
  double oz = node->get_parameter_or<double>("oz", 0.006);
  double ow = node->get_parameter_or<double>("ow", 0.707);

  RCLCPP_INFO(logger, "===========================================");
  RCLCPP_INFO(logger, "  Pose Planner Node");
  RCLCPP_INFO(logger, "===========================================");
  RCLCPP_INFO(logger, "Target position  : (%.3f, %.3f, %.3f)", px, py, pz);
  RCLCPP_INFO(logger, "Target orient.   : (%.3f, %.3f, %.3f, %.3f)", ox, oy, oz,
              ow);
  RCLCPP_INFO(logger, "===========================================");

  // ── MoveGroupInterface ──
  // The planning group name matches the SRDF: "arm"
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");

  // Print some useful info
  RCLCPP_INFO(logger, "Planning frame   : %s",
              move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End-effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // Print current EE pose (helps figure out orientation)
  auto current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current EE position   : (%.4f, %.4f, %.4f)",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z);
  RCLCPP_INFO(logger,
              "Current EE orientation : (x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
              current_pose.orientation.x, current_pose.orientation.y,
              current_pose.orientation.z, current_pose.orientation.w);

  // General settings
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // ── Common orientation for both steps ──
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  pose.orientation.w = ow;

  // ================================================================
  //  STEP 1 – Pre-final pose (OMPL, free joint-space path)
  // ================================================================
  double pre_z = pz + 0.03;
  RCLCPP_INFO(logger, "-------------------------------------------");
  RCLCPP_INFO(logger, "STEP 1: Pre-final pose (%.3f, %.3f, %.3f)", px, py,
              pre_z);
  RCLCPP_INFO(logger, "-------------------------------------------");

  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pre_z;

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPoseTarget(pose);
  move_group.move();

  RCLCPP_INFO(logger, "STEP 1 DONE – reached pre-final pose.");

  // ================================================================
  //  STEP 2 – Final pose via Pilz LIN (straight line)
  // ================================================================
  RCLCPP_INFO(logger, "-------------------------------------------");
  RCLCPP_INFO(logger, "STEP 2: Final pose (%.3f, %.3f, %.3f)  [Pilz LIN]", px,
              py, pz);
  RCLCPP_INFO(logger, "-------------------------------------------");

  // Slow down for Pilz to stay within limits
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");

  pose.position.z = pz;
  move_group.setPoseTarget(pose);
  move_group.move();

  RCLCPP_INFO(logger, "STEP 2 DONE – reached final target pose!");

  // Restore OMPL settings
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");

  // ── Print final joint values ──
  auto joint_values = move_group.getCurrentJointValues();
  RCLCPP_INFO(logger, "Final joint values:");
  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "  joint_%zu = %.4f rad (%.2f deg)", i + 1,
                joint_values[i], joint_values[i] * 180.0 / M_PI);
  }

  rclcpp::shutdown();
  return 0;
}
