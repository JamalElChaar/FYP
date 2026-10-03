/**
 * @file joint_target_node.cpp
 * @brief Simulation-only test: plan and execute directly to a joint-space
 *        target expressed in servo degrees, using MoveIt's own trajectory
 *        execution (arm_controller + mock hardware). No ESP32 involved --
 *        unlike direct_esp32_moveit_node, this actually moves the simulated
 *        robot state so RViz shows the result.
 */

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class JointTargetNode : public rclcpp::Node
{
public:
  JointTargetNode()
  : Node("joint_target_node")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    // Target expressed in servo degrees (same convention as
    // direct_esp32_moveit_node), converted to ROS radians below.
    servo_degrees_ = declare_parameter<std::vector<double>>(
      "servo_degrees", {90.0, 90.0, 90.0, 90.0, 90.0, 90.0});
    // Measured calibration: servo_deg = offset + direction * ros_deg.
    // Joints 2-5 calibrated against the real arm; joint_1 and joint_6 are
    // still the uncalibrated placeholder. Keep in sync with joint_offsets_/
    // joint_directions_ in custom_hardware.cpp and with
    // direct_esp32_moveit_node.
    servo_offsets_deg_ = declare_parameter<std::vector<double>>(
      "servo_offsets_deg", {90.0, 103.6364, 111.3713, 0.0, 90.0, 90.0});
    servo_directions_ = declare_parameter<std::vector<double>>(
      "servo_directions", {1.0, 1.0, 1.0, 1.0, -1.0, 1.0});

    if (joint_names_.size() != 6 || servo_degrees_.size() != 6 ||
      servo_offsets_deg_.size() != 6 || servo_directions_.size() != 6)
    {
      throw std::runtime_error(
              "joint_names, servo_degrees, servo_offsets_deg, and servo_directions "
              "must each contain six values");
    }
  }

  bool run()
  {
    moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "arm");
    arm.setPlanningPipelineId("ompl");
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setNumPlanningAttempts(10);
    arm.setPlanningTime(10.0);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setGoalJointTolerance(0.01);
    arm.startStateMonitor(2.0);

    std::vector<double> joint_radians(joint_names_.size(), 0.0);
    RCLCPP_INFO(get_logger(), "Target servo degrees -> ROS radians:");
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const double ros_degrees =
        (servo_degrees_[i] - servo_offsets_deg_[i]) / servo_directions_[i];
      joint_radians[i] = ros_degrees * M_PI / 180.0;
      RCLCPP_INFO(
        get_logger(), "  %s: servo %.1f deg -> %.4f rad",
        joint_names_[i].c_str(), servo_degrees_[i], joint_radians[i]);
    }

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joint_names_, joint_radians);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = arm.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt planning to the joint target failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Executing plan through arm_controller (simulation only)...");
    const auto exec_result = arm.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt execution to the joint target failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Joint target reached");
    return true;
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> servo_degrees_;
  std::vector<double> servo_offsets_deg_;
  std::vector<double> servo_directions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTargetNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  bool success = false;
  try {
    success = node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Joint target test failed: %s", error.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return success ? 0 : 1;
}
