#ifndef CONTROL_ARM__MOVEIT_HARDWARE_NODE_HPP_
#define CONTROL_ARM__MOVEIT_HARDWARE_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::PoseStamped;

/**
 * @brief MoveIt2 hardware execution node for robot_arm
 *
 * Interactive node that plans and executes trajectories on real hardware
 * through the ros2_control → CustomHardwareInterface → ESP32 pipeline.
 *
 * Provides a command-line menu to:
 *   1) Move to home (all joints zero)
 *   2) Move to predefined safe poses (validated joint targets)
 *   3) Move to a custom joint configuration
 *   4) Move to a Cartesian pose target
 *   5) Print current joint state & end-effector pose
 */
class MoveItHardwareNode : public rclcpp::Node {
public:
  explicit MoveItHardwareNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /// Initialize MoveIt interface (must be called after adding node to executor)
  void init();

  /// Main interactive loop
  void run();

private:
  // ── Motion commands ─────────────────────────────────────────────────
  bool moveToHome();
  bool moveToJointTarget(const std::vector<double> &joint_values);
  bool moveToPose(const Pose &target);

  // ── Predefined safe poses (joint-space) ─────────────────────────────
  struct NamedPose {
    std::string name;
    std::vector<double> joints; // 6 values in radians
  };
  std::vector<NamedPose> safe_poses_;
  void initSafePoses();

  // ── Utilities ───────────────────────────────────────────────────────
  void printCurrentState();
  bool confirmExecution(
      const moveit::planning_interface::MoveGroupInterface::Plan &plan);

  // ── MoveIt ──────────────────────────────────────────────────────────
  std::unique_ptr<MoveGroupInterface> arm_;
};

#endif // CONTROL_ARM__MOVEIT_HARDWARE_NODE_HPP_
