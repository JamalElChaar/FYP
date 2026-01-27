#ifndef CONTROL_ARM__TEST_MOTION_NODE_HPP_
#define CONTROL_ARM__TEST_MOTION_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::PoseStamped;

/**
 * @brief Test motion planning node for robot_arm
 *
 * This node demonstrates basic MoveIt 2 motion planning capabilities
 * by moving the arm to predefined test poses.
 */
class TestMotionNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit TestMotionNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Initialize MoveIt interface (must be called after node is added to
   * executor)
   */
  void init();

  /**
   * @brief Execute test motion sequence
   * @return true if successful, false otherwise
   */
  bool executeTestMotion();

private:
  /**
   * @brief Move arm to home position (all joints at 0)
   * @return true if successful
   */
  bool moveToHome();

  /**
   * @brief Move arm to a test pose
   * @param pose Target pose
   * @return true if successful
   */
  bool moveToPose(const Pose &pose);

  /**
   * @brief Move arm using joint values
   * @param joint_values Map of joint names to values
   * @return true if successful
   */
  bool moveToJointValues(const std::map<std::string, double> &joint_values);

  /// MoveIt arm interface
  std::unique_ptr<MoveGroupInterface> arm_;

  /// Test poses
  Pose test_pose_1_;
  Pose test_pose_2_;
};

#endif // CONTROL_ARM__TEST_MOTION_NODE_HPP_
