#include "control_arm/test_motion_node.hpp"

TestMotionNode::TestMotionNode(const rclcpp::NodeOptions &options)
    : Node("test_motion_node", options) {
  RCLCPP_INFO(this->get_logger(), "Test Motion Node created");
}

void TestMotionNode::init() {
  // Initialize MoveIt interface for the "arm" planning group
  arm_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");

  // Set reference frame and end effector
  arm_->setPoseReferenceFrame("base_link");
  arm_->setEndEffectorLink("link_6");

  // Set planning parameters
  arm_->setNumPlanningAttempts(10);
  arm_->setPlanningTime(5.0);
  arm_->setMaxVelocityScalingFactor(0.5);
  arm_->setMaxAccelerationScalingFactor(0.5);

  // Use OMPL planner
  arm_->setPlanningPipelineId("ompl");
  arm_->setPlannerId("RRTConnectkConfigDefault");

  RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized");
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s",
              arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s",
              arm_->getEndEffectorLink().c_str());

  // Setup test poses - these are simple joint-based poses for testing
  // Pose 1: A slightly bent configuration
  test_pose_1_.header.frame_id = "base_link";
  test_pose_1_.pose.position.x = 0.1;
  test_pose_1_.pose.position.y = 0.1;
  test_pose_1_.pose.position.z = 0.2;
  test_pose_1_.pose.orientation.w = 1.0;
  test_pose_1_.pose.orientation.x = 0.0;
  test_pose_1_.pose.orientation.y = 0.0;
  test_pose_1_.pose.orientation.z = 0.0;

  // Pose 2: Another test position
  test_pose_2_.header.frame_id = "base_link";
  test_pose_2_.pose.position.x = 0.15;
  test_pose_2_.pose.position.y = -0.05;
  test_pose_2_.pose.position.z = 0.15;
  test_pose_2_.pose.orientation.w = 0.7071;
  test_pose_2_.pose.orientation.x = 0.0;
  test_pose_2_.pose.orientation.y = 0.7071;
  test_pose_2_.pose.orientation.z = 0.0;
}

bool TestMotionNode::moveToHome() {
  RCLCPP_INFO(this->get_logger(),
              "Moving to home position (named target: zero)...");

  arm_->setNamedTarget("zero");

  MoveGroupInterface::Plan plan;
  bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Plan to home successful, executing...");
    arm_->execute(plan);
    RCLCPP_INFO(this->get_logger(), "Moved to home position");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to home position");
    return false;
  }
}

bool TestMotionNode::moveToPose(const Pose &pose) {
  RCLCPP_INFO(this->get_logger(), "Planning to pose: [%.3f, %.3f, %.3f]",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  arm_->setPoseTarget(pose);

  MoveGroupInterface::Plan plan;
  bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
    arm_->execute(plan);
    RCLCPP_INFO(this->get_logger(), "Motion completed");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to target pose");
    return false;
  }
}

bool TestMotionNode::moveToJointValues(
    const std::map<std::string, double> &joint_values) {
  RCLCPP_INFO(this->get_logger(), "Moving to joint values...");

  arm_->setJointValueTarget(joint_values);

  MoveGroupInterface::Plan plan;
  bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
    arm_->execute(plan);
    RCLCPP_INFO(this->get_logger(), "Motion completed");
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to joint values");
    return false;
  }
}

bool TestMotionNode::executeTestMotion() {
  RCLCPP_INFO(this->get_logger(), "=== Starting Test Motion Sequence ===");

  // Get current state info
  arm_->setStartStateToCurrentState();

  // Get current end effector pose
  geometry_msgs::msg::PoseStamped current_pose = arm_->getCurrentPose();
  RCLCPP_INFO(this->get_logger(),
              "=== Current End Effector Pose (Before Planning) ===");
  RCLCPP_INFO(this->get_logger(), "Position: [x: %.4f, y: %.4f, z: %.4f]",
              current_pose.pose.position.x, current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(),
              "Orientation: [w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
              current_pose.pose.orientation.w, current_pose.pose.orientation.x,
              current_pose.pose.orientation.y, current_pose.pose.orientation.z);

  // Create target pose in world frame at (0.168, 0.199, 0.137)
  RCLCPP_INFO(
      this->get_logger(),
      "=== Moving to Target Position: [0.168, 0.199, 0.137] (world frame) ===");

  Pose target_pose;
  target_pose.header.frame_id = "world";
  target_pose.pose.position.x = 0.168;
  target_pose.pose.position.y = 0.199;
  target_pose.pose.position.z = 0.137;
  // Keep current orientation
  target_pose.pose.orientation.w = 1.0;
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;

  if (moveToPose(target_pose)) {
    RCLCPP_INFO(this->get_logger(), "Motion to target position SUCCESS!");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Motion to target position FAILED - IK may not have solution");
  }

  // Print final pose
  geometry_msgs::msg::PoseStamped final_pose = arm_->getCurrentPose();
  RCLCPP_INFO(this->get_logger(), "=== Final End Effector Pose ===");
  RCLCPP_INFO(this->get_logger(), "Position: [x: %.4f, y: %.4f, z: %.4f]",
              final_pose.pose.position.x, final_pose.pose.position.y,
              final_pose.pose.position.z);

  RCLCPP_INFO(this->get_logger(), "=== Test Motion Sequence Complete ===");
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create node with options that allow parameters from launch file
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<TestMotionNode>(node_options);

  // Create executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin in a separate thread to allow init() to work
  std::thread spinner([&executor]() { executor.spin(); });

  // Wait a bit for everything to initialize
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Initialize MoveIt interface
  node->init();

  // Execute test motion
  node->executeTestMotion();

  // Keep node alive for a bit to see final state
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Shutdown
  rclcpp::shutdown();
  spinner.join();

  return 0;
}
