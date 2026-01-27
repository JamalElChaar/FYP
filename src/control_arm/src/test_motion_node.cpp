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
  RCLCPP_INFO(this->get_logger(), "=== Current End Effector Pose ===");
  RCLCPP_INFO(this->get_logger(), "Position: [x: %.4f, y: %.4f, z: %.4f]",
              current_pose.pose.position.x, current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(),
              "Orientation: [w: %.4f, x: %.4f, y: %.4f, z: %.4f]",
              current_pose.pose.orientation.w, current_pose.pose.orientation.x,
              current_pose.pose.orientation.y, current_pose.pose.orientation.z);

  // Get current joint values
  std::vector<double> current_joints = arm_->getCurrentJointValues();
  std::vector<std::string> joint_names = arm_->getJointNames();
  RCLCPP_INFO(this->get_logger(), "=== Current Joint Values ===");
  for (size_t i = 0; i < joint_names.size() && i < current_joints.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "%s: %.4f rad (%.2f deg)",
                joint_names[i].c_str(), current_joints[i],
                current_joints[i] * 180.0 / M_PI);
  }

  // Step 1: Move to a slightly different joint configuration (very small
  // change)
  RCLCPP_INFO(this->get_logger(), "Step 1: Moving joints by small offset...");
  std::map<std::string, double> small_offset_joints;
  for (size_t i = 0; i < joint_names.size() && i < current_joints.size(); ++i) {
    // Add a small offset of 0.1 radians (~5.7 degrees) to joint 1 only
    if (i == 0) {
      small_offset_joints[joint_names[i]] = current_joints[i] + 0.1;
    } else {
      small_offset_joints[joint_names[i]] = current_joints[i];
    }
  }

  if (moveToJointValues(small_offset_joints)) {
    RCLCPP_INFO(this->get_logger(), "Small joint motion SUCCESS!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Small joint motion FAILED!");
  }

  rclcpp::sleep_for(std::chrono::seconds(2));

  // Step 2: Move back to original position
  RCLCPP_INFO(this->get_logger(),
              "Step 2: Moving back to original joint position...");
  std::map<std::string, double> original_joints;
  for (size_t i = 0; i < joint_names.size() && i < current_joints.size(); ++i) {
    original_joints[joint_names[i]] = current_joints[i];
  }

  if (moveToJointValues(original_joints)) {
    RCLCPP_INFO(this->get_logger(), "Return motion SUCCESS!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Return motion FAILED!");
  }

  rclcpp::sleep_for(std::chrono::seconds(2));

  // Step 3: Try Cartesian motion - move end effector by tiny amount (1cm in Z)
  RCLCPP_INFO(this->get_logger(),
              "Step 3: Attempting small Cartesian motion (1cm in Z)...");
  Pose target_pose = current_pose;
  target_pose.pose.position.z -= 0.1; // Move 1cm up

  if (moveToPose(target_pose)) {
    RCLCPP_INFO(this->get_logger(), "Small Cartesian motion SUCCESS!");
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Small Cartesian motion FAILED - IK may not have solution");
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
