/**
 * @file moveit_hardware_node.cpp
 * @brief Interactive MoveIt2 planning & execution on real hardware
 *
 * This node connects MoveIt2 motion planning to the real robot arm via:
 *   MoveIt2 → JointTrajectoryController → CustomHardwareInterface → ESP32
 *
 * It provides an interactive command-line menu so you can:
 *   - Move to home (zero) position
 *   - Move to predefined safe joint poses
 *   - Enter custom joint angles (in degrees — converted to radians internally)
 *   - Enter a Cartesian pose target
 *   - Print current state before each move
 *   - Preview the plan and confirm before executing on hardware
 *
 * @author Jamal
 * @date March 2026
 */

#include "control_arm/moveit_hardware_node.hpp"

#include <cmath>
#include <iostream>
#include <sstream>

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

MoveItHardwareNode::MoveItHardwareNode(const rclcpp::NodeOptions &options)
    : Node("moveit_hardware_node", options) {
  RCLCPP_INFO(this->get_logger(), "MoveIt Hardware Node created");
}

// ═══════════════════════════════════════════════════════════════════════════
// Predefined safe poses  (joint-space, in RADIANS)
//   These stay well within ±90° servo range (≈ ±1.57 rad)
//   Servo mapping: servo_angle = 90 + degrees(ros_angle)
//   So ±90° ROS  →  0–180° servo  (full range of MG995)
//   We keep these conservative: ±30° (~0.52 rad) to start
// ═══════════════════════════════════════════════════════════════════════════

static double deg2rad(double d) { return d * M_PI / 180.0; }

void MoveItHardwareNode::initSafePoses() {
  // All zeros – servos at 90° center
  safe_poses_.push_back({"home (all zero)", {0, 0, 0, 0, 0, 0}});

  // Small wave – only joint 1 rotated 20°
  safe_poses_.push_back({"joint1 +20deg", {deg2rad(20), 0, 0, 0, 0, 0}});

  // Small tilt – joints 2 and 3 at ±15°
  safe_poses_.push_back({"tilt fwd", {0, deg2rad(15), deg2rad(-15), 0, 0, 0}});

  // Slight reach – conservative combination
  safe_poses_.push_back(
      {"slight reach",
       {deg2rad(10), deg2rad(20), deg2rad(-10), deg2rad(5), deg2rad(-5), 0}});

  // Wrist test – only joints 4,5,6
  safe_poses_.push_back(
      {"wrist test", {0, 0, 0, deg2rad(20), deg2rad(-15), deg2rad(10)}});
}

// ═══════════════════════════════════════════════════════════════════════════
// Init
// ═══════════════════════════════════════════════════════════════════════════

void MoveItHardwareNode::init() {
  arm_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "arm");

  arm_->setPoseReferenceFrame("base_link");
  arm_->setEndEffectorLink("link_6");

  // Planning parameters – slow and safe for real hardware
  arm_->setNumPlanningAttempts(20);
  arm_->setPlanningTime(10.0);
  arm_->setMaxVelocityScalingFactor(0.1);     // 10 % of limit
  arm_->setMaxAccelerationScalingFactor(0.1); // 10 % of limit
  arm_->setPlanningPipelineId("ompl");
  arm_->setPlannerId("RRTConnectkConfigDefault");

  // Allow some goal tolerance for real hardware
  arm_->setGoalJointTolerance(0.02);       // ~1°
  arm_->setGoalPositionTolerance(0.005);   // 5 mm
  arm_->setGoalOrientationTolerance(0.05); // ~3°

  initSafePoses();

  RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized");
  RCLCPP_INFO(this->get_logger(), "  Planning frame : %s",
              arm_->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "  End effector   : %s",
              arm_->getEndEffectorLink().c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// Motion helpers
// ═══════════════════════════════════════════════════════════════════════════

void MoveItHardwareNode::printCurrentState() {
  auto joints = arm_->getCurrentJointValues();
  auto pose = arm_->getCurrentPose();

  std::cout << "\n╔══ Current State ══════════════════════════════════╗\n";
  std::cout << "║  Joints (rad / deg):";
  for (size_t i = 0; i < joints.size(); ++i) {
    printf("\n║    joint_%zu = %+7.4f rad  (%+7.2f°)", i + 1, joints[i],
           joints[i] * 180.0 / M_PI);
  }
  std::cout << "\n║  End-effector:";
  printf("\n║    pos  = (%.4f, %.4f, %.4f)", pose.pose.position.x,
         pose.pose.position.y, pose.pose.position.z);
  printf("\n║    quat = (w=%.4f, x=%.4f, y=%.4f, z=%.4f)",
         pose.pose.orientation.w, pose.pose.orientation.x,
         pose.pose.orientation.y, pose.pose.orientation.z);
  std::cout << "\n╚══════════════════════════════════════════════════╝\n";
}

bool MoveItHardwareNode::confirmExecution(
    const MoveGroupInterface::Plan &plan) {
  double duration =
      plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
      plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec /
          1e9;

  size_t n_points = plan.trajectory_.joint_trajectory.points.size();

  std::cout << "\n── Plan found ─────────────────────────────────────\n";
  printf("   Trajectory points : %zu\n", n_points);
  printf("   Duration          : %.2f s\n", duration);

  // Show the final joint values in the plan
  auto &final_pt = plan.trajectory_.joint_trajectory.points.back();
  std::cout << "   Final joint values (deg):";
  for (size_t i = 0; i < final_pt.positions.size(); ++i) {
    printf(" %.1f", final_pt.positions[i] * 180.0 / M_PI);
  }
  std::cout << "\n   Final servo angles (deg):";
  for (size_t i = 0; i < final_pt.positions.size(); ++i) {
    // servo = 90 + ros_deg
    double servo = 90.0 + final_pt.positions[i] * 180.0 / M_PI;
    printf(" %.1f", servo);
  }
  std::cout << "\n───────────────────────────────────────────────────\n";
  std::cout << "   Execute on REAL hardware? (y/n): ";

  std::string answer;
  std::getline(std::cin, answer);
  return (answer == "y" || answer == "Y" || answer == "yes");
}

bool MoveItHardwareNode::moveToHome() {
  RCLCPP_INFO(this->get_logger(), "Planning to HOME (zero) ...");
  arm_->setStartStateToCurrentState();
  arm_->setNamedTarget("zero");

  MoveGroupInterface::Plan plan;
  bool ok = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), "Planning FAILED for home position");
    return false;
  }

  if (confirmExecution(plan)) {
    RCLCPP_INFO(this->get_logger(), "Executing on hardware ...");
    auto result = arm_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Execution SUCCESS");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Execution FAILED (code %d)",
                   static_cast<int>(result.val));
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Execution cancelled by user.");
  return false;
}

bool MoveItHardwareNode::moveToJointTarget(
    const std::vector<double> &joint_values) {
  // Safety check: all within ±90°
  for (size_t i = 0; i < joint_values.size(); ++i) {
    double deg = joint_values[i] * 180.0 / M_PI;
    if (std::abs(deg) > 90.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Joint %zu = %.1f° exceeds ±90° servo range! Aborting.",
                   i + 1, deg);
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Planning to joint target ...");
  arm_->setStartStateToCurrentState();
  arm_->setJointValueTarget(joint_values);

  MoveGroupInterface::Plan plan;
  bool ok = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), "Planning FAILED");
    return false;
  }

  if (confirmExecution(plan)) {
    RCLCPP_INFO(this->get_logger(), "Executing on hardware ...");
    auto result = arm_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Execution SUCCESS");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Execution FAILED (code %d)",
                   static_cast<int>(result.val));
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Execution cancelled by user.");
  return false;
}

bool MoveItHardwareNode::moveToPose(const Pose &target) {
  RCLCPP_INFO(this->get_logger(), "Planning to pose (%.3f, %.3f, %.3f) ...",
              target.pose.position.x, target.pose.position.y,
              target.pose.position.z);

  arm_->setStartStateToCurrentState();
  arm_->setPoseTarget(target);

  MoveGroupInterface::Plan plan;
  bool ok = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(this->get_logger(),
                 "Planning FAILED — pose may be unreachable");
    return false;
  }

  // Extra safety: check all trajectory points stay within servo range
  for (auto &pt : plan.trajectory_.joint_trajectory.points) {
    for (size_t i = 0; i < pt.positions.size(); ++i) {
      double deg = pt.positions[i] * 180.0 / M_PI;
      if (std::abs(deg) > 90.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Trajectory has joint %zu at %.1f° — EXCEEDS servo range! "
                     "Aborting.",
                     i + 1, deg);
        return false;
      }
    }
  }

  if (confirmExecution(plan)) {
    RCLCPP_INFO(this->get_logger(), "Executing on hardware ...");
    auto result = arm_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Execution SUCCESS");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Execution FAILED (code %d)",
                   static_cast<int>(result.val));
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "Execution cancelled by user.");
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// Interactive menu
// ═══════════════════════════════════════════════════════════════════════════

void MoveItHardwareNode::run() {
  while (rclcpp::ok()) {
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║       MoveIt2 → Real Hardware Control Menu          ║\n"
              << "╠══════════════════════════════════════════════════════╣\n"
              << "║  1) Move to HOME (all joints zero)                  ║\n"
              << "║  2) Move to a predefined safe pose                  ║\n"
              << "║  3) Enter custom joint angles (degrees)             ║\n"
              << "║  4) Enter Cartesian pose target (x y z)             ║\n"
              << "║  5) Print current state                             ║\n"
              << "║  6) Plan with RViz (use MotionPlanning plugin)      ║\n"
              << "║  q) Quit                                            ║\n"
              << "╚══════════════════════════════════════════════════════╝\n"
              << " > ";

    std::string choice;
    std::getline(std::cin, choice);

    if (choice == "q" || choice == "Q") {
      RCLCPP_INFO(this->get_logger(), "Shutting down...");
      break;
    }

    if (choice == "1") {
      printCurrentState();
      moveToHome();

    } else if (choice == "2") {
      printCurrentState();
      std::cout << "\nSafe poses:\n";
      for (size_t i = 0; i < safe_poses_.size(); ++i) {
        printf("  %zu) %s  →  [", i, safe_poses_[i].name.c_str());
        for (size_t j = 0; j < safe_poses_[i].joints.size(); ++j) {
          printf("%.1f°", safe_poses_[i].joints[j] * 180.0 / M_PI);
          if (j + 1 < safe_poses_[i].joints.size())
            printf(", ");
        }
        printf("]\n");
      }
      std::cout << "Select pose number: ";
      std::string idx_str;
      std::getline(std::cin, idx_str);
      int idx = std::stoi(idx_str);
      if (idx >= 0 && idx < static_cast<int>(safe_poses_.size())) {
        moveToJointTarget(safe_poses_[idx].joints);
      } else {
        std::cout << "Invalid pose index.\n";
      }

    } else if (choice == "3") {
      printCurrentState();
      std::cout << "Enter 6 joint angles in DEGREES (space-separated):\n > ";
      std::string line;
      std::getline(std::cin, line);
      std::istringstream iss(line);
      std::vector<double> joints;
      double val;
      while (iss >> val) {
        joints.push_back(deg2rad(val));
      }
      if (joints.size() != 6) {
        std::cout << "Expected 6 values, got " << joints.size() << "\n";
      } else {
        moveToJointTarget(joints);
      }

    } else if (choice == "4") {
      printCurrentState();
      std::cout << "Enter target position (x y z in meters):\n > ";
      std::string line;
      std::getline(std::cin, line);
      std::istringstream iss(line);
      double x, y, z;
      if (iss >> x >> y >> z) {
        Pose target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        // Default orientation: identity quaternion
        target.pose.orientation.w = 1.0;
        target.pose.orientation.x = 0.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;

        std::cout << "Enter orientation quaternion (w x y z), or press Enter "
                     "for default:\n > ";
        std::string qline;
        std::getline(std::cin, qline);
        if (!qline.empty()) {
          std::istringstream qss(qline);
          qss >> target.pose.orientation.w >> target.pose.orientation.x >>
              target.pose.orientation.y >> target.pose.orientation.z;
        }
        moveToPose(target);
      } else {
        std::cout << "Invalid input.\n";
      }

    } else if (choice == "5") {
      printCurrentState();

    } else if (choice == "6") {
      std::cout
          << "\n"
          << "  Use the MotionPlanning plugin in RViz:\n"
          << "    1. Drag the interactive marker to desired pose\n"
          << "    2. Click 'Plan' to see the trajectory\n"
          << "    3. Click 'Execute' to send to real hardware\n"
          << "  The trajectory goes through:\n"
          << "    MoveIt → arm_controller → CustomHardwareInterface → ESP32\n"
          << "\n"
          << "  Press Enter to return to menu...\n";
      std::string dummy;
      std::getline(std::cin, dummy);

    } else {
      std::cout << "Invalid choice.\n";
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MoveItHardwareNode>(node_options);

  // Spin in background thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Wait for MoveIt to be ready
  RCLCPP_INFO(node->get_logger(),
              "Waiting 3 s for move_group to be fully ready ...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Initialize MoveIt interface
  node->init();

  // Run interactive menu
  node->run();

  // Cleanup
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
