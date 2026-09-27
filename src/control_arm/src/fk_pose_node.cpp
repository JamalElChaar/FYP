/**
 * @file fk_pose_node.cpp
 * @brief Forward kinematics helper: given a joint configuration (in servo
 *        degrees, the same convention as the rest of the package), print the
 *        end-effector pose that results from it.
 *
 * This is the inverse of the workflow that keeps failing: instead of guessing
 * a Cartesian pose and hoping IK finds a solution inside the joint limits,
 * pick joint angles the servos can actually reach and derive the pose from
 * them. FK always has exactly one answer, so a pose produced this way is
 * reachable by construction.
 *
 * Prints the pose in the exact argument form the final_angles_sequential
 * launch files expect.
 */

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

class FkPoseNode : public rclcpp::Node
{
public:
  FkPoseNode()
  : Node("fk_pose_node")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    servo_degrees_ = declare_parameter<std::vector<double>>(
      "servo_degrees", {90.0, 90.0, 90.0, 90.0, 90.0, 90.0});
    // When true, servo_degrees is interpreted as ROS joint degrees instead,
    // skipping the servo offset/direction conversion.
    input_is_ros_degrees_ = declare_parameter<bool>("input_is_ros_degrees", false);

    // Must match custom_hardware.cpp and direct_esp32_moveit_node.
    servo_offsets_deg_ = declare_parameter<std::vector<double>>(
      "servo_offsets_deg", {90.0, 45.0, 115.0, 0.0, -20.0, 90.0});
    servo_directions_ = declare_parameter<std::vector<double>>(
      "servo_directions", {1.0, 1.0, 1.0, 1.0, -1.0, 1.0});

    base_link_ = declare_parameter<std::string>("base_link", "base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "link_6");
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");

    if (joint_names_.size() != 6 || servo_degrees_.size() != 6 ||
      servo_offsets_deg_.size() != 6 || servo_directions_.size() != 6)
    {
      throw std::runtime_error(
              "joint_names, servo_degrees, servo_offsets_deg and servo_directions "
              "must each contain six values");
    }
  }

  bool run()
  {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model) {
      RCLCPP_ERROR(get_logger(), "Could not load the robot model from 'robot_description'");
      return false;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();

    std::ostringstream report;
    report << "\n"
           << "=================== forward kinematics ===================\n";
    report << std::fixed << std::setprecision(2);
    report << "  joint      input      ROS deg       ROS rad    within limits\n";

    bool all_in_limits = true;
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      double ros_degrees;
      if (input_is_ros_degrees_) {
        ros_degrees = servo_degrees_[i];
      } else {
        ros_degrees =
          (servo_degrees_[i] - servo_offsets_deg_[i]) / servo_directions_[i];
      }
      const double ros_radians = ros_degrees * M_PI / 180.0;

      const auto * joint_model = model->getJointModel(joint_names_[i]);
      if (!joint_model) {
        RCLCPP_ERROR(
          get_logger(), "Joint '%s' is not in the robot model", joint_names_[i].c_str());
        return false;
      }

      const bool in_limits = joint_model->satisfiesPositionBounds(&ros_radians);
      all_in_limits &= in_limits;

      report << "  " << std::setw(9) << std::left << joint_names_[i] << std::right
             << std::setw(9) << servo_degrees_[i]
             << std::setw(13) << ros_degrees
             << std::setw(13) << std::setprecision(4) << ros_radians
             << std::setprecision(2)
             << std::setw(15) << (in_limits ? "yes" : "NO -- CLAMPED") << "\n";

      state.setJointPositions(joint_names_[i], &ros_radians);
    }

    if (!all_in_limits) {
      report << "\n  WARNING: at least one joint is outside its position limits.\n"
             << "  MoveIt will not be able to plan to the pose below from, or to,\n"
             << "  that configuration. Pick angles inside the limits instead.\n";
    }

    state.enforceBounds();
    state.update();

    if (!state.knowsFrameTransform(base_link_) || !state.knowsFrameTransform(tip_link_)) {
      RCLCPP_ERROR(
        get_logger(), "Robot model does not know '%s' or '%s'",
        base_link_.c_str(), tip_link_.c_str());
      return false;
    }

    // Pose of the tip expressed in the base frame, which is the frame
    // direct_esp32_moveit_node plans in (setPoseReferenceFrame("base_link")).
    const Eigen::Isometry3d base = state.getGlobalLinkTransform(base_link_);
    const Eigen::Isometry3d tip = state.getGlobalLinkTransform(tip_link_);
    const Eigen::Isometry3d pose = base.inverse() * tip;

    const Eigen::Vector3d p = pose.translation();
    Eigen::Quaterniond q(pose.rotation());
    q.normalize();

    report << std::setprecision(4);
    report << "\n  pose of " << tip_link_ << " in " << base_link_ << ":\n"
           << "    position     x=" << p.x() << "  y=" << p.y() << "  z=" << p.z() << "\n"
           << "    orientation ox=" << q.x() << " oy=" << q.y()
           << " oz=" << q.z() << " ow=" << q.w() << "\n";

    report << "\n  launch arguments (copy/paste):\n\n"
           << "    ros2 launch control_arm final_angles_sequential.launch.py \\\n"
           << "      x:=" << p.x() << " y:=" << p.y() << " z:=" << p.z() << " \\\n"
           << "      ox:=" << q.x() << " oy:=" << q.y()
           << " oz:=" << q.z() << " ow:=" << q.w() << "\n";
    report << "==========================================================\n";

    RCLCPP_INFO(get_logger(), "%s", report.str().c_str());
    return all_in_limits;
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> servo_degrees_;
  std::vector<double> servo_offsets_deg_;
  std::vector<double> servo_directions_;
  bool input_is_ros_degrees_{false};
  std::string base_link_;
  std::string tip_link_;
  std::string planning_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FkPoseNode>();

  bool success = false;
  try {
    success = node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Forward kinematics failed: %s", error.what());
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}
