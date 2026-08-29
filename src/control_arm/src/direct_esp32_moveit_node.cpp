/**
 * @file direct_esp32_moveit_node.cpp
 * @brief Hardware experiments that plan with MoveIt and stream servo degrees
 *        directly to the ESP32, bypassing trajectory execution.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

std::vector<double> interpolate_positions(
  const trajectory_msgs::msg::JointTrajectory & trajectory, double time)
{
  if (trajectory.points.empty()) {
    throw std::runtime_error("MoveIt returned an empty trajectory");
  }

  const auto & first = trajectory.points.front();
  if (time <= duration_seconds(first.time_from_start)) {
    return first.positions;
  }

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & before = trajectory.points[i - 1];
    const auto & after = trajectory.points[i];
    const double before_time = duration_seconds(before.time_from_start);
    const double after_time = duration_seconds(after.time_from_start);

    if (time <= after_time) {
      if (before.positions.size() != after.positions.size()) {
        throw std::runtime_error("MoveIt trajectory point sizes do not match");
      }

      const double span = after_time - before_time;
      const double ratio = span > 0.0 ? (time - before_time) / span : 1.0;
      std::vector<double> result(after.positions.size(), 0.0);
      for (std::size_t joint = 0; joint < result.size(); ++joint) {
        result[joint] = before.positions[joint] +
          ratio * (after.positions[joint] - before.positions[joint]);
      }
      return result;
    }
  }

  return trajectory.points.back().positions;
}

}  // namespace

class DirectEsp32MoveItNode : public rclcpp::Node
{
public:
  DirectEsp32MoveItNode()
  : Node("direct_esp32_moveit_node")
  {
    mode_ = declare_parameter<std::string>("mode", "sequential_final");
    command_interval_seconds_ =
      declare_parameter<double>("command_interval_seconds", 0.2);
    joint_delay_seconds_ = declare_parameter<double>("joint_delay_seconds", 1.0);
    subscriber_wait_seconds_ = declare_parameter<double>("subscriber_wait_seconds", 30.0);

    target_x_ = declare_parameter<double>("x", 0.166);
    target_y_ = declare_parameter<double>("y", 0.132);
    target_z_ = declare_parameter<double>("z", 0.0508);
    target_ox_ = declare_parameter<double>("ox", -0.548);
    target_oy_ = declare_parameter<double>("oy", 0.447);
    target_oz_ = declare_parameter<double>("oz", 0.006);
    target_ow_ = declare_parameter<double>("ow", 0.707);
    pre_final_offset_ = declare_parameter<double>("pre_final_offset", 0.03);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    servo_offsets_deg_ = declare_parameter<std::vector<double>>(
      "servo_offsets_deg", {90.0, 90.0, 90.0, 90.0, 90.0, 90.0});
    servo_directions_ = declare_parameter<std::vector<double>>(
      "servo_directions", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    servo_min_deg_ = declare_parameter<double>("servo_min_deg", 0.0);
    servo_max_deg_ = declare_parameter<double>("servo_max_deg", 180.0);

    if (joint_names_.size() != 6 || servo_offsets_deg_.size() != joint_names_.size() ||
      servo_directions_.size() != joint_names_.size())
    {
      throw std::runtime_error(
              "joint_names, servo_offsets_deg, and servo_directions must contain six values");
    }
    if (command_interval_seconds_ <= 0.0 || joint_delay_seconds_ <= 0.0) {
      throw std::runtime_error("Command intervals must be greater than zero");
    }

    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/esp32/joint_commands", 10);
  }

  bool run()
  {
    if (!wait_for_esp32()) {
      return false;
    }

    moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "arm");
    arm.setPoseReferenceFrame("base_link");
    arm.setEndEffectorLink("link_6");
    arm.setNumPlanningAttempts(20);
    arm.setPlanningTime(10.0);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setGoalJointTolerance(0.02);
    arm.setGoalPositionTolerance(0.005);
    arm.setGoalOrientationTolerance(0.05);
    arm.startStateMonitor(2.0);

    const auto current_state = arm.getCurrentState(10.0);
    if (!current_state) {
      RCLCPP_ERROR(get_logger(), "MoveIt did not provide a current robot state");
      return false;
    }

    if (mode_ == "sequential_final") {
      return run_sequential_final(arm);
    }
    if (mode_ == "slow_full_path") {
      return run_slow_full_path(arm, *current_state);
    }

    RCLCPP_ERROR(get_logger(), "Unknown mode '%s'", mode_.c_str());
    return false;
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using Plan = MoveGroupInterface::Plan;

  geometry_msgs::msg::Pose target_pose(double z) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = target_x_;
    pose.position.y = target_y_;
    pose.position.z = z;
    pose.orientation.x = target_ox_;
    pose.orientation.y = target_oy_;
    pose.orientation.z = target_oz_;
    pose.orientation.w = target_ow_;
    return pose;
  }

  bool wait_for_esp32()
  {
    RCLCPP_INFO(
      get_logger(), "Waiting for an ESP32 subscriber on /esp32/joint_commands ...");
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(subscriber_wait_seconds_);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (command_publisher_->get_subscription_count() > 0) {
        RCLCPP_INFO(get_logger(), "ESP32 command subscriber discovered");
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_ERROR(
      get_logger(), "No subscriber appeared on /esp32/joint_commands within %.1f seconds",
      subscriber_wait_seconds_);
    return false;
  }

  bool plan_to_pose(
    MoveGroupInterface & arm, const geometry_msgs::msg::Pose & pose,
    const std::string & pipeline, const std::string & planner, Plan & plan)
  {
    arm.setPlanningPipelineId(pipeline);
    arm.setPlannerId(planner);
    arm.setPoseTarget(pose);
    const auto result = arm.plan(plan);
    arm.clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "MoveIt planning failed with pipeline '%s' and planner '%s'",
        pipeline.c_str(), planner.c_str());
      return false;
    }
    if (plan.trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "MoveIt returned a successful but empty trajectory");
      return false;
    }
    return true;
  }

  std::vector<double> to_servo_degrees(
    const std::vector<std::string> & trajectory_joint_names,
    const std::vector<double> & positions_rad) const
  {
    if (trajectory_joint_names.size() != positions_rad.size()) {
      throw std::runtime_error("Trajectory joint names and positions have different sizes");
    }

    std::unordered_map<std::string, double> positions_by_name;
    for (std::size_t i = 0; i < trajectory_joint_names.size(); ++i) {
      positions_by_name[trajectory_joint_names[i]] = positions_rad[i];
    }

    std::vector<double> servo_degrees(joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto position = positions_by_name.find(joint_names_[i]);
      if (position == positions_by_name.end()) {
        throw std::runtime_error("Trajectory is missing " + joint_names_[i]);
      }

      const double ros_degrees = position->second * 180.0 / M_PI;
      const double servo_angle =
        servo_offsets_deg_[i] + servo_directions_[i] * ros_degrees;
      if (!std::isfinite(servo_angle) || servo_angle < servo_min_deg_ ||
        servo_angle > servo_max_deg_)
      {
        throw std::runtime_error(
                joint_names_[i] + " produces unsafe servo angle " +
                std::to_string(servo_angle) + " deg");
      }
      servo_degrees[i] = servo_angle;
    }
    return servo_degrees;
  }

  void publish_complete_command(const std::vector<double> & servo_degrees)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = servo_degrees;
    command_publisher_->publish(message);
  }

  void publish_one_joint(std::size_t joint, double servo_degrees)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    message.data[joint] = servo_degrees;
    command_publisher_->publish(message);
  }

  bool run_sequential_final(MoveGroupInterface & arm)
  {
    RCLCPP_INFO(
      get_logger(),
      "Planning directly to final pose (%.3f, %.3f, %.3f); Pilz descent is disabled",
      target_x_, target_y_, target_z_);

    arm.setStartStateToCurrentState();
    Plan plan;
    if (!plan_to_pose(
        arm, target_pose(target_z_), "ompl", "RRTConnectkConfigDefault", plan))
    {
      return false;
    }

    try {
      const auto & trajectory = plan.trajectory_.joint_trajectory;
      const auto servo_degrees = to_servo_degrees(
        trajectory.joint_names, trajectory.points.back().positions);

      RCLCPP_INFO(
        get_logger(), "Sending final target one joint at a time, joint_1 through joint_6");
      for (std::size_t joint = 0; joint < servo_degrees.size() && rclcpp::ok(); ++joint) {
        publish_one_joint(joint, servo_degrees[joint]);
        RCLCPP_INFO(
          get_logger(), "Commanded %s = %.2f deg; ESP32 retains the other five positions",
          joint_names_[joint].c_str(), servo_degrees[joint]);
        if (joint + 1 < servo_degrees.size()) {
          std::this_thread::sleep_for(std::chrono::duration<double>(joint_delay_seconds_));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot send final joint angles: %s", error.what());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Sequential final-angle test completed");
    return true;
  }

  bool stream_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory, const std::string & label,
    bool skip_first_sample)
  {
    const double duration = duration_seconds(trajectory.points.back().time_from_start);
    std::vector<double> sample_times;
    for (double time = 0.0; time < duration; time += command_interval_seconds_) {
      sample_times.push_back(time);
    }
    if (sample_times.empty() || std::abs(sample_times.back() - duration) > 1e-9) {
      sample_times.push_back(duration);
    }
    if (skip_first_sample && !sample_times.empty()) {
      sample_times.erase(sample_times.begin());
    }

    RCLCPP_INFO(
      get_logger(), "Streaming %s: %zu samples at %.3f-second intervals",
      label.c_str(), sample_times.size(), command_interval_seconds_);

    auto next_publish = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(command_interval_seconds_));

    try {
      for (std::size_t sample = 0; sample < sample_times.size() && rclcpp::ok(); ++sample) {
        const auto positions = interpolate_positions(trajectory, sample_times[sample]);
        publish_complete_command(to_servo_degrees(trajectory.joint_names, positions));

        if (sample + 1 < sample_times.size()) {
          next_publish += period;
          std::this_thread::sleep_until(next_publish);
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot stream %s: %s", label.c_str(), error.what());
      return false;
    }
    return true;
  }

  bool run_slow_full_path(
    MoveGroupInterface & arm, const moveit::core::RobotState & current_state)
  {
    const double pre_final_z = target_z_ + pre_final_offset_;
    RCLCPP_INFO(
      get_logger(), "Planning OMPL path to pre-final pose (%.3f, %.3f, %.3f)",
      target_x_, target_y_, pre_final_z);

    arm.setStartState(current_state);
    Plan pre_final_plan;
    if (!plan_to_pose(
        arm, target_pose(pre_final_z), "ompl", "RRTConnectkConfigDefault",
        pre_final_plan))
    {
      return false;
    }

    const auto & pre_final_trajectory = pre_final_plan.trajectory_.joint_trajectory;
    moveit::core::RobotState pre_final_state(current_state);
    pre_final_state.setVariablePositions(
      pre_final_trajectory.joint_names, pre_final_trajectory.points.back().positions);
    pre_final_state.update();

    RCLCPP_INFO(
      get_logger(), "Planning Pilz LIN descent from z=%.3f to z=%.3f",
      pre_final_z, target_z_);
    arm.setStartState(pre_final_state);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);
    Plan final_plan;
    if (!plan_to_pose(
        arm, target_pose(target_z_), "pilz_industrial_motion_planner", "LIN", final_plan))
    {
      return false;
    }

    if (!stream_trajectory(pre_final_trajectory, "OMPL pre-final path", false)) {
      return false;
    }
    if (!stream_trajectory(final_plan.trajectory_.joint_trajectory, "Pilz LIN descent", true)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Slow full-path test completed");
    return true;
  }

  std::string mode_;
  double command_interval_seconds_{0.2};
  double joint_delay_seconds_{1.0};
  double subscriber_wait_seconds_{30.0};
  double target_x_{0.166};
  double target_y_{0.132};
  double target_z_{0.0508};
  double target_ox_{-0.548};
  double target_oy_{0.447};
  double target_oz_{0.006};
  double target_ow_{0.707};
  double pre_final_offset_{0.03};
  std::vector<std::string> joint_names_;
  std::vector<double> servo_offsets_deg_;
  std::vector<double> servo_directions_;
  double servo_min_deg_{0.0};
  double servo_max_deg_{180.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectEsp32MoveItNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  bool success = false;
  try {
    success = node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Direct ESP32 MoveIt test failed: %s", error.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return success ? 0 : 1;
}
