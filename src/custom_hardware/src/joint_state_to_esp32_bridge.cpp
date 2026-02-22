#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointStateToEsp32Bridge : public rclcpp::Node {
public:
  JointStateToEsp32Bridge() : Node("joint_state_to_esp32_bridge") {
    joint_names_ = this->declare_parameter<std::vector<std::string>>(
        "joint_names", {"joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6"});
    servo_offsets_deg_ = this->declare_parameter<std::vector<double>>(
        "servo_offsets_deg", {90.0, 90.0, 90.0, 90.0, 90.0, 90.0});
    servo_directions_ = this->declare_parameter<std::vector<double>>(
        "servo_directions", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    servo_min_deg_ = this->declare_parameter<double>("servo_min_deg", 0.0);
    servo_max_deg_ = this->declare_parameter<double>("servo_max_deg", 180.0);
    publish_on_change_only_ =
        this->declare_parameter<bool>("publish_on_change_only", true);
    epsilon_deg_ = this->declare_parameter<double>("epsilon_deg", 0.1);
    auto_return_to_neutral_ =
        this->declare_parameter<bool>("auto_return_to_neutral", true);
    neutral_delay_sec_ = this->declare_parameter<double>("neutral_delay_sec", 2.0);
    neutral_command_deg_ = this->declare_parameter<std::vector<double>>(
        "neutral_command_deg", {90.0, 90.0, 90.0, 90.0, 90.0, 90.0});

    const std::size_t n = joint_names_.size();
    if (n == 0) {
      RCLCPP_FATAL(this->get_logger(), "joint_names must not be empty");
      throw std::runtime_error("joint_names empty");
    }

    if (servo_offsets_deg_.size() != n) {
      RCLCPP_WARN(this->get_logger(),
                  "servo_offsets_deg size (%zu) != joint_names size (%zu), "
                  "using default 90 deg",
                  servo_offsets_deg_.size(), n);
      servo_offsets_deg_.assign(n, 90.0);
    }
    if (servo_directions_.size() != n) {
      RCLCPP_WARN(this->get_logger(),
                  "servo_directions size (%zu) != joint_names size (%zu), "
                  "using default direction +1",
                  servo_directions_.size(), n);
      servo_directions_.assign(n, 1.0);
    }
    if (neutral_command_deg_.size() != n) {
      RCLCPP_WARN(this->get_logger(),
                  "neutral_command_deg size (%zu) != joint_names size (%zu), "
                  "using default 90 deg",
                  neutral_command_deg_.size(), n);
      neutral_command_deg_.assign(n, 90.0);
    }
    if (neutral_delay_sec_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "neutral_delay_sec must be > 0, forcing to 2.0");
      neutral_delay_sec_ = 2.0;
    }

    last_command_deg_.assign(n, 90.0);
    last_slider_command_deg_.assign(n, 90.0);

    cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/esp32/joint_commands", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JointStateToEsp32Bridge::joint_state_callback, this,
                  std::placeholders::_1));

    neutral_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(neutral_delay_sec_)),
        std::bind(&JointStateToEsp32Bridge::neutral_timer_callback, this));
    neutral_timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "Bridge ready: /joint_states -> /esp32/joint_commands (%zu joints)",
                n);
  }

private:
  static double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

  double clamp_deg(double value) const {
    return std::clamp(value, servo_min_deg_, servo_max_deg_);
  }

  bool command_is_neutral(const std::vector<double> &command) const {
    for (std::size_t i = 0; i < command.size(); ++i) {
      if (std::abs(command[i] - neutral_command_deg_[i]) > epsilon_deg_) {
        return false;
      }
    }
    return true;
  }

  void publish_command(const std::vector<double> &command) {
    std_msgs::msg::Float64MultiArray out;
    out.data = command;
    cmd_publisher_->publish(out);
    last_command_deg_ = command;
  }

  void neutral_timer_callback() {
    if (!auto_return_to_neutral_) {
      neutral_timer_->cancel();
      return;
    }

    publish_command(neutral_command_deg_);
    neutral_timer_->cancel();
    RCLCPP_INFO(this->get_logger(),
                "Auto-return: published neutral command [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                neutral_command_deg_[0], neutral_command_deg_[1],
                neutral_command_deg_[2], neutral_command_deg_[3],
                neutral_command_deg_[4], neutral_command_deg_[5]);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::vector<double> command_deg(joint_names_.size(), 90.0);

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        command_deg[i] =
            slider_command_initialized_ ? last_slider_command_deg_[i] : 90.0;
        continue;
      }

      const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (idx >= msg->position.size()) {
        command_deg[i] =
            slider_command_initialized_ ? last_slider_command_deg_[i] : 90.0;
        continue;
      }

      const double ros_deg = rad_to_deg(msg->position[idx]);
      const double servo_deg =
          servo_offsets_deg_[i] + servo_directions_[i] * ros_deg;
      command_deg[i] = clamp_deg(servo_deg);
    }

    bool slider_changed = !slider_command_initialized_;
    if (!slider_changed) {
      for (std::size_t i = 0; i < command_deg.size(); ++i) {
        if (std::abs(command_deg[i] - last_slider_command_deg_[i]) > epsilon_deg_) {
          slider_changed = true;
          break;
        }
      }
    }

    if (publish_on_change_only_ && !slider_changed) {
      return;
    }

    last_slider_command_deg_ = command_deg;
    slider_command_initialized_ = true;
    publish_command(command_deg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Published ESP32 command [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                         command_deg[0], command_deg[1], command_deg[2],
                         command_deg[3], command_deg[4], command_deg[5]);

    if (!auto_return_to_neutral_) {
      return;
    }

    if (command_is_neutral(command_deg)) {
      neutral_timer_->cancel();
      return;
    }

    // Restart one-shot timer on each non-neutral command.
    neutral_timer_->reset();
  }

  std::vector<std::string> joint_names_;
  std::vector<double> servo_offsets_deg_;
  std::vector<double> servo_directions_;
  std::vector<double> last_command_deg_;
  std::vector<double> last_slider_command_deg_;
  double servo_min_deg_{0.0};
  double servo_max_deg_{180.0};
  bool publish_on_change_only_{true};
  double epsilon_deg_{0.1};
  bool auto_return_to_neutral_{true};
  double neutral_delay_sec_{2.0};
  std::vector<double> neutral_command_deg_;
  bool slider_command_initialized_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr neutral_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateToEsp32Bridge>());
  rclcpp::shutdown();
  return 0;
}
