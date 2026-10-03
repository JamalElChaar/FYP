// Copyright 2026 Jamal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "custom_hardware/custom_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace custom_hardware {

// ============================================================================
// Helper Methods
// ============================================================================

double CustomHardwareInterface::radians_to_degrees(double radians) {
  return radians * 180.0 / M_PI;
}

double CustomHardwareInterface::degrees_to_radians(double degrees) {
  return degrees * M_PI / 180.0;
}

double CustomHardwareInterface::ros_to_servo_angle(double ros_angle_rad,
                                                   size_t joint_index) {
  // Convert ROS angle (radians) to servo angle (degrees)
  // servo_angle = offset + direction * ros_angle_in_degrees
  double ros_angle_deg = radians_to_degrees(ros_angle_rad);
  double servo_angle = joint_offsets_[joint_index] +
                       joint_directions_[joint_index] * ros_angle_deg;
  return clamp_servo_angle(servo_angle, joint_index);
}

double CustomHardwareInterface::servo_to_ros_angle(double servo_angle_deg,
                                                   size_t joint_index) {
  // Convert servo angle (degrees) to ROS angle (radians)
  double ros_angle_deg = (servo_angle_deg - joint_offsets_[joint_index]) /
                         joint_directions_[joint_index];
  return degrees_to_radians(ros_angle_deg);
}

double CustomHardwareInterface::clamp_servo_angle(double angle,
                                                  size_t joint_index) {
  return std::clamp(angle, servo_min_angle_[joint_index],
                    servo_max_angle_[joint_index]);
}

// ============================================================================
// Lifecycle Callbacks
// ============================================================================

hardware_interface::CallbackReturn
CustomHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get simulation parameter from URDF
  // Note: xacro may output "True" (Python-style), so check both cases
  if (info_.hardware_parameters.count("use_simulation") > 0) {
    auto val = info_.hardware_parameters.at("use_simulation");
    use_simulation_ = (val == "true" || val == "True" || val == "1");
  } else {
    use_simulation_ = false;
  }

  esp32_connected_ = false;

  RCLCPP_INFO(logger_, "Hardware parameters:");
  RCLCPP_INFO(logger_, "  Simulation mode: %s",
              use_simulation_ ? "true" : "false");
  RCLCPP_INFO(logger_, "  Communication: micro-ROS (ROS2 topics)");

  // Initialize storage for joint data
  const size_t num_joints = info_.joints.size();

  hw_commands_effort_.resize(num_joints, 0.0);
  hw_commands_position_.resize(num_joints, 0.0);
  hw_commands_velocity_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);
  prev_commands_position_.resize(num_joints, -999.0); // Force initial send
  latest_esp32_states_.resize(num_joints, 90.0); // Default to center position
  joint_names_.resize(num_joints);

  // Configure servo mapping
  joint_offsets_.resize(num_joints);
  joint_directions_.resize(num_joints);
  servo_min_angle_.resize(num_joints);
  servo_max_angle_.resize(num_joints);

  // Measured servo calibration: servo_deg = offset + direction * ros_deg.
  //
  // Joints 2-5 were calibrated against the real arm by matching the RViz pose
  // to the physical pose at two points, servo 0 deg and servo 90 deg:
  //   joint_2: ros -78.782 -> servo 0 (re-measured after the servo was
  //            reassembled; taken as the reference, direction forced to +1)
  //   joint_3: ros -30 -> servo 90, ros -110 -> servo 0
  //   joint_4: ros  90 -> servo 90, ros    0 -> servo 0
  //   joint_5: ros -110 -> servo 90, ros -20 -> servo 0
  //
  // Joints 2-5 are direct drive: 180 deg of servo equals 180 deg of joint,
  // so their direction is +-1 and the offsets are the least-squares fit at
  // that fixed slope (joint_3 within +-5 deg; 4 and 5 exact).
  //
  //
  // joint_1 and joint_6 are NOT calibrated yet and keep the old placeholder
  // (offset 90, direction +1). joint_1 is still disabled in the firmware.
  //
  // These values must stay in sync with servo_offsets_deg/servo_directions in
  // control_arm's direct_esp32_moveit_node and joint_target_node.
  static constexpr double kJointOffsets[6] = {90.0, 103.6364, 111.3713, 0.0, 90.0, 90.0};
  static constexpr double kJointDirections[6] = {1.0, 1.0, 1.0, 1.0, -1.0, 1.0};

  for (size_t i = 0; i < num_joints; ++i) {
    joint_offsets_[i] = (i < 6) ? kJointOffsets[i] : 90.0;
    joint_directions_[i] = (i < 6) ? kJointDirections[i] : 1.0;
    servo_min_angle_[i] = 0.0;   // Servo min
    servo_max_angle_[i] = 180.0; // Servo max
  }

  // Validate joint configuration from URDF
  for (size_t i = 0; i < num_joints; ++i) {
    const auto &joint = info_.joints[i];
    joint_names_[i] = joint.name;
    RCLCPP_INFO(logger_, "Joint %zu: '%s'", i, joint.name.c_str());
  }

  RCLCPP_INFO(logger_, "CustomHardwareInterface initialized with %zu joints",
              num_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Configuring CustomHardwareInterface...");

  // Reset all states and commands to the servo centre (90 deg), which is the
  // position the ESP32 firmware drives every servo to at boot.
  //
  // Zero is NOT a safe default any more. With the calibrated offsets, ROS 0
  // is outside some joints' position limits -- joint_5's range is -200..-20
  // deg -- so a zeroed start state is rejected by OMPL ("Skipping invalid
  // start state (invalid bounds)") and MoveIt cannot plan at all until a real
  // /esp32/joint_states message arrives. latest_esp32_states_ is initialised
  // to 90 deg in on_init, so this is in bounds by construction.
  //
  // Commands are seeded to match the states so that simulate_states() holds
  // this pose instead of driving back toward zero.
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    hw_states_position_[i] = servo_to_ros_angle(latest_esp32_states_[i], i);
    hw_states_velocity_[i] = 0.0;
    hw_states_effort_[i] = 0.0;
    hw_commands_effort_[i] = 0.0;
    hw_commands_position_[i] = hw_states_position_[i];
    hw_commands_velocity_[i] = 0.0;
  }

  RCLCPP_INFO(logger_, "CustomHardwareInterface configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CustomHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint = info_.joints[i];

    for (const auto &state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_POSITION,
            &hw_states_position_[i]));
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY,
            &hw_states_velocity_[i]));
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_EFFORT,
            &hw_states_effort_[i]));
      }
    }
  }

  RCLCPP_INFO(logger_, "Exported %zu state interfaces",
              state_interfaces.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CustomHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint = info_.joints[i];

    for (const auto &cmd_interface : joint.command_interfaces) {
      if (cmd_interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_POSITION,
            &hw_commands_position_[i]));
      } else if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY,
            &hw_commands_velocity_[i]));
      } else if (cmd_interface.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_EFFORT,
            &hw_commands_effort_[i]));
      }
    }
  }

  RCLCPP_INFO(logger_, "Exported %zu command interfaces",
              command_interfaces.size());
  return command_interfaces;
}

hardware_interface::CallbackReturn CustomHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating CustomHardwareInterface...");

  // Create a ROS2 node for communication with ESP32 via micro-ROS topics
  node_ = std::make_shared<rclcpp::Node>("custom_hardware_bridge");

  // Create publisher for joint commands to ESP32
  cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/esp32/joint_commands", 10);

  // Create subscriber for joint states from ESP32
  state_subscriber_ =
      node_->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/esp32/joint_states", 10,
          std::bind(&CustomHardwareInterface::state_callback, this,
                    std::placeholders::_1));

  // Create executor for the node
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  // Start executor in a separate thread
  executor_running_ = true;
  executor_thread_ = std::thread([this]() {
    while (executor_running_) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  RCLCPP_INFO(logger_, "micro-ROS communication initialized");
  RCLCPP_INFO(logger_, "  Publishing to: /esp32/joint_commands");
  RCLCPP_INFO(logger_, "  Subscribing to: /esp32/joint_states");

  // Initialize commands to current positions
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    hw_commands_position_[i] = hw_states_position_[i];
    hw_commands_velocity_[i] = 0.0;
    hw_commands_effort_[i] = 0.0;
  }

  RCLCPP_INFO(logger_, "CustomHardwareInterface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating CustomHardwareInterface...");

  // Stop executor thread
  executor_running_ = false;
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  // Clean up ROS2 objects
  if (executor_) {
    executor_->remove_node(node_);
  }
  state_subscriber_.reset();
  cmd_publisher_.reset();
  node_.reset();
  executor_.reset();

  RCLCPP_INFO(logger_, "CustomHardwareInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
CustomHardwareInterface::read(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration & /*period*/) {
  // Read states from the latest received ESP32 message
  if (states_received_) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0;
         i < hw_states_position_.size() && i < latest_esp32_states_.size();
         ++i) {
      hw_states_position_[i] = servo_to_ros_angle(latest_esp32_states_[i], i);
    }
    esp32_connected_ = true;
  } else if (use_simulation_ || !esp32_connected_) {
    // On real hardware this branch means no /esp32/joint_states has arrived,
    // so /joint_states is an assumption, not feedback. Say so out loud --
    // silently reporting a fabricated pose makes MoveIt look healthy while
    // the arm is not actually being tracked.
    if (!use_simulation_) {
      static int no_state_warn_counter = 0;
      if (no_state_warn_counter++ % 500 == 0) {  // ~5 s at 100 Hz
        RCLCPP_WARN(logger_,
                    "No /esp32/joint_states received yet. Reporting the assumed "
                    "servo-90 pose; joint feedback is NOT live. Check that the "
                    "ESP32 state publisher and micro-ROS Agent are running.");
      }
    }
    // Simulation: positions track commands
    simulate_states();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
CustomHardwareInterface::write(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
  publish_commands_to_esp32();
  return hardware_interface::return_type::OK;
}

// ============================================================================
// micro-ROS Communication Methods
// ============================================================================

void CustomHardwareInterface::state_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  // Store received states in thread-safe manner
  std::lock_guard<std::mutex> lock(state_mutex_);

  size_t num_values = std::min(msg->data.size(), latest_esp32_states_.size());
  for (size_t i = 0; i < num_values; ++i) {
    latest_esp32_states_[i] = msg->data[i];
  }
  states_received_ = true;
  esp32_connected_ = true;

  RCLCPP_DEBUG(logger_, "Received joint states from ESP32");
}

void CustomHardwareInterface::publish_commands_to_esp32() {
  // Check if any command has changed significantly
  bool changed = false;
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    if (std::abs(hw_commands_position_[i] - prev_commands_position_[i]) >
        0.001) {
      changed = true;
      break;
    }
  }

  if (!changed) {
    return; // No need to publish if nothing changed
  }

  // Build message with servo angles (degrees)
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.resize(hw_commands_position_.size());

  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    msg.data[i] = ros_to_servo_angle(hw_commands_position_[i], i);
    prev_commands_position_[i] = hw_commands_position_[i];
  }

  // Publish to ESP32
  if (cmd_publisher_) {
    cmd_publisher_->publish(msg);
    RCLCPP_DEBUG(logger_, "Published joint commands to ESP32");
  }

  // If in simulation mode, also update simulated states
  if (use_simulation_ && !esp32_connected_) {
    simulate_states();
  }
}

void CustomHardwareInterface::simulate_states() {
  // Simulate servo response - positions gradually track commands
  static constexpr double SIMULATION_RATE = 0.1;

  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    double position_error = hw_commands_position_[i] - hw_states_position_[i];
    hw_states_position_[i] += SIMULATION_RATE * position_error;
    hw_states_velocity_[i] = SIMULATION_RATE * position_error / 0.01;
    hw_states_effort_[i] = 0.0;
  }
}

} // namespace custom_hardware

// Register the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_hardware::CustomHardwareInterface,
                       hardware_interface::SystemInterface)
