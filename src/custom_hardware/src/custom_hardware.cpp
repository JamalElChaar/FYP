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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace custom_hardware {

hardware_interface::CallbackReturn
CustomHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  // Call parent init first
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF (optional hardware parameters)
  // These can be specified in the ros2_control URDF tag
  serial_port_ = info_.hardware_parameters.count("serial_port") > 0
                     ? info_.hardware_parameters.at("serial_port")
                     : "/dev/ttyUSB0";
  baud_rate_ = info_.hardware_parameters.count("baud_rate") > 0
                   ? std::stoi(info_.hardware_parameters.at("baud_rate"))
                   : 115200;
  timeout_ms_ = info_.hardware_parameters.count("timeout_ms") > 0
                    ? std::stod(info_.hardware_parameters.at("timeout_ms"))
                    : 1000.0;

  RCLCPP_INFO(logger_, "Hardware parameters loaded:");
  RCLCPP_INFO(logger_, "  Serial port: %s", serial_port_.c_str());
  RCLCPP_INFO(logger_, "  Baud rate: %d", baud_rate_);
  RCLCPP_INFO(logger_, "  Timeout: %.1f ms", timeout_ms_);

  // Initialize storage for joint data
  const size_t num_joints = info_.joints.size();

  hw_commands_effort_.resize(num_joints, 0.0);
  hw_commands_position_.resize(num_joints, 0.0);
  hw_commands_velocity_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);
  joint_names_.resize(num_joints);

  // Validate joint configuration from URDF
  for (size_t i = 0; i < num_joints; ++i) {
    const auto &joint = info_.joints[i];
    joint_names_[i] = joint.name;

    RCLCPP_INFO(logger_, "Joint '%s' found with:", joint.name.c_str());

    // Check command interfaces
    for (const auto &cmd_interface : joint.command_interfaces) {
      RCLCPP_INFO(logger_, "  - Command interface: %s",
                  cmd_interface.name.c_str());
    }

    // Check state interfaces
    for (const auto &state_interface : joint.state_interfaces) {
      RCLCPP_INFO(logger_, "  - State interface: %s",
                  state_interface.name.c_str());
    }
  }

  RCLCPP_INFO(
      logger_,
      "CustomHardwareInterface initialized successfully with %zu joints",
      num_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Configuring CustomHardwareInterface...");

  // Reset all states and commands to default values
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    hw_states_position_[i] = 0.0;
    hw_states_velocity_[i] = 0.0;
    hw_states_effort_[i] = 0.0;
    hw_commands_effort_[i] = 0.0;
    hw_commands_position_[i] = 0.0;
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
        RCLCPP_DEBUG(logger_, "Exported state interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY,
            &hw_states_velocity_[i]));
        RCLCPP_DEBUG(logger_, "Exported state interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name, hardware_interface::HW_IF_EFFORT,
            &hw_states_effort_[i]));
        RCLCPP_DEBUG(logger_, "Exported state interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
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
        RCLCPP_DEBUG(logger_, "Exported command interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      } else if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_VELOCITY,
            &hw_commands_velocity_[i]));
        RCLCPP_DEBUG(logger_, "Exported command interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      } else if (cmd_interface.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name, hardware_interface::HW_IF_EFFORT,
            &hw_commands_effort_[i]));
        RCLCPP_DEBUG(logger_, "Exported command interface: %s/%s",
                     joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
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

  // Connect to actual hardware
  if (!connect_to_hardware()) {
    RCLCPP_ERROR(logger_, "Failed to connect to hardware!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial states from hardware
  if (!read_states_from_hardware()) {
    RCLCPP_WARN(logger_, "Could not read initial states, using defaults");
  }

  // Initialize commands to current positions (prevents sudden movement)
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

  // Send zero commands before disconnecting (safety)
  for (size_t i = 0; i < hw_commands_effort_.size(); ++i) {
    hw_commands_effort_[i] = 0.0;
    hw_commands_velocity_[i] = 0.0;
  }
  send_commands_to_hardware();

  // Disconnect from hardware
  disconnect_from_hardware();

  RCLCPP_INFO(logger_, "CustomHardwareInterface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
CustomHardwareInterface::read(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration & /*period*/) {
  // Read the current state from hardware
  if (!read_states_from_hardware()) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "Failed to read from hardware");
    return hardware_interface::return_type::ERROR;
  }

  // Debug output (throttled to avoid spam)
  RCLCPP_DEBUG_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 1000,
      "Read - Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      hw_states_position_[0], hw_states_position_[1], hw_states_position_[2],
      hw_states_position_[3], hw_states_position_[4], hw_states_position_[5]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
CustomHardwareInterface::write(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
  // Write commands to hardware
  if (!send_commands_to_hardware()) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }

  // Debug output (throttled to avoid spam)
  RCLCPP_DEBUG_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 1000,
      "Write - Effort commands: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
      hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5]);

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Hardware Communication Methods
// TODO: Implement these methods based on your actual hardware communication
// ============================================================================

bool CustomHardwareInterface::connect_to_hardware() {
  // TODO: Implement actual hardware connection
  // Example: Open serial port, establish socket connection, etc.
  //
  // For serial communication:
  //   serial_port_.open(serial_port_, baud_rate_);
  //
  // For now, we'll simulate a successful connection

  RCLCPP_INFO(logger_, "Connecting to hardware on %s at %d baud...",
              serial_port_.c_str(), baud_rate_);

  // Simulated connection delay
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(logger_, "Hardware connection established (SIMULATED)");
  return true;
}

void CustomHardwareInterface::disconnect_from_hardware() {
  // TODO: Implement actual hardware disconnection
  // Example: Close serial port, close socket connection, etc.

  RCLCPP_INFO(logger_, "Disconnecting from hardware...");
  RCLCPP_INFO(logger_, "Hardware disconnected (SIMULATED)");
}

bool CustomHardwareInterface::send_commands_to_hardware() {
  // TODO: Implement actual command sending to hardware
  //
  // Example protocol (customize based on your hardware):
  // - Send effort/position/velocity commands via serial/socket
  // - Format: "CMD:<joint1_cmd>,<joint2_cmd>,...,<joint6_cmd>\n"
  //
  // For effort control:
  //   std::stringstream ss;
  //   ss << "EFF:";
  //   for (size_t i = 0; i < hw_commands_effort_.size(); ++i) {
  //     ss << hw_commands_effort_[i];
  //     if (i < hw_commands_effort_.size() - 1) ss << ",";
  //   }
  //   ss << "\n";
  //   serial_port_.write(ss.str());

  // SIMULATION: For testing without hardware, simulate instant response
  // In real implementation, remove this and add actual communication

  return true;
}

bool CustomHardwareInterface::read_states_from_hardware() {
  // TODO: Implement actual state reading from hardware
  //
  // Example protocol (customize based on your hardware):
  // - Request state via serial/socket: "GET_STATE\n"
  // - Parse response: "STATE:<pos1>,<vel1>,<eff1>,<pos2>,..."
  //
  // For reading encoder values:
  //   serial_port_.write("GET_STATE\n");
  //   std::string response = serial_port_.readline();
  //   // Parse response and fill hw_states_position_, hw_states_velocity_, etc.

  // SIMULATION: For testing without hardware, simulate positions following
  // commands Remove this simulation code when implementing real hardware
  // communication

  // Simulation rate for position tracking
  static constexpr double SIMULATION_RATE =
      0.1; // How fast positions track commands

  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    // Calculate position error
    double position_error = hw_commands_position_[i] - hw_states_position_[i];

    // Simulate position tracking (servo-like behavior)
    hw_states_position_[i] += SIMULATION_RATE * position_error;

    // Simulate velocity based on position change
    hw_states_velocity_[i] =
        SIMULATION_RATE * position_error / 0.01; // Approximate

    // Effort is not used in position control mode
    hw_states_effort_[i] = 0.0;
  }

  return true;
}

} // namespace custom_hardware

// Register the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_hardware::CustomHardwareInterface,
                       hardware_interface::SystemInterface)
