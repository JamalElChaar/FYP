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
#include <poll.h>
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

  // Get ESP32 connection parameters from URDF
  esp32_ip_ = info_.hardware_parameters.count("esp32_ip") > 0
                  ? info_.hardware_parameters.at("esp32_ip")
                  : "172.20.10.4";

  esp32_port_ = info_.hardware_parameters.count("esp32_port") > 0
                    ? std::stoi(info_.hardware_parameters.at("esp32_port"))
                    : 5000;

  timeout_ms_ = info_.hardware_parameters.count("timeout_ms") > 0
                    ? std::stoi(info_.hardware_parameters.at("timeout_ms"))
                    : 100;

  use_simulation_ =
      info_.hardware_parameters.count("use_simulation") > 0
          ? (info_.hardware_parameters.at("use_simulation") == "true")
          : false;

  socket_fd_ = -1;
  connected_ = false;

  RCLCPP_INFO(logger_, "Hardware parameters:");
  RCLCPP_INFO(logger_, "  ESP32 IP: %s", esp32_ip_.c_str());
  RCLCPP_INFO(logger_, "  ESP32 Port: %d", esp32_port_);
  RCLCPP_INFO(logger_, "  Timeout: %d ms", timeout_ms_);
  RCLCPP_INFO(logger_, "  Simulation mode: %s",
              use_simulation_ ? "true" : "false");

  // Initialize storage for joint data
  const size_t num_joints = info_.joints.size();

  hw_commands_effort_.resize(num_joints, 0.0);
  hw_commands_position_.resize(num_joints, 0.0);
  hw_commands_velocity_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);
  prev_commands_position_.resize(num_joints, -999.0); // Force initial send
  joint_names_.resize(num_joints);

  // Configure servo mapping
  joint_offsets_.resize(num_joints);
  joint_directions_.resize(num_joints);
  servo_min_angle_.resize(num_joints);
  servo_max_angle_.resize(num_joints);

  // Default servo configuration (MG995: 0-180° range)
  for (size_t i = 0; i < num_joints; ++i) {
    joint_offsets_[i] = 90.0;    // Servo 90° = ROS 0 rad
    joint_directions_[i] = 1.0;  // Normal direction
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

  // Reset all states and commands
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

  // Connect to ESP32
  if (!connect_to_hardware()) {
    if (use_simulation_) {
      RCLCPP_WARN(logger_,
                  "ESP32 connection failed, running in SIMULATION mode");
    } else {
      RCLCPP_ERROR(logger_, "Failed to connect to ESP32!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

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

  disconnect_from_hardware();

  RCLCPP_INFO(logger_, "CustomHardwareInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
CustomHardwareInterface::read(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration & /*period*/) {
  if (!read_states_from_hardware()) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "Failed to read from hardware");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
CustomHardwareInterface::write(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
  if (!send_commands_to_hardware()) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// TCP Communication Methods
// ============================================================================

bool CustomHardwareInterface::send_tcp_message(const std::string &message) {
  if (socket_fd_ < 0 || !connected_) {
    return false;
  }

  ssize_t bytes_sent = send(socket_fd_, message.c_str(), message.length(), 0);
  if (bytes_sent < 0) {
    RCLCPP_ERROR(logger_, "TCP send failed: %s", strerror(errno));
    connected_ = false;
    return false;
  }

  return true;
}

std::string CustomHardwareInterface::receive_tcp_message(int timeout_ms) {
  if (socket_fd_ < 0 || !connected_) {
    return "";
  }

  // Use poll to check for data with timeout
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return ""; // Timeout or error
  }

  char buffer[256];
  ssize_t bytes_received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0) {
    if (bytes_received < 0) {
      RCLCPP_ERROR(logger_, "TCP receive failed: %s", strerror(errno));
    }
    connected_ = false;
    return "";
  }

  buffer[bytes_received] = '\0';
  return std::string(buffer);
}

bool CustomHardwareInterface::connect_to_hardware() {
  if (use_simulation_ && esp32_ip_ == "simulation") {
    RCLCPP_INFO(logger_,
                "Running in pure SIMULATION mode (no ESP32 connection)");
    connected_ = false;
    return true; // Don't fail, just simulate
  }

  RCLCPP_INFO(logger_, "Connecting to ESP32 at %s:%d...", esp32_ip_.c_str(),
              esp32_port_);

  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create socket: %s", strerror(errno));
    return false;
  }

  // Set socket timeout
  struct timeval tv;
  tv.tv_sec = timeout_ms_ / 1000;
  tv.tv_usec = (timeout_ms_ % 1000) * 1000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  // Set up server address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(esp32_port_);

  if (inet_pton(AF_INET, esp32_ip_.c_str(), &server_addr.sin_addr) <= 0) {
    RCLCPP_ERROR(logger_, "Invalid ESP32 IP address: %s", esp32_ip_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Connect with timeout
  // Set non-blocking for connection attempt
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  int result =
      connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (result < 0 && errno != EINPROGRESS) {
    RCLCPP_ERROR(logger_, "Connection failed immediately: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Wait for connection with timeout
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLOUT;

  int poll_result = poll(&pfd, 1, 3000); // 3 second timeout for connection
  if (poll_result <= 0) {
    RCLCPP_ERROR(logger_, "Connection timeout - ESP32 not responding");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Check if connection succeeded
  int error;
  socklen_t len = sizeof(error);
  if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 ||
      error != 0) {
    RCLCPP_ERROR(logger_, "Connection failed: %s", strerror(error));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set back to blocking mode
  fcntl(socket_fd_, F_SETFL, flags);

  connected_ = true;
  RCLCPP_INFO(logger_, "Connected to ESP32 successfully!");

  // Send initialization command
  if (!send_tcp_message("INIT\n")) {
    RCLCPP_WARN(logger_, "Failed to send init command");
  } else {
    std::string response = receive_tcp_message(1000);
    RCLCPP_INFO(logger_, "ESP32 response: %s", response.c_str());
  }

  return true;
}

void CustomHardwareInterface::disconnect_from_hardware() {
  if (socket_fd_ >= 0) {
    // Send stop command before disconnecting
    if (connected_) {
      send_tcp_message("STOP\n");
    }
    close(socket_fd_);
    socket_fd_ = -1;
    connected_ = false;
    RCLCPP_INFO(logger_, "Disconnected from ESP32");
  }
}

bool CustomHardwareInterface::send_commands_to_hardware() {
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
    return true; // No need to send if nothing changed
  }

  // Build command string: "POS:angle1,angle2,angle3,angle4,angle5,angle6\n"
  // Angles are in degrees for the servo driver
  std::stringstream ss;
  ss << "POS:";
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    double servo_angle = ros_to_servo_angle(hw_commands_position_[i], i);
    ss << std::fixed << std::setprecision(1) << servo_angle;
    if (i < hw_commands_position_.size() - 1) {
      ss << ",";
    }
  }
  ss << "\n";

  // Update previous commands
  for (size_t i = 0; i < hw_commands_position_.size(); ++i) {
    prev_commands_position_[i] = hw_commands_position_[i];
  }

  // Send to ESP32 or simulate
  if (connected_) {
    if (!send_tcp_message(ss.str())) {
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                           "Failed to send command to ESP32");
      // Try to reconnect on next cycle
      disconnect_from_hardware();
      return use_simulation_; // Continue if simulation fallback is enabled
    }
    RCLCPP_DEBUG(logger_, "Sent: %s", ss.str().c_str());
  } else if (use_simulation_) {
    // Simulation mode - just log the command
    RCLCPP_DEBUG_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
                          "SIM: %s", ss.str().c_str());
  } else {
    return false; // Not connected and not in simulation mode
  }

  return true;
}

bool CustomHardwareInterface::read_states_from_hardware() {
  if (connected_) {
    // Request state from ESP32
    if (send_tcp_message("GET\n")) {
      std::string response = receive_tcp_message(timeout_ms_);
      if (!response.empty() && response.substr(0, 4) == "POS:") {
        // Parse response: "POS:angle1,angle2,angle3,angle4,angle5,angle6"
        std::string data = response.substr(4);
        std::stringstream ss(data);
        std::string token;
        size_t i = 0;
        while (std::getline(ss, token, ',') && i < hw_states_position_.size()) {
          try {
            double servo_angle = std::stod(token);
            hw_states_position_[i] = servo_to_ros_angle(servo_angle, i);
          } catch (...) {
            RCLCPP_WARN(logger_, "Failed to parse position for joint %zu", i);
          }
          ++i;
        }
        return true;
      }
    }
    // If communication failed, fall through to simulation
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                         "No response from ESP32, using simulated states");
  }

  // Simulation: positions track commands
  simulate_states();
  return true;
}

void CustomHardwareInterface::simulate_states() {
  // Simulate servo response - positions gradually track commands
  static constexpr double SIMULATION_RATE =
      0.1; // How fast positions track commands

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
