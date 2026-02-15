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

#ifndef CUSTOM_HARDWARE__CUSTOM_HARDWARE_HPP_
#define CUSTOM_HARDWARE__CUSTOM_HARDWARE_HPP_

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "custom_hardware/visibility_control.h"

namespace custom_hardware {

class CustomHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CustomHardwareInterface)

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CUSTOM_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  CUSTOM_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  CUSTOM_HARDWARE_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Parameters for the robot hardware interface
  std::vector<std::string> joint_names_;

  // ESP32 TCP connection parameters
  std::string esp32_ip_;
  int esp32_port_;
  int timeout_ms_;
  bool use_simulation_; // Fallback to simulation if ESP32 not available

  // TCP socket
  int socket_fd_;
  bool connected_;

  // Store the commands and states
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // Previous commands (for change detection - only send when changed)
  std::vector<double> prev_commands_position_;

  // Joint configuration (for servo mapping)
  std::vector<double> joint_offsets_;    // Offset from ROS angle to servo angle
  std::vector<double> joint_directions_; // Direction multiplier (1.0 or -1.0)
  std::vector<double> servo_min_angle_;  // Servo minimum angle (degrees)
  std::vector<double> servo_max_angle_;  // Servo maximum angle (degrees)

  // Logger for debugging
  rclcpp::Logger logger_{rclcpp::get_logger("CustomHardwareInterface")};

  // Communication helpers
  bool connect_to_hardware();
  void disconnect_from_hardware();
  bool send_commands_to_hardware();
  bool read_states_from_hardware();

  // Helper methods
  double radians_to_degrees(double radians);
  double degrees_to_radians(double degrees);
  double ros_to_servo_angle(double ros_angle_rad, size_t joint_index);
  double servo_to_ros_angle(double servo_angle_deg, size_t joint_index);
  double clamp_servo_angle(double angle, size_t joint_index);
  bool send_tcp_message(const std::string &message);
  std::string receive_tcp_message(int timeout_ms = 100);
  void simulate_states();
};

} // namespace custom_hardware

#endif // CUSTOM_HARDWARE__CUSTOM_HARDWARE_HPP_
