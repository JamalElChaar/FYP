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

#include <memory>
#include <string>
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
  // Store the command and state interface types
  std::vector<std::string> joint_names_;

  // Hardware communication parameters (can be loaded from URDF)
  std::string serial_port_;
  int baud_rate_;
  double timeout_ms_;

  // Store the commands and states
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // Logger for debugging
  rclcpp::Logger logger_{rclcpp::get_logger("CustomHardwareInterface")};

  // Communication helpers (implement based on your hardware)
  bool connect_to_hardware();
  void disconnect_from_hardware();
  bool send_commands_to_hardware();
  bool read_states_from_hardware();
};

} // namespace custom_hardware

#endif // CUSTOM_HARDWARE__CUSTOM_HARDWARE_HPP_
