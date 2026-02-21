cmake_minimum_required(VERSION 3.8)
project(custom_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Compile the hardware interface as a shared library
add_library(${PROJECT_NAME} SHARED
  src/custom_hardware.cpp
)

add_executable(joint1_loop_node
  src/joint1_loop_node.cpp
)

add_executable(joint_state_to_esp32_bridge
  src/joint_state_to_esp32_bridge.cpp
)

target_compile_features(joint1_loop_node PUBLIC cxx_std_17)

ament_target_dependencies(joint1_loop_node
  rclcpp
  std_msgs
)

ament_target_dependencies(joint_state_to_esp32_bridge
  rclcpp
  sensor_msgs
  std_msgs
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface custom_hardware_plugin.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/test_hardware_interface.py
    scripts/monitor_hardware.py
    scripts/send_test_command.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS joint1_loop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS joint_state_to_esp32_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
