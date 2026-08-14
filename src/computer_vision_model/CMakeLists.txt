cmake_minimum_required(VERSION 3.8)
project(computer_vision_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(rgbd_capture_node src/rgbd_capture_node.cpp)
ament_target_dependencies(rgbd_capture_node rclcpp sensor_msgs std_srvs)
target_link_libraries(rgbd_capture_node ${OpenCV_LIBS})
target_compile_features(rgbd_capture_node PUBLIC cxx_std_17)

add_executable(synthetic_rgbd_publisher src/synthetic_rgbd_publisher.cpp)
ament_target_dependencies(synthetic_rgbd_publisher rclcpp sensor_msgs)
target_compile_features(synthetic_rgbd_publisher PUBLIC cxx_std_17)

install(TARGETS
  rgbd_capture_node
  synthetic_rgbd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fruit_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fruit_detection_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
