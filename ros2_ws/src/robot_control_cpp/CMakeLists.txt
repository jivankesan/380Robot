cmake_minimum_required(VERSION 3.8)
project(robot_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# Line follow controller node
add_executable(line_follow_controller_node
  src/line_follow_controller_node.cpp
)
ament_target_dependencies(line_follow_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

# Speed profile node
add_executable(speed_profile_node
  src/speed_profile_node.cpp
)
ament_target_dependencies(speed_profile_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

# Safety stop node
add_executable(safety_stop_node
  src/safety_stop_node.cpp
)
ament_target_dependencies(safety_stop_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  line_follow_controller_node
  speed_profile_node
  safety_stop_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
