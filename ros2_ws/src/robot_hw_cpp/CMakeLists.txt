cmake_minimum_required(VERSION 3.8)
project(robot_hw_cpp)

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

# Serial bridge node
add_executable(serial_bridge_node
  src/serial_bridge_node.cpp
)
ament_target_dependencies(serial_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

# Dummy HW node for development
add_executable(dummy_hw_node
  src/dummy_hw_node.cpp
)
ament_target_dependencies(dummy_hw_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  serial_bridge_node
  dummy_hw_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
