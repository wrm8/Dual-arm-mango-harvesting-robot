cmake_minimum_required(VERSION 3.8)
project(robot_chassis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(base_serial_node
  src/base_serial_node.cpp
)
target_include_directories(base_serial_node PRIVATE include)
ament_target_dependencies(base_serial_node
  geometry_msgs nav_msgs rclcpp robot_arm_interfaces sensor_msgs)

add_executable(row_navigation_node
  src/row_navigation_node.cpp
)
target_include_directories(row_navigation_node PRIVATE include)
ament_target_dependencies(row_navigation_node
  geometry_msgs nav_msgs rclcpp rclcpp_action robot_arm_interfaces sensor_msgs)

add_executable(mission_behavior_node
  src/mission_behavior_node.cpp
)
target_include_directories(mission_behavior_node PRIVATE include)
ament_target_dependencies(mission_behavior_node
  behaviortree_cpp_v3 geometry_msgs rclcpp rclcpp_action robot_arm_interfaces)

install(TARGETS
  base_serial_node
  row_navigation_node
  mission_behavior_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
