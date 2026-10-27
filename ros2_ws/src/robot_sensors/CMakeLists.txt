cmake_minimum_required(VERSION 3.8)
project(robot_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ultrasonic_node
  src/ultrasonic_node.cpp
)
target_include_directories(ultrasonic_node PRIVATE include)
ament_target_dependencies(ultrasonic_node
  rclcpp robot_arm_interfaces sensor_msgs std_msgs)

install(TARGETS ultrasonic_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
