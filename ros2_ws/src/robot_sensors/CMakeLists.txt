cmake_minimum_required(VERSION 3.8)
project(robot_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ultrasonic_node
  src/ultrasonic_node.cpp
)
target_include_directories(ultrasonic_node PRIVATE include)
ament_target_dependencies(ultrasonic_node
  rclcpp robot_arm_interfaces sensor_msgs std_msgs)

add_executable(wit_imu_node src/wit_imu_node.cpp)
target_include_directories(wit_imu_node PRIVATE include)
ament_target_dependencies(wit_imu_node rclcpp rcl_interfaces sensor_msgs std_msgs)

install(TARGETS ultrasonic_node wit_imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_wit_imu_protocol test/test_wit_imu_protocol.cpp)
  target_include_directories(test_wit_imu_protocol PRIVATE include)
  add_test(NAME wit_imu_protocol COMMAND test_wit_imu_protocol)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(wit_imu_serial test/test_wit_imu_serial.py TIMEOUT 45
    APPEND_ENV MANGO_TEST_WIT_IMU=$<TARGET_FILE:wit_imu_node>)
endif()

ament_package()
