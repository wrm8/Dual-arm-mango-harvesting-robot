cmake_minimum_required(VERSION 3.8)
project(robot_arm_driver)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(servo_protocol src/servo_protocol.cpp)
target_compile_features(servo_protocol PUBLIC cxx_std_17)
target_include_directories(servo_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(servo_bus src/servo_bus.cpp)
target_compile_features(servo_bus PUBLIC cxx_std_17)
target_include_directories(servo_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(servo_bus servo_protocol)

add_library(coordinate_protocol src/coordinate_protocol.cpp)
target_compile_features(coordinate_protocol PUBLIC cxx_std_17)
target_include_directories(coordinate_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(coordinate_bus src/coordinate_bus.cpp)
target_compile_features(coordinate_bus PUBLIC cxx_std_17)
target_include_directories(coordinate_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(planar_arm_kinematics src/planar_arm_kinematics.cpp)
target_compile_features(planar_arm_kinematics PUBLIC cxx_std_17)
target_include_directories(planar_arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot_arm_driver_node src/main.cpp src/arm_driver_node.cpp)
target_compile_features(robot_arm_driver_node PRIVATE cxx_std_17)
target_include_directories(robot_arm_driver_node PRIVATE include)
target_link_libraries(robot_arm_driver_node servo_bus servo_protocol)
ament_target_dependencies(robot_arm_driver_node control_msgs rclcpp rclcpp_action robot_arm_interfaces sensor_msgs std_msgs std_srvs)

add_executable(robot_arm_hx_driver_node src/hx_main.cpp src/hx_arm_driver_node.cpp)
target_compile_features(robot_arm_hx_driver_node PRIVATE cxx_std_17)
target_include_directories(robot_arm_hx_driver_node PRIVATE include)
target_link_libraries(robot_arm_hx_driver_node coordinate_protocol coordinate_bus)
ament_target_dependencies(robot_arm_hx_driver_node control_msgs rclcpp rclcpp_action robot_arm_interfaces sensor_msgs std_msgs std_srvs)

add_executable(robot_arm_coordinate_node src/coordinate_main.cpp src/coordinate_arm_node.cpp)
target_compile_features(robot_arm_coordinate_node PRIVATE cxx_std_17)
target_include_directories(robot_arm_coordinate_node PRIVATE include)
target_link_libraries(robot_arm_coordinate_node coordinate_protocol coordinate_bus)
ament_target_dependencies(robot_arm_coordinate_node rclcpp rclcpp_action robot_arm_interfaces geometry_msgs)

add_executable(robot_arm_ik_node src/ik_main.cpp src/ik_arm_node.cpp)
target_compile_features(robot_arm_ik_node PRIVATE cxx_std_17)
target_include_directories(robot_arm_ik_node PRIVATE include)
target_link_libraries(robot_arm_ik_node servo_bus planar_arm_kinematics)
ament_target_dependencies(robot_arm_ik_node rclcpp rclcpp_action robot_arm_interfaces geometry_msgs sensor_msgs)

install(TARGETS servo_protocol servo_bus coordinate_protocol coordinate_bus planar_arm_kinematics robot_arm_driver_node robot_arm_hx_driver_node robot_arm_coordinate_node robot_arm_ik_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo_protocol test/test_servo_protocol.cpp)
  target_link_libraries(test_servo_protocol servo_protocol)
  target_include_directories(test_servo_protocol PRIVATE include)
  ament_add_gtest(test_coordinate_protocol test/test_coordinate_protocol.cpp)
  target_link_libraries(test_coordinate_protocol coordinate_protocol)
  target_include_directories(test_coordinate_protocol PRIVATE include)
  ament_add_gtest(test_planar_arm_kinematics test/test_planar_arm_kinematics.cpp)
  target_link_libraries(test_planar_arm_kinematics planar_arm_kinematics)
  target_include_directories(test_planar_arm_kinematics PRIVATE include)
endif()

ament_package()
