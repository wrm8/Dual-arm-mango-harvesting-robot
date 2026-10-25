cmake_minimum_required(VERSION 3.8)
project(robot_arm_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# 1. 找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)    
find_package(robot_arm_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 先编译你的底层运动学库（这是地基）
add_library(robot_arm_motion
  src/motion/gravity_model.cpp
  src/motion/jacobian.cpp
  src/motion/joint_limits.cpp
  src/motion/kinematics.cpp
  src/motion/urdf_kinematics.cpp
  src/motion/planning.cpp
  src/motion/trajectory.cpp
)

target_include_directories(robot_arm_motion PUBLIC
  ${orocos_kdl_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_arm_motion ${orocos_kdl_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(robot_arm_motion kdl_parser urdf)

# 3. 编译你的第一个节点（pick_sequence_node）
add_executable(pick_sequence_node
  src/control/pick_sequence_node.cpp
)
target_link_libraries(pick_sequence_node robot_arm_motion)
ament_target_dependencies(pick_sequence_node
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  robot_arm_interfaces
)

# 4. 编译你的第二个节点（ik_pick_node）—— 真正需要被修好的！
add_executable(ik_pick_node
  src/control/ik_pick_node.cpp
)
target_link_libraries(ik_pick_node robot_arm_motion)
ament_target_dependencies(ik_pick_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  robot_arm_interfaces
)

# 5. 统一安装规则（非常关键，决定了 ros2 run 能不能找到）
install(TARGETS 
  robot_arm_motion 
  pick_sequence_node 
  ik_pick_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

# 6. 测试（如果有的话）
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion test/test_motion.cpp)
  target_link_libraries(test_motion robot_arm_motion)
endif()

ament_package()