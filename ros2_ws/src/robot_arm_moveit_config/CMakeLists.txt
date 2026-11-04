cmake_minimum_required(VERSION 3.8)
project(robot_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/moveit_xyz_goal.py DESTINATION lib/${PROJECT_NAME} RENAME moveit_xyz_goal)

ament_package()
