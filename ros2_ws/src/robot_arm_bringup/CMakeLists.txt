cmake_minimum_required(VERSION 3.8)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/startup_home.py DESTINATION lib/${PROJECT_NAME})
install(FILES MANUAL_ARM_CONTROL.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibrated_ik_limits test/test_calibrated_ik_limits.py)
  ament_add_pytest_test(test_dual_arm_description test/test_dual_arm_description.py)
  ament_add_pytest_test(test_grasp_profile_launch test/test_grasp_profile_launch.py)
  ament_add_pytest_test(test_harvest_system_launch test/test_harvest_system_launch.py)
endif()

ament_package()
