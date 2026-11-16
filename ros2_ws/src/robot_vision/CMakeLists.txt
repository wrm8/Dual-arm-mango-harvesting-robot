cmake_minimum_required(VERSION 3.8)
project(robot_vision)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/mango_vision_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/target_contract.py scripts/harvest_observation.py scripts/dual_arm_contract.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY yolo_mango DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py" PATTERN "*.json" PATTERN "*.yaml"
  PATTERN "tests" EXCLUDE PATTERN "datasets" EXCLUDE
  PATTERN "outputs" EXCLUDE PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES config/camera_to_bases.json DESTINATION share/${PROJECT_NAME}/config)
ament_package()
