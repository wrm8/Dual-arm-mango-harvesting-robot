# 测试命令
ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.5, 1.5, 0.5, 0.0, 0.0], speeds: [100, 200, 700, 200, 5, 5], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"

ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speeds: [100, 200, 700, 200, 20, 20], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"
