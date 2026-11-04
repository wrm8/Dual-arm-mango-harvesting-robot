# 测试命令
ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.5, 1.5, 0.5, 0.0, 0.0], speeds: [100, 200, 700, 200, 5, 5], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"

ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speeds: [100, 200, 700, 200, 20, 20], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"

ros2 action send_goal /pick_point robot_arm_interfaces/action/PickPoint \
"{target: {header: {frame_id: base_link}, point: {x: 0.15, y: 0.0, z: 0.20}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.0, duration: {sec: 2, nanosec: 0}, return_home: false}" \
--feedback
