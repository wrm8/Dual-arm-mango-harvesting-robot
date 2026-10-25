# 第一步：工程目标是什么？
已知：我想要末端到达目标位姿T，当前在T0，当前关节角是 q 
求：关节角应该增加多少（Δq），才能让误差变成 0？
所以，从工程角度看，我们想要得到的东西就是 Δq,准确的说是很小的Δq1累加
# 
把非线性方程，变成了线性方程：现在，方程变成了极其优美的 
JxΔq=T-T0
# 雅可比以及矩阵讲解
T-T0由x,y,z,pitch,roll组成，终点位置由每个电机转动的角度综合影响，所以可以看成x = q1+q2+q3+q4+q5
所以对每个电机转角求偏导，所以是5x5
所以行代表一个参数对每个角度求得偏导，列代表位姿的5个参数，对一个角度求偏导
最终策略，每次只转一个电机，也就是5个参数对一个角度求偏导，求出来列，然后一共求出来5列，拼成雅可比

# 自我理解
整个ik反解是非线性的，非线性没法反解Δq，但是在局部可以用泰勒一阶展开，近似为线性的，然后加上循环周期特性，就实现了，整体逐步走完所有的角度变化。所以整体流程就是，
1.获取当前关节角：编码器读出当前角度（ros），因为有多个电机，就是一个5行一列的向量KDL::JntArray q = to_kdl(seed);（转换为kdl）
2.获取当前位姿：然后通过kdl自带的函数。fk正运算根据当前角度值算出当前位姿，KDL::Frame current_frame; 
3.算出位姿差：根据传入的目标位姿算出位姿差，const auto error = task_error。
4.求雅可比（求导）：重头戏，定义5x5雅可比，循环5次，每次求5个参数对一个角度求偏导的列向量，需要5个参数误差，和一个角度误差，所以循环条件看角度。每一个角度都加上了步长（专门用于求导数，很小，不实际走），然后算出加了步长之后的角度的位姿，然后求出位姿差，然后中心差分法求导
5.求转动角度：通过算出起始点的雅可比，还是假设那条直线，算出应该走多少度，然后有限幅，又循环迭代，次数大于循环次数就结束，误差小于容差也结束

## 仿真运动示例
ros2 action send_goal --feedback /pick_point \
  robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.15, y: 0.0, z: 0.2}}, approach_height_m: 0.00, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_pitch: 1.0, target_roll: 0.5, duration: {sec: 2, nanosec: 0}, return_home: true}"

ros2 action send_goal --feedback /pick_point \
  robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.3, y: 0.0, z: 0.3}}, approach_height_m: 0.00, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_pitch: 1.5, target_roll: 1.5, duration: {sec: 2, nanosec: 0}, return_home: true}"
# 位置抓取，需要ik解算
ros2 action send_goal --feedback /pick_point robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.3, y: 0.0, z: 0.25}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.0, duration: {sec: 2, nanosec: 0}, return_home: false}"

ros2 action send_goal --feedback /pick_point robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.1, y: 0.0, z: 0.15}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: -0.5, target_joint5: 1.0, duration: {sec: 2, nanosec: 0}, return_home: false}"

ros2 action send_goal --feedback /pick_point robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.1, y: 0.0, z: 0.15}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: -0.5, target_joint5: 0.0, duration: {sec: 2, nanosec: 0}, return_home: false}"

# 角度回来，不需要ik解算
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'], points: [{positions: [0.0, -0.5, 3, -0.5, 0.5], time_from_start: {sec: 3, nanosec: 0}}]}}"