# 五轴机械臂 MoveIt 后端

这个包独立于原来的 `robot_arm_node` 自定义 IK。`robot_arm_bringup` 的 `real_backend.launch.py` 可在启动时选择 `legacy` 或 `moveit`；默认仍是 `legacy`。MoveIt 组只包含 `joint1` 到 `joint5`，不包含当前 URDF 中不存在的夹爪模型。

## 启动与测试

先在已安装 ROS 2 Humble 和 MoveIt 2 的 Jetson 上编译：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_moveit_config robot_arm_bringup
source install/setup.bash
```

先只规划、不执行：

```bash
ros2 launch robot_arm_bringup real_backend.launch.py backend:=moveit allow_motion:=false
ros2 run robot_arm_moveit_config moveit_xyz_goal --ros-args -p x:=0.30 -p y:=0.0 -p z:=0.20
```

命令中的 XYZ 是 `base_link` 下 TCP 目标；它只约束末端位置，不强制指定 yaw。若要控制姿态，可为任意中间关节添加目标约束，例如 joint4 和 joint5：

```bash
ros2 run robot_arm_moveit_config moveit_xyz_goal --ros-args -p x:=0.30 -p y:=0.0 -p z:=0.20 -p constrain_joint4:=true -p joint4_target:=0.2 -p constrain_joint5:=true -p joint5_target:=0.0
```

确认规划结果与现场安全后，才启动允许运动的驱动并执行：

```bash
ros2 launch robot_arm_bringup real_backend.launch.py backend:=moveit allow_motion:=true
ros2 run robot_arm_moveit_config moveit_xyz_goal --ros-args -p x:=0.30 -p y:=0.0 -p z:=0.20 -p execute:=true
```

执行前确保机械臂周围无障碍并准备好急停。测试入口执行的是一次末端定位，不会自动控制夹爪，也不等同于旧 `/pick_point` 的预抓取、闭爪和抬升流程。

## 限制与配置

- 该 5 轴组使用 XYZ 位置目标与可选关节目标约束，不要求完整六维末端姿态；否则会把没有 yaw 电机的问题重新引入求解。
- 现按用户要求关闭了机械臂自身碰撞检查；没有配置外部障碍物避障。
- 启动时 MoveIt 会读取传入的 `params_file`，按 ticks、方向、零点和 URDF 限位动态收紧规划范围。规划速度/加速度限制只用于生成时间参数，不代表舵机实际速度。
- MoveIt 轨迹中的速度、加速度用于规划/时间参数化。舵机驱动仍按 `goal_speed` 下发 HX 速度，不会按每个关节的 MoveIt 速度闭环跟踪；位置反馈仍由驱动用于轨迹点到位判断。
- 旧后端保持原样：`backend:=legacy` 启动现有 `/pick_point` 自定义 IK；`backend:=moveit` 启动 MoveIt 的 `/move_action` 和上述 XYZ 测试入口。
