# robot_chassis

底盘任务层由四个节点组成：

- `base_serial_node`：把 `/base/cmd_vel` 的 `vx/vy/wz` 发送给 STM32，并发布 `/base/odom`、`/base/imu` 和 `/base/state`。
- `robot_sensors/ultrasonic_node`：读取四个 DYP-A21 UART 传感器并发布 `/base/ultrasonic`；协议和传感器配置见 [robot_sensors/README.md](../robot_sensors/README.md)。
- `row_navigation_node`：通过 `FollowRow`、`ShiftLane`、`StopBase` Action 读取超声波和里程计，计算底盘速度；不处理 PWM。
- `mission_behavior_node`：BehaviorTree.CPP 任务调度。芒果目标稳定后优先执行停车、`/pick_point` 抓取，再恢复当前垄道阶段。

## 安装依赖

```bash
sudo apt update
sudo apt install ros-humble-behaviortree-cpp-v3
```

## 编译和启动

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_sensors robot_chassis robot_arm_bringup --symlink-install
source install/setup.bash
ros2 launch robot_chassis chassis.launch.py
```

只构建超声波驱动：

```bash
colcon build --packages-select robot_sensors --symlink-install
source install/setup.bash
```

直接启动 `chassis.launch.py` 默认按真机读取串口；仿真请传 `serial_enabled:=false`。顶层 `competition.launch.py use_sim:=true/false` 会自动选择这两个模式。

统一启动仿真和底盘任务：

```bash
ros2 launch robot_arm_bringup competition.launch.py use_sim:=true
```

实物模式仍然默认安全关闭：`dry_run: true`、`allow_motion: false`。完成串口协议、急停、方向、零位和限位标定后，才可以在参数文件中逐项打开。

视觉输入：`/vision/mango_target`，消息类型为 `robot_arm_interfaces/msg/MangoTarget`。摄像头节点尚未加入；消息中的 `center` 必须已经转换到 `base_link` 坐标系，才能直接传给机械臂 `PickPoint` Action。

## STM32 临时协议

当前 Jetson 侧预留文本协议：

```text
CMD_VEL vx vy wz\n
IMU yaw gyro_z\n
ODOM x y yaw vx vy wz\n
```

这只是占位协议。实际 V3/STM32 帧格式确定后，只替换 `base_serial_node` 的编码和解析，不改变行为树或跑垄 Action 接口。
