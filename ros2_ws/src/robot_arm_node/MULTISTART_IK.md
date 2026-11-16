# 直接 IK 多初值重试

`UrdfKinematics::inverse_xyz_tilt_joint5()` 现在默认最多尝试 16 组数值初值。
`ik_pick_node` 的现有三个调用会自动使用多初值，无须修改 `/pick_point` 命令或 Action 接口。

1. 先使用传入的当前关节角或上一阶段的解；成功时直接返回，保持原有连续性。
2. 仅在不收敛时，尝试弯曲参考姿态、相反弯曲分支、限位中点和确定性的分布采样。
3. 将初值约束在现有 URDF 限位内，第五轴始终固定为请求值。
4. 从成功的回退结果中，选择前四轴相对原初值的角度差平方和最小的解。
5. 每组最多更新 200 次（沿用构造参数）；全部失败时报告尝试次数以及首尾初值的失败原因和残差。

数值初值只参与内存中的求解，不会让机械臂先移动到参考姿态。
TCP 正 Z 轴相对水平面的倾角定义、1 mm 位置容差、0.01 rad 倾角容差保持原样。
多初值提高收敛机会，不保证任何目标均可达，也不保证完整末端姿态或无碰撞。
限位仍来自传入的 URDF；本次没有把 `arm_driver.yaml` 的标定限位接入 IK，驱动的标定限位检查继续生效。
本次没有修改速度、轨迹时间、夹爪动作、机械零点或 MoveIt。

## C++ 调用

现有六个参数的调用会默认启用最多 16 组初值。可选的第七个参数为总尝试上限，范围 1～64：

```cpp
auto q = solver.inverse_xyz_tilt_joint5(seed, x, y, z, tool_tilt, joint5);       // 默认 16 组
auto q = solver.inverse_xyz_tilt_joint5(seed, x, y, z, tool_tilt, joint5, 32U); // 最多 32 组
auto q = solver.inverse_xyz_tilt_joint5(seed, x, y, z, tool_tilt, joint5, 1U);  // 原单初值行为
```

这不是 ROS 参数；`ik_pick_node` 默认使用 16 组。重复调用在相同模型、目标和初值下采用相同的初值序列。

## 构建与测试

在 Jetson 工程已同步这些文件后，使用新终端构建：

```bash
source /opt/ros/humble/setup.bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
colcon build --symlink-install --packages-up-to robot_arm_node
source install/setup.bash
colcon test --packages-select robot_arm_node --event-handlers console_direct+
colcon test-result --verbose
```

新增 `test_urdf_kinematics` 对用户报错坐标、单初值失败/多初值恢复、第五轴固定、确定性、限位和无效输入做离线回归。
测试使用相邻 `robot_arm_description` 源码包中的真实 URDF，不打开串口或发送动作。
部署后需要重启现有 legacy 节点才能加载新库。`/pick_point` 是执行接口，求解成功后会发送真实轨迹。
