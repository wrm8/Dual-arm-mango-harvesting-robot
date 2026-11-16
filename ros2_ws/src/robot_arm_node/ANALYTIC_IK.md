# 五轴机械臂解析 IK

实现：`src/motion/analytic_ik.cpp`，入口为
`UrdfKinematics::inverse_xyz_tilt_joint5_analytic()`。
由现有 `ik_pick_node` 调用，保持 `/pick_point` Action 不变。
原多初值数值法保留，可通过 `ik_solver:=numerical` 选择；解析法失败时不会偷偷放宽姿态或限位。

## 坐标与角度

- XYZ 是 `tcp_link` 原点在 `base_link` 中的位置，单位米。
- `target_tool_tilt` 是 TCP **正 Z 轴**与 `base_link` XY 平面的有符号夹角，单位弧度。
  `0` 为水平，正值向上、负值向下，有效范围 `[-pi/2, pi/2]`。
- 它不是 `joint4`，也不是 TF/RPY 中的 pitch。
- `target_joint5` 是第五关节的固定目标角，并不等于任意姿态下的 RPY roll。
- 同一倾角可能有向外/向内的工具方向以及不同弯肘解。本实现枚举这些分支，
  在合法解中选取离当前关节角最近的解；倾角约束本身不保证总是同一种弯肘姿态。

## 从三维降到二维

从当前 URDF 读取每节的 `origin` 和轴向，不写死连杆长度。
适用拓扑为当前 `joint1..joint5` 加固定 `joint_tcp`：底座偏转、三个平行俯仰轴、工具滚转。
若关节顺序、轴向或 TCP 偏置不满足该拓扑，直接报错，不输出未经验证的角度。

1. 减去 joint1 原点，求底座偏转。由于当前 joint1 的轴是 **-Z**，
   忽略侧偏时为 `q1=-atan2(y-y0,x-x0)`；实际代码保留所有固定平移和侧向偏移。
2. 指定工具倾角，反向减去腕部到 TCP 的偏移，得到肩部平面中的腕点。
3. 用两连杆余弦定理枚举弯肘方向，计算肩、肘角。
4. 由目标工具方向反推 joint4，joint5 使用请求角度。
5. 与 URDF 限位、驱动标定限位的交集比较；选择距当前关节角最近的合法解。
6. 用**原始 URDF 的 KDL 正解**核对 XYZ 和工具倾角，任何未通过核对的解都不返回。

二维公式中，`R(a)` 表示平面旋转，`S=q2+q3+q4`：

```text
P = R(-q2) A + R(-q2-q3) B + R(-S) C

A = (joint3.origin.x,  joint3.origin.y)
B = (joint4.origin.x, -joint4.origin.y)
C = (joint5.origin.x,  joint5.origin.y + joint_tcp.origin.z)

TCP_Z 的竖直分量 = cos(S)
S = +/-acos(sin(target_tool_tilt))
```

这里处理了 URDF 中 `Rx(+/-pi)` 引起的符号变化，不能直接套用三节正长度相加。
URDF 的 `1.5708/3.1416` 是近似直角，因此先解析求解，再最多做 8 次有界微调，
补偿这些小数舍入。此微调不做随机多初值搜索。
正解核对门限是位置 `1e-6 m`、倾角 `1e-5 rad`；这是模型内部一致性门限，**不是实物精度承诺**。

## 标定限位

`robot_arm_bringup/launch/real.launch.py` 读取启动时所选 `params_file`，
以名称匹配 joint1..joint5，用与驱动相同的映射换算角度限位：

```text
model_zero = urdf_zero_ticks（非空时），否则 zero_ticks
q = (raw_tick-model_zero)/(direction*ticks_per_radian)
```

分别换算 `min_ticks/max_ticks` 并排序，传入 `ik_lower_limits/ik_upper_limits`。
反向安装的关节也按 `directions` 正确换算。求解器再与 URDF 取交集。
未修改 `zero_ticks`、`urdf_zero_ticks`、`directions` 或 `ticks_per_radian`。
若直接 `ros2 run ... ik_pick_node` 而未提供角度限位参数，只会使用 URDF 限位。

## 编译和启动

在需要运行机械臂的机器上更新源码，然后在该机器编译。仅在 WSL 改代码不会更新 Jetson。
Jetson Humble 示例：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_arm_bringup
source install/setup.bash
ros2 launch robot_arm_bringup real_backend.launch.py \
  backend:=legacy ik_solver:=analytic allow_motion:=false
```

启动后，拿到 `/robot_description` 应显示：

```text
IK solver=analytic; calibrated limits=enabled
```

`allow_motion:=false` 用于启动核对，不代表执行 Action 能成功。
完成零点/方向/限位核对、需要实物动作时，停止旧后端，以 `allow_motion:=true` 重新启动。
不要同时启动另一个占用相同串口的后端。

已有抓取测试命令保持不变：

```bash
ros2 action send_goal /pick_point robot_arm_interfaces/action/PickPoint \
"{target: {header: {frame_id: base_link}, point: {x: 0.35, y: 0.0, z: 0.25}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.0, duration: {sec: 5, nanosec: 0}, return_home: false}" \
--feedback
```

结果日志新增 `tool_tilt=...`，对照请求的倾角；不要用 RPY pitch 来判断水平抓取。
`backend:=moveit` 和 `moveit_xyz_goal` 不走此解析求解器。

## 测试与边界

`test/test_analytic_ik.cpp` 包含原问题目标、600 个完整 URDF 范围正解目标、
200 个标定限位内正解目标、不同弯肘分支、确定性、边界与异常输入测试。
`robot_arm_bringup/test/test_calibrated_ik_limits.py` 检查反向关节、独立模型零位、
名称顺序和错误标定处理。

解析 IK 只解决末端几何逆解，不提供障碍物避让，也不保证关节插值的 TCP 轨迹是直线或由下向上。
本次没有改执行器速度和现有每阶段 2 秒的轨迹时序；当前 `PickPoint.duration` 仍未参与该时序。
模型与实物的零点、方向、计数跨圈、连杆尺寸或 TCP 定义不一致时，换用解析法仍不能消除实物位置误差。
