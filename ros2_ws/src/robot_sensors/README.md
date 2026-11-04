# robot_sensors

`ultrasonic_node` 负责读取连接到 Jetson 的四个 DYP-A21 超声波传感器。当前使用 CH9344 前四个串口：

```text
/dev/ttyCH9344USB0 -> front
/dev/ttyCH9344USB1 -> rear
/dev/ttyCH9344USB2 -> left
/dev/ttyCH9344USB3 -> right
```

设备映射、串口和量程参数在 `config/ultrasonic.yaml`。默认串口参数按产品规格书配置为 115200 baud、8N1、UART 自动输出。UART 受控输出也可启用；驱动每 `controlled_trigger_period_s` 发送一个触发字节，周期必须大于 150 ms。

## 协议

四字节数据帧为 `FF distance_high distance_low checksum`，其中：

```text
checksum = (0xFF + distance_high + distance_low) & 0xFF
raw_distance = distance_high * 256 + distance_low
```

Modbus 寄存器 `0x0209` 决定单位。`distance_unit: mm` 时原始距离单位是毫米；`distance_unit: us` 时按 `raw_distance / 5.75` 换算成毫米。确认传感器设置后再选择对应参数。规格书定义 `0xFFFD` 为无目标、`0xFFFE` 为同频干扰，驱动将它们作为无效值。

## ROS 话题

```text
/ultrasonic/front       sensor_msgs/msg/Range       # 米
/ultrasonic/front_cm    std_msgs/msg/Float64        # 厘米，四舍五入到 0.1 cm
/ultrasonic/rear        sensor_msgs/msg/Range       # 米
/ultrasonic/rear_cm     std_msgs/msg/Float64        # 厘米，四舍五入到 0.1 cm
/ultrasonic/left        sensor_msgs/msg/Range       # 米
/ultrasonic/left_cm     std_msgs/msg/Float64        # 厘米，四舍五入到 0.1 cm
/ultrasonic/right       sensor_msgs/msg/Range       # 米
/ultrasonic/right_cm    std_msgs/msg/Float64        # 厘米，四舍五入到 0.1 cm
/base/ultrasonic        robot_arm_interfaces/msg/UltrasonicArray  # 米，含有效标志
```

无目标、干扰、越界或超时会以 `NaN` 发布到单路 `Range` / `_cm` 话题；聚合消息用相应的 `*_valid` 标志表示有效性。输出四舍五入到 0.1 cm 只是数值分辨率，不代表传感器测量精度；规格书精度指标为 `1 + (S * 0.3%) cm`。

## 单独验证

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sensors --symlink-install
source install/setup.bash
ls -l /dev/ttyCH9344USB*
ros2 run robot_sensors ultrasonic_node --ros-args \
  --params-file install/robot_sensors/share/robot_sensors/config/ultrasonic.yaml
```

另开终端运行 `ros2 topic echo /ultrasonic/front_cm` 等检查四路数值，并逐个遮挡/移动目标确认 0–3 号串口对应的方向。串口权限不足时将运行用户加入 `dialout` 组并重新登录。单独测试时无需启动底盘串口或行为树。

`sensor_msgs/msg/Range` 和 `/base/ultrasonic` 按 ROS 约定使用米；底盘跑垄节点继续消费 `/base/ultrasonic`，因此无需改动其接口。

## 测试命令
# 直行
ros2 action send_goal /follow_row \
robot_arm_interfaces/action/FollowRow \
"{direction: 1, row_id: 0, max_speed_mps: 0.2, front_slow_distance_m: 0.30, front_stop_distance_m: 0.10, use_side_centering: true, side_target_m: 0.50}" \
--feedback

ros2 action send_goal /follow_row   robot_arm_interfaces/action/FollowRow   "{direction: 1, row_id: 0, max_speed_mps: 0.3, front_slow_distance_m: 0.30, front_stop_distance_m: 0.05, use_side_centering: false, side_target_m: 0.05}"   --feedback

ros2 action send_goal /follow_row \
robot_arm_interfaces/action/FollowRow \
"{direction: 1, row_id: 0, max_speed_mps: 0.50, side_target_m: 0.40, front_slow_distance_m: 0.80, front_stop_distance_m: 0.20, use_side_centering: true, reverse_stop_on_zero_velocity: false}" \
--feedback

ros2 action send_goal /follow_row robot_arm_interfaces/action/FollowRow \
"{row_id: 0, stage_id: 1, direction: 1, max_speed_mps: 0.70, side_target_m: 0.05, front_slow_distance_m: 0.30, front_stop_distance_m: 0.05, use_side_centering: true, reverse_stop_on_zero_velocity: false}" \
--feedback
# 横移
ros2 action send_goal /shift_lane \
  robot_arm_interfaces/action/ShiftLane \
  "{direction: -1, distance_m: 0.30, max_speed_mps: 0.05, safety_distance_m: 0.05}" \
  --feedback

ros2 action send_goal /shift_lane \
robot_arm_interfaces/action/ShiftLane \
"{direction: -1, distance_m: 0.50, max_speed_mps: 0.60, safety_distance_m: 0.1, wall_sensor: 1, wall_target_m: 0.40, wall_enable_tolerance_m: 0.40, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" \
--feedback

ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane \
"{stage_id: 0, direction: -1, distance_m: 0.30, max_speed_mps: 0.6, safety_distance_m: 0.05, wall_sensor: 0, wall_target_m: 0.05, wall_enable_tolerance_m: 0.10, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" \
--feedback
# 后退（编码器）
ros2 action send_goal /follow_row \
robot_arm_interfaces/action/FollowRow \
"{direction: -1, row_id: 0, max_speed_mps: 0.2, front_slow_distance_m: 0.30, front_stop_distance_m: 0.10, use_side_centering: false, side_target_m: 0.05}" \
--feedback
编码器后退最后因为最后有一点没走到位，电机一直响，所以我要加一个容差值，然后再把编码器行进距离也给我相应的参数入口，前进，后退横移，都需要，如果不传参数代表正常行走，传参数代表按照编码器距离行走



# 超声波
ros2 topic hz /base/ultrasonic
ros2 topic echo --qos-profile sensor_data /base/ultrasonic
ros2 topic echo --once /base/odom
ros2 topic echo /base/state --field yaw_rad
ros2 topic echo /vision/row_observation

# 命令参数
| 字段 | 当前值 | 作用 |
|---|---:|---|
| `direction` | `1` | 横移方向。`1` 通常是向左，`-1` 是向右 |
| `distance_m` | `0.30` | 横移目标距离，单位米。侧面超声波无效时使用编码器判断 |
| `max_speed_mps` | `0.10` | 最大横移速度，实际作用于 `linear.y` |
| `safety_distance_m` | `0.05` | 侧面距离小于此值时触发安全停止并使 action 失败 |
| `wall_sensor` | `1` | 前后墙距传感器选择：`0` 不用，`1` 前方，`2` 后方 |
| `wall_target_m` | `0.50` | 前方或后方超声波的目标距离，单位米 |
| `wall_enable_tolerance_m` | `0.10` | 开始横移时，只有墙距进入 `0.50 ± 0.10 m`，即 `0.40~0.60 m`，才启用前后墙距修正 |
| `disable_wall_control_on_side_distance` | `true` | 是否在侧面距离达到指定值时取消前后墙距修正 |
| `wall_control_cancel_side_distance_m` | `0.25` | 侧面距离目标，单位米 |
| `wall_control_cancel_side_tolerance_m` | `0.02` | 判断范围。实际侧距在 `0.23~0.27 m` 时触发取消 |
| `--feedback` | - | 在终端持续显示 action 反馈 |

# 15段流程action
1.距离很近
ros2 action send_goal /follow_row robot_arm_interfaces/action/FollowRow "{row_id: 0, stage_id: 1, direction: 1, max_speed_mps: 0.90, side_target_m: 0.05, front_slow_distance_m: 0.80, front_stop_distance_m: 0.10, use_side_centering: true, reverse_stop_on_zero_velocity: false}" --feedback

2.速度太快撞
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 2, direction: -1, distance_m: 0.30, max_speed_mps: 0.9, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback
3.后退，距离大了
ros2 action send_goal /follow_row robot_arm_interfaces/action/FollowRow "{row_id: 0, stage_id: 3, direction: -1, max_speed_mps: 0.50, side_target_m: 0.05, front_slow_distance_m: 0.30, front_stop_distance_m: 0.05, use_side_centering: true, reverse_stop_on_zero_velocity: false}" --feedback
4.左边距离大了一点点，但是没问题，
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 4, direction: 1, distance_m: 0.30, max_speed_mps: 0.6, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback
5.后退速度小了，还是多了一点点
ros2 action send_goal /follow_row robot_arm_interfaces/action/FollowRow "{row_id: 0, stage_id: 5, direction: -1, max_speed_mps: 0.40, side_target_m: 0.05, front_slow_distance_m: 0.30, front_stop_distance_m: 0.05, use_side_centering: true, reverse_stop_on_zero_velocity: false}" --feedback
6.完美不过横移中间有点卡顿
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 6, direction: -1, distance_m: 0.30, max_speed_mps: 0.6, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback
7.比较准，但是慢
ros2 action send_goal /follow_row robot_arm_interfaces/action/FollowRow "{row_id: 0, stage_id: 7, direction: -1, max_speed_mps: 0.35, side_target_m: 0.05, front_slow_distance_m: 0.30, front_stop_distance_m: 0.05, use_side_centering: true, reverse_stop_on_zero_velocity: false}" --feedback
8.
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 8, direction: 1, distance_m: 0.30, max_speed_mps: 0.6, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback
9.
10.
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 10, direction: -1, distance_m: 0.30, max_speed_mps: 0.6, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback
15.
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane "{stage_id: 15, direction: 1, distance_m: 0.30, max_speed_mps: 0.7, safety_distance_m: 0.04, wall_sensor: 1, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02}" --feedback





