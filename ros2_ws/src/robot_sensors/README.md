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
