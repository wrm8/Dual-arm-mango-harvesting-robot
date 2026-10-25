# 机械臂 ROS 2 工作区

该工作区使用 C++ ROS 2 总线舵机驱动，替代 Jetson 侧原有的 Python 示例脚本。
原 ESP32 工程仍保留在 `~/robot_arm` 中且没有被修改；这里只参考它恢复运动范围和旧通信功能。

## 软件包

| 软件包 | 职责 |
| --- | --- |
| `robot_arm_interfaces` | `MoveJoints`、`MoveCartesian` 与 `PickPoint` action 接口。 |
| `robot_arm_driver` | C++ 串口传输、V3 调试板 `0x55` 总线舵机直驱、4 轴解析 FK/IK 与抓取状态机。 |
| `robot_arm_bringup` | Python ROS 2 启动文件。 |

## 代码结构

`robot_arm_driver` 按职责分层，避免节点、串口和协议代码相互嵌套：

| 目录/文件 | 职责 |
| --- | --- |
| `include/robot_arm_driver/*_protocol.hpp` 与 `src/*_protocol.cpp` | 只负责协议常量、帧编码和解码。 |
| `include/robot_arm_driver/*_bus.hpp` 与 `src/*_bus.cpp` | 只负责串口生命周期、读写和 GPIO 方向控制。 |
| `*_arm_node.hpp/.cpp` 与对应的 `*_main.cpp` | ROS 2 action、反馈、急停和节点入口。 |
| `planar_arm_kinematics.hpp/.cpp` | 可独立测试的平面 FK/IK。 |
| `launch/`、`config/` | 启动入口和参数，不在 C++ 中硬编码部署配置。 |

仓库内 C++ 使用 `.uncrustify.cfg` 统一为 2 空格缩进和多行控制流；新增源文件应保持
“头文件声明、源文件实现、入口单独保留”的布局。格式化命令为：

```bash
find src -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
  | xargs -0 uncrustify -c .uncrustify.cfg --no-backup --replace
```

默认启动的是 Jetson 自主 IK 后端，提供 `/move_cartesian` 和 `/pick_point`。它在 Jetson 计算 4 个定位关节的解析逆运动学，随后通过 `0x55` 总线协议将关节目标下发给 V3 调试板。`PickPoint` 执行打开夹爪、预抓取、下探、闭合、抬起和可选回 home。
所有运动时间必须在 20–30000 ms 之间。

## 首次硬件启动

1. 安装 ROS 2；其基础/桌面安装应包含 `rclcpp`、`rclcpp_action`、`sensor_msgs`、`std_msgs` 与 `std_srvs`。
2. 默认配置文件为 `robot_arm_driver/config/ik_driver.yaml`。请改为实际稳定的 `/dev/serial/by-id/` 串口路径。
3. 当前硬件通过 V3 调试板使用 `0x55 0x55` 协议直接控制舵机。先逐个标定舵机 ID、零位、方向、每弧度 tick 数、机械限位与夹爪开闭 tick。
4. 在 `ik_driver.yaml` 填入当前机械臂的 `base_height_m`、`shoulder_offset_m`、`upper_arm_length_m`、`forearm_length_m`、`tool_length_m`。未填写上臂和前臂长度时，节点会拒绝启动。
5. 编译并启动：

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_arm_bringup arm.launch.py
```

填完标定前，先用 `arm_driver.yaml` 的关节直驱节点做单关节小位移测试。填完 IK 参数后，再启动默认节点并用空间点测试：

```bash
ros2 launch robot_arm_bringup joint_test.launch.py
```

```bash
ros2 action send_goal --feedback /move_cartesian robot_arm_interfaces/action/MoveCartesian \
  "{target: {header: {frame_id: base_link}, point: {x: 0.20, y: 0.0, z: 0.20}}, pitch_deg: -90.0, roll_deg: 0.0, claw_position: 500.0, duration: {sec: 2, nanosec: 0}, wait_for_completion: true}"
```

空间点验证通过后，可执行抓取：

```bash
ros2 action send_goal --feedback /pick_point robot_arm_interfaces/action/PickPoint \
  "{target: {header: {frame_id: base_link}, point: {x: 0.25, y: 0.0, z: 0.10}}, approach_height_m: 0.10, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, duration: {sec: 2, nanosec: 0}, return_home: true}"
```

未完成实测标定前，切勿让已上电的机械臂执行全行程运动。

## 首次串口舵机测试

先不要启动 IK。确认调试板被 Jetson 识别后，使用稳定设备名：

```bash
ls -l /dev/serial/by-id/
cd ~/robot_arm_jetson/servo_test_jetson
python3 servo_smoke_test.py --port /dev/serial/by-id/替换为实际设备 --id 1
```

上述命令只读取 ID 为 1 的当前位置，不会运动。确认位置读取正确、机械臂已架空且周围无障碍后，才允许
10 tick 的小位移测试：

```bash
python3 servo_smoke_test.py --port /dev/serial/by-id/替换为实际设备 --id 1 --move --delta 10 --duration-ms 1000
```

该脚本依赖 `pyserial` 与 Jetson GPIO Python 库，且沿用提供示例中 V3 板的半双工方向控制。若读取失败，先检查
电源、舵机 ID、串口设备名，以及 V3 板 RX/TX 方向引脚映射；不要改用原有 `serial_servo_move_demo.py`，它会做大范围往复运动。

## 底盘后续扩展约定

后续底盘应单独做成 `robot_chassis` 软件包，不与机械臂驱动混在一起。四路超声波传感器应发布标准的
`sensor_msgs/Range` 话题：`/ultrasonic/front_left`、`/ultrasonic/front_right`、`/ultrasonic/rear_left`、
`/ultrasonic/rear_right`。直线行驶/PID 控制器使用这些话题和 `geometry_msgs/Twist` 目标，输出
`geometry_msgs/Twist` 到 `/cmd_vel`。只有独立的电机桥节点负责把该命令翻译为电机控制器的串口 PWM 帧。

目前尚未给出 8 路 USB 转串口设备与电机控制器的实际报文格式，因此没有猜测超声波解析规则或 PWM 字节。
建议的可配置端口映射见 `robot_chassis/config/chassis_ports.yaml`；补齐真实传感器和电机协议后，才可启用 PWM 输出。

## echo $AMENT_PREFIX_PATH 
source /opt/ros/humble/setup.bash
reset  AMENT_PREFIX_PATH

## wsl的usb识别问题
1. usbipd list //列出usb设备，在powershell中
2. usbipd bind --busid “要接入wsl的busid”  //在powershell管理员身份打开，成功后 usbipd list 里该设备的 STATE 会从 Not shared 变成 Shared
   lsusb //wsl中列usb
   sudo modprobe cdc_acm  //sudo modprobe 手动加载驱动
   dmesg | tail -n 20
   sudo usermod -aG dialout wrm newgrp dialout //加权限
   groups

   sudo chgrp dialout /dev/ttyACM0
   sudo chmod 660 /dev/ttyACM0
ls -l /dev/ttyACM0
3. usbipd attach --wsl --busid “要接入wsl的busid”  //在powershell中，成功后 STATE 会变成 Attached
   usbipd detach --busid 4-1
   usbipd attach --wsl Ubuntu --busid 4-1
4. ls /dev/ttyUSB* /dev/ttyACM* //wsl中验证
