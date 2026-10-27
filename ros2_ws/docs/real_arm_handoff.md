# 机械臂实物驱动交接文档

最后更新：2026-10-25

## 1. 当前目标

在 ROS 2 Humble 中通过 V3/0x55 串口协议读取 6 个舵机的位置，随后再进行零位标定、限位标定和小范围位置控制。

当前硬件配置：

- 总线波特率：115200
- 舵机 ID：1、2、3、4、5、6
- 舵机模式：位置模式
- 上位机环境：Windows + WSL2
- USB 转换器：CH341，Windows 设备为 COM4，WSL 中显示为 `ttyUSB0` 和 `ttyUSB1`
- 安全状态：`allow_motion=false`，禁止 ROS 驱动发送运动命令

## 2. 软件结构

### 实物启动入口

```text
robot_arm_bringup/launch/real.launch.py
    ├── robot_state_publisher
    ├── robot_arm_driver_node      # 当前为 V3/0x55 驱动
    └── ik_pick_node               # 等待 /arm_controller/follow_joint_trajectory
```

### 当前驱动目标

- `robot_arm_driver_node`
  - 源文件：`robot_arm_driver/src/arm_driver_node.cpp`
  - 串口实现：`servo_bus.cpp`
  - 协议实现：`servo_protocol.cpp`
  - 帧头：`0x55 0x55`
  - 提供：`/joint_states`、`/move_joints`、`/arm_controller/follow_joint_trajectory`
  - 提供 `/emergency_stop` 话题和服务

- `robot_arm_hx_driver_node`
  - 旧 HX/AT32 驱动，仅用于旧协议测试
  - 不要把它用于当前 V3/0x55 控制板

### 配置文件

实物启动默认读取：

```text
robot_arm_driver/config/v3_driver.yaml
```

不要误用 `arm_driver.yaml`、`coordinate_driver.yaml` 或 `ik_driver.yaml`。它们对应旧的 HX、坐标控制或其他实验接口。

## 3. 当前 V3 配置

```yaml
serial_device: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5114366-if00-port0
baud_rate: 115200
allow_motion: false
feedback_poll_period_ms: 200
feedback_timeout_ms: 50
joint_names: [joint1, joint2, joint3, joint4, joint5, gripper]
servo_ids: [1, 2, 3, 4, 5, 6]
```

以下仍是临时值，不能用于正式运动：

```yaml
zero_ticks: [500, 500, 500, 500, 500, 500]
ticks_per_radian: [238.7324, 238.7324, 238.7324, 238.7324, 238.7324, 238.7324]
directions: [1, 1, 1, 1, 1, 1]
min_ticks: [0, 0, 0, 0, 0, 0]
max_ticks: [1000, 1000, 1000, 1000, 1000, 1000]
```

必须确认 ID 6 是夹爪还是第六个机械臂关节。如果是第六个关节，必须同步修改 URDF、IK 链、`joint_names` 和动作轨迹。

## 4. 只读位置验证流程

### 4.1 Windows 侧转发 USB

控制板接在 Windows 时，先关闭串口助手和舵机调试软件，然后在 PowerShell 执行：

```powershell
usbipd list
usbipd detach --busid 4-2
usbipd attach --wsl --busid 4-2
usbipd list
```

确认设备状态为 `Attached`。

### 4.2 WSL 侧确认设备

```bash
lsusb
ls -l /dev/serial/by-id/
readlink -f /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5114366-if00-port0
```

正常时应看到：

```text
usb-1a86_USB_Single_Serial_5CE5114366-if00-port0 -> ../../ttyUSB0
usb-1a86_USB_Single_Serial_5CE5114366-if01-port0 -> ../../ttyUSB1
```

先使用 `if00-port0`。`if01-port0` 是同一个 CH341 设备的另一个接口，不要把两个接口同时交给驱动。

### 4.3 启动只读模式

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_driver robot_arm_bringup
source install/setup.bash
ros2 launch robot_arm_bringup real.launch.py allow_motion:=false
```

预期日志：

```text
V3 servo id=1 joint=joint1 raw_tick=...
...
V3 servo id=6 joint=gripper raw_tick=...
```

另开终端查看：

```bash
source ~/robot_arm_jetson/ros2_ws/install/setup.bash
ros2 topic echo /joint_states
ros2 action list
ros2 service list | grep -E 'emergency_stop|follow_joint_trajectory|move_joints'
```

只读模式下，驱动只发送位置读取帧，不发送位置写入、停止或扭矩使能命令。

## 5. 已知故障与判断

### `cannot open serial device ... REPLACE_WITH_V3_DEVICE`

说明配置仍是占位路径。修改 `v3_driver.yaml` 的 `serial_device`。

### `serial write failed: No such device`

说明设备打开后在 USBIP 层断开，常见原因是 Windows 端程序占用、USBIP 连接断开、USB 线或控制板重置。

### `Input/output error`

说明对应的串口接口不可用或已经失效。不要通过反复切换 `if00`/`if01` 代替 USBIP 重连。

### WSL 日志中的断开证据

曾出现：

```text
vhci_hcd: connection closed
vhci_hcd: disconnect device
ch341-uart ttyUSB0: ... disconnected
ch341-uart ttyUSB1: ... disconnected
```

这属于 USBIP/设备连接问题，不是 URDF、IK 或 KDL 问题。若重连后仍反复出现，优先使用原生 Ubuntu/Jetson 运行串口驱动。

### KDL 根惯性警告

```text
The root link base_link has an inertia specified in the URDF
```

这是 KDL 对根 link 惯性的提示，不会导致串口驱动失败。后续可通过增加 dummy root link 处理，但不属于当前串口排查范围。

## 6. 当前验证结论

- V3 驱动代码已编译并通过现有 3 个单元测试。
- `real.launch.py` 已指向 V3 配置和 V3 驱动。
- USB 设备能够被 WSL 枚举为 CH341 双接口。
- 尚未成功获得 1～6 号舵机的真实 `raw_tick`。
- 当前未验证实物位置控制，`allow_motion` 必须保持 `false`。

## 7. 后续工作顺序

1. 先解决 USBIP/串口稳定性，连续运行只读模式并确认 1～6 号 ID 都能返回位置。
2. 记录每个 ID 的原始位置值，确认 ID 与机械关节的对应关系。
3. 确认 `raw_tick` 的实际单位和有效范围，核对上位机显示的限位值是否为同一单位。
4. 在机械臂固定、急停可用的情况下，逐个标定零位、正方向和软件限位。
5. 更新 `zero_ticks`、`directions`、`min_ticks`、`max_ticks` 和 `ticks_per_radian`。
6. 仅允许一个关节进行极小角度位置测试，再逐步扩大范围。
7. 最后测试 `/arm_controller/follow_joint_trajectory`，再接入 `ik_pick_node` 和 `PickPoint.action`。

任何标定值未确认前，不要把 `allow_motion` 改为 `true`，也不要直接发送轨迹或 IK 抓取动作。
