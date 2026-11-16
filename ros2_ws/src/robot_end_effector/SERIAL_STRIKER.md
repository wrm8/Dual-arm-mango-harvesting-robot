# STM32 串口击打机构

击打板尚未接入时，用双臂启动参数 `use_motor:=true skip_strike:=true stop_at_target:=false`。机械臂仍执行到位 → 本地等待 `motor_return_delay_s` → 回位，不启动本包串口节点，也不发送旧夹爪命令。整套视觉、底盘、双臂的一键测试入口为 `robot_arm_bringup harvest_system.launch.py`，默认跳过击打，详见 [统一启动说明](../robot_arm_bringup/README.md)。板子接入后显式改为 `skip_strike:=false`。

当前双臂启动使用本方案。`use_motor:=false` 使用原夹爪，`use_motor:=true` 使用独立的 `serial_striker_node.py`。共用原来的坐标转换、四组大小果偏移、IK、关节速度和 native 弯折零位；击打分支不发送旧夹爪命令。Jetson 不再负责 PWM 或编码器计数。

每轮动作是：机械臂到位 → 向独立 STM32 发送一次对应侧触发帧 → 等待 `motor_return_delay_s`（默认 1 秒）→ 机械臂回位。下位机自行转 10 圈、停止、重新等待下一次触发。圈数和转速由下位机决定。上位机只发送、不读取下位机消息；ROS Action 的成功仅表示完整帧写入本机串口且本地等待结束，无法判断下位机是否收到了命令、完成 10 圈或发生堵转。若 10 圈超过 1 秒，按默认设置会边转边回位，延时需按实际动作时间调整。

## 两块相同 STM32 的串口

每块板使用不同串口设备，可以同时通信。不要依赖易变化的 `/dev/ttyUSB0`、`/dev/ttyUSB1`，也不要仅按相同 VID/PID 创建规则，否则两块相同板可能匹配到一起。

在 Jetson 两块板都插上时查看固定 USB 拓扑路径：

```bash
ls -l /dev/serial/by-path/
ls -l /dev/stm32_chassis
```

停掉相关节点，再仅拔插击打板，观察哪条 `by-path` 链接消失/恢复，由此确认击打板对应路径。将其完整路径填入配置 `serial_device`，以后固定插在同一 USB 口。使用具有不同唯一序列号的转串口模块时也可使用 `by-id`，但相同或缺失序列号时不能依赖它。已有底盘配置继续使用 `/dev/stm32_chassis`；如果底盘实际配置用了别的路径，也要同步填写 `chassis_serial_device`。

默认 `/dev/stm32_striker` 是便于识别的别名，程序不会自动创建它。可以直接填真实的 `/dev/serial/by-path/...`，或由已有 udev 规则按不同 `ID_PATH` 创建该别名。不要让两个别名指向同一个 tty。程序会检查这两个配置路径解析后是否是同一串口，并拒绝相同路径；击打端还会申请独占锁。独占锁只防止同样遵循锁的程序，不能替代正确的设备路径配置。

### 当前 Jetson 的固定别名

2026-11-15 已确认两块 STM32 的 USB 唯一序列号不同，使用 [config/99-mango-robot-serial.rules](config/99-mango-robot-serial.rules) 按序列号分别匹配；不能继续保留仅按 `1a86:55d4` 匹配的旧底盘规则，否则它会同时匹配击打板。

| 用途 | USB 序列号 | 固定路径 | 本次枚举设备（可能变化） |
| --- | --- | --- | --- |
| 底盘 STM32 | `5CF0016103` | `/dev/stm32_chassis` | `/dev/ttyCH343USB1` |
| 击打 STM32 | `5C2C070402` | `/dev/stm32_striker` | `/dev/ttyCH343USB2` |

在 Jetson 停掉相关节点后，从 `ros2_ws` 目录安装规则。已有规则先备份，规则文件保留原机械臂适配器配置：

```bash
sudo cp -a /etc/udev/rules.d/99-mango-robot-serial.rules "/etc/udev/rules.d/99-mango-robot-serial.rules.backup.$(date +%Y%m%d-%H%M%S)"
sudo install -o root -g root -m 0644 src/robot_end_effector/config/99-mango-robot-serial.rules /etc/udev/rules.d/99-mango-robot-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match='ttyCH343USB*' --settle
ls -l /dev/stm32_chassis /dev/stm32_striker /dev/robot_arm
```

两个 STM32 别名必须分别指向各自板子。更换板子时需要重新读取序列号并更新规则；只更换 USB 插口或设备编号不需要修改 YAML。此配置修改不需要编译，重启相关节点即可生效。

USB 转串口接线为 TX → 击打 STM32 RX、GND → GND；上位机不接收反馈时可不接 STM32 TX → 适配器 RX。确认 UART 逻辑电平匹配，不能把 RS-232 电平直接接 MCU。若板上 USB 已实现 USB CDC 或 USB 转串口，使用该 USB 口，无需另接 Jetson 排针。底盘和击打板各用各的串口，不共用同一 TX。

## 击打串口协议 v1（后续 STM32 按此实现）

通信参数为 **115200 波特率、8 数据位、无校验、1 停止位（8N1）、无流控**。上位机仅发送，下位机不需要回复。每条命令固定 **6 个原始二进制字节**：

```text
AA 55 01 MASK 0D 0A
```

| 字节位置（从 0 开始） | 内容 | 含义 |
| --- | --- | --- |
| 0～1 | `AA 55` | 固定帧头 |
| 2 | `01` | 单次击打触发命令 |
| 3 | `01` / `02` / `03` | 左 / 右 / 双侧选择位，bit0=左、bit1=右 |
| 4～5 | `0D 0A` | 固定帧尾 |

| 动作 | 完整发送内容（十六进制） |
| --- | --- |
| 只触发左侧 | `AA 55 01 01 0D 0A` |
| 只触发右侧 | `AA 55 01 02 0D 0A` |
| 同时触发左右 | `AA 55 01 03 0D 0A` |

只包含帧头、两字节数据和帧尾，没有长度字段、校验和、序号、应答或停止命令。上位机发送的是 `0xAA` 等实际字节，不是字符串 `"AA 55 ..."`；帧尾已包含在这 6 字节里，不能再额外追加换行。`00` 选择位不是停止命令，不发送；下位机只接受表内的三种帧。

下位机的后续实现约定如下，目前只完成上位机：

1. 从串口字节流查找 `AA 55`，收齐固定 6 字节后检查命令、选择位和帧尾；兼容一帧分多次接收或多帧连续到达。无效帧丢弃并重新找帧头，不能只凭单个 `01` 就启动。
2. 空闲时收到一帧有效命令，只将指定侧设为一次待执行事件；双侧命令只触发左右各一次。
3. 执行所选电机正转 10 圈并停机，清除本次事件，重新等待下一条命令。忙时丢弃新的触发，不排队、不重复启动；上位机也不自动重发。
4. 相同命令在本次执行结束后再次收到，表示新的一次击打；不能永久去重，也不能因触发标志未清除而连续击打。下位机启动和复位时初始为停机/等待状态。

本协议不改变底盘原有串口协议，两套节点各自使用不同串口。

## 上位机串口配置

编辑 [config/serial_striker.yaml](config/serial_striker.yaml)：

```yaml
serial_device: /dev/stm32_striker
chassis_serial_device: /dev/stm32_chassis
baud_rate: 115200
left_command_hex: "AA 55 01 01 0D 0A"
right_command_hex: "AA 55 01 02 0D 0A"
both_command_hex: "AA 55 01 03 0D 0A"
```

协议、波特率和三个触发帧已填写，节点不传配置文件时也默认使用同一协议。现在只需将 `serial_device` 改成实际击打板的固定串口路径；保留已有底盘路径用于冲突检查。双臂同轮只发送一次 `both_command_hex`，不会再额外发送左右帧。以后若修改协议，需同步修改下位机与这些配置；空帧仍拒绝物理启动。

程序不在启动/退出时发送触发，不循环发布标志位，不读取回包，不自动重发。每个新 Action 执行一次串口写入；发送异常或短写会报错。下位机收到每帧后应把触发作为一次事件消费，完成后清除状态，忙时拒绝新触发且不积压，避免同一标志一直保持导致重复转动。

## 构建与只测击打板

在 Jetson 的项目 `ros2_ws` 目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-serial
colcon build --packages-select robot_arm_interfaces robot_end_effector robot_arm_node robot_arm_bringup --symlink-install
source install/setup.bash
```

下位机按 v1 协议实现后，填好串口路径，先不启动机械臂、视觉或整套抓取，只启动串口服务：

```bash
ros2 run robot_end_effector serial_striker_node.py --ros-args \
  --params-file "$PWD/src/robot_end_effector/config/serial_striker.yaml" \
  -p dry_run:=false -p allow_motion:=true
```

另一个加载同样环境的终端，以下命令会发送左侧帧 `AA 55 01 01 0D 0A`，由已实现协议的 STM32 驱动左侧机构转 10 圈，测试前确认机构可转动：

```bash
ros2 action send_goal /end_effector/trigger_strike \
  robot_arm_interfaces/action/TriggerStrike \
  "{arms: [left], wait_s: 1.0}" --feedback
```

右侧改为 `[right]`，双侧改为 `[left, right]`。`wait_s` 只改变上位机何时返回结果，不改变 STM32 运行时间或圈数。发出一次后等实际停转再发下一次；上位机延时结束后不知道下位机是否仍忙。取消 Action 只取消本地等待，不能停下下位机；本方案没有可用的远程停机命令，机构停机由下位机和电源开关处理。

## 与双臂一起启动

先退出独立串口节点，再按 [双臂启动说明](../robot_arm_bringup/README.md) 启动；保留原来的相机与关节标定参数、HOLDING 条件、`targets_topic` 和目标消息，只替换以下参数：

```bash
use_motor:=true \
motor_params:="$PWD/src/robot_end_effector/config/serial_striker.yaml" \
motor_return_delay_s:=1.0 \
stop_at_target:=false
```

切回夹爪仅需 `use_motor:=false`。`stop_at_target:=true` 只到位停住，不启动击打串口节点、不发送触发、不回位，可在击打 STM32 尚未接入时先标定机械臂。`dry_run:=true` 或 `allow_motion:=false` 不打开串口、不发触发。串口节点启动失败时协调节点会拒绝前往击打目标；动作中发送失败则留在目标并报错。由于没有反馈，帧虽写入但下位机未执行的情况无法检测。

旧 PWM 代码保留供历史排查，但双臂启动不再使用。不要把旧的 `pwm_motors.yaml` 传给新串口节点；旧 `/end_effector/run_motors` 和 `/end_effector/stop` 也不能控制这个 STM32。
