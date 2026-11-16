# 双臂末端控制

当前使用 STM32 串口击打方案：双臂启动 `use_motor:=true` 自动启动 `serial_striker_node.py`，到目标后发送一次触发、等待默认 1 秒，再回原位；`false` 保留原夹爪流程。协议配置、双串口固定设备路径、构建与测试见 [STM32 串口击打说明](SERIAL_STRIKER.md)。以下 PWM/编码器内容作为旧方案参考，不再由双臂启动使用；旧参数与停止服务不适用于新 STM32 节点。

## 旧方案：Jetson PWM 与编码器电机

独立控制 Jetson Orin Nano 上的两路电机。默认使用用户提供的《塔克创新 AT8236 双路编码器电机驱动用户手册 V2.0.0》接法；也支持 TB6612FNG。保留原伺服夹爪：双臂启动的 `use_motor:=false` 不启动此节点，继续原夹爪流程。

默认电机流程：机械臂到目标 → 本轮对应电机正转 10 个输出轴圈 → 关闭输出 → 机械臂回 native 弯折零位。电机不反转回起点，每次以启动时的编码器读数为相对起点；断电后的绝对零位不在此功能内。

## AT8236 V2 接线

手册第 4 页标注输入 5.5～15V、每路连续 2A/峰值 6A、PWM 0～100kHz。6A 为峰值，仍需核对电机堵转电流。第 9 页逻辑是 IN1=PWM、IN2=0 时正转快衰减，两个输入为 0 时滑行/休眠。程序默认 20kHz，接近计数目标减速，关闭输出后有惯性滑行，明显超转会报错。

下表全部是 Jetson 40 针排针的物理 **BOARD 针号**，不是 BCM。先断开电机电源再接线。

| Jetson / 外部接线 | AT8236 端子 | 用途 |
| --- | --- | --- |
| 32 | AIN1 | 左电机 PWM |
| 11 | AIN2 | 左电机另一输入，保持 LOW |
| 33 | BIN1 | 右电机 PWM |
| 16 | BIN2 | 右电机另一输入，保持 LOW |
| 12 | E1A | 左编码器 A 输入 |
| 29 | E1B | 左编码器 B 输入 |
| 31 | E2A | 右编码器 A 输入 |
| 15 | E2B | 右编码器 B 输入 |
| 6（GND） | GND | Jetson、驱动板、电机电源共地 |
| 外部 12V 正/负 | VIN 输入 + / − | 电机供电，正负不可反接 |
| 左电机两根电源线 | AO1 / AO2 | 左电机输出 |
| 右电机两根电源线 | BO1 / BO2 | 右电机输出 |

编码器 A/B 与编码器电源/GND 按电机自身定义连接，不能仅凭同形插头插入。确认编码器额定电源电压；支持 3.3V 时可使用板上 3V3 给编码器供电。若编码器要求 5V 且 A/B 输出为 5V，必须经过适当的单向电平转换到 3.3V，再接 Jetson，不能直连。不开编码器模式时，可把编码器线逐根绝缘不接。

AT8236 上的 3V3/5V 是板载电源输出，这里不与 Jetson 电源针相连；不要将 5V、VIN 或 ADC 接入 Jetson GPIO。这份模块手册未给出 AIN/BIN 输入 VIH/VIL 门限，需向卖家确认接受 3.3V 高电平，否则增加合适的电平转换。无需从 Jetson 给 AT8236 另接逻辑电源。可给四个 IN 输入各加约 10kΩ 下拉，保持控制线断开时停机；软件不能保证强杀进程或系统断电后的输出状态。

输出方向相反时交换对应电机的 AO1/AO2（或 BO1/BO2）电源线。编码器计数正负另外由 `encoder_direction_sign` 校准。

## Jetson 准备

使用 ROS 节点实际使用的 `/usr/bin/python3`。JetPack 通常提供 GPIO 库，缺失再安装：

```bash
/usr/bin/python3 -m pip show Jetson.GPIO
sudo apt install python3-jetson-gpio
sudo groupadd -f -r gpio
sudo usermod -aG gpio "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

发行版没有该 apt 包时，按 [NVIDIA Jetson.GPIO 安装说明](https://github.com/NVIDIA/jetson-gpio#installation) 在 ROS 解释器中安装，并复制包内 `99-gpio.rules` 到 `/etc/udev/rules.d/`。加入 gpio 组和安装规则后重新登录，普通权限启动节点。

Jetson.GPIO 只支持硬件 PWM，且不会自动设置引脚复用。Orin Nano 官方开发板的针 32、33 在 NVIDIA 引脚表中支持 PWM；先断开电机电源，运行：

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

按实际 JetPack 菜单为 40 针排针启用针 32、33 的硬件 PWM；AT8236 配置还将针 11、16 设为 GPIO 输出、针 12、29、31、15 设为 GPIO 输入。针 12 原复用 I2S，针 15 有 PWM 复用；按 JetPack/载板支持将其设置为 GPIO。针 29、31 是 GPIO 功能。保存重启。菜单和支持范围以实际载板为准，没有选项时按载板文档设置 pinmux。仅安装 Python 库不足以让 PWM/编码器 GPIO 工作。

## 标定每输出圈脉冲数

在 Jetson 项目的 `ros2_ws` 目录构建并独立启动电机节点，启动后输出保持关闭：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_interfaces robot_end_effector --symlink-install
source install/setup.bash
ros2 run robot_end_effector pwm_motor_node.py --ros-args \
  --params-file "$PWD/src/robot_end_effector/config/pwm_motors.yaml" \
  -p dry_run:=false -p allow_motion:=true
```

另一个已加载环境的终端查看：

```bash
ros2 topic echo /end_effector/encoder_counts
```

`data: [左累计计数, 右累计计数]`，计数从节点启动开始累积。这是 **A 相上升沿 1 倍计数**，B 相仅用于判向；不是 A/B 四倍频。手册或卖家给出每转脉冲数时先确认同一计数口径。如果“7 PPR”指减速前电机轴每圈 A 相 7 个周期，那么每输出圈计数为 `7 × 减速比`，不能仅根据 540 RPM 推算减速比。

减速比未知时，在输出轴做标记。若机构允许手转，断开电机两根动力线、编码器仍按额定电压供电，准确转动输出轴 N 圈，取计数差绝对值除以 N；减速机构不能反拖时不要硬拧，可向卖家查参数，或用下面纯 PWM 模式低速试转并人工核对实际圈数。先检查正转时计数递增，若递减，把该侧 `encoder_direction_sign` 从 `1` 改为 `-1` 后重启。

在 [config/pwm_motors.yaml](config/pwm_motors.yaml) 分别填写：

```yaml
left.counts_per_output_rev: 0.0   # 待标定，替换为实际每输出圈 A 上升沿数
right.counts_per_output_rev: 0.0  # 不知道时保持 0，不要猜测
```

保存后重启节点。默认 `0.0` 仍能读编码器，但拒绝对应侧的圈数 Action；整条抓取链路也会在机械臂前往目标前拒绝。原伺服电机的 tick 零位配置不受影响。

## 单独测试电机

保持上述独立节点运行。已标定后，先左电机低速转 1 圈，再测试 10 圈：

```bash
ros2 action send_goal /end_effector/run_motors robot_arm_interfaces/action/RunMotors \
  "{arms: [left], duty_cycle: 0.5, duration_s: 15.0, use_encoder: true, revolutions: 1.0}" --feedback
```

`revolutions: 10.0` 是 10 圈；`arms: [right]` 测右侧，`[left, right]` 同时测试。每次都使用新的相对起点，两个电机分别到位停转，全部到位并经过短暂稳定等待后才成功。`duration_s` 在编码器模式中是整个动作的超时上限，不是转速。成功结果包含按编码器换算的实际圈数。

`duty_cycle` 是 PWM 上限，必须在 `(0,1]`，`1.0` 为满占空比；临近目标会降低占空比。它不代表精确 RPM，也不保证 0.5 恰为一半转速。可调 `encoder_slowdown_revolutions`（默认最后 0.5 圈减速）、`encoder_minimum_duty`（0.15）、`encoder_tolerance_ticks`（2 个脉冲）、`encoder_settle_s`（0.15 秒）。低占空比可能无法启动。2 秒无新脉冲、反向计数、明显超转、整体超时都会关闭两路输出并报错。

无需编码器的纯 PWM 也保留，便于接线试验/标定，下面运行 1 秒后关闭输出：

```bash
ros2 action send_goal /end_effector/run_motors robot_arm_interfaces/action/RunMotors \
  "{arms: [left], duty_cycle: 0.5, duration_s: 1.0, use_encoder: false}" --feedback
```

纯 PWM 中 `duration_s: 0.0` 表示持续转动，Action 只确认输出已开启便返回成功，不表示已经停止或达到圈数。新命令选择一组完整输出：未列出的侧会停止。`enable_encoders: false` 可不初始化编码器引脚；纯 PWM 正时间单次默认最多 30 秒。

停止两路新电机：

```bash
ros2 service call /end_effector/stop std_srvs/srv/Trigger '{}'
```

停止服务会中止正在等待计圈/计时的电机 Action，阻止该轮继续自动回位；它不停止机械臂。持续模式的 Action 已返回成功，因此此时只关闭电机输出。新 Action 可以重新启动。忙时拒绝第二个 Action；正常退出、取消和报错会尝试关闭输出，硬件故障或强杀进程需要使用驱动板电源开关。

## 接入抓取与精度边界

停止独立 PWM 节点，避免两个进程操作相同引脚。构建和完整启动见 [双臂说明](../robot_arm_bringup/README.md)。在原命令中设置 `use_motor:=true motor_use_encoder:=true motor_revolutions:=10.0 motor_duty_cycle:=1.0 motor_timeout_s:=15.0`，保持原相机/关节标定和 HOLDING 条件。对应侧计圈成功后，机械臂回原弯折零位；不发原夹爪命令。

`motor_use_encoder:=false` 切回纯 PWM，`motor_run_time_s:=0.0` 为持续运行，正数为指定秒数后停转。`use_motor:=false` 保留原夹爪的大小果角度及回位延时松开。`stop_at_target:=true` 只移动到目标，电机不运行；`dry_run:=true` 或 `allow_motion:=false` 不访问物理 GPIO。

编码器改善按时间估算圈数的误差，但 GPIO 回调不是硬件计数器。在 540 RPM 时，每秒 A 上升沿数约为 `counts_per_output_rev × 9`；减速比大、系统同时跑 YOLO 时可能丢脉冲。应在实际满速和计算负载下反复核对标记圈数，不能把软件测试当成圈数精度验证。严格保证高速计数建议把编码器接 STM32 硬件定时器计数，再由 Jetson 下发目标。没有零位开关时，增量编码器不能在断电后自行找回绝对零位。

## TB6612FNG 备用接法

采用 [config/tb6612_motors.yaml](config/tb6612_motors.yaml)，接法与 AT8236 不同：

| Jetson / 外部 | TB6612FNG |
| --- | --- |
| 32 / 11 / 13 | PWMA / AIN1 / AIN2 |
| 33 / 16 / 18 | PWMB / BIN1 / BIN2 |
| 7 | STBY（建议外加下拉） |
| 1（3.3V） | VCC 逻辑电源 |
| 6（GND） | GND，与外部电源共地 |
| 外部 12V | VM，核对所购板额定电压/电流 |
| 左 / 右电机 | AO1、AO2 / BO1、BO2 |

编码器 A/B 仍直接经合适的 3.3V 电平接 Jetson 12/29、31/15。针 7、11、13、16、18 为 GPIO 输出；针 12/15 需设成 GPIO 输入。`left.direction` / `right.direction` 为 `1` 或 `-1`，只在 TB6612 后端有效。启动覆盖 `motor_params:="$PWD/src/robot_end_effector/config/tb6612_motors.yaml"`，独立测试替换 `--params-file` 路径。勿混用两种板的配置与接线。

离线测试仅验证 GPIO 序列、计数方向/减速/超转判断、Action 与双臂流程，不代表 Jetson 接线、pinmux、GPIO 中断性能或实测圈数通过。
