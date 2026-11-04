# 光电兜底停车与段号接口

上位机保持原有超声波、编码器减速及到位策略。下位机按当前段选择光电，触发后立即把四轮目标速度置零并保持，同时反馈该段到位。上位机收到匹配反馈后持续发零速，确认停稳再完成当前 Action，行为树进入下一段。

## 串口约定

沿用现有 ASCII 文本协议，波特率和四轮速度单位不变。每帧以 `\r\n` 结束，字段用空格分隔。段号是整数文本字段，不是在现有文本中直接插入一个二进制字节。

开启 `base_serial_node.stage_protocol_enabled` 后，上位机发送：

```text
CMD_WHEELS A B C D stage_id\r\n
```

`A/B/C/D` 为四轮有符号 mm/s，顺序为左后、左前、右前、右后。`stage_id=1..15` 表示光电阶段，`0` 表示独立动作或编码器终点动作，不使用光电到位反馈。

下位机增加上报：

```text
STAGE_FB stage_id reached\r\n
```

`reached` 只能是数字 `0` 或 `1`；它表示**本段到位锁存**，不是 GPIO 瞬时电平。下位机继续发送原有 `FB seq period_ms encA encB encC encD yaw_mdeg` 编码器反馈；`STAGE_FB` 不替代 `FB`。

第一段前进、到位、第二段右移示例（轮方向符号配置均为 `1`）：

```text
Jetson -> STM32: CMD_WHEELS 0 0 0 0 1
Jetson -> STM32: CMD_WHEELS 200 200 200 200 1
STM32 -> Jetson: STAGE_FB 1 1
Jetson -> STM32: CMD_WHEELS 0 0 0 0 1
                 ...收到新鲜零速反馈并停稳...
Jetson -> STM32: CMD_WHEELS 0 0 0 0 2
Jetson -> STM32: CMD_WHEELS -100 100 -100 100 2
STM32 -> Jetson: STAGE_FB 2 0
```

## 下位机需要配套实现的规则

基于本次提供的 STM32F4 USART3 代码，已整理 [串口三替换模块与接入步骤](../../../../deployment/stm32_chassis/README.md)。模块保留原 `FB` 反馈，自动追加 `STAGE_FB`；具体光电 GPIO 需按实际接线接入。

1. 收到不同段号时切换本段光电规则并清除上一段到位锁存。反复收到相同段号不能清锁存。
2. 本段未到位时执行上位机四轮速度，保留已有的通信超时停车逻辑。
3. 本段选定的光电有效时立即锁存到位，覆盖四轮目标速度为零。同段后续非零速度、光电松开均不能解除停车。
4. 周期上报当前段号及锁存状态；收到下一段号后，仅判断新段对应的光电。上一段光电保持有效不影响新段。
5. 新段对应光电若已经有效，也应立即锁存停车，不能无条件放行。

上位机同样锁存匹配段的到位反馈，后续 `reached=0` 不解除锁存；段号不匹配、超出范围或格式错误的串口反馈会被丢弃。

## 当前 15 段与光电选择

段号由 `mission_behavior_node` 自动设置为内部路线索引加一，不改变原路线：

| 段号 | 动作 | 下位机光电选择 |
| --- | --- | --- |
| 1 | 前进 | 前方 |
| 2、6、10、14 | 向右横移 | 右侧 |
| 4、8、12 | 向左横移 | 左侧 |
| 3、5、7、9、11、13、15 | 后退 | 依实际后方传感器配置决定 |

目前规划的前、左、右三个光电不覆盖后方。上位机不替下位机指定后退段光电；这些段仍保留原有后退停止策略。没有对应光电的段，下位机应禁用该段光电到位判定。

## ROS 接口与停车完成

- `/base/route_command`：`robot_arm_interfaces/msg/RouteCommand`，同时携带 `stage_id` 和 `velocity`。换段先发送同段零速，再发送运动速度。
- `/base/stage_feedback`：`robot_arm_interfaces/msg/StageFeedback`，包含 `stage_id` 和 `reached`。
- `/base/cmd_vel`：导航节点继续发布供原串口模式及观测使用。开启段号协议后，串口桥只执行 `/base/route_command`，忽略 `/base/cmd_vel`，避免重复发送或混用段号。
- `FollowRow`、`ShiftLane` Goal 新增 `stage_id`。独立 Action 默认 `0`；光电阶段填写 `1..15`；编码器终点动作使用 `0`。

触发后绕过速度斜坡立即发零速。随后要求触发后收到的里程计仍新鲜，`vx/vy/wz` 连续低于已有停车阈值达到 `stop_settle_s`，才返回到位成功。此过程不要求超声波达到目标距离；`photoelectric_stop_timeout_s`（默认 3 秒）内不能确认停稳则 Action 失败，任务不会继续下一段，零速锁存保持。

相同段号的动作重试不会解除光电锁存。重启上位机或重跑任务时，下位机也不能因收到同段号而清锁存；需按实际操作先切换到 `0` 或明确的新段，并确保动作条件满足。

## 编译、启用和观察

Jetson（Humble）工程中运行：

```bash
cd /path/to/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_chassis --symlink-install
source install/setup.bash
```

Action 定义已变化，所有相关客户端和服务端需要重新编译并在新终端加载同一份 `install/setup.bash`。WSL 若使用 Jazzy，对应加载 `/opt/ros/jazzy/setup.bash`。

在 `config/chassis.yaml` 的 `base_serial_node.ros__parameters` 中设置：

```yaml
stage_protocol_enabled: true
```

该开关在节点启动时读取，修改后需重启串口节点。默认 `false` 兼容原下位机；上位机源码完成不代表 STM32 已实现新协议。独立调试发速度时，启用模式需改发 `/base/route_command` 并带段号。

观察下发与到位：

```bash
ros2 topic echo /base/route_command
ros2 topic echo /base/stage_feedback
```

不连接实物的自动测试使用独立 ROS 域、伪终端和模拟传感器，覆盖旧协议、新协议锁存、过期段反馈、前进/横移动作到位、停车确认超时、原超声波到位及完整 15 段：

```bash
colcon test --packages-select robot_chassis --event-handlers console_direct+
colcon test-result --verbose
```
