# robot_chassis

底盘任务层由五个节点组成：

- `base_serial_node`：把 `/base/cmd_vel` 的 `vx/vy/wz` 发送给 STM32，并发布 `/base/odom`、`/base/imu` 和 `/base/state`。
- `robot_sensors/ultrasonic_node`：读取四个 DYP-A21 UART 传感器并发布 `/base/ultrasonic`；协议和传感器配置见 [robot_sensors/README.md](../robot_sensors/README.md)。
- `robot_sensors/wit_imu_node`：读取 HWT901B-TTL，发布 `/imu/wit/data`，为导航提供独立航向。
- `row_navigation_node`：通过 `FollowRow`、`ShiftLane`、`StopBase` Action 读取超声波、编码器里程计和维特航向，计算底盘速度；不处理 PWM。
- `mission_behavior_node`：BehaviorTree.CPP 任务调度。当前视觉测试模式在指定横移段停车并保持；自动摘取分支默认关闭。

## 导航航向已切换为维特 IMU

前进、后退、横移的 `yaw_kp` / `yaw_ki` 航向保持都从 `/imu/wit/data`
（`sensor_msgs/Imu`，`SensorDataQoS`）取偏航角。参数在 `config/chassis.yaml`：

```yaml
row_navigation_node:
  ros__parameters:
    use_external_imu: true
    external_imu_topic: /imu/wit/data
    imu_timeout_s: 0.30
```

这三个参数在启动时读取。无 IMU、姿态无效、时间戳过旧/异常，或超过 `imu_timeout_s` 无新帧时，
导航拒绝输出运动或中止正在运行的动作并发送零速；不会自动切回 STM32 航向。
前进动作最多等候传感器 3 秒，横移动作要求发送目标前数据已就绪。`StopBase` 不依赖 IMU 才能下发停车。

STM32 固件、`base_serial_node` 反馈解析和 `/base/odom` 位置/速度均保留。
编码器行程、距离投影、零速停车确认继续使用原里程计；它的 XY 积分和姿态仍属于原 STM32 航向链路，
没有把维特的不同零点写进旧里程计。`/base/imu`、`/base/state` 中的旧航向仍可供对比，
导航角度控制不再使用它们。

维特串口配置为已验证的 `/dev/ttyCH341USB0`、`115200`，在 `robot_sensors/config/wit_imu.yaml` 中修改。
`zero_yaw_on_start: true` 让每次维特节点启动取零，任务开始时再捕获航向保持参考。
任务运行期间不要单独重启/归零 IMU；若发生重启，停止任务并重启导航节点后重新开始。
平放安装时先验证小车逆时针旋转对应新偏航角增大；若相反，在维特配置中改 `yaw_sign`，再重启两个节点。

### 单独启动测试

在 Jetson 的 `ros2_ws` 目录先同步源码、编译并重新 source：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_chassis robot_sensors --symlink-install
source install/setup.bash

# 终端 1：先启动维特节点；可单独静置观察，不会发运动命令。
ros2 run robot_sensors wit_imu_node --ros-args \
  --params-file src/robot_sensors/config/wit_imu.yaml

# 终端 2：启动导航；原底盘串口和超声波节点仍按原命令运行。
ros2 run robot_chassis row_navigation_node --ros-args \
  --params-file src/robot_chassis/config/chassis.yaml

# 另一个终端：确认订阅和源配置，再按原 FollowRow/ShiftLane Action 测试。
ros2 node info /row_navigation_node
ros2 param get /row_navigation_node use_external_imu
ros2 param get /row_navigation_node external_imu_topic
ros2 topic echo /imu/wit/yaw_deg --qos-profile sensor_data
```

启动日志应显示 `navigation heading source: /imu/wit/data; encoder position/velocity: /base/odom`。
独立测试确认方向后，再从低速测试原动作；`yaw_kp` / `yaw_ki` 仍是原来的航向控制增益。

### 整体启动与仿真

`ros2 launch robot_chassis chassis.launch.py` 现在会一并启动维特节点，不要再另开同名节点占用串口。
可用 `imu_params_file:=/绝对路径/wit_imu.yaml` 指定 IMU 配置。
`serial_enabled:=false` 默认同时关闭维特并使用原里程计航向，供既有仿真流程使用；
如需组合模式，可显式传 `use_external_imu:=true/false`。
只有主动选择 `use_external_imu:=false`（单独节点使用 `-p use_external_imu:=false`）才恢复旧航向。

`photoelectric_integration` 的模拟测试覆盖新航向对前进/后退/横移的影响、旧航向不再参与角度控制、
IMU 缺失/坏姿态/超时/重放帧时零速中止，以及原编码器行程和光电/视觉停车逻辑。

## STM32 航向微小变化过滤（保留的旧链路）

以下 `base_serial_node` 参数只影响原 STM32 反馈、里程计和对比话题，
不会对 `/imu/wit/data` 的导航航向进行死区处理。

`config/chassis.yaml` 中死区 `imu_yaw_increment_deadband_rad` 默认 `0.0001 rad/反馈帧`，延时参数 `imu_yaw_increment_deadband_startup_delay_s` 默认 `5.0 s`。延时从 `base_serial_node` 启动时开始计时；前 5 秒只关闭死区过滤，IMU 数据仍正常接收、发布和参与计算。延时结束后，在下一帧有效 yaw 到达时启用过滤。串口 `FB` 的 `yaw_mdeg` 先转换为弧度，再比较相邻两帧原始 yaw：变化绝对值小于阈值就丢弃该次增量，正负方向都处理；被丢弃的值不会在后续帧累积补回。跨越 ±π 时按最短角度处理。`/base/state`、`/base/wheel_state`、`/base/imu`、`/base/odom` 统一使用过滤后的航向和角速度，里程计 XY 积分也使用同一航向。旧的 `IMU` / `ODOM` 输入同样应用该过滤。
**注意：**如果复位的是 STM32，而 `base_serial_node` 没有重启，这个启动延时不会重新计时；该配置对应“重启底盘 ROS 节点后等待 5 秒”。

`0.0001 rad` 约 `0.00573°`，这是每帧角度阈值，不是 rad/s。它能屏蔽微小漂移，也会屏蔽真实慢转：例如 20 Hz 反馈时，持续每秒小于约 `0.002 rad` 的转动可能被忽略。它不能替代 STM32 的陀螺零偏标定，也不能消除较大的漂移或机械抖动。

调整 YAML 后重启底盘节点。设为 `0.0` 关闭过滤；也可启动单个桥接节点时显式覆盖（先停止原来的桥接节点，避免串口冲突）：

```bash
ros2 run robot_chassis base_serial_node --ros-args \
  --params-file "$PWD/src/robot_chassis/config/chassis.yaml" \
  -p imu_yaw_increment_deadband_rad:=0.0
```

仅静态观察时运行 `ros2 topic echo /base/state --field yaw_rad`。离线测试 `yaw_increment_filter` 和 `yaw_feedback` 使用模拟数据和虚拟串口，不向真实底盘下发命令。

## 第一段进入垄道后启用左超声波调节

路线第 1 段（`FollowRow.stage_id: 1`、前进、启用左侧测距调节）开始时不输出左侧横向修正。左侧有效正距离经过现有中值滤波后，**小于 `0.10 m`** 才启用左侧调节；刚好等于阈值不会触发。启用后在该次行驶动作内保持开启，即使距离稍后大于阈值也继续修正，避免在垄道边缘反复开关。新动作重新等待进入条件。其他路段和右侧测距调节保持原逻辑。

在 `config/chassis.yaml` 的 `row_navigation_node.ros__parameters` 下修改：

```yaml
first_row_left_enable_distance_m: 0.10  # 10 厘米，进入垄道的启用阈值
```

也可运行时调整：`ros2 param set /row_navigation_node first_row_left_enable_distance_m 0.10`。阈值必须为正数。未启用左侧调节期间，前进速度、IMU 航向保持、前方减速/停车、光电停车和传感器超时停车仍然生效。启用阈值不是贴边目标距离；目标仍由任务层的 `row_side_target_m` 控制，当前为 `0.05 m`。

进入条件满足时会打印 `stage 1: left side correction enabled ...`。`photoelectric_integration` 离线测试通过模拟超声波和里程计检查进入前/后的速度输出，不连接真实底盘。

## 安装依赖

```bash
sudo apt update
sudo apt install ros-humble-behaviortree-cpp-v3

rosdep install --from-paths src --ignore-src -r -y #一键安装依赖
```
###### #################################################
## 编译和启动

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_sensors robot_chassis robot_arm_bringup --symlink-install
source install/setup.bash
ros2 launch robot_chassis chassis.launch.py
```
###### #################################################
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

实物模式的串口访问和运动开关由 `dry_run`、`allow_motion` 控制，当前配置见 `config/chassis.yaml`；光电段号协议另外由 `stage_protocol_enabled` 控制。

视觉输入为 `robot_vision` 发布的 `/vision/row_observation`，停车确认通过 `/base/vision_stop_state` 返回。停稳选果输出为 `/vision/harvest_targets`，详见下文。原 `/vision/mango_target` 单果自动摘取链路仅在显式开启兼容选项后使用。

## STM32 串口协议

当前桥接节点发送四轮目标速度，单位为有符号整数 mm/s；轮序为 A 左后、B 左前、C 右前、D 右后：

```text
CMD_WHEELS A B C D\r\n
FB seq period_ms encA encB encC encD yaw_mdeg\r\n
```

`FB` 为下位机上报的编码器增量及航向反馈，继续保留。光电兜底增加段号和到位反馈，完整约定及启用方法见 [光电停车接口](docs/photoelectric_protocol.md)。

```text
CMD_WHEELS A B C D stage_id\r\n
STAGE_FB stage_id reached\r\n
```

`stage_protocol_enabled` 默认 `false`，继续使用原四轮协议；与下位机实现一致后再设为 `true`。段号与速度一起发送，到位锁存和新段切换规则见上述文档。
 ## 运行命令
 ros2 run robot_chassis row_navigation_node   --ros-args   --params-file install/robot_chassis/share/robot_chassis/config/chassis.yaml

ros2 run robot_sensors ultrasonic_node   --ros-args   --params-file install/robot_sensors/share/robot_sensors/config/ultrasonic.yaml

ros2 run robot_chassis base_serial_node \
  --ros-args \
  --params-file install/robot_chassis/share/robot_chassis/config/chassis.yaml

ros2 launch robot_chassis chassis.launch.py

cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision
bash tools/run_jetson.sh \
  weights:="$PWD/models/best.pt" \
  source:=realsense \
  device:=0 \
  publish_targets:=true

python3 vision.py \
  --weights ../models/best.pt \
  --source realsense \
  --device 0 \
  --show
bash tools/run_jetson.sh \
  publish_targets:=true \
  weights:="$PWD/models/best.pt" \
  calibration:="$PWD/config/camera_to_base.json"

bash tools/run_jetson.sh \
  publish_targets:=true \
  weights:="$PWD/models/best.pt" \
  calibration:="$PWD/config/camera_to_base.json" \
  decision_config:="$PWD/yolo_mango/configs/vision.json"

ros2 run rqt_reconfigure rqt_reconfigure
# 横移时视觉减速、树干居中和停稳检查

当前默认测试模式在树干前停稳并保持，不自动调用机械臂，也不推进到下一路线步骤。
路线第 2、15 步关闭视觉停车；第 4、6、8、10、12、14 步打开。
`mission_behavior_node` 的 `shift_vision_stop_step_1..8` 按横移次数对应路线
2、4、6、8、10、12、14、15。每段发出新 goal 时读取参数。
`legacy_mango_interrupt_enabled: false` 关闭旧的单果触发摘取分支。

移动中，RealSense 检测到任意一个 0.40–0.60 m 内的芒果便触发直接低速
（上限 `vision_tree_max_speed_mps: 0.12`），不要求先看到四个果实。
同一深度区间也独立筛选树干，含 0.40 m 和 0.60 m 边界。RGB 输入没有深度，不触发视觉运动。

导航继续作为唯一速度控制者。找到有效树干后，以
`(树干框中心 X - 画面中心 X) / 画面宽度` 为误差，通过 PI 控制横移；
正值代表树干在画面右侧，负值代表在左侧。这个居中误差直接来自 YOLO 的旋转后图像，
**不需要相机到机械臂 base_link 的标定，也不要求底盘原点对准树干**。
相机到机械臂的外参仅供抓取目标坐标变换使用。进入画面宽度 ±2% 的容差后发零速，
随后等待实测零速并保持。

减速寻树（SEARCHING）和树干对中（ALIGNING）期间，继续使用当前横移段的超声波调距：
沿用该段选择的前侧/后侧传感器、目标距离、启用/取消条件、`shift_front_kp` 和
`max_front_correction_mps`，前后修正方向仍随传感器选择切换。该距离控制保持原有 P 控制，
与树干图像横移 PI、IMU 航向控制同时工作。进入停稳确认（SETTLING）或等待抓取（HOLDING）后，
停止调距并输出零速；收到机械臂回位完成、恢复横移后，再按原条件恢复调距。

`vision_tree_control_sign` 决定画面偏差映射到底盘 Y 轴的方向。首次测试前必须在
抬轮/禁用驱动的安全状态下验证命令符号，实际控制方向相反时改为 `-1.0`；
方向未经验证不要让车辆接触树干测试。

`chassis.yaml` 的 `row_navigation_node` 参数：

| 参数 | 用途 |
|---|---|
| `vision_tree_kp` / `vision_tree_ki` | 树干图像误差 PI；误差是画面宽度比例，输出为横移速度 |
| `vision_tree_integral_limit` | 图像比例误差的积分限幅 |
| `vision_tree_max_speed_mps` | 对准树干时最大横移速度，默认 0.12 m/s |
| `vision_tree_center_tolerance_ratio` | 图像居中误差容差比例，默认 0.02（画面宽度的 ±2%） |
| `vision_stop_settle_s` | 指令和新鲜里程计的 x/y/角速度均接近零后持续确认时间 |
| `vision_observation_timeout_s` | 相机采样时间及接收时间的最大允许延迟 |
| `vision_tree_search_distance_m` | 未识别到树干前的制动搜索距离上限 |
| `vision_tree_alignment_timeout_s` | 对准树干的最长时间，超时跳过该树继续当前横移段（此时机械臂尚未动作） |
| `vision_tree_control_sign` | 实测方向修正，正常为 1.0，可设 -1.0 |

以上 `vision_*` 参数支持动态调整。相机过期、对准时丢树干、侧向安全距离和
光电到位均优先停车；它们不会被选果逻辑覆盖。进入 HOLDING 后持续发零速，
不因果实消失或参数关闭而自行恢复。

双臂节点发送匹配 `stage_id`、`stop_id` 的 `/harvest/result`，且确认成功回位后，导航解除本次 HOLDING 并继续原横移段；错误、dry-run、目标点停住测试、旧停车结果均不放行。双臂自动模式最多尝试抓取 10 秒，超时或 IK 失败后回位、跳过目标，回位成功才通知续行。无目标快照同样在 10 秒后执行回位。回位另有超时上限，故不承诺总停车恰好不超过 10 秒。

同一采摘横移段可连续停靠两棵或更多棵树，每棵树使用独立 `stop_id`。`vision_rearm_distance_m`（默认 0.10 m）、`vision_rearm_delay_s`（1.0 s）、`vision_rearm_departure_ratio`（0.08）共同控制再次触发：沿行驶方向实际移动达到距离、最短时间已过，并观察到上一树干离开中心区域后，下一树干位于行驶前方或中心区域就重新启用 PI 对中，不再要求检测帧恰好落入中心容差。旧树留在后方侧边时继续走，避免倒回旧树。

回位后先以 `vision_rearm_speed_mps`（默认 0.10 m/s）低速离开，且不超过路线速度和 `vision_tree_max_speed_mps`。沿路线走够 `vision_rearm_distance_m`、经过 `vision_rearm_delay_s`，并观察到旧树干离开中心区域或暂时不可见后，立即解除离开阶段的限速，恢复当前路线速度，无需等待下一棵树出现。此时仍抑制后方旧树；下一树干位于行驶前方或中心且有合格芒果时，照常减速对中。最后一棵树后没有新目标也能恢复路线速度。日志 `departure complete; restoring route speed` 表示该限速已解除。

无需没有芒果的空白帧；距离过近、树干无法区分时仍可能跳过，可按实际树间距调小重触发距离和延时。相机和里程计失效仍走原停车保护。正常回位与超时回位均使用独立的 `home_timeout_ms`，抓取的 10 秒预算不会中途取消回位；必须收到匹配本次停车的回位成功通知才续行。

接口新增后必须重新生成消息并构建两个使用端：
###### ###############################################################
```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_interfaces robot_chassis robot_vision --symlink-install
source install/setup.bash
```

按视觉 README 启动带实测标定的视觉节点，再启动底盘行为树。观察：

```bash
ros2 topic echo /vision/row_observation
ros2 topic echo /base/vision_stop_state
ros2 topic echo /base/cmd_vel
ros2 topic echo /vision/harvest_targets
ros2 topic echo /base/odom --field twist.twist
```

状态：0=等待，1=芒果触发低速找树，2=按画面误差 PI 居中，3=确认停稳，4=HOLDING，5=故障。
§tree_center_error_ratio§ 是画面比例误差，不是米；误差容差为 ±2%。
看到状态 4、实际里程计零速且持续保持，才说明停车链路完成。
仅发布 `/vision/mango_target` 不会测试新停车流程；直接 action 测试需显式传
`vision_stop_enabled: true`。普通 action 调用默认不启用。
