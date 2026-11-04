# robot_chassis

底盘任务层由四个节点组成：

- `base_serial_node`：把 `/base/cmd_vel` 的 `vx/vy/wz` 发送给 STM32，并发布 `/base/odom`、`/base/imu` 和 `/base/state`。
- `robot_sensors/ultrasonic_node`：读取四个 DYP-A21 UART 传感器并发布 `/base/ultrasonic`；协议和传感器配置见 [robot_sensors/README.md](../robot_sensors/README.md)。
- `row_navigation_node`：通过 `FollowRow`、`ShiftLane`、`StopBase` Action 读取超声波和里程计，计算底盘速度；不处理 PWM。
- `mission_behavior_node`：BehaviorTree.CPP 任务调度。当前视觉测试模式在指定横移段停车并保持；自动摘取分支默认关闭。

## 安装依赖

```bash
sudo apt update
sudo apt install ros-humble-behaviortree-cpp-v3

rosdep install --from-paths src --ignore-src -r -y #一键安装依赖
```

## 编译和启动

```bash
cd ~/robot_arm_jetson/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_sensors robot_chassis robot_arm_bringup --symlink-install
source install/setup.bash
ros2 launch robot_chassis chassis.launch.py
```

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
| `vision_tree_alignment_timeout_s` | 对准树干的最长时间，超时停车报错 |
| `vision_tree_control_sign` | 实测方向修正，正常为 1.0，可设 -1.0 |

以上 `vision_*` 参数支持动态调整。相机过期、对准时丢树干、侧向安全距离和
光电到位均优先停车；它们不会被选果逻辑覆盖。进入 HOLDING 后持续发零速，
不因果实消失或参数关闭而自行恢复。

接口新增后必须重新生成消息并构建两个使用端：

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
