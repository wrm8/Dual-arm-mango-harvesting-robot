# 测试命令

## 上方、下方和回位三组速度

速度默认值集中在 `robot_arm_node/src/control/dual_arm_harvest_node.cpp` 构造函数中，直接改这三行并重新编译：

```cpp
top_motion_ = declare_motion_profile("top", {50, 300, 500, 500, 20}, 5000);
bottom_motion_ = declare_motion_profile("bottom", {50, 300, 500, 500, 20}, 5000);
home_motion_ = declare_motion_profile("home", {50, 300, 500, 500, 20}, 5000);
```

数组依次对应 joint1～joint5，左右臂共用。`left_top/right_top` 使用 top；`left_bottom/right_bottom` 使用 bottom，和大小果、颜色无关。正常回位和 10 秒超时回位均使用 home。前四项控制 HX 关节速度，正数越大越快；末尾的 `5000` 为该组 V3 第五关节的行程时间，单位毫秒，越小越快。第 5 项速度字段不决定当前 V3 的速度。原夹爪开合角和开合时间仍按原参数控制。

一键入口默认不覆盖代码中的三组数值，也没有新增 YAML 速度设置。需要临时覆盖时，可传 `top_arm_speeds:=50,300,500,500,20`、`bottom_arm_speeds:=50,300,500,500,20`、`home_arm_speeds:=50,300,500,500,20`，以及各自的 `top_move_duration_ms`、`bottom_move_duration_ms`、`home_move_duration_ms`。旧公共参数 `arm_speeds`、`move_duration_ms` 已拆分，启动命令中的旧参数须删除或换成这三组名称；使用旧名称会报出迁移提示。

## 一键启动新机构联调（击打板可不接）

新入口 `harvest_system.launch.py` 一并启动双臂、视觉和底盘（含超声波、维特 IMU、路线状态机），不再使用旧的 `competition.launch.py` 机械臂链路。先退出独立启动的这些节点及人工 HOLDING 发布器，确认两臂在已标定的弯折起始位。首次联调先在 `robot_chassis/config/chassis.yaml` 将前进、后退、横移和视觉对中速度降到 0.10 m/s。

更新后在 Jetson 构建：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-up-to robot_arm_bringup robot_chassis robot_vision --symlink-install
source install/setup.bash
```

在同一个终端启动整套系统：

```bash
source src/robot_vision/tools/env_jetson.sh
source install/setup.bash
ros2 launch robot_arm_bringup harvest_system.launch.py \
  dry_run:=false allow_motion:=true \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  skip_strike:=true
```

`skip_strike:=true` 不启动击打串口节点、不打开击打板串口，不要求旧夹爪在线。采摘流程为：实际停车确认 → 视觉自动识别 → 原坐标转换和四组偏移 → IK 到位 → 原地等待 `motor_return_delay_s`（默认 1 秒）→ 回 native 弯折零位。无需发布手动坐标。机械臂未到位或等待中停车许可失效时停止后续动作并报错。一键入口现已在启动时先让双臂回零；单独的 `dual_arm_harvest.launch.py` 保持由用户控制初始姿态。

一键启动先启动视觉和双臂，由 `startup_home.py` 读取协调节点实际生效的 `home_arm_speeds`、`home_move_duration_ms`、`home_timeout_ms` 并请求双臂回 native 弯折零位。新机构仅控制左右各 5 个关节，不发送击打指令；旧夹爪模式保留回位时延迟张开夹爪。实机回位需要两个 `*_ticks_calibrated` 开关均为 true。日志出现 `Startup home confirmed` 才放行底盘；回位被拒绝、失败或超时会退出整套系统，不启动底盘。回位仍按目标位置容差确认。默认还要求从一键启动起至少经过 10 秒；`chassis_start_delay_s:=20.0` 可延长最短等待。回位超过该延时，完成后即启动底盘，不再额外等待 10 秒。`dry_run:=true` 或 `allow_motion:=false` 跳过实际回位，底盘运动也继续受相同开关限制。`enable_chassis:=false` 仍执行启动回位，但不启动底盘。

默认使用安装目录中的配置，也可用 `driver_params`、`camera_transforms`、`harvest_params`、`vision_params`、`chassis_params`、`sensor_params`、`imu_params` 指定实际配置文件。启动回位节点成功后正常退出；其余子进程退出会请求整套系统退出。在此终端按 Ctrl+C 可结束整套测试。

仅启动视觉和双臂、不运行底盘时加 `enable_chassis:=false`，并按下文持续发布人工 HOLDING。`dry_run:=true allow_motion:=false` 是统一入口的默认值，覆盖机械臂和底盘串口桥接的运动开关；进入实机测试须显式传入上述两个开关。

后续接上击打板、确认串口与协议正常后，启动改成 `skip_strike:=false`，恢复发送 STM32 击打命令。单独的 `dual_arm_harvest.launch.py` 也支持 `use_motor:=true skip_strike:=true stop_at_target:=false` 做同样的空载来回测试。

本批全部目标完成并成功回位后，双臂通过 `/harvest/result` 通知导航继续当前横移段；当前段结束后路线状态机进入下一段。停车状态、视觉快照和完成通知携带同一个 `stop_id`，旧通知不会放行新的停车。`stop_at_target:=true` 和 dry-run 不通知续行，人工 HOLDING 的默认 `stop_id=0` 也不会自动启动车辆。

自动模式从本次首次 HOLDING 起最多尝试 `harvest_timeout_s: 10.0` 秒（双臂配置）。视觉未能生成快照也计时；超时取消正在执行的抓取动作，跳过剩余目标，双臂回 native 弯折零位，确认动作成功后续行。抓取动作使用 `MoveJoints.hold_on_cancel=true`：取消时以新鲜反馈位置保持 HX 关节，不关闭力矩，随后才回位；默认手动取消和急停仍走原逻辑。IK/目标错误同样尝试回位再跳过。不要求每棵树采摘成功。10 秒是开始退出抓取的期限，正常回位和超时回位均使用独立的 `home_timeout_ms`，不会因为抓取预算刚过期就中途取消回位；下一轮果实仍受本次停车的剩余预算约束。取消未确认、回位失败或停车许可丢失时保持停车。

双臂配置启用 `approach_complete_when_stationary: true`：伸向果实正常达到 `position_tolerance_ticks: 20` 就完成；如果读数仍在目标容差之外，但全部请求关节已停稳，也算完成，随后击打、等待、回位并续行。目标坐标不变。驱动配置 `dual_arm_driver.yaml` 的 `stationary_tolerance_ticks: 3` 是整个观测窗口内每个关节的最大位置范围，`stationary_duration_ms: 500` 是连续停稳时间（至少 3 轮全关节有效反馈），`stationary_min_wait_ms: 1000` 是全部指令发出后的最短等待。这三个参数只约束静止补充判定，不延迟正常到位判定。V3 关节需要移动超过静止范围时，补充判定还会等其设定的 `durations_ms` 行程时间结束，避免慢速量化读数被误认为停止。丢失任一请求关节的反馈会重置停稳计时。通过静止补充条件完成时，日志含 `requested joints stationary` 和各关节目标误差。静止完成包括未到目标就卡住的情况，不代表达到目标精度。回零位和单独夹紧仍只检查位置容差，回位成功后才放行底盘。将采摘参数改为 `approach_complete_when_stationary: false` 可关闭补充判定。手动动作启用该条件时，在 goal 设置 `complete_when_stationary: true`；V3 的 `durations_ms` 应填写实际行程时间，避免把 goal 超时时间当成行程时间。

此功能增加了 `MoveJoints.action` 字段，更新时应一并传输 `robot_arm_interfaces`、`robot_arm_driver`、`robot_arm_node`、`robot_arm_bringup` 源码并重新编译依赖包：`colcon build --packages-up-to robot_arm_bringup robot_chassis robot_vision --symlink-install`，随后 `source install/setup.bash` 并重启整套节点。不要只重编译驱动而继续使用旧 action 接口。

回位请求遇到短暂无应答时，驱动先对离线关节最多读取 3 次；仍离线会打印具体关节和舵机 ID，拒绝发送整组动作。协调节点对被拒绝的回位请求最多尝试 3 次，间隔 200 ms；只有明确拒绝、尚未发出动作才重试。持续离线或回位运动失败仍报告失败并保持停车，不会把失败当作回位成功。

进入 HOLDING 之前树干无法在 `vision_tree_alignment_timeout_s` 内对中时，此时尚未允许机械臂动作，直接跳过该树继续当前横移段；相机过期、超声波安全限位等故障仍停车。

同一采摘路段可以连续停靠两棵或更多棵树，每次生成新的 `stop_id`，不按路段限制为一棵。防重复触发使用底盘配置的 `vision_rearm_distance_m: 0.10`、`vision_rearm_delay_s: 1.0` 和 `vision_rearm_departure_ratio: 0.08`：恢复后沿路线至少走过 10 cm、至少 1 秒，且观察到上一树干离开画面中心 ±8% 区域（或暂不可见），再有下一树干出现在行驶前方或中心区域，就重新启用树干 PI 对中，不必刚好采到中心容差内的一帧。旧树仍在后方侧边时继续走，避免倒回旧树。离树期间速度由 `vision_rearm_speed_mps` 限制，默认 0.10 m/s，并受路线速度和 `vision_tree_max_speed_mps` 上限约束。相邻树的芒果可以持续可见，无需无果画面；间距小于阈值或无法区分树干时仍可能跳过，可按实测树间距调整距离和时间参数。

击打 STM32 仍无远程停止协议，超时只取消上位机等待，不能停止已经触发的下位机动作；Ctrl+C 也不会发送停机命令。

先校准机械臂、再测试击打、最后接入跑垄的完整步骤见 [机械臂单独控制方案](MANUAL_ARM_CONTROL.md)。`use_motor:=true stop_at_target:=true` 专用于落点校准：不启动击打串口节点，因此无需连接或实现击打 STM32。完整来回击打改为 `stop_at_target:=false`。

## 切换 STM32 串口击打 / 原夹爪

在原来的双臂启动命令后增加 `use_motor:=true`，左右两臂使用独立的 STM32 串口击打节点；传 `use_motor:=false` 恢复原夹爪流程，也不打开击打串口。也可以修改 [config/dual_arm_harvest.yaml](config/dual_arm_harvest.yaml) 中的 `use_motor: false`，保存后重启节点；显式启动参数优先于 YAML。此开关在启动时读取。

| `use_motor` | 每轮动作 |
| --- | --- |
| `false`（默认） | 到目标并预张开夹爪 → 到位后夹紧 → 返回原 native 弯折零位，回位开始后 500 ms 张开 |
| `true` | 只发送 joint1～joint5 到目标 → 串口发送本轮左/右/双臂触发帧一次 → 等待 1 秒 → 返回原 native 弯折零位 |

下位机自行执行转 10 圈并停止，上位机不接收或解析它的消息。`motor_return_delay_s:=1.0` 是发送触发帧后的本地等待时间，不是电机运行时间，也不证明 10 圈已经完成。若实际转 10 圈超过 1 秒，机械臂会在击打尚未停止时回位；需要先等停机再回位时应把该延时调到实测完成时间以上，或将来增加完成反馈。

[串口配置](../robot_end_effector/config/serial_striker.yaml) 已填好 v1 协议：115200、8N1，固定 6 字节 `AA 55 01 MASK 0D 0A`，其中 `MASK` 为 `01` 左侧、`02` 右侧、`03` 双侧，无校验、无回包。现在只需填写击打板实际固定串口路径；STM32 固件后续按此协议实现。完整协议、固定双串口路径及独立测试见 [STM32 击打说明](../robot_end_effector/SERIAL_STRIKER.md)。

双臂启动已改用串口节点，不再启动 Jetson PWM/编码器节点，也不再使用 `motor_use_encoder`、`motor_revolutions`、`motor_duty_cycle`、`motor_run_time_s`、`motor_timeout_s` 参数。10 圈和速度在 STM32 固件设置。

`stop_at_target:=true` 仍优先：到目标后停住，不启动击打串口节点、不触发击打、不回位；需要完整来回流程时使用 `stop_at_target:=false`。双臂并行、同侧多个目标逐轮抓取、各臂各果型的 XYZ 偏移和 `top_arm_speeds` / `bottom_arm_speeds` / `home_arm_speeds` 继续有效；只触发本轮选中的一侧或两侧。击打模式下启动文件自动从 12 项标定中选出实际的 10 个机械臂关节交给驱动，不轮询已拆除的 ID 6/12 旧夹爪；协调节点仍读取完整标定用于 IK。`use_motor:=false` 恢复原 12 舵机配置。串口节点不可用时拒绝前往目标；发送报错时不自动重试击打，自动路线模式尝试回位并跳过本树，手动模式停在当前位置。原驱动 YAML 的零位、方向、限位保持原值，无需删除原标定。

在 `ros2_ws` 目录构建并重新加载环境：

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-serial
colcon build --packages-select robot_arm_interfaces robot_end_effector robot_arm_node robot_arm_bringup --symlink-install
source install/setup.bash
```

下面是手动发目标测试的启动命令，标定参数请与实际保持一致：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  motor_params:="$PWD/src/robot_end_effector/config/serial_striker.yaml" \
  targets_topic:=/manual/harvest_targets \
  use_motor:=true stop_at_target:=false dry_run:=false allow_motion:=true \
  motor_return_delay_s:=1.0 \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000
```

之后按下文持续发送 HOLDING 和手动目标。到位日志会显示 `sending STM32 strike trigger`，串口节点显示 `TX arms=... hex=...; local wait=1.000s`，本地等待结束后发送 `return home after STM32 strike delay`。ROS Action 成功只确认写入完整帧且本地延时结束，不代表下位机收到或完成动作。取消等待、节点退出都不能通过当前单向协议停止下位机；下位机必须自行完成并停机。旧 `/end_effector/stop` 是 PWM 节点的服务，不能停止新 STM32。独立测试节点与完整双臂启动不要同时运行，否则会争用击打串口。

## 双臂按大小果自动选择夹爪角度与偏移

配置文件为 [config/dual_arm_harvest.yaml](config/dual_arm_harvest.yaml)。先根据目标的 `slot` 选择左/右臂，再根据 `is_small` 选择小果/大果；颜色不决定大小，所以绿色大果、绿色小果和黄色大小果都会按实际大小选择。

| 果型 | 到目标前张开角 | 夹住角 | 开合时间 |
| --- | --- | --- | --- |
| 小果，`is_small: true` | -1.0 rad | -2.0 rad | 2000 ms |
| 大果，`is_small: false` | 0.0 rad | -1.0 rad | 2000 ms |

左右臂各自有一套大果和小果参数。四组 XYZ 偏移分别是 `left_large_offset_m`、`left_small_offset_m`、`right_large_offset_m`、`right_small_offset_m`。向量顺序为 `[X, Y, Z]`，单位米，沿所选机械臂自身 `base_link` 的轴，在相机点转换后、求 IK 前相加。修改它们不会改变相机外参和模型 TF。左大果保留原代码 `[-0.02, 0.03, 0.013]`，左小果默认零；右臂原通用偏移 `[0.0, -0.01, -0.01]` 已分别迁入右大果、右小果，后续可单独修改。配置修改后需重启节点。

构建后用下面的入口测试，机械臂启动不会自行抓取，需持续 HOLDING 并手动发目标：

```bash
colcon build --packages-select robot_arm_node robot_arm_bringup robot_chassis --symlink-install
source install/setup.bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  targets_topic:=/manual/harvest_targets \
  stop_at_target:=false dry_run:=false allow_motion:=true \
  left_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000
```

右臂标定已完成时，启动增加 `right_ticks_calibrated:=true`；目标用 `right_top` / `right_bottom`。两侧同轮可以使用不同果型，张开和闭合角独立选取。同侧下一轮会重新选型。

底盘实际停稳后，在另一个终端保持发送：

```bash
ros2 topic pub -r 10 /base/vision_stop_state \
  robot_arm_interfaces/msg/VisionStopState \
  "{header: auto, stage_id: 0, state: 4, tree_error_m: 0.0, tree_center_error_ratio: 0.0, message: 'manual size profile test'}"
```

再发送当前测量的相机坐标。下面是左下大果的历史示例，实测时替换坐标；测试小果只需设置 `is_small: true`，不能省略该字段后还期望自动按小果执行。

```bash
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_bottom, color: yellow_mango, is_small: false, surface_point: {x: 0.1018580124, y: 0.0659120977, z: 0.4750000238}, confidence: 0.92}]}"
```

日志 `Grasp profile=left_large ... offset_base_m=... pregrasp=... close=...` 可核对选型。流程为到目标并预张开、到位后夹紧、返回 native 弯折零位，回位开始后 500 ms 张开。V3 夹爪用 `gripper_duration_ms` 控制时间，`gripper_speed` 不改变其速度；遇到果子导致夹爪到不了目标角时，仍会按原来的到位检查报超时。

默认 `use_size_profiles: true`，旧的通用 `gripper_pregrasp_native_rad` / `gripper_close_native_rad` 不再覆盖四组果型角度。需要继续用下面旧的固定角度测试命令时，显式增加 `use_size_profiles:=false`。`gripper_duration_ms`、`gripper_open_native_rad` 和 `use_size_profiles` 都可在 YAML 设置，也可用同名启动参数覆盖。

直接解析 IK：使用 `backend:=legacy ik_solver:=analytic`，仍调用 `/pick_point`。
求解坐标、工具倾角定义、构建和使用步骤见 [ANALYTIC_IK.md](../robot_arm_node/ANALYTIC_IK.md)。

ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.5, 1.5, 0.5, 0.0, 0.0], speeds: [100, 200, 700, 200, 5, 5], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"

ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [joint1, joint2, joint3, joint4, joint5, gripper], positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speeds: [100, 200, 700, 200, 20, 20], duration: {sec: 5, nanosec: 0}, wait_for_completion: false}"

ros2 action send_goal /pick_point robot_arm_interfaces/action/PickPoint \
"{target: {header: {frame_id: base_link}, point: {x: 0.15, y: 0.0, z: 0.20}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.0, duration: {sec: 2, nanosec: 0}, return_home: false}" \
--feedback

# 先规划，不动机械臂
ros2 run robot_arm_moveit_config moveit_xyz_goal --ros-args \
  -p x:=0.30 -p y:=0.0 -p z:=0.20 \
  -p velocity_scale:=0.3 -p acceleration_scale:=0.3 \
  -p execute:=false

##解析规划启动命令
ros2 launch robot_arm_bringup real_backend.launch.py \
  backend:=legacy ik_solver:=analytic allow_motion:=false

ros2 action send_goal /pick_point robot_arm_interfaces/action/PickPoint \
"{target: {header: {frame_id: base_link}, point: {x: 0.32, y: 0.02, z: 0.25}}, approach_height_m: 0.0, grasp_offset_x_m: 0.0, grasp_offset_y_m: 0.0, grasp_offset_z_m: 0.0, target_tool_tilt: 0.0, target_joint5: 0.0, duration: {sec: 5, nanosec: 0}, return_home: false}" \
--feedback
###### ##############################################
## 机械臂

机械臂回到零位
ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints "{joint_names: [left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, right_joint1, right_joint2, right_joint3, right_joint4, right_joint5], positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speeds: [30, 700, 900, 700, 20, 30, 700, 900, 700, 20], duration: {sec: 2, nanosec: 0}, wait_for_completion: true}" --feedback


双机械臂加rviz启动
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  dry_run:=true allow_motion:=false rviz:=true

双臂模型加驱动-旧版不可以设置速度
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  dry_run:=true allow_motion:=true rviz:=true

  这里两个参数的意思是：dry_run:=true 让视觉自动抓取流程只计算；allow_motion:=true 允许你通过手动 Action 控制机械臂。驱动会按配置使能力矩。

双臂模型加驱动-新版可以设置速度
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  targets_topic:=/manual/harvest_targets \
  stop_at_target:=true \
  dry_run:=false allow_motion:=true rviz:=true \
  left_ticks_calibrated:=true \
  top_arm_speeds:=300,600,850,400,20 bottom_arm_speeds:=300,600,850,400,20 home_arm_speeds:=300,600,850,400,20 \
  top_move_duration_ms:=1000 bottom_move_duration_ms:=1000 home_move_duration_ms:=1000 move_timeout_ms:=15000 \
  target_tool_tilt:=0.174533

ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  targets_topic:=/manual/harvest_targets \
  stop_at_target:=true \
  dry_run:=false allow_motion:=true rviz:=true \
  right_ticks_calibrated:=true \
  top_arm_speeds:=300,600,850,400,20 bottom_arm_speeds:=300,600,850,400,20 home_arm_speeds:=300,600,850,400,20 \
  top_move_duration_ms:=1000 bottom_move_duration_ms:=1000 home_move_duration_ms:=1000 move_timeout_ms:=15000 \
  target_tool_tilt:=0.349066

  想要的姿态	启动参数
  水平	target_tool_tilt:=0.0
  向上抬 10°	target_tool_tilt:=0.174533
  向上抬 20°	target_tool_tilt:=0.349066
  向下压 10°	target_tool_tilt:=-0.174533
  向下压 20°	target_tool_tilt:=-0.349066

  top/bottom/home_arm_speeds 各自依次对应 joint1～joint5。**当前 joint1～4 的 HX 电机使用速度字段；joint5 的 V3 电机使用 move_duration_ms 控制运行时间。

抓取-夹住-回位

自动-新机构
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  targets_topic:=/vision/harvest_targets \
  use_motor:=true \
  use_size_profiles:=true \
  stop_at_target:=true \
  dry_run:=false allow_motion:=true rviz:=true \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 \
  gripper_duration_ms:=2000 \
  move_timeout_ms:=15000 home_timeout_ms:=15000 \
  target_tool_tilt:=0.0 target_joint5:=0.0


小芒果
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  use_size_profiles:=false \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  targets_topic:=/manual/harvest_targets \
  stop_at_target:=false dry_run:=false allow_motion:=true \
  left_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 \
  gripper_pregrasp_native_rad:=-1.0 \
  gripper_close_native_rad:=-2.0 \
  gripper_open_native_rad:=0.0 \
  gripper_duration_ms:=2000 \
  open_delay_ms:=500

大芒果
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  use_size_profiles:=false \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  targets_topic:=/manual/harvest_targets \
  stop_at_target:=false dry_run:=false allow_motion:=true \
  left_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 \
  gripper_pregrasp_native_rad:=0.0 \
  gripper_close_native_rad:=-1.0 \
  gripper_open_native_rad:=0.0 \
  gripper_duration_ms:=2000 \
  open_delay_ms:=500

## 视觉

source src/robot_vision/tools/env_jetson.sh
source install/setup.bash

ros2 launch robot_vision vision.launch.py \
  params_file:="$PWD/src/robot_vision/config/vision.yaml" \
  publish_targets:=true

ros2 launch robot_vision vision.launch.py publish_targets:=true

## 手动发送车子停稳状态标志位
ros2 topic pub -r 10 /base/vision_stop_state \
  robot_arm_interfaces/msg/VisionStopState \
  "{header: auto, stage_id: 0, state: 4, tree_error_m: 0.0, tree_center_error_ratio: 0.0, message: 'manual speed test'}"

## 读取坐标
ros2 topic echo /vision/harvest_targets --once

## 发命令
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_top, color: yellow_mango, surface_point: {x: 0.10185801237821579, y: 0.06591209769248962, z: 0.4750000238418579}, confidence: 0.92}]}"

ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_bottom, color: yellow_mango, is_small: false, surface_point: {x: 0.09191010892391205, y: 0.06104132905602455, z: 0.5520000457763672}, confidence: 0.92}]}"

ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 2, slot: right_top, color: yellow_mango, surface_point: {x: 0.10006383806467056, y: -0.08126313239336014, z: 0.5370000004768372}, confidence: 0.92}]}"

  将左侧目标的 surface_point 填进下面命令。这里填的是相机坐标，节点会自动转换：

###### ##############################################################
## 双臂视觉抓取（先 dry-run）

###### ##############################################
## 底盘



###### ##############################################

### 手动触发，按关节速度到位并停住

这条测试链路由 `dual_arm_harvest_node` 直接完成：相机坐标 → 对应基座坐标 → 解析 IK → native 关节角 → `/move_joints`（含 `speeds`）。不需要另起 `ik_pick_node`，它走的是旧轨迹接口，不使用这里的 `top_arm_speeds` / `bottom_arm_speeds` / `home_arm_speeds`。

更新并在 Jetson 构建后执行：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select robot_arm_node robot_arm_bringup --symlink-install
source install/setup.bash
```

每个终端都先进入上述工作区并加载 `install/setup.bash`。目标侧 tick 标定需已确认；相机坐标测试需要在 `camera_to_bases.json` 中核对该侧矩阵并将该侧 `calibrated` 设为 `true`，不能只为通过检查而修改开关。先前单点对齐不能证明完整外参标定。

终端一（以下先测左臂）：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  targets_topic:=/manual/harvest_targets stop_at_target:=true \
  dry_run:=false allow_motion:=true rviz:=true \
  left_ticks_calibrated:=true \
  top_arm_speeds:=50,300,300,300,20 bottom_arm_speeds:=50,300,300,300,20 home_arm_speeds:=50,300,300,300,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 move_timeout_ms:=15000
```

`targets_topic` 将机械臂触发入口改为手动话题；视觉继续发布 `/vision/harvest_targets`，识别到目标不会自动移动。`stop_at_target:=true` 只发送所选臂的五个关节，夹爪保持原状，到位后停住。`top_arm_speeds` / `bottom_arm_speeds` / `home_arm_speeds` 依次对应 joint1..joint5：当前 HX joint1..4 使用速度字段；V3 joint5 使用 `top_move_duration_ms` / `bottom_move_duration_ms` / `home_move_duration_ms` 行程时间。`move_timeout_ms` 是等待到位上限。共享驱动的 `allow_motion` 会按配置给两臂使能力矩。

终端二（视觉及 GPU 环境）：

```bash
source src/robot_vision/tools/env_jetson.sh
source install/setup.bash
ros2 launch robot_vision vision.launch.py \
  params_file:="$PWD/src/robot_vision/config/vision.yaml" publish_targets:=true
```

终端三（底盘实际已停车后，持续发布停止状态）：

```bash
ros2 topic pub -r 10 /base/vision_stop_state \
  robot_arm_interfaces/msg/VisionStopState \
  "{header: auto, stage_id: 0, state: 4, tree_error_m: 0.0, tree_center_error_ratio: 0.0, message: 'manual speed test'}"
```

终端四：先读取视觉目标，复制要测一侧的 `surface_point`，仍为相机坐标。

```bash
ros2 topic echo /vision/harvest_targets --once
```

然后手动发一次（下面是旧日志中的左侧示例，实际用当前测量值）：

```bash
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_top, color: yellow_mango, surface_point: {x: -0.0028500671, y: 0.1205296665, z: 0.5279999971}, confidence: 0.92}]}"
```

预期日志先显示 `IK left target ... base=(...)`，随后逐关节显示 `native=... rad speed=... duration=...`；到位显示 `target pose reached; holding position, grippers unchanged`。使用 `stamp: now` 让每次手动命令都有新时间戳；同一时间戳的重发会被去重。保持目标不动，一次仅测试一个侧别；右臂在确认其标定后，启动时增加 `right_ticks_calibrated:=true`，命令的 `slot` 改为 `right_top`，坐标换成右侧的相机点。不要把日志中的 `base` 坐标填进这个相机坐标消息，以免重复转换。

需要完整抓取时，将启动参数改成 `stop_at_target:=false`，同一条手动消息会触发：到目标并按 `is_small` 预张开 → 按大小果闭合 → 弯折回位 → 回位开始后延迟 500 ms 张开。夹爪默认使用已测的 `2000 ms`，HX 关节速度仍由 `top_arm_speeds` / `bottom_arm_speeds` / `home_arm_speeds` 控制。

双臂入口订阅 `/vision/harvest_targets`。视觉消息保留 `camera_color_optical_frame` 坐标；协调节点根据目标 slot 选择左/右机械臂，再分别用 `camera_to_bases.json` 中的变换换算到各自的 `left_base_link` / `right_base_link` 后独立做 IK。**当前外参未标定，右臂零位也只是左臂临时复制值，不能执行实机运动。**

先在 Jetson 工作区构建并重新加载环境：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_arm_interfaces robot_arm_driver robot_arm_node robot_arm_bringup robot_vision --symlink-install
source install/setup.bash
```

先启动底盘/路线流程，使机械臂目标只在底盘报告新鲜 `HOLDING` 后产生。然后分别在两个终端运行，双臂启动默认 `dry_run=true`、`allow_motion=false`，不会下发机械臂动作：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py
ros2 launch robot_vision vision.launch.py publish_targets:=true
```

双臂日志应显示视觉目标的相机坐标、左右侧各自的 IK 角度和 `DRY RUN`。没有新鲜的 12 个关节实测反馈、URDF 或匹配 stage 的 `HOLDING` 时，快照会被忽略；若任一舵机未回读，双臂配置不会发布 `/joint_states`。

### 双臂模型与坐标系检查

双臂启动会从原来的 `arm_urdf_v1.urdf` 自动复制出左右两套模型，发布到 `/robot_description`：左臂使用 `left_base_link`、`left_joint1..5`、`left_tcp_link`，右臂对应 `right_*`。不用手工复制 URDF。两条 IK 链分别从各自的基座到 TCP，均复用原来的几何尺寸；原单臂启动仍使用无前缀模型。模型未包含夹爪开合关节，因此 RViz 只显示已有臂段和 TCP，不显示夹爪开合。

`camera_to_bases.json` 的含义为 `p_base = T_base_camera * p_camera`。该文件同时用于 IK 坐标转换和模型的固定 TF。URDF 以 `camera_color_optical_frame` 为根，挂载左右基座时使用矩阵的逆，不能直接把矩阵平移列当成相机到基座的安装位置。修改 JSON 后需要重启双臂启动；`calibrated: false` 时仍可显示和 dry-run，不代表已标定。

在 Jetson 桌面启动显示并检查 TF：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  dry_run:=true allow_motion:=false rviz:=true
```

RViz 默认 Fixed Frame 为 `left_base_link`，RobotModel 读取 `/robot_description`，TF 显示两个基座、两个 TCP 和相机的坐标轴。可切换 Fixed Frame 到 `right_base_link`，查看右臂坐标。不要同时运行单臂 `display.launch.py`、`real.launch.py` 或另一个驱动/robot_state_publisher，它们会与当前模型和串口冲突。

```bash
ros2 run tf2_ros tf2_echo left_base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo right_base_link camera_color_optical_frame
```

两个命令应分别显示 JSON 中的 `T_left_base_camera`、`T_right_base_camera`。矩阵内的米制尺寸、轴向及相机原点相对各基座的位置仍须实测；TF 存在只证明模型接通，不能证明外参正确。外参配置可用 `camera_transforms:=/绝对路径/camera_to_bases.json` 指定，避免误用旧单臂的 `camera_to_base.json`。

每轮抓取顺序为：左右目标若同轮存在则一起移动到各自 IK 角度，并按各自目标的 `is_small` 选择预张开角与闭合角，配置见本文开头；随后两臂回到驱动配置中的 native 零位，夹爪在回位开始后延迟 `500 ms` 张开。臂关节速度可用 `top_arm_speeds:=100,500,700,200,20 bottom_arm_speeds:=100,500,700,200,20 home_arm_speeds:=100,500,700,200,20` 设置，HX 关节使用速度字段，V3 关节行程时间由 `top_move_duration_ms` / `bottom_move_duration_ms` / `home_move_duration_ms` 控制，V3 夹爪时间由 `gripper_duration_ms` 控制。

实机启用前必须先测量并填写 `src/robot_vision/config/camera_to_bases.json` 的左右完整矩阵且各自 `calibrated: true`；在 `src/robot_arm_driver/config/dual_arm_driver.yaml` 独立标定 ID 7–12 的零位、方向和限位，不能沿用复制的左臂值。两侧都标定后，检查急停、清空工作空间、确保机械臂有支撑，再显式启动：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  dry_run:=false allow_motion:=true \
  left_ticks_calibrated:=true right_ticks_calibrated:=true
```

这个启用开关会让共享驱动对所有在线 HX 舵机使能力矩，即使当前只有一侧有目标；所以不能用它单独试未标定的另一臂。先保持默认 dry-run，直到两套外参和 12 个舵机反馈/零位均逐项确认。实际运动仍需在 Jetson + ROS 2 Humble、相机和机械臂上现场验证；桌面侧测试不会证明实机坐标准确或抓取可靠。









###### ############################################最终一键启动
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
source src/robot_vision/tools/env_jetson.sh
source install/setup.bash

ros2 launch robot_arm_bringup harvest_system.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  motor_params:="$PWD/src/robot_end_effector/config/serial_striker.yaml" \
  vision_params:="$PWD/src/robot_vision/config/vision.yaml" \
  chassis_params:="$PWD/src/robot_chassis/config/chassis.yaml" \
  dry_run:=false allow_motion:=true \
  use_motor:=true skip_strike:=false stop_at_target:=false \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  motor_return_delay_s:=1.0
