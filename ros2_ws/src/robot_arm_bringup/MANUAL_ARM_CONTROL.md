# 机械臂单独控制：先校准落点，再测试击打

视觉自动识别、底盘与双臂的一键联调命令见 [统一启动](README.md#一键启动新机构联调击打板可不接)。击打板未接入但想完整来回时，用 `use_motor:=true skip_strike:=true stop_at_target:=false`：到位后只等待设定时间再回位，不打开击打串口；原四组偏移与 IK 均保留。

当前先完成机械臂精度调试，底盘保持实际停车，不启动 `robot_chassis chassis.launch.py`。机械臂手动入口为 `/manual/harvest_targets`，视觉入口仍为 `/vision/harvest_targets`，识别到目标后由人选择坐标并手动触发。

这里的“起始位”是当前驱动定义的 **native 弯折零位**：左右各 joint1～joint5 的 `/move_joints` 位置均为 `0.0`。它不等于 URDF 五个关节角都为零，也不包括旧夹爪的零位。新击打模式只控制 10 个臂关节和独立 STM32。

## 1. Jetson 构建及每个终端的环境

在已使用的 Jetson 工程目录执行：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo apt install python3-serial
colcon build --packages-select robot_arm_interfaces robot_arm_driver robot_arm_node robot_end_effector robot_arm_bringup --symlink-install
source install/setup.bash
```

以后每个新终端先执行以下三行；只有源码更新后才需要重新构建：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

相机外参见 `src/robot_vision/config/camera_to_bases.json`，关节零位/方向/限位见 `src/robot_arm_driver/config/dual_arm_driver.yaml`。下列实机命令中的左右标定标志应以实际已确认的标定为前提；未确认时先用 `dry_run:=true allow_motion:=false` 核对模型和 IK 日志。日志出现某个 joint1～joint5 没有位置回包、离线、IK 无解或越限时停止该次调试。

## 2. 终端 A：只启动机械臂，先用到位停住模式

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  targets_topic:=/manual/harvest_targets \
  use_motor:=true stop_at_target:=true \
  dry_run:=false allow_motion:=true rviz:=true \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 move_timeout_ms:=15000 home_timeout_ms:=15000 \
  target_tool_tilt:=0.0 target_joint5:=0.0
```

这一模式不启动 `serial_striker_node.py`，不打开击打串口，不依赖击打板或其固件。目标只移动指定侧的五个臂关节，到位后停住，旧夹爪不接收运动命令。不要同时启动另一个机械臂驱动或 `real.launch.py`。

`top_arm_speeds` / `bottom_arm_speeds` / `home_arm_speeds` 的五个值依次对应 joint1～joint5，左右共用这五个设置。当前 HX joint1～joint4 按速度字段控制，V3 joint5 按 `top_move_duration_ms` / `bottom_move_duration_ms` / `home_move_duration_ms` 行程时间控制。`move_timeout_ms` 是等待到位的上限，调大它不会降低实际运动速度。

## 3. 终端 B：先回起始位

保持终端 A 运行，终端 B 执行下列命令。它直接走 `/move_joints`，不需要视觉或 HOLDING：

```bash
ros2 action send_goal /move_joints robot_arm_interfaces/action/MoveJoints \
  "{joint_names: [left_joint1, left_joint2, left_joint3, left_joint4, left_joint5, right_joint1, right_joint2, right_joint3, right_joint4, right_joint5], positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speeds: [50, 300, 500, 500, 20, 50, 300, 500, 500, 20], duration: {sec: 5, nanosec: 0}, wait_for_completion: true}" \
  --feedback
```

等 `success: true` 并确认实际姿态为已标定的弯折起始位，再发下一条动作。不要把 `left_gripper/right_gripper` 加进这条命令：新击打模式不使用它们，原 right_gripper 的 native 零值也不适用于直接发送 V3 的负 tick。

如需查看模型角度：

```bash
ros2 topic echo /joint_states --once
```

`/joint_states` 发布 URDF 角度，native 回位后读数可以不为 0；不要把它的 URDF 角度直接当作 `/move_joints` 的 native 角度照抄。现场反馈还需核对驱动日志中的各关节真实回包，不能只凭 JointState 中有名字就判定电机在线。

## 4. 终端 C：手动维持停车许可

底盘实际停稳、自动底盘节点未运行时执行，并保持这个终端运行：

```bash
ros2 topic pub -r 10 /base/vision_stop_state \
  robot_arm_interfaces/msg/VisionStopState \
  "{header: auto, stage_id: 0, state: 4, tree_error_m: 0.0, tree_center_error_ratio: 0.0, message: 'manual arm calibration; chassis parked'}"
```

`state: 4` 是 HOLDING，允许协调节点接收对应 `stage_id: 0` 的目标。它只是手动调试许可，不会实际给底盘刹车。自动跑垄时退出这个发布器，由底盘自己发布真实停车状态，不能同时伪造 HOLDING。

## 5. 终端 D：读取视觉坐标（相机接入时）

```bash
source src/robot_vision/tools/env_jetson.sh
source install/setup.bash
ros2 launch robot_vision vision.launch.py \
  params_file:="$PWD/src/robot_vision/config/vision.yaml" \
  publish_targets:=true
```

另一个已加载环境的终端读取当前冻结目标：

```bash
ros2 topic echo /vision/harvest_targets --once
```

等消息 `valid: true` 且 `targets` 非空，再记录目标的 `surface_point`、`slot`、`is_small`。读取的点仍在 `camera_color_optical_frame` 下，协调节点会按选中臂自动转换到各自的基座，再加对应的大小果偏移并求 IK。不要将转换后的 `base=(...)` 填回相机坐标消息，避免重复转换。

同一轮 HOLDING 会冻结视觉快照，移动芒果后不会自动更新该快照。需要重新测点时，先等当前机械臂动作结束，在终端 C 按 Ctrl+C 停止 HOLDING 发布，再发一次 IDLE：

```bash
ros2 topic pub --once /base/vision_stop_state \
  robot_arm_interfaces/msg/VisionStopState \
  "{header: auto, stage_id: 0, state: 0, message: 'refresh manual target snapshot'}"
```

随后重新运行步骤 4 的 10 Hz HOLDING 命令，等视觉稳定，再读取时间戳已更新、`valid: true` 且非空的目标消息。

没有相机时可省略终端 D，使用已知的、与当前相机安装一致的相机坐标手动测试。相机没连接并不妨碍已知点的坐标转换和 IK，但不能由此获得新目标坐标。

## 6. 手动发目标：先左臂，再右臂，最后双臂

以下坐标来自历史调试记录，只用于展示完整命令。实际发之前，将 XYZ 替换为当前待测目标的相机坐标；每次使用 `stamp: now`，等该次动作结束再发下一次。

左下大果：

```bash
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_bottom, color: yellow_mango, is_small: false, surface_point: {x: 0.1018580124, y: 0.0659120977, z: 0.4750000238}, confidence: 0.92}]}"
```

右侧大果：

```bash
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 2, slot: right_top, color: green_mango, is_small: false, surface_point: {x: 0.1000638381, y: -0.0812631324, z: 0.5370000005}, confidence: 0.92}]}"
```

左小果/右小果分别把目标的 `is_small` 改为 `true`，坐标换成该小果的当前测量值。`slot` 决定机械臂，`is_small` 决定大小果偏移；颜色不决定机械臂或大小。`left_top` 与 `left_bottom` 都属于左臂，`right_*` 属于右臂。

两臂分别核对通过后，可以把两个目标放进同一消息：

```bash
ros2 topic pub --once /manual/harvest_targets \
  robot_arm_interfaces/msg/HarvestTargetArray \
  "{header: {stamp: now, frame_id: camera_color_optical_frame}, stage_id: 0, valid: true, targets: [{track_id: 1, slot: left_bottom, color: yellow_mango, is_small: false, surface_point: {x: 0.1018580124, y: 0.0659120977, z: 0.4750000238}, confidence: 0.92}, {track_id: 2, slot: right_top, color: green_mango, is_small: false, surface_point: {x: 0.1000638381, y: -0.0812631324, z: 0.5370000005}, confidence: 0.92}]}"
```

同轮左右两臂分别求 IK，通过一个 `/move_joints` Goal 协同发送。单侧消息只移动该侧。调试模式的预期日志是：`Grasp profile=...` → `IK left/right target ... camera=(...) base=(...)` → `move to IK target ... speed=...` → `target pose reached; holding position`。到位后用步骤 3 的命令手动回起始位。

## 7. 调准四组偏移、工具倾角与速度

四组偏移编辑 `src/robot_arm_bringup/config/dual_arm_harvest.yaml`，每组都是 `[X,Y,Z]`，单位米：

| 机械臂与大小 | 参数 |
| --- | --- |
| 左大果 | `left_large_offset_m` |
| 左小果 | `left_small_offset_m` |
| 右大果 | `right_large_offset_m` |
| 右小果 | `right_small_offset_m` |

偏移沿各自 `left_base_link/right_base_link` 的坐标轴，在相机转换后、IK 前叠加。例如希望沿左臂基座 Z 正方向多抬 1 cm，就将当前对应组的第三项增加 `0.01`，不替换其他已标定项。四组偏移在到位停住模式、串口击打模式和原夹爪模式中共用。

末端倾角通过启动参数设置，例如 `target_tool_tilt:=0.2` 约为向上 11.46°。精确定义是 TCP 正 Z 轴与所选臂基座 XY 平面的夹角：`0.0` 水平，正值向上、负值向下，有效范围 `[-pi/2,pi/2]`；它不等于 TF/RPY 的 pitch，也不等于 joint4 的关节角。完整说明见 [解析 IK](../robot_arm_node/ANALYTIC_IK.md)。当前两臂共用这一个工具倾角。

修改 YAML 偏移或启动参数后，退出并重启终端 A，再保持手动 HOLDING 后重新发同一测量点。不要只 `ros2 param set` 就期待当前已缓存的这些控制变量立即改变。建议每轮只改一组、一个轴，并保留当前已测好的数值；回起始位后重复测试左大、左小、右大、右小。

## 8. 击打 STM32 完成后，测试到位—击打—回位

STM32 固件后续按 [击打协议 v1](../robot_end_effector/SERIAL_STRIKER.md) 实现：115200、8N1，左侧 `AA 55 01 01 0D 0A`、右侧 `AA 55 01 02 0D 0A`、双侧 `AA 55 01 03 0D 0A`；收一帧只执行一次，完成 10 圈并停机。

将 `src/robot_end_effector/config/serial_striker.yaml` 的 `serial_device` 改为击打板实际固定串口路径。先按击打说明独立验证下位机，再退出独立串口节点。退出终端 A 原启动，重新完整启动：

```bash
ros2 launch robot_arm_bringup dual_arm_harvest.launch.py \
  driver_params:="$PWD/src/robot_arm_driver/config/dual_arm_driver.yaml" \
  camera_transforms:="$PWD/src/robot_vision/config/camera_to_bases.json" \
  harvest_params:="$PWD/src/robot_arm_bringup/config/dual_arm_harvest.yaml" \
  motor_params:="$PWD/src/robot_end_effector/config/serial_striker.yaml" \
  targets_topic:=/manual/harvest_targets \
  use_motor:=true stop_at_target:=false \
  dry_run:=false allow_motion:=true rviz:=true \
  left_ticks_calibrated:=true right_ticks_calibrated:=true \
  top_arm_speeds:=50,300,500,500,20 bottom_arm_speeds:=50,300,500,500,20 home_arm_speeds:=50,300,500,500,20 \
  top_move_duration_ms:=5000 bottom_move_duration_ms:=5000 home_move_duration_ms:=5000 move_timeout_ms:=15000 home_timeout_ms:=15000 \
  target_tool_tilt:=0.0 target_joint5:=0.0 motor_return_delay_s:=1.0
```

先用步骤 3 回起始位，保持终端 C 的停车许可，再用步骤 6 发一个目标。完整动作是：起始位 → IK 击打位 → 确认臂关节到位 → 串口发一次该侧触发 → 等待默认 1 秒 → 回 native 弯折零位。下一次目标另行手动发送。正常来回流程不会自动先回零；所以第一轮应由人确认起始位，上一轮成功回位后自然作为下一轮起点。

`motor_return_delay_s` 只决定机械臂到位、发触发后等多久再回位，不控制 STM32 的圈数或转速。上位机不读取击打板反馈，因此不能确认 10 圈是否已经完成。动作超时、串口写入失败或到位检查失败时，该轮会报错并停止后续步骤，不能因为已经接近目标就视为整轮成功。

## 9. 异常停止与下一阶段自动跑垄

机械臂驱动的软件停机接口为：

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger '{}'
```

当前驱动会尝试对在线 V3 发送停止、对 HX 关闭力矩。关闭 HX 力矩可能失去支撑，使用前准备好机构支撑；它不停止独立击打 STM32，也不能替代击打电源开关。异常后退出启动、查清原因，再重新启动并确认起始位，不继续堆积目标命令。

目标自动流程为：

```text
底盘沿路线行驶
  → 合格芒果进入检测范围，进入视觉减速/搜索
  → 树干对中，实测停稳并发布 HOLDING
  → 冻结目标，双臂从起始位到各自击打位
  → 触发对应侧 STM32，等待设定击打延时
  → 双臂回起始位且 MoveJoints 确认成功
  → 通知底盘本批完成，解除停车并重新布防
  → 继续本段路线，再处理下一棵树，直到整条路线结束
```

当前已有视觉对中停车、目标冻结和双臂单批击打回位；**双臂完成后通知底盘解除 HOLDING 并继续路线的闭环还未接入**。底盘当前树干对中后锁存零速度，视觉消失也不会自动继续，双臂回位成功也没有发布整批完成状态。因此当前不能仅改为 `/vision/harvest_targets` 就视为全自动跑通。

机械臂调准、击打下位机完成后，下一阶段需接入按 stage/目标批次匹配的“执行中、回位完成、失败”状态，以及底盘解除 HOLDING 与同一棵树的重新触发抑制。击打 STM32 可以继续只收命令；机械臂上位机到位/回位完成通知底盘是另一层通信，两者并不冲突。失败时底盘继续停车，所有本批目标成功回位后才继续；路线结束后统一停车。
