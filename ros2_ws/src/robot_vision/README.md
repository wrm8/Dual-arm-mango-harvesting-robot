# 芒果视觉与 ROS 2 接口

`yolo_mango/` 从 Windows 桌面的 `yolo_mango-main` 迁入，保留训练、评估、RGB/RealSense 推理、四果选小果规则和原有测试。`scripts/mango_vision_node.py` 复用这些函数发布 ROS 2 消息。运行视觉节点不会启动底盘或机械臂。

## Jetson 环境和构建

已适配 Ubuntu 22.04 / ROS 2 Humble / JetPack 6 / CUDA 12.6 / Python 3.10。WSL 的 CUDA 11.8 锁定文件只供 WSL 参考，不能装到 Jetson。GPU Python 环境在 `~/venvs/mango_gpu`，与 ROS 系统环境分开。环境脚本禁止加载 `~/.local` 中残留的 CUDA 13 版 PyTorch，并加入 cuDSS 的动态库路径。

首次配置或重建环境：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision
bash tools/setup_jetson.sh
```

构建只包含接口和视觉包：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_interfaces robot_vision --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

连接 D435i 后启动识别：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision
bash tools/run_jetson.sh
# 普通图片验证，只发布观测和无效目标，不触发摘取
bash tools/run_jetson.sh source:=/absolute/path/test.jpg
```

在其他终端使用 GPU Python：

```bash
source ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision/tools/env_jetson.sh
python -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

## ROS 2 输出

横移视觉控制只需 RealSense 深度，不依赖相机到机械臂的外参：
合格芒果和树干均按 `harvest_min_depth_m..harvest_max_depth_m` 过滤（默认
0.40–0.60 m，含边界）。芒果有效后底盘切到低速；树干中心用已经旋转后的
YOLO 画面坐标做 PI 横移对中。机械臂 `base_link` 外参仅用于抓取目标的三维坐标，
不会决定树干是否能居中。非 RealSense/RGB 输入没有有效深度，不会触发这项控制。

| 话题 | 消息 | 内容 |
|---|---|---|
| `/vision/observations` | `std_msgs/msg/String` | JSON，颜色、框高、选果结果、稳定标志、相机光学坐标中的表面点 |
| `/vision/mango_target` | `robot_arm_interfaces/msg/MangoTarget` | 单个目标的 `base_link` 三维位置（米）、置信度、临时跟踪号及有效标志 |
| `/vision/row_observation` | `robot_arm_interfaces/msg/VisionRowObservation` | 深度范围内芒果和树干横移误差，供底盘停车使用 |
| `/vision/harvest_targets` | `robot_arm_interfaces/msg/HarvestTargetArray` | 停稳后冻结的多个小果、绿色果，包含位置和投影尺寸 |

默认 `publish_targets=false`，识别仍运行并发布观测，摘取目标消息的 `valid=false`。二维图像没有深度，不能生成有效三维摘取目标。当前默认规则要求深度范围内四个果实、左右各两个，左右独立选小果，同时选所有绿色果并去重。原单果话题默认不发布有效目标，需要额外开启 `publish_legacy_target`。训练框内编号不是真实跟踪号；多果目标号在本次停车快照内保持不变。

真实标定后，另存 `config/camera_to_base.json`，其字段格式见 `config/camera_to_base.example.json`。矩阵输入是相机未旋转的彩色光学坐标（X右/Y下/Z前），输出必须是机械臂 `base_link`，平移单位米。示例旋转和平移不能替代实际手眼标定。填入实测矩阵并设置 `calibrated=true` 后：

```bash
bash tools/run_jetson.sh publish_targets:=true \
  calibration:="$PWD/config/camera_to_base.json"
```

此时需要底盘报告停稳、四果选果稳定、深度有效且所选目标置信度至少 0.75，才会发布有效的多果快照。仅开启旧单果兼容模式时，原任务分支才使用前向距离和横向误差门限；该自动摘取分支当前默认关闭。同平台安装的相机外参也仍需实测，URDF 中的占位相机变换不会被这里自动采用。

当前深度点来自果实可见表面，`MangoTarget.center` 沿用该表面点，不能当作已估计的果实体积中心。实际抓取位置需结合现有 `mango_grasp_offset_*_m`、工具尺寸和实物标定验证。

## 训练、权重与 Git

WSL 在源码目录使用原有训练流程：

```bash
cd ros2_ws/src/robot_vision/yolo_mango
python train.py --model ../models/yolo11n-seg.pt --device 0
python evaluate.py --weights ../models/best.pt
python vision.py --weights ../models/best.pt --source test.jpg --device 0
python -m unittest discover -s tests -v
```

桌面的 `runs/segment/mango/weights/best.pt` 已复制到 `models/best.pt`，SHA256 见 `models/manifest.json`。预训练 `yolo11n-seg.pt` 也在 `models/`。数据集复制到 `yolo_mango/datasets/` 供 WSL 继续训练。模型、数据集、生成结果和机器专属标定文件由 Git 忽略；代码、配置、环境安装脚本和权重校验清单可提交 GitHub。新克隆仓库需另行复制权重，随后校验 SHA256。

WSL 和 Jetson 同步同一份源码，虚拟环境及 `build/install/log` 在各机器独立生成。部署校验清单和 GPU 检查结果放在 `deployment/jetson/`；不要把密码、SSH 私钥或虚拟环境提交仓库。
# 横移停车与多果快照

新版通过 `/vision/row_observation` 提供运动中深度门限和树干图像中心误差，
由 `row_navigation_node` 完成芒果触发低速、图像 PI 对中和实测停稳。相机节点不发布底盘速度。
只有收到新鲜的 `/base/vision_stop_state` 状态 HOLDING，才开始累计新采集的选果帧。
单独开相机可观察检测结果；没有停车状态时不会发布有效摘取快照。

先构建 `robot_arm_interfaces`、`robot_chassis`、`robot_vision` 并重新 source。
从本包源码目录启动：

```bash
bash tools/run_jetson.sh publish_targets:=true \
  weights:="$PWD/models/best.pt" \
  calibration:="$PWD/config/camera_to_base.json" \
  decision_config:="$PWD/yolo_mango/configs/vision.json"
```

`config/vision.yaml`：

```yaml
harvest_min_depth_m: 0.40
harvest_max_depth_m: 0.60
selection_stable_frames: 3  # 设 1 为取一帧
tree_center_x_ratio: 0.5  # 旋转后检测画面的目标横坐标，0.5 为图像中央
publish_legacy_target: false
```

深度范围、选果帧数和居中目标支持 rqt 动态修改；树干必须自身落在该深度范围，
无需与芒果做深度关联。例如：`ros2 param set /mango_vision_node harvest_min_depth_m 0.42`。
这些参数在运行中读取；权重、相机源和抓取外参修改仍需重启。
`process_hz` 是处理频率，不等于相机 fps；应通过实际帧间隔与
`vision_observation_timeout_s` 检查 Jetson 推理速度。

选果只统计深度范围内的果实，必须正好四个、画面左右各两个。
`selection_mode: per_side` 时左右分别选择明显小果，允许两侧均有小果；
`tie_relative_tolerance: 0.05` 表示框高相差不超过 5% 算相同大小。
`target_policy: green_or_small` 同时选择绿色果，绿色小果只出现一次。
`unique_small` 模式仍保留原来的全画面唯一小果约束。

`/vision/harvest_targets` 类型为 `robot_arm_interfaces/msg/HarvestTargetArray`：

| 字段 | 含义 |
|---|---|
| `header.frame_id` | 坐标系，默认 `base_link`，由实测相机外参转换 |
| `header.stamp` | 停稳后选定快照的采样时间；重复发布不改变该时间 |
| `stage_id` | 当前路线步骤编号 |
| `valid` | 停稳、选果稳定、深度和坐标有效后为 true |
| `targets[].slot` | `left_top`、`left_bottom`、`right_top`、`right_bottom`，基于旋转后画面 |
| `track_id` | 本次冻结快照内的目标号，不是跨树或跨重启永久 ID |
| `color` / `is_small` / `is_green` | 颜色和选择原因 |
| `surface_point` | 目标可见表面 XYZ，单位米；不等于果实体积中心 |
| `depth_m` | 相机光学深度，单位米 |
| `width_px` / `height_px` | 检测框尺寸，单位像素 |
| `width_m` / `height_m` | 按表面深度和相机内参投影估计的尺寸，单位米 |
| `confidence` | 模型检测置信度 |

`valid=true, targets=[]` 表示四果选果完成但没有小果或绿色果。
外参/深度/置信度不满足时不输出有效坐标；已选快照在保持停车期间固定并重复发布。
失去停稳状态、状态心跳超时、相机错误或参数调整会使快照失效并重新采集。
机械臂后续应订阅这个数组，由调度逻辑选择目标；当前不会发起摘取动作。
旧 `/vision/mango_target` 默认保持无效，避免触发旧的单目标摘取流程。

诊断命令：

```bash
ros2 topic echo /vision/observations --field data --full-length
ros2 topic echo /vision/row_observation
ros2 topic echo /base/vision_stop_state
ros2 topic echo /base/cmd_vel
ros2 topic echo /vision/harvest_targets
```

检查 `mango_in_range`、`tree_valid`、`tree_center_error_ratio` 和状态：
1=芒果触发低速找树，2=图像误差 PI 对中，3=等实测停稳，4=锁定保持，5=故障。
调方向时先在安全条件下验证正负号：`vision_tree_control_sign` 把“树干在画面右侧”
映射到底盘正/负 Y；方向配错会越调越偏。只有抓取目标发布才需要真实相机到机械臂外参。
JSON 中 `trunks` 是树干原始检测，`tree_alignment` 是经过 0.40–0.60 m 深度筛选的
图像居中结果；前者非空但后者为空时检查树干深度及深度掩码。

##### 视觉测试命令
ros2 action send_goal /shift_lane robot_arm_interfaces/action/ShiftLane \
"{stage_id: 6, direction: -1, distance_m: 0.30, max_speed_mps: 0.60, safety_distance_m: 0.04, wall_sensor: 0, wall_target_m: 0.04, wall_enable_tolerance_m: 0.05, disable_wall_control_on_side_distance: true, wall_control_cancel_side_distance_m: 0.25, wall_control_cancel_side_tolerance_m: 0.02, vision_stop_enabled: true}" \
--feedback
# 视觉启动
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision
source tools/env_jetson.sh
cd yolo_mango
python3 vision.py \
  --weights ../models/best.pt \
  --source realsense \
  --device 0 \
  --show



##### 视觉调试步骤，为什么识别到芒果，却没有减速
wrm@wrm-desktop:~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws$ ros2 topic info -v /vision/mango_target
ros2 node list
ros2 node info /row_navigation_node
Type: robot_arm_interfaces/msg/MangoTarget

Publisher count: 1

Node name: mango_vision_node
Node namespace: /
Topic type: robot_arm_interfaces/msg/MangoTarget
Endpoint type: PUBLISHER
GID: 01.0f.c6.a5.a1.9b.5b.52.00.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0

/base_serial_node
/mango_vision_node
/row_navigation_node
/ultrasonic_node
/row_navigation_node
  Subscribers:
    /base/odom: nav_msgs/msg/Odometry
    /base/stage_feedback: robot_arm_interfaces/msg/StageFeedback
    /base/ultrasonic: robot_arm_interfaces/msg/UltrasonicArray
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /vision/row_observation: robot_arm_interfaces/msg/VisionRowObservation
  Publishers:
    /base/cmd_vel: geometry_msgs/msg/Twist
    /base/route_command: robot_arm_interfaces/msg/RouteCommand
    /base/vision_stop_state: robot_arm_interfaces/msg/VisionStopState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /row_navigation_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /row_navigation_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /row_navigation_node/get_parameters: rcl_interfaces/srv/GetParameters
    /row_navigation_node/list_parameters: rcl_interfaces/srv/ListParameters
    /row_navigation_node/set_parameters: rcl_interfaces/srv/SetParameters
    /row_navigation_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /follow_row: robot_arm_interfaces/action/FollowRow
    /shift_lane: robot_arm_interfaces/action/ShiftLane
    /stop_base: robot_arm_interfaces/action/StopBase
  Action Clients:

wrm@wrm-desktop:~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws$ ros2 topic info -v /vision/row_observation
ros2 topic echo /vision/row_observation --once
ros2 interface show robot_arm_interfaces/msg/VisionRowObservation
Type: robot_arm_interfaces/msg/VisionRowObservation

Publisher count: 1

Node name: mango_vision_node
Node namespace: /
Topic type: robot_arm_interfaces/msg/VisionRowObservation
Endpoint type: PUBLISHER
GID: 01.0f.c6.a5.a1.9b.5b.52.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 2

Node name: row_navigation_node
Node namespace: /
Topic type: robot_arm_interfaces/msg/VisionRowObservation
Endpoint type: SUBSCRIPTION
GID: 01.0f.c6.a5.93.9a.12.8b.00.00.00.00.00.00.14.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: _ros2cli_246471
Node namespace: /
Topic type: robot_arm_interfaces/msg/VisionRowObservation
Endpoint type: SUBSCRIPTION
GID: 01.0f.c6.a5.c7.c2.25.a3.00.00.00.00.00.00.05.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

header:
  stamp:
    sec: 1793727185
    nanosec: 39571680
  frame_id: base_link
frame_sequence: 10083
mango_in_range: false
mango_count_in_range: 1
nearest_mango_depth_m: 0.4750000238418579
tree_valid: false
tree_lateral_error_m: 0.0
tree_center_error_ratio: 0.0
tree_confidence: 0.0
---
# Acquisition time of the camera frame, before inference.
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
uint32 frame_sequence
bool mango_in_range
uint32 mango_count_in_range
float64 nearest_mango_depth_m
bool tree_valid
# Distance from the tree ray to the image-center ray, projected on base_link Y.
# Positive means move left. Requires measured camera-to-base rotation.
float64 tree_lateral_error_m
float64 tree_center_error_ratio
float32 tree_confidence
wrm@wrm-desktop:~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws$ grep -RInE --include='*.py' 'mango_in_range|mango_count_in_range|nearest_mango_depth_m|tree_valid' src/robot_vision
grep -RInE 'mango.*(range|distance)|calibration|publish_targets' src/robot_vision/config
src/robot_vision/scripts/mango_vision_node.py:242:            scene.mango_in_range = bool(nearby) and self.matrix is not None and rs is not None
src/robot_vision/scripts/mango_vision_node.py:243:            scene.mango_count_in_range = len(nearby)
src/robot_vision/scripts/mango_vision_node.py:244:            scene.nearest_mango_depth_m = min((f.depth_m for f in nearby), default=0.0)
src/robot_vision/scripts/mango_vision_node.py:246:                scene.tree_valid = True
src/robot_vision/config/vision.yaml:15:    publish_targets: false
src/robot_vision/config/vision.yaml:16:    calibration: ""
wrm@wrm-desktop:~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws$ ^C

