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

| 话题 | 消息 | 内容 |
|---|---|---|
| `/vision/observations` | `std_msgs/msg/String` | JSON，颜色、框高、选果结果、稳定标志、相机光学坐标中的表面点 |
| `/vision/mango_target` | `robot_arm_interfaces/msg/MangoTarget` | 单个目标的 `base_link` 三维位置（米）、置信度、临时跟踪号及有效标志 |

默认 `publish_targets=false`，识别仍运行并发布观测，目标消息的 `valid=false`。二维图像没有深度，不能生成有效三维摘取目标。选果规则要求四个果实、左右各两个，默认仅选择唯一小果；规则详见原始 [yolo_mango/README.md](yolo_mango/README.md)。训练框内编号不是真实跟踪号；ROS 适配层按目标槽位与空间位置关联临时编号。

真实标定后，另存 `config/camera_to_base.json`，其字段格式见 `config/camera_to_base.example.json`。矩阵输入是相机未旋转的彩色光学坐标（X右/Y下/Z前），输出必须是机械臂 `base_link`，平移单位米。示例旋转和平移不能替代实际手眼标定。填入实测矩阵并设置 `calibrated=true` 后：

```bash
bash tools/run_jetson.sh publish_targets:=true \
  calibration:="$PWD/config/camera_to_base.json"
```

此时只有稳定、单个、深度有效、置信度至少 0.75 的目标会标记有效。现有任务节点还会检查前向距离 0.30–1.50 m、横向误差不超过 0.10 m 和连续帧数；实际值见 `robot_chassis/config/chassis.yaml`。同平台安装的相机外参也仍需实测，URDF 中的占位相机变换不会被这里自动采用。

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
