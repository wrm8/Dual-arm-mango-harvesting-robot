# Jetson 部署记录（2026-11-01，Asia/Shanghai）

Jetson：`wrm@192.168.31.91`，Orin Nano 8GB，Ubuntu 22.04.5，L4T R36.5.0，CUDA 12.6，ROS 2 Humble。WSL 使用 ROS 2 Jazzy，源码相同，构建产物和 Python 环境分别生成。

| 项目 | 路径 |
|---|---|
| WSL 源码 | `/home/wrm/Dual-arm-mango-harvesting-robot` |
| Jetson 源码 | `/home/wrm/wrm/Dual-arm-mango-harvesting-robot` |
| Jetson GPU 环境 | `/home/wrm/venvs/mango_gpu` |
| 部署前 Jetson 备份 | `/home/wrm/wrm/deployment-backups/20261101-before-vision/source.tar.gz` |
| Jetson 原有改动补丁 | `/home/wrm/wrm/deployment-backups/20261101-before-vision/tracked-changes.patch` |

原 Jetson 的两个独有超声波源码已移入该备份目录的 `extra-files/`，完整副本也在压缩包中，可恢复。WSL 原有的底盘、机械臂和超声波改动保留，并作为同步来源。`.git`、ROS 构建目录和虚拟环境未被源码覆盖。

视觉来源：Windows 桌面 `yolo_mango-main`。训练、评估和推理代码迁入 `ros2_ws/src/robot_vision/yolo_mango`；261 张训练/验证图片及模型在两端本地保存，按既有规则忽略于 Git。已训练权重位于 `ros2_ws/src/robot_vision/models/best.pt`，SHA256 为 `4634485486b23499e6d23ae35d34e86f1289fc216f069c330981cc7632e8321d`。

## GPU 修复与验证

系统驱动和 CUDA 原本可用；GPU 环境缺少 cuDSS 动态库及其加载路径，用户级 `~/.local` 还存在 CUDA 13 版 PyTorch。现使用隔离环境、禁止用户级包覆盖，并明确加载 Jetson CUDA 12.6 和 cuDSS。

安装脚本专门避免 PyPI 通用 ARM/SBSA cuBLAS 覆盖 Jetson 库。已观察到该覆盖导致 `CUBLAS_STATUS_ALLOC_FAILED`，恢复 `/usr/local/cuda-12.6/targets/aarch64-linux/lib/` 下的 cuBLAS 后实际矩阵计算恢复。修复后的脚本已在设备上运行验证；setuptools 固定为 78.1.0，以满足 PyTorch 和 colcon 的共同要求。

实测版本：PyTorch 2.11.0、torchvision 0.26.0、Ultralytics 8.4.148、NumPy 1.26.4、OpenCV 4.8.1、cuDSS 0.7.1.6。设备实际 GPU 检查输出见 [gpu-check.json](gpu-check.json)。[pip-freeze.local.txt](pip-freeze.local.txt) 为此设备虚拟环境快照，供审计；重建环境请使用包中的 `tools/setup_jetson.sh`，不要直接对快照执行 pip install。

已通过：

- WSL 和 Jetson 均构建 `robot_arm_interfaces`、`robot_vision`。
- 两端均通过原有视觉 49 项测试及新增目标坐标转换 5 项测试。
- CUDA 矩阵计算、卷积反向传播、torchvision GPU NMS、YOLO11 分割网络前后向计算。
- 真实 `best.pt` 在 Jetson GPU 上处理验证集图片，识别 4 个果实，并通过 ROS 2 订阅者收到观测和目标消息。该图的选果规则返回 `uncertain`，未发布有效摘取目标。
- 一键 ROS launch、真实图片 GPU 推理和退出清理；测试运行在 ROS_DOMAIN_ID=73，不启动底盘或机械臂节点。
- GPU 环境 `pip check` 无依赖错误。

源码一致性范围包括 `.gitignore`、`ros2_ws/src`、`servo_test_jetson`、`02 程序源码` 及本部署记录，包含本地模型和数据，排除 Python 缓存、训练输出、ROS 构建产物和 Git 元数据。最终逐文件清单保存在 [source.sha256](source.sha256)，可在各自仓库根目录执行 `sha256sum -c deployment/jetson/source.sha256`。

## 启动与待验证项

连接 D435i 后，在 Jetson 执行：

```bash
cd ~/wrm/Dual-arm-mango-harvesting-robot/ros2_ws/src/robot_vision
bash tools/run_jetson.sh
```

启动、训练、标定和 ROS 接口说明见 [robot_vision/README.md](../../ros2_ws/src/robot_vision/README.md)。截至本次检查，Jetson 未发现 RealSense 设备；相机实时深度、相机到机械臂的实测外参及实物摘取尚未验证。默认 `publish_targets=false`，不把未标定坐标发送给摘取流程。未配置开机自动启动；测试进程已结束。此次未刷新系统镜像，也未升级系统 GPU 驱动。
