# 训练与环境说明

## 当前检查结论

项目使用三类 YOLO 实例分割：0=绿芒果、1=树干、2=黄芒果；推理时树干会从芒果大小判断中排除。
261 张图片已按采集批次拆分为 190 张训练图和 71 张验证图，并完成 100 轮 YOLO11n-seg 训练。
最佳权重为 `runs/segment/mango/weights/best.pt`；验证集 mask mAP50=0.938、mAP50-95=0.826。相机/实拍流程尚未验证。

本次训练使用 Windows Python 3.14.7、PyTorch 2.12.1+cu130、torchvision 0.27.1+cu130、Ultralytics 8.4.190 和 RTX 5060 Laptop GPU。
当前 GPU 环境已通过卷积反向传播、torchvision NMS 与 YOLO 分割前后向检查。

## 重建训练环境

```bash
cd /home/wrm/yolo_mango
bash setup_wsl.sh
source .venv/bin/activate
python check_environment.py
# 必须使用 GPU 时额外执行：
python check_environment.py --require-cuda
```

脚本把不可用环境改名保留，不删除旧环境。
依赖通过 pip 安装到项目 `.venv`，不会修改系统 Python 包。
Python 没有 ensurepip 时，从 Python Packaging 官方地址下载 get-pip 引导文件。
需要 curl 和可访问 PyPI、PyTorch 下载源的网络。
当前任务没有安装完整训练依赖或重新验证 GPU；环境检查以你执行脚本后的结果为准。

默认 PyTorch 2.7.1、torchvision 0.22.1，CUDA 11.8；
CPU 安装可设置 `MANGO_TORCH_INDEX=https://download.pytorch.org/whl/cpu`。
其他依赖来自 `requirements.txt`，不自动套用不同 Python 环境的旧锁文件。
训练完成后可用 `python -m pip freeze > requirements-local.lock.txt` 保存当前环境版本。

## 数据与训练

按 [README.md](README.md) 放入训练/验证照片及多边形标签。
类别 0 是绿芒果，1 是树干，2 是黄芒果；小果仍按颜色标注。

```bash
python train.py --check-only
python train.py --epochs 100 --batch 4 --imgsz 640 --device 0
python evaluate.py --weights runs/segment/mango/weights/best.pt --device 0
```

没有数据时检查会非零退出并说明空目录，这是预期行为。
显存不足可降低 batch，例如 `--batch 2`；没有 GPU 可指定 `--device cpu`。
预训练权重需要下载或自行准备，可用 `--model /path/to/yolo11n-seg.pt` 指定。
完全离线时可用 `--model yolo11n-seg.yaml` 从随机参数训练，但通常需要更多训练数据。
训练输出 `runs/segment/mango*/weights/best.pt`，重复运行目录可能递增。

训练关闭色相变化、Mosaic、MixUp；大小完全由运行时高度规则决定。
最终使用现场四果照片核对：颜色、左右分组、框高、0/1 小果结果，以及相同果实的高度误差容限。
## 使用 Conda（可替代 setup_wsl.sh）

如果习惯 Conda，可用它管理 Python 版本和环境。模型、训练方式、D435i 用法保持相同。
在 WSL Ubuntu 中运行项目时，请使用 Ubuntu 内的 Linux 版 Conda；Windows 中的 Conda 环境不能直接作为此处的 Linux 环境。

在 Ubuntu 终端执行：

```bash
conda --version
conda create -n mango python=3.10 pip -y
conda activate mango
cd /home/wrm/yolo_mango
bash setup_conda.sh
```

若找不到 conda，先安装/初始化 Linux 版 Miniconda 或 Miniforge，再重新打开 Ubuntu 终端。
`setup_conda.sh` 安装现有版本的 PyTorch、YOLO、OpenCV 和 D435i 的 Python 依赖，
并运行依赖与模型环境检查；它要求先激活项目环境，不向 base 安装。
使用 Conda 时无需再运行 `setup_wsl.sh` 或激活 `.venv`。

之后每次使用：

```bash
conda activate mango
cd /home/wrm/yolo_mango
bash run.sh capture --count 100 --show
bash run.sh check
bash run.sh train --epochs 100 --batch 4
```

采集窗口中按空格键同时保存当前 RGB 和深度画面，按 Q 或 Ctrl+C 停止；达到 `--count` 组后会自动结束。每次按键保存一组，不按时间间隔自动拍摄。默认保存在桌面 `D435i_Captures/` 下新建的时间戳目录中，分别放入 `rgb/` 和 `depth/`；深度图已对齐到彩色画面，16 位 PNG 的实际距离换算比例记录在 `session.json`。

相机竖直安装时，采集脚本默认将画面顺时针旋转为竖幅；如现场方向相反，可使用 `--rotate ccw`，不需要旋转时使用 `--rotate none`。检测命令同样使用 `--rotate cw`，这样模型画面、深度图和像素坐标保持一致。

`run.sh` 使用已激活 Conda 环境的 Python；若设置了 `MANGO_VENV`，该显式指定优先。
未激活 Conda 且未指定 `MANGO_VENV` 时，才使用项目 `.venv`。
纯 CPU 安装可执行：
`MANGO_TORCH_INDEX=https://download.pytorch.org/whl/cpu bash setup_conda.sh`。

Conda 管理 Python/pip，训练与相机依赖统一由该环境内的 pip 安装。
不要接着用 conda 在同一个环境中重新安装另一套 PyTorch。
官方参考：[Conda 环境管理](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html)、
[PyTorch 版本安装命令](https://docs.pytorch.org/get-started/previous-versions/)。
