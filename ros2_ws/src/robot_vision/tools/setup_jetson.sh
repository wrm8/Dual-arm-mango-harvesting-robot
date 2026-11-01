#!/usr/bin/env bash
set -eo pipefail
VISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export MANGO_VENV="${MANGO_VENV:-$HOME/venvs/mango_gpu}"
if [[ "$(uname -m)" != aarch64 ]] || [[ ! -d /usr/local/cuda-12.6 ]]; then
  echo 'This installer requires a Jetson with an existing CUDA 12.6 installation.' >&2
  exit 1
fi
if [[ ! -x "$MANGO_VENV/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$MANGO_VENV"
fi
export PYTHONNOUSERSITE=1
"$MANGO_VENV/bin/python" -m pip install --index-url https://pypi.org/simple 'pip>=24,<27' 'setuptools==78.1.0'
# The Jetson wheels use the system CUDA stack, avoiding desktop CUDA 13 packages.
if ! LD_LIBRARY_PATH="$MANGO_VENV/lib/python3.10/site-packages/nvidia/cu12/lib:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$MANGO_VENV/bin/python" -c 'import torch, torchvision; assert torch.__version__ == "2.11.0" and torch.version.cuda == "12.6" and torchvision.__version__ == "0.26.0"' >/dev/null 2>&1; then
  "$MANGO_VENV/bin/python" -m pip install --force-reinstall --no-deps \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126/+simple torch==2.11.0 torchvision==0.26.0
fi
# Generic PyPI ARM cuBLAS wheels target SBSA; use the Jetson system cuBLAS.
if "$MANGO_VENV/bin/python" -m pip show nvidia-cublas-cu12 >/dev/null 2>&1; then
  "$MANGO_VENV/bin/python" -m pip uninstall -y nvidia-cublas-cu12
fi
"$MANGO_VENV/bin/python" -m pip install --no-deps --index-url https://pypi.org/simple \
  nvidia-cudss-cu12==0.7.1.6
"$MANGO_VENV/bin/python" -m pip install --index-url https://pypi.org/simple \
  -r "$VISION_ROOT/requirements-jetson.txt"
source "$VISION_ROOT/tools/env_jetson.sh"
python "$VISION_ROOT/yolo_mango/check_environment.py" --require-cuda
