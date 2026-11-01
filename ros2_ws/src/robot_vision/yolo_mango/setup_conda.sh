#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "Activate a Conda environment first: conda activate mango" >&2
    exit 1
fi
if [ "${CONDA_DEFAULT_ENV:-}" = "base" ]; then
    echo "Create a project environment instead of base: conda create -n mango python=3.10 pip" >&2
    exit 1
fi
python_bin="$CONDA_PREFIX/bin/python"
if [ ! -x "$python_bin" ] || [ ! -d "$CONDA_PREFIX/conda-meta" ]; then
    echo "Expected an activated Linux Conda environment. In WSL, use Linux Conda." >&2
    exit 1
fi
"$python_bin" -c 'import sys; assert sys.version_info >= (3, 10), "Python 3.10+ required"'
"$python_bin" -m pip --version
"$python_bin" -m pip install torch==2.7.1 torchvision==0.22.1 \
    --index-url "${MANGO_TORCH_INDEX:-https://download.pytorch.org/whl/cu118}"
"$python_bin" -m pip install -r requirements-realsense.txt
"$python_bin" -m pip check
"$python_bin" check_environment.py
echo "Ready. Keep this Conda environment active when running bash run.sh."