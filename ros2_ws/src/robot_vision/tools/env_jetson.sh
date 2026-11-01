#!/usr/bin/env bash
# Source this file to use the isolated GPU environment with ROS 2 Humble.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Use: source tools/env_jetson.sh' >&2
  exit 1
fi
export MANGO_VENV="${MANGO_VENV:-$HOME/venvs/mango_gpu}"
if [[ ! -x "$MANGO_VENV/bin/python" ]]; then
  echo "Missing GPU environment: $MANGO_VENV" >&2
  return 1
fi
source "$MANGO_VENV/bin/activate"
export PYTHONNOUSERSITE=1
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="$MANGO_VENV/lib/python3.10/site-packages/nvidia/cu12/lib:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
