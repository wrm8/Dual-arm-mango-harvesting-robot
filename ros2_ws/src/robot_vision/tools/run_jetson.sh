#!/usr/bin/env bash
set -eo pipefail
VISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$VISION_ROOT/../.." && pwd)"
source "$VISION_ROOT/tools/env_jetson.sh"
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo 'Build robot_arm_interfaces and robot_vision first; see README.md.' >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"
exec ros2 launch robot_vision vision.launch.py "$@"
