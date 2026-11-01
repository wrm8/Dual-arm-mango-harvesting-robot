#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
env_dir="${MANGO_VENV:-.venv}"
if [ -e "$env_dir" ] && ! "$env_dir/bin/python" -m pip --version >/dev/null 2>&1; then
    backup="$env_dir.broken.$(date +%Y%m%d%H%M%S).$$"
    mv -- "$env_dir" "$backup"
    echo "Preserved unusable environment at $backup"
fi
if [ ! -d "$env_dir" ]; then
    python3 -m venv --without-pip "$env_dir"
fi
if ! "$env_dir/bin/python" -m pip --version >/dev/null 2>&1; then
    if ! "$env_dir/bin/python" -m ensurepip --upgrade; then
        bootstrap_file="$(mktemp ./get-pip.XXXXXX.py)"
        trap 'rm -f -- "$bootstrap_file"' EXIT
        curl -fSL --connect-timeout 20 --max-time 180 \
            https://bootstrap.pypa.io/get-pip.py -o "$bootstrap_file"
        "$env_dir/bin/python" "$bootstrap_file"
        rm -f -- "$bootstrap_file"
        trap - EXIT
    fi
fi
"$env_dir/bin/python" -m pip install --upgrade pip
"$env_dir/bin/python" -m pip install torch==2.7.1 torchvision==0.22.1 \
    --index-url "${MANGO_TORCH_INDEX:-https://download.pytorch.org/whl/cu118}"
"$env_dir/bin/python" -m pip install -r requirements.txt
"$env_dir/bin/python" -m pip check
"$env_dir/bin/python" check_environment.py
echo "Ready. Activate with: source $env_dir/bin/activate"