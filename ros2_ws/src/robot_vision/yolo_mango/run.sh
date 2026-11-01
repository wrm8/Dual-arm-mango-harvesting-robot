#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
command_name="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$command_name" in
    help|-h|--help)
        echo "Usage: bash run.sh {check|train|evaluate|detect|capture|test|demo} [arguments]"
        exit 0 ;;
    check|train|evaluate|detect|capture|test|demo) ;;
    *) echo "Unknown command: $command_name" >&2; exit 2 ;;
esac
# Explicit venv override, then active Conda, then the project's original venv.
if [ -n "${MANGO_VENV:-}" ]; then
    python_bin="$MANGO_VENV/bin/python"
elif [ -n "${CONDA_PREFIX:-}" ]; then
    python_bin="$CONDA_PREFIX/bin/python"
else
    python_bin=".venv/bin/python"
fi
if ! "$python_bin" --version >/dev/null 2>&1; then
    echo "Python unavailable. Activate your Linux Conda environment or run bash setup_wsl.sh." >&2
    exit 1
fi
case "$command_name" in
    check) exec "$python_bin" train.py --check-only "$@" ;;
    train) exec "$python_bin" train.py "$@" ;;
    evaluate) exec "$python_bin" evaluate.py "$@" ;;
    detect) exec "$python_bin" vision.py "$@" ;;
    capture) exec "$python_bin" capture_d435i.py "$@" ;;
    test) exec "$python_bin" -m unittest discover -s tests -v "$@" ;;
    demo) exec "$python_bin" demo.py "$@" ;;
esac