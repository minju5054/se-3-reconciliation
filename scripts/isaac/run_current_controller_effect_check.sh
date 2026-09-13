#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
isaac_root="${ISAACSIM_ROOT:-$HOME/isaacsim}"
cd "$repo_root"
exec env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  ISAAC_SIM_ROOT="$isaac_root" PYTHONUNBUFFERED=1 \
  "$isaac_root/python.sh" scripts/isaac/current_controller_effect_check.py "$@"
