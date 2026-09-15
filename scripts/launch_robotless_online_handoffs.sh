#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh "$REPO_ROOT/scripts/isaac/robotless_online_handoffs.py" "$@"
