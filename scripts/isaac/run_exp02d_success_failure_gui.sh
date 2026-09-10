#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
    echo "Isaac Sim launcher unavailable: ${ISAAC_PYTHON}" >&2
    exit 1
fi

cd "${REPOSITORY_ROOT}"
exec env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
    ISAAC_SIM_ROOT="${ISAACSIM_ROOT:-${HOME}/isaacsim}" \
    PYTHONUNBUFFERED=1 \
    "${ISAAC_PYTHON}" "${SCRIPT_DIR}/exp02d_success_failure_gui.py" "$@"
