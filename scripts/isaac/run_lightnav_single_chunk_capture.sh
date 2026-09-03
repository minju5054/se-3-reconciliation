#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"

if [[ ! -x "${ISAAC_SIM_ROOT}/python.sh" ]]; then
    echo "Isaac Sim launcher is not executable: ${ISAAC_SIM_ROOT}/python.sh" >&2
    exit 1
fi

cd "${REPOSITORY_ROOT}"
exec env \
    -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    -u CUDA_HOME \
    -u CUDA_PATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u RMW_IMPLEMENTATION \
    ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT}" \
    PYTHONUNBUFFERED=1 \
    "${ISAAC_SIM_ROOT}/python.sh" \
    "${REPOSITORY_ROOT}/scripts/isaac/lightnav_capture_observation.py" \
    "$@"
