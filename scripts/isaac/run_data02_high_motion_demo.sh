#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"
REPOSITORY_PYTHON="${REPOSITORY_ROOT}/.venv/bin/python"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
    echo "Isaac Sim launcher unavailable: ${ISAAC_PYTHON}" >&2
    exit 1
fi
if [[ ! -x "${REPOSITORY_PYTHON}" ]]; then
    echo "Repository Python unavailable: ${REPOSITORY_PYTHON}" >&2
    exit 1
fi

cd "${REPOSITORY_ROOT}"
PYTHONPATH="${REPOSITORY_ROOT}/src" "${REPOSITORY_PYTHON}" \
    -m reconciliation.data02_high_motion_demo --repository-root "${REPOSITORY_ROOT}"

exec env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
    ISAAC_SIM_ROOT="${ISAACSIM_ROOT:-${HOME}/isaacsim}" \
    PYTHONUNBUFFERED=1 \
    "${ISAAC_PYTHON}" "${SCRIPT_DIR}/data02_high_motion_demo.py" "$@"
