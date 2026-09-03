#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_ROOT="${LIGHTNAV_ROOT:-${REPOSITORY_ROOT}/../external/LightNav-0}"
LIGHTNAV_PYTHON="${LIGHTNAV_ROOT}/.venv/bin/python"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"
CONFIG="${REPOSITORY_ROOT}/configs/exp01b_online_raw_switch.yaml"
EXPERIMENT_ID="${EXP01B_EXPERIMENT_ID:-exp01b-$(date -u +%Y%m%dT%H%M%SZ)}"
RUNTIME_DIR="$(mktemp -d -t exp01b-ipc-XXXXXX)"
SOCKET_PATH="${RUNTIME_DIR}/lightnav.sock"
READY_FILE="${RUNTIME_DIR}/ready.json"
SERVER_LOG="${RUNTIME_DIR}/lightnav-server.log"

if [[ ! -x "${LIGHTNAV_PYTHON}" ]]; then
    echo "LightNav Python is unavailable: ${LIGHTNAV_PYTHON}" >&2
    exit 1
fi
if [[ ! -x "${ISAAC_PYTHON}" ]]; then
    echo "Isaac Sim launcher is unavailable: ${ISAAC_PYTHON}" >&2
    exit 1
fi

server_pid=""
cleanup() {
    if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
        kill "${server_pid}" 2>/dev/null || true
        wait "${server_pid}" 2>/dev/null || true
    fi
    rm -f "${SOCKET_PATH}" "${READY_FILE}"
    if [[ -s "${SERVER_LOG}" ]]; then
        echo "[EXP-01B] LightNav server log retained for this shell run: ${SERVER_LOG}"
    else
        rm -f "${SERVER_LOG}"
    fi
}
trap cleanup EXIT INT TERM

cd "${REPOSITORY_ROOT}"
env \
    CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
    VLN_KV_CACHE_GIB="${VLN_KV_CACHE_GIB:-1}" \
    VLN_VLLM_ENFORCE_EAGER="${VLN_VLLM_ENFORCE_EAGER:-1}" \
    PYTHONUNBUFFERED=1 \
    "${LIGHTNAV_PYTHON}" \
    "${REPOSITORY_ROOT}/scripts/lightnav/serve_online_lightnav.py" \
    --socket "${SOCKET_PATH}" \
    --ready-file "${READY_FILE}" \
    --config "${CONFIG}" >"${SERVER_LOG}" 2>&1 &
server_pid=$!

deadline=$((SECONDS + 180))
while [[ ! -s "${READY_FILE}" ]]; do
    if ! kill -0 "${server_pid}" 2>/dev/null; then
        echo "LightNav server exited before readiness" >&2
        sed -n '1,240p' "${SERVER_LOG}" >&2
        exit 1
    fi
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for the warmed LightNav server" >&2
        sed -n '1,240p' "${SERVER_LOG}" >&2
        exit 1
    fi
    sleep 0.25
done

echo "[EXP-01B] Persistent LightNav server is loaded and warmed."
sed -n '/EXP01B_LIGHTNAV_READY=/p' "${SERVER_LOG}"

env \
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
    PYTHONUNBUFFERED=1 \
    "${ISAAC_PYTHON}" \
    "${REPOSITORY_ROOT}/scripts/isaac/exp01b_online_raw_switch.py" \
    --socket "${SOCKET_PATH}" \
    --experiment-id "${EXPERIMENT_ID}" \
    --config "${CONFIG}" \
    "$@"

wait "${server_pid}"
server_pid=""
echo "[EXP-01B] LightNav server completed cleanly."
