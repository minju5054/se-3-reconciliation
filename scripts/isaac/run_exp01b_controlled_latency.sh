#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_ROOT="${LIGHTNAV_ROOT:-${REPOSITORY_ROOT}/../external/LightNav-0}"
LIGHTNAV_PYTHON="${LIGHTNAV_ROOT}/.venv/bin/python"
RESEARCH_PYTHON="${REPOSITORY_ROOT}/.venv/bin/python"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"
CONFIG="${EXP01B_CONTROLLED_CONFIG:-${REPOSITORY_ROOT}/configs/exp01b_controlled_latency.yaml}"
PHASE="${1:-primary}"
if [[ "${PHASE}" != "qualification" && "${PHASE}" != "smoke" && "${PHASE}" != "primary" ]]; then
    echo "Usage: $0 {qualification|smoke|primary} [cell-or-geometry-id]" >&2
    exit 2
fi
shift || true
CELL="${1:-}"
if [[ "${PHASE}" == "smoke" && -z "${CELL}" ]]; then
    echo "Smoke phase requires one exact cell id, for example G0_straight__L0_natural" >&2
    exit 2
fi
RUN_ID="${EXP01B_CONTROLLED_ID:-exp01b-controlled-${PHASE}-$(date -u +%Y%m%dT%H%M%SZ)}"
OUTPUT_DIR="${REPOSITORY_ROOT}/data/exp01b_redesign/${RUN_ID}"
RUNTIME_DIR="$(mktemp -d -t exp01b-controlled-ipc-XXXXXX)"
SOCKET_PATH="${RUNTIME_DIR}/lightnav.sock"
READY_FILE="${RUNTIME_DIR}/ready.json"
SERVER_LOG="${RUNTIME_DIR}/lightnav-server.log"

for executable in "${LIGHTNAV_PYTHON}" "${RESEARCH_PYTHON}" "${ISAAC_PYTHON}"; do
    if [[ ! -x "${executable}" ]]; then
        echo "Required Python launcher is unavailable: ${executable}" >&2
        exit 1
    fi
done

server_pid=""
cleanup() {
    if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
        kill "${server_pid}" 2>/dev/null || true
        wait "${server_pid}" 2>/dev/null || true
    fi
    rm -f "${SOCKET_PATH}" "${READY_FILE}"
    if [[ -s "${SERVER_LOG}" ]]; then
        echo "[EXP-01B Controlled] LightNav server log: ${SERVER_LOG}"
    else
        rm -f "${SERVER_LOG}"
    fi
}
trap cleanup EXIT INT TERM

cd "${REPOSITORY_ROOT}"
env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
    VLN_KV_CACHE_GIB="${VLN_KV_CACHE_GIB:-1}" \
    VLN_VLLM_ENFORCE_EAGER="${VLN_VLLM_ENFORCE_EAGER:-1}" \
    PYTHONUNBUFFERED=1 \
    "${LIGHTNAV_PYTHON}" scripts/lightnav/serve_online_lightnav.py \
    --socket "${SOCKET_PATH}" --ready-file "${READY_FILE}" --config "${CONFIG}" \
    >"${SERVER_LOG}" 2>&1 &
server_pid=$!

deadline=$((SECONDS + 180))
while [[ ! -s "${READY_FILE}" ]]; do
    if ! kill -0 "${server_pid}" 2>/dev/null; then
        echo "LightNav server exited before readiness" >&2
        sed -n '1,240p' "${SERVER_LOG}" >&2
        exit 1
    fi
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for warmed LightNav server" >&2
        sed -n '1,240p' "${SERVER_LOG}" >&2
        exit 1
    fi
    sleep 0.25
done

client_args=(--socket "${SOCKET_PATH}" --experiment-id "${RUN_ID}" --config "${CONFIG}" --controlled-phase "${PHASE}")
if [[ -n "${CELL}" ]]; then
    client_args+=(--controlled-cell "${CELL}")
fi
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
    PYTHONUNBUFFERED=1 "${ISAAC_PYTHON}" scripts/isaac/exp01b_online_raw_switch.py \
    "${client_args[@]}"

wait "${server_pid}"
server_pid=""
if [[ "${PHASE}" == "primary" ]]; then
    env -u PYTHONPATH "${RESEARCH_PYTHON}" scripts/summarize_exp01b_controlled_latency.py "${OUTPUT_DIR}"
fi
echo "[EXP-01B Controlled] Complete: ${OUTPUT_DIR}"
