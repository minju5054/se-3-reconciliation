#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_ROOT="${LIGHTNAV_ROOT:-${REPOSITORY_ROOT}/../external/LightNav-0}"
LIGHTNAV_PYTHON="${LIGHTNAV_ROOT}/.venv/bin/python"
RESEARCH_PYTHON="${REPOSITORY_ROOT}/.venv/bin/python"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"
CONFIG="${DATA02_CONFIG:-${REPOSITORY_ROOT}/configs/data02_diverse_lightnav_transitions.yaml}"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 {qualification|primary|smoke} [--run-id ID] [--scenario D0_STRAIGHT|all] [--qualification-run PATH] [--gui] [--no-hold] [--max-attempts N]" >&2
    exit 2
fi
PHASE="$1"
shift
if [[ "${PHASE}" != "qualification" && "${PHASE}" != "primary" && "${PHASE}" != "smoke" ]]; then
    echo "Invalid phase: ${PHASE}" >&2
    exit 2
fi

RUN_ID=""
SCENARIO="all"
QUALIFICATION_RUN=""
GUI=0
NO_HOLD=0
MAX_ATTEMPTS=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id) RUN_ID="$2"; shift 2 ;;
        --scenario) SCENARIO="$2"; shift 2 ;;
        --qualification-run) QUALIFICATION_RUN="$2"; shift 2 ;;
        --max-attempts) MAX_ATTEMPTS="$2"; shift 2 ;;
        --gui) GUI=1; shift ;;
        --no-hold) NO_HOLD=1; shift ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
if [[ -z "${RUN_ID}" ]]; then
    RUN_ID="data02-${PHASE}-$(date -u +%Y%m%dT%H%M%SZ)"
fi
if [[ "${PHASE}" == "primary" && -z "${QUALIFICATION_RUN}" ]]; then
    echo "Primary collection requires --qualification-run PATH" >&2
    exit 2
fi

for executable in "${LIGHTNAV_PYTHON}" "${RESEARCH_PYTHON}" "${ISAAC_PYTHON}"; do
    if [[ ! -x "${executable}" ]]; then
        echo "Required Python launcher is unavailable: ${executable}" >&2
        exit 1
    fi
done

OUTPUT_DIR="${REPOSITORY_ROOT}/data/data02_lightnav_diverse_transitions/${RUN_ID}"
RUNTIME_DIR="$(mktemp -d -t data02-ipc-XXXXXX)"
SOCKET_PATH="${RUNTIME_DIR}/lightnav.sock"
READY_FILE="${RUNTIME_DIR}/ready.json"
SERVER_LOG="${RUNTIME_DIR}/lightnav-server.log"
server_pid=""

cleanup() {
    if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
        kill "${server_pid}" 2>/dev/null || true
        wait "${server_pid}" 2>/dev/null || true
    fi
    rm -f "${SOCKET_PATH}" "${READY_FILE}"
    if [[ -s "${SERVER_LOG}" ]]; then
        echo "[DATA-02] LightNav server log retained: ${SERVER_LOG}"
    else
        rm -f "${SERVER_LOG}"
        rmdir "${RUNTIME_DIR}" 2>/dev/null || true
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

client_args=(--socket "${SOCKET_PATH}" --run-id "${RUN_ID}" --phase "${PHASE}" --config "${CONFIG}" --scenario "${SCENARIO}")
if [[ -n "${MAX_ATTEMPTS}" ]]; then
    client_args+=(--max-attempts "${MAX_ATTEMPTS}")
fi
if (( GUI )); then
    client_args+=(--gui)
fi
if (( NO_HOLD )); then
    client_args+=(--no-hold)
fi

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
    PYTHONUNBUFFERED=1 "${ISAAC_PYTHON}" \
    scripts/isaac/data02_diverse_transition_collection.py "${client_args[@]}"

wait "${server_pid}"
server_pid=""
if [[ "${PHASE}" == "qualification" ]]; then
    env -u PYTHONPATH "${RESEARCH_PYTHON}" scripts/summarize_data02_transitions.py "${OUTPUT_DIR}"
elif [[ "${PHASE}" == "primary" ]]; then
    env -u PYTHONPATH "${RESEARCH_PYTHON}" scripts/summarize_data02_transitions.py \
        "${OUTPUT_DIR}" --qualification-run "${QUALIFICATION_RUN}"
fi
echo "[DATA-02] Complete: ${OUTPUT_DIR}"
