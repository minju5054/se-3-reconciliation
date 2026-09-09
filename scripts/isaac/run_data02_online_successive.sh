#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_ROOT="${LIGHTNAV_ROOT:-${REPOSITORY_ROOT}/../external/LightNav-0}"
LIGHTNAV_PYTHON="${LIGHTNAV_ROOT}/.venv/bin/python"
ISAAC_PYTHON="${ISAACSIM_ROOT:-${HOME}/isaacsim}/python.sh"
CONFIG="${DATA02_CONFIG:-${REPOSITORY_ROOT}/configs/data02_online_successive_v1.yaml}"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
    echo "Isaac Sim launcher unavailable: ${ISAAC_PYTHON}" >&2
    exit 1
fi

replay=false
preview=false
for argument in "$@"; do
    [[ "${argument}" == "--replay-run" ]] && replay=true
    [[ "${argument}" == "--phase=preview" ]] && preview=true
done
if [[ "${1:-}" == "--phase" && "${2:-}" == "preview" ]]; then
    preview=true
fi

isaac_env=(
    env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH
    -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION
    ISAAC_SIM_ROOT="${ISAACSIM_ROOT:-${HOME}/isaacsim}"
    PYTHONUNBUFFERED=1
)

if [[ "${replay}" == true || "${preview}" == true ]]; then
    cd "${REPOSITORY_ROOT}"
    "${isaac_env[@]}" "${ISAAC_PYTHON}" \
        "${REPOSITORY_ROOT}/scripts/isaac/data02_online_successive.py" \
        --config "${CONFIG}" "$@"
    exit 0
fi

if [[ ! -x "${LIGHTNAV_PYTHON}" ]]; then
    echo "LightNav Python unavailable: ${LIGHTNAV_PYTHON}" >&2
    exit 1
fi

RUNTIME_DIR="$(mktemp -d -t data02-online-successive-XXXXXX)"
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
        echo "[DATA-02] LightNav log: ${SERVER_LOG}"
    fi
}
trap cleanup EXIT INT TERM

cd "${REPOSITORY_ROOT}"
env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
    VLN_KV_CACHE_GIB="${VLN_KV_CACHE_GIB:-1}" \
    VLN_VLLM_ENFORCE_EAGER="${VLN_VLLM_ENFORCE_EAGER:-1}" \
    PYTHONUNBUFFERED=1 \
    "${LIGHTNAV_PYTHON}" "${REPOSITORY_ROOT}/scripts/lightnav/serve_online_lightnav.py" \
    --socket "${SOCKET_PATH}" --ready-file "${READY_FILE}" --config "${CONFIG}" \
    >"${SERVER_LOG}" 2>&1 &
server_pid=$!

deadline=$((SECONDS + 240))
while [[ ! -s "${READY_FILE}" ]]; do
    if ! kill -0 "${server_pid}" 2>/dev/null; then
        echo "LightNav server exited before readiness" >&2
        sed -n '1,260p' "${SERVER_LOG}" >&2
        exit 1
    fi
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for warmed LightNav server" >&2
        sed -n '1,260p' "${SERVER_LOG}" >&2
        exit 1
    fi
    sleep 0.25
done

echo "[DATA-02] One persistent LightNav process is loaded and warmed."
sed -n '/EXP01B_LIGHTNAV_READY=/p' "${SERVER_LOG}"
"${isaac_env[@]}" "${ISAAC_PYTHON}" \
    "${REPOSITORY_ROOT}/scripts/isaac/data02_online_successive.py" \
    --socket "${SOCKET_PATH}" --config "${CONFIG}" "$@"

wait "${server_pid}"
server_pid=""
echo "[DATA-02] LightNav server completed cleanly."
