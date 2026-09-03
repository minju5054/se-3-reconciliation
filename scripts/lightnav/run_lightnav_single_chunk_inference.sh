#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_ROOT="${LIGHTNAV_ROOT:-${REPOSITORY_ROOT}/../external/LightNav-0}"
LIGHTNAV_PYTHON="${LIGHTNAV_ROOT}/.venv/bin/python"

if [[ ! -x "${LIGHTNAV_PYTHON}" ]]; then
    echo "LightNav Python 3.11 environment is unavailable: ${LIGHTNAV_PYTHON}" >&2
    exit 1
fi

cd "${REPOSITORY_ROOT}"
exec env \
    CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
    VLN_KV_CACHE_GIB="${VLN_KV_CACHE_GIB:-2}" \
    VLN_VLLM_ENFORCE_EAGER="${VLN_VLLM_ENFORCE_EAGER:-1}" \
    PYTHONUNBUFFERED=1 \
    "${LIGHTNAV_PYTHON}" \
    "${REPOSITORY_ROOT}/scripts/lightnav/infer_single_chunk.py" \
    "$@"
