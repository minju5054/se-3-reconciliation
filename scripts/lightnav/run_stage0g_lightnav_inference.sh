#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/../.." && pwd)"
lightnav_root="${LIGHTNAV_ROOT:-${repository_root}/../external/LightNav-0}"
lightnav_python="${lightnav_root}/.venv/bin/python"

if [[ ! -x "${lightnav_python}" ]]; then
  echo "LightNav Python environment unavailable: ${lightnav_python}" >&2
  exit 1
fi

cd "${repository_root}"
exec env \
  CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
  VLN_KV_CACHE_GIB="${VLN_KV_CACHE_GIB:-2}" \
  VLN_VLLM_ENFORCE_EAGER="${VLN_VLLM_ENFORCE_EAGER:-1}" \
  PYTHONUNBUFFERED=1 \
  "${lightnav_python}" \
  "${repository_root}/scripts/lightnav/stage0g_lightnav_inference.py" \
  "$@"
