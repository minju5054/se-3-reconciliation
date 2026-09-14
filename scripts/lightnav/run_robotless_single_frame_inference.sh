#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LIGHTNAV_CLIENT_PYTHON="${LIGHTNAV_CLIENT_PYTHON:-${REPOSITORY_ROOT}/../external/LightNav-0-official-demo/.venv/bin/python}"

if [[ ! -x "${LIGHTNAV_CLIENT_PYTHON}" ]]; then
    echo "Existing isolated client environment unavailable: ${LIGHTNAV_CLIENT_PYTHON}" >&2
    exit 1
fi

cd "${REPOSITORY_ROOT}"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONUNBUFFERED=1 \
    "${LIGHTNAV_CLIENT_PYTHON}" "${SCRIPT_DIR}/robotless_single_frame_inference.py" "$@"
