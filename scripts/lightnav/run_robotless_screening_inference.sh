#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/../.." && pwd)"
lightnav_client_python="${LIGHTNAV_CLIENT_PYTHON:-${repository_root}/../external/LightNav-0-official-demo/.venv/bin/python}"
if [[ ! -x "${lightnav_client_python}" ]]; then
  echo "Existing isolated client environment unavailable: ${lightnav_client_python}" >&2
  exit 1
fi
cd "${repository_root}"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONUNBUFFERED=1 \
  "${lightnav_client_python}" "${script_dir}/robotless_screening_inference.py" "$@"
