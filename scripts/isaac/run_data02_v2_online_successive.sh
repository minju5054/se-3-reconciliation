#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

export DATA02_CONFIG="${REPOSITORY_ROOT}/configs/data02_online_successive_v2.yaml"
exec "${SCRIPT_DIR}/run_data02_online_successive.sh" "$@"
