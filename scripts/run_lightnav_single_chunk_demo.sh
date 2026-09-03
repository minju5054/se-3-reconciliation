#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CAPTURE_LOG="$(mktemp)"
trap 'rm -f -- "${CAPTURE_LOG}"' EXIT

cd "${REPOSITORY_ROOT}"
echo "[Stage 0-C] 1/4 RGB capture: the first GUI is intentionally stationary."
./scripts/isaac/run_lightnav_single_chunk_capture.sh | tee "${CAPTURE_LOG}"
RUN_DIRECTORY="$(sed -n 's/^STAGE0C_RUN_DIRECTORY=//p' "${CAPTURE_LOG}" | tail -n 1)"
if [[ -z "${RUN_DIRECTORY}" || ! -d "${RUN_DIRECTORY}" ]]; then
    echo "Could not determine the Stage 0-C run directory from capture output." >&2
    exit 1
fi

echo "[Stage 0-C] 2/4 LightNav inference: the GUI stays closed to free GPU memory."
./scripts/lightnav/run_lightnav_single_chunk_inference.sh "${RUN_DIRECTORY}"

echo "[Stage 0-C] 3/4 Derivation and safety validation."
.venv/bin/python scripts/validate_lightnav_single_chunk.py "${RUN_DIRECTORY}"

echo "[Stage 0-C] 4/4 GUI playback: the Jackal starts moving after a visible countdown."
./scripts/isaac/run_lightnav_single_chunk_playback.sh "${RUN_DIRECTORY}"

.venv/bin/python scripts/validate_lightnav_single_chunk.py \
    "${RUN_DIRECTORY}" --require-execution
