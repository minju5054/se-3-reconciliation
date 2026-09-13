#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
exec env MPLCONFIGDIR="${TMPDIR:-/tmp}/exp02c-presentation-mpl" \
  "$repo_root/.venv/bin/python" scripts/view_exp02c_direction_mechanism.py \
  --output data/exp02c_presentation/slide03-20260913-final --gui "$@"
