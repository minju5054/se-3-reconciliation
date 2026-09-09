#!/usr/bin/env python3
"""Run the strict read-only DATA-02 validator."""

from __future__ import annotations

import subprocess
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: validate_data02_online_successive.py RUN_DIRECTORY")
    command = [
        sys.executable,
        str(ROOT / "scripts/summarize_data02_online_successive.py"),
        str(Path(sys.argv[1]).resolve()),
        "--validate-only",
    ]
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == "__main__":
    main()
