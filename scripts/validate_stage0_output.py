#!/usr/bin/env python3
"""Validate a saved Stage 0 Jackal run without importing Isaac Sim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from reconciliation.stage0_jackal import validate_stage0_output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(validate_stage0_output(args.run_directory), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
