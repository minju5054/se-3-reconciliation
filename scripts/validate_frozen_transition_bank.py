#!/usr/bin/env python3
"""Strictly validate a generated DATA-01 frozen transition bank."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.frozen_transition_bank import validate_bank  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bank_directory", type=Path)
    parser.add_argument(
        "--allow-missing-plots",
        action="store_true",
        help="validate the build phase before the separate immutable plotting step",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = validate_bank(
        args.bank_directory, require_plots=not args.allow_missing_plots
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
