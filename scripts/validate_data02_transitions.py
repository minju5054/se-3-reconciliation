#!/usr/bin/env python3
"""Strictly validate a frozen DATA-02 primary collection and bank."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.data02_diverse_transitions import strict_validate_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    result = strict_validate_dataset(args.run_directory.resolve())
    print("DATA02_VALIDATION=" + json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
