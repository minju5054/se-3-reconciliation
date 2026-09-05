#!/usr/bin/env python3
"""Strictly validate and print a saved EXP-02A run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from reconciliation.exp02a import validate_exp02a_output
from reconciliation.online_switch import load_strict_json


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    validation = validate_exp02a_output(args.run_directory)
    summary = load_strict_json(args.run_directory / "summary.json")
    print(json.dumps({"validation": validation, "summary": summary}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
