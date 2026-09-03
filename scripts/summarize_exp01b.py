#!/usr/bin/env python3
"""Strictly validate and print a saved EXP-01B live experiment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from reconciliation.online_switch import load_strict_json, validate_experiment_output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_directory", type=Path)
    args = parser.parse_args()
    validation = validate_experiment_output(args.experiment_directory)
    summary = load_strict_json(args.experiment_directory / "summary.json")
    print(json.dumps({"validation": validation, "summary": summary}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
