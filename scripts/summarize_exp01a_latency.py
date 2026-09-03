#!/usr/bin/env python3
"""Strictly validate and print one saved EXP-01A benchmark summary."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from reconciliation.latency_benchmark import validate_benchmark_output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark_directory", type=Path)
    args = parser.parse_args()
    result = validate_benchmark_output(args.benchmark_directory)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
