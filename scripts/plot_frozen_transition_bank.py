#!/usr/bin/env python3
"""Create immutable per-pair and overview plots for one DATA-01 bank."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.frozen_transition_bank import plot_bank  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bank_directory", type=Path)
    return parser.parse_args()


def main() -> None:
    result = plot_bank(parse_args().bank_directory)
    print(f"DATA01_PLOTS={len(result['plots_sha256'])}")
    print(f"DATA01_PAIR_COUNT={result['pair_count']}")


if __name__ == "__main__":
    main()
