#!/usr/bin/env python3
"""Build the immutable DATA-01 frozen LightNav transition bank (without plots)."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.frozen_transition_bank import build_bank  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/lightnav_transition_bank_v1.yaml",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="parent output override for isolated preflight/testing; version is still appended",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = build_bank(
        repository_root=REPOSITORY_ROOT,
        config_path=args.config,
        output_root_override=args.output_root,
    )
    print(f"DATA01_BANK_ROOT={root}")
    print("DATA01_NEXT=plot_frozen_transition_bank.py then validate_frozen_transition_bank.py")


if __name__ == "__main__":
    main()
