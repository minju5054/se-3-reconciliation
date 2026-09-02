#!/usr/bin/env python3
"""Build a validated, derived chunk JSON file from raw poses plus metadata."""

from __future__ import annotations

import argparse
from pathlib import Path

from reconciliation.io import build_chunk, load_json, save_chunk


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine an untouched N x 3 .npy/JSON pose recording with explicit metadata. "
            "The output is derived data; the input is never modified."
        )
    )
    parser.add_argument("--poses", required=True, type=Path, help="raw N x 3 .npy or JSON")
    parser.add_argument("--metadata", required=True, type=Path, help="chunk metadata JSON")
    parser.add_argument("--output", required=True, type=Path, help="derived chunk JSON")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    metadata = load_json(args.metadata)
    if not isinstance(metadata, dict):
        raise ValueError("metadata JSON must contain an object")
    chunk = build_chunk(args.poses, metadata)
    save_chunk(chunk, args.output)
    print(f"wrote {args.output} ({chunk.horizon} waypoints, source={chunk.source})")


if __name__ == "__main__":
    main()
