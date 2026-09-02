#!/usr/bin/env python3
"""Validate Stage 0-B run directories and summarize comparable metrics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.controller_validation import validate_controller_run


DISPLAY_METRICS = (
    "position_rmse_m",
    "final_position_error_m",
    "yaw_rmse_rad",
    "final_yaw_error_rad",
    "wheel_velocity_tracking_rmse_rad_s",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "sessions",
        nargs="+",
        type=Path,
        help="one or more controller-validation session directories",
    )
    parser.add_argument("--output", type=Path, help="optional new JSON summary path")
    return parser.parse_args()


def discover_runs(session: Path) -> list[Path]:
    if not session.is_dir():
        raise ValueError(f"session directory does not exist: {session}")
    return sorted(
        child
        for child in session.iterdir()
        if child.is_dir() and (child / "metadata.json").is_file()
    )


def summarize(sessions: list[Path]) -> dict[str, object]:
    rows = []
    for session in sessions:
        for run in discover_runs(session):
            validation = validate_controller_run(run)
            metrics = validation["metrics"]
            rows.append(
                {
                    "session_id": session.name,
                    "run_directory": str(run.resolve()),
                    "controller": validation["controller"],
                    "scenario": validation["scenario"],
                    "reference_shape": validation["reference_shape"],
                    "actual_shape": validation["actual_shape"],
                    **{name: metrics[name] for name in DISPLAY_METRICS},
                    "valid": True,
                }
            )
    if not rows:
        raise ValueError("no Stage 0-B runs found")
    return {"schema": "stage0-b-controller-comparison-v1", "runs": rows}


def print_table(summary: dict[str, object]) -> None:
    rows = summary["runs"]
    headers = ("controller", "scenario", *DISPLAY_METRICS)
    widths = {
        header: max(
            len(header),
            *(len(f"{row[header]:.6g}") if isinstance(row[header], float) else len(str(row[header])) for row in rows),
        )
        for header in headers
    }
    print("  ".join(header.ljust(widths[header]) for header in headers))
    for row in rows:
        print(
            "  ".join(
                (f"{row[header]:.6g}" if isinstance(row[header], float) else str(row[header])).ljust(
                    widths[header]
                )
                for header in headers
            )
        )


def main() -> None:
    args = parse_args()
    summary = summarize(args.sessions)
    print_table(summary)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
