#!/usr/bin/env python3
"""Analyze one naive OLD-to-NEW trajectory switch from recorded chunk JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from reconciliation.io import load_chunk, save_json
from reconciliation.metrics import analyze_raw_switch
from reconciliation.se2 import local_trajectory_to_world


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute EXP-01 raw-switch boundary metrics")
    parser.add_argument("--old", required=True, type=Path, help="validated OLD chunk JSON")
    parser.add_argument("--new", required=True, type=Path, help="validated NEW chunk JSON")
    parser.add_argument("--output", type=Path, help="optional derived metrics JSON")
    parser.add_argument("--plot", type=Path, help="optional derived world-frame trajectory PNG")
    return parser.parse_args()


def save_plot(old, new, analysis, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    old_world = local_trajectory_to_world(old.robot_pose_at_observation, old.poses_local)
    new_world = local_trajectory_to_world(new.robot_pose_at_observation, new.poses_local)
    first = analysis.new_first_usable_index

    figure, axis = plt.subplots(figsize=(7, 6))
    axis.plot(old_world[:, 0], old_world[:, 1], "o-", label="OLD (world)")
    axis.plot(new_world[:, 0], new_world[:, 1], "o--", alpha=0.35, label="NEW stale/all")
    axis.plot(new_world[first:, 0], new_world[first:, 1], "o-", label="NEW usable suffix")
    boundary = analysis.old_boundary_pose_world
    new_first = analysis.new_first_pose_world
    axis.plot(
        [boundary[0], new_first[0]],
        [boundary[1], new_first[1]],
        "r--",
        linewidth=2,
        label="raw switch gap",
    )
    axis.set_title("EXP-01 raw OLD→NEW switch (synthetic if fixture inputs are used)")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    old = load_chunk(args.old)
    new = load_chunk(args.new)
    analysis = analyze_raw_switch(old, new)
    payload = analysis.to_dict()
    payload["inputs"] = {"old": str(args.old), "new": str(args.new)}
    if args.output:
        save_json(payload, args.output)
    if args.plot:
        save_plot(old, new, analysis, args.plot)
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
