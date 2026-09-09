#!/usr/bin/env python3
"""Generate the ten frozen descriptive DATA-02 overview plots."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    return parser.parse_args()


def save(figure, path: Path) -> None:
    figure.savefig(path, dpi=160)
    plt.close(figure)


def bars(path: Path, title: str, xlabel: str, labels, values) -> None:
    figure, axis = plt.subplots(figsize=(9, 5), constrained_layout=True)
    axis.bar(range(len(labels)), values, color="#3977b7")
    axis.set_xticks(range(len(labels)), labels, rotation=25, ha="right")
    axis.set_ylabel("transition count")
    axis.set_xlabel(xlabel)
    axis.set_title(title)
    axis.grid(axis="y", alpha=.25)
    save(figure, path)


def hist(path: Path, title: str, xlabel: str, series: list[tuple[str, list[float]]]) -> None:
    figure, axis = plt.subplots(figsize=(8, 5), constrained_layout=True)
    for label, values in series:
        if values:
            axis.hist(values, bins=min(30, max(5, round(math.sqrt(len(values))))), alpha=.55, label=label)
    axis.set_xlabel(xlabel); axis.set_ylabel("transition count"); axis.set_title(title); axis.grid(alpha=.25)
    if len(series) > 1: axis.legend()
    save(figure, path)


def matrix(path: Path, title: str, rows: list[str], columns: list[str], values: np.ndarray) -> None:
    figure, axis = plt.subplots(figsize=(9, 6), constrained_layout=True)
    image = axis.imshow(values, cmap="Blues", aspect="auto")
    axis.set_xticks(range(len(columns)), columns, rotation=25, ha="right")
    axis.set_yticks(range(len(rows)), rows)
    for i in range(len(rows)):
        for j in range(len(columns)):
            axis.text(j, i, str(int(values[i, j])), ha="center", va="center", color="black")
    axis.set_xlabel("category"); axis.set_ylabel("template / split"); axis.set_title(title)
    figure.colorbar(image, ax=axis, label="transition count")
    save(figure, path)


def main() -> None:
    run = arguments().run_directory.resolve()
    summary_dir = run / "summary"
    records = json.loads((summary_dir / "transition_index.json").read_text())["records"]
    eligible = [row for row in records if row["status"] == "ELIGIBLE_MOVING"]
    plots = run / "plots"; plots.mkdir(exist_ok=False)
    counts = Counter(row["status"] for row in records)
    bars(plots / "eligible_transition_counts.png", "DATA-02 transition status distribution", "transition status", list(counts), list(counts.values()))
    hist(plots / "latency_distribution.png", "Online FRESH latency", "latency [s]", [("effective simulation", [r["tau_effective_s"] for r in eligible]), ("host request/response", [r["host_latency_s"] for r in eligible]), ("model reported", [r["model_reported_s"] for r in eligible])])
    figure, axis = plt.subplots(figsize=(7, 6), constrained_layout=True)
    axis.scatter([r["tau_effective_s"] for r in eligible], [r["robot_translation_during_inference_m"] for r in eligible], s=14, alpha=.7)
    axis.set_xlabel("effective observation-to-switch latency [s]"); axis.set_ylabel("robot translation during inference [m]"); axis.set_title("OLD execution motion while FRESH is inferred"); axis.grid(alpha=.25)
    save(figure, plots / "robot_motion_during_inference.png")
    hist(plots / "command_jump_distribution.png", "Frozen follower command discontinuity", "absolute command jump [m/s or rad/s]", [("|delta v| [m/s]", [abs(r["delta_v_mps"]) for r in eligible]), ("|delta omega| [rad/s]", [abs(r["delta_omega_rps"]) for r in eligible])])
    geometry = Counter(r["fresh_geometry_bin"] for r in eligible)
    bars(plots / "fresh_geometry_distribution.png", "FRESH spatial geometry coverage", "FRESH geometry bin (untimed)", list(geometry), list(geometry.values()))
    pair_counts = sorted(Counter(r["ordered_raw_pair_sha256"] for r in eligible).values(), reverse=True)
    figure, axis = plt.subplots(figsize=(8, 5), constrained_layout=True); axis.plot(range(1, len(pair_counts)+1), pair_counts, marker=".")
    axis.set_xlabel("ordered raw pair rank"); axis.set_ylabel("exact pair frequency [transitions]"); axis.set_title("Ordered raw OLD/FRESH pair frequency"); axis.grid(alpha=.25)
    save(figure, plots / "raw_pair_frequency.png")
    templates = sorted({r["template_id"] for r in eligible}); geometry_bins = ["STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING", "OTHER"]
    geom_values = np.array([[sum(r["template_id"] == t and r["fresh_geometry_bin"] == g for r in eligible) for g in geometry_bins] for t in templates])
    matrix(plots / "scenario_geometry_matrix.png", "Template by FRESH geometry", templates, geometry_bins, geom_values)
    difficulty_bins = ["BENIGN", "INTERMEDIATE", "CHALLENGING"]
    diff_values = np.array([[sum(r["template_id"] == t and r["difficulty_bin"] == d for r in eligible) for d in difficulty_bins] for t in templates])
    matrix(plots / "benign_challenging_matrix.png", "Template by command-jump difficulty", templates, difficulty_bins, diff_values)
    split = json.loads((summary_dir / "split.json").read_text()); dev = set(split["development_transition_ids"]); held = set(split["heldout_transition_ids"])
    split_values = np.array([[sum(r["transition_id"] in group and r["fresh_geometry_bin"] == g for r in eligible) for g in geometry_bins] for group in (dev, held)])
    matrix(plots / "development_heldout_summary.png", "Leakage-isolated development/held-out split", ["development", "held-out"], geometry_bins, split_values)
    representative_ids = json.loads((summary_dir / "representatives.json").read_text())["transition_ids"]
    lookup = {row["transition_id"]: row for row in eligible}
    selected = [lookup[item] for item in representative_ids]
    columns = 5; rows = max(1, math.ceil(len(selected) / columns))
    figure, axes = plt.subplots(rows, columns, figsize=(columns * 3.2, rows * 3.0), squeeze=False, constrained_layout=True)
    for axis in axes.flat: axis.set_visible(False)
    for axis, record in zip(axes.flat, selected):
        axis.set_visible(True)
        directory = run / record["artifact_relative_path"]
        old = np.load(directory / "derived/old_world.npy", allow_pickle=False)
        fresh = np.load(directory / "derived/fresh_world.npy", allow_pickle=False)
        actual = np.load(directory / "actual.npy", allow_pickle=False)
        context = json.loads((directory / "transition.json").read_text())
        axis.plot(old[:,0], old[:,1], color="#1764aa", label="OLD"); axis.plot(fresh[:,0], fresh[:,1], color="#d22f9b", label="FRESH"); axis.plot(actual[:,0], actual[:,1], color="#12a64a", label="actual")
        for key, color, marker in (("observation_pose_world_se2", "#d9ad00", "o"), ("pose_immediately_before_switch_P_world_se2", "#ee7411", "s"), ("switch_boundary_B_world_se2", "#c71919", "X")):
            pose = context[key]; axis.scatter(pose[0], pose[1], c=color, marker=marker, s=28)
        axis.set_aspect("equal", adjustable="datalim"); axis.set_title(record["transition_id"], fontsize=7); axis.tick_params(labelsize=6); axis.set_xlabel("world x [m]", fontsize=7); axis.set_ylabel("world y [m]", fontsize=7)
    if selected: axes.flat[0].legend(fontsize=6)
    figure.suptitle("Deterministic DATA-02 representative transitions (no optimized path)")
    save(figure, plots / "representative_transition_overview.png")
    print(f"DATA02_PLOTS={plots} count=10")


if __name__ == "__main__":
    main()
