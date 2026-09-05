#!/usr/bin/env python3
"""Aggregate, plot, and strictly validate an EXP-01B Extension cohort."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from reconciliation.exp01b_extension import (
    aggregate_extension,
    load_attempt_records,
    select_representative_samples,
    validate_extension_output,
    write_csv_exclusive,
)
from reconciliation.online_switch import load_strict_json, save_json_exclusive


def _valid(records):
    return [row for row in records if row["timing_valid"]]


def _condition_metric_plot(records, condition_ids, metrics, labels, destination, title):
    figure, axes = plt.subplots(1, len(metrics), figsize=(6.2 * len(metrics), 4.8))
    axes = np.atleast_1d(axes)
    for axis, metric, ylabel in zip(axes, metrics, labels, strict=True):
        groups = []
        for position, condition_id in enumerate(condition_ids, start=1):
            rows = [row for row in _valid(records) if row["condition_id"] == condition_id]
            values = [float(row["metrics"][metric]) for row in rows]
            groups.append(values)
            jitter = [position + 0.025 * ((int(row["attempt_index"]) % 5) - 2) for row in rows]
            colors = ["#d95f02" if row["stop_output"] else "#1b9e77" for row in rows]
            axis.scatter(jitter, values, c=colors, s=38, zorder=3, edgecolor="black", linewidth=0.3)
        nonempty = [(index + 1, values) for index, values in enumerate(groups) if values]
        if nonempty:
            axis.boxplot(
                [values for _, values in nonempty],
                positions=[position for position, _ in nonempty],
                widths=0.45,
                showfliers=False,
            )
        axis.axhline(0.05, color="#666666", linestyle="--", linewidth=1, label="0.05 threshold")
        axis.set_xticks(range(1, len(condition_ids) + 1), [value.replace("condition_", "") for value in condition_ids], rotation=18)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.22)
    axes[0].legend(loc="best", fontsize=8)
    figure.suptitle(title + " (green=moving FRESH, orange=STOP)")
    figure.tight_layout()
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def _scatter_plots(records, destination, *, heading=False):
    rows = _valid(records)
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 4.8))
    condition_ids = sorted({row["condition_id"] for row in rows})
    colors = dict(zip(condition_ids, plt.cm.tab10.colors, strict=False))
    for row in rows:
        if heading:
            x_value = row["geometry"]["old_fresh_initial_heading_disagreement_abs_rad"]
            if x_value is None:
                continue
        else:
            x_value = row["observation_to_ready_translation_m"]
        for axis, metric in zip(
            axes, ("translational_motion_jump_m", "yaw_motion_jump_rad"), strict=True
        ):
            axis.scatter(
                x_value,
                row["metrics"][metric],
                color=colors[row["condition_id"]],
                marker="x" if row["stop_output"] else "o",
                s=42,
                label=row["condition_id"],
            )
            axis.set_ylabel(metric)
            axis.grid(alpha=0.22)
    xlabel = (
        "|OLD/FRESH initial heading disagreement| [rad]"
        if heading
        else "robot translation during inference [m]"
    )
    for axis in axes:
        axis.set_xlabel(xlabel)
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles, strict=False))
    figure.legend(unique.values(), unique.keys(), loc="upper center", ncol=2, fontsize=8)
    figure.suptitle("Descriptive relationship only; no causal inference")
    figure.tight_layout(rect=(0, 0, 1, 0.88))
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def _representative_plot(root: Path, artifact: str, condition_id: str, destination: Path):
    attempt = Path(artifact)
    if not attempt.is_absolute():
        attempt = root.parent.parent / attempt if not attempt.exists() else attempt
    old = np.load(attempt / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(attempt / "derived/new_world.npy", allow_pickle=False)
    metrics = load_strict_json(attempt / "results/switch_metrics.json")
    observation = np.asarray(metrics["robot_pose_at_new_observation"], dtype=float)
    ready = np.asarray(metrics["robot_pose_at_new_ready"], dtype=float)
    before = np.asarray(metrics["actual_pose_before_ready"], dtype=float)
    observation_time = float(metrics["timing"]["observation_sim_time_s"])
    ready_time = float(metrics["timing"]["ready_sim_time_s"])
    with (attempt / "derived/timeline.csv").open(newline="", encoding="utf-8") as stream:
        timeline = list(csv.DictReader(stream))
    actual_inference = np.asarray(
        [
            [float(row["actual_x"]), float(row["actual_y"])]
            for row in timeline
            if observation_time - 1e-9 <= float(row["sim_time_s"]) <= ready_time + 1e-9
        ],
        dtype=float,
    )
    figure, axis = plt.subplots(figsize=(7.0, 5.2))
    axis.plot(old[:, 0], old[:, 1], "o-", color="#377eb8", label="planned OLD world")
    axis.plot(fresh[:, 0], fresh[:, 1], "o-", color="#e41a1c", label="raw FRESH world")
    if actual_inference.size:
        axis.plot(
            actual_inference[:, 0],
            actual_inference[:, 1],
            "-",
            color="#4daf4a",
            linewidth=3,
            label="actual observation-to-ready motion",
        )
    axis.plot(
        [before[0], ready[0]],
        [before[1], ready[1]],
        "s-",
        color="#4daf4a",
        linewidth=1,
        label="actual final control interval",
    )
    axis.scatter(*observation[:2], marker="^", s=100, color="#984ea3", label="FRESH observation pose")
    axis.scatter(*ready[:2], marker="*", s=160, color="#ff7f00", label="FRESH ready pose")
    axis.set_title(f"Deterministic representative: {condition_id}\n{attempt.name}")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(destination, dpi=180)
    plt.close(figure)


def create_outputs(root: Path) -> dict:
    protocol = load_strict_json(root / "protocol.json")
    condition_ids = [item["id"] for item in protocol["conditions"]]
    records = load_attempt_records(root, condition_ids)
    aggregate = aggregate_extension(records, condition_ids=condition_ids)
    representatives = select_representative_samples(records, condition_ids)
    write_csv_exclusive(root / "all_attempts.csv", records)
    write_csv_exclusive(
        root / "valid_transitions.csv", [row for row in records if row["timing_valid"]]
    )
    plots = root / "plots"
    plots.mkdir(exist_ok=False)
    _condition_metric_plot(
        records,
        condition_ids,
        ("translation_gap_m", "translational_motion_jump_m"),
        ("translation gap [m]", "translation-motion jump [m]"),
        plots / "translation_metrics_by_condition.png",
        "EXP-01B Extension translation metrics",
    )
    _condition_metric_plot(
        records,
        condition_ids,
        ("yaw_gap_rad", "yaw_motion_jump_rad"),
        ("yaw gap [rad]", "yaw-motion jump [rad]"),
        plots / "yaw_metrics_by_condition.png",
        "EXP-01B Extension yaw metrics",
    )
    _scatter_plots(records, plots / "inference_motion_vs_discontinuity.png")
    _scatter_plots(records, plots / "heading_disagreement_vs_jump.png", heading=True)
    representative_plots = {}
    for condition_id, artifact in representatives.items():
        if artifact is None:
            representative_plots[condition_id] = None
            continue
        path = plots / f"representative_inference_motion_{condition_id}.png"
        _representative_plot(root, artifact, condition_id, path)
        representative_plots[condition_id] = str(path)
    summary = {
        "schema_version": 1,
        "experiment": "EXP-01B Extension",
        "aggregate": aggregate,
        "representative_samples": representatives,
        "representative_plots": representative_plots,
        "stop_handling": (
            "STOP is preserved as a valid model output and reported both included and excluded."
        ),
        "geometry_interpretation": "Exploratory/descriptive only; no causal claim.",
    }
    save_json_exclusive(root / "summary.json", summary)
    validation = validate_extension_output(root)
    save_json_exclusive(root / "validation.json", validation)
    return {"summary": summary, "validation": validation}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    result = (
        {"validation": validate_extension_output(args.experiment_directory)}
        if args.validate_only
        else create_outputs(args.experiment_directory)
    )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
