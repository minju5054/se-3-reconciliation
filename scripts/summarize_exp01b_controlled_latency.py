#!/usr/bin/env python3
"""Summarize and strictly validate a primary redesigned EXP-01B cohort."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.exp01b_controlled_latency import (  # noqa: E402
    aggregate_controlled,
    load_attempt_records,
    select_representative_samples,
    validate_controlled_output,
)
from reconciliation.controller_switch_metrics import align_command_rows_to_raw_switch  # noqa: E402
from reconciliation.online_switch import load_strict_json, save_json_exclusive  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--augment-existing",
        action="store_true",
        help="add non-overwriting trajectory plots and supplemental contrasts to an existing summary",
    )
    return parser.parse_args()


def _write_csv(path: Path, rows: list[dict]) -> None:
    if path.exists():
        raise FileExistsError(path)
    if not rows:
        raise ValueError("cannot write an empty aggregate CSV")
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _flat(row: dict) -> dict:
    controller = row["controller_metrics"]
    geometry = row["geometry"]
    timing = row["timing"]
    return {
        "cell_id": row["cell_id"],
        "geometry_condition_id": row["condition_id"],
        "geometry_class": row["geometry_class"],
        "latency_condition_id": row["latency_condition_id"],
        "attempt_index": row["attempt_index"],
        "classification": row["classification"],
        "stop_output": row["stop_output"],
        "artifact_path": row["artifact_path"],
        "model_latency_wall_s": timing["model_latency_wall_s"],
        "configured_added_delay_s": timing["configured_added_delay_s"],
        "measured_added_delay_sim_s": timing["measured_added_delay_sim_s"],
        "effective_latency_wall_s": timing["effective_latency_wall_s"],
        "effective_latency_sim_s": timing["effective_latency_sim_s"],
        "real_time_factor": timing["real_time_factor"],
        "robot_translation_observation_to_switch_m": timing["robot_translation_observation_to_switch_m"],
        "robot_yaw_observation_to_switch_rad": timing["robot_yaw_observation_to_switch_rad"],
        "delta_v_signed_mps": controller.get("delta_v_signed_mps"),
        "delta_v_abs_mps": controller.get("delta_v_abs_mps"),
        "delta_omega_signed_rps": controller.get("delta_omega_signed_rps"),
        "delta_omega_abs_rps": controller.get("delta_omega_abs_rps"),
        "post_switch_max_abs_delta_v_mps": controller.get("post_switch_max_abs_delta_v_mps"),
        "post_switch_mean_abs_delta_v_mps": controller.get("post_switch_mean_abs_delta_v_mps"),
        "post_switch_max_abs_delta_omega_rps": controller.get("post_switch_max_abs_delta_omega_rps"),
        "post_switch_mean_abs_delta_omega_rps": controller.get("post_switch_mean_abs_delta_omega_rps"),
        "translation_pose_gap_m": geometry["translation_pose_gap_m"],
        "yaw_pose_gap_rad": geometry["yaw_pose_gap_rad"],
        "old_fresh_tangent_disagreement_abs_rad": geometry["old_fresh_tangent_disagreement_abs_rad"],
        "fresh_total_yaw_progression_rad": geometry["fresh_total_yaw_progression_rad"],
        "fresh_endpoint_lateral_from_incoming_tangent_m": geometry["fresh_endpoint_lateral_from_incoming_tangent_m"],
        "local_spatial_step_magnitude_mismatch_m": geometry["local_spatial_step_magnitude_mismatch_m"],
    }


def _box_strip(rows: list[dict], cells: list[str], key: str, ylabel: str, path: Path) -> None:
    fig, axis = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
    values = [[float(row[key]) for row in rows if row["cell_id"] == cell] for cell in cells]
    axis.boxplot(values, positions=np.arange(len(cells)), widths=0.55, showfliers=False)
    for index, samples in enumerate(values):
        offsets = np.linspace(-0.09, 0.09, len(samples)) if samples else []
        axis.scatter(np.full(len(samples), index) + offsets, samples, color="black", s=24, zorder=3)
    axis.set_xticks(np.arange(len(cells)), cells, rotation=18, ha="right")
    axis.set_ylabel(ylabel)
    axis.grid(alpha=0.25, axis="y")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _scatter(rows: list[dict], x: str, y: str, xlabel: str, ylabel: str, path: Path) -> None:
    fig, axis = plt.subplots(figsize=(7, 5.5), constrained_layout=True)
    classes = sorted({row["geometry_class"] for row in rows})
    markers = {"straight": "o", "turn": "^", "route_change": "s"}
    for geometry in classes:
        selected = [row for row in rows if row["geometry_class"] == geometry and row[x] not in (None, "")]
        axis.scatter(
            [float(row[x]) for row in selected],
            [float(row[y]) for row in selected],
            label=geometry,
            marker=markers.get(geometry, "o"),
            s=42,
            alpha=0.82,
        )
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(alpha=0.25)
    axis.legend()
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _command_trace(attempt: Path, path: Path) -> None:
    with (attempt / "derived/controller_commands.csv").open(newline="", encoding="utf-8") as stream:
        rows = align_command_rows_to_raw_switch(list(csv.DictReader(stream)))
    selected = [row for row in rows if abs(float(row["time_from_raw_switch_s"])) <= 0.35]
    time_values = [float(row["time_from_raw_switch_s"]) for row in selected]
    fig, axes = plt.subplots(2, 1, figsize=(7.5, 6.2), sharex=True, constrained_layout=True)
    for axis, key, label in (
        (axes[0], "v_command_mps", "v command [m/s]"),
        (axes[1], "omega_command_rps", "omega command [rad/s]"),
    ):
        for source, color in (("OLD", "tab:blue"), ("FRESH", "tab:orange")):
            mask = [row["reference_source"] == source for row in selected]
            axis.plot(
                [t for t, keep in zip(time_values, mask, strict=True) if keep],
                [float(row[key]) for row, keep in zip(selected, mask, strict=True) if keep],
                marker="o",
                label=source,
                color=color,
            )
        axis.axvline(0.0, color="black", linestyle="--", linewidth=1, label="raw switch")
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[0].legend()
    axes[1].set_xlabel("time relative to raw switch [s]")
    fig.suptitle(attempt.parent.parent.name + " / " + attempt.name)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _transition_plot(attempt: Path, path: Path) -> None:
    old = np.load(attempt / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(attempt / "derived/fresh_world.npy", allow_pickle=False)
    result = load_strict_json(attempt / "results/attempt.json")
    timing = result["timing"]
    with (attempt / "derived/timeline.csv").open(newline="", encoding="utf-8") as stream:
        timeline = list(csv.DictReader(stream))
    observation_time = float(timing["observation_sim_time_s"])
    switch_time = float(timing["fresh_usable_sim_time_s"])
    actual = np.asarray(
        [
            [float(row["actual_x"]), float(row["actual_y"])]
            for row in timeline
            if observation_time <= float(row["sim_time_s"]) <= switch_time
        ],
        dtype=float,
    )
    observation = np.asarray(result["robot_pose_at_new_observation"], dtype=float)
    boundary = np.asarray(result["robot_pose_at_new_ready"], dtype=float)

    fig, axis = plt.subplots(figsize=(7.5, 6.2), constrained_layout=True)
    axis.plot(old[:, 0], old[:, 1], "o-", label="OLD world", color="tab:blue", alpha=0.75)
    axis.plot(fresh[:, 0], fresh[:, 1], "o-", label="raw FRESH world", color="tab:orange")
    if actual.size:
        axis.plot(actual[:, 0], actual[:, 1], "-", label="actual observation->switch", color="black", linewidth=2)
    axis.scatter(*observation[:2], marker="s", s=70, label="FRESH observation")
    axis.scatter(*boundary[:2], marker="X", s=90, label="raw switch B")
    axis.scatter(*fresh[0, :2], marker="*", s=120, label="FRESH entry F0")
    scale = 0.15
    for pose, color in ((boundary, "black"), (fresh[0], "tab:orange")):
        axis.arrow(
            pose[0], pose[1], scale * np.cos(pose[2]), scale * np.sin(pose[2]),
            color=color, head_width=0.035, length_includes_head=True,
        )
    metrics = result["controller_metrics"]
    axis.set_title(
        f"{result['cell_id']} attempt {result['attempt_index']}\n"
        f"|Δv|={metrics['delta_v_abs_mps']:.3f} m/s, "
        f"|Δω|={metrics['delta_omega_abs_rps']:.3f} rad/s"
    )
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def _correlation(rows: list[dict], x: str, y: str) -> dict:
    pairs = [(float(row[x]), float(row[y])) for row in rows if row[x] not in (None, "") and row[y] not in (None, "")]
    if len(pairs) < 3 or np.std([p[0] for p in pairs]) <= 1e-12 or np.std([p[1] for p in pairs]) <= 1e-12:
        return {"count": len(pairs), "pearson_r": None}
    return {"count": len(pairs), "pearson_r": float(np.corrcoef(np.asarray(pairs).T)[0, 1])}


def _group_statistics(rows: list[dict], group_key: str, metric: str) -> dict:
    from reconciliation.exp01b_extension import descriptive_statistics

    return {
        group: descriptive_statistics([float(row[metric]) for row in rows if row[group_key] == group])
        for group in sorted({str(row[group_key]) for row in rows})
    }


def _supplemental_analysis(flat_moving: list[dict]) -> dict:
    metrics = (
        "effective_latency_sim_s",
        "robot_translation_observation_to_switch_m",
        "delta_v_abs_mps",
        "delta_omega_abs_rps",
        "translation_pose_gap_m",
        "yaw_pose_gap_rad",
        "old_fresh_tangent_disagreement_abs_rad",
        "fresh_total_yaw_progression_rad",
        "fresh_endpoint_lateral_from_incoming_tangent_m",
    )
    by_geometry: dict[str, dict] = {}
    for geometry in sorted({row["geometry_condition_id"] for row in flat_moving}):
        selected = [row for row in flat_moving if row["geometry_condition_id"] == geometry]
        natural = [row for row in selected if row["latency_condition_id"] == "L0_natural"]
        delayed = [row for row in selected if row["latency_condition_id"] == "L1_added_050"]
        comparisons = {}
        for metric in metrics:
            n = np.asarray([float(row[metric]) for row in natural if row[metric] not in (None, "")])
            d = np.asarray([float(row[metric]) for row in delayed if row[metric] not in (None, "")])
            comparisons[metric] = {
                "natural_mean": float(np.mean(n)) if n.size else None,
                "added_050_mean": float(np.mean(d)) if d.size else None,
                "mean_difference_added_minus_natural": float(np.mean(d) - np.mean(n)) if n.size and d.size else None,
                "natural_median": float(np.median(n)) if n.size else None,
                "added_050_median": float(np.median(d)) if d.size else None,
                "median_difference_added_minus_natural": float(np.median(d) - np.median(n)) if n.size and d.size else None,
            }
        by_geometry[geometry] = comparisons
    return {
        "within_geometry_latency_contrasts": by_geometry,
        "geometry_descriptor_statistics": {
            metric: _group_statistics(flat_moving, "geometry_condition_id", metric)
            for metric in (
                "old_fresh_tangent_disagreement_abs_rad",
                "fresh_total_yaw_progression_rad",
                "fresh_endpoint_lateral_from_incoming_tangent_m",
            )
        },
        "note": "descriptive fixed-cell contrasts; no causal or population-level claim",
    }


def create_summary(root: Path) -> dict:
    protocol = load_strict_json(root / "protocol.json")
    cells = [cell["cell_id"] for cell in protocol["cells"]]
    records = load_attempt_records(root)
    aggregate = aggregate_controlled(records, cell_ids=cells)
    representatives = select_representative_samples(records, cells)
    flat_all = [_flat(row) for row in records]
    flat_moving = [row for row in flat_all if row["classification"] == "VALID_MOVING"]
    _write_csv(root / "all_attempts.csv", flat_all)
    _write_csv(root / "valid_moving_transitions.csv", flat_moving)

    plots = root / "plots"
    plots.mkdir(exist_ok=False)
    _box_strip(flat_moving, cells, "robot_translation_observation_to_switch_m", "robot travel observation->switch [m]", plots / "robot_travel_by_latency_geometry.png")
    _box_strip(flat_moving, cells, "delta_v_abs_mps", "absolute delta v [m/s]", plots / "delta_v_by_cell.png")
    _box_strip(flat_moving, cells, "delta_omega_abs_rps", "absolute delta omega [rad/s]", plots / "delta_omega_by_cell.png")
    _scatter(flat_moving, "effective_latency_sim_s", "delta_v_abs_mps", "effective latency [sim s]", "absolute delta v [m/s]", plots / "effective_latency_vs_delta_v.png")
    _scatter(flat_moving, "effective_latency_sim_s", "delta_omega_abs_rps", "effective latency [sim s]", "absolute delta omega [rad/s]", plots / "effective_latency_vs_delta_omega.png")
    _scatter(flat_moving, "old_fresh_tangent_disagreement_abs_rad", "delta_omega_abs_rps", "OLD/FRESH tangent disagreement [rad]", "absolute delta omega [rad/s]", plots / "tangent_disagreement_vs_delta_omega.png")
    trace_paths = {}
    transition_paths = {}
    for cell, artifact in representatives.items():
        if artifact is None:
            trace_paths[cell] = None
            transition_paths[cell] = None
            continue
        path = plots / f"representative_controller_trace_{cell}.png"
        _command_trace(Path(artifact), path)
        trace_paths[cell] = str(path)
        transition_path = plots / f"representative_transition_{cell}.png"
        _transition_plot(Path(artifact), transition_path)
        transition_paths[cell] = str(transition_path)

    summary = {
        "experiment": "EXP-01B Redesigned Controlled Latency",
        "aggregate": aggregate,
        "representative_samples": representatives,
        "representative_controller_trace_plots": trace_paths,
        "representative_transition_plots": transition_paths,
        "controlled_contrasts": _supplemental_analysis(flat_moving),
        "exploratory_associations": {
            "effective_latency_vs_robot_travel": _correlation(flat_moving, "effective_latency_sim_s", "robot_translation_observation_to_switch_m"),
            "effective_latency_vs_delta_v": _correlation(flat_moving, "effective_latency_sim_s", "delta_v_abs_mps"),
            "effective_latency_vs_delta_omega": _correlation(flat_moving, "effective_latency_sim_s", "delta_omega_abs_rps"),
            "heading_disagreement_vs_delta_omega": _correlation(flat_moving, "old_fresh_tangent_disagreement_abs_rad", "delta_omega_abs_rps"),
            "translation_pose_gap_vs_delta_v": _correlation(flat_moving, "translation_pose_gap_m", "delta_v_abs_mps"),
            "interpretation": "descriptive associations only; no causal claim",
        },
        "secondary_metric_warning": (
            "local_spatial_step_magnitude_mismatch_m compares an actual control-interval "
            "displacement with untimed LightNav waypoint spacing; it is not velocity or acceleration"
        ),
    }
    save_json_exclusive(root / "summary.json", summary)
    validation = validate_controlled_output(root)
    save_json_exclusive(root / "validation.json", validation)
    return {"summary": summary, "validation": validation}


def augment_existing(root: Path) -> dict:
    """Add review plots/contrasts without overwriting the completed primary summary."""

    protocol = load_strict_json(root / "protocol.json")
    cells = [cell["cell_id"] for cell in protocol["cells"]]
    records = load_attempt_records(root)
    representatives = select_representative_samples(records, cells)
    flat_moving = [_flat(row) for row in records if row["classification"] == "VALID_MOVING"]
    paths: dict[str, str | None] = {}
    for cell, artifact in representatives.items():
        if artifact is None:
            paths[cell] = None
            continue
        destination = root / "plots" / f"representative_transition_{cell}.png"
        _transition_plot(Path(artifact), destination)
        paths[cell] = str(destination)
    payload = {
        "representative_transition_plots": paths,
        "controlled_contrasts": _supplemental_analysis(flat_moving),
        "source_summary": "summary.json",
    }
    save_json_exclusive(root / "supplemental_analysis.json", payload)
    return payload


def main() -> None:
    args = parse_args()
    if args.validate_only:
        print(json.dumps(validate_controlled_output(args.experiment_directory), indent=2, sort_keys=True))
    elif args.augment_existing:
        print(json.dumps(augment_existing(args.experiment_directory), indent=2, sort_keys=True))
    else:
        print(json.dumps(create_summary(args.experiment_directory), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
