#!/usr/bin/env python3
"""Summarize qualification or freeze a validated DATA-02 primary bank."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.data02_diverse_transitions import (
    GEOMETRY_LABELS,
    SCENARIO_IDS,
    canonicalize_at_boundary,
    dataset_acceptance,
    distribution,
    grouped_split,
    load_attempt_records,
    scenario_qualification,
    strict_validate_dataset,
    validate_attempt_directory,
    validate_data02_config,
    write_index_csv,
    write_json_exclusive,
)
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--qualification-run", type=Path)
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("config snapshot must contain a mapping")
    validate_data02_config(value)
    return value


def spatial_length(path: np.ndarray) -> np.ndarray:
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))))


def actual_observation_to_boundary(attempt: Path, context: Mapping[str, Any]) -> np.ndarray:
    actual = np.load(attempt / "actual/old_execution.npy", allow_pickle=False)
    with (attempt / "telemetry/old_execution.csv").open("r", encoding="utf-8", newline="") as stream:
        times = np.asarray([float(row["sim_time_s"]) for row in csv.DictReader(stream)])
    relative_fresh = float(context["timing"]["fresh_observation_sim_time_s"]) - float(
        context["timing"]["old_observation_sim_time_s"]
    )
    index = int(np.searchsorted(times, relative_fresh - 1e-9, side="left"))
    return actual[index:].copy()


def plot_pair(attempt: Path, output: Path, record: Mapping[str, Any]) -> None:
    output.mkdir(parents=True, exist_ok=False)
    old = np.load(attempt / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(attempt / "derived/fresh_world.npy", allow_pickle=False)
    context = load_strict_json(attempt / "context.json")
    descriptors = load_strict_json(attempt / "descriptors.json")
    actual = actual_observation_to_boundary(attempt, context)
    p = np.asarray(context["previous_control_pose_world"])
    b = np.asarray(context["boundary_pose_world"])
    old_obs = np.asarray(context["old_observation_pose_world"])
    fresh_obs = np.asarray(context["fresh_observation_pose_world"])
    transition = descriptors["transition"]
    title = (
        f"{record['scenario_id']} / {record['variant_id']} / {record['transition_context_id']}\n"
        f"natural latency={transition['natural_inference_latency_s']:.3f}s, "
        f"|dv|={transition['abs_delta_v_mps']:.3f}m/s, "
        f"|domega|={transition['abs_delta_omega_rps']:.3f}rad/s"
    )

    figure, axis = plt.subplots(figsize=(7.2, 6.2))
    axis.plot(old[:, 0], old[:, 1], color="#1976d2", label="OLD world")
    axis.plot(fresh[:, 0], fresh[:, 1], color="#ef6c00", label="FRESH world")
    axis.plot(actual[:, 0], actual[:, 1], color="black", linewidth=2, label="actual fresh-observation to B")
    axis.scatter([b[0]], [b[1]], color="red", marker="*", s=130, label="B")
    axis.scatter([p[0]], [p[1]], color="black", marker="o", s=42, label="P")
    axis.scatter([old_obs[0]], [old_obs[1]], facecolors="none", edgecolors="#1976d2", marker="s", s=65, label="OLD observation")
    axis.scatter([fresh_obs[0]], [fresh_obs[1]], facecolors="none", edgecolors="#ef6c00", marker="D", s=60, label="FRESH observation")
    axis.set_title(title)
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "world_xy.png", dpi=150)
    plt.close(figure)

    old_c = canonicalize_at_boundary(old, b)
    fresh_c = canonicalize_at_boundary(fresh, b)
    actual_c = canonicalize_at_boundary(actual, b)
    p_c = canonicalize_at_boundary(p, b)[0]
    old_obs_c = canonicalize_at_boundary(old_obs, b)[0]
    fresh_obs_c = canonicalize_at_boundary(fresh_obs, b)[0]
    figure, axis = plt.subplots(figsize=(7.2, 6.2))
    axis.plot(old_c[:, 0], old_c[:, 1], color="#1976d2", label="OLD")
    axis.plot(fresh_c[:, 0], fresh_c[:, 1], color="#ef6c00", label="FRESH")
    axis.plot(actual_c[:, 0], actual_c[:, 1], color="black", linewidth=2, label="actual obs->B")
    axis.scatter([0.0], [0.0], color="red", marker="*", s=130, label="B=[0,0,0]")
    axis.scatter([p_c[0]], [p_c[1]], color="black", s=42, label="P")
    axis.scatter([old_obs_c[0]], [old_obs_c[1]], facecolors="none", edgecolors="#1976d2", marker="s", s=65, label="OLD obs")
    axis.scatter([fresh_obs_c[0]], [fresh_obs_c[1]], facecolors="none", edgecolors="#ef6c00", marker="D", s=60, label="FRESH obs")
    axis.set_title(title + "\nVISUALIZATION-ONLY B-CENTERED FRAME")
    axis.set_xlabel("B-centered x [m]")
    axis.set_ylabel("B-centered y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "boundary_canonical_xy.png", dpi=150)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.2, 4.8))
    axis.plot(spatial_length(old), np.unwrap(old[:, 2]), color="#1976d2", label="OLD yaw")
    axis.plot(spatial_length(fresh), np.unwrap(fresh[:, 2]), color="#ef6c00", label="FRESH yaw")
    axis.set_title(f"{record['scenario_id']} / {record['variant_id']} — waypoint rows are untimed")
    axis.set_xlabel("cumulative spatial path length [m] (NOT TIME)")
    axis.set_ylabel("display-unwrapped yaw [rad]")
    axis.grid(alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / "yaw_vs_path_length.png", dpi=150)
    plt.close(figure)


def plot_canonical_axis(axis, attempt: Path, record: Mapping[str, Any], title: str | None = None) -> None:
    old = np.load(attempt / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(attempt / "derived/fresh_world.npy", allow_pickle=False)
    context = load_strict_json(attempt / "context.json")
    b = np.asarray(context["boundary_pose_world"])
    old_c = canonicalize_at_boundary(old, b)
    fresh_c = canonicalize_at_boundary(fresh, b)
    axis.plot(old_c[:, 0], old_c[:, 1], color="#1976d2", linewidth=1.4)
    axis.plot(fresh_c[:, 0], fresh_c[:, 1], color="#ef6c00", linewidth=1.4)
    axis.scatter([0.0], [0.0], color="red", marker="*", s=35)
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.2)
    axis.set_title(title or f"{record['scenario_id']}\n{record['variant_id']}", fontsize=8)


def plot_overviews(root: Path, records: Sequence[Mapping[str, Any]], attempts: Mapping[str, Path]) -> list[str]:
    overview = root / "overview"
    overview.mkdir(parents=True, exist_ok=False)
    count = len(records)
    columns = 5
    rows = max(1, math.ceil(count / columns))
    figure, axes = plt.subplots(rows, columns, figsize=(3.1 * columns, 2.8 * rows), squeeze=False)
    for axis, record in zip(axes.flat, records, strict=False):
        plot_canonical_axis(axis, attempts[record["transition_context_id"]], record)
    for axis in list(axes.flat)[count:]:
        axis.axis("off")
    figure.suptitle("DATA-02 all valid primary pairs — visualization-only B-centered frames")
    figure.tight_layout()
    figure.savefig(overview / "all_pairs_boundary_canonical.png", dpi=120)
    plt.close(figure)

    by_scenario = defaultdict(list)
    for record in records:
        by_scenario[record["scenario_id"]].append(record)
    max_per = max((len(values) for values in by_scenario.values()), default=1)
    figure, axes = plt.subplots(10, max_per, figsize=(3.0 * max_per, 2.55 * 10), squeeze=False)
    for row, scenario in enumerate(SCENARIO_IDS):
        values = sorted(by_scenario[scenario], key=lambda item: (item["variant_id"], item["transition_context_id"]))
        for column in range(max_per):
            axis = axes[row, column]
            if column < len(values):
                record = values[column]
                plot_canonical_axis(axis, attempts[record["transition_context_id"]], record, f"{scenario} / {record['variant_id']}")
            else:
                axis.axis("off")
    figure.suptitle("DATA-02 by scenario family — each primary context is a separate subplot")
    figure.tight_layout()
    figure.savefig(overview / "by_scenario_family.png", dpi=110)
    plt.close(figure)

    representatives = []
    for scenario in SCENARIO_IDS:
        values = sorted(by_scenario[scenario], key=lambda item: item["transition_context_id"])
        if values:
            representatives.append(values[0])
    figure, axes = plt.subplots(2, 5, figsize=(15.0, 6.0), squeeze=False)
    for axis, record in zip(axes.flat, representatives, strict=False):
        plot_canonical_axis(axis, attempts[record["transition_context_id"]], record, record["scenario_id"])
    for axis in list(axes.flat)[len(representatives):]:
        axis.axis("off")
    figure.suptitle("Deterministic one-per-family representatives (minimum context ID; not quality-selected)")
    figure.tight_layout()
    figure.savefig(overview / "scenario_family_representatives.png", dpi=150)
    plt.close(figure)

    by_label = defaultdict(list)
    for record in records:
        by_label[record["geometry_label"]].append(record)
    figure, axes = plt.subplots(1, len(GEOMETRY_LABELS), figsize=(16, 3.4), squeeze=False)
    for axis, label in zip(axes.flat, GEOMETRY_LABELS, strict=True):
        for record in sorted(by_label[label], key=lambda item: item["transition_context_id"]):
            old = np.load(attempts[record["transition_context_id"]] / "derived/old_world.npy", allow_pickle=False)
            fresh = np.load(attempts[record["transition_context_id"]] / "derived/fresh_world.npy", allow_pickle=False)
            context = load_strict_json(attempts[record["transition_context_id"]] / "context.json")
            b = np.asarray(context["boundary_pose_world"])
            old_c = canonicalize_at_boundary(old, b)
            fresh_c = canonicalize_at_boundary(fresh, b)
            axis.plot(old_c[:, 0], old_c[:, 1], color="#1976d2", alpha=0.35)
            axis.plot(fresh_c[:, 0], fresh_c[:, 1], color="#ef6c00", alpha=0.55)
        axis.set_title(f"{label}\nn={len(by_label[label])}", fontsize=9)
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(overview / "by_geometry_class.png", dpi=150)
    plt.close(figure)

    dv = [record["transition"]["abs_delta_v_mps"] for record in records]
    dw = [record["transition"]["abs_delta_omega_rps"] for record in records]
    figure, axes = plt.subplots(1, 2, figsize=(9.5, 4.0))
    axes[0].hist(dv, bins=min(12, max(3, len(dv))))
    axes[0].set_xlabel("raw |delta v| [m/s]")
    axes[1].hist(dw, bins=min(12, max(3, len(dw))))
    axes[1].set_xlabel("raw |delta omega| [rad/s]")
    for axis in axes:
        axis.set_ylabel("context count")
        axis.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(overview / "raw_delta_distribution.png", dpi=150)
    plt.close(figure)

    labels = list(GEOMETRY_LABELS)
    values = [len(by_label[label]) for label in labels]
    figure, axis = plt.subplots(figsize=(10.5, 4.5))
    axis.bar(np.arange(len(labels)), values)
    axis.set_xticks(np.arange(len(labels)), labels, rotation=20, ha="right")
    axis.set_ylabel("valid primary context count")
    axis.grid(axis="y", alpha=0.2)
    figure.tight_layout()
    figure.savefig(overview / "geometry_diversity.png", dpi=150)
    plt.close(figure)
    return [str(path.relative_to(root)) for path in sorted(overview.glob("*.png"))]


def qualification_summary(root: Path, config: Mapping[str, Any], records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    scenarios = {item["id"]: item for item in validate_data02_config(config)}
    grouped = defaultdict(list)
    for record in records:
        grouped[record["scenario_id"]].append(record)
    per_scenario = {
        scenario: scenario_qualification(grouped[scenario], scenarios[scenario], config)
        for scenario in SCENARIO_IDS
    }
    passed = all(value["passed"] for value in per_scenario.values())
    plotted = 0
    for record in records:
        if record.get("transition_context_id"):
            attempt = root / record["attempt_relative_path"]
            plot_pair(attempt, attempt / "plots", record)
            plotted += 1
    summary = {
        "status": "DATA02_QUALIFICATION_PASSED" if passed else "DATA02_QUALIFICATION_FAILED",
        "all_scenarios_passed": passed,
        "scenario_results": per_scenario,
        "thresholds": dict(config["qualification_thresholds"]),
        "primary_config_frozen": bool(config["protocol"]["primary_config_frozen"]),
        "reconciliation_result_used": False,
        "pair_plot_count": plotted * 3,
    }
    output = root / "summary"
    output.mkdir(parents=True, exist_ok=False)
    write_json_exclusive(output / "qualification_summary.json", summary)
    return summary


def freeze_primary(root: Path, config: Mapping[str, Any], records: Sequence[Mapping[str, Any]], qualification: Path) -> dict[str, Any]:
    qualification_summary_path = qualification / "summary/qualification_summary.json"
    q_summary = load_strict_json(qualification_summary_path)
    if q_summary["status"] != "DATA02_QUALIFICATION_PASSED":
        raise ValueError("primary bank cannot be frozen from failed qualification")
    if sha256_file(qualification / "config_snapshot.yaml") != sha256_file(root / "config_snapshot.yaml"):
        raise ValueError("qualification and primary config snapshots differ")
    valid = [dict(record) for record in records if record["classification"] == "VALID_MOVING"]
    bank = root / "bank"
    pairs = bank / "pairs"
    pairs.mkdir(parents=True, exist_ok=False)
    attempt_by_id = {}
    source_hashes = {}
    for record in valid:
        attempt = root / record["attempt_relative_path"]
        context_id = record["transition_context_id"]
        attempt_by_id[context_id] = attempt
        destination = pairs / context_id
        (destination / "raw").mkdir(parents=True, exist_ok=False)
        (destination / "derived").mkdir(parents=True, exist_ok=False)
        for relative in (
            "raw/old_lightnav.npy",
            "raw/fresh_lightnav.npy",
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "context.json",
            "descriptors.json",
            "provenance.json",
        ):
            source = attempt / relative
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            if sha256_file(source) != sha256_file(target):
                raise ValueError(f"bank copy hash differs: {relative}")
            source_hashes[f"{context_id}/{relative}"] = sha256_file(source)
        plot_pair(attempt, destination / "plots", record)
    split = grouped_split(
        valid,
        heldout_fraction=float(config["split"]["heldout_fraction"]),
        seed=str(config["split"]["deterministic_seed"]),
    )
    write_json_exclusive(bank / "split_manifest.json", split)
    write_index_csv(bank / "dataset_index.csv", valid)
    representatives = {
        scenario: min(
            (record["transition_context_id"] for record in valid if record["scenario_id"] == scenario),
            default=None,
        )
        for scenario in SCENARIO_IDS
    }
    write_json_exclusive(
        bank / "representatives.json",
        {
            "selection": "minimum transition_context_id per scenario; no reconciliation or quality metric",
            "scenario_representatives": representatives,
        },
    )
    geometry = defaultdict(list)
    for record in valid:
        geometry[record["geometry_label"]].append(record)
    diversity = {
        "valid_transition_context_count": len(valid),
        "unique_old_raw_count": len({record["old_raw_sha256"] for record in valid}),
        "unique_fresh_raw_count": len({record["fresh_raw_sha256"] for record in valid}),
        "unique_raw_pair_count": len({record["raw_pair_id"] for record in valid}),
        "unique_transition_context_count": len({record["transition_context_id"] for record in valid}),
        "geometry_classes": {
            label: {
                "count": len(geometry[label]),
                "unique_raw_pair_count": len({record["raw_pair_id"] for record in geometry[label]}),
                "scenario_families": sorted({record["scenario_id"] for record in geometry[label]}),
            }
            for label in GEOMETRY_LABELS
        },
        "raw_abs_delta_v_mps": distribution(record["transition"]["abs_delta_v_mps"] for record in valid),
        "raw_abs_delta_omega_rps": distribution(record["transition"]["abs_delta_omega_rps"] for record in valid),
        "natural_inference_latency_s": distribution(record["transition"]["natural_inference_latency_s"] for record in valid),
        "fresh_observation_to_boundary_translation_m": distribution(
            record["transition"]["fresh_observation_to_boundary_translation_m"] for record in valid
        ),
    }
    summary_dir = root / "summary"
    summary_dir.mkdir(parents=True, exist_ok=False)
    write_json_exclusive(summary_dir / "diversity_summary.json", diversity)
    acceptance = dataset_acceptance(valid, config)
    counts = Counter(record["classification"] for record in records)
    collection = {
        "status": acceptance["status"],
        "attempt_count": len(records),
        "classification_counts": dict(sorted(counts.items())),
        "scenario_valid_counts": acceptance["scenario_counts"],
        "acceptance": acceptance,
        "qualification_run": str(qualification.resolve()),
        "qualification_summary_sha256": sha256_file(qualification_summary_path),
        "config_snapshot_sha256": sha256_file(root / "config_snapshot.yaml"),
        "no_reconciliation_result_used": True,
    }
    write_json_exclusive(summary_dir / "collection_summary.json", collection)
    plots = plot_overviews(root, valid, attempt_by_id)
    manifest = {
        "dataset": config["dataset"],
        "status": acceptance["status"],
        "valid_transition_context_count": len(valid),
        "transition_context_ids": sorted(record["transition_context_id"] for record in valid),
        "source_artifact_sha256": dict(sorted(source_hashes.items())),
        "config_snapshot_sha256": sha256_file(root / "config_snapshot.yaml"),
        "qualification_summary_sha256": sha256_file(qualification_summary_path),
        "split_manifest_sha256": sha256_file(bank / "split_manifest.json"),
        "overview_plots": plots,
        "raw_lightnav_immutable": True,
        "intrinsic_waypoint_time_base": False,
        "no_optimizer_or_reconciliation_executed": True,
    }
    write_json_exclusive(bank / "dataset_manifest.json", manifest)
    return collection


def main() -> None:
    args = parse_args()
    root = args.run_directory.resolve()
    metadata = load_strict_json(root / "metadata.json")
    config = load_config(root / "config_snapshot.yaml")
    records = load_attempt_records(root)
    for record in records:
        validate_attempt_directory(root / record["attempt_relative_path"])
    if metadata["phase"] == "qualification":
        if args.qualification_run is not None:
            raise ValueError("qualification summarization does not accept --qualification-run")
        result = qualification_summary(root, config, records)
    elif metadata["phase"] == "primary":
        if args.qualification_run is None:
            raise ValueError("primary summarization requires --qualification-run")
        result = freeze_primary(root, config, records, args.qualification_run.resolve())
        strict_validate_dataset(root)
    else:
        result = {"status": "SMOKE_ONLY", "attempt_count": len(records)}
    print("DATA02_SUMMARY=" + json.dumps(result, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
