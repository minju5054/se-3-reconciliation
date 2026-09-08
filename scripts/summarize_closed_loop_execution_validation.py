#!/usr/bin/env python3
"""Validate a Stage 0-E run and generate the frozen comparison plots."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.closed_loop_execution_validation import (  # noqa: E402
    CONTROLLED_IDS,
    MODES,
    validate_closed_loop_trial,
)
from reconciliation.graph_metrics import assert_finite_json_tree  # noqa: E402
from reconciliation.online_switch import load_strict_json, sha256_file  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def trial_path(root: Path, subset: str, scenario: str, mode: str, repetition: int = 0) -> Path:
    return root / subset / scenario / mode / f"repetition_{repetition:02d}"


def load_telemetry(path: Path) -> dict[str, np.ndarray]:
    with (path / "raw/telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    numeric = [key for key in rows[0] if key not in {"phase", "goal_reached", "saturated", "sign_protection_event"}]
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=np.float64)
        for key in numeric
    }


def load_metrics(path: Path) -> dict[str, Any]:
    return load_strict_json(path / "derived/metrics.json")


def validate_run(root: Path) -> dict[str, Any]:
    required = (
        "metadata.json",
        "config_snapshot.yaml",
        "acceptance_criteria.json",
        "config_validation.json",
        "summary/controlled_paths_summary.json",
        "summary/composite_summary.json",
        "summary/exp02b_reference_summary.json",
        "summary/mode_comparison.json",
        "summary/final_execution_platform_decision.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"Stage 0-E run is missing: {', '.join(missing)}")
    metadata = load_strict_json(root / "metadata.json")
    if metadata.get("stage") != "stage0-e-closed-loop-trajectory-execution-validation":
        raise ValueError("invalid Stage 0-E metadata")
    if metadata.get("historical_body_commands_reused") is not False:
        raise ValueError("primary evaluation improperly reused historical commands")
    if metadata.get("held_out_results_used_for_controller_tuning") is not False:
        raise ValueError("held-out result tuning flag is invalid")
    if metadata.get("config_snapshot_sha256") != sha256_file(root / "config_snapshot.yaml"):
        raise ValueError("config snapshot hash changed")
    controlled = list(CONTROLLED_IDS)
    composite = ["stage0b_composite"]
    exp02b = ["case_high_delta_omega", "case_high_delta_v", "case_benign_delayed"]
    validated = []
    for subset, scenarios in (
        ("controlled_paths", controlled),
        ("stage0b_composite", composite),
        ("exp02b_old_reference", exp02b),
    ):
        for scenario in scenarios:
            reference = root / subset / scenario / "reference_trajectory.npy"
            if not reference.is_file():
                raise ValueError(f"missing reference trajectory: {reference}")
            for mode in MODES:
                for repetition in range(int(metadata["repetitions_per_condition_per_mode"])):
                    validated.append(
                        validate_closed_loop_trial(
                            trial_path(root, subset, scenario, mode, repetition)
                        )
                    )
    if len(validated) != int(metadata["trial_count"]):
        raise ValueError("validated trial count differs from metadata")
    summaries = {
        name: load_strict_json(root / "summary" / name)
        for name in (
            "controlled_paths_summary.json",
            "composite_summary.json",
            "exp02b_reference_summary.json",
            "mode_comparison.json",
            "final_execution_platform_decision.json",
        )
    }
    assert_finite_json_tree(summaries)
    return {
        "valid": True,
        "run_id": metadata["run_id"],
        "trial_count": len(validated),
        "decision": summaries["final_execution_platform_decision.json"]["status"],
        "summaries": summaries,
    }


def overlay(ax, root: Path, subset: str, scenario: str, title: str) -> None:
    reference = np.load(
        root / subset / scenario / "reference_trajectory.npy", allow_pickle=False
    )
    ax.plot(reference[:, 0], reference[:, 1], color="#1976d2", label="reference", linewidth=2.5)
    colors = {"nominal": "#f4511e", "calibrated": "#19a83e"}
    for mode in MODES:
        for repetition in range(3):
            actual = np.load(
                trial_path(root, subset, scenario, mode, repetition)
                / "raw/actual_trajectory.npy",
                allow_pickle=False,
            )
            ax.plot(
                actual[:, 0],
                actual[:, 1],
                color=colors[mode],
                alpha=0.35 if repetition else 0.95,
                linewidth=1.2,
                label=mode if repetition == 0 else None,
            )
    ax.set_title(title)
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()


def save_figure(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def generate_plots(root: Path, validation: dict[str, Any]) -> list[Path]:
    output = root / "plots"
    output.mkdir(parents=True, exist_ok=False)
    paths: list[Path] = []
    for index, scenario in enumerate(CONTROLLED_IDS, start=1):
        fig, ax = plt.subplots(figsize=(6.2, 5.0))
        overlay(ax, root, "controlled_paths", scenario, f"Controlled path: {scenario}")
        path = output / f"01_{index:02d}_controlled_{scenario}_xy.png"
        save_figure(fig, path)
        paths.append(path)

    yaw_values: list[list[float]] = []
    yaw_labels: list[str] = []
    for mode in MODES:
        values = []
        for scenario in CONTROLLED_IDS:
            for repetition in range(3):
                telemetry = load_telemetry(
                    trial_path(root, "controlled_paths", scenario, mode, repetition)
                )
                values.extend(np.abs(telemetry["nearest_reference_yaw_error_rad"]).tolist())
        yaw_values.append(values)
        yaw_labels.append(mode)
    fig, ax = plt.subplots(figsize=(6.2, 4.5))
    ax.boxplot(yaw_values, tick_labels=yaw_labels, showfliers=False)
    ax.set_ylabel("nearest-reference |yaw error| [rad]")
    ax.set_title("Controlled-path yaw error distributions")
    ax.grid(True, axis="y", alpha=0.25)
    path = output / "02_yaw_error_distributions.png"
    save_figure(fig, path)
    paths.append(path)

    controlled = validation["summaries"]["controlled_paths_summary.json"]
    x = np.arange(len(CONTROLLED_IDS))
    width = 0.36
    for metric, number, ylabel, title in (
        ("position_rmse_m", "03", "position RMSE [m]", "Position RMSE by controlled condition"),
        ("yaw_rmse_rad", "04", "yaw RMSE [rad]", "Yaw RMSE by controlled condition"),
    ):
        fig, ax = plt.subplots(figsize=(10.5, 4.8))
        for offset, mode in ((-width / 2, "nominal"), (width / 2, "calibrated")):
            means = [
                controlled[mode]["scenarios"][scenario]["repeatability"][metric]["mean"]
                for scenario in CONTROLLED_IDS
            ]
            stds = [
                controlled[mode]["scenarios"][scenario]["repeatability"][metric]["std"]
                for scenario in CONTROLLED_IDS
            ]
            ax.bar(x + offset, means, width, yerr=stds, capsize=3, label=mode)
        ax.set_xticks(x, CONTROLLED_IDS, rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend()
        path = output / f"{number}_{metric}_by_condition.png"
        save_figure(fig, path)
        paths.append(path)

    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    for offset, mode in ((-width / 2, "nominal"), (width / 2, "calibrated")):
        rates = [
            controlled[mode]["scenarios"][scenario]["goal_success_rate"]
            for scenario in CONTROLLED_IDS
        ]
        ax.bar(x + offset, rates, width, label=mode)
    ax.set_xticks(x, CONTROLLED_IDS, rotation=25, ha="right")
    ax.set_ylabel("goal success rate [fraction]")
    ax.set_ylim(0.0, 1.05)
    ax.set_title("Goal success by controlled condition")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    path = output / "05_goal_success_by_condition.png"
    save_figure(fig, path)
    paths.append(path)

    fig, axes = plt.subplots(2, 1, figsize=(9.0, 7.0), sharex=True)
    for ax, mode in zip(axes, MODES):
        telemetry = load_telemetry(
            trial_path(root, "exp02b_old_reference", "case_high_delta_omega", mode)
        )
        ax.plot(telemetry["sim_time_s"], telemetry["desired_omega_rps"], label="desired omega")
        ax.plot(telemetry["sim_time_s"], telemetry["executed_omega_rps"], label="executed omega")
        ax.plot(telemetry["sim_time_s"], telemetry["measured_omega_rps"], label="measured omega")
        ax.set_ylabel("omega [rad/s]")
        ax.set_title(f"EXP-02B high-delta-omega: {mode}")
        ax.grid(True, alpha=0.25)
        ax.legend()
    axes[-1].set_xlabel("simulation time [s]")
    path = output / "06_high_delta_omega_body_trace.png"
    save_figure(fig, path)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(6.2, 5.0))
    overlay(ax, root, "stage0b_composite", "stage0b_composite", "Stage 0-B composite")
    path = output / "07_stage0b_composite_xy.png"
    save_figure(fig, path)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(6.2, 5.0))
    overlay(
        ax,
        root,
        "exp02b_old_reference",
        "case_high_delta_omega",
        "EXP-02B high-delta-omega OLD reference",
    )
    path = output / "08_exp02b_high_delta_omega_xy.png"
    save_figure(fig, path)
    paths.append(path)

    cases = ("case_high_delta_omega", "case_high_delta_v", "case_benign_delayed")
    fig, axes = plt.subplots(1, 3, figsize=(16.0, 4.8))
    for ax, case in zip(axes, cases):
        overlay(ax, root, "exp02b_old_reference", case, case)
    path = output / "09_all_exp02b_old_reference_xy.png"
    save_figure(fig, path)
    paths.append(path)
    manifest = {
        "plot_count": len(paths),
        "plots": [
            {"path": str(path), "sha256": sha256_file(path)} for path in paths
        ],
        "units_and_legends_included": True,
    }
    with (output / "plot_manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return paths


def main() -> None:
    args = parse_args()
    root = args.run_directory.resolve()
    validation = validate_run(root)
    if not args.validate_only:
        plots = generate_plots(root, validation)
        validation["plot_count"] = len(plots)
    print("STAGE0E_VALIDATION=" + json.dumps(validation, sort_keys=True))


if __name__ == "__main__":
    main()
