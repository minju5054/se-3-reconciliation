#!/usr/bin/env python3
"""Summarize one completed Stage 0-D run and render frozen quantitative plots."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
from typing import Any, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.execution_calibration import (
    evaluate_composite_acceptance,
    evaluate_exp02b_replay_acceptance,
    final_validation_decision,
    write_json_exclusive,
)
from reconciliation.online_switch import load_strict_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    return parser.parse_args()


def paired_runs(rows: list[Mapping[str, Any]], key: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        result.setdefault(str(row[key]), {})[str(row["mode"])] = dict(row)
    if any(set(value) != {"nominal", "calibrated"} for value in result.values()):
        raise ValueError("paired summary is missing nominal or calibrated data")
    return result


def save_figure(path: Path) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_response(root: Path, held_out: Mapping[str, Any]) -> None:
    rows = held_out["paired_conditions"]
    plt.figure(figsize=(7, 5))
    bounds = [float(row["desired_omega_rps"]) for row in rows]
    extent = max(abs(value) for value in bounds) * 1.1
    plt.plot([-extent, extent], [-extent, extent], "k--", label="ideal")
    for mode, marker in (("nominal", "o"), ("calibrated", "s")):
        selected = [row for row in rows if row["mode"] == mode]
        plt.scatter(
            [row["desired_omega_rps"] for row in selected],
            [row["steady_measured_omega_mean_rps"] for row in selected],
            marker=marker,
            label=mode,
        )
    plt.xlabel("desired omega [rad/s]")
    plt.ylabel("steady measured omega [rad/s]")
    plt.title("Held-out desired versus measured yaw rate")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/01_desired_vs_measured_omega.png")

    plt.figure(figsize=(7, 5))
    for mode, marker in (("nominal", "o"), ("calibrated", "s")):
        selected = [row for row in rows if row["mode"] == mode]
        plt.scatter(
            [row["desired_omega_rps"] for row in selected],
            [
                row["desired_omega_rps"] - row["steady_measured_omega_mean_rps"]
                for row in selected
            ],
            marker=marker,
            label=mode,
        )
    plt.axhline(0.0, color="black", linestyle="--")
    plt.xlabel("desired omega [rad/s]")
    plt.ylabel("desired - measured omega [rad/s]")
    plt.title("Held-out steady-state yaw-rate error")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/02_omega_error_vs_desired.png")

    calibrated = [row for row in rows if row["mode"] == "calibrated"]
    figure, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].scatter(
        [row["desired_v_mps"] for row in calibrated],
        [row["omega_steady_state_gain"] for row in calibrated],
    )
    axes[0].set_xlabel("desired v [m/s]")
    axes[0].set_ylabel("omega gain [1]")
    axes[0].set_title("Gain versus linear speed")
    axes[1].scatter(
        [abs(row["desired_omega_rps"]) for row in calibrated],
        [row["omega_steady_state_gain"] for row in calibrated],
    )
    axes[1].set_xlabel("|desired omega| [rad/s]")
    axes[1].set_ylabel("omega gain [1]")
    axes[1].set_title("Gain versus yaw-rate magnitude")
    for axis in axes:
        axis.grid(alpha=0.3)
    figure.suptitle("Held-out calibrated yaw-response dependence")
    save_figure(root / "plots/03_omega_gain_dependence.png")

    plt.figure(figsize=(8, 5))
    for mode, marker in (("nominal", "o"), ("calibrated", "s")):
        selected = [row for row in rows if row["mode"] == mode]
        plt.scatter(
            [row["desired_omega_rps"] for row in selected],
            [row["wheel_rmse_rad_s"] for row in selected],
            marker=marker,
            label=mode,
        )
    plt.xlabel("desired omega [rad/s]")
    plt.ylabel("wheel target-measured RMSE [rad/s]")
    plt.title("Held-out wheel tracking error")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/04_wheel_tracking_error.png")

    paired = paired_runs(rows, "condition_id")
    identifiers = sorted(paired)
    x = np.arange(len(identifiers))
    width = 0.38
    plt.figure(figsize=(12, 5))
    plt.bar(
        x - width / 2,
        [paired[key]["nominal"]["omega_rmse_rps"] for key in identifiers],
        width,
        label="nominal",
    )
    plt.bar(
        x + width / 2,
        [paired[key]["calibrated"]["omega_rmse_rps"] for key in identifiers],
        width,
        label="calibrated",
    )
    plt.xticks(x, identifiers, rotation=55, ha="right")
    plt.ylabel("body omega RMSE [rad/s]")
    plt.title("Held-out per-condition yaw-rate RMSE")
    plt.legend()
    plt.grid(axis="y", alpha=0.3)
    save_figure(root / "plots/05_held_out_condition_omega_rmse.png")


def plot_composite(root: Path, composite: Mapping[str, Any]) -> None:
    reference = np.load(root / "composite_validation/reference_trajectory.npy")
    plt.figure(figsize=(7, 6))
    plt.plot(reference[:, 0], reference[:, 1], color="royalblue", label="reference")
    for row, color in zip(composite["runs"], ("orangered", "green"), strict=True):
        actual = np.load(Path(row["trial_directory"]) / "raw/actual_trajectory.npy")
        plt.plot(actual[:, 0], actual[:, 1], color=color, label=row["mode"])
    plt.xlabel("world x [m]")
    plt.ylabel("world y [m]")
    plt.title("Stage 0-B composite trajectory")
    plt.axis("equal")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/06_composite_xy.png")


def plot_replay(root: Path, paired: Mapping[str, Mapping[str, Any]], representative: str) -> None:
    selected = paired[representative]
    nominal_dir = Path(selected["nominal"]["trial_directory"])
    nominal_metadata = load_strict_json(nominal_dir / "metadata.json")
    old = np.load(Path(nominal_metadata["source_trial_path"]) / "derived/old_world.npy")
    plt.figure(figsize=(7, 6))
    plt.plot(old[:, 0], old[:, 1], color="royalblue", label="planned OLD")
    for mode, color in (("nominal", "orangered"), ("calibrated", "green")):
        actual = np.load(
            Path(selected[mode]["trial_directory"]) / "raw/actual_trajectory.npy"
        )
        plt.plot(actual[:, 0], actual[:, 1], color=color, label=f"{mode} actual")
    plt.xlabel("world x [m]")
    plt.ylabel("world y [m]")
    plt.title(f"EXP-02B OLD replay: {representative}")
    plt.axis("equal")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/07_exp02b_old_replay_xy.png")

    plt.figure(figsize=(9, 5))
    for mode, color in (("nominal", "orangered"), ("calibrated", "green")):
        telemetry_path = Path(selected[mode]["trial_directory"]) / "raw/telemetry.csv"
        with telemetry_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        times = [float(row["sim_time_s"]) for row in rows]
        plt.plot(
            times,
            [float(row["measured_omega_rps"]) for row in rows],
            color=color,
            label=f"{mode} measured",
        )
        if mode == "nominal":
            plt.plot(
                times,
                [float(row["desired_omega_rps"]) for row in rows],
                color="black",
                linestyle="--",
                label="source desired",
            )
    plt.xlabel("replay time [s]")
    plt.ylabel("omega [rad/s]")
    plt.title(f"EXP-02B OLD replay yaw-rate trace: {representative}")
    plt.legend()
    plt.grid(alpha=0.3)
    save_figure(root / "plots/08_exp02b_omega_time_trace.png")


def main() -> None:
    root = ARGS.run_directory.resolve()
    metadata = load_strict_json(root / "metadata.json")
    if metadata.get("smoke"):
        raise ValueError("smoke output is not eligible for final validation")
    criteria = load_strict_json(root / "acceptance_criteria.json")
    calibration = load_strict_json(root / "calibration_summary.json")
    held_out = load_strict_json(root / "held_out_summary.json")
    composite = load_strict_json(root / "composite_summary.json")
    replay = load_strict_json(root / "exp02b_replay_summary.json")

    composite_pair = {row["mode"]: row for row in composite["runs"]}
    composite_evaluation = evaluate_composite_acceptance(
        composite_pair["nominal"], composite_pair["calibrated"], criteria["composite"]
    )
    replay_pairs = paired_runs(replay["runs"], "case_id")
    replay_comparisons = {}
    for case_id, pair in replay_pairs.items():
        nominal_body = pair["nominal"]["body_and_wheel_execution_metrics"]
        calibrated_body = pair["calibrated"]["body_and_wheel_execution_metrics"]
        replay_comparisons[case_id] = {
            "nominal": pair["nominal"],
            "calibrated": pair["calibrated"],
            "omega_rmse_reduction_fraction": (
                float(nominal_body["omega_command_vs_measured_rmse_rps"])
                - float(calibrated_body["omega_command_vs_measured_rmse_rps"])
            )
            / float(nominal_body["omega_command_vs_measured_rmse_rps"]),
        }
    representative = str(criteria["exp02b_replay"]["representative_case"])
    representative_evaluation = evaluate_exp02b_replay_acceptance(
        replay_pairs[representative]["nominal"],
        replay_pairs[representative]["calibrated"],
        criteria["exp02b_replay"],
    )
    decision = final_validation_decision(
        calibration_grid_passed=bool(calibration["calibration_grid_passed"]),
        held_out_passed=bool(held_out["calibrated"]["passed"]),
        composite_passed=bool(composite_evaluation["passed"]),
        representative_replay_passed=bool(representative_evaluation["passed"]),
        held_out_leakage=bool(held_out["held_out_data_used_for_fitting_or_tuning"]),
    )
    final = {
        "stage": metadata["stage"],
        "run_id": metadata["run_id"],
        "decision": decision,
        "calibration_method": calibration["selected_method"],
        "calibration_parameters": calibration["selected_parameters"],
        "held_out_evaluation": held_out["calibrated"],
        "composite_evaluation": composite_evaluation,
        "representative_exp02b_replay_evaluation": representative_evaluation,
        "exp02b_replay_comparisons": replay_comparisons,
        "held_out_data_used_for_fitting_or_tuning": False,
        "claim_scope": (
            "engineering validation of the Jackal execution layer only; not evidence for "
            "a reconciliation optimization method or a causal skid-steer diagnosis"
        ),
    }
    write_json_exclusive(root / "final_execution_layer_validation.json", final)
    plot_response(root, held_out)
    plot_composite(root, composite)
    plot_replay(root, replay_pairs, representative)
    print(f"FINAL_STATUS={decision['status']}")
    print(f"SUMMARY={root / 'final_execution_layer_validation.json'}")


ARGS = parse_args()
main()
