#!/usr/bin/env python3
"""Validate and plot a completed, untuned current-controller comparison."""

from __future__ import annotations

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
from reconciliation.closed_loop_execution_validation import validate_closed_loop_trial
from reconciliation.execution_calibration import validate_execution_trial
from reconciliation.online_switch import sha256_file


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def column(rows, key):
    return np.asarray([float(row[key]) for row in rows])


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: summarize_controller_effect_check.py RUN_DIRECTORY")
    run = Path(sys.argv[1]).resolve()
    summary = read_json(run / "summary.json")
    trials = summary["trials"]
    if len(trials) != 36 or summary["trial_count"] != 36:
        raise ValueError("comparison requires all 36 trials")
    metrics = {}
    for trial in trials:
        directory = Path(trial["directory"])
        validator = validate_closed_loop_trial if trial["kind"] == "saved_old" else validate_execution_trial
        result = validator(directory)
        if result["metrics"] != trial["metrics"]:
            raise ValueError("summary metrics differ from trial metrics")
        for name, digest in trial["raw_sha256"].items():
            if sha256_file(directory / name) != digest:
                raise ValueError("raw trial artifact hash mismatch")
        key = (trial["kind"], trial["scenario"], trial["mode"])
        metrics.setdefault(key, []).append(trial)
    if any(sorted(r["repetition"] for r in values) != [0, 1, 2]
           for values in metrics.values()):
        raise ValueError("each condition requires three distinct repetitions")
    plots = run / "plots"
    plots.mkdir(exist_ok=False)
    colors = {"nominal": "#e66b22", "calibrated": "#168c50"}
    labels = {"nominal": "Before correction", "calibrated": "Current correction"}
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    figures = []
    scenarios = sorted({trial["scenario"] for trial in trials if trial["kind"] == "saved_old"})
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout="constrained")
    for col, scenario in enumerate(scenarios):
        reference = np.load(run / "references" / f"{scenario}.npy", allow_pickle=False)
        ax = axes[0, col]
        ax.plot(reference[:, 0], reference[:, 1], color="#1764aa", lw=2, label="Saved OLD reference")
        for mode in colors:
            values = metrics[("saved_old", scenario, mode)]
            for trial in values:
                directory = Path(trial["directory"])
                actual = np.load(directory / "raw/actual_trajectory.npy", allow_pickle=False)
                data = read_csv(directory / "raw/telemetry.csv")
                label = labels[mode] if trial["repetition"] == 0 else None
                ax.plot(actual[:, 0], actual[:, 1], color=colors[mode], lw=1.5, alpha=.8, label=label)
                if trial["repetition"] == 0:
                    ax.scatter(actual[0, 0], actual[0, 1], marker="^", c=colors[mode], s=50)
                    ax.scatter(actual[-1, 0], actual[-1, 1], marker="x", c=colors[mode], s=50)
                axes[1, col].plot(column(data, "sim_time_s"), column(data, "nearest_reference_distance_m") * 100,
                                  color=colors[mode], lw=1.5, alpha=.8, label=label)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set(title=scenario, xlabel="World X [m]", ylabel="World Y [m]")
        ax.ticklabel_format(useOffset=False, style="plain")
        ax.grid(alpha=.2)
        ax.legend(fontsize=9)
        axes[1, col].set(xlabel="Time since settled start [s]", ylabel="Distance to OLD [cm]",
                         title="Three reset repetitions; stop at goal or 8 seconds")
        axes[1, col].grid(alpha=.2)
        axes[1, col].legend(fontsize=9)
    fig.suptitle("Same saved OLD and activation pose: reset-state flat-ground comparison\n"
                 "Original online velocity, PI history and Hospital contacts are not restored", fontsize=14)
    path = plots / "saved_old_comparison.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    figures.append(path)

    rotations = sorted({trial["scenario"] for trial in trials if trial["kind"] == "rotation"})
    fig, axes = plt.subplots(2, 4, figsize=(16, 8), layout="constrained")
    for col, scenario in enumerate(rotations):
        for mode in colors:
            for trial in metrics[("rotation", scenario, mode)]:
                data = read_csv(Path(trial["directory"]) / "raw/telemetry.csv")
                times = column(data, "sim_time_s")
                omega = column(data, "measured_omega_rps")
                label = labels[mode] if trial["repetition"] == 0 else None
                axes[0, col].plot(times, omega, c=colors[mode], lw=1, alpha=.8, label=label)
                xy = np.column_stack((column(data, "actual_x"), column(data, "actual_y")))
                active_indices = [i for i, row in enumerate(data) if row["phase"] == "ACTIVE"]
                anchor = xy[active_indices[0] - 1]
                axes[1, col].plot(times, np.linalg.norm(xy - anchor, axis=1) * 100,
                                  c=colors[mode], lw=1.4, alpha=.8, label=label)
                if mode == "nominal" and trial["repetition"] == 0:
                    axes[0, col].plot(times, column(data, "desired_omega_rps"), "k--", lw=1.2, label="Desired omega")
        for row in range(2):
            axes[row, col].axvspan(.5, 2.5, color="#888888", alpha=.08)
            axes[row, col].grid(alpha=.2)
            axes[row, col].set_xlabel("Time [s]")
        axes[0, col].set(title=scenario, ylabel="Measured / desired omega [rad/s]")
        axes[1, col].set_ylabel("XY drift from pre-rotation pose [cm]")
    axes[0, 0].legend(fontsize=8)
    axes[1, 0].legend(fontsize=8)
    fig.suptitle("Engineering rotation checks: zero desired translation, same flat ground and reset\n"
                 "Shaded interval: 2-second rotation; three repetitions per mode", fontsize=14)
    path = plots / "rotation_response_and_drift.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    figures.append(path)

    aggregate = []
    for (kind, scenario, mode), rows in sorted(metrics.items()):
        fields = (["position_rmse_m", "yaw_rmse_rad", "desired_omega_vs_measured_omega_rmse_rps",
                   "active_saturation_fraction", "time_to_goal_or_timeout_s"] if kind == "saved_old"
                  else ["omega_rmse_rps", "steady_measured_omega_mean_rps", "active_end_xy_drift_m",
                        "active_max_xy_drift_m", "after_final_stop_xy_drift_m", "active_saturation_fraction"])
        item = {"kind": kind, "scenario": scenario, "mode": mode, "repetitions": len(rows)}
        for field in fields:
            values = [row["metrics"][field] for row in rows]
            item[field] = {"mean": float(np.mean(values)), "min": float(np.min(values)),
                           "max": float(np.max(values)), "std": float(np.std(values))}
        if kind == "saved_old":
            item["goals"] = sum(row["metrics"]["goal_reached"] for row in rows)
        aggregate.append(item)
    result = {"validated_trial_count": len(trials), "summary_sha256": sha256_file(run / "summary.json"),
              "aggregation": "equal mean over three reset repetitions; no new acceptance thresholds",
              "plot_semantics": "world XY metres, no coordinate transform; centimetres are metres * 100; three runs shown per mode",
              "plots": {path.name: sha256_file(path) for path in figures}, "conditions": aggregate}
    with (plots / "comparison.json").open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
