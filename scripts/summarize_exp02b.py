#!/usr/bin/env python3
"""Validate, summarize, and plot an immutable EXP-02B run."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from reconciliation.exp02b import CASE_IDS, METHODS, validate_exp02b_output
from reconciliation.online_switch import load_strict_json, save_json_exclusive


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
METHOD_LABELS = {
    "raw_f0": "raw FRESH[0:]",
    "raw_k": "raw FRESH[k:]",
    "pose_anchor": "pose anchor",
    "rigid": "analytic rigid",
    "graph": "incoming-aware graph",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02b_controller_aware.yaml",
    )
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("config must contain a mapping")
    return value


def branch(root: Path, execution: dict[str, Any], case: str, k: int, method: str) -> dict[str, Any]:
    execution_row = next(
        row
        for row in execution["branches"]
        if row["case_id"] == case and row["entry_index"] == k and row["method"] == method
    )
    geometric = load_strict_json(root / case / f"k_{k}" / method / "geometric_metrics.json")
    return {"geometric": geometric, **execution_row}


def controller_rows(root: Path, case: str, k: int, method: str) -> list[dict[str, str]]:
    path = root / case / f"k_{k}" / method / "execution/controller_commands.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def plot_turning_trajectory(root: Path, output: Path, source: dict[str, Any], k: int) -> None:
    case = "case_high_delta_omega"
    trial = Path(source["cases"][case]["trial_directory"])
    old = np.load(trial / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(trial / "derived/fresh_world.npy", allow_pickle=False)
    attempt = load_strict_json(trial / "results/attempt.json")
    boundary = np.asarray(attempt["robot_pose_at_new_ready"])
    previous = np.asarray(attempt["actual_pose_before_ready"])
    with (trial / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        source_commands = list(csv.DictReader(stream))
    actual_old = np.asarray(
        [
            [float(row["actual_x"]), float(row["actual_y"])]
            for row in source_commands
            if row["reference_source"] == "OLD"
        ]
    )
    figure, axis = plt.subplots(figsize=(8.8, 6.2))
    axis.plot(old[:, 0], old[:, 1], "o-", label="OLD reference")
    axis.plot(
        actual_old[:, 0], actual_old[:, 1], "k.-", linewidth=1.2, label="actual pre-switch path"
    )
    axis.plot(fresh[:, 0], fresh[:, 1], ".--", color="0.72", label="full raw FRESH")
    for method, style in (("raw_k", "x--"), ("pose_anchor", "+:"), ("rigid", "s-"), ("graph", "D-")):
        candidate = np.load(root / case / f"k_{k}" / method / "candidate.npy", allow_pickle=False)
        axis.plot(candidate[:, 0], candidate[:, 1], style, label=METHOD_LABELS[method])
    axis.scatter([boundary[0]], [boundary[1]], marker="*", s=180, label="committed B")
    axis.annotate("", xy=boundary[:2], xytext=previous[:2], arrowprops={"arrowstyle": "->", "lw": 2})
    axis.set_title(f"EXP-02B turning case, spatial entry k={k}")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_command_trace(
    root: Path, output: Path, source: dict[str, Any], *, component: str
) -> None:
    case, k = "case_high_delta_omega", 3
    field = "v_command_mps" if component == "v" else "omega_command_rps"
    units = "m/s" if component == "v" else "rad/s"
    figure, axis = plt.subplots(figsize=(8.2, 4.8))
    trial = Path(source["cases"][case]["trial_directory"])
    with (trial / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        old_rows = [
            row for row in csv.DictReader(stream) if row["reference_source"] == "OLD"
        ][-3:]
    for method in ("raw_f0", "raw_k", "rigid", "graph"):
        rows = controller_rows(root, case, k, method)
        times = [-0.3, -0.2, -0.1] + [float(row["time_from_switch_s"]) for row in rows]
        values = [float(row[field]) for row in old_rows] + [float(row[field]) for row in rows]
        axis.plot(
            times,
            values,
            marker="o",
            label=METHOD_LABELS[method],
        )
    axis.axvline(0.0, color="black", linewidth=1.0, linestyle="--")
    axis.set_xlabel("time relative to switch [s]")
    axis.set_ylabel(f"controller {component} [{units}]")
    axis.set_title(f"Turning case k=3: controller {component} trace")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_immediate(rows: list[dict[str, Any]], output: Path, *, component: str) -> None:
    metric = "delta_v_abs_mps" if component == "v" else "delta_omega_abs_rps"
    units = "m/s" if component == "v" else "rad/s"
    case_labels = {
        "case_high_delta_v": "A high delta-v",
        "case_high_delta_omega": "B high delta-omega",
        "case_benign_delayed": "C benign delayed",
    }
    labels = [f"{case_labels[case]}\nk={k}" for case in CASE_IDS for k in (0, 3, 6)]
    x = np.arange(len(labels), dtype=float)
    width = 0.16
    figure, axis = plt.subplots(figsize=(13, 5.2))
    for offset, method in enumerate(METHODS):
        values = [
            next(
                row["controller_metrics"][metric]
                for row in rows
                if row["case_id"] == case and row["entry_index"] == k and row["method"] == method
            )
            for case in CASE_IDS
            for k in (0, 3, 6)
        ]
        axis.bar(x + (offset - 2) * width, values, width, label=METHOD_LABELS[method])
        axis.scatter(x + (offset - 2) * width, values, s=12, color="black", zorder=3)
    axis.set_xticks(x, labels)
    axis.set_ylabel(f"immediate |delta {component}| [{units}]")
    axis.set_title(f"EXP-02B immediate controller delta-{component}")
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend(loc="upper right", fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_tradeoff(summary_rows: list[dict[str, Any]], output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    for method, marker in (("pose_anchor", "^"), ("rigid", "s"), ("graph", "o")):
        selected = [row for row in summary_rows if row["method"] == method]
        selected_v = [
            row for row in selected if row["same_k_improvement"]["delta_v"] is not None
        ]
        selected_omega = [
            row for row in selected if row["same_k_improvement"]["delta_omega"] is not None
        ]
        axes[0].scatter(
            [row["same_k_improvement"]["delta_v"] for row in selected_v],
            [row["geometric"]["endpoint_deviation"]["translation_m"] for row in selected_v],
            marker=marker,
            label=METHOD_LABELS[method],
        )
        axes[1].scatter(
            [row["same_k_improvement"]["delta_omega"] for row in selected_omega],
            [row["geometric"]["selected_entry_displacement"]["translation_m"] for row in selected_omega],
            marker=marker,
            label=METHOD_LABELS[method],
        )
    axes[0].set(xlabel="immediate delta-v improvement vs raw-k", ylabel="endpoint deviation [m]")
    axes[1].set(xlabel="immediate delta-omega improvement vs raw-k", ylabel="entry displacement [m]")
    for axis in axes:
        axis.axvline(0.0, color="black", linewidth=1, linestyle="--")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle("Controller smoothness versus FRESH geometric modification")
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_benign(root: Path, output: Path, source: dict[str, Any]) -> None:
    case, k = "case_benign_delayed", 0
    trial = Path(source["cases"][case]["trial_directory"])
    old = np.load(trial / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(trial / "derived/fresh_world.npy", allow_pickle=False)
    boundary = np.asarray(source["cases"][case]["boundary_pose_world"])
    figure, axis = plt.subplots(figsize=(8.5, 5.5))
    axis.plot(old[:, 0], old[:, 1], "o-", label="OLD")
    axis.plot(fresh[:, 0], fresh[:, 1], "x--", label="raw benign FRESH[0:]")
    for method, style in (("pose_anchor", "+:"), ("rigid", "s-"), ("graph", "D-")):
        candidate = np.load(root / case / f"k_{k}" / method / "candidate.npy", allow_pickle=False)
        axis.plot(candidate[:, 0], candidate[:, 1], style, label=METHOD_LABELS[method])
    axis.scatter([boundary[0]], [boundary[1]], marker="*", s=180, label="committed B")
    axis.set_title("Benign delayed case k=0: unnecessary-modification check")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    root = args.run_dir.resolve()
    config = load_yaml(args.config.resolve())
    validation = validate_exp02b_output(root, config)
    if not validation["execution_present"]:
        raise ValueError("EXP-02B summarization requires completed Isaac branch execution")
    source = load_strict_json(root / "source_selection.json")
    offline = load_strict_json(root / "offline_summary.json")
    execution = load_strict_json(root / "execution_summary.json")
    rows = []
    for case in CASE_IDS:
        for k in config["entry_indices"]:
            for method in METHODS:
                rows.append(branch(root, execution, case, int(k), method))
    plots = root / "plots"
    plots.mkdir(parents=True, exist_ok=False)
    for k in config["entry_indices"]:
        plot_turning_trajectory(
            root, plots / f"turning_case_trajectory_comparison_k{k}.png", source, int(k)
        )
    plot_command_trace(root, plots / "turning_case_controller_v_trace_k3.png", source, component="v")
    plot_command_trace(root, plots / "turning_case_controller_omega_trace_k3.png", source, component="omega")
    plot_immediate(execution["branches"], plots / "immediate_delta_v_comparison.png", component="v")
    plot_immediate(execution["branches"], plots / "immediate_delta_omega_comparison.png", component="omega")
    plot_tradeoff(rows, plots / "smoothness_vs_fresh_deformation.png")
    plot_benign(root, plots / "benign_case_trajectory_comparison_k0.png", source)
    summary = {
        "experiment": "EXP-02B",
        "run_id": root.name,
        "source_cases": source["cases"],
        "entry_indices": list(config["entry_indices"]),
        "methods": list(METHODS),
        "branches": rows,
        "inter_k": {case: offline["cases"][case]["inter_k"] for case in CASE_IDS},
        "plot_files": sorted(str(path.relative_to(root)) for path in plots.glob("*.png")),
        "claim_boundary": (
            "Three selected development/stress transitions with manual k; no selector, held-out, "
            "navigation-performance, frequency, or end-to-end LightNav superiority claim."
        ),
    }
    save_json_exclusive(root / "summary.json", summary)
    validation = validate_exp02b_output(root, config)
    validation["summary_present"] = True
    validation["plot_count"] = len(summary["plot_files"])
    save_json_exclusive(root / "validation.json", validation)
    print("EXP02B_VALIDATION=" + str(validation))


if __name__ == "__main__":
    main()
