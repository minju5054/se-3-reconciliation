#!/usr/bin/env python3
"""Validate EXP-02C summaries and render the required diagnostic plots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.online_switch import load_strict_json, save_json_exclusive, sha256_file  # noqa: E402


PLOT_NAMES = (
    "factor_initial_cost_benign_k0.png",
    "factor_gradient_norm_benign_k0.png",
    "factor_gradient_cosine_benign_k0.png",
    "additive_ablation_delta_v.png",
    "additive_ablation_delta_omega.png",
    "entry_displacement_by_variant.png",
    "endpoint_displacement_by_variant.png",
    "correction_profile_benign_k0.png",
    "rigid_fit_residual_by_variant.png",
    "synthetic_direction_only.png",
    "synthetic_yaw_only.png",
    "synthetic_downstream_conflict.png",
    "real_benign_k0_trajectory_overlay.png",
)
FACTORS = ("entry_preservation", "fresh_motion", "incoming_direction", "incoming_yaw")
DISPLAY = {
    "V0_RAW": "RAW",
    "V1_ENTRY_FRESH": "E+F",
    "V2_ENTRY_DIRECTION_FRESH": "E+D+F",
    "V3_ENTRY_YAW_FRESH": "E+Y+F",
    "V4_FULL_CURRENT_M4": "FULL",
    "V5_NO_ENTRY": "NO E",
    "V6_NO_DIRECTION": "NO D",
    "V7_NO_YAW": "NO Y",
    "V8_DIAGNOSTIC_NO_PROPAGATION": "NO PROP",
}
COLORS = {
    "V0_RAW": "#777777",
    "V1_ENTRY_FRESH": "#17becf",
    "V2_ENTRY_DIRECTION_FRESH": "#ff7f0e",
    "V3_ENTRY_YAW_FRESH": "#9467bd",
    "V4_FULL_CURRENT_M4": "#d62728",
    "V5_NO_ENTRY": "#2ca02c",
    "V8_DIAGNOSTIC_NO_PROPAGATION": "#e377c2",
}
ADDITIVE = (
    "V0_RAW",
    "V1_ENTRY_FRESH",
    "V2_ENTRY_DIRECTION_FRESH",
    "V3_ENTRY_YAW_FRESH",
    "V4_FULL_CURRENT_M4",
    "V5_NO_ENTRY",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    return parser.parse_args()


def save(fig, path: Path) -> None:
    if path.exists():
        raise FileExistsError(path)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def setup(title: str, ylabel: str):
    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.25)
    return fig, ax


def trajectory(ax, path: Path, variant: str, *, linestyle="-", linewidth=2.2):
    poses = np.load(path / variant / "optimized.npy", allow_pickle=False)
    ax.plot(
        poses[:, 0],
        poses[:, 1],
        linestyle,
        color=COLORS.get(variant, "black"),
        linewidth=linewidth,
        marker="o",
        markersize=3,
        label=DISPLAY[variant],
    )
    return poses


def main() -> None:
    run = parse_args().run_directory.resolve()
    metadata = load_strict_json(run / "metadata.json")
    if metadata.get("experiment") != "EXP-02C" or metadata.get("artifact_variant_count") != 126:
        raise ValueError("run is not a complete EXP-02C artifact")
    summary = run / "summary"
    attribution = load_strict_json(summary / "factor_attribution_summary.json")
    additive = load_strict_json(summary / "additive_ablation_summary.json")
    synthetic = load_strict_json(summary / "synthetic_mechanism_summary.json")
    benign = load_strict_json(summary / "benign_case_failure_attribution.json")
    if len(attribution["rows"]) != 81 or len(additive["rows"]) != 9:
        raise ValueError("EXP-02C real summary is incomplete")
    if len(synthetic["cases"]) != 5:
        raise ValueError("EXP-02C synthetic summary is incomplete")
    plots = run / "plots"
    plots.mkdir(parents=True, exist_ok=False)

    labels = [name.replace("_preservation", "").replace("incoming_", "") for name in FACTORS]
    costs = [benign["raw_initial_factor_costs"][name] for name in FACTORS]
    fig, ax = setup("Benign C, k=0: factor cost at raw FRESH", "weighted cost")
    ax.bar(labels, costs, color=["#4c78a8", "#72b7b2", "#f58518", "#b279a2"])
    ax.set_yscale("symlog", linthresh=1e-12)
    save(fig, plots / PLOT_NAMES[0])

    norms = [benign["raw_initial_weighted_gradient_norms"][name] for name in FACTORS]
    fig, ax = setup("Benign C, k=0: factor gradient at raw FRESH", "||Jᵀr|| (weighted)")
    ax.bar(labels, norms, color=["#4c78a8", "#72b7b2", "#f58518", "#b279a2"])
    ax.set_yscale("symlog", linthresh=1e-12)
    save(fig, plots / PLOT_NAMES[1])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, key, title in (
        (axes[0], "full_initial_gradient_cosines", "raw initialization"),
        (axes[1], "full_final_gradient_cosines", "FULL solution"),
    ):
        raw_matrix = benign[key]["matrix"]
        matrix = np.asarray(
            [[np.nan if value is None else value for value in row] for row in raw_matrix],
            dtype=float,
        )
        image = ax.imshow(matrix, vmin=-1, vmax=1, cmap="coolwarm")
        ax.set_xticks(range(4), labels, rotation=35, ha="right")
        ax.set_yticks(range(4), labels)
        ax.set_title(title)
        for y in range(4):
            for x in range(4):
                value = matrix[y, x]
                ax.text(x, y, "undef" if np.isnan(value) else f"{value:.2f}", ha="center", va="center", fontsize=8)
    fig.suptitle("Benign C, k=0: factor gradient alignment")
    fig.subplots_adjust(left=0.09, right=0.88, bottom=0.23, top=0.85, wspace=0.42)
    colorbar_axis = fig.add_axes([0.91, 0.22, 0.018, 0.56])
    fig.colorbar(image, cax=colorbar_axis, label="weighted gradient cosine")
    if (plots / PLOT_NAMES[2]).exists():
        raise FileExistsError(plots / PLOT_NAMES[2])
    fig.savefig(plots / PLOT_NAMES[2], dpi=180)
    plt.close(fig)

    conditions = [f"{row['case_id'].replace('case_', '')}\nk{row['entry_index']}" for row in additive["rows"]]
    x = np.arange(len(conditions))
    for filename, metric, ylabel, title in (
        (PLOT_NAMES[3], "delta_v_des_abs_mps", "|Δv desired| (m/s)", "Additive ablation: desired Δv"),
        (PLOT_NAMES[4], "delta_omega_des_abs_rps", "|Δω desired| (rad/s)", "Additive ablation: desired Δω"),
        (PLOT_NAMES[5], "entry_displacement_m", "entry displacement (m)", "Entry displacement by variant"),
        (PLOT_NAMES[6], "endpoint_displacement_m", "endpoint displacement (m)", "Endpoint displacement by variant"),
        (PLOT_NAMES[8], "rigid_fit_translation_rms_m", "rigid-fit translation RMS (m)", "Single-left-SE(2) fit residual"),
    ):
        fig, ax = setup(title, ylabel)
        for variant in ADDITIVE:
            values = [row["variants"][variant][metric] for row in additive["rows"]]
            ax.plot(x, values, marker="o", linewidth=1.5, color=COLORS[variant], label=DISPLAY[variant])
        ax.set_xticks(x, conditions, rotation=30, ha="right", fontsize=8)
        if metric == "rigid_fit_translation_rms_m":
            ax.set_yscale("log")
        ax.legend(ncol=3, fontsize=8)
        save(fig, plots / filename)

    benign_root = run / "real/case_benign_delayed/k_0"
    fig, ax = setup("Benign C, k=0: correction propagation", "||Log(Fⱼ⁻¹Xⱼ).translation|| (m)")
    for variant in (
        "V0_RAW", "V2_ENTRY_DIRECTION_FRESH", "V3_ENTRY_YAW_FRESH",
        "V4_FULL_CURRENT_M4", "V5_NO_ENTRY", "V8_DIAGNOSTIC_NO_PROPAGATION",
    ):
        geometry = load_strict_json(benign_root / variant / "geometry_metrics.json")
        values = geometry["absolute_fresh_correction"]["translation_m"]
        ax.plot(range(len(values)), values, marker="o", color=COLORS[variant], label=DISPLAY[variant])
    ax.set_xlabel("suffix node j")
    ax.legend(ncol=3, fontsize=8)
    save(fig, plots / PLOT_NAMES[7])

    for filename, case_name, variants, title in (
        (PLOT_NAMES[9], "S1_DIRECTION_ONLY", ("V0_RAW", "V2_ENTRY_DIRECTION_FRESH", "V3_ENTRY_YAW_FRESH", "V4_FULL_CURRENT_M4"), "Synthetic S1: direction-only mechanism"),
        (PLOT_NAMES[10], "S2_YAW_ONLY", ("V0_RAW", "V2_ENTRY_DIRECTION_FRESH", "V3_ENTRY_YAW_FRESH", "V4_FULL_CURRENT_M4"), "Synthetic S2: yaw-only mechanism"),
    ):
        root = run / "synthetic" / case_name
        fig, ax = plt.subplots(figsize=(7, 6))
        for variant in variants:
            trajectory(ax, root, variant)
        boundary = np.asarray([0.0, 0.0])
        ax.scatter(*boundary, marker="*", s=140, color="black", label="B")
        ax.axis("equal")
        ax.grid(alpha=0.25)
        ax.set_title(title)
        ax.set_xlabel("world x (m)")
        ax.set_ylabel("world y (m)")
        ax.legend()
        save(fig, plots / filename)

    root = run / "synthetic/S4_DOWNSTREAM_CONFLICT"
    fig, ax = plt.subplots(figsize=(7, 6))
    trajectory(ax, root, "V0_RAW")
    trajectory(ax, root, "V4_FULL_CURRENT_M4")
    trajectory(ax, root, "V8_DIAGNOSTIC_NO_PROPAGATION")
    target = np.load(root / "desired_diagnostic_target.npy", allow_pickle=False)
    ax.plot(target[:, 0], target[:, 1], "k--", marker="x", label="desired diagnostic target")
    ax.scatter(0.0, 0.0, marker="*", s=140, color="black", label="B")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.set_title("Synthetic S4: local correction / downstream conflict")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.legend()
    save(fig, plots / PLOT_NAMES[11])

    fig, ax = plt.subplots(figsize=(7.5, 6))
    for variant in (
        "V0_RAW", "V2_ENTRY_DIRECTION_FRESH", "V3_ENTRY_YAW_FRESH",
        "V4_FULL_CURRENT_M4", "V5_NO_ENTRY",
    ):
        trajectory(ax, benign_root, variant)
    old = np.load(
        Path(load_strict_json(run / "source_provenance.json")["real_cases"]["case_benign_delayed"]["trial_directory"])
        / "derived/old_world.npy",
        allow_pickle=False,
    )
    ax.plot(old[:, 0], old[:, 1], color="#1f77b4", linewidth=2.5, label="OLD")
    source = load_strict_json(run / "source_provenance.json")
    trial = Path(source["real_cases"]["case_benign_delayed"]["trial_directory"])
    attempt = load_strict_json(trial / "results/attempt.json")
    boundary = attempt["robot_pose_at_new_ready"]
    ax.scatter(boundary[0], boundary[1], marker="*", s=160, color="black", label="B")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.set_title("Real benign C, k=0: factor-isolated trajectories")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.legend(ncol=2, fontsize=8)
    save(fig, plots / PLOT_NAMES[12])

    manifest = {
        "experiment": "EXP-02C",
        "plot_count": len(PLOT_NAMES),
        "plots": [
            {"filename": name, "sha256": sha256_file(plots / name)} for name in PLOT_NAMES
        ],
        "quantitative_evidence": False,
        "role": "visual summaries of saved offline numeric artifacts",
    }
    save_json_exclusive(summary / "plot_manifest.json", manifest)
    print("EXP02C_PLOT_DIR=" + str(plots))


if __name__ == "__main__":
    main()
