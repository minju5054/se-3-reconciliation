#!/usr/bin/env python3
"""Run offline EXP-02A synthetic and real LightNav spatial-entry pilots."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
from typing import Any, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from reconciliation.exp02 import save_csv_exclusive
from reconciliation.exp02a import (
    CONTEXT_LABELS,
    contexts_from_config,
    synthetic_inputs,
    validate_exp02a_output,
)
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares
from reconciliation.online_switch import (
    load_strict_json,
    save_json_exclusive,
    save_npy_exclusive,
    validate_trial_output,
)
from reconciliation.oracle_correspondence import sha256_file
from reconciliation.spatial_entry import TransitionReconciliationInput
from reconciliation.transition_graph import (
    SUPPORTED_VARIANTS,
    problem_from_config,
    transition_residual_vector,
)
from reconciliation.transition_metrics import evaluate_transition_state, inter_k_separation


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=REPOSITORY_ROOT / "configs/exp02a_spatial_entry.yaml"
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def plot_variant(
    output: Path,
    *,
    inputs: TransitionReconciliationInput,
    optimized: np.ndarray,
    variant: str,
) -> None:
    old = inputs.old_poses_world
    fresh = inputs.fresh_poses_world
    suffix = inputs.selected_suffix
    boundary = inputs.committed_pose_world
    previous = inputs.incoming_previous_pose
    k = inputs.entry_context.entry_index
    figure, axis = plt.subplots(figsize=(8.3, 5.5))
    axis.plot(old[:, 0], old[:, 1], "o-", label="fixed planned OLD")
    axis.plot(fresh[:, 0], fresh[:, 1], ".--", color="0.7", label="full raw FRESH")
    axis.plot(suffix[:, 0], suffix[:, 1], "x--", label=f"selected raw FRESH[{k}:]")
    axis.plot(optimized[:, 0], optimized[:, 1], "s-", label=f"{variant} output")
    axis.scatter([boundary[0]], [boundary[1]], marker="*", s=180, label="committed B")
    axis.scatter([fresh[k, 0]], [fresh[k, 1]], marker="D", s=70, label="selected raw entry")
    axis.annotate(
        "",
        xy=boundary[:2],
        xytext=previous[:2],
        arrowprops={"arrowstyle": "->", "linewidth": 2.0, "color": "tab:green"},
    )
    axis.annotate(
        "",
        xy=optimized[0, :2],
        xytext=boundary[:2],
        arrowprops={"arrowstyle": "->", "linewidth": 2.0, "color": "tab:red"},
    )
    axis.set_title(f"EXP-02A k={k}: {variant}")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_correction(output: Path, profile: list[dict[str, Any]], variant: str) -> None:
    indices = [row["fresh_index"] for row in profile]
    translations = [row["translation_correction_m"] for row in profile]
    yaws = [row["yaw_correction_rad"] for row in profile]
    figure, translation_axis = plt.subplots(figsize=(8, 4.8))
    yaw_axis = translation_axis.twinx()
    translation_axis.plot(indices, translations, "o-", label="translation correction")
    yaw_axis.plot(indices, yaws, "s--", color="tab:orange", label="yaw correction")
    translation_axis.set_xlabel("original FRESH pose index")
    translation_axis.set_ylabel("translation correction [m]")
    yaw_axis.set_ylabel("yaw correction [rad]")
    translation_axis.set_title(variant)
    translation_axis.grid(True, alpha=0.3)
    lines = translation_axis.lines + yaw_axis.lines
    translation_axis.legend(lines, [line.get_label() for line in lines], loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_inter_k(
    output: Path,
    *,
    inputs: Mapping[str, TransitionReconciliationInput],
    states: Mapping[str, np.ndarray],
    variant: str,
) -> None:
    first = inputs[CONTEXT_LABELS[0]]
    figure, axis = plt.subplots(figsize=(8.3, 5.5))
    axis.plot(
        first.fresh_poses_world[:, 0],
        first.fresh_poses_world[:, 1],
        ".--",
        color="0.75",
        label="full raw FRESH",
    )
    axis.plot(first.old_poses_world[:, 0], first.old_poses_world[:, 1], "o-", label="fixed OLD")
    for label in CONTEXT_LABELS:
        state = states[label]
        k = inputs[label].entry_context.entry_index
        axis.plot(state[:, 0], state[:, 1], "s-", label=f"{label} k={k}")
    boundary = first.committed_pose_world
    axis.scatter([boundary[0]], [boundary[1]], marker="*", s=180, label="committed B")
    axis.set_title(f"EXP-02A inter-k comparison: {variant}")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def run_phase(
    phase_root: Path,
    *,
    inputs_by_label: Mapping[str, TransitionReconciliationInput],
    config: Mapping[str, Any],
    solver: SolverConfig,
) -> dict[str, Any]:
    first = inputs_by_label[CONTEXT_LABELS[0]]
    save_npy_exclusive(phase_root / "old_world.npy", first.old_poses_world)
    save_npy_exclusive(phase_root / "full_fresh_world.npy", first.fresh_poses_world)
    save_npy_exclusive(phase_root / "committed_pose_world.npy", first.committed_pose_world)
    save_npy_exclusive(
        phase_root / "actual_pose_before_committed.npy", first.incoming_previous_pose
    )

    phase_summary: dict[str, Any] = {"contexts": {}, "inter_k": {}}
    states_by_variant: dict[str, dict[str, np.ndarray]] = {
        variant: {} for variant in SUPPORTED_VARIANTS
    }
    for label in CONTEXT_LABELS:
        inputs = inputs_by_label[label]
        old_copy = inputs.old_poses_world.copy()
        fresh_copy = inputs.fresh_poses_world.copy()
        boundary_copy = inputs.committed_pose_world.copy()
        context_copy = inputs.entry_context.to_dict()
        directory = phase_root / f"k_{label}"
        directory.mkdir(parents=True, exist_ok=False)
        raw = inputs.selected_suffix
        save_json_exclusive(directory / "entry_context.json", context_copy)
        save_npy_exclusive(directory / "raw_suffix.npy", raw)
        raw_metrics = evaluate_transition_state(
            inputs,
            raw,
            translation_scale_m=float(config["residual_scales"]["translation_m"]),
            yaw_scale_rad=float(config["residual_scales"]["yaw_rad"]),
            minimum_translation_m=float(config["minimum_direction_translation_m"]),
        )
        save_json_exclusive(directory / "metrics_raw.json", raw_metrics)
        context_summary: dict[str, Any] = {
            "entry_context": context_copy,
            "raw": raw_metrics,
            "variants": {},
        }
        for variant in SUPPORTED_VARIANTS:
            problem = problem_from_config(
                inputs,
                variant,
                residual_scales=config["residual_scales"],
                weights=config["variants"][variant],
                minimum_translation_m=float(config["minimum_direction_translation_m"]),
            )
            result = solve_least_squares(
                problem.raw_new,
                lambda state, graph=problem: transition_residual_vector(graph, state),
                solver,
            )
            metrics = evaluate_transition_state(
                inputs,
                result.optimized,
                translation_scale_m=problem.scales.translation_m,
                yaw_scale_rad=problem.scales.yaw_rad,
                minimum_translation_m=problem.minimum_translation_m,
            )
            save_npy_exclusive(directory / f"optimized_{variant}.npy", result.optimized)
            save_json_exclusive(directory / f"metrics_{variant}.json", metrics)
            save_json_exclusive(directory / f"optimization_{variant}.json", result.to_dict())
            profile = metrics["downstream_correction"]["per_pose"]
            save_csv_exclusive(directory / f"correction_profile_{variant}.csv", profile)
            plot_variant(
                directory / f"trajectory_{variant}.png",
                inputs=inputs,
                optimized=result.optimized,
                variant=variant,
            )
            plot_correction(
                directory / f"correction_profile_{variant}.png", profile, variant
            )
            states_by_variant[variant][label] = result.optimized
            context_summary["variants"][variant] = {
                "metrics": metrics,
                "optimization": result.to_dict(),
                "weights": dict(config["variants"][variant]),
            }

        if not np.array_equal(inputs.old_poses_world, old_copy):
            raise RuntimeError("OLD input changed during optimization")
        if not np.array_equal(inputs.fresh_poses_world, fresh_copy):
            raise RuntimeError("full FRESH input changed during optimization")
        if not np.array_equal(inputs.committed_pose_world, boundary_copy):
            raise RuntimeError("committed boundary changed during optimization")
        if inputs.entry_context.to_dict() != context_copy:
            raise RuntimeError("SpatialEntryContext evidence changed during optimization")
        phase_summary["contexts"][label] = context_summary

    for variant in SUPPORTED_VARIANTS:
        separation = inter_k_separation(
            inputs_by_label,
            states_by_variant[variant],
            first_pose_count=int(config["inter_k_first_pose_count"]),
        )
        save_json_exclusive(phase_root / f"inter_k_separation_{variant}.json", separation)
        plot_inter_k(
            phase_root / f"inter_k_comparison_{variant}.png",
            inputs=inputs_by_label,
            states=states_by_variant[variant],
            variant=variant,
        )
        phase_summary["inter_k"][variant] = separation
    return phase_summary


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get("experiment") != "EXP-02A":
        raise ValueError("invalid EXP-02A config")
    if tuple(config["variants"]) != SUPPORTED_VARIANTS:
        raise ValueError("variants must be exactly pose_anchor, entry_preservation, incoming_motion_aware")
    run_id = args.run_id or datetime.now(timezone.utc).strftime("exp02a-%Y%m%dT%H%M%SZ")
    output = resolve_path(config["paths"]["output_root"]) / run_id
    output.mkdir(parents=True, exist_ok=False)
    solver = SolverConfig(**config["solver"])

    synthetic_by_label = synthetic_inputs(config["synthetic"])
    synthetic_summary = run_phase(
        output / "synthetic",
        inputs_by_label=synthetic_by_label,
        config=config,
        solver=solver,
    )

    source_experiment = resolve_path(config["paths"]["real_source_experiment"])
    source_trial = source_experiment / str(config["paths"]["real_source_trial"])
    source_validation = validate_trial_output(source_trial)
    if not source_validation["research_valid"]:
        raise ValueError("selected EXP-01B pilot trial is not timing-valid")
    raw_fresh_actions = np.load(source_trial / "raw/new_actions.npy", allow_pickle=False)
    if np.allclose(raw_fresh_actions, 0.0, atol=0.0, rtol=0.0):
        raise ValueError("selected EXP-01B FRESH trajectory is an all-zero stop response")
    old_world = np.load(source_trial / "derived/old_world.npy", allow_pickle=False)
    fresh_world = np.load(source_trial / "derived/new_world.npy", allow_pickle=False)
    switch = load_strict_json(source_trial / "results/switch_metrics.json")
    boundary = np.asarray(switch["robot_pose_at_new_ready"], dtype=np.float64)
    previous = np.asarray(switch["actual_pose_before_ready"], dtype=np.float64)
    real_contexts = contexts_from_config(config["real_contexts"])
    real_by_label = {
        label: TransitionReconciliationInput(
            old_poses_world=old_world,
            fresh_poses_world=fresh_world,
            committed_pose_world=boundary,
            entry_context=context,
            actual_pose_before_committed=previous,
            metadata={
                "source_experiment": source_experiment.name,
                "source_trial": source_trial.name,
                "role": "backend pilot, not authoritative k label",
            },
        )
        for label, context in real_contexts.items()
    }
    real_summary = run_phase(
        output / "real_lightnav",
        inputs_by_label=real_by_label,
        config=config,
        solver=solver,
    )
    source_payload = {
        "source_experiment_id": source_experiment.name,
        "source_trial_id": source_trial.name,
        "source_trial_directory": str(source_trial),
        "selection": "predeclared k=[0,3,6] before graph output inspection",
        "source_role": "EXP-01B problem-existence pilot; not a representative dataset",
        "source_validation": source_validation,
        "raw_hashes": {
            "old_actions.npy": sha256_file(source_trial / "raw/old_actions.npy"),
            "new_actions.npy": sha256_file(source_trial / "raw/new_actions.npy"),
        },
        "old_shape": list(old_world.shape),
        "fresh_shape": list(fresh_world.shape),
        "exp01b_before_metrics": switch["metrics"],
    }
    save_json_exclusive(output / "real_lightnav/source.json", source_payload)

    summary = {
        "experiment": "EXP-02A",
        "run_id": run_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_sha_at_run": subprocess.check_output(
            ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
        ).strip(),
        "config": str(args.config.resolve()),
        "interface": "OLD + FRESH + SpatialEntryContext(entry_index, evidence)",
        "evidence_used_as_graph_measurement": False,
        "variants": list(SUPPORTED_VARIANTS),
        "synthetic": synthetic_summary,
        "real_lightnav": {"source": source_payload, **real_summary},
        "claim_boundary": (
            "Oracle/manually specified k only; no selector-feature validation, generalization, "
            "online execution, navigation-quality, NavDP, or real-robot claim."
        ),
        "future_item": (
            "EXP-01B-extension: collect a larger and more geometrically diverse timing-valid "
            "transition cohort."
        ),
    }
    save_json_exclusive(output / "summary.json", summary)
    validation = validate_exp02a_output(output)
    save_json_exclusive(output / "validation.json", validation)
    print("EXP02A_OUTPUT_DIR=" + str(output))


if __name__ == "__main__":
    main()
