#!/usr/bin/env python3
"""Run offline synthetic and one-pair oracle SE(2) graph reconciliation."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from reconciliation.exp02 import save_csv_exclusive, synthetic_inputs, validate_exp02_output
from reconciliation.graph_metrics import (
    evaluate_graph_state,
    trajectory_error,
)
from reconciliation.graph_optimizer import SolverConfig, solve_graph
from reconciliation.online_switch import (
    load_strict_json,
    save_json_exclusive,
    save_npy_exclusive,
    validate_trial_output,
)
from reconciliation.oracle_correspondence import (
    load_oracle_annotation,
    sha256_file,
    validate_source_hashes,
)
from reconciliation.se2_graph import GraphWeights, problem_from_config


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=REPOSITORY_ROOT / "configs/exp02_oracle_graph.yaml"
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def solver_config(values) -> SolverConfig:
    return SolverConfig(**values)


def plot_trajectories(
    output: Path,
    *,
    old: np.ndarray,
    raw: np.ndarray,
    optimized: np.ndarray,
    correspondences,
    ground_truth: np.ndarray | None = None,
    boundary: np.ndarray | None = None,
) -> None:
    figure, axis = plt.subplots(figsize=(8, 5.5))
    axis.plot(old[:, 0], old[:, 1], "o-", label="fixed OLD")
    if ground_truth is not None:
        axis.plot(ground_truth[:, 0], ground_truth[:, 1], ".-", label="ground truth")
    axis.plot(raw[:, 0], raw[:, 1], "x--", label="raw NEW")
    axis.plot(optimized[:, 0], optimized[:, 1], "s-", label="optimized NEW")
    for index, item in enumerate(correspondences):
        points = np.vstack((old[item.old_index, :2], optimized[item.new_index, :2]))
        axis.plot(points[:, 0], points[:, 1], ":", color="0.35", label="oracle link" if index == 0 else None)
    if boundary is not None:
        axis.scatter([boundary[0]], [boundary[1]], marker="*", s=180, label="fixed ready boundary")
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_corrections(output: Path, profile: list[dict]) -> None:
    indices = [row["new_index"] for row in profile]
    translations = [row["translation_correction_m"] for row in profile]
    yaws = [row["yaw_correction_rad"] for row in profile]
    figure, translation_axis = plt.subplots(figsize=(8, 4.8))
    yaw_axis = translation_axis.twinx()
    translation_axis.plot(indices, translations, "o-", label="translation correction")
    yaw_axis.plot(indices, yaws, "s--", color="tab:orange", label="yaw correction")
    translation_axis.set_xlabel("NEW pose index")
    translation_axis.set_ylabel("translation correction [m]")
    yaw_axis.set_ylabel("yaw correction [rad]")
    translation_axis.grid(True, alpha=0.3)
    lines = translation_axis.lines + yaw_axis.lines
    translation_axis.legend(lines, [line.get_label() for line in lines], loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def run_variant(*, old, raw, boundary, correspondences, scales, weights, solver):
    problem = problem_from_config(
        fixed_old=old,
        raw_new=raw,
        fixed_boundary=boundary,
        correspondences=correspondences,
        residual_scales=scales,
        weights=weights,
    )
    result = solve_graph(problem, solver)
    return problem, result, evaluate_graph_state(problem, result.optimized)


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get("experiment") != "EXP-02":
        raise ValueError("invalid EXP-02 config")
    run_id = args.run_id or datetime.now(timezone.utc).strftime("exp02-%Y%m%dT%H%M%SZ")
    output = resolve_path(config["paths"]["output_root"]) / run_id
    output.mkdir(parents=True, exist_ok=False)
    solver = solver_config(config["solver"])
    scales = config["residual_scales"]
    full_weights = config["weights"]

    # Phase A: all constraints are exactly consistent with the known answer.
    synthetic = synthetic_inputs(config["synthetic"])
    synthetic_problem, synthetic_result, synthetic_after = run_variant(
        old=synthetic["fixed_old"],
        raw=synthetic["raw_new"],
        boundary=synthetic["fixed_boundary"],
        correspondences=synthetic["correspondences"],
        scales=scales,
        weights=full_weights,
        solver=solver,
    )
    synthetic_before = evaluate_graph_state(synthetic_problem, synthetic["raw_new"])
    recovery = trajectory_error(synthetic["expected_new"], synthetic_result.optimized)
    acceptance_config = config["synthetic"]["acceptance"]
    acceptance = {
        "solver_converged": synthetic_result.converged,
        "cost_ratio": synthetic_result.final_cost / synthetic_result.initial_cost,
        "pose_translation": recovery["translation_max_m"]
        <= float(acceptance_config["pose_translation_max_m"]),
        "pose_yaw": recovery["yaw_max_rad"] <= float(acceptance_config["pose_yaw_max_rad"]),
        "correspondence": synthetic_after["correspondence"]["translation_rms_m"]
        <= float(acceptance_config["correspondence_translation_rms_m"]),
        "new_motion": synthetic_after["new_motion_distortion"]["translation_rms_m"]
        <= float(acceptance_config["new_motion_translation_rms_m"]),
    }
    acceptance["cost"] = acceptance["cost_ratio"] <= float(
        acceptance_config["final_to_initial_cost_ratio_max"]
    )
    accepted = all(acceptance.values())
    synthetic_dir = output / "synthetic"
    synthetic_dir.mkdir()
    save_npy_exclusive(synthetic_dir / "raw_new.npy", synthetic["raw_new"])
    save_npy_exclusive(synthetic_dir / "ground_truth.npy", synthetic["expected_new"])
    save_npy_exclusive(synthetic_dir / "optimized_new.npy", synthetic_result.optimized)
    save_json_exclusive(
        synthetic_dir / "oracle.json",
        {
            "measurement_convention": "Z_ij = inverse(O_i) * desired_X_j",
            "correspondences": [item.to_dict() for item in synthetic["correspondences"]],
            "known_global_left_perturbation": synthetic["perturbation"].tolist(),
        },
    )
    synthetic_metrics = {
        "accepted": accepted,
        "acceptance_checks": acceptance,
        "recovery_error": recovery,
        "before": synthetic_before,
        "after": synthetic_after,
    }
    save_json_exclusive(synthetic_dir / "metrics.json", synthetic_metrics)
    save_json_exclusive(synthetic_dir / "optimization.json", synthetic_result.to_dict())
    save_csv_exclusive(synthetic_dir / "correction_profile.csv", synthetic_after["correction_profile"])
    plot_trajectories(
        synthetic_dir / "trajectory.png",
        old=synthetic["fixed_old"],
        raw=synthetic["raw_new"],
        optimized=synthetic_result.optimized,
        ground_truth=synthetic["expected_new"],
        boundary=synthetic["fixed_boundary"],
        correspondences=synthetic["correspondences"],
    )
    plot_corrections(synthetic_dir / "correction_profile.png", synthetic_after["correction_profile"])
    if not accepted:
        raise RuntimeError("synthetic known-answer acceptance failed; real pair was not run")

    # Phase B: one immutable valid, non-stop EXP-01B development pair.
    source_experiment = resolve_path(config["paths"]["real_source_experiment"])
    source_trial = source_experiment / str(config["paths"]["real_source_trial"])
    source_validation = validate_trial_output(source_trial)
    if not source_validation["research_valid"]:
        raise ValueError("selected EXP-01B source trial is not timing-valid")
    raw_actions = np.load(source_trial / "raw/new_actions.npy", allow_pickle=False)
    if np.allclose(raw_actions, 0.0, atol=0.0, rtol=0.0):
        raise ValueError("selected real development NEW is a stop chunk")
    old_world = np.load(source_trial / "derived/old_world.npy", allow_pickle=False)
    new_world = np.load(source_trial / "derived/new_world.npy", allow_pickle=False)
    switch = load_strict_json(source_trial / "results/switch_metrics.json")
    boundary = np.asarray(switch["robot_pose_at_new_ready"], dtype=np.float64)
    previous = np.asarray(switch["actual_pose_before_ready"], dtype=np.float64)
    oracle_path = resolve_path(config["paths"]["real_oracle"])
    oracle_document, correspondences = load_oracle_annotation(
        oracle_path, old_count=old_world.shape[0], new_count=new_world.shape[0]
    )
    validate_source_hashes(oracle_document, source_trial)

    ablation_results = {}
    full_problem = None
    full_result = None
    full_metrics = None
    for name, weights in config["ablations"].items():
        problem, result, metrics = run_variant(
            old=old_world,
            raw=new_world,
            boundary=boundary,
            correspondences=correspondences,
            scales=scales,
            weights=weights,
            solver=solver,
        )
        metrics = evaluate_graph_state(problem, result.optimized, actual_pose_before_boundary=previous)
        variant_dir = output / "ablations" / name
        variant_dir.mkdir(parents=True)
        save_npy_exclusive(variant_dir / "optimized_new_world.npy", result.optimized)
        save_json_exclusive(variant_dir / "metrics.json", metrics)
        save_json_exclusive(variant_dir / "optimization.json", result.to_dict())
        ablation_results[name] = {
            "weights": dict(weights),
            "metrics": metrics,
            "optimization": result.to_dict(),
        }
        if name == "full":
            full_problem, full_result, full_metrics = problem, result, metrics
    if full_problem is None or full_result is None or full_metrics is None:
        raise ValueError("ablations must contain a full variant")

    sensitivity_results = {}
    sensitivity_root = output / "weight_sensitivity"
    sensitivity_root.mkdir()
    ratios = list(config["weight_sensitivity"]["predefined_correspondence_to_new_motion_ratios"])
    if ratios != [0.25, 1.0, 4.0]:
        raise ValueError("EXP-02 sensitivity grid must remain the predefined [0.25, 1.0, 4.0]")
    for ratio in ratios:
        weights = GraphWeights(
            boundary=float(config["weight_sensitivity"]["fixed_boundary_weight"]),
            correspondence=float(ratio),
            new_motion=float(config["weight_sensitivity"]["fixed_new_motion_weight"]),
        )
        problem, result, metrics = run_variant(
            old=old_world,
            raw=new_world,
            boundary=boundary,
            correspondences=correspondences,
            scales=scales,
            weights={
                "boundary": weights.boundary,
                "correspondence": weights.correspondence,
                "new_motion": weights.new_motion,
            },
            solver=solver,
        )
        metrics = evaluate_graph_state(problem, result.optimized, actual_pose_before_boundary=previous)
        label = f"ratio_{ratio:g}".replace(".", "p")
        directory = sensitivity_root / label
        directory.mkdir()
        save_npy_exclusive(directory / "optimized_new_world.npy", result.optimized)
        save_json_exclusive(directory / "metrics.json", metrics)
        save_json_exclusive(directory / "optimization.json", result.to_dict())
        sensitivity_results[str(ratio)] = {
            "weights": {
                "boundary": weights.boundary,
                "correspondence": weights.correspondence,
                "new_motion": weights.new_motion,
            },
            "metrics": metrics,
            "optimization": result.to_dict(),
        }

    real_before = evaluate_graph_state(
        full_problem, new_world, actual_pose_before_boundary=previous
    )
    real_dir = output / "real_lightnav"
    real_dir.mkdir()
    source_payload = {
        "source_experiment_id": source_experiment.name,
        "source_trial_id": source_trial.name,
        "source_trial_directory": str(source_trial),
        "selection_reason": oracle_document["selection"]["reason"],
        "source_validation": source_validation,
        "raw_hashes": {
            "old_actions.npy": sha256_file(source_trial / "raw/old_actions.npy"),
            "new_actions.npy": sha256_file(source_trial / "raw/new_actions.npy"),
        },
        "old_shape": list(old_world.shape),
        "new_shape": list(new_world.shape),
        "exp01b_before_metrics": switch["metrics"],
    }
    save_json_exclusive(real_dir / "source.json", source_payload)
    save_npy_exclusive(real_dir / "raw_new_world.npy", new_world)
    save_npy_exclusive(real_dir / "optimized_new_world.npy", full_result.optimized)
    save_json_exclusive(real_dir / "oracle.json", oracle_document)
    save_json_exclusive(real_dir / "metrics_before.json", real_before)
    save_json_exclusive(real_dir / "metrics_after.json", full_metrics)
    save_json_exclusive(
        real_dir / "factor_residuals.json",
        {
            "before": {
                "boundary": real_before["boundary"],
                "correspondence": real_before["correspondence"],
                "new_motion_distortion": real_before["new_motion_distortion"],
            },
            "after": {
                "boundary": full_metrics["boundary"],
                "correspondence": full_metrics["correspondence"],
                "new_motion_distortion": full_metrics["new_motion_distortion"],
            },
        },
    )
    save_json_exclusive(real_dir / "optimization.json", full_result.to_dict())
    save_csv_exclusive(real_dir / "correction_profile.csv", full_metrics["correction_profile"])
    plot_trajectories(
        real_dir / "trajectory_before_after.png",
        old=old_world,
        raw=new_world,
        optimized=full_result.optimized,
        boundary=boundary,
        correspondences=correspondences,
    )
    plot_corrections(real_dir / "correction_profile.png", full_metrics["correction_profile"])

    summary = {
        "experiment": "EXP-02",
        "run_id": run_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_sha_at_run": subprocess.check_output(
            ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
        ).strip(),
        "config": str(args.config.resolve()),
        "synthetic": {
            "accepted": accepted,
            "perturbation": synthetic["perturbation"].tolist(),
            "initial_cost": synthetic_result.initial_cost,
            "final_cost": synthetic_result.final_cost,
            "optimization": synthetic_result.to_dict(),
            "recovery_error": recovery,
            "correspondence_before": synthetic_before["correspondence"],
            "correspondence_after": synthetic_after["correspondence"],
            "new_motion_after": synthetic_after["new_motion_distortion"],
        },
        "real_lightnav": {
            "source": source_payload,
            "oracle_correspondences": [item.to_dict() for item in correspondences],
            "before": real_before,
            "full": {"metrics": full_metrics, "optimization": full_result.to_dict()},
            "ablations": ablation_results,
            "weight_sensitivity": sensitivity_results,
        },
        "claim_boundary": (
            "One oracle-annotated LightNav development pair; no detector, generalization, "
            "online execution, navigation-quality, or real-robot claim."
        ),
    }
    save_json_exclusive(output / "summary.json", summary)
    validation = validate_exp02_output(output)
    save_json_exclusive(output / "validation.json", validation)
    print("EXP02_OUTPUT_DIR=" + str(output))
    print("EXP02_VALIDATION=" + json.dumps(validation, sort_keys=True))
    print("EXP02_SUMMARY=" + json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
