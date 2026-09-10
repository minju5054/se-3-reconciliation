#!/usr/bin/env python3
"""Run the frozen EXP-02D lookahead-direction development-corpus study."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np
import yaml


CODE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CODE_ROOT / "src"))

from reconciliation.exp02d_analysis import (  # noqa: E402
    EXP02D_INPUT_RECONSTRUCTION_FAILED,
    RegimeThresholds,
    cluster_bootstrap_pair_balanced_difference,
    command_discontinuity,
    load_exp02d_development_corpus,
    m4_false_correction_rescued,
    ordered_pair_groups,
    pair_balanced_mean,
    regime_memberships,
    select_representative_ids,
    success_failure_label,
    transition_weighted_mean,
)
from reconciliation.exp02d_lookahead_direction import (  # noqa: E402
    METHODS,
    METHOD_FACTORS,
    GeometryUndefinedError,
    candidate_command_metrics,
    candidate_deformation_metrics,
    problem_from_config,
    solve_method,
    transition_geometry_metrics,
    undefined_geometry_statuses,
)
from reconciliation.graph_optimizer import OptimizationError, SolverConfig  # noqa: E402
from reconciliation.online_switch import (  # noqa: E402
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
)
from reconciliation.spatial_entry import (  # noqa: E402
    SpatialEntryContext,
    TransitionReconciliationInput,
)


METHOD_SHORT = {
    "M0_RAW": "raw",
    "M1_HISTORICAL_M4": "m1",
    "M2_NO_DIRECTION": "m2",
    "M3_LOOKAHEAD": "m3",
}

DEFORMATION_FIELDS = (
    "translation_deformation_rms_m",
    "yaw_deformation_rms_rad",
    "entry_displacement_m",
    "q_node_displacement_m",
    "endpoint_displacement_m",
    "fresh_relative_motion_translation_rms_m",
    "fresh_relative_motion_yaw_rms_rad",
    "rigid_fit_translation_rms_m",
    "rigid_fit_yaw_rms_rad",
)

SUCCESS_FAILURE_LABELS = (
    "BENIGN_PRESERVED",
    "BENIGN_BROKEN",
    "CHALLENGING_RESCUED",
    "CHALLENGING_IMPROVED",
    "CHALLENGING_WORSE",
    "CHALLENGING_MIXED",
    "GEOMETRY_UNDEFINED",
    "SOLVER_FAILURE",
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=CODE_ROOT / "configs/exp02d_lookahead_direction.yaml",
    )
    parser.add_argument(
        "--phase", choices=("dry-validation", "primary"), required=True
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--source-repository-root",
        type=Path,
        default=CODE_ROOT,
        help="repository containing immutable ignored DATA-02 source trees",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        help="override the generated output root (used by a clean detached worktree)",
    )
    parser.add_argument("--require-git-sha")
    return parser.parse_args()


def strict_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def git(*args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(CODE_ROOT), *args], text=True
    ).strip()


def resolve_under(root: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def write_yaml_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(dict(value), stream, sort_keys=False)


def write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError("cannot write an empty EXP-02D CSV")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0])
    if any(list(row) != fields for row in rows):
        raise ValueError("EXP-02D CSV rows have inconsistent field order")
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def validate_protocol(config: Mapping[str, Any]) -> None:
    if (
        config.get("experiment") != "EXP-02D"
        or tuple(config.get("methods", ())) != METHODS
    ):
        raise ValueError("EXP-02D method identity/order changed")
    scope = config["scope"]
    if scope["data02_combined_decision"] != "DATA02_COMBINED_DIVERSITY_INSUFFICIENT":
        raise ValueError("historical DATA-02 combined decision changed")
    if scope["final_test_set"] is not False:
        raise ValueError("EXP-02D corpus must remain development-only")

    exp02a = strict_yaml(
        resolve_under(CODE_ROOT, config["paths"]["historical_exp02a_config"])
    )
    exp02c = strict_yaml(
        resolve_under(CODE_ROOT, config["paths"]["historical_exp02c_config"])
    )
    if (
        config["residual_scales"] != exp02a["residual_scales"]
        or config["residual_scales"] != exp02c["residual_scales"]
    ):
        raise ValueError("EXP-02D residual scales differ from historical M4")
    historical = exp02a["variants"]["incoming_motion_aware"]
    weight_names = {
        "entry_preservation",
        "incoming_direction",
        "incoming_yaw",
        "fresh_motion",
    }
    if set(config["historical_weights"]) != weight_names:
        raise ValueError("EXP-02D historical weight names changed")
    for key, value in config["historical_weights"].items():
        if float(value) != float(historical[key]) or float(value) != float(
            exp02c["historical_weights"][key]
        ):
            raise ValueError(f"EXP-02D historical weight changed: {key}")
    if float(config["minimum_direction_translation_m"]) != float(
        exp02a["minimum_direction_translation_m"]
    ):
        raise ValueError("EXP-02D minimum translation differs from historical M4")
    if config["solver"] != exp02a["solver"] or config["solver"] != exp02c["solver"]:
        raise ValueError("EXP-02D solver differs from historical M4")

    evaluation = config["evaluation"]
    frozen_evaluation = {
        "command_score_linear_scale_mps": 0.15,
        "command_score_angular_scale_rps": 0.30,
        "benign_max_abs_delta_v_mps": 0.05,
        "benign_max_abs_delta_omega_rps": 0.10,
        "challenging_min_abs_delta_v_mps": 0.15,
        "challenging_min_abs_delta_omega_rps": 0.30,
        "challenging_improved_ratio": 0.80,
        "challenging_worse_ratio": 1.20,
        "bootstrap_seed": 20260910,
        "bootstrap_repetitions": 10000,
        "confidence_level": 0.95,
    }
    if set(evaluation) != set(frozen_evaluation):
        raise ValueError("EXP-02D frozen evaluation fields changed")
    for key, expected in frozen_evaluation.items():
        if float(evaluation[key]) != float(expected):
            raise ValueError(f"EXP-02D frozen evaluation setting changed: {key}")
    if float(config["oracle"]["expected_lookahead_distance_m"]) != 0.25:
        raise ValueError("EXP-02D frozen follower lookahead expectation changed")
    regime_thresholds(config)


def validate_source_integrity(
    config: Mapping[str, Any], source_repository: Path
) -> dict[str, str]:
    paths = config["paths"]
    expected = config["source_integrity"]
    checks = {
        "combined_manifest_sha256": resolve_under(source_repository, paths["combined_corpus"]) / "combined_manifest.json",
        "all_transition_index_sha256": resolve_under(source_repository, paths["combined_corpus"]) / "all_transition_index.json",
        "v1_collection_manifest_sha256": resolve_under(source_repository, paths["v1_run"]) / "collection_manifest.json",
        "v1_config_snapshot_sha256": resolve_under(source_repository, paths["v1_run"]) / "config_snapshot.yaml",
        "v2_collection_manifest_sha256": resolve_under(source_repository, paths["v2_run"]) / "collection_manifest.json",
        "v2_config_snapshot_sha256": resolve_under(source_repository, paths["v2_run"]) / "config_snapshot.yaml",
    }
    actual = {key: sha256_file(path) for key, path in checks.items()}
    for key, digest in actual.items():
        if digest != str(expected[key]):
            raise ValueError(f"immutable DATA-02 source hash changed: {key}")
    return actual


def transition_problem(source: Any, config: Mapping[str, Any]):
    context = SpatialEntryContext(
        entry_index=source.k_fresh,
        source="EXP02D_FROZEN_TRAJECTORY_FOLLOWER_AT_B",
        metadata={
            "q_fresh": source.q_fresh,
            "oracle_only": True,
            "final_correspondence_detector": False,
        },
    )
    inputs = TransitionReconciliationInput(
        old_poses_world=source.old_world,
        fresh_poses_world=source.fresh_world,
        committed_pose_world=source.boundary_pose,
        actual_pose_before_committed=source.pose_before_boundary,
        entry_context=context,
        metadata={
            "corpus_transition_id": source.corpus_transition_id,
            "waypoint_dt": None,
            "corpus_role": "EXP02D_DEVELOPMENT_CORPUS",
        },
    )
    return problem_from_config(
        inputs,
        source.q_fresh,
        residual_scales=config["residual_scales"],
        weights=config["historical_weights"],
        minimum_translation_m=float(config["minimum_direction_translation_m"]),
    )


def compact_deformation(full: Mapping[str, Any]) -> dict[str, Any]:
    rigid = full["best_fit_single_rigid_transform"]
    fresh = full["fresh_relative_motion"]
    return {
        "entry_displacement_m": float(full["entry_displacement_from_raw"]["translation_m"]),
        "q_node_displacement_m": float(full["q_node_displacement_from_raw"]["translation_m"]),
        "endpoint_displacement_m": float(full["endpoint_displacement_from_raw"]["translation_m"]),
        "translation_deformation_rms_m": float(full["translation_deformation_rms_m"]),
        "yaw_deformation_rms_rad": float(full["yaw_deformation_rms_rad"]),
        "fresh_relative_motion_translation_rms_m": float(fresh["translation_rms_m"]),
        "fresh_relative_motion_yaw_rms_rad": float(fresh["yaw_rms_rad"]),
        "rigid_fit_translation_rms_m": float(rigid["translation_rms_m"]),
        "rigid_fit_yaw_rms_rad": float(rigid["yaw_rms_rad"]),
    }


def _integer_distribution(values: Iterable[int]) -> dict[str, int]:
    counts = Counter(int(value) for value in values)
    return {str(key): counts[key] for key in sorted(counts)}


def validate_m0_follower_reconstruction(
    corpus: Any, config: Mapping[str, Any]
) -> dict[str, Any]:
    """Preflight full-raw and selected-suffix follower invariants before output."""

    tolerance = float(config["oracle"]["first_command_absolute_tolerance"])
    expected_lookahead = float(config["oracle"]["expected_lookahead_distance_m"])
    maximum_full_error = 0.0
    maximum_suffix_error = 0.0
    for source in corpus.transitions:
        if source.follower_config.lookahead_distance_m != expected_lookahead:
            raise RuntimeError(
                f"{EXP02D_INPUT_RECONSTRUCTION_FAILED}: "
                f"{source.corpus_transition_id} follower lookahead differs from frozen 0.25 m"
            )
        full_command = np.asarray(
            [
                source.raw_follower_command.linear_velocity_mps,
                source.raw_follower_command.angular_velocity_rps,
            ],
            dtype=np.float64,
        )
        full_error = float(
            np.max(np.abs(full_command - source.raw_fresh_first_desired_command))
        )
        maximum_full_error = max(maximum_full_error, full_error)
        suffix = candidate_command_metrics(
            source.fresh_world[source.k_fresh :],
            boundary=source.boundary_pose,
            old_last_command=source.old_desired_command,
            follower_values=asdict(source.follower_config),
            original_entry_index=source.k_fresh,
            raw_target_index=source.q_fresh,
            linear_normalization_mps=float(
                config["evaluation"]["command_score_linear_scale_mps"]
            ),
            angular_normalization_rps=float(
                config["evaluation"]["command_score_angular_scale_rps"]
            ),
        )
        suffix_command = np.asarray(
            suffix["candidate_first_desired_v_omega"], dtype=np.float64
        )
        suffix_error = float(
            np.max(np.abs(suffix_command - source.raw_fresh_first_desired_command))
        )
        maximum_suffix_error = max(maximum_suffix_error, suffix_error)
        indices_match = (
            suffix["candidate_nearest_index"] == source.k_fresh
            and suffix["candidate_target_index"] == source.q_fresh
        )
        if full_error > tolerance or suffix_error > tolerance or not indices_match:
            raise RuntimeError(
                f"{EXP02D_INPUT_RECONSTRUCTION_FAILED}: "
                f"{source.corpus_transition_id} M0 follower reconstruction differs; "
                f"full_error={full_error:.17g}, suffix_error={suffix_error:.17g}, "
                f"saved_k_q=({source.k_fresh},{source.q_fresh}), "
                f"suffix_k_q=({suffix['candidate_nearest_index']},"
                f"{suffix['candidate_target_index']})"
            )
    return {
        "transition_count": len(corpus.transitions),
        "unique_ordered_raw_pair_count": len(
            {source.ordered_raw_pair_sha256 for source in corpus.transitions}
        ),
        "lookahead_distance_m": expected_lookahead,
        "first_command_absolute_tolerance": tolerance,
        "maximum_full_raw_first_command_absolute_error": maximum_full_error,
        "maximum_selected_suffix_first_command_absolute_error": maximum_suffix_error,
        "all_selected_suffix_absolute_indices_match_raw_k_q": True,
        "k_fresh_distribution": _integer_distribution(
            source.k_fresh for source in corpus.transitions
        ),
        "q_fresh_distribution": _integer_distribution(
            source.q_fresh for source in corpus.transitions
        ),
        "q_minus_k_distribution": _integer_distribution(
            source.q_relative for source in corpus.transitions
        ),
    }


def evaluate_transition(source: Any, config: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate all four frozen methods without writing or mutating source arrays."""

    old_copy = np.array(source.old_world, copy=True)
    fresh_copy = np.array(source.fresh_world, copy=True)
    problem = transition_problem(source, config)
    geometry = transition_geometry_metrics(problem, observation_pose=source.observation_pose)
    undefined = undefined_geometry_statuses(problem)
    if undefined:
        return {
            "source": source,
            "problem": problem,
            "geometry": geometry,
            "undefined": undefined,
            "methods": {},
            "solver_failures": {},
        }
    solver = SolverConfig(**config["solver"])
    follower = asdict(source.follower_config)
    methods: dict[str, Any] = {}
    failures: dict[str, str] = {}
    for method in METHODS:
        try:
            result = solve_method(problem, method, solver)
        except (GeometryUndefinedError, OptimizationError, FloatingPointError) as error:
            failures[method] = f"{type(error).__name__}: {error}"
            continue
        if result.optimization is not None and not result.optimization.converged:
            failures[method] = (
                "solver did not converge: " + result.optimization.termination_reason
            )
            continue
        # Metric/schema errors indicate a technical implementation problem, not
        # scientific evidence of solver failure, and therefore remain fatal.
        full_deformation = candidate_deformation_metrics(problem, result.candidate)
        command = candidate_command_metrics(
            result.candidate,
            boundary=source.boundary_pose,
            old_last_command=source.old_desired_command,
            follower_values=follower,
            original_entry_index=source.k_fresh,
            raw_target_index=source.q_fresh,
            linear_normalization_mps=float(
                config["evaluation"]["command_score_linear_scale_mps"]
            ),
            angular_normalization_rps=float(
                config["evaluation"]["command_score_angular_scale_rps"]
            ),
        )
        command["difficulty"] = command_discontinuity(
            command["delta_v_signed_mps"], command["delta_omega_signed_rps"]
        ).difficulty
        methods[method] = {
            "result": result,
            "command": command,
            "deformation_full": full_deformation,
            "deformation": compact_deformation(full_deformation),
        }
    if not np.array_equal(source.old_world, old_copy) or not np.array_equal(source.fresh_world, fresh_copy):
        raise RuntimeError("immutable DATA-02 source array changed during EXP-02D evaluation")
    return {
        "source": source,
        "problem": problem,
        "geometry": geometry,
        "undefined": undefined,
        "methods": methods,
        "solver_failures": failures,
    }


def normalized_geometry(source: Any, value: Mapping[str, Any]) -> dict[str, Any]:
    fresh = source.metrics["fresh_geometry"]
    return {
        **dict(value),
        "full_fresh_net_yaw_rad": float(fresh["signed_net_yaw_rad"]),
        "full_fresh_lateral_excursion_m": float(fresh["maximum_lateral_excursion_m"]),
        "effective_latency_s": float(source.effective_latency_s),
        "inference_translation_m": float(source.metrics["observation_to_model_ready_translation_m"]),
    }


def analysis_row(evaluation: Mapping[str, Any], pair_frequency: int) -> dict[str, Any]:
    source = evaluation["source"]
    geometry = evaluation["geometry"]
    methods = evaluation["methods"]
    undefined = bool(evaluation["undefined"])
    solver_failure = bool(evaluation["solver_failures"]) or len(methods) != len(METHODS)
    base = {
        "corpus_transition_id": source.corpus_transition_id,
        "cohort_id": source.cohort_id,
        "episode_id": source.episode_id,
        "transition_index": source.transition_index,
        "template_id": source.template_id,
        "variant_id": source.variant_id,
        "semantic_family": source.semantic_family,
        "physical_region": source.physical_region,
        "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
        "ordered_raw_pair_frequency": int(pair_frequency),
        "fresh_geometry_bin": source.fresh_geometry_bin,
        "raw_difficulty_bin": source.difficulty_bin,
        "geometry_undefined": undefined,
        "undefined_statuses": list(evaluation["undefined"]),
        "solver_failure": solver_failure,
        "solver_failures": dict(evaluation["solver_failures"]),
        "k_fresh": source.k_fresh,
        "q_fresh": source.q_fresh,
        "q_minus_k": source.q_relative,
        "alpha_entry_rad": geometry["alpha_entry_rad"],
        "alpha_look_rad": geometry["alpha_look_rad"],
        "alpha_entry_minus_alpha_look_rad": geometry["alpha_entry_minus_alpha_look_rad"],
        "d_k_m": geometry["boundary_to_entry_gap_m"],
        "d_q_m": geometry["boundary_to_lookahead_gap_m"],
        "b_to_fresh_k_translation_m": geometry["boundary_to_entry_gap_m"],
        "b_to_fresh_q_translation_m": geometry["boundary_to_lookahead_gap_m"],
        "fresh_arc_k_to_q_m": geometry["entry_to_lookahead_arc_length_m"],
        "observation_to_boundary_translation_m": source.metrics["observation_to_boundary_translation_m"],
        "observation_to_boundary_yaw_rad": source.metrics["observation_to_boundary_yaw_rad"],
        "p_to_b_translation_m": geometry["previous_to_boundary_translation_m"],
        "p_to_b_yaw_rad": geometry["previous_to_boundary_yaw_rad"],
        "fresh_net_yaw_rad": source.metrics["fresh_geometry"]["signed_net_yaw_rad"],
        "fresh_lateral_excursion_m": source.metrics["fresh_geometry"]["maximum_lateral_excursion_m"],
        "host_latency_s": source.host_latency_s,
        "effective_latency_s": source.effective_latency_s,
        "inference_translation_m": source.metrics["observation_to_model_ready_translation_m"],
    }
    if solver_failure or undefined:
        for short in METHOD_SHORT.values():
            base[f"j_cmd_{short}"] = None
            for key in (
                "j_cmd",
                "delta_v_signed_mps",
                "delta_omega_signed_rps",
                "delta_v_abs_mps",
                "delta_omega_abs_rps",
                "difficulty_bin",
                "candidate_nearest_index",
                "candidate_target_index",
                "candidate_target_changed_from_raw_q",
                *DEFORMATION_FIELDS,
            ):
                base[f"{short}_{key}"] = None
        base["m3_rms_translation_deformation_m"] = None
        base["m3_best_fit_rigid_translation_rms_m"] = None
        base["m3_best_fit_rigid_yaw_rms_rad"] = None
        base["success_failure_label"] = (
            "GEOMETRY_UNDEFINED" if undefined else "SOLVER_FAILURE"
        )
        base["m4_false_correction_rescued"] = False
        base["regimes"] = []
        return base
    discontinuities = {}
    for method, short in METHOD_SHORT.items():
        command = methods[method]["command"]
        deformation = methods[method]["deformation"]
        discontinuity = command_discontinuity(
            command["delta_v_signed_mps"], command["delta_omega_signed_rps"]
        )
        discontinuities[method] = discontinuity
        base[f"j_cmd_{short}"] = discontinuity.j_cmd
        base[f"{short}_j_cmd"] = discontinuity.j_cmd
        base[f"{short}_delta_v_signed_mps"] = discontinuity.delta_v_mps
        base[f"{short}_delta_omega_signed_rps"] = discontinuity.delta_omega_rps
        base[f"{short}_delta_v_abs_mps"] = abs(discontinuity.delta_v_mps)
        base[f"{short}_delta_omega_abs_rps"] = abs(discontinuity.delta_omega_rps)
        base[f"{short}_difficulty_bin"] = discontinuity.difficulty
        base[f"{short}_candidate_nearest_index"] = command[
            "candidate_nearest_index"
        ]
        base[f"{short}_candidate_target_index"] = command["candidate_target_index"]
        base[f"{short}_candidate_target_changed_from_raw_q"] = command[
            "candidate_target_changed_from_raw_q"
        ]
        for key, value in deformation.items():
            base[f"{short}_{key}"] = value
    # Stable aliases used by the predeclared pure classification helpers.
    base["j_cmd_raw"] = discontinuities["M0_RAW"].j_cmd
    base["j_cmd_m1"] = discontinuities["M1_HISTORICAL_M4"].j_cmd
    base["j_cmd_m2"] = discontinuities["M2_NO_DIRECTION"].j_cmd
    base["j_cmd_m3"] = discontinuities["M3_LOOKAHEAD"].j_cmd
    if base["raw_difficulty_bin"] != discontinuities["M0_RAW"].difficulty:
        raise RuntimeError("M0 difficulty differs from saved raw-FRESH difficulty")
    base["m3_rms_translation_deformation_m"] = base["m3_translation_deformation_rms_m"]
    base["m3_best_fit_rigid_translation_rms_m"] = base["m3_rigid_fit_translation_rms_m"]
    base["m3_best_fit_rigid_yaw_rms_rad"] = base["m3_rigid_fit_yaw_rms_rad"]
    label = success_failure_label(
        discontinuities["M0_RAW"], discontinuities["M3_LOOKAHEAD"]
    )
    base["success_failure_label"] = label or "RAW_INTERMEDIATE_UNLABELED"
    base["m4_false_correction_rescued"] = m4_false_correction_rescued(
        discontinuities["M0_RAW"],
        discontinuities["M1_HISTORICAL_M4"],
        discontinuities["M3_LOOKAHEAD"],
    )
    return base


def regime_thresholds(config: Mapping[str, Any]) -> RegimeThresholds:
    values = config["regimes"]
    return RegimeThresholds(
        alpha_entry_large_rad=float(values["alpha_entry_large_rad"]),
        alpha_look_small_rad=float(values["alpha_look_small_rad"]),
        alpha_look_large_rad=float(values["alpha_look_large_rad"]),
        positional_gap_large_m=float(values["positional_gap_large_m"]),
        degenerate_lookahead_radius_m=float(values["degenerate_lookahead_radius_m"]),
        insufficient_lookahead_arc_m=float(values["insufficient_lookahead_arc_m"]),
        large_deformation_rms_m=float(values["large_deformation_rms_m"]),
        small_command_benefit_fraction=float(values["small_command_benefit_fraction"]),
        rigid_correction_min_entry_m=float(values["rigid_correction_min_entry_m"]),
        rigid_fit_translation_rms_m=float(values["rigid_fit_translation_rms_m"]),
        rigid_fit_yaw_rms_rad=float(values["rigid_fit_yaw_rms_rad"]),
        rigid_endpoint_entry_difference_m=float(
            values["rigid_endpoint_entry_difference_m"]
        ),
    )


def numeric_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "min": None, "median": None, "mean": None, "p90": None, "max": None}
    if not np.all(np.isfinite(array)):
        raise ValueError("summary contains NaN/Inf")
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "mean": float(np.mean(array)),
        "p90": float(np.percentile(array, 90)),
        "max": float(np.max(array)),
    }


def method_table(rows: Sequence[Mapping[str, Any]], short: str) -> dict[str, Any]:
    if not rows:
        return {
            "count": 0,
            "median_abs_delta_v_mps": None,
            "median_abs_delta_omega_rps": None,
            "median_J_cmd": None,
            "p90_J_cmd": None,
            "transition_weighted_mean_J_cmd": None,
            "pair_balanced_mean_J_cmd": None,
            "median_translation_deformation_rms_m": None,
            "median_yaw_deformation_rms_rad": None,
            "median_rigid_fit_translation_rms_m": None,
            "median_rigid_fit_yaw_rms_rad": None,
            "transition_weighted_mean_translation_deformation_rms_m": None,
            "transition_weighted_mean_yaw_deformation_rms_rad": None,
            "transition_weighted_mean_rigid_fit_translation_rms_m": None,
            "transition_weighted_mean_rigid_fit_yaw_rms_rad": None,
            "pair_balanced_mean_translation_deformation_rms_m": None,
            "pair_balanced_mean_yaw_deformation_rms_rad": None,
            "pair_balanced_mean_rigid_fit_translation_rms_m": None,
            "pair_balanced_mean_rigid_fit_yaw_rms_rad": None,
            "candidate_target_changed_count": 0,
            "candidate_target_changed_fraction": None,
            "candidate_target_changed_pair_balanced_fraction": None,
            "candidate_nearest_index_distribution": {},
            "candidate_target_index_distribution": {},
        }
    changed = [bool(row[f"{short}_candidate_target_changed_from_raw_q"]) for row in rows]
    return {
        "count": len(rows),
        "median_abs_delta_v_mps": float(np.median([row[f"{short}_delta_v_abs_mps"] for row in rows])),
        "median_abs_delta_omega_rps": float(np.median([row[f"{short}_delta_omega_abs_rps"] for row in rows])),
        "median_J_cmd": float(np.median([row[f"j_cmd_{short}"] for row in rows])),
        "p90_J_cmd": float(np.percentile([row[f"j_cmd_{short}"] for row in rows], 90)),
        "transition_weighted_mean_J_cmd": transition_weighted_mean(rows, f"j_cmd_{short}"),
        "pair_balanced_mean_J_cmd": pair_balanced_mean(rows, f"j_cmd_{short}"),
        "median_translation_deformation_rms_m": float(np.median([row[f"{short}_translation_deformation_rms_m"] for row in rows])),
        "median_yaw_deformation_rms_rad": float(np.median([row[f"{short}_yaw_deformation_rms_rad"] for row in rows])),
        "median_rigid_fit_translation_rms_m": float(np.median([row[f"{short}_rigid_fit_translation_rms_m"] for row in rows])),
        "median_rigid_fit_yaw_rms_rad": float(np.median([row[f"{short}_rigid_fit_yaw_rms_rad"] for row in rows])),
        "transition_weighted_mean_translation_deformation_rms_m": transition_weighted_mean(
            rows, f"{short}_translation_deformation_rms_m"
        ),
        "transition_weighted_mean_yaw_deformation_rms_rad": transition_weighted_mean(
            rows, f"{short}_yaw_deformation_rms_rad"
        ),
        "transition_weighted_mean_rigid_fit_translation_rms_m": transition_weighted_mean(
            rows, f"{short}_rigid_fit_translation_rms_m"
        ),
        "transition_weighted_mean_rigid_fit_yaw_rms_rad": transition_weighted_mean(
            rows, f"{short}_rigid_fit_yaw_rms_rad"
        ),
        "pair_balanced_mean_translation_deformation_rms_m": pair_balanced_mean(
            rows, f"{short}_translation_deformation_rms_m"
        ),
        "pair_balanced_mean_yaw_deformation_rms_rad": pair_balanced_mean(
            rows, f"{short}_yaw_deformation_rms_rad"
        ),
        "pair_balanced_mean_rigid_fit_translation_rms_m": pair_balanced_mean(
            rows, f"{short}_rigid_fit_translation_rms_m"
        ),
        "pair_balanced_mean_rigid_fit_yaw_rms_rad": pair_balanced_mean(
            rows, f"{short}_rigid_fit_yaw_rms_rad"
        ),
        "candidate_target_changed_count": sum(changed),
        "candidate_target_changed_fraction": float(np.mean(changed)),
        "candidate_target_changed_pair_balanced_fraction": pair_balanced_mean(
            rows,
            lambda row: float(bool(row[f"{short}_candidate_target_changed_from_raw_q"])),
        ),
        "candidate_nearest_index_distribution": _integer_distribution(
            int(row[f"{short}_candidate_nearest_index"]) for row in rows
        ),
        "candidate_target_index_distribution": _integer_distribution(
            int(row[f"{short}_candidate_target_index"]) for row in rows
        ),
    }


def feature_summaries(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    features = (
        "alpha_entry_rad",
        "alpha_look_rad",
        "alpha_entry_minus_alpha_look_rad",
        "b_to_fresh_k_translation_m",
        "b_to_fresh_q_translation_m",
        "fresh_net_yaw_rad",
        "fresh_lateral_excursion_m",
        "effective_latency_s",
        "inference_translation_m",
        "ordered_raw_pair_frequency",
    )
    return {key: numeric_summary(float(row[key]) for row in rows) for key in features}


def pearson(x: Sequence[float], y: Sequence[float]) -> float | None:
    left = np.asarray(x, dtype=float); right = np.asarray(y, dtype=float)
    if len(left) < 2 or np.std(left) == 0.0 or np.std(right) == 0.0:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def hypothesis_analysis(rows: Sequence[Mapping[str, Any]], config: Mapping[str, Any]) -> dict[str, Any]:
    x = [float(row["alpha_entry_minus_alpha_look_rad"]) for row in rows]
    y = [float(row["j_cmd_m1"] - row["j_cmd_m3"]) for row in rows]
    groups = ordered_pair_groups(rows)
    gx = np.asarray([np.mean([float(row["alpha_entry_minus_alpha_look_rad"]) for row in group]) for group in groups.values()])
    gy = np.asarray([np.mean([float(row["j_cmd_m1"] - row["j_cmd_m3"]) for row in group]) for group in groups.values()])
    evaluation = config["evaluation"]
    repetitions = int(evaluation["bootstrap_repetitions"])
    rng = np.random.default_rng(int(evaluation["bootstrap_seed"]) + 1)
    indices = rng.integers(0, len(gx), size=(repetitions, len(gx)))
    bx = gx[indices]; by = gy[indices]
    bx_center = bx - np.mean(bx, axis=1, keepdims=True)
    by_center = by - np.mean(by, axis=1, keepdims=True)
    denominator = np.sqrt(np.sum(bx_center**2, axis=1) * np.sum(by_center**2, axis=1))
    correlations = np.divide(
        np.sum(bx_center * by_center, axis=1),
        denominator,
        out=np.full(repetitions, np.nan),
        where=denominator > 0.0,
    )
    finite = correlations[np.isfinite(correlations)]
    tail = (1.0 - float(evaluation["confidence_level"])) / 2.0
    return {
        "x": "alpha_entry_minus_alpha_look_rad",
        "y": "J_M1_minus_J_M3",
        "transition_weighted_pearson_r": pearson(x, y),
        "pair_balanced_pearson_r": pearson(gx.tolist(), gy.tolist()),
        "pair_cluster_bootstrap_pearson_ci": {
            "ci_lower": float(np.quantile(finite, tail)),
            "ci_upper": float(np.quantile(finite, 1.0 - tail)),
            "valid_draw_count": int(len(finite)),
            "seed": int(evaluation["bootstrap_seed"]) + 1,
            "repetitions": repetitions,
            "bootstrap_unit": "ordered_raw_pair_sha256",
        },
        "causal_claim": False,
    }


def plot_record(evaluation: Mapping[str, Any], row: Mapping[str, Any]) -> dict[str, Any]:
    source = evaluation["source"]
    return {
        "corpus_transition_id": source.corpus_transition_id,
        "raw_difficulty": source.difficulty_bin,
        "fresh_geometry": source.fresh_geometry_bin,
        "effective_latency_s": source.effective_latency_s,
        "ordered_raw_pair_frequency": row["ordered_raw_pair_frequency"],
        "success_failure_label": row["success_failure_label"],
        "geometry": {
            "alpha_entry_rad": row["alpha_entry_rad"],
            "alpha_look_rad": row["alpha_look_rad"],
        },
        "methods": {
            method: {
                "command": dict(evaluation["methods"][method]["command"]),
                "deformation": dict(evaluation["methods"][method]["deformation"]),
            }
            for method in METHODS
        },
    }


def artifact_id(corpus_transition_id: str) -> str:
    return corpus_transition_id.replace(":", "__")


def save_transition_result(
    output: Path,
    evaluation: Mapping[str, Any],
    row: Mapping[str, Any],
) -> dict[str, Any]:
    source = evaluation["source"]
    destination = output / "transitions" / artifact_id(source.corpus_transition_id)
    destination.mkdir(parents=True, exist_ok=False)
    input_reference = {
        "schema": "EXP02D_TransitionInputReference_v1",
        "corpus_transition_id": source.corpus_transition_id,
        "corpus_role": "EXP02D_DEVELOPMENT_CORPUS",
        "source_run": str(source.source_run),
        "source_transition": str(source.source_transition),
        "source_transition_sha256": source.source_transition_sha256,
        "source_artifact_sha256": dict(source.source_artifact_sha256),
        "cohort_id": source.cohort_id,
        "episode_id": source.episode_id,
        "transition_id": source.transition_id,
        "transition_index": source.transition_index,
        "template_id": source.template_id,
        "variant_id": source.variant_id,
        "semantic_family": source.semantic_family,
        "physical_region": source.physical_region,
        "old_chunk_id": source.old_chunk_id,
        "fresh_chunk_id": source.fresh_chunk_id,
        "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
        "old_shape": list(source.old_world.shape),
        "fresh_shape": list(source.fresh_world.shape),
        "P_world_se2": source.pose_before_boundary.tolist(),
        "B_world_se2": source.boundary_pose.tolist(),
        "observation_world_se2": source.observation_pose.tolist(),
        "t_obs_sim_s": source.t_obs_sim_s,
        "t_ready_sim_s": source.t_ready_sim_s,
        "t_switch_sim_s": source.t_switch_sim_s,
        "host_latency_s": source.host_latency_s,
        "effective_latency_s": source.effective_latency_s,
        "old_last_desired_v_omega": source.old_desired_command.tolist(),
        "saved_raw_fresh_first_desired_v_omega": source.raw_fresh_first_desired_command.tolist(),
        "coordinate_frame": "Isaac world x/y metres; yaw radians CCW about +Z",
        "raw_local_axes": ["forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive"],
        "waypoint_dt": None,
        "fresh_anchor": "robot world pose at final FRESH observation; B not inserted",
    }
    save_json_exclusive(destination / "input_reference.json", input_reference)
    reconstructed_raw_command = np.asarray(
        [
            source.raw_follower_command.linear_velocity_mps,
            source.raw_follower_command.angular_velocity_rps,
        ],
        dtype=np.float64,
    )
    reconstructed_error = np.abs(
        reconstructed_raw_command - source.raw_fresh_first_desired_command
    )
    m0_command = evaluation["methods"]["M0_RAW"]["command"]
    oracle = {
        "schema": "EXP02D_FrozenFollowerOracle_v1",
        "k_fresh": source.k_fresh,
        "q_fresh": source.q_fresh,
        "q_minus_k": source.q_relative,
        "lookahead_distance_m": source.follower_config.lookahead_distance_m,
        "follower_config": asdict(source.follower_config),
        "reconstructed_first_desired_v_omega": reconstructed_raw_command.tolist(),
        "saved_first_desired_v_omega": source.raw_fresh_first_desired_command.tolist(),
        "saved_command_abs_error_v_omega": reconstructed_error.tolist(),
        "saved_command_max_abs_error": float(np.max(reconstructed_error)),
        "selected_suffix_first_desired_v_omega": m0_command[
            "candidate_first_desired_v_omega"
        ],
        "selected_suffix_saved_command_max_abs_error": float(
            np.max(
                np.abs(
                    np.asarray(
                        m0_command["candidate_first_desired_v_omega"], dtype=np.float64
                    )
                    - source.raw_fresh_first_desired_command
                )
            )
        ),
        "selected_suffix_absolute_nearest_index": m0_command[
            "candidate_nearest_index"
        ],
        "selected_suffix_absolute_target_index": m0_command[
            "candidate_target_index"
        ],
        "selected_suffix_indices_match_raw_k_q": bool(
            m0_command["candidate_nearest_index"] == source.k_fresh
            and m0_command["candidate_target_index"] == source.q_fresh
        ),
        "call_count_at_B": 1,
        "experimental_oracle": True,
        "final_correspondence_detector": False,
    }
    save_json_exclusive(destination / "oracle_indices.json", oracle)
    method_hashes: dict[str, Any] = {}
    for method in METHODS:
        payload = evaluation["methods"][method]
        method_root = destination / method
        method_root.mkdir()
        candidate_path = method_root / "candidate.npy"
        save_npy_exclusive(candidate_path, payload["result"].candidate)
        candidate_hash = sha256_file(candidate_path)
        metrics = {
            "schema": "EXP02D_MethodMetrics_v1",
            "method": method,
            "active_factors": list(METHOD_FACTORS[method]),
            "candidate_sha256": candidate_hash,
            "command": payload["command"],
            "deformation": payload["deformation_full"],
            "candidate_nearest_index": payload["command"]["candidate_nearest_index"],
            "candidate_target_index": payload["command"]["candidate_target_index"],
            "raw_target_index_q": source.q_fresh,
            "effective_target_changed_from_raw_q": payload["command"]["candidate_target_index"] != source.q_fresh,
            "candidate_physically_executed": False,
            "J_cmd_in_objective": False,
        }
        save_json_exclusive(method_root / "metrics.json", metrics)
        if method != "M0_RAW":
            optimization = {
                **payload["result"].optimization_dict(),
                "active_factors": list(METHOD_FACTORS[method]),
                "solver": "frozen solve_least_squares",
                "right_local_retraction": True,
            }
            save_json_exclusive(method_root / "optimization.json", optimization)
        method_hashes[method] = {
            "candidate_sha256": candidate_hash,
            "metrics_sha256": sha256_file(method_root / "metrics.json"),
            "optimization_sha256": (
                None if method == "M0_RAW" else sha256_file(method_root / "optimization.json")
            ),
        }
    result = {
        "artifact_relative_path": str(destination.relative_to(output)),
        "input_reference_sha256": sha256_file(destination / "input_reference.json"),
        "oracle_indices_sha256": sha256_file(destination / "oracle_indices.json"),
        "methods": method_hashes,
        "analysis": dict(row),
    }
    return result


def csv_row(row: Mapping[str, Any]) -> dict[str, Any]:
    fields = (
        "corpus_transition_id", "cohort_id", "episode_id", "transition_index",
        "template_id", "variant_id", "semantic_family", "physical_region",
        "ordered_raw_pair_sha256",
        "ordered_raw_pair_frequency", "fresh_geometry_bin", "raw_difficulty_bin",
        "k_fresh", "q_fresh", "q_minus_k", "alpha_entry_rad", "alpha_look_rad",
        "alpha_entry_minus_alpha_look_rad", "d_k_m", "d_q_m",
        "fresh_arc_k_to_q_m", "observation_to_boundary_translation_m",
        "observation_to_boundary_yaw_rad", "p_to_b_translation_m", "p_to_b_yaw_rad",
        "fresh_net_yaw_rad", "fresh_lateral_excursion_m", "host_latency_s",
        "effective_latency_s", "inference_translation_m", "geometry_undefined", "undefined_statuses",
        "solver_failure", "solver_failures", "success_failure_label",
        "m4_false_correction_rescued",
    )
    result = {field: row[field] for field in fields}
    result["undefined_statuses"] = json.dumps(row["undefined_statuses"], sort_keys=True)
    result["solver_failures"] = json.dumps(row["solver_failures"], sort_keys=True)
    result["regimes"] = ";".join(row["regimes"])
    for method, short in METHOD_SHORT.items():
        for key in (
            "delta_v_signed_mps", "delta_omega_signed_rps", "delta_v_abs_mps",
            "delta_omega_abs_rps", "difficulty_bin", "candidate_nearest_index",
            "candidate_target_index", "candidate_target_changed_from_raw_q",
            *DEFORMATION_FIELDS,
        ):
            result[f"{method}_{key}"] = row[f"{short}_{key}"]
        result[f"{method}_J_cmd"] = row[f"j_cmd_{short}"]
    return result


def build_summaries(
    output: Path,
    evaluations: Sequence[Mapping[str, Any]],
    rows: Sequence[dict[str, Any]],
    transition_artifacts: Mapping[str, Any],
    config: Mapping[str, Any],
) -> dict[str, Any]:
    valid = [row for row in rows if not row["geometry_undefined"] and not row["solver_failure"]]
    partitions = {
        "ALL_VALID_TRANSITIONS": valid,
        "RAW_BENIGN": [row for row in valid if row["raw_difficulty_bin"] == "BENIGN"],
        "RAW_INTERMEDIATE": [row for row in valid if row["raw_difficulty_bin"] == "INTERMEDIATE"],
        "RAW_CHALLENGING": [row for row in valid if row["raw_difficulty_bin"] == "CHALLENGING"],
    }
    aggregate = {
        name: {
            "count": len(partition),
            "methods": {
                method: method_table(partition, METHOD_SHORT[method])
                for method in METHODS
            },
        }
        for name, partition in partitions.items()
    }
    save_json_exclusive(output / "summary/aggregate.json", aggregate)
    pair_balanced = {
        "ordered_raw_pair_group_count": len(ordered_pair_groups(valid)),
        "primary_weighting": "mean within ordered raw pair, then mean across pairs",
        "methods": {
            method: {
                "mean_J_cmd": pair_balanced_mean(valid, f"j_cmd_{METHOD_SHORT[method]}"),
                "mean_abs_delta_v_mps": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_delta_v_abs_mps"),
                "mean_abs_delta_omega_rps": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_delta_omega_abs_rps"),
                "transition_weighted_mean_J_cmd": transition_weighted_mean(valid, f"j_cmd_{METHOD_SHORT[method]}"),
                "mean_translation_deformation_rms_m": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_translation_deformation_rms_m"),
                "mean_yaw_deformation_rms_rad": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_yaw_deformation_rms_rad"),
                "mean_rigid_fit_translation_rms_m": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_rigid_fit_translation_rms_m"),
                "mean_rigid_fit_yaw_rms_rad": pair_balanced_mean(valid, f"{METHOD_SHORT[method]}_rigid_fit_yaw_rms_rad"),
                "transition_weighted_mean_translation_deformation_rms_m": transition_weighted_mean(
                    valid, f"{METHOD_SHORT[method]}_translation_deformation_rms_m"
                ),
                "transition_weighted_mean_yaw_deformation_rms_rad": transition_weighted_mean(
                    valid, f"{METHOD_SHORT[method]}_yaw_deformation_rms_rad"
                ),
                "transition_weighted_mean_rigid_fit_translation_rms_m": transition_weighted_mean(
                    valid, f"{METHOD_SHORT[method]}_rigid_fit_translation_rms_m"
                ),
                "transition_weighted_mean_rigid_fit_yaw_rms_rad": transition_weighted_mean(
                    valid, f"{METHOD_SHORT[method]}_rigid_fit_yaw_rms_rad"
                ),
                "candidate_target_changed_count": sum(
                    bool(row[f"{METHOD_SHORT[method]}_candidate_target_changed_from_raw_q"])
                    for row in valid
                ),
                "candidate_target_changed_fraction": float(
                    np.mean(
                        [
                            bool(row[f"{METHOD_SHORT[method]}_candidate_target_changed_from_raw_q"])
                            for row in valid
                        ]
                    )
                ),
                "candidate_target_changed_pair_balanced_fraction": pair_balanced_mean(
                    valid,
                    lambda row, short=METHOD_SHORT[method]: float(
                        bool(row[f"{short}_candidate_target_changed_from_raw_q"])
                    ),
                ),
            }
            for method in METHODS
        },
        "bootstrap_differences": {},
        "bootstrap_differences_by_partition": {},
    }
    bootstrap = config["evaluation"]
    for right in ("raw", "m1", "m2"):
        pair_balanced["bootstrap_differences"][f"M3_minus_{right.upper()}"] = (
            cluster_bootstrap_pair_balanced_difference(
                valid,
                "j_cmd_m3",
                f"j_cmd_{right}",
                seed=int(bootstrap["bootstrap_seed"]),
                repetitions=int(bootstrap["bootstrap_repetitions"]),
                confidence=float(bootstrap["confidence_level"]),
            )
        )
    for partition_name, partition in partitions.items():
        pair_balanced["bootstrap_differences_by_partition"][partition_name] = {}
        for right in ("raw", "m1", "m2"):
            key = f"M3_minus_{right.upper()}"
            pair_balanced["bootstrap_differences_by_partition"][partition_name][key] = (
                None
                if not partition
                else cluster_bootstrap_pair_balanced_difference(
                    partition,
                    "j_cmd_m3",
                    f"j_cmd_{right}",
                    seed=int(bootstrap["bootstrap_seed"]),
                    repetitions=int(bootstrap["bootstrap_repetitions"]),
                    confidence=float(bootstrap["confidence_level"]),
                )
            )
    save_json_exclusive(output / "summary/pair_balanced.json", pair_balanced)

    labels = Counter(str(row["success_failure_label"]) for row in rows)
    benign = partitions["RAW_BENIGN"]
    challenging = partitions["RAW_CHALLENGING"]
    rescued = [row for row in valid if bool(row["m4_false_correction_rescued"])]
    success = {
        "counts": {
            label: labels.get(label, 0) for label in SUCCESS_FAILURE_LABELS
        },
        "other_counts": {
            label: count
            for label, count in sorted(labels.items())
            if label not in SUCCESS_FAILURE_LABELS
        },
        "raw_benign_count": len(benign),
        "benign_broken_by_method": {
            method: sum(row[f"{METHOD_SHORT[method]}_difficulty_bin"] != "BENIGN" for row in benign)
            for method in METHODS
        },
        "raw_challenging_count": len(challenging),
        "m3_challenging_outcomes": {
            name: labels.get(name, 0)
            for name in (
                "CHALLENGING_RESCUED", "CHALLENGING_IMPROVED",
                "CHALLENGING_MIXED", "CHALLENGING_WORSE",
            )
        },
        "M4_FALSE_CORRECTION_RESCUED": {
            "count": len(rescued),
            "all_valid_denominator": len(valid),
            "fraction_of_all_valid": (
                None if not valid else len(rescued) / len(valid)
            ),
            "raw_pair_balanced_fraction_all_valid": (
                None
                if not valid
                else pair_balanced_mean(
                    valid,
                    lambda row: float(bool(row["m4_false_correction_rescued"])),
                )
            ),
            "raw_benign_denominator": len(benign),
            "fraction_of_raw_benign": (
                None if not benign else len(rescued) / len(benign)
            ),
            "raw_pair_balanced_fraction_within_raw_benign": (
                None
                if not benign
                else pair_balanced_mean(
                    benign,
                    lambda row: float(bool(row["m4_false_correction_rescued"])),
                )
            ),
            "feature_distributions": feature_summaries(rescued),
        },
        "geometry_undefined_count": sum(bool(row["geometry_undefined"]) for row in rows),
        "geometry_undefined_transition_ids": [
            row["corpus_transition_id"] for row in rows if row["geometry_undefined"]
        ],
        "solver_failure_count": sum(bool(row["solver_failure"]) for row in rows),
        "solver_failure_transition_ids": [
            row["corpus_transition_id"] for row in rows if row["solver_failure"]
        ],
    }
    save_json_exclusive(output / "summary/success_failure_counts.json", success)

    categories = {
        label: [row for row in valid if row["success_failure_label"] == label]
        for label in sorted({row["success_failure_label"] for row in valid})
    }
    regimes = {
        f"R{index}": [row for row in valid if any(value.startswith(f"R{index}_") for value in row["regimes"])]
        for index in range(1, 7)
    }
    regime_summary = {
        "thresholds": dict(config["regimes"]),
        "success_failure_categories": {
            name: {"count": len(group), "features": feature_summaries(group)}
            for name, group in categories.items()
        },
        "regimes": {
            name: {
                "count": len(group),
                "transition_ids": [row["corpus_transition_id"] for row in group],
                "features": feature_summaries(group),
            }
            for name, group in regimes.items()
        },
        "geometry_hypothesis": hypothesis_analysis(valid, config),
        "interpretation": "descriptive associations; no causal claim",
    }
    save_json_exclusive(output / "summary/regime_analysis.json", regime_summary)

    ids = select_representative_ids(valid)
    lookup = {row["corpus_transition_id"]: row for row in valid}
    representative_rules = config["representatives"]
    representatives: dict[str, Any] = {}
    for case, identifier in ids.items():
        if identifier.endswith("_NOT_AVAILABLE"):
            representatives[case] = {
                "status": identifier,
                "corpus_transition_id": None,
                "selection_rule": representative_rules[case],
                "criteria_relaxed": False,
            }
            continue
        row = lookup[identifier]
        representatives[case] = {
            "status": "AVAILABLE",
            "corpus_transition_id": identifier,
            "selection_rule": representative_rules[case],
            "criteria_relaxed": False,
            "artifact_relative_path": transition_artifacts[identifier]["artifact_relative_path"],
            "success_failure_label": row["success_failure_label"],
            "fresh_geometry_bin": row["fresh_geometry_bin"],
            "metrics": {
                key: row[key]
                for key in (
                    "j_cmd_raw", "j_cmd_m1", "j_cmd_m2", "j_cmd_m3",
                    "raw_delta_v_abs_mps", "raw_delta_omega_abs_rps",
                    "m1_delta_v_abs_mps", "m1_delta_omega_abs_rps",
                    "m3_delta_v_abs_mps", "m3_delta_omega_abs_rps",
                    "alpha_entry_rad", "alpha_look_rad",
                    "b_to_fresh_k_translation_m", "b_to_fresh_q_translation_m",
                    "k_fresh", "q_fresh",
                )
            },
        }
    save_json_exclusive(
        output / "summary/representatives.json",
        {
            "selection_after_complete_primary": True,
            "identity_tie_break": representative_rules["identity_tie_break"],
            "representatives": representatives,
        },
    )
    save_json_exclusive(
        output / "summary/plot_records.json",
        {
            "records": [
                plot_record(evaluation, row)
                for evaluation, row in zip(evaluations, rows, strict=True)
                if not row["geometry_undefined"] and not row["solver_failure"]
            ]
        },
    )
    return {
        "aggregate": aggregate,
        "pair_balanced": pair_balanced,
        "success_failure": success,
        "regime_analysis": regime_summary,
        "representatives": representatives,
    }


def dry_validation(
    output: Path,
    corpus: Any,
    config: Mapping[str, Any],
    source_hashes: Mapping[str, str],
    oracle_summary: Mapping[str, Any],
) -> None:
    undefined: list[str] = []
    problems = []
    for source in corpus.transitions:
        problem = transition_problem(source, config)
        status = undefined_geometry_statuses(problem)
        if status:
            undefined.append(source.corpus_transition_id)
        problems.append((source, status))
    criteria: tuple[tuple[str, Callable[[Any], bool]], ...] = (
        ("BENIGN", lambda value: value.difficulty_bin == "BENIGN"),
        ("CHALLENGING", lambda value: value.difficulty_bin == "CHALLENGING"),
        ("POSITIVE_TURNING", lambda value: value.fresh_geometry_bin == "POSITIVE_TURNING"),
        ("NEGATIVE_TURNING", lambda value: value.fresh_geometry_bin == "NEGATIVE_TURNING"),
    )
    selected = []
    for label, predicate in criteria:
        matches = [source for source, status in problems if not status and predicate(source)]
        if not matches:
            raise RuntimeError(f"dry validation has no {label} transition")
        selected.append((label, min(matches, key=lambda value: value.corpus_transition_id)))
    rows = []
    for label, source in selected:
        evaluation = evaluate_transition(source, config)
        if evaluation["undefined"] or evaluation["solver_failures"] or len(evaluation["methods"]) != 4:
            raise RuntimeError(f"dry validation failed: {label}/{source.corpus_transition_id}")
        rows.append(
            {
                "validation_class": label,
                "corpus_transition_id": source.corpus_transition_id,
                "fresh_geometry_bin": source.fresh_geometry_bin,
                "raw_difficulty_bin": source.difficulty_bin,
                "k": source.k_fresh,
                "q": source.q_fresh,
                "methods": {
                    method: {
                        "J_cmd": evaluation["methods"][method]["command"]["J_cmd"],
                        "converged": method == "M0_RAW" or evaluation["methods"][method]["result"].optimization.converged,
                    }
                    for method in METHODS
                },
            }
        )
    output.mkdir(parents=True, exist_ok=False)
    write_yaml_exclusive(output / "config_snapshot.yaml", config)
    save_json_exclusive(
        output / "dry_validation.json",
        {
            "schema": "EXP02D_DryValidation_v1",
            "eligible_count": corpus.index.eligible_count,
            "reconstruction_valid_count": corpus.valid_count,
            "input_reconstruction_failures": [asdict(value) for value in corpus.failures],
            "undefined_geometry_count": len(undefined),
            "undefined_geometry_transition_ids": undefined,
            "source_sha256": dict(source_hashes),
            "oracle_follower_reconstruction": dict(oracle_summary),
            "selected_technical_cases": rows,
            "parameter_tuning_performed": False,
            "scientific_evidence": False,
        },
    )
    print(f"EXP02D_DRY_ELIGIBLE={corpus.index.eligible_count}")
    print(f"EXP02D_DRY_RECONSTRUCTION_VALID={corpus.valid_count}")
    print(f"EXP02D_DRY_UNDEFINED={len(undefined)}")
    print(f"EXP02D_DRY_OUTPUT={output}")


def primary(
    output: Path,
    corpus: Any,
    config: Mapping[str, Any],
    config_path: Path,
    source_repository: Path,
    source_hashes: Mapping[str, str],
    git_sha: str,
    git_status: str,
    oracle_summary: Mapping[str, Any],
) -> None:
    corpus.require_complete()
    # Geometry is screened before creating an output root; undefined rows remain
    # legitimate in the manifest but cannot create a partial silent objective.
    geometry_status = {
        source.corpus_transition_id: list(
            undefined_geometry_statuses(transition_problem(source, config))
        )
        for source in corpus.transitions
    }
    output.mkdir(parents=True, exist_ok=False)
    write_yaml_exclusive(output / "config_snapshot.yaml", config)
    save_json_exclusive(
        output / "provenance.json",
        {
            "schema": "EXP02D_Provenance_v1",
            "exp02d_protocol_git_sha": git_sha,
            "git_status_at_primary_start": git_status,
            "primary_started_from_clean_protocol_worktree": git_status == "",
            "config_source": str(config_path),
            "config_snapshot_sha256": sha256_file(output / "config_snapshot.yaml"),
            "source_repository_root": str(source_repository),
            "immutable_source_sha256": dict(source_hashes),
            "combined_manifest_sha256": corpus.index.combined_manifest_sha256,
            "data02_combined_decision": "DATA02_COMBINED_DIVERSITY_INSUFFICIENT",
            "corpus_role": "EXP02D_DEVELOPMENT_CORPUS",
        },
    )
    save_json_exclusive(
        output / "summary/oracle_follower_reconstruction.json",
        dict(oracle_summary),
    )
    pair_counts = Counter(
        source.ordered_raw_pair_sha256 for source in corpus.transitions
    )
    evaluations = []
    rows: list[dict[str, Any]] = []
    artifacts: dict[str, Any] = {}
    progress_interval = int(config["execution"]["progress_interval"])
    threshold = regime_thresholds(config)
    for index, source in enumerate(corpus.transitions, start=1):
        evaluation = evaluate_transition(source, config)
        row = analysis_row(evaluation, pair_counts[source.ordered_raw_pair_sha256])
        if not row["geometry_undefined"] and not row["solver_failure"]:
            row["regimes"] = list(regime_memberships(row, threshold))
        evaluations.append(evaluation)
        rows.append(row)
        if not row["geometry_undefined"] and not row["solver_failure"]:
            artifacts[source.corpus_transition_id] = save_transition_result(output, evaluation, row)
        if index % progress_interval == 0 or index == len(corpus.transitions):
            print(
                f"EXP02D_PROGRESS={index}/{len(corpus.transitions)} "
                f"undefined={sum(bool(value['geometry_undefined']) for value in rows)} "
                f"solver_failure={sum(bool(value['solver_failure']) for value in rows)}",
                flush=True,
            )
    valid_count = sum(not row["geometry_undefined"] and not row["solver_failure"] for row in rows)
    if valid_count == 0:
        raise RuntimeError("EXP02D_TECHNICAL_INVALID: no solver-valid transitions")
    write_csv_exclusive(output / "transition_summary.csv", [csv_row(row) for row in rows])
    save_json_exclusive(
        output / "corpus_manifest.json",
        {
            "schema": "EXP02D_DevelopmentCorpusManifest_v1",
            "corpus_role": "EXP02D_DEVELOPMENT_CORPUS",
            "final_test_set": False,
            "eligible_moving_count": corpus.index.eligible_count,
            "reconstruction_valid_count": corpus.valid_count,
            "input_reconstruction_failures": [asdict(value) for value in corpus.failures],
            "undefined_geometry": {
                "count": sum(bool(value) for value in geometry_status.values()),
                "transitions": {key: value for key, value in geometry_status.items() if value},
            },
            "ordered_raw_pair_count": len(pair_counts),
            "ordered_raw_pair_frequencies": dict(sorted(pair_counts.items())),
            "oracle_follower_reconstruction": dict(oracle_summary),
            "solver_failures": {
                "count": sum(bool(row["solver_failure"]) for row in rows),
                "transitions": {
                    row["corpus_transition_id"]: row["solver_failures"]
                    for row in rows
                    if row["solver_failure"]
                },
            },
            "transition_artifacts": artifacts,
        },
    )
    summaries = build_summaries(output, evaluations, rows, artifacts, config)
    subprocess.run(
        [sys.executable, str(CODE_ROOT / "scripts/plot_exp02d_lookahead_direction.py"), str(output)],
        check=True,
    )
    save_json_exclusive(
        output / "metadata.json",
        {
            "schema": "EXP02D_Run_v1",
            "experiment": "EXP-02D",
            "run_id": output.name,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "technical_status": "EXP02D_RUN_COMPLETE",
            "exp02d_protocol_git_sha": git_sha,
            "eligible_moving_count": corpus.index.eligible_count,
            "reconstruction_valid_count": corpus.valid_count,
            "geometry_undefined_count": sum(bool(row["geometry_undefined"]) for row in rows),
            "solver_valid_count": valid_count,
            "solver_failure_count": sum(bool(row["solver_failure"]) for row in rows),
            "method_order": list(METHODS),
            "candidate_methods_physically_executed": False,
            "command_score_in_objective": False,
            "intrinsic_waypoint_time_base": False,
            "pair_balanced_primary": True,
            "transition_weighted_secondary": True,
            "representatives_selected_after_complete_primary": True,
            "summary_files": [
                "summary/aggregate.json", "summary/pair_balanced.json",
                "summary/success_failure_counts.json", "summary/regime_analysis.json",
                "summary/representatives.json",
                "summary/oracle_follower_reconstruction.json",
            ],
            "representatives": summaries["representatives"],
        },
    )
    files = sorted(path for path in output.rglob("*") if path.is_file())
    manifest = {
        "schema": "EXP02D_ResultManifest_v1",
        "file_count_excluding_manifest": len(files),
        "artifact_sha256": {
            str(path.relative_to(output)): sha256_file(path) for path in files
        },
    }
    save_json_exclusive(output / "result_manifest.json", manifest)
    print(f"EXP02D_TECHNICAL_STATUS=EXP02D_RUN_COMPLETE", flush=True)
    print(f"EXP02D_OUTPUT={output}", flush=True)


def main() -> None:
    args = arguments()
    config_path = args.config.expanduser().resolve()
    config = strict_yaml(config_path)
    validate_protocol(config)
    sha = git("rev-parse", "HEAD")
    status = git("status", "--porcelain")
    if args.require_git_sha is not None and sha != args.require_git_sha:
        raise RuntimeError(f"protocol SHA mismatch: {sha} != {args.require_git_sha}")
    if args.phase == "primary" and status:
        raise RuntimeError("EXP-02D primary requires a clean protocol worktree")
    source_repository = args.source_repository_root.expanduser().resolve()
    source_hashes = validate_source_integrity(config, source_repository)
    combined = resolve_under(source_repository, config["paths"]["combined_corpus"])
    corpus = load_exp02d_development_corpus(
        source_repository,
        combined,
        command_atol=float(config["oracle"]["first_command_absolute_tolerance"]),
    )
    if args.phase == "primary":
        # Surface reconstruction failure before any downstream corpus statistic
        # or output-directory mutation can mask the required technical status.
        corpus.require_complete()
    oracle_summary = validate_m0_follower_reconstruction(corpus, config)
    output_root = (
        args.output_root.expanduser().resolve()
        if args.output_root is not None
        else resolve_under(source_repository, config["paths"]["output_root"])
    )
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / args.run_id
    if output.exists():
        raise FileExistsError(f"refusing to overwrite EXP-02D output: {output}")
    if args.phase == "dry-validation":
        dry_validation(output, corpus, config, source_hashes, oracle_summary)
    else:
        primary(
            output, corpus, config, config_path, source_repository,
            source_hashes, sha, status, oracle_summary,
        )


if __name__ == "__main__":
    main()
