#!/usr/bin/env python3
"""Run offline EXP-02C factor isolation on frozen real and synthetic inputs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping

import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.exp02b import (  # noqa: E402
    CASE_IDS,
    load_source_case,
    transition_input,
    validate_exp02b_config,
    verify_frozen_source_selection,
)
from reconciliation.exp02c_factor_isolation import (  # noqa: E402
    FACTOR_ORDER,
    VARIANT_ACTIVE_FACTORS,
    best_fit_left_se2,
    controller_desired_metrics,
    factor_residual_vector,
    geometry_metrics,
    gradient_cosines,
    gradient_diagnostics,
    problem_for_variant,
    residual_diagnostics,
    solve_variant,
    synthetic_conditions,
)
from reconciliation.graph_optimizer import SolverConfig  # noqa: E402
from reconciliation.online_switch import (  # noqa: E402
    load_strict_json,
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
)
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput  # noqa: E402
from reconciliation.transition_graph import (  # noqa: E402
    problem_from_config,
    transition_residual_vector,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02c_factor_isolation.yaml",
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def validate_config(config: Mapping[str, Any], graph: Mapping[str, Any]) -> None:
    if config.get("experiment") != "EXP-02C":
        raise ValueError("configuration is not EXP-02C")
    if tuple(config.get("cases", ())) != CASE_IDS:
        raise ValueError(f"real cases must remain {CASE_IDS}")
    if tuple(config.get("entry_indices", ())) != (0, 3, 6):
        raise ValueError("entry indices must remain (0, 3, 6)")
    if tuple(config.get("variants", ())) != tuple(VARIANT_ACTIVE_FACTORS):
        raise ValueError("factor variants or order changed")
    if config["residual_scales"] != graph["residual_scales"]:
        raise ValueError("EXP-02C residual scales differ from frozen EXP-02A")
    expected = graph["variants"]["incoming_motion_aware"]
    weights = config["historical_weights"]
    for name in FACTOR_ORDER:
        if float(weights[name]) != float(expected[name]):
            raise ValueError(f"historical weight changed: {name}")
    if float(config["minimum_direction_translation_m"]) != float(
        graph["minimum_direction_translation_m"]
    ):
        raise ValueError("minimum direction translation changed")
    if config["solver"] != graph["solver"]:
        raise ValueError("EXP-02C solver differs from frozen EXP-02A")


def write_yaml_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(dict(value), stream, sort_keys=False)


def synthetic_input(name: str, values: Mapping[str, Any]) -> TransitionReconciliationInput:
    return TransitionReconciliationInput(
        old_poses_world=values["old"],
        fresh_poses_world=values["fresh"],
        committed_pose_world=values["boundary"],
        actual_pose_before_committed=np.asarray(values["old"])[-1],
        entry_context=SpatialEntryContext(
            entry_index=0,
            source="exp02c_synthetic_mechanism_fixture",
            metadata={"case": name, "experimental_evidence": False},
        ),
        metadata={"synthetic": True, "purpose": values["purpose"]},
    )


def patch_activity(payload: dict[str, Any], active: tuple[str, ...]) -> dict[str, Any]:
    for name, item in payload.items():
        item["active"] = name in active
        item["objective_weighted_cost"] = (
            item["diagnostic_weighted_cost"] if name in active else 0.0
        )
    return payload


def cost_payload(residuals: Mapping[str, Any]) -> dict[str, Any]:
    return {
        name: {
            "active": item["active"],
            "physical_norm": item["physical_norm"],
            "normalized_norm": item["normalized_norm"],
            "diagnostic_weighted_cost": item["diagnostic_weighted_cost"],
            "objective_weighted_cost": item["objective_weighted_cost"],
        }
        for name, item in residuals.items()
    }


def compact_metrics(row: Mapping[str, Any]) -> dict[str, float]:
    controller = row["controller"]
    geometry = row["geometry"]
    rigid = row["rigid"]
    return {
        "delta_v_des_abs_mps": controller["delta_v_des_abs_mps"],
        "delta_omega_des_abs_rps": controller["delta_omega_des_abs_rps"],
        "entry_displacement_m": geometry["entry"]["translation_displacement_m"],
        "endpoint_displacement_m": geometry["endpoint"]["translation_displacement_m"],
        "fresh_edge_translation_rms_m": geometry["fresh_relative_motion"]["translation_rms_m"],
        "fresh_edge_yaw_rms_rad": geometry["fresh_relative_motion"]["yaw_rms_rad"],
        "rigid_fit_translation_rms_m": rigid["translation_rms_m"],
        "rigid_fit_yaw_rms_rad": rigid["yaw_rms_rad"],
    }


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = load_yaml(config_path)
    graph_path = resolve(config["paths"]["exp02a_graph_config"])
    graph = load_yaml(graph_path)
    validate_config(config, graph)
    exp02b_config_path = resolve(config["paths"]["exp02b_config"])
    exp02b_config = load_yaml(exp02b_config_path)
    validate_exp02b_config(exp02b_config)
    source_root = resolve(exp02b_config["paths"]["source_root"])
    selected = verify_frozen_source_selection(source_root, exp02b_config)
    historical = resolve(config["paths"]["frozen_exp02b_run"])
    candidate_manifest_path = resolve(config["paths"]["frozen_candidate_manifest"])
    candidate_manifest = load_yaml(candidate_manifest_path)
    if candidate_manifest.get("experiment") != "EXP-02B-R":
        raise ValueError("frozen candidate manifest is not EXP-02B-R")
    for filename, key in (
        ("source_selection.json", "source_selection_sha256"),
        ("protocol.json", "protocol_sha256"),
        ("offline_summary.json", "offline_summary_sha256"),
    ):
        actual = sha256_file(historical / filename)
        expected = candidate_manifest["historical_provenance"][key]
        if actual != expected:
            raise ValueError(f"frozen EXP-02B provenance mismatch: {filename}")
    exp01b_config_path = resolve(config["paths"]["exp01b_config"])
    exp01b_config = load_yaml(exp01b_config_path)
    solver = SolverConfig(**config["solver"])

    real_inputs: dict[tuple[str, int], tuple[TransitionReconciliationInput, Any]] = {}
    for case in CASE_IDS:
        source = load_source_case(case, selected[case]["artifact_path"])
        for k in config["entry_indices"]:
            real_inputs[(case, int(k))] = (transition_input(source, int(k)), source)
    synthetic_values = synthetic_conditions()
    synthetic_inputs = {
        name: synthetic_input(name, values) for name, values in synthetic_values.items()
    }

    # Preflight every V4 before creating output. A mismatch leaves no partial run.
    regression_rows = []
    precomputed_v4: dict[tuple[str, int], Any] = {}
    tolerance = float(config["diagnostics"]["full_m4_regression_tolerance"])
    for (case, k), (inputs, _) in real_inputs.items():
        diagnostic = problem_for_variant(
            inputs,
            "V4_FULL_CURRENT_M4",
            residual_scales=config["residual_scales"],
            weights=config["historical_weights"],
            minimum_translation_m=float(config["minimum_direction_translation_m"]),
        )
        assert diagnostic is not None
        production = problem_from_config(
            inputs,
            "incoming_motion_aware",
            residual_scales=graph["residual_scales"],
            weights=graph["variants"]["incoming_motion_aware"],
            minimum_translation_m=float(graph["minimum_direction_translation_m"]),
        )
        residual_difference = float(
            np.max(
                np.abs(
                    factor_residual_vector(diagnostic, diagnostic.raw)
                    - transition_residual_vector(production, production.raw_new)
                )
            )
        )
        result = solve_variant(diagnostic, diagnostic.raw, solver)
        frozen_path = historical / case / f"k_{k}" / "graph/candidate.npy"
        expected_frozen_hash = candidate_manifest["candidate_sha256"][case][f"k_{k}"]["graph"]
        actual_frozen_hash = sha256_file(frozen_path)
        if actual_frozen_hash != expected_frozen_hash:
            raise RuntimeError(
                f"FULL_M4_REGRESSION_FAILURE: frozen candidate hash mismatch {case}/k{k}"
            )
        frozen = np.load(frozen_path, allow_pickle=False)
        translation = np.linalg.norm(result.optimized[:, :2] - frozen[:, :2], axis=1)
        yaw = np.abs(np.asarray(wrap_angles(result.optimized[:, 2] - frozen[:, 2])))
        row = {
            "case_id": case,
            "entry_index": k,
            "frozen_candidate_path": str(frozen_path),
            "frozen_candidate_sha256": actual_frozen_hash,
            "raw_residual_vector_max_abs_difference": residual_difference,
            "max_translation_difference_m": float(np.max(translation)),
            "max_yaw_difference_rad": float(np.max(yaw)),
            "tolerance": tolerance,
        }
        row["passed"] = bool(
            residual_difference <= tolerance
            and row["max_translation_difference_m"] <= tolerance
            and row["max_yaw_difference_rad"] <= tolerance
        )
        if not row["passed"]:
            raise RuntimeError("FULL_M4_REGRESSION_FAILURE: " + json.dumps(row, sort_keys=True))
        regression_rows.append(row)
        precomputed_v4[(case, k)] = result

    run_id = args.run_id or datetime.now(timezone.utc).strftime("exp02c-%Y%m%dT%H%M%SZ")
    output = resolve(config["paths"]["output_root"]) / run_id
    output.mkdir(parents=True, exist_ok=False)
    write_yaml_exclusive(output / "config_snapshot.yaml", config)

    source_cases_payload = {}
    for case in CASE_IDS:
        trial = Path(selected[case]["artifact_path"]).resolve()
        source_cases_payload[case] = {
            "trial_directory": str(trial),
            "source_file_sha256": {
                relative: sha256_file(trial / relative)
                for relative in exp02b_config["frozen_source_cases"][case]["sha256"]
            },
        }
    provenance = {
        "frozen_exp02b_run": str(historical),
        "frozen_top_level_sha256": {
            name: sha256_file(historical / name)
            for name in ("source_selection.json", "protocol.json", "offline_summary.json")
        },
        "exp02a_graph_config": str(graph_path),
        "exp02a_graph_config_sha256": sha256_file(graph_path),
        "exp02b_config": str(exp02b_config_path),
        "exp02b_config_sha256": sha256_file(exp02b_config_path),
        "frozen_candidate_manifest": str(candidate_manifest_path),
        "frozen_candidate_manifest_sha256": sha256_file(candidate_manifest_path),
        "exp01b_config": str(exp01b_config_path),
        "exp01b_config_sha256": sha256_file(exp01b_config_path),
        "real_cases": source_cases_payload,
        "full_m4_regression": regression_rows,
    }
    save_json_exclusive(output / "source_provenance.json", provenance)

    all_rows: list[dict[str, Any]] = []

    def execute_condition(
        *,
        kind: str,
        condition: str,
        inputs: TransitionReconciliationInput,
        root: Path,
        old_final_command: np.ndarray,
        entry_index: int,
    ) -> None:
        raw = inputs.selected_suffix
        raw_copy = raw.copy()
        full_problem = problem_for_variant(
            inputs,
            "V4_FULL_CURRENT_M4",
            residual_scales=config["residual_scales"],
            weights=config["historical_weights"],
            minimum_translation_m=float(config["minimum_direction_translation_m"]),
        )
        assert full_problem is not None
        cache: dict[str, Any] = {}
        for variant in config["variants"]:
            alias = (
                "V3_ENTRY_YAW_FRESH"
                if variant == "V6_NO_DIRECTION"
                else "V2_ENTRY_DIRECTION_FRESH"
                if variant == "V7_NO_YAW"
                else None
            )
            problem = problem_for_variant(
                inputs,
                variant,
                residual_scales=config["residual_scales"],
                weights=config["historical_weights"],
                minimum_translation_m=float(config["minimum_direction_translation_m"]),
            )
            if variant == "V4_FULL_CURRENT_M4" and kind == "real":
                result = precomputed_v4[(condition, entry_index)]
            elif alias is not None:
                result = cache[alias]
            else:
                result = solve_variant(problem, raw, solver)
            cache[variant] = result
            if not result.converged:
                raise RuntimeError(f"{kind}/{condition}/k{entry_index}/{variant} did not converge")
            active = VARIANT_ACTIVE_FACTORS[variant]
            diagnostic_problem = full_problem if problem is None else problem
            initial_residuals = patch_activity(
                residual_diagnostics(diagnostic_problem, raw), active
            )
            final_residuals = patch_activity(
                residual_diagnostics(diagnostic_problem, result.optimized), active
            )
            initial_gradients, initial_gradient_json = gradient_diagnostics(
                diagnostic_problem, raw, epsilon=solver.finite_difference_epsilon
            )
            final_gradients, final_gradient_json = gradient_diagnostics(
                diagnostic_problem,
                result.optimized,
                epsilon=solver.finite_difference_epsilon,
            )
            for payload in (initial_gradient_json, final_gradient_json):
                for name in FACTOR_ORDER:
                    payload["factors"][name]["active"] = name in active
            cosine = gradient_cosines(
                initial_gradients,
                active_factors=active,
                zero_norm_tolerance=float(
                    config["diagnostics"]["cosine_zero_norm_tolerance"]
                ),
            )
            final_cosine = gradient_cosines(
                final_gradients,
                active_factors=active,
                zero_norm_tolerance=float(
                    config["diagnostics"]["cosine_zero_norm_tolerance"]
                ),
            )
            geometry = geometry_metrics(
                inputs,
                result.optimized,
                minimum_translation_m=float(config["minimum_direction_translation_m"]),
            )
            rigid = best_fit_left_se2(raw, result.optimized)
            controller = controller_desired_metrics(
                result.optimized,
                boundary=inputs.committed_pose_world,
                old_final_command=old_final_command,
                follower_values=exp01b_config["closed_loop"],
                window_size=int(config["diagnostics"]["controller_command_window"]),
            )
            variant_root = root / variant
            variant_root.mkdir(parents=True, exist_ok=False)
            save_npy_exclusive(variant_root / "raw_fresh.npy", raw)
            save_npy_exclusive(variant_root / "optimized.npy", result.optimized)
            save_json_exclusive(variant_root / "factor_residuals_initial.json", initial_residuals)
            save_json_exclusive(variant_root / "factor_residuals_final.json", final_residuals)
            save_json_exclusive(variant_root / "factor_costs_initial.json", cost_payload(initial_residuals))
            save_json_exclusive(variant_root / "factor_costs_final.json", cost_payload(final_residuals))
            save_npy_exclusive(variant_root / "factor_gradients_initial.npy", initial_gradients)
            save_json_exclusive(variant_root / "factor_gradients_initial.json", initial_gradient_json)
            save_npy_exclusive(variant_root / "factor_gradients_final.npy", final_gradients)
            save_json_exclusive(variant_root / "factor_gradients_final.json", final_gradient_json)
            save_json_exclusive(variant_root / "gradient_cosine_initial.json", cosine)
            save_json_exclusive(variant_root / "gradient_cosine_final.json", final_cosine)
            optimization = {
                **result.to_dict(),
                "residual_vector_size": (
                    0 if problem is None else int(factor_residual_vector(problem, raw).size)
                ),
                "active_factors": list(active),
                "diagnostic_no_propagation": variant == "V8_DIAGNOSTIC_NO_PROPAGATION",
                "reused_result_from": alias,
            }
            save_json_exclusive(variant_root / "optimization_history.json", optimization)
            save_json_exclusive(variant_root / "geometry_metrics.json", geometry)
            save_json_exclusive(variant_root / "controller_desired_metrics.json", controller)
            save_json_exclusive(variant_root / "rigid_fit_metrics.json", rigid)
            metadata = {
                "experiment": "EXP-02C",
                "diagnostic_only": True,
                "condition_kind": kind,
                "condition": condition,
                "entry_index": entry_index,
                "variant": variant,
                "active_factors": list(active),
                "weights_renormalized": False,
                "controller_residual_in_objective": False,
                "intrinsic_waypoint_time_base": False,
                "production_transition_graph_modified": False,
                "raw_input_unchanged": bool(np.array_equal(raw, raw_copy)),
            }
            save_json_exclusive(variant_root / "metadata.json", metadata)
            all_rows.append(
                {
                    **metadata,
                    "initial_factor_residuals": initial_residuals,
                    "initial_factor_gradients": initial_gradient_json,
                    "initial_gradient_cosine": cosine,
                    "final_gradient_cosine": final_cosine,
                    "optimization": optimization,
                    "geometry": geometry,
                    "controller": controller,
                    "rigid": rigid,
                }
            )
        if not np.array_equal(raw, raw_copy):
            raise RuntimeError("immutable raw FRESH suffix changed")

    for name, inputs in synthetic_inputs.items():
        case_root = output / "synthetic" / name
        case_root.mkdir(parents=True, exist_ok=False)
        values = synthetic_values[name]
        save_npy_exclusive(case_root / "old.npy", values["old"])
        save_npy_exclusive(case_root / "raw_fresh.npy", values["fresh"])
        if "desired_diagnostic_target" in values:
            save_npy_exclusive(
                case_root / "desired_diagnostic_target.npy",
                values["desired_diagnostic_target"],
            )
        execute_condition(
            kind="synthetic",
            condition=name,
            inputs=inputs,
            root=case_root,
            old_final_command=np.asarray(
                config["diagnostics"]["synthetic_old_final_command_v_omega"]
            ),
            entry_index=0,
        )
    for (case, k), (inputs, source) in real_inputs.items():
        execute_condition(
            kind="real",
            condition=case,
            inputs=inputs,
            root=output / "real" / case / f"k_{k}",
            old_final_command=source.old_commands[-1],
            entry_index=k,
        )

    summary_root = output / "summary"
    summary_root.mkdir(parents=True, exist_ok=False)
    real_rows = [row for row in all_rows if row["condition_kind"] == "real"]
    synthetic_rows = [row for row in all_rows if row["condition_kind"] == "synthetic"]
    attribution = {
        "experiment": "EXP-02C",
        "real_condition_count": 9,
        "variant_count": len(VARIANT_ACTIVE_FACTORS),
        "rows": [
            {
                "case_id": row["condition"],
                "entry_index": row["entry_index"],
                "variant": row["variant"],
                "metrics": compact_metrics(row),
                "initial_factor_costs": {
                    name: item["diagnostic_weighted_cost"]
                    for name, item in row["initial_factor_residuals"].items()
                },
                "initial_weighted_gradient_norms": {
                    name: item["weighted"]["gradient_norm"]
                    for name, item in row["initial_factor_gradients"]["factors"].items()
                },
            }
            for row in real_rows
        ],
    }
    save_json_exclusive(summary_root / "factor_attribution_summary.json", attribution)

    index = {
        (row["condition"], row["entry_index"], row["variant"]): row
        for row in real_rows
    }
    additive_rows = []
    leave_rows = []
    additive_variants = (
        "V0_RAW",
        "V1_ENTRY_FRESH",
        "V2_ENTRY_DIRECTION_FRESH",
        "V3_ENTRY_YAW_FRESH",
        "V4_FULL_CURRENT_M4",
        "V5_NO_ENTRY",
        "V8_DIAGNOSTIC_NO_PROPAGATION",
    )
    metric_names = tuple(compact_metrics(real_rows[0]))
    for case in CASE_IDS:
        for k in (0, 3, 6):
            metrics = {
                variant: compact_metrics(index[(case, k, variant)])
                for variant in additive_variants
            }
            contexts = {
                "direction_added_to_entry_fresh": ("V2_ENTRY_DIRECTION_FRESH", "V1_ENTRY_FRESH"),
                "yaw_added_to_entry_fresh": ("V3_ENTRY_YAW_FRESH", "V1_ENTRY_FRESH"),
                "yaw_added_after_direction": ("V4_FULL_CURRENT_M4", "V2_ENTRY_DIRECTION_FRESH"),
                "direction_added_after_yaw": ("V4_FULL_CURRENT_M4", "V3_ENTRY_YAW_FRESH"),
            }
            incremental = {
                label: {
                    name: metrics[left][name] - metrics[right][name]
                    for name in metric_names
                }
                for label, (left, right) in contexts.items()
            }
            additive_rows.append(
                {
                    "case_id": case,
                    "entry_index": k,
                    "variants": metrics,
                    "incremental_effect_under_factor_context": incremental,
                }
            )
            full = compact_metrics(index[(case, k, "V4_FULL_CURRENT_M4")])
            ablations = {
                "entry": "V5_NO_ENTRY",
                "direction": "V6_NO_DIRECTION",
                "yaw": "V7_NO_YAW",
                "fresh_motion": "V8_DIAGNOSTIC_NO_PROPAGATION",
            }
            leave_rows.append(
                {
                    "case_id": case,
                    "entry_index": k,
                    "full": full,
                    "without_factor": {
                        factor: {
                            "variant": variant,
                            "metrics": compact_metrics(index[(case, k, variant)]),
                            "full_minus_without": {
                                name: full[name]
                                - compact_metrics(index[(case, k, variant)])[name]
                                for name in metric_names
                            },
                        }
                        for factor, variant in ablations.items()
                    },
                }
            )
    save_json_exclusive(
        summary_root / "additive_ablation_summary.json",
        {"effect_semantics": "incremental effect under the named factor context", "rows": additive_rows},
    )
    save_json_exclusive(
        summary_root / "leave_one_out_summary.json",
        {
            "fresh_motion_warning": "V8 fixes downstream raw nodes and is a diagnostic counterfactual, not a standard unconstrained ablation",
            "rows": leave_rows,
        },
    )

    synthetic_index = {(row["condition"], row["variant"]): row for row in synthetic_rows}
    synthetic_summary = {
        "evidence_status": "synthetic mechanism checks only; not experimental evidence",
        "cases": {},
    }
    zero_tolerance = float(config["diagnostics"]["raw_zero_tolerance"])
    for name in synthetic_inputs:
        raw_row = synthetic_index[(name, "V0_RAW")]
        costs = {
            factor: item["diagnostic_weighted_cost"]
            for factor, item in raw_row["initial_factor_residuals"].items()
        }
        synthetic_summary["cases"][name] = {
            "initial_factor_costs": costs,
            "nonzero_factors": [factor for factor, cost in costs.items() if cost > zero_tolerance],
            "variant_metrics": {
                variant: compact_metrics(synthetic_index[(name, variant)])
                for variant in VARIANT_ACTIVE_FACTORS
            },
        }
    save_json_exclusive(summary_root / "synthetic_mechanism_summary.json", synthetic_summary)

    benign_rows = {
        variant: index[("case_benign_delayed", 0, variant)]
        for variant in VARIANT_ACTIVE_FACTORS
    }
    initial = benign_rows["V0_RAW"]["initial_factor_residuals"]
    gradients = benign_rows["V0_RAW"]["initial_factor_gradients"]["factors"]
    initial_costs = {name: item["diagnostic_weighted_cost"] for name, item in initial.items()}
    gradient_norms = {name: item["weighted"]["gradient_norm"] for name, item in gradients.items()}
    benign = {
        "case_id": "case_benign_delayed",
        "entry_index": 0,
        "raw_initial_factor_costs": initial_costs,
        "raw_initial_weighted_gradient_norms": gradient_norms,
        "largest_nonzero_initial_cost_factor": max(initial_costs, key=initial_costs.get),
        "largest_initial_weighted_gradient_factor": max(gradient_norms, key=gradient_norms.get),
        "variant_metrics": {variant: compact_metrics(row) for variant, row in benign_rows.items()},
        "full_initial_gradient_cosines": benign_rows["V4_FULL_CURRENT_M4"]["initial_gradient_cosine"],
        "full_final_gradient_cosines": benign_rows["V4_FULL_CURRENT_M4"]["final_gradient_cosine"],
        "objective_knows_raw_controller_is_benign": False,
        "reason": "no TrajectoryFollower/controller residual appears in current M4",
    }
    save_json_exclusive(summary_root / "benign_case_failure_attribution.json", benign)

    full_real = [row for row in real_rows if row["variant"] == "V4_FULL_CURRENT_M4"]
    full_rigid_max = max(row["rigid"]["translation_rms_m"] for row in full_real)
    direction_delta = abs(
        compact_metrics(benign_rows["V2_ENTRY_DIRECTION_FRESH"])["delta_v_des_abs_mps"]
        - compact_metrics(benign_rows["V1_ENTRY_FRESH"])["delta_v_des_abs_mps"]
    )
    yaw_delta = abs(
        compact_metrics(benign_rows["V3_ENTRY_YAW_FRESH"])["delta_v_des_abs_mps"]
        - compact_metrics(benign_rows["V1_ENTRY_FRESH"])["delta_v_des_abs_mps"]
    )
    recommendation = (
        "Redesign the incoming-direction transition residual semantics before adding another factor."
        if direction_delta >= yaw_delta
        else "Redesign the incoming-yaw transition residual semantics before adding another factor."
    )
    final = {
        "experiment": "EXP-02C",
        "success_criterion": "failure attribution, not improved candidate performance",
        "full_m4_regression_passed": all(row["passed"] for row in regression_rows),
        "classification": {
            "bad_factor_candidate": benign["largest_nonzero_initial_cost_factor"],
            "factor_conflict_at_raw": benign["full_initial_gradient_cosines"]["pairs"],
            "factor_conflict_at_full_solution": benign["full_final_gradient_cosines"]["pairs"],
            "structural_limitation": "fresh_motion preserves only relative edges and provides no return to absolute raw FRESH",
            "missing_mechanism": "no direct transition-magnitude residual and no downstream absolute-FRESH recovery residual",
            "redundant_factor": "not established by factor identity alone; V6/V3 and V7/V2 are intentional duplicate labels",
        },
        "full_real_rigid_fit_translation_rms_max_m": full_rigid_max,
        "benign_direction_increment_delta_v_abs_mps": direction_delta,
        "benign_yaw_increment_delta_v_abs_mps": yaw_delta,
        "exactly_one_next_formulation_recommendation": recommendation,
        "claim_limitations": [
            "offline desired-command probes are not Isaac measured response",
            "synthetic cases demonstrate mechanisms only",
            "correlation and ablation do not prove physical causality",
            "LightNav waypoint rows have no intrinsic timestamps",
        ],
    }
    save_json_exclusive(summary_root / "final_exp02c_interpretation.json", final)
    save_json_exclusive(
        output / "metadata.json",
        {
            "experiment": "EXP-02C",
            "run_id": run_id,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "git_sha_at_run": subprocess.check_output(
                ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
            ).strip(),
            "diagnostic_only": True,
            "real_condition_count": 9,
            "synthetic_condition_count": 5,
            "variant_count_per_condition": 9,
            "artifact_variant_count": len(all_rows),
            "production_objective_changed": False,
            "controller_residual_implemented": False,
            "new_formulation_implemented": False,
            "intrinsic_waypoint_time_base": False,
            "numerical_jacobian": {
                "scheme": "central finite difference",
                "epsilon": solver.finite_difference_epsilon,
                "retract": "right-local T <- T * Exp(delta)",
            },
        },
    )
    print("EXP02C_OUTPUT_DIR=" + str(output))


def wrap_angles(values: np.ndarray) -> np.ndarray:
    return (np.asarray(values, dtype=np.float64) + np.pi) % (2.0 * np.pi) - np.pi


if __name__ == "__main__":
    main()
