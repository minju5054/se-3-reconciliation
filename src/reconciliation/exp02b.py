"""Pure selection, reconciliation, metrics, and validation for EXP-02B."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.controller_switch_metrics import controller_switch_metrics
from reconciliation.exp01b_controlled_latency import load_attempt_records
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.rigid_reconciliation import analytic_rigid_reconciliation
from reconciliation.se2 import relative_pose, se2_log, wrap_angle
from reconciliation.se2_graph import new_motion_residual
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory
from reconciliation.transition_graph import problem_from_config, transition_residual_vector


CASE_IDS = ("case_high_delta_v", "case_high_delta_omega", "case_benign_delayed")
METHODS = ("raw_f0", "raw_k", "pose_anchor", "rigid", "graph")
SAME_K_METHODS = ("raw_k", "pose_anchor", "rigid", "graph")


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _sign(value: float, tolerance: float = 1e-12) -> int:
    return 1 if value > tolerance else -1 if value < -tolerance else 0


def validate_exp02b_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if config.get("experiment") != "EXP-02B":
        raise ValueError("config is not EXP-02B")
    indices = config.get("entry_indices")
    if indices != [0, 3, 6]:
        raise ValueError("EXP-02B entry_indices must be the predeclared [0, 3, 6]")
    if tuple(config.get("methods", ())) != METHODS:
        raise ValueError(f"methods must be exactly {METHODS}")
    epsilon = _positive(config["selection"]["normalization_epsilon"], "normalization_epsilon")
    frozen = config.get("frozen_source_cases")
    if not isinstance(frozen, Mapping) or tuple(frozen) != CASE_IDS:
        raise ValueError("frozen_source_cases must contain the three ordered case ids")
    for case_id, item in frozen.items():
        if not isinstance(item, Mapping) or not str(item.get("relative_trial_path", "")):
            raise ValueError(f"{case_id} requires relative_trial_path")
        hashes = item.get("sha256")
        if not isinstance(hashes, Mapping):
            raise ValueError(f"{case_id} requires source hashes")
        for filename in (
            "raw/old_actions.npy",
            "raw/fresh_actions.npy",
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "derived/controller_commands.csv",
            "results/attempt.json",
            "metadata.json",
        ):
            digest = str(hashes.get(filename, ""))
            if len(digest) != 64:
                raise ValueError(f"{case_id} missing SHA-256 for {filename}")
    return {"entry_indices": list(indices), "methods": list(METHODS), "epsilon": epsilon}


def select_source_cases(source_root: str | Path, *, epsilon: float) -> dict[str, dict[str, Any]]:
    """Apply the frozen A/B/C source rules with full-path tie breaking."""

    root = Path(source_root).resolve()
    eps = _positive(epsilon, "epsilon")
    moving = sorted(
        (
            row
            for row in load_attempt_records(root)
            if row.get("classification") == "VALID_MOVING"
        ),
        key=lambda row: str(Path(row["artifact_path"]).resolve()),
    )
    if not moving:
        raise ValueError("source cohort contains no timing-valid moving transitions")

    def metric(row: Mapping[str, Any], name: str) -> float:
        value = float(row["controller_metrics"][name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"invalid source {name}")
        return value

    high_v = min(
        moving,
        key=lambda row: (-metric(row, "delta_v_abs_mps"), str(row["artifact_path"])),
    )
    high_omega = min(
        moving,
        key=lambda row: (-metric(row, "delta_omega_abs_rps"), str(row["artifact_path"])),
    )
    median_v = float(np.median([metric(row, "delta_v_abs_mps") for row in moving]))
    median_omega = float(np.median([metric(row, "delta_omega_abs_rps") for row in moving]))
    delayed = [row for row in moving if row.get("latency_condition_id") == "L1_added_050"]
    if not delayed:
        raise ValueError("source cohort has no valid moving L1_added_050 transition")

    def benign_score(row: Mapping[str, Any]) -> float:
        return metric(row, "delta_v_abs_mps") / (median_v + eps) + metric(
            row, "delta_omega_abs_rps"
        ) / (median_omega + eps)

    benign = min(delayed, key=lambda row: (benign_score(row), str(row["artifact_path"])))
    return {
        "case_high_delta_v": dict(high_v),
        "case_high_delta_omega": dict(high_omega),
        "case_benign_delayed": {**dict(benign), "selection_score": benign_score(benign)},
        "_normalization": {
            "valid_moving_count": len(moving),
            "median_delta_v_abs_mps": median_v,
            "median_delta_omega_abs_rps": median_omega,
            "epsilon": eps,
        },
    }


def verify_frozen_source_selection(
    source_root: str | Path, config: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    normalized = validate_exp02b_config(config)
    root = Path(source_root).resolve()
    selected = select_source_cases(root, epsilon=normalized["epsilon"])
    for case_id in CASE_IDS:
        trial = Path(selected[case_id]["artifact_path"]).resolve()
        expected = (root / config["frozen_source_cases"][case_id]["relative_trial_path"]).resolve()
        if trial != expected:
            raise ValueError(f"frozen {case_id} no longer matches deterministic selection")
        for relative, digest in config["frozen_source_cases"][case_id]["sha256"].items():
            if sha256_file(trial / relative) != digest:
                raise ValueError(f"immutable source hash changed: {case_id}/{relative}")
    return selected


@dataclass(frozen=True, slots=True)
class SourceCase:
    case_id: str
    trial_directory: Path
    old_world: np.ndarray
    fresh_world: np.ndarray
    previous_pose_world: np.ndarray
    boundary_pose_world: np.ndarray
    source_attempt: Mapping[str, Any]
    source_metadata: Mapping[str, Any]
    old_commands: np.ndarray


def load_source_case(case_id: str, trial_directory: str | Path) -> SourceCase:
    if case_id not in CASE_IDS:
        raise ValueError(f"unknown case id: {case_id}")
    trial = Path(trial_directory).resolve()
    attempt = load_strict_json(trial / "results/attempt.json")
    metadata = load_strict_json(trial / "metadata.json")
    if attempt.get("classification") != "VALID_MOVING" or attempt.get("stop_output"):
        raise ValueError("EXP-02B source must be a valid moving FRESH trial")
    old = validate_se2_trajectory(np.load(trial / "derived/old_world.npy", allow_pickle=False))
    fresh = validate_se2_trajectory(np.load(trial / "derived/fresh_world.npy", allow_pickle=False))
    previous = validate_pose_se2(attempt["actual_pose_before_ready"], name="previous_pose_world")
    boundary = validate_pose_se2(attempt["robot_pose_at_new_ready"], name="boundary_pose_world")
    with (trial / "derived/controller_commands.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    old_commands = np.asarray(
        [
            [float(row["v_command_mps"]), float(row["omega_command_rps"])]
            for row in rows
            if row["reference_source"] == "OLD"
        ],
        dtype=np.float64,
    )
    if old_commands.shape[0] < 3:
        raise ValueError("source must contain at least three OLD controller commands")
    return SourceCase(
        case_id,
        trial,
        old,
        fresh,
        previous,
        boundary,
        attempt,
        metadata,
        old_commands,
    )


def transition_input(source: SourceCase, entry_index: int) -> TransitionReconciliationInput:
    return TransitionReconciliationInput(
        old_poses_world=source.old_world,
        fresh_poses_world=source.fresh_world,
        committed_pose_world=source.boundary_pose_world,
        actual_pose_before_committed=source.previous_pose_world,
        entry_context=SpatialEntryContext(
            entry_index=entry_index,
            evidence={},
            evidence_status={},
            source="manual_exp02b",
            metadata={
                "selector_implemented": False,
                "semantics": "spatial FRESH entry; not temporal delay",
            },
        ),
        metadata={"source_trial": str(source.trial_directory), "case_id": source.case_id},
    )


def build_candidate_methods(
    inputs: TransitionReconciliationInput,
    *,
    graph_config: Mapping[str, Any],
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Build the five frozen methods without changing the EXP-02A graph."""

    raw_k = inputs.selected_suffix
    rigid = analytic_rigid_reconciliation(
        inputs.incoming_previous_pose,
        inputs.committed_pose_world,
        raw_k,
        minimum_translation_m=float(graph_config["minimum_direction_translation_m"]),
    )
    solver = SolverConfig(**graph_config["solver"])
    optimized: dict[str, np.ndarray] = {}
    reports: dict[str, Any] = {"rigid": rigid.to_dict()}
    for method, variant in (("pose_anchor", "pose_anchor"), ("graph", "incoming_motion_aware")):
        problem = problem_from_config(
            inputs,
            variant,
            residual_scales=graph_config["residual_scales"],
            weights=graph_config["variants"][variant],
            minimum_translation_m=float(graph_config["minimum_direction_translation_m"]),
        )
        result = solve_least_squares(
            problem.raw_new,
            lambda state, graph=problem: transition_residual_vector(graph, state),
            solver,
        )
        if not result.converged:
            raise RuntimeError(f"{method} optimization did not converge: {result.termination_reason}")
        optimized[method] = result.optimized
        reports[method] = result.to_dict()
    candidates = {
        "raw_f0": inputs.fresh_poses_world.copy(),
        "raw_k": raw_k,
        "pose_anchor": optimized["pose_anchor"],
        "rigid": rigid.trajectory_world,
        "graph": optimized["graph"],
    }
    return candidates, reports


def geometric_preservation_metrics(
    *,
    raw_reference: ArrayLike,
    candidate: ArrayLike,
    previous_pose_world: ArrayLike,
    boundary_pose_world: ArrayLike,
    minimum_translation_m: float,
) -> dict[str, Any]:
    """Keep execution smoothness and FRESH intent as separate measurements."""

    raw = validate_se2_trajectory(raw_reference, name="raw_reference")
    output = validate_se2_trajectory(candidate, name="candidate")
    if raw.shape != output.shape:
        raise ValueError("candidate shape must match the method's raw reference")
    previous = validate_pose_se2(previous_pose_world, name="previous_pose_world")
    boundary = validate_pose_se2(boundary_pose_world, name="boundary_pose_world")
    threshold = _positive(minimum_translation_m, "minimum_translation_m")
    incoming = boundary[:2] - previous[:2]
    incoming_norm = float(np.linalg.norm(incoming))
    if incoming_norm <= threshold:
        raise ValueError("incoming tangent is undefined")
    tangent = incoming / incoming_norm
    normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)

    edge_residuals = np.asarray(
        [
            new_motion_residual(raw[j], raw[j + 1], output[j], output[j + 1])
            for j in range(raw.shape[0] - 1)
        ],
        dtype=np.float64,
    )
    edge_translation = np.linalg.norm(edge_residuals[:, :2], axis=1)
    edge_yaw = np.abs(edge_residuals[:, 2])
    entry_delta = se2_log(relative_pose(raw[0], output[0]))
    endpoint_delta = se2_log(relative_pose(raw[-1], output[-1]))
    raw_yaw_progression = float(np.sum(wrap_angle(np.diff(raw[:, 2]))))
    output_yaw_progression = float(np.sum(wrap_angle(np.diff(output[:, 2]))))
    raw_lateral = float((raw[-1, :2] - boundary[:2]) @ normal)
    output_lateral = float((output[-1, :2] - boundary[:2]) @ normal)
    raw_turn_sign = _sign(raw_yaw_progression)
    output_turn_sign = _sign(output_yaw_progression)
    correction = np.asarray(
        [se2_log(relative_pose(raw[j], output[j])) for j in range(raw.shape[0])]
    )
    transition = output[0, :2] - boundary[:2]
    transition_norm = float(np.linalg.norm(transition))
    direction_jump = (
        None
        if transition_norm <= threshold
        else abs(
            float(
                wrap_angle(
                    math.atan2(float(transition[1]), float(transition[0]))
                    - math.atan2(float(tangent[1]), float(tangent[0]))
                )
            )
        )
    )
    return {
        "shape": list(output.shape),
        "selected_entry_displacement": {
            "translation_m": float(np.linalg.norm(entry_delta[:2])),
            "yaw_rad": float(entry_delta[2]),
            "abs_yaw_rad": abs(float(entry_delta[2])),
        },
        "transition": {
            "translation_from_boundary_m": transition_norm,
            "abs_yaw_from_boundary_rad": abs(float(relative_pose(boundary, output[0])[2])),
            "incoming_to_transition_direction_jump_rad": direction_jump,
        },
        "fresh_edge_deformation": {
            "translation_rms_m": float(np.sqrt(np.mean(edge_translation**2))),
            "translation_max_m": float(np.max(edge_translation)),
            "yaw_rms_rad": float(np.sqrt(np.mean(edge_yaw**2))),
            "yaw_max_rad": float(np.max(edge_yaw)),
            "per_edge_se2_log": edge_residuals.tolist(),
        },
        "endpoint_deviation": {
            "translation_m": float(np.linalg.norm(endpoint_delta[:2])),
            "yaw_rad": float(endpoint_delta[2]),
            "abs_yaw_rad": abs(float(endpoint_delta[2])),
        },
        "yaw_progression": {
            "raw_rad": raw_yaw_progression,
            "candidate_rad": output_yaw_progression,
            "difference_rad": float(wrap_angle(output_yaw_progression - raw_yaw_progression)),
            "raw_turn_sign": raw_turn_sign,
            "candidate_turn_sign": output_turn_sign,
            "turn_direction_preserved": raw_turn_sign == output_turn_sign,
        },
        "incoming_tangent_frame": {
            "raw_endpoint_lateral_m": raw_lateral,
            "candidate_endpoint_lateral_m": output_lateral,
            "lateral_difference_m": output_lateral - raw_lateral,
            "lateral_sign_preserved": _sign(raw_lateral) == _sign(output_lateral),
        },
        "downstream_correction": {
            "translation_m": np.linalg.norm(correction[:, :2], axis=1).tolist(),
            "yaw_rad": correction[:, 2].tolist(),
        },
    }


def inter_k_retention(
    *,
    fresh_world: ArrayLike,
    entry_indices: Sequence[int],
    states_by_k: Mapping[int, ArrayLike],
) -> dict[str, Any]:
    fresh = validate_se2_trajectory(fresh_world, name="fresh_world")
    indices = list(entry_indices)
    if sorted(states_by_k) != sorted(indices) or len(indices) < 2:
        raise ValueError("states_by_k must contain every requested k")
    pairs: list[dict[str, Any]] = []
    for left_offset, left in enumerate(indices):
        for right in indices[left_offset + 1 :]:
            left_state = validate_se2_trajectory(states_by_k[left], name=f"k_{left}")
            right_state = validate_se2_trajectory(states_by_k[right], name=f"k_{right}")
            raw_distance = float(np.linalg.norm(fresh[left, :2] - fresh[right, :2]))
            output_distance = float(np.linalg.norm(left_state[0, :2] - right_state[0, :2]))
            if raw_distance <= 0.0:
                raise ValueError("raw inter-k entry separation must be positive")
            pairs.append(
                {
                    "left_k": left,
                    "right_k": right,
                    "raw_entry_separation_m": raw_distance,
                    "candidate_entry_separation_m": output_distance,
                    "entry_separation_retention_ratio": output_distance / raw_distance,
                }
            )
    return {"pair_count": len(pairs), "pairs": pairs}


def comparability_gate(
    *,
    source_boundary: ArrayLike,
    reproduced_boundary: ArrayLike,
    source_pre_switch_command: ArrayLike,
    reproduced_pre_switch_command: ArrayLike,
    translation_tolerance_m: float,
    yaw_tolerance_rad: float,
    command_tolerance: float,
) -> dict[str, Any]:
    source = validate_pose_se2(source_boundary, name="source_boundary")
    reproduced = validate_pose_se2(reproduced_boundary, name="reproduced_boundary")
    source_command = np.asarray(source_pre_switch_command, dtype=np.float64)
    reproduced_command = np.asarray(reproduced_pre_switch_command, dtype=np.float64)
    if source_command.shape != (2,) or reproduced_command.shape != (2,):
        raise ValueError("pre-switch commands must have shape (2,)")
    if not np.all(np.isfinite(source_command)) or not np.all(np.isfinite(reproduced_command)):
        raise ValueError("pre-switch commands must be finite")
    translation_error = float(np.linalg.norm(source[:2] - reproduced[:2]))
    yaw_error = abs(float(wrap_angle(reproduced[2] - source[2])))
    command_error = np.abs(reproduced_command - source_command)
    checks = {
        "translation": translation_error <= _positive(translation_tolerance_m, "translation_tolerance_m"),
        "yaw": yaw_error <= _positive(yaw_tolerance_rad, "yaw_tolerance_rad"),
        "command_v": float(command_error[0]) <= _positive(command_tolerance, "command_tolerance"),
        "command_omega": float(command_error[1]) <= float(command_tolerance),
    }
    return {
        "valid": all(checks.values()),
        "checks": checks,
        "translation_error_m": translation_error,
        "yaw_error_rad": yaw_error,
        "pre_switch_command_abs_error": command_error.tolist(),
    }


def controller_improvement(method_value: float, raw_value: float, *, epsilon: float) -> float | None:
    raw = float(raw_value)
    value = float(method_value)
    eps = _positive(epsilon, "epsilon")
    if not math.isfinite(raw) or not math.isfinite(value) or raw < 0.0 or value < 0.0:
        raise ValueError("controller discontinuities must be finite and nonnegative")
    return None if raw <= eps else 1.0 - value / (raw + eps)


def validate_exp02b_output(path: str | Path, config: Mapping[str, Any]) -> dict[str, Any]:
    root = Path(path)
    source = load_strict_json(root / "source_selection.json")
    protocol = load_strict_json(root / "protocol.json")
    assert_finite_json_tree(source)
    assert_finite_json_tree(protocol)
    if source.get("experiment") != "EXP-02B" or protocol.get("methods") != list(METHODS):
        raise ValueError("invalid EXP-02B source/protocol")
    source_root = Path(source["source_root"])
    verify_frozen_source_selection(source_root, config)
    branch_count = 0
    for case_id in CASE_IDS:
        trial = Path(source["cases"][case_id]["trial_directory"])
        for relative, digest in source["cases"][case_id]["sha256"].items():
            if sha256_file(trial / relative) != digest:
                raise ValueError(f"source changed after EXP-02B: {case_id}/{relative}")
        fresh = validate_se2_trajectory(np.load(trial / "derived/fresh_world.npy", allow_pickle=False))
        case_root = root / case_id
        for k in config["entry_indices"]:
            raw_k = fresh[k:]
            for method in METHODS:
                branch = case_root / f"k_{k}" / method
                candidate = validate_se2_trajectory(np.load(branch / "candidate.npy", allow_pickle=False))
                expected_raw = fresh if method == "raw_f0" else raw_k
                if method == "raw_f0" and not np.array_equal(candidate, fresh):
                    raise ValueError("M0 must be exact raw FRESH[0:]")
                if method == "raw_k" and not np.array_equal(candidate, raw_k):
                    raise ValueError("M1 must be exact raw FRESH[k:]")
                if candidate.shape != expected_raw.shape:
                    raise ValueError(f"candidate shape mismatch: {case_id}/k{k}/{method}")
                assert_finite_json_tree(load_strict_json(branch / "geometric_metrics.json"))
                branch_count += 1
    execution_exists = (root / "execution_summary.json").is_file()
    if execution_exists:
        execution = load_strict_json(root / "execution_summary.json")
        assert_finite_json_tree(execution)
        if not execution.get("all_branches_valid"):
            raise ValueError("one or more Isaac branches failed comparability/execution validation")
        branches = execution.get("branches")
        if not isinstance(branches, list) or len(branches) != len(CASE_IDS) * len(config["entry_indices"]) * len(METHODS):
            raise ValueError("execution summary must contain every predeclared branch")
        seen: set[tuple[str, int, str]] = set()
        for row in branches:
            key = (str(row["case_id"]), int(row["entry_index"]), str(row["method"]))
            if key in seen or key[0] not in CASE_IDS or key[1] not in config["entry_indices"] or key[2] not in METHODS:
                raise ValueError(f"invalid or duplicate execution branch: {key}")
            seen.add(key)
            branch = root / key[0] / f"k_{key[1]}" / key[2] / "execution"
            actual = validate_se2_trajectory(
                np.load(branch / "actual_trajectory.npy", allow_pickle=False),
                name="actual_trajectory",
            )
            if actual.shape[0] < int(config["execution"]["command_window"]):
                raise ValueError(f"execution trajectory is too short: {key}")
            with (branch / "controller_commands.csv").open(
                newline="", encoding="utf-8"
            ) as stream:
                command_rows = list(csv.DictReader(stream))
            fresh_commands = np.asarray(
                [
                    [float(item["v_command_mps"]), float(item["omega_command_rps"])]
                    for item in command_rows
                ],
                dtype=np.float64,
            )
            source_case = load_source_case(key[0], source["cases"][key[0]]["trial_directory"])
            rebuilt = controller_switch_metrics(
                source_case.old_commands,
                fresh_commands,
                control_dt_s=float(config["execution"]["control_dt"]),
                window_size=min(
                    int(config["execution"]["command_window"]),
                    len(source_case.old_commands),
                    len(fresh_commands),
                ),
            )
            stored = load_strict_json(branch / "controller_metrics.json")
            for name in (
                "delta_v_signed_mps",
                "delta_omega_signed_rps",
                "delta_v_abs_mps",
                "delta_omega_abs_rps",
                "post_switch_max_abs_delta_v_mps",
                "post_switch_mean_abs_delta_v_mps",
                "post_switch_max_abs_delta_omega_rps",
                "post_switch_mean_abs_delta_omega_rps",
            ):
                if not math.isclose(float(stored[name]), float(rebuilt[name]), abs_tol=1e-12):
                    raise ValueError(f"controller metric reconstruction failed: {key}/{name}")
            gate = load_strict_json(branch / "comparability.json")
            if gate != row["comparability"] or not gate.get("valid"):
                raise ValueError(f"comparability gate failed reconstruction: {key}")
        expected = {
            (case, int(k), method)
            for case in CASE_IDS
            for k in config["entry_indices"]
            for method in METHODS
        }
        if seen != expected:
            raise ValueError("execution branch set differs from frozen protocol")
    summary_exists = (root / "summary.json").is_file()
    plot_count = 0
    if summary_exists:
        final_summary = load_strict_json(root / "summary.json")
        assert_finite_json_tree(final_summary)
        if final_summary.get("experiment") != "EXP-02B":
            raise ValueError("final summary is not EXP-02B")
        summary_branches = final_summary.get("branches")
        if not isinstance(summary_branches, list) or len(summary_branches) != branch_count:
            raise ValueError("final summary does not contain every offline branch")
        plot_files = final_summary.get("plot_files")
        if not isinstance(plot_files, list) or len(plot_files) < 7:
            raise ValueError("final summary must list all required review plots")
        for relative in plot_files:
            plot = root / str(relative)
            if not plot.is_file() or plot.stat().st_size == 0:
                raise ValueError(f"missing or empty plot: {relative}")
        plot_count = len(plot_files)
    return {
        "valid": True,
        "source_hashes_match": True,
        "branch_count": branch_count,
        "execution_present": execution_exists,
        "summary_present": summary_exists,
        "plot_count": plot_count,
    }
