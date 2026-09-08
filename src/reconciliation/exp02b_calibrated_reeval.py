"""Pure provenance, invariant, and metric helpers for frozen EXP-02B-R."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02b import CASE_IDS, validate_exp02b_output
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


FloatArray = NDArray[np.float64]
REEVAL_METHODS = ("raw_k", "rigid", "graph")
ENTRY_INDICES = (0, 3, 6)


def _finite_array(name: str, value: ArrayLike, *, columns: int | None = None) -> FloatArray:
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or (columns is not None and result.shape[1] != columns):
        raise ValueError(f"{name} must be a two-dimensional array")
    if result.shape[0] == 0 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain finite samples")
    return result.copy()


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _rmse(values: FloatArray) -> float:
    if values.size == 0:
        raise ValueError("RMSE requires samples")
    return float(np.sqrt(np.mean(np.square(values))))


def follower_config(values: Mapping[str, Any]) -> FollowerConfig:
    return FollowerConfig(
        **{name: values[name] for name in FollowerConfig.__dataclass_fields__}
    )


def validate_reeval_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if config.get("experiment") != "EXP-02B-R":
        raise ValueError("configuration is not EXP-02B-R")
    if tuple(config.get("cases", ())) != CASE_IDS:
        raise ValueError(f"cases must remain {CASE_IDS}")
    if tuple(config.get("entry_indices", ())) != ENTRY_INDICES:
        raise ValueError(f"entry indices must remain {ENTRY_INDICES}")
    if tuple(config.get("methods", ())) != REEVAL_METHODS:
        raise ValueError(f"methods must remain {REEVAL_METHODS}")
    simulation = config["simulation"]
    physics_dt = _positive(simulation["physics_dt_s"], "physics_dt_s")
    control_dt = _positive(simulation["control_dt_s"], "control_dt_s")
    steps = round(control_dt / physics_dt)
    if steps < 1 or not math.isclose(steps * physics_dt, control_dt, abs_tol=1e-12):
        raise ValueError("control_dt_s must be an integer multiple of physics_dt_s")
    protocol = config["protocol"]
    required_true = (
        "replay_old_with_historical_nominal_execution",
        "exact_boundary_state_reset_after_gate",
        "restore_last_old_wheel_target",
    )
    if any(protocol.get(name) is not True for name in required_true):
        raise ValueError("historical replay/reset protocol cannot be disabled")
    if protocol.get("calibrated_controller_state_at_switch") != "RESET":
        raise ValueError("calibrated controller state at switch must be RESET")
    if (
        protocol.get("first_calibrated_feedback_measurement")
        != "final_pre_reset_old_control_interval"
    ):
        raise ValueError("first calibrated feedback measurement semantics changed")
    _positive(protocol["first_desired_invariant_tolerance"], "invariant tolerance")
    manifest = config.get("candidate_sha256")
    for case_id in CASE_IDS:
        for k in ENTRY_INDICES:
            values = manifest.get(case_id, {}).get(f"k_{k}", {}) if isinstance(manifest, Mapping) else {}
            for method in REEVAL_METHODS:
                if len(str(values.get(method, ""))) != 64:
                    raise ValueError(f"missing candidate hash: {case_id}/k{k}/{method}")
    assert_finite_json_tree(config)
    return {"valid": True, "physics_steps_per_control": int(steps), "branch_count": 27}


def verify_file_hash(path: str | Path, expected: str, label: str) -> str:
    digest = sha256_file(path)
    if digest != expected:
        raise ValueError(f"frozen provenance mismatch for {label}: {digest} != {expected}")
    return digest


def verify_historical_run(
    historical_run: str | Path,
    config: Mapping[str, Any],
    historical_exp02b_config: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate all historical outputs and freeze the 27 reused branches."""

    validate_reeval_config(config)
    root = Path(historical_run).resolve()
    names = {
        "source_selection.json": "source_selection_sha256",
        "protocol.json": "protocol_sha256",
        "offline_summary.json": "offline_summary_sha256",
        "offline_validation.json": "offline_validation_sha256",
        "execution_summary.json": "execution_summary_sha256",
        "summary.json": "summary_sha256",
    }
    provenance = config["historical_provenance"]
    top_hashes = {
        name: verify_file_hash(root / name, provenance[key], f"historical {name}")
        for name, key in names.items()
    }
    validation = validate_exp02b_output(root, historical_exp02b_config)
    if not validation.get("valid") or validation.get("branch_count") != 45:
        raise ValueError("historical EXP-02B strict validation failed")
    source_selection = load_strict_json(root / "source_selection.json")
    execution = load_strict_json(root / "execution_summary.json")
    execution_by_key = {
        (row["case_id"], int(row["entry_index"]), row["method"]): row
        for row in execution["branches"]
    }
    branches: list[dict[str, Any]] = []
    for case_id in CASE_IDS:
        for k in ENTRY_INDICES:
            for method in REEVAL_METHODS:
                branch = root / case_id / f"k_{k}" / method
                candidate_path = branch / "candidate.npy"
                expected = config["candidate_sha256"][case_id][f"k_{k}"][method]
                candidate_hash = verify_file_hash(
                    candidate_path, expected, f"candidate {case_id}/k{k}/{method}"
                )
                candidate = validate_se2_trajectory(
                    np.load(candidate_path, allow_pickle=False), name="frozen candidate"
                )
                if method == "raw_k":
                    source_dir = Path(source_selection["cases"][case_id]["trial_directory"])
                    fresh = np.load(source_dir / "derived/fresh_world.npy", allow_pickle=False)
                    if not np.array_equal(candidate, fresh[k:]):
                        raise ValueError(f"raw_k candidate changed: {case_id}/k{k}")
                geometric_path = branch / "geometric_metrics.json"
                historical_metrics_path = branch / "execution/controller_metrics.json"
                historical_commands_path = branch / "execution/controller_commands.csv"
                historical_actual_path = branch / "execution/actual_trajectory.npy"
                historical_metrics = load_strict_json(historical_metrics_path)
                summary_row = execution_by_key[(case_id, k, method)]
                if historical_metrics != summary_row["controller_metrics"]:
                    raise ValueError(f"historical metric summary mismatch: {case_id}/k{k}/{method}")
                with historical_commands_path.open(newline="", encoding="utf-8") as stream:
                    command_rows = list(csv.DictReader(stream))
                if len(command_rows) < 4:
                    raise ValueError("historical command trace is too short")
                effective_boundary = [
                    float(command_rows[0]["actual_x"]),
                    float(command_rows[0]["actual_y"]),
                    float(command_rows[0]["actual_yaw"]),
                ]
                branches.append(
                    {
                        "case_id": case_id,
                        "entry_index": k,
                        "method": method,
                        "candidate_path": str(candidate_path),
                        "candidate_sha256": candidate_hash,
                        "candidate_shape": list(candidate.shape),
                        "geometric_metrics": load_strict_json(geometric_path),
                        "geometric_metrics_sha256": sha256_file(geometric_path),
                        "historical_controller_metrics": historical_metrics,
                        "historical_controller_metrics_sha256": sha256_file(
                            historical_metrics_path
                        ),
                        "historical_command_trace_path": str(historical_commands_path),
                        "historical_command_trace_sha256": sha256_file(
                            historical_commands_path
                        ),
                        "historical_actual_path": str(historical_actual_path),
                        "historical_actual_sha256": sha256_file(historical_actual_path),
                        "historical_follower_effective_boundary_se2": effective_boundary,
                    }
                )
    return {
        "historical_run": str(root),
        "run_id": root.name,
        "top_level_sha256": top_hashes,
        "strict_validation": validation,
        "source_selection": source_selection,
        "branch_count": len(branches),
        "branches": branches,
    }


def first_command_invariant(
    historical_first_desired: ArrayLike,
    reevaluated_first_desired: ArrayLike,
    *,
    tolerance: float,
) -> dict[str, Any]:
    historical = np.asarray(historical_first_desired, dtype=np.float64)
    reevaluated = np.asarray(reevaluated_first_desired, dtype=np.float64)
    if historical.shape != (2,) or reevaluated.shape != (2,):
        raise ValueError("first desired commands must have shape (2,)")
    if not np.all(np.isfinite(historical)) or not np.all(np.isfinite(reevaluated)):
        raise ValueError("first desired commands must be finite")
    limit = _positive(tolerance, "invariant tolerance")
    difference = reevaluated - historical
    checks = np.abs(difference) <= limit
    return {
        "passed": bool(np.all(checks)),
        "tolerance": limit,
        "historical_first_desired_v_omega": historical.tolist(),
        "reevaluated_first_desired_v_omega": reevaluated.tolist(),
        "signed_difference_v_omega": difference.tolist(),
        "absolute_difference_v_omega": np.abs(difference).tolist(),
        "checks": {"v": bool(checks[0]), "omega": bool(checks[1])},
    }


def offline_first_command_invariant(
    branch: Mapping[str, Any], follower_values: Mapping[str, Any], *, tolerance: float
) -> dict[str, Any]:
    candidate = np.load(branch["candidate_path"], allow_pickle=False)
    boundary = np.asarray(branch["historical_follower_effective_boundary_se2"])
    command = TrajectoryFollower(candidate, follower_config(follower_values)).forward(boundary)
    actual = [command.linear_velocity_mps, command.angular_velocity_rps]
    expected = branch["historical_controller_metrics"]["fresh_first_command_v_omega"]
    return first_command_invariant(expected, actual, tolerance=tolerance)


def body_interval_motion(start_pose: ArrayLike, end_pose: ArrayLike, duration_s: float) -> FloatArray:
    start = np.asarray(start_pose, dtype=np.float64)
    end = np.asarray(end_pose, dtype=np.float64)
    dt = _positive(duration_s, "interval duration")
    if start.shape != (3,) or end.shape != (3,) or not np.all(np.isfinite([start, end])):
        raise ValueError("interval poses must be finite SE(2) vectors")
    delta = end[:2] - start[:2]
    forward = np.asarray([math.cos(float(start[2])), math.sin(float(start[2]))])
    return np.asarray(
        [float(delta @ forward / dt), float(wrap_angle(end[2] - start[2]) / dt)],
        dtype=np.float64,
    )


def control_interval_body(poses: ArrayLike, control_dt_s: float) -> FloatArray:
    values = validate_se2_trajectory(poses, name="control interval poses")
    dt = _positive(control_dt_s, "control_dt_s")
    if values.shape[0] < 2:
        raise ValueError("control interval body motion requires two poses")
    return np.asarray(
        [body_interval_motion(values[index], values[index + 1], dt) for index in range(len(values) - 1)]
    )


def transition_change_metrics(
    pre_switch: ArrayLike,
    post_switch: ArrayLike,
    *,
    control_dt_s: float,
    window_size: int = 3,
) -> dict[str, Any]:
    pre = np.asarray(pre_switch, dtype=np.float64)
    post = _finite_array("post_switch", post_switch, columns=2)
    if pre.shape != (2,) or not np.all(np.isfinite(pre)):
        raise ValueError("pre_switch must be a finite body command/motion")
    if window_size < 1 or post.shape[0] < window_size:
        raise ValueError("post-switch sequence is shorter than the requested window")
    sequence = np.vstack((pre, post[:window_size]))
    deltas = np.diff(sequence, axis=0)
    dt = _positive(control_dt_s, "control_dt_s")
    return {
        "pre_switch_v_omega": pre.tolist(),
        "post_switch_first_v_omega": post[0].tolist(),
        "post_switch_first3_v_omega": post[:window_size].tolist(),
        "delta_v_signed": float(deltas[0, 0]),
        "delta_omega_signed": float(deltas[0, 1]),
        "delta_v_abs": abs(float(deltas[0, 0])),
        "delta_omega_abs": abs(float(deltas[0, 1])),
        "consecutive_delta_v_signed": deltas[:, 0].tolist(),
        "consecutive_delta_omega_signed": deltas[:, 1].tolist(),
        "max3_delta_v_abs": float(np.max(np.abs(deltas[:, 0]))),
        "max3_delta_omega_abs": float(np.max(np.abs(deltas[:, 1]))),
        "mean3_delta_v_abs": float(np.mean(np.abs(deltas[:, 0]))),
        "mean3_delta_omega_abs": float(np.mean(np.abs(deltas[:, 1]))),
        "immediate_discrete_slew_v_mps2": float(deltas[0, 0] / dt),
        "immediate_discrete_slew_omega_rps2": float(deltas[0, 1] / dt),
        "window_size": int(window_size),
        "slew_semantics": "discrete interval-to-interval change divided by control_dt; not continuous physical acceleration",
    }


def measured_transition_without_reset_crossing(
    *,
    old_interval_start: ArrayLike,
    old_interval_end: ArrayLike,
    old_interval_duration_s: float,
    post_control_poses: ArrayLike,
    control_dt_s: float,
    window_size: int = 3,
) -> dict[str, Any]:
    """Measure OLD and post-switch intervals separately; never differentiate the reset jump."""

    old = body_interval_motion(old_interval_start, old_interval_end, old_interval_duration_s)
    post = control_interval_body(post_control_poses, control_dt_s)
    result = transition_change_metrics(
        old, post, control_dt_s=control_dt_s, window_size=window_size
    )
    result["interval_semantics"] = (
        "OLD motion is the final pre-reset replay interval; post motion uses intervals beginning "
        "at exact-reset B; no derivative spans B_reproduced to B_saved"
    )
    result["old_interval_duration_s"] = float(old_interval_duration_s)
    return result


def historical_nominal_post_metrics(
    *, candidate: ArrayLike, actual: ArrayLike, desired: ArrayLike, control_dt_s: float
) -> dict[str, Any]:
    reference = validate_se2_trajectory(candidate, name="historical candidate")
    poses = validate_se2_trajectory(actual, name="historical nominal actual")
    commands = _finite_array("historical desired", desired, columns=2)
    count = min(poses.shape[0] - 1, commands.shape[0])
    if count < 3:
        raise ValueError("historical nominal trace needs at least three intervals")
    measured = control_interval_body(poses[: count + 1], control_dt_s)
    error = commands[:count] - measured
    nearest = nearest_polyline_samples(reference, poses)
    return {
        "interval_count": count,
        "measured_body_first3_v_omega": measured[:3].tolist(),
        "desired_v_vs_measured_v_rmse_mps": _rmse(error[:, 0]),
        "desired_omega_vs_measured_omega_rmse_rps": _rmse(error[:, 1]),
        "position_mean_m": float(np.mean(nearest["distance_m"])),
        "position_rmse_m": _rmse(nearest["distance_m"]),
        "position_max_m": float(np.max(nearest["distance_m"])),
        "yaw_rmse_rad": _rmse(nearest["yaw_error_rad"]),
        "yaw_max_abs_rad": float(np.max(np.abs(nearest["yaw_error_rad"]))),
        "sampling_semantics": "historical nominal EXP-02B control-time poses and desired commands at 0.1 s; no wheel telemetry was stored",
    }


def command_level_metrics(
    *,
    old_desired: ArrayLike,
    post_desired: ArrayLike,
    post_executed: ArrayLike,
    old_measured: ArrayLike,
    post_measured: ArrayLike,
    control_dt_s: float,
    window_size: int,
) -> dict[str, Any]:
    desired = _finite_array("post desired", post_desired, columns=2)
    executed = _finite_array("post executed", post_executed, columns=2)
    measured = _finite_array("post measured", post_measured, columns=2)
    if desired.shape != executed.shape or measured.shape[0] < window_size:
        raise ValueError("desired/executed/measured command traces are inconsistent")
    correction = executed - desired
    result = {
        "desired": transition_change_metrics(
            old_desired, desired, control_dt_s=control_dt_s, window_size=window_size
        ),
        "executed": transition_change_metrics(
            old_desired, executed, control_dt_s=control_dt_s, window_size=window_size
        ),
        "measured": transition_change_metrics(
            old_measured, measured, control_dt_s=control_dt_s, window_size=window_size
        ),
        "desired_to_executed": {
            "first_correction_v_omega": correction[0].tolist(),
            "maximum_absolute_correction_v_mps": float(np.max(np.abs(correction[:, 0]))),
            "maximum_absolute_correction_omega_rps": float(np.max(np.abs(correction[:, 1]))),
        },
        "command_level_semantics": {
            "desired": "TrajectoryFollower output",
            "executed": "frozen calibrated controller output sent to DifferentialController",
            "measured": "body motion measured over separate physical control intervals",
            "executed_switch_warning": "OLD is nominal and post-switch is calibrated with reset PI state; executed discontinuity is not a production end-to-end metric",
        },
    }
    assert_finite_json_tree(result)
    return result


def compare_method_pairs(
    branch_rows: Sequence[Mapping[str, Any]],
    *,
    method: str,
    baseline: str,
    metric_paths: Sequence[tuple[str, ...]],
    tie_tolerance: float,
) -> dict[str, Any]:
    tolerance = _positive(tie_tolerance, "pairwise tie tolerance")
    indexed = {
        (row["case_id"], int(row["entry_index"]), row["method"]): row
        for row in branch_rows
    }

    def nested(row: Mapping[str, Any], path: tuple[str, ...]) -> float:
        value: Any = row
        for key in path:
            value = value[key]
        result = float(value)
        if not math.isfinite(result):
            raise ValueError(f"non-finite pairwise metric: {'.'.join(path)}")
        return result

    rows: list[dict[str, Any]] = []
    counts = {"better": 0, "worse": 0, "tie": 0}
    for case_id in CASE_IDS:
        for k in ENTRY_INDICES:
            left = indexed[(case_id, k, method)]
            right = indexed[(case_id, k, baseline)]
            metrics = {}
            for path in metric_paths:
                name = ".".join(path)
                method_value = nested(left, path)
                baseline_value = nested(right, path)
                delta = method_value - baseline_value
                outcome = "tie" if abs(delta) <= tolerance else "better" if delta < 0.0 else "worse"
                counts[outcome] += 1
                metrics[name] = {
                    "method": method_value,
                    "baseline": baseline_value,
                    "method_minus_baseline": delta,
                    "outcome_lower_is_better": outcome,
                }
            rows.append(
                {"case_id": case_id, "entry_index": k, "method": method, "baseline": baseline, "metrics": metrics}
            )
    return {
        "method": method,
        "baseline": baseline,
        "pair_count": len(rows),
        "metric_comparison_counts": counts,
        "rows": rows,
    }


def final_interpretation(
    *,
    invariant_passed: bool,
    graph_vs_raw_consistent: bool,
    graph_vs_rigid_consistent: bool,
    physical_tracking_improved: bool,
    physical_method_ranking_materially_changed: bool,
) -> dict[str, Any]:
    if not invariant_passed:
        return {
            "primary_status": "PROTOCOL_INVARIANT_FAILURE",
            "secondary_status": [],
            "scientific_interpretation_permitted": False,
        }
    rescued = graph_vs_raw_consistent and graph_vs_rigid_consistent
    secondary: list[str] = []
    if physical_tracking_improved and not rescued:
        secondary.append("CALIBRATION_IMPROVES_PHYSICAL_EXECUTION_BUT_NOT_FORMULATION")
    if physical_method_ranking_materially_changed:
        secondary.append("EXECUTION_LAYER_MATERIALLY_CHANGES_PHYSICAL_METHOD_RANKING")
    return {
        "primary_status": (
            "EXP02B_FORMULATION_CONCLUSION_CHANGED"
            if rescued
            else "EXP02B_FORMULATION_CONCLUSION_UNCHANGED"
        ),
        "secondary_status": secondary,
        "scientific_interpretation_permitted": True,
        "graph_rescued": rescued,
        "graph_vs_raw_consistently_better": graph_vs_raw_consistent,
        "graph_vs_rigid_consistently_better": graph_vs_rigid_consistent,
    }


def write_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    assert_finite_json_tree(value)
    with target.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return target


def validate_branch_output(path: str | Path, expected_candidate_sha256: str) -> dict[str, Any]:
    root = Path(path)
    required = (
        "actual_trajectory.npy",
        "telemetry.csv",
        "desired_commands.csv",
        "executed_commands.csv",
        "measured_body.csv",
        "controller_metrics.json",
        "tracking_metrics.json",
        "metadata.json",
        "raw_provenance.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"EXP-02B-R branch missing files: {missing}")
    actual = validate_se2_trajectory(
        np.load(root / "actual_trajectory.npy", allow_pickle=False), name="actual trajectory"
    )
    provenance = load_strict_json(root / "raw_provenance.json")
    for name, digest in provenance["sha256"].items():
        if sha256_file(root / name) != digest:
            raise ValueError(f"EXP-02B-R branch raw hash mismatch: {name}")
    metadata = load_strict_json(root / "metadata.json")
    if metadata["candidate_sha256"] != expected_candidate_sha256:
        raise ValueError("EXP-02B-R branch candidate hash mismatch")
    if metadata["calibrated_controller_state_at_switch"] != "RESET":
        raise ValueError("EXP-02B-R branch controller reset semantics changed")
    controller = load_strict_json(root / "controller_metrics.json")
    tracking = load_strict_json(root / "tracking_metrics.json")
    if not controller["first_desired_invariant"]["passed"]:
        raise ValueError("EXP-02B-R first desired invariant failed")
    for csv_name in required[1:5]:
        with (root / csv_name).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise ValueError(f"empty EXP-02B-R CSV: {csv_name}")
        for row in rows:
            for key, value in row.items():
                if key in {"phase", "segment", "goal_reached", "saturated", "sign_protection_event"}:
                    continue
                if value != "" and not math.isfinite(float(value)):
                    raise ValueError(f"non-finite CSV value: {csv_name}/{key}")
    assert_finite_json_tree(controller)
    assert_finite_json_tree(tracking)
    return {"valid": True, "sample_count": int(actual.shape[0]), "candidate_sha256": expected_candidate_sha256}
