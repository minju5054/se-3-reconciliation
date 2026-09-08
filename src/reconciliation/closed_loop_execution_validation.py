"""Pure Stage 0-E same-reference closed-loop validation helpers.

All reference tracking metrics are spatial/progress based.  Reference rows are
never assigned an intrinsic LightNav time base.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.controller_validation import WHEEL_LABELS, estimate_body_velocities
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import (
    generate_reference_trajectory,
    segments_from_config,
    validate_se2_trajectory,
)


FloatArray = NDArray[np.float64]
MODES = ("nominal", "calibrated")
CONTROLLED_IDS = (
    "straight",
    "gentle_left",
    "gentle_right",
    "strong_left",
    "strong_right",
    "s_curve",
    "straight_turn_straight",
)
TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "control_index",
    "phase",
    "nearest_index",
    "target_index",
    "goal_reached",
    "desired_v_mps",
    "desired_omega_rps",
    "executed_v_mps",
    "executed_omega_rps",
    "measured_v_mps",
    "measured_omega_rps",
    *(f"target_{label}_rad_s" for label in WHEEL_LABELS),
    *(f"measured_{label}_rad_s" for label in WHEEL_LABELS),
    "actual_x",
    "actual_y",
    "actual_yaw",
    "nearest_reference_distance_m",
    "nearest_reference_yaw_error_rad",
    "pi_correction_rps",
    "integral_error_rad",
    "saturated",
    "sign_protection_event",
)


def _finite_array(name: str, value: ArrayLike, shape: tuple[int, ...]) -> FloatArray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result.copy()


def _rmse(values: FloatArray) -> float:
    if values.size == 0:
        raise ValueError("RMSE requires at least one sample")
    return float(np.sqrt(np.mean(np.square(values))))


@dataclass(frozen=True, slots=True)
class ClosedLoopTelemetry:
    reference_trajectory: FloatArray
    actual_trajectory: FloatArray
    sim_times_s: FloatArray
    control_indices: NDArray[np.int64]
    nearest_indices: NDArray[np.int64]
    target_indices: NDArray[np.int64]
    goal_reached: NDArray[np.bool_]
    desired_body: FloatArray
    executed_body: FloatArray
    target_wheels: FloatArray
    measured_wheels: FloatArray
    pi_correction_rps: FloatArray
    integral_error_rad: FloatArray
    saturated: NDArray[np.bool_]
    sign_protection_events: NDArray[np.bool_]

    def __post_init__(self) -> None:
        reference = validate_se2_trajectory(
            self.reference_trajectory, name="reference_trajectory"
        )
        actual = validate_se2_trajectory(self.actual_trajectory, name="actual_trajectory")
        count = actual.shape[0]
        times = _finite_array("sim_times_s", self.sim_times_s, (count,))
        controls = np.asarray(self.control_indices, dtype=np.int64)
        nearest = np.asarray(self.nearest_indices, dtype=np.int64)
        target = np.asarray(self.target_indices, dtype=np.int64)
        reached = np.asarray(self.goal_reached, dtype=np.bool_)
        desired = _finite_array("desired_body", self.desired_body, (count, 2))
        executed = _finite_array("executed_body", self.executed_body, (count, 2))
        targets = _finite_array("target_wheels", self.target_wheels, (count, 4))
        measured = _finite_array("measured_wheels", self.measured_wheels, (count, 4))
        correction = _finite_array("pi_correction_rps", self.pi_correction_rps, (count,))
        integral = _finite_array("integral_error_rad", self.integral_error_rad, (count,))
        saturated = np.asarray(self.saturated, dtype=np.bool_)
        sign_events = np.asarray(self.sign_protection_events, dtype=np.bool_)
        for name, values in (
            ("control_indices", controls),
            ("nearest_indices", nearest),
            ("target_indices", target),
            ("goal_reached", reached),
            ("saturated", saturated),
            ("sign_protection_events", sign_events),
        ):
            if values.shape != (count,):
                raise ValueError(f"{name} must have shape ({count},)")
        if not np.isclose(times[0], 0.0, atol=1e-12) or np.any(np.diff(times) <= 0.0):
            raise ValueError("sim_times_s must start at zero and increase strictly")
        if controls[0] != -1 or np.any(np.diff(controls) < 0):
            raise ValueError("control_indices must start at -1 and be non-decreasing")
        if np.any(nearest < 0) or np.any(nearest >= reference.shape[0]):
            raise ValueError("nearest_indices contains an out-of-range value")
        if np.any(target < 0) or np.any(target >= reference.shape[0]):
            raise ValueError("target_indices contains an out-of-range value")
        if np.any(np.diff(nearest) < 0):
            raise ValueError("nearest progress must be monotonic")
        for name, value in (
            ("reference_trajectory", reference),
            ("actual_trajectory", actual),
            ("sim_times_s", times),
            ("control_indices", controls.copy()),
            ("nearest_indices", nearest.copy()),
            ("target_indices", target.copy()),
            ("goal_reached", reached.copy()),
            ("desired_body", desired),
            ("executed_body", executed),
            ("target_wheels", targets),
            ("measured_wheels", measured),
            ("pi_correction_rps", correction),
            ("integral_error_rad", integral),
            ("saturated", saturated.copy()),
            ("sign_protection_events", sign_events.copy()),
        ):
            object.__setattr__(self, name, value)

    @property
    def sample_count(self) -> int:
        return int(self.actual_trajectory.shape[0])


def generate_controlled_references(
    config: Mapping[str, Any],
) -> dict[str, FloatArray]:
    """Generate copied deterministic arbitrary-N SE(2) held-out references."""

    dt = float(config["simulation"]["control_dt_s"])
    initial = config["simulation"]["initial_pose_se2"]
    result: dict[str, FloatArray] = {}
    for scenario in config["controlled_paths"]["scenarios"]:
        identifier = str(scenario["id"])
        if identifier in result:
            raise ValueError(f"duplicate controlled path id: {identifier}")
        result[identifier] = generate_reference_trajectory(
            initial, segments_from_config(scenario["segments"]), dt
        ).poses
    if tuple(result) != CONTROLLED_IDS:
        raise ValueError(f"controlled paths must be ordered as {CONTROLLED_IDS}")
    return result


def validate_controlled_geometry(references: Mapping[str, ArrayLike]) -> dict[str, Any]:
    paths = {
        key: validate_se2_trajectory(value, name=f"controlled path {key}")
        for key, value in references.items()
    }
    if tuple(paths) != CONTROLLED_IDS:
        raise ValueError(f"controlled paths must be ordered as {CONTROLLED_IDS}")
    yaw_steps = {key: np.asarray(wrap_angle(np.diff(path[:, 2]))) for key, path in paths.items()}
    checks = {
        "straight_has_negligible_turn": bool(np.max(np.abs(yaw_steps["straight"])) < 1e-12),
        "left_paths_turn_ccw": bool(
            paths["gentle_left"][-1, 2] > 0.0 and paths["strong_left"][-1, 2] > 0.0
        ),
        "right_paths_turn_cw": bool(
            paths["gentle_right"][-1, 2] < 0.0 and paths["strong_right"][-1, 2] < 0.0
        ),
        "s_curve_changes_direction": bool(
            np.any(yaw_steps["s_curve"] > 0.0) and np.any(yaw_steps["s_curve"] < 0.0)
        ),
        "straight_turn_straight_has_zero_and_positive_curvature": bool(
            np.any(np.abs(yaw_steps["straight_turn_straight"]) < 1e-12)
            and np.any(yaw_steps["straight_turn_straight"] > 0.0)
        ),
    }
    if not all(checks.values()):
        raise ValueError(f"invalid controlled path geometry: {checks}")
    return {
        "valid": True,
        "checks": checks,
        "sample_counts": {key: int(value.shape[0]) for key, value in paths.items()},
        "path_lengths_m": {
            key: float(np.sum(np.linalg.norm(np.diff(value[:, :2], axis=0), axis=1)))
            for key, value in paths.items()
        },
    }


def load_frozen_stage0d_candidate(
    run_directory: str | Path,
    provenance: Mapping[str, Any],
    *,
    repository_root: str | Path | None = None,
) -> dict[str, Any]:
    """Load and verify the completed Stage 0-D candidate without copying gains."""

    root = Path(run_directory).resolve()
    expected = {
        "calibration_model.json": str(provenance["calibration_model_sha256"]),
        "metadata.json": str(provenance["metadata_sha256"]),
        "config_snapshot.yaml": str(provenance["config_snapshot_sha256"]),
    }
    for name, digest in expected.items():
        if sha256_file(root / name) != digest:
            raise ValueError(f"frozen Stage 0-D provenance mismatch: {name}")
    model = load_strict_json(root / "calibration_model.json")
    metadata = load_strict_json(root / "metadata.json")
    if metadata.get("run_id") != provenance["run_id"] or metadata.get("smoke"):
        raise ValueError("Stage 0-D source must be the frozen completed primary run")
    feedback = model.get("feedback_selection") or {}
    if feedback.get("selected_candidate_id") != provenance["selected_candidate_id"]:
        raise ValueError("frozen Stage 0-D selected candidate changed")
    if model.get("held_out_data_used_for_fitting_or_tuning") is not False:
        raise ValueError("Stage 0-D candidate provenance does not exclude held-out tuning")
    parameters = model.get("selected_parameters")
    if not isinstance(parameters, Mapping):
        raise ValueError("Stage 0-D model has no selected parameters")
    assert_finite_json_tree(parameters)
    if repository_root is not None:
        code = model["runtime_controller_code"]
        if sha256_file(Path(repository_root) / code["path"]) != code["sha256"]:
            raise ValueError("Stage 0-D runtime controller code hash changed")
    return {
        "run_directory": str(root),
        "model_sha256": expected["calibration_model.json"],
        "model_type": model["model_type"],
        "selected_candidate_id": feedback["selected_candidate_id"],
        "selected_parameters": dict(parameters),
        "runtime_controller_config": dict(model["runtime_controller_config"]),
        "stage0d_validation_status": load_strict_json(
            root / "final_execution_layer_validation.json"
        )["decision"]["status"],
        "held_out_data_used_for_fitting_or_tuning": False,
    }


def compute_closed_loop_metrics(telemetry: ClosedLoopTelemetry) -> dict[str, Any]:
    """Separate spatial reference, body execution, wheel, and controller metrics."""

    nearest = nearest_polyline_samples(
        telemetry.reference_trajectory, telemetry.actual_trajectory
    )
    position = nearest["distance_m"]
    yaw_error = nearest["yaw_error_rad"]
    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    measured_body = np.column_stack((measured_v, measured_omega))
    active = np.arange(telemetry.sample_count) > 0
    desired_error = telemetry.desired_body[active] - measured_body[active]
    executed_error = telemetry.executed_body[active] - measured_body[active]
    wheel_error = telemetry.target_wheels[active] - telemetry.measured_wheels[active]
    control_rows = np.flatnonzero(
        (telemetry.control_indices >= 0)
        & np.concatenate(([True], np.diff(telemetry.control_indices) != 0))
    )
    desired_controls = telemetry.desired_body[control_rows]
    if desired_controls.shape[0] > 1:
        deltas = np.diff(desired_controls, axis=0)
        dt = np.diff(telemetry.sim_times_s[control_rows])[:, None]
        slew = deltas / dt
    else:
        deltas = np.zeros((1, 2), dtype=np.float64)
        slew = np.zeros((1, 2), dtype=np.float64)
    reference_count = telemetry.reference_trajectory.shape[0]
    progress_ratio = (
        float(np.max(telemetry.nearest_indices)) / float(reference_count - 1)
        if reference_count > 1
        else 1.0
    )
    final_goal = telemetry.reference_trajectory[-1]
    final_actual = telemetry.actual_trajectory[-1]
    result = {
        "sample_count": telemetry.sample_count,
        "control_step_count": int(control_rows.size),
        "goal_reached": bool(np.any(telemetry.goal_reached)),
        "time_to_goal_or_timeout_s": float(telemetry.sim_times_s[-1]),
        "progress_completion_ratio": progress_ratio,
        "progress_monotonic": bool(np.all(np.diff(telemetry.nearest_indices) >= 0)),
        "position_rmse_m": _rmse(position),
        "mean_position_error_m": float(np.mean(position)),
        "max_position_error_m": float(np.max(position)),
        "final_position_error_m": float(np.linalg.norm(final_goal[:2] - final_actual[:2])),
        "yaw_rmse_rad": _rmse(yaw_error),
        "mean_absolute_yaw_error_rad": float(np.mean(np.abs(yaw_error))),
        "max_yaw_error_rad": float(np.max(np.abs(yaw_error))),
        "final_yaw_error_rad": abs(float(wrap_angle(final_goal[2] - final_actual[2]))),
        "desired_v_vs_measured_v_rmse_mps": _rmse(desired_error[:, 0]),
        "desired_omega_vs_measured_omega_rmse_rps": _rmse(desired_error[:, 1]),
        "executed_v_vs_measured_v_rmse_mps": _rmse(executed_error[:, 0]),
        "executed_omega_vs_measured_omega_rmse_rps": _rmse(executed_error[:, 1]),
        "wheel_target_vs_measured_rmse_rad_s": _rmse(wheel_error),
        "wheel_target_vs_measured_rmse_by_corner_rad_s": {
            label: _rmse(wheel_error[:, index])
            for index, label in enumerate(WHEEL_LABELS)
        },
        "desired_command_delta_v_mean_abs_mps": float(np.mean(np.abs(deltas[:, 0]))),
        "desired_command_delta_v_max_abs_mps": float(np.max(np.abs(deltas[:, 0]))),
        "desired_command_delta_omega_mean_abs_rps": float(np.mean(np.abs(deltas[:, 1]))),
        "desired_command_delta_omega_max_abs_rps": float(np.max(np.abs(deltas[:, 1]))),
        "desired_command_v_slew_max_abs_mps2": float(np.max(np.abs(slew[:, 0]))),
        "desired_command_omega_slew_max_abs_rps2": float(np.max(np.abs(slew[:, 1]))),
        "active_saturation_fraction": float(np.mean(telemetry.saturated[control_rows])),
        "maximum_abs_pi_correction_rps": float(
            np.max(np.abs(telemetry.pi_correction_rps[control_rows]))
        ),
        "integral_state_min_rad": float(np.min(telemetry.integral_error_rad[control_rows])),
        "integral_state_max_rad": float(np.max(telemetry.integral_error_rad[control_rows])),
        "sign_protection_event_count": int(
            np.sum(telemetry.sign_protection_events[control_rows])
        ),
        "numerically_stable": True,
        "metric_semantics": {
            "trajectory": (
                "actual poses projected to nearest reference-polyline segments; yaw uses "
                "wrapped interpolation on that segment; not time-aligned waypoint error"
            ),
            "body": (
                "finite-difference measured body motion over the interval ending at each "
                "physics sample, compared separately with desired and executed commands"
            ),
            "command_levels": (
                "desired is TrajectoryFollower output; executed is low-level command after "
                "the frozen candidate (equal to desired in nominal mode); measured is motion"
            ),
            "wheel": "applied target minus measured articulation velocity in canonical order",
        },
    }
    assert_finite_json_tree(result)
    return result


def evaluate_scenario(
    trial_rows: Sequence[Mapping[str, Any]], criteria: Mapping[str, Any]
) -> dict[str, Any]:
    if not trial_rows:
        raise ValueError("scenario evaluation requires trials")
    repetitions = int(criteria["repetitions"])
    if len(trial_rows) != repetitions:
        raise ValueError(f"scenario requires exactly {repetitions} repetitions")
    scalar_names = (
        "position_rmse_m",
        "yaw_rmse_rad",
        "final_position_error_m",
        "final_yaw_error_rad",
    )
    stats = {
        name: {
            "mean": float(np.mean([float(row[name]) for row in trial_rows])),
            "std": float(np.std([float(row[name]) for row in trial_rows])),
            "max": float(np.max([float(row[name]) for row in trial_rows])),
        }
        for name in scalar_names
    }
    goal_successes = sum(bool(row["goal_reached"]) for row in trial_rows)
    checks = {
        "goal_success_rate": goal_successes >= int(criteria["minimum_goal_successes"]),
        "position_rmse": stats["position_rmse_m"]["max"]
        < float(criteria["position_rmse_m"]),
        "final_position_error": stats["final_position_error_m"]["max"]
        < float(criteria["final_position_error_m"]),
        "yaw_rmse": stats["yaw_rmse_rad"]["max"] < float(criteria["yaw_rmse_rad"]),
        "final_yaw_error": stats["final_yaw_error_rad"]["max"]
        < float(criteria["final_yaw_error_rad"]),
        "numerical_stability": all(bool(row["numerically_stable"]) for row in trial_rows),
        "saturation": max(float(row["active_saturation_fraction"]) for row in trial_rows)
        <= float(criteria["maximum_active_saturation_fraction"]),
        "progress_monotonic": all(bool(row["progress_monotonic"]) for row in trial_rows),
        "no_pathological_stuck": all(
            bool(row["goal_reached"])
            or float(row["progress_completion_ratio"])
            >= float(criteria["minimum_progress_completion_ratio"])
            for row in trial_rows
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "repetitions": repetitions,
        "goal_successes": goal_successes,
        "goal_success_rate": goal_successes / repetitions,
        "repeatability": stats,
        "maximum_active_saturation_fraction": max(
            float(row["active_saturation_fraction"]) for row in trial_rows
        ),
        "minimum_progress_completion_ratio": min(
            float(row["progress_completion_ratio"]) for row in trial_rows
        ),
        "trial_metrics": [dict(row) for row in trial_rows],
    }


def evaluate_controlled_group(
    scenarios: Mapping[str, Mapping[str, Any]],
    geometry: Mapping[str, str],
    criteria: Mapping[str, Any],
) -> dict[str, Any]:
    if tuple(scenarios) != CONTROLLED_IDS:
        raise ValueError("controlled scenario result set is incomplete or reordered")
    passing = [key for key, value in scenarios.items() if bool(value["passed"])]
    left = [key for key in passing if geometry[key] == "left"]
    right = [key for key in passing if geometry[key] == "right"]
    direction = [key for key in passing if geometry[key] == "direction_change"]
    checks = {
        "minimum_scenario_count": len(passing)
        >= int(criteria["controlled_minimum_passing_scenarios"]),
        "left_curved_path": bool(left),
        "right_curved_path": bool(right),
        "direction_change_path": bool(direction),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "passing_scenarios": passing,
        "passing_scenario_count": len(passing),
        "scenario_count": len(scenarios),
        "scenarios": {key: dict(value) for key, value in scenarios.items()},
    }


def evaluate_exp02b_group(
    scenarios: Mapping[str, Mapping[str, Any]], criteria: Mapping[str, Any]
) -> dict[str, Any]:
    passing = [key for key, value in scenarios.items() if bool(value["passed"])]
    expected = int(criteria["exp02b_case_count"])
    if len(scenarios) != expected:
        raise ValueError(f"EXP-02B group requires {expected} cases")
    return {
        "passed": len(passing) >= int(criteria["exp02b_minimum_passing_cases"]),
        "passing_cases": passing,
        "passing_case_count": len(passing),
        "case_count": len(scenarios),
        "scenarios": {key: dict(value) for key, value in scenarios.items()},
    }


def compare_modes(
    nominal: Mapping[str, Any], calibrated: Mapping[str, Any]
) -> dict[str, Any]:
    """Report paired aggregate deltas without assuming calibrated is superior."""

    keys = (
        "position_rmse_m",
        "yaw_rmse_rad",
        "final_position_error_m",
        "final_yaw_error_rad",
        "desired_v_vs_measured_v_rmse_mps",
        "desired_omega_vs_measured_omega_rmse_rps",
        "wheel_target_vs_measured_rmse_rad_s",
        "active_saturation_fraction",
    )
    return {
        key: {
            "nominal_mean": float(np.mean([float(row[key]) for row in nominal["trials"]])),
            "calibrated_mean": float(
                np.mean([float(row[key]) for row in calibrated["trials"]])
            ),
            "calibrated_minus_nominal": float(
                np.mean([float(row[key]) for row in calibrated["trials"]])
                - np.mean([float(row[key]) for row in nominal["trials"]])
            ),
        }
        for key in keys
    }


def final_platform_decision(
    *, nominal_passed: bool, calibrated_passed: bool
) -> dict[str, Any]:
    if nominal_passed and calibrated_passed:
        status = "BOTH_PASS_NOMINAL_SELECTED_FOR_SIMPLICITY"
        selected = "nominal"
    elif nominal_passed:
        status = "NOMINAL_PLATFORM_SELECTED"
        selected = "nominal"
    elif calibrated_passed:
        status = "CALIBRATED_PLATFORM_SELECTED"
        selected = "calibrated"
    else:
        status = "EXECUTION_PLATFORM_NOT_READY"
        selected = None
    return {
        "status": status,
        "selected_mode": selected,
        "nominal_passed": bool(nominal_passed),
        "calibrated_passed": bool(calibrated_passed),
        "selection_rule": (
            "absolute gates for controlled, Stage 0-B composite, and EXP-02B OLD-reference "
            "groups; when both pass, select the simpler nominal execution layer"
        ),
    }


def validate_stage0e_config(config: Mapping[str, Any]) -> dict[str, Any]:
    if config.get("stage") != "stage0-e-closed-loop-trajectory-execution-validation":
        raise ValueError("invalid Stage 0-E stage identifier")
    simulation = config["simulation"]
    physics_dt = float(simulation["physics_dt_s"])
    control_dt = float(simulation["control_dt_s"])
    ratio = round(control_dt / physics_dt)
    if ratio < 1 or not math.isclose(ratio * physics_dt, control_dt, abs_tol=1e-12):
        raise ValueError("control_dt_s must be an integer multiple of physics_dt_s")
    for key in ("physics_dt_s", "control_dt_s", "settling_duration_s", "spawn_height_m"):
        value = float(simulation[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{key} must be finite and positive")
    repetitions = int(config["controlled_paths"]["repetitions"])
    if repetitions < 3:
        raise ValueError("controlled paths require at least three repetitions")
    if int(config["stage0b_composite"]["repetitions"]) != repetitions:
        raise ValueError("all primary groups must use the same repetitions")
    if int(config["exp02b_old_reference"]["repetitions"]) != repetitions:
        raise ValueError("all primary groups must use the same repetitions")
    criteria = config["acceptance_criteria"]
    if int(criteria["repetitions"]) != repetitions:
        raise ValueError("acceptance repetitions differ from experiment repetitions")
    if int(criteria["controlled_scenario_count"]) != len(CONTROLLED_IDS):
        raise ValueError("controlled scenario gate count changed")
    for key in (
        "position_rmse_m",
        "final_position_error_m",
        "yaw_rmse_rad",
        "final_yaw_error_rad",
        "maximum_active_saturation_fraction",
        "minimum_progress_completion_ratio",
    ):
        value = float(criteria[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"acceptance criterion {key} must be finite and positive")
    references = generate_controlled_references(config)
    geometry = validate_controlled_geometry(references)
    assert_finite_json_tree(config)
    return {
        "valid": True,
        "physics_steps_per_control": int(ratio),
        "criteria_frozen": True,
        "held_out_controller_tuning_permitted": False,
        "historical_commands_reused": False,
        "controlled_geometry": geometry,
    }


def save_closed_loop_trial(
    destination: str | Path,
    telemetry: ClosedLoopTelemetry,
    metrics: Mapping[str, Any],
    metadata: Mapping[str, Any],
) -> dict[str, str]:
    root = Path(destination)
    assert_finite_json_tree(metrics)
    assert_finite_json_tree(metadata)
    root.mkdir(parents=True, exist_ok=False)
    raw = root / "raw"
    derived = root / "derived"
    raw.mkdir()
    derived.mkdir()
    np.save(raw / "actual_trajectory.npy", telemetry.actual_trajectory)
    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    nearest = nearest_polyline_samples(
        telemetry.reference_trajectory, telemetry.actual_trajectory
    )
    with (raw / "telemetry.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TELEMETRY_COLUMNS)
        for index in range(telemetry.sample_count):
            writer.writerow(
                [
                    index,
                    telemetry.sim_times_s[index],
                    int(telemetry.control_indices[index]),
                    "INITIAL" if index == 0 else "EXECUTION",
                    int(telemetry.nearest_indices[index]),
                    int(telemetry.target_indices[index]),
                    bool(telemetry.goal_reached[index]),
                    *telemetry.desired_body[index],
                    *telemetry.executed_body[index],
                    measured_v[index],
                    measured_omega[index],
                    *telemetry.target_wheels[index],
                    *telemetry.measured_wheels[index],
                    *telemetry.actual_trajectory[index],
                    nearest["distance_m"][index],
                    nearest["yaw_error_rad"][index],
                    telemetry.pi_correction_rps[index],
                    telemetry.integral_error_rad[index],
                    bool(telemetry.saturated[index]),
                    bool(telemetry.sign_protection_events[index]),
                ]
            )
    for path, value in (
        (derived / "metrics.json", metrics),
        (root / "metadata.json", metadata),
    ):
        with path.open("x", encoding="utf-8") as stream:
            json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    hashes = {
        "raw/actual_trajectory.npy": sha256_file(raw / "actual_trajectory.npy"),
        "raw/telemetry.csv": sha256_file(raw / "telemetry.csv"),
    }
    with (derived / "raw_provenance.json").open("x", encoding="utf-8") as stream:
        json.dump({"sha256": hashes}, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return hashes


def validate_closed_loop_trial(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    required = (
        "raw/actual_trajectory.npy",
        "raw/telemetry.csv",
        "derived/metrics.json",
        "derived/raw_provenance.json",
        "metadata.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"closed-loop trial is missing: {', '.join(missing)}")
    actual = validate_se2_trajectory(
        np.load(root / "raw/actual_trajectory.npy", allow_pickle=False), name="actual"
    )
    with (root / "raw/telemetry.csv").open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != TELEMETRY_COLUMNS:
            raise ValueError("telemetry columns differ from Stage 0-E schema")
        rows = list(reader)
    if len(rows) != actual.shape[0]:
        raise ValueError("telemetry row count differs from actual trajectory")
    boolean = {"goal_reached", "saturated", "sign_protection_event"}
    textual = {"phase"}
    for row in rows:
        for key, value in row.items():
            if key in boolean:
                if value not in ("True", "False"):
                    raise ValueError(f"invalid boolean telemetry value in {key}")
            elif key in textual:
                if value not in ("INITIAL", "EXECUTION"):
                    raise ValueError("invalid telemetry phase")
            elif not math.isfinite(float(value)):
                raise ValueError(f"non-finite telemetry value in {key}")
    provenance = load_strict_json(root / "derived/raw_provenance.json")
    for relative, digest in provenance["sha256"].items():
        if sha256_file(root / relative) != digest:
            raise ValueError(f"raw provenance hash changed: {relative}")
    metrics = load_strict_json(root / "derived/metrics.json")
    metadata = load_strict_json(root / "metadata.json")
    assert_finite_json_tree(metrics)
    assert_finite_json_tree(metadata)
    if metadata.get("evaluation_semantics") != "reference-matched closed-loop evaluation":
        raise ValueError("invalid Stage 0-E evaluation semantics")
    return {
        "valid": True,
        "mode": metadata["mode"],
        "scenario": metadata["scenario"],
        "repetition": metadata["repetition"],
        "sample_count": int(actual.shape[0]),
        "metrics": metrics,
    }
