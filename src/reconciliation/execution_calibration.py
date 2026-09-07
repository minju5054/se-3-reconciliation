"""Pure Stage 0-D execution-calibration metrics, fitting, and artifacts."""

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
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


FloatArray = NDArray[np.float64]
PHASES = ("INITIAL_STOP", "ACTIVE", "FINAL_STOP")
TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "control_index",
    "phase",
    "desired_v_mps",
    "desired_omega_rps",
    "executed_v_mps",
    "executed_omega_rps",
    "ideal_left_rad_s",
    "ideal_right_rad_s",
    *(f"target_{label}_rad_s" for label in WHEEL_LABELS),
    *(f"measured_{label}_rad_s" for label in WHEEL_LABELS),
    "actual_x",
    "actual_y",
    "actual_yaw",
    "measured_v_mps",
    "measured_omega_rps",
    "yaw_rate_error_rps",
    "saturated",
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


def _mae(values: FloatArray) -> float:
    if values.size == 0:
        raise ValueError("MAE requires at least one sample")
    return float(np.mean(np.abs(values)))


@dataclass(frozen=True, slots=True)
class ExecutionTelemetry:
    actual_trajectory: FloatArray
    sim_times_s: FloatArray
    control_indices: NDArray[np.int64]
    phases: tuple[str, ...]
    desired_body: FloatArray
    executed_body: FloatArray
    ideal_left_right: FloatArray
    target_wheels: FloatArray
    measured_wheels: FloatArray
    saturated: NDArray[np.bool_]

    def __post_init__(self) -> None:
        actual = validate_se2_trajectory(self.actual_trajectory, name="actual_trajectory")
        count = actual.shape[0]
        times = _finite_array("sim_times_s", self.sim_times_s, (count,))
        controls = np.asarray(self.control_indices, dtype=np.int64)
        phases = tuple(str(value) for value in self.phases)
        desired = _finite_array("desired_body", self.desired_body, (count, 2))
        executed = _finite_array("executed_body", self.executed_body, (count, 2))
        ideal = _finite_array("ideal_left_right", self.ideal_left_right, (count, 2))
        targets = _finite_array("target_wheels", self.target_wheels, (count, 4))
        measured = _finite_array("measured_wheels", self.measured_wheels, (count, 4))
        saturated = np.asarray(self.saturated, dtype=np.bool_)
        if controls.shape != (count,) or saturated.shape != (count,):
            raise ValueError("control_indices and saturated must match sample count")
        if len(phases) != count or any(value not in PHASES for value in phases):
            raise ValueError(f"phases must contain only {PHASES}")
        if not np.isclose(times[0], 0.0, atol=1e-12) or np.any(np.diff(times) <= 0.0):
            raise ValueError("sim_times_s must start at zero and increase strictly")
        if controls[0] != -1 or np.any(np.diff(controls) < 0):
            raise ValueError("control_indices must start at -1 and be non-decreasing")
        object.__setattr__(self, "actual_trajectory", actual)
        object.__setattr__(self, "sim_times_s", times)
        object.__setattr__(self, "control_indices", controls.copy())
        object.__setattr__(self, "phases", phases)
        object.__setattr__(self, "desired_body", desired)
        object.__setattr__(self, "executed_body", executed)
        object.__setattr__(self, "ideal_left_right", ideal)
        object.__setattr__(self, "target_wheels", targets)
        object.__setattr__(self, "measured_wheels", measured)
        object.__setattr__(self, "saturated", saturated.copy())

    @property
    def sample_count(self) -> int:
        return int(self.actual_trajectory.shape[0])


def ideal_trajectory_from_desired(telemetry: ExecutionTelemetry) -> FloatArray:
    """Integrate desired body commands with the explicit ending-interval convention."""

    poses = np.empty_like(telemetry.actual_trajectory)
    poses[0] = telemetry.actual_trajectory[0]
    for index in range(1, telemetry.sample_count):
        dt = float(telemetry.sim_times_s[index] - telemetry.sim_times_s[index - 1])
        v, omega = telemetry.desired_body[index]
        yaw = float(poses[index - 1, 2])
        poses[index, 0] = poses[index - 1, 0] + float(v) * math.cos(yaw) * dt
        poses[index, 1] = poses[index - 1, 1] + float(v) * math.sin(yaw) * dt
        poses[index, 2] = float(wrap_angle(yaw + float(omega) * dt))
    return poses


def _sign_alternations(values: FloatArray, threshold: float) -> int:
    selected = np.sign(values[np.abs(values) >= threshold])
    return int(np.sum(selected[1:] != selected[:-1])) if selected.size > 1 else 0


def compute_trial_metrics(
    telemetry: ExecutionTelemetry,
    *,
    wheel_radius_m: float,
    steady_state_tail_s: float,
    minimum_abs_measured_omega_rps: float,
) -> dict[str, Any]:
    """Measure one constant-command primitive without hiding transient behavior."""

    radius = float(wheel_radius_m)
    tail = float(steady_state_tail_s)
    omega_floor = float(minimum_abs_measured_omega_rps)
    if not all(math.isfinite(value) and value > 0.0 for value in (radius, tail, omega_floor)):
        raise ValueError("radius, steady tail, and measured-omega floor must be positive")
    phases = np.asarray(telemetry.phases)
    active = phases == "ACTIVE"
    if not np.any(active):
        raise ValueError("telemetry has no ACTIVE samples")
    active_times = telemetry.sim_times_s[active]
    steady = active & (telemetry.sim_times_s >= float(active_times[-1]) - tail)
    if not np.any(steady):
        raise ValueError("steady-state window is empty")

    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    desired = telemetry.desired_body
    executed = telemetry.executed_body
    v_error = desired[:, 0] - measured_v
    omega_error = desired[:, 1] - measured_omega
    wheel_error = telemetry.target_wheels - telemetry.measured_wheels
    desired_v = float(np.median(desired[active, 0]))
    desired_omega = float(np.median(desired[active, 1]))
    steady_v_mean = float(np.mean(measured_v[steady]))
    steady_omega_mean = float(np.mean(measured_omega[steady]))
    gain_v = steady_v_mean / desired_v if abs(desired_v) > 1e-12 else None
    gain_omega = steady_omega_mean / desired_omega if abs(desired_omega) > 1e-12 else None
    sign_correct = (
        bool(steady_omega_mean * desired_omega > 0.0)
        if abs(desired_omega) >= 0.15
        else None
    )
    left = np.mean(telemetry.measured_wheels[:, (0, 2)], axis=1)
    right = np.mean(telemetry.measured_wheels[:, (1, 3)], axis=1)
    effective_mask = steady & (np.abs(measured_omega) >= omega_floor)
    effective_width = radius * (right[effective_mask] - left[effective_mask]) / measured_omega[
        effective_mask
    ]
    effective_width = effective_width[np.isfinite(effective_width) & (effective_width > 0.0)]

    active_indices = np.flatnonzero(active)
    active_measured_omega = measured_omega[active]
    rise_time: float | None = None
    if abs(desired_omega) >= 0.15:
        reached = np.flatnonzero(
            np.sign(desired_omega) * active_measured_omega >= 0.9 * abs(desired_omega)
        )
        if reached.size:
            rise_time = float(
                telemetry.sim_times_s[active_indices[int(reached[0])]] - active_times[0]
            )
    omega_overshoot = (
        max(0.0, float(np.max(np.sign(desired_omega) * active_measured_omega)) - abs(desired_omega))
        if abs(desired_omega) >= 0.15
        else None
    )
    ideal = ideal_trajectory_from_desired(telemetry)
    return {
        "sample_count": telemetry.sample_count,
        "active_sample_count": int(np.sum(active)),
        "steady_state_sample_count": int(np.sum(steady)),
        "desired_v_mps": desired_v,
        "desired_omega_rps": desired_omega,
        "active_executed_v_mean_mps": float(np.mean(executed[active, 0])),
        "active_executed_omega_mean_rps": float(np.mean(executed[active, 1])),
        "v_mean_error_mps": float(np.mean(v_error[active])),
        "v_rmse_mps": _rmse(v_error[active]),
        "v_mae_mps": _mae(v_error[active]),
        "v_steady_state_gain": gain_v,
        "omega_mean_error_rps": float(np.mean(omega_error[active])),
        "omega_rmse_rps": _rmse(omega_error[active]),
        "omega_mae_rps": _mae(omega_error[active]),
        "omega_steady_state_gain": gain_omega,
        "steady_measured_v_mean_mps": steady_v_mean,
        "steady_measured_omega_mean_rps": steady_omega_mean,
        "steady_omega_mae_rps": _mae(omega_error[steady]),
        "steady_sign_correct": sign_correct,
        "wheel_target_measured_rmse_rad_s": _rmse(wheel_error[active]),
        "wheel_target_measured_rmse_by_corner_rad_s": {
            label: _rmse(wheel_error[active, index])
            for index, label in enumerate(WHEEL_LABELS)
        },
        "steady_measured_left_wheel_mean_rad_s": float(np.mean(left[steady])),
        "steady_measured_right_wheel_mean_rad_s": float(np.mean(right[steady])),
        "effective_track_width_sample_count": int(effective_width.size),
        "effective_track_width_mean_m": (
            float(np.mean(effective_width)) if effective_width.size else None
        ),
        "effective_track_width_median_m": (
            float(np.median(effective_width)) if effective_width.size else None
        ),
        "effective_track_width_std_m": (
            float(np.std(effective_width)) if effective_width.size else None
        ),
        "effective_track_width_min_m": (
            float(np.min(effective_width)) if effective_width.size else None
        ),
        "effective_track_width_max_m": (
            float(np.max(effective_width)) if effective_width.size else None
        ),
        "omega_rise_time_s": rise_time,
        "omega_overshoot_rps": omega_overshoot,
        "omega_settling_error_rps": abs(desired_omega - steady_omega_mean),
        "active_saturation_fraction": float(np.mean(telemetry.saturated[active])),
        "measured_omega_sign_alternations": _sign_alternations(
            active_measured_omega, omega_floor
        ),
        "ideal_endpoint_se2": ideal[-1].tolist(),
        "actual_endpoint_se2": telemetry.actual_trajectory[-1].tolist(),
        "endpoint_translation_error_m": float(
            np.linalg.norm(ideal[-1, :2] - telemetry.actual_trajectory[-1, :2])
        ),
        "endpoint_yaw_error_rad": abs(
            float(wrap_angle(ideal[-1, 2] - telemetry.actual_trajectory[-1, 2]))
        ),
        "metric_semantics": {
            "body": "desired body command minus finite-difference measured body motion",
            "wheel": "applied four-wheel target minus measured articulation velocity",
            "steady_state": f"last {tail:g} s of ACTIVE command samples",
            "effective_track_width": (
                "r*(mean_right_measured-mean_left_measured)/measured_body_omega; "
                f"undefined when |omega_measured| < {omega_floor:g} rad/s"
            ),
        },
    }


def fit_effective_width_model(
    trial_rows: Sequence[Mapping[str, Any]],
    *,
    physical_wheel_separation_m: float,
    maximum_feedforward_scale: float,
    assessment: Mapping[str, Any],
) -> dict[str, Any]:
    """Fit and cross-check one through-origin nominal yaw-response gain."""

    physical = float(physical_wheel_separation_m)
    maximum_scale = float(maximum_feedforward_scale)
    if not math.isfinite(physical) or physical <= 0.0 or maximum_scale <= 0.0:
        raise ValueError("physical separation and maximum scale must be positive")
    moving = [row for row in trial_rows if abs(float(row["desired_omega_rps"])) >= 0.15]
    if len(moving) < 2:
        raise ValueError("effective-width fit requires at least two moving trials")
    commanded = np.asarray([float(row["desired_omega_rps"]) for row in moving])
    measured = np.asarray([float(row["steady_measured_omega_mean_rps"]) for row in moving])
    denominator = float(commanded @ commanded)
    gain = float(commanded @ measured / denominator)
    if not math.isfinite(gain) or gain <= 0.0:
        raise ValueError("nominal yaw-response gain must be finite and positive")
    raw_scale = 1.0 / gain
    scale = min(raw_scale, maximum_scale)

    condition_ids = sorted({str(row["condition_id"]) for row in moving})
    predictions: list[dict[str, Any]] = []
    for condition_id in condition_ids:
        train = [row for row in moving if str(row["condition_id"]) != condition_id]
        test = [row for row in moving if str(row["condition_id"]) == condition_id]
        train_command = np.asarray([float(row["desired_omega_rps"]) for row in train])
        train_measured = np.asarray(
            [float(row["steady_measured_omega_mean_rps"]) for row in train]
        )
        fold_gain = float(train_command @ train_measured / (train_command @ train_command))
        for row in test:
            desired = float(row["desired_omega_rps"])
            predicted = fold_gain * desired
            observed = float(row["steady_measured_omega_mean_rps"])
            predictions.append(
                {
                    "condition_id": condition_id,
                    "desired_omega_rps": desired,
                    "observed_omega_rps": observed,
                    "predicted_omega_rps": predicted,
                    "prediction_error_rps": predicted - observed,
                }
            )
    errors = np.asarray([float(row["prediction_error_rps"]) for row in predictions])
    condition_mean_errors = {
        condition_id: float(
            np.mean(
                [
                    abs(float(row["prediction_error_rps"]))
                    for row in predictions
                    if row["condition_id"] == condition_id
                ]
            )
        )
        for condition_id in condition_ids
    }
    gains = np.asarray(
        [
            float(row["steady_measured_omega_mean_rps"])
            / float(row["desired_omega_rps"])
            for row in moving
        ]
    )
    sign_fraction = float(
        np.mean(
            [
                float(row["steady_measured_omega_mean_rps"])
                * float(row["desired_omega_rps"])
                > 0.0
                for row in moving
            ]
        )
    )
    checks = {
        "leave_one_condition_out_omega_rmse_rps": _rmse(errors)
        <= float(assessment["leave_one_condition_out_omega_rmse_rps"]),
        "maximum_condition_mean_abs_error_rps": max(condition_mean_errors.values())
        <= float(assessment["maximum_condition_mean_abs_error_rps"]),
        "minimum_sign_correct_fraction": sign_fraction
        >= float(assessment["minimum_sign_correct_fraction"]),
        "maximum_group_gain_range": float(np.max(gains) - np.min(gains))
        <= float(assessment["maximum_group_gain_range"]),
        "feedforward_scale_within_limit": raw_scale <= maximum_scale,
    }
    return {
        "model_type": "single_effective_track_width_feedforward",
        "nominal_yaw_response_gain": gain,
        "raw_feedforward_scale": raw_scale,
        "applied_feedforward_scale": scale,
        "physical_wheel_separation_m": physical,
        "calibrated_effective_wheel_separation_m": physical * scale,
        "leave_one_condition_out_omega_rmse_rps": _rmse(errors),
        "maximum_condition_mean_abs_error_rps": max(condition_mean_errors.values()),
        "observed_condition_gain_min": float(np.min(gains)),
        "observed_condition_gain_max": float(np.max(gains)),
        "observed_condition_gain_range": float(np.max(gains) - np.min(gains)),
        "sign_correct_fraction": sign_fraction,
        "condition_mean_abs_prediction_error_rps": condition_mean_errors,
        "predictions": predictions,
        "assessment_thresholds": dict(assessment),
        "assessment_checks": checks,
        "sufficient": all(checks.values()),
        "fit_semantics": (
            "least-squares through-origin omega_measured=c*omega_des on calibration trials; "
            "effective separation is physical separation/c and never overwrites physical geometry"
        ),
    }


def summarize_conditions(trial_rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    keys = sorted(
        {
            (
                str(row["mode"]),
                str(row["condition_id"]),
                float(row["desired_v_mps"]),
                float(row["desired_omega_rps"]),
            )
            for row in trial_rows
        }
    )
    result: list[dict[str, Any]] = []
    for mode, condition_id, desired_v, desired_omega in keys:
        rows = [
            row
            for row in trial_rows
            if str(row["mode"]) == mode and str(row["condition_id"]) == condition_id
        ]
        omega_means = np.asarray(
            [float(row["steady_measured_omega_mean_rps"]) for row in rows]
        )
        result.append(
            {
                "mode": mode,
                "condition_id": condition_id,
                "desired_v_mps": desired_v,
                "desired_omega_rps": desired_omega,
                "repetitions": len(rows),
                "steady_measured_v_mean_mps": float(
                    np.mean([float(row["steady_measured_v_mean_mps"]) for row in rows])
                ),
                "steady_measured_omega_mean_rps": float(np.mean(omega_means)),
                "steady_omega_repeatability_std_rps": float(np.std(omega_means)),
                "v_rmse_mps": float(
                    np.sqrt(np.mean([float(row["v_rmse_mps"]) ** 2 for row in rows]))
                ),
                "omega_rmse_rps": float(
                    np.sqrt(np.mean([float(row["omega_rmse_rps"]) ** 2 for row in rows]))
                ),
                "steady_omega_mae_rps": float(
                    np.mean([float(row["steady_omega_mae_rps"]) for row in rows])
                ),
                "omega_steady_state_gain": (
                    float(np.mean(omega_means) / desired_omega)
                    if abs(desired_omega) > 1e-12
                    else None
                ),
                "sign_correct_fraction": (
                    float(np.mean([bool(row["steady_sign_correct"]) for row in rows]))
                    if abs(desired_omega) >= 0.15
                    else None
                ),
                "wheel_rmse_rad_s": float(
                    np.sqrt(
                        np.mean(
                            [float(row["wheel_target_measured_rmse_rad_s"]) ** 2 for row in rows]
                        )
                    )
                ),
                "maximum_active_saturation_fraction": max(
                    float(row["active_saturation_fraction"]) for row in rows
                ),
                "maximum_measured_omega_sign_alternations": max(
                    int(row["measured_omega_sign_alternations"]) for row in rows
                ),
            }
        )
    return result


def response_dependence(condition_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    moving = [row for row in condition_rows if abs(float(row["desired_omega_rps"])) > 1e-12]
    by_v: dict[str, list[float]] = {}
    by_abs_omega: dict[str, list[float]] = {}
    for row in moving:
        gain = float(row["omega_steady_state_gain"])
        by_v.setdefault(f"{float(row['desired_v_mps']):.6g}", []).append(gain)
        by_abs_omega.setdefault(f"{abs(float(row['desired_omega_rps'])):.6g}", []).append(gain)
    asymmetry = []
    pairs = sorted(
        {(float(row["desired_v_mps"]), abs(float(row["desired_omega_rps"]))) for row in moving}
    )
    for v, magnitude in pairs:
        positive = [
            row
            for row in moving
            if float(row["desired_v_mps"]) == v
            and float(row["desired_omega_rps"]) == magnitude
        ]
        negative = [
            row
            for row in moving
            if float(row["desired_v_mps"]) == v
            and float(row["desired_omega_rps"]) == -magnitude
        ]
        if positive and negative:
            asymmetry.append(
                {
                    "desired_v_mps": v,
                    "abs_desired_omega_rps": magnitude,
                    "left_abs_measured_omega_rps": abs(
                        float(positive[0]["steady_measured_omega_mean_rps"])
                    ),
                    "right_abs_measured_omega_rps": abs(
                        float(negative[0]["steady_measured_omega_mean_rps"])
                    ),
                    "left_minus_right_abs_response_rps": abs(
                        float(positive[0]["steady_measured_omega_mean_rps"])
                    )
                    - abs(float(negative[0]["steady_measured_omega_mean_rps"])),
                }
            )
    return {
        "gain_by_v": {
            key: {"mean": float(np.mean(values)), "range": float(np.ptp(values))}
            for key, values in by_v.items()
        },
        "gain_by_abs_omega": {
            key: {"mean": float(np.mean(values)), "range": float(np.ptp(values))}
            for key, values in by_abs_omega.items()
        },
        "left_right_asymmetry": asymmetry,
        "maximum_abs_left_right_response_difference_rps": (
            max(abs(float(row["left_minus_right_abs_response_rps"])) for row in asymmetry)
            if asymmetry
            else None
        ),
    }


def evaluate_tracking_acceptance(
    trial_rows: Sequence[Mapping[str, Any]], criteria: Mapping[str, Any]
) -> dict[str, Any]:
    conditions = summarize_conditions(trial_rows)
    moving = [row for row in conditions if abs(float(row["desired_omega_rps"])) >= 0.15]
    if not moving:
        raise ValueError("acceptance requires moving yaw conditions")
    sign_fraction = min(float(row["sign_correct_fraction"]) for row in moving)
    condition_error_checks = {
        str(row["condition_id"]): float(row["steady_omega_mae_rps"])
        <= max(
            float(criteria["per_condition_omega_mean_abs_error_floor_rps"]),
            float(criteria["per_condition_omega_mean_abs_error_fraction"])
            * abs(float(row["desired_omega_rps"])),
        )
        for row in moving
    }
    omega_rmse = float(
        np.sqrt(np.mean([float(row["omega_rmse_rps"]) ** 2 for row in conditions]))
    )
    v_rmse = float(np.sqrt(np.mean([float(row["v_rmse_mps"]) ** 2 for row in conditions])))
    max_repeatability = max(
        float(row["steady_omega_repeatability_std_rps"]) for row in conditions
    )
    max_saturation = max(float(row["maximum_active_saturation_fraction"]) for row in conditions)
    max_alternations = max(
        int(row["maximum_measured_omega_sign_alternations"]) for row in conditions
    )
    checks = {
        "sign_correctness": sign_fraction
        >= float(criteria["steady_state_sign_correct_fraction"]),
        "per_condition_omega_mean_abs_error": all(condition_error_checks.values()),
        "aggregate_omega_rmse": omega_rmse <= float(criteria["aggregate_omega_rmse_rps"]),
        "aggregate_v_rmse": v_rmse <= float(criteria["aggregate_v_rmse_mps"]),
        "repeatability": max_repeatability
        <= float(criteria["per_condition_repeatability_std_omega_rps"]),
        "no_unstable_oscillation": max_alternations
        <= int(criteria["maximum_sustained_sign_alternations"]),
        "saturation": max_saturation
        <= float(criteria["maximum_active_saturation_fraction"]),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "per_condition_omega_error_checks": condition_error_checks,
        "aggregate_omega_rmse_rps": omega_rmse,
        "aggregate_v_rmse_mps": v_rmse,
        "minimum_condition_sign_correct_fraction": sign_fraction,
        "maximum_condition_repeatability_std_omega_rps": max_repeatability,
        "maximum_active_saturation_fraction": max_saturation,
        "maximum_measured_omega_sign_alternations": max_alternations,
        "conditions": conditions,
        "criteria": dict(criteria),
    }


def select_feedback_candidate(
    candidates: Sequence[Mapping[str, Any]], criteria: Mapping[str, Any]
) -> dict[str, Any]:
    if not candidates:
        raise ValueError("feedback candidate results are empty")
    evaluated = []
    for candidate in candidates:
        evaluation = evaluate_tracking_acceptance(candidate["trials"], criteria)
        evaluated.append({**dict(candidate), "evaluation": evaluation})
    passing = [row for row in evaluated if row["evaluation"]["passed"]]
    selected = passing[0] if passing else min(
        evaluated,
        key=lambda row: (
            float(row["evaluation"]["aggregate_omega_rmse_rps"]),
            float(row["evaluation"]["aggregate_v_rmse_mps"]),
            str(row["candidate_id"]),
        ),
    )
    return {
        "selected_candidate_id": selected["candidate_id"],
        "selected_passed_calibration_criteria": bool(selected["evaluation"]["passed"]),
        "selection_basis": (
            "first predeclared candidate passing calibration criteria"
            if passing
            else "lowest calibration aggregate omega RMSE; no candidate passed all criteria"
        ),
        "candidates": [
            {
                "candidate_id": row["candidate_id"],
                "parameters": dict(row["parameters"]),
                "evaluation": row["evaluation"],
            }
            for row in evaluated
        ],
    }


def write_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    output = Path(path)
    assert_finite_json_tree(value)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return output


def save_execution_trial(
    destination: str | Path,
    telemetry: ExecutionTelemetry,
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
    with (raw / "telemetry.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TELEMETRY_COLUMNS)
        for index in range(telemetry.sample_count):
            writer.writerow(
                [
                    index,
                    telemetry.sim_times_s[index],
                    int(telemetry.control_indices[index]),
                    telemetry.phases[index],
                    *telemetry.desired_body[index],
                    *telemetry.executed_body[index],
                    *telemetry.ideal_left_right[index],
                    *telemetry.target_wheels[index],
                    *telemetry.measured_wheels[index],
                    *telemetry.actual_trajectory[index],
                    measured_v[index],
                    measured_omega[index],
                    telemetry.desired_body[index, 1] - measured_omega[index],
                    bool(telemetry.saturated[index]),
                ]
            )
    write_json_exclusive(derived / "metrics.json", metrics)
    write_json_exclusive(root / "metadata.json", metadata)
    hashes = {
        "raw/actual_trajectory.npy": sha256_file(raw / "actual_trajectory.npy"),
        "raw/telemetry.csv": sha256_file(raw / "telemetry.csv"),
    }
    write_json_exclusive(derived / "raw_provenance.json", {"sha256": hashes})
    return hashes


def validate_execution_trial(path: str | Path) -> dict[str, Any]:
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
        raise ValueError(f"execution trial is missing: {', '.join(missing)}")
    trajectory = validate_se2_trajectory(
        np.load(root / "raw/actual_trajectory.npy", allow_pickle=False), name="actual"
    )
    with (root / "raw/telemetry.csv").open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != TELEMETRY_COLUMNS:
            raise ValueError("telemetry columns differ from Stage 0-D schema")
        rows = list(reader)
    if len(rows) != trajectory.shape[0]:
        raise ValueError("telemetry row count differs from actual trajectory")
    for row in rows:
        for key, value in row.items():
            if key == "phase":
                if value not in PHASES:
                    raise ValueError("invalid telemetry phase")
            elif key == "saturated":
                if value not in ("True", "False"):
                    raise ValueError("invalid telemetry saturation flag")
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
    return {
        "valid": True,
        "mode": metadata["mode"],
        "condition_id": metadata["condition_id"],
        "repetition": metadata["repetition"],
        "sample_count": int(trajectory.shape[0]),
        "metrics": metrics,
    }


def assert_disjoint_condition_sets(
    calibration_conditions: Sequence[tuple[float, float]],
    held_out_conditions: Sequence[tuple[float, float]],
) -> None:
    calibration = {(float(v), float(omega)) for v, omega in calibration_conditions}
    held_out = {(float(v), float(omega)) for v, omega in held_out_conditions}
    overlap = sorted(calibration & held_out)
    if overlap:
        raise ValueError(f"held-out leakage: conditions overlap: {overlap}")


def validate_execution_calibration_config(config: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the frozen Stage 0-D design before any simulator result exists."""

    if config.get("stage") != "stage0-d-jackal-execution-layer-calibration-and-validation":
        raise ValueError("invalid Stage 0-D stage identifier")
    simulation = config["simulation"]
    physics_dt = float(simulation["physics_dt_s"])
    control_dt = float(simulation["control_dt_s"])
    if not math.isfinite(physics_dt) or physics_dt <= 0.0:
        raise ValueError("physics_dt_s must be finite and positive")
    if not math.isfinite(control_dt) or control_dt <= 0.0:
        raise ValueError("control_dt_s must be finite and positive")
    physics_steps = round(control_dt / physics_dt)
    if physics_steps < 1 or not math.isclose(
        physics_steps * physics_dt, control_dt, abs_tol=1e-12
    ):
        raise ValueError("control_dt_s must be an integer multiple of physics_dt_s")
    for key in (
        "settling_duration_s",
        "initial_stop_duration_s",
        "active_duration_s",
        "final_stop_duration_s",
        "steady_state_tail_s",
    ):
        value = float(simulation[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{key} must be finite and positive")
    if float(simulation["steady_state_tail_s"]) > float(simulation["active_duration_s"]):
        raise ValueError("steady_state_tail_s cannot exceed active_duration_s")

    def conditions(section: Mapping[str, Any]) -> list[tuple[float, float]]:
        result = [
            (float(v), float(omega))
            for v in section["linear_velocity_mps"]
            for omega in section["angular_velocity_rps"]
        ]
        if not result or not all(math.isfinite(v) and math.isfinite(w) for v, w in result):
            raise ValueError("command grids must be non-empty and finite")
        if len(result) != len(set(result)):
            raise ValueError("command grids cannot contain duplicate conditions")
        repetitions = int(section["repetitions"])
        if repetitions < 3:
            raise ValueError("primary command-grid repetitions must be at least three")
        return result

    calibration = conditions(config["characterization"])
    held_out = conditions(config["held_out_validation"])
    assert_disjoint_condition_sets(calibration, held_out)
    criteria = config["acceptance_criteria"]
    positive_keys = (
        "per_condition_omega_mean_abs_error_floor_rps",
        "per_condition_omega_mean_abs_error_fraction",
        "aggregate_omega_rmse_rps",
        "aggregate_v_rmse_mps",
        "per_condition_repeatability_std_omega_rps",
        "maximum_active_saturation_fraction",
    )
    for key in positive_keys:
        value = float(criteria[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"acceptance criterion {key} must be finite and positive")
    if not 0.0 < float(criteria["steady_state_sign_correct_fraction"]) <= 1.0:
        raise ValueError("steady_state_sign_correct_fraction must be in (0, 1]")
    candidates = config["model_selection"]["feedback_candidates"]
    identifiers = [str(row["id"]) for row in candidates]
    if not candidates or len(identifiers) != len(set(identifiers)):
        raise ValueError("feedback candidates must have unique identifiers")
    assert_finite_json_tree(config)
    return {
        "valid": True,
        "physics_steps_per_control": int(physics_steps),
        "calibration_condition_count": len(calibration),
        "held_out_condition_count": len(held_out),
        "held_out_disjoint": True,
        "criteria_frozen": True,
    }


def evaluate_composite_acceptance(
    nominal: Mapping[str, Any],
    calibrated: Mapping[str, Any],
    criteria: Mapping[str, Any],
) -> dict[str, Any]:
    """Apply the frozen Stage 0-B absolute gates and non-degradation checks."""

    tolerance = float(criteria["non_degradation_tolerance"])
    absolute = {
        "goal_reached": bool(calibrated["goal_reached"]),
        "final_position_error": float(calibrated["final_position_error_m"])
        < float(criteria["final_position_error_m"]),
        "position_rmse": float(calibrated["position_rmse_m"])
        < float(criteria["position_rmse_m"]),
        "final_yaw_error": float(calibrated["final_yaw_error_rad"])
        < float(criteria["final_yaw_error_rad"]),
        "yaw_rmse": float(calibrated["yaw_rmse_rad"])
        < float(criteria["yaw_rmse_rad"]),
    }
    non_degradation = {
        key: float(calibrated[key]) <= float(nominal[key]) + tolerance
        for key in (
            "position_rmse_m",
            "final_position_error_m",
            "yaw_rmse_rad",
            "final_yaw_error_rad",
            "desired_omega_rmse_rps",
        )
    }
    return {
        "passed": all(absolute.values()) and all(non_degradation.values()),
        "absolute_gate_passed": all(absolute.values()),
        "non_degradation_passed": all(non_degradation.values()),
        "absolute_checks": absolute,
        "non_degradation_checks": non_degradation,
        "criteria": dict(criteria),
    }


def evaluate_exp02b_replay_acceptance(
    nominal: Mapping[str, Any],
    calibrated: Mapping[str, Any],
    criteria: Mapping[str, Any],
) -> dict[str, Any]:
    """Evaluate the predeclared representative replay improvement gates."""

    nominal_body = nominal["body_and_wheel_execution_metrics"]
    calibrated_body = calibrated["body_and_wheel_execution_metrics"]
    nominal_omega = float(nominal_body["omega_command_vs_measured_rmse_rps"])
    calibrated_omega = float(calibrated_body["omega_command_vs_measured_rmse_rps"])
    reduction = (nominal_omega - calibrated_omega) / nominal_omega
    nominal_spatial = float(
        nominal["old_reference_spatial_metrics"]["old_reference_distance_rms_m"]
    )
    calibrated_spatial = float(
        calibrated["old_reference_spatial_metrics"]["old_reference_distance_rms_m"]
    )
    spatial_limit = max(
        nominal_spatial * float(criteria["maximum_spatial_rms_ratio"]),
        nominal_spatial + float(criteria["maximum_spatial_rms_absolute_increase_m"]),
    )
    checks = {
        "meaningful_omega_rmse_reduction": reduction
        >= float(criteria["minimum_representative_omega_rmse_reduction_fraction"]),
        "spatial_tracking_not_catastrophically_worse": calibrated_spatial <= spatial_limit,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "omega_rmse_reduction_fraction": reduction,
        "nominal_omega_rmse_rps": nominal_omega,
        "calibrated_omega_rmse_rps": calibrated_omega,
        "nominal_spatial_rms_m": nominal_spatial,
        "calibrated_spatial_rms_m": calibrated_spatial,
        "spatial_rms_limit_m": spatial_limit,
        "criteria": dict(criteria),
    }


def final_validation_decision(
    *,
    calibration_grid_passed: bool,
    held_out_passed: bool,
    composite_passed: bool,
    representative_replay_passed: bool,
    held_out_leakage: bool,
) -> dict[str, Any]:
    checks = {
        "calibration_grid_passed": bool(calibration_grid_passed),
        "held_out_primitive_grid_passed": bool(held_out_passed),
        "composite_passed": bool(composite_passed),
        "representative_exp02b_replay_passed": bool(representative_replay_passed),
        "held_out_data_not_used_for_tuning": not bool(held_out_leakage),
    }
    if all(checks.values()):
        status = "EXECUTION_LAYER_CALIBRATED_AND_VALIDATED"
    elif any(
        checks[key]
        for key in (
            "calibration_grid_passed",
            "held_out_primitive_grid_passed",
            "composite_passed",
            "representative_exp02b_replay_passed",
        )
    ) and checks["held_out_data_not_used_for_tuning"]:
        status = "PARTIAL_VALIDATION"
    else:
        status = "EXECUTION_LAYER_NOT_YET_VALIDATED"
    return {"status": status, "passed": all(checks.values()), "checks": checks}
