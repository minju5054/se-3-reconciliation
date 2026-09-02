"""Isaac-independent telemetry, metrics, and immutable Stage 0-B output."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


FloatArray = NDArray[np.float64]
WHEEL_LABELS = ("front_left", "front_right", "rear_left", "rear_right")
POSE_SAMPLE_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "reference_index",
    "ref_x",
    "ref_y",
    "ref_yaw",
    "actual_x",
    "actual_y",
    "actual_yaw",
)
TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "commanded_v_mps",
    "commanded_omega_rps",
    *(f"target_{label}_rad_s" for label in WHEEL_LABELS),
    *(f"actual_{label}_rad_s" for label in WHEEL_LABELS),
    "actual_x",
    "actual_y",
    "actual_yaw",
    "measured_linear_speed_mps",
    "measured_yaw_rate_rps",
)


def _finite_array(name: str, value: ArrayLike, shape: tuple[int, ...]) -> FloatArray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite array with shape {shape}")
    return result.copy()


def estimate_body_velocities(poses: ArrayLike, times_s: ArrayLike) -> tuple[FloatArray, FloatArray]:
    """Estimate body-forward speed and yaw rate over each interval ending at sample ``k``."""

    trajectory = validate_se2_trajectory(poses, name="poses")
    times = _finite_array("times_s", times_s, (trajectory.shape[0],))
    if not np.isclose(times[0], 0.0, atol=1e-12):
        raise ValueError("times_s must start at zero")
    if trajectory.shape[0] > 1 and np.any(np.diff(times) <= 0.0):
        raise ValueError("times_s must be strictly increasing")
    linear = np.zeros(trajectory.shape[0], dtype=np.float64)
    angular = np.zeros(trajectory.shape[0], dtype=np.float64)
    if trajectory.shape[0] == 1:
        return linear, angular
    delta_xy = np.diff(trajectory[:, :2], axis=0)
    dt = np.diff(times)
    interval_heading = trajectory[:-1, 2]
    forward_axes = np.column_stack((np.cos(interval_heading), np.sin(interval_heading)))
    linear[1:] = np.sum(delta_xy * forward_axes, axis=1) / dt
    angular[1:] = np.asarray(
        [wrap_angle(value) for value in np.diff(trajectory[:, 2])], dtype=np.float64
    ) / dt
    return linear, angular


@dataclass(frozen=True, slots=True)
class ControllerTelemetry:
    reference_trajectory: FloatArray
    actual_trajectory: FloatArray
    sim_times_s: FloatArray
    commanded_body: FloatArray
    target_wheel_velocities: FloatArray
    actual_wheel_velocities: FloatArray
    reference_indices: NDArray[np.int64]
    wheel_names: tuple[str, ...]

    def __post_init__(self) -> None:
        reference = validate_se2_trajectory(self.reference_trajectory, name="reference_trajectory")
        actual = validate_se2_trajectory(self.actual_trajectory, name="actual_trajectory")
        count = actual.shape[0]
        times = _finite_array("sim_times_s", self.sim_times_s, (count,))
        commanded = _finite_array("commanded_body", self.commanded_body, (count, 2))
        targets = _finite_array("target_wheel_velocities", self.target_wheel_velocities, (count, 4))
        actual_wheels = _finite_array(
            "actual_wheel_velocities", self.actual_wheel_velocities, (count, 4)
        )
        reference_indices = np.asarray(self.reference_indices, dtype=np.int64)
        if reference_indices.shape != (count,):
            raise ValueError(f"reference_indices must have shape ({count},)")
        if np.any(reference_indices < 0) or np.any(reference_indices >= reference.shape[0]):
            raise ValueError("reference_indices contains an out-of-range index")
        if count > 1 and np.any(np.diff(reference_indices) < 0):
            raise ValueError("reference_indices must be monotonically non-decreasing")
        if len(self.wheel_names) != 4 or len(set(self.wheel_names)) != 4:
            raise ValueError("wheel_names must contain four unique runtime DOF names")
        if not np.isclose(times[0], 0.0, atol=1e-12):
            raise ValueError("sim_times_s must start at zero")
        if count > 1 and np.any(np.diff(times) <= 0.0):
            raise ValueError("sim_times_s must be strictly increasing")
        object.__setattr__(self, "reference_trajectory", reference)
        object.__setattr__(self, "actual_trajectory", actual)
        object.__setattr__(self, "sim_times_s", times)
        object.__setattr__(self, "commanded_body", commanded)
        object.__setattr__(self, "target_wheel_velocities", targets)
        object.__setattr__(self, "actual_wheel_velocities", actual_wheels)
        object.__setattr__(self, "reference_indices", reference_indices.copy())
        object.__setattr__(self, "wheel_names", tuple(self.wheel_names))

    @property
    def sample_count(self) -> int:
        return int(self.actual_trajectory.shape[0])


def compute_controller_metrics(telemetry: ControllerTelemetry) -> dict[str, Any]:
    """Compute sample-aligned pose, body-rate, and wheel-tracking diagnostics."""

    reference = telemetry.reference_trajectory
    actual = telemetry.actual_trajectory
    matched_reference = reference[telemetry.reference_indices]
    position_error = np.linalg.norm(matched_reference[:, :2] - actual[:, :2], axis=1)
    yaw_error = np.asarray(
        [wrap_angle(value) for value in matched_reference[:, 2] - actual[:, 2]], dtype=np.float64
    )
    measured_v, measured_omega = estimate_body_velocities(actual, telemetry.sim_times_s)
    body_error = telemetry.commanded_body - np.column_stack((measured_v, measured_omega))
    wheel_error = telemetry.target_wheel_velocities - telemetry.actual_wheel_velocities
    active_linear = np.abs(telemetry.commanded_body[:, 0]) > 1e-9
    active_angular = np.abs(telemetry.commanded_body[:, 1]) > 1e-9

    def active_mean(values: FloatArray, mask: NDArray[np.bool_]) -> float | None:
        return float(np.mean(values[mask])) if np.any(mask) else None

    return {
        "sample_count": telemetry.sample_count,
        "reference_sample_count": int(reference.shape[0]),
        "position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "mean_position_error_m": float(np.mean(position_error)),
        "max_position_error_m": float(np.max(position_error)),
        "final_position_error_m": float(np.linalg.norm(reference[-1, :2] - actual[-1, :2])),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error**2))),
        "mean_absolute_yaw_error_rad": float(np.mean(np.abs(yaw_error))),
        "max_yaw_error_rad": float(np.max(np.abs(yaw_error))),
        "final_yaw_error_rad": abs(float(wrap_angle(reference[-1, 2] - actual[-1, 2]))),
        "linear_velocity_rmse_mps": float(np.sqrt(np.mean(body_error[:, 0] ** 2))),
        "angular_velocity_rmse_rps": float(np.sqrt(np.mean(body_error[:, 1] ** 2))),
        "active_commanded_linear_velocity_mean_mps": active_mean(
            telemetry.commanded_body[:, 0], active_linear
        ),
        "active_measured_linear_velocity_mean_mps": active_mean(measured_v, active_linear),
        "active_commanded_angular_velocity_mean_rps": active_mean(
            telemetry.commanded_body[:, 1], active_angular
        ),
        "active_measured_angular_velocity_mean_rps": active_mean(measured_omega, active_angular),
        "wheel_velocity_tracking_rmse_rad_s": float(np.sqrt(np.mean(wheel_error**2))),
        "wheel_velocity_tracking_rmse_by_dof_rad_s": {
            name: float(np.sqrt(np.mean(wheel_error[:, index] ** 2)))
            for index, name in enumerate(telemetry.wheel_names)
        },
        "actual_total_displacement_m": float(np.linalg.norm(actual[-1, :2] - actual[0, :2])),
        "actual_total_yaw_change_rad": float(wrap_angle(actual[-1, 2] - actual[0, 2])),
    }


def save_controller_run(
    destination: str | Path,
    telemetry: ControllerTelemetry,
    metrics: Mapping[str, Any],
    metadata: Mapping[str, Any],
) -> Path:
    """Write one immutable Stage 0-B scenario directory."""

    output = Path(destination)
    output.mkdir(parents=True, exist_ok=False)
    np.save(output / "reference_trajectory.npy", telemetry.reference_trajectory)
    np.save(output / "actual_trajectory.npy", telemetry.actual_trajectory)
    with (output / "samples.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(POSE_SAMPLE_COLUMNS)
        for index in range(telemetry.sample_count):
            writer.writerow(
                [
                    index,
                    telemetry.sim_times_s[index],
                    int(telemetry.reference_indices[index]),
                    *telemetry.reference_trajectory[telemetry.reference_indices[index]],
                    *telemetry.actual_trajectory[index],
                ]
            )
    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    with (output / "telemetry.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TELEMETRY_COLUMNS)
        for index in range(telemetry.sample_count):
            writer.writerow(
                [
                    index,
                    telemetry.sim_times_s[index],
                    *telemetry.commanded_body[index],
                    *telemetry.target_wheel_velocities[index],
                    *telemetry.actual_wheel_velocities[index],
                    *telemetry.actual_trajectory[index],
                    measured_v[index],
                    measured_omega[index],
                ]
            )
    for name, value in (("metrics.json", metrics), ("metadata.json", metadata)):
        with (output / name).open("x", encoding="utf-8") as stream:
            json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    return output


def validate_controller_run(destination: str | Path) -> dict[str, Any]:
    """Validate one saved scenario without importing Isaac Sim."""

    source = Path(destination)
    required = {
        "reference_trajectory.npy",
        "actual_trajectory.npy",
        "samples.csv",
        "telemetry.csv",
        "metrics.json",
        "metadata.json",
    }
    missing = sorted(name for name in required if not (source / name).is_file())
    if missing:
        raise ValueError(f"controller run is missing files: {', '.join(missing)}")
    reference = validate_se2_trajectory(
        np.load(source / "reference_trajectory.npy", allow_pickle=False), name="reference"
    )
    actual = validate_se2_trajectory(
        np.load(source / "actual_trajectory.npy", allow_pickle=False), name="actual"
    )
    with (source / "samples.csv").open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != POSE_SAMPLE_COLUMNS:
            raise ValueError("samples.csv columns do not match the Stage 0-B schema")
        sample_rows = list(reader)
    if len(sample_rows) != actual.shape[0]:
        raise ValueError("samples row count differs from actual trajectory sample count")
    reference_indices = np.asarray(
        [int(row["reference_index"]) for row in sample_rows], dtype=np.int64
    )
    if np.any(reference_indices < 0) or np.any(reference_indices >= reference.shape[0]):
        raise ValueError("samples.csv contains an out-of-range reference index")
    if reference_indices.shape[0] > 1 and np.any(np.diff(reference_indices) < 0):
        raise ValueError("samples.csv reference indices must be monotonic")
    with (source / "telemetry.csv").open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != TELEMETRY_COLUMNS:
            raise ValueError("telemetry.csv columns do not match the Stage 0-B schema")
        telemetry_rows = list(reader)
    if len(telemetry_rows) != actual.shape[0]:
        raise ValueError("telemetry row count differs from trajectory sample count")
    with (source / "metrics.json").open("r", encoding="utf-8") as stream:
        metrics = json.load(stream)
    with (source / "metadata.json").open("r", encoding="utf-8") as stream:
        metadata = json.load(stream)
    if not isinstance(metrics, Mapping) or not isinstance(metadata, Mapping):
        raise ValueError("metrics and metadata must contain JSON objects")
    scalar_metrics = [value for value in metrics.values() if isinstance(value, (int, float))]
    if not scalar_metrics or not all(math.isfinite(float(value)) for value in scalar_metrics):
        raise ValueError("scalar metrics must be finite")
    return {
        "valid": True,
        "scenario": metadata.get("scenario"),
        "controller": metadata.get("controller"),
        "reference_shape": list(reference.shape),
        "actual_shape": list(actual.shape),
        "metrics": metrics,
    }


def save_session_summary(destination: str | Path, summary: Mapping[str, Any]) -> Path:
    path = Path(destination) / "summary.json"
    with path.open("x", encoding="utf-8") as stream:
        json.dump(dict(summary), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return path
