"""Isaac-independent Stage 0 recording, metrics, and output validation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


SAMPLE_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "ref_x",
    "ref_y",
    "ref_yaw",
    "actual_x",
    "actual_y",
    "actual_yaw",
)

REQUIRED_METADATA_FIELDS = {
    "stage",
    "run_id",
    "creation_time",
    "git_commit_sha",
    "isaac_sim_version",
    "robot_asset",
    "robot_prim_path",
    "actual_dof_names",
    "physics_dt",
    "sample_dt",
    "trajectory_convention",
    "pose_frame",
    "yaw_convention",
    "motion_profile",
    "wheel_parameters",
    "visualization_height_m",
}


def _finite_vector(name: str, values: ArrayLike, length: int) -> NDArray[np.float64]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite array with shape ({length},)")
    return result.copy()


@dataclass(frozen=True, slots=True)
class Stage0Recording:
    """Reference and actual poses sampled at exactly the same explicit timestamps."""

    reference_trajectory: NDArray[np.float64]
    actual_trajectory: NDArray[np.float64]
    sim_times_s: NDArray[np.float64]

    def __post_init__(self) -> None:
        reference = validate_se2_trajectory(
            self.reference_trajectory,
            name="reference_trajectory",
        )
        actual = validate_se2_trajectory(self.actual_trajectory, name="actual_trajectory")
        if reference.shape[0] != actual.shape[0]:
            raise ValueError(
                "Stage 0 does not interpolate: reference and actual sample counts must match"
            )
        times = _finite_vector("sim_times_s", self.sim_times_s, reference.shape[0])
        if not np.isclose(times[0], 0.0, atol=1e-12):
            raise ValueError("sim_times_s must start at zero")
        if times.shape[0] > 1 and np.any(np.diff(times) <= 0.0):
            raise ValueError("sim_times_s must be strictly increasing")
        object.__setattr__(self, "reference_trajectory", reference)
        object.__setattr__(self, "actual_trajectory", actual)
        object.__setattr__(self, "sim_times_s", times)

    @property
    def sample_count(self) -> int:
        return int(self.reference_trajectory.shape[0])


def compute_stage0_metrics(recording: Stage0Recording) -> dict[str, float]:
    reference = recording.reference_trajectory
    actual = recording.actual_trajectory
    position_errors = np.linalg.norm(reference[:, :2] - actual[:, :2], axis=1)
    return {
        "final_position_error_m": float(np.linalg.norm(reference[-1, :2] - actual[-1, :2])),
        "final_yaw_error_rad": abs(float(wrap_angle(reference[-1, 2] - actual[-1, 2]))),
        "mean_sampled_position_error_m": float(np.mean(position_errors)),
        "actual_total_displacement_m": float(np.linalg.norm(actual[-1, :2] - actual[0, :2])),
        "actual_total_yaw_change_rad": float(wrap_angle(actual[-1, 2] - actual[0, 2])),
    }


def _validate_metadata(metadata: Mapping[str, Any]) -> None:
    missing = sorted(REQUIRED_METADATA_FIELDS.difference(metadata))
    if missing:
        raise ValueError(f"metadata is missing fields: {', '.join(missing)}")
    if not isinstance(metadata["run_id"], str) or not metadata["run_id"].strip():
        raise ValueError("metadata run_id must be a non-empty string")
    try:
        datetime.fromisoformat(str(metadata["creation_time"]))
    except ValueError as error:
        raise ValueError("metadata creation_time must be ISO 8601") from error


def save_stage0_run(
    output_dir: str | Path,
    recording: Stage0Recording,
    metadata: Mapping[str, Any],
) -> Path:
    """Create one immutable run directory; existing targets are never overwritten."""

    _validate_metadata(metadata)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=False)

    np.save(destination / "reference_trajectory.npy", recording.reference_trajectory)
    np.save(destination / "actual_trajectory.npy", recording.actual_trajectory)
    with (destination / "samples.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(SAMPLE_COLUMNS)
        for index, (time_s, reference, actual) in enumerate(
            zip(
                recording.sim_times_s,
                recording.reference_trajectory,
                recording.actual_trajectory,
                strict=True,
            )
        ):
            writer.writerow([index, time_s, *reference, *actual])
    with (destination / "metadata.json").open("x", encoding="utf-8") as stream:
        json.dump(dict(metadata), stream, indent=2, sort_keys=True)
        stream.write("\n")
    return destination


def validate_stage0_output(output_dir: str | Path) -> dict[str, Any]:
    """Validate a saved run without Isaac Sim and return its smoke-test summary."""

    source = Path(output_dir)
    required_files = {
        "reference_trajectory.npy",
        "actual_trajectory.npy",
        "samples.csv",
        "metadata.json",
    }
    missing_files = sorted(name for name in required_files if not (source / name).is_file())
    if missing_files:
        raise ValueError(f"Stage 0 output is missing files: {', '.join(missing_files)}")

    reference = validate_se2_trajectory(
        np.load(source / "reference_trajectory.npy", allow_pickle=False),
        name="reference_trajectory.npy",
    )
    actual = validate_se2_trajectory(
        np.load(source / "actual_trajectory.npy", allow_pickle=False),
        name="actual_trajectory.npy",
    )
    with (source / "metadata.json").open("r", encoding="utf-8") as stream:
        metadata = json.load(stream)
    if not isinstance(metadata, Mapping):
        raise ValueError("metadata.json must contain an object")
    _validate_metadata(metadata)
    smoke_checks = metadata.get("smoke_success_checks")
    if smoke_checks is not None:
        if not isinstance(smoke_checks, Mapping) or smoke_checks.get("passed") is not True:
            raise ValueError("metadata smoke_success_checks did not pass")

    with (source / "samples.csv").open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != SAMPLE_COLUMNS:
            raise ValueError("samples.csv columns do not match the Stage 0 schema")
        rows = list(reader)
    if len(rows) != reference.shape[0] or len(rows) != actual.shape[0]:
        raise ValueError("saved trajectory and CSV sample counts differ; no interpolation is allowed")
    times = np.asarray([float(row["sim_time_s"]) for row in rows], dtype=np.float64)
    csv_reference = np.asarray(
        [[float(row[key]) for key in ("ref_x", "ref_y", "ref_yaw")] for row in rows],
        dtype=np.float64,
    )
    csv_actual = np.asarray(
        [
            [float(row[key]) for key in ("actual_x", "actual_y", "actual_yaw")]
            for row in rows
        ],
        dtype=np.float64,
    )
    if not np.array_equal(csv_reference, reference) or not np.array_equal(csv_actual, actual):
        raise ValueError("samples.csv pose values differ from the saved NPY arrays")

    recording = Stage0Recording(reference, actual, times)
    metrics = compute_stage0_metrics(recording)
    if not all(math.isfinite(value) for value in metrics.values()):
        raise ValueError("computed Stage 0 metrics must be finite")
    return {
        "valid": True,
        "run_id": metadata["run_id"],
        "reference_shape": list(reference.shape),
        "actual_shape": list(actual.shape),
        "start_pose": actual[0].tolist(),
        "end_pose": actual[-1].tolist(),
        "metrics": metrics,
        "smoke_success_checks": smoke_checks,
    }
