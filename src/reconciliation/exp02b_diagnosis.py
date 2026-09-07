"""Pure telemetry metrics and immutable output for EXP-02B GUI diagnosis.

The path metric in this module is spatial.  It deliberately does not align
LightNav waypoint rows to execution samples in time because those rows do not
have an intrinsic time base.
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
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.online_switch import load_strict_json
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


FloatArray = NDArray[np.float64]
DIAGNOSTIC_PHASES = (
    "SETTLING",
    "OLD_REPLAY",
    "PRE_RESET_INSPECTION",
    "EXACT_BOUNDARY_RESET",
    "POST_SWITCH_EXECUTION",
    "FINISHED",
)
OLD_TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "source_command_index",
    "actual_x",
    "actual_y",
    "actual_yaw",
    "commanded_v_mps",
    "commanded_omega_rps",
    "measured_body_v_mps",
    "measured_body_omega_rps",
    *(f"target_{label}_rad_s" for label in WHEEL_LABELS),
    *(f"measured_{label}_rad_s" for label in WHEEL_LABELS),
)
POST_TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "control_command_index",
    "actual_x",
    "actual_y",
    "actual_yaw",
    "commanded_v_mps",
    "commanded_omega_rps",
    "measured_body_v_mps",
    "measured_body_omega_rps",
    *(f"target_{label}_rad_s" for label in WHEEL_LABELS),
    *(f"measured_{label}_rad_s" for label in WHEEL_LABELS),
    "nearest_index",
    "target_index",
    "goal_reached",
)


def _finite_array(name: str, value: ArrayLike, shape: tuple[int, ...]) -> FloatArray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite array with shape {shape}")
    return result.copy()


def _rmse(values: FloatArray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


@dataclass(frozen=True, slots=True)
class DiagnosticTelemetry:
    """Physics-step samples with commands associated to the ending interval."""

    actual_trajectory: FloatArray
    sim_times_s: FloatArray
    commanded_body: FloatArray
    target_wheel_velocities: FloatArray
    measured_wheel_velocities: FloatArray

    def __post_init__(self) -> None:
        actual = validate_se2_trajectory(self.actual_trajectory, name="actual_trajectory")
        count = actual.shape[0]
        times = _finite_array("sim_times_s", self.sim_times_s, (count,))
        commands = _finite_array("commanded_body", self.commanded_body, (count, 2))
        targets = _finite_array(
            "target_wheel_velocities", self.target_wheel_velocities, (count, 4)
        )
        measured = _finite_array(
            "measured_wheel_velocities", self.measured_wheel_velocities, (count, 4)
        )
        if not np.isclose(times[0], 0.0, atol=1e-12):
            raise ValueError("sim_times_s must start at zero")
        if np.any(np.diff(times) <= 0.0):
            raise ValueError("sim_times_s must be strictly increasing")
        object.__setattr__(self, "actual_trajectory", actual)
        object.__setattr__(self, "sim_times_s", times)
        object.__setattr__(self, "commanded_body", commands)
        object.__setattr__(self, "target_wheel_velocities", targets)
        object.__setattr__(self, "measured_wheel_velocities", measured)

    @property
    def sample_count(self) -> int:
        return int(self.actual_trajectory.shape[0])


def nearest_polyline_samples(reference: ArrayLike, actual: ArrayLike) -> dict[str, FloatArray]:
    """Project each actual XY pose to the nearest OLD polyline segment.

    Yaw is interpolated along the selected segment using the wrapped segment
    yaw increment.  It is a nearest-spatial-reference descriptor, never a
    time-aligned tracking comparison.
    """

    ref = validate_se2_trajectory(reference, name="reference")
    observed = validate_se2_trajectory(actual, name="actual")
    starts = ref[:-1, :2]
    delta = ref[1:, :2] - starts
    squared_length = np.sum(delta * delta, axis=1)
    distances = np.empty(observed.shape[0], dtype=np.float64)
    segment_indices = np.empty(observed.shape[0], dtype=np.int64)
    fractions = np.empty(observed.shape[0], dtype=np.float64)
    nearest_xy = np.empty((observed.shape[0], 2), dtype=np.float64)
    nearest_yaw = np.empty(observed.shape[0], dtype=np.float64)
    yaw_delta = np.asarray(wrap_angle(ref[1:, 2] - ref[:-1, 2]), dtype=np.float64)

    for sample_index, xy in enumerate(observed[:, :2]):
        relative = xy - starts
        projection = np.zeros_like(squared_length)
        nondegenerate = squared_length > 0.0
        projection[nondegenerate] = (
            np.sum(relative[nondegenerate] * delta[nondegenerate], axis=1)
            / squared_length[nondegenerate]
        )
        projection = np.clip(projection, 0.0, 1.0)
        candidates = starts + projection[:, None] * delta
        squared_distance = np.sum(np.square(candidates - xy), axis=1)
        segment_index = int(np.argmin(squared_distance))
        fraction = float(projection[segment_index])
        segment_indices[sample_index] = segment_index
        fractions[sample_index] = fraction
        nearest_xy[sample_index] = candidates[segment_index]
        distances[sample_index] = math.sqrt(float(squared_distance[segment_index]))
        nearest_yaw[sample_index] = float(
            wrap_angle(ref[segment_index, 2] + fraction * yaw_delta[segment_index])
        )

    return {
        "distance_m": distances,
        "segment_index": segment_indices,
        "segment_fraction": fractions,
        "nearest_xy": nearest_xy,
        "nearest_yaw_rad": nearest_yaw,
        "yaw_error_rad": np.asarray(
            wrap_angle(observed[:, 2] - nearest_yaw), dtype=np.float64
        ),
    }


def spatial_reference_metrics(reference: ArrayLike, actual: ArrayLike) -> dict[str, Any]:
    """Summarize nearest-polyline deviation without a waypoint time assumption."""

    samples = nearest_polyline_samples(reference, actual)
    distances = samples["distance_m"]
    yaw = np.abs(samples["yaw_error_rad"])
    return {
        "old_reference_distance_mean_m": float(np.mean(distances)),
        "old_reference_distance_rms_m": _rmse(distances),
        "old_reference_distance_max_m": float(np.max(distances)),
        "old_reference_distance_final_m": float(distances[-1]),
        "old_nearest_segment_yaw_abs_mean_rad": float(np.mean(yaw)),
        "old_nearest_segment_yaw_rms_rad": _rmse(yaw),
        "old_nearest_segment_yaw_abs_max_rad": float(np.max(yaw)),
        "old_nearest_segment_yaw_abs_final_rad": float(yaw[-1]),
        "final_nearest_segment_index": int(samples["segment_index"][-1]),
        "metric_semantics": (
            "actual poses projected to the nearest OLD world-polyline segment; yaw uses the "
            "wrapped interpolation on that nearest segment; not time-aligned trajectory error"
        ),
    }


def execution_tracking_metrics(telemetry: DiagnosticTelemetry) -> dict[str, Any]:
    """Separate body-command and wheel-target execution discrepancies."""

    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    # Row zero describes the state before any diagnostic interval.  Commands
    # in rows 1: are the commands applied over the interval ending at the row.
    body_error = telemetry.commanded_body[1:] - np.column_stack(
        (measured_v[1:], measured_omega[1:])
    )
    wheel_error = telemetry.target_wheel_velocities[1:] - telemetry.measured_wheel_velocities[1:]
    return {
        "evaluation_sample_count": telemetry.sample_count - 1,
        "v_command_vs_measured_rmse_mps": _rmse(body_error[:, 0]),
        "omega_command_vs_measured_rmse_rps": _rmse(body_error[:, 1]),
        "commanded_v_mean_mps": float(np.mean(telemetry.commanded_body[1:, 0])),
        "measured_v_mean_mps": float(np.mean(measured_v[1:])),
        "commanded_omega_mean_rps": float(np.mean(telemetry.commanded_body[1:, 1])),
        "measured_omega_mean_rps": float(np.mean(measured_omega[1:])),
        "wheel_target_vs_measured_rmse_rad_s": _rmse(wheel_error),
        "wheel_target_vs_measured_rmse_by_corner_rad_s": {
            label: _rmse(wheel_error[:, index]) for index, label in enumerate(WHEEL_LABELS)
        },
        "body_motion_semantics": (
            "finite-difference body-forward velocity and wrapped yaw rate over each physics "
            "interval, compared with the command applied over that ending interval"
        ),
        "wheel_motion_semantics": (
            "articulation joint velocity target minus directly measured joint velocity, in "
            "canonical front_left/front_right/rear_left/rear_right order"
        ),
    }


def interpretation_status(
    *,
    old_spatial: Mapping[str, Any],
    old_execution: Mapping[str, Any],
    reset_translation_m: float,
    reset_yaw_rad: float,
    thresholds: Mapping[str, Any],
) -> dict[str, Any]:
    """Return non-causal visibility labels under documented thresholds."""

    values = {name: float(value) for name, value in thresholds.items()}
    if not values or not all(math.isfinite(value) and value >= 0.0 for value in values.values()):
        raise ValueError("diagnostic thresholds must be finite and non-negative")
    required = {
        "reference_rms_m",
        "body_v_rmse_mps",
        "body_omega_rmse_rps",
        "wheel_rmse_rad_s",
        "reset_translation_m",
        "reset_yaw_rad",
    }
    if set(values) != required:
        raise ValueError(f"diagnostic thresholds must contain exactly {sorted(required)}")
    labels: list[str] = []
    if float(old_spatial["old_reference_distance_rms_m"]) > values["reference_rms_m"]:
        labels.append("CONTROLLER_REFERENCE_DEVIATION_VISIBLE")
    if (
        float(old_execution["v_command_vs_measured_rmse_mps"]) > values["body_v_rmse_mps"]
        or float(old_execution["omega_command_vs_measured_rmse_rps"])
        > values["body_omega_rmse_rps"]
    ):
        labels.append("BODY_EXECUTION_MISMATCH_VISIBLE")
    if (
        float(old_execution["wheel_target_vs_measured_rmse_rad_s"])
        > values["wheel_rmse_rad_s"]
    ):
        labels.append("WHEEL_TRACKING_MISMATCH_VISIBLE")
    if (
        float(reset_translation_m) > values["reset_translation_m"]
        or abs(float(reset_yaw_rad)) > values["reset_yaw_rad"]
    ):
        labels.append("REPLAY_RESET_ARTIFACT_RELEVANT")
    if not labels:
        labels.append("INSUFFICIENT_EVIDENCE")
    return {
        "labels": labels,
        "thresholds": values,
        "semantics": (
            "descriptive threshold flags only; multiple labels may apply and none identifies "
            "a causal root cause"
        ),
    }


def validate_diagnosis_summary(summary: Mapping[str, Any]) -> dict[str, Any]:
    required = {
        "experiment",
        "source_trial_path",
        "source_provenance",
        "case_id",
        "entry_index",
        "method",
        "saved_boundary_se2",
        "reproduced_boundary_se2",
        "pre_reset_error",
        "exact_reset",
        "old_reference_spatial_metrics",
        "old_body_command_vs_measured_metrics",
        "old_wheel_target_vs_measured_metrics",
        "post_switch_controller_metrics",
        "post_switch_execution_metrics",
        "interpretation_status",
    }
    missing = sorted(required - set(summary))
    if missing:
        raise ValueError(f"diagnosis summary is missing fields: {', '.join(missing)}")
    if summary.get("experiment") != "EXP-02B GUI execution diagnosis":
        raise ValueError("invalid diagnosis experiment name")
    if summary.get("case_id") not in (
        "case_high_delta_v",
        "case_high_delta_omega",
        "case_benign_delayed",
    ):
        raise ValueError("invalid diagnostic case_id")
    if summary.get("method") not in ("raw_f0", "raw_k", "pose_anchor", "rigid", "graph"):
        raise ValueError("invalid diagnostic method")
    if not isinstance(summary.get("entry_index"), int) or int(summary["entry_index"]) < 0:
        raise ValueError("entry_index must be a non-negative integer")
    for key in ("saved_boundary_se2", "reproduced_boundary_se2"):
        value = np.asarray(summary[key], dtype=np.float64)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{key} must be a finite SE(2) pose")
    if summary["exact_reset"].get("occurred") is not True:
        raise ValueError("diagnostic summary must disclose the exact boundary reset")
    labels = summary["interpretation_status"].get("labels")
    if not isinstance(labels, list) or not labels or not all(isinstance(item, str) for item in labels):
        raise ValueError("interpretation_status must contain one or more labels")
    assert_finite_json_tree(summary)
    return dict(summary)


def telemetry_csv_rows(
    telemetry: DiagnosticTelemetry,
    *,
    command_indices: Sequence[int],
    progress: Sequence[tuple[int, int, bool]] | None = None,
) -> tuple[tuple[str, ...], list[list[Any]]]:
    if len(command_indices) != telemetry.sample_count:
        raise ValueError("command_indices length differs from telemetry")
    if progress is not None and len(progress) != telemetry.sample_count:
        raise ValueError("progress length differs from telemetry")
    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    rows: list[list[Any]] = []
    for index in range(telemetry.sample_count):
        row: list[Any] = [
            index,
            float(telemetry.sim_times_s[index]),
            int(command_indices[index]),
            *telemetry.actual_trajectory[index].tolist(),
            *telemetry.commanded_body[index].tolist(),
            float(measured_v[index]),
            float(measured_omega[index]),
            *telemetry.target_wheel_velocities[index].tolist(),
            *telemetry.measured_wheel_velocities[index].tolist(),
        ]
        if progress is not None:
            nearest, target, reached = progress[index]
            row.extend((int(nearest), int(target), bool(reached)))
        rows.append(row)
    return (POST_TELEMETRY_COLUMNS if progress is not None else OLD_TELEMETRY_COLUMNS), rows


def save_diagnostic_output(
    destination: str | Path,
    *,
    metadata: Mapping[str, Any],
    summary: Mapping[str, Any],
    old_telemetry: DiagnosticTelemetry,
    old_command_indices: Sequence[int],
    post_telemetry: DiagnosticTelemetry,
    post_command_indices: Sequence[int],
    post_progress: Sequence[tuple[int, int, bool]],
) -> Path:
    """Create one immutable diagnostic directory and all required artifacts."""

    output = Path(destination)
    validated_summary = validate_diagnosis_summary(summary)
    assert_finite_json_tree(metadata)
    output.mkdir(parents=True, exist_ok=False)
    np.save(output / "old_replay_actual.npy", old_telemetry.actual_trajectory)
    np.save(output / "post_switch_actual.npy", post_telemetry.actual_trajectory)
    for filename, telemetry, indices, progress in (
        ("old_replay_telemetry.csv", old_telemetry, old_command_indices, None),
        ("post_switch_telemetry.csv", post_telemetry, post_command_indices, post_progress),
    ):
        columns, rows = telemetry_csv_rows(telemetry, command_indices=indices, progress=progress)
        with (output / filename).open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            writer.writerows(rows)
    for filename, value in (
        ("metadata.json", metadata),
        ("diagnosis_summary.json", validated_summary),
    ):
        with (output / filename).open("x", encoding="utf-8") as stream:
            json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    return output


def validate_diagnostic_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    required = (
        "metadata.json",
        "diagnosis_summary.json",
        "old_replay_actual.npy",
        "old_replay_telemetry.csv",
        "post_switch_actual.npy",
        "post_switch_telemetry.csv",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"diagnostic output is missing files: {', '.join(missing)}")
    metadata = load_strict_json(root / "metadata.json")
    summary = validate_diagnosis_summary(load_strict_json(root / "diagnosis_summary.json"))
    old = validate_se2_trajectory(
        np.load(root / "old_replay_actual.npy", allow_pickle=False), name="old_replay_actual"
    )
    post = validate_se2_trajectory(
        np.load(root / "post_switch_actual.npy", allow_pickle=False), name="post_switch_actual"
    )
    for filename, columns, expected_count in (
        ("old_replay_telemetry.csv", OLD_TELEMETRY_COLUMNS, old.shape[0]),
        ("post_switch_telemetry.csv", POST_TELEMETRY_COLUMNS, post.shape[0]),
    ):
        with (root / filename).open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != columns:
                raise ValueError(f"{filename} columns differ from the diagnostic schema")
            rows = list(reader)
        if len(rows) != expected_count:
            raise ValueError(f"{filename} row count differs from its actual trajectory")
        for row in rows:
            for key, value in row.items():
                if key == "goal_reached":
                    if value not in ("True", "False"):
                        raise ValueError("goal_reached must be an explicit boolean")
                elif not math.isfinite(float(value)):
                    raise ValueError(f"{filename} contains a non-finite value in {key}")
    assert_finite_json_tree(metadata)
    return {
        "valid": True,
        "case_id": summary["case_id"],
        "entry_index": summary["entry_index"],
        "method": summary["method"],
        "old_sample_count": int(old.shape[0]),
        "post_switch_sample_count": int(post.shape[0]),
    }
