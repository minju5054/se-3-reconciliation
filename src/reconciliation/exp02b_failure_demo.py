"""Read-only evidence and discrete playback timing for the EXP-02B OLD failure demo."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .controller_validation import estimate_body_velocities
from .exp02b_diagnosis import (
    DiagnosticTelemetry, WHEEL_LABELS, execution_tracking_metrics,
    spatial_reference_metrics, validate_diagnostic_output,
)
from .online_switch import load_strict_json, sha256_file
from .trajectory import validate_se2_trajectory


def checked_file(path: Path, expected_sha256: str) -> Path:
    if sha256_file(path) != expected_sha256:
        raise ValueError(f"archived source hash mismatch: {path}")
    return path


def read_old_telemetry(path: Path, poses: np.ndarray) -> tuple[DiagnosticTelemetry, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    def columns(*names: str) -> np.ndarray:
        values = np.asarray([[float(row[name]) for name in names] for row in rows])
        if not np.all(np.isfinite(values)):
            raise ValueError("non-finite archived telemetry")
        return values

    csv_poses = columns("actual_x", "actual_y", "actual_yaw")
    if not np.array_equal(csv_poses, poses):
        raise ValueError("CSV poses differ from archived NPY")
    if not np.array_equal(columns("sample_index")[:, 0], np.arange(len(poses))):
        raise ValueError("archived sample indices must be consecutive from zero")
    telemetry = DiagnosticTelemetry(
        poses.copy(), columns("sim_time_s")[:, 0],
        columns("commanded_v_mps", "commanded_omega_rps"),
        columns(*(f"target_{label}_rad_s" for label in WHEEL_LABELS)),
        columns(*(f"measured_{label}_rad_s" for label in WHEEL_LABELS)),
    )
    measured = columns("measured_body_v_mps", "measured_body_omega_rps")
    reconstructed = np.column_stack(estimate_body_velocities(poses, telemetry.sim_times_s))
    if not np.allclose(measured, reconstructed, rtol=0, atol=1e-12):
        raise ValueError("saved body velocities differ from ending-interval reconstruction")
    for array in (
        telemetry.actual_trajectory, telemetry.sim_times_s, telemetry.commanded_body,
        telemetry.target_wheel_velocities, telemetry.measured_wheel_velocities, measured,
    ):
        array.flags.writeable = False
    return telemetry, measured


def verify_metric_subset(saved: Mapping[str, Any], computed: Mapping[str, Any]) -> None:
    for key, expected in saved.items():
        if isinstance(expected, dict):
            verify_metric_subset(expected, computed[key])
        elif isinstance(expected, (int, float)):
            if not math.isclose(expected, computed[key], rel_tol=0, abs_tol=1e-12):
                raise ValueError(f"archived metric does not reproduce: {key}")


def saved_sample_index(times: np.ndarray, elapsed_wall_s: float, duration_wall_s: float) -> int:
    """Select the last recorded sample at/before mapped time; never interpolate SE(2).

    The GUI freezes elapsed_wall_s while paused. Replay starts a new wall-clock mapping.
    Row zero is pre-interval; telemetry in subsequent rows describes the ending interval.
    """
    values = np.asarray(times, dtype=float)
    if (values.ndim != 1 or len(values) < 2 or not np.all(np.isfinite(values))
            or values[0] != 0 or np.any(np.diff(values) <= 0)):
        raise ValueError("playback requires increasing saved times starting at zero")
    if not math.isfinite(elapsed_wall_s) or elapsed_wall_s < 0:
        raise ValueError("elapsed wall time must be finite and nonnegative")
    if not math.isfinite(duration_wall_s) or duration_wall_s <= 0:
        raise ValueError("presentation duration must be positive and finite")
    if elapsed_wall_s >= duration_wall_s:
        return len(values) - 1
    mapped = elapsed_wall_s / duration_wall_s * values[-1]
    return max(0, int(np.searchsorted(values, mapped, side="right") - 1))


@dataclass(frozen=True)
class FailureEvidence:
    config: dict[str, Any]
    metadata: dict[str, Any]
    summary: dict[str, Any]
    telemetry: DiagnosticTelemetry
    measured_body: np.ndarray
    old_reference: np.ndarray
    hashes: dict[str, str]


def load_failure_evidence(root: Path, config_path: Path) -> FailureEvidence:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    source = root / config["source_run"]
    hashes = {}
    for name, digest in config["source_sha256"].items():
        path = checked_file(source / name, digest)
        hashes[str(path.resolve())] = digest
    validate_diagnostic_output(source)
    metadata = load_strict_json(source / "metadata.json")
    summary = load_strict_json(source / "diagnosis_summary.json")
    if (summary["case_id"], summary["entry_index"], summary["method"]) != (
        "case_high_delta_omega", 3, "raw_k"
    ):
        raise ValueError("demo requires the archived high-omega k=3 raw_k diagnosis")
    provenance = summary["source_provenance"]
    for name, digest in provenance["source_trial_sha256"].items():
        path = checked_file(Path(summary["source_trial_path"]) / name, digest)
        hashes[str(path)] = digest
    old_path = Path(summary["source_trial_path"]) / "derived/old_world.npy"
    reference = validate_se2_trajectory(np.load(old_path, allow_pickle=False))
    poses = np.load(source / "old_replay_actual.npy", allow_pickle=False)
    telemetry, measured = read_old_telemetry(source / "old_replay_telemetry.csv", poses)
    verify_metric_subset(summary["old_reference_spatial_metrics"], spatial_reference_metrics(reference, poses))
    metrics = execution_tracking_metrics(telemetry)
    verify_metric_subset(summary["old_body_command_vs_measured_metrics"], metrics)
    verify_metric_subset(summary["old_wheel_target_vs_measured_metrics"], metrics)
    if not np.array_equal(poses[-1], summary["reproduced_boundary_se2"]):
        raise ValueError("OLD replay must end at reproduced B, before exact reset")
    for key in ("exp01b_config", "exp02b_config"):
        path = checked_file(Path(provenance[f"{key}_path"]), provenance[f"{key}_sha256"])
        hashes[str(path)] = provenance[f"{key}_sha256"]
    hashes[str(config_path.resolve())] = sha256_file(config_path)
    reference.flags.writeable = False
    return FailureEvidence(config, metadata, summary, telemetry, measured, reference, hashes)
