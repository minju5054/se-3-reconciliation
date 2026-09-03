"""Measured, waypoint-time-free online raw-switch analysis for EXP-01B."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.lightnav_adapter import (
    DECODED_OUTPUT_SEMANTICS,
    lightnav_local_to_world,
    raw_actions_to_local_path,
    validate_raw_lightnav_actions,
)
from reconciliation.metrics import TransitionMetrics, compute_transition_metrics
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
FORBIDDEN_MODEL_TIME_FIELDS = frozenset(
    {"waypoint_dt", "waypoint_dt_s", "waypoint_dt_seconds", "ready_time"}
)


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class MeasuredReadyTiming:
    observation_sim_time_s: float
    ready_sim_time_s: float
    request_host_monotonic_ns: int
    response_host_monotonic_ns: int

    def __post_init__(self) -> None:
        observation = _finite(self.observation_sim_time_s, "observation_sim_time_s")
        ready = _finite(self.ready_sim_time_s, "ready_sim_time_s")
        object.__setattr__(self, "observation_sim_time_s", observation)
        object.__setattr__(self, "ready_sim_time_s", ready)
        if ready <= observation:
            raise ValueError("NEW ready simulation time must be after observation time")
        if (
            not isinstance(self.request_host_monotonic_ns, int)
            or not isinstance(self.response_host_monotonic_ns, int)
            or self.request_host_monotonic_ns < 0
            or self.response_host_monotonic_ns <= self.request_host_monotonic_ns
        ):
            raise ValueError("host request/response monotonic timestamps are invalid")

    @property
    def simulation_latency_s(self) -> float:
        return self.ready_sim_time_s - self.observation_sim_time_s

    @property
    def request_response_wall_s(self) -> float:
        return (self.response_host_monotonic_ns - self.request_host_monotonic_ns) / 1e9

    @property
    def real_time_factor(self) -> float:
        return self.simulation_latency_s / self.request_response_wall_s


@dataclass(frozen=True, slots=True)
class OnlineReadySwitchAnalysis:
    timing: MeasuredReadyTiming
    robot_pose_at_new_observation: FloatArray
    robot_pose_at_new_ready: FloatArray
    actual_pose_before_ready: FloatArray
    new_first_pose_world: FloatArray
    old_progress_at_observation: int
    old_progress_at_ready: int
    old_exhausted_before_new_ready: bool
    observation_to_ready_translation_m: float
    observation_to_ready_yaw_rad: float
    transition: TransitionMetrics

    def to_dict(self) -> dict[str, Any]:
        return {
            "analysis": "live_measured_online_ready_switch",
            "raw_switch_policy": "use NEW row 0; no time-based stale-row deletion",
            "intrinsic_waypoint_time_base": False,
            "timing": {
                **asdict(self.timing),
                "simulation_observation_to_ready_s": self.timing.simulation_latency_s,
                "request_response_wall_s": self.timing.request_response_wall_s,
                "real_time_factor": self.timing.real_time_factor,
            },
            "robot_pose_at_new_observation": self.robot_pose_at_new_observation.tolist(),
            "robot_pose_at_new_ready": self.robot_pose_at_new_ready.tolist(),
            "actual_pose_before_ready": self.actual_pose_before_ready.tolist(),
            "new_first_pose_world": self.new_first_pose_world.tolist(),
            "old_progress_at_observation": self.old_progress_at_observation,
            "old_progress_at_ready": self.old_progress_at_ready,
            "old_exhausted_before_new_ready": self.old_exhausted_before_new_ready,
            "observation_to_ready_translation_m": self.observation_to_ready_translation_m,
            "observation_to_ready_yaw_rad": self.observation_to_ready_yaw_rad,
            "metrics": self.transition.to_dict(),
        }


def analyze_online_ready_switch(
    *,
    actual_pose_before_ready: ArrayLike,
    actual_pose_at_ready: ArrayLike,
    robot_pose_at_new_observation: ArrayLike,
    new_world_trajectory: ArrayLike,
    timing: MeasuredReadyTiming,
    old_progress_at_observation: int,
    old_progress_at_ready: int,
    old_exhausted_before_new_ready: bool,
) -> OnlineReadySwitchAnalysis:
    """Measure a live raw boundary without any waypoint-time/stale-row policy."""

    before = validate_pose_se2(actual_pose_before_ready, name="actual_pose_before_ready")
    ready = validate_pose_se2(actual_pose_at_ready, name="actual_pose_at_ready")
    observation = validate_pose_se2(
        robot_pose_at_new_observation, name="robot_pose_at_new_observation"
    )
    new_world = validate_se2_trajectory(new_world_trajectory, name="new_world_trajectory")
    if new_world.shape[0] < 2:
        raise ValueError("NEW trajectory requires at least two rows for motion metrics")
    for name, value in (
        ("old_progress_at_observation", old_progress_at_observation),
        ("old_progress_at_ready", old_progress_at_ready),
    ):
        if not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a nonnegative integer")
    if old_progress_at_ready < old_progress_at_observation:
        raise ValueError("OLD progress must not regress during NEW inference")
    movement = float(np.linalg.norm(ready[:2] - observation[:2]))
    yaw = float(wrap_angle(ready[2] - observation[2]))
    transition = compute_transition_metrics(before, ready, new_world[0], new_world[1])
    return OnlineReadySwitchAnalysis(
        timing=timing,
        robot_pose_at_new_observation=observation.copy(),
        robot_pose_at_new_ready=ready.copy(),
        actual_pose_before_ready=before.copy(),
        new_first_pose_world=new_world[0].copy(),
        old_progress_at_observation=old_progress_at_observation,
        old_progress_at_ready=old_progress_at_ready,
        old_exhausted_before_new_ready=bool(old_exhausted_before_new_ready),
        observation_to_ready_translation_m=movement,
        observation_to_ready_yaw_rad=yaw,
        transition=transition,
    )


def apply_reporting_thresholds(
    analysis: OnlineReadySwitchAnalysis,
    thresholds: Mapping[str, Any],
) -> dict[str, bool]:
    values = analysis.transition.to_dict()
    mapping = {
        "translation_gap_m": "exceeds_translation_threshold",
        "yaw_gap_rad": "exceeds_yaw_threshold",
        "translational_motion_jump_m": "exceeds_translational_motion_jump_threshold",
        "yaw_motion_jump_rad": "exceeds_yaw_motion_jump_threshold",
    }
    result: dict[str, bool] = {}
    for metric, output_name in mapping.items():
        threshold = _finite(thresholds[metric], metric)
        if threshold < 0.0:
            raise ValueError(f"threshold {metric} must be nonnegative")
        result[output_name] = values[metric] > threshold
    return result


def timing_validity(
    analysis: OnlineReadySwitchAnalysis,
    *,
    acceptable_rtf_range: Sequence[float],
    motion_noise_floor_m: float,
) -> dict[str, Any]:
    if len(acceptable_rtf_range) != 2:
        raise ValueError("acceptable_rtf_range must have two values")
    low = _finite(acceptable_rtf_range[0], "acceptable RTF low")
    high = _finite(acceptable_rtf_range[1], "acceptable RTF high")
    noise = _finite(motion_noise_floor_m, "motion_noise_floor_m")
    if low <= 0.0 or high < low or noise < 0.0:
        raise ValueError("RTF range/noise floor is invalid")
    checks = {
        "ready_after_observation": analysis.timing.ready_sim_time_s
        > analysis.timing.observation_sim_time_s,
        "rtf_in_range": low <= analysis.timing.real_time_factor <= high,
        "robot_moved_during_inference": analysis.observation_to_ready_translation_m > noise,
        "old_not_exhausted_before_ready": not analysis.old_exhausted_before_new_ready,
        "old_progress_nonregressing": analysis.old_progress_at_ready
        >= analysis.old_progress_at_observation,
    }
    return {"valid": all(checks.values()), "checks": checks}


def _stats(values: Sequence[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or array.size < 1 or not np.all(np.isfinite(array)):
        raise ValueError("aggregate metric list must be finite and non-empty")
    return {
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def aggregate_experiment(trials: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not trials:
        raise ValueError("at least one trial is required")
    valid = [dict(item) for item in trials if item.get("valid") is True]
    invalid = [dict(item) for item in trials if item.get("valid") is not True]
    metric_paths = {
        "host_new_latency_s": lambda row: row["timing"]["request_response_wall_s"],
        "simulation_new_latency_s": lambda row: row["timing"][
            "simulation_observation_to_ready_s"
        ],
        "real_time_factor": lambda row: row["timing"]["real_time_factor"],
        "translation_gap_m": lambda row: row["metrics"]["translation_gap_m"],
        "yaw_gap_rad": lambda row: row["metrics"]["yaw_gap_rad"],
        "translational_motion_jump_m": lambda row: row["metrics"][
            "translational_motion_jump_m"
        ],
        "yaw_motion_jump_rad": lambda row: row["metrics"]["yaw_motion_jump_rad"],
    }
    raw: dict[str, list[float]] = {
        key: [_finite(accessor(row), key) for row in valid]
        for key, accessor in metric_paths.items()
    }
    statistics = {key: _stats(values) for key, values in raw.items()} if valid else {}
    threshold_keys = (
        "exceeds_translation_threshold",
        "exceeds_yaw_threshold",
        "exceeds_translational_motion_jump_threshold",
        "exceeds_yaw_motion_jump_threshold",
    )
    exceedance_counts = {
        key: sum(bool(row["threshold_exceedance"][key]) for row in valid)
        for key in threshold_keys
    }
    return {
        "experiment": "EXP-01B",
        "valid_trial_count": len(valid),
        "invalid_trial_count": len(invalid),
        "trial_count": len(trials),
        "raw_lists": raw,
        "statistics": statistics,
        "threshold_exceedance_counts": exceedance_counts,
        "intrinsic_waypoint_time_base": False,
    }


def save_json_exclusive(path: str | Path, value: Any) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def save_npy_exclusive(path: str | Path, value: ArrayLike) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        np.save(stream, np.asarray(value), allow_pickle=False)
    return destination


def load_strict_json(path: str | Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(
            stream,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _find_forbidden_timing(value: Any, path: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(value, Mapping):
        for key, child in value.items():
            child_path = f"{path}.{key}" if path else str(key)
            if key in FORBIDDEN_MODEL_TIME_FIELDS:
                found.append(child_path)
            found.extend(_find_forbidden_timing(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_find_forbidden_timing(child, f"{path}[{index}]"))
    return found


def validate_trial_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    required = (
        "raw/old_actions.npy",
        "raw/old_raw_text.txt",
        "raw/new_actions.npy",
        "raw/new_raw_text.txt",
        "raw/new_observation_rgb.png",
        "raw/event_log.json",
        "derived/old_world.npy",
        "derived/new_world.npy",
        "derived/new_controller_reference.npy",
        "derived/actual_trajectory.npy",
        "derived/timeline.csv",
        "results/switch_metrics.json",
        "results/timing.json",
        "metadata.json",
    )
    missing = [item for item in required if not (root / item).is_file()]
    if missing:
        raise ValueError(f"trial output is missing: {', '.join(missing)}")
    metadata = load_strict_json(root / "metadata.json")
    metrics = load_strict_json(root / "results/switch_metrics.json")
    timing = load_strict_json(root / "results/timing.json")
    events = load_strict_json(root / "raw/event_log.json")
    forbidden = _find_forbidden_timing(metadata) + _find_forbidden_timing(metrics)
    if forbidden:
        raise ValueError(f"live output contains forbidden model timing fields: {forbidden}")
    horizon = int(metadata["action_horizon"])
    old_raw = validate_raw_lightnav_actions(
        np.load(root / "raw/old_actions.npy", allow_pickle=False), expected_horizon=horizon
    )
    new_raw = validate_raw_lightnav_actions(
        np.load(root / "raw/new_actions.npy", allow_pickle=False), expected_horizon=horizon
    )
    old_world = validate_se2_trajectory(np.load(root / "derived/old_world.npy", allow_pickle=False))
    new_world = validate_se2_trajectory(np.load(root / "derived/new_world.npy", allow_pickle=False))
    actual = validate_se2_trajectory(
        np.load(root / "derived/actual_trajectory.npy", allow_pickle=False)
    )
    controller_reference = validate_se2_trajectory(
        np.load(root / "derived/new_controller_reference.npy", allow_pickle=False)
    )
    expected_old = lightnav_local_to_world(
        raw_actions_to_local_path(old_raw, decoded_output_semantics=DECODED_OUTPUT_SEMANTICS),
        metadata["robot_pose_at_old_observation"],
    )
    expected_new = lightnav_local_to_world(
        raw_actions_to_local_path(new_raw, decoded_output_semantics=DECODED_OUTPUT_SEMANTICS),
        metrics["robot_pose_at_new_observation"],
    )
    if not np.array_equal(old_world, expected_old):
        raise ValueError("OLD world trajectory does not match its observation anchor")
    if not np.array_equal(new_world, expected_new):
        raise ValueError("NEW world trajectory does not match its observation anchor")
    if not np.array_equal(controller_reference[0], metrics["robot_pose_at_new_ready"]):
        raise ValueError("controller reference does not start at measured ready pose")
    if not np.array_equal(controller_reference[1:], new_world):
        raise ValueError("controller reference modified the derived NEW path")
    if metadata["raw_sha256"]["old_actions.npy"] != sha256_file(root / "raw/old_actions.npy"):
        raise ValueError("OLD raw action hash differs from metadata")
    if metadata["raw_sha256"]["new_actions.npy"] != sha256_file(root / "raw/new_actions.npy"):
        raise ValueError("NEW raw action hash differs from metadata")
    event_names = [event["event"] for event in events]
    required_events = (
        "episode_start",
        "old_observation",
        "old_ready",
        "old_execution_start",
        "new_observation",
        "new_request_sent",
        "new_ready",
        "raw_switch",
        "new_execution_start",
        "episode_end",
    )
    if any(name not in event_names for name in required_events):
        raise ValueError("event log does not contain the complete online sequence")
    with (root / "derived/timeline.csv").open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    expected_columns = {
        "sim_time_s",
        "host_monotonic_ns",
        "phase",
        "actual_x",
        "actual_y",
        "actual_yaw",
        "commanded_v",
        "commanded_omega",
        "active_chunk",
        "active_reference_index",
        "new_inference_in_flight",
        "rgb_frame_index",
    }
    if not rows or not expected_columns.issubset(rows[0]):
        raise ValueError("timeline.csv is empty or missing required columns")
    if timing.get("valid") is not metrics.get("valid"):
        raise ValueError("timing and switch metric validity disagree")
    return {
        "valid_output": True,
        "research_valid": bool(metrics["valid"]),
        "old_shape": list(old_raw.shape),
        "new_shape": list(new_raw.shape),
        "actual_shape": list(actual.shape),
        "timeline_rows": len(rows),
    }


def validate_experiment_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    summary = load_strict_json(root / "summary.json")
    metadata = load_strict_json(root / "metadata.json")
    if _find_forbidden_timing(summary) or _find_forbidden_timing(metadata):
        raise ValueError("experiment output fabricates a model waypoint time base")
    trial_dirs = sorted(item for item in root.glob("trial_*" ) if item.is_dir())
    declared_trial_count = metadata.get("attempted_trial_count", metadata.get("trial_count"))
    if declared_trial_count is None:
        raise ValueError("experiment metadata has no attempted trial count")
    if len(trial_dirs) != int(declared_trial_count):
        raise ValueError("experiment trial count differs from metadata")
    validated = [validate_trial_output(item) for item in trial_dirs]
    trial_metrics = [load_strict_json(item / "results/switch_metrics.json") for item in trial_dirs]
    rebuilt = aggregate_experiment(trial_metrics)
    if summary.get("aggregate") != rebuilt:
        raise ValueError("summary aggregate does not match trial artifacts")
    return {
        "valid_output": True,
        "trial_count": len(validated),
        "research_valid_trial_count": sum(item["research_valid"] for item in validated),
        "trials": validated,
    }
