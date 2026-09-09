"""Pure contracts and metrics for Stage 0-G3 moving visual history."""

from __future__ import annotations

from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from reconciliation.se2 import wrap_angle
from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    SCENARIO_IDS,
    VARIANT_IDS,
    sha256_file,
    validate_case as validate_g2_case,
    validate_config as validate_g2_config,
)


EXPECTED_G2_CONFIG_SHA256 = "49d339d16694d96dc5113271a2236e79ce413ae2411915e340885f8310fc9f01"
FINAL_STATUSES = {
    "STAGE0G3_READY_FOR_SUCCESSIVE_DATA_COLLECTION",
    "STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED",
    "STAGE0G3_APPROACH_PATH_NOT_FEASIBLE",
    "STAGE0G3_TECHNICAL_INVALID",
}
QUALIFICATION_CONDITION_KEYS = frozenset({
    "stage0g2_control_validated",
    "all_final_observation_poses_equal",
    "all_histories_verified_nonzero_movement",
    "q0_straight_at_least_4_of_5",
    "q1_left_at_least_4_of_5",
    "q2_right_at_least_4_of_5",
    "doorway_distinct_or_detour_family_at_least_4_of_5",
    "minimum_five_unique_moving_raw_outputs",
    "dominant_exact_raw_group_within_limit",
    "no_invalid_or_stop_dominance",
})


def _finite_pose(value: np.ndarray, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be finite SE(2)")
    return pose


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("configuration must be a mapping")
    return value


def validate_config(config: Mapping[str, Any], repository_root: str | Path | None = None) -> None:
    if config.get("stage") != "stage0-g3-moving-history-qualification":
        raise ValueError("invalid Stage 0-G3 stage name")
    control = config.get("frozen_control", {})
    if control.get("expected_config_sha256") != EXPECTED_G2_CONFIG_SHA256:
        raise ValueError("frozen Stage 0-G2 config SHA is not exact")
    if control.get("expected_status") != "STAGE0G2_SCENE_QUALIFICATION_FAILED" or int(control.get("expected_case_count", 0)) != 30:
        raise ValueError("frozen Stage 0-G2 result contract changed")
    history = config.get("history", {})
    if (
        history.get("mode") != "moving_scripted"
        or int(history.get("frame_count", 0)) != 64
        or float(history.get("fps", 0.0)) != 4.0
        or history.get("interpolation") != "linear_se2_straight_approach"
        or history.get("yaw_policy") != "constant_final_yaw"
        or history.get("approach_direction") != "behind_final_heading"
    ):
        raise ValueError("Stage 0-G3 moving-history rule is frozen")
    maximum = float(history.get("maximum_common_distance_m", math.nan))
    minimum = float(history.get("minimum_common_distance_m", math.nan))
    step = float(history.get("distance_search_step_m", math.nan))
    if not (math.isfinite(maximum + minimum + step) and maximum == 1.5 and minimum == 0.8 and step == 0.1):
        raise ValueError("invalid common approach-distance policy")
    if float(history.get("final_translation_tolerance_m", 0.0)) != 1e-6 or float(history.get("final_yaw_tolerance_rad", 0.0)) != 1e-6:
        raise ValueError("final-pose equality tolerance changed")
    paired = config.get("paired_comparison", {})
    if paired != {
        "raw_equality": "exact_array_equal",
        "translation_difference": "rowwise_euclidean_rms_and_max",
        "yaw_difference": "wrapped_rowwise_rms_and_max",
        "endpoint_difference": "moving_minus_stationary_local",
        "class_transition": "frozen_stage0g2_geometry_classifier",
    }:
        raise ValueError("paired comparison contract changed")
    qualification = config.get("qualification", {})
    if qualification != {
        "required_matches_per_scenario": 4,
        "minimum_unique_raw_outputs": 5,
        "maximum_dominant_exact_raw_fraction": 0.80,
        "maximum_stop_or_nonmoving_count": 6,
    }:
        raise ValueError("Stage 0-G3 qualification thresholds changed")
    if repository_root is not None:
        root = Path(repository_root).resolve()
        control_path = (root / str(control["config_path"])).resolve()
        if sha256_file(control_path) != EXPECTED_G2_CONFIG_SHA256:
            raise ValueError("repository Stage 0-G2 config hash mismatch")
        validate_g2_config(load_yaml(control_path))


def resolved_scientific_config(config: Mapping[str, Any], repository_root: str | Path) -> dict[str, Any]:
    """Return G2 scientific fields with only the G3 history identity overlaid."""
    validate_config(config, repository_root)
    root = Path(repository_root).resolve()
    g2 = load_yaml(root / str(config["frozen_control"]["config_path"]))
    validate_g2_config(g2)
    result = copy.deepcopy(g2)
    result["stage"] = str(config["stage"])
    result["capture"]["stationary_history"] = False
    result["capture"]["history_mode"] = "moving_scripted"
    result["stage0g3"] = copy.deepcopy(dict(config))
    return result


def approach_history(final_pose_se2: np.ndarray, distance_m: float, frame_count: int = 64) -> np.ndarray:
    final = _finite_pose(final_pose_se2, "final_pose_se2")
    distance = float(distance_m)
    if not math.isfinite(distance) or distance <= 0.0 or frame_count != 64:
        raise ValueError("approach requires positive distance and exactly 64 poses")
    progress = np.linspace(0.0, 1.0, frame_count, dtype=np.float64)
    direction = np.array([math.cos(final[2]), math.sin(final[2])], dtype=np.float64)
    poses = np.empty((frame_count, 3), dtype=np.float64)
    poses[:, :2] = final[:2] - (1.0 - progress[:, None]) * distance * direction
    poses[:, 2] = final[2]
    poses[-1] = final
    return poses


def validate_history_poses(
    poses: np.ndarray,
    final_pose_se2: np.ndarray,
    distance_m: float,
    translation_tolerance_m: float = 1e-6,
    yaw_tolerance_rad: float = 1e-6,
) -> dict[str, float | bool | int]:
    values = np.asarray(poses, dtype=np.float64)
    original = np.asarray(poses).copy()
    final = _finite_pose(final_pose_se2, "final_pose_se2")
    if values.shape != (64, 3) or not np.all(np.isfinite(values)):
        raise ValueError("moving history must be finite 64x3 SE(2)")
    yaw_error = np.asarray([wrap_angle(float(x - final[2])) for x in values[:, 2]])
    final_translation = float(np.linalg.norm(values[-1, :2] - final[:2]))
    final_yaw = abs(float(wrap_angle(float(values[-1, 2] - final[2]))))
    radial = np.linalg.norm(values[:, :2] - final[:2], axis=1)
    steps = np.linalg.norm(np.diff(values[:, :2], axis=0), axis=1)
    if final_translation > translation_tolerance_m or final_yaw > yaw_tolerance_rad:
        raise ValueError("moving history final pose does not equal frozen G2 observation pose")
    # Base-pose reads are float32 in Isaac; around 32 m their ULP exceeds the
    # 1 um final-pairing gate. The final pose remains subject to the strict
    # tolerance, while the derived start-distance check allows 10 um.
    if abs(float(radial[0]) - float(distance_m)) > max(translation_tolerance_m, 1e-5):
        raise ValueError("moving history start is not the common distance behind final")
    if np.max(np.abs(yaw_error)) > yaw_tolerance_rad:
        raise ValueError("moving history yaw is not constant")
    if np.any(np.diff(radial) > translation_tolerance_m) or np.any(steps <= 0.0):
        raise ValueError("moving history progress is not strictly monotonic")
    if not np.array_equal(np.asarray(poses), original, equal_nan=True):
        raise AssertionError("source history array was mutated")
    return {
        "frame_count": 64,
        "nonzero_motion": bool(np.all(steps > 0.0)),
        "history_translation_m": float(np.sum(steps)),
        "mean_interframe_translation_m": float(np.mean(steps)),
        "max_interframe_translation_m": float(np.max(steps)),
        "max_abs_yaw_change_rad": float(np.max(np.abs(yaw_error))),
        "final_translation_mismatch_m": final_translation,
        "final_yaw_mismatch_rad": final_yaw,
    }


def validate_timestamps(timestamps_s: np.ndarray, fps: float = 4.0) -> dict[str, float | int]:
    values = np.asarray(timestamps_s, dtype=np.float64)
    if values.shape != (64,) or not np.all(np.isfinite(values)):
        raise ValueError("frame timestamps must be finite length 64")
    deltas = np.diff(values)
    # Isaac/G2 records the simulator clock directly; 15 float physics ticks are
    # 0.250000013... s rather than an analytically exact 0.25 s.
    if np.any(deltas <= 0.0) or not np.allclose(deltas, 1.0 / float(fps), atol=1e-6, rtol=0.0):
        raise ValueError("frame timestamps must be strictly ordered at frozen 4 Hz")
    return {"frame_count": 64, "duration_s": float(values[-1] - values[0]), "mean_period_s": float(np.mean(deltas))}


def paired_trajectory_metrics(stationary: np.ndarray, moving: np.ndarray) -> dict[str, Any]:
    left = np.asarray(stationary)
    right = np.asarray(moving)
    left_before, right_before = left.copy(), right.copy()
    if left.shape != (10, 3) or right.shape != (10, 3) or left.dtype != np.float32 or right.dtype != np.float32:
        raise ValueError("paired LightNav outputs must both be float32 10x3")
    if not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        raise ValueError("paired LightNav outputs must be finite")
    translation = np.linalg.norm(right[:, :2].astype(np.float64) - left[:, :2].astype(np.float64), axis=1)
    yaw = np.asarray([wrap_angle(float(value)) for value in right[:, 2].astype(np.float64) - left[:, 2].astype(np.float64)])
    endpoint = right[-1].astype(np.float64) - left[-1].astype(np.float64)
    endpoint[2] = wrap_angle(float(endpoint[2]))
    if not np.array_equal(left, left_before) or not np.array_equal(right, right_before):
        raise AssertionError("source LightNav arrays were mutated")
    return {
        "exact_raw_equal": bool(np.array_equal(left, right)),
        "translation_rms_difference_m": float(np.sqrt(np.mean(np.square(translation)))),
        "translation_max_difference_m": float(np.max(translation)),
        "wrapped_yaw_rms_difference_rad": float(np.sqrt(np.mean(np.square(yaw)))),
        "wrapped_yaw_max_abs_difference_rad": float(np.max(np.abs(yaw))),
        "delta_endpoint_forward_m": float(endpoint[0]),
        "delta_endpoint_lateral_m": float(endpoint[1]),
        "delta_endpoint_yaw_rad": float(endpoint[2]),
    }


def class_transition(stationary_class: str, moving_class: str) -> dict[str, str | bool]:
    """Describe a paired frozen-classifier transition without changing either label."""
    if not isinstance(stationary_class, str) or not stationary_class:
        raise ValueError("stationary_class must be a non-empty string")
    if not isinstance(moving_class, str) or not moving_class:
        raise ValueError("moving_class must be a non-empty string")
    return {
        "stationary_class": stationary_class,
        "moving_class": moving_class,
        "class_transition": f"{stationary_class}->{moving_class}",
        "class_changed": stationary_class != moving_class,
    }


def qualification_status(conditions: Mapping[str, Any], *, approach_feasible: bool = True, technical_valid: bool = True) -> str:
    if not technical_valid:
        return "STAGE0G3_TECHNICAL_INVALID"
    if not approach_feasible:
        return "STAGE0G3_APPROACH_PATH_NOT_FEASIBLE"
    if not conditions or any(not isinstance(value, (bool, np.bool_)) for value in conditions.values()):
        raise ValueError("qualification conditions must be non-empty booleans")
    return "STAGE0G3_READY_FOR_SUCCESSIVE_DATA_COLLECTION" if all(bool(value) for value in conditions.values()) else "STAGE0G3_MOVING_HISTORY_QUALIFICATION_FAILED"


def dominant_exact_raw_fraction(named_arrays: Mapping[str, np.ndarray]) -> float:
    if not named_arrays:
        raise ValueError("at least one raw output is required")
    hashes = [hashlib.sha256(np.asarray(value).tobytes()).hexdigest() for value in named_arrays.values()]
    return max(Counter(hashes).values()) / len(hashes)


def validate_frozen_control(config: Mapping[str, Any], repository_root: str | Path) -> dict[str, Any]:
    """Strictly validate the saved G2 control without modifying it."""
    validate_config(config, repository_root)
    root = Path(repository_root).resolve()
    control = config["frozen_control"]
    run = (root / str(control["primary_run"])).resolve()
    control_config_path = root / str(control["config_path"])
    snapshot = run / "config_snapshot.yaml"
    if not run.is_dir() or sha256_file(snapshot) != EXPECTED_G2_CONFIG_SHA256 or sha256_file(control_config_path) != EXPECTED_G2_CONFIG_SHA256:
        raise ValueError("frozen Stage 0-G2 config/run is missing or changed")
    g2 = load_yaml(snapshot)
    validate_g2_config(g2)
    result = json.loads((run / "qualification_result.json").read_text(encoding="utf-8"))
    if result.get("status") != control["expected_status"]:
        raise ValueError("frozen Stage 0-G2 status changed")
    scenarios = {item["id"]: item for item in g2["scenarios"]}
    cases: dict[str, Any] = {}
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            case = run / "qualification" / scenario_id / variant_id
            validation = validate_g2_case(case, expected_horizon=10)
            capture = json.loads((case / "input/capture_metadata.json").read_text(encoding="utf-8"))
            metadata = json.loads((case / "metadata.json").read_text(encoding="utf-8"))
            if capture.get("instruction") != scenarios[scenario_id]["instruction"] or metadata.get("instruction") != scenarios[scenario_id]["instruction"]:
                raise ValueError(f"G2 instruction mismatch: {scenario_id}/{variant_id}")
            pose = _finite_pose(np.asarray(capture["robot_pose_at_observation"]), "G2 observation pose")
            cases[f"{scenario_id}/{variant_id}"] = {
                "instruction": capture["instruction"],
                "final_pose_se2": pose.tolist(),
                "raw_sha256": validation["raw_sha256"],
            }
    if len(cases) != int(control["expected_case_count"]):
        raise ValueError("frozen Stage 0-G2 case count changed")
    return {
        "valid": True,
        "config_path": str(control_config_path),
        "config_sha256": EXPECTED_G2_CONFIG_SHA256,
        "primary_run": str(run),
        "result_status": result["status"],
        "case_count": len(cases),
        "scenario_ids": list(SCENARIO_IDS),
        "variant_ids": list(VARIANT_IDS),
        "cases": cases,
    }


__all__ = [
    "EXPECTED_G2_CONFIG_SHA256", "FINAL_STATUSES", "QUALIFICATION_CONDITION_KEYS", "SCENARIO_IDS", "VARIANT_IDS",
    "approach_history", "class_transition", "dominant_exact_raw_fraction", "load_yaml", "paired_trajectory_metrics",
    "qualification_status", "resolved_scientific_config", "sha256_file", "validate_config",
    "validate_frozen_control", "validate_history_poses", "validate_timestamps",
]
