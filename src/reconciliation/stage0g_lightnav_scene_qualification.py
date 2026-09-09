"""Pure Stage 0-G input-contract and LightNav scene-qualification helpers."""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.lightnav_adapter import (
    lightnav_local_to_world,
    validate_raw_lightnav_actions,
)
from reconciliation.se2 import wrap_angle


SCENARIO_IDS = (
    "Q0_STRAIGHT",
    "Q1_LEFT_TURN",
    "Q2_RIGHT_TURN",
    "Q3_DOORWAY",
    "Q4_DETOUR_LEFT",
    "Q5_DETOUR_RIGHT",
)
VARIANT_IDS = ("V0", "V1", "V2", "V3", "V4")
FINAL_STATUSES = {
    "STAGE0G_READY_FOR_DATA_COLLECTION",
    "STAGE0G_INPUT_CONTRACT_NOT_RESOLVED",
    "STAGE0G_SCENE_QUALIFICATION_FAILED",
}


def create_run_directory_exclusive(path: str | Path) -> Path:
    """Create a new run root and reject even an otherwise-empty existing directory."""

    destination = Path(path)
    destination.mkdir(parents=True, exist_ok=False)
    return destination


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_array(array: ArrayLike) -> str:
    value = np.ascontiguousarray(np.asarray(array))
    return hashlib.sha256(value.tobytes()).hexdigest()


def load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        value = json.load(
            stream,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def validate_config(config: Mapping[str, Any]) -> None:
    required = {
        "stage", "paths", "baseline", "lightnav", "simulation", "robot", "camera",
        "capture", "scene", "variants", "scenarios", "qualification", "visualization",
    }
    missing = sorted(required.difference(config))
    if missing:
        raise ValueError(f"Stage 0-G config missing fields: {missing}")
    if config["stage"] != "stage0-g-lightnav-scene-qualification":
        raise ValueError("invalid Stage 0-G stage name")
    camera = config["camera"]
    if camera.get("rgb_format") != "HWC_uint8_RGB":
        raise ValueError("Stage 0-G requires HWC uint8 RGB")
    if camera.get("orientation") != "level_robot_forward":
        raise ValueError("Stage 0-G camera must be level and robot-forward")
    if [int(v) for v in camera.get("model_preprocess_size_hw", [])] != [256, 448]:
        raise ValueError("checkpoint preprocessing size must be [H=256, W=448]")
    if config["lightnav"].get("aspect_mode") != "stretch":
        raise ValueError("released LightNav qualification must use the training stretch profile")
    if bool(config["lightnav"].get("intrinsic_waypoint_time_base")):
        raise ValueError("LightNav waypoint rows have no intrinsic time base")
    if int(config["capture"].get("frame_count", 0)) != 64:
        raise ValueError("qualification history must contain exactly 64 frames")
    if not math.isclose(float(config["capture"].get("fps", 0.0)), 4.0):
        raise ValueError("qualification history cadence must be 4 Hz")
    scenarios = list(config["scenarios"])
    variants = list(config["variants"])
    if tuple(item.get("id") for item in scenarios) != SCENARIO_IDS:
        raise ValueError("Stage 0-G requires the six ordered unique scenario IDs")
    if tuple(item.get("id") for item in variants) != VARIANT_IDS:
        raise ValueError("Stage 0-G requires five ordered deterministic variants")
    poses = []
    for scenario in scenarios:
        pose = np.asarray(scenario.get("baseline_pose_se2"), dtype=np.float64)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)):
            raise ValueError(f"invalid baseline pose for {scenario.get('id')}")
        poses.append(tuple(pose))
    if len(set(poses)) != len(poses):
        raise ValueError("scenario baseline poses must be unique")
    by_id = {item["id"]: item for item in variants}
    if float(by_id["V1"]["yaw_offset_rad"]) != -float(by_id["V2"]["yaw_offset_rad"]):
        raise ValueError("V1/V2 yaw perturbations must be mirrored")
    if float(by_id["V3"]["lateral_offset_m"]) != -float(by_id["V4"]["lateral_offset_m"]):
        raise ValueError("V3/V4 lateral perturbations must be mirrored")
    for variant in variants:
        values = (float(variant["yaw_offset_rad"]), float(variant["lateral_offset_m"]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("variant offsets must be finite")
        if abs(values[0]) > 0.1 or abs(values[1]) > 0.15:
            raise ValueError("nearby-state perturbation exceeds the Stage 0-G safety envelope")


def scenario_map(config: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    validate_config(config)
    return {str(item["id"]): item for item in config["scenarios"]}


def variant_map(config: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    validate_config(config)
    return {str(item["id"]): item for item in config["variants"]}


def variant_pose(config: Mapping[str, Any], scenario_id: str, variant_id: str) -> np.ndarray:
    scenario = scenario_map(config)[scenario_id]
    variant = variant_map(config)[variant_id]
    base = np.asarray(scenario["baseline_pose_se2"], dtype=np.float64)
    yaw = float(base[2])
    lateral = float(variant["lateral_offset_m"])
    # Positive lateral is left in the baseline robot frame.
    offset = np.array([-math.sin(yaw) * lateral, math.cos(yaw) * lateral])
    return np.array(
        [base[0] + offset[0], base[1] + offset[1], wrap_angle(yaw + float(variant["yaw_offset_rad"]))],
        dtype=np.float64,
    )


def _origin_augmented(path: np.ndarray) -> np.ndarray:
    return np.vstack((np.zeros((1, 3), dtype=np.float64), path.astype(np.float64, copy=False)))


def geometry_descriptors(actions: ArrayLike, *, stop_path_length_m: float = 0.01) -> dict[str, Any]:
    raw = validate_raw_lightnav_actions(actions)
    path = raw.astype(np.float64, copy=False)
    augmented = _origin_augmented(path)
    xy_steps = np.diff(augmented[:, :2], axis=0)
    distances = np.linalg.norm(xy_steps, axis=1)
    yaw_steps = np.asarray(wrap_angle(np.diff(augmented[:, 2])), dtype=np.float64)
    moving = distances > 1e-9
    headings = np.arctan2(xy_steps[moving, 1], xy_steps[moving, 0])
    tangent_steps = (
        np.asarray(wrap_angle(np.diff(np.concatenate(([0.0], headings)))), dtype=np.float64)
        if headings.size
        else np.zeros(0, dtype=np.float64)
    )
    nonzero_signs = np.sign(tangent_steps[np.abs(tangent_steps) > 1e-6])
    sign_changes = int(np.count_nonzero(np.diff(nonzero_signs) != 0)) if nonzero_signs.size > 1 else 0
    path_length = float(np.sum(distances))
    exact_zero = bool(np.array_equal(raw, np.zeros_like(raw)))
    return {
        "pose_count": int(path.shape[0]),
        "path_length_m": path_length,
        "endpoint_forward_m": float(path[-1, 0]),
        "endpoint_lateral_m": float(path[-1, 1]),
        "max_left_lateral_excursion_m": float(np.max(np.concatenate(([0.0], path[:, 1])))),
        "max_right_lateral_excursion_m": float(np.min(np.concatenate(([0.0], path[:, 1])))),
        "max_abs_lateral_excursion_m": float(np.max(np.abs(path[:, 1]))),
        "signed_net_yaw_rad": float(wrap_angle(path[-1, 2])),
        "cumulative_absolute_yaw_rad": float(np.sum(np.abs(yaw_steps))),
        "maximum_yaw_increment_rad": float(np.max(np.abs(yaw_steps))) if yaw_steps.size else 0.0,
        "signed_tangent_turn_rad": float(np.sum(tangent_steps)),
        "cumulative_absolute_tangent_turn_rad": float(np.sum(np.abs(tangent_steps))),
        "curvature_sign_changes": sign_changes,
        "exact_stop_output": exact_zero,
        "nonmoving": bool(path_length <= float(stop_path_length_m)),
        "intrinsic_waypoint_time_base": False,
    }


def classify_descriptors(
    descriptors: Mapping[str, Any], intended_class: str, criteria: Mapping[str, Any]
) -> dict[str, Any]:
    d = descriptors
    meaningful = float(d["path_length_m"]) >= float(criteria["meaningful_path_length_m"])
    forward = float(d["endpoint_forward_m"]) >= float(criteria["meaningful_forward_progress_m"])
    if intended_class == "straight":
        c = criteria["straight"]
        matched = (
            meaningful
            and abs(float(d["endpoint_lateral_m"])) <= float(c["max_abs_endpoint_lateral_m"])
            and abs(float(d["signed_net_yaw_rad"])) <= float(c["max_abs_net_yaw_rad"])
            and float(d["max_abs_lateral_excursion_m"]) <= float(c["max_abs_lateral_excursion_m"])
        )
    elif intended_class == "left":
        c = criteria["left"]
        positive = float(d["endpoint_lateral_m"]) >= float(c["min_endpoint_lateral_m"]) or float(d["signed_net_yaw_rad"]) >= float(c["min_net_yaw_rad"])
        contradictory = float(d["endpoint_lateral_m"]) < float(c["contradictory_endpoint_lateral_m"]) or float(d["signed_net_yaw_rad"]) < float(c["contradictory_net_yaw_rad"])
        matched = meaningful and positive and not contradictory
    elif intended_class == "right":
        c = criteria["right"]
        negative = float(d["endpoint_lateral_m"]) <= float(c["max_endpoint_lateral_m"]) or float(d["signed_net_yaw_rad"]) <= float(c["max_net_yaw_rad"])
        contradictory = float(d["endpoint_lateral_m"]) > float(c["contradictory_endpoint_lateral_m"]) or float(d["signed_net_yaw_rad"]) > float(c["contradictory_net_yaw_rad"])
        matched = meaningful and negative and not contradictory
    elif intended_class == "doorway":
        matched = meaningful and forward
    elif intended_class == "detour_left":
        matched = meaningful and forward and float(d["max_left_lateral_excursion_m"]) >= float(criteria["detour_left"]["min_left_excursion_m"])
    elif intended_class == "detour_right":
        matched = meaningful and forward and float(d["max_right_lateral_excursion_m"]) <= float(criteria["detour_right"]["max_right_excursion_m"])
    else:
        raise ValueError(f"unknown intended class: {intended_class}")
    return {"intended_class": intended_class, "valid": True, "meaningful": meaningful, "matches_intended_geometry": bool(matched)}


def geometry_signature(descriptors: Mapping[str, Any]) -> str:
    def sign(value: float, epsilon: float = 0.05) -> str:
        return "+" if value > epsilon else "-" if value < -epsilon else "0"
    return ":".join(
        (
            "STOP" if descriptors["nonmoving"] else "MOVE",
            sign(float(descriptors["endpoint_forward_m"])),
            sign(float(descriptors["endpoint_lateral_m"])),
            sign(float(descriptors["signed_net_yaw_rad"])),
            sign(float(descriptors["signed_tangent_turn_rad"])),
        )
    )


def duplicate_groups(named_arrays: Mapping[str, ArrayLike]) -> dict[str, list[str]]:
    groups: dict[str, list[str]] = defaultdict(list)
    for name, value in named_arrays.items():
        raw = validate_raw_lightnav_actions(value)
        groups[sha256_array(raw)].append(name)
    return {digest: names for digest, names in sorted(groups.items()) if len(names) > 1}


def pairwise_trajectory_distance(named_arrays: Mapping[str, ArrayLike]) -> dict[str, Any]:
    names = sorted(named_arrays)
    arrays = {name: validate_raw_lightnav_actions(named_arrays[name]).astype(np.float64) for name in names}
    sizes = {value.shape for value in arrays.values()}
    if len(sizes) != 1:
        raise ValueError("pairwise comparison requires equal trajectory shapes")
    matrix = np.zeros((len(names), len(names)), dtype=np.float64)
    for i, left in enumerate(names):
        for j, right in enumerate(names):
            delta_xy = arrays[left][:, :2] - arrays[right][:, :2]
            delta_yaw = np.asarray(wrap_angle(arrays[left][:, 2] - arrays[right][:, 2]))
            matrix[i, j] = math.sqrt(float(np.mean(np.sum(delta_xy * delta_xy, axis=1) + delta_yaw * delta_yaw)))
    return {"names": names, "metric": "RMS sqrt(dx^2+dy^2+wrapped_dyaw^2); descriptive mixed-unit distance", "matrix": matrix.tolist()}


def validate_case(case_dir: str | Path, expected_horizon: int = 10) -> dict[str, Any]:
    root = Path(case_dir)
    required = (
        "raw/lightnav.npy", "raw/lightnav_raw_text.txt", "derived/trajectory_world.npy",
        "input/latest_rgb.png", "input/frame_samples.csv", "input/capture_metadata.json",
        "metadata.json", "descriptors.json", "provenance.json",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"missing Stage 0-G case artifacts: {missing}")
    raw = validate_raw_lightnav_actions(np.load(root / "raw/lightnav.npy", allow_pickle=False), expected_horizon=expected_horizon)
    metadata = load_json(root / "metadata.json")
    capture = load_json(root / "input/capture_metadata.json")
    world = np.load(root / "derived/trajectory_world.npy", allow_pickle=False)
    expected_world = lightnav_local_to_world(raw, capture["robot_pose_at_observation"])
    if not np.array_equal(world, expected_world):
        raise ValueError("world trajectory is not anchored at the observation pose")
    provenance = load_json(root / "provenance.json")
    if provenance.get("raw_lightnav_sha256") != sha256_file(root / "raw/lightnav.npy"):
        raise ValueError("raw LightNav provenance hash mismatch")
    descriptors = load_json(root / "descriptors.json")
    if descriptors.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("descriptor must reject an intrinsic waypoint time base")
    if metadata.get("scenario_id") not in SCENARIO_IDS or metadata.get("variant_id") not in VARIANT_IDS:
        raise ValueError("invalid Stage 0-G case identity")
    return {"valid": True, "nonmoving": bool(descriptors["nonmoving"]), "raw_sha256": provenance["raw_lightnav_sha256"]}
