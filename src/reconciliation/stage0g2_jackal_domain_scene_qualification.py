"""Pure Stage 0-G2 contract, classification, diversity, and comparison helpers."""

from __future__ import annotations

from collections import Counter
import copy
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from reconciliation.stage0g_lightnav_scene_qualification import (
    classify_descriptors,
    duplicate_groups,
    geometry_descriptors,
    geometry_signature,
    load_json,
    pairwise_trajectory_distance,
    sha256_array,
    sha256_file,
    validate_config as validate_stage0g_config,
)
from reconciliation.lightnav_adapter import lightnav_local_to_world, validate_raw_lightnav_actions
from reconciliation.se2 import wrap_angle


SCENARIO_IDS = (
    "G2_Q0_STRAIGHT", "G2_Q1_LEFT_TURN", "G2_Q2_RIGHT_TURN",
    "G2_Q3_DOORWAY", "G2_Q4_DETOUR_LEFT", "G2_Q5_DETOUR_RIGHT",
)
VARIANT_IDS = ("V0", "V1", "V2", "V3", "V4")
FINAL_STATUSES = {
    "STAGE0G2_READY_FOR_DATA_COLLECTION", "STAGE0G2_SCENE_QUALIFICATION_FAILED",
    "STAGE0G2_NO_SUITABLE_DOMAIN_SCENE", "STAGE0G2_TECHNICAL_INVALID",
}


def _as_stage0g(config: Mapping[str, Any]) -> dict[str, Any]:
    value = copy.deepcopy(dict(config))
    value["stage"] = "stage0-g-lightnav-scene-qualification"
    value["scene"] = {"family": value.get("environment", {}).get("family", "domain_scene")}
    for index, item in enumerate(value["scenarios"]):
        item["id"] = ("Q0_STRAIGHT", "Q1_LEFT_TURN", "Q2_RIGHT_TURN", "Q3_DOORWAY", "Q4_DETOUR_LEFT", "Q5_DETOUR_RIGHT")[index]
        # The shared validator's synthetic-map uniqueness rule is not part of G2:
        # two instructions deliberately observe the same realistic obstacle.
        item["baseline_pose_se2"] = [float(index), float(index * 10), 0.0]
    return value


def validate_config(config: Mapping[str, Any]) -> None:
    if config.get("stage") != "stage0-g2-jackal-domain-scene-qualification":
        raise ValueError("invalid Stage 0-G2 stage name")
    if tuple(item.get("id") for item in config.get("scenarios", [])) != SCENARIO_IDS:
        raise ValueError("Stage 0-G2 requires the six ordered G2 scenario IDs")
    if config.get("robot", {}).get("required_embodiment") != "Jackal":
        raise ValueError("Stage 0-G2 requires Jackal and forbids alternate embodiments")
    if config.get("robot", {}).get("asset_relative_path") != "Clearpath/Jackal/jackal.usd":
        raise ValueError("Stage 0-G2 Jackal asset is frozen")
    baseline = config.get("baseline", {})
    if baseline.get("expected_lightnav_git_sha") != "a645828d81a8439651172197ca80a75dc1377977" or baseline.get("expected_checkpoint_revision") != "7221d418bfff55cfcbadd09f7a26aaab81e1f8a6":
        raise ValueError("Stage 0-G2 LightNav baseline is frozen")
    lightnav = config.get("lightnav", {})
    if (lightnav.get("task"), lightnav.get("task_key"), lightnav.get("task_type"), lightnav.get("prompt_style")) != ("vln", "vlnce", "vlnce_traj", "unified_traj"):
        raise ValueError("Stage 0-G2 LightNav task/prompt contract is frozen")
    if int(lightnav.get("expected_horizon", 0)) != 10 or lightnav.get("axes") != ["forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive"]:
        raise ValueError("Stage 0-G2 output shape/axis contract is frozen")
    camera = config.get("camera", {})
    fixed_camera = (
        camera.get("relative_translation_m") == [0.30, 0.0, 0.48]
        and camera.get("relative_rotation_xyz_deg") == [90.0, 0.0, -90.0]
        and int(camera.get("resolution_width", 0)) == 480
        and int(camera.get("resolution_height", 0)) == 270
        and float(camera.get("horizontal_fov_deg", 0.0)) == 112.0
    )
    if not fixed_camera: raise ValueError("Stage 0-G2 camera contract is frozen")
    environment = config.get("environment", {})
    if environment.get("family") != "isaac_hospital_6_0_1" or environment.get("asset_relative_path") != "/Isaac/Environments/Hospital/hospital.usd":
        raise ValueError("Stage 0-G2 selected Hospital asset is frozen")
    if not environment.get("selection_fixed_before_inference") or float(environment.get("scale", 0.0)) != 1.0:
        raise ValueError("environment selection and unit scale must be frozen")
    validate_stage0g_config(_as_stage0g(config))
    expected_variants = (("V0", 0.0, 0.0), ("V1", 0.05, 0.0), ("V2", -0.05, 0.0), ("V3", 0.0, 0.09), ("V4", 0.0, -0.09))
    actual_variants = tuple((item["id"], float(item["yaw_offset_rad"]), float(item["lateral_offset_m"])) for item in config["variants"])
    if actual_variants != expected_variants: raise ValueError("Stage 0-G2 nearby variants are frozen")
    for scenario in config["scenarios"]:
        pose = np.asarray(scenario.get("baseline_pose_se2"), dtype=np.float64)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)): raise ValueError("scenario pose must be finite SE(2)")
        width = float(scenario["route_width_m"])
        clearance = float(scenario["minimum_intended_clearance_m"])
        if not math.isfinite(width + clearance) or clearance <= 0.0:
            raise ValueError("route feasibility values must be finite and positive")


def variant_pose(config: Mapping[str, Any], scenario_id: str, variant_id: str) -> np.ndarray:
    validate_config(config)
    scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
    variant = next(item for item in config["variants"] if item["id"] == variant_id)
    base = np.asarray(scenario["baseline_pose_se2"], dtype=np.float64)
    yaw = float(base[2]); lateral = float(variant["lateral_offset_m"])
    return np.array([base[0] - math.sin(yaw) * lateral, base[1] + math.cos(yaw) * lateral,
                     wrap_angle(yaw + float(variant["yaw_offset_rad"]))], dtype=np.float64)


def dominant_exact_raw_fraction(named_arrays: Mapping[str, np.ndarray]) -> float:
    if not named_arrays:
        raise ValueError("at least one output is required")
    counts = Counter(sha256_array(value) for value in named_arrays.values())
    return max(counts.values()) / len(named_arrays)


def doorway_distinct_status(doorway_hashes: set[str], straight_hashes: set[str], unrelated_hashes: set[str]) -> str:
    if not doorway_hashes:
        raise ValueError("doorway outputs are required")
    return "DISTINCT_INTENT_NOT_DEMONSTRATED" if doorway_hashes.issubset(straight_hashes | unrelated_hashes) else "DISTINCT_INTENT_DEMONSTRATED"


def compare_stage0g(stage0g: Mapping[str, Any], stage0g2: Mapping[str, Any]) -> dict[str, Any]:
    fields = ("unique_raw_output_count", "dominant_exact_raw_fraction", "unique_geometry_signature_count")
    result = {"comparison_is_descriptive_only": True, "no_causal_statistical_claim": True}
    for field in fields:
        result[field] = {"stage0g": stage0g.get(field), "stage0g2": stage0g2.get(field)}
    return result


def validate_preview_gate(value: Mapping[str, Any]) -> None:
    required = ("free_space_visible", "intended_route_visible", "referenced_feature_visible", "natural_scene", "camera_usable", "initial_pose_collision_free")
    if any(value.get(key) is not True for key in required):
        raise ValueError("every Stage 0-G2 V0 preview gate item must pass")


def validate_asset_manifest(value: Mapping[str, Any]) -> None:
    candidates = value.get("candidates")
    if value.get("audit_before_primary_inference") is not True or not isinstance(candidates, list) or len(candidates) < 4:
        raise ValueError("Stage 0-G2 asset manifest is incomplete")
    required = {"family", "relative_path", "available", "loadable", "scale_m_per_stage_unit", "lighting", "prim_count", "collision_prim_count", "visual_quality", "jackal_fit", "decision"}
    if any(not required.issubset(item) for item in candidates):
        raise ValueError("Stage 0-G2 asset candidate schema is incomplete")
    selected = [item for item in candidates if item["decision"] == "SELECTED_BEFORE_INFERENCE"]
    if len(selected) != 1 or selected[0]["family"] != "Hospital" or selected[0]["loadable"] is not True:
        raise ValueError("manifest must select exactly one loadable Hospital before inference")


def qualification_status(conditions: Mapping[str, Any]) -> str:
    if not conditions or any(not isinstance(value, (bool, np.bool_)) for value in conditions.values()):
        raise ValueError("qualification conditions must be non-empty booleans")
    return "STAGE0G2_READY_FOR_DATA_COLLECTION" if all(bool(value) for value in conditions.values()) else "STAGE0G2_SCENE_QUALIFICATION_FAILED"


def validate_case(case_dir: str | Path, expected_horizon: int = 10) -> dict[str, Any]:
    root = Path(case_dir)
    required = ("raw/lightnav.npy", "raw/lightnav_raw_text.txt", "derived/trajectory_world.npy", "input/latest_rgb.png", "input/frame_samples.csv", "input/capture_metadata.json", "metadata.json", "descriptors.json", "provenance.json")
    missing = [name for name in required if not (root / name).is_file()]
    if missing: raise ValueError(f"missing Stage 0-G2 case artifacts: {missing}")
    raw = validate_raw_lightnav_actions(np.load(root / "raw/lightnav.npy", allow_pickle=False), expected_horizon=expected_horizon)
    capture = load_json(root / "input/capture_metadata.json")
    world = np.load(root / "derived/trajectory_world.npy", allow_pickle=False)
    if not np.array_equal(world, lightnav_local_to_world(raw, capture["robot_pose_at_observation"])):
        raise ValueError("world trajectory is not observation-pose anchored")
    provenance = load_json(root / "provenance.json")
    if provenance.get("raw_lightnav_sha256") != sha256_file(root / "raw/lightnav.npy"):
        raise ValueError("raw LightNav provenance hash mismatch")
    metadata = load_json(root / "metadata.json")
    if metadata.get("scenario_id") not in SCENARIO_IDS:
        raise ValueError("invalid Stage 0-G2 case identity")
    descriptors = load_json(root / "descriptors.json")
    if descriptors.get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("waypoint intrinsic timestamp claim is forbidden")
    return {"valid": True, "nonmoving": bool(descriptors["nonmoving"]), "raw_sha256": provenance["raw_lightnav_sha256"]}


__all__ = [
    "FINAL_STATUSES", "SCENARIO_IDS", "VARIANT_IDS", "classify_descriptors", "compare_stage0g",
    "dominant_exact_raw_fraction", "doorway_distinct_status", "duplicate_groups", "geometry_descriptors",
    "geometry_signature", "load_json", "pairwise_trajectory_distance", "qualification_status", "sha256_file", "validate_asset_manifest", "validate_case",
    "validate_config", "validate_preview_gate", "variant_pose",
]
