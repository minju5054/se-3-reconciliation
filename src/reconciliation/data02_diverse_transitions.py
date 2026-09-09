"""Pure DATA-02 validation, identity, diversity, plotting, and split helpers.

DATA-02 contains real same-episode successive LightNav outputs.  LightNav waypoint
rows are spatial poses and have no intrinsic timestamp; no function in this module
performs row-wise time alignment or reconciliation.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.exp02b_diagnosis import spatial_reference_metrics
from reconciliation.exp01b_extension import is_stop_actions
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.lightnav_adapter import lightnav_local_to_world
from reconciliation.lightnav_execution_envelope import geometry_descriptor
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import relative_pose, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


SCENARIO_IDS = (
    "D0_STRAIGHT",
    "D1_GENTLE_LEFT",
    "D2_GENTLE_RIGHT",
    "D3_STRONG_LEFT",
    "D4_STRONG_RIGHT",
    "D5_OBSTACLE_DETOUR_LEFT",
    "D6_OBSTACLE_DETOUR_RIGHT",
    "D7_BRANCH_OR_DOORWAY_LEFT",
    "D8_BRANCH_OR_DOORWAY_RIGHT",
    "D9_S_CURVE_OR_CHICANE",
)
VARIATION_IDS = (
    "V0_BASELINE",
    "V1_YAW_POS",
    "V2_YAW_NEG",
    "V3_LATERAL_POS",
    "V4_LATERAL_NEG",
)
ATTEMPT_CLASSIFICATIONS = (
    "VALID_MOVING",
    "MODEL_STOP_OUTPUT",
    "OLD_EXHAUSTED",
    "TIMING_INVALID",
    "INFERENCE_ERROR",
    "EXECUTION_INVALID",
    "OTHER_FAILURE",
)
GEOMETRY_LABELS = (
    "straight-like",
    "left-turn-like",
    "right-turn-like",
    "route-change/detour-like",
    "S/compound-curvature-like",
)
TIMING_KEYS = (
    "old_observation_sim_time_s",
    "old_request_host_monotonic_ns",
    "old_ready_host_monotonic_ns",
    "fresh_observation_sim_time_s",
    "fresh_observation_host_monotonic_ns",
    "fresh_request_host_monotonic_ns",
    "fresh_model_ready_sim_time_s",
    "fresh_model_ready_host_monotonic_ns",
    "fresh_usable_sim_time_s",
    "fresh_usable_host_monotonic_ns",
)


def _canonical_json(value: Any) -> bytes:
    assert_finite_json_tree(value)
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def _digest_json(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def write_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    destination = Path(path)
    assert_finite_json_tree(value)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def write_npy_exclusive(path: str | Path, value: ArrayLike) -> Path:
    destination = Path(path)
    array = np.asarray(value)
    if not np.issubdtype(array.dtype, np.number) or not np.all(np.isfinite(array)):
        raise ValueError("NPY output must contain only finite numeric values")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        np.save(stream, array, allow_pickle=False)
    return destination


def validate_data02_config(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Validate the frozen scenario, variation, timing, and scope contract."""

    if bool(config["lightnav"]["intrinsic_waypoint_time_base"]):
        raise ValueError("DATA-02 may not fabricate LightNav waypoint timestamps")
    protocol = config["protocol"]
    if protocol.get("natural_latency_only") is not True:
        raise ValueError("DATA-02 primary must use natural latency")
    if protocol.get("execute_fresh_after_collection") is not False:
        raise ValueError("DATA-02 collection must stop before FRESH execution")
    for key in (
        "fresh_observation_delay_s",
        "pace_real_time_factor",
        "maximum_fresh_wait_sim_s",
        "moving_path_length_min_m",
    ):
        value = float(protocol[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"protocol {key} must be finite and positive")
    variations = list(config["variations"])
    if tuple(item.get("id") for item in variations) != VARIATION_IDS:
        raise ValueError(f"variation order must be {VARIATION_IDS}")
    for item in variations:
        if not all(
            math.isfinite(float(item[key]))
            for key in ("yaw_offset_rad", "lateral_offset_m")
        ):
            raise ValueError("variation offsets must be finite")
    design = config["data02_collection"]
    if tuple(design["scenario_order"]) != SCENARIO_IDS:
        raise ValueError(f"scenario_order must be {SCENARIO_IDS}")
    scenarios = list(design["scenarios"])
    if tuple(item.get("id") for item in scenarios) != SCENARIO_IDS:
        raise ValueError("scenario definitions must match scenario_order exactly")
    valid_rules = {
        "straight",
        "gentle_left",
        "gentle_right",
        "strong_left",
        "strong_right",
        "route_left",
        "route_right",
        "s_curve",
    }
    output = []
    box_ids: set[str] = set()
    for scenario in scenarios:
        item = dict(scenario)
        item["initial_pose_se2"] = validate_pose_se2(
            scenario["initial_pose_se2"], name=f"{scenario['id']} initial pose"
        ).tolist()
        if not str(item.get("instruction", "")).strip():
            raise ValueError(f"{scenario['id']} must have an instruction")
        if item.get("qualification_rule") not in valid_rules:
            raise ValueError(f"{scenario['id']} has an invalid qualification rule")
        boxes = list(item.get("boxes", []))
        if not boxes:
            raise ValueError(f"{scenario['id']} must define static geometry")
        for box in boxes:
            key = f"{scenario['id']}__{box['id']}"
            if key in box_ids:
                raise ValueError(f"duplicate static box ID: {key}")
            box_ids.add(key)
            center = np.asarray(box["center"], dtype=np.float64)
            size = np.asarray(box["size"], dtype=np.float64)
            color = np.asarray(box["color"], dtype=np.float64)
            if center.shape != (3,) or size.shape != (3,) or color.shape != (3,):
                raise ValueError(f"{key} center/size/color must have three values")
            if not np.all(np.isfinite(center)) or not np.all(np.isfinite(size)):
                raise ValueError(f"{key} geometry must be finite")
            if np.any(size <= 0.0) or np.any(color < 0.0) or np.any(color > 1.0):
                raise ValueError(f"{key} has invalid size or color")
        output.append(item)
    assert_finite_json_tree(config)
    return output


def apply_variation(initial_pose: ArrayLike, variation: Mapping[str, Any]) -> np.ndarray:
    """Apply a frozen body-left lateral and yaw perturbation without randomness."""

    pose = validate_pose_se2(initial_pose, name="initial_pose")
    lateral = float(variation["lateral_offset_m"])
    yaw_offset = float(variation["yaw_offset_rad"])
    if not math.isfinite(lateral) or not math.isfinite(yaw_offset):
        raise ValueError("variation offsets must be finite")
    result = pose.copy()
    result[0] += -math.sin(float(pose[2])) * lateral
    result[1] += math.cos(float(pose[2])) * lateral
    result[2] = wrap_angle(result[2] + yaw_offset)
    return result


def variation_schedule(config: Mapping[str, Any], attempt_index: int) -> dict[str, Any]:
    if not isinstance(attempt_index, int) or attempt_index < 0:
        raise ValueError("attempt_index must be a non-negative integer")
    variations = list(config["variations"])
    return dict(variations[attempt_index % len(variations)])


def raw_pair_id(old_raw_sha256: str, fresh_raw_sha256: str) -> str:
    for digest in (old_raw_sha256, fresh_raw_sha256):
        if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise ValueError("raw pair inputs must be lowercase SHA-256 digests")
    return "rawpair_" + _digest_json(
        {"old_raw_sha256": old_raw_sha256, "fresh_raw_sha256": fresh_raw_sha256}
    )[:16]


def transition_context_id(context: Mapping[str, Any]) -> str:
    required = (
        "raw_pair_id",
        "scenario_id",
        "variant_id",
        "old_observation_pose_world",
        "fresh_observation_pose_world",
        "previous_control_pose_world",
        "boundary_pose_world",
        "timing",
    )
    missing = [key for key in required if key not in context]
    if missing:
        raise ValueError(f"transition context is missing: {missing}")
    for key in (
        "old_observation_pose_world",
        "fresh_observation_pose_world",
        "previous_control_pose_world",
        "boundary_pose_world",
    ):
        validate_pose_se2(context[key], name=key)
    validate_timing(context["timing"])
    return "ctx_" + _digest_json({key: context[key] for key in required})[:16]


def validate_timing(timing: Mapping[str, Any]) -> dict[str, bool]:
    missing = [key for key in TIMING_KEYS if key not in timing]
    if missing:
        raise ValueError(f"timing is missing: {missing}")
    for key in TIMING_KEYS:
        value = float(timing[key])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"timing {key} must be finite and non-negative")
    checks = {
        "old_request_after_observation": int(timing["old_request_host_monotonic_ns"])
        >= int(timing.get("old_observation_host_monotonic_ns", 0)),
        "old_ready_after_request": int(timing["old_ready_host_monotonic_ns"])
        >= int(timing["old_request_host_monotonic_ns"]),
        "fresh_request_after_observation": int(timing["fresh_request_host_monotonic_ns"])
        >= int(timing["fresh_observation_host_monotonic_ns"]),
        "fresh_ready_after_request": int(timing["fresh_model_ready_host_monotonic_ns"])
        >= int(timing["fresh_request_host_monotonic_ns"]),
        "fresh_usable_after_ready": int(timing["fresh_usable_host_monotonic_ns"])
        >= int(timing["fresh_model_ready_host_monotonic_ns"]),
        "fresh_observation_after_old_observation": float(
            timing["fresh_observation_sim_time_s"]
        )
        > float(timing["old_observation_sim_time_s"]),
        "fresh_usable_after_observation": float(timing["fresh_usable_sim_time_s"])
        >= float(timing["fresh_observation_sim_time_s"]),
        "fresh_usable_equals_model_ready": math.isclose(
            float(timing["fresh_usable_sim_time_s"]),
            float(timing["fresh_model_ready_sim_time_s"]),
            abs_tol=1e-12,
        ),
    }
    return checks


def previous_control_pose(
    control_samples: Sequence[Mapping[str, Any]], boundary_sim_time_s: float
) -> np.ndarray:
    """Return the last actual control pose strictly before B."""

    boundary = float(boundary_sim_time_s)
    if not math.isfinite(boundary):
        raise ValueError("boundary time must be finite")
    eligible = [
        row
        for row in control_samples
        if float(row["sim_time_s"]) < boundary - 1e-12
    ]
    if not eligible:
        raise ValueError("no actual control pose precedes B")
    return validate_pose_se2(eligible[-1]["actual_pose_world"], name="P").copy()


def anchor_successive_paths(
    old_raw: ArrayLike,
    fresh_raw: ArrayLike,
    old_observation_pose: ArrayLike,
    fresh_observation_pose: ArrayLike,
) -> tuple[np.ndarray, np.ndarray]:
    old = validate_se2_trajectory(old_raw, name="raw OLD")
    fresh = validate_se2_trajectory(fresh_raw, name="raw FRESH")
    return (
        lightnav_local_to_world(old, old_observation_pose),
        lightnav_local_to_world(fresh, fresh_observation_pose),
    )


def canonicalize_at_boundary(values: ArrayLike, boundary: ArrayLike) -> np.ndarray:
    poses = np.asarray(values, dtype=np.float64)
    if poses.ndim == 1:
        poses = validate_pose_se2(poses, name="pose")[None, :]
    else:
        poses = validate_se2_trajectory(poses, name="trajectory")
    result = relative_pose(validate_pose_se2(boundary, name="B"), poses)
    return np.asarray(result, dtype=np.float64)


def qualify_geometry(
    descriptor: Mapping[str, Any], rule: str, thresholds: Mapping[str, Any]
) -> dict[str, Any]:
    cumulative_yaw = float(descriptor["cumulative_abs_pose_yaw_change_rad"])
    lateral = float(descriptor["endpoint_lateral_departure_m"])
    tangent = float(descriptor["cumulative_abs_tangent_turn_rad"])
    signed_yaw = float(descriptor["net_pose_yaw_change_rad"])
    signed_tangent = float(descriptor["cumulative_signed_tangent_turn_rad"])
    direction = signed_yaw if abs(signed_yaw) >= abs(signed_tangent) else signed_tangent
    epsilon = float(thresholds["direction_sign_epsilon_rad"])
    checks: dict[str, bool]
    if rule == "straight":
        checks = {
            "small_pose_yaw": cumulative_yaw
            <= float(thresholds["straight_max_cumulative_abs_pose_yaw_rad"]),
            "small_lateral": abs(lateral)
            <= float(thresholds["straight_max_abs_lateral_departure_m"]),
        }
    elif rule.startswith("gentle_"):
        expected = 1.0 if rule.endswith("left") else -1.0
        checks = {
            "direction_sign": expected * direction >= epsilon,
            "turn_or_lateral": cumulative_yaw
            >= float(thresholds["gentle_min_cumulative_abs_pose_yaw_rad"])
            or abs(lateral)
            >= float(thresholds["gentle_min_abs_lateral_departure_m"]),
        }
    elif rule.startswith("strong_"):
        expected = 1.0 if rule.endswith("left") else -1.0
        checks = {
            "direction_sign": expected * direction >= epsilon,
            "strong_turn": cumulative_yaw
            >= float(thresholds["strong_min_cumulative_abs_pose_yaw_rad"])
            or tangent
            >= float(thresholds["strong_min_cumulative_abs_tangent_turn_rad"]),
        }
    elif rule.startswith("route_"):
        expected = 1.0 if rule.endswith("left") else -1.0
        route_signal = lateral if abs(lateral) >= 0.05 else direction
        checks = {
            "direction_sign": expected * route_signal >= 0.0,
            "route_departure": abs(lateral)
            >= float(thresholds["route_min_abs_lateral_departure_m"])
            or tangent
            >= float(thresholds["route_min_cumulative_abs_tangent_turn_rad"]),
        }
    elif rule == "s_curve":
        checks = {
            "curvature_sign_change": int(descriptor["curvature_sign_change_count"])
            >= int(thresholds["s_min_curvature_sign_changes"]),
            "compound_turn": tangent
            >= float(thresholds["s_min_cumulative_abs_tangent_turn_rad"]),
        }
    else:
        raise ValueError(f"unknown qualification rule: {rule}")
    return {"passed": all(checks.values()), "rule": rule, "checks": checks}


def descriptive_geometry_label(
    descriptor: Mapping[str, Any], thresholds: Mapping[str, Any]
) -> str:
    if (
        int(descriptor["curvature_sign_change_count"])
        >= int(thresholds["s_min_curvature_sign_changes"])
        and float(descriptor["cumulative_abs_tangent_turn_rad"])
        >= float(thresholds["s_min_cumulative_abs_tangent_turn_rad"])
    ):
        return "S/compound-curvature-like"
    lateral = float(descriptor["endpoint_lateral_departure_m"])
    tangent = float(descriptor["cumulative_abs_tangent_turn_rad"])
    if abs(lateral) >= float(thresholds["route_min_abs_lateral_departure_m"]):
        return "route-change/detour-like"
    direction = float(descriptor["net_pose_yaw_change_rad"])
    if abs(direction) < float(thresholds["direction_sign_epsilon_rad"]):
        direction = float(descriptor["cumulative_signed_tangent_turn_rad"])
    if (
        float(descriptor["cumulative_abs_pose_yaw_change_rad"])
        <= float(thresholds["straight_max_cumulative_abs_pose_yaw_rad"])
        and tangent < float(thresholds["route_min_cumulative_abs_tangent_turn_rad"])
    ):
        return "straight-like"
    return "left-turn-like" if direction >= 0.0 else "right-turn-like"


def transition_descriptor(
    previous_pose: ArrayLike,
    boundary: ArrayLike,
    fresh_world: ArrayLike,
    *,
    last_old_desired: ArrayLike,
    first_fresh_desired: ArrayLike,
    fresh_observation_pose: ArrayLike,
    natural_latency_s: float,
) -> dict[str, Any]:
    p = validate_pose_se2(previous_pose, name="P")
    b = validate_pose_se2(boundary, name="B")
    fresh = validate_se2_trajectory(fresh_world, name="FRESH world")
    old_command = np.asarray(last_old_desired, dtype=np.float64)
    new_command = np.asarray(first_fresh_desired, dtype=np.float64)
    if old_command.shape != (2,) or new_command.shape != (2,):
        raise ValueError("desired commands must have shape (2,)")
    if not np.all(np.isfinite(old_command)) or not np.all(np.isfinite(new_command)):
        raise ValueError("desired commands must be finite")
    incoming = b[:2] - p[:2]
    outgoing = fresh[0, :2] - b[:2]
    in_norm = float(np.linalg.norm(incoming))
    out_norm = float(np.linalg.norm(outgoing))
    incoming_direction = math.atan2(float(incoming[1]), float(incoming[0])) if in_norm > 1e-6 else None
    outgoing_direction = math.atan2(float(outgoing[1]), float(outgoing[0])) if out_norm > 1e-6 else None
    disagreement = (
        abs(float(wrap_angle(outgoing_direction - incoming_direction)))
        if incoming_direction is not None and outgoing_direction is not None
        else None
    )
    observation = validate_pose_se2(fresh_observation_pose, name="FRESH observation pose")
    latency = float(natural_latency_s)
    if not math.isfinite(latency) or latency < 0.0:
        raise ValueError("natural latency must be finite and non-negative")
    result = {
        "boundary_to_fresh0_distance_m": out_norm,
        "incoming_p_to_b_translation_m": in_norm,
        "incoming_direction_rad": incoming_direction,
        "boundary_to_fresh0_direction_rad": outgoing_direction,
        "direction_disagreement_abs_rad": disagreement,
        "yaw_disagreement_abs_rad": abs(
            float(wrap_angle((fresh[0, 2] - b[2]) - (b[2] - p[2])))
        ),
        "last_old_desired_v_mps": float(old_command[0]),
        "last_old_desired_omega_rps": float(old_command[1]),
        "first_raw_fresh_desired_v_mps": float(new_command[0]),
        "first_raw_fresh_desired_omega_rps": float(new_command[1]),
        "delta_v_mps": float(new_command[0] - old_command[0]),
        "delta_omega_rps": float(new_command[1] - old_command[1]),
        "abs_delta_v_mps": abs(float(new_command[0] - old_command[0])),
        "abs_delta_omega_rps": abs(float(new_command[1] - old_command[1])),
        "fresh_observation_to_boundary_translation_m": float(
            np.linalg.norm(b[:2] - observation[:2])
        ),
        "fresh_observation_to_boundary_yaw_rad": abs(
            float(wrap_angle(b[2] - observation[2]))
        ),
        "natural_inference_latency_s": latency,
        "short_window_desired_command_diagnostic": None,
        "short_window_undefined_reason": "FRESH is not executed in DATA-02 collection",
    }
    assert_finite_json_tree(result)
    return result


def classify_attempt(
    *,
    fresh_raw: ArrayLike | None,
    old_goal_reached: bool,
    timing_checks: Mapping[str, bool],
    execution_checks: Mapping[str, bool],
    inference_error: bool = False,
    other_error: bool = False,
    stop_tolerance: float = 1e-8,
) -> str:
    if inference_error:
        return "INFERENCE_ERROR"
    if other_error or fresh_raw is None:
        return "OTHER_FAILURE"
    if is_stop_actions(fresh_raw, absolute_tolerance=stop_tolerance):
        return "MODEL_STOP_OUTPUT"
    if old_goal_reached:
        return "OLD_EXHAUSTED"
    if not all(bool(value) for value in timing_checks.values()):
        return "TIMING_INVALID"
    if not all(bool(value) for value in execution_checks.values()):
        return "EXECUTION_INVALID"
    return "VALID_MOVING"


def execution_diagnostics(
    old_reference: ArrayLike,
    actual: ArrayLike,
    saturated: ArrayLike,
    *,
    desired_body: ArrayLike | None = None,
    executed_body: ArrayLike | None = None,
    measured_body: ArrayLike | None = None,
    target_wheels: ArrayLike | None = None,
    measured_wheels: ArrayLike | None = None,
) -> dict[str, Any]:
    metrics = spatial_reference_metrics(old_reference, actual)
    saturation = np.asarray(saturated, dtype=np.bool_)
    if saturation.ndim != 1 or saturation.shape[0] != np.asarray(actual).shape[0]:
        raise ValueError("saturation rows must match actual trajectory")
    result = {
        "spatial_reference": metrics,
        "active_saturation_fraction": float(np.mean(saturation)),
        "intrinsic_waypoint_time_base": False,
        "tracking_metric_semantics": "nearest OLD polyline; not time-aligned",
    }
    optional = (desired_body, executed_body, measured_body, target_wheels, measured_wheels)
    if any(value is not None for value in optional):
        if any(value is None for value in optional):
            raise ValueError("all command/motion/wheel telemetry arrays must be supplied together")
        count = np.asarray(actual).shape[0]
        desired = np.asarray(desired_body, dtype=np.float64)
        executed = np.asarray(executed_body, dtype=np.float64)
        measured = np.asarray(measured_body, dtype=np.float64)
        targets = np.asarray(target_wheels, dtype=np.float64)
        wheels = np.asarray(measured_wheels, dtype=np.float64)
        expected = ((count, 2), (count, 2), (count, 2), (count, 4), (count, 4))
        for name, value, shape in zip(
            ("desired_body", "executed_body", "measured_body", "target_wheels", "measured_wheels"),
            (desired, executed, measured, targets, wheels),
            expected,
            strict=True,
        ):
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be finite with shape {shape}")
        active = slice(1, None)
        desired_error = desired[active] - measured[active]
        executed_error = executed[active] - measured[active]
        wheel_error = targets[active] - wheels[active]
        rmse = lambda values: float(np.sqrt(np.mean(np.square(values))))
        labels = ("front_left", "front_right", "rear_left", "rear_right")
        result["body_command_vs_measured"] = {
            "desired_v_rmse_mps": rmse(desired_error[:, 0]),
            "desired_omega_rmse_rps": rmse(desired_error[:, 1]),
            "calibrated_executed_v_rmse_mps": rmse(executed_error[:, 0]),
            "calibrated_executed_omega_rmse_rps": rmse(executed_error[:, 1]),
        }
        result["wheel_target_vs_measured"] = {
            "aggregate_rmse_rad_s": rmse(wheel_error),
            "per_corner_rmse_rad_s": {
                label: rmse(wheel_error[:, index]) for index, label in enumerate(labels)
            },
        }
    return result


def scenario_qualification(
    records: Sequence[Mapping[str, Any]], scenario: Mapping[str, Any], config: Mapping[str, Any]
) -> dict[str, Any]:
    valid = [record for record in records if record["classification"] == "VALID_MOVING"]
    target = int(config["protocol"]["qualification_valid_outputs_per_family"])
    selected = valid[:target]
    checks = [
        qualify_geometry(record["fresh_geometry"], scenario["qualification_rule"], config["qualification_thresholds"])
        for record in selected
    ]
    geometry_passes = sum(item["passed"] for item in checks)
    required_geometry = target if scenario["qualification_rule"] == "straight" else 2
    return {
        "scenario_id": scenario["id"],
        "valid_output_count": len(valid),
        "target_valid_output_count": target,
        "geometry_pass_count": geometry_passes,
        "required_geometry_pass_count": required_geometry,
        "attempt_count": len(records),
        "per_output_checks": checks,
        "passed": len(valid) >= target and geometry_passes >= required_geometry,
    }


def dataset_acceptance(records: Sequence[Mapping[str, Any]], config: Mapping[str, Any]) -> dict[str, Any]:
    valid = [record for record in records if record["classification"] == "VALID_MOVING"]
    families = {str(record["scenario_id"]) for record in valid}
    raw_pairs = {str(record["raw_pair_id"]) for record in valid}
    counts = Counter(str(record["scenario_id"]) for record in valid)
    required_total = int(config["protocol"]["primary_minimum_valid_moving_total"])
    required_unique = int(config["protocol"]["primary_minimum_unique_raw_pairs"])
    checks = {
        "minimum_valid_contexts": len(valid) >= required_total,
        "all_scenario_families_represented": families == set(SCENARIO_IDS),
        "both_left_and_right_nonstraight_represented": bool(
            families.intersection({"D1_GENTLE_LEFT", "D3_STRONG_LEFT", "D5_OBSTACLE_DETOUR_LEFT", "D7_BRANCH_OR_DOORWAY_LEFT"})
            and families.intersection({"D2_GENTLE_RIGHT", "D4_STRONG_RIGHT", "D6_OBSTACLE_DETOUR_RIGHT", "D8_BRANCH_OR_DOORWAY_RIGHT"})
        ),
        "minimum_unique_raw_pairs": len(raw_pairs) >= required_unique,
    }
    ready = all(checks.values())
    return {
        "status": "DATA02_READY_FOR_FORMULATION_RESEARCH" if ready else "DATA02_DIVERSITY_TARGET_NOT_MET",
        "checks": checks,
        "valid_moving_contexts": len(valid),
        "unique_raw_pairs": len(raw_pairs),
        "scenario_counts": {scenario: counts.get(scenario, 0) for scenario in SCENARIO_IDS},
    }


def grouped_split(
    records: Sequence[Mapping[str, Any]], *, heldout_fraction: float, seed: str
) -> dict[str, Any]:
    """Deterministically split complete raw-pair groups; no result metric is used."""

    fraction = float(heldout_fraction)
    if not 0.0 < fraction < 1.0:
        raise ValueError("heldout_fraction must lie in (0, 1)")
    groups: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for record in records:
        groups[str(record["raw_pair_id"])].append(record)
    ordered = sorted(
        groups,
        key=lambda group: hashlib.sha256(f"{seed}:{group}".encode()).hexdigest(),
    )
    target = max(1, min(len(records) - 1, int(round(len(records) * fraction))))
    heldout_groups: set[str] = set()
    heldout_count = 0
    # Predeclare coverage-oriented strata. This uses only scenario/geometry metadata,
    # never a transition magnitude or reconciliation result.
    strata: list[tuple[str, str]] = []
    for scenario in SCENARIO_IDS:
        strata.append(("scenario_id", scenario))
    for label in GEOMETRY_LABELS:
        strata.append(("geometry_label", label))
    for key, value in strata:
        candidates = [
            group
            for group in ordered
            if any(str(record.get(key, "")) == value for record in groups[group])
        ]
        if not candidates:
            continue
        # Keep at least one group for development whenever the stratum has two groups.
        selectable = [group for group in candidates if group not in heldout_groups]
        if len(candidates) > 1 and selectable:
            chosen = selectable[0]
            heldout_groups.add(chosen)
            heldout_count += len(groups[chosen])
    for group in ordered:
        if group in heldout_groups:
            continue
        size = len(groups[group])
        if abs((heldout_count + size) - target) <= abs(heldout_count - target):
            heldout_groups.add(group)
            heldout_count += size
    if not heldout_groups and ordered:
        heldout_groups.add(ordered[0])
    if heldout_groups == set(ordered) and len(ordered) > 1:
        heldout_groups.remove(ordered[-1])
    development = sorted(
        str(record["transition_context_id"])
        for record in records
        if str(record["raw_pair_id"]) not in heldout_groups
    )
    heldout = sorted(
        str(record["transition_context_id"])
        for record in records
        if str(record["raw_pair_id"]) in heldout_groups
    )
    development_groups = set(ordered) - heldout_groups
    return {
        "algorithm": (
            "SHA256-seeded whole-raw-pair groups with scenario/geometry coverage prepass; "
            "no transition magnitude or reconciliation result"
        ),
        "seed": seed,
        "target_heldout_fraction": fraction,
        "development_transition_context_ids": development,
        "heldout_transition_context_ids": heldout,
        "development_raw_pair_ids": sorted(development_groups),
        "heldout_raw_pair_ids": sorted(heldout_groups),
        "raw_pair_leakage_count": len(development_groups.intersection(heldout_groups)),
    }


def distribution(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "min": None, "median": None, "max": None}
    if not np.all(np.isfinite(array)):
        raise ValueError("distribution values must be finite")
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "max": float(np.max(array)),
    }


def load_attempt_records(run_directory: str | Path) -> list[dict[str, Any]]:
    root = Path(run_directory)
    paths = sorted((root / "primary/attempts").glob("*/attempt.json"))
    if not paths:
        paths = sorted((root / "qualification/attempts").glob("*/attempt.json"))
    return [load_strict_json(path) for path in paths]


def validate_attempt_directory(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    attempt = load_strict_json(root / "attempt.json")
    if attempt["classification"] not in ATTEMPT_CLASSIFICATIONS:
        raise ValueError("invalid attempt classification")
    required_common = ("attempt.json", "provenance.json")
    for name in required_common:
        if not (root / name).is_file():
            raise ValueError(f"attempt missing {name}")
    has_context = (root / "context.json").is_file()
    if attempt["classification"] == "MODEL_STOP_OUTPUT" and not has_context:
        if not (root / "raw/old_lightnav.npy").is_file():
            raise ValueError("OLD STOP attempt must preserve raw OLD output")
        return attempt
    if attempt["classification"] in {"VALID_MOVING", "MODEL_STOP_OUTPUT", "OLD_EXHAUSTED", "TIMING_INVALID", "EXECUTION_INVALID"}:
        required = (
            "raw/old_lightnav.npy",
            "raw/fresh_lightnav.npy",
            "derived/old_world.npy",
            "derived/fresh_world.npy",
            "actual/old_execution.npy",
            "telemetry/old_execution.csv",
            "context.json",
            "descriptors.json",
        )
        missing = [name for name in required if not (root / name).is_file()]
        if missing:
            raise ValueError(f"attempt missing source artifacts: {missing}")
        old_raw = np.load(root / "raw/old_lightnav.npy", allow_pickle=False)
        fresh_raw = np.load(root / "raw/fresh_lightnav.npy", allow_pickle=False)
        old_world = np.load(root / "derived/old_world.npy", allow_pickle=False)
        fresh_world = np.load(root / "derived/fresh_world.npy", allow_pickle=False)
        context = load_strict_json(root / "context.json")
        validate_se2_trajectory(old_raw, name="raw OLD")
        validate_se2_trajectory(fresh_raw, name="raw FRESH")
        expected_old, expected_fresh = anchor_successive_paths(
            old_raw,
            fresh_raw,
            context["old_observation_pose_world"],
            context["fresh_observation_pose_world"],
        )
        if not np.array_equal(old_world, expected_old) or not np.array_equal(fresh_world, expected_fresh):
            raise ValueError("world trajectory anchoring does not reconstruct")
        if np.array_equal(
            fresh_world,
            lightnav_local_to_world(fresh_raw, context["boundary_pose_world"]),
        ) and not np.array_equal(
            context["fresh_observation_pose_world"], context["boundary_pose_world"]
        ):
            raise ValueError("FRESH appears to be incorrectly anchored at B")
        timing_checks = validate_timing(context["timing"])
        if not all(timing_checks.values()) and attempt["classification"] == "VALID_MOVING":
            raise ValueError("VALID_MOVING attempt has invalid timing")
        raw_id = raw_pair_id(
            sha256_file(root / "raw/old_lightnav.npy"),
            sha256_file(root / "raw/fresh_lightnav.npy"),
        )
        if raw_id != attempt["raw_pair_id"]:
            raise ValueError("raw pair ID does not reconstruct")
        identity_context = dict(context)
        identity_context["raw_pair_id"] = raw_id
        if transition_context_id(identity_context) != attempt["transition_context_id"]:
            raise ValueError("transition context ID does not reconstruct")
    return attempt


def strict_validate_dataset(run_directory: str | Path, *, require_plots: bool = True) -> dict[str, Any]:
    root = Path(run_directory)
    metadata = load_strict_json(root / "metadata.json")
    config_snapshot = root / "config_snapshot.yaml"
    if not config_snapshot.is_file():
        raise ValueError("missing config snapshot")
    attempt_paths = sorted((root / "primary/attempts").glob("*"))
    attempt_paths = [path for path in attempt_paths if path.is_dir()]
    records = [validate_attempt_directory(path) for path in attempt_paths]
    valid = [record for record in records if record["classification"] == "VALID_MOVING"]
    if metadata.get("phase") == "primary":
        manifest = load_strict_json(root / "bank/dataset_manifest.json")
        split = load_strict_json(root / "bank/split_manifest.json")
        if manifest["valid_transition_context_count"] != len(valid):
            raise ValueError("manifest valid count differs from attempts")
        development = set(split["development_transition_context_ids"])
        heldout = set(split["heldout_transition_context_ids"])
        ids = {record["transition_context_id"] for record in valid}
        if development & heldout or development | heldout != ids:
            raise ValueError("split union/disjointness failed")
        if set(split["development_raw_pair_ids"]) & set(split["heldout_raw_pair_ids"]):
            raise ValueError("raw pair leakage detected")
        for record in valid:
            pair = root / "bank/pairs" / record["transition_context_id"]
            if not pair.is_dir():
                raise ValueError("bank pair directory missing")
            if require_plots:
                for name in ("world_xy.png", "boundary_canonical_xy.png", "yaw_vs_path_length.png"):
                    if not (pair / "plots" / name).is_file():
                        raise ValueError(f"pair plot missing: {name}")
        if require_plots:
            for name in (
                "all_pairs_boundary_canonical.png",
                "by_scenario_family.png",
                "scenario_family_representatives.png",
                "by_geometry_class.png",
                "raw_delta_distribution.png",
                "geometry_diversity.png",
            ):
                if not (root / "overview" / name).is_file():
                    raise ValueError(f"overview plot missing: {name}")
    return {
        "valid": True,
        "run_id": metadata["run_id"],
        "phase": metadata["phase"],
        "attempt_count": len(records),
        "valid_moving_count": len(valid),
    }


def write_index_csv(path: Path, records: Sequence[Mapping[str, Any]]) -> None:
    columns = (
        "transition_context_id",
        "raw_pair_id",
        "scenario_id",
        "variant_id",
        "classification",
        "geometry_label",
        "attempt_relative_path",
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record[key] for key in columns})
