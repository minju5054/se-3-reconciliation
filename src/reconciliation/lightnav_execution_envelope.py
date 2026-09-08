"""Pure Stage 0-F LightNav geometry and execution-envelope helpers.

LightNav waypoint rows have no intrinsic time base.  Every descriptor here is
spatial, and pose-yaw progression is kept separate from XY tangent curvature.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.closed_loop_execution_validation import (
    CONTROLLED_IDS,
    ClosedLoopTelemetry,
    compute_closed_loop_metrics,
    generate_controlled_references,
)
from reconciliation.exp01b_controlled_latency import (
    load_attempt_records,
    validate_controlled_output,
)
from reconciliation.exp01b_extension import is_stop_actions
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.lightnav_adapter import lightnav_local_to_world, raw_actions_to_local_path
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import compose_poses, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
ENVELOPE_LABELS = (
    "PASS_FIXTURE_LIKE",
    "STRONG_TURN_FAIL_FIXTURE_LIKE",
    "OUTSIDE_STAGE0E_TESTED_GEOMETRY",
    "AMBIGUOUS",
)
FINAL_LABELS = (
    "LIGHTNAV_ENVELOPE_COVERED_BY_CURRENT_CALIBRATED_PLATFORM",
    "LIGHTNAV_ENVELOPE_PARTIALLY_COVERED",
    "LIGHTNAV_ENVELOPE_REACHES_UNVALIDATED_STRONG_TURN_REGION",
    "LIGHTNAV_ENVELOPE_INSUFFICIENT_DATA",
)
SUMMARY_DESCRIPTORS = (
    "number_of_poses",
    "path_length_m",
    "endpoint_displacement_m",
    "net_pose_yaw_change_rad",
    "cumulative_abs_pose_yaw_change_rad",
    "max_abs_pose_yaw_increment_rad",
    "pose_yaw_per_meter_median_abs_rad_per_m",
    "pose_yaw_per_meter_p90_abs_rad_per_m",
    "pose_yaw_per_meter_p95_abs_rad_per_m",
    "pose_yaw_per_meter_max_abs_rad_per_m",
    "tangent_curvature_median_abs_per_m",
    "tangent_curvature_p90_abs_per_m",
    "tangent_curvature_p95_abs_per_m",
    "tangent_curvature_max_abs_per_m",
    "minimum_finite_local_turning_radius_m",
    "endpoint_lateral_departure_m",
    "max_abs_lateral_departure_m",
    "cumulative_signed_tangent_turn_rad",
    "cumulative_abs_tangent_turn_rad",
    "curvature_sign_change_count",
    "near_zero_segment_fraction",
)


def _percentile(values: FloatArray, percentile: float) -> float | None:
    return (
        float(np.percentile(values, percentile, method="linear"))
        if values.size
        else None
    )


def _abs_statistics(values: FloatArray, prefix: str) -> dict[str, float | None]:
    absolute = np.abs(values)
    return {
        f"{prefix}_median_abs": _percentile(absolute, 50),
        f"{prefix}_p90_abs": _percentile(absolute, 90),
        f"{prefix}_p95_abs": _percentile(absolute, 95),
        f"{prefix}_max_abs": float(np.max(absolute)) if absolute.size else None,
    }


def geometry_descriptor(
    trajectory: ArrayLike,
    *,
    segment_epsilon_m: float = 1e-6,
    curvature_zero_epsilon_per_m: float = 1e-9,
) -> dict[str, Any]:
    """Describe an untimed SE(2) path without fabricating velocities."""

    poses = validate_se2_trajectory(trajectory, name="geometry trajectory")
    epsilon = float(segment_epsilon_m)
    curvature_epsilon = float(curvature_zero_epsilon_per_m)
    if not math.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError("segment_epsilon_m must be finite and positive")
    if not math.isfinite(curvature_epsilon) or curvature_epsilon <= 0.0:
        raise ValueError("curvature_zero_epsilon_per_m must be finite and positive")

    delta_xy = np.diff(poses[:, :2], axis=0)
    segment_lengths = np.linalg.norm(delta_xy, axis=1)
    valid_segments = segment_lengths >= epsilon
    pose_yaw_steps = np.asarray(wrap_angle(np.diff(poses[:, 2])), dtype=np.float64)
    pose_curvature = pose_yaw_steps[valid_segments] / segment_lengths[valid_segments]

    tangents = np.full(segment_lengths.shape, np.nan, dtype=np.float64)
    tangents[valid_segments] = np.arctan2(
        delta_xy[valid_segments, 1], delta_xy[valid_segments, 0]
    )
    valid_pairs = valid_segments[:-1] & valid_segments[1:]
    tangent_turn = np.asarray(
        wrap_angle(tangents[1:][valid_pairs] - tangents[:-1][valid_pairs]),
        dtype=np.float64,
    )
    pair_scale = (segment_lengths[:-1][valid_pairs] + segment_lengths[1:][valid_pairs]) / 2.0
    tangent_curvature = tangent_turn / pair_scale
    nonzero_signs = np.sign(
        tangent_curvature[np.abs(tangent_curvature) > curvature_epsilon]
    )
    sign_changes = (
        int(np.sum(nonzero_signs[1:] != nonzero_signs[:-1]))
        if nonzero_signs.size > 1
        else 0
    )
    initial_yaw = float(poses[0, 2])
    displacement = poses[:, :2] - poses[0, :2]
    lateral = -math.sin(initial_yaw) * displacement[:, 0] + math.cos(
        initial_yaw
    ) * displacement[:, 1]
    max_tangent_curvature = (
        float(np.max(np.abs(tangent_curvature))) if tangent_curvature.size else 0.0
    )
    minimum_radius = (
        1.0 / max_tangent_curvature
        if max_tangent_curvature > curvature_epsilon
        else None
    )
    pose_stats = _abs_statistics(pose_curvature, "pose_yaw_per_meter")
    tangent_stats = _abs_statistics(tangent_curvature, "tangent_curvature")
    result = {
        "number_of_poses": int(poses.shape[0]),
        "path_length_m": float(np.sum(segment_lengths)),
        "endpoint_displacement_m": float(np.linalg.norm(displacement[-1])),
        "net_pose_yaw_change_rad": float(wrap_angle(poses[-1, 2] - poses[0, 2])),
        "abs_net_pose_yaw_change_rad": abs(
            float(wrap_angle(poses[-1, 2] - poses[0, 2]))
        ),
        "cumulative_abs_pose_yaw_change_rad": float(np.sum(np.abs(pose_yaw_steps))),
        "max_abs_pose_yaw_increment_rad": (
            float(np.max(np.abs(pose_yaw_steps))) if pose_yaw_steps.size else 0.0
        ),
        "pose_yaw_per_meter_finite_count": int(pose_curvature.size),
        "pose_yaw_per_meter_median_abs_rad_per_m": pose_stats[
            "pose_yaw_per_meter_median_abs"
        ],
        "pose_yaw_per_meter_p90_abs_rad_per_m": pose_stats[
            "pose_yaw_per_meter_p90_abs"
        ],
        "pose_yaw_per_meter_p95_abs_rad_per_m": pose_stats[
            "pose_yaw_per_meter_p95_abs"
        ],
        "pose_yaw_per_meter_max_abs_rad_per_m": pose_stats[
            "pose_yaw_per_meter_max_abs"
        ],
        "path_tangent_defined_segment_count": int(np.sum(valid_segments)),
        "tangent_curvature_finite_count": int(tangent_curvature.size),
        "tangent_curvature_median_abs_per_m": tangent_stats[
            "tangent_curvature_median_abs"
        ],
        "tangent_curvature_p90_abs_per_m": tangent_stats[
            "tangent_curvature_p90_abs"
        ],
        "tangent_curvature_p95_abs_per_m": tangent_stats[
            "tangent_curvature_p95_abs"
        ],
        "tangent_curvature_max_abs_per_m": tangent_stats[
            "tangent_curvature_max_abs"
        ],
        "minimum_finite_local_turning_radius_m": minimum_radius,
        "endpoint_lateral_departure_m": float(lateral[-1]),
        "max_abs_lateral_departure_m": float(np.max(np.abs(lateral))),
        "cumulative_signed_tangent_turn_rad": float(np.sum(tangent_turn)),
        "cumulative_abs_tangent_turn_rad": float(np.sum(np.abs(tangent_turn))),
        "curvature_sign_change_count": sign_changes,
        "near_zero_segment_count": int(np.sum(~valid_segments)),
        "near_zero_segment_fraction": float(np.mean(~valid_segments)),
        "segment_epsilon_m": epsilon,
        "curvature_zero_epsilon_per_m": curvature_epsilon,
        "intrinsic_waypoint_time_base": False,
        "pose_yaw_and_xy_tangent_are_distinct": True,
        "minimum_radius_convention": (
            "null when no finite tangent curvature exceeds curvature_zero_epsilon_per_m"
        ),
    }
    assert_finite_json_tree(result)
    return result


def rigid_transform_trajectory(trajectory: ArrayLike, transform: ArrayLike) -> FloatArray:
    """Return a transformed copy; source arrays are never mutated."""

    poses = validate_se2_trajectory(trajectory, name="trajectory")
    rigid = validate_pose_se2(transform, name="transform")
    return np.asarray(compose_poses(rigid, poses), dtype=np.float64)


def trajectory_content_sha256(trajectory: ArrayLike) -> str:
    poses = np.ascontiguousarray(
        validate_se2_trajectory(trajectory, name="trajectory"), dtype="<f8"
    )
    digest = hashlib.sha256()
    digest.update(str(poses.shape).encode("ascii"))
    digest.update(poses.tobytes(order="C"))
    return digest.hexdigest()


def group_records_by_hash(
    records: Sequence[Mapping[str, Any]], key: str
) -> dict[str, list[dict[str, Any]]]:
    """Group provenance rows without treating duplicates as independent geometry."""

    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        digest = str(record[key])
        if len(digest) != 64 or any(value not in "0123456789abcdef" for value in digest):
            raise ValueError(f"{key} must contain lowercase SHA-256 values")
        groups[digest].append(dict(record))
    return dict(sorted(groups.items()))


def classify_fresh_output(actions: ArrayLike, *, absolute_tolerance: float = 1e-8) -> str:
    return (
        "STOP_OUTPUT"
        if is_stop_actions(actions, absolute_tolerance=absolute_tolerance)
        else "MOVING_OUTPUT"
    )


def extract_suffixes(trajectory: ArrayLike, *, minimum_poses: int = 2) -> list[tuple[int, FloatArray]]:
    poses = validate_se2_trajectory(trajectory, name="FRESH trajectory")
    if not isinstance(minimum_poses, int) or minimum_poses < 2:
        raise ValueError("minimum_poses must be an integer >= 2")
    return [
        (index, poses[index:].copy())
        for index in range(max(0, poses.shape[0] - minimum_poses + 1))
    ]


def _verify_hash(path: Path, expected: str, label: str) -> None:
    if sha256_file(path) != expected:
        raise ValueError(f"frozen provenance mismatch: {label}")


def verify_named_hashes(
    root: str | Path, expected: Mapping[str, str]
) -> dict[str, str]:
    directory = Path(root)
    verified = {}
    for relative, digest in expected.items():
        path = directory / relative
        _verify_hash(path, str(digest), relative)
        verified[str(relative)] = str(digest)
    return verified


def write_json_exclusive(path: str | Path, value: Mapping[str, Any]) -> Path:
    destination = Path(path)
    assert_finite_json_tree(value)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return destination


def validate_source_provenance(root: str | Path, provenance: Mapping[str, Any]) -> dict[str, Any]:
    source = Path(root).resolve()
    expected = {
        filename: str(provenance[key])
        for key, filename in (
            ("metadata_sha256", "metadata.json"),
            ("protocol_sha256", "protocol.json"),
            ("validation_sha256", "validation.json"),
            ("collection_summary_sha256", "collection_summary.json"),
            ("summary_sha256", "summary.json"),
        )
    }
    verify_named_hashes(source, expected)
    metadata = load_strict_json(source / "metadata.json")
    if metadata.get("experiment_id") != provenance["experiment_id"]:
        raise ValueError("frozen source experiment id changed")
    validation = validate_controlled_output(source)
    if validation["attempt_count"] != int(provenance["expected_attempt_count"]):
        raise ValueError("frozen source attempt count changed")
    if validation["valid_moving_count"] != int(
        provenance["expected_valid_moving_count"]
    ):
        raise ValueError("frozen source moving count changed")
    return validation


def _artifact_hashes(trial: Path) -> dict[str, str]:
    required = (
        "raw/old_actions.npy",
        "raw/fresh_actions.npy",
        "derived/old_world.npy",
        "derived/fresh_world.npy",
        "metadata.json",
        "results/attempt.json",
        "derived/controller_commands.csv",
    )
    missing = [name for name in required if not (trial / name).is_file()]
    if missing:
        raise ValueError(f"source trial is missing required artifacts: {missing}")
    return {name: sha256_file(trial / name) for name in required}


def build_source_inventory(
    root: str | Path,
    provenance: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate all frozen attempts and deduplicate eligible moving world arrays."""

    source = Path(root).resolve()
    validation = validate_source_provenance(source, provenance)
    source_metadata = load_strict_json(source / "metadata.json")
    tolerance = float(provenance["stop_action_absolute_tolerance"])
    attempts: list[dict[str, Any]] = []
    unique: dict[str, dict[str, dict[str, Any]]] = {"OLD": {}, "FRESH": {}}
    for attempt in load_attempt_records(source):
        trial = Path(attempt["artifact_path"]).resolve()
        metadata = load_strict_json(trial / "metadata.json")
        hashes = _artifact_hashes(trial)
        old_raw = np.load(trial / "raw/old_actions.npy", allow_pickle=False)
        fresh_raw = np.load(trial / "raw/fresh_actions.npy", allow_pickle=False)
        old_world = validate_se2_trajectory(
            np.load(trial / "derived/old_world.npy", allow_pickle=False),
            name="source OLD world",
        )
        fresh_world = validate_se2_trajectory(
            np.load(trial / "derived/fresh_world.npy", allow_pickle=False),
            name="source FRESH world",
        )
        expected_old = lightnav_local_to_world(
            raw_actions_to_local_path(old_raw), metadata["robot_pose_at_old_observation"]
        )
        expected_fresh = lightnav_local_to_world(
            raw_actions_to_local_path(fresh_raw), metadata["robot_pose_at_fresh_observation"]
        )
        if not np.array_equal(old_world, expected_old):
            raise ValueError(f"OLD world reconstruction mismatch: {trial}")
        if not np.array_equal(fresh_world, expected_fresh):
            raise ValueError(f"FRESH world reconstruction mismatch: {trial}")
        stop = is_stop_actions(fresh_raw, absolute_tolerance=tolerance)
        if stop != bool(attempt["stop_output"]):
            raise ValueError(f"STOP classification mismatch: {trial}")
        relative = str(trial.relative_to(source))
        eligible = bool(
            attempt["classification"] == "VALID_MOVING"
            and attempt["timing_valid"]
            and not stop
        )
        attempts.append(
            {
                "relative_trial_path": relative,
                "classification": attempt["classification"],
                "timing_valid": bool(attempt["timing_valid"]),
                "stop_category": (
                    "STOP_OUTPUT"
                    if attempt["classification"] == "MODEL_STOP_OUTPUT"
                    else None
                ),
                "fresh_all_zero_output": stop,
                "eligible_moving": eligible,
                "geometry_id": metadata["condition_id"],
                "geometry_class": metadata["geometry_class"],
                "latency_id": attempt["latency_condition_id"],
                "trial_index": int(metadata["trial_index"]),
                "artifacts_sha256": hashes,
            }
        )
        if not eligible:
            continue
        for kind, world, world_name, raw_name, start_name in (
            (
                "OLD",
                old_world,
                "derived/old_world.npy",
                "raw/old_actions.npy",
                "robot_pose_at_old_observation",
            ),
            (
                "FRESH",
                fresh_world,
                "derived/fresh_world.npy",
                "raw/fresh_actions.npy",
                "robot_pose_at_fresh_observation",
            ),
        ):
            world_hash = hashes[world_name]
            reference_id = f"{kind.lower()}_{world_hash[:16]}"
            source_row = {
                "relative_trial_path": relative,
                "raw_action_sha256": hashes[raw_name],
                "world_trajectory_sha256": world_hash,
                "metadata_sha256": hashes["metadata.json"],
                "attempt_sha256": hashes["results/attempt.json"],
                "controller_commands_sha256": hashes[
                    "derived/controller_commands.csv"
                ],
                "geometry_id": metadata["condition_id"],
                "geometry_class": metadata["geometry_class"],
                "latency_id": attempt["latency_condition_id"],
                "start_pose_se2": validate_pose_se2(metadata[start_name]).tolist(),
            }
            row = unique[kind].setdefault(
                world_hash,
                {
                    "reference_id": reference_id,
                    "kind": kind,
                    "world_trajectory_sha256": world_hash,
                    "array_content_sha256": trajectory_content_sha256(world),
                    "number_of_poses": int(world.shape[0]),
                    "source_trials": [],
                },
            )
            if row["array_content_sha256"] != trajectory_content_sha256(world):
                raise ValueError("identical NPY hash produced different trajectory content")
            row["source_trials"].append(source_row)

    classes = Counter(row["classification"] for row in attempts)
    for kind in ("OLD", "FRESH"):
        for row in unique[kind].values():
            row["source_trials"].sort(key=lambda value: value["relative_trial_path"])
            row["representative_source_trial"] = row["source_trials"][0][
                "relative_trial_path"
            ]
            starts = {tuple(item["start_pose_se2"]) for item in row["source_trials"]}
            if len(starts) != 1:
                raise ValueError("deduplicated world trajectory has inconsistent start poses")
            row["execution_start_pose_se2"] = list(next(iter(starts)))
    raw_counts = {
        kind.lower(): len(
            {
                item["raw_action_sha256"]
                for row in unique[kind].values()
                for item in row["source_trials"]
            }
        )
        for kind in ("OLD", "FRESH")
    }
    unique_rows = {
        kind.lower(): sorted(unique[kind].values(), key=lambda row: row["reference_id"])
        for kind in ("OLD", "FRESH")
    }
    result = {
        "source_root": str(source),
        "source_provenance": {
            "experiment_id": provenance["experiment_id"],
            "metadata_sha256": provenance["metadata_sha256"],
            "protocol_sha256": provenance["protocol_sha256"],
            "validation_sha256": provenance["validation_sha256"],
            "collection_summary_sha256": provenance[
                "collection_summary_sha256"
            ],
            "summary_sha256": provenance["summary_sha256"],
            "source_config_sha256": provenance["source_config_sha256"],
            "research_git_commit_sha_at_run": source_metadata[
                "research_git_commit_sha_at_run"
            ],
            "lightnav_sha": source_metadata["lightnav_server_end"]["lightnav_sha"],
            "lightnav_checkpoint_revision": source_metadata["lightnav_server_end"][
                "checkpoint_revision"
            ],
        },
        "strict_source_validation": validation,
        "attempt_count": len(attempts),
        "classification_counts": dict(sorted(classes.items())),
        "timing_valid_count": sum(row["timing_valid"] for row in attempts),
        "eligible_moving_trial_count": sum(row["eligible_moving"] for row in attempts),
        "stop_output_count": sum(
            row["classification"] == "MODEL_STOP_OUTPUT" for row in attempts
        ),
        "fresh_all_zero_attempt_count": sum(
            row["fresh_all_zero_output"] for row in attempts
        ),
        "fresh_all_zero_nonstop_failure_count": sum(
            row["fresh_all_zero_output"]
            and row["classification"] != "MODEL_STOP_OUTPUT"
            for row in attempts
        ),
        "old_source_trial_count": sum(row["eligible_moving"] for row in attempts),
        "fresh_source_trial_count": sum(row["eligible_moving"] for row in attempts),
        "unique_old_raw_action_count": raw_counts["old"],
        "unique_fresh_raw_action_count": raw_counts["fresh"],
        "unique_old_world_reference_count": len(unique_rows["old"]),
        "unique_fresh_world_reference_count": len(unique_rows["fresh"]),
        "unique_execution_reference_count": len(unique_rows["old"])
        + len(unique_rows["fresh"]),
        "attempts": sorted(attempts, key=lambda row: row["relative_trial_path"]),
        "unique_references": unique_rows,
        "intrinsic_waypoint_time_base": False,
        "stop_outputs_excluded_from_moving_geometry": True,
    }
    assert_finite_json_tree(result)
    return result


def load_inventory_reference(
    inventory: Mapping[str, Any], entry: Mapping[str, Any]
) -> FloatArray:
    source = Path(inventory["source_root"])
    trial = source / entry["representative_source_trial"]
    filename = "old_world.npy" if entry["kind"] == "OLD" else "fresh_world.npy"
    path = trial / "derived" / filename
    if sha256_file(path) != entry["world_trajectory_sha256"]:
        raise ValueError("inventory reference provenance changed")
    return validate_se2_trajectory(np.load(path, allow_pickle=False), name="inventory reference")


def build_fixture_geometry(
    stage0e_config: Mapping[str, Any], stage0e_summary: Mapping[str, Any], geometry: Mapping[str, Any]
) -> dict[str, Any]:
    references = generate_controlled_references(stage0e_config)
    rows = []
    for identifier in CONTROLLED_IDS:
        execution = stage0e_summary["calibrated"]["scenarios"][identifier]
        rows.append(
            {
                "fixture_id": identifier,
                "calibrated_passed": bool(execution["passed"]),
                "position_rmse_m": execution["repeatability"]["position_rmse_m"]["mean"],
                "yaw_rmse_rad": execution["repeatability"]["yaw_rmse_rad"]["mean"],
                "descriptor": geometry_descriptor(
                    references[identifier],
                    segment_epsilon_m=float(geometry["segment_epsilon_m"]),
                    curvature_zero_epsilon_per_m=float(
                        geometry["curvature_zero_epsilon_per_m"]
                    ),
                ),
            }
        )
    return {
        "fixtures": rows,
        "pass_fixture_ids": [row["fixture_id"] for row in rows if row["calibrated_passed"]],
        "fail_fixture_ids": [row["fixture_id"] for row in rows if not row["calibrated_passed"]],
        "stage0e_labels_reused_without_change": True,
    }


def fit_fixture_distance_model(
    fixture_geometry: Mapping[str, Any], distance_config: Mapping[str, Any]
) -> dict[str, Any]:
    features = tuple(str(value) for value in distance_config["features"])
    rows = fixture_geometry["fixtures"]
    matrix = np.asarray(
        [[float(row["descriptor"][key]) for key in features] for row in rows],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(matrix)):
        raise ValueError("fixture distance features must all be finite")
    minimum = np.min(matrix, axis=0)
    scale = np.maximum(
        np.max(matrix, axis=0) - minimum, float(distance_config["scale_floor"])
    )
    normalized = (matrix - minimum) / scale
    if len(rows) < 2:
        raise ValueError("fixture distance model requires at least two fixtures")
    nearest_loo = []
    for index in range(len(rows)):
        distances = np.linalg.norm(normalized - normalized[index], axis=1)
        distances[index] = np.inf
        nearest_loo.append(float(np.min(distances)))
    outside_radius = float(np.max(nearest_loo))
    if outside_radius <= 0.0:
        raise ValueError("fixture-only outside radius is degenerate")
    return {
        "features": list(features),
        "minimum": minimum.tolist(),
        "scale": scale.tolist(),
        "normalization": distance_config["normalization"],
        "fixture_leave_one_out_nearest_distances": {
            row["fixture_id"]: nearest_loo[index] for index, row in enumerate(rows)
        },
        "outside_radius": outside_radius,
        "ambiguous_margin": outside_radius
        * float(distance_config["ambiguous_margin_fraction_of_outside_radius"]),
        "fitted_from_lightnav": False,
    }


def compare_descriptor_to_fixtures(
    descriptor: Mapping[str, Any],
    fixture_geometry: Mapping[str, Any],
    model: Mapping[str, Any],
) -> dict[str, Any]:
    features = model["features"]
    vector = np.asarray([float(descriptor[key]) for key in features], dtype=np.float64)
    minimum = np.asarray(model["minimum"], dtype=np.float64)
    scale = np.asarray(model["scale"], dtype=np.float64)
    query = (vector - minimum) / scale
    distances = {
        row["fixture_id"]: float(
            np.linalg.norm(
                query
                - (
                    np.asarray(
                        [float(row["descriptor"][key]) for key in features],
                        dtype=np.float64,
                    )
                    - minimum
                )
                / scale
            )
        )
        for row in fixture_geometry["fixtures"]
    }
    pass_ids = fixture_geometry["pass_fixture_ids"]
    fail_ids = fixture_geometry["fail_fixture_ids"]
    nearest = min(distances, key=lambda key: (distances[key], key))
    nearest_pass = min(pass_ids, key=lambda key: (distances[key], key))
    nearest_fail = min(fail_ids, key=lambda key: (distances[key], key))
    nearest_distance = distances[nearest]
    if nearest_distance > float(model["outside_radius"]):
        label = "OUTSIDE_STAGE0E_TESTED_GEOMETRY"
    elif abs(distances[nearest_pass] - distances[nearest_fail]) <= float(
        model["ambiguous_margin"]
    ):
        label = "AMBIGUOUS"
    elif distances[nearest_pass] < distances[nearest_fail]:
        label = "PASS_FIXTURE_LIKE"
    else:
        label = "STRONG_TURN_FAIL_FIXTURE_LIKE"
    if label not in ENVELOPE_LABELS:
        raise AssertionError(label)
    return {
        "label": label,
        "nearest_fixture": nearest,
        "nearest_fixture_passed": nearest in pass_ids,
        "nearest_fixture_distance": nearest_distance,
        "nearest_passing_fixture": nearest_pass,
        "nearest_passing_distance": distances[nearest_pass],
        "nearest_failing_strong_turn_fixture": nearest_fail,
        "nearest_failing_distance": distances[nearest_fail],
        "all_fixture_distances": distances,
        "descriptive_not_classifier": True,
    }


def severity_key(descriptor: Mapping[str, Any], order: Sequence[str]) -> tuple[float, ...]:
    values = []
    for name in order:
        value = descriptor.get(name)
        values.append(0.0 if value is None else float(value))
    if not np.all(np.isfinite(values)):
        raise ValueError("severity values must be finite or null")
    return tuple(values)


def select_representatives(
    rows: Sequence[Mapping[str, Any]], severity_order: Sequence[str]
) -> dict[str, Any]:
    selected: dict[tuple[str, str], dict[str, Any]] = {}

    def add(row: Mapping[str, Any], reason: str) -> None:
        key = (str(row["kind"]), str(row["world_trajectory_sha256"]))
        if key not in selected:
            selected[key] = {
                "reference_id": row["reference_id"],
                "kind": row["kind"],
                "world_trajectory_sha256": row["world_trajectory_sha256"],
                "reasons": [],
            }
        selected[key]["reasons"].append(reason)

    for kind in ("OLD", "FRESH"):
        candidates = sorted(
            [row for row in rows if row["kind"] == kind],
            key=lambda row: (
                severity_key(row["descriptor"], severity_order), row["reference_id"]
            ),
        )
        if not candidates:
            raise ValueError(f"no {kind} references available for representative selection")
        add(candidates[0], f"lowest_severity_{kind.lower()}")
        add(candidates[(len(candidates) - 1) // 2], f"median_severity_{kind.lower()}")
        add(candidates[-1], f"highest_severity_{kind.lower()}")
    strong = min(
        rows,
        key=lambda row: (
            float(row["fixture_comparison"]["nearest_failing_distance"]),
            row["kind"],
            row["reference_id"],
        ),
    )
    add(strong, "nearest_strong_turn_failing_fixture")
    direction = [
        row for row in rows if int(row["descriptor"]["curvature_sign_change_count"]) > 0
    ]
    if direction:
        direction_row = max(
            direction,
            key=lambda row: (
                int(row["descriptor"]["curvature_sign_change_count"]),
                severity_key(row["descriptor"], severity_order),
                row["reference_id"],
            ),
        )
        add(direction_row, "highest_sign_change_reference")
    result = sorted(selected.values(), key=lambda row: (row["kind"], row["reference_id"]))
    return {
        "selection_frozen_before_execution": True,
        "median_rule": "lower_middle_after_severity_sort",
        "reference_count": len(result),
        "references": result,
    }


def distribution_summary(values: Iterable[float | int | None]) -> dict[str, Any]:
    finite = np.asarray(
        [float(value) for value in values if value is not None], dtype=np.float64
    )
    if finite.size == 0 or not np.all(np.isfinite(finite)):
        return {"count": 0, "min": None, "p25": None, "median": None, "p75": None, "p90": None, "p95": None, "max": None}
    return {
        "count": int(finite.size),
        "min": float(np.min(finite)),
        "p25": _percentile(finite, 25),
        "median": _percentile(finite, 50),
        "p75": _percentile(finite, 75),
        "p90": _percentile(finite, 90),
        "p95": _percentile(finite, 95),
        "max": float(np.max(finite)),
    }


def summarize_descriptors(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "reference_count": len(rows),
        "descriptors": {
            name: distribution_summary(row["descriptor"].get(name) for row in rows)
            for name in SUMMARY_DESCRIPTORS
        },
    }


def build_grouped_geometry_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    geometry_groups: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    latency_groups: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        for group in row["source_geometry_ids"]:
            geometry_groups[group].append(row)
        for group in row["source_latency_ids"]:
            latency_groups[group].append(row)
    return {
        "all_unique_references": summarize_descriptors(rows),
        "by_geometry_id": {
            key: summarize_descriptors(value)
            for key, value in sorted(geometry_groups.items())
        },
        "by_latency_id": {
            key: summarize_descriptors(value)
            for key, value in sorted(latency_groups.items())
        },
        "summary_weighting": "unique world reference within each reported group",
    }


def extend_closed_loop_metrics(telemetry: ClosedLoopTelemetry) -> dict[str, Any]:
    metrics = compute_closed_loop_metrics(telemetry)
    control_rows = np.flatnonzero(
        (telemetry.control_indices >= 0)
        & np.concatenate(([True], np.diff(telemetry.control_indices) != 0))
    )
    desired = telemetry.desired_body[control_rows]
    if desired.size == 0:
        raise ValueError("execution telemetry contains no controller commands")
    omega = desired[:, 1]
    nonzero = np.sign(omega[np.abs(omega) > 1e-9])
    rotate = (np.abs(desired[:, 0]) <= 1e-9) & (np.abs(omega) > 1e-9)
    metrics.update(
        {
            "desired_v_mean_mps": float(np.mean(desired[:, 0])),
            "desired_v_p95_mps": _percentile(desired[:, 0], 95),
            "desired_v_max_mps": float(np.max(desired[:, 0])),
            "desired_abs_omega_mean_rps": float(np.mean(np.abs(omega))),
            "desired_abs_omega_p95_rps": _percentile(np.abs(omega), 95),
            "desired_abs_omega_max_rps": float(np.max(np.abs(omega))),
            "desired_omega_sign_change_count": (
                int(np.sum(nonzero[1:] != nonzero[:-1])) if nonzero.size > 1 else 0
            ),
            "rotate_in_place_event_count": int(np.sum(rotate)),
            "rotate_in_place_fraction": float(np.mean(rotate)),
        }
    )
    assert_finite_json_tree(metrics)
    return metrics


def final_envelope_decision(
    *,
    old_results: Sequence[Mapping[str, Any]],
    fresh_results: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    if not old_results or not fresh_results:
        label = "LIGHTNAV_ENVELOPE_INSUFFICIENT_DATA"
    else:
        failed = [row for row in (*old_results, *fresh_results) if not row["passed"]]
        if not failed:
            label = "LIGHTNAV_ENVELOPE_COVERED_BY_CURRENT_CALIBRATED_PLATFORM"
        elif any(
            row["fixture_label"] == "STRONG_TURN_FAIL_FIXTURE_LIKE" for row in failed
        ):
            label = "LIGHTNAV_ENVELOPE_REACHES_UNVALIDATED_STRONG_TURN_REGION"
        else:
            label = "LIGHTNAV_ENVELOPE_PARTIALLY_COVERED"
    if label not in FINAL_LABELS:
        raise AssertionError(label)
    old_pass = bool(old_results) and all(row["passed"] for row in old_results)
    fresh_pass = bool(fresh_results) and all(row["passed"] for row in fresh_results)
    return {
        "status": label,
        "old_coverage": (
            "OLD_EXECUTION_ENVELOPE_COVERED"
            if old_pass
            else "OLD_EXECUTION_ENVELOPE_HAS_UNVALIDATED_CASES"
        ),
        "fresh_coverage": (
            "FRESH_EXECUTION_ENVELOPE_COVERED_AT_OBSERVATION_STATE"
            if fresh_pass
            else "FRESH_EXECUTION_ENVELOPE_HAS_UNVALIDATED_CASES"
        ),
        "unique_old_count": len(old_results),
        "unique_old_pass_count": sum(bool(row["passed"]) for row in old_results),
        "unique_fresh_count": len(fresh_results),
        "unique_fresh_pass_count": sum(bool(row["passed"]) for row in fresh_results),
        "execution_platform_validated": False,
        "stage0e_strong_turn_failure_overridden": False,
        "claim_scope": "frozen LightNav workload coverage in deterministic Isaac simulation",
    }
