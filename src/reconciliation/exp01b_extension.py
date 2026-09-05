"""Pure cohort validation and aggregation for the EXP-01B extension."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike

from reconciliation.online_switch import (
    load_strict_json,
    validate_trial_output,
)
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


ATTEMPT_CLASSIFICATIONS = frozenset(
    {
        "VALID",
        "TIMING_INVALID",
        "OLD_EXHAUSTED",
        "NEW_TIMEOUT",
        "MODEL_STOP_OUTPUT",
        "OTHER_PROTOCOL_FAILURE",
    }
)
PRIMARY_METRICS = (
    "translation_gap_m",
    "yaw_gap_rad",
    "translational_motion_jump_m",
    "yaw_motion_jump_rad",
)


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_extension_config(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Validate and return the predeclared cohort in deterministic order."""

    cohort = config.get("extension_cohort")
    if not isinstance(cohort, Mapping):
        raise ValueError("extension_cohort must be a mapping")
    target = cohort.get("target_valid_per_condition")
    maximum = cohort.get("max_attempts_per_condition")
    if not isinstance(target, int) or target < 1:
        raise ValueError("target_valid_per_condition must be a positive integer")
    if not isinstance(maximum, int) or maximum < target:
        raise ValueError("max_attempts_per_condition must be at least the target")
    conditions = cohort.get("conditions")
    if not isinstance(conditions, list) or len(conditions) != 4:
        raise ValueError("exactly four extension conditions are required")
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for position, source in enumerate(conditions):
        if not isinstance(source, Mapping):
            raise ValueError("each condition must be a mapping")
        condition_id = str(source.get("id", ""))
        if not condition_id or condition_id in seen:
            raise ValueError("condition ids must be non-empty and unique")
        seen.add(condition_id)
        pose = validate_pose_se2(source.get("initial_pose_se2"), name="initial_pose_se2")
        delay = _finite(source.get("new_observation_delay_s"), "new_observation_delay_s")
        if delay <= 0.0:
            raise ValueError("new_observation_delay_s must be positive")
        description = str(source.get("description", "")).strip()
        if not description:
            raise ValueError("condition description must not be empty")
        normalized.append(
            {
                "order": position,
                "id": condition_id,
                "description": description,
                "initial_pose_se2": pose.tolist(),
                "new_observation_delay_s": delay,
                "target_valid": target,
                "max_attempts": maximum,
            }
        )
    return normalized


def enumerate_condition_attempts(config: Mapping[str, Any]) -> list[tuple[str, int]]:
    """Return the finite maximum attempt plan without looking at outcomes."""

    return [
        (condition["id"], index)
        for condition in validate_extension_config(config)
        for index in range(condition["max_attempts"])
    ]


def classify_attempt(
    *, validity_checks: Mapping[str, Any], stop_output: bool, completed: bool = True
) -> str:
    """Classify one attempt; STOP remains a timing-valid model outcome."""

    if not completed:
        return "OTHER_PROTOCOL_FAILURE"
    if validity_checks.get("within_configured_new_wait") is False:
        return "NEW_TIMEOUT"
    if validity_checks.get("old_not_exhausted_before_ready") is False:
        return "OLD_EXHAUSTED"
    if validity_checks.get("rtf_in_range") is False:
        return "TIMING_INVALID"
    required = (
        "ready_after_observation",
        "robot_moved_during_inference",
        "old_progress_nonregressing",
    )
    if not all(validity_checks.get(key) is True for key in required):
        return "OTHER_PROTOCOL_FAILURE"
    return "MODEL_STOP_OUTPUT" if stop_output else "VALID"


def is_stop_actions(actions: ArrayLike, *, absolute_tolerance: float = 1e-8) -> bool:
    array = np.asarray(actions)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] < 1:
        raise ValueError("actions must have arbitrary N x 3 shape")
    if not np.issubdtype(array.dtype, np.floating) or not np.all(np.isfinite(array)):
        raise ValueError("actions must be finite floating-point values")
    tolerance = _finite(absolute_tolerance, "absolute_tolerance")
    if tolerance < 0.0:
        raise ValueError("absolute_tolerance must be nonnegative")
    return bool(np.max(np.abs(array)) <= tolerance)


def geometry_descriptors(
    *,
    actual_pose_before_ready: ArrayLike,
    actual_pose_at_ready: ArrayLike,
    fresh_world: ArrayLike,
    zero_motion_tolerance_m: float = 1e-8,
) -> dict[str, Any]:
    """Describe actual incoming and raw FRESH first motion without causal claims."""

    before = validate_pose_se2(actual_pose_before_ready, name="actual_pose_before_ready")
    ready = validate_pose_se2(actual_pose_at_ready, name="actual_pose_at_ready")
    fresh = validate_se2_trajectory(fresh_world, name="fresh_world")
    if fresh.shape[0] < 2:
        raise ValueError("fresh_world needs at least two rows for geometry descriptors")
    tolerance = _finite(zero_motion_tolerance_m, "zero_motion_tolerance_m")
    if tolerance < 0.0:
        raise ValueError("zero_motion_tolerance_m must be nonnegative")

    incoming_vector = ready[:2] - before[:2]
    fresh_vector = fresh[1, :2] - fresh[0, :2]
    incoming_magnitude = float(np.linalg.norm(incoming_vector))
    fresh_magnitude = float(np.linalg.norm(fresh_vector))
    incoming_defined = incoming_magnitude > tolerance
    fresh_defined = fresh_magnitude > tolerance
    incoming_heading = (
        float(math.atan2(incoming_vector[1], incoming_vector[0])) if incoming_defined else None
    )
    fresh_heading = (
        float(math.atan2(fresh_vector[1], fresh_vector[0])) if fresh_defined else None
    )
    disagreement = (
        float(wrap_angle(fresh_heading - incoming_heading))
        if incoming_defined and fresh_defined
        else None
    )
    ratio = fresh_magnitude / incoming_magnitude if incoming_defined else None
    return {
        "definition": (
            "incoming is actual pose one control interval before ready -> actual ready; "
            "FRESH first motion is world F[0] -> F[1]"
        ),
        "old_incoming_motion_m": incoming_magnitude,
        "old_incoming_heading_rad": incoming_heading,
        "old_incoming_heading_defined": incoming_defined,
        "old_incoming_yaw_increment_rad": float(wrap_angle(ready[2] - before[2])),
        "fresh_first_motion_m": fresh_magnitude,
        "fresh_first_heading_rad": fresh_heading,
        "fresh_first_heading_defined": fresh_defined,
        "fresh_first_yaw_increment_rad": float(wrap_angle(fresh[1, 2] - fresh[0, 2])),
        "old_fresh_initial_heading_disagreement_rad": disagreement,
        "old_fresh_initial_heading_disagreement_abs_rad": (
            abs(disagreement) if disagreement is not None else None
        ),
        "fresh_minus_old_motion_m": fresh_magnitude - incoming_magnitude,
        "fresh_old_motion_mismatch_abs_m": abs(fresh_magnitude - incoming_magnitude),
        "fresh_to_old_motion_ratio": ratio,
        "zero_motion_tolerance_m": tolerance,
    }


def timeline_inference_activity(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    in_flight = [row for row in rows if bool(int(row["new_inference_in_flight"]))]
    nonzero = [
        row
        for row in in_flight
        if abs(_finite(row["commanded_v"], "commanded_v")) > 1e-9
        or abs(_finite(row["commanded_omega"], "commanded_omega")) > 1e-9
    ]
    return {
        "in_flight_timeline_samples": len(in_flight),
        "in_flight_nonzero_old_command_samples": len(nonzero),
        "in_flight_nonzero_old_command_fraction": (
            len(nonzero) / len(in_flight) if in_flight else 0.0
        ),
    }


def descriptive_statistics(values: Sequence[float]) -> dict[str, Any]:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or array.size < 1 or not np.all(np.isfinite(array)):
        raise ValueError("statistics require a finite non-empty one-dimensional sequence")
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "std": float(np.std(array, ddof=0)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        "p25": float(np.percentile(array, 25, method="linear")),
        "p75": float(np.percentile(array, 75, method="linear")),
        "p90": float(np.percentile(array, 90, method="linear")) if array.size >= 3 else None,
    }


def _metric_value(record: Mapping[str, Any], name: str) -> float | None:
    paths = {
        "host_request_response_latency_s": ("timing", "request_response_wall_s"),
        "lightnav_predict_latency_ms": ("lightnav_predict_host_latency_ms",),
        "lightnav_reported_latency_ms": ("lightnav_reported_latency_ms",),
        "simulation_latency_s": ("timing", "simulation_observation_to_ready_s"),
        "real_time_factor": ("timing", "real_time_factor"),
        "robot_translation_observation_to_ready_m": ("observation_to_ready_translation_m",),
        "robot_abs_yaw_observation_to_ready_rad": ("observation_to_ready_yaw_rad",),
        "old_progress_delta": ("old_progress_delta",),
        "in_flight_timeline_samples": ("inference_activity", "in_flight_timeline_samples"),
        "in_flight_nonzero_old_command_fraction": (
            "inference_activity",
            "in_flight_nonzero_old_command_fraction",
        ),
        "translation_gap_m": ("metrics", "translation_gap_m"),
        "yaw_gap_rad": ("metrics", "yaw_gap_rad"),
        "translational_motion_jump_m": ("metrics", "translational_motion_jump_m"),
        "yaw_motion_jump_rad": ("metrics", "yaw_motion_jump_rad"),
        "old_incoming_motion_m": ("geometry", "old_incoming_motion_m"),
        "fresh_first_motion_m": ("geometry", "fresh_first_motion_m"),
        "heading_disagreement_abs_rad": (
            "geometry",
            "old_fresh_initial_heading_disagreement_abs_rad",
        ),
        "motion_mismatch_abs_m": ("geometry", "fresh_old_motion_mismatch_abs_m"),
    }
    value: Any = record
    for key in paths[name]:
        value = value[key]
    if value is None:
        return None
    number = _finite(value, name)
    if name == "robot_abs_yaw_observation_to_ready_rad":
        number = abs(number)
    return number


AGGREGATE_METRICS = (
    "host_request_response_latency_s",
    "lightnav_predict_latency_ms",
    "lightnav_reported_latency_ms",
    "simulation_latency_s",
    "real_time_factor",
    "robot_translation_observation_to_ready_m",
    "robot_abs_yaw_observation_to_ready_rad",
    "old_progress_delta",
    "in_flight_timeline_samples",
    "in_flight_nonzero_old_command_fraction",
    *PRIMARY_METRICS,
    "old_incoming_motion_m",
    "fresh_first_motion_m",
    "heading_disagreement_abs_rad",
    "motion_mismatch_abs_m",
)


def _summarize_subset(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    statistics: dict[str, Any] = {}
    raw: dict[str, list[float]] = {}
    for name in AGGREGATE_METRICS:
        values = [value for row in records if (value := _metric_value(row, name)) is not None]
        raw[name] = values
        statistics[name] = descriptive_statistics(values) if values else None
    threshold_names = {
        "translation_gap": "exceeds_translation_threshold",
        "yaw_gap": "exceeds_yaw_threshold",
        "translation_motion_jump": "exceeds_translational_motion_jump_threshold",
        "yaw_motion_jump": "exceeds_yaw_motion_jump_threshold",
    }
    threshold_counts = {
        label: sum(bool(row["threshold_exceedance"][key]) for row in records)
        for label, key in threshold_names.items()
    }
    translation_combined = sum(
        bool(row["threshold_exceedance"]["exceeds_translation_threshold"])
        or bool(row["threshold_exceedance"]["exceeds_translational_motion_jump_threshold"])
        for row in records
    )
    yaw_combined = sum(
        bool(row["threshold_exceedance"]["exceeds_yaw_threshold"])
        or bool(row["threshold_exceedance"]["exceeds_yaw_motion_jump_threshold"])
        for row in records
    )
    count = len(records)
    return {
        "count": count,
        "raw_lists": raw,
        "statistics": statistics,
        "threshold_exceedance": {
            **{
                name: {"count": value, "percentage": 100.0 * value / count if count else 0.0}
                for name, value in threshold_counts.items()
            },
            "translation_inconsistent": {
                "count": translation_combined,
                "percentage": 100.0 * translation_combined / count if count else 0.0,
                "definition": "translation gap > threshold OR translation-motion jump > threshold",
            },
            "yaw_inconsistent": {
                "count": yaw_combined,
                "percentage": 100.0 * yaw_combined / count if count else 0.0,
                "definition": "yaw gap > threshold OR yaw-motion jump > threshold",
            },
        },
    }


def aggregate_extension(
    records: Sequence[Mapping[str, Any]], *, condition_ids: Sequence[str]
) -> dict[str, Any]:
    if not records:
        raise ValueError("at least one attempt record is required")
    ids = list(condition_ids)
    if not ids or len(set(ids)) != len(ids):
        raise ValueError("condition ids must be non-empty and unique")
    normalized = [dict(row) for row in records]
    for row in normalized:
        if row.get("classification") not in ATTEMPT_CLASSIFICATIONS:
            raise ValueError("attempt has invalid classification")
        if row.get("condition_id") not in ids:
            raise ValueError("attempt has undeclared condition")
    valid = [row for row in normalized if row.get("timing_valid") is True]

    def summarize_group(group: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        group_valid = [row for row in group if row.get("timing_valid") is True]
        stop = [row for row in group_valid if row.get("stop_output") is True]
        non_stop = [row for row in group_valid if row.get("stop_output") is not True]
        classifications = {
            name: sum(row["classification"] == name for row in group)
            for name in sorted(ATTEMPT_CLASSIFICATIONS)
        }
        return {
            "attempted": len(group),
            "timing_valid": len(group_valid),
            "timing_invalid_or_failed": len(group) - len(group_valid),
            "stop": len(stop),
            "non_stop": len(non_stop),
            "classifications": classifications,
            "including_stop": _summarize_subset(group_valid),
            "excluding_stop": _summarize_subset(non_stop),
            "stop_only": _summarize_subset(stop),
        }

    return {
        "experiment": "EXP-01B Extension",
        "condition_order": ids,
        "overall": summarize_group(normalized),
        "per_condition": {
            condition_id: summarize_group(
                [row for row in normalized if row["condition_id"] == condition_id]
            )
            for condition_id in ids
        },
        "timing_valid_attempt_paths": [row["artifact_path"] for row in valid],
        "claim_boundary": (
            "Controlled online raw-switch characterization only; percentages are descriptive, "
            "not statistical significance or a general LightNav occurrence rate."
        ),
    }


def select_representative_samples(
    records: Sequence[Mapping[str, Any]], condition_ids: Sequence[str]
) -> dict[str, str | None]:
    """Choose valid non-STOP sample nearest condition median, then attempt/path."""

    result: dict[str, str | None] = {}
    for condition_id in condition_ids:
        candidates = [
            row
            for row in records
            if row.get("condition_id") == condition_id
            and row.get("timing_valid") is True
            and row.get("stop_output") is not True
        ]
        if not candidates:
            result[condition_id] = None
            continue
        median = float(
            np.median([row["metrics"]["translational_motion_jump_m"] for row in candidates])
        )
        selected = min(
            candidates,
            key=lambda row: (
                abs(float(row["metrics"]["translational_motion_jump_m"]) - median),
                int(row["attempt_index"]),
                str(row["artifact_path"]),
            ),
        )
        result[condition_id] = str(selected["artifact_path"])
    return result


def attempt_csv_row(record: Mapping[str, Any]) -> dict[str, Any]:
    geometry = record.get("geometry", {})
    timing = record.get("timing", {})
    metrics = record.get("metrics", {})
    thresholds = record.get("threshold_exceedance", {})
    activity = record.get("inference_activity", {})
    return {
        "condition_id": record["condition_id"],
        "attempt_index": record["attempt_index"],
        "global_episode_index": record["global_episode_index"],
        "classification": record["classification"],
        "timing_valid": record["timing_valid"],
        "stop_output": record["stop_output"],
        "artifact_path": record["artifact_path"],
        "host_request_response_latency_s": timing.get("request_response_wall_s"),
        "lightnav_predict_latency_ms": record.get("lightnav_predict_host_latency_ms"),
        "lightnav_reported_latency_ms": record.get("lightnav_reported_latency_ms"),
        "simulation_latency_s": timing.get("simulation_observation_to_ready_s"),
        "real_time_factor": timing.get("real_time_factor"),
        "robot_translation_observation_to_ready_m": record.get(
            "observation_to_ready_translation_m"
        ),
        "robot_yaw_observation_to_ready_rad": record.get("observation_to_ready_yaw_rad"),
        "old_progress_at_observation": record.get("old_progress_at_observation"),
        "old_progress_at_ready": record.get("old_progress_at_ready"),
        "old_progress_delta": record.get("old_progress_delta"),
        "in_flight_timeline_samples": activity.get("in_flight_timeline_samples"),
        "in_flight_nonzero_old_command_fraction": activity.get(
            "in_flight_nonzero_old_command_fraction"
        ),
        **{name: metrics.get(name) for name in PRIMARY_METRICS},
        "old_incoming_motion_m": geometry.get("old_incoming_motion_m"),
        "old_incoming_heading_rad": geometry.get("old_incoming_heading_rad"),
        "fresh_first_motion_m": geometry.get("fresh_first_motion_m"),
        "fresh_first_heading_rad": geometry.get("fresh_first_heading_rad"),
        "heading_disagreement_rad": geometry.get(
            "old_fresh_initial_heading_disagreement_rad"
        ),
        "motion_mismatch_abs_m": geometry.get("fresh_old_motion_mismatch_abs_m"),
        **{name: thresholds.get(name) for name in sorted(thresholds)},
    }


def write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError("CSV output requires at least one row")
    flattened = [attempt_csv_row(row) for row in rows]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flattened[0]))
        writer.writeheader()
        writer.writerows(flattened)


def load_attempt_records(root: Path, condition_ids: Sequence[str]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for condition_id in condition_ids:
        for attempt_dir in sorted((root / "attempts" / condition_id).glob("attempt_*")):
            if not attempt_dir.is_dir():
                continue
            validation = validate_trial_output(attempt_dir)
            payload = load_strict_json(attempt_dir / "results/switch_metrics.json")
            if validation["research_valid"] is not bool(payload["timing_valid"]):
                raise ValueError("attempt validation and timing_valid disagree")
            if payload["condition_id"] != condition_id:
                raise ValueError("attempt condition does not match its directory")
            fresh_actions = np.load(attempt_dir / "raw/new_actions.npy", allow_pickle=False)
            if is_stop_actions(fresh_actions) is not bool(payload["stop_output"]):
                raise ValueError("STOP classification differs from immutable raw FRESH")
            expected_classification = classify_attempt(
                validity_checks=payload["validity_checks"],
                stop_output=bool(payload["stop_output"]),
            )
            if payload["classification"] != expected_classification:
                raise ValueError("attempt classification differs from recorded validity checks")
            if Path(payload["artifact_path"]).resolve() != attempt_dir.resolve():
                raise ValueError("attempt artifact path does not match its directory")
            records.append(payload)
    return records


def validate_extension_output(root: str | Path) -> dict[str, Any]:
    directory = Path(root)
    protocol = load_strict_json(directory / "protocol.json")
    conditions = protocol["conditions"]
    condition_ids = [item["id"] for item in conditions]
    records = load_attempt_records(directory, condition_ids)
    summary = load_strict_json(directory / "summary.json")
    rebuilt = aggregate_extension(records, condition_ids=condition_ids)
    if summary["aggregate"] != rebuilt:
        raise ValueError("summary aggregate does not match immutable attempt artifacts")
    expected_representatives = select_representative_samples(records, condition_ids)
    if summary["representative_samples"] != expected_representatives:
        raise ValueError("representative samples are not deterministically selected")
    for plot in summary.get("representative_plots", {}).values():
        if plot is not None and not Path(plot).is_file():
            raise ValueError(f"representative plot is missing: {plot}")
    with (directory / "all_attempts.csv").open(newline="", encoding="utf-8") as stream:
        all_rows = list(csv.DictReader(stream))
    with (directory / "valid_transitions.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        valid_rows = list(csv.DictReader(stream))
    if len(all_rows) != len(records):
        raise ValueError("all_attempts.csv row count differs from artifacts")
    if len(valid_rows) != sum(row["timing_valid"] for row in records):
        raise ValueError("valid_transitions.csv row count differs from artifacts")
    return {
        "valid_output": True,
        "attempt_count": len(records),
        "timing_valid_count": sum(row["timing_valid"] for row in records),
        "stop_count": sum(row["timing_valid"] and row["stop_output"] for row in records),
        "condition_counts": {
            condition_id: sum(
                row["condition_id"] == condition_id and row["timing_valid"] for row in records
            )
            for condition_id in condition_ids
        },
    }
