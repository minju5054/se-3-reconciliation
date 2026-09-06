"""Pure protocol, timing, aggregation, and validation for redesigned EXP-01B."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from reconciliation.controller_switch_metrics import (
    command_rows_to_array,
    controller_switch_metrics,
)
from reconciliation.exp01b_extension import descriptive_statistics, is_stop_actions
from reconciliation.lightnav_adapter import lightnav_local_to_world, raw_actions_to_local_path
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.trajectory import validate_pose_se2


CELL_METRICS = (
    "delta_v_abs_mps",
    "delta_omega_abs_rps",
    "post_switch_max_abs_delta_v_mps",
    "post_switch_mean_abs_delta_v_mps",
    "post_switch_max_abs_delta_omega_rps",
    "post_switch_mean_abs_delta_omega_rps",
    "translation_pose_gap_m",
    "yaw_pose_gap_rad",
    "local_spatial_step_magnitude_mismatch_m",
    "effective_latency_sim_s",
    "model_latency_wall_s",
    "robot_translation_observation_to_switch_m",
    "robot_yaw_observation_to_switch_rad",
    "real_time_factor",
    "old_fresh_tangent_disagreement_abs_rad",
)


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f"{name} must be finite" + (f" and >= {minimum}" if minimum is not None else ""))
    return result


def validate_controlled_config(config: Mapping[str, Any]) -> list[dict[str, Any]]:
    design = config.get("controlled_latency_design")
    if not isinstance(design, Mapping):
        raise ValueError("controlled_latency_design must be a mapping")
    target = design.get("target_valid_moving_per_cell")
    cap = design.get("max_attempts_per_cell")
    if not isinstance(target, int) or target < 1:
        raise ValueError("target_valid_moving_per_cell must be a positive integer")
    if not isinstance(cap, int) or cap < target:
        raise ValueError("max_attempts_per_cell must be at least the target")
    latencies = design.get("latency_conditions")
    if not isinstance(latencies, list) or len(latencies) != 2:
        raise ValueError("exactly two latency conditions are required")
    normalized_latency: list[dict[str, Any]] = []
    seen_latency: set[str] = set()
    for item in latencies:
        if not isinstance(item, Mapping):
            raise ValueError("latency condition must be a mapping")
        identifier = str(item.get("id", ""))
        delay = _finite(item.get("added_delay_s"), "added_delay_s", minimum=0.0)
        if not identifier or identifier in seen_latency:
            raise ValueError("latency ids must be non-empty and unique")
        seen_latency.add(identifier)
        normalized_latency.append({"id": identifier, "added_delay_s": delay})
    if sorted(item["added_delay_s"] for item in normalized_latency) != [0.0, 0.5]:
        raise ValueError("latency conditions must be exactly 0.0 and 0.5 seconds")

    geometries = design.get("geometry_conditions")
    if not isinstance(geometries, list) or len(geometries) != 3:
        raise ValueError("exactly three geometry conditions are required")
    required_classes = {"straight", "turn", "route_change"}
    seen_geometry: set[str] = set()
    normalized_geometry: list[dict[str, Any]] = []
    for item in geometries:
        if not isinstance(item, Mapping):
            raise ValueError("geometry condition must be a mapping")
        identifier = str(item.get("id", ""))
        geometry_class = str(item.get("geometry_class", ""))
        if not identifier or identifier in seen_geometry:
            raise ValueError("geometry ids must be non-empty and unique")
        if geometry_class not in required_classes:
            raise ValueError("geometry_class must be straight, turn, or route_change")
        instruction = str(item.get("instruction", "")).strip()
        scene_id = str(item.get("scene_id", "")).strip()
        if not instruction or not scene_id:
            raise ValueError("geometry instruction and scene_id must be non-empty")
        pose = validate_pose_se2(item.get("initial_pose_se2"))
        observation_delay = _finite(
            item.get("fresh_observation_delay_s"),
            "fresh_observation_delay_s",
            minimum=0.0,
        )
        qualification = item.get("qualification")
        if not isinstance(qualification, Mapping):
            raise ValueError("each geometry must contain qualification metadata")
        if design.get("primary_config_frozen") is True and qualification.get("accepted") is not True:
            raise ValueError("a frozen primary config requires every geometry to be accepted")
        seen_geometry.add(identifier)
        normalized_geometry.append(
            {
                "id": identifier,
                "geometry_class": geometry_class,
                "instruction": instruction,
                "scene_id": scene_id,
                "initial_pose_se2": pose.tolist(),
                "fresh_observation_delay_s": observation_delay,
                "qualification": json.loads(json.dumps(qualification, allow_nan=False)),
            }
        )
    if {item["geometry_class"] for item in normalized_geometry} != required_classes:
        raise ValueError("geometry classes must contain exactly straight, turn, and route_change")

    return [
        {
            "cell_id": f"{geometry['id']}__{latency['id']}",
            "geometry": geometry,
            "latency": latency,
            "target_valid_moving": target,
            "max_attempts": cap,
        }
        for geometry in normalized_geometry
        for latency in normalized_latency
    ]


def enumerate_primary_attempts(config: Mapping[str, Any]) -> list[tuple[str, int]]:
    return [
        (cell["cell_id"], attempt)
        for cell in validate_controlled_config(config)
        for attempt in range(cell["max_attempts"])
    ]


@dataclass(frozen=True, slots=True)
class ControlledLatencyTiming:
    observation_sim_time_s: float
    request_host_monotonic_ns: int
    model_ready_sim_time_s: float
    model_ready_host_monotonic_ns: int
    fresh_usable_sim_time_s: float
    fresh_usable_host_monotonic_ns: int
    configured_added_delay_s: float

    def __post_init__(self) -> None:
        for name in (
            "observation_sim_time_s",
            "model_ready_sim_time_s",
            "fresh_usable_sim_time_s",
            "configured_added_delay_s",
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name, minimum=0.0))
        for name in (
            "request_host_monotonic_ns",
            "model_ready_host_monotonic_ns",
            "fresh_usable_host_monotonic_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.model_ready_sim_time_s < self.observation_sim_time_s:
            raise ValueError("model ready simulation time precedes observation")
        if self.fresh_usable_sim_time_s < self.model_ready_sim_time_s:
            raise ValueError("FRESH usable simulation time precedes model ready")
        if self.model_ready_host_monotonic_ns < self.request_host_monotonic_ns:
            raise ValueError("model ready host time precedes request")
        if self.fresh_usable_host_monotonic_ns < self.model_ready_host_monotonic_ns:
            raise ValueError("FRESH usable host time precedes model ready")

    def to_dict(self) -> dict[str, float | int | str]:
        model_wall = (self.model_ready_host_monotonic_ns - self.request_host_monotonic_ns) / 1e9
        effective_wall = (
            self.fresh_usable_host_monotonic_ns - self.request_host_monotonic_ns
        ) / 1e9
        model_sim = self.model_ready_sim_time_s - self.observation_sim_time_s
        effective_sim = self.fresh_usable_sim_time_s - self.observation_sim_time_s
        added_sim = self.fresh_usable_sim_time_s - self.model_ready_sim_time_s
        return {
            "observation_sim_time_s": self.observation_sim_time_s,
            "request_host_monotonic_ns": self.request_host_monotonic_ns,
            "model_ready_sim_time_s": self.model_ready_sim_time_s,
            "model_ready_host_monotonic_ns": self.model_ready_host_monotonic_ns,
            "fresh_usable_sim_time_s": self.fresh_usable_sim_time_s,
            "fresh_usable_host_monotonic_ns": self.fresh_usable_host_monotonic_ns,
            "configured_added_delay_s": self.configured_added_delay_s,
            "model_latency_wall_s": model_wall,
            "model_latency_sim_s": model_sim,
            "measured_added_delay_wall_s": effective_wall - model_wall,
            "measured_added_delay_sim_s": added_sim,
            "effective_latency_wall_s": effective_wall,
            "effective_latency_sim_s": effective_sim,
            "real_time_factor": effective_sim / effective_wall if effective_wall > 0.0 else math.inf,
            "semantics": "model inference untouched; added delay scheduled in simulation while OLD remains active",
        }


def classify_controlled_attempt(
    *, checks: Mapping[str, bool], stop_output: bool
) -> str:
    if not checks.get("response_received", False):
        return "NEW_TIMEOUT"
    protocol_checks = (
        "request_after_observation",
        "fresh_anchored_at_observation",
        "fresh_raw_unchanged",
    )
    if not all(checks.get(name, False) for name in protocol_checks):
        return "OTHER_PROTOCOL_FAILURE"
    if not checks.get("rtf_in_range", False):
        return "TIMING_INVALID"
    if not checks.get("old_active_during_model", False) or not checks.get(
        "old_active_during_added_delay", False
    ):
        return "OLD_EXHAUSTED"
    if not checks.get("robot_moved_observation_to_switch", False):
        return "OTHER_PROTOCOL_FAILURE"
    if not checks.get("controller_switch_recorded", False):
        return "OTHER_PROTOCOL_FAILURE"
    if stop_output:
        return "MODEL_STOP_OUTPUT"
    return "VALID_MOVING"


def _metric(record: Mapping[str, Any], name: str) -> float | None:
    for section in ("controller_metrics", "geometry", "timing"):
        value = record.get(section, {}).get(name)
        if value is not None:
            return _finite(value, name)
    return None


def _summarize(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    moving = [row for row in records if row.get("classification") == "VALID_MOVING"]
    stop = [row for row in records if row.get("classification") == "MODEL_STOP_OUTPUT"]
    def metric_values(selected: Sequence[Mapping[str, Any]], name: str) -> list[float]:
        return [value for row in selected if (value := _metric(row, name)) is not None]

    return {
        "attempted": len(records),
        "valid_moving": len(moving),
        "stop": sum(row.get("classification") == "MODEL_STOP_OUTPUT" for row in records),
        "timing_invalid": sum(row.get("classification") == "TIMING_INVALID" for row in records),
        "old_exhausted": sum(row.get("classification") == "OLD_EXHAUSTED" for row in records),
        "new_timeout": sum(row.get("classification") == "NEW_TIMEOUT" for row in records),
        "other_failures": sum(row.get("classification") == "OTHER_PROTOCOL_FAILURE" for row in records),
        "statistics": {
            name: descriptive_statistics(values)
            for name in CELL_METRICS
            if (values := metric_values(moving, name))
        },
        "raw_values": {
            name: values
            for name in CELL_METRICS
            if (values := metric_values(moving, name))
        },
        "stop_statistics": {
            name: descriptive_statistics(values)
            for name in CELL_METRICS
            if (values := metric_values(stop, name))
        },
    }


def aggregate_controlled(
    records: Sequence[Mapping[str, Any]], *, cell_ids: Sequence[str]
) -> dict[str, Any]:
    ids = list(cell_ids)
    if not ids or len(ids) != len(set(ids)):
        raise ValueError("cell_ids must be non-empty and unique")
    for row in records:
        if row.get("cell_id") not in ids:
            raise ValueError("record contains an undeclared cell")
    return {
        "cell_order": ids,
        "overall": _summarize(records),
        "per_cell": {
            cell: _summarize([row for row in records if row["cell_id"] == cell])
            for cell in ids
        },
    }


def select_representative_samples(
    records: Sequence[Mapping[str, Any]], cell_ids: Sequence[str]
) -> dict[str, str | None]:
    """Select nearest median normalized two-command score, tie by attempt/path."""

    result: dict[str, str | None] = {}
    for cell in cell_ids:
        rows = [
            row for row in records
            if row.get("cell_id") == cell and row.get("classification") == "VALID_MOVING"
        ]
        if not rows:
            result[cell] = None
            continue
        dv = np.asarray([row["controller_metrics"]["delta_v_abs_mps"] for row in rows])
        dw = np.asarray([row["controller_metrics"]["delta_omega_abs_rps"] for row in rows])
        dv_scale = max(float(np.median(dv)), 1e-12)
        dw_scale = max(float(np.median(dw)), 1e-12)
        scores = dv / dv_scale + dw / dw_scale
        median_score = float(np.median(scores))
        selected = min(
            zip(rows, scores, strict=True),
            key=lambda pair: (
                abs(float(pair[1]) - median_score),
                int(pair[0]["attempt_index"]),
                str(pair[0]["artifact_path"]),
            ),
        )[0]
        result[cell] = str(selected["artifact_path"])
    return result


def load_attempt_records(root: str | Path) -> list[dict[str, Any]]:
    directory = Path(root)
    records: list[dict[str, Any]] = []
    for path in sorted((directory / "primary").glob("*/*/attempt_*/results/attempt.json")):
        row = load_strict_json(path)
        if not isinstance(row, dict):
            raise ValueError(f"{path} must contain an object")
        records.append(row)
    return records


def validate_attempt_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    metadata = load_strict_json(root / "metadata.json")
    attempt = load_strict_json(root / "results/attempt.json")
    timing = load_strict_json(root / "results/timing.json")
    controller = load_strict_json(root / "results/controller_switch_metrics.json")
    geometry = load_strict_json(root / "results/geometry.json")
    old = np.load(root / "raw/old_actions.npy", allow_pickle=False)
    fresh = np.load(root / "raw/fresh_actions.npy", allow_pickle=False)
    old_hash = sha256_file(root / "raw/old_actions.npy")
    fresh_hash = sha256_file(root / "raw/fresh_actions.npy")
    if old.ndim != 2 or fresh.ndim != 2 or old.shape[1] != 3 or fresh.shape[1] != 3:
        raise ValueError("raw actions must have arbitrary non-empty (N,3) shape")
    if not np.all(np.isfinite(old)) or not np.all(np.isfinite(fresh)):
        raise ValueError("raw actions contain non-finite values")
    if metadata["raw_sha256"] != {"old_actions.npy": old_hash, "fresh_actions.npy": fresh_hash}:
        raise ValueError("raw action hash mismatch")
    if bool(metadata.get("intrinsic_waypoint_time_base")):
        raise ValueError("EXP-01B must not fabricate a waypoint time base")
    if attempt["timing"] != timing or attempt["controller_metrics"] != controller or attempt["geometry"] != geometry:
        raise ValueError("attempt summary does not reconstruct from result artifacts")
    with (root / "derived/controller_commands.csv").open(newline="", encoding="utf-8") as stream:
        command_rows = list(csv.DictReader(stream))
    old_commands = command_rows_to_array(command_rows, source="OLD")
    fresh_commands = command_rows_to_array(command_rows, source="FRESH")
    if not is_stop_actions(fresh):
        rebuilt_controller = controller_switch_metrics(
            old_commands,
            fresh_commands,
            control_dt_s=float(controller["control_dt_s"]),
            window_size=int(controller["window_size"]),
        )
        if rebuilt_controller != controller:
            raise ValueError("controller switch metrics do not reconstruct from command CSV")
    fresh_world = np.load(root / "derived/fresh_world.npy", allow_pickle=False)
    expected_world = lightnav_local_to_world(
        raw_actions_to_local_path(fresh),
        metadata["robot_pose_at_new_observation"],
    )
    if not np.array_equal(fresh_world, expected_world):
        raise ValueError("FRESH world path is not the immutable raw output anchored at observation")
    if is_stop_actions(fresh) != bool(attempt["stop_output"]):
        raise ValueError("STOP classification differs from raw FRESH")
    return {
        "valid_output": True,
        "classification": attempt["classification"],
        "old_shape": list(old.shape),
        "fresh_shape": list(fresh.shape),
        "controller_rows": len(command_rows),
    }


def validate_controlled_output(path: str | Path) -> dict[str, Any]:
    root = Path(path)
    protocol = load_strict_json(root / "protocol.json")
    summary = load_strict_json(root / "summary.json")
    cells = [item["cell_id"] for item in protocol["cells"]]
    records = load_attempt_records(root)
    rebuilt = aggregate_controlled(records, cell_ids=cells)
    if summary["aggregate"] != rebuilt:
        raise ValueError("summary aggregate does not reconstruct from attempts")
    validations = [validate_attempt_output(row["artifact_path"]) for row in records]
    return {
        "valid_output": True,
        "attempt_count": len(records),
        "valid_moving_count": sum(row["classification"] == "VALID_MOVING" for row in records),
        "cell_count": len(cells),
        "attempts_validated": len(validations),
    }
