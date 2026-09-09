"""Immutable DATA-01 LightNav transition-bank helpers.

The bank unit is one frozen EXP-01B transition context, including repeated
trajectory arrays.  LightNav waypoint rows remain untimed; all path descriptors
are spatial.  Boundary canonicalization is visualization-only.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

from reconciliation.exp01b_controlled_latency import (
    load_attempt_records,
    validate_attempt_output,
)
from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.lightnav_execution_envelope import (
    build_source_inventory,
    geometry_descriptor,
    validate_source_provenance,
)
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import relative_pose, wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


PAIR_INDEX_FIELDS = (
    "pair_id",
    "display_index",
    "source_trial",
    "geometry_group",
    "latency_group",
    "old_raw_hash_short",
    "fresh_raw_hash_short",
    "raw_pair_group",
    "old_world_hash_short",
    "fresh_world_hash_short",
    "world_pair_group",
    "effective_latency_s",
    "robot_travel_obs_to_switch_m",
    "raw_delta_v_abs_mps",
    "raw_delta_omega_abs_rps",
    "boundary_to_f0_distance_m",
    "incoming_vs_f0_direction_disagreement_rad",
    "yaw_disagreement_rad",
    "old_path_length_m",
    "fresh_path_length_m",
    "rank_abs_delta_v",
    "rank_abs_delta_omega",
    "rank_direction_disagreement",
    "rank_yaw_disagreement",
    "rank_effective_latency",
    "rank_robot_travel",
    "split",
)
PAIR_SOURCE_FILES = (
    "raw/old_actions.npy",
    "raw/fresh_actions.npy",
    "derived/old_world.npy",
    "derived/fresh_world.npy",
    "derived/controller_commands.csv",
    "derived/timeline.csv",
    "metadata.json",
    "raw/event_log.json",
    "results/attempt.json",
    "results/controller_switch_metrics.json",
    "results/geometry.json",
    "results/timing.json",
    "results/validation.json",
)
PAIR_COPIES = {
    "raw/old_lightnav.npy": "raw/old_actions.npy",
    "raw/fresh_lightnav.npy": "raw/fresh_actions.npy",
    "derived/old_world.npy": "derived/old_world.npy",
    "derived/fresh_world.npy": "derived/fresh_world.npy",
}
PAIR_PLOTS = (
    "plots/world_xy.png",
    "plots/boundary_canonical_xy.png",
    "plots/yaw_vs_path_length.png",
)
OVERVIEW_PLOTS = (
    "overview/all_pairs_world.png",
    "overview/all_pairs_boundary_canonical.png",
    "overview/development_pairs.png",
    "overview/heldout_pairs.png",
    "overview/by_geometry_group/G0_straight.png",
    "overview/by_geometry_group/G1_turn.png",
    "overview/by_geometry_group/G2_route_change.png",
    "overview/by_latency_group/L0_natural.png",
    "overview/by_latency_group/L1_added_050.png",
)
RANK_FIELDS = {
    "abs_delta_v": "historical_raw_switch.delta_v_raw_abs_mps",
    "abs_delta_omega": "historical_raw_switch.delta_omega_raw_abs_rps",
    "direction_disagreement": (
        "transition.incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"
    ),
    "yaw_disagreement": "transition.yaw_increment_disagreement_abs_rad",
    "effective_latency": "timing.effective_latency_sim_s",
    "robot_travel": "timing.robot_translation_observation_to_switch_m",
}


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    assert_finite_json_tree(value)
    return value


def canonical_json_sha256(value: Any) -> str:
    assert_finite_json_tree(value)
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def pair_identity(identity: Mapping[str, Any]) -> str:
    """Return an order-independent stable ID for one full transition context."""

    return f"pair_{canonical_json_sha256(dict(identity))[:12]}"


def duplicate_pair_identity(first_sha256: str, second_sha256: str, *, kind: str) -> str:
    for value in (first_sha256, second_sha256):
        if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
            raise ValueError("duplicate identity inputs must be lowercase SHA-256 values")
    if kind not in {"raw", "world"}:
        raise ValueError("duplicate identity kind must be raw or world")
    return f"{kind}_pair_{canonical_json_sha256([first_sha256, second_sha256])[:16]}"


def classify_attempt_for_bank(
    attempt: Mapping[str, Any], *, fresh_is_stop: bool
) -> tuple[bool, str]:
    classification = str(attempt.get("classification"))
    timing_valid = bool(attempt.get("timing_valid"))
    if classification == "VALID_MOVING" and timing_valid and not fresh_is_stop:
        return True, "ELIGIBLE_VALID_MOVING"
    if classification == "MODEL_STOP_OUTPUT":
        return False, "EXCLUDED_MODEL_STOP_OUTPUT"
    if classification == "OLD_EXHAUSTED":
        return False, "EXCLUDED_OLD_EXHAUSTED"
    if classification == "TIMING_INVALID" or not timing_valid:
        return False, "EXCLUDED_TIMING_INVALID"
    if fresh_is_stop:
        return False, "EXCLUDED_FRESH_ALL_ZERO"
    return False, f"EXCLUDED_{classification}"


def boundary_canonicalize(
    boundary: Sequence[float], *values: Sequence[float] | np.ndarray
) -> tuple[np.ndarray, ...]:
    """Express poses in B coordinates without modifying any input."""

    base = validate_pose_se2(boundary, name="boundary B")
    result: list[np.ndarray] = []
    for index, value in enumerate(values):
        array = np.asarray(value)
        if array.ndim == 1:
            pose = validate_pose_se2(array, name=f"canonical value {index}")
            result.append(np.asarray(relative_pose(base, pose), dtype=np.float64))
        else:
            path = validate_se2_trajectory(array, name=f"canonical value {index}")
            result.append(np.asarray(relative_pose(base, path), dtype=np.float64))
    return tuple(result)


def _direction(delta_xy: np.ndarray, epsilon_m: float) -> tuple[bool, float | None, float]:
    distance = float(np.linalg.norm(delta_xy))
    if distance < epsilon_m:
        return False, None, distance
    return True, float(math.atan2(float(delta_xy[1]), float(delta_xy[0]))), distance


def transition_descriptor(
    *,
    previous_pose: Sequence[float],
    boundary_pose: Sequence[float],
    old_world: np.ndarray,
    fresh_world: np.ndarray,
    direction_epsilon_m: float = 1e-3,
) -> dict[str, Any]:
    """Describe the frozen raw k=0 boundary without selecting a future k."""

    epsilon = float(direction_epsilon_m)
    if not math.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError("direction_epsilon_m must be finite and positive")
    previous = validate_pose_se2(previous_pose, name="previous pose P")
    boundary = validate_pose_se2(boundary_pose, name="boundary pose B")
    old = validate_se2_trajectory(old_world, name="OLD world")
    fresh = validate_se2_trajectory(fresh_world, name="FRESH world")
    incoming_defined, incoming_direction, incoming_distance = _direction(
        boundary[:2] - previous[:2], epsilon
    )
    entry_defined, entry_direction, entry_distance = _direction(
        fresh[0, :2] - boundary[:2], epsilon
    )
    direction_disagreement = (
        abs(float(wrap_angle(float(entry_direction) - float(incoming_direction))))
        if incoming_defined and entry_defined
        else None
    )
    incoming_yaw_increment = float(wrap_angle(boundary[2] - previous[2]))
    boundary_to_entry_yaw = float(wrap_angle(fresh[0, 2] - boundary[2]))
    result = {
        "scope": "descriptive frozen raw-switch context at k_fresh=0",
        "intrinsic_waypoint_time_base": False,
        "direction_epsilon_m": epsilon,
        "boundary_to_fresh0_distance_m": entry_distance,
        "boundary_to_fresh0_wrapped_yaw_rad": boundary_to_entry_yaw,
        "boundary_to_fresh0_world_direction_defined": entry_defined,
        "boundary_to_fresh0_world_direction_rad": entry_direction,
        "previous_to_boundary_distance_m": incoming_distance,
        "previous_to_boundary_world_direction_defined": incoming_defined,
        "previous_to_boundary_world_direction_rad": incoming_direction,
        "incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad": (
            direction_disagreement
        ),
        "previous_to_boundary_yaw_increment_rad": incoming_yaw_increment,
        "yaw_increment_disagreement_abs_rad": abs(
            float(wrap_angle(boundary_to_entry_yaw - incoming_yaw_increment))
        ),
        "fresh_endpoint_relative_to_boundary_se2": np.asarray(
            relative_pose(boundary, fresh[-1]), dtype=np.float64
        ).tolist(),
        "old_endpoint_relative_to_boundary_se2": np.asarray(
            relative_pose(boundary, old[-1]), dtype=np.float64
        ).tolist(),
    }
    assert_finite_json_tree(result)
    return result


def _nested(mapping: Mapping[str, Any], path: str) -> Any:
    value: Any = mapping
    for part in path.split("."):
        if not isinstance(value, Mapping):
            raise KeyError(path)
        value = value[part]
    return value


def add_deterministic_ranks(
    rows: Sequence[Mapping[str, Any]], field_paths: Mapping[str, str] = RANK_FIELDS
) -> list[dict[str, Any]]:
    """Add unique, deterministic one-based ranks and descriptive quartiles."""

    ranked = [dict(row) for row in rows]
    count = len(ranked)
    if count < 1:
        raise ValueError("cannot rank an empty bank")
    for label, path in field_paths.items():
        values: list[tuple[bool, float, str, int]] = []
        for index, row in enumerate(ranked):
            raw_value = _nested(row, path)
            if raw_value is None:
                values.append((True, math.inf, str(row["pair_id"]), index))
            else:
                value = float(raw_value)
                if not math.isfinite(value):
                    raise ValueError(f"rank value {label} must be finite or null")
                values.append((False, value, str(row["pair_id"]), index))
        for rank, (_, _, _, index) in enumerate(sorted(values), start=1):
            ranked[index][f"rank_{label}"] = rank
            ranked[index][f"quantile_{label}"] = f"Q{min(4, 1 + (rank - 1) * 4 // count)}"
    return ranked


def _split_score(
    *,
    held_groups: frozenset[str],
    group_rows: Mapping[str, Sequence[Mapping[str, Any]]],
    all_strata: Sequence[tuple[str, str]],
    heldout_fraction: float,
    target_heldout: int,
    seed: str,
) -> tuple[int, int, float, str]:
    held = [row for group in held_groups for row in group_rows[group]]
    development = [
        row for group, values in group_rows.items() if group not in held_groups for row in values
    ]
    held_counts = Counter((str(r["geometry_group"]), str(r["latency_group"])) for r in held)
    dev_counts = Counter(
        (str(r["geometry_group"]), str(r["latency_group"])) for r in development
    )
    missing = sum(
        held_counts[stratum] == 0 or dev_counts[stratum] == 0 for stratum in all_strata
    )
    total_counts = Counter(
        (str(r["geometry_group"]), str(r["latency_group"]))
        for values in group_rows.values()
        for r in values
    )
    stratum_error = sum(
        ((held_counts[key] - total_counts[key] * heldout_fraction) ** 2)
        / max(1, total_counts[key])
        for key in all_strata
    )
    tie = hashlib.sha256(
        (seed + "|" + "|".join(sorted(held_groups))).encode("utf-8")
    ).hexdigest()
    return missing, abs(len(held) - target_heldout), float(stratum_error), tie


def deterministic_grouped_split(
    rows: Sequence[Mapping[str, Any]],
    *,
    group_field: str = "duplicate_raw_pair_group",
    heldout_fraction: float = 1.0 / 3.0,
    seed: str = "DATA-01-lightnav-exp01b-v1",
) -> dict[str, Any]:
    """Split complete leakage groups while balancing source G/L strata.

    For this bank's 15 raw-pair groups, every non-trivial assignment is searched.
    Larger inputs use a deterministic greedy fallback to keep the helper bounded.
    No graph or reconciliation result is accepted as an input.
    """

    if not 0.0 < float(heldout_fraction) < 1.0:
        raise ValueError("heldout_fraction must lie strictly between zero and one")
    if not rows:
        raise ValueError("cannot split an empty bank")
    group_rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    ids: set[str] = set()
    for source in rows:
        row = dict(source)
        pair_id = str(row["pair_id"])
        if pair_id in ids:
            raise ValueError("pair IDs must be unique")
        ids.add(pair_id)
        group_rows[str(row[group_field])].append(row)
    groups = sorted(group_rows)
    if len(groups) < 2:
        raise ValueError("at least two leakage groups are required")
    all_strata = sorted(
        {
            (str(row["geometry_group"]), str(row["latency_group"]))
            for row in rows
        }
    )
    target = int(round(len(rows) * float(heldout_fraction)))
    candidates: Iterable[frozenset[str]]
    if len(groups) <= 22:
        candidates = (
            frozenset(groups[index] for index in range(len(groups)) if mask & (1 << index))
            for mask in range(1, (1 << len(groups)) - 1)
        )
    else:
        ordered = sorted(
            groups,
            key=lambda group: hashlib.sha256((seed + group).encode()).hexdigest(),
        )
        selected: set[str] = set()
        current = 0
        for group in ordered:
            size = len(group_rows[group])
            if abs(current + size - target) <= abs(current - target) or not selected:
                selected.add(group)
                current += size
        candidates = (frozenset(selected),)
    held_groups = min(
        candidates,
        key=lambda candidate: _split_score(
            held_groups=candidate,
            group_rows=group_rows,
            all_strata=all_strata,
            heldout_fraction=float(heldout_fraction),
            target_heldout=target,
            seed=seed,
        ),
    )
    held_ids = sorted(
        str(row["pair_id"])
        for group in held_groups
        for row in group_rows[group]
    )
    development_ids = sorted(ids.difference(held_ids))
    assignment = {
        pair_id: ("heldout" if pair_id in set(held_ids) else "development")
        for pair_id in sorted(ids)
    }
    strata: dict[str, dict[str, int]] = {}
    for geometry, latency in all_strata:
        key = f"{geometry}__{latency}"
        selected = [
            row
            for row in rows
            if str(row["geometry_group"]) == geometry
            and str(row["latency_group"]) == latency
        ]
        strata[key] = {
            "total": len(selected),
            "development": sum(assignment[str(row["pair_id"])] == "development" for row in selected),
            "heldout": sum(assignment[str(row["pair_id"])] == "heldout" for row in selected),
        }
    dev_groups = {
        str(row[group_field]) for row in rows if assignment[str(row["pair_id"])] == "development"
    }
    held_group_set = set(held_groups)
    return {
        "version": "lightnav_exp01b_v1",
        "source_bank": "DATA-01 frozen EXP-01B transition contexts",
        "split_rule": (
            "exhaustive deterministic grouped assignment minimizing missing G/L coverage, "
            "then pair-count error, then normalized stratum error; stable SHA-256 tie-break"
            if len(groups) <= 22
            else "deterministic stable-hash greedy grouped assignment"
        ),
        "grouping_rule": "exact ordered (OLD raw SHA-256, FRESH raw SHA-256) pair",
        "graph_or_reconciliation_outcomes_used": False,
        "heldout_fraction_target": float(heldout_fraction),
        "seed": seed,
        "development_pair_ids": development_ids,
        "heldout_pair_ids": held_ids,
        "assignment": assignment,
        "counts": {
            "total": len(ids),
            "development": len(development_ids),
            "heldout": len(held_ids),
            "raw_pair_groups": len(groups),
        },
        "stratum_counts": strata,
        "leakage_check": {
            "passed": dev_groups.isdisjoint(held_group_set),
            "development_raw_pair_groups": sorted(dev_groups),
            "heldout_raw_pair_groups": sorted(held_group_set),
            "overlap": sorted(dev_groups.intersection(held_group_set)),
        },
    }


def descriptive_statistics(values: Sequence[float]) -> dict[str, float | int]:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or array.size < 1 or not np.all(np.isfinite(array)):
        raise ValueError("statistics require finite non-empty one-dimensional values")
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "q1": float(np.percentile(array, 25)),
        "median": float(np.median(array)),
        "q3": float(np.percentile(array, 75)),
        "max": float(np.max(array)),
        "mean": float(np.mean(array)),
    }


def select_visualization_representatives(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not rows:
        raise ValueError("cannot select representatives from an empty bank")
    dv_values = np.asarray(
        [float(_nested(row, RANK_FIELDS["abs_delta_v"])) for row in rows], dtype=float
    )
    dw_values = np.asarray(
        [float(_nested(row, RANK_FIELDS["abs_delta_omega"])) for row in rows], dtype=float
    )
    dv_scale = max(float(np.median(dv_values)), 1e-12)
    dw_scale = max(float(np.median(dw_values)), 1e-12)
    scored = sorted(
        (
            float(dv / dv_scale + dw / dw_scale),
            str(row["pair_id"]),
            row,
        )
        for row, dv, dw in zip(rows, dv_values, dw_values, strict=True)
    )
    median_index = (len(scored) - 1) // 2

    def maximum(path: str) -> str:
        eligible = [row for row in rows if _nested(row, path) is not None]
        return str(max(eligible, key=lambda row: (float(_nested(row, path)), str(row["pair_id"])))["pair_id"])

    roles = {
        "minimum_combined_raw_discontinuity": scored[0][1],
        "median_combined_raw_discontinuity": scored[median_index][1],
        "highest_abs_delta_v": maximum(RANK_FIELDS["abs_delta_v"]),
        "highest_abs_delta_omega": maximum(RANK_FIELDS["abs_delta_omega"]),
        "largest_direction_disagreement": maximum(RANK_FIELDS["direction_disagreement"]),
        "largest_yaw_disagreement": maximum(RANK_FIELDS["yaw_disagreement"]),
    }
    return {
        "purpose": "VISUALIZATION_REPRESENTATIVES; not a research evaluation subset",
        "selection_uses_graph_or_reconciliation_results": False,
        "selection_rule": (
            "raw frozen metrics only; combined score uses dataset-median |delta v| and "
            "|delta omega| scales; lower-middle median; pair-id tie-break"
        ),
        "combined_score_scales": {
            "median_abs_delta_v_mps": dv_scale,
            "median_abs_delta_omega_rps": dw_scale,
        },
        "roles": roles,
        "unique_pair_ids": sorted(set(roles.values())),
    }


def _write_json_exclusive(path: Path, value: Mapping[str, Any]) -> None:
    assert_finite_json_tree(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _copy_file_exclusive(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with source.open("rb") as input_stream, os.fdopen(descriptor, "wb") as output_stream:
            descriptor = -1
            shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _write_csv_exclusive(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError("cannot write empty dataset index")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(PAIR_INDEX_FIELDS))
        writer.writeheader()
        writer.writerows([{key: row[key] for key in PAIR_INDEX_FIELDS} for row in rows])


def _find_event(events: Sequence[Mapping[str, Any]], name: str) -> dict[str, Any]:
    matches = [dict(row) for row in events if row.get("event") == name]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {name} event")
    return matches[0]


def _find_timeline_pose_timestamp(path: Path, pose: Sequence[float]) -> dict[str, Any]:
    expected = validate_pose_se2(pose)
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    matches = []
    for row in rows:
        actual = np.asarray(
            [float(row["actual_x"]), float(row["actual_y"]), float(row["actual_yaw"])],
            dtype=float,
        )
        if np.array_equal(actual, expected):
            matches.append(row)
    if len(matches) != 1:
        raise ValueError("P/B pose must match exactly one frozen timeline sample")
    return {
        "sim_time_s": float(matches[0]["sim_time_s"]),
        "host_monotonic_ns": int(matches[0]["host_monotonic_ns"]),
        "timeline_phase": matches[0]["phase"],
    }


def _required_source_snapshot(source_root: Path) -> dict[str, dict[str, int | str]]:
    rows: dict[str, dict[str, int | str]] = {}
    for trial in sorted(source_root.glob("primary/*/*/attempt_*")):
        for relative in PAIR_SOURCE_FILES:
            path = trial / relative
            if not path.is_file():
                raise ValueError(f"source trial missing {relative}: {trial}")
            key = path.relative_to(source_root).as_posix()
            rows[key] = {
                "sha256": sha256_file(path),
                "mtime_ns": path.stat().st_mtime_ns,
                "size_bytes": path.stat().st_size,
            }
    return rows


def _git_head(repository_root: Path) -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repository_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def create_bank_root(path: str | Path) -> Path:
    """Create a new bank root and reject every existing target."""

    root = Path(path)
    root.mkdir(parents=True, exist_ok=False)
    return root


def _source_row(
    *,
    source_root: Path,
    trial: Path,
    display_index: int,
    geometry_config: Mapping[str, Any],
    shared_source_provenance: Mapping[str, Any],
) -> dict[str, Any]:
    validate_attempt_output(trial)
    attempt = load_strict_json(trial / "results/attempt.json")
    metadata = load_strict_json(trial / "metadata.json")
    timing = load_strict_json(trial / "results/timing.json")
    controller = load_strict_json(trial / "results/controller_switch_metrics.json")
    source_geometry = load_strict_json(trial / "results/geometry.json")
    events = load_strict_json(trial / "raw/event_log.json")
    if not isinstance(events, list):
        raise ValueError("event_log.json must contain a list")
    hashes = {name: sha256_file(trial / name) for name in PAIR_SOURCE_FILES}
    old_raw = np.load(trial / "raw/old_actions.npy", allow_pickle=False)
    fresh_raw = np.load(trial / "raw/fresh_actions.npy", allow_pickle=False)
    old_world = validate_se2_trajectory(
        np.load(trial / "derived/old_world.npy", allow_pickle=False), name="OLD world"
    )
    fresh_world = validate_se2_trajectory(
        np.load(trial / "derived/fresh_world.npy", allow_pickle=False), name="FRESH world"
    )
    if not np.all(np.isfinite(old_raw)) or not np.all(np.isfinite(fresh_raw)):
        raise ValueError("raw actions contain non-finite values")
    if attempt["classification"] != "VALID_MOVING" or not attempt["timing_valid"]:
        raise ValueError("only timing-valid VALID_MOVING trials enter pair construction")
    relative = trial.relative_to(source_root).as_posix()
    previous = validate_pose_se2(attempt["actual_pose_before_ready"], name="P")
    boundary = validate_pose_se2(attempt["robot_pose_at_new_ready"], name="B")
    if not np.array_equal(boundary, validate_pose_se2(metadata["robot_pose_at_fresh_usable"])):
        raise ValueError("B differs between source ready and usable fields")
    raw_switch_event = _find_event(events, "raw_switch")
    if not np.array_equal(boundary, validate_pose_se2(raw_switch_event["robot_pose_world"])):
        raise ValueError("B differs from raw_switch event pose")
    old_observation_event = _find_event(events, "old_observation")
    fresh_observation_event = _find_event(events, "fresh_observation")
    model_ready_event = _find_event(events, "fresh_model_ready")
    usable_event = _find_event(events, "fresh_usable")
    p_timestamp = _find_timeline_pose_timestamp(trial / "derived/timeline.csv", previous)
    b_timestamp = _find_timeline_pose_timestamp(trial / "derived/timeline.csv", boundary)
    raw_group = duplicate_pair_identity(
        hashes["raw/old_actions.npy"], hashes["raw/fresh_actions.npy"], kind="raw"
    )
    world_group = duplicate_pair_identity(
        hashes["derived/old_world.npy"], hashes["derived/fresh_world.npy"], kind="world"
    )
    identity = {
        "source_trial_relative_path": relative,
        "old_raw_sha256": hashes["raw/old_actions.npy"],
        "fresh_raw_sha256": hashes["raw/fresh_actions.npy"],
        "old_world_sha256": hashes["derived/old_world.npy"],
        "fresh_world_sha256": hashes["derived/fresh_world.npy"],
        "previous_pose": previous.tolist(),
        "boundary_pose": boundary.tolist(),
        "fresh_observation_pose": metadata["robot_pose_at_fresh_observation"],
        "observation_sim_time_s": timing["observation_sim_time_s"],
        "fresh_usable_sim_time_s": timing["fresh_usable_sim_time_s"],
    }
    pair_id_value = pair_identity(identity)
    transition = transition_descriptor(
        previous_pose=previous,
        boundary_pose=boundary,
        old_world=old_world,
        fresh_world=fresh_world,
        direction_epsilon_m=float(geometry_config["direction_epsilon_m"]),
    )
    historical = {
        "source": "frozen EXP-01B results/controller_switch_metrics.json; copied, not recomputed",
        "source_relative_path": f"{relative}/results/controller_switch_metrics.json",
        "source_sha256": hashes["results/controller_switch_metrics.json"],
        "delta_v_raw_signed_mps": controller["delta_v_signed_mps"],
        "delta_v_raw_abs_mps": controller["delta_v_abs_mps"],
        "delta_omega_raw_signed_rps": controller["delta_omega_signed_rps"],
        "delta_omega_raw_abs_rps": controller["delta_omega_abs_rps"],
        "post_switch_max_abs_delta_v_mps": controller["post_switch_max_abs_delta_v_mps"],
        "post_switch_mean_abs_delta_v_mps": controller["post_switch_mean_abs_delta_v_mps"],
        "post_switch_max_abs_delta_omega_rps": controller["post_switch_max_abs_delta_omega_rps"],
        "post_switch_mean_abs_delta_omega_rps": controller["post_switch_mean_abs_delta_omega_rps"],
        "last_old_desired_command_v_omega": controller["old_last_command_v_omega"],
        "first_fresh_desired_command_v_omega": controller["fresh_first_command_v_omega"],
        "window_size": controller["window_size"],
        "control_dt_s": controller["control_dt_s"],
    }
    descriptors = {
        "pair_id": pair_id_value,
        "descriptor_semantics": "untimed spatial descriptors; no waypoint time alignment",
        "old": geometry_descriptor(
            old_world,
            segment_epsilon_m=float(geometry_config["segment_epsilon_m"]),
            curvature_zero_epsilon_per_m=float(
                geometry_config["curvature_zero_epsilon_per_m"]
            ),
        ),
        "fresh": geometry_descriptor(
            fresh_world,
            segment_epsilon_m=float(geometry_config["segment_epsilon_m"]),
            curvature_zero_epsilon_per_m=float(
                geometry_config["curvature_zero_epsilon_per_m"]
            ),
        ),
        "transition": transition,
        "historical_raw_switch": historical,
        "source_geometry_diagnostic": {
            "source_relative_path": f"{relative}/results/geometry.json",
            "source_sha256": hashes["results/geometry.json"],
            "values": source_geometry,
        },
        "timing": {
            "effective_latency_sim_s": timing["effective_latency_sim_s"],
            "effective_latency_wall_s": timing["effective_latency_wall_s"],
            "robot_translation_observation_to_switch_m": timing[
                "robot_translation_observation_to_switch_m"
            ],
            "robot_yaw_observation_to_switch_rad": timing[
                "robot_yaw_observation_to_switch_rad"
            ],
        },
    }
    context = {
        "pair_id": pair_id_value,
        "display_index": f"{display_index:03d}",
        "source_trial_relative_path": relative,
        "source_status": attempt["classification"],
        "geometry_group": attempt["condition_id"],
        "geometry_class": attempt["geometry_class"],
        "latency_group": attempt["latency_condition_id"],
        "duplicate_raw_pair_group": raw_group,
        "duplicate_world_pair_group": world_group,
        "identity_payload": identity,
        "old": {
            "raw_shape": list(old_raw.shape),
            "raw_dtype": str(old_raw.dtype),
            "world_shape": list(old_world.shape),
            "observation_pose_world_se2": metadata["robot_pose_at_old_observation"],
            "observation_sim_time_s": old_observation_event["sim_time_s"],
            "observation_host_monotonic_ns": old_observation_event["host_monotonic_ns"],
            "raw_sha256": hashes["raw/old_actions.npy"],
            "world_sha256": hashes["derived/old_world.npy"],
        },
        "fresh": {
            "raw_shape": list(fresh_raw.shape),
            "raw_dtype": str(fresh_raw.dtype),
            "world_shape": list(fresh_world.shape),
            "observation_pose_world_se2": metadata["robot_pose_at_fresh_observation"],
            "observation_sim_time_s": fresh_observation_event["sim_time_s"],
            "observation_host_monotonic_ns": fresh_observation_event["host_monotonic_ns"],
            "model_ready_pose_world_se2": metadata["robot_pose_at_fresh_model_ready"],
            "model_ready_sim_time_s": model_ready_event["sim_time_s"],
            "model_ready_host_monotonic_ns": model_ready_event["host_monotonic_ns"],
            "usable_raw_switch_pose_world_se2": metadata["robot_pose_at_fresh_usable"],
            "usable_raw_switch_sim_time_s": usable_event["sim_time_s"],
            "usable_raw_switch_host_monotonic_ns": usable_event["host_monotonic_ns"],
            "raw_sha256": hashes["raw/fresh_actions.npy"],
            "world_sha256": hashes["derived/fresh_world.npy"],
        },
        "execution_context": {
            "previous_pose_P_world_se2": previous.tolist(),
            "boundary_pose_B_world_se2": boundary.tolist(),
            "previous_pose_source_field": "results/attempt.json.actual_pose_before_ready",
            "boundary_pose_source_fields": [
                "results/attempt.json.robot_pose_at_new_ready",
                "metadata.json.robot_pose_at_fresh_usable",
                "raw/event_log.json[event=raw_switch].robot_pose_world",
            ],
            "previous_pose_timestamp": p_timestamp,
            "boundary_pose_timestamp": b_timestamp,
            "previous_pose_semantics": (
                "actual Jackal pose at the final frozen control sample immediately before "
                "the raw-switch boundary"
            ),
            "boundary_semantics": (
                "actual Jackal pose when FRESH became usable and the frozen raw switch occurred"
            ),
            "last_old_desired_command_v_omega": controller["old_last_command_v_omega"],
            "first_fresh_desired_command_v_omega": controller[
                "fresh_first_command_v_omega"
            ],
        },
        "timing": timing,
        "frame": {
            "raw_lightnav_frame": "robot frame at the respective OLD/FRESH observation",
            "raw_columns": ["forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive"],
            "decoded_action_semantics": metadata["decoded_action_semantics"],
            "world_frame": "Isaac Sim world [x,y,yaw]",
            "world_axis_convention": "x/y metres; yaw radians CCW about +Z",
            "units": {"translation": "m", "yaw": "rad", "time": "s"},
            "intrinsic_waypoint_time_base": False,
            "boundary_canonical_frame": "visualization-only B-centered SE(2) frame; not persisted as data",
        },
        "source_metadata": dict(shared_source_provenance),
    }
    provenance = {
        "pair_id": pair_id_value,
        "source_trial_relative_path": relative,
        "source_artifacts_sha256": hashes,
        "copied_artifacts": {
            destination: {
                "source_relative_path": f"{relative}/{source}",
                "source_sha256": hashes[source],
            }
            for destination, source in PAIR_COPIES.items()
        },
        "raw_files_modified": False,
        "derived_files_modified": False,
        "source_cohort": dict(shared_source_provenance),
    }
    return {
        "pair_id": pair_id_value,
        "display_index": f"{display_index:03d}",
        "source_trial": relative,
        "geometry_group": attempt["condition_id"],
        "latency_group": attempt["latency_condition_id"],
        "duplicate_raw_pair_group": raw_group,
        "duplicate_world_pair_group": world_group,
        "context": context,
        "descriptors": descriptors,
        "provenance": provenance,
        "source_trial_path": trial,
        "source_hashes": hashes,
    }


def _rank_pair_rows(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    working = [
        {
            **dict(row),
            "historical_raw_switch": row["descriptors"]["historical_raw_switch"],
            "transition": row["descriptors"]["transition"],
            "timing": row["descriptors"]["timing"],
        }
        for row in rows
    ]
    return add_deterministic_ranks(working)


def _summary(
    *, inventory: Mapping[str, Any], rows: Sequence[Mapping[str, Any]], split: Mapping[str, Any]
) -> dict[str, Any]:
    classes = Counter(row["classification"] for row in inventory["attempts"])
    values = {
        "raw_delta_v_abs_mps": [float(row["historical_raw_switch"]["delta_v_raw_abs_mps"]) for row in rows],
        "raw_delta_omega_abs_rps": [float(row["historical_raw_switch"]["delta_omega_raw_abs_rps"]) for row in rows],
        "direction_disagreement_rad": [
            float(row["transition"]["incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"])
            for row in rows
            if row["transition"]["incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"] is not None
        ],
        "yaw_disagreement_rad": [float(row["transition"]["yaw_increment_disagreement_abs_rad"]) for row in rows],
        "effective_latency_sim_s": [float(row["timing"]["effective_latency_sim_s"]) for row in rows],
    }
    geometry_split = {
        group: {
            name: sum(row["geometry_group"] == group and row["split"] == name for row in rows)
            for name in ("development", "heldout")
        }
        for group in sorted({str(row["geometry_group"]) for row in rows})
    }
    latency_split = {
        group: {
            name: sum(row["latency_group"] == group and row["split"] == name for row in rows)
            for name in ("development", "heldout")
        }
        for group in sorted({str(row["latency_group"]) for row in rows})
    }
    return {
        "total_retained_attempts": len(inventory["attempts"]),
        "timing_valid_attempts": sum(
            bool(row["timing_valid"]) for row in inventory["attempts"]
        ),
        "eligible_moving_transitions": sum(row["eligible_moving"] for row in inventory["attempts"]),
        "bank_pair_count": len(rows),
        "classification_counts": dict(sorted(classes.items())),
        "unique_counts": {
            "old_raw": len({row["context"]["old"]["raw_sha256"] for row in rows}),
            "fresh_raw": len({row["context"]["fresh"]["raw_sha256"] for row in rows}),
            "raw_pairs": len({row["duplicate_raw_pair_group"] for row in rows}),
            "old_world": len({row["context"]["old"]["world_sha256"] for row in rows}),
            "fresh_world": len({row["context"]["fresh"]["world_sha256"] for row in rows}),
            "world_pairs": len({row["duplicate_world_pair_group"] for row in rows}),
        },
        "split_counts": split["counts"],
        "geometry_split_counts": geometry_split,
        "latency_split_counts": latency_split,
        "distributions": {key: descriptive_statistics(item) for key, item in values.items()},
        "intrinsic_waypoint_time_base": False,
        "optimization_executed": False,
        "isaac_physics_executed": False,
    }


def build_bank(
    *,
    repository_root: str | Path,
    config_path: str | Path,
    output_root_override: str | Path | None = None,
) -> Path:
    repository = Path(repository_root).resolve()
    config_file = Path(config_path).resolve()
    config = load_yaml(config_file)
    if config.get("dataset") != "DATA-01-frozen-lightnav-transition-bank":
        raise ValueError("invalid DATA-01 config")
    if config["policy"].get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("DATA-01 must not fabricate waypoint timestamps")
    forbidden_true = ("new_lightnav_inference", "optimization", "isaac_physics")
    if any(config["policy"].get(key) is not False for key in forbidden_true):
        raise ValueError("DATA-01 config enables an out-of-scope operation")
    resolve = lambda value: (
        Path(value).expanduser().resolve()
        if Path(value).expanduser().is_absolute()
        else (repository / Path(value).expanduser()).resolve()
    )
    source_root = resolve(config["paths"]["source_cohort"])
    source_config = resolve(config["paths"]["source_config"])
    if sha256_file(source_config) != config["source_provenance"]["source_config_sha256"]:
        raise ValueError("frozen source config hash changed")
    validate_source_provenance(source_root, config["source_provenance"])
    before = _required_source_snapshot(source_root)
    inventory = build_source_inventory(source_root, config["source_provenance"])
    output_root = (
        resolve(output_root_override)
        if output_root_override is not None
        else resolve(config["paths"]["output_root"])
    )
    bank_root = output_root / str(config["version"])
    create_bank_root(bank_root)
    try:
        _copy_file_exclusive(config_file, bank_root / "config_snapshot.yaml")
        eligible = sorted(
            (row for row in inventory["attempts"] if row["eligible_moving"]),
            key=lambda row: row["relative_trial_path"],
        )
        source_metadata = load_strict_json(source_root / "metadata.json")
        shared_source_provenance = {
            "experiment_id": source_metadata["experiment_id"],
            "source_cohort_path": str(source_root),
            "research_git_commit_sha_at_exp01b_run": source_metadata[
                "research_git_commit_sha_at_run"
            ],
            "lightnav_commit": source_metadata["lightnav_server_end"]["lightnav_sha"],
            "lightnav_checkpoint_identifier": source_metadata["lightnav_server_end"][
                "checkpoint_identifier"
            ],
            "lightnav_checkpoint_revision": source_metadata["lightnav_server_end"][
                "checkpoint_revision"
            ],
            "source_config_path": str(source_config),
            "source_config_sha256": sha256_file(source_config),
        }
        rows = [
            _source_row(
                source_root=source_root,
                trial=source_root / source["relative_trial_path"],
                display_index=index,
                geometry_config=config["geometry"],
                shared_source_provenance=shared_source_provenance,
            )
            for index, source in enumerate(eligible)
        ]
        ids = [str(row["pair_id"]) for row in rows]
        if len(ids) != len(set(ids)):
            raise ValueError("stable pair IDs collided")
        ranked = _rank_pair_rows(rows)
        split = deterministic_grouped_split(
            ranked,
            heldout_fraction=float(config["split"]["heldout_fraction"]),
            seed=str(config["split"]["seed"]),
        )
        for row in ranked:
            row["split"] = split["assignment"][row["pair_id"]]
            row["descriptors"]["difficulty_ranks"] = {
                key: row[key]
                for key in row
                if key.startswith("rank_") or key.startswith("quantile_")
            }
            row["context"]["split"] = row["split"]
            pair_root = bank_root / "pairs" / row["pair_id"]
            trial = Path(row["source_trial_path"])
            for destination, source in PAIR_COPIES.items():
                _copy_file_exclusive(trial / source, pair_root / destination)
                if sha256_file(pair_root / destination) != row["source_hashes"][source]:
                    raise ValueError("byte-identical pair copy validation failed")
            _write_json_exclusive(pair_root / "context.json", row["context"])
            _write_json_exclusive(pair_root / "descriptors.json", row["descriptors"])
            _write_json_exclusive(pair_root / "source_provenance.json", row["provenance"])
        index_rows = []
        for row in ranked:
            index_rows.append(
                {
                    "pair_id": row["pair_id"],
                    "display_index": row["display_index"],
                    "source_trial": row["source_trial"],
                    "geometry_group": row["geometry_group"],
                    "latency_group": row["latency_group"],
                    "old_raw_hash_short": row["context"]["old"]["raw_sha256"][:12],
                    "fresh_raw_hash_short": row["context"]["fresh"]["raw_sha256"][:12],
                    "raw_pair_group": row["duplicate_raw_pair_group"],
                    "old_world_hash_short": row["context"]["old"]["world_sha256"][:12],
                    "fresh_world_hash_short": row["context"]["fresh"]["world_sha256"][:12],
                    "world_pair_group": row["duplicate_world_pair_group"],
                    "effective_latency_s": row["timing"]["effective_latency_sim_s"],
                    "robot_travel_obs_to_switch_m": row["timing"]["robot_translation_observation_to_switch_m"],
                    "raw_delta_v_abs_mps": row["historical_raw_switch"]["delta_v_raw_abs_mps"],
                    "raw_delta_omega_abs_rps": row["historical_raw_switch"]["delta_omega_raw_abs_rps"],
                    "boundary_to_f0_distance_m": row["transition"]["boundary_to_fresh0_distance_m"],
                    "incoming_vs_f0_direction_disagreement_rad": row["transition"]["incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"],
                    "yaw_disagreement_rad": row["transition"]["yaw_increment_disagreement_abs_rad"],
                    "old_path_length_m": row["descriptors"]["old"]["path_length_m"],
                    "fresh_path_length_m": row["descriptors"]["fresh"]["path_length_m"],
                    "rank_abs_delta_v": row["rank_abs_delta_v"],
                    "rank_abs_delta_omega": row["rank_abs_delta_omega"],
                    "rank_direction_disagreement": row["rank_direction_disagreement"],
                    "rank_yaw_disagreement": row["rank_yaw_disagreement"],
                    "rank_effective_latency": row["rank_effective_latency"],
                    "rank_robot_travel": row["rank_robot_travel"],
                    "split": row["split"],
                }
            )
        _write_csv_exclusive(bank_root / "dataset_index.csv", index_rows)
        source_artifact_manifest = canonical_json_sha256(before)
        inventory_output = {
            **inventory,
            "source_required_artifacts": before,
            "source_required_artifacts_manifest_sha256": source_artifact_manifest,
            "bank_inclusion_rule": "timing-valid VALID_MOVING and non-STOP FRESH",
            "bank_exclusion_counts": dict(
                sorted(
                    Counter(
                        classify_attempt_for_bank(
                            load_strict_json(
                                source_root
                                / row["relative_trial_path"]
                                / "results/attempt.json"
                            ),
                            fresh_is_stop=bool(row["fresh_all_zero_output"]),
                        )[1]
                        for row in inventory["attempts"]
                        if not row["eligible_moving"]
                    ).items()
                )
            ),
        }
        _write_json_exclusive(bank_root / "source_inventory.json", inventory_output)
        _write_json_exclusive(bank_root / "split_manifest.json", split)
        representatives = select_visualization_representatives(ranked)
        _write_json_exclusive(bank_root / "representatives.json", representatives)
        summary = _summary(inventory=inventory, rows=ranked, split=split)
        _write_json_exclusive(bank_root / "summary.json", summary)
        manifest = {
            "dataset": config["dataset"],
            "version": config["version"],
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "data01_generator_git_commit": _git_head(repository),
            "generator_source_files_sha256": {
                "src/reconciliation/frozen_transition_bank.py": sha256_file(
                    repository / "src/reconciliation/frozen_transition_bank.py"
                ),
                "scripts/build_frozen_transition_bank.py": sha256_file(
                    repository / "scripts/build_frozen_transition_bank.py"
                ),
                "scripts/plot_frozen_transition_bank.py": sha256_file(
                    repository / "scripts/plot_frozen_transition_bank.py"
                ),
                "scripts/validate_frozen_transition_bank.py": sha256_file(
                    repository / "scripts/validate_frozen_transition_bank.py"
                ),
            },
            "source_exp01b_path": str(source_root),
            "source_frozen_root_hashes": inventory["source_provenance"],
            "source_required_artifacts_manifest_sha256": source_artifact_manifest,
            "research_repo_source_commit_recorded_in_exp01b": source_metadata[
                "research_git_commit_sha_at_run"
            ],
            "lightnav_commit": source_metadata["lightnav_server_end"]["lightnav_sha"],
            "lightnav_checkpoint_identifier": source_metadata["lightnav_server_end"][
                "checkpoint_identifier"
            ],
            "lightnav_checkpoint_revision": source_metadata["lightnav_server_end"][
                "checkpoint_revision"
            ],
            "source_config_path": str(source_config),
            "source_config_sha256": sha256_file(source_config),
            "dataset_config_path": str(config_file),
            "dataset_config_sha256": sha256_file(config_file),
            "pair_count": len(ranked),
            "raw_files_modified": False,
            "derived_source_files_modified": False,
            "new_lightnav_inference": False,
            "optimization_executed": False,
            "isaac_physics_executed": False,
            "plots_generated_by_separate_immutable_step": True,
        }
        _write_json_exclusive(bank_root / "dataset_manifest.json", manifest)
        after = _required_source_snapshot(source_root)
        if before != after:
            raise RuntimeError("frozen source content or mtime changed during bank generation")
    except Exception:
        # The root is intentionally retained as evidence of a failed immutable attempt.
        raise
    return bank_root


def read_index(bank_root: str | Path) -> list[dict[str, str]]:
    with (Path(bank_root) / "dataset_index.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or tuple(rows[0]) != PAIR_INDEX_FIELDS:
        raise ValueError("dataset index schema mismatch or empty index")
    return rows


def _validate_png(path: Path) -> None:
    content = path.read_bytes()
    if len(content) < 1024 or not content.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"empty or invalid PNG: {path}")


def validate_bank(bank_root: str | Path, *, require_plots: bool = True) -> dict[str, Any]:
    root = Path(bank_root).resolve()
    manifest = load_strict_json(root / "dataset_manifest.json")
    inventory = load_strict_json(root / "source_inventory.json")
    split = load_strict_json(root / "split_manifest.json")
    summary = load_strict_json(root / "summary.json")
    representatives = load_strict_json(root / "representatives.json")
    rows = read_index(root)
    source_root = Path(manifest["source_exp01b_path"])
    if sha256_file(root / "config_snapshot.yaml") != manifest["dataset_config_sha256"]:
        raise ValueError("dataset config snapshot hash mismatch")
    if sha256_file(manifest["source_config_path"]) != manifest["source_config_sha256"]:
        raise ValueError("source config hash mismatch")
    current_source = _required_source_snapshot(source_root)
    if current_source != inventory["source_required_artifacts"]:
        raise ValueError("frozen source required artifact content/mtime changed")
    if canonical_json_sha256(current_source) != manifest[
        "source_required_artifacts_manifest_sha256"
    ]:
        raise ValueError("source artifact manifest hash mismatch")
    if len(rows) != int(manifest["pair_count"]) or len(rows) != int(summary["bank_pair_count"]):
        raise ValueError("pair count mismatch")
    pair_ids = {row["pair_id"] for row in rows}
    if pair_ids != set(split["development_pair_ids"]).union(split["heldout_pair_ids"]):
        raise ValueError("split union differs from bank pairs")
    if set(split["development_pair_ids"]).intersection(split["heldout_pair_ids"]):
        raise ValueError("development/heldout overlap")
    raw_groups: dict[str, set[str]] = defaultdict(set)
    reconstructed_rank_rows: list[dict[str, Any]] = []
    for row in rows:
        pair_root = root / "pairs" / row["pair_id"]
        context = load_strict_json(pair_root / "context.json")
        descriptors = load_strict_json(pair_root / "descriptors.json")
        provenance = load_strict_json(pair_root / "source_provenance.json")
        if pair_identity(context["identity_payload"]) != row["pair_id"]:
            raise ValueError("pair identity reconstruction failed")
        if context["split"] != row["split"] or split["assignment"][row["pair_id"]] != row["split"]:
            raise ValueError("split assignment mismatch")
        trial = source_root / context["source_trial_relative_path"]
        old_world = np.load(pair_root / "derived/old_world.npy", allow_pickle=False)
        fresh_world = np.load(pair_root / "derived/fresh_world.npy", allow_pickle=False)
        old_raw = np.load(pair_root / "raw/old_lightnav.npy", allow_pickle=False)
        fresh_raw = np.load(pair_root / "raw/fresh_lightnav.npy", allow_pickle=False)
        for value, name in ((old_world, "old world"), (fresh_world, "fresh world"), (old_raw, "old raw"), (fresh_raw, "fresh raw")):
            if value.ndim != 2 or value.shape[1] != 3 or not np.all(np.isfinite(value)):
                raise ValueError(f"invalid {name} array")
        for destination, source in PAIR_COPIES.items():
            expected = provenance["copied_artifacts"][destination]["source_sha256"]
            if sha256_file(pair_root / destination) != expected or sha256_file(trial / source) != expected:
                raise ValueError("source/copy hash mismatch")
        rebuilt = transition_descriptor(
            previous_pose=context["execution_context"]["previous_pose_P_world_se2"],
            boundary_pose=context["execution_context"]["boundary_pose_B_world_se2"],
            old_world=old_world,
            fresh_world=fresh_world,
            direction_epsilon_m=float(descriptors["transition"]["direction_epsilon_m"]),
        )
        if rebuilt != descriptors["transition"]:
            raise ValueError("transition descriptor reconstruction failed")
        config = load_yaml(root / "config_snapshot.yaml")
        for name, path in (("old", old_world), ("fresh", fresh_world)):
            rebuilt_geometry = geometry_descriptor(
                path,
                segment_epsilon_m=float(config["geometry"]["segment_epsilon_m"]),
                curvature_zero_epsilon_per_m=float(
                    config["geometry"]["curvature_zero_epsilon_per_m"]
                ),
            )
            if rebuilt_geometry != descriptors[name]:
                raise ValueError(f"{name} geometry descriptor reconstruction failed")
        historical_source = load_strict_json(
            trial / "results/controller_switch_metrics.json"
        )
        historical = descriptors["historical_raw_switch"]
        historical_mapping = {
            "delta_v_raw_signed_mps": "delta_v_signed_mps",
            "delta_v_raw_abs_mps": "delta_v_abs_mps",
            "delta_omega_raw_signed_rps": "delta_omega_signed_rps",
            "delta_omega_raw_abs_rps": "delta_omega_abs_rps",
            "post_switch_max_abs_delta_v_mps": "post_switch_max_abs_delta_v_mps",
            "post_switch_mean_abs_delta_v_mps": "post_switch_mean_abs_delta_v_mps",
            "post_switch_max_abs_delta_omega_rps": "post_switch_max_abs_delta_omega_rps",
            "post_switch_mean_abs_delta_omega_rps": "post_switch_mean_abs_delta_omega_rps",
            "last_old_desired_command_v_omega": "old_last_command_v_omega",
            "first_fresh_desired_command_v_omega": "fresh_first_command_v_omega",
            "window_size": "window_size",
            "control_dt_s": "control_dt_s",
        }
        if any(
            historical[target] != historical_source[source]
            for target, source in historical_mapping.items()
        ):
            raise ValueError("historical raw-switch metric differs from frozen source")
        expected_raw_group = duplicate_pair_identity(
            context["old"]["raw_sha256"], context["fresh"]["raw_sha256"], kind="raw"
        )
        expected_world_group = duplicate_pair_identity(
            context["old"]["world_sha256"], context["fresh"]["world_sha256"], kind="world"
        )
        if row["raw_pair_group"] != expected_raw_group or row["world_pair_group"] != expected_world_group:
            raise ValueError("duplicate group identity reconstruction failed")
        reconstructed_rank_rows.append(
            {
                "pair_id": row["pair_id"],
                "duplicate_raw_pair_group": row["raw_pair_group"],
                "geometry_group": row["geometry_group"],
                "latency_group": row["latency_group"],
                "historical_raw_switch": historical,
                "transition": descriptors["transition"],
                "timing": descriptors["timing"],
            }
        )
        raw_groups[row["raw_pair_group"]].add(row["split"])
        if require_plots:
            for relative in PAIR_PLOTS:
                _validate_png(pair_root / relative)
    if any(len(values) != 1 for values in raw_groups.values()):
        raise ValueError("exact raw-pair leakage detected")
    reranked = add_deterministic_ranks(reconstructed_rank_rows)
    for rebuilt_row in reranked:
        index_row = next(row for row in rows if row["pair_id"] == rebuilt_row["pair_id"])
        for label in RANK_FIELDS:
            if int(index_row[f"rank_{label}"]) != rebuilt_row[f"rank_{label}"]:
                raise ValueError("difficulty rank reconstruction failed")
    config = load_yaml(root / "config_snapshot.yaml")
    rebuilt_split = deterministic_grouped_split(
        reranked,
        heldout_fraction=float(config["split"]["heldout_fraction"]),
        seed=str(config["split"]["seed"]),
    )
    if rebuilt_split != split:
        raise ValueError("deterministic split reconstruction failed")
    if not split["leakage_check"]["passed"]:
        raise ValueError("split manifest leakage check failed")
    if not set(representatives["unique_pair_ids"]).issubset(pair_ids):
        raise ValueError("representative manifest references an unknown pair")
    plot_count = 0
    if require_plots:
        plot_manifest = load_strict_json(root / "plot_manifest.json")
        expected_plots = {
            *(f"pairs/{pair_id}/{relative}" for pair_id in pair_ids for relative in PAIR_PLOTS),
            *OVERVIEW_PLOTS,
        }
        if set(plot_manifest["plots_sha256"]) != expected_plots:
            raise ValueError("plot manifest file set mismatch")
        for relative, digest in plot_manifest["plots_sha256"].items():
            path = root / relative
            _validate_png(path)
            if sha256_file(path) != digest:
                raise ValueError("plot hash mismatch")
        plot_count = len(expected_plots)
    forbidden = ("new_lightnav_inference", "optimization_executed", "isaac_physics_executed")
    if any(manifest.get(key) is not False for key in forbidden):
        raise ValueError("manifest claims an out-of-scope operation")
    return {
        "valid": True,
        "pair_count": len(rows),
        "development_count": len(split["development_pair_ids"]),
        "heldout_count": len(split["heldout_pair_ids"]),
        "raw_pair_group_count": len(raw_groups),
        "plot_count": plot_count,
        "source_artifacts_verified": len(current_source),
        "raw_pair_leakage": False,
    }


def cumulative_path_length(trajectory: np.ndarray) -> np.ndarray:
    poses = validate_se2_trajectory(trajectory)
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))))


def _draw_xy(
    axis: Any,
    old: np.ndarray,
    fresh: np.ndarray,
    previous: np.ndarray,
    boundary: np.ndarray,
    *,
    compact: bool = False,
) -> None:
    axis.plot(old[:, 0], old[:, 1], "o-", color="tab:blue", markersize=2.5, linewidth=1.4, label="OLD")
    axis.plot(fresh[:, 0], fresh[:, 1], "o-", color="tab:orange", markersize=2.5, linewidth=1.4, label="FRESH")
    axis.scatter(boundary[0], boundary[1], marker="*", color="red", s=75 if compact else 130, label="B boundary", zorder=5)
    if not compact:
        axis.scatter(previous[0], previous[1], marker="o", facecolor="black", s=45, label="P previous", zorder=5)
        delta = boundary[:2] - previous[:2]
        axis.arrow(previous[0], previous[1], delta[0], delta[1], color="black", width=0.002, head_width=0.035, length_includes_head=True, label="P→B")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.22)


def _save_figure_exclusive(figure: Any, path: Path, *, dpi: int) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)


def _contact_sheet(
    *, root: Path, rows: Sequence[Mapping[str, str]], output: Path, canonical: bool, title: str, dpi: int
) -> None:
    import matplotlib.pyplot as plt

    columns = 5
    count = len(rows)
    figure, axes = plt.subplots(
        max(1, math.ceil(count / columns)), columns, figsize=(15, max(3.2, 3.0 * math.ceil(count / columns))), squeeze=False, constrained_layout=True
    )
    for axis, row in zip(axes.flat, rows, strict=False):
        pair_root = root / "pairs" / row["pair_id"]
        context = load_strict_json(pair_root / "context.json")
        old = np.load(pair_root / "derived/old_world.npy", allow_pickle=False)
        fresh = np.load(pair_root / "derived/fresh_world.npy", allow_pickle=False)
        previous = np.asarray(context["execution_context"]["previous_pose_P_world_se2"], dtype=float)
        boundary = np.asarray(context["execution_context"]["boundary_pose_B_world_se2"], dtype=float)
        if canonical:
            old, fresh, previous, boundary = boundary_canonicalize(boundary, old, fresh, previous, boundary)
        _draw_xy(axis, old, fresh, previous, boundary, compact=True)
        axis.set_title(f"{row['display_index']} {row['geometry_group']} / {row['latency_group']}", fontsize=8)
        axis.tick_params(labelsize=7)
    for axis in axes.flat[count:]:
        axis.axis("off")
    figure.suptitle(title, fontsize=13)
    _save_figure_exclusive(figure, output, dpi=dpi)
    plt.close(figure)


def plot_bank(bank_root: str | Path) -> dict[str, Any]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    root = Path(bank_root).resolve()
    if (root / "plot_manifest.json").exists():
        raise FileExistsError(root / "plot_manifest.json")
    rows = read_index(root)
    config = load_yaml(root / "config_snapshot.yaml")
    dpi = int(config["plotting"]["dpi"])
    for row in rows:
        pair_root = root / "pairs" / row["pair_id"]
        plot_root = pair_root / "plots"
        if plot_root.exists():
            raise FileExistsError(plot_root)
        context = load_strict_json(pair_root / "context.json")
        old = np.load(pair_root / "derived/old_world.npy", allow_pickle=False)
        fresh = np.load(pair_root / "derived/fresh_world.npy", allow_pickle=False)
        previous = np.asarray(context["execution_context"]["previous_pose_P_world_se2"], dtype=float)
        boundary = np.asarray(context["execution_context"]["boundary_pose_B_world_se2"], dtype=float)
        old_observation = np.asarray(context["old"]["observation_pose_world_se2"], dtype=float)
        fresh_observation = np.asarray(context["fresh"]["observation_pose_world_se2"], dtype=float)
        figure, axis = plt.subplots(figsize=(8.2, 6.6), constrained_layout=True)
        _draw_xy(axis, old, fresh, previous, boundary)
        axis.scatter(old_observation[0], old_observation[1], marker="s", facecolor="none", edgecolor="tab:blue", s=55, label="OLD obs")
        axis.scatter(fresh_observation[0], fresh_observation[1], marker="D", facecolor="none", edgecolor="tab:orange", s=55, label="FRESH obs")
        axis.scatter(old[-1, 0], old[-1, 1], marker="x", color="tab:blue", s=45, label="OLD endpoint")
        axis.scatter(fresh[-1, 0], fresh[-1, 1], marker="x", color="tab:orange", s=45, label="FRESH endpoint")
        axis.set_xlabel("world x [m]")
        axis.set_ylabel("world y [m]")
        axis.set_title(
            f"{row['pair_id']} | {row['geometry_group']} / {row['latency_group']}\n"
            f"latency={float(row['effective_latency_s']):.3f} s, |Δv|={float(row['raw_delta_v_abs_mps']):.3f} m/s, "
            f"|Δω|={float(row['raw_delta_omega_abs_rps']):.3f} rad/s"
        )
        axis.legend(fontsize=8, ncol=2)
        _save_figure_exclusive(figure, plot_root / "world_xy.png", dpi=dpi)
        plt.close(figure)

        old_c, fresh_c, previous_c, boundary_c = boundary_canonicalize(
            boundary, old, fresh, previous, boundary
        )
        if not np.allclose(boundary_c, np.zeros(3), rtol=0.0, atol=1e-12):
            raise ValueError("canonical boundary does not map to zero")
        figure, axis = plt.subplots(figsize=(8.2, 6.6), constrained_layout=True)
        _draw_xy(axis, old_c, fresh_c, previous_c, boundary_c)
        axis.set_xlabel("B-centered x [m]")
        axis.set_ylabel("B-centered y [m]")
        axis.set_title(
            f"{row['pair_id']} | visualization-only B-centered frame\n"
            f"{row['geometry_group']} / {row['latency_group']}"
        )
        axis.legend(fontsize=8, ncol=2)
        _save_figure_exclusive(figure, plot_root / "boundary_canonical_xy.png", dpi=dpi)
        plt.close(figure)

        figure, axis = plt.subplots(figsize=(8.2, 5.7), constrained_layout=True)
        axis.plot(cumulative_path_length(old), np.unwrap(old[:, 2]), "o-", color="tab:blue", label="OLD yaw (display-unwrapped)")
        axis.plot(cumulative_path_length(fresh), np.unwrap(fresh[:, 2]), "o-", color="tab:orange", label="FRESH yaw (display-unwrapped)")
        axis.set_xlabel("cumulative spatial path length [m] (not time)")
        axis.set_ylabel("yaw [rad]")
        axis.set_title(f"{row['pair_id']} | untimed spatial yaw progression")
        axis.grid(alpha=0.22)
        axis.legend()
        _save_figure_exclusive(figure, plot_root / "yaw_vs_path_length.png", dpi=dpi)
        plt.close(figure)

    _contact_sheet(root=root, rows=rows, output=root / "overview/all_pairs_world.png", canonical=False, title="DATA-01 all pairs — independent world-frame axes", dpi=dpi)
    _contact_sheet(root=root, rows=rows, output=root / "overview/all_pairs_boundary_canonical.png", canonical=True, title="DATA-01 all pairs — visualization-only B-centered frame", dpi=dpi)
    for split_name in ("development", "heldout"):
        selected = [row for row in rows if row["split"] == split_name]
        _contact_sheet(root=root, rows=selected, output=root / f"overview/{split_name}_pairs.png", canonical=True, title=f"DATA-01 {split_name} — visualization-only B-centered frame", dpi=dpi)
    for group in ("G0_straight", "G1_turn", "G2_route_change"):
        selected = [row for row in rows if row["geometry_group"] == group]
        _contact_sheet(root=root, rows=selected, output=root / f"overview/by_geometry_group/{group}.png", canonical=True, title=f"DATA-01 {group} — visualization-only B-centered frame", dpi=dpi)
    for group in ("L0_natural", "L1_added_050"):
        selected = [row for row in rows if row["latency_group"] == group]
        _contact_sheet(root=root, rows=selected, output=root / f"overview/by_latency_group/{group}.png", canonical=True, title=f"DATA-01 {group} — visualization-only B-centered frame", dpi=dpi)
    plots = sorted(
        [path for pair in (root / "pairs").iterdir() for path in (pair / "plots").glob("*.png")]
        + list((root / "overview").rglob("*.png"))
    )
    manifest = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "pair_count": len(rows),
        "per_pair_plot_count": len(rows) * len(PAIR_PLOTS),
        "overview_plot_count": len(OVERVIEW_PLOTS),
        "expected_pair_count_represented_in_contact_sheets": len(rows),
        "canonical_frame_is_visualization_only": True,
        "intrinsic_waypoint_time_base": False,
        "plots_sha256": {
            path.relative_to(root).as_posix(): sha256_file(path) for path in plots
        },
    }
    _write_json_exclusive(root / "plot_manifest.json", manifest)
    return manifest
