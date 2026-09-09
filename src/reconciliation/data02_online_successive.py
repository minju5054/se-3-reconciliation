"""Pure contracts and analysis for DATA-02 online-successive v1.

LightNav path rows are spatial SE(2) waypoints without an intrinsic timestamp.
Timing in this module therefore describes observations, model readiness, switches,
and measured execution only; it never time-aligns waypoint row indices.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reconciliation.graph_metrics import assert_finite_json_tree
from reconciliation.lightnav_adapter import (
    DECODED_OUTPUT_SEMANTICS,
    lightnav_local_to_world,
    raw_actions_to_local_path,
)
from reconciliation.lightnav_execution_envelope import geometry_descriptor
from reconciliation.online_switch import sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_pose_se2, validate_se2_trajectory


FloatArray = NDArray[np.float64]
TRANSITION_STATUSES = (
    "ELIGIBLE_MOVING",
    "MODEL_STOP",
    "OLD_EXHAUSTED",
    "CHUNK_EXHAUSTED_BEFORE_TRIGGER",
    "TIMING_INVALID",
    "NONFINITE_MODEL_OUTPUT",
    "EXECUTION_COLLISION",
    "EXECUTION_OUT_OF_ENVELOPE",
    "TECHNICAL_INVALID",
)
GEOMETRY_BINS = ("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING", "OTHER")
DIFFICULTY_BINS = ("BENIGN", "CHALLENGING", "INTERMEDIATE")
FINAL_STATUSES = (
    "DATA02_READY_FOR_FORMULATION_RESEARCH",
    "DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT",
    "DATA02_TECHNICAL_INVALID",
    "DATA02_EPISODE_TEMPLATE_INSUFFICIENT",
)
WHEEL_LABELS = ("front_left", "front_right", "rear_left", "rear_right")


def strict_json(path: str | Path) -> dict[str, Any]:
    value = json.loads(
        Path(path).read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
    )
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def validate_raw_chunk(value: ArrayLike, *, name: str = "raw LightNav chunk") -> np.ndarray:
    """Validate arbitrary non-empty N x 3 numeric output and return a copy."""

    array = np.asarray(value)
    if array.ndim != 2 or array.shape[0] < 1 or array.shape[1] != 3:
        raise ValueError(f"{name} must have arbitrary non-empty shape (N, 3)")
    if not np.issubdtype(array.dtype, np.number) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite and numeric")
    return np.array(array, copy=True, order="C")


def chunk_content_sha256(value: ArrayLike) -> str:
    raw = validate_raw_chunk(value)
    digest = hashlib.sha256()
    digest.update(str(raw.dtype).encode("ascii"))
    digest.update(str(tuple(raw.shape)).encode("ascii"))
    digest.update(np.ascontiguousarray(raw).tobytes(order="C"))
    return digest.hexdigest()


def ordered_pair_sha256(old_hash: str, fresh_hash: str) -> str:
    for label, digest in (("OLD", old_hash), ("FRESH", fresh_hash)):
        if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise ValueError(f"{label} hash is not lowercase SHA-256")
    return hashlib.sha256(f"DATA02_ORDERED_RAW_PAIR_V1\0{old_hash}\0{fresh_hash}".encode()).hexdigest()


def observation_anchored_paths(raw: ArrayLike, observation_pose: ArrayLike) -> tuple[FloatArray, FloatArray]:
    """Create copied local/world paths; no B pose is inserted or substituted."""

    source = validate_raw_chunk(raw)
    local = raw_actions_to_local_path(
        source,
        decoded_output_semantics=DECODED_OUTPUT_SEMANTICS,
        expected_horizon=None,
    )
    world = lightnav_local_to_world(local, validate_pose_se2(observation_pose))
    if local.shape != source.shape or world.shape != source.shape:
        raise AssertionError("anchoring changed the LightNav pose count")
    return local.copy(), world.copy()


def variant_pose(base_pose: ArrayLike, variant: Mapping[str, Any]) -> FloatArray:
    base = validate_pose_se2(base_pose)
    yaw = float(base[2])
    longitudinal = float(variant["longitudinal_offset_m"])
    lateral = float(variant["lateral_offset_m"])
    yaw_offset = float(variant["yaw_offset_rad"])
    if not all(math.isfinite(item) for item in (longitudinal, lateral, yaw_offset)):
        raise ValueError("variant offsets must be finite")
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    result = base.copy()
    result[:2] += longitudinal * forward + lateral * left
    result[2] = float(wrap_angle(yaw + yaw_offset))
    return result


@dataclass(frozen=True, slots=True)
class TransitionTiming:
    t_obs_sim_s: float
    t_request_host_ns: int
    t_model_ready_host_ns: int
    t_ready_sim_s: float
    t_switch_sim_s: float
    host_request_response_s: float
    model_reported_s: float
    real_time_factor: float

    def validate(self, acceptable_rtf: Sequence[float]) -> dict[str, Any]:
        values = (
            self.t_obs_sim_s,
            self.t_ready_sim_s,
            self.t_switch_sim_s,
            self.host_request_response_s,
            self.model_reported_s,
            self.real_time_factor,
        )
        finite = all(math.isfinite(float(value)) for value in values)
        host_order = self.t_request_host_ns <= self.t_model_ready_host_ns
        sim_order = self.t_obs_sim_s <= self.t_ready_sim_s <= self.t_switch_sim_s
        host_consistent = math.isclose(
            self.host_request_response_s,
            (self.t_model_ready_host_ns - self.t_request_host_ns) / 1e9,
            abs_tol=1e-6,
            rel_tol=1e-6,
        )
        low, high = (float(acceptable_rtf[0]), float(acceptable_rtf[1]))
        rtf_valid = low <= self.real_time_factor <= high
        checks = {
            "finite": finite,
            "host_order": host_order,
            "simulation_order": sim_order,
            "host_latency_consistent": host_consistent,
            "real_time_factor_in_range": rtf_valid,
        }
        return {
            "valid": all(checks.values()),
            "checks": checks,
            "t_obs_sim_s": float(self.t_obs_sim_s),
            "t_request_host_monotonic_ns": int(self.t_request_host_ns),
            "t_model_ready_host_monotonic_ns": int(self.t_model_ready_host_ns),
            "t_ready_sim_s": float(self.t_ready_sim_s),
            "t_switch_sim_s": float(self.t_switch_sim_s),
            "tau_effective_s": float(self.t_switch_sim_s - self.t_obs_sim_s),
            "observation_to_ready_sim_s": float(self.t_ready_sim_s - self.t_obs_sim_s),
            "ready_to_switch_sim_s": float(self.t_switch_sim_s - self.t_ready_sim_s),
            "request_response_host_s": float(self.host_request_response_s),
            "model_reported_s": float(self.model_reported_s),
            "real_time_factor": float(self.real_time_factor),
        }


def classify_transition(
    *,
    model_stop: bool = False,
    old_exhausted: bool = False,
    exhausted_before_trigger: bool = False,
    timing_valid: bool = True,
    finite_model_output: bool = True,
    collision: bool = False,
    out_of_envelope: bool = False,
    technical_valid: bool = True,
) -> str:
    """Return exactly one status using the frozen exclusion precedence."""

    if not technical_valid:
        return "TECHNICAL_INVALID"
    if not finite_model_output:
        return "NONFINITE_MODEL_OUTPUT"
    if collision:
        return "EXECUTION_COLLISION"
    if out_of_envelope:
        return "EXECUTION_OUT_OF_ENVELOPE"
    if exhausted_before_trigger:
        return "CHUNK_EXHAUSTED_BEFORE_TRIGGER"
    if old_exhausted:
        return "OLD_EXHAUSTED"
    if not timing_valid:
        return "TIMING_INVALID"
    if model_stop:
        return "MODEL_STOP"
    return "ELIGIBLE_MOVING"


def path_geometry(value: ArrayLike) -> dict[str, Any]:
    poses = validate_se2_trajectory(value)
    result = geometry_descriptor(poses)
    yaw = float(poses[0, 2])
    displacement = poses[-1, :2] - poses[0, :2]
    result["endpoint_forward_m"] = float(displacement @ np.array([math.cos(yaw), math.sin(yaw)]))
    result["endpoint_lateral_m"] = float(displacement @ np.array([-math.sin(yaw), math.cos(yaw)]))
    result["signed_net_yaw_rad"] = float(result["net_pose_yaw_change_rad"])
    result["signed_tangent_turn_rad"] = float(result["cumulative_signed_tangent_turn_rad"])
    result["maximum_lateral_excursion_m"] = float(result["max_abs_lateral_departure_m"])
    assert_finite_json_tree(result)
    return result


def geometry_bin(descriptor: Mapping[str, Any]) -> str:
    yaw = float(descriptor["signed_net_yaw_rad"])
    lateral = float(descriptor["endpoint_lateral_m"])
    if abs(yaw) < 0.10 and abs(lateral) < 0.10:
        return "STRAIGHT_LIKE"
    if yaw >= 0.15 or lateral >= 0.15:
        return "POSITIVE_TURNING"
    if yaw <= -0.15 or lateral <= -0.15:
        return "NEGATIVE_TURNING"
    return "OTHER"


def difficulty_bin(delta_v_mps: float, delta_omega_rps: float) -> str:
    dv, domega = abs(float(delta_v_mps)), abs(float(delta_omega_rps))
    if not math.isfinite(dv) or not math.isfinite(domega):
        raise ValueError("command discontinuity must be finite")
    if dv <= 0.05 and domega <= 0.10:
        return "BENIGN"
    if dv >= 0.15 or domega >= 0.30:
        return "CHALLENGING"
    return "INTERMEDIATE"


def transition_metrics(
    old_world: ArrayLike,
    fresh_world: ArrayLike,
    observation_pose: ArrayLike,
    pose_pre_switch: ArrayLike,
    boundary_pose: ArrayLike,
    old_desired_command: ArrayLike,
    first_fresh_desired_command: ArrayLike,
    *,
    model_ready_pose: ArrayLike | None = None,
) -> dict[str, Any]:
    old = validate_se2_trajectory(old_world, name="OLD world")
    fresh = validate_se2_trajectory(fresh_world, name="FRESH world")
    obs = validate_pose_se2(observation_pose)
    pre = validate_pose_se2(pose_pre_switch)
    boundary = validate_pose_se2(boundary_pose)
    ready_pose = boundary if model_ready_pose is None else validate_pose_se2(model_ready_pose)
    old_command = np.asarray(old_desired_command, dtype=np.float64)
    new_command = np.asarray(first_fresh_desired_command, dtype=np.float64)
    if old_command.shape != (2,) or new_command.shape != (2,) or not np.all(np.isfinite([old_command, new_command])):
        raise ValueError("commands must be finite [v, omega]")
    old_geometry = path_geometry(old)
    fresh_geometry = path_geometry(fresh)
    delta = new_command - old_command
    result = {
        "old_geometry": old_geometry,
        "fresh_geometry": fresh_geometry,
        "fresh_geometry_bin": geometry_bin(fresh_geometry),
        "command_discontinuity": {
            "old_last_desired_v_mps": float(old_command[0]),
            "old_last_desired_omega_rps": float(old_command[1]),
            "fresh_first_desired_v_mps": float(new_command[0]),
            "fresh_first_desired_omega_rps": float(new_command[1]),
            "delta_v_mps": float(delta[0]),
            "delta_omega_rps": float(delta[1]),
            "difficulty_bin": difficulty_bin(float(delta[0]), float(delta[1])),
            "fresh_command_semantics": "first TrajectoryFollower desired command evaluated at B, not FRESH row 0",
        },
        "b_to_fresh0_translation_m": float(np.linalg.norm(boundary[:2] - fresh[0, :2])),
        "b_to_fresh0_yaw_rad": abs(float(wrap_angle(boundary[2] - fresh[0, 2]))),
        "p_to_b_translation_m": float(np.linalg.norm(boundary[:2] - pre[:2])),
        "p_to_b_yaw_rad": abs(float(wrap_angle(boundary[2] - pre[2]))),
        "observation_to_boundary_translation_m": float(np.linalg.norm(boundary[:2] - obs[:2])),
        "observation_to_boundary_yaw_rad": abs(float(wrap_angle(boundary[2] - obs[2]))),
        "observation_to_model_ready_translation_m": float(np.linalg.norm(ready_pose[:2] - obs[:2])),
        "observation_to_model_ready_yaw_rad": abs(float(wrap_angle(ready_pose[2] - obs[2]))),
        "waypoint_time_semantics": "none; all OLD/FRESH descriptors are spatial",
    }
    assert_finite_json_tree(result)
    return result


@dataclass(slots=True)
class SuccessiveEpisodeState:
    """Small audited state machine: one reset, one request, FRESH becomes next OLD."""

    episode_id: str
    reset_count: int = 0
    active_chunk_id: str | None = None
    in_flight: bool = False
    transition_index: int = 0

    def reset(self) -> None:
        if self.reset_count:
            raise RuntimeError("LightNav may reset only once per DATA-02 episode")
        self.reset_count = 1

    def bootstrap(self, chunk_id: str) -> None:
        if self.reset_count != 1 or self.active_chunk_id is not None:
            raise RuntimeError("episode must reset once before bootstrap")
        self.active_chunk_id = str(chunk_id)

    def start_request(self) -> None:
        if self.active_chunk_id is None or self.in_flight:
            raise RuntimeError("exactly one FRESH request may be in flight")
        self.in_flight = True

    def promote_fresh(self, fresh_chunk_id: str) -> tuple[str, str]:
        if not self.in_flight or self.active_chunk_id is None:
            raise RuntimeError("cannot switch without a completed in-flight request")
        old = self.active_chunk_id
        fresh = str(fresh_chunk_id)
        self.active_chunk_id = fresh
        self.in_flight = False
        self.transition_index += 1
        return old, fresh


class _UnionFind:
    def __init__(self) -> None:
        self.parent: dict[str, str] = {}

    def find(self, item: str) -> str:
        self.parent.setdefault(item, item)
        if self.parent[item] != item:
            self.parent[item] = self.find(self.parent[item])
        return self.parent[item]

    def union(self, left: str, right: str) -> None:
        a, b = self.find(left), self.find(right)
        if a != b:
            self.parent[max(a, b)] = min(a, b)


def episode_pair_components(records: Sequence[Mapping[str, Any]]) -> list[list[dict[str, Any]]]:
    """Connected components prevent episode and ordered-raw-pair leakage."""

    uf = _UnionFind()
    retained: list[dict[str, Any]] = []
    for source in records:
        if source.get("status") != "ELIGIBLE_MOVING":
            continue
        row = dict(source)
        episode = f"episode:{row['episode_id']}"
        pair = f"pair:{row['ordered_raw_pair_sha256']}"
        uf.union(episode, pair)
        retained.append(row)
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in retained:
        groups[uf.find(f"episode:{row['episode_id']}")].append(row)
    return sorted((sorted(rows, key=lambda item: str(item["transition_id"])) for rows in groups.values()), key=lambda rows: str(rows[0]["transition_id"]))


def isolated_split(records: Sequence[Mapping[str, Any]], heldout_fraction: float = 0.25) -> dict[str, Any]:
    components = episode_pair_components(records)
    total = sum(len(group) for group in components)
    if total == 0:
        return {"development_transition_ids": [], "heldout_transition_ids": [], "no_episode_leakage": True, "no_ordered_raw_pair_leakage": True, "scenario_coverage": False}
    target = int(round(total * float(heldout_fraction)))
    # Components are indivisible. Seed held-out coverage deterministically before
    # filling toward the requested size, while refusing choices that would remove
    # the last development component for a scenario. This is an outcome-independent
    # split heuristic; infeasible coverage remains explicit in the returned flags.
    ranked = sorted(components, key=lambda group: (len(group), str(group[0]["transition_id"])))
    component_templates = [set(str(row["template_id"]) for row in group) for group in ranked]
    all_scenarios = set().union(*component_templates)
    selected: set[int] = set()

    def development_keeps_scenarios(candidate: int) -> bool:
        removed = selected | {candidate}
        remaining = set().union(
            *(templates for index, templates in enumerate(component_templates) if index not in removed)
        ) if len(removed) < len(ranked) else set()
        return remaining == all_scenarios

    covered_scenarios: set[str] = set()
    for scenario in sorted(all_scenarios):
        if scenario in covered_scenarios:
            continue
        choices = [
            index for index, templates in enumerate(component_templates)
            if index not in selected and scenario in templates and development_keeps_scenarios(index)
        ]
        if choices:
            choice = min(
                choices,
                key=lambda index: (
                    len(ranked[index]),
                    -len(component_templates[index] - covered_scenarios),
                    str(ranked[index][0]["transition_id"]),
                ),
            )
            selected.add(choice)
            covered_scenarios.update(component_templates[choice])

    while True:
        heldout_size = sum(len(ranked[index]) for index in selected)
        choices = [
            index for index in range(len(ranked))
            if index not in selected and development_keeps_scenarios(index)
        ]
        if not choices:
            break
        choice = min(
            choices,
            key=lambda index: (
                abs(heldout_size + len(ranked[index]) - target),
                heldout_size + len(ranked[index]) > target,
                str(ranked[index][0]["transition_id"]),
            ),
        )
        if selected and abs(heldout_size - target) <= abs(heldout_size + len(ranked[choice]) - target):
            break
        selected.add(choice)

    if not selected and ranked:
        # This fallback can only make scenario_coverage false when the graph itself
        # makes two-sided coverage impossible; leakage isolation is still preserved.
        selected.add(0)
    heldout = [row for index in sorted(selected) for row in ranked[index]]
    development = [row for index, group in enumerate(ranked) if index not in selected for row in group]
    dev_episodes = {row["episode_id"] for row in development}
    test_episodes = {row["episode_id"] for row in heldout}
    dev_pairs = {row["ordered_raw_pair_sha256"] for row in development}
    test_pairs = {row["ordered_raw_pair_sha256"] for row in heldout}
    all_scenarios = {row["template_id"] for row in development + heldout}
    covered = bool(all_scenarios) and {row["template_id"] for row in development} == all_scenarios and {row["template_id"] for row in heldout} == all_scenarios
    all_geometry = {row["fresh_geometry_bin"] for row in development + heldout}
    all_difficulty = {row["difficulty_bin"] for row in development + heldout}
    return {
        "development_transition_ids": sorted(str(row["transition_id"]) for row in development),
        "heldout_transition_ids": sorted(str(row["transition_id"]) for row in heldout),
        "development_episode_ids": sorted(dev_episodes),
        "heldout_episode_ids": sorted(test_episodes),
        "no_episode_leakage": not bool(dev_episodes & test_episodes),
        "no_ordered_raw_pair_leakage": not bool(dev_pairs & test_pairs),
        "scenario_coverage": covered,
        "development_scenarios": sorted({str(row["template_id"]) for row in development}),
        "heldout_scenarios": sorted({str(row["template_id"]) for row in heldout}),
        "geometry_coverage_both_splits": bool(all_geometry) and {row["fresh_geometry_bin"] for row in development} == all_geometry and {row["fresh_geometry_bin"] for row in heldout} == all_geometry,
        "difficulty_coverage_both_splits": bool(all_difficulty) and {row["difficulty_bin"] for row in development} == all_difficulty and {row["difficulty_bin"] for row in heldout} == all_difficulty,
        "target_heldout_fraction": float(heldout_fraction),
        "actual_heldout_fraction": len(heldout) / total,
        "component_count": len(components),
    }


def readiness_decision(records: Sequence[Mapping[str, Any]], split: Mapping[str, Any], criteria: Mapping[str, Any], *, artifacts_valid: bool, template_count: int) -> dict[str, Any]:
    if template_count < 10:
        return {"status": "DATA02_EPISODE_TEMPLATE_INSUFFICIENT", "checks": {"template_count": False}}
    if not artifacts_valid:
        return {"status": "DATA02_TECHNICAL_INVALID", "checks": {"strict_artifacts": False}}
    eligible = [row for row in records if row.get("status") == "ELIGIBLE_MOVING"]
    pairs = Counter(str(row["ordered_raw_pair_sha256"]) for row in eligible)
    geometries = Counter(str(row["fresh_geometry_bin"]) for row in eligible)
    difficulties = Counter(str(row["difficulty_bin"]) for row in eligible)
    checks = {
        "A_minimum_eligible_moving": len(eligible) >= int(criteria["minimum_eligible_moving"]),
        "B_minimum_unique_ordered_raw_pairs": len(pairs) >= int(criteria["minimum_unique_ordered_raw_pairs"]),
        "C_largest_pair_fraction": (max(pairs.values(), default=0) / len(eligible) if eligible else 1.0) <= float(criteria["maximum_largest_pair_fraction"]),
        "D_unique_old_and_fresh": len({row["old_raw_sha256"] for row in eligible}) >= int(criteria["minimum_unique_old_chunks"]) and len({row["fresh_raw_sha256"] for row in eligible}) >= int(criteria["minimum_unique_fresh_chunks"]),
        "E_geometry_coverage": geometries["STRAIGHT_LIKE"] >= int(criteria["minimum_straight_like"]) and geometries["POSITIVE_TURNING"] >= int(criteria["minimum_positive_turning"]) and geometries["NEGATIVE_TURNING"] >= int(criteria["minimum_negative_turning"]),
        "F_difficulty_coverage": difficulties["BENIGN"] >= int(criteria["minimum_benign"]) and difficulties["CHALLENGING"] >= int(criteria["minimum_challenging"]),
        "G_isolated_split": bool(split.get("no_episode_leakage")) and bool(split.get("no_ordered_raw_pair_leakage")) and bool(split.get("scenario_coverage")) and len(split.get("heldout_transition_ids", [])) >= int(criteria["minimum_heldout_eligible"]),
        "H_strict_artifacts": True,
    }
    status = "DATA02_READY_FOR_FORMULATION_RESEARCH" if all(checks.values()) else "DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT"
    return {
        "status": status,
        "checks": checks,
        "counts": {
            "eligible_moving": len(eligible),
            "unique_ordered_raw_pairs": len(pairs),
            "unique_old_chunks": len({row["old_raw_sha256"] for row in eligible}),
            "unique_fresh_chunks": len({row["fresh_raw_sha256"] for row in eligible}),
            "largest_pair_count": max(pairs.values(), default=0),
            "geometry_bins": dict(sorted(geometries.items())),
            "difficulty_bins": dict(sorted(difficulties.items())),
        },
    }


def validate_transition_artifact(directory: str | Path) -> dict[str, Any]:
    root = Path(directory)
    metadata = strict_json(root / "transition.json")
    required = (
        "raw/old_actions.npy", "raw/fresh_actions.npy", "derived/old_world.npy",
        "derived/fresh_world.npy", "actual.npy", "telemetry.csv",
    )
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise ValueError(f"transition artifact missing: {missing}")
    old_raw = validate_raw_chunk(np.load(root / "raw/old_actions.npy", allow_pickle=False))
    fresh_raw = validate_raw_chunk(np.load(root / "raw/fresh_actions.npy", allow_pickle=False))
    old_world = validate_se2_trajectory(np.load(root / "derived/old_world.npy", allow_pickle=False))
    fresh_world = validate_se2_trajectory(np.load(root / "derived/fresh_world.npy", allow_pickle=False))
    actual = validate_se2_trajectory(np.load(root / "actual.npy", allow_pickle=False))
    if old_world.shape != old_raw.shape or fresh_world.shape != fresh_raw.shape:
        raise ValueError("raw/derived pose count mismatch")
    if metadata.get("waypoint_dt") is not None:
        raise ValueError("DATA-02 must not fabricate waypoint_dt")
    hashes = metadata["hashes"]
    if chunk_content_sha256(old_raw) != hashes["old_raw_sha256"] or chunk_content_sha256(fresh_raw) != hashes["fresh_raw_sha256"]:
        raise ValueError("raw chunk content hash mismatch")
    for relative, expected in metadata["artifact_sha256"].items():
        if sha256_file(root / relative) != expected:
            raise ValueError(f"artifact hash mismatch: {relative}")
    if metadata["status"] not in TRANSITION_STATUSES or actual.shape[0] < 1:
        raise ValueError("invalid transition status or actual path")
    return metadata


def percentile_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "min": None, "median": None, "p90": None, "p95": None, "max": None}
    if not np.all(np.isfinite(array)):
        raise ValueError("summary values must be finite")
    return {
        "count": int(array.size), "min": float(np.min(array)),
        "median": float(np.median(array)), "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)), "max": float(np.max(array)),
    }
