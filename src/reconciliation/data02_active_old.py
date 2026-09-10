"""Read-only reconstruction of one DATA-02 current-OLD active interval.

The transition-level ``actual.npy`` and ``telemetry.csv`` artifacts are the
primary source.  For transition ``i > 0`` the previous saved switch boundary
is prepended as the exact activation event for the current OLD.  It is not
misrepresented as a telemetry row: the first current-OLD telemetry sample is
one physics step later in the frozen collector.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from reconciliation.data02_online_successive import (
    strict_json,
    validate_transition_artifact,
)
from reconciliation.online_switch import sha256_file


FloatArray = NDArray[np.float64]
_TIME_ATOL_S = 1e-6
_POSE_ATOL = 1e-6


@dataclass(frozen=True, slots=True)
class ActiveOldInterval:
    """Hash-verified saved motion while one transition's OLD was active."""

    run: Path
    episode_id: str
    transition_index: int
    transition_directory: Path
    transition_metadata: Mapping[str, Any]
    old_chunk_id: str
    fresh_chunk_id: str
    old_world: FloatArray
    fresh_world: FloatArray
    telemetry_rows: tuple[Mapping[str, str], ...]
    telemetry_sim_times_s: FloatArray
    telemetry_actual_poses: FloatArray
    display_sim_times_s: FloatArray
    display_actual_poses: FloatArray
    activation_sim_time_s: float
    activation_pose_world_se2: FloatArray
    activation_source: str
    activation_boundary_prepended: bool
    observation_sim_time_s: float
    observation_pose_world_se2: FloatArray
    model_ready_sim_time_s: float
    model_ready_pose_world_se2: FloatArray
    p_sim_time_s: float
    p_pose_world_se2: FloatArray
    switch_sim_time_s: float
    boundary_pose_world_se2: FloatArray
    active_old_path_length_m: float
    active_old_net_displacement_m: float
    active_old_yaw_change_rad: float
    source_sha256: Mapping[str, str]


def _csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = list(reader.fieldnames or ())
        rows = list(reader)
    if not fieldnames or not rows:
        raise ValueError(f"{path} must contain a header and at least one row")
    return fieldnames, rows


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _readonly_pose(value: Sequence[float], name: str) -> FloatArray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite SE(2) pose")
    result.setflags(write=False)
    return result


def _readonly_trajectory(value: np.ndarray, name: str) -> FloatArray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.ndim != 2 or result.shape[0] < 1 or result.shape[1] != 3:
        raise ValueError(f"{name} must be a non-empty N x 3 trajectory")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains NaN/Inf")
    result.setflags(write=False)
    return result


def _readonly_times(value: Sequence[float], name: str) -> FloatArray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.ndim != 1 or result.size < 1 or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a non-empty finite vector")
    if result.size > 1 and not np.all(np.diff(result) > 0.0):
        raise ValueError(f"{name} must be strictly increasing")
    result.setflags(write=False)
    return result


def _poses_from_rows(rows: Sequence[Mapping[str, str]]) -> FloatArray:
    return _readonly_trajectory(
        np.asarray(
            [
                [
                    _finite_float(row["actual_x"], "actual_x"),
                    _finite_float(row["actual_y"], "actual_y"),
                    _finite_float(row["actual_yaw"], "actual_yaw"),
                ]
                for row in rows
            ],
            dtype=np.float64,
        ),
        "transition telemetry actual poses",
    )


def _manifest_episode(run: Path, episode_id: str) -> tuple[Mapping[str, Any], Path]:
    manifest_path = run / "collection_manifest.json"
    manifest = strict_json(manifest_path)
    matches = [row for row in manifest["episodes"] if row["episode_id"] == episode_id]
    if len(matches) != 1:
        raise ValueError(f"collection manifest has no unique entry for {episode_id}")
    return matches[0], manifest_path


def _verified_transition(
    episode: Path,
    manifest_episode: Mapping[str, Any],
    transition_index: int,
) -> tuple[Path, dict[str, Any], str]:
    if transition_index < 0:
        raise ValueError("transition index must be non-negative")
    recorded_hashes = list(manifest_episode["transition_metadata_sha256"])
    if transition_index >= len(recorded_hashes):
        raise ValueError("transition index is outside the collection manifest")
    directory = episode / "transitions" / f"transition_{transition_index:02d}"
    metadata = validate_transition_artifact(directory)
    metadata_path = directory / "transition.json"
    actual_hash = sha256_file(metadata_path)
    if actual_hash != str(recorded_hashes[transition_index]):
        raise ValueError("transition JSON hash differs from collection manifest")
    if int(metadata["transition_index"]) != transition_index:
        raise ValueError("transition metadata index differs from its path")
    return directory, metadata, actual_hash


def _matching_pose_at(
    rows: Sequence[Mapping[str, str]], time_s: float, expected: np.ndarray, name: str
) -> None:
    matches = [
        row
        for row in rows
        if math.isclose(float(row["sim_time_s"]), time_s, rel_tol=0.0, abs_tol=_TIME_ATOL_S)
    ]
    if len(matches) != 1:
        raise ValueError(f"transition telemetry lacks a unique {name} sample")
    actual = np.asarray(
        [matches[0]["actual_x"], matches[0]["actual_y"], matches[0]["actual_yaw"]],
        dtype=np.float64,
    )
    if not np.allclose(actual, expected, rtol=0.0, atol=_POSE_ATOL):
        raise ValueError(f"{name} pose differs from transition telemetry")


def _path_length(poses: np.ndarray) -> float:
    if len(poses) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum())


def _wrapped_angle(value: float) -> float:
    return float((float(value) + math.pi) % (2.0 * math.pi) - math.pi)


def load_active_old_interval(
    run: str | Path, episode_id: str, transition_index: int
) -> ActiveOldInterval:
    """Load and validate exactly one saved current-OLD active interval.

    No source is written.  All returned NumPy arrays are independent read-only
    copies.  For transition zero, the earliest transition/episode telemetry
    state is the activation state; no earlier pose is extrapolated.
    """

    run_path = Path(run).expanduser().resolve()
    episode = run_path / "episodes" / str(episode_id)
    manifest_episode, manifest_path = _manifest_episode(run_path, str(episode_id))
    directory, metadata, transition_json_hash = _verified_transition(
        episode, manifest_episode, int(transition_index)
    )
    if str(metadata["episode_id"]) != str(episode_id):
        raise ValueError("transition metadata episode differs from its path")

    episode_telemetry_path = episode / "telemetry.csv"
    episode_hash = sha256_file(episode_telemetry_path)
    if episode_hash != str(manifest_episode["telemetry_sha256"]):
        raise ValueError("episode telemetry hash differs from collection manifest")
    episode_fields, episode_rows = _csv_rows(episode_telemetry_path)
    transition_telemetry_path = directory / "telemetry.csv"
    transition_fields, transition_rows = _csv_rows(transition_telemetry_path)
    if transition_fields != episode_fields:
        raise ValueError("transition and episode telemetry schemas differ")

    episode_indices = [int(row["sample_index"]) for row in episode_rows]
    if episode_indices != list(range(len(episode_rows))):
        raise ValueError("episode telemetry sample indices are not contiguous from zero")
    transition_indices = [int(row["sample_index"]) for row in transition_rows]
    if transition_indices != list(
        range(transition_indices[0], transition_indices[0] + len(transition_indices))
    ):
        raise ValueError("transition telemetry sample indices are not contiguous")
    episode_slice = episode_rows[
        transition_indices[0] : transition_indices[0] + len(transition_rows)
    ]
    if episode_slice != transition_rows:
        raise ValueError("transition telemetry is not an exact contiguous episode slice")

    old_chunk_id = str(metadata["old_chunk_id"])
    if any(row["active_chunk_id"] != old_chunk_id for row in transition_rows):
        raise ValueError("transition telemetry contains motion from a different active chunk")
    times = _readonly_times(
        [_finite_float(row["sim_time_s"], "sim_time_s") for row in transition_rows],
        "transition telemetry simulation times",
    )
    poses = _poses_from_rows(transition_rows)
    actual = _readonly_trajectory(
        np.load(directory / "actual.npy", allow_pickle=False), "transition actual.npy"
    )
    if not np.array_equal(actual, poses):
        raise ValueError("actual.npy differs from transition telemetry poses")

    old_world = _readonly_trajectory(
        np.load(directory / "derived/old_world.npy", allow_pickle=False), "OLD world"
    )
    fresh_world = _readonly_trajectory(
        np.load(directory / "derived/fresh_world.npy", allow_pickle=False), "FRESH world"
    )
    timing = metadata["timing"]
    if timing.get("valid") is not True:
        raise ValueError("selected transition timing is not valid")
    observation_time = _finite_float(timing["t_obs_sim_s"], "t_obs_sim_s")
    ready_time = _finite_float(timing["t_ready_sim_s"], "t_ready_sim_s")
    switch_time = _finite_float(timing["t_switch_sim_s"], "t_switch_sim_s")
    p_time = _finite_float(
        metadata["pose_immediately_before_switch_P_sim_time_s"], "P sim time"
    )
    observation_pose = _readonly_pose(
        metadata["observation_pose_world_se2"], "observation pose"
    )
    ready_pose = _readonly_pose(metadata["model_ready_pose_world_se2"], "model-ready pose")
    p_pose = _readonly_pose(
        metadata["pose_immediately_before_switch_P_world_se2"], "P pose"
    )
    boundary = _readonly_pose(metadata["switch_boundary_B_world_se2"], "B pose")
    if not times[0] <= observation_time <= ready_time <= switch_time:
        raise ValueError("observation/readiness/switch lie outside the active OLD interval")
    if not observation_time <= p_time < switch_time:
        raise ValueError("P must follow observation and precede B")
    if not math.isclose(float(times[-1]), switch_time, rel_tol=0.0, abs_tol=_TIME_ATOL_S):
        raise ValueError("transition telemetry does not end at saved switch time")
    if not np.allclose(poses[-1], boundary, rtol=0.0, atol=_POSE_ATOL):
        raise ValueError("transition actual path does not end at saved B")
    _matching_pose_at(transition_rows, observation_time, observation_pose, "observation")
    _matching_pose_at(transition_rows, ready_time, ready_pose, "model-ready")
    _matching_pose_at(transition_rows, p_time, p_pose, "P")

    source_hashes: dict[str, str] = {
        "collection_manifest.json": sha256_file(manifest_path),
        "episode_telemetry.csv": episode_hash,
        "transition.json": transition_json_hash,
        "derived/old_world.npy": str(metadata["artifact_sha256"]["derived/old_world.npy"]),
        "derived/fresh_world.npy": str(
            metadata["artifact_sha256"]["derived/fresh_world.npy"]
        ),
        "actual.npy": str(metadata["artifact_sha256"]["actual.npy"]),
        "transition_telemetry.csv": str(metadata["artifact_sha256"]["telemetry.csv"]),
    }
    if transition_index > 0:
        previous_directory, previous, previous_json_hash = _verified_transition(
            episode, manifest_episode, transition_index - 1
        )
        if str(previous["fresh_chunk_id"]) != old_chunk_id:
            raise ValueError("previous FRESH was not promoted to the selected current OLD")
        activation_time = _finite_float(
            previous["timing"]["t_switch_sim_s"], "previous switch time"
        )
        activation_pose = _readonly_pose(
            previous["switch_boundary_B_world_se2"], "previous switch boundary"
        )
        previous_actual = _readonly_trajectory(
            np.load(previous_directory / "actual.npy", allow_pickle=False),
            "previous transition actual.npy",
        )
        if not np.allclose(previous_actual[-1], activation_pose, rtol=0.0, atol=_POSE_ATOL):
            raise ValueError("previous transition actual path does not end at activation B")
        if not activation_time < float(times[0]):
            raise ValueError("current OLD telemetry must start after its activation boundary")
        display_times = _readonly_times(
            np.concatenate(([activation_time], times)), "active OLD display times"
        )
        display_poses = _readonly_trajectory(
            np.vstack((activation_pose, poses)), "active OLD display poses"
        )
        activation_source = "PREVIOUS_TRANSITION_SWITCH_BOUNDARY"
        activation_prepended = True
        source_hashes["previous_transition.json"] = previous_json_hash
    else:
        if transition_indices[0] != 0 or episode_rows[0] != transition_rows[0]:
            raise ValueError("bootstrap OLD does not start at the earliest episode telemetry state")
        activation_time = float(times[0])
        activation_pose = _readonly_pose(poses[0], "bootstrap activation pose")
        display_times = _readonly_times(times, "active OLD display times")
        display_poses = _readonly_trajectory(poses, "active OLD display poses")
        activation_source = "EARLIEST_BOOTSTRAP_OLD_TELEMETRY"
        activation_prepended = False

    if not activation_time <= observation_time <= switch_time:
        raise ValueError("FRESH observation is outside the exact current-OLD active interval")
    frozen_rows = tuple(MappingProxyType(dict(row)) for row in transition_rows)
    source_hashes_proxy = MappingProxyType(dict(sorted(source_hashes.items())))
    metadata_proxy = MappingProxyType(metadata)
    return ActiveOldInterval(
        run=run_path,
        episode_id=str(episode_id),
        transition_index=int(transition_index),
        transition_directory=directory,
        transition_metadata=metadata_proxy,
        old_chunk_id=old_chunk_id,
        fresh_chunk_id=str(metadata["fresh_chunk_id"]),
        old_world=old_world,
        fresh_world=fresh_world,
        telemetry_rows=frozen_rows,
        telemetry_sim_times_s=times,
        telemetry_actual_poses=poses,
        display_sim_times_s=display_times,
        display_actual_poses=display_poses,
        activation_sim_time_s=activation_time,
        activation_pose_world_se2=activation_pose,
        activation_source=activation_source,
        activation_boundary_prepended=activation_prepended,
        observation_sim_time_s=observation_time,
        observation_pose_world_se2=observation_pose,
        model_ready_sim_time_s=ready_time,
        model_ready_pose_world_se2=ready_pose,
        p_sim_time_s=p_time,
        p_pose_world_se2=p_pose,
        switch_sim_time_s=switch_time,
        boundary_pose_world_se2=boundary,
        active_old_path_length_m=_path_length(display_poses),
        active_old_net_displacement_m=float(
            np.linalg.norm(display_poses[-1, :2] - display_poses[0, :2])
        ),
        active_old_yaw_change_rad=abs(
            _wrapped_angle(float(display_poses[-1, 2] - display_poses[0, 2]))
        ),
        source_sha256=source_hashes_proxy,
    )
