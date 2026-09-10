"""Saved-evidence contract for the short DATA-02 collection GUI demo.

The demo deliberately separates presentation time from the immutable scientific
timestamps in DATA-02.  Robot poses and OLD/FRESH paths are loaded from saved
artifacts only; this module performs no inference, control, or physics execution.
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from reconciliation.online_switch import sha256_file


DEFAULT_RUN_RELATIVE = Path(
    "data/data02_online_successive_v2/data02-online-successive-extension-v2"
)
DEFAULT_EPISODE = "episode_000061"
DEFAULT_TRANSITION = 4
DEFAULT_DURATION_S = 12.0
MINIMUM_DURATION_S = 10.0
MAXIMUM_DURATION_S = 15.0


@dataclass(frozen=True, slots=True)
class DemoPhase:
    key: str
    terminal_name: str
    title: str
    subtitle: str
    start_s: float
    end_s: float


@dataclass(frozen=True, slots=True)
class ScientificTiming:
    replay_start_sim_s: float
    t_obs_sim_s: float
    t_ready_sim_s: float
    t_switch_sim_s: float
    post_switch_end_sim_s: float
    model_reported_s: float
    effective_latency_s: float


@dataclass(frozen=True, slots=True)
class SavedDemoEvidence:
    run: Path
    episode_id: str
    transition_index: int
    transition_id: str
    old_chunk_id: str
    fresh_chunk_id: str
    old_world: NDArray[np.float64]
    fresh_world: NDArray[np.float64]
    actual_poses: NDArray[np.float64]
    actual_sim_times_s: NDArray[np.float64]
    observation_pose: NDArray[np.float64]
    p_pose: NDArray[np.float64]
    boundary_pose: NDArray[np.float64]
    timing: ScientificTiming
    motion_during_inference_m: float
    rgb_frame_index: int
    rgb_path: Path
    rgb_relative_path: str
    source_sha256: Mapping[str, str]


_PHASE_DEFINITIONS = (
    ("OLD_EXECUTING", "OLD_EXECUTING", "PHASE 1 — OLD EXECUTING", "OLD is already active"),
    (
        "FRESH_IN_FLIGHT",
        "FRESH_IN_FLIGHT",
        "PHASE 2 — FRESH INFERENCE",
        "OLD CONTINUES EXECUTING",
    ),
    ("FRESH_READY", "FRESH_READY", "PHASE 3 — FRESH READY", "P and B record switch context"),
    ("RAW_SWITCH", "RAW_SWITCH", "PHASE 4 — OLD → FRESH RAW SWITCH", "raw FRESH is activated unchanged"),
    ("FRESH_ACTIVE", "FRESH_ACTIVE", "PHASE 5 — FRESH ACTIVE", "saved post-switch motion"),
)
_REFERENCE_BREAKPOINTS_S = (0.0, 2.0, 5.0, 7.0, 10.0, 12.0)


def phase_schedule(duration_s: float = DEFAULT_DURATION_S) -> tuple[DemoPhase, ...]:
    """Return the ordered five-phase presentation schedule."""

    duration = float(duration_s)
    if not math.isfinite(duration) or not MINIMUM_DURATION_S <= duration <= MAXIMUM_DURATION_S:
        raise ValueError(
            f"demo duration must be finite and in [{MINIMUM_DURATION_S}, {MAXIMUM_DURATION_S}] seconds"
        )
    scale = duration / DEFAULT_DURATION_S
    boundaries = tuple(value * scale for value in _REFERENCE_BREAKPOINTS_S)
    return tuple(
        DemoPhase(*definition, boundaries[index], boundaries[index + 1])
        for index, definition in enumerate(_PHASE_DEFINITIONS)
    )


def phase_at(presentation_time_s: float, duration_s: float = DEFAULT_DURATION_S) -> DemoPhase:
    """Select a phase, clamping presentation time to the demo interval."""

    phases = phase_schedule(duration_s)
    value = min(max(float(presentation_time_s), 0.0), float(duration_s))
    for phase in phases[:-1]:
        if value < phase.end_s:
            return phase
    return phases[-1]


def phase_progress(presentation_time_s: float, phase: DemoPhase) -> float:
    span = phase.end_s - phase.start_s
    if span <= 0.0:
        raise ValueError("phase duration must be positive")
    return min(max((float(presentation_time_s) - phase.start_s) / span, 0.0), 1.0)


def scientific_time_for_presentation(
    presentation_time_s: float,
    duration_s: float,
    timing: ScientificTiming,
) -> float:
    """Map presentation time monotonically onto recorded scientific time.

    Phase 3 is an intentional visual hold at the saved ready/switch instant.  No
    scientific timestamp is changed; only the rate at which saved states are shown
    changes.  With simultaneous ready and switch timestamps this remains naturally
    non-decreasing.
    """

    phase = phase_at(presentation_time_s, duration_s)
    progress = phase_progress(presentation_time_s, phase)
    post_split = timing.t_switch_sim_s + 0.6 * (
        timing.post_switch_end_sim_s - timing.t_switch_sim_s
    )
    intervals = {
        "OLD_EXECUTING": (timing.replay_start_sim_s, timing.t_obs_sim_s),
        "FRESH_IN_FLIGHT": (timing.t_obs_sim_s, timing.t_ready_sim_s),
        "FRESH_READY": (timing.t_ready_sim_s, timing.t_switch_sim_s),
        "RAW_SWITCH": (timing.t_switch_sim_s, post_split),
        "FRESH_ACTIVE": (post_split, timing.post_switch_end_sim_s),
    }
    start, end = intervals[phase.key]
    return float(start + progress * (end - start))


def saved_sample_index(evidence: SavedDemoEvidence, scientific_time_s: float) -> int:
    """Return the last saved pose at or before a scientific timestamp."""

    index = int(np.searchsorted(evidence.actual_sim_times_s, scientific_time_s, side="right") - 1)
    return min(max(index, 0), evidence.actual_sim_times_s.size - 1)


def _strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
    )
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} has no telemetry rows")
    return rows


def _finite_pose(value: Sequence[float], name: str) -> NDArray[np.float64]:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be a finite SE(2) pose")
    pose.setflags(write=False)
    return pose


def _trajectory(path: Path, name: str) -> NDArray[np.float64]:
    array = np.asarray(np.load(path, allow_pickle=False), dtype=np.float64)
    if array.ndim != 2 or array.shape[0] < 1 or array.shape[1] != 3:
        raise ValueError(f"{name} must be non-empty N x 3")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN/Inf")
    result = np.array(array, copy=True)
    result.setflags(write=False)
    return result


def _float(row: Mapping[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite telemetry value: {key}")
    return value


def _verify_file_hash(path: Path, expected: str) -> str:
    actual = sha256_file(path)
    if actual != expected:
        raise ValueError(f"saved artifact hash mismatch: {path}")
    return actual


def load_saved_demo(
    run: str | Path,
    episode_id: str = DEFAULT_EPISODE,
    transition_index: int = DEFAULT_TRANSITION,
) -> SavedDemoEvidence:
    """Load and validate one saved DATA-02 transition without writing to it."""

    run_path = Path(run).expanduser().resolve()
    if not episode_id.startswith("episode_") or not episode_id[8:].isdigit():
        raise ValueError("episode must have the form episode_000000")
    if transition_index < 0:
        raise ValueError("transition index must be non-negative")
    episode = run_path / "episodes" / episode_id
    transition_dir = episode / "transitions" / f"transition_{transition_index:02d}"
    transition_path = transition_dir / "transition.json"
    transition = _strict_json(transition_path)
    if transition.get("episode_id") != episode_id or int(transition.get("transition_index", -1)) != transition_index:
        raise ValueError("saved transition identity does not match the requested selection")
    old_chunk_id = str(transition["old_chunk_id"])
    fresh_chunk_id = str(transition["fresh_chunk_id"])

    old_path = transition_dir / "derived/old_world.npy"
    fresh_path = transition_dir / "derived/fresh_world.npy"
    actual_path = transition_dir / "actual.npy"
    transition_telemetry_path = transition_dir / "telemetry.csv"
    artifacts = transition["artifact_sha256"]
    source_sha256 = {
        "transition.json": sha256_file(transition_path),
        "derived/old_world.npy": _verify_file_hash(old_path, str(artifacts["derived/old_world.npy"])),
        "derived/fresh_world.npy": _verify_file_hash(fresh_path, str(artifacts["derived/fresh_world.npy"])),
        "actual.npy": _verify_file_hash(actual_path, str(artifacts["actual.npy"])),
        "transition_telemetry.csv": _verify_file_hash(
            transition_telemetry_path, str(artifacts["telemetry.csv"])
        ),
    }
    old_world = _trajectory(old_path, "saved OLD world path")
    fresh_world = _trajectory(fresh_path, "saved raw FRESH world path")
    transition_actual = _trajectory(actual_path, "saved transition actual poses")
    transition_rows = _csv_rows(transition_telemetry_path)
    if len(transition_rows) != len(transition_actual):
        raise ValueError("saved transition pose and telemetry lengths differ")

    timing_document = transition["timing"]
    t_obs = float(timing_document["t_obs_sim_s"])
    t_ready = float(timing_document["t_ready_sim_s"])
    t_switch = float(timing_document["t_switch_sim_s"])
    model_reported = float(timing_document["model_reported_s"])
    effective_latency = float(timing_document["tau_effective_s"])
    p_time = float(transition["pose_immediately_before_switch_P_sim_time_s"])
    if not all(math.isfinite(value) for value in (
        t_obs, t_ready, t_switch, model_reported, effective_latency, p_time
    )):
        raise ValueError("saved transition contains non-finite timing")
    if not t_obs <= t_ready <= t_switch:
        raise ValueError("saved observation/ready/switch ordering is invalid")
    if not p_time < t_switch:
        raise ValueError("saved P must strictly precede saved B/switch")

    transition_times = np.asarray([_float(row, "sim_time_s") for row in transition_rows])
    if not np.all(np.diff(transition_times) > 0.0):
        raise ValueError("saved transition telemetry time must be strictly increasing")
    csv_transition_actual = np.asarray([
        [_float(row, "actual_x"), _float(row, "actual_y"), _float(row, "actual_yaw")]
        for row in transition_rows
    ])
    if not np.allclose(csv_transition_actual, transition_actual, rtol=0.0, atol=1e-12):
        raise ValueError("saved transition actual.npy differs from its telemetry")
    inflight = [
        row for row in transition_rows
        if row["phase"] == "FRESH_IN_FLIGHT" and t_obs <= _float(row, "sim_time_s") <= t_switch
    ]
    if not inflight:
        raise ValueError("saved transition has no FRESH_IN_FLIGHT interval")
    if any(row["active_chunk_id"] != old_chunk_id or int(row["fresh_request_in_flight"]) != 1 for row in inflight):
        raise ValueError("OLD must remain active throughout saved FRESH inference")
    inflight_start = np.asarray([
        _float(inflight[0], "actual_x"), _float(inflight[0], "actual_y")
    ])
    inflight_end = np.asarray([
        _float(inflight[-1], "actual_x"), _float(inflight[-1], "actual_y")
    ])
    if float(np.linalg.norm(inflight_end - inflight_start)) <= 0.0:
        raise ValueError("saved robot did not move during FRESH inference")

    episode_telemetry_path = episode / "telemetry.csv"
    episode_rows = _csv_rows(episode_telemetry_path)
    source_sha256["episode_telemetry.csv"] = sha256_file(episode_telemetry_path)
    post_rows: list[dict[str, str]] = []
    after_switch = False
    for row in episode_rows:
        sample_time = _float(row, "sim_time_s")
        if sample_time <= t_switch:
            continue
        after_switch = True
        if row["active_chunk_id"] != fresh_chunk_id or int(row["fresh_request_in_flight"]) != 0:
            break
        post_rows.append(row)
    if not after_switch or not post_rows:
        raise ValueError("saved episode has no post-switch FRESH-active motion")
    post_times = np.asarray([_float(row, "sim_time_s") for row in post_rows])
    post_actual = np.asarray([
        [_float(row, "actual_x"), _float(row, "actual_y"), _float(row, "actual_yaw")]
        for row in post_rows
    ])
    combined_times = np.concatenate((transition_times, post_times)).astype(np.float64)
    combined_actual = np.vstack((transition_actual, post_actual)).astype(np.float64)
    if not np.all(np.diff(combined_times) > 0.0):
        raise ValueError("combined saved replay time must be strictly increasing")
    combined_times.setflags(write=False)
    combined_actual.setflags(write=False)

    fresh_chunk_metadata_path = episode / "chunks" / fresh_chunk_id / "metadata.json"
    fresh_chunk = _strict_json(fresh_chunk_metadata_path)
    source_sha256["fresh_chunk_metadata.json"] = sha256_file(fresh_chunk_metadata_path)
    if fresh_chunk.get("chunk_id") != fresh_chunk_id:
        raise ValueError("saved FRESH chunk identity mismatch")
    rgb_frame_index = int(fresh_chunk["observation_rgb_frame_index"])
    if float(fresh_chunk["observation_sim_time_s"]) != t_obs:
        raise ValueError("FRESH chunk observation time differs from transition")
    rgb_index_path = episode / "rgb_frames.csv"
    rgb_rows = _csv_rows(rgb_index_path)
    source_sha256["rgb_frames.csv"] = sha256_file(rgb_index_path)
    matches = [row for row in rgb_rows if int(row["frame_index"]) == rgb_frame_index]
    if len(matches) != 1 or _float(matches[0], "sim_time_s") != t_obs:
        raise ValueError("exact saved FRESH observation RGB index/time is unavailable")
    rgb_relative = Path(matches[0]["rgb_relative_path"])
    if rgb_relative.is_absolute() or ".." in rgb_relative.parts:
        raise ValueError("saved RGB relative path escapes its episode")
    rgb_path = episode / rgb_relative
    source_sha256["observation_rgb"] = _verify_file_hash(
        rgb_path, str(matches[0]["rgb_file_sha256"])
    )

    observation_pose = _finite_pose(transition["observation_pose_world_se2"], "observation pose")
    p_pose = _finite_pose(transition["pose_immediately_before_switch_P_world_se2"], "P pose")
    boundary_pose = _finite_pose(transition["switch_boundary_B_world_se2"], "B pose")
    if not np.allclose(transition_actual[-1], boundary_pose, rtol=0.0, atol=1e-12):
        raise ValueError("saved transition actual endpoint differs from B")
    motion = float(transition["metrics"]["observation_to_model_ready_translation_m"])
    if not math.isfinite(motion) or motion < 0.0:
        raise ValueError("invalid saved motion-during-inference metric")

    return SavedDemoEvidence(
        run=run_path,
        episode_id=episode_id,
        transition_index=transition_index,
        transition_id=str(transition["transition_id"]),
        old_chunk_id=old_chunk_id,
        fresh_chunk_id=fresh_chunk_id,
        old_world=old_world,
        fresh_world=fresh_world,
        actual_poses=combined_actual,
        actual_sim_times_s=combined_times,
        observation_pose=observation_pose,
        p_pose=p_pose,
        boundary_pose=boundary_pose,
        timing=ScientificTiming(
            replay_start_sim_s=float(transition_times[0]),
            t_obs_sim_s=t_obs,
            t_ready_sim_s=t_ready,
            t_switch_sim_s=t_switch,
            post_switch_end_sim_s=float(post_times[-1]),
            model_reported_s=model_reported,
            effective_latency_s=effective_latency,
        ),
        motion_during_inference_m=motion,
        rgb_frame_index=rgb_frame_index,
        rgb_path=rgb_path,
        rgb_relative_path=str(rgb_relative),
        source_sha256=dict(source_sha256),
    )


def saved_only_contract() -> dict[str, bool]:
    """Machine-testable execution boundary for the GUI layer."""

    return {
        "saved_only": True,
        "lightnav_inference": False,
        "physics_reexecution": False,
        "controller_execution": False,
        "scientific_timestamps_modified": False,
    }
