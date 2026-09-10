"""Deterministic saved-data selection and replay helpers for the DATA-02 GUI.

This module is deliberately Isaac-free.  It reads immutable DATA-02 artifacts,
verifies their recorded hashes, selects a high-motion transition, and maps a
presentation clock to adjacent saved telemetry samples.  It never invokes
LightNav, a controller, or simulation physics.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from reconciliation.data02_online_successive import validate_transition_artifact
from reconciliation.online_switch import sha256_file


DEFAULT_RUNS_RELATIVE = (
    Path("data/data02_online_successive_v1/data02-online-successive-primary-v1"),
    Path("data/data02_online_successive_v2/data02-online-successive-extension-v2"),
)
DEFAULT_SELECTION_OUTPUT_RELATIVE = Path("data/data02_collection_demo_selection")
DEFAULT_DEMO_OUTPUT_RELATIVE = Path("data/data02_high_motion_demo")
OLD_DEFAULT_RUN_ID = "data02-online-successive-extension-v2"
OLD_DEFAULT_EPISODE = "episode_000061"
OLD_DEFAULT_TRANSITION = 4
DEFAULT_DURATION_S = 15.0
MINIMUM_DURATION_S = 12.0
MAXIMUM_DURATION_S = 18.0
PRE_OBSERVATION_MARGIN_S = 1.0
POST_SWITCH_MARGIN_S = 2.0
TOP_INFERENCE_FRACTION = 0.10


@dataclass(frozen=True, slots=True)
class MotionCandidate:
    run_id: str
    source_run_path: str
    episode_id: str
    template_id: str
    variant_id: str
    transition_id: str
    transition_index: int
    status: str
    fresh_geometry: str
    t_obs_sim_s: float
    t_ready_sim_s: float
    t_switch_sim_s: float
    p_sim_time_s: float
    replay_start_sim_s: float
    replay_end_sim_s: float
    replay_sample_count: int
    inference_translation_m: float
    inference_path_length_m: float
    full_demo_path_length_m: float
    full_demo_net_displacement_m: float
    full_demo_yaw_change_rad: float
    abs_delta_v_des_mps: float
    abs_delta_omega_des_rps: float
    transition_json_sha256: str
    old_world_sha256: str
    fresh_world_sha256: str
    actual_sha256: str
    transition_telemetry_sha256: str
    episode_telemetry_sha256: str


@dataclass(frozen=True, slots=True)
class ScanResult:
    candidates: tuple[MotionCandidate, ...]
    eligible_transition_count: int
    exclusion_counts: Mapping[str, int]


@dataclass(frozen=True, slots=True)
class SelectionResult:
    selected: MotionCandidate
    previous_default: MotionCandidate
    ranked_candidates: tuple[MotionCandidate, ...]
    inference_top_decile_count: int
    inference_top_decile_threshold_m: float


@dataclass(frozen=True, slots=True)
class ReplayPhase:
    key: str
    title: str
    subtitle: str
    start_s: float
    end_s: float


@dataclass(frozen=True, slots=True)
class ReplayEvidence:
    candidate: MotionCandidate
    run: Path
    old_world: NDArray[np.float64]
    fresh_world: NDArray[np.float64]
    actual_sim_times_s: NDArray[np.float64]
    actual_poses: NDArray[np.float64]
    observation_pose: NDArray[np.float64]
    model_ready_pose: NDArray[np.float64]
    p_pose: NDArray[np.float64]
    boundary_pose: NDArray[np.float64]
    observation_rgb_path: Path | None
    observation_rgb_frame_index: int | None
    source_sha256: Mapping[str, str]


@dataclass(frozen=True, slots=True)
class InterpolatedPose:
    pose: NDArray[np.float64]
    lower_index: int
    upper_index: int
    alpha: float


@dataclass(frozen=True, slots=True)
class CameraFraming:
    eye_xyz: tuple[float, float, float]
    target_xyz: tuple[float, float, float]
    horizontal_fov_deg: float
    actual_bounds_xy: tuple[tuple[float, float], tuple[float, float]]
    all_geometry_bounds_xy: tuple[tuple[float, float], tuple[float, float]]
    estimated_actual_viewport_fraction: float
    estimated_all_geometry_viewport_fraction: float


_PHASES = (
    ("OLD_EXECUTING", "PHASE 1 — OLD EXECUTING", "saved pre-observation approach"),
    (
        "FRESH_INFERENCE",
        "PHASE 2 — FRESH INFERENCE — OLD STILL EXECUTING",
        "fixed observation ghost; OLD remains active",
    ),
    (
        "FRESH_READY_SWITCH",
        "PHASE 3 — FRESH READY / SWITCH AT B",
        "P is last pre-switch pose; B is switch boundary; raw FRESH stays observation-anchored",
    ),
    ("FRESH_ACTIVE", "PHASE 4 — FRESH ACTIVE", "saved post-switch robot motion"),
)
_DEFAULT_PHASE_BOUNDARIES_S = (0.0, 4.0, 9.0, 11.0, 15.0)


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
        raise ValueError(f"{path} has no rows")
    return rows


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _readonly(array: np.ndarray, name: str, *, columns: int = 3) -> NDArray[np.float64]:
    result = np.array(array, dtype=np.float64, copy=True)
    if result.ndim != 2 or result.shape[0] < 1 or result.shape[1] != columns:
        raise ValueError(f"{name} must be non-empty N x {columns}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains NaN/Inf")
    result.setflags(write=False)
    return result


def _readonly_pose(value: Sequence[float], name: str) -> NDArray[np.float64]:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite SE(2)")
    result.setflags(write=False)
    return result


def _telemetry_arrays(
    rows: Sequence[Mapping[str, str]],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    times = np.asarray(
        [_finite_float(row["sim_time_s"], "sim_time_s") for row in rows], dtype=np.float64
    )
    poses = np.asarray(
        [
            [
                _finite_float(row["actual_x"], "actual_x"),
                _finite_float(row["actual_y"], "actual_y"),
                _finite_float(row["actual_yaw"], "actual_yaw"),
            ]
            for row in rows
        ],
        dtype=np.float64,
    )
    if times.size < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError("episode telemetry times must be strictly increasing")
    if not np.all(np.isfinite(poses)):
        raise ValueError("episode telemetry poses contain NaN/Inf")
    return times, poses


def wrapped_angle(value: float) -> float:
    result = (float(value) + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if result == -math.pi and value > 0.0 else result


def _path_length(poses: np.ndarray) -> float:
    if len(poses) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum())


def _manifest_episode(run: Path, episode_id: str) -> Mapping[str, Any]:
    manifest = _strict_json(run / "collection_manifest.json")
    matches = [row for row in manifest["episodes"] if row["episode_id"] == episode_id]
    if len(matches) != 1:
        raise ValueError(f"manifest has no unique entry for {episode_id}")
    return matches[0]


def _verified_transition_metadata(
    run: Path,
    episode_id: str,
    transition_index: int,
    manifest_episode: Mapping[str, Any],
) -> tuple[Path, dict[str, Any], dict[str, str]]:
    transition_dir = (
        run / "episodes" / episode_id / "transitions" / f"transition_{transition_index:02d}"
    )
    metadata = validate_transition_artifact(transition_dir)
    transition_path = transition_dir / "transition.json"
    transition_hashes = list(manifest_episode["transition_metadata_sha256"])
    if transition_index >= len(transition_hashes):
        raise ValueError("transition index is outside collection manifest")
    transition_json_sha256 = sha256_file(transition_path)
    if transition_json_sha256 != str(transition_hashes[transition_index]):
        raise ValueError("transition JSON hash differs from collection manifest")
    artifacts = metadata["artifact_sha256"]
    return transition_dir, metadata, {
        "transition.json": transition_json_sha256,
        "derived/old_world.npy": str(artifacts["derived/old_world.npy"]),
        "derived/fresh_world.npy": str(artifacts["derived/fresh_world.npy"]),
        "actual.npy": str(artifacts["actual.npy"]),
        "transition_telemetry.csv": str(artifacts["telemetry.csv"]),
    }


def _candidate_from_saved(
    run: Path,
    episode_id: str,
    transition_index: int,
    manifest_episode: Mapping[str, Any],
    episode_times: NDArray[np.float64],
    episode_poses: NDArray[np.float64],
    episode_telemetry_sha256: str,
) -> MotionCandidate:
    transition_dir, metadata, source_hashes = _verified_transition_metadata(
        run, episode_id, transition_index, manifest_episode
    )
    if metadata["status"] != "ELIGIBLE_MOVING":
        raise ValueError("not ELIGIBLE_MOVING")
    timing = metadata["timing"]
    t_obs = _finite_float(timing["t_obs_sim_s"], "t_obs")
    t_ready = _finite_float(timing["t_ready_sim_s"], "t_ready")
    t_switch = _finite_float(timing["t_switch_sim_s"], "t_switch")
    p_time = _finite_float(
        metadata["pose_immediately_before_switch_P_sim_time_s"], "P time"
    )
    if timing.get("valid") is not True or not t_obs <= p_time < t_switch:
        raise ValueError("invalid P/B timing")
    if not t_obs <= t_ready <= t_switch:
        raise ValueError("invalid observation/ready/switch timing")

    pre_mask = (episode_times >= t_obs - PRE_OBSERVATION_MARGIN_S - 1e-9) & (
        episode_times < t_obs - 1e-9
    )
    post_mask = (episode_times > t_switch + 1e-9) & (
        episode_times <= t_switch + POST_SWITCH_MARGIN_S + 1e-9
    )
    replay_mask = (episode_times >= t_obs - PRE_OBSERVATION_MARGIN_S - 1e-9) & (
        episode_times <= t_switch + POST_SWITCH_MARGIN_S + 1e-9
    )
    inference_mask = (episode_times >= t_obs - 1e-9) & (
        episode_times <= t_ready + 1e-9
    )
    if not np.any(pre_mask):
        raise ValueError("missing pre-observation telemetry")
    if not np.any(post_mask):
        raise ValueError("missing post-switch telemetry")
    replay = episode_poses[replay_mask]
    replay_times = episode_times[replay_mask]
    inference = episode_poses[inference_mask]
    inference_times = episode_times[inference_mask]
    if len(replay) < 2 or len(inference) < 2:
        raise ValueError("replay or inference interval has too few samples")

    observation_pose = np.asarray(metadata["observation_pose_world_se2"], dtype=np.float64)
    ready_pose = np.asarray(metadata["model_ready_pose_world_se2"], dtype=np.float64)
    if observation_pose.shape != (3,) or ready_pose.shape != (3,):
        raise ValueError("observation/model-ready pose is malformed")
    if not np.all(np.isfinite(np.concatenate((observation_pose, ready_pose)))):
        raise ValueError("observation/model-ready pose is non-finite")
    if not math.isclose(float(inference_times[0]), t_obs, abs_tol=1e-6):
        raise ValueError("episode telemetry lacks exact observation sample")
    if not math.isclose(float(inference_times[-1]), t_ready, abs_tol=1e-6):
        raise ValueError("episode telemetry lacks exact model-ready sample")
    if not np.allclose(inference[0], observation_pose, rtol=0.0, atol=1e-6):
        raise ValueError("observation pose differs from episode telemetry")
    if not np.allclose(inference[-1], ready_pose, rtol=0.0, atol=1e-6):
        raise ValueError("model-ready pose differs from episode telemetry")

    full_path = _path_length(replay)
    if full_path <= 1e-9:
        raise ValueError("saved replay has no actual robot motion")
    command = metadata["metrics"]["command_discontinuity"]
    return MotionCandidate(
        run_id=run.name,
        source_run_path=str(run),
        episode_id=episode_id,
        template_id=str(metadata["template_id"]),
        variant_id=str(metadata["variant_id"]),
        transition_id=str(metadata["transition_id"]),
        transition_index=transition_index,
        status=str(metadata["status"]),
        fresh_geometry=str(metadata["metrics"]["fresh_geometry_bin"]),
        t_obs_sim_s=t_obs,
        t_ready_sim_s=t_ready,
        t_switch_sim_s=t_switch,
        p_sim_time_s=p_time,
        replay_start_sim_s=float(replay_times[0]),
        replay_end_sim_s=float(replay_times[-1]),
        replay_sample_count=int(len(replay_times)),
        inference_translation_m=float(np.linalg.norm(ready_pose[:2] - observation_pose[:2])),
        inference_path_length_m=_path_length(inference),
        full_demo_path_length_m=full_path,
        full_demo_net_displacement_m=float(np.linalg.norm(replay[-1, :2] - replay[0, :2])),
        full_demo_yaw_change_rad=abs(wrapped_angle(float(replay[-1, 2] - replay[0, 2]))),
        abs_delta_v_des_mps=abs(_finite_float(command["delta_v_mps"], "delta_v")),
        abs_delta_omega_des_rps=abs(
            _finite_float(command["delta_omega_rps"], "delta_omega")
        ),
        transition_json_sha256=source_hashes["transition.json"],
        old_world_sha256=source_hashes["derived/old_world.npy"],
        fresh_world_sha256=source_hashes["derived/fresh_world.npy"],
        actual_sha256=source_hashes["actual.npy"],
        transition_telemetry_sha256=source_hashes["transition_telemetry.csv"],
        episode_telemetry_sha256=episode_telemetry_sha256,
    )


def scan_high_motion_candidates(run_paths: Iterable[str | Path]) -> ScanResult:
    """Hash-verify and measure all usable ELIGIBLE_MOVING transitions."""

    candidates: list[MotionCandidate] = []
    exclusions: dict[str, int] = {}
    eligible_count = 0
    for run_value in sorted((Path(value).expanduser().resolve() for value in run_paths), key=str):
        manifest = _strict_json(run_value / "collection_manifest.json")
        for manifest_episode in sorted(manifest["episodes"], key=lambda row: row["episode_id"]):
            episode_id = str(manifest_episode["episode_id"])
            episode_dir = run_value / "episodes" / episode_id
            episode_telemetry_path = episode_dir / "telemetry.csv"
            episode_hash = sha256_file(episode_telemetry_path)
            if episode_hash != str(manifest_episode["telemetry_sha256"]):
                raise ValueError(f"episode telemetry hash mismatch: {episode_id}")
            rows = _csv_rows(episode_telemetry_path)
            episode_times, episode_poses = _telemetry_arrays(rows)
            transition_count = int(manifest_episode["transition_count"])
            for transition_index in range(transition_count):
                transition_json = _strict_json(
                    episode_dir
                    / "transitions"
                    / f"transition_{transition_index:02d}"
                    / "transition.json"
                )
                if transition_json.get("status") != "ELIGIBLE_MOVING":
                    continue
                eligible_count += 1
                try:
                    candidate = _candidate_from_saved(
                        run_value,
                        episode_id,
                        transition_index,
                        manifest_episode,
                        episode_times,
                        episode_poses,
                        episode_hash,
                    )
                except ValueError as error:
                    reason = str(error)
                    exclusions[reason] = exclusions.get(reason, 0) + 1
                    continue
                candidates.append(candidate)
    if not candidates:
        raise ValueError("no valid high-motion candidates")
    return ScanResult(tuple(candidates), eligible_count, dict(sorted(exclusions.items())))


def _identity(candidate: MotionCandidate) -> tuple[str, str, str]:
    return candidate.run_id, candidate.episode_id, candidate.transition_id


def select_high_motion_candidate(candidates: Sequence[MotionCandidate]) -> SelectionResult:
    """Apply the fixed top-decile constraint and deterministic motion ranking."""

    eligible = [candidate for candidate in candidates if candidate.status == "ELIGIBLE_MOVING"]
    if not eligible:
        raise ValueError("selection has no ELIGIBLE_MOVING candidates")
    inference_order = sorted(
        eligible, key=lambda candidate: (-candidate.inference_translation_m, _identity(candidate))
    )
    top_count = max(1, math.ceil(len(inference_order) * TOP_INFERENCE_FRACTION))
    threshold = inference_order[top_count - 1].inference_translation_m
    top_decile = [
        candidate
        for candidate in eligible
        if candidate.inference_translation_m >= threshold
    ]

    def rank_key(candidate: MotionCandidate) -> tuple[Any, ...]:
        return (
            -candidate.full_demo_path_length_m,
            -candidate.inference_translation_m,
            -candidate.full_demo_net_displacement_m,
            candidate.fresh_geometry == "STRAIGHT_LIKE",
            -candidate.abs_delta_omega_des_rps,
            *_identity(candidate),
        )

    top_ranked = sorted(top_decile, key=rank_key)
    below_ranked = sorted(
        (candidate for candidate in eligible if candidate not in top_decile), key=rank_key
    )
    old_matches = [
        candidate
        for candidate in eligible
        if candidate.run_id == OLD_DEFAULT_RUN_ID
        and candidate.episode_id == OLD_DEFAULT_EPISODE
        and candidate.transition_index == OLD_DEFAULT_TRANSITION
    ]
    if len(old_matches) != 1:
        raise ValueError("previous default is unavailable from valid candidates")
    return SelectionResult(
        selected=top_ranked[0],
        previous_default=old_matches[0],
        ranked_candidates=tuple(top_ranked + below_ranked),
        inference_top_decile_count=len(top_decile),
        inference_top_decile_threshold_m=float(threshold),
    )


def selection_is_clearly_higher_motion(selection: SelectionResult) -> bool:
    selected = selection.selected
    previous = selection.previous_default
    return (
        selected.inference_translation_m > previous.inference_translation_m
        and selected.full_demo_path_length_m > previous.full_demo_path_length_m
        and selected.full_demo_net_displacement_m > previous.full_demo_net_displacement_m
    )


def write_selection_outputs(
    scan: ScanResult, selection: SelectionResult, output_directory: str | Path
) -> tuple[Path, Path]:
    """Write reproducible generated ranking files outside immutable source runs."""

    output = Path(output_directory).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    ranking_path = output / "high_motion_candidates.csv"
    selected_path = output / "selected_transition.json"
    fieldnames = [
        "selection_rank",
        "inference_translation_rank",
        "inference_top_10_percent",
        *MotionCandidate.__dataclass_fields__,
    ]
    inference_rank = {
        _identity(candidate): index
        for index, candidate in enumerate(
            sorted(
                selection.ranked_candidates,
                key=lambda candidate: (-candidate.inference_translation_m, _identity(candidate)),
            ),
            start=1,
        )
    }
    with ranking_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for rank, candidate in enumerate(selection.ranked_candidates, start=1):
            row = asdict(candidate)
            row.update(
                {
                    "selection_rank": rank,
                    "inference_translation_rank": inference_rank[_identity(candidate)],
                    "inference_top_10_percent": (
                        candidate.inference_translation_m
                        >= selection.inference_top_decile_threshold_m
                    ),
                }
            )
            writer.writerow(row)
    document = {
        "schema": "DATA02HighMotionDemoSelection_v1",
        "selection_purpose": "visualization only; not scientific sample selection",
        "eligible_transition_count": scan.eligible_transition_count,
        "valid_candidate_count": len(scan.candidates),
        "exclusion_counts": dict(scan.exclusion_counts),
        "selection_rule": {
            "constraint": "inference_translation_m in top 10% by rank (boundary ties included)",
            "top_fraction": TOP_INFERENCE_FRACTION,
            "top_count_before_boundary_ties": max(
                1, math.ceil(len(scan.candidates) * TOP_INFERENCE_FRACTION)
            ),
            "top_count_with_boundary_ties": selection.inference_top_decile_count,
            "threshold_m": selection.inference_top_decile_threshold_m,
            "primary": "maximum full_demo_path_length_m",
            "tie_break": [
                "larger inference_translation_m",
                "larger full_demo_net_displacement_m",
                "non-straight FRESH geometry",
                "larger abs_delta_omega_des_rps",
                "lexicographic run/episode/transition ID",
            ],
        },
        "selected": asdict(selection.selected),
        "previous_default": asdict(selection.previous_default),
        "clearly_higher_motion_than_previous_default": selection_is_clearly_higher_motion(
            selection
        ),
        "ratios_selected_over_previous": {
            "inference_translation": selection.selected.inference_translation_m
            / selection.previous_default.inference_translation_m,
            "full_demo_path_length": selection.selected.full_demo_path_length_m
            / selection.previous_default.full_demo_path_length_m,
            "full_demo_net_displacement": selection.selected.full_demo_net_displacement_m
            / selection.previous_default.full_demo_net_displacement_m,
        },
    }
    selected_path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return ranking_path, selected_path


def select_and_write_defaults(repository_root: str | Path) -> SelectionResult:
    root = Path(repository_root).expanduser().resolve()
    scan = scan_high_motion_candidates(root / path for path in DEFAULT_RUNS_RELATIVE)
    selection = select_high_motion_candidate(scan.candidates)
    write_selection_outputs(scan, selection, root / DEFAULT_SELECTION_OUTPUT_RELATIVE)
    if not selection_is_clearly_higher_motion(selection):
        raise RuntimeError("selected transition is not clearly higher motion than previous default")
    return selection


def _load_candidate_for_replay(run: Path, episode_id: str, transition_index: int) -> MotionCandidate:
    manifest_episode = _manifest_episode(run, episode_id)
    telemetry_path = run / "episodes" / episode_id / "telemetry.csv"
    episode_hash = sha256_file(telemetry_path)
    if episode_hash != str(manifest_episode["telemetry_sha256"]):
        raise ValueError("episode telemetry hash mismatch")
    times, poses = _telemetry_arrays(_csv_rows(telemetry_path))
    return _candidate_from_saved(
        run,
        episode_id,
        transition_index,
        manifest_episode,
        times,
        poses,
        episode_hash,
    )


def load_replay_evidence(
    run: str | Path, episode_id: str, transition_index: int
) -> ReplayEvidence:
    """Load a replay window and mark every returned scientific array read-only."""

    run_path = Path(run).expanduser().resolve()
    candidate = _load_candidate_for_replay(run_path, episode_id, transition_index)
    episode_dir = run_path / "episodes" / episode_id
    transition_dir = episode_dir / "transitions" / f"transition_{transition_index:02d}"
    metadata = _strict_json(transition_dir / "transition.json")
    rows = _csv_rows(episode_dir / "telemetry.csv")
    times, poses = _telemetry_arrays(rows)
    mask = (times >= candidate.replay_start_sim_s - 1e-9) & (
        times <= candidate.replay_end_sim_s + 1e-9
    )
    replay_times = np.array(times[mask], copy=True)
    replay_poses = np.array(poses[mask], copy=True)
    replay_times.setflags(write=False)
    replay_poses.setflags(write=False)
    old_world = _readonly(
        np.load(transition_dir / "derived/old_world.npy", allow_pickle=False), "OLD world"
    )
    fresh_world = _readonly(
        np.load(transition_dir / "derived/fresh_world.npy", allow_pickle=False), "raw FRESH world"
    )
    source_hashes = {
        "transition.json": candidate.transition_json_sha256,
        "derived/old_world.npy": candidate.old_world_sha256,
        "derived/fresh_world.npy": candidate.fresh_world_sha256,
        "actual.npy": candidate.actual_sha256,
        "transition_telemetry.csv": candidate.transition_telemetry_sha256,
        "episode_telemetry.csv": candidate.episode_telemetry_sha256,
    }

    rgb_path: Path | None = None
    rgb_frame_index: int | None = None
    fresh_chunk_id = str(metadata["fresh_chunk_id"])
    chunk_metadata_path = episode_dir / "chunks" / fresh_chunk_id / "metadata.json"
    if chunk_metadata_path.is_file():
        chunk = _strict_json(chunk_metadata_path)
        rgb_frame_index = int(chunk["observation_rgb_frame_index"])
        rgb_rows = _csv_rows(episode_dir / "rgb_frames.csv")
        matches = [row for row in rgb_rows if int(row["frame_index"]) == rgb_frame_index]
        if len(matches) != 1:
            raise ValueError("saved observation RGB index is not unique")
        relative = Path(matches[0]["rgb_relative_path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("saved observation RGB path escapes episode")
        rgb_path = episode_dir / relative
        rgb_hash = sha256_file(rgb_path)
        if rgb_hash != matches[0]["rgb_file_sha256"]:
            raise ValueError("saved observation RGB hash mismatch")
        source_hashes["observation_rgb"] = rgb_hash

    return ReplayEvidence(
        candidate=candidate,
        run=run_path,
        old_world=old_world,
        fresh_world=fresh_world,
        actual_sim_times_s=replay_times,
        actual_poses=replay_poses,
        observation_pose=_readonly_pose(
            metadata["observation_pose_world_se2"], "observation pose"
        ),
        model_ready_pose=_readonly_pose(
            metadata["model_ready_pose_world_se2"], "model-ready pose"
        ),
        p_pose=_readonly_pose(
            metadata["pose_immediately_before_switch_P_world_se2"], "P pose"
        ),
        boundary_pose=_readonly_pose(
            metadata["switch_boundary_B_world_se2"], "B pose"
        ),
        observation_rgb_path=rgb_path,
        observation_rgb_frame_index=rgb_frame_index,
        source_sha256=source_hashes,
    )


def phase_schedule(duration_s: float = DEFAULT_DURATION_S) -> tuple[ReplayPhase, ...]:
    duration = _finite_float(duration_s, "duration")
    if not MINIMUM_DURATION_S <= duration <= MAXIMUM_DURATION_S:
        raise ValueError(f"duration must be in [{MINIMUM_DURATION_S}, {MAXIMUM_DURATION_S}] s")
    scale = duration / DEFAULT_DURATION_S
    bounds = tuple(value * scale for value in _DEFAULT_PHASE_BOUNDARIES_S)
    return tuple(
        ReplayPhase(*phase, bounds[index], bounds[index + 1])
        for index, phase in enumerate(_PHASES)
    )


def phase_at(presentation_time_s: float, duration_s: float = DEFAULT_DURATION_S) -> ReplayPhase:
    phases = phase_schedule(duration_s)
    value = min(max(_finite_float(presentation_time_s, "presentation time"), 0.0), duration_s)
    for phase in phases[:-1]:
        if value < phase.end_s:
            return phase
    return phases[-1]


def saved_time_for_presentation(
    presentation_time_s: float, duration_s: float, candidate: MotionCandidate
) -> float:
    phase = phase_at(presentation_time_s, duration_s)
    progress = min(
        max((presentation_time_s - phase.start_s) / (phase.end_s - phase.start_s), 0.0), 1.0
    )
    intervals = {
        "OLD_EXECUTING": (candidate.replay_start_sim_s, candidate.t_obs_sim_s),
        "FRESH_INFERENCE": (candidate.t_obs_sim_s, candidate.t_ready_sim_s),
        "FRESH_READY_SWITCH": (candidate.t_ready_sim_s, candidate.t_switch_sim_s),
        "FRESH_ACTIVE": (candidate.t_switch_sim_s, candidate.replay_end_sim_s),
    }
    start, end = intervals[phase.key]
    return float(start + progress * (end - start))


def interpolate_adjacent_saved_pose(
    first_pose: Sequence[float], second_pose: Sequence[float], alpha: float
) -> NDArray[np.float64]:
    first = np.asarray(first_pose, dtype=np.float64)
    second = np.asarray(second_pose, dtype=np.float64)
    amount = _finite_float(alpha, "interpolation alpha")
    if first.shape != (3,) or second.shape != (3,) or not 0.0 <= amount <= 1.0:
        raise ValueError("interpolation requires two SE(2) poses and alpha in [0, 1]")
    if not np.all(np.isfinite(np.concatenate((first, second)))):
        raise ValueError("interpolation pose contains NaN/Inf")
    result = np.empty(3, dtype=np.float64)
    result[:2] = first[:2] + amount * (second[:2] - first[:2])
    result[2] = wrapped_angle(first[2] + amount * wrapped_angle(second[2] - first[2]))
    result.setflags(write=False)
    return result


def interpolated_pose_at(evidence: ReplayEvidence, saved_time_s: float) -> InterpolatedPose:
    target = min(
        max(_finite_float(saved_time_s, "saved replay time"), evidence.actual_sim_times_s[0]),
        evidence.actual_sim_times_s[-1],
    )
    upper = int(np.searchsorted(evidence.actual_sim_times_s, target, side="right"))
    if upper == 0:
        lower = upper = 0
        alpha = 0.0
    elif upper >= len(evidence.actual_sim_times_s):
        lower = upper = len(evidence.actual_sim_times_s) - 1
        alpha = 0.0
    else:
        lower = upper - 1
        span = evidence.actual_sim_times_s[upper] - evidence.actual_sim_times_s[lower]
        alpha = float((target - evidence.actual_sim_times_s[lower]) / span)
    pose = (
        np.array(evidence.actual_poses[lower], copy=True)
        if lower == upper
        else interpolate_adjacent_saved_pose(
            evidence.actual_poses[lower], evidence.actual_poses[upper], alpha
        )
    )
    pose.setflags(write=False)
    return InterpolatedPose(pose, lower, upper, alpha)


def compute_camera_framing(
    actual_poses: np.ndarray,
    old_world: np.ndarray,
    fresh_world: np.ndarray,
    *,
    horizontal_fov_deg: float = 55.0,
) -> CameraFraming:
    """Compute a stable oblique view with the actual replay occupying about 64%."""

    actual = _readonly(actual_poses, "camera actual path")
    old = _readonly(old_world, "camera OLD path")
    fresh = _readonly(fresh_world, "camera FRESH path")
    all_xy = np.vstack((actual[:, :2], old[:, :2], fresh[:, :2]))
    actual_min, actual_max = actual[:, :2].min(axis=0), actual[:, :2].max(axis=0)
    all_min, all_max = all_xy.min(axis=0), all_xy.max(axis=0)
    actual_extent = max(float(np.linalg.norm(actual_max - actual_min)), 0.50)
    all_extent = max(float(np.linalg.norm(all_max - all_min)), actual_extent)
    fov = _finite_float(horizontal_fov_deg, "camera horizontal FOV")
    if not 30.0 <= fov <= 90.0:
        raise ValueError("camera horizontal FOV must be in [30, 90] degrees")
    actual_target = 0.64
    all_target = 0.82
    actual_distance = (actual_extent / 2.0) / math.tan(
        math.radians(fov * actual_target / 2.0)
    )
    all_distance = (all_extent / 2.0) / math.tan(math.radians(fov * all_target / 2.0))
    distance = max(actual_distance, all_distance, 1.55)
    center = (all_min + all_max) / 2.0
    direction = actual[-1, :2] - actual[0, :2]
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        direction = np.array([math.cos(actual[0, 2]), math.sin(actual[0, 2])])
    else:
        direction = direction / norm
    side = np.array([-direction[1], direction[0]])
    horizontal_view = 0.90 * side - 0.435889894 * direction
    horizontal_view /= np.linalg.norm(horizontal_view)
    elevation_rad = math.radians(43.0)
    horizontal_distance = distance * math.cos(elevation_rad)
    height = distance * math.sin(elevation_rad)
    eye_xy = center - horizontal_view * horizontal_distance
    target_z = 0.18
    eye = (float(eye_xy[0]), float(eye_xy[1]), float(target_z + height))
    target = (float(center[0]), float(center[1]), target_z)
    actual_fraction = math.degrees(2.0 * math.atan(actual_extent / (2.0 * distance))) / fov
    all_fraction = math.degrees(2.0 * math.atan(all_extent / (2.0 * distance))) / fov
    values = np.asarray((*eye, *target, actual_fraction, all_fraction), dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("computed camera framing is non-finite")
    return CameraFraming(
        eye_xyz=eye,
        target_xyz=target,
        horizontal_fov_deg=fov,
        actual_bounds_xy=(tuple(map(float, actual_min)), tuple(map(float, actual_max))),
        all_geometry_bounds_xy=(tuple(map(float, all_min)), tuple(map(float, all_max))),
        estimated_actual_viewport_fraction=float(actual_fraction),
        estimated_all_geometry_viewport_fraction=float(all_fraction),
    )


def saved_only_contract() -> dict[str, bool]:
    return {
        "saved_only": True,
        "lightnav_inference": False,
        "controller_execution": False,
        "physics_reexecution": False,
        "spatial_scaling": False,
        "adjacent_sample_display_interpolation_only": True,
    }


def _print_selection(selection: SelectionResult) -> None:
    selected = selection.selected
    previous = selection.previous_default
    print(f"SELECTED_RUN={selected.source_run_path}")
    print(f"SELECTED_EPISODE={selected.episode_id}")
    print(f"SELECTED_TRANSITION={selected.transition_index}")
    for prefix, candidate in (("", selected), ("PREVIOUS_DEFAULT_", previous)):
        print(f"{prefix}INFERENCE_TRANSLATION_M={candidate.inference_translation_m:.15g}")
        print(f"{prefix}INFERENCE_PATH_LENGTH_M={candidate.inference_path_length_m:.15g}")
        print(f"{prefix}FULL_DEMO_PATH_LENGTH_M={candidate.full_demo_path_length_m:.15g}")
        print(
            f"{prefix}FULL_DEMO_NET_DISPLACEMENT_M="
            f"{candidate.full_demo_net_displacement_m:.15g}"
        )
        print(f"{prefix}FULL_DEMO_YAW_CHANGE_RAD={candidate.full_demo_yaw_change_rad:.15g}")
        print(f"{prefix}FRESH_GEOMETRY={candidate.fresh_geometry}")
        print(f"{prefix}ABS_DELTA_V_DES={candidate.abs_delta_v_des_mps:.15g}")
        print(f"{prefix}ABS_DELTA_OMEGA_DES={candidate.abs_delta_omega_des_rps:.15g}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Select DATA-02 high-motion saved demo evidence")
    parser.add_argument("--repository-root", type=Path, default=Path.cwd())
    args = parser.parse_args()
    selection = select_and_write_defaults(args.repository_root)
    _print_selection(selection)


if __name__ == "__main__":
    main()
