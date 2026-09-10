"""Read-only DATA-02 reconstruction and frozen EXP-02D analysis contracts.

This module contains no optimizer and no Isaac Sim dependency.  It verifies the
immutable combined DATA-02 index and its two source runs, reconstructs the exact
one-step raw-FRESH :class:`TrajectoryFollower` decision at the saved boundary,
and provides the pair-balanced/statistical classification helpers used by
EXP-02D.

LightNav rows remain untimed spatial SE(2) waypoints.  Times loaded here refer
only to saved observation, readiness, controller-boundary, and execution events.
"""

from __future__ import annotations

from collections import defaultdict
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray
import yaml

from reconciliation.controllers.trajectory_follower import (
    FollowerCommand,
    FollowerConfig,
    TrajectoryFollower,
)
from reconciliation.data02_online_successive import (
    chunk_content_sha256,
    difficulty_bin,
    observation_anchored_paths,
    ordered_pair_sha256,
    strict_json,
    validate_transition_artifact,
)
from reconciliation.online_switch import sha256_file


FloatArray = NDArray[np.float64]
DEFAULT_COMBINED_CORPUS = Path(
    "data/data02_combined_v1_v2/data02-combined-v1-v2-final"
)
EXPECTED_COMBINED_DECISION = "DATA02_COMBINED_DIVERSITY_INSUFFICIENT"
INPUT_RECONSTRUCTION_FAILED = "INPUT_RECONSTRUCTION_FAILED"
EXP02D_INPUT_RECONSTRUCTION_FAILED = "EXP02D_INPUT_RECONSTRUCTION_FAILED"
RAW_COMMAND_RECONSTRUCTION_ATOL = 1.0e-12
COMMAND_V_SCALE_MPS = 0.15
COMMAND_OMEGA_SCALE_RPS = 0.30
TURNING_GEOMETRIES = frozenset(("POSITIVE_TURNING", "NEGATIVE_TURNING"))


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _nonnegative(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _strict_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def _within(root: Path, value: str | Path, name: str) -> Path:
    root_value = root.resolve()
    path = Path(value)
    result = path.resolve() if path.is_absolute() else (root_value / path).resolve()
    if not result.is_relative_to(root_value):
        raise ValueError(f"{name} escapes repository root: {value}")
    return result


def _readonly_array(value: Any, name: str, *, ndim: int, columns: int | None = None) -> np.ndarray:
    source = np.asarray(value)
    if source.ndim != ndim or (columns is not None and source.shape[-1] != columns):
        suffix = f" with {columns} columns" if columns is not None else ""
        raise ValueError(f"{name} must have {ndim} dimensions{suffix}")
    if source.size == 0 or not np.issubdtype(source.dtype, np.number) or not np.all(np.isfinite(source)):
        raise ValueError(f"{name} must be non-empty, numeric, and finite")
    result = np.array(source, copy=True, order="C")
    result.setflags(write=False)
    return result


def _readonly_pose(value: Any, name: str) -> FloatArray:
    try:
        result = np.array(value, dtype=np.float64, copy=True, order="C")
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must have shape (3,)")
    result.setflags(write=False)
    return result


def _readonly_trajectory(value: Any, name: str) -> FloatArray:
    source = _readonly_array(value, name, ndim=2, columns=3)
    result = np.array(source, dtype=np.float64, copy=True, order="C")
    result.setflags(write=False)
    return result


def _readonly_command(value: Any, name: str) -> FloatArray:
    try:
        result = np.array(value, dtype=np.float64, copy=True, order="C")
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must have shape (2,)")
    result.setflags(write=False)
    return result


def _json_records(path: Path) -> list[dict[str, Any]]:
    document = strict_json(path)
    records = document.get("records")
    if not isinstance(records, list) or any(not isinstance(row, dict) for row in records):
        raise ValueError(f"{path} must contain a records list")
    return records


@dataclass(frozen=True, slots=True)
class SourceCorpus:
    cohort_id: str
    run: Path
    description: Mapping[str, Any]
    config: Mapping[str, Any]
    collection_manifest: Mapping[str, Any]
    source_records: Mapping[str, Mapping[str, Any]]
    manifest_episodes: Mapping[str, Mapping[str, Any]]


@dataclass(frozen=True, slots=True)
class Exp02DCorpusIndex:
    repository_root: Path
    combined_root: Path
    combined_manifest_sha256: str
    combined_manifest: Mapping[str, Any]
    eligible_records: tuple[Mapping[str, Any], ...]
    sources: Mapping[str, SourceCorpus]

    @property
    def eligible_count(self) -> int:
        return len(self.eligible_records)


@dataclass(frozen=True, slots=True, eq=False)
class Exp02DTransitionInput:
    corpus_transition_id: str
    cohort_id: str
    source_run: Path
    source_transition: Path
    source_transition_sha256: str
    source_artifact_sha256: Mapping[str, str]
    episode_id: str
    transition_id: str
    transition_index: int
    template_id: str
    variant_id: str
    semantic_family: str
    physical_region: str
    old_chunk_id: str
    fresh_chunk_id: str
    ordered_raw_pair_sha256: str
    fresh_geometry_bin: str
    difficulty_bin: str
    old_raw: np.ndarray
    old_world: FloatArray
    fresh_raw: np.ndarray
    fresh_world: FloatArray
    old_observation_pose: FloatArray
    observation_pose: FloatArray
    pose_before_boundary: FloatArray
    boundary_pose: FloatArray
    t_obs_sim_s: float
    t_ready_sim_s: float
    t_switch_sim_s: float
    host_latency_s: float
    effective_latency_s: float
    old_desired_command: FloatArray
    raw_fresh_first_desired_command: FloatArray
    follower_config: FollowerConfig
    k_fresh: int
    q_fresh: int
    raw_follower_command: FollowerCommand
    raw_follower_goal_reached: bool
    metrics: Mapping[str, Any]

    @property
    def q_relative(self) -> int:
        return self.q_fresh - self.k_fresh

    @property
    def old_geometry(self) -> Mapping[str, Any]:
        return self.metrics["old_geometry"]

    @property
    def fresh_geometry(self) -> Mapping[str, Any]:
        return self.metrics["fresh_geometry"]

    @property
    def inference_translation_m(self) -> float:
        return float(self.metrics["observation_to_model_ready_translation_m"])

    @property
    def observation_to_boundary_translation_m(self) -> float:
        return float(self.metrics["observation_to_boundary_translation_m"])


class Exp02DInputError(ValueError):
    """A transition-specific source or follower reconstruction failure."""

    def __init__(self, corpus_transition_id: str, message: str) -> None:
        self.corpus_transition_id = str(corpus_transition_id)
        self.status = INPUT_RECONSTRUCTION_FAILED
        super().__init__(f"{self.corpus_transition_id}: {message}")


@dataclass(frozen=True, slots=True)
class ReconstructionFailure:
    corpus_transition_id: str
    status: str
    reason: str


@dataclass(frozen=True, slots=True)
class ReconstructedCorpus:
    index: Exp02DCorpusIndex
    transitions: tuple[Exp02DTransitionInput, ...]
    failures: tuple[ReconstructionFailure, ...]

    @property
    def valid_count(self) -> int:
        return len(self.transitions)

    def require_complete(self) -> None:
        if self.failures:
            raise RuntimeError(
                f"{EXP02D_INPUT_RECONSTRUCTION_FAILED}: "
                f"{len(self.failures)}/{self.index.eligible_count} transitions failed"
            )


def _validate_source_corpus(
    repository_root: Path, description: Mapping[str, Any]
) -> SourceCorpus:
    cohort = str(description.get("cohort_id", ""))
    if cohort not in ("v1", "v2"):
        raise ValueError(f"unexpected DATA-02 source cohort: {cohort!r}")
    run = _within(repository_root, str(description["run_path"]), f"{cohort} run_path")
    files = {
        "metadata_sha256": run / "metadata.json",
        "config_snapshot_sha256": run / "config_snapshot.yaml",
        "protocol_sha256": run / "protocol.json",
        "collection_manifest_sha256": run / "collection_manifest.json",
        "transition_index_sha256": run / "summary/transition_index.json",
        "strict_validation_sha256": run / "summary/validation.json",
    }
    for key, path in files.items():
        if sha256_file(path) != str(description[key]):
            raise ValueError(f"{cohort} provenance hash mismatch: {path}")
    validation = strict_json(files["strict_validation_sha256"])
    if validation.get("valid") is not True or description.get("strict_validation_valid") is not True:
        raise ValueError(f"{cohort} source strict validation is not valid")
    metadata = strict_json(files["metadata_sha256"])
    if str(metadata.get("collector_git_sha")) != str(description["collector_git_sha"]):
        raise ValueError(f"{cohort} collector git SHA mismatch")
    config = _strict_yaml(files["config_snapshot_sha256"])
    collection = strict_json(files["collection_manifest_sha256"])
    source_rows = _json_records(files["transition_index_sha256"])
    by_id: dict[str, Mapping[str, Any]] = {}
    for row in source_rows:
        identifier = str(row.get("transition_id", ""))
        if not identifier or identifier in by_id:
            raise ValueError(f"{cohort} source transition IDs are not unique")
        by_id[identifier] = row
    episodes: dict[str, Mapping[str, Any]] = {}
    for row in collection.get("episodes", []):
        identifier = str(row.get("episode_id", ""))
        if not identifier or identifier in episodes:
            raise ValueError(f"{cohort} collection episode IDs are not unique")
        episodes[identifier] = row
    if int(collection.get("episode_count", -1)) != len(episodes):
        raise ValueError(f"{cohort} collection manifest episode count mismatch")
    return SourceCorpus(cohort, run, dict(description), config, collection, by_id, episodes)


def load_exp02d_corpus_index(
    repository_root: str | Path,
    combined_root: str | Path = DEFAULT_COMBINED_CORPUS,
    *,
    integrity_config: Mapping[str, Any] | None = None,
) -> Exp02DCorpusIndex:
    """Verify immutable combined/source provenance and return eligible records only."""

    repository = Path(repository_root).expanduser().resolve()
    combined = _within(repository, combined_root, "combined corpus")
    manifest_path = combined / "combined_manifest.json"
    manifest = strict_json(manifest_path)
    integrity = {} if integrity_config is None else dict(integrity_config)
    pinned_hashes = {
        "combined_manifest_sha256": manifest_path,
        "all_transition_index_sha256": combined / "all_transition_index.json",
    }
    for key, path in pinned_hashes.items():
        if key in integrity and sha256_file(path) != str(integrity[key]):
            raise ValueError(f"EXP-02D pinned source hash mismatch: {key}")
    if manifest.get("schema") != "DATA02_CombinedReferenceCorpus_v1":
        raise ValueError("unexpected combined DATA-02 schema")
    artifacts = manifest.get("artifact_sha256")
    if not isinstance(artifacts, dict) or "all_transition_index.json" not in artifacts:
        raise ValueError("combined manifest has no complete artifact map")
    for relative, expected in artifacts.items():
        path = _within(combined, str(relative), "combined artifact")
        if sha256_file(path) != str(expected):
            raise ValueError(f"combined artifact hash mismatch: {relative}")
    readiness = strict_json(combined / "readiness.json")
    if readiness.get("status") != EXPECTED_COMBINED_DECISION:
        raise ValueError("historical combined DATA-02 decision changed")
    descriptions = manifest.get("sources")
    if not isinstance(descriptions, list) or len(descriptions) != 2:
        raise ValueError("combined corpus must identify exactly two source cohorts")
    sources = {
        source.cohort_id: source
        for source in (
            _validate_source_corpus(repository, description) for description in descriptions
        )
    }
    if set(sources) != {"v1", "v2"}:
        raise ValueError("combined source cohorts must be exactly v1 and v2")
    for cohort, source in sources.items():
        for suffix, relative in (
            ("collection_manifest_sha256", "collection_manifest.json"),
            ("config_snapshot_sha256", "config_snapshot.yaml"),
        ):
            key = f"{cohort}_{suffix}"
            if key in integrity and sha256_file(source.run / relative) != str(integrity[key]):
                raise ValueError(f"EXP-02D pinned source hash mismatch: {key}")
    records = _json_records(combined / "all_transition_index.json")
    if len(records) != int(manifest.get("attempt_count", -1)):
        raise ValueError("combined attempt count does not match its index")
    eligible = [row for row in records if row.get("status") == "ELIGIBLE_MOVING"]
    if len(eligible) != int(manifest.get("eligible_count", -1)):
        raise ValueError("combined eligible count does not match its index")
    if "expected_eligible_moving" in integrity and len(eligible) != int(
        integrity["expected_eligible_moving"]
    ):
        raise ValueError("recomputed ELIGIBLE_MOVING count differs from frozen integrity config")
    if "expected_unique_ordered_raw_pairs" in integrity:
        pair_count = len({str(row["ordered_raw_pair_sha256"]) for row in eligible})
        if pair_count != int(integrity["expected_unique_ordered_raw_pairs"]):
            raise ValueError("recomputed ordered raw-pair count differs from frozen integrity config")
    identities = [str(row.get("corpus_transition_id", "")) for row in eligible]
    if any(not identifier for identifier in identities) or len(set(identities)) != len(identities):
        raise ValueError("eligible corpus transition identities are empty or duplicated")
    return Exp02DCorpusIndex(
        repository,
        combined,
        sha256_file(manifest_path),
        manifest,
        tuple(dict(row) for row in eligible),
        sources,
    )


def _follower_config(source: SourceCorpus) -> FollowerConfig:
    values = source.config.get("follower")
    if not isinstance(values, Mapping):
        raise ValueError(f"{source.cohort_id} source follower config is missing")
    return FollowerConfig(
        **{name: values[name] for name in FollowerConfig.__dataclass_fields__}
    )


def _transition_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("transition telemetry is empty")
    return rows


def _exact_row_at_time(rows: Sequence[Mapping[str, str]], time_s: float, name: str) -> Mapping[str, str]:
    matches = [row for row in rows if math.isclose(_finite(row["sim_time_s"], "telemetry time"), time_s, rel_tol=0.0, abs_tol=1e-9)]
    if not matches:
        raise ValueError(f"transition telemetry has no exact {name} row")
    return matches[-1]


def _row_pose(row: Mapping[str, str]) -> FloatArray:
    return np.asarray(
        [_finite(row[name], name) for name in ("actual_x", "actual_y", "actual_yaw")],
        dtype=np.float64,
    )


def reconstruct_transition_input(
    index: Exp02DCorpusIndex,
    record: Mapping[str, Any],
    *,
    command_atol: float = RAW_COMMAND_RECONSTRUCTION_ATOL,
) -> Exp02DTransitionInput:
    """Hash-check and reconstruct one eligible transition without mutating sources."""

    identity = str(record.get("corpus_transition_id", "<missing>"))
    try:
        if record.get("status") != "ELIGIBLE_MOVING":
            raise ValueError("only ELIGIBLE_MOVING transitions may enter EXP-02D")
        for key in ("semantic_family", "physical_region"):
            if not isinstance(record.get(key), str) or not record[key].strip():
                raise ValueError(f"combined record has no non-empty {key}")
        cohort = str(record["cohort_id"])
        source = index.sources[cohort]
        if str(record["source_run_path"]) != str(source.description["run_path"]):
            raise ValueError("combined record source run differs from source provenance")
        transition_path = _within(
            index.repository_root,
            str(record["source_transition_relative_path"]),
            "source transition",
        )
        if not transition_path.is_relative_to(source.run):
            raise ValueError("source transition is outside its declared run")
        if sha256_file(transition_path) != str(record["source_transition_sha256"]):
            raise ValueError("source transition JSON hash differs from combined index")
        transition_dir = transition_path.parent
        document = validate_transition_artifact(transition_dir)
        expected_artifact = source.run / str(record["artifact_relative_path"])
        if transition_dir.resolve() != expected_artifact.resolve():
            raise ValueError("combined artifact paths disagree")
        for key in (
            "transition_id",
            "episode_id",
            "transition_index",
            "template_id",
            "variant_id",
            "status",
        ):
            if document.get(key) != record.get(key):
                raise ValueError(f"combined/source transition identity mismatch: {key}")
        source_record = source.source_records.get(str(document["transition_id"]))
        if source_record is None:
            raise ValueError("transition is absent from source transition index")
        for key in (
            "artifact_relative_path",
            "old_raw_sha256",
            "fresh_raw_sha256",
            "ordered_raw_pair_sha256",
            "old_world_sha256",
            "fresh_world_sha256",
            "status",
        ):
            if source_record.get(key) != record.get(key):
                raise ValueError(f"combined/source index mismatch: {key}")
        episode_id = str(document["episode_id"])
        episode_manifest = source.manifest_episodes.get(episode_id)
        if episode_manifest is None:
            raise ValueError("episode is absent from source collection manifest")
        transition_number = int(document["transition_index"])
        transition_hashes = episode_manifest.get("transition_metadata_sha256", [])
        if transition_number >= len(transition_hashes) or str(transition_hashes[transition_number]) != sha256_file(transition_path):
            raise ValueError("transition JSON differs from source collection manifest")

        old_raw = np.load(transition_dir / "raw/old_actions.npy", allow_pickle=False)
        fresh_raw = np.load(transition_dir / "raw/fresh_actions.npy", allow_pickle=False)
        old_world = np.load(transition_dir / "derived/old_world.npy", allow_pickle=False)
        fresh_world = np.load(transition_dir / "derived/fresh_world.npy", allow_pickle=False)
        old_expected = observation_anchored_paths(
            old_raw, document["old_observation_pose_world_se2"]
        )[1]
        fresh_expected = observation_anchored_paths(
            fresh_raw, document["observation_pose_world_se2"]
        )[1]
        if not np.array_equal(old_world, old_expected) or not np.array_equal(fresh_world, fresh_expected):
            raise ValueError("observation-anchored world trajectory does not reconstruct")
        hashes = document["hashes"]
        for key in (
            "old_raw_sha256",
            "fresh_raw_sha256",
            "ordered_raw_pair_sha256",
            "old_world_sha256",
            "fresh_world_sha256",
        ):
            if str(hashes[key]) != str(record[key]):
                raise ValueError(f"combined/source transition hash mismatch: {key}")
        if (
            chunk_content_sha256(old_world) != str(hashes["old_world_sha256"])
            or chunk_content_sha256(fresh_world) != str(hashes["fresh_world_sha256"])
            or ordered_pair_sha256(
                str(hashes["old_raw_sha256"]), str(hashes["fresh_raw_sha256"])
            )
            != str(hashes["ordered_raw_pair_sha256"])
        ):
            raise ValueError("world trajectory or ordered-pair content hash mismatch")

        rows = _transition_rows(transition_dir / "telemetry.csv")
        if any(str(row.get("active_chunk_id")) != str(document["old_chunk_id"]) for row in rows):
            raise ValueError("transition telemetry contains a non-OLD active chunk")
        times = np.asarray([_finite(row["sim_time_s"], "telemetry time") for row in rows])
        if np.any(np.diff(times) <= 0.0):
            raise ValueError("transition telemetry time is not strictly increasing")
        actual = np.load(transition_dir / "actual.npy", allow_pickle=False)
        telemetry_actual = np.asarray([_row_pose(row) for row in rows])
        if not np.array_equal(actual, telemetry_actual):
            raise ValueError("actual.npy differs from transition telemetry")
        boundary = _readonly_pose(document["switch_boundary_B_world_se2"], "B")
        observation = _readonly_pose(document["observation_pose_world_se2"], "observation")
        before = _readonly_pose(document["pose_immediately_before_switch_P_world_se2"], "P")
        old_observation = _readonly_pose(document["old_observation_pose_world_se2"], "OLD observation")
        timing = document["timing"]
        t_obs = _finite(timing["t_obs_sim_s"], "t_obs")
        t_ready = _finite(timing["t_ready_sim_s"], "t_ready")
        t_switch = _finite(timing["t_switch_sim_s"], "t_switch")
        p_time = _finite(document["pose_immediately_before_switch_P_sim_time_s"], "P time")
        if not times[0] <= t_obs <= t_ready <= t_switch or not p_time < t_switch:
            raise ValueError("saved transition event ordering is invalid")
        if not math.isclose(times[-1], t_switch, rel_tol=0.0, abs_tol=1e-9) or not np.allclose(actual[-1], boundary, rtol=0.0, atol=1e-9):
            raise ValueError("transition actual history does not end at B")
        if not np.allclose(_row_pose(_exact_row_at_time(rows, t_obs, "observation")), observation, rtol=0.0, atol=1e-9):
            raise ValueError("saved observation pose differs from transition telemetry")
        if not np.allclose(_row_pose(_exact_row_at_time(rows, p_time, "P")), before, rtol=0.0, atol=1e-9):
            raise ValueError("saved P differs from transition telemetry")

        command_metrics = document["metrics"]["command_discontinuity"]
        old_command = _readonly_command(
            [
                command_metrics["old_last_desired_v_mps"],
                command_metrics["old_last_desired_omega_rps"],
            ],
            "old desired command",
        )
        telemetry_old = _readonly_command(
            [rows[-1]["desired_v_mps"], rows[-1]["desired_omega_rps"]],
            "telemetry old desired command",
        )
        if not np.array_equal(old_command, telemetry_old):
            raise ValueError("saved OLD command differs from final OLD-active telemetry")
        saved_fresh_command = _readonly_command(
            [
                command_metrics["fresh_first_desired_v_mps"],
                command_metrics["fresh_first_desired_omega_rps"],
            ],
            "saved raw-FRESH first desired command",
        )
        saved_delta = saved_fresh_command - old_command
        if not np.allclose(
            saved_delta,
            [command_metrics["delta_v_mps"], command_metrics["delta_omega_rps"]],
            rtol=0.0,
            atol=1e-15,
        ):
            raise ValueError("saved command discontinuity does not reconstruct")
        if difficulty_bin(*saved_delta) != str(command_metrics["difficulty_bin"]):
            raise ValueError("saved command difficulty bin does not reconstruct")
        if str(record["difficulty_bin"]) != str(command_metrics["difficulty_bin"]):
            raise ValueError("combined/source difficulty bin mismatch")
        if str(record["fresh_geometry_bin"]) != str(
            document["metrics"]["fresh_geometry_bin"]
        ):
            raise ValueError("combined/source geometry bin mismatch")
        follower_config = _follower_config(source)
        follower_command: FollowerCommand = TrajectoryFollower(
            fresh_world, follower_config
        ).forward(boundary)
        reconstructed = np.asarray(
            [follower_command.linear_velocity_mps, follower_command.angular_velocity_rps],
            dtype=np.float64,
        )
        tolerance = _nonnegative(command_atol, "command_atol")
        if not np.allclose(reconstructed, saved_fresh_command, rtol=0.0, atol=tolerance):
            raise ValueError(
                "raw-FRESH first command reconstruction mismatch: "
                f"max_abs={float(np.max(np.abs(reconstructed - saved_fresh_command))):.17g}"
            )
        if not 0 <= follower_command.nearest_index <= follower_command.target_index < len(fresh_world):
            raise ValueError("reconstructed follower indices are outside FRESH")

        return Exp02DTransitionInput(
            corpus_transition_id=identity,
            cohort_id=cohort,
            source_run=source.run,
            source_transition=transition_path,
            source_transition_sha256=sha256_file(transition_path),
            source_artifact_sha256=dict(document["artifact_sha256"]),
            episode_id=episode_id,
            transition_id=str(document["transition_id"]),
            transition_index=transition_number,
            template_id=str(document["template_id"]),
            variant_id=str(document["variant_id"]),
            semantic_family=str(record["semantic_family"]),
            physical_region=str(record["physical_region"]),
            old_chunk_id=str(document["old_chunk_id"]),
            fresh_chunk_id=str(document["fresh_chunk_id"]),
            ordered_raw_pair_sha256=str(hashes["ordered_raw_pair_sha256"]),
            fresh_geometry_bin=str(document["metrics"]["fresh_geometry_bin"]),
            difficulty_bin=str(command_metrics["difficulty_bin"]),
            old_raw=_readonly_array(old_raw, "OLD raw", ndim=2, columns=3),
            old_world=_readonly_trajectory(old_world, "OLD world"),
            fresh_raw=_readonly_array(fresh_raw, "FRESH raw", ndim=2, columns=3),
            fresh_world=_readonly_trajectory(fresh_world, "FRESH world"),
            old_observation_pose=old_observation,
            observation_pose=observation,
            pose_before_boundary=before,
            boundary_pose=boundary,
            t_obs_sim_s=t_obs,
            t_ready_sim_s=t_ready,
            t_switch_sim_s=t_switch,
            host_latency_s=_finite(timing["request_response_host_s"], "host latency"),
            effective_latency_s=_finite(timing["tau_effective_s"], "effective latency"),
            old_desired_command=old_command,
            raw_fresh_first_desired_command=saved_fresh_command,
            follower_config=follower_config,
            k_fresh=int(follower_command.nearest_index),
            q_fresh=int(follower_command.target_index),
            raw_follower_command=follower_command,
            raw_follower_goal_reached=bool(follower_command.goal_reached),
            metrics=dict(document["metrics"]),
        )
    except Exp02DInputError:
        raise
    except (KeyError, IndexError, TypeError, ValueError, OSError) as error:
        raise Exp02DInputError(identity, str(error)) from error


def reconstruct_exp02d_corpus(
    index: Exp02DCorpusIndex,
    *,
    command_atol: float = RAW_COMMAND_RECONSTRUCTION_ATOL,
) -> ReconstructedCorpus:
    """Reconstruct every indexed eligible transition and retain explicit failures."""

    transitions: list[Exp02DTransitionInput] = []
    failures: list[ReconstructionFailure] = []
    for record in index.eligible_records:
        try:
            transitions.append(
                reconstruct_transition_input(index, record, command_atol=command_atol)
            )
        except Exp02DInputError as error:
            failures.append(
                ReconstructionFailure(
                    error.corpus_transition_id, error.status, str(error)
                )
            )
    return ReconstructedCorpus(index, tuple(transitions), tuple(failures))


def load_exp02d_development_corpus(
    repository_root: str | Path,
    combined_root: str | Path = DEFAULT_COMBINED_CORPUS,
    *,
    integrity_config: Mapping[str, Any] | None = None,
    command_atol: float = RAW_COMMAND_RECONSTRUCTION_ATOL,
) -> ReconstructedCorpus:
    return reconstruct_exp02d_corpus(
        load_exp02d_corpus_index(
            repository_root, combined_root, integrity_config=integrity_config
        ),
        command_atol=command_atol,
    )


def load_development_corpus(
    repository_root: str | Path,
    combined_path: str | Path = DEFAULT_COMBINED_CORPUS,
    integrity_config: Mapping[str, Any] | None = None,
    command_tol: float = RAW_COMMAND_RECONSTRUCTION_ATOL,
) -> ReconstructedCorpus:
    """Runner-facing alias returning valid inputs plus explicit failures."""

    return load_exp02d_development_corpus(
        repository_root,
        combined_path,
        integrity_config=integrity_config,
        command_atol=command_tol,
    )


@dataclass(frozen=True, slots=True)
class CommandDiscontinuity:
    delta_v_mps: float
    delta_omega_rps: float
    j_cmd: float
    difficulty: str


def command_discontinuity(delta_v_mps: float, delta_omega_rps: float) -> CommandDiscontinuity:
    delta_v = _finite(delta_v_mps, "delta_v_mps")
    delta_omega = _finite(delta_omega_rps, "delta_omega_rps")
    score = math.hypot(
        abs(delta_v) / COMMAND_V_SCALE_MPS,
        abs(delta_omega) / COMMAND_OMEGA_SCALE_RPS,
    )
    return CommandDiscontinuity(
        delta_v,
        delta_omega,
        score,
        difficulty_bin(delta_v, delta_omega),
    )


Metric = str | Callable[[Mapping[str, Any]], float]


def _metric_value(row: Mapping[str, Any], metric: Metric) -> float:
    value = metric(row) if callable(metric) else row[metric]
    return _finite(value, "metric")


def ordered_pair_groups(
    records: Sequence[Mapping[str, Any]],
) -> dict[str, tuple[Mapping[str, Any], ...]]:
    """Group contexts by exact ordered raw-pair identity in stable order."""

    groups: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in records:
        pair = str(row.get("ordered_raw_pair_sha256", ""))
        if not pair:
            raise ValueError("record has no ordered_raw_pair_sha256")
        groups[pair].append(row)
    return {key: tuple(groups[key]) for key in sorted(groups)}


def pair_balanced_group_means(
    records: Sequence[Mapping[str, Any]], metric: Metric
) -> dict[str, float]:
    groups = ordered_pair_groups(records)
    if not groups:
        raise ValueError("pair-balanced mean requires at least one record")
    return {
        pair: float(np.mean([_metric_value(row, metric) for row in rows]))
        for pair, rows in groups.items()
    }


def pair_balanced_mean(records: Sequence[Mapping[str, Any]], metric: Metric) -> float:
    values = tuple(pair_balanced_group_means(records, metric).values())
    return float(np.mean(values))


def transition_weighted_mean(records: Sequence[Mapping[str, Any]], metric: Metric) -> float:
    if not records:
        raise ValueError("transition-weighted mean requires at least one record")
    return float(np.mean([_metric_value(row, metric) for row in records]))


def cluster_bootstrap_pair_balanced_difference(
    records: Sequence[Mapping[str, Any]],
    left_metric: Metric,
    right_metric: Metric,
    *,
    seed: int,
    repetitions: int,
    confidence: float = 0.95,
) -> dict[str, float | int | str]:
    """Bootstrap pair-level mean differences, resampling ordered pairs only."""

    if not isinstance(seed, int) or seed < 0:
        raise ValueError("bootstrap seed must be a non-negative integer")
    if not isinstance(repetitions, int) or repetitions < 1:
        raise ValueError("bootstrap repetitions must be a positive integer")
    confidence_value = _finite(confidence, "confidence")
    if not 0.0 < confidence_value < 1.0:
        raise ValueError("confidence must be in (0, 1)")
    groups = ordered_pair_groups(records)
    if not groups:
        raise ValueError("cluster bootstrap requires at least one ordered-pair group")
    differences = np.asarray(
        [
            np.mean(
                [
                    _metric_value(row, left_metric) - _metric_value(row, right_metric)
                    for row in rows
                ]
            )
            for rows in groups.values()
        ],
        dtype=np.float64,
    )
    generator = np.random.default_rng(seed)
    indices = generator.integers(
        0, len(differences), size=(repetitions, len(differences)), endpoint=False
    )
    draws = np.mean(differences[indices], axis=1)
    tail = (1.0 - confidence_value) / 2.0
    return {
        "estimate": float(np.mean(differences)),
        "ci_lower": float(np.quantile(draws, tail)),
        "ci_upper": float(np.quantile(draws, 1.0 - tail)),
        "confidence": confidence_value,
        "seed": seed,
        "repetitions": repetitions,
        "ordered_pair_group_count": len(differences),
        "bootstrap_unit": "ordered_raw_pair_sha256",
        "difference_semantics": "left_minus_right",
    }


def success_failure_label(
    raw: CommandDiscontinuity,
    m3: CommandDiscontinuity | None,
    *,
    geometry_undefined: bool = False,
    solver_failure: bool = False,
) -> str | None:
    """Apply the predeclared A--H labels; RAW intermediate has no such label."""

    if geometry_undefined:
        return "GEOMETRY_UNDEFINED"
    if solver_failure or m3 is None:
        return "SOLVER_FAILURE"
    if raw.difficulty == "BENIGN":
        return "BENIGN_PRESERVED" if m3.difficulty == "BENIGN" else "BENIGN_BROKEN"
    if raw.difficulty != "CHALLENGING":
        return None
    if m3.difficulty != "CHALLENGING":
        return "CHALLENGING_RESCUED"
    if m3.j_cmd <= 0.80 * raw.j_cmd:
        return "CHALLENGING_IMPROVED"
    if m3.j_cmd >= 1.20 * raw.j_cmd:
        return "CHALLENGING_WORSE"
    return "CHALLENGING_MIXED"


def m4_false_correction_rescued(
    raw: CommandDiscontinuity,
    m1: CommandDiscontinuity,
    m3: CommandDiscontinuity,
) -> bool:
    return (
        raw.difficulty == "BENIGN"
        and m1.difficulty != "BENIGN"
        and m3.difficulty == "BENIGN"
    )


@dataclass(frozen=True, slots=True)
class RegimeThresholds:
    alpha_entry_large_rad: float
    alpha_look_small_rad: float
    alpha_look_large_rad: float
    positional_gap_large_m: float
    degenerate_lookahead_radius_m: float
    insufficient_lookahead_arc_m: float
    large_deformation_rms_m: float
    small_command_benefit_fraction: float
    rigid_correction_min_entry_m: float
    rigid_fit_translation_rms_m: float
    rigid_fit_yaw_rms_rad: float
    rigid_endpoint_entry_difference_m: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, _nonnegative(getattr(self, name), name))
        if self.alpha_look_small_rad > self.alpha_look_large_rad:
            raise ValueError("alpha_look_small_rad cannot exceed alpha_look_large_rad")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "RegimeThresholds":
        return cls(**{name: values[name] for name in cls.__dataclass_fields__})


def regime_memberships(
    row: Mapping[str, Any], thresholds: RegimeThresholds
) -> tuple[str, ...]:
    """Return all R1--R6 diagnostic regimes satisfied by a valid transition."""

    alpha_entry = _nonnegative(row["alpha_entry_rad"], "alpha_entry_rad")
    alpha_look = _nonnegative(row["alpha_look_rad"], "alpha_look_rad")
    entry_gap = _nonnegative(row["b_to_fresh_k_translation_m"], "B-to-F_k gap")
    d_q = _nonnegative(row["d_q_m"], "d_q_m")
    arc = _nonnegative(row["fresh_arc_k_to_q_m"], "fresh_arc_k_to_q_m")
    k = int(row["k_fresh"])
    q = int(row["q_fresh"])
    if k < 0 or q < k:
        raise ValueError("follower k/q indices are invalid")
    j_raw = _nonnegative(row["j_cmd_raw"], "j_cmd_raw")
    j_m3 = _nonnegative(row["j_cmd_m3"], "j_cmd_m3")
    deformation = _nonnegative(
        row["m3_rms_translation_deformation_m"],
        "m3_rms_translation_deformation_m",
    )
    entry_displacement = _nonnegative(
        row["m3_entry_displacement_m"], "m3_entry_displacement_m"
    )
    endpoint_displacement = _nonnegative(
        row["m3_endpoint_displacement_m"], "m3_endpoint_displacement_m"
    )
    rigid_translation = _nonnegative(
        row["m3_best_fit_rigid_translation_rms_m"],
        "m3_best_fit_rigid_translation_rms_m",
    )
    rigid_yaw = _nonnegative(
        row["m3_best_fit_rigid_yaw_rms_rad"], "m3_best_fit_rigid_yaw_rms_rad"
    )
    memberships: list[str] = []
    if (
        alpha_entry >= thresholds.alpha_entry_large_rad
        and alpha_look <= thresholds.alpha_look_small_rad
    ):
        memberships.append("R1_FALSE_ENTRY_DIRECTION_MISMATCH")
    if (
        alpha_look >= thresholds.alpha_look_large_rad
        and str(row["fresh_geometry_bin"]) in TURNING_GEOMETRIES
    ):
        memberships.append("R2_INTENT_CONFLICT_CANDIDATE")
    if (
        alpha_look <= thresholds.alpha_look_small_rad
        and entry_gap >= thresholds.positional_gap_large_m
    ):
        memberships.append("R3_POSITION_GAP_DIRECTION_CONSISTENT")
    if (
        q == k
        or d_q <= thresholds.degenerate_lookahead_radius_m
        or arc <= thresholds.insufficient_lookahead_arc_m
    ):
        memberships.append("R4_DEGENERATE_LOOKAHEAD")
    relative_benefit = (j_raw - j_m3) / max(j_raw, 1.0e-9)
    if (
        deformation >= thresholds.large_deformation_rms_m
        and relative_benefit <= thresholds.small_command_benefit_fraction
    ):
        memberships.append("R5_LARGE_CORRECTION_LITTLE_COMMAND_BENEFIT")
    if (
        entry_displacement >= thresholds.rigid_correction_min_entry_m
        and rigid_translation <= thresholds.rigid_fit_translation_rms_m
        and rigid_yaw <= thresholds.rigid_fit_yaw_rms_rad
        and abs(endpoint_displacement - entry_displacement)
        <= thresholds.rigid_endpoint_entry_difference_m
    ):
        memberships.append("R6_RIGID_PROPAGATION")
    return tuple(memberships)


def _valid_representative_row(row: Mapping[str, Any]) -> bool:
    return not bool(row.get("geometry_undefined", False)) and not bool(
        row.get("solver_failure", False)
    )


def select_representative_ids(
    records: Sequence[Mapping[str, Any]],
) -> dict[str, str]:
    """Apply the frozen S1/S2/F1/F2 rules without relaxing unavailable cases."""

    rows = [row for row in records if _valid_representative_row(row)]
    for row in rows:
        for key in ("j_cmd_raw", "j_cmd_m1", "j_cmd_m2", "j_cmd_m3"):
            _nonnegative(row[key], key)
        identifier = str(row.get("corpus_transition_id", ""))
        if not identifier:
            raise ValueError("representative record has no corpus_transition_id")

    s1 = [
        row
        for row in rows
        if str(row["raw_difficulty_bin"]) == "BENIGN"
        and str(row["m1_difficulty_bin"]) != "BENIGN"
        and str(row["m3_difficulty_bin"]) == "BENIGN"
    ]
    s2 = [
        row
        for row in rows
        if str(row["raw_difficulty_bin"]) == "CHALLENGING"
        and _finite(row["j_cmd_m3"], "j_cmd_m3")
        < _finite(row["j_cmd_raw"], "j_cmd_raw")
        and _finite(row["j_cmd_m3"], "j_cmd_m3")
        < _finite(row["j_cmd_m2"], "j_cmd_m2")
    ]
    f1 = [
        row
        for row in rows
        if str(row["fresh_geometry_bin"]) in TURNING_GEOMETRIES
        and _finite(row["j_cmd_m3"], "j_cmd_m3")
        >= 1.20 * _finite(row["j_cmd_raw"], "j_cmd_raw")
    ]
    f2 = [
        row
        for row in rows
        if _nonnegative(row["alpha_look_rad"], "alpha_look_rad")
        <= math.radians(15.0)
        and _nonnegative(
            row["b_to_fresh_k_translation_m"], "b_to_fresh_k_translation_m"
        )
        >= 0.20
        and abs(
            _finite(row["j_cmd_raw"], "j_cmd_raw")
            - _finite(row["j_cmd_m3"], "j_cmd_m3")
        )
        <= 0.10 * max(_finite(row["j_cmd_raw"], "j_cmd_raw"), 1.0e-9)
    ]

    def choose(
        label: str,
        candidates: Sequence[Mapping[str, Any]],
        score: Callable[[Mapping[str, Any]], float],
    ) -> str:
        if not candidates:
            return f"{label}_NOT_AVAILABLE"
        winner = min(
            candidates,
            key=lambda row: (
                -_finite(score(row), f"{label} rank score"),
                str(row["corpus_transition_id"]),
            ),
        )
        return str(winner["corpus_transition_id"])

    return {
        "S1": choose("S1", s1, lambda row: row["j_cmd_m1"] - row["j_cmd_m3"]),
        "S2": choose("S2", s2, lambda row: row["j_cmd_raw"] - row["j_cmd_m3"]),
        "F1": choose("F1", f1, lambda row: row["j_cmd_m3"] - row["j_cmd_raw"]),
        "F2": choose("F2", f2, lambda row: row["b_to_fresh_k_translation_m"]),
    }


# Short, explicit runner-facing names requested by the frozen protocol.
cluster_bootstrap_difference = cluster_bootstrap_pair_balanced_difference
classify_m3_outcome = success_failure_label
select_representatives = select_representative_ids
