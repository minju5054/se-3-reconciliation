"""Pure saved-evidence loader for the EXP-02D success/failure GUI.

The GUI is intentionally downstream of a completed primary run.  This module
loads only frozen representative IDs and their saved candidate artifacts, then
joins them to the exact current-OLD active interval.  It never runs an
optimizer, follower, LightNav, controller, or physics simulation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray
import yaml

from reconciliation.data02_active_old import ActiveOldInterval, load_active_old_interval
from reconciliation.data02_online_successive import strict_json
from reconciliation.online_switch import sha256_file


FloatArray = NDArray[np.float64]
REPRESENTATIVE_CASES = ("S1", "S2", "F1", "F2")
METHODS = ("M0_RAW", "M1_HISTORICAL_M4", "M2_NO_DIRECTION", "M3_LOOKAHEAD")
PHASE_A_FRACTION = 0.70


class RepresentativeUnavailableError(ValueError):
    """Raised when a frozen representative rule returned ``NOT_AVAILABLE``."""


@dataclass(frozen=True, slots=True)
class SavedDisplayPose:
    pose: FloatArray
    lower_index: int
    upper_index: int
    alpha: float


@dataclass(frozen=True, slots=True)
class Exp02DGuiCase:
    run: Path
    case: str
    corpus_transition_id: str
    outcome_title: str
    interpretation: str
    config: Mapping[str, Any]
    representative: Mapping[str, Any]
    input_reference: Mapping[str, Any]
    oracle_indices: Mapping[str, Any]
    active_old: ActiveOldInterval
    selected_raw_fresh: FloatArray
    method_candidates: Mapping[str, FloatArray]
    method_metrics: Mapping[str, Mapping[str, Any]]
    f_k_world_se2: FloatArray
    f_q_world_se2: FloatArray
    source_sha256: Mapping[str, str]
    result_sha256: Mapping[str, str]

    @property
    def k_fresh(self) -> int:
        return int(self.oracle_indices["k_fresh"])

    @property
    def q_fresh(self) -> int:
        return int(self.oracle_indices["q_fresh"])


def _resolve_inside(root: Path, relative: str | Path, name: str) -> Path:
    value = Path(relative)
    if value.is_absolute() or ".." in value.parts:
        raise ValueError(f"{name} must be a safe path relative to the EXP-02D run")
    result = (root / value).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f"{name} escapes the EXP-02D run")
    return result


def _strict_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def _readonly_trajectory(value: Any, name: str) -> FloatArray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.ndim != 2 or result.shape[0] < 1 or result.shape[1] != 3:
        raise ValueError(f"{name} must be a non-empty N x 3 trajectory")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains NaN/Inf")
    result.setflags(write=False)
    return result


def _readonly_pose(value: Sequence[float], name: str) -> FloatArray:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite SE(2) pose")
    result.setflags(write=False)
    return result


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _verify_result_file(
    run: Path, manifest: Mapping[str, Any], relative: str, name: str
) -> tuple[Path, str]:
    artifacts = manifest.get("artifact_sha256")
    if not isinstance(artifacts, Mapping) or relative not in artifacts:
        raise ValueError(f"result manifest does not identify {name}: {relative}")
    path = _resolve_inside(run, relative, name)
    digest = sha256_file(path)
    if digest != str(artifacts[relative]):
        raise ValueError(f"EXP-02D result hash mismatch: {relative}")
    return path, digest


def _command(metrics: Mapping[str, Any]) -> Mapping[str, Any]:
    command = metrics.get("command")
    if not isinstance(command, Mapping):
        raise ValueError("method metrics have no command mapping")
    for key in ("delta_v_abs_mps", "delta_omega_abs_rps", "J_cmd"):
        value = _finite(command.get(key), key)
        if value < 0.0:
            raise ValueError(f"{key} must be non-negative")
    return command


def _benign(command: Mapping[str, Any]) -> bool:
    return float(command["delta_v_abs_mps"]) <= 0.05 and float(
        command["delta_omega_abs_rps"]
    ) <= 0.10


def _challenging(command: Mapping[str, Any]) -> bool:
    return float(command["delta_v_abs_mps"]) >= 0.15 or float(
        command["delta_omega_abs_rps"]
    ) >= 0.30


def _validate_case_rule(
    case: str,
    representative: Mapping[str, Any],
    method_metrics: Mapping[str, Mapping[str, Any]],
    active: ActiveOldInterval,
) -> tuple[str, str]:
    raw = _command(method_metrics["M0_RAW"])
    m1 = _command(method_metrics["M1_HISTORICAL_M4"])
    m2 = _command(method_metrics["M2_NO_DIRECTION"])
    m3 = _command(method_metrics["M3_LOOKAHEAD"])
    values = representative.get("metrics")
    if not isinstance(values, Mapping):
        raise ValueError("available representative has no frozen metrics")
    alpha_entry = _finite(values["alpha_entry_rad"], "alpha_entry_rad")
    alpha_look = _finite(values["alpha_look_rad"], "alpha_look_rad")
    entry_gap = _finite(values["b_to_fresh_k_translation_m"], "B-to-F_k gap")
    raw_j = float(raw["J_cmd"])
    m3_j = float(m3["J_cmd"])

    if case == "S1":
        if not (_benign(raw) and not _benign(m1) and _benign(m3)):
            raise ValueError("saved S1 artifact violates its frozen representative rule")
        if alpha_entry >= math.radians(60.0) and alpha_look <= math.radians(15.0):
            explanation = (
                "Historical M4 saw a large B->F_k mismatch while the raw follower "
                "lookahead was much more consistent; M3 avoided that false correction."
            )
        else:
            explanation = (
                "Historical M4 broke a benign raw transition while M3 remained benign. "
                "The displayed geometry is not labeled as the R1 mechanism."
            )
        return "SUCCESS — BENIGN RESCUE", explanation
    if case == "S2":
        if not (_challenging(raw) and m3_j < raw_j and m3_j < float(m2["J_cmd"])):
            raise ValueError("saved S2 artifact violates its frozen representative rule")
        return (
            "SUCCESS — CHALLENGING IMPROVEMENT",
            "Raw switching was challenging; M3 reduced the immediate command score more "
            "than RAW and NO-DIRECTION.",
        )
    if case == "F1":
        geometry = str(active.transition_metadata["metrics"]["fresh_geometry_bin"])
        if geometry not in ("POSITIVE_TURNING", "NEGATIVE_TURNING") or not (
            m3_j >= 1.20 * raw_j
        ):
            raise ValueError("saved F1 artifact violates its frozen representative rule")
        return (
            "CANDIDATE FAILURE — INTENT-CONFLICT CANDIDATE",
            "M3 worsened a turning FRESH transition. This is consistent with a possible "
            "continuity-versus-new-intent conflict, but does not prove instruction intent.",
        )
    if case == "F2":
        if not (
            alpha_look <= math.radians(15.0)
            and entry_gap >= 0.20
            and abs(raw_j - m3_j) <= 0.10 * max(raw_j, 1e-9)
        ):
            raise ValueError("saved F2 artifact violates its frozen representative rule")
        return (
            "FAILURE — POSITIONAL / MISSING MECHANISM",
            "Incoming and lookahead directions were aligned, but a large positional mismatch "
            "remained; changing direction semantics alone did not solve it.",
        )
    raise ValueError(f"unknown representative case: {case}")


def available_representatives(run: str | Path) -> dict[str, str | None]:
    """Return the frozen representative ID or ``None`` for each declared case."""

    root = Path(run).expanduser().resolve()
    document = strict_json(root / "summary/representatives.json")
    rows = document.get("representatives")
    if not isinstance(rows, Mapping) or set(rows) != set(REPRESENTATIVE_CASES):
        raise ValueError("representatives.json must contain exactly S1/S2/F1/F2")
    result: dict[str, str | None] = {}
    for case in REPRESENTATIVE_CASES:
        row = rows[case]
        if not isinstance(row, Mapping):
            raise ValueError(f"representative {case} must be a mapping")
        if row.get("criteria_relaxed") is True:
            raise ValueError(f"representative {case} relaxed its frozen criterion")
        if row.get("status") == "AVAILABLE":
            identifier = row.get("corpus_transition_id")
            if not isinstance(identifier, str) or not identifier:
                raise ValueError(f"available representative {case} has no identity")
            result[case] = identifier
        elif row.get("status") == f"{case}_NOT_AVAILABLE":
            if row.get("corpus_transition_id") is not None:
                raise ValueError(f"unavailable representative {case} has an identity")
            result[case] = None
        else:
            raise ValueError(f"representative {case} has an invalid status")
    return result


def latest_completed_run(output_root: str | Path) -> Path:
    """Return the lexicographically latest complete primary run.

    Dry-validation and interrupted directories are ignored because they have
    no completed primary result manifest.  The GUI still validates every
    selected artifact after this convenience lookup.
    """

    root = Path(output_root).expanduser().resolve()
    candidates: list[Path] = []
    if root.is_dir():
        for path in root.iterdir():
            manifest_path = path / "result_manifest.json"
            metadata_path = path / "metadata.json"
            if not path.is_dir() or not manifest_path.is_file() or not metadata_path.is_file():
                continue
            try:
                manifest = strict_json(manifest_path)
                metadata = strict_json(metadata_path)
            except (OSError, TypeError, ValueError):
                continue
            if (
                manifest.get("schema") == "EXP02D_ResultManifest_v1"
                and metadata.get("experiment") == "EXP-02D"
                and metadata.get("technical_status") == "EXP02D_RUN_COMPLETE"
            ):
                candidates.append(path.resolve())
    if not candidates:
        raise FileNotFoundError(f"no completed EXP-02D primary run under {root}")
    return max(candidates, key=lambda path: path.name)


def load_exp02d_gui_case(run: str | Path, case: str) -> Exp02DGuiCase:
    """Load one hash-verified saved representative and its active-OLD replay."""

    root = Path(run).expanduser().resolve()
    if case not in REPRESENTATIVE_CASES:
        raise ValueError(f"case must be one of {REPRESENTATIVE_CASES}")
    result_manifest = strict_json(root / "result_manifest.json")
    if result_manifest.get("schema") != "EXP02D_ResultManifest_v1":
        raise ValueError("GUI requires a complete EXP-02D result manifest")
    metadata_path, metadata_hash = _verify_result_file(
        root, result_manifest, "metadata.json", "run metadata"
    )
    metadata = strict_json(metadata_path)
    if (
        metadata.get("experiment") != "EXP-02D"
        or metadata.get("technical_status") != "EXP02D_RUN_COMPLETE"
        or metadata.get("candidate_methods_physically_executed") is not False
    ):
        raise ValueError("GUI requires a completed candidate-only EXP-02D primary run")
    config_path, config_hash = _verify_result_file(
        root, result_manifest, "config_snapshot.yaml", "config snapshot"
    )
    config = _strict_yaml(config_path)
    if config.get("experiment") != "EXP-02D":
        raise ValueError("EXP-02D config snapshot has the wrong experiment")
    representatives_path, representatives_hash = _verify_result_file(
        root, result_manifest, "summary/representatives.json", "representative summary"
    )
    representatives = strict_json(representatives_path)
    rows = representatives.get("representatives")
    if not isinstance(rows, Mapping) or set(rows) != set(REPRESENTATIVE_CASES):
        raise ValueError("representatives.json must contain exactly S1/S2/F1/F2")
    representative = rows[case]
    if not isinstance(representative, Mapping):
        raise ValueError(f"representative {case} must be a mapping")
    if representative.get("criteria_relaxed") is not False:
        raise ValueError(f"representative {case} did not preserve its frozen criterion")
    if representative.get("status") != "AVAILABLE":
        expected = f"{case}_NOT_AVAILABLE"
        if representative.get("status") != expected:
            raise ValueError(f"invalid unavailable representative status for {case}")
        raise RepresentativeUnavailableError(expected)
    identifier = str(representative["corpus_transition_id"])
    artifact_relative = str(representative["artifact_relative_path"])
    artifact_root = _resolve_inside(root, artifact_relative, "representative artifact")
    if not artifact_root.is_dir():
        raise ValueError("representative artifact directory is missing")

    corpus_path, corpus_hash = _verify_result_file(
        root, result_manifest, "corpus_manifest.json", "corpus manifest"
    )
    corpus = strict_json(corpus_path)
    transition_artifacts = corpus.get("transition_artifacts")
    if not isinstance(transition_artifacts, Mapping) or identifier not in transition_artifacts:
        raise ValueError("representative is absent from the EXP-02D corpus manifest")
    artifact_record = transition_artifacts[identifier]
    if str(artifact_record["artifact_relative_path"]) != artifact_relative:
        raise ValueError("representative and corpus artifact paths differ")

    result_hashes: dict[str, str] = {
        "metadata.json": metadata_hash,
        "config_snapshot.yaml": config_hash,
        "summary/representatives.json": representatives_hash,
        "corpus_manifest.json": corpus_hash,
    }
    input_relative = f"{artifact_relative}/input_reference.json"
    oracle_relative = f"{artifact_relative}/oracle_indices.json"
    input_path, input_hash = _verify_result_file(
        root, result_manifest, input_relative, "transition input reference"
    )
    oracle_path, oracle_hash = _verify_result_file(
        root, result_manifest, oracle_relative, "follower oracle"
    )
    if input_hash != str(artifact_record["input_reference_sha256"]):
        raise ValueError("input reference hash differs from corpus manifest")
    if oracle_hash != str(artifact_record["oracle_indices_sha256"]):
        raise ValueError("oracle hash differs from corpus manifest")
    result_hashes[input_relative] = input_hash
    result_hashes[oracle_relative] = oracle_hash
    input_reference = strict_json(input_path)
    oracle = strict_json(oracle_path)
    if (
        input_reference.get("corpus_transition_id") != identifier
        or int(input_reference["transition_index"]) < 0
        or oracle.get("experimental_oracle") is not True
        or oracle.get("final_correspondence_detector") is not False
        or int(oracle.get("call_count_at_B", -1)) != 1
    ):
        raise ValueError("representative input/oracle identity or semantics changed")
    k = int(oracle["k_fresh"])
    q = int(oracle["q_fresh"])
    if q - k != int(oracle["q_minus_k"]):
        raise ValueError("saved q-k mapping is inconsistent")
    representative_metrics = representative.get("metrics")
    if not isinstance(representative_metrics, Mapping) or (
        int(representative_metrics["k_fresh"]) != k
        or int(representative_metrics["q_fresh"]) != q
    ):
        raise ValueError("representative k/q differ from the saved follower oracle")

    active = load_active_old_interval(
        input_reference["source_run"],
        str(input_reference["episode_id"]),
        int(input_reference["transition_index"]),
    )
    source_transition = Path(str(input_reference["source_transition"])).resolve()
    if source_transition != (active.transition_directory / "transition.json").resolve():
        raise ValueError("input reference points to a different source transition")
    if active.source_sha256["transition.json"] != str(
        input_reference["source_transition_sha256"]
    ):
        raise ValueError("source transition hash differs from EXP-02D input reference")
    for key in (
        "derived/old_world.npy",
        "derived/fresh_world.npy",
        "actual.npy",
        "telemetry.csv",
    ):
        active_key = "transition_telemetry.csv" if key == "telemetry.csv" else key
        if active.source_sha256[active_key] != str(
            input_reference["source_artifact_sha256"][key]
        ):
            raise ValueError(f"source artifact hash differs from input reference: {key}")
    for key, actual in (
        ("corpus_transition_id", identifier),
        ("old_chunk_id", active.old_chunk_id),
        ("fresh_chunk_id", active.fresh_chunk_id),
    ):
        if str(input_reference[key]) != str(actual):
            raise ValueError(f"source/EXP-02D identity mismatch: {key}")
    if not 0 <= k <= q < len(active.fresh_world):
        raise ValueError("saved k/q lie outside raw FRESH")
    selected_raw = _readonly_trajectory(active.fresh_world[k:], "selected raw FRESH")

    candidates: dict[str, FloatArray] = {}
    metrics_by_method: dict[str, Mapping[str, Any]] = {}
    for method in METHODS:
        method_record = artifact_record["methods"][method]
        candidate_relative = f"{artifact_relative}/{method}/candidate.npy"
        metrics_relative = f"{artifact_relative}/{method}/metrics.json"
        candidate_path, candidate_hash = _verify_result_file(
            root, result_manifest, candidate_relative, f"{method} candidate"
        )
        metrics_path, metrics_hash = _verify_result_file(
            root, result_manifest, metrics_relative, f"{method} metrics"
        )
        if (
            candidate_hash != str(method_record["candidate_sha256"])
            or metrics_hash != str(method_record["metrics_sha256"])
        ):
            raise ValueError(f"{method} hashes differ from corpus manifest")
        metrics = strict_json(metrics_path)
        if (
            metrics.get("method") != method
            or metrics.get("candidate_sha256") != candidate_hash
            or metrics.get("candidate_physically_executed") is not False
            or metrics.get("J_cmd_in_objective") is not False
        ):
            raise ValueError(f"{method} saved-only metric semantics changed")
        _command(metrics)
        candidate = _readonly_trajectory(
            np.load(candidate_path, allow_pickle=False), f"{method} candidate"
        )
        if candidate.shape != selected_raw.shape:
            raise ValueError(f"{method} candidate shape differs from raw selected FRESH")
        if method == "M0_RAW" and not np.array_equal(candidate, selected_raw):
            raise ValueError("M0_RAW candidate differs from raw FRESH[k:]")
        candidates[method] = candidate
        metrics_by_method[method] = MappingProxyType(metrics)
        result_hashes[candidate_relative] = candidate_hash
        result_hashes[metrics_relative] = metrics_hash
        optimization_hash = method_record.get("optimization_sha256")
        if method != "M0_RAW":
            optimization_relative = f"{artifact_relative}/{method}/optimization.json"
            _, verified = _verify_result_file(
                root, result_manifest, optimization_relative, f"{method} optimization"
            )
            if verified != str(optimization_hash):
                raise ValueError(f"{method} optimization hash differs from corpus manifest")
            result_hashes[optimization_relative] = verified
        elif optimization_hash is not None:
            raise ValueError("M0_RAW must not have an optimization artifact")

    outcome, interpretation = _validate_case_rule(
        case, representative, metrics_by_method, active
    )
    f_k = _readonly_pose(active.fresh_world[k], "F_k")
    f_q = _readonly_pose(active.fresh_world[q], "F_q")
    source_hashes = dict(active.source_sha256)
    source_manifest = strict_json(active.run / "collection_manifest.json")
    source_config_path = active.run / "config_snapshot.yaml"
    source_config_hash = sha256_file(source_config_path)
    if source_config_hash != str(source_manifest["config_snapshot_sha256"]):
        raise ValueError("source DATA-02 config snapshot hash changed")
    source_hashes["config_snapshot.yaml"] = source_config_hash
    return Exp02DGuiCase(
        run=root,
        case=case,
        corpus_transition_id=identifier,
        outcome_title=outcome,
        interpretation=interpretation,
        config=MappingProxyType(config),
        representative=MappingProxyType(dict(representative)),
        input_reference=MappingProxyType(input_reference),
        oracle_indices=MappingProxyType(oracle),
        active_old=active,
        selected_raw_fresh=selected_raw,
        method_candidates=MappingProxyType(candidates),
        method_metrics=MappingProxyType(metrics_by_method),
        f_k_world_se2=f_k,
        f_q_world_se2=f_q,
        source_sha256=MappingProxyType(dict(sorted(source_hashes.items()))),
        result_sha256=MappingProxyType(dict(sorted(result_hashes.items()))),
    )


def saved_pose_at(active: ActiveOldInterval, sim_time_s: float) -> SavedDisplayPose:
    """Interpolate between adjacent saved active-OLD display states only."""

    target = min(
        max(_finite(sim_time_s, "saved display time"), active.display_sim_times_s[0]),
        active.display_sim_times_s[-1],
    )
    upper = int(np.searchsorted(active.display_sim_times_s, target, side="left"))
    if upper < len(active.display_sim_times_s) and target == float(
        active.display_sim_times_s[upper]
    ):
        lower = upper
        alpha = 0.0
    elif upper == 0:
        lower = upper = 0
        alpha = 0.0
    elif upper >= len(active.display_sim_times_s):
        lower = upper = len(active.display_sim_times_s) - 1
        alpha = 0.0
    else:
        lower = upper - 1
        span = float(active.display_sim_times_s[upper] - active.display_sim_times_s[lower])
        alpha = (target - float(active.display_sim_times_s[lower])) / span
    if lower == upper:
        pose = np.array(active.display_actual_poses[lower], copy=True)
    else:
        first = active.display_actual_poses[lower]
        second = active.display_actual_poses[upper]
        pose = np.empty(3, dtype=np.float64)
        pose[:2] = first[:2] + alpha * (second[:2] - first[:2])
        yaw_delta = (float(second[2] - first[2]) + math.pi) % (2.0 * math.pi) - math.pi
        pose[2] = (float(first[2]) + alpha * yaw_delta + math.pi) % (
            2.0 * math.pi
        ) - math.pi
    pose.setflags(write=False)
    return SavedDisplayPose(pose, lower, upper, float(alpha))


def gui_phase_and_saved_time(
    presentation_time_s: float, duration_s: float, active: ActiveOldInterval
) -> tuple[str, float]:
    """Map presentation time to Phase A saved time or Phase B's frozen B."""

    duration = _finite(duration_s, "GUI duration")
    if duration <= 0.0:
        raise ValueError("GUI duration must be positive")
    elapsed = min(max(_finite(presentation_time_s, "presentation time"), 0.0), duration)
    phase_a_end = PHASE_A_FRACTION * duration
    if elapsed < phase_a_end:
        progress = elapsed / phase_a_end
        saved = active.activation_sim_time_s + progress * (
            active.switch_sim_time_s - active.activation_sim_time_s
        )
        return "PHASE_A_ACTUAL_OLD_ACTIVE", float(saved)
    return "PHASE_B_OFFLINE_CANDIDATES_AT_B", active.switch_sim_time_s


def saved_only_gui_contract() -> dict[str, bool]:
    return {
        "saved_primary_results_only": True,
        "lightnav_invoked": False,
        "optimizer_invoked": False,
        "controller_invoked": False,
        "physics_reexecution": False,
        "candidate_execution": False,
        "previous_chunk_actual_displayed": False,
        "post_switch_actual_displayed": False,
        "m2_visible_by_default": False,
    }
