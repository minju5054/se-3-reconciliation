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
class Exp02DCameraPlan:
    """Indoor-safe framing for saved EXP-02D GUI evidence."""

    eye_xyz: tuple[float, float, float]
    target_xyz: tuple[float, float, float]
    horizontal_fov_deg: float
    vertical_fov_deg: float
    viewport_aspect_ratio: float
    elevation_deg: float
    slant_distance_m: float
    back_distance_m: float
    lateral_offset_m: float
    nominal_view_direction_xy: tuple[float, float]
    eye_direction_xy: tuple[float, float]
    preferred_eye_direction_xy: tuple[float, float] | None
    preferred_eye_half_angle_deg: float | None
    azimuth_world_deg: float
    azimuth_offset_from_initial_heading_deg: float
    azimuth_search_coarse_step_deg: float
    azimuth_search_fine_step_deg: float
    azimuth_candidates_evaluated: int
    minimum_slant_distance_m: float
    minimum_back_distance_m: float
    maximum_eye_height_m: float
    containment_limit_ndc: float
    maximum_projected_ndc: float
    minimum_projected_depth_m: float
    old_path_z_m: float
    actual_path_ground_z_m: float
    actual_path_visual_z_m: float
    observation_marker_visual_z_m: float
    candidate_path_z_m: tuple[float, ...]
    candidate_bounds_xy: tuple[tuple[float, float], tuple[float, float]]
    focus_bounds_xy: tuple[tuple[float, float], tuple[float, float]]
    drawn_bounds_xy: tuple[tuple[float, float], tuple[float, float]]
    projected_candidate_geometry_fraction: float
    projected_focus_geometry_fraction: float
    requested_candidate_geometry_fraction: float
    candidate_fraction_target_achieved: bool
    full_context_and_jackal_contained: bool
    limiting_constraint: str


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


def compute_exp02d_camera_plan(
    old_world: np.ndarray,
    actual_poses: np.ndarray,
    candidate_paths: Sequence[np.ndarray],
    *,
    horizontal_fov_deg: float = 55.0,
    candidate_geometry_fraction: float = 0.74,
    viewport_aspect_ratio: float = 1.60,
    minimum_slant_distance_m: float = 1.80,
    minimum_back_distance_m: float = 1.55,
    eye_height_m: float = 1.45,
    minimum_eye_height_m: float = 1.15,
    maximum_eye_height_m: float = 1.85,
    target_z_m: float = 0.25,
    lateral_offset_m: float = 0.25,
    azimuth_coarse_step_deg: float = 5.0,
    azimuth_fine_step_deg: float = 0.25,
    containment_limit_ndc: float = 0.95,
    minimum_depth_m: float = 0.15,
    old_path_z_m: float = 0.78,
    actual_path_ground_z_m: float = 0.255,
    actual_path_visual_z_m: float = 0.80,
    candidate_path_z_m: Sequence[float] | None = None,
    observation_pose_world_se2: Sequence[float] | None = None,
    observation_marker_visual_z_m: float = 0.88,
    observation_marker_radius_m: float = 0.04,
    preferred_eye_direction_xy: Sequence[float] | None = None,
    preferred_eye_half_angle_deg: float = 90.0,
) -> Exp02DCameraPlan:
    """Fit full saved context and a swept Jackal proxy with exact pinhole projection.

    The camera azimuth is selected by a deterministic coarse/fine sweep so
    path geometry is not unnecessarily foreshortened.  Every rendered path,
    the visual-only raised actual-path duplicate, and a swept 0.51 x 0.43 m
    Jackal proxy are constrained inside the useful viewport.  Requested
    65--80% candidate occupancy is reported as infeasible only after the full
    azimuth sweep; evidence is never scaled.
    """

    old = _readonly_trajectory(old_world, "camera OLD trajectory")
    actual = _readonly_trajectory(actual_poses, "camera active-OLD actual")
    if not candidate_paths:
        raise ValueError("camera requires at least one candidate trajectory")
    candidates = tuple(
        _readonly_trajectory(path, f"camera candidate {index}")
        for index, path in enumerate(candidate_paths)
    )
    fov = _finite(horizontal_fov_deg, "camera horizontal FOV")
    aspect = _finite(viewport_aspect_ratio, "camera viewport aspect ratio")
    requested = _finite(
        candidate_geometry_fraction, "camera candidate geometry fraction"
    )
    minimum_distance = _finite(
        minimum_slant_distance_m, "camera minimum slant distance"
    )
    minimum_back = _finite(
        minimum_back_distance_m, "camera minimum back distance"
    )
    requested_eye_height = _finite(eye_height_m, "camera eye height")
    minimum_eye_height = _finite(
        minimum_eye_height_m, "camera minimum eye height"
    )
    maximum_height = _finite(maximum_eye_height_m, "camera maximum eye height")
    target_z = _finite(target_z_m, "camera target z")
    lateral_offset = _finite(lateral_offset_m, "camera lateral offset")
    coarse_step = _finite(
        azimuth_coarse_step_deg, "camera azimuth coarse step"
    )
    fine_step = _finite(azimuth_fine_step_deg, "camera azimuth fine step")
    containment = _finite(containment_limit_ndc, "camera containment NDC")
    minimum_depth = _finite(minimum_depth_m, "camera minimum depth")
    old_visual_z = _finite(old_path_z_m, "camera OLD path z")
    actual_ground_z = _finite(
        actual_path_ground_z_m, "camera actual-path ground z"
    )
    actual_visual_z = _finite(
        actual_path_visual_z_m, "camera actual-path visual z"
    )
    observation_visual_z = _finite(
        observation_marker_visual_z_m, "camera observation-marker z"
    )
    observation_radius = _finite(
        observation_marker_radius_m, "camera observation-marker radius"
    )
    preferred_eye: np.ndarray | None = None
    preferred_half_angle: float | None = None
    if preferred_eye_direction_xy is not None:
        preferred_eye = np.asarray(preferred_eye_direction_xy, dtype=np.float64)
        if preferred_eye.shape != (2,) or not np.all(np.isfinite(preferred_eye)):
            raise ValueError("camera preferred eye direction must be one finite XY vector")
        preferred_norm = float(np.linalg.norm(preferred_eye))
        if preferred_norm <= 1e-12:
            raise ValueError("camera preferred eye direction must be nonzero")
        preferred_eye = preferred_eye / preferred_norm
        preferred_half_angle = _finite(
            preferred_eye_half_angle_deg, "camera preferred eye half angle"
        )
        if not 0.0 < preferred_half_angle <= 180.0:
            raise ValueError("camera preferred eye half angle must lie in (0, 180]")
    if candidate_path_z_m is None:
        candidate_visual_z = (0.84,) * len(candidates)
    else:
        if len(candidate_path_z_m) != len(candidates):
            raise ValueError("camera candidate z count must match candidate paths")
        candidate_visual_z = tuple(
            _finite(value, f"camera candidate {index} z")
            for index, value in enumerate(candidate_path_z_m)
        )
    if not 30.0 <= fov <= 90.0:
        raise ValueError("camera horizontal FOV must be in [30, 90] degrees")
    if aspect <= 0.0:
        raise ValueError("camera viewport aspect ratio must be positive")
    if not 0.65 <= requested <= 0.80:
        raise ValueError("camera candidate geometry fraction must be in [0.65, 0.80]")
    if minimum_distance <= 0.0 or minimum_back <= 0.0:
        raise ValueError("camera minimum distances must be positive")
    if not target_z < minimum_eye_height <= maximum_height:
        raise ValueError("camera eye-height bounds must lie above target z")
    if requested_eye_height < minimum_eye_height:
        raise ValueError("camera requested eye height is below its indoor minimum")
    if not 0.0 < containment < 1.0 or minimum_depth <= 0.0:
        raise ValueError("camera projection limits are invalid")
    if not 0.0 < fine_step <= coarse_step <= 45.0:
        raise ValueError("camera azimuth steps must satisfy 0 < fine <= coarse <= 45")
    if min(
        old_visual_z,
        actual_ground_z,
        actual_visual_z,
        observation_visual_z,
        *candidate_visual_z,
    ) <= 0.0:
        raise ValueError("camera rendered path heights must be positive")
    if observation_radius <= 0.0:
        raise ValueError("camera observation-marker radius must be positive")
    applied_eye_height = min(requested_eye_height, maximum_height)

    candidate_xy = np.vstack([path[:, :2] for path in candidates])
    focus_xy = np.vstack((actual[:, :2], candidate_xy))
    drawn_xy = np.vstack((old[:, :2], focus_xy))

    def bounds_and_extent(
        xy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        minimum = np.min(xy, axis=0)
        maximum = np.max(xy, axis=0)
        return minimum, maximum, float(np.linalg.norm(maximum - minimum))

    candidate_min, candidate_max, candidate_extent = bounds_and_extent(candidate_xy)
    focus_min, focus_max, focus_extent = bounds_and_extent(focus_xy)
    drawn_min, drawn_max, drawn_extent = bounds_and_extent(drawn_xy)
    if candidate_extent <= 1e-9 or focus_extent <= 1e-9 or drawn_extent <= 1e-9:
        raise ValueError("camera geometry has no finite spatial extent")

    def xyz_at_height(path: np.ndarray, z: float) -> np.ndarray:
        return np.column_stack((path[:, :2], np.full(len(path), z)))

    candidate_xyz = np.vstack(
        [
            xyz_at_height(path, height)
            for path, height in zip(candidates, candidate_visual_z, strict=True)
        ]
    )
    focus_xyz = np.vstack(
        (
            candidate_xyz,
            xyz_at_height(actual, actual_ground_z),
            xyz_at_height(actual, actual_visual_z),
        )
    )
    path_xyz = np.vstack((xyz_at_height(old, old_visual_z), focus_xyz))

    # Clearpath Jackal's nominal footprint is about 0.51 x 0.43 m.  Include
    # every saved pose so a stable camera does not crop the moving robot.
    robot_points: list[list[float]] = []
    for x, y, yaw in actual:
        cosine = math.cos(float(yaw))
        sine = math.sin(float(yaw))
        for local_x in (-0.255, 0.255):
            for local_y in (-0.215, 0.215):
                world_x = float(x) + cosine * local_x - sine * local_y
                world_y = float(y) + sine * local_x + cosine * local_y
                for world_z in (0.05, 0.75):
                    robot_points.append([world_x, world_y, world_z])
    required_parts = [path_xyz, np.asarray(robot_points, dtype=np.float64)]
    if observation_pose_world_se2 is not None:
        observation = np.asarray(observation_pose_world_se2, dtype=np.float64)
        if observation.shape != (3,) or not np.all(np.isfinite(observation)):
            raise ValueError("camera observation pose must be one finite SE(2) pose")
        x, y = map(float, observation[:2])
        required_parts.append(
            np.asarray(
                [
                    [x, y, observation_visual_z],
                    [x - observation_radius, y, observation_visual_z],
                    [x + observation_radius, y, observation_visual_z],
                    [x, y - observation_radius, observation_visual_z],
                    [x, y + observation_radius, observation_visual_z],
                ],
                dtype=np.float64,
            )
        )
    required_xyz = np.vstack(required_parts)

    center = (focus_min + focus_max) / 2.0
    initial_heading_angle = float(actual[0, 2])
    target_array = np.array([center[0], center[1], target_z], dtype=np.float64)
    horizontal_fov = math.radians(fov)
    vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) / aspect)

    def projection(back_distance: float, azimuth_rad: float) -> dict[str, Any]:
        view_direction = np.array(
            [math.cos(azimuth_rad), math.sin(azimuth_rad)], dtype=np.float64
        )
        side = np.array([-view_direction[1], view_direction[0]], dtype=np.float64)
        eye_array = np.array(
            [
                center[0]
                - back_distance * view_direction[0]
                + lateral_offset * side[0],
                center[1]
                - back_distance * view_direction[1]
                + lateral_offset * side[1],
                applied_eye_height,
            ],
            dtype=np.float64,
        )
        forward = target_array - eye_array
        distance = float(np.linalg.norm(forward))
        forward /= distance
        right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)

        def project(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            relative = points - eye_array
            depth = relative @ forward
            with np.errstate(divide="ignore", invalid="ignore"):
                x_ndc = (relative @ right) / (depth * math.tan(horizontal_fov / 2.0))
                y_ndc = (relative @ up) / (depth * math.tan(vertical_fov / 2.0))
            return depth, x_ndc, y_ndc

        depth, x_ndc, y_ndc = project(required_xyz)
        candidate_depth, candidate_x, candidate_y = project(candidate_xyz)
        focus_depth, focus_x, focus_y = project(focus_xyz)

        def screen_span(x_values: np.ndarray, y_values: np.ndarray) -> float:
            return float(
                max(
                    (np.max(x_values) - np.min(x_values)) / 2.0,
                    (np.max(y_values) - np.min(y_values)) / 2.0,
                )
            )

        maximum_ndc = float(max(np.max(np.abs(x_ndc)), np.max(np.abs(y_ndc))))
        eye_direction = eye_array[:2] - target_array[:2]
        eye_direction /= np.linalg.norm(eye_direction)
        return {
            "eye": eye_array,
            "distance": distance,
            "minimum_depth": float(
                min(np.min(depth), np.min(candidate_depth), np.min(focus_depth))
            ),
            "maximum_ndc": maximum_ndc,
            "candidate_fraction": screen_span(candidate_x, candidate_y),
            "focus_fraction": screen_span(focus_x, focus_y),
            "azimuth_rad": float(azimuth_rad),
            "view_direction": view_direction,
            "eye_direction": eye_direction,
        }

    vertical_delta = applied_eye_height - target_z
    distance_floor_squared = max(
        minimum_distance * minimum_distance
        - lateral_offset * lateral_offset
        - vertical_delta * vertical_delta,
        0.0,
    )
    lower = max(minimum_back, math.sqrt(distance_floor_squared))

    def acceptable(result: Mapping[str, Any]) -> bool:
        return bool(
            result["minimum_depth"] >= minimum_depth
            and result["maximum_ndc"] <= containment
            and result["candidate_fraction"] <= 0.80
        )

    def fit_at_azimuth(azimuth_rad: float) -> tuple[dict[str, Any], float] | None:
        if preferred_eye is not None and preferred_half_angle is not None:
            nominal_eye = -np.asarray(
                [math.cos(azimuth_rad), math.sin(azimuth_rad)], dtype=np.float64
            )
            minimum_dot = math.cos(math.radians(preferred_half_angle))
            if float(nominal_eye @ preferred_eye) < minimum_dot - 1e-12:
                return None
        lower_result = projection(lower, azimuth_rad)
        if acceptable(lower_result):
            return lower_result, lower
        upper = lower
        upper_result = lower_result
        while upper < 50.0 and not acceptable(upper_result):
            upper *= 1.10
            upper_result = projection(upper, azimuth_rad)
        if not acceptable(upper_result):
            return None
        failed = lower
        passed = upper
        fitted = upper_result
        for _ in range(64):
            midpoint = 0.5 * (failed + passed)
            midpoint_result = projection(midpoint, azimuth_rad)
            if acceptable(midpoint_result):
                passed = midpoint
                fitted = midpoint_result
            else:
                failed = midpoint
        # The nearest containment-valid view can make candidate geometry larger
        # than the requested framing fraction.  Back off deterministically to
        # the request instead of merely accepting any value below the 0.80 cap.
        if float(fitted["candidate_fraction"]) > requested:
            near = passed
            far = passed
            far_result = fitted
            while far < 100.0 and float(far_result["candidate_fraction"]) > requested:
                far *= 1.10
                far_result = projection(far, azimuth_rad)
            if float(far_result["candidate_fraction"]) > requested:
                return None
            requested_fit = far_result
            for _ in range(64):
                midpoint = 0.5 * (near + far)
                midpoint_result = projection(midpoint, azimuth_rad)
                if float(midpoint_result["candidate_fraction"]) <= requested:
                    far = midpoint
                    requested_fit = midpoint_result
                else:
                    near = midpoint
            return requested_fit, far
        return fitted, passed

    def wrapped_offset(azimuth_rad: float) -> float:
        return math.atan2(
            math.sin(azimuth_rad - initial_heading_angle),
            math.cos(azimuth_rad - initial_heading_angle),
        )

    def camera_rank(item: tuple[dict[str, Any], float]) -> tuple[float, ...]:
        result, fitted_back = item
        fraction = float(result["candidate_fraction"])
        offset = abs(wrapped_offset(float(result["azimuth_rad"])))
        preference_offset = 0.0
        if preferred_eye is not None:
            preference_offset = math.acos(
                float(
                    np.clip(
                        np.asarray(result["eye_direction"], dtype=np.float64)
                        @ preferred_eye,
                        -1.0,
                        1.0,
                    )
                )
            )
        if fraction >= 0.65:
            return (
                0.0,
                abs(fraction - requested),
                preference_offset,
                offset,
                fitted_back,
            )
        return (1.0, -fraction, preference_offset, offset, fitted_back)

    coarse_count = max(8, int(math.ceil(360.0 / coarse_step)))
    coarse_angles = [
        initial_heading_angle + 2.0 * math.pi * index / coarse_count
        for index in range(coarse_count)
    ]
    coarse_fits = [
        fitted
        for angle in coarse_angles
        if (fitted := fit_at_azimuth(angle)) is not None
    ]
    if not coarse_fits:
        raise ValueError("no finite indoor camera fit contains EXP-02D context")
    coarse_best = min(coarse_fits, key=camera_rank)
    coarse_best_angle = float(coarse_best[0]["azimuth_rad"])
    fine_radius = 2.0 * math.pi / coarse_count
    fine_step_rad = math.radians(fine_step)
    fine_count_each_side = max(1, int(math.ceil(fine_radius / fine_step_rad)))
    fine_angles = [
        coarse_best_angle + index * fine_step_rad
        for index in range(-fine_count_each_side, fine_count_each_side + 1)
    ]
    fine_fits = [
        fitted
        for angle in fine_angles
        if (fitted := fit_at_azimuth(angle)) is not None
    ]
    chosen, back_distance = min((*coarse_fits, *fine_fits), key=camera_rank)

    eye_array = np.asarray(chosen["eye"], dtype=np.float64)
    eye = tuple(map(float, eye_array))
    target = tuple(map(float, target_array))
    distance = float(chosen["distance"])
    horizontal_distance = float(np.linalg.norm(target_array[:2] - eye_array[:2]))
    elevation = math.degrees(math.atan2(vertical_delta, horizontal_distance))
    candidate_fraction = float(chosen["candidate_fraction"])
    focus_fraction = float(chosen["focus_fraction"])
    achieved = bool(0.65 <= candidate_fraction <= 0.80)
    limiting_constraint = (
        "CANDIDATE_TARGET"
        if achieved
        else "FULL_CONTEXT_AND_JACKAL_VISIBILITY_LIMIT"
    )
    values = np.asarray(
        (
            *eye,
            *target,
            elevation,
            distance,
            candidate_fraction,
            focus_fraction,
            float(chosen["maximum_ndc"]),
            float(chosen["minimum_depth"]),
        ),
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("computed EXP-02D camera plan is non-finite")
    return Exp02DCameraPlan(
        eye_xyz=eye,
        target_xyz=target,
        horizontal_fov_deg=float(fov),
        vertical_fov_deg=float(math.degrees(vertical_fov)),
        viewport_aspect_ratio=float(aspect),
        elevation_deg=float(elevation),
        slant_distance_m=float(distance),
        back_distance_m=float(back_distance),
        lateral_offset_m=float(lateral_offset),
        nominal_view_direction_xy=tuple(
            map(float, np.asarray(chosen["view_direction"], dtype=np.float64))
        ),
        eye_direction_xy=tuple(
            map(float, np.asarray(chosen["eye_direction"], dtype=np.float64))
        ),
        preferred_eye_direction_xy=(
            None if preferred_eye is None else tuple(map(float, preferred_eye))
        ),
        preferred_eye_half_angle_deg=(
            None if preferred_half_angle is None else float(preferred_half_angle)
        ),
        azimuth_world_deg=float(
            math.degrees(float(chosen["azimuth_rad"])) % 360.0
        ),
        azimuth_offset_from_initial_heading_deg=float(
            math.degrees(wrapped_offset(float(chosen["azimuth_rad"])))
        ),
        azimuth_search_coarse_step_deg=float(360.0 / coarse_count),
        azimuth_search_fine_step_deg=float(fine_step),
        azimuth_candidates_evaluated=len(coarse_fits) + len(fine_fits),
        minimum_slant_distance_m=float(minimum_distance),
        minimum_back_distance_m=float(minimum_back),
        maximum_eye_height_m=float(maximum_height),
        containment_limit_ndc=float(containment),
        maximum_projected_ndc=float(chosen["maximum_ndc"]),
        minimum_projected_depth_m=float(chosen["minimum_depth"]),
        old_path_z_m=float(old_visual_z),
        actual_path_ground_z_m=float(actual_ground_z),
        actual_path_visual_z_m=float(actual_visual_z),
        observation_marker_visual_z_m=float(observation_visual_z),
        candidate_path_z_m=tuple(map(float, candidate_visual_z)),
        candidate_bounds_xy=(
            tuple(map(float, candidate_min)),
            tuple(map(float, candidate_max)),
        ),
        focus_bounds_xy=(
            tuple(map(float, focus_min)),
            tuple(map(float, focus_max)),
        ),
        drawn_bounds_xy=(
            tuple(map(float, drawn_min)),
            tuple(map(float, drawn_max)),
        ),
        projected_candidate_geometry_fraction=float(candidate_fraction),
        projected_focus_geometry_fraction=float(focus_fraction),
        requested_candidate_geometry_fraction=float(requested),
        candidate_fraction_target_achieved=achieved,
        full_context_and_jackal_contained=bool(
            chosen["maximum_ndc"] <= containment
            and chosen["minimum_depth"] >= minimum_depth
        ),
        limiting_constraint=limiting_constraint,
    )


def validate_gui_capture_rgb(
    rgb: np.ndarray,
    *,
    required_palette: Mapping[str, Sequence[float]] | None = None,
    require_jackal: bool = False,
    minimum_palette_pixels: int = 16,
    minimum_jackal_pixels: int = 500,
) -> dict[str, Any]:
    """Reject blank/occluded captures missing requested visible evidence.

    Palette checks target the unlit DebugDraw RGB values within a small tolerance.
    The Jackal check uses its large yellow/olive body panels across direct and
    oblique lighting; later yellow point markers are too small to satisfy the
    minimum on their own.
    """

    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("GUI capture must be a non-empty H x W x 3 RGB image")
    if not np.issubdtype(image.dtype, np.number) or not np.all(np.isfinite(image)):
        raise ValueError("GUI capture contains non-finite/non-numeric pixels")
    values = image.astype(np.float64, copy=False)
    if np.min(values) < 0.0 or np.max(values) > 255.0:
        raise ValueError("GUI capture RGB values must lie in [0, 255]")
    luminance = np.mean(values, axis=2)
    chroma = np.max(values, axis=2) - np.min(values, axis=2)
    dark_scene_fraction = float(np.mean(luminance < 220.0))
    chromatic_fraction = float(np.mean(chroma > 40.0))
    red = values[:, :, 0]
    green = values[:, :, 1]
    blue = values[:, :, 2]
    jackal_yellow = (
        (red > 100.0)
        & (green > 90.0)
        & (blue < 80.0)
        & (np.minimum(red, green) - blue > 40.0)
    )
    palette_pixel_counts: dict[str, int] = {}
    if required_palette is not None:
        for name, color in required_palette.items():
            target = np.asarray(color, dtype=np.float64)
            if target.shape not in ((3,), (4,)) or not np.all(np.isfinite(target)):
                raise ValueError(f"invalid GUI palette color for {name}")
            target = target[:3]
            if np.max(target) <= 1.0:
                target = 255.0 * target
            if np.min(target) < 0.0 or np.max(target) > 255.0:
                raise ValueError(f"GUI palette color for {name} lies outside [0, 255]")
            count = int(
                np.sum(np.max(np.abs(values - target[None, None, :]), axis=2) <= 12.0)
            )
            palette_pixel_counts[str(name)] = count
            if count < minimum_palette_pixels:
                raise ValueError(
                    f"GUI capture lacks visible {name} evidence "
                    f"({count} < {minimum_palette_pixels} pixels)"
                )
    jackal_pixel_count = int(np.sum(jackal_yellow))
    metrics: dict[str, Any] = {
        "mean_luminance": float(np.mean(luminance)),
        "p05_luminance": float(np.percentile(luminance, 5.0)),
        "dark_scene_fraction": dark_scene_fraction,
        "chromatic_fraction": chromatic_fraction,
        "jackal_yellow_pixel_count": jackal_pixel_count,
        "palette_pixel_counts": palette_pixel_counts,
    }
    if dark_scene_fraction < 0.005 or chromatic_fraction < 0.0001:
        raise ValueError(
            "GUI capture is blank/occluded: insufficient scene contrast or colored evidence"
        )
    if require_jackal and jackal_pixel_count < minimum_jackal_pixels:
        raise ValueError(
            "GUI capture lacks a visible Jackal yellow body "
            f"({jackal_pixel_count} < {minimum_jackal_pixels} pixels)"
        )
    return metrics


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
