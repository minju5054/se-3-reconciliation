#!/usr/bin/env python3
"""Strict read-only validator for a completed EXP-02D primary result."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.exp02d_analysis import (  # noqa: E402
    RegimeThresholds,
    cluster_bootstrap_pair_balanced_difference,
    command_discontinuity,
    load_development_corpus,
    m4_false_correction_rescued,
    ordered_pair_groups,
    pair_balanced_mean,
    regime_memberships,
    select_representatives,
    success_failure_label,
    transition_weighted_mean,
)
from reconciliation.exp02d_lookahead_direction import (  # noqa: E402
    METHODS,
    METHOD_FACTORS,
    candidate_command_metrics,
)
from reconciliation.online_switch import sha256_file  # noqa: E402


PLOT_NAMES = (
    "01_method_command_score_pair_balanced.png",
    "02_benign_preservation.png",
    "03_challenging_outcomes.png",
    "04_alpha_entry_vs_alpha_look.png",
    "05_geometry_semantics_gain.png",
    "06_deformation_vs_benefit.png",
    "07_rigid_fit_distribution.png",
    "08_success_failure_by_fresh_geometry.png",
    "09_success_failure_by_latency.png",
    "10_pair_frequency_vs_method_effect.png",
)
METHOD_SHORT = {
    "M0_RAW": "raw",
    "M1_HISTORICAL_M4": "m1",
    "M2_NO_DIRECTION": "m2",
    "M3_LOOKAHEAD": "m3",
}
SUMMARY_FILES = (
    "summary/aggregate.json",
    "summary/pair_balanced.json",
    "summary/success_failure_counts.json",
    "summary/regime_analysis.json",
    "summary/representatives.json",
    "summary/oracle_follower_reconstruction.json",
)
SUCCESS_FAILURE_LABELS = (
    "BENIGN_PRESERVED",
    "BENIGN_BROKEN",
    "CHALLENGING_RESCUED",
    "CHALLENGING_IMPROVED",
    "CHALLENGING_WORSE",
    "CHALLENGING_MIXED",
    "GEOMETRY_UNDEFINED",
    "SOLVER_FAILURE",
)
DEFORMATION_FIELDS = (
    "translation_deformation_rms_m",
    "yaw_deformation_rms_rad",
    "entry_displacement_m",
    "q_node_displacement_m",
    "endpoint_displacement_m",
    "fresh_relative_motion_translation_rms_m",
    "fresh_relative_motion_yaw_rms_rad",
    "rigid_fit_translation_rms_m",
    "rigid_fit_yaw_rms_rad",
)
CSV_IDENTITY_FIELDS = (
    "corpus_transition_id",
    "cohort_id",
    "episode_id",
    "transition_index",
    "template_id",
    "variant_id",
    "semantic_family",
    "physical_region",
    "ordered_raw_pair_sha256",
    "ordered_raw_pair_frequency",
    "fresh_geometry_bin",
    "raw_difficulty_bin",
    "k_fresh",
    "q_fresh",
    "q_minus_k",
    "alpha_entry_rad",
    "alpha_look_rad",
    "alpha_entry_minus_alpha_look_rad",
    "d_k_m",
    "d_q_m",
    "fresh_arc_k_to_q_m",
    "observation_to_boundary_translation_m",
    "observation_to_boundary_yaw_rad",
    "p_to_b_translation_m",
    "p_to_b_yaw_rad",
    "fresh_net_yaw_rad",
    "fresh_lateral_excursion_m",
    "host_latency_s",
    "effective_latency_s",
    "inference_translation_m",
    "geometry_undefined",
    "undefined_statuses",
    "solver_failure",
    "solver_failures",
    "success_failure_label",
    "m4_false_correction_rescued",
    "regimes",
)
CSV_METHOD_SUFFIXES = (
    "delta_v_signed_mps",
    "delta_omega_signed_rps",
    "delta_v_abs_mps",
    "delta_omega_abs_rps",
    "difficulty_bin",
    "candidate_nearest_index",
    "candidate_target_index",
    "candidate_target_changed_from_raw_q",
    *DEFORMATION_FIELDS,
    "J_cmd",
)
CSV_FIELDS = CSV_IDENTITY_FIELDS + tuple(
    f"{method}_{suffix}" for method in METHODS for suffix in CSV_METHOD_SUFFIXES
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument(
        "--source-repository-root",
        type=Path,
        help="override the immutable DATA-02 repository recorded in provenance",
    )
    return parser.parse_args()


def strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
    )
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def strict_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def integer(value: Any, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be an integer") from error
    if str(result) != str(value) and not isinstance(value, int):
        try:
            if float(value) != result:
                raise ValueError(f"{name} must be an integer")
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} must be an integer") from error
    if result < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return result


def close(actual: Any, expected: Any, name: str, *, atol: float = 1e-12) -> None:
    if not math.isclose(
        finite(actual, name),
        finite(expected, f"expected {name}"),
        rel_tol=1e-11,
        abs_tol=atol,
    ):
        raise ValueError(f"{name} mismatch: {actual} != {expected}")


def _same_json_value(actual: Any, expected: Any, name: str) -> None:
    """Compare regenerated JSON-like values while requiring finite numbers."""

    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping) or set(actual) != set(expected):
            raise ValueError(f"{name} mapping keys mismatch")
        for key, value in expected.items():
            _same_json_value(actual[key], value, f"{name}.{key}")
        return
    if isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
            raise ValueError(f"{name} sequence shape mismatch")
        for index, value in enumerate(expected):
            _same_json_value(actual[index], value, f"{name}[{index}]")
        return
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        if actual != expected or type(actual) is not type(expected):
            raise ValueError(f"{name} mismatch: {actual!r} != {expected!r}")
        return
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        close(actual, expected, name)
        return
    if actual != expected:
        raise ValueError(f"{name} mismatch")


def resolve_inside(root: Path, relative: str, name: str) -> Path:
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe {name}: {relative}")
    result = (root / path).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f"{name} escapes result root: {relative}")
    return result


def _canonical_files(run: Path) -> set[str]:
    return {
        str(path.relative_to(run))
        for path in run.rglob("*")
        if path.is_file()
        and path.relative_to(run) != Path("result_manifest.json")
        and path.relative_to(run).parts[0] != "gui"
    }


def validate_result_manifest(run: Path) -> dict[str, str]:
    """Validate exact canonical inventory and hashes; ignore later GUI captures only."""

    manifest_path = run / "result_manifest.json"
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise ValueError("result manifest is missing or symlinked")
    document = strict_json(manifest_path)
    if document.get("schema") != "EXP02D_ResultManifest_v1":
        raise ValueError("unexpected EXP-02D result-manifest schema")
    hashes = document.get("artifact_sha256")
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError("result manifest has no artifact hash map")
    if integer(document.get("file_count_excluding_manifest"), "manifest file count") != len(hashes):
        raise ValueError("result manifest file count differs from its hash map")
    recorded = set(hashes)
    actual = _canonical_files(run)
    if recorded != actual:
        missing = sorted(recorded - actual)
        unexpected = sorted(actual - recorded)
        raise ValueError(
            f"result manifest inventory mismatch: missing={missing}, unexpected={unexpected}"
        )
    validated: dict[str, str] = {}
    for relative, expected in sorted(hashes.items()):
        path = resolve_inside(run, relative, "manifest artifact")
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"manifest artifact is missing or symlinked: {relative}")
        digest = sha256_file(path)
        if digest != str(expected):
            raise ValueError(f"result artifact hash mismatch: {relative}")
        validated[relative] = digest
    return validated


def _csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames
    if not rows or fieldnames is None:
        raise ValueError("EXP-02D transition summary is empty")
    if tuple(fieldnames) != CSV_FIELDS:
        raise ValueError("EXP-02D transition-summary CSV schema/order changed")
    identities = [row.get("corpus_transition_id", "") for row in rows]
    if any(not value for value in identities) or len(set(identities)) != len(identities):
        raise ValueError("transition summary identities are empty or duplicated")
    return rows


def _csv_bool(value: Any, name: str) -> bool:
    if value == "True":
        return True
    if value == "False":
        return False
    raise ValueError(f"{name} must be serialized as True or False")


def _assert_source_separation(run: Path, corpus: Any) -> None:
    roots = [corpus.index.combined_root]
    roots.extend(source.run for source in corpus.index.sources.values())
    result = run.resolve()
    for source in roots:
        immutable = source.resolve()
        if result == immutable or result.is_relative_to(immutable) or immutable.is_relative_to(result):
            raise ValueError(f"EXP-02D result overlaps immutable source tree: {immutable}")


def _validate_run_identity(
    run: Path,
    config: Mapping[str, Any],
    provenance: Mapping[str, Any],
    metadata: Mapping[str, Any],
    corpus_manifest: Mapping[str, Any],
    corpus: Any,
) -> None:
    expected_provenance_keys = {
        "schema",
        "exp02d_protocol_git_sha",
        "git_status_at_primary_start",
        "primary_started_from_clean_protocol_worktree",
        "config_source",
        "config_snapshot_sha256",
        "source_repository_root",
        "immutable_source_sha256",
        "combined_manifest_sha256",
        "data02_combined_decision",
        "corpus_role",
    }
    expected_metadata_keys = {
        "schema",
        "experiment",
        "run_id",
        "created_utc",
        "technical_status",
        "exp02d_protocol_git_sha",
        "eligible_moving_count",
        "reconstruction_valid_count",
        "geometry_undefined_count",
        "solver_valid_count",
        "solver_failure_count",
        "method_order",
        "candidate_methods_physically_executed",
        "command_score_in_objective",
        "intrinsic_waypoint_time_base",
        "pair_balanced_primary",
        "transition_weighted_secondary",
        "representatives_selected_after_complete_primary",
        "summary_files",
        "representatives",
    }
    expected_corpus_manifest_keys = {
        "schema",
        "corpus_role",
        "final_test_set",
        "eligible_moving_count",
        "reconstruction_valid_count",
        "input_reconstruction_failures",
        "undefined_geometry",
        "ordered_raw_pair_count",
        "ordered_raw_pair_frequencies",
        "oracle_follower_reconstruction",
        "solver_failures",
        "transition_artifacts",
    }
    if set(provenance) != expected_provenance_keys:
        raise ValueError("EXP-02D provenance schema changed")
    if set(metadata) != expected_metadata_keys:
        raise ValueError("EXP-02D metadata schema changed")
    if set(corpus_manifest) != expected_corpus_manifest_keys:
        raise ValueError("EXP-02D corpus-manifest schema changed")
    if config.get("experiment") != "EXP-02D" or tuple(config.get("methods", ())) != METHODS:
        raise ValueError("EXP-02D config identity/method order changed")
    scope = config.get("scope", {})
    if (
        scope.get("data02_combined_decision")
        != "DATA02_COMBINED_DIVERSITY_INSUFFICIENT"
        or scope.get("final_test_set") is not False
        or scope.get("corpus_role") != "EXP02D_DEVELOPMENT_CORPUS"
        or scope.get("candidate_execution") is not False
        or scope.get("lightnav_waypoint_timestamps") is not False
        or scope.get("only_semantic_change")
        != "B_to_F_k_direction_target_becomes_B_to_F_q"
    ):
        raise ValueError("development-only DATA-02 decision changed")
    if provenance.get("schema") != "EXP02D_Provenance_v1":
        raise ValueError("unexpected EXP-02D provenance schema")
    if provenance.get("primary_started_from_clean_protocol_worktree") is not True or provenance.get("git_status_at_primary_start") != "":
        raise ValueError("EXP-02D primary was not recorded from a clean protocol tree")
    if provenance.get("data02_combined_decision") != scope["data02_combined_decision"] or provenance.get("corpus_role") != "EXP02D_DEVELOPMENT_CORPUS":
        raise ValueError("EXP-02D provenance changed the corpus interpretation")
    if provenance.get("config_snapshot_sha256") != sha256_file(run / "config_snapshot.yaml"):
        raise ValueError("EXP-02D config snapshot hash mismatch")
    if provenance.get("combined_manifest_sha256") != corpus.index.combined_manifest_sha256:
        raise ValueError("EXP-02D combined-manifest provenance mismatch")
    if provenance.get("immutable_source_sha256") != config.get("source_integrity", {}):
        # expected counts are not file hashes and are intentionally absent from provenance.
        expected_hashes = {
            key: value
            for key, value in config.get("source_integrity", {}).items()
            if key.endswith("_sha256")
        }
        if provenance.get("immutable_source_sha256") != expected_hashes:
            raise ValueError("EXP-02D immutable source-hash provenance mismatch")
    sha = str(provenance.get("exp02d_protocol_git_sha", ""))
    if len(sha) != 40 or any(character not in "0123456789abcdef" for character in sha):
        raise ValueError("invalid EXP-02D protocol git SHA")
    if metadata.get("schema") != "EXP02D_Run_v1" or metadata.get("experiment") != "EXP-02D":
        raise ValueError("unexpected EXP-02D run metadata schema")
    if metadata.get("technical_status") != "EXP02D_RUN_COMPLETE":
        raise ValueError("EXP-02D technical status is not complete")
    if metadata.get("run_id") != run.name or metadata.get("exp02d_protocol_git_sha") != sha:
        raise ValueError("EXP-02D run/protocol identity mismatch")
    required_false = (
        "candidate_methods_physically_executed",
        "command_score_in_objective",
        "intrinsic_waypoint_time_base",
    )
    if any(metadata.get(key) is not False for key in required_false):
        raise ValueError("candidate execution/objective/waypoint-time limitation changed")
    if (
        metadata.get("pair_balanced_primary") is not True
        or metadata.get("transition_weighted_secondary") is not True
        or metadata.get("representatives_selected_after_complete_primary") is not True
        or tuple(metadata.get("method_order", ())) != METHODS
        or tuple(metadata.get("summary_files", ())) != SUMMARY_FILES
    ):
        raise ValueError("EXP-02D frozen analysis protocol flags changed")
    if corpus_manifest.get("schema") != "EXP02D_DevelopmentCorpusManifest_v1":
        raise ValueError("unexpected EXP-02D corpus-manifest schema")
    if corpus_manifest.get("corpus_role") != "EXP02D_DEVELOPMENT_CORPUS" or corpus_manifest.get("final_test_set") is not False:
        raise ValueError("EXP-02D corpus manifest is not development-only")
    eligible = corpus.index.eligible_count
    valid = corpus.valid_count
    for document, key, expected in (
        (metadata, "eligible_moving_count", eligible),
        (metadata, "reconstruction_valid_count", valid),
        (corpus_manifest, "eligible_moving_count", eligible),
        (corpus_manifest, "reconstruction_valid_count", valid),
    ):
        if integer(document.get(key), key) != expected:
            raise ValueError(f"{key} differs from recomputed immutable source count")
    if corpus.failures or corpus_manifest.get("input_reconstruction_failures") != []:
        raise ValueError("completed EXP-02D result contains input reconstruction failures")


def _validate_input_and_oracle(
    destination: Path,
    artifact: Mapping[str, Any],
    source: Any,
    config: Mapping[str, Any],
) -> None:
    reference_path = destination / "input_reference.json"
    oracle_path = destination / "oracle_indices.json"
    if sha256_file(reference_path) != artifact.get("input_reference_sha256"):
        raise ValueError("transition input-reference hash mismatch")
    if sha256_file(oracle_path) != artifact.get("oracle_indices_sha256"):
        raise ValueError("transition oracle hash mismatch")
    reference = strict_json(reference_path)
    oracle = strict_json(oracle_path)
    expected_reference_keys = {
        "schema",
        "corpus_transition_id",
        "corpus_role",
        "source_run",
        "source_transition",
        "source_transition_sha256",
        "source_artifact_sha256",
        "cohort_id",
        "episode_id",
        "transition_id",
        "transition_index",
        "template_id",
        "variant_id",
        "semantic_family",
        "physical_region",
        "old_chunk_id",
        "fresh_chunk_id",
        "ordered_raw_pair_sha256",
        "old_shape",
        "fresh_shape",
        "P_world_se2",
        "B_world_se2",
        "observation_world_se2",
        "t_obs_sim_s",
        "t_ready_sim_s",
        "t_switch_sim_s",
        "host_latency_s",
        "effective_latency_s",
        "old_last_desired_v_omega",
        "saved_raw_fresh_first_desired_v_omega",
        "coordinate_frame",
        "raw_local_axes",
        "waypoint_dt",
        "fresh_anchor",
    }
    expected_oracle_keys = {
        "schema",
        "k_fresh",
        "q_fresh",
        "q_minus_k",
        "lookahead_distance_m",
        "follower_config",
        "reconstructed_first_desired_v_omega",
        "saved_first_desired_v_omega",
        "saved_command_abs_error_v_omega",
        "saved_command_max_abs_error",
        "selected_suffix_first_desired_v_omega",
        "selected_suffix_saved_command_max_abs_error",
        "selected_suffix_absolute_nearest_index",
        "selected_suffix_absolute_target_index",
        "selected_suffix_indices_match_raw_k_q",
        "call_count_at_B",
        "experimental_oracle",
        "final_correspondence_detector",
    }
    if set(reference) != expected_reference_keys or set(oracle) != expected_oracle_keys:
        raise ValueError("transition input-reference/oracle schema changed")
    if reference.get("schema") != "EXP02D_TransitionInputReference_v1" or reference.get("corpus_transition_id") != source.corpus_transition_id:
        raise ValueError("transition input-reference identity mismatch")
    if reference.get("corpus_role") != "EXP02D_DEVELOPMENT_CORPUS" or reference.get("waypoint_dt") is not None:
        raise ValueError("transition input-reference semantics changed")
    if (
        reference.get("coordinate_frame")
        != "Isaac world x/y metres; yaw radians CCW about +Z"
        or reference.get("raw_local_axes")
        != ["forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive"]
        or reference.get("fresh_anchor")
        != "robot world pose at final FRESH observation; B not inserted"
    ):
        raise ValueError("transition coordinate-frame semantics changed")
    scalar_identity = {
        "cohort_id": source.cohort_id,
        "episode_id": source.episode_id,
        "transition_id": source.transition_id,
        "transition_index": source.transition_index,
        "template_id": source.template_id,
        "variant_id": source.variant_id,
        "semantic_family": source.semantic_family,
        "physical_region": source.physical_region,
        "old_chunk_id": source.old_chunk_id,
        "fresh_chunk_id": source.fresh_chunk_id,
        "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
        "source_transition_sha256": source.source_transition_sha256,
    }
    for key, expected in scalar_identity.items():
        if reference.get(key) != expected:
            raise ValueError(f"transition input-reference mismatch: {key}")
    if Path(str(reference.get("source_run"))).resolve() != source.source_run.resolve() or Path(str(reference.get("source_transition"))).resolve() != source.source_transition.resolve():
        raise ValueError("transition input-reference source path mismatch")
    if reference.get("source_artifact_sha256") != source.source_artifact_sha256:
        raise ValueError("transition input-reference source artifact hashes changed")
    arrays = (
        (reference.get("P_world_se2"), source.pose_before_boundary, "P"),
        (reference.get("B_world_se2"), source.boundary_pose, "B"),
        (reference.get("observation_world_se2"), source.observation_pose, "observation"),
        (reference.get("old_last_desired_v_omega"), source.old_desired_command, "OLD command"),
        (
            reference.get("saved_raw_fresh_first_desired_v_omega"),
            source.raw_fresh_first_desired_command,
            "raw-FRESH command",
        ),
    )
    for actual, expected, name in arrays:
        if not np.array_equal(np.asarray(actual, dtype=np.float64), expected):
            raise ValueError(f"transition input-reference {name} mismatch")
    for key, expected in (
        ("t_obs_sim_s", source.t_obs_sim_s),
        ("t_ready_sim_s", source.t_ready_sim_s),
        ("t_switch_sim_s", source.t_switch_sim_s),
        ("host_latency_s", source.host_latency_s),
        ("effective_latency_s", source.effective_latency_s),
    ):
        close(reference.get(key), expected, key)
    if reference.get("old_shape") != list(source.old_world.shape) or reference.get("fresh_shape") != list(source.fresh_world.shape):
        raise ValueError("transition input-reference shape mismatch")
    if oracle.get("schema") != "EXP02D_FrozenFollowerOracle_v1":
        raise ValueError("unexpected follower-oracle schema")
    expected_oracle = {
        "k_fresh": source.k_fresh,
        "q_fresh": source.q_fresh,
        "q_minus_k": source.q_relative,
        "call_count_at_B": 1,
        "experimental_oracle": True,
        "final_correspondence_detector": False,
    }
    for key, expected in expected_oracle.items():
        if oracle.get(key) != expected:
            raise ValueError(f"follower-oracle mismatch: {key}")
    if oracle.get("follower_config") != asdict(source.follower_config):
        raise ValueError("follower-oracle config differs from immutable source")
    close(oracle.get("lookahead_distance_m"), source.follower_config.lookahead_distance_m, "lookahead distance")
    if not np.array_equal(
        np.asarray(oracle.get("reconstructed_first_desired_v_omega"), dtype=np.float64),
        source.raw_fresh_first_desired_command,
    ):
        raise ValueError("follower-oracle command mismatch")
    reconstructed = np.asarray(
        oracle.get("reconstructed_first_desired_v_omega"), dtype=np.float64
    )
    saved = np.asarray(oracle.get("saved_first_desired_v_omega"), dtype=np.float64)
    selected = np.asarray(
        oracle.get("selected_suffix_first_desired_v_omega"), dtype=np.float64
    )
    if saved.shape != (2,) or selected.shape != (2,) or not np.array_equal(
        saved, source.raw_fresh_first_desired_command
    ):
        raise ValueError("follower-oracle saved command mismatch")
    errors = np.abs(reconstructed - saved)
    if not np.array_equal(
        np.asarray(oracle.get("saved_command_abs_error_v_omega"), dtype=np.float64),
        errors,
    ):
        raise ValueError("follower-oracle saved command error vector mismatch")
    close(
        oracle.get("saved_command_max_abs_error"),
        np.max(errors),
        "saved command maximum error",
    )
    selected_error = float(
        np.max(np.abs(selected - source.raw_fresh_first_desired_command))
    )
    close(
        oracle.get("selected_suffix_saved_command_max_abs_error"),
        selected_error,
        "selected-suffix saved command maximum error",
    )
    if (
        integer(oracle.get("selected_suffix_absolute_nearest_index"), "suffix k")
        != source.k_fresh
        or integer(oracle.get("selected_suffix_absolute_target_index"), "suffix q")
        != source.q_fresh
        or oracle.get("selected_suffix_indices_match_raw_k_q") is not True
    ):
        raise ValueError("follower-oracle selected-suffix k/q mismatch")
    tolerance = float(config["oracle"]["first_command_absolute_tolerance"])
    if max(float(np.max(errors)), selected_error) > tolerance:
        raise ValueError("follower-oracle command error exceeds frozen tolerance")


def _validate_method(
    method_root: Path,
    method: str,
    hashes: Mapping[str, Any],
    source: Any,
    config: Mapping[str, Any],
) -> Mapping[str, Any]:
    if not method_root.is_dir() or method_root.is_symlink():
        raise ValueError(f"{method} artifact directory is missing or symlinked")
    expected_names = {"candidate.npy", "metrics.json"}
    if method != "M0_RAW":
        expected_names.add("optimization.json")
    actual_names = {path.name for path in method_root.iterdir() if path.is_file()}
    if actual_names != expected_names:
        raise ValueError(f"{method} artifact inventory mismatch")
    candidate_path = method_root / "candidate.npy"
    metrics_path = method_root / "metrics.json"
    candidate_hash = sha256_file(candidate_path)
    if candidate_hash != hashes.get("candidate_sha256") or sha256_file(metrics_path) != hashes.get("metrics_sha256"):
        raise ValueError(f"{method} artifact hash mismatch")
    candidate = np.load(candidate_path, allow_pickle=False)
    expected_shape = (source.fresh_world.shape[0] - source.k_fresh, 3)
    if candidate.shape != expected_shape or not np.issubdtype(candidate.dtype, np.number) or not np.all(np.isfinite(candidate)):
        raise ValueError(f"{method} candidate is not finite selected-suffix SE(2)")
    if method == "M0_RAW" and not np.array_equal(candidate, source.fresh_world[source.k_fresh :]):
        raise ValueError("M0_RAW candidate differs from raw selected FRESH suffix")
    metrics = strict_json(metrics_path)
    expected_metric_keys = {
        "schema",
        "method",
        "active_factors",
        "candidate_sha256",
        "command",
        "deformation",
        "candidate_nearest_index",
        "candidate_target_index",
        "raw_target_index_q",
        "effective_target_changed_from_raw_q",
        "candidate_physically_executed",
        "J_cmd_in_objective",
    }
    if set(metrics) != expected_metric_keys:
        raise ValueError(f"{method} metrics schema changed")
    if metrics.get("schema") != "EXP02D_MethodMetrics_v1" or metrics.get("method") != method:
        raise ValueError(f"{method} metrics identity mismatch")
    if metrics.get("active_factors") != list(METHOD_FACTORS[method]):
        raise ValueError(f"{method} active factor set changed")
    if metrics.get("candidate_sha256") != candidate_hash:
        raise ValueError(f"{method} metrics candidate hash mismatch")
    if metrics.get("candidate_physically_executed") is not False or metrics.get("J_cmd_in_objective") is not False:
        raise ValueError(f"{method} incorrectly claims physical execution or J_cmd objective use")
    if any("command" in str(factor).lower() for factor in metrics["active_factors"]):
        raise ValueError(f"{method} contains a command objective factor")
    command = metrics.get("command")
    if not isinstance(command, dict) or command.get("normalization", {}).get("evaluation_only_not_objective") is not True:
        raise ValueError(f"{method} command metric is not marked evaluation-only")
    reconstructed = command_discontinuity(
        command.get("delta_v_signed_mps"), command.get("delta_omega_signed_rps")
    )
    close(command.get("J_cmd"), reconstructed.j_cmd, f"{method} J_cmd")
    if command.get("difficulty") != reconstructed.difficulty:
        raise ValueError(f"{method} command difficulty mismatch")
    expected_command = candidate_command_metrics(
        candidate,
        boundary=source.boundary_pose,
        old_last_command=source.old_desired_command,
        follower_values=asdict(source.follower_config),
        original_entry_index=source.k_fresh,
        raw_target_index=source.q_fresh,
        linear_normalization_mps=float(
            config["evaluation"]["command_score_linear_scale_mps"]
        ),
        angular_normalization_rps=float(
            config["evaluation"]["command_score_angular_scale_rps"]
        ),
    )
    expected_command["difficulty"] = reconstructed.difficulty
    _same_json_value(command, expected_command, f"{method} command")
    nearest = integer(metrics.get("candidate_nearest_index"), f"{method} nearest index")
    target = integer(metrics.get("candidate_target_index"), f"{method} target index")
    if not source.k_fresh <= nearest <= target < source.fresh_world.shape[0]:
        raise ValueError(f"{method} post-hoc follower indices are out of bounds")
    if integer(metrics.get("raw_target_index_q"), f"{method} raw q") != source.q_fresh:
        raise ValueError(f"{method} raw q changed")
    if metrics.get("effective_target_changed_from_raw_q") is not (target != source.q_fresh):
        raise ValueError(f"{method} target-change diagnostic mismatch")
    deformation = metrics.get("deformation")
    if not isinstance(deformation, dict):
        raise ValueError(f"{method} deformation metrics are missing")
    # Traverse every numeric leaf and reject NaN/Inf without constraining labels.
    stack: list[Any] = [deformation]
    while stack:
        value = stack.pop()
        if isinstance(value, dict):
            stack.extend(value.values())
        elif isinstance(value, list):
            stack.extend(value)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            finite(value, f"{method} deformation metric")
    optimization_path = method_root / "optimization.json"
    if method == "M0_RAW":
        if hashes.get("optimization_sha256") is not None or optimization_path.exists():
            raise ValueError("M0_RAW must not have an optimization artifact")
        return metrics
    if sha256_file(optimization_path) != hashes.get("optimization_sha256"):
        raise ValueError(f"{method} optimization hash mismatch")
    optimization = strict_json(optimization_path)
    expected_optimization_keys = {
        "initial_cost",
        "final_cost",
        "iterations",
        "converged",
        "termination_reason",
        "cost_history",
        "damping_history",
        "active_factors",
        "solver",
        "right_local_retraction",
    }
    if set(optimization) != expected_optimization_keys:
        raise ValueError(f"{method} optimization schema changed")
    if optimization.get("active_factors") != list(METHOD_FACTORS[method]):
        raise ValueError(f"{method} optimization factor set changed")
    if optimization.get("solver") != "frozen solve_least_squares" or optimization.get("right_local_retraction") is not True:
        raise ValueError(f"{method} solver semantics changed")
    if optimization.get("converged") is not True:
        raise ValueError(f"{method} saved non-converged optimization")
    for key in ("initial_cost", "final_cost"):
        finite(optimization.get(key), f"{method} {key}")
    integer(optimization.get("iterations"), f"{method} iterations")
    for key in ("cost_history", "damping_history"):
        values = optimization.get(key)
        if not isinstance(values, list) or (key == "cost_history" and not values):
            raise ValueError(f"{method} {key} must be a valid list")
        for index, value in enumerate(values):
            finite(value, f"{method} {key}[{index}]")
    if not isinstance(optimization.get("termination_reason"), str) or not optimization[
        "termination_reason"
    ]:
        raise ValueError(f"{method} optimization termination reason is missing")
    if _contains_command_objective_field(optimization):
        raise ValueError(f"{method} optimization contains command-evaluation fields")
    return metrics


def _compact_deformation(value: Mapping[str, Any]) -> dict[str, Any]:
    try:
        rigid = value["best_fit_single_rigid_transform"]
        fresh = value["fresh_relative_motion"]
        return {
            "entry_displacement_m": value["entry_displacement_from_raw"]["translation_m"],
            "q_node_displacement_m": value["q_node_displacement_from_raw"]["translation_m"],
            "endpoint_displacement_m": value["endpoint_displacement_from_raw"]["translation_m"],
            "translation_deformation_rms_m": value["translation_deformation_rms_m"],
            "yaw_deformation_rms_rad": value["yaw_deformation_rms_rad"],
            "fresh_relative_motion_translation_rms_m": fresh["translation_rms_m"],
            "fresh_relative_motion_yaw_rms_rad": fresh["yaw_rms_rad"],
            "rigid_fit_translation_rms_m": rigid["translation_rms_m"],
            "rigid_fit_yaw_rms_rad": rigid["yaw_rms_rad"],
        }
    except (KeyError, TypeError) as error:
        raise ValueError("method deformation schema is incomplete") from error


def _contains_command_objective_field(value: Any) -> bool:
    if isinstance(value, Mapping):
        for key, child in value.items():
            normalized = str(key).lower()
            if "command" in normalized or normalized in {
                "j_cmd",
                "delta_v",
                "delta_omega",
            }:
                return True
            if _contains_command_objective_field(child):
                return True
    elif isinstance(value, list):
        return any(_contains_command_objective_field(child) for child in value)
    return False


def _validate_analysis_link(
    analysis: Mapping[str, Any],
    source: Any,
    method_metrics: Mapping[str, Mapping[str, Any]],
    pair_frequency: int,
    thresholds: RegimeThresholds,
) -> None:
    expected_identity = {
        "corpus_transition_id": source.corpus_transition_id,
        "cohort_id": source.cohort_id,
        "episode_id": source.episode_id,
        "transition_index": source.transition_index,
        "template_id": source.template_id,
        "variant_id": source.variant_id,
        "semantic_family": source.semantic_family,
        "physical_region": source.physical_region,
        "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
        "ordered_raw_pair_frequency": pair_frequency,
        "fresh_geometry_bin": source.fresh_geometry_bin,
        "raw_difficulty_bin": source.difficulty_bin,
        "k_fresh": source.k_fresh,
        "q_fresh": source.q_fresh,
        "q_minus_k": source.q_relative,
    }
    for key, expected in expected_identity.items():
        if analysis.get(key) != expected:
            raise ValueError(f"transition analysis/source mismatch: {key}")
    if analysis.get("geometry_undefined") is not False or analysis.get("solver_failure") is not False:
        raise ValueError("invalid transition was given candidate artifacts")
    if analysis.get("undefined_statuses") != [] or analysis.get("solver_failures") != {}:
        raise ValueError("valid transition retains an undefined/solver-failure status")
    for method, short in METHOD_SHORT.items():
        metrics = method_metrics[method]
        command = metrics["command"]
        compact = _compact_deformation(metrics["deformation"])
        expected = {
            f"j_cmd_{short}": command["J_cmd"],
            f"{short}_j_cmd": command["J_cmd"],
            f"{short}_delta_v_signed_mps": command["delta_v_signed_mps"],
            f"{short}_delta_omega_signed_rps": command["delta_omega_signed_rps"],
            f"{short}_delta_v_abs_mps": command["delta_v_abs_mps"],
            f"{short}_delta_omega_abs_rps": command["delta_omega_abs_rps"],
            f"{short}_difficulty_bin": command["difficulty"],
            f"{short}_candidate_nearest_index": command["candidate_nearest_index"],
            f"{short}_candidate_target_index": command["candidate_target_index"],
            f"{short}_candidate_target_changed_from_raw_q": command[
                "candidate_target_changed_from_raw_q"
            ],
        }
        expected.update({f"{short}_{key}": value for key, value in compact.items()})
        for key, value in expected.items():
            _same_json_value(analysis.get(key), value, f"analysis.{key}")
    if analysis.get("j_cmd_raw") != analysis.get("raw_j_cmd"):
        raise ValueError("raw command-score aliases differ")
    if analysis.get("j_cmd_m1") != analysis.get("m1_j_cmd"):
        raise ValueError("M1 command-score aliases differ")
    if analysis.get("j_cmd_m2") != analysis.get("m2_j_cmd"):
        raise ValueError("M2 command-score aliases differ")
    if analysis.get("j_cmd_m3") != analysis.get("m3_j_cmd"):
        raise ValueError("M3 command-score aliases differ")
    discontinuities = {
        short: command_discontinuity(
            analysis[f"{short}_delta_v_signed_mps"],
            analysis[f"{short}_delta_omega_signed_rps"],
        )
        for short in METHOD_SHORT.values()
    }
    label = success_failure_label(discontinuities["raw"], discontinuities["m3"])
    if analysis.get("success_failure_label") != (
        label or "RAW_INTERMEDIATE_UNLABELED"
    ):
        raise ValueError("transition success/failure classification mismatch")
    if analysis.get("m4_false_correction_rescued") is not m4_false_correction_rescued(
        discontinuities["raw"], discontinuities["m1"], discontinuities["m3"]
    ):
        raise ValueError("transition M4 rescue label mismatch")
    if tuple(analysis.get("regimes", ())) != regime_memberships(analysis, thresholds):
        raise ValueError("transition R1-R6 membership mismatch")


def _validate_transitions(
    run: Path,
    corpus_manifest: Mapping[str, Any],
    corpus: Any,
    config: Mapping[str, Any],
) -> tuple[list[Mapping[str, Any]], Mapping[str, Any]]:
    source_by_id = {item.corpus_transition_id: item for item in corpus.transitions}
    pair_counts = Counter(item.ordered_raw_pair_sha256 for item in corpus.transitions)
    thresholds = RegimeThresholds.from_mapping(config["regimes"])
    artifacts = corpus_manifest.get("transition_artifacts")
    if not isinstance(artifacts, dict):
        raise ValueError("corpus manifest has no transition-artifact map")
    solver_valid = len(artifacts)
    analysis_by_id: dict[str, Mapping[str, Any]] = {}
    for identifier, artifact in sorted(artifacts.items()):
        if identifier not in source_by_id or not isinstance(artifact, dict):
            raise ValueError(f"unknown transition artifact identity: {identifier}")
        source = source_by_id[identifier]
        expected_relative = f"transitions/{identifier.replace(':', '__')}"
        if artifact.get("artifact_relative_path") != expected_relative:
            raise ValueError(f"transition artifact directory mismatch: {identifier}")
        destination = resolve_inside(run, expected_relative, "transition artifact")
        if not destination.is_dir():
            raise ValueError(f"transition artifact directory is missing: {identifier}")
        expected_entries = {"input_reference.json", "oracle_indices.json", *METHODS}
        if {path.name for path in destination.iterdir()} != expected_entries:
            raise ValueError(f"transition artifact top-level inventory mismatch: {identifier}")
        _validate_input_and_oracle(destination, artifact, source, config)
        method_hashes = artifact.get("methods")
        if not isinstance(method_hashes, dict) or tuple(method_hashes) != METHODS:
            raise ValueError(f"transition method order/set mismatch: {identifier}")
        for method in METHODS:
            expected_hash_keys = {
                "candidate_sha256",
                "metrics_sha256",
                "optimization_sha256",
            }
            if not isinstance(method_hashes[method], Mapping) or set(
                method_hashes[method]
            ) != expected_hash_keys:
                raise ValueError(f"transition method hash schema mismatch: {identifier}/{method}")
        parsed_metrics = {
            method: _validate_method(
                destination / method,
                method,
                method_hashes[method],
                source,
                config,
            )
            for method in METHODS
        }
        analysis = artifact.get("analysis")
        if not isinstance(analysis, dict) or analysis.get("corpus_transition_id") != identifier:
            raise ValueError(f"transition analysis payload mismatch: {identifier}")
        _validate_analysis_link(
            analysis,
            source,
            parsed_metrics,
            pair_counts[source.ordered_raw_pair_sha256],
            thresholds,
        )
        analysis_by_id[identifier] = analysis
    if solver_valid == 0:
        raise ValueError("EXP-02D result has no solver-valid transition artifacts")
    analyses = [
        analysis_by_id[source.corpus_transition_id]
        for source in corpus.transitions
        if source.corpus_transition_id in analysis_by_id
    ]
    return analyses, artifacts


def _validate_csv_rows(
    rows: Sequence[Mapping[str, str]],
    corpus: Any,
    corpus_manifest: Mapping[str, Any],
    artifacts: Mapping[str, Any],
) -> tuple[set[str], set[str]]:
    source_by_id = {item.corpus_transition_id: item for item in corpus.transitions}
    row_by_id = {str(row["corpus_transition_id"]): row for row in rows}
    if set(row_by_id) != set(source_by_id):
        raise ValueError("transition-summary identity set differs from immutable source corpus")
    pair_counts = Counter(item.ordered_raw_pair_sha256 for item in corpus.transitions)
    undefined_ids: set[str] = set()
    solver_ids: set[str] = set()
    for identifier, source in source_by_id.items():
        row = row_by_id[identifier]
        expected_strings = {
            "cohort_id": source.cohort_id,
            "episode_id": source.episode_id,
            "template_id": source.template_id,
            "variant_id": source.variant_id,
            "semantic_family": source.semantic_family,
            "physical_region": source.physical_region,
            "ordered_raw_pair_sha256": source.ordered_raw_pair_sha256,
            "fresh_geometry_bin": source.fresh_geometry_bin,
            "raw_difficulty_bin": source.difficulty_bin,
        }
        for key, expected in expected_strings.items():
            if row.get(key) != str(expected):
                raise ValueError(f"transition-summary source mismatch: {identifier}/{key}")
        for key, expected in (
            ("transition_index", source.transition_index),
            ("ordered_raw_pair_frequency", pair_counts[source.ordered_raw_pair_sha256]),
            ("k_fresh", source.k_fresh),
            ("q_fresh", source.q_fresh),
            ("q_minus_k", source.q_relative),
        ):
            if integer(row.get(key), f"{identifier}/{key}") != expected:
                raise ValueError(f"transition-summary source mismatch: {identifier}/{key}")
        undefined = _csv_bool(row.get("geometry_undefined"), f"{identifier}/geometry")
        solver_failure = _csv_bool(row.get("solver_failure"), f"{identifier}/solver")
        if undefined:
            undefined_ids.add(identifier)
        if solver_failure:
            solver_ids.add(identifier)
        valid = not undefined and not solver_failure
        if (identifier in artifacts) is not valid:
            raise ValueError(f"transition artifact validity mismatch: {identifier}")
        if valid:
            analysis = artifacts[identifier]["analysis"]
            if row.get("success_failure_label") != analysis["success_failure_label"]:
                raise ValueError(f"transition-summary label mismatch: {identifier}")
            for key in (
                "alpha_entry_rad",
                "alpha_look_rad",
                "alpha_entry_minus_alpha_look_rad",
                "d_k_m",
                "d_q_m",
                "fresh_arc_k_to_q_m",
                "observation_to_boundary_translation_m",
                "observation_to_boundary_yaw_rad",
                "p_to_b_translation_m",
                "p_to_b_yaw_rad",
                "fresh_net_yaw_rad",
                "fresh_lateral_excursion_m",
                "effective_latency_s",
                "inference_translation_m",
            ):
                close(row.get(key), analysis[key], f"{identifier}/{key}")
            if _csv_bool(
                row.get("m4_false_correction_rescued"), f"{identifier}/M4 rescue"
            ) is not bool(analysis["m4_false_correction_rescued"]):
                raise ValueError(f"transition-summary M4 rescue flag mismatch: {identifier}")
            if json.loads(row.get("undefined_statuses", "null")) != [] or json.loads(
                row.get("solver_failures", "null")
            ) != {}:
                raise ValueError(f"valid transition CSV retains invalid status: {identifier}")
            csv_regimes = row.get("regimes", "").split(";") if row.get("regimes") else []
            if csv_regimes != list(analysis["regimes"]):
                raise ValueError(f"transition-summary regime mismatch: {identifier}")
            for method, short in METHOD_SHORT.items():
                for csv_suffix, analysis_key in (
                    ("J_cmd", f"j_cmd_{short}"),
                    ("delta_v_signed_mps", f"{short}_delta_v_signed_mps"),
                    ("delta_omega_signed_rps", f"{short}_delta_omega_signed_rps"),
                    ("delta_v_abs_mps", f"{short}_delta_v_abs_mps"),
                    ("delta_omega_abs_rps", f"{short}_delta_omega_abs_rps"),
                    *(
                        (field, f"{short}_{field}")
                        for field in DEFORMATION_FIELDS
                    ),
                ):
                    close(
                        row.get(f"{method}_{csv_suffix}"),
                        analysis[analysis_key],
                        f"{identifier}/{method}/{csv_suffix}",
                    )
                if row.get(f"{method}_difficulty_bin") != analysis[
                    f"{short}_difficulty_bin"
                ]:
                    raise ValueError(f"transition-summary method bin mismatch: {identifier}/{method}")
                for suffix, analysis_key in (
                    ("candidate_nearest_index", f"{short}_candidate_nearest_index"),
                    ("candidate_target_index", f"{short}_candidate_target_index"),
                ):
                    if integer(
                        row.get(f"{method}_{suffix}"), f"{identifier}/{method}/{suffix}"
                    ) != int(analysis[analysis_key]):
                        raise ValueError(f"transition-summary method index mismatch: {identifier}/{method}")
                if _csv_bool(
                    row.get(f"{method}_candidate_target_changed_from_raw_q"),
                    f"{identifier}/{method}/target-changed",
                ) is not bool(analysis[f"{short}_candidate_target_changed_from_raw_q"]):
                    raise ValueError(f"transition-summary method flag mismatch: {identifier}/{method}")
        else:
            expected_label = "GEOMETRY_UNDEFINED" if undefined else "SOLVER_FAILURE"
            if row.get("success_failure_label") != expected_label:
                raise ValueError(f"invalid transition label mismatch: {identifier}")
            if row.get("regimes") not in (None, "") or _csv_bool(
                row.get("m4_false_correction_rescued"), f"{identifier}/M4 rescue"
            ):
                raise ValueError(f"invalid transition retains regime/rescue label: {identifier}")
            for method in METHODS:
                for suffix in ("J_cmd", "delta_v_signed_mps", *DEFORMATION_FIELDS):
                    if row.get(f"{method}_{suffix}") not in (None, ""):
                        raise ValueError(f"invalid transition has method metric: {identifier}/{method}")

    undefined = corpus_manifest.get("undefined_geometry")
    solver = corpus_manifest.get("solver_failures")
    if not isinstance(undefined, Mapping) or not isinstance(solver, Mapping):
        raise ValueError("corpus manifest lacks undefined/solver status maps")
    undefined_map = undefined.get("transitions")
    solver_map = solver.get("transitions")
    if not isinstance(undefined_map, Mapping) or not isinstance(solver_map, Mapping):
        raise ValueError("corpus manifest status transitions must be mappings")
    if set(undefined_map) != undefined_ids or integer(
        undefined.get("count"), "undefined count"
    ) != len(undefined_ids):
        raise ValueError("undefined-geometry manifest differs from transition summary")
    if set(solver_map) != solver_ids or integer(
        solver.get("count"), "solver-failure count"
    ) != len(solver_ids):
        raise ValueError("solver-failure manifest differs from transition summary")
    invalid_ids = undefined_ids | solver_ids
    if set(artifacts) | invalid_ids != set(source_by_id) or set(artifacts) & invalid_ids:
        raise ValueError("valid and invalid transition status sets do not cover the corpus")
    return undefined_ids, solver_ids


def _validate_aggregate(
    aggregate: Mapping[str, Any], analyses: Sequence[Mapping[str, Any]]
) -> None:
    partitions = {
        "ALL_VALID_TRANSITIONS": list(analyses),
        "RAW_BENIGN": [row for row in analyses if row["raw_difficulty_bin"] == "BENIGN"],
        "RAW_INTERMEDIATE": [row for row in analyses if row["raw_difficulty_bin"] == "INTERMEDIATE"],
        "RAW_CHALLENGING": [row for row in analyses if row["raw_difficulty_bin"] == "CHALLENGING"],
    }
    if set(aggregate) != set(partitions):
        raise ValueError("aggregate partition set changed")
    for name, rows in partitions.items():
        block = aggregate[name]
        if integer(block.get("count"), f"{name} count") != len(rows):
            raise ValueError(f"{name} count mismatch")
        if tuple(block.get("methods", {})) != METHODS:
            raise ValueError(f"{name} method set/order changed")
        for method, short in METHOD_SHORT.items():
            metrics = block["methods"][method]
            if integer(metrics.get("count"), f"{name}/{method} count") != len(rows):
                raise ValueError(f"{name}/{method} count mismatch")
            if not rows:
                nullable = (
                    "median_abs_delta_v_mps",
                    "median_abs_delta_omega_rps",
                    "median_J_cmd",
                    "p90_J_cmd",
                    "transition_weighted_mean_J_cmd",
                    "pair_balanced_mean_J_cmd",
                    "median_translation_deformation_rms_m",
                    "median_rigid_fit_translation_rms_m",
                    "candidate_target_changed_fraction",
                    "candidate_target_changed_pair_balanced_fraction",
                )
                if any(metrics.get(key) is not None for key in nullable):
                    raise ValueError(f"{name}/{method} empty summary is not null")
                if integer(
                    metrics.get("candidate_target_changed_count"),
                    f"{name}/{method}/target-changed count",
                ) != 0:
                    raise ValueError(f"{name}/{method} empty target-change count is nonzero")
                if metrics.get("candidate_nearest_index_distribution") != {} or metrics.get(
                    "candidate_target_index_distribution"
                ) != {}:
                    raise ValueError(f"{name}/{method} empty index distribution is nonempty")
                continue
            changed = [
                bool(row[f"{short}_candidate_target_changed_from_raw_q"])
                for row in rows
            ]
            nearest = Counter(
                int(row[f"{short}_candidate_nearest_index"]) for row in rows
            )
            target = Counter(
                int(row[f"{short}_candidate_target_index"]) for row in rows
            )
            expected = {
                "median_abs_delta_v_mps": np.median(
                    [row[f"{short}_delta_v_abs_mps"] for row in rows]
                ),
                "median_abs_delta_omega_rps": np.median(
                    [row[f"{short}_delta_omega_abs_rps"] for row in rows]
                ),
                "median_J_cmd": np.median([row[f"j_cmd_{short}"] for row in rows]),
                "p90_J_cmd": np.percentile([row[f"j_cmd_{short}"] for row in rows], 90),
                "transition_weighted_mean_J_cmd": transition_weighted_mean(
                    rows, f"j_cmd_{short}"
                ),
                "pair_balanced_mean_J_cmd": pair_balanced_mean(
                    rows, f"j_cmd_{short}"
                ),
                "median_translation_deformation_rms_m": np.median(
                    [row[f"{short}_translation_deformation_rms_m"] for row in rows]
                ),
                "median_rigid_fit_translation_rms_m": np.median(
                    [row[f"{short}_rigid_fit_translation_rms_m"] for row in rows]
                ),
                "candidate_target_changed_fraction": float(np.mean(changed)),
                "candidate_target_changed_pair_balanced_fraction": pair_balanced_mean(
                    rows,
                    lambda row, key=f"{short}_candidate_target_changed_from_raw_q": float(
                        bool(row[key])
                    ),
                ),
            }
            for key, value in expected.items():
                close(metrics.get(key), value, f"{name}/{method}/{key}")
            if integer(
                metrics.get("candidate_target_changed_count"),
                f"{name}/{method}/target-changed count",
            ) != sum(changed):
                raise ValueError(f"{name}/{method} target-change count mismatch")
            expected_nearest = {str(key): nearest[key] for key in sorted(nearest)}
            expected_target = {str(key): target[key] for key in sorted(target)}
            if metrics.get("candidate_nearest_index_distribution") != expected_nearest:
                raise ValueError(f"{name}/{method} nearest-index distribution mismatch")
            if metrics.get("candidate_target_index_distribution") != expected_target:
                raise ValueError(f"{name}/{method} target-index distribution mismatch")


def _validate_pair_balanced(
    document: Mapping[str, Any],
    analyses: Sequence[Mapping[str, Any]],
    config: Mapping[str, Any],
) -> None:
    groups = ordered_pair_groups(analyses)
    if integer(document.get("ordered_raw_pair_group_count"), "pair group count") != len(groups):
        raise ValueError("pair-balanced ordered-pair count mismatch")
    if document.get("primary_weighting") != "mean within ordered raw pair, then mean across pairs":
        raise ValueError("pair-balanced weighting semantics changed")
    if tuple(document.get("methods", {})) != METHODS:
        raise ValueError("pair-balanced method set/order changed")
    for method, short in METHOD_SHORT.items():
        block = document["methods"][method]
        expected_method = {
            "mean_J_cmd": pair_balanced_mean(analyses, f"j_cmd_{short}"),
            "mean_abs_delta_v_mps": pair_balanced_mean(
                analyses, f"{short}_delta_v_abs_mps"
            ),
            "mean_abs_delta_omega_rps": pair_balanced_mean(
                analyses, f"{short}_delta_omega_abs_rps"
            ),
            "transition_weighted_mean_J_cmd": transition_weighted_mean(
                analyses, f"j_cmd_{short}"
            ),
            "mean_translation_deformation_rms_m": pair_balanced_mean(
                analyses, f"{short}_translation_deformation_rms_m"
            ),
            "mean_rigid_fit_translation_rms_m": pair_balanced_mean(
                analyses, f"{short}_rigid_fit_translation_rms_m"
            ),
            "candidate_target_changed_count": sum(
                bool(row[f"{short}_candidate_target_changed_from_raw_q"])
                for row in analyses
            ),
            "candidate_target_changed_fraction": float(
                np.mean(
                    [
                        bool(row[f"{short}_candidate_target_changed_from_raw_q"])
                        for row in analyses
                    ]
                )
            ),
            "candidate_target_changed_pair_balanced_fraction": pair_balanced_mean(
                analyses,
                lambda row, key=f"{short}_candidate_target_changed_from_raw_q": float(
                    bool(row[key])
                ),
            ),
        }
        for key, value in expected_method.items():
            if isinstance(value, int):
                if integer(block.get(key), f"{method}/{key}") != value:
                    raise ValueError(f"{method}/{key} mismatch")
            else:
                close(block.get(key), value, f"{method}/{key}")
    bootstrap = config["evaluation"]
    expected_keys = ("M3_minus_RAW", "M3_minus_M1", "M3_minus_M2")
    if set(document.get("bootstrap_differences", {})) != set(expected_keys):
        raise ValueError("bootstrap comparison set changed")
    for right, key in (("raw", expected_keys[0]), ("m1", expected_keys[1]), ("m2", expected_keys[2])):
        expected = cluster_bootstrap_pair_balanced_difference(
            analyses,
            "j_cmd_m3",
            f"j_cmd_{right}",
            seed=int(bootstrap["bootstrap_seed"]),
            repetitions=int(bootstrap["bootstrap_repetitions"]),
            confidence=float(bootstrap["confidence_level"]),
        )
        saved = document["bootstrap_differences"][key]
        for metadata_key in (
            "seed",
            "repetitions",
            "ordered_pair_group_count",
            "bootstrap_unit",
            "difference_semantics",
        ):
            if saved.get(metadata_key) != expected[metadata_key]:
                raise ValueError(f"{key} bootstrap metadata mismatch: {metadata_key}")
        for numeric_key in ("estimate", "ci_lower", "ci_upper", "confidence"):
            close(saved.get(numeric_key), expected[numeric_key], f"{key}/{numeric_key}")
    partitions = {
        "ALL_VALID_TRANSITIONS": list(analyses),
        "RAW_BENIGN": [row for row in analyses if row["raw_difficulty_bin"] == "BENIGN"],
        "RAW_INTERMEDIATE": [
            row for row in analyses if row["raw_difficulty_bin"] == "INTERMEDIATE"
        ],
        "RAW_CHALLENGING": [
            row for row in analyses if row["raw_difficulty_bin"] == "CHALLENGING"
        ],
    }
    partition_saved = document.get("bootstrap_differences_by_partition")
    if not isinstance(partition_saved, Mapping) or set(partition_saved) != set(partitions):
        raise ValueError("partition bootstrap set changed")
    for partition_name, rows in partitions.items():
        block = partition_saved[partition_name]
        if not isinstance(block, Mapping) or set(block) != set(expected_keys):
            raise ValueError(f"{partition_name} bootstrap comparison set changed")
        for right, key in (
            ("raw", expected_keys[0]),
            ("m1", expected_keys[1]),
            ("m2", expected_keys[2]),
        ):
            saved = block[key]
            if not rows:
                if saved is not None:
                    raise ValueError(f"empty {partition_name}/{key} bootstrap is not null")
                continue
            expected = cluster_bootstrap_pair_balanced_difference(
                rows,
                "j_cmd_m3",
                f"j_cmd_{right}",
                seed=int(bootstrap["bootstrap_seed"]),
                repetitions=int(bootstrap["bootstrap_repetitions"]),
                confidence=float(bootstrap["confidence_level"]),
            )
            _same_json_value(saved, expected, f"{partition_name}/{key}")


def _validate_representatives(
    document: Mapping[str, Any],
    analyses: Sequence[Mapping[str, Any]],
    config: Mapping[str, Any],
) -> Mapping[str, Any]:
    expected_ids = select_representatives(analyses)
    lookup = {row["corpus_transition_id"]: row for row in analyses}
    rules = config["representatives"]
    expected_representatives: dict[str, Any] = {}
    metric_names = (
        "j_cmd_raw",
        "j_cmd_m1",
        "j_cmd_m2",
        "j_cmd_m3",
        "raw_delta_v_abs_mps",
        "raw_delta_omega_abs_rps",
        "m1_delta_v_abs_mps",
        "m1_delta_omega_abs_rps",
        "m3_delta_v_abs_mps",
        "m3_delta_omega_abs_rps",
        "alpha_entry_rad",
        "alpha_look_rad",
        "b_to_fresh_k_translation_m",
        "b_to_fresh_q_translation_m",
        "k_fresh",
        "q_fresh",
    )
    for case, expected_id in expected_ids.items():
        if expected_id.endswith("_NOT_AVAILABLE"):
            expected_representatives[case] = {
                "status": expected_id,
                "corpus_transition_id": None,
                "selection_rule": rules[case],
                "criteria_relaxed": False,
            }
            continue
        row = lookup[expected_id]
        expected_representatives[case] = {
            "status": "AVAILABLE",
            "corpus_transition_id": expected_id,
            "selection_rule": rules[case],
            "criteria_relaxed": False,
            "artifact_relative_path": f"transitions/{expected_id.replace(':', '__')}",
            "success_failure_label": row["success_failure_label"],
            "fresh_geometry_bin": row["fresh_geometry_bin"],
            "metrics": {key: row[key] for key in metric_names},
        }
    expected_document = {
        "selection_after_complete_primary": True,
        "identity_tie_break": rules["identity_tie_break"],
        "representatives": expected_representatives,
    }
    _same_json_value(document, expected_document, "representatives")
    return expected_representatives


def _integer_distribution(values: Sequence[int]) -> dict[str, int]:
    counts = Counter(int(value) for value in values)
    return {str(key): counts[key] for key in sorted(counts)}


def _validate_oracle_summary(
    run: Path,
    corpus_manifest: Mapping[str, Any],
    corpus: Any,
    config: Mapping[str, Any],
) -> None:
    saved = strict_json(run / "summary/oracle_follower_reconstruction.json")
    if corpus_manifest.get("oracle_follower_reconstruction") != saved:
        raise ValueError("corpus-manifest oracle summary differs from summary artifact")
    full_errors: list[float] = []
    suffix_errors: list[float] = []
    indices_match = True
    for source in corpus.transitions:
        full = np.asarray(
            [
                source.raw_follower_command.linear_velocity_mps,
                source.raw_follower_command.angular_velocity_rps,
            ],
            dtype=np.float64,
        )
        full_errors.append(
            float(np.max(np.abs(full - source.raw_fresh_first_desired_command)))
        )
        suffix = candidate_command_metrics(
            source.fresh_world[source.k_fresh :],
            boundary=source.boundary_pose,
            old_last_command=source.old_desired_command,
            follower_values=asdict(source.follower_config),
            original_entry_index=source.k_fresh,
            raw_target_index=source.q_fresh,
            linear_normalization_mps=float(
                config["evaluation"]["command_score_linear_scale_mps"]
            ),
            angular_normalization_rps=float(
                config["evaluation"]["command_score_angular_scale_rps"]
            ),
        )
        suffix_errors.append(
            float(
                np.max(
                    np.abs(
                        np.asarray(
                            suffix["candidate_first_desired_v_omega"], dtype=np.float64
                        )
                        - source.raw_fresh_first_desired_command
                    )
                )
            )
        )
        indices_match = indices_match and (
            suffix["candidate_nearest_index"] == source.k_fresh
            and suffix["candidate_target_index"] == source.q_fresh
        )
    expected = {
        "transition_count": corpus.valid_count,
        "unique_ordered_raw_pair_count": len(
            {source.ordered_raw_pair_sha256 for source in corpus.transitions}
        ),
        "lookahead_distance_m": float(config["oracle"]["expected_lookahead_distance_m"]),
        "first_command_absolute_tolerance": float(
            config["oracle"]["first_command_absolute_tolerance"]
        ),
        "maximum_full_raw_first_command_absolute_error": max(full_errors),
        "maximum_selected_suffix_first_command_absolute_error": max(suffix_errors),
        "all_selected_suffix_absolute_indices_match_raw_k_q": indices_match,
        "k_fresh_distribution": _integer_distribution(
            [source.k_fresh for source in corpus.transitions]
        ),
        "q_fresh_distribution": _integer_distribution(
            [source.q_fresh for source in corpus.transitions]
        ),
        "q_minus_k_distribution": _integer_distribution(
            [source.q_relative for source in corpus.transitions]
        ),
    }
    _same_json_value(saved, expected, "oracle follower reconstruction")


def _validate_summaries(
    run: Path,
    analyses: Sequence[Mapping[str, Any]],
    csv_rows: Sequence[Mapping[str, str]],
    config: Mapping[str, Any],
) -> Mapping[str, Any]:
    aggregate = strict_json(run / "summary/aggregate.json")
    pair_balanced = strict_json(run / "summary/pair_balanced.json")
    success = strict_json(run / "summary/success_failure_counts.json")
    regimes = strict_json(run / "summary/regime_analysis.json")
    representatives = strict_json(run / "summary/representatives.json")
    plot_records = strict_json(run / "summary/plot_records.json")
    _validate_aggregate(aggregate, analyses)
    _validate_pair_balanced(pair_balanced, analyses, config)
    counts = Counter(str(row["success_failure_label"]) for row in csv_rows)
    expected_counts = {label: counts.get(label, 0) for label in SUCCESS_FAILURE_LABELS}
    expected_other = {
        label: count
        for label, count in sorted(counts.items())
        if label not in SUCCESS_FAILURE_LABELS
    }
    if success.get("counts") != expected_counts or success.get("other_counts") != expected_other:
        raise ValueError("success/failure label counts mismatch")
    if integer(success.get("raw_benign_count"), "raw benign count") != sum(
        row["raw_difficulty_bin"] == "BENIGN" for row in analyses
    ):
        raise ValueError("raw benign summary count mismatch")
    if integer(success.get("raw_challenging_count"), "raw challenging count") != sum(
        row["raw_difficulty_bin"] == "CHALLENGING" for row in analyses
    ):
        raise ValueError("raw challenging summary count mismatch")
    undefined_ids = [
        row["corpus_transition_id"]
        for row in csv_rows
        if _csv_bool(row.get("geometry_undefined"), "summary geometry flag")
    ]
    solver_ids = [
        row["corpus_transition_id"]
        for row in csv_rows
        if _csv_bool(row.get("solver_failure"), "summary solver flag")
    ]
    if integer(success.get("geometry_undefined_count"), "summary undefined count") != len(
        undefined_ids
    ) or success.get("geometry_undefined_transition_ids") != undefined_ids:
        raise ValueError("success summary undefined-geometry rows mismatch")
    if integer(success.get("solver_failure_count"), "summary solver count") != len(
        solver_ids
    ) or success.get("solver_failure_transition_ids") != solver_ids:
        raise ValueError("success summary solver-failure rows mismatch")
    benign = [row for row in analyses if row["raw_difficulty_bin"] == "BENIGN"]
    challenging = [
        row for row in analyses if row["raw_difficulty_bin"] == "CHALLENGING"
    ]
    expected_broken = {
        method: sum(
            row[f"{METHOD_SHORT[method]}_difficulty_bin"] != "BENIGN"
            for row in benign
        )
        for method in METHODS
    }
    if success.get("benign_broken_by_method") != expected_broken:
        raise ValueError("benign-broken method counts mismatch")
    expected_challenging = {
        label: counts.get(label, 0)
        for label in (
            "CHALLENGING_RESCUED",
            "CHALLENGING_IMPROVED",
            "CHALLENGING_MIXED",
            "CHALLENGING_WORSE",
        )
    }
    if success.get("m3_challenging_outcomes") != expected_challenging:
        raise ValueError("challenging outcome counts mismatch")
    rescued = [row for row in analyses if bool(row["m4_false_correction_rescued"])]
    rescue = success.get("M4_FALSE_CORRECTION_RESCUED")
    if not isinstance(rescue, Mapping):
        raise ValueError("M4 rescue summary is missing")
    if integer(rescue.get("count"), "M4 rescue count") != len(rescued) or integer(
        rescue.get("raw_benign_denominator"), "M4 rescue denominator"
    ) != len(benign):
        raise ValueError("M4 rescue counts mismatch")
    if benign:
        close(
            rescue.get("fraction_of_raw_benign"),
            len(rescued) / len(benign),
            "M4 rescue transition-weighted fraction",
        )
        close(
            rescue.get("raw_pair_balanced_fraction_within_raw_benign"),
            pair_balanced_mean(
                benign, lambda row: float(bool(row["m4_false_correction_rescued"]))
            ),
            "M4 rescue pair-balanced fraction",
        )
    elif rescue.get("fraction_of_raw_benign") is not None or rescue.get(
        "raw_pair_balanced_fraction_within_raw_benign"
    ) is not None:
        raise ValueError("empty M4 rescue denominator is not null")
    if regimes.get("thresholds") != config.get("regimes") or regimes.get("interpretation") != "descriptive associations; no causal claim":
        raise ValueError("regime thresholds or causal limitation changed")
    hypothesis = regimes.get("geometry_hypothesis")
    if not isinstance(hypothesis, Mapping):
        raise ValueError("geometry-hypothesis summary is missing")
    hypothesis_bootstrap = hypothesis.get("pair_cluster_bootstrap_pearson_ci")
    evaluation = config["evaluation"]
    if (
        hypothesis.get("x") != "alpha_entry_minus_alpha_look_rad"
        or hypothesis.get("y") != "J_M1_minus_J_M3"
        or hypothesis.get("causal_claim") is not False
        or not isinstance(hypothesis_bootstrap, Mapping)
        or integer(hypothesis_bootstrap.get("seed"), "hypothesis bootstrap seed")
        != int(evaluation["bootstrap_seed"])
        or integer(
            hypothesis_bootstrap.get("repetitions"),
            "hypothesis bootstrap repetitions",
            minimum=1,
        )
        != int(evaluation["bootstrap_repetitions"])
        or hypothesis_bootstrap.get("bootstrap_unit") != "ordered_raw_pair_sha256"
    ):
        raise ValueError("geometry-hypothesis bootstrap protocol changed")
    valid_draws = integer(
        hypothesis_bootstrap.get("valid_draw_count"),
        "hypothesis bootstrap valid draw count",
        minimum=1,
    )
    if valid_draws > int(evaluation["bootstrap_repetitions"]):
        raise ValueError("geometry-hypothesis valid draw count exceeds repetitions")
    for key in (
        "transition_weighted_pearson_r",
        "pair_balanced_pearson_r",
    ):
        finite(hypothesis.get(key), f"geometry hypothesis {key}")
    ci_lower = finite(hypothesis_bootstrap.get("ci_lower"), "hypothesis CI lower")
    ci_upper = finite(hypothesis_bootstrap.get("ci_upper"), "hypothesis CI upper")
    if ci_lower > ci_upper or ci_lower < -1.0 or ci_upper > 1.0:
        raise ValueError("geometry-hypothesis bootstrap CI is invalid")
    for index in range(1, 7):
        key = f"R{index}"
        block = regimes.get("regimes", {}).get(key)
        expected_ids = [
            row["corpus_transition_id"]
            for row in analyses
            if any(str(value).startswith(f"R{index}_") for value in row["regimes"])
        ]
        if not isinstance(block, dict) or block.get("transition_ids") != expected_ids or block.get("count") != len(expected_ids):
            raise ValueError(f"{key} regime membership summary mismatch")
    expected_representatives = _validate_representatives(
        representatives, analyses, config
    )
    records = plot_records.get("records")
    if not isinstance(records, list) or len(records) != len(analyses):
        raise ValueError("plot-record count differs from valid transitions")
    if [row.get("corpus_transition_id") for row in records] != [
        row["corpus_transition_id"] for row in analyses
    ]:
        raise ValueError("plot-record identity order differs from transition artifacts")
    return expected_representatives


def _validate_plots(run: Path) -> None:
    plots = run / "plots"
    names = tuple(sorted(path.name for path in plots.iterdir() if path.is_file()))
    if names != PLOT_NAMES:
        raise ValueError(f"EXP-02D must contain exactly ten frozen plots: {names}")
    for name in names:
        data = (plots / name).read_bytes()
        if len(data) < 100 or not data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError(f"invalid EXP-02D PNG: {name}")


def validate_exp02d_result(
    run_directory: str | Path,
    *,
    source_repository_root: str | Path | None = None,
) -> dict[str, Any]:
    """Validate a completed result without opening any path for writing."""

    run = Path(run_directory).expanduser().resolve()
    if not run.is_dir():
        raise ValueError(f"EXP-02D run does not exist: {run}")
    hashes = validate_result_manifest(run)
    config = strict_yaml(run / "config_snapshot.yaml")
    provenance = strict_json(run / "provenance.json")
    metadata = strict_json(run / "metadata.json")
    corpus_manifest = strict_json(run / "corpus_manifest.json")
    source_root = (
        Path(source_repository_root).expanduser().resolve()
        if source_repository_root is not None
        else Path(str(provenance["source_repository_root"])).expanduser().resolve()
    )
    corpus = load_development_corpus(
        source_root,
        config["paths"]["combined_corpus"],
        config["source_integrity"],
        float(config["oracle"]["first_command_absolute_tolerance"]),
    )
    corpus.require_complete()
    _assert_source_separation(run, corpus)
    _validate_run_identity(run, config, provenance, metadata, corpus_manifest, corpus)
    rows = _csv_rows(run / "transition_summary.csv")
    if len(rows) != corpus.index.eligible_count:
        raise ValueError("transition-summary count differs from recomputed eligible source count")
    analyses, artifacts = _validate_transitions(run, corpus_manifest, corpus, config)
    undefined_ids, solver_ids = _validate_csv_rows(
        rows, corpus, corpus_manifest, artifacts
    )
    if integer(metadata.get("solver_valid_count"), "solver-valid count") != len(analyses):
        raise ValueError("solver-valid metadata count differs from transition artifacts")
    undefined_count = integer(metadata.get("geometry_undefined_count"), "geometry undefined count")
    solver_failure_count = integer(metadata.get("solver_failure_count"), "solver failure count")
    if undefined_count != len(undefined_ids) or solver_failure_count != len(solver_ids):
        raise ValueError("metadata invalid-status counts differ from transition summary")
    if len(analyses) + len(undefined_ids | solver_ids) != corpus.index.eligible_count:
        raise ValueError("valid and invalid transition status sets do not cover eligible corpus")
    pair_counts = Counter(item.ordered_raw_pair_sha256 for item in corpus.transitions)
    if integer(corpus_manifest.get("ordered_raw_pair_count"), "ordered-pair count") != len(pair_counts) or corpus_manifest.get("ordered_raw_pair_frequencies") != dict(sorted(pair_counts.items())):
        raise ValueError("ordered raw-pair corpus manifest mismatch")
    if len(artifacts) != len(analyses):
        raise AssertionError("transition artifact/analysis count diverged")
    _validate_oracle_summary(run, corpus_manifest, corpus, config)
    representatives = _validate_summaries(run, analyses, rows, config)
    if metadata.get("representatives") != representatives:
        raise ValueError("metadata representatives differ from frozen representative summary")
    _validate_plots(run)
    return {
        "valid": True,
        "technical_status": "EXP02D_RUN_COMPLETE",
        "run": str(run),
        "manifest_artifact_count": len(hashes),
        "eligible_moving_recomputed": corpus.index.eligible_count,
        "reconstruction_valid": corpus.valid_count,
        "input_reconstruction_failed": len(corpus.failures),
        "geometry_undefined": undefined_count,
        "solver_valid": len(analyses),
        "solver_failure": solver_failure_count,
        "ordered_raw_pair_groups": len(pair_counts),
        "plots": len(PLOT_NAMES),
        "candidate_methods_physically_executed": False,
        "J_cmd_in_objective": False,
        "corpus_role": "EXP02D_DEVELOPMENT_CORPUS",
        "data02_combined_decision": "DATA02_COMBINED_DIVERSITY_INSUFFICIENT",
    }


def main() -> None:
    options = arguments()
    result = validate_exp02d_result(
        options.run,
        source_repository_root=options.source_repository_root,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
