#!/usr/bin/env python3
"""Strictly validate, characterize, split, and summarize a DATA-02 run."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.data02_online_successive import (
    chunk_content_sha256,
    isolated_split,
    observation_anchored_paths,
    percentile_summary,
    readiness_decision,
    strict_json,
    validate_transition_artifact,
)
from reconciliation.lightnav_adapter import lightnav_local_to_world
from reconciliation.online_switch import save_json_exclusive, sha256_file


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    return parser.parse_args()


def load_config(run: Path) -> dict[str, Any]:
    value = yaml.safe_load((run / "config_snapshot.yaml").read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("config snapshot is not a mapping")
    return value


def validate_rgb_stream(episode: Path, initial_count: int) -> tuple[dict[str, Any], list[dict[str, str]]]:
    index_path = episode / "rgb_frames.csv"
    if not index_path.is_file():
        raise ValueError(f"missing episode RGB index: {episode}")
    with index_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < initial_count:
        raise ValueError(f"episode has fewer than {initial_count} initial frames")
    indices = [int(row["frame_index"]) for row in rows]
    times = np.asarray([float(row["sim_time_s"]) for row in rows])
    if indices != list(range(len(rows))) or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("broken RGB frame indices/timestamps")
    for row in rows:
        path = episode / row["rgb_relative_path"]
        if not path.is_file() or sha256_file(path) != row["rgb_file_sha256"]:
            raise ValueError(f"RGB artifact hash mismatch: {path}")
    observed = [int(row["server_observed_frames"]) for row in rows]
    if any(value < 1 for value in observed) or observed != sorted(observed):
        raise ValueError("episode frames were not completely delivered in order")
    return ({"frame_count": len(rows), "first_sim_time_s": float(times[0]), "last_sim_time_s": float(times[-1])}, rows)


def validate_chunk(directory: Path, episode_id: str, rgb_rows: list[dict[str, str]], initial_count: int) -> dict[str, Any]:
    metadata = strict_json(directory / "metadata.json")
    for name in ("raw_actions.npy", "local_path.npy", "world_path.npy", "raw_text.txt"):
        if not (directory / name).is_file():
            raise ValueError(f"chunk is missing {name}")
    raw = np.load(directory / "raw_actions.npy", allow_pickle=False)
    local = np.load(directory / "local_path.npy", allow_pickle=False)
    world = np.load(directory / "world_path.npy", allow_pickle=False)
    expected_local, expected = observation_anchored_paths(raw, metadata["observation_pose_world_se2"])
    if not np.array_equal(local, expected_local) or not np.array_equal(world, expected) or raw.shape != local.shape or raw.shape != world.shape:
        raise ValueError("chunk world transform or arbitrary-N shape is invalid")
    if metadata["episode_id"] != episode_id:
        raise ValueError("chunk episode identity mismatch")
    if metadata.get("waypoint_dt") is not None or metadata.get("boundary_inserted") is not False:
        raise ValueError("chunk fabricated time or inserted B")
    if sha256_file(directory / "raw_actions.npy") != metadata["raw_actions_file_sha256"]:
        raise ValueError("canonical raw action file was modified")
    if chunk_content_sha256(raw) != metadata["raw_sha256"] or chunk_content_sha256(world) != metadata["world_sha256"]:
        raise ValueError("chunk content identity mismatch")
    if sha256_file(directory / "raw_text.txt") != metadata["raw_text_sha256"]:
        raise ValueError("canonical raw text was modified")
    frame_index = int(metadata["observation_rgb_frame_index"])
    if frame_index < 0 or frame_index >= len(rgb_rows):
        raise ValueError("chunk observation frame is outside episode RGB stream")
    frame = rgb_rows[frame_index]
    frame_pose = np.asarray([frame["actual_x"], frame["actual_y"], frame["actual_yaw"]], dtype=float)
    if not math.isclose(float(frame["sim_time_s"]), float(metadata["observation_sim_time_s"]), abs_tol=1e-9) or not np.allclose(frame_pose, metadata["observation_pose_world_se2"], atol=1e-9):
        raise ValueError("chunk observation timestamp/pose does not reconstruct from RGB index")
    history_first = max(0, frame_index - initial_count + 1)
    expected_live = max(0, frame_index + 1 - initial_count)
    if int(metadata["total_observed_frame_count"]) != frame_index + 1 or int(metadata["live_frame_count"]) != expected_live:
        raise ValueError("chunk frame-count metadata is inconsistent")
    if int(metadata["response"]["observed_frames"]) != frame_index + 1:
        raise ValueError("server observation count is inconsistent")
    if not math.isclose(float(metadata["history_oldest_frame_sim_time_s"]), float(rgb_rows[history_first]["sim_time_s"]), abs_tol=1e-9):
        raise ValueError("chunk history age cannot be reconstructed")
    if not math.isclose(float(metadata["history_age_at_observation_s"]), float(metadata["observation_sim_time_s"]) - float(rgb_rows[history_first]["sim_time_s"]), abs_tol=1e-9):
        raise ValueError("chunk history age is inconsistent")
    if float(metadata["ready_sim_time_s"]) < float(metadata["observation_sim_time_s"]):
        raise ValueError("chunk became ready before its observation")
    return metadata


def validate_transition_timing(document: Mapping[str, Any], control_dt: float) -> None:
    timing = document["timing"]
    obs = float(timing["t_obs_sim_s"])
    ready = float(timing["t_ready_sim_s"])
    switch = float(timing["t_switch_sim_s"])
    host = float(timing["request_response_host_s"])
    values = [obs, ready, switch, host, float(timing["tau_effective_s"]), float(timing["real_time_factor"])]
    if not np.all(np.isfinite(values)) or not obs <= ready <= switch or host <= 0.0:
        raise ValueError("transition clock ordering is invalid")
    if int(timing["t_request_host_monotonic_ns"]) > int(timing["t_model_ready_host_monotonic_ns"]):
        raise ValueError("transition host clock ordering is invalid")
    if not math.isclose(float(timing["tau_effective_s"]), switch - obs, abs_tol=1e-9):
        raise ValueError("effective latency does not reconstruct")
    expected_rtf = (ready - obs) / host
    if not math.isclose(float(timing["real_time_factor"]), expected_rtf, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError("RTF does not reconstruct")
    pre_time = float(document["pose_immediately_before_switch_P_sim_time_s"])
    if not pre_time < switch or not math.isclose(switch - pre_time, control_dt, abs_tol=1e-7):
        raise ValueError("P is not the controller sample immediately before B")


def representative_ids(records: list[dict[str, Any]], count_limit: int = 36) -> list[str]:
    if not records:
        return []
    selected: dict[str, list[str]] = defaultdict(list)
    def extremes(name: str, getter) -> None:
        rows = sorted(records, key=lambda row: (getter(row), row["transition_id"]))
        for label, row in (("lowest", rows[0]), ("median", rows[(len(rows) - 1) // 2]), ("highest", rows[-1])):
            selected[row["transition_id"]].append(f"{label}_{name}")
    extremes("tau_effective", lambda row: row["tau_effective_s"])
    extremes("abs_delta_v", lambda row: abs(row["delta_v_mps"]))
    extremes("abs_delta_omega", lambda row: abs(row["delta_omega_rps"]))
    for field, values in (("fresh_geometry_bin", ("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING")), ("difficulty_bin", ("BENIGN", "CHALLENGING"))):
        for value in values:
            candidates = [row for row in records if row[field] == value]
            if candidates:
                selected[min(candidates, key=lambda row: row["transition_id"])["transition_id"]].append(value.lower())
    frequency = Counter(row["ordered_raw_pair_sha256"] for row in records)
    largest = max(frequency.values())
    for label, wanted in (("largest_duplicate_group", largest), ("rare_raw_pair", min(frequency.values()))):
        candidate = min((row for row in records if frequency[row["ordered_raw_pair_sha256"]] == wanted), key=lambda row: row["transition_id"])
        selected[candidate["transition_id"]].append(label)
    # Fill the panel deterministically to the declared review range when data permits.
    target = min(30, len(records))
    ordered_records = sorted(records, key=lambda row: row["transition_id"])
    for row in ordered_records:
        if len(selected) >= target:
            break
        selected[row["transition_id"]].append("deterministic_coverage_fill")
    ordered = sorted(selected.items())[:count_limit]
    return [identifier for identifier, _ in ordered]


def main() -> None:
    options = args()
    run = options.run_directory.resolve()
    config = load_config(run)
    metadata = strict_json(run / "metadata.json")
    complete = strict_json(run / "collection_complete.json")
    protocol = strict_json(run / "protocol.json")
    manifest = strict_json(run / "collection_manifest.json")
    if sha256_file(run / "protocol.json") != metadata.get("protocol_sha256"):
        raise ValueError("protocol hash differs from run provenance")
    if sha256_file(run / "config_snapshot.yaml") != metadata.get("config_snapshot_sha256"):
        raise ValueError("config snapshot hash differs from run provenance")
    if protocol.get("identity") != config.get("stage") or protocol.get("coordinates", {}).get("waypoint_dt") is not None:
        raise ValueError("protocol identity or waypoint-time contract is invalid")
    expected = len(config["episode_templates"]) * len(config["variants"]) if metadata["phase"] == "primary" else complete["episode_count"]
    episode_paths = sorted((run / "episodes").glob("episode_*"))
    errors: list[str] = []
    records: list[dict[str, Any]] = []
    episode_summaries = []
    if len(episode_paths) != expected or not complete.get("complete"):
        errors.append(f"episode grid incomplete: {len(episode_paths)} != {expected}")
    if metadata["phase"] == "primary" and metadata.get("collector_git_status"):
        errors.append("primary dataset was not collected from a clean collector worktree")
    for episode in episode_paths:
        try:
            episode_metadata = strict_json(episode / "metadata.json")
            marker = strict_json(episode / "attempt_complete.json")
            if marker.get("complete") is not True or episode_metadata["lightnav_episode_reset_count"] != 1:
                raise ValueError("episode completion/reset contract invalid")
            rgb, rgb_rows = validate_rgb_stream(episode, int(config["protocol"]["initial_history_frames"]))
            chunks = sorted((episode / "chunks").glob("chunk_*"))
            chunk_metadata = [validate_chunk(path, episode.name, rgb_rows, int(config["protocol"]["initial_history_frames"])) for path in chunks]
            transitions = sorted((episode / "transitions").glob("transition_*"))
            transition_documents = [validate_transition_artifact(path) for path in transitions]
            if len(chunks) != len(transitions) + 1 or len(transitions) != int(episode_metadata["transition_count"]):
                raise ValueError("episode chunk/transition succession count mismatch")
            for index, document in enumerate(transition_documents):
                if document["old_chunk_id"] != chunks[index].name or document["fresh_chunk_id"] != chunks[index + 1].name:
                    raise ValueError("FRESH_i was not promoted to OLD_(i+1)")
                old_copy = np.load(transitions[index] / "raw/old_actions.npy", allow_pickle=False)
                fresh_copy = np.load(transitions[index] / "raw/fresh_actions.npy", allow_pickle=False)
                if not np.array_equal(old_copy, np.load(chunks[index] / "raw_actions.npy", allow_pickle=False)) or not np.array_equal(fresh_copy, np.load(chunks[index + 1] / "raw_actions.npy", allow_pickle=False)):
                    raise ValueError("transition raw copy differs from canonical request output")
                timing = document["timing"]
                validate_transition_timing(document, float(config["simulation"]["control_dt"]))
                if not math.isclose(float(document["fresh_observation_sim_time_s"]), float(timing["t_obs_sim_s"]), abs_tol=1e-9):
                    raise ValueError("FRESH observation time differs from transition timing")
                actual = np.load(transitions[index] / "actual.npy", allow_pickle=False)
                if not np.allclose(actual[-1], document["switch_boundary_B_world_se2"], atol=1e-9):
                    raise ValueError("saved actual history does not end at B")
                telemetry_rows = list(csv.DictReader((transitions[index] / "telemetry.csv").open(encoding="utf-8", newline="")))
                reconstructed_actual = np.asarray([[row["actual_x"], row["actual_y"], row["actual_yaw"]] for row in telemetry_rows], dtype=float)
                if not np.array_equal(actual, reconstructed_actual):
                    raise ValueError("transition actual path differs from telemetry")
                ready_rows = [row for row in telemetry_rows if math.isclose(float(row["sim_time_s"]), float(timing["t_ready_sim_s"]), abs_tol=1e-9)]
                if not ready_rows or not np.allclose(
                    [float(ready_rows[-1]["actual_x"]), float(ready_rows[-1]["actual_y"]), float(ready_rows[-1]["actual_yaw"])],
                    document["model_ready_pose_world_se2"], atol=1e-9,
                ):
                    raise ValueError("model-ready pose does not reconstruct from telemetry")
                pre_time = float(document["pose_immediately_before_switch_P_sim_time_s"])
                matching = [row for row in telemetry_rows if math.isclose(float(row["sim_time_s"]), pre_time, abs_tol=1e-9)]
                if not matching or not np.allclose(
                    [float(matching[-1]["actual_x"]), float(matching[-1]["actual_y"]), float(matching[-1]["actual_yaw"])],
                    document["pose_immediately_before_switch_P_world_se2"], atol=1e-9,
                ):
                    raise ValueError("P does not reconstruct from transition telemetry")
                metrics = document["metrics"]
                command = metrics["command_discontinuity"]
                row = {
                    "transition_id": document["transition_id"], "episode_id": document["episode_id"],
                    "transition_index": document["transition_index"], "template_id": document["template_id"],
                    "variant_id": document["variant_id"], "status": document["status"],
                    "old_raw_sha256": document["hashes"]["old_raw_sha256"], "fresh_raw_sha256": document["hashes"]["fresh_raw_sha256"],
                    "ordered_raw_pair_sha256": document["hashes"]["ordered_raw_pair_sha256"],
                    "old_world_sha256": document["hashes"]["old_world_sha256"], "fresh_world_sha256": document["hashes"]["fresh_world_sha256"],
                    "ordered_world_pair_sha256": document["hashes"]["ordered_world_pair_sha256"],
                    "tau_effective_s": timing["tau_effective_s"], "host_latency_s": timing["request_response_host_s"],
                    "model_reported_s": timing["model_reported_s"], "rtf": timing["real_time_factor"],
                    "observation_to_ready_sim_s": timing["observation_to_ready_sim_s"], "ready_to_switch_sim_s": timing["ready_to_switch_sim_s"],
                    "robot_translation_during_inference_m": metrics["observation_to_model_ready_translation_m"],
                    "robot_yaw_during_inference_rad": metrics["observation_to_model_ready_yaw_rad"],
                    "robot_translation_during_effective_latency_m": metrics["observation_to_boundary_translation_m"],
                    "robot_yaw_during_effective_latency_rad": metrics["observation_to_boundary_yaw_rad"],
                    "b_to_fresh0_translation_m": metrics["b_to_fresh0_translation_m"], "b_to_fresh0_yaw_rad": metrics["b_to_fresh0_yaw_rad"],
                    "p_to_b_translation_m": metrics["p_to_b_translation_m"], "p_to_b_yaw_rad": metrics["p_to_b_yaw_rad"],
                    "delta_v_mps": command["delta_v_mps"], "delta_omega_rps": command["delta_omega_rps"],
                    "difficulty_bin": command["difficulty_bin"], "fresh_geometry_bin": metrics["fresh_geometry_bin"],
                    "old_geometry": metrics["old_geometry"], "fresh_geometry": metrics["fresh_geometry"],
                    "artifact_relative_path": str(transitions[index].relative_to(run)),
                }
                if not all(math.isfinite(float(row[key])) for key in ("tau_effective_s", "host_latency_s", "model_reported_s", "rtf", "delta_v_mps", "delta_omega_rps")):
                    raise ValueError("transition index contains non-finite metrics")
                records.append(row)
            episode_summaries.append({"episode_id": episode.name, "template_id": episode_metadata["template_id"], "variant_id": episode_metadata["variant_id"], "transition_count": len(transitions), "termination_reason": episode_metadata["termination_reason"], **rgb})
        except Exception as error:
            errors.append(f"{episode.name}: {type(error).__name__}: {error}")
    manifest_entries = {entry["episode_id"]: entry for entry in manifest.get("episodes", [])}
    if manifest.get("episode_count") != len(episode_paths) or set(manifest_entries) != {path.name for path in episode_paths}:
        errors.append("collection manifest episode set is inconsistent")
    for episode in episode_paths:
        entry = manifest_entries.get(episode.name)
        if entry is None:
            continue
        checks = {
            "episode_metadata_sha256": episode / "metadata.json",
            "attempt_complete_sha256": episode / "attempt_complete.json",
            "telemetry_sha256": episode / "telemetry.csv",
            "rgb_index_sha256": episode / "rgb_frames.csv",
        }
        for key, path in checks.items():
            if sha256_file(path) != entry.get(key):
                errors.append(f"{episode.name}: collection manifest hash mismatch: {key}")
        chunks = sorted((episode / "chunks").glob("chunk_*"))
        transitions = sorted((episode / "transitions").glob("transition_*"))
        if [sha256_file(path / "metadata.json") for path in chunks] != entry.get("chunk_metadata_sha256"):
            errors.append(f"{episode.name}: collection manifest chunk hashes mismatch")
        if [sha256_file(path / "transition.json") for path in transitions] != entry.get("transition_metadata_sha256"):
            errors.append(f"{episode.name}: collection manifest transition hashes mismatch")
    for key, path in (
        ("config_snapshot_sha256", run / "config_snapshot.yaml"),
        ("protocol_sha256", run / "protocol.json"),
        ("template_bank_manifest_sha256", run / "template_bank/manifest.json"),
    ):
        if sha256_file(path) != manifest.get(key):
            errors.append(f"collection manifest run-level hash mismatch: {key}")
    validation = {
        "valid": not errors, "errors": errors, "episode_count": len(episode_paths),
        "transition_count": len(records), "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
        "raw_world_transform_reconstructed": not errors, "frame_streams_reconstructed": not errors,
        "exclusive_write_audit": "all canonical artifacts are hash-checked; collector opens new artifacts exclusively",
    }
    if options.validate_only:
        print(json.dumps(validation, indent=2, sort_keys=True))
        if errors:
            raise SystemExit(1)
        return
    summary_dir = run / "summary"
    summary_dir.mkdir(exist_ok=False)
    save_json_exclusive(summary_dir / "validation.json", validation)
    save_json_exclusive(summary_dir / "transition_index.json", {"records": records})
    eligible = [row for row in records if row["status"] == "ELIGIBLE_MOVING"]
    split = isolated_split(records, float(config["readiness"]["heldout_fraction"]))
    save_json_exclusive(summary_dir / "split.json", split)
    split_hash = sha256_file(summary_dir / "split.json")
    split["split_manifest_sha256"] = split_hash
    decision = readiness_decision(records, split, config["readiness"], artifacts_valid=not errors, template_count=len(config["episode_templates"]))
    save_json_exclusive(summary_dir / "readiness.json", decision)
    status_counts = Counter(row["status"] for row in records)
    geometry_counts = Counter(row["fresh_geometry_bin"] for row in eligible)
    difficulty_counts = Counter(row["difficulty_bin"] for row in eligible)
    pair_frequency = Counter(row["ordered_raw_pair_sha256"] for row in eligible)
    summary = {
        "run_id": run.name, "collector_git_sha": metadata["collector_git_sha"], "phase": metadata["phase"],
        "episode_count": len(episode_paths), "maximum_possible_transitions": int(complete["maximum_transition_count"]),
        "attempted_transition_count": len(records), "status_counts": dict(sorted(status_counts.items())),
        "eligible_transition_count": len(eligible),
        "unique_counts": {
            "old_raw": len({row["old_raw_sha256"] for row in eligible}), "fresh_raw": len({row["fresh_raw_sha256"] for row in eligible}),
            "ordered_raw_pairs": len(pair_frequency), "old_world": len({row["old_world_sha256"] for row in eligible}),
            "fresh_world": len({row["fresh_world_sha256"] for row in eligible}), "ordered_world_pairs": len({row["ordered_world_pair_sha256"] for row in eligible}),
        },
        "largest_raw_pair_fraction": max(pair_frequency.values(), default=0) / len(eligible) if eligible else None,
        "geometry_bin_counts": dict(sorted(geometry_counts.items())), "difficulty_bin_counts": dict(sorted(difficulty_counts.items())),
        "timing": {
            "tau_effective_s": percentile_summary(row["tau_effective_s"] for row in eligible),
            "host_latency_s": percentile_summary(row["host_latency_s"] for row in eligible),
            "model_reported_s": percentile_summary(row["model_reported_s"] for row in eligible),
            "rtf": percentile_summary(row["rtf"] for row in eligible),
        },
        "motion": {
            "translation_during_inference_m": percentile_summary(row["robot_translation_during_inference_m"] for row in eligible),
            "yaw_during_inference_rad": percentile_summary(row["robot_yaw_during_inference_rad"] for row in eligible),
            "translation_during_effective_latency_m": percentile_summary(row["robot_translation_during_effective_latency_m"] for row in eligible),
            "yaw_during_effective_latency_rad": percentile_summary(row["robot_yaw_during_effective_latency_rad"] for row in eligible),
        },
        "command_jump": {
            "abs_delta_v_mps": percentile_summary(abs(row["delta_v_mps"]) for row in eligible),
            "abs_delta_omega_rps": percentile_summary(abs(row["delta_omega_rps"]) for row in eligible),
        },
        "episode_summaries": episode_summaries, "split_manifest_sha256": split_hash,
        "readiness_status": decision["status"],
        "claim_limit": "descriptive transition data only; no reconciliation method or causal navigation-quality claim",
    }
    save_json_exclusive(summary_dir / "summary.json", summary)
    representatives = representative_ids(eligible)
    save_json_exclusive(summary_dir / "representatives.json", {"selection_rule": "predeclared deterministic extrema/bin/frequency union", "target_range": [24, 36], "transition_ids": representatives})
    print(json.dumps({"validation": validation, "summary": summary, "readiness": decision}, indent=2, sort_keys=True))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
