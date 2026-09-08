#!/usr/bin/env python3
"""Freeze Stage 0-F source inventory, geometry, and representatives before execution."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping

import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.lightnav_execution_envelope import (  # noqa: E402
    build_fixture_geometry,
    build_grouped_geometry_summary,
    build_source_inventory,
    compare_descriptor_to_fixtures,
    extract_suffixes,
    fit_fixture_distance_model,
    geometry_descriptor,
    load_inventory_reference,
    select_representatives,
    summarize_descriptors,
    trajectory_content_sha256,
)
from reconciliation.online_switch import load_strict_json, sha256_file  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_lightnav_execution_envelope.yaml",
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def save_npy(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.save(stream, value, allow_pickle=False)


def verify_file(path: Path, digest: str, label: str) -> None:
    if sha256_file(path) != digest:
        raise ValueError(f"Stage 0-F provenance mismatch: {label}")


def validate_stage0e(
    config: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    root = resolve_path(config["paths"]["stage0e_run"])
    provenance = config["stage0e_provenance"]
    for key, relative in (
        ("metadata_sha256", "metadata.json"),
        ("config_snapshot_sha256", "config_snapshot.yaml"),
        ("controlled_summary_sha256", "summary/controlled_paths_summary.json"),
        ("decision_sha256", "summary/final_execution_platform_decision.json"),
    ):
        verify_file(root / relative, str(provenance[key]), relative)
    metadata = load_strict_json(root / "metadata.json")
    decision = load_strict_json(
        root / "summary/final_execution_platform_decision.json"
    )
    summary = load_strict_json(root / "summary/controlled_paths_summary.json")
    if metadata["run_id"] != provenance["run_id"]:
        raise ValueError("Stage 0-E run id changed")
    if decision["status"] != provenance["required_decision"]:
        raise ValueError("Stage 0-E decision changed")
    passed = sum(
        bool(row["passed"]) for row in summary["calibrated"]["scenarios"].values()
    )
    if passed != int(provenance["calibrated_controlled_pass_count"]):
        raise ValueError("Stage 0-E calibrated fixture pass count changed")
    failed = {
        key
        for key, row in summary["calibrated"]["scenarios"].items()
        if not row["passed"]
    }
    if failed != set(provenance["strong_turn_failures"]):
        raise ValueError("Stage 0-E failing fixture set changed")
    return load_yaml(root / "config_snapshot.yaml"), summary


def descriptor_row(
    entry: Mapping[str, Any],
    inventory: Mapping[str, Any],
    fixture: Mapping[str, Any],
    distance_model: Mapping[str, Any],
    geometry_config: Mapping[str, Any],
) -> tuple[dict[str, Any], np.ndarray]:
    reference = load_inventory_reference(inventory, entry)
    descriptor = geometry_descriptor(
        reference,
        segment_epsilon_m=float(geometry_config["segment_epsilon_m"]),
        curvature_zero_epsilon_per_m=float(
            geometry_config["curvature_zero_epsilon_per_m"]
        ),
    )
    comparison = compare_descriptor_to_fixtures(
        descriptor, fixture, distance_model
    )
    return (
        {
            "reference_id": entry["reference_id"],
            "kind": entry["kind"],
            "world_trajectory_sha256": entry["world_trajectory_sha256"],
            "array_content_sha256": entry["array_content_sha256"],
            "execution_start_pose_se2": entry["execution_start_pose_se2"],
            "source_trial_count": len(entry["source_trials"]),
            "source_trial_paths": [
                value["relative_trial_path"] for value in entry["source_trials"]
            ],
            "source_geometry_ids": sorted(
                {value["geometry_id"] for value in entry["source_trials"]}
            ),
            "source_geometry_classes": sorted(
                {value["geometry_class"] for value in entry["source_trials"]}
            ),
            "source_latency_ids": sorted(
                {value["latency_id"] for value in entry["source_trials"]}
            ),
            "descriptor": descriptor,
            "fixture_comparison": comparison,
        },
        reference,
    )


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = load_yaml(config_path)
    if config.get("stage") != "stage0-f-lightnav-trajectory-execution-envelope":
        raise ValueError("invalid Stage 0-F config stage")
    if config["geometry"].get("intrinsic_waypoint_time_base") is not False:
        raise ValueError("Stage 0-F must not fabricate waypoint timestamps")
    if config["execution"].get("historical_controller_commands_reused") is not False:
        raise ValueError("Stage 0-F must not reuse historical controller commands")
    if config["execution"].get("canonicalize_references") is not False:
        raise ValueError("the frozen protocol executes source world references directly")
    source_config = resolve_path(config["paths"]["source_config"])
    verify_file(
        source_config,
        str(config["source_provenance"]["source_config_sha256"]),
        "source config",
    )

    stage0e_config, stage0e_summary = validate_stage0e(config)
    source_root = resolve_path(config["paths"]["source_cohort"])
    inventory = build_source_inventory(source_root, config["source_provenance"])
    source_metadata = load_strict_json(source_root / "metadata.json")
    stage0e_root = resolve_path(config["paths"]["stage0e_run"])
    stage0e_metadata = load_strict_json(stage0e_root / "metadata.json")
    stage0b_config_path = resolve_path(stage0e_config["paths"]["stage0b_config"])
    stage0b_config = load_yaml(stage0b_config_path)
    shared_reference_provenance = {
        "source_cohort": {
            "experiment_id": config["source_provenance"]["experiment_id"],
            "metadata_sha256": config["source_provenance"]["metadata_sha256"],
            "protocol_sha256": config["source_provenance"]["protocol_sha256"],
            "source_config_path": str(source_config),
            "source_config_sha256": config["source_provenance"][
                "source_config_sha256"
            ],
            "research_git_commit_sha_at_run": source_metadata[
                "research_git_commit_sha_at_run"
            ],
            "lightnav_sha": source_metadata["lightnav_server_end"]["lightnav_sha"],
            "lightnav_checkpoint_revision": source_metadata["lightnav_server_end"][
                "checkpoint_revision"
            ],
        },
        "stage0d_controller": {
            "run_directory": stage0e_metadata["stage0d_candidate"]["run_directory"],
            "model_sha256": stage0e_metadata["stage0d_candidate"]["model_sha256"],
            "selected_candidate_id": stage0e_metadata["stage0d_candidate"][
                "selected_candidate_id"
            ],
            "metadata_sha256": stage0e_config["stage0d_candidate_provenance"][
                "metadata_sha256"
            ],
            "config_snapshot_sha256": stage0e_config[
                "stage0d_candidate_provenance"
            ]["config_snapshot_sha256"],
        },
        "stage0e_follower": {
            "run_id": config["stage0e_provenance"]["run_id"],
            "config_snapshot_sha256": config["stage0e_provenance"][
                "config_snapshot_sha256"
            ],
            "stage0b_config_path": str(stage0b_config_path),
            "stage0b_config_sha256_at_stage0f_run": sha256_file(stage0b_config_path),
            "closed_loop_config": stage0b_config["closed_loop"],
        },
    }
    unique_count = int(inventory["unique_execution_reference_count"])
    maximum = int(config["execution"]["maximum_unique_references"])
    if unique_count > maximum:
        raise RuntimeError(
            f"unique execution references {unique_count} exceed frozen maximum {maximum}"
        )

    fixture = build_fixture_geometry(
        stage0e_config, stage0e_summary, config["geometry"]
    )
    distance_model = fit_fixture_distance_model(
        fixture, config["geometry"]["descriptor_distance"]
    )
    fixture["distance_model"] = distance_model

    run_id = args.run_id or datetime.now(timezone.utc).strftime(
        "stage0f-%Y%m%dT%H%M%SZ"
    )
    root = resolve_path(config["paths"]["output_root"]) / run_id
    root.mkdir(parents=True, exist_ok=False)
    with (root / "config_snapshot.yaml").open("x", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

    write_json(root / "summary/source_inventory.json", inventory)
    write_json(root / "summary/stage0e_fixture_geometry.json", fixture)
    geometry_rows: list[dict[str, Any]] = []
    for kind in ("old", "fresh"):
        for entry in inventory["unique_references"][kind]:
            row, reference = descriptor_row(
                entry,
                inventory,
                fixture,
                distance_model,
                config["geometry"],
            )
            destination = root / "geometry" / kind / row["reference_id"]
            save_npy(destination / "reference_world.npy", reference)
            write_json(destination / "descriptor.json", row)
            write_json(
                destination / "source_provenance.json",
                {
                    "source_root": inventory["source_root"],
                    "reference_id": row["reference_id"],
                    "kind": row["kind"],
                    "source_trials": entry["source_trials"],
                    "world_trajectory_sha256": row["world_trajectory_sha256"],
                    "derived_copy_sha256": sha256_file(
                        destination / "reference_world.npy"
                    ),
                    "source_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
                    "execution_frame": "same source world frame; no canonical transform",
                    "canonical_transform": None,
                    **shared_reference_provenance,
                },
            )
            geometry_rows.append(row)

    old_rows = [row for row in geometry_rows if row["kind"] == "OLD"]
    fresh_rows = [row for row in geometry_rows if row["kind"] == "FRESH"]
    write_json(
        root / "summary/old_geometry_summary.json",
        build_grouped_geometry_summary(old_rows),
    )
    write_json(
        root / "summary/fresh_geometry_summary.json",
        build_grouped_geometry_summary(fresh_rows),
    )

    suffix_rows: list[dict[str, Any]] = []
    for row in fresh_rows:
        reference = np.load(
            root / "geometry/fresh" / row["reference_id"] / "reference_world.npy",
            allow_pickle=False,
        )
        for k, suffix in extract_suffixes(reference):
            suffix_rows.append(
                {
                    "parent_reference_id": row["reference_id"],
                    "kind": "FRESH_SUFFIX",
                    "k": k,
                    "suffix_content_sha256": trajectory_content_sha256(suffix),
                    "source_geometry_ids": row["source_geometry_ids"],
                    "source_latency_ids": row["source_latency_ids"],
                    "descriptor": geometry_descriptor(
                        suffix,
                        segment_epsilon_m=float(
                            config["geometry"]["segment_epsilon_m"]
                        ),
                        curvature_zero_epsilon_per_m=float(
                            config["geometry"]["curvature_zero_epsilon_per_m"]
                        ),
                    ),
                }
            )
    write_json(
        root / "summary/fresh_suffix_geometry_summary.json",
        {
            **build_grouped_geometry_summary(suffix_rows),
            "suffix_count": len(suffix_rows),
            "minimum_suffix_pose_count": 2,
            "selector_or_boundary_evaluation": False,
            "suffixes": suffix_rows,
        },
    )

    label_counts = Counter(
        row["fixture_comparison"]["label"] for row in geometry_rows
    )
    comparison = {
        "distance_model": distance_model,
        "label_counts": dict(sorted(label_counts.items())),
        "by_kind": {
            kind: dict(
                sorted(
                    Counter(
                        row["fixture_comparison"]["label"]
                        for row in geometry_rows
                        if row["kind"] == kind
                    ).items()
                )
            )
            for kind in ("OLD", "FRESH")
        },
        "references": geometry_rows,
        "descriptive_not_learned_classifier": True,
    }
    write_json(root / "summary/geometry_fixture_comparison.json", comparison)
    representatives = select_representatives(
        geometry_rows, config["geometry"]["severity_order"]
    )
    write_json(root / "summary/representative_selection.json", representatives)
    write_json(
        root / "preparation_metadata.json",
        {
            "stage": config["stage"],
            "run_id": run_id,
            "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "git_commit_sha": subprocess.check_output(
                ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"],
                text=True,
            ).strip(),
            "config_source": str(config_path),
            "config_source_sha256": sha256_file(config_path),
            "config_snapshot_sha256": sha256_file(root / "config_snapshot.yaml"),
            "source_inventory_sha256": sha256_file(
                root / "summary/source_inventory.json"
            ),
            "stage0e_fixture_geometry_sha256": sha256_file(
                root / "summary/stage0e_fixture_geometry.json"
            ),
            "geometry_fixture_comparison_sha256": sha256_file(
                root / "summary/geometry_fixture_comparison.json"
            ),
            "representative_selection_sha256": sha256_file(
                root / "summary/representative_selection.json"
            ),
            "unique_execution_reference_count": unique_count,
            "maximum_unique_reference_gate": maximum,
            "execution_results_observed_before_selection": False,
            "intrinsic_waypoint_time_base": False,
        },
    )
    print(
        "STAGE0F_PREPARED="
        + json.dumps(
            {
                "run_directory": str(root),
                "attempt_count": inventory["attempt_count"],
                "moving_trial_count": inventory["eligible_moving_trial_count"],
                "stop_output_count": inventory["stop_output_count"],
                "unique_old": inventory["unique_old_world_reference_count"],
                "unique_fresh": inventory["unique_fresh_world_reference_count"],
                "representative_count": representatives["reference_count"],
            },
            sort_keys=True,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
