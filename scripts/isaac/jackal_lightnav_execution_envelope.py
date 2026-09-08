#!/usr/bin/env python3
"""Execute the frozen Stage 0-F LightNav reference cohort in Isaac Sim."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": True})

import numpy as np
import yaml

from closed_loop_execution_runtime import ClosedLoopRuntime, follower_config, run_closed_loop
from lightnav_stage0c_runtime import canonical_wheel_names
from reconciliation.closed_loop_execution_validation import (
    compare_modes,
    evaluate_scenario,
    load_frozen_stage0d_candidate,
    save_closed_loop_trial,
    validate_closed_loop_trial,
)
from reconciliation.lightnav_execution_envelope import extend_closed_loop_metrics
from reconciliation.online_switch import load_strict_json, sha256_file


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


def verify_hash(path: Path, expected: str, label: str) -> None:
    if sha256_file(path) != expected:
        raise ValueError(f"prepared Stage 0-F artifact changed: {label}")


def load_prepared(run: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    config = load_yaml(run / "config_snapshot.yaml")
    metadata = load_strict_json(run / "preparation_metadata.json")
    inventory_path = run / "summary/source_inventory.json"
    fixture_path = run / "summary/stage0e_fixture_geometry.json"
    comparison_path = run / "summary/geometry_fixture_comparison.json"
    selection_path = run / "summary/representative_selection.json"
    verify_hash(
        run / "config_snapshot.yaml",
        metadata["config_snapshot_sha256"],
        "config snapshot",
    )
    verify_hash(
        inventory_path, metadata["source_inventory_sha256"], "source inventory"
    )
    verify_hash(
        fixture_path,
        metadata["stage0e_fixture_geometry_sha256"],
        "Stage 0-E fixture geometry",
    )
    verify_hash(
        comparison_path,
        metadata["geometry_fixture_comparison_sha256"],
        "fixture comparison",
    )
    verify_hash(
        selection_path,
        metadata["representative_selection_sha256"],
        "representative selection",
    )
    inventory = load_strict_json(inventory_path)
    comparison = load_strict_json(comparison_path)
    selection = load_strict_json(selection_path)
    if not selection.get("selection_frozen_before_execution"):
        raise ValueError("representative selection was not frozen before execution")
    return config, inventory, comparison


def run_one(
    runtime: ClosedLoopRuntime,
    runtime_config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    candidate: Mapping[str, Any],
    *,
    run: Path,
    destination: Path,
    row: Mapping[str, Any],
    mode: str,
    repetition: int,
) -> dict[str, Any]:
    kind = str(row["kind"]).lower()
    reference_path = (
        run / "geometry" / kind / row["reference_id"] / "reference_world.npy"
    )
    provenance_path = (
        run / "geometry" / kind / row["reference_id"] / "source_provenance.json"
    )
    provenance = load_strict_json(provenance_path)
    if sha256_file(reference_path) != provenance["derived_copy_sha256"]:
        raise ValueError("prepared execution reference changed")
    reference = np.load(reference_path, allow_pickle=False)
    initial = np.asarray(row["execution_start_pose_se2"], dtype=np.float64)
    telemetry = run_closed_loop(
        runtime,
        runtime_config,
        stage0b,
        reference,
        initial,
        mode,
    )
    metrics = extend_closed_loop_metrics(telemetry)
    follower = follower_config(stage0b)
    metadata = {
        "stage": "stage0-f-lightnav-trajectory-execution-envelope",
        # Stage 0-E's strict trial validator uses this generic key.  Keep the
        # Stage 0-F-specific reference_id beside it so the reuse is explicit.
        "scenario": row["reference_id"],
        "reference_id": row["reference_id"],
        "kind": row["kind"],
        "mode": mode,
        "repetition": repetition,
        "evaluation_semantics": "reference-matched closed-loop evaluation",
        "trajectory_follower_feedback": "actual measured world pose at every control step",
        "historical_body_commands_reused": False,
        "reference_path": str(reference_path),
        "reference_derived_copy_sha256": sha256_file(reference_path),
        "source_world_trajectory_sha256": row["world_trajectory_sha256"],
        "source_trial_paths": row["source_trial_paths"],
        "source_geometry_ids": row["source_geometry_ids"],
        "source_latency_ids": row["source_latency_ids"],
        "source_provenance_sha256": sha256_file(provenance_path),
        "start_pose_source": (
            "robot_pose_at_old_observation"
            if row["kind"] == "OLD"
            else "robot_pose_at_fresh_observation"
        ),
        "initial_pose_se2": initial.tolist(),
        "zero_wheel_state_before_settling": True,
        "settling_duration_s": float(
            runtime_config["simulation"]["settling_duration_s"]
        ),
        "follower_config": {
            name: getattr(follower, name)
            for name in follower.__dataclass_fields__
        },
        "execution_layer": (
            "TrajectoryFollower -> DifferentialController"
            if mode == "nominal"
            else "TrajectoryFollower -> frozen Stage 0-D JackalExecutionController -> DifferentialController"
        ),
        "controller_parameters": (
            {} if mode == "nominal" else candidate["selected_parameters"]
        ),
        "stage0d_candidate_model_sha256": candidate["model_sha256"],
        "fixture_label": row["fixture_comparison"]["label"],
        "nearest_stage0e_fixture": row["fixture_comparison"]["nearest_fixture"],
        "coordinate_frame": "source Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
        "canonical_transform_applied": False,
        "intrinsic_waypoint_time_base": False,
        "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
        "runtime_wheel_dof_names": list(canonical_wheel_names(runtime.wheels)),
        "timing": {
            "physics_dt_s": runtime.physics_dt,
            "control_dt_s": runtime.control_dt,
            "observation": "pose and wheel state observed after each physics step",
            "controller_ready": "follower and low-level command computed synchronously",
            "execution": "desired/executed command applies over interval ending at saved row",
        },
        "runtime_physics_overrides": {},
        "research_evidence": False,
    }
    hashes = save_closed_loop_trial(
        destination, telemetry, metrics, metadata
    )
    validate_closed_loop_trial(destination)
    result = {
        "reference_id": row["reference_id"],
        "kind": row["kind"],
        "mode": mode,
        "repetition": repetition,
        "trial_directory": str(destination),
        "raw_sha256": hashes,
        **metrics,
    }
    print(
        "STAGE0F_TRIAL="
        + json.dumps(
            {
                "reference_id": row["reference_id"],
                "kind": row["kind"],
                "mode": mode,
                "repetition": repetition,
                "goal": metrics["goal_reached"],
                "position_rmse_m": metrics["position_rmse_m"],
                "yaw_rmse_rad": metrics["yaw_rmse_rad"],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return result


def summarize_conditions(
    rows: list[dict[str, Any]],
    geometry: list[dict[str, Any]],
    criteria: Mapping[str, Any],
    *,
    mode: str,
) -> dict[str, Any]:
    conditions = {}
    for reference in geometry:
        trials = [
            row
            for row in rows
            if row["mode"] == mode
            and row["kind"] == reference["kind"]
            and row["reference_id"] == reference["reference_id"]
        ]
        evaluation = evaluate_scenario(trials, criteria)
        conditions[reference["reference_id"]] = {
            **evaluation,
            "reference_id": reference["reference_id"],
            "kind": reference["kind"],
            "world_trajectory_sha256": reference["world_trajectory_sha256"],
            "source_trial_count": reference["source_trial_count"],
            "source_trial_paths": reference["source_trial_paths"],
            "source_geometry_ids": reference["source_geometry_ids"],
            "source_latency_ids": reference["source_latency_ids"],
            "descriptor": reference["descriptor"],
            "fixture_comparison": reference["fixture_comparison"],
        }
    return {
        "mode": mode,
        "unique_reference_count": len(conditions),
        "passing_reference_count": sum(
            bool(row["passed"]) for row in conditions.values()
        ),
        "all_references_passed": all(
            bool(row["passed"]) for row in conditions.values()
        ),
        "conditions": conditions,
    }


def main() -> None:
    run = ARGS.run_directory.resolve()
    config, inventory, comparison = load_prepared(run)
    runtime_config = load_yaml(
        resolve_path(config["paths"]["stage0e_run"]) / "config_snapshot.yaml"
    )
    stage0b_path = resolve_path(runtime_config["paths"]["stage0b_config"])
    stage0b = load_yaml(stage0b_path)
    candidate = load_frozen_stage0d_candidate(
        resolve_path(runtime_config["paths"]["stage0d_run"]),
        runtime_config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    geometry = list(comparison["references"])
    expected = int(inventory["unique_execution_reference_count"])
    if len(geometry) != expected:
        raise ValueError("prepared geometry count differs from source inventory")
    maximum = int(config["execution"]["maximum_unique_references"])
    if expected > maximum:
        raise RuntimeError(
            f"unique execution references {expected} exceed frozen maximum {maximum}"
        )
    repetitions = int(config["execution"]["repetitions"])
    criteria = runtime_config["acceptance_criteria"]
    if repetitions != int(criteria["repetitions"]):
        raise ValueError("Stage 0-F repetitions differ from frozen Stage 0-E gate")
    runtime = ClosedLoopRuntime(
        SIMULATION_APP, runtime_config, stage0b, candidate, render=False
    )

    calibrated_rows: list[dict[str, Any]] = []
    for row in geometry:
        for repetition in range(repetitions):
            calibrated_rows.append(
                run_one(
                    runtime,
                    runtime_config,
                    stage0b,
                    candidate,
                    run=run,
                    destination=(
                        run
                        / "execution"
                        / row["kind"].lower()
                        / row["reference_id"]
                        / "calibrated"
                        / f"repetition_{repetition:02d}"
                    ),
                    row=row,
                    mode="calibrated",
                    repetition=repetition,
                )
            )

    old_geometry = [row for row in geometry if row["kind"] == "OLD"]
    fresh_geometry = [row for row in geometry if row["kind"] == "FRESH"]
    old_summary = summarize_conditions(
        calibrated_rows, old_geometry, criteria, mode="calibrated"
    )
    old_summary["coverage_label"] = (
        "OLD_EXECUTION_ENVELOPE_COVERED"
        if old_summary["all_references_passed"]
        else "OLD_EXECUTION_ENVELOPE_HAS_UNVALIDATED_CASES"
    )
    fresh_summary = summarize_conditions(
        calibrated_rows, fresh_geometry, criteria, mode="calibrated"
    )
    fresh_summary["coverage_label"] = (
        "FRESH_EXECUTION_ENVELOPE_COVERED_AT_OBSERVATION_STATE"
        if fresh_summary["all_references_passed"]
        else "FRESH_EXECUTION_ENVELOPE_HAS_UNVALIDATED_CASES"
    )
    write_json(run / "summary/old_execution_summary.json", old_summary)
    write_json(run / "summary/fresh_execution_summary.json", fresh_summary)

    selection = load_strict_json(run / "summary/representative_selection.json")
    selected_keys = {
        (row["kind"], row["world_trajectory_sha256"])
        for row in selection["references"]
    }
    selected_geometry = [
        row
        for row in geometry
        if (row["kind"], row["world_trajectory_sha256"]) in selected_keys
    ]
    nominal_rows: list[dict[str, Any]] = []
    for row in selected_geometry:
        for repetition in range(repetitions):
            nominal_rows.append(
                run_one(
                    runtime,
                    runtime_config,
                    stage0b,
                    candidate,
                    run=run,
                    destination=(
                        run
                        / "nominal_representatives"
                        / row["kind"].lower()
                        / row["reference_id"]
                        / "nominal"
                        / f"repetition_{repetition:02d}"
                    ),
                    row=row,
                    mode="nominal",
                    repetition=repetition,
                )
            )
    nominal_summary = summarize_conditions(
        nominal_rows, selected_geometry, criteria, mode="nominal"
    )
    calibrated_selected = summarize_conditions(
        calibrated_rows, selected_geometry, criteria, mode="calibrated"
    )
    comparisons = {}
    for row in selected_geometry:
        reference_id = row["reference_id"]
        nominal_trials = [
            value
            for value in nominal_rows
            if value["reference_id"] == reference_id
        ]
        calibrated_trials = [
            value
            for value in calibrated_rows
            if value["reference_id"] == reference_id
        ]
        comparisons[reference_id] = compare_modes(
            {"trials": nominal_trials}, {"trials": calibrated_trials}
        )
    write_json(
        run / "summary/representative_mode_comparison.json",
        {
            "selection_sha256": sha256_file(
                run / "summary/representative_selection.json"
            ),
            "selection_frozen_before_execution": True,
            "nominal": nominal_summary,
            "calibrated": calibrated_selected,
            "comparisons": comparisons,
        },
    )
    write_json(
        run / "execution_metadata.json",
        {
            "stage": config["stage"],
            "run_id": run.name,
            "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "git_commit_sha": subprocess.check_output(
                ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"],
                text=True,
            ).strip(),
            "calibrated_unique_reference_count": len(geometry),
            "calibrated_trial_count": len(calibrated_rows),
            "nominal_representative_count": len(selected_geometry),
            "nominal_trial_count": len(nominal_rows),
            "repetitions": repetitions,
            "stage0d_candidate": candidate,
            "stage0e_runtime_config_sha256": sha256_file(
                resolve_path(config["paths"]["stage0e_run"])
                / "config_snapshot.yaml"
            ),
            "stage0b_config_path": str(stage0b_path),
            "stage0b_config_sha256": sha256_file(stage0b_path),
            "isaac_sim_version": (
                Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION"
            ).read_text(encoding="utf-8").strip(),
            "historical_controller_commands_reused": False,
            "controller_or_physics_tuned": False,
            "canonical_transform_applied": False,
            "runtime_physics_overrides": {},
        },
    )
    print(
        "STAGE0F_EXECUTION="
        + json.dumps(
            {
                "run_directory": str(run),
                "calibrated_trials": len(calibrated_rows),
                "nominal_trials": len(nominal_rows),
                "old_pass": old_summary["passing_reference_count"],
                "old_count": old_summary["unique_reference_count"],
                "fresh_pass": fresh_summary["passing_reference_count"],
                "fresh_count": fresh_summary["unique_reference_count"],
            },
            sort_keys=True,
        ),
        flush=True,
    )


try:
    main()
finally:
    SIMULATION_APP.close()
