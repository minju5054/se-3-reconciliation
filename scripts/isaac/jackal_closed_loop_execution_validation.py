#!/usr/bin/env python3
"""Run the headless Stage 0-E same-reference closed-loop validation."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
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
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_closed_loop_execution_validation.yaml",
    )
    parser.add_argument("--run-id")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": True})

import numpy as np
import yaml

from closed_loop_execution_runtime import ClosedLoopRuntime, follower_config, run_closed_loop
from lightnav_stage0c_runtime import canonical_wheel_names
from reconciliation.closed_loop_execution_validation import (
    CONTROLLED_IDS,
    MODES,
    compare_modes,
    compute_closed_loop_metrics,
    evaluate_controlled_group,
    evaluate_exp02b_group,
    evaluate_scenario,
    final_platform_decision,
    generate_controlled_references,
    load_frozen_stage0d_candidate,
    save_closed_loop_trial,
    validate_closed_loop_trial,
    validate_stage0e_config,
)
from reconciliation.exp02b import load_source_case, verify_frozen_source_selection
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.trajectory import generate_reference_trajectory, segments_from_config


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(dict(value), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def save_reference(path: Path, reference: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        np.save(stream, reference)


def git_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()


def isaac_version() -> str:
    return (Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION").read_text(
        encoding="utf-8"
    ).strip()


def run_trial(
    runtime: ClosedLoopRuntime,
    config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    candidate: Mapping[str, Any],
    *,
    root: Path,
    subset: str,
    scenario: str,
    mode: str,
    repetition: int,
    reference: np.ndarray,
    reference_path: Path,
    initial_pose: np.ndarray,
    source: Mapping[str, Any],
) -> dict[str, Any]:
    telemetry = run_closed_loop(
        runtime, config, stage0b, reference, initial_pose, mode
    )
    metrics = compute_closed_loop_metrics(telemetry)
    destination = root / subset / scenario / mode / f"repetition_{repetition:02d}"
    metadata = {
        "stage": config["stage"],
        "subset": subset,
        "scenario": scenario,
        "mode": mode,
        "repetition": repetition,
        "evaluation_semantics": "reference-matched closed-loop evaluation",
        "trajectory_follower_feedback": "actual measured world pose at every control step",
        "historical_body_commands_reused": False,
        "initial_pose_se2": initial_pose.tolist(),
        "zero_wheel_state_before_settling": True,
        "settling_duration_s": float(config["simulation"]["settling_duration_s"]),
        "reference_path": str(reference_path.resolve()),
        "reference_sha256": sha256_file(reference_path),
        "reference_source": dict(source),
        "reference_waypoint_intrinsic_timestamps": False,
        "follower_config": {
            key: getattr(follower_config(stage0b), key)
            for key in follower_config(stage0b).__dataclass_fields__
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
        "coordinate_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
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
    hashes = save_closed_loop_trial(destination, telemetry, metrics, metadata)
    validate_closed_loop_trial(destination)
    row = {
        "subset": subset,
        "scenario": scenario,
        "mode": mode,
        "repetition": repetition,
        "trial_directory": str(destination.resolve()),
        "raw_sha256": hashes,
        **metrics,
    }
    print(
        "STAGE0E_TRIAL="
        + json.dumps(
            {
                "subset": subset,
                "scenario": scenario,
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
    return row


def summarize_group(
    rows: list[dict[str, Any]],
    scenario_order: list[str],
    criteria: Mapping[str, Any],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for mode in MODES:
        result[mode] = {
            scenario: evaluate_scenario(
                [
                    row
                    for row in rows
                    if row["mode"] == mode and row["scenario"] == scenario
                ],
                criteria,
            )
            for scenario in scenario_order
        }
    return result


def main() -> None:
    config_path = ARGS.config.resolve()
    config = load_yaml(config_path)
    config_validation = validate_stage0e_config(config)
    stage0b_path = resolve_path(config["paths"]["stage0b_config"])
    stage0b = load_yaml(stage0b_path)
    stage0d_run = resolve_path(config["paths"]["stage0d_run"])
    candidate = load_frozen_stage0d_candidate(
        stage0d_run,
        config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    exp02b_config_path = resolve_path(config["paths"]["exp02b_config"])
    exp02b_config = load_yaml(exp02b_config_path)
    exp02b_run = resolve_path(config["paths"]["exp02b_run"])
    selection = load_strict_json(exp02b_run / "source_selection.json")
    verify_frozen_source_selection(selection["source_root"], exp02b_config)
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("stage0e-%Y%m%dT%H%M%SZ")
    root = resolve_path(config["paths"]["output_root"]) / run_id
    root.mkdir(parents=True, exist_ok=False)
    with (root / "config_snapshot.yaml").open("x", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    write_json(root / "config_validation.json", config_validation)
    write_json(root / "acceptance_criteria.json", config["acceptance_criteria"])
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    runtime = ClosedLoopRuntime(
        SIMULATION_APP, config, stage0b, candidate, render=False
    )
    repetitions = int(config["acceptance_criteria"]["repetitions"])
    criteria = config["acceptance_criteria"]

    controlled_references = generate_controlled_references(config)
    controlled_rows: list[dict[str, Any]] = []
    geometry = {
        str(row["id"]): str(row["geometry"])
        for row in config["controlled_paths"]["scenarios"]
    }
    for scenario in CONTROLLED_IDS:
        reference = controlled_references[scenario]
        reference_path = root / "controlled_paths" / scenario / "reference_trajectory.npy"
        save_reference(reference_path, reference)
        source = {
            "kind": "deterministic_synthetic_engineering_fixture",
            "profile": next(
                row["segments"]
                for row in config["controlled_paths"]["scenarios"]
                if row["id"] == scenario
            ),
            "geometry": geometry[scenario],
        }
        for mode in MODES:
            for repetition in range(repetitions):
                controlled_rows.append(
                    run_trial(
                        runtime,
                        config,
                        stage0b,
                        candidate,
                        root=root,
                        subset="controlled_paths",
                        scenario=scenario,
                        mode=mode,
                        repetition=repetition,
                        reference=reference,
                        reference_path=reference_path,
                        initial_pose=np.asarray(config["simulation"]["initial_pose_se2"]),
                        source=source,
                    )
                )
    controlled_scenarios = summarize_group(
        controlled_rows, list(CONTROLLED_IDS), criteria
    )
    controlled_summary = {
        mode: evaluate_controlled_group(
            controlled_scenarios[mode], geometry, criteria
        )
        for mode in MODES
    }
    controlled_summary["evaluation_boundary"] = (
        "deterministic synthetic execution-platform validation; not research evidence"
    )
    write_json(root / "summary" / "controlled_paths_summary.json", controlled_summary)

    composite = generate_reference_trajectory(
        config["simulation"]["initial_pose_se2"],
        segments_from_config(stage0b["composite_motion_profile"]),
        float(config["simulation"]["control_dt_s"]),
    ).poses
    composite_path = root / "stage0b_composite" / "stage0b_composite" / "reference_trajectory.npy"
    save_reference(composite_path, composite)
    composite_rows: list[dict[str, Any]] = []
    for mode in MODES:
        for repetition in range(repetitions):
            composite_rows.append(
                run_trial(
                    runtime,
                    config,
                    stage0b,
                    candidate,
                    root=root,
                    subset="stage0b_composite",
                    scenario="stage0b_composite",
                    mode=mode,
                    repetition=repetition,
                    reference=composite,
                    reference_path=composite_path,
                    initial_pose=np.asarray(config["simulation"]["initial_pose_se2"]),
                    source={
                        "kind": "unchanged_stage0b_composite_motion_profile",
                        "stage0b_config_path": str(stage0b_path),
                        "stage0b_config_sha256": sha256_file(stage0b_path),
                        "profile": stage0b["composite_motion_profile"],
                    },
                )
            )
    composite_scenarios = summarize_group(
        composite_rows, ["stage0b_composite"], criteria
    )
    composite_summary = {
        mode: {
            **composite_scenarios[mode]["stage0b_composite"],
            "scenarios": {
                "stage0b_composite": composite_scenarios[mode]["stage0b_composite"]
            },
        }
        for mode in MODES
    }
    write_json(root / "summary" / "composite_summary.json", composite_summary)

    exp_rows: list[dict[str, Any]] = []
    case_order = list(config["exp02b_old_reference"]["cases"])
    for case_id in case_order:
        source_case = load_source_case(
            case_id, selection["cases"][case_id]["trial_directory"]
        )
        reference = source_case.old_world
        reference_path = (
            root / "exp02b_old_reference" / case_id / "reference_trajectory.npy"
        )
        save_reference(reference_path, reference)
        initial = np.asarray(source_case.source_metadata["initial_pose_se2"])
        source_info = {
            "kind": "frozen_exp02b_old_world_reference",
            "source_trial_path": str(source_case.trial_directory),
            "source_old_world_path": str(
                source_case.trial_directory / "derived/old_world.npy"
            ),
            "source_old_world_sha256": sha256_file(
                source_case.trial_directory / "derived/old_world.npy"
            ),
            "source_metadata_sha256": sha256_file(
                source_case.trial_directory / "metadata.json"
            ),
            "historical_controller_commands_used": False,
        }
        for mode in MODES:
            for repetition in range(repetitions):
                exp_rows.append(
                    run_trial(
                        runtime,
                        config,
                        stage0b,
                        candidate,
                        root=root,
                        subset="exp02b_old_reference",
                        scenario=case_id,
                        mode=mode,
                        repetition=repetition,
                        reference=reference,
                        reference_path=reference_path,
                        initial_pose=initial,
                        source=source_info,
                    )
                )
    exp_scenarios = summarize_group(exp_rows, case_order, criteria)
    exp_summary = {
        mode: evaluate_exp02b_group(exp_scenarios[mode], criteria) for mode in MODES
    }
    exp_summary["evaluation_semantics"] = "reference-matched closed-loop evaluation"
    exp_summary["representative_case"] = config["exp02b_old_reference"][
        "representative_case"
    ]
    write_json(root / "summary" / "exp02b_reference_summary.json", exp_summary)

    all_rows = controlled_rows + composite_rows + exp_rows
    mode_comparison = {
        "all_primary_trials": compare_modes(
            {"trials": [row for row in all_rows if row["mode"] == "nominal"]},
            {"trials": [row for row in all_rows if row["mode"] == "calibrated"]},
        ),
        "controlled_paths": compare_modes(
            {"trials": [row for row in controlled_rows if row["mode"] == "nominal"]},
            {"trials": [row for row in controlled_rows if row["mode"] == "calibrated"]},
        ),
        "stage0b_composite": compare_modes(
            {"trials": [row for row in composite_rows if row["mode"] == "nominal"]},
            {"trials": [row for row in composite_rows if row["mode"] == "calibrated"]},
        ),
        "exp02b_old_reference": compare_modes(
            {"trials": [row for row in exp_rows if row["mode"] == "nominal"]},
            {"trials": [row for row in exp_rows if row["mode"] == "calibrated"]},
        ),
    }
    write_json(root / "summary" / "mode_comparison.json", mode_comparison)
    mode_pass = {
        mode: bool(
            controlled_summary[mode]["passed"]
            and composite_summary[mode]["passed"]
            and exp_summary[mode]["passed"]
        )
        for mode in MODES
    }
    decision = final_platform_decision(
        nominal_passed=mode_pass["nominal"],
        calibrated_passed=mode_pass["calibrated"],
    )
    decision.update(
        {
            "group_checks": {
                mode: {
                    "controlled_paths": controlled_summary[mode]["passed"],
                    "stage0b_composite": composite_summary[mode]["passed"],
                    "exp02b_old_reference": exp_summary[mode]["passed"],
                }
                for mode in MODES
            },
            "stage0d_candidate_status_before_stage0e": candidate[
                "stage0d_validation_status"
            ],
            "new_controller_developed_or_tuned": False,
            "claim_scope": (
                "simulation trajectory-level execution platform selection only; not "
                "reconciliation performance or real-robot evidence"
            ),
        }
    )
    write_json(root / "summary" / "final_execution_platform_decision.json", decision)
    metadata = {
        "stage": config["stage"],
        "run_id": run_id,
        "created_at": started_at,
        "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit_sha": git_sha(),
        "config_source": str(config_path),
        "config_source_sha256": sha256_file(config_path),
        "config_snapshot_sha256": sha256_file(root / "config_snapshot.yaml"),
        "stage0d_candidate": candidate,
        "acceptance_criteria_frozen_before_results": True,
        "held_out_results_used_for_controller_tuning": False,
        "historical_body_commands_reused": False,
        "reference_matched_closed_loop": True,
        "repetitions_per_condition_per_mode": repetitions,
        "trial_count": len(all_rows),
        "isaac_sim_version": isaac_version(),
        "jackal_asset": runtime.asset,
        "articulation_prim_path": runtime.articulation_root,
        "wheel_runtime": runtime.wheels,
        "runtime_physics_overrides": {},
        "frozen_stage0b_stage0d_exp02b_results_modified": False,
        "research_evidence": False,
    }
    write_json(root / "metadata.json", metadata)
    print(
        "STAGE0E_RUN="
        + json.dumps(
            {
                "run_directory": str(root),
                "trial_count": len(all_rows),
                "decision": decision["status"],
            },
            sort_keys=True,
        ),
        flush=True,
    )


try:
    main()
finally:
    SIMULATION_APP.close()
