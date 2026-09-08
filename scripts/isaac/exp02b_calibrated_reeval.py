#!/usr/bin/env python3
"""Execute 27 frozen EXP-02B candidates through the calibrated Jackal layer."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
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
import omni.usd
import yaml
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, UsdGeom, UsdPhysics

from closed_loop_execution_runtime import ClosedLoopRuntime
from lightnav_stage0c_runtime import (
    canonical_wheel_names,
    canonical_wheel_values,
    quaternion_from_yaw,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.closed_loop_execution_validation import (
    ClosedLoopTelemetry,
    TELEMETRY_COLUMNS,
    compute_closed_loop_metrics,
    load_frozen_stage0d_candidate,
)
from reconciliation.controller_validation import WHEEL_LABELS, estimate_body_velocities
from reconciliation.controllers.trajectory_follower import TrajectoryFollower
from reconciliation.exp02b import comparability_gate, load_source_case
from reconciliation.exp02b_calibrated_reeval import (
    command_level_metrics,
    control_interval_body,
    first_command_invariant,
    follower_config,
    validate_branch_output,
    validate_reeval_config,
    verify_file_hash,
    verify_historical_run,
    write_json_exclusive,
)
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def add_static_box(path: str, center, size, color) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    transform = UsdGeom.XformCommonAPI(cube)
    transform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    transform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def source_command_rows(source_dir: Path) -> tuple[list[dict[str, str]], dict[str, str]]:
    with (source_dir / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    split = next(index for index, row in enumerate(rows) if row["reference_source"] == "FRESH")
    if split < 3:
        raise ValueError("source requires at least three OLD commands")
    return rows[:split], rows[split]


def replay_old_nominal(
    runtime: ClosedLoopRuntime,
    source,
    execution: Mapping[str, Any],
) -> dict[str, Any]:
    initial = np.asarray(source.source_metadata["initial_pose_se2"], dtype=np.float64)
    runtime.reset(initial, float(execution["settling_duration_s"]))
    old_rows, fresh_first = source_command_rows(source.trial_directory)
    poses = [se2_from_world_pose(runtime.robot)]
    times = [0.0]
    origin = float(runtime.world.current_time)
    last_start = poses[0]
    last_duration = 0.0
    last_target = np.zeros(4, dtype=np.float64)
    for index, row in enumerate(old_rows):
        next_time = (
            float(old_rows[index + 1]["sim_time_s"])
            if index + 1 < len(old_rows)
            else float(fresh_first["sim_time_s"])
        )
        duration = next_time - float(row["sim_time_s"])
        steps = int(round(duration / runtime.physics_dt))
        if steps < 1 or not math.isclose(
            steps * runtime.physics_dt, duration, abs_tol=1e-6
        ):
            raise ValueError(f"source command interval is not physics-step aligned: {duration}")
        desired = np.asarray(
            [float(row["v_command_mps"]), float(row["omega_command_rps"])],
            dtype=np.float64,
        )
        runtime_target = runtime_wheel_command(
            runtime.differential.forward(desired), runtime.wheels
        )
        last_target = canonical_wheel_values(runtime_target, runtime.wheels)
        runtime.articulation.apply_action(
            ArticulationAction(joint_velocities=runtime_target)
        )
        last_start = poses[-1].copy()
        last_duration = steps * runtime.physics_dt
        for _ in range(steps):
            runtime.step()
            poses.append(se2_from_world_pose(runtime.robot))
            times.append(float(runtime.world.current_time) - origin)
    pre_switch = np.asarray(
        [
            float(old_rows[-1]["v_command_mps"]),
            float(old_rows[-1]["omega_command_rps"]),
        ],
        dtype=np.float64,
    )
    last_end = poses[-1].copy()
    from reconciliation.exp02b_calibrated_reeval import body_interval_motion

    measured = body_interval_motion(last_start, last_end, last_duration)
    return {
        "actual": np.asarray(poses),
        "times": np.asarray(times),
        "reproduced_boundary": last_end,
        "pre_switch_desired": pre_switch,
        "last_interval_start": last_start,
        "last_interval_end": last_end,
        "last_interval_duration_s": last_duration,
        "last_interval_measured": measured,
        "last_wheel_target": last_target,
        "old_command_count": len(old_rows),
        "physics_step_count": len(poses) - 1,
        "source_switch_sim_time_s": float(fresh_first["sim_time_s"]),
    }


def exact_boundary_reset(
    runtime: ClosedLoopRuntime, boundary: np.ndarray, pre_switch_desired: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    runtime.robot.set_world_pose(
        position=np.asarray([boundary[0], boundary[1], runtime.spawn_height]),
        orientation=quaternion_from_yaw(float(boundary[2])),
    )
    wheel_runtime = runtime_wheel_command(
        runtime.differential.forward(pre_switch_desired), runtime.wheels
    )
    runtime.robot.set_joint_velocities(wheel_runtime)
    runtime.articulation.apply_action(
        ArticulationAction(joint_velocities=wheel_runtime)
    )
    return se2_from_world_pose(runtime.robot), canonical_wheel_values(
        wheel_runtime, runtime.wheels
    )


def execute_calibrated_post_switch(
    runtime: ClosedLoopRuntime,
    config: Mapping[str, Any],
    exp01b_config: Mapping[str, Any],
    candidate: np.ndarray,
    exact_boundary: np.ndarray,
    restored_wheel_target: np.ndarray,
    old_measured: np.ndarray,
    historical_first_desired: np.ndarray,
) -> tuple[ClosedLoopTelemetry, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    follower = TrajectoryFollower(candidate, follower_config(exp01b_config["closed_loop"]))
    correction = runtime.correction()
    if correction.integral_error_rad != 0.0:
        raise RuntimeError("calibrated controller did not start with reset integral state")
    current = exact_boundary.copy()
    command = follower.forward(current)
    first_desired = np.asarray(
        [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
    )
    invariant = first_command_invariant(
        historical_first_desired,
        first_desired,
        tolerance=float(config["protocol"]["first_desired_invariant_tolerance"]),
    )
    if not invariant["passed"]:
        raise RuntimeError(f"PROTOCOL_INVARIANT_FAILURE: {invariant}")

    poses = [current.copy()]
    times = [0.0]
    control_indices = [-1]
    nearest = [command.nearest_index]
    target_indices = [command.target_index]
    goals = [command.goal_reached]
    desired_rows = [np.zeros(2, dtype=np.float64)]
    executed_rows = [np.zeros(2, dtype=np.float64)]
    target_wheels = [restored_wheel_target.copy()]
    measured_wheels = [
        canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
    ]
    pi_rows = [0.0]
    integral_rows = [0.0]
    saturation_rows = [False]
    sign_rows = [False]
    control_poses = [current.copy()]
    desired_controls = []
    executed_controls = []
    states = []
    measured_feedback = old_measured.copy()
    origin = float(runtime.world.current_time)
    maximum_controls = int(
        round(float(config["protocol"]["branch_duration_s"]) / runtime.control_dt)
    )
    for control_index in range(maximum_controls):
        if command.goal_reached:
            break
        desired = np.asarray(
            [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
        )
        executed, runtime_target, state = runtime.apply(
            desired, "calibrated", correction, float(measured_feedback[1])
        )
        desired_controls.append(desired.copy())
        executed_controls.append(executed.copy())
        states.append(dict(state))
        canonical_target = canonical_wheel_values(runtime_target, runtime.wheels)
        interval_start = poses[-1].copy()
        for _ in range(runtime.physics_steps):
            runtime.step()
            poses.append(se2_from_world_pose(runtime.robot))
            times.append(float(runtime.world.current_time) - origin)
            control_indices.append(control_index)
            nearest.append(command.nearest_index)
            target_indices.append(command.target_index)
            goals.append(False)
            desired_rows.append(desired.copy())
            executed_rows.append(executed.copy())
            target_wheels.append(canonical_target.copy())
            measured_wheels.append(
                canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
            )
            pi_rows.append(float(state["pi_correction_rps"]))
            integral_rows.append(float(state["integral_error_rad"]))
            saturation_rows.append(bool(state["saturated"]))
            sign_rows.append(bool(state["sign_protection_event"]))
        current = poses[-1].copy()
        control_poses.append(current)
        from reconciliation.exp02b_calibrated_reeval import body_interval_motion

        measured_feedback = body_interval_motion(
            interval_start, current, runtime.control_dt
        )
        command = follower.forward(current)
        nearest[-1] = command.nearest_index
        target_indices[-1] = command.target_index
        goals[-1] = command.goal_reached
    runtime.stop()
    telemetry = ClosedLoopTelemetry(
        candidate,
        np.asarray(poses),
        np.asarray(times),
        np.asarray(control_indices),
        np.asarray(nearest),
        np.asarray(target_indices),
        np.asarray(goals),
        np.asarray(desired_rows),
        np.asarray(executed_rows),
        np.asarray(target_wheels),
        np.asarray(measured_wheels),
        np.asarray(pi_rows),
        np.asarray(integral_rows),
        np.asarray(saturation_rows),
        np.asarray(sign_rows),
    )
    desired_array = np.asarray(desired_controls)
    executed_array = np.asarray(executed_controls)
    control_pose_array = np.asarray(control_poses)
    measured_array = control_interval_body(control_pose_array, runtime.control_dt)
    return telemetry, desired_array, executed_array, measured_array, {
        "first_desired_invariant": invariant,
        "control_poses": control_pose_array,
        "controller_states": states,
        "goal_reached_within_branch": bool(command.goal_reached),
    }


def write_telemetry(path: Path, telemetry: ClosedLoopTelemetry) -> None:
    measured_v, measured_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    nearest = nearest_polyline_samples(
        telemetry.reference_trajectory, telemetry.actual_trajectory
    )
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TELEMETRY_COLUMNS)
        for index in range(telemetry.sample_count):
            writer.writerow(
                [
                    index,
                    telemetry.sim_times_s[index],
                    int(telemetry.control_indices[index]),
                    "EXACT_RESET" if index == 0 else "POST_SWITCH_EXECUTION",
                    int(telemetry.nearest_indices[index]),
                    int(telemetry.target_indices[index]),
                    bool(telemetry.goal_reached[index]),
                    *telemetry.desired_body[index],
                    *telemetry.executed_body[index],
                    measured_v[index],
                    measured_omega[index],
                    *telemetry.target_wheels[index],
                    *telemetry.measured_wheels[index],
                    *telemetry.actual_trajectory[index],
                    nearest["distance_m"][index],
                    nearest["yaw_error_rad"][index],
                    telemetry.pi_correction_rps[index],
                    telemetry.integral_error_rad[index],
                    bool(telemetry.saturated[index]),
                    bool(telemetry.sign_protection_events[index]),
                ]
            )


def write_control_csv(
    path: Path,
    values: np.ndarray,
    *,
    value_names: tuple[str, str],
    states: list[Mapping[str, Any]] | None = None,
) -> None:
    fields = ["control_index", "time_from_switch_s", *value_names]
    if states is not None:
        fields += [
            "pi_correction_rps",
            "integral_error_rad",
            "saturated",
            "sign_protection_event",
        ]
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(values):
            output = {
                "control_index": index,
                "time_from_switch_s": index * 0.1,
                value_names[0]: float(row[0]),
                value_names[1]: float(row[1]),
            }
            if states is not None:
                output.update(states[index])
            writer.writerow(output)


def write_measured_csv(
    path: Path,
    old_replay: Mapping[str, Any],
    control_poses: np.ndarray,
    measured: np.ndarray,
    control_dt: float,
) -> None:
    fields = [
        "segment",
        "interval_index",
        "duration_s",
        "measured_v_mps",
        "measured_omega_rps",
        "start_x",
        "start_y",
        "start_yaw",
        "end_x",
        "end_y",
        "end_yaw",
    ]
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow(
            {
                "segment": "OLD_PRE_RESET_FINAL_INTERVAL",
                "interval_index": -1,
                "duration_s": old_replay["last_interval_duration_s"],
                "measured_v_mps": old_replay["last_interval_measured"][0],
                "measured_omega_rps": old_replay["last_interval_measured"][1],
                **{
                    f"start_{name}": old_replay["last_interval_start"][index]
                    for index, name in enumerate(("x", "y", "yaw"))
                },
                **{
                    f"end_{name}": old_replay["last_interval_end"][index]
                    for index, name in enumerate(("x", "y", "yaw"))
                },
            }
        )
        for index, body in enumerate(measured):
            writer.writerow(
                {
                    "segment": "POST_RESET_CANDIDATE_INTERVAL",
                    "interval_index": index,
                    "duration_s": control_dt,
                    "measured_v_mps": body[0],
                    "measured_omega_rps": body[1],
                    **{
                        f"start_{name}": control_poses[index, component]
                        for component, name in enumerate(("x", "y", "yaw"))
                    },
                    **{
                        f"end_{name}": control_poses[index + 1, component]
                        for component, name in enumerate(("x", "y", "yaw"))
                    },
                }
            )


def save_branch(
    destination: Path,
    telemetry: ClosedLoopTelemetry,
    desired: np.ndarray,
    executed: np.ndarray,
    measured: np.ndarray,
    old_replay: Mapping[str, Any],
    details: Mapping[str, Any],
    controller_metrics: Mapping[str, Any],
    tracking_metrics: Mapping[str, Any],
    metadata: Mapping[str, Any],
) -> dict[str, str]:
    destination.mkdir(parents=True, exist_ok=False)
    np.save(destination / "actual_trajectory.npy", telemetry.actual_trajectory)
    np.save(destination / "old_replay_actual.npy", old_replay["actual"])
    write_telemetry(destination / "telemetry.csv", telemetry)
    write_control_csv(
        destination / "desired_commands.csv",
        desired,
        value_names=("desired_v_mps", "desired_omega_rps"),
    )
    write_control_csv(
        destination / "executed_commands.csv",
        executed,
        value_names=("executed_v_mps", "executed_omega_rps"),
        states=list(details["controller_states"]),
    )
    write_measured_csv(
        destination / "measured_body.csv",
        old_replay,
        np.asarray(details["control_poses"]),
        measured,
        float(metadata["control_dt_s"]),
    )
    write_json_exclusive(destination / "controller_metrics.json", controller_metrics)
    write_json_exclusive(destination / "tracking_metrics.json", tracking_metrics)
    write_json_exclusive(destination / "metadata.json", metadata)
    raw_names = (
        "actual_trajectory.npy",
        "old_replay_actual.npy",
        "telemetry.csv",
        "desired_commands.csv",
        "executed_commands.csv",
        "measured_body.csv",
    )
    hashes = {name: sha256_file(destination / name) for name in raw_names}
    write_json_exclusive(destination / "raw_provenance.json", {"sha256": hashes})
    return hashes


def main() -> None:
    run = ARGS.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    validate_reeval_config(config)
    preparation = load_strict_json(run / "metadata.json")
    verify_file_hash(
        run / "config_snapshot.yaml",
        preparation["config_snapshot_sha256"],
        "prepared config snapshot",
    )
    verify_file_hash(
        run / "source_provenance.json",
        preparation["source_provenance_sha256"],
        "prepared source provenance",
    )
    verify_file_hash(
        run / "historical_metric_snapshot.json",
        preparation["historical_metric_snapshot_sha256"],
        "prepared historical metric snapshot",
    )
    paths = config["paths"]
    exp02b_config = load_yaml(resolve_path(paths["historical_exp02b_config"]))
    exp01b_config = load_yaml(resolve_path(paths["source_exp01b_config"]))
    historical = verify_historical_run(
        resolve_path(paths["historical_exp02b_run"]), config, exp02b_config
    )
    snapshot = load_strict_json(run / "historical_metric_snapshot.json")
    snapshot_by_key = {
        (row["case_id"], int(row["entry_index"]), row["method"]): row
        for row in snapshot["branches"]
    }
    candidate = load_frozen_stage0d_candidate(
        resolve_path(paths["stage0d_run"]),
        config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    runtime = ClosedLoopRuntime(
        SIMULATION_APP,
        {"simulation": config["simulation"]},
        exp01b_config,
        candidate,
        render=False,
    )
    for box in exp01b_config["scene"]["static_boxes"]:
        add_static_box(f"/World/{box['id']}", box["center"], box["size"], box["color"])
    runtime.world.reset()

    gate_values = exp02b_config["execution"]["comparability_gate"]
    execution = config["simulation"]
    branch_summaries = []
    for branch in historical["branches"]:
        key = (branch["case_id"], int(branch["entry_index"]), branch["method"])
        frozen = snapshot_by_key[key]
        verify_file_hash(
            Path(frozen["candidate_path"]),
            frozen["candidate_sha256"],
            f"candidate {key}",
        )
        source = load_source_case(
            branch["case_id"],
            historical["source_selection"]["cases"][branch["case_id"]]["trial_directory"],
        )
        old_replay = replay_old_nominal(runtime, source, execution)
        gate = comparability_gate(
            source_boundary=source.boundary_pose_world,
            reproduced_boundary=old_replay["reproduced_boundary"],
            source_pre_switch_command=source.old_commands[-1],
            reproduced_pre_switch_command=old_replay["pre_switch_desired"],
            translation_tolerance_m=float(gate_values["translation_tolerance_m"]),
            yaw_tolerance_rad=float(gate_values["yaw_tolerance_rad"]),
            command_tolerance=float(gate_values["command_tolerance"]),
        )
        if not gate["valid"]:
            raise RuntimeError(f"historical nominal OLD comparability failed: {key}: {gate}")
        exact_pose, restored_target = exact_boundary_reset(
            runtime, source.boundary_pose_world, source.old_commands[-1]
        )
        expected_effective = np.asarray(
            frozen["historical_follower_effective_boundary_se2"], dtype=np.float64
        )
        if not np.allclose(exact_pose, expected_effective, rtol=0.0, atol=1e-12):
            raise RuntimeError(
                f"exact-reset follower boundary differs from historical branch: {key}: "
                f"{exact_pose.tolist()} vs {expected_effective.tolist()}"
            )
        candidate_array = np.load(frozen["candidate_path"], allow_pickle=False)
        historical_first = np.asarray(
            frozen["historical_controller_metrics"]["fresh_first_command_v_omega"]
        )
        telemetry, desired, executed, measured, details = execute_calibrated_post_switch(
            runtime,
            config,
            exp01b_config,
            candidate_array,
            exact_pose,
            restored_target,
            np.asarray(old_replay["last_interval_measured"]),
            historical_first,
        )
        window = int(config["protocol"]["command_window"])
        controller_metrics = command_level_metrics(
            old_desired=source.old_commands[-1],
            post_desired=desired,
            post_executed=executed,
            old_measured=old_replay["last_interval_measured"],
            post_measured=measured,
            control_dt_s=runtime.control_dt,
            window_size=window,
        )
        controller_metrics["first_desired_invariant"] = details["first_desired_invariant"]
        controller_metrics["calibrated_controller"] = {
            "state_at_switch": "RESET",
            "first_feedback_measurement_source": "final_pre_reset_old_control_interval",
            "first_feedback_measured_v_omega": old_replay["last_interval_measured"].tolist(),
            "maximum_abs_pi_correction_rps": float(
                np.max(np.abs(telemetry.pi_correction_rps))
            ),
            "integral_state_min_rad": float(np.min(telemetry.integral_error_rad)),
            "integral_state_max_rad": float(np.max(telemetry.integral_error_rad)),
            "saturation_fraction_by_control": float(
                np.mean([bool(row["saturated"]) for row in details["controller_states"]])
            ),
            "sign_protection_event_count": int(
                sum(bool(row["sign_protection_event"]) for row in details["controller_states"])
            ),
        }
        tracking_metrics = compute_closed_loop_metrics(telemetry)
        destination = (
            run
            / "branch_results"
            / key[0]
            / f"k_{key[1]}"
            / key[2]
        )
        metadata = {
            "experiment": "EXP-02B-R",
            "case_id": key[0],
            "entry_index": key[1],
            "method": key[2],
            "candidate_path": frozen["candidate_path"],
            "candidate_sha256": frozen["candidate_sha256"],
            "candidate_regenerated": False,
            "historical_controller_metrics_sha256": frozen[
                "historical_controller_metrics_sha256"
            ],
            "historical_geometric_metrics_sha256": frozen[
                "geometric_metrics_sha256"
            ],
            "historical_geometric_metrics": frozen["geometric_metrics"],
            "saved_boundary_se2": source.boundary_pose_world.tolist(),
            "reproduced_boundary_se2": old_replay["reproduced_boundary"].tolist(),
            "exact_reset_effective_boundary_se2": exact_pose.tolist(),
            "historical_follower_effective_boundary_se2": expected_effective.tolist(),
            "comparability_gate": gate,
            "exact_boundary_reset_occurred": True,
            "last_old_wheel_target_restored": True,
            "calibrated_controller_state_at_switch": "RESET",
            "first_calibrated_feedback_measurement": "final_pre_reset_old_control_interval",
            "historical_old_execution_mode": "nominal",
            "post_switch_execution_mode": "calibrated",
            "control_dt_s": runtime.control_dt,
            "physics_dt_s": runtime.physics_dt,
            "branch_duration_s": float(config["protocol"]["branch_duration_s"]),
            "coordinate_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
            "wheel_order": list(WHEEL_LABELS),
            "runtime_wheel_dof_names": list(canonical_wheel_names(runtime.wheels)),
            "stage0d_candidate_model_sha256": candidate["model_sha256"],
            "stage0d_candidate_id": candidate["selected_candidate_id"],
            "execution_layer": "TrajectoryFollower -> frozen Stage 0-D calibrated controller -> DifferentialController -> Jackal",
            "reset_derivative_computed": False,
            "goal_reached_within_short_branch": details["goal_reached_within_branch"],
            "goal_semantics": "descriptive only; 2 s branch endpoint reach is not a navigation pass/fail gate",
        }
        hashes = save_branch(
            destination,
            telemetry,
            desired,
            executed,
            measured,
            old_replay,
            details,
            controller_metrics,
            tracking_metrics,
            metadata,
        )
        validation = validate_branch_output(destination, frozen["candidate_sha256"])
        summary = {
            "case_id": key[0],
            "entry_index": key[1],
            "method": key[2],
            "branch_directory": str(destination),
            "candidate_sha256": frozen["candidate_sha256"],
            "comparability": gate,
            "first_desired_invariant": details["first_desired_invariant"],
            "controller_metrics": controller_metrics,
            "tracking_metrics": tracking_metrics,
            "historical_nominal_post_metrics": frozen["historical_nominal_post_metrics"],
            "raw_sha256": hashes,
            "validation": validation,
        }
        branch_summaries.append(summary)
        print(
            "EXP02B_R_BRANCH="
            + json.dumps(
                {
                    "case": key[0],
                    "k": key[1],
                    "method": key[2],
                    "desired_delta": [
                        controller_metrics["desired"]["delta_v_abs"],
                        controller_metrics["desired"]["delta_omega_abs"],
                    ],
                    "measured_delta": [
                        controller_metrics["measured"]["delta_v_abs"],
                        controller_metrics["measured"]["delta_omega_abs"],
                    ],
                    "position_rmse_m": tracking_metrics["position_rmse_m"],
                    "yaw_rmse_rad": tracking_metrics["yaw_rmse_rad"],
                },
                sort_keys=True,
            ),
            flush=True,
        )
    execution_summary = {
        "experiment": "EXP-02B-R",
        "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "branch_count": len(branch_summaries),
        "all_comparability_gates_passed": all(
            row["comparability"]["valid"] for row in branch_summaries
        ),
        "all_first_desired_invariants_passed": all(
            row["first_desired_invariant"]["passed"] for row in branch_summaries
        ),
        "historical_old_execution": "nominal command replay",
        "post_switch_execution": "frozen Stage 0-D pi_strong calibrated execution",
        "calibrated_controller_state_at_switch": "RESET",
        "candidate_regeneration_performed": False,
        "reconciliation_or_controller_tuning_performed": False,
        "isaac_sim_version": (
            Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION"
        ).read_text(encoding="utf-8").strip(),
        "git_commit_sha": subprocess.check_output(
            ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
        ).strip(),
        "branches": branch_summaries,
    }
    write_json_exclusive(run / "execution_summary.json", execution_summary)
    print(f"EXP02B_R_EXECUTION_BRANCH_COUNT={len(branch_summaries)}")


try:
    main()
finally:
    SIMULATION_APP.close()
