#!/usr/bin/env python3
"""Run the headless Stage 0-D Jackal execution calibration and validation."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_jackal_execution_calibration.yaml",
    )
    parser.add_argument("--run-id", help="exclusive output directory name")
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="one calibration condition only; never eligible for final validation",
    )
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": True})

import numpy as np
import omni.usd
import yaml
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from pxr import UsdLux

from lightnav_stage0c_runtime import (
    canonical_wheel_names,
    canonical_wheel_values,
    discover_wheels,
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.controller_validation import (
    ControllerTelemetry,
    compute_controller_metrics,
    estimate_body_velocities,
)
from reconciliation.controllers.jackal_execution_controller import (
    JackalExecutionController,
    JackalExecutionControllerConfig,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.execution_calibration import (
    ExecutionTelemetry,
    assert_disjoint_condition_sets,
    compute_trial_metrics,
    evaluate_tracking_acceptance,
    fit_effective_width_model,
    response_dependence,
    save_execution_trial,
    select_feedback_candidate,
    summarize_conditions,
    validate_execution_calibration_config,
    validate_execution_trial,
    write_json_exclusive,
)
from reconciliation.exp02b import load_source_case, verify_frozen_source_selection
from reconciliation.exp02b_diagnosis import (
    DiagnosticTelemetry,
    execution_tracking_metrics,
    spatial_reference_metrics,
)
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
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


def condition_id(v: float, omega: float) -> str:
    def token(value: float) -> str:
        sign = "p" if value >= 0.0 else "m"
        return sign + f"{abs(value):.2f}".replace(".", "")

    return f"v_{token(v)}__omega_{token(omega)}"


def grid_conditions(values: Mapping[str, Any], *, smoke: bool = False) -> list[tuple[float, float]]:
    conditions = [
        (float(v), float(omega))
        for v in values["linear_velocity_mps"]
        for omega in values["angular_velocity_rps"]
    ]
    if smoke:
        return [(0.0, -0.3), (0.0, 0.3)]
    return conditions


def follower_config(stage0b: Mapping[str, Any]) -> FollowerConfig:
    values = stage0b["closed_loop"]
    return FollowerConfig(
        lookahead_distance_m=float(values["lookahead_distance_m"]),
        position_gain=float(values["position_gain"]),
        heading_gain=float(values["heading_gain"]),
        cross_track_gain=float(values["cross_track_gain"]),
        max_linear_velocity_mps=float(values["max_linear_velocity_mps"]),
        max_angular_velocity_rps=float(values["max_angular_velocity_rps"]),
        goal_position_tolerance_m=float(values["goal_position_tolerance_m"]),
        goal_yaw_tolerance_rad=float(values["goal_yaw_tolerance_rad"]),
        rotate_in_place_threshold_rad=float(values["rotate_in_place_threshold_rad"]),
        nearest_search_window=int(values["nearest_search_window"]),
    )


def git_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()


def isaac_version() -> str:
    path = Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION"
    return path.read_text(encoding="utf-8").strip()


def source_old_rows(source_dir: Path) -> list[dict[str, str]]:
    with (source_dir / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    old = [row for row in rows if row["reference_source"] == "OLD"]
    if len(old) < 2:
        raise ValueError("EXP-02B source needs OLD controller commands")
    return old


class Runtime:
    def __init__(self, config: Mapping[str, Any], stage0b: Mapping[str, Any]) -> None:
        simulation = config["simulation"]
        self.physics_dt = float(simulation["physics_dt_s"])
        self.control_dt = float(simulation["control_dt_s"])
        self.physics_steps = int(round(self.control_dt / self.physics_dt))
        if not math.isclose(
            self.physics_steps * self.physics_dt, self.control_dt, abs_tol=1e-9
        ):
            raise ValueError("control_dt_s must be divisible by physics_dt_s")
        self.spawn_height = float(simulation["spawn_height_m"])
        self.initial_pose = np.asarray(simulation["initial_pose_se2"], dtype=np.float64)
        robot_config = stage0b["robot"]
        self.asset = resolve_jackal_asset(robot_config)
        self.world = World(
            physics_dt=self.physics_dt,
            rendering_dt=self.physics_dt,
            stage_units_in_meters=1.0,
        )
        self.world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, "/World/Stage0DLight")
        dome.CreateIntensityAttr(1000.0)
        reference_path = str(robot_config["reference_prim_path"])
        add_reference_to_stage(str(self.asset["resolved_path"]), reference_path)
        while is_stage_loading():
            SIMULATION_APP.update()
        self.articulation_root = find_articulation_root(reference_path)
        self.robot = self.world.scene.add(
            SingleArticulation(
                self.articulation_root,
                name="stage0d_jackal",
                position=np.array([0.0, 0.0, self.spawn_height]),
            )
        )
        self.world.reset()
        self.wheels = discover_wheels(self.robot, self.articulation_root)
        self.differential = DifferentialController(
            wheel_radius=float(self.wheels["radius_m"]),
            wheel_base=float(self.wheels["separation_m"]),
        )
        self.articulation_controller = self.robot.get_articulation_controller()
        self.maximum_wheel_target = float(
            config["model_selection"]["maximum_abs_wheel_target_rad_s"]
        )

    def reset(self, pose: np.ndarray, settling_s: float) -> None:
        self.world.reset()
        self.robot.set_world_pose(
            position=np.array([pose[0], pose[1], self.spawn_height]),
            orientation=quaternion_from_yaw(float(pose[2])),
        )
        zero = np.zeros(self.robot.num_dof, dtype=np.float64)
        self.robot.set_joint_velocities(zero)
        self.articulation_controller.apply_action(ArticulationAction(joint_velocities=zero))
        for _ in range(int(round(settling_s / self.physics_dt))):
            self.world.step(render=False)

    def apply_body_command(
        self,
        desired: np.ndarray,
        mode: str,
        correction: JackalExecutionController | None,
        measured_omega: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool, dict[str, Any]]:
        ideal_side = np.asarray(self.differential.forward(desired), dtype=np.float64)
        if mode == "nominal":
            executed = desired.copy()
            side = ideal_side.copy()
            saturated = False
            state = {
                "omega_error_rps": float(desired[1] - measured_omega),
                "integral_error_rad": 0.0,
                "unsaturated_omega_rps": float(desired[1]),
            }
        elif mode == "calibrated":
            if correction is None:
                raise ValueError("calibrated mode requires an execution controller")
            output = correction.forward(
                desired_v_mps=float(desired[0]),
                desired_omega_rps=float(desired[1]),
                measured_omega_rps=float(measured_omega),
                control_dt_s=dt,
            )
            executed = np.asarray(
                [output.executed_v_mps, output.executed_omega_rps], dtype=np.float64
            )
            side = np.asarray(self.differential.forward(executed), dtype=np.float64)
            saturated = bool(output.saturated)
            state = {
                "omega_error_rps": output.omega_error_rps,
                "integral_error_rad": output.integral_error_rad,
                "unsaturated_omega_rps": output.unsaturated_omega_rps,
            }
            clipped = np.clip(side, -self.maximum_wheel_target, self.maximum_wheel_target)
            if not np.array_equal(clipped, side):
                saturated = True
                side = clipped
                radius = float(self.wheels["radius_m"])
                separation = float(self.wheels["separation_m"])
                executed = np.asarray(
                    [
                        radius * (side[0] + side[1]) / 2.0,
                        radius * (side[1] - side[0]) / separation,
                    ],
                    dtype=np.float64,
                )
        else:
            raise ValueError(f"unknown execution mode: {mode}")
        runtime_target = runtime_wheel_command(side, self.wheels)
        self.articulation_controller.apply_action(
            ArticulationAction(joint_velocities=runtime_target)
        )
        return ideal_side, executed, runtime_target, saturated, state


def correction_from_parameters(
    runtime: Runtime, config: Mapping[str, Any], parameters: Mapping[str, Any]
) -> JackalExecutionController:
    physical = float(runtime.wheels["separation_m"])
    return JackalExecutionController(
        JackalExecutionControllerConfig(
            physical_wheel_separation_m=physical,
            calibrated_effective_wheel_separation_m=float(
                parameters["calibrated_effective_wheel_separation_m"]
            ),
            yaw_rate_kp=float(parameters["yaw_rate_kp"]),
            yaw_rate_ki=float(parameters["yaw_rate_ki"]),
            maximum_abs_executed_omega_rps=float(
                config["model_selection"]["maximum_abs_executed_omega_rps"]
            ),
            sign_protection=bool(config["model_selection"]["sign_protection"]),
        )
    )


def primitive_telemetry(
    runtime: Runtime,
    config: Mapping[str, Any],
    *,
    desired_v: float,
    desired_omega: float,
    mode: str,
    parameters: Mapping[str, Any] | None,
) -> tuple[ExecutionTelemetry, list[dict[str, Any]]]:
    simulation = config["simulation"]
    runtime.reset(runtime.initial_pose, float(simulation["settling_duration_s"]))
    correction = (
        correction_from_parameters(runtime, config, parameters)
        if mode == "calibrated" and parameters is not None
        else None
    )
    phase_durations = (
        ("INITIAL_STOP", float(simulation["initial_stop_duration_s"])),
        ("ACTIVE", float(simulation["active_duration_s"])),
        ("FINAL_STOP", float(simulation["final_stop_duration_s"])),
    )
    poses = [se2_from_world_pose(runtime.robot)]
    times = [0.0]
    controls = [-1]
    phases = ["INITIAL_STOP"]
    desired_rows = [[0.0, 0.0]]
    executed_rows = [[0.0, 0.0]]
    ideal_rows = [[0.0, 0.0]]
    targets = [np.zeros(4, dtype=np.float64)]
    measured_wheels = [canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)]
    saturated_rows = [False]
    controller_states: list[dict[str, Any]] = []
    origin = float(runtime.world.current_time)
    measured_omega = 0.0
    previous_control_pose = poses[0]
    control_index = 0
    for phase, duration in phase_durations:
        count = int(round(duration / runtime.control_dt))
        if not math.isclose(count * runtime.control_dt, duration, abs_tol=1e-9):
            raise ValueError("phase duration must be divisible by control dt")
        for _ in range(count):
            desired = np.asarray(
                [desired_v, desired_omega] if phase == "ACTIVE" else [0.0, 0.0],
                dtype=np.float64,
            )
            ideal, executed, runtime_target, saturated, state = runtime.apply_body_command(
                desired,
                mode,
                correction,
                measured_omega,
                runtime.control_dt,
            )
            controller_states.append(
                {"control_index": control_index, "phase": phase, **state, "saturated": saturated}
            )
            for _ in range(runtime.physics_steps):
                runtime.world.step(render=False)
                poses.append(se2_from_world_pose(runtime.robot))
                times.append(float(runtime.world.current_time) - origin)
                controls.append(control_index)
                phases.append(phase)
                desired_rows.append(desired.copy())
                executed_rows.append(executed.copy())
                ideal_rows.append(ideal.copy())
                targets.append(canonical_wheel_values(runtime_target, runtime.wheels))
                measured_wheels.append(
                    canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
                )
                saturated_rows.append(saturated)
            current = poses[-1]
            measured_omega = float(
                wrap_angle(current[2] - previous_control_pose[2]) / runtime.control_dt
            )
            previous_control_pose = current
            control_index += 1
    runtime.articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(runtime.robot.num_dof, dtype=np.float64))
    )
    return (
        ExecutionTelemetry(
            np.asarray(poses),
            np.asarray(times),
            np.asarray(controls),
            tuple(phases),
            np.asarray(desired_rows),
            np.asarray(executed_rows),
            np.asarray(ideal_rows),
            np.asarray(targets),
            np.asarray(measured_wheels),
            np.asarray(saturated_rows),
        ),
        controller_states,
    )


def save_primitive_grid(
    runtime: Runtime,
    config: Mapping[str, Any],
    root: Path,
    *,
    subset: str,
    mode: str,
    conditions: Sequence[tuple[float, float]],
    repetitions: int,
    parameters: Mapping[str, Any] | None,
    candidate_id: str | None = None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    omega_floor = float(
        config["model_selection"]["effective_track_width_assessment"][
            "minimum_abs_measured_omega_rps"
        ]
    )
    for v, omega in conditions:
        reps = 1 if v == 0.0 and omega == 0.0 else repetitions
        for repetition in range(reps):
            telemetry, controller_states = primitive_telemetry(
                runtime,
                config,
                desired_v=v,
                desired_omega=omega,
                mode=mode,
                parameters=parameters,
            )
            metrics = compute_trial_metrics(
                telemetry,
                wheel_radius_m=float(runtime.wheels["radius_m"]),
                steady_state_tail_s=float(config["simulation"]["steady_state_tail_s"]),
                minimum_abs_measured_omega_rps=omega_floor,
            )
            identifier = condition_id(v, omega)
            destination = root / subset / mode
            if candidate_id is not None:
                destination = destination / candidate_id
            destination = destination / identifier / f"repetition_{repetition:02d}"
            metadata = {
                "stage": config["stage"],
                "subset": subset,
                "mode": mode,
                "candidate_id": candidate_id,
                "condition_id": identifier,
                "repetition": repetition,
                "desired_body_command": [v, omega],
                "coordinate_frame": (
                    "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]; body v forward, omega CCW"
                ),
                "timing": {
                    "observation": "physics-step pose/joint observation at each saved row",
                    "controller_ready": "synchronous command computation before ending interval",
                    "execution": "command held over physics interval ending at saved row",
                    "physics_dt_s": runtime.physics_dt,
                    "control_dt_s": runtime.control_dt,
                },
                "controller_parameters": dict(parameters or {}),
                "physical_wheel_separation_m": float(runtime.wheels["separation_m"]),
                "physical_wheel_radius_m": float(runtime.wheels["radius_m"]),
                "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
                "runtime_wheel_dof_names": list(canonical_wheel_names(runtime.wheels)),
                "runtime_physics_overrides": {},
                "controller_states": controller_states,
                "research_evidence": False,
            }
            hashes = save_execution_trial(destination, telemetry, metrics, metadata)
            validation = validate_execution_trial(destination)
            row = {
                "subset": subset,
                "mode": mode,
                "candidate_id": candidate_id,
                "condition_id": identifier,
                "repetition": repetition,
                "trial_directory": str(destination.resolve()),
                "raw_sha256": hashes,
                **metrics,
            }
            rows.append(row)
            print(
                "STAGE0D_TRIAL="
                + json.dumps(
                    {
                        "subset": subset,
                        "mode": mode,
                        "candidate": candidate_id,
                        "condition": identifier,
                        "repetition": repetition,
                        "omega_rmse": metrics["omega_rmse_rps"],
                        "wheel_rmse": metrics["wheel_target_measured_rmse_rad_s"],
                        "valid": validation["valid"],
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
    return rows


def run_composite_mode(
    runtime: Runtime,
    config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    root: Path,
    *,
    mode: str,
    parameters: Mapping[str, Any] | None,
) -> dict[str, Any]:
    segments = segments_from_config(stage0b["composite_motion_profile"])
    reference = generate_reference_trajectory(runtime.initial_pose, segments, runtime.control_dt)
    reference_path = root / "composite_validation" / "reference_trajectory.npy"
    reference_path.parent.mkdir(parents=True, exist_ok=True)
    if not reference_path.exists():
        np.save(reference_path, reference.poses)
    follower = TrajectoryFollower(reference.poses, follower_config(stage0b))
    runtime.reset(runtime.initial_pose, float(config["simulation"]["settling_duration_s"]))
    correction = (
        correction_from_parameters(runtime, config, parameters)
        if mode == "calibrated" and parameters is not None
        else None
    )
    pose = se2_from_world_pose(runtime.robot)
    next_command = follower.forward(pose)
    poses = [pose]
    times = [0.0]
    controls = [-1]
    phases = ["INITIAL_STOP"]
    desired_rows = [[0.0, 0.0]]
    executed_rows = [[0.0, 0.0]]
    ideal_rows = [[0.0, 0.0]]
    targets = [np.zeros(4)]
    measured_wheels = [canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)]
    saturated_rows = [False]
    progress = [next_command.nearest_index]
    origin = float(runtime.world.current_time)
    measured_omega = 0.0
    maximum_controls = int(
        round(float(stage0b["closed_loop"]["maximum_duration_s"]) / runtime.control_dt)
    )
    for control_index in range(maximum_controls):
        if next_command.goal_reached:
            break
        desired = np.asarray(
            [next_command.linear_velocity_mps, next_command.angular_velocity_rps],
            dtype=np.float64,
        )
        ideal, executed, runtime_target, saturated, _ = runtime.apply_body_command(
            desired, mode, correction, measured_omega, runtime.control_dt
        )
        previous = poses[-1]
        for _ in range(runtime.physics_steps):
            runtime.world.step(render=False)
            poses.append(se2_from_world_pose(runtime.robot))
            times.append(float(runtime.world.current_time) - origin)
            controls.append(control_index)
            phases.append("ACTIVE")
            desired_rows.append(desired.copy())
            executed_rows.append(executed.copy())
            ideal_rows.append(ideal.copy())
            targets.append(canonical_wheel_values(runtime_target, runtime.wheels))
            measured_wheels.append(
                canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
            )
            saturated_rows.append(saturated)
            progress.append(next_command.nearest_index)
        pose = poses[-1]
        measured_omega = float(wrap_angle(pose[2] - previous[2]) / runtime.control_dt)
        next_command = follower.forward(pose)
    runtime.articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(runtime.robot.num_dof))
    )
    telemetry = ExecutionTelemetry(
        np.asarray(poses),
        np.asarray(times),
        np.asarray(controls),
        tuple(phases),
        np.asarray(desired_rows),
        np.asarray(executed_rows),
        np.asarray(ideal_rows),
        np.asarray(targets),
        np.asarray(measured_wheels),
        np.asarray(saturated_rows),
    )
    controller_telemetry = ControllerTelemetry(
        reference.poses,
        telemetry.actual_trajectory,
        telemetry.sim_times_s,
        telemetry.desired_body,
        telemetry.target_wheels,
        telemetry.measured_wheels,
        np.asarray(progress),
        canonical_wheel_names(runtime.wheels),
    )
    metrics = compute_controller_metrics(controller_telemetry)
    measured_v, measured_body_omega = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    desired_error = telemetry.desired_body - np.column_stack((measured_v, measured_body_omega))
    metrics.update(
        {
            "goal_reached": bool(next_command.goal_reached),
            "active_saturation_fraction": float(np.mean(telemetry.saturated[1:])),
            "desired_v_rmse_mps": float(np.sqrt(np.mean(desired_error[1:, 0] ** 2))),
            "desired_omega_rmse_rps": float(np.sqrt(np.mean(desired_error[1:, 1] ** 2))),
        }
    )
    destination = root / "composite_validation" / mode
    metadata = {
        "stage": config["stage"],
        "subset": "composite_validation",
        "mode": mode,
        "condition_id": "stage0b_composite",
        "repetition": 0,
        "controller_parameters": dict(parameters or {}),
        "reference_path": str(reference_path.resolve()),
        "reference_sha256": sha256_file(reference_path),
        "coordinate_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
        "timing": {
            "observation": "physics-step pose observation",
            "controller_ready": "TrajectoryFollower and execution correction computed synchronously",
            "execution": "command applied over ending physics interval",
        },
        "runtime_physics_overrides": {},
    }
    hashes = save_execution_trial(destination, telemetry, metrics, metadata)
    validate_execution_trial(destination)
    return {"mode": mode, "trial_directory": str(destination), "raw_sha256": hashes, **metrics}


def run_exp02b_replay_mode(
    runtime: Runtime,
    config: Mapping[str, Any],
    root: Path,
    *,
    case_id: str,
    source,
    mode: str,
    parameters: Mapping[str, Any] | None,
) -> dict[str, Any]:
    simulation = config["simulation"]
    initial = np.asarray(source.source_metadata["initial_pose_se2"], dtype=np.float64)
    runtime.reset(initial, float(simulation["settling_duration_s"]))
    correction = (
        correction_from_parameters(runtime, config, parameters)
        if mode == "calibrated" and parameters is not None
        else None
    )
    rows = source_old_rows(source.trial_directory)
    with (source.trial_directory / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        all_rows = list(csv.DictReader(stream))
    fresh_first = next(row for row in all_rows if row["reference_source"] == "FRESH")
    poses = [se2_from_world_pose(runtime.robot)]
    times = [0.0]
    controls = [-1]
    phases = ["INITIAL_STOP"]
    desired_rows = [[0.0, 0.0]]
    executed_rows = [[0.0, 0.0]]
    ideal_rows = [[0.0, 0.0]]
    targets = [np.zeros(4)]
    measured_wheels = [canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)]
    saturated_rows = [False]
    origin = float(runtime.world.current_time)
    measured_omega = 0.0
    for index, row in enumerate(rows):
        next_time = (
            float(rows[index + 1]["sim_time_s"])
            if index + 1 < len(rows)
            else float(fresh_first["sim_time_s"])
        )
        duration = next_time - float(row["sim_time_s"])
        steps = int(round(duration / runtime.physics_dt))
        if steps < 1 or not math.isclose(steps * runtime.physics_dt, duration, abs_tol=1e-6):
            raise ValueError("source OLD command interval is not physics-step aligned")
        desired = np.asarray(
            [float(row["v_command_mps"]), float(row["omega_command_rps"])], dtype=np.float64
        )
        ideal, executed, runtime_target, saturated, _ = runtime.apply_body_command(
            desired, mode, correction, measured_omega, duration
        )
        previous = poses[-1]
        for _ in range(steps):
            runtime.world.step(render=False)
            poses.append(se2_from_world_pose(runtime.robot))
            times.append(float(runtime.world.current_time) - origin)
            controls.append(index)
            phases.append("ACTIVE")
            desired_rows.append(desired.copy())
            executed_rows.append(executed.copy())
            ideal_rows.append(ideal.copy())
            targets.append(canonical_wheel_values(runtime_target, runtime.wheels))
            measured_wheels.append(
                canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
            )
            saturated_rows.append(saturated)
        measured_omega = float(wrap_angle(poses[-1][2] - previous[2]) / duration)
    runtime.articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(runtime.robot.num_dof))
    )
    telemetry = ExecutionTelemetry(
        np.asarray(poses),
        np.asarray(times),
        np.asarray(controls),
        tuple(phases),
        np.asarray(desired_rows),
        np.asarray(executed_rows),
        np.asarray(ideal_rows),
        np.asarray(targets),
        np.asarray(measured_wheels),
        np.asarray(saturated_rows),
    )
    diagnostic = DiagnosticTelemetry(
        telemetry.actual_trajectory,
        telemetry.sim_times_s,
        telemetry.desired_body,
        telemetry.target_wheels,
        telemetry.measured_wheels,
    )
    execution = execution_tracking_metrics(diagnostic)
    spatial = spatial_reference_metrics(source.old_world, telemetry.actual_trajectory)
    reproduced = telemetry.actual_trajectory[-1]
    metrics = {
        "old_reference_spatial_metrics": spatial,
        "body_and_wheel_execution_metrics": execution,
        "saved_boundary_se2": source.boundary_pose_world.tolist(),
        "reproduced_boundary_se2": reproduced.tolist(),
        "boundary_translation_error_m": float(
            np.linalg.norm(reproduced[:2] - source.boundary_pose_world[:2])
        ),
        "boundary_yaw_error_rad": abs(
            float(wrap_angle(reproduced[2] - source.boundary_pose_world[2]))
        ),
        "active_saturation_fraction": float(np.mean(telemetry.saturated[1:])),
    }
    destination = root / "exp02b_replay_validation" / case_id / mode
    source_timing = source.source_attempt["timing"]
    metadata = {
        "stage": config["stage"],
        "subset": "exp02b_replay_validation",
        "mode": mode,
        "condition_id": case_id,
        "repetition": 0,
        "controller_parameters": dict(parameters or {}),
        "source_trial_path": str(source.trial_directory),
        "source_hashes": {
            "old_world": sha256_file(source.trial_directory / "derived/old_world.npy"),
            "controller_commands": sha256_file(
                source.trial_directory / "derived/controller_commands.csv"
            ),
        },
        "source_observation_readiness_execution_timestamps": {
            "observation_sim_time_s": source_timing["observation_sim_time_s"],
            "model_ready_sim_time_s": source_timing["model_ready_sim_time_s"],
            "fresh_usable_sim_time_s": source_timing["fresh_usable_sim_time_s"],
        },
        "coordinate_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
        "timing": "saved OLD desired body commands replayed at source simulation intervals",
        "source_commands_modified": False,
        "runtime_physics_overrides": {},
    }
    hashes = save_execution_trial(destination, telemetry, metrics, metadata)
    validate_execution_trial(destination)
    return {
        "case_id": case_id,
        "mode": mode,
        "trial_directory": str(destination),
        "raw_sha256": hashes,
        **metrics,
    }


def main() -> None:
    config_path = ARGS.config.resolve()
    config = load_yaml(config_path)
    config_validation = validate_execution_calibration_config(config)
    stage0b_path = resolve_path(config["paths"]["stage0b_config"])
    exp02b_config_path = resolve_path(config["paths"]["exp02b_config"])
    stage0b = load_yaml(stage0b_path)
    exp02b_config = load_yaml(exp02b_config_path)
    calibration_conditions = grid_conditions(config["characterization"], smoke=ARGS.smoke)
    held_out_conditions = grid_conditions(config["held_out_validation"], smoke=ARGS.smoke)
    if not ARGS.smoke:
        assert_disjoint_condition_sets(calibration_conditions, held_out_conditions)
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("stage0d-%Y%m%dT%H%M%SZ")
    root = resolve_path(config["paths"]["output_root"]) / run_id
    root.mkdir(parents=True, exist_ok=False)
    with (root / "config_snapshot.yaml").open("x", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    write_json_exclusive(root / "acceptance_criteria.json", config["acceptance_criteria"])
    write_json_exclusive(root / "config_validation.json", config_validation)
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    runtime = Runtime(config, stage0b)

    characterization = save_primitive_grid(
        runtime,
        config,
        root,
        subset="characterization",
        mode="nominal",
        conditions=calibration_conditions,
        repetitions=(1 if ARGS.smoke else int(config["characterization"]["repetitions"])),
        parameters=None,
    )
    characterization_summary = {
        "stage": config["stage"],
        "smoke": ARGS.smoke,
        "trial_count": len(characterization),
        "conditions": summarize_conditions(characterization),
    }
    characterization_summary["response_dependence"] = response_dependence(
        characterization_summary["conditions"]
    )
    write_json_exclusive(root / "characterization_summary.json", characterization_summary)
    assessment_config = config["model_selection"]["effective_track_width_assessment"]
    feedforward = fit_effective_width_model(
        characterization,
        physical_wheel_separation_m=float(runtime.wheels["separation_m"]),
        maximum_feedforward_scale=float(assessment_config["maximum_feedforward_scale"]),
        assessment=assessment_config,
    )
    write_json_exclusive(root / "calibration" / "feedforward_assessment.json", feedforward)
    base_parameters = {
        "physical_wheel_separation_m": float(runtime.wheels["separation_m"]),
        "calibrated_effective_wheel_separation_m": float(
            feedforward["calibrated_effective_wheel_separation_m"]
        ),
        "feedforward_scale": float(feedforward["applied_feedforward_scale"]),
        "yaw_rate_kp": 0.0,
        "yaw_rate_ki": 0.0,
    }
    feedback_selection = None
    candidate_results: list[dict[str, Any]] = []
    if feedforward["sufficient"]:
        selected_parameters = base_parameters
        selected_method = "effective_track_width_feedforward"
        calibration_passed = True
    else:
        for candidate in config["model_selection"]["feedback_candidates"]:
            parameters = {
                **base_parameters,
                "yaw_rate_kp": float(candidate["yaw_rate_kp"]),
                "yaw_rate_ki": float(candidate["yaw_rate_ki"]),
            }
            trials = save_primitive_grid(
                runtime,
                config,
                root,
                subset="calibration/feedback_candidates",
                mode="calibrated",
                conditions=calibration_conditions,
                repetitions=(1 if ARGS.smoke else int(config["characterization"]["repetitions"])),
                parameters=parameters,
                candidate_id=str(candidate["id"]),
            )
            candidate_results.append(
                {
                    "candidate_id": str(candidate["id"]),
                    "parameters": parameters,
                    "trials": trials,
                }
            )
        feedback_selection = select_feedback_candidate(
            candidate_results, config["acceptance_criteria"]
        )
        selected_id = feedback_selection["selected_candidate_id"]
        selected_parameters = next(
            row["parameters"] for row in candidate_results if row["candidate_id"] == selected_id
        )
        selected_method = "effective_track_width_feedforward_plus_pi_yaw_rate_feedback"
        calibration_passed = bool(
            feedback_selection["selected_passed_calibration_criteria"]
        )
        write_json_exclusive(
            root / "calibration" / "feedback_selection.json", feedback_selection
        )

    calibration_hashes = {
        row["trial_directory"]: row["raw_sha256"] for row in characterization
    }
    feedback_hashes = {
        row["trial_directory"]: row["raw_sha256"]
        for candidate in candidate_results
        for row in candidate["trials"]
    }
    fitting_code = REPOSITORY_ROOT / "src/reconciliation/execution_calibration.py"
    controller_code = (
        REPOSITORY_ROOT
        / "src/reconciliation/controllers/jackal_execution_controller.py"
    )
    model = {
        "stage": config["stage"],
        "model_type": selected_method,
        "selected_parameters": selected_parameters,
        "physical_wheel_radius_m": float(runtime.wheels["radius_m"]),
        "physical_wheel_separation_m": float(runtime.wheels["separation_m"]),
        "calibrated_effective_wheel_separation_m": float(
            selected_parameters["calibrated_effective_wheel_separation_m"]
        ),
        "feedforward_assessment": feedforward,
        "feedback_selection": feedback_selection,
        "calibration_passed_calibration_grid": calibration_passed,
        "calibration_input_trial_ids": [row["trial_directory"] for row in characterization],
        "calibration_input_raw_hashes": calibration_hashes,
        "feedback_candidate_input_trial_ids": list(feedback_hashes),
        "feedback_candidate_input_raw_hashes": feedback_hashes,
        "calibration_conditions": [list(value) for value in calibration_conditions],
        "held_out_conditions": [list(value) for value in held_out_conditions],
        "held_out_data_used_for_fitting_or_tuning": False,
        "fitting_method": (
            "through-origin nominal yaw-response gain, then first predeclared PI candidate "
            "passing calibration criteria if the single-width assessment fails"
        ),
        "parameter_fitting_code": {
            "path": str(fitting_code.relative_to(REPOSITORY_ROOT)),
            "sha256": sha256_file(fitting_code),
        },
        "runtime_controller_code": {
            "path": str(controller_code.relative_to(REPOSITORY_ROOT)),
            "sha256": sha256_file(controller_code),
        },
        "runtime_controller_config": dict(config["model_selection"]),
        "isaac_sim_version": isaac_version(),
        "jackal_asset": runtime.asset,
        "wheel_runtime": runtime.wheels,
        "runtime_physics_overrides": {},
    }
    write_json_exclusive(root / "calibration_model.json", model)
    write_json_exclusive(
        root / "calibration_summary.json",
        {
            "option_a_effective_width_assessment": feedforward,
            "feedback_evaluated_only_after_option_a_failed": not bool(
                feedforward["sufficient"]
            ),
            "feedback_selection": feedback_selection,
            "selected_method": selected_method,
            "selected_parameters": selected_parameters,
            "calibration_grid_passed": calibration_passed,
            "held_out_data_used_for_fitting_or_tuning": False,
        },
    )

    held_out_rows: list[dict[str, Any]] = []
    if not ARGS.smoke:
        for mode, parameters in (("nominal", None), ("calibrated", selected_parameters)):
            held_out_rows.extend(
                save_primitive_grid(
                    runtime,
                    config,
                    root,
                    subset="held_out_validation",
                    mode=mode,
                    conditions=held_out_conditions,
                    repetitions=int(config["held_out_validation"]["repetitions"]),
                    parameters=parameters,
                )
            )
        held_out_summary = {
            "nominal": evaluate_tracking_acceptance(
                [row for row in held_out_rows if row["mode"] == "nominal"],
                config["acceptance_criteria"],
            ),
            "calibrated": evaluate_tracking_acceptance(
                [row for row in held_out_rows if row["mode"] == "calibrated"],
                config["acceptance_criteria"],
            ),
            "paired_conditions": summarize_conditions(held_out_rows),
            "response_dependence": response_dependence(summarize_conditions(held_out_rows)),
            "held_out_data_used_for_fitting_or_tuning": False,
        }
        write_json_exclusive(root / "held_out_summary.json", held_out_summary)

        composite_rows = [
            run_composite_mode(
                runtime, config, stage0b, root, mode="nominal", parameters=None
            ),
            run_composite_mode(
                runtime,
                config,
                stage0b,
                root,
                mode="calibrated",
                parameters=selected_parameters,
            ),
        ]
        write_json_exclusive(
            root / "composite_summary.json", {"runs": composite_rows}
        )

        exp02b_run = resolve_path(config["paths"]["exp02b_run"])
        source_selection = load_strict_json(exp02b_run / "source_selection.json")
        verify_frozen_source_selection(source_selection["source_root"], exp02b_config)
        replay_rows = []
        for case_id in config["exp02b_replay_validation"]["cases"]:
            source = load_source_case(
                case_id, source_selection["cases"][case_id]["trial_directory"]
            )
            for mode, parameters in (("nominal", None), ("calibrated", selected_parameters)):
                replay_rows.append(
                    run_exp02b_replay_mode(
                        runtime,
                        config,
                        root,
                        case_id=case_id,
                        source=source,
                        mode=mode,
                        parameters=parameters,
                    )
                )
        write_json_exclusive(
            root / "exp02b_replay_summary.json", {"runs": replay_rows}
        )

    metadata = {
        "stage": config["stage"],
        "run_id": run_id,
        "smoke": ARGS.smoke,
        "created_at": started_at,
        "completed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit_sha": git_sha(),
        "config_source": str(config_path),
        "config_source_sha256": sha256_file(config_path),
        "config_snapshot_sha256": sha256_file(root / "config_snapshot.yaml"),
        "acceptance_criteria_frozen_before_results": True,
        "coordinate_frame": "Isaac Sim world [x_m,y_m,yaw_CCW_about_+Z_rad]",
        "observation_readiness_execution_timing": (
            "pose/joints observed at physics rows; correction and wheel targets computed "
            "synchronously at control boundaries; command associated with ending interval"
        ),
        "isaac_sim_version": isaac_version(),
        "jackal_asset": runtime.asset,
        "articulation_prim_path": runtime.articulation_root,
        "wheel_runtime": runtime.wheels,
        "runtime_physics_overrides": {},
        "frozen_reconciliation_outputs_modified": False,
        "research_evidence": False,
    }
    write_json_exclusive(root / "metadata.json", metadata)
    print("STAGE0D_RUN=" + json.dumps({"run_directory": str(root), "model": model}, sort_keys=True))


try:
    main()
finally:
    SIMULATION_APP.close()
