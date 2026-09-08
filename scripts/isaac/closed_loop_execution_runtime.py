"""Shared Isaac runtime for Stage 0-E headless and GUI execution."""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Mapping

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from pxr import UsdLux

from lightnav_stage0c_runtime import (
    canonical_wheel_values,
    discover_wheels,
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.closed_loop_execution_validation import ClosedLoopTelemetry
from reconciliation.controllers.jackal_execution_controller import (
    JackalExecutionController,
    JackalExecutionControllerConfig,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.se2 import wrap_angle


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


class ClosedLoopRuntime:
    def __init__(
        self,
        simulation_app,
        config: Mapping[str, Any],
        stage0b: Mapping[str, Any],
        candidate: Mapping[str, Any],
        *,
        render: bool,
        real_time_factor: float | None = None,
    ) -> None:
        simulation = config["simulation"]
        self.simulation_app = simulation_app
        self.physics_dt = float(simulation["physics_dt_s"])
        self.control_dt = float(simulation["control_dt_s"])
        self.physics_steps = int(round(self.control_dt / self.physics_dt))
        self.spawn_height = float(simulation["spawn_height_m"])
        self.render = bool(render)
        self.real_time_factor = real_time_factor
        if real_time_factor is not None and (
            not math.isfinite(real_time_factor) or real_time_factor <= 0.0
        ):
            raise ValueError("real_time_factor must be finite and positive")
        self.parameters = dict(candidate["selected_parameters"])
        self.controller_config = dict(candidate["runtime_controller_config"])
        self.maximum_wheel_target = float(
            self.controller_config["maximum_abs_wheel_target_rad_s"]
        )
        self.asset = resolve_jackal_asset(stage0b["robot"])
        self.world = World(
            physics_dt=self.physics_dt,
            rendering_dt=self.physics_dt,
            stage_units_in_meters=1.0,
        )
        self.world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, "/World/Stage0ELight")
        dome.CreateIntensityAttr(1000.0)
        reference_path = str(stage0b["robot"]["reference_prim_path"])
        add_reference_to_stage(str(self.asset["resolved_path"]), reference_path)
        while is_stage_loading():
            simulation_app.update()
        self.articulation_root = find_articulation_root(reference_path)
        self.robot = self.world.scene.add(
            SingleArticulation(
                self.articulation_root,
                name="stage0e_jackal",
                position=np.array([0.0, 0.0, self.spawn_height]),
            )
        )
        self.world.reset()
        self.wheels = discover_wheels(self.robot, self.articulation_root)
        self.differential = DifferentialController(
            wheel_radius=float(self.wheels["radius_m"]),
            wheel_base=float(self.wheels["separation_m"]),
        )
        self.articulation = self.robot.get_articulation_controller()

    def step(self) -> None:
        if not self.simulation_app.is_running():
            raise RuntimeError("Isaac Sim closed before Stage 0-E execution completed")
        started = time.monotonic()
        self.world.step(render=self.render)
        if self.real_time_factor is not None:
            remaining = self.physics_dt / self.real_time_factor - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)

    def reset(self, pose: np.ndarray, settling_s: float) -> None:
        """Reset world, pose, all joint rates, command, and low-level state."""

        self.world.reset()
        self.robot.set_world_pose(
            position=np.array([pose[0], pose[1], self.spawn_height]),
            orientation=quaternion_from_yaw(float(pose[2])),
        )
        zero = np.zeros(self.robot.num_dof, dtype=np.float64)
        self.robot.set_joint_velocities(zero)
        self.articulation.apply_action(ArticulationAction(joint_velocities=zero))
        for _ in range(int(round(settling_s / self.physics_dt))):
            self.step()

    def correction(self) -> JackalExecutionController:
        return JackalExecutionController(
            JackalExecutionControllerConfig(
                physical_wheel_separation_m=float(self.wheels["separation_m"]),
                calibrated_effective_wheel_separation_m=float(
                    self.parameters["calibrated_effective_wheel_separation_m"]
                ),
                yaw_rate_kp=float(self.parameters["yaw_rate_kp"]),
                yaw_rate_ki=float(self.parameters["yaw_rate_ki"]),
                maximum_abs_executed_omega_rps=float(
                    self.controller_config["maximum_abs_executed_omega_rps"]
                ),
                sign_protection=bool(self.controller_config["sign_protection"]),
            )
        )

    def apply(
        self,
        desired: np.ndarray,
        mode: str,
        correction: JackalExecutionController | None,
        measured_omega: float,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, float | bool]]:
        pi_correction = 0.0
        integral = 0.0
        sign_event = False
        if mode == "nominal":
            executed = desired.copy()
            saturated = False
        elif mode == "calibrated":
            if correction is None:
                raise ValueError("calibrated execution requires the frozen correction")
            output = correction.forward(
                desired_v_mps=float(desired[0]),
                desired_omega_rps=float(desired[1]),
                measured_omega_rps=float(measured_omega),
                control_dt_s=self.control_dt,
            )
            executed = np.asarray(
                [output.executed_v_mps, output.executed_omega_rps], dtype=np.float64
            )
            saturated = bool(output.saturated)
            integral = float(output.integral_error_rad)
            pi_correction = float(
                output.unsaturated_omega_rps
                - correction.config.feedforward_scale * float(desired[1])
            )
            sign_event = bool(
                abs(float(desired[1])) > 0.0
                and output.unsaturated_omega_rps * float(desired[1]) < 0.0
                and output.executed_omega_rps == 0.0
            )
        else:
            raise ValueError(f"unknown execution mode: {mode}")
        side = np.asarray(self.differential.forward(executed), dtype=np.float64)
        clipped = np.clip(side, -self.maximum_wheel_target, self.maximum_wheel_target)
        if not np.array_equal(side, clipped):
            saturated = True
            radius = float(self.wheels["radius_m"])
            separation = float(self.wheels["separation_m"])
            executed = np.asarray(
                [
                    radius * (clipped[0] + clipped[1]) / 2.0,
                    radius * (clipped[1] - clipped[0]) / separation,
                ],
                dtype=np.float64,
            )
        runtime_target = runtime_wheel_command(clipped, self.wheels)
        self.articulation.apply_action(
            ArticulationAction(joint_velocities=runtime_target)
        )
        return executed, runtime_target, {
            "pi_correction_rps": pi_correction,
            "integral_error_rad": integral,
            "saturated": saturated,
            "sign_protection_event": sign_event,
        }

    def stop(self) -> None:
        self.articulation.apply_action(
            ArticulationAction(joint_velocities=np.zeros(self.robot.num_dof))
        )


def run_closed_loop(
    runtime: ClosedLoopRuntime,
    config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    reference: np.ndarray,
    initial_pose: np.ndarray,
    mode: str,
    *,
    on_control: Callable[[dict[str, Any], list[np.ndarray]], None] | None = None,
) -> ClosedLoopTelemetry:
    """Execute one reference with fresh actual-pose follower feedback each step."""

    runtime.reset(initial_pose, float(config["simulation"]["settling_duration_s"]))
    follower = TrajectoryFollower(reference, follower_config(stage0b))
    correction = runtime.correction() if mode == "calibrated" else None
    current = se2_from_world_pose(runtime.robot)
    command = follower.forward(current)
    poses = [current]
    times = [0.0]
    controls = [-1]
    nearest = [command.nearest_index]
    targets_index = [command.target_index]
    goals = [command.goal_reached]
    desired_rows = [np.zeros(2)]
    executed_rows = [np.zeros(2)]
    target_wheels = [np.zeros(4)]
    measured_wheels = [
        canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
    ]
    pi_rows = [0.0]
    integral_rows = [0.0]
    saturated_rows = [False]
    sign_rows = [False]
    origin = float(runtime.world.current_time)
    measured_body = np.zeros(2, dtype=np.float64)
    maximum_controls = int(
        round(float(stage0b["closed_loop"]["maximum_duration_s"]) / runtime.control_dt)
    )
    for control_index in range(maximum_controls):
        if command.goal_reached:
            break
        desired = np.asarray(
            [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
        )
        executed, runtime_target, state = runtime.apply(
            desired, mode, correction, float(measured_body[1])
        )
        previous_control_pose = poses[-1]
        canonical_target = canonical_wheel_values(runtime_target, runtime.wheels)
        for _ in range(runtime.physics_steps):
            runtime.step()
            poses.append(se2_from_world_pose(runtime.robot))
            times.append(float(runtime.world.current_time) - origin)
            controls.append(control_index)
            nearest.append(command.nearest_index)
            targets_index.append(command.target_index)
            goals.append(False)
            desired_rows.append(desired.copy())
            executed_rows.append(executed.copy())
            target_wheels.append(canonical_target.copy())
            measured_wheels.append(
                canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
            )
            pi_rows.append(float(state["pi_correction_rps"]))
            integral_rows.append(float(state["integral_error_rad"]))
            saturated_rows.append(bool(state["saturated"]))
            sign_rows.append(bool(state["sign_protection_event"]))
        current = poses[-1]
        delta = current[:2] - previous_control_pose[:2]
        forward = np.array(
            [math.cos(float(previous_control_pose[2])), math.sin(float(previous_control_pose[2]))]
        )
        measured_body = np.asarray(
            [
                float(delta @ forward / runtime.control_dt),
                float(
                    wrap_angle(current[2] - previous_control_pose[2]) / runtime.control_dt
                ),
            ]
        )
        command = follower.forward(current)
        nearest[-1] = command.nearest_index
        targets_index[-1] = command.target_index
        goals[-1] = command.goal_reached
        if on_control is not None:
            on_control(
                {
                    "control_index": control_index,
                    "mode": mode,
                    "actual": current.tolist(),
                    "nearest_index": command.nearest_index,
                    "target_index": command.target_index,
                    "goal_reached": command.goal_reached,
                    "desired": desired.tolist(),
                    "executed": executed.tolist(),
                    "measured": measured_body.tolist(),
                    "target_wheels": canonical_target.tolist(),
                    "measured_wheels": measured_wheels[-1].tolist(),
                    **state,
                },
                poses,
            )
    runtime.stop()
    return ClosedLoopTelemetry(
        reference,
        np.asarray(poses),
        np.asarray(times),
        np.asarray(controls),
        np.asarray(nearest),
        np.asarray(targets_index),
        np.asarray(goals),
        np.asarray(desired_rows),
        np.asarray(executed_rows),
        np.asarray(target_wheels),
        np.asarray(measured_wheels),
        np.asarray(pi_rows),
        np.asarray(integral_rows),
        np.asarray(saturated_rows),
        np.asarray(sign_rows),
    )
