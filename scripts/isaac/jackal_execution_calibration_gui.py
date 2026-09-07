#!/usr/bin/env python3
"""GUI qualitative replay for a completed Stage 0-D calibration run."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_run", type=Path)
    parser.add_argument(
        "--suite", choices=("primitive", "composite", "exp02b"), default="primitive"
    )
    parser.add_argument(
        "--scenario",
        choices=("all", "straight", "left_turn", "right_turn", "left_arc", "right_arc"),
        default="all",
    )
    parser.add_argument("--case", default="case_high_delta_omega")
    parser.add_argument(
        "--k",
        type=int,
        default=3,
        help="frozen EXP-02B diagnostic context (OLD replay itself is k-independent)",
    )
    parser.add_argument(
        "--method",
        choices=("raw_k",),
        default="raw_k",
        help="frozen EXP-02B diagnostic context; Stage 0-D does not compare methods",
    )
    parser.add_argument("--real-time-factor", type=float)
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true", help="automation-only render check")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.usd
import yaml
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.util.debug_draw import _debug_draw
from pxr import UsdLux
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    canonical_wheel_values,
    discover_wheels,
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.controllers.jackal_execution_controller import (
    JackalExecutionController,
    JackalExecutionControllerConfig,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02b import load_source_case, verify_frozen_source_selection
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import generate_reference_trajectory, segments_from_config


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"{path} must contain a mapping")
    return result


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


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


class GuiRuntime:
    def __init__(self, config: Mapping[str, Any], stage0b: Mapping[str, Any]) -> None:
        simulation = config["simulation"]
        self.physics_dt = float(simulation["physics_dt_s"])
        self.control_dt = float(simulation["control_dt_s"])
        self.physics_steps = int(round(self.control_dt / self.physics_dt))
        self.spawn_height = float(simulation["spawn_height_m"])
        self.initial_pose = np.asarray(simulation["initial_pose_se2"], dtype=np.float64)
        self.real_time_factor = float(
            ARGS.real_time_factor
            if ARGS.real_time_factor is not None
            else config["gui"]["real_time_factor"]
        )
        if not math.isfinite(self.real_time_factor) or self.real_time_factor <= 0.0:
            raise ValueError("real-time factor must be finite and positive")
        asset = resolve_jackal_asset(stage0b["robot"])
        self.asset = asset
        self.world = World(
            physics_dt=self.physics_dt,
            rendering_dt=self.physics_dt,
            stage_units_in_meters=1.0,
        )
        self.world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, "/World/Stage0DGuiLight")
        dome.CreateIntensityAttr(1000.0)
        reference_path = str(stage0b["robot"]["reference_prim_path"])
        add_reference_to_stage(str(asset["resolved_path"]), reference_path)
        while is_stage_loading():
            SIMULATION_APP.update()
        self.articulation_root = find_articulation_root(reference_path)
        self.robot = self.world.scene.add(
            SingleArticulation(
                self.articulation_root,
                name="stage0d_gui_jackal",
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
        self.maximum_wheel_target = float(
            config["model_selection"]["maximum_abs_wheel_target_rad_s"]
        )

    def paced_step(self) -> None:
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed before GUI validation completed")
        started = time.monotonic()
        self.world.step(render=not ARGS.headless)
        remaining = self.physics_dt / self.real_time_factor - (time.monotonic() - started)
        if remaining > 0.0:
            time.sleep(remaining)

    def reset(self, pose: np.ndarray, settling_s: float) -> None:
        self.world.reset()
        self.robot.set_world_pose(
            position=np.array([pose[0], pose[1], self.spawn_height]),
            orientation=quaternion_from_yaw(float(pose[2])),
        )
        zero = np.zeros(self.robot.num_dof, dtype=np.float64)
        self.robot.set_joint_velocities(zero)
        self.articulation.apply_action(ArticulationAction(joint_velocities=zero))
        for _ in range(int(round(settling_s / self.physics_dt))):
            self.paced_step()

    def correction(
        self, parameters: Mapping[str, Any], config: Mapping[str, Any]
    ) -> JackalExecutionController:
        return JackalExecutionController(
            JackalExecutionControllerConfig(
                physical_wheel_separation_m=float(self.wheels["separation_m"]),
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

    def apply(
        self,
        desired: np.ndarray,
        mode: str,
        correction: JackalExecutionController | None,
        measured_omega: float,
        config: Mapping[str, Any],
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, bool]:
        if mode == "nominal":
            executed = desired.copy()
            saturated = False
        else:
            if correction is None:
                raise ValueError("calibrated mode requires correction")
            output = correction.forward(
                desired_v_mps=float(desired[0]),
                desired_omega_rps=float(desired[1]),
                measured_omega_rps=measured_omega,
                control_dt_s=dt,
            )
            executed = np.array([output.executed_v_mps, output.executed_omega_rps])
            saturated = output.saturated
        side = np.asarray(self.differential.forward(executed), dtype=np.float64)
        clipped = np.clip(side, -self.maximum_wheel_target, self.maximum_wheel_target)
        wheel_clipped = not np.array_equal(side, clipped)
        saturated = saturated or wheel_clipped
        if wheel_clipped:
            radius = float(self.wheels["radius_m"])
            separation = float(self.wheels["separation_m"])
            executed = np.array(
                [
                    radius * (clipped[0] + clipped[1]) / 2.0,
                    radius * (clipped[1] - clipped[0]) / separation,
                ]
            )
        target = runtime_wheel_command(clipped, self.wheels)
        self.articulation.apply_action(ArticulationAction(joint_velocities=target))
        return executed, target, saturated


def body_motion(previous: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    forward = np.array([math.cos(float(previous[2])), math.sin(float(previous[2]))])
    return np.array(
        [
            float((current[:2] - previous[:2]) @ forward / dt),
            float(wrap_angle(current[2] - previous[2]) / dt),
        ]
    )


def execute(
    runtime: GuiRuntime,
    config: Mapping[str, Any],
    *,
    mode: str,
    pose: np.ndarray,
    command_source: Callable[[int, np.ndarray], tuple[np.ndarray, bool]],
    maximum_controls: int,
    parameters: Mapping[str, Any] | None,
    scenario: str,
    on_path: Callable[[list[np.ndarray]], None],
) -> list[np.ndarray]:
    runtime.reset(pose, float(config["simulation"]["settling_duration_s"]))
    correction = runtime.correction(parameters, config) if parameters is not None else None
    actual = [se2_from_world_pose(runtime.robot)]
    measured = np.zeros(2)
    for control_index in range(maximum_controls):
        desired, finished = command_source(control_index, actual[-1])
        if finished:
            break
        executed, target, saturated = runtime.apply(
            desired,
            mode,
            correction,
            float(measured[1]),
            config,
            runtime.control_dt,
        )
        previous = actual[-1]
        for _ in range(runtime.physics_steps):
            runtime.paced_step()
            actual.append(se2_from_world_pose(runtime.robot))
        measured = body_motion(previous, actual[-1], runtime.control_dt)
        measured_wheels = canonical_wheel_values(
            runtime.robot.get_joint_velocities(), runtime.wheels
        )
        print(
            "STAGE0D_GUI_TELEMETRY="
            + json.dumps(
                {
                    "phase": "EXECUTION",
                    "scenario": scenario,
                    "mode": mode.upper(),
                    "desired_v_mps": float(desired[0]),
                    "desired_omega_rps": float(desired[1]),
                    "executed_v_mps": float(executed[0]),
                    "executed_omega_rps": float(executed[1]),
                    "measured_v_mps": float(measured[0]),
                    "measured_omega_rps": float(measured[1]),
                    "yaw_rate_error_rps": float(desired[1] - measured[1]),
                    "wheel_targets_rad_s": canonical_wheel_values(target, runtime.wheels).tolist(),
                    "wheel_measured_rad_s": measured_wheels.tolist(),
                    "calibration_mode": (
                        "none" if parameters is None else "effective-width + optional PI"
                    ),
                    "effective_track_width_m": (
                        None
                        if parameters is None
                        else parameters["calibrated_effective_wheel_separation_m"]
                    ),
                    "saturated": saturated,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        on_path(actual)
    runtime.articulation.apply_action(
        ArticulationAction(joint_velocities=np.zeros(runtime.robot.num_dof))
    )
    return actual


def configure_camera(paths: list[np.ndarray], height: float) -> None:
    xy = np.vstack([path[:, :2] for path in paths])
    center = (np.min(xy, axis=0) + np.max(xy, axis=0)) / 2.0
    set_camera_view(
        eye=[float(center[0]), float(center[1] - 0.5), height],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )


def redraw(
    draw,
    config: Mapping[str, Any],
    reference: np.ndarray,
    nominal: list[np.ndarray],
    calibrated: list[np.ndarray],
    extras: list[tuple[np.ndarray, list[float], str]] | None = None,
) -> None:
    visual = config["gui"]["visualization"]
    # Keep diagnostic overlays above the Jackal chassis so short replay paths and
    # co-located boundary markers remain visible in the top-down viewport.
    base_z = max(float(visual["z_offset_m"]), 0.65)
    draw.clear_lines()
    draw.clear_points()
    draw_polyline(
        draw,
        reference,
        z=base_z,
        color=visual["reference_color_rgba"],
        width=float(visual["line_width"]),
    )
    if len(nominal) > 1:
        draw_polyline(
            draw,
            np.asarray(nominal),
            z=base_z + 0.03,
            color=visual["nominal_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )
    if len(calibrated) > 1:
        draw_polyline(
            draw,
            np.asarray(calibrated),
            z=base_z + 0.06,
            color=visual["calibrated_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )
    marker_index = 0
    for path, color, kind in extras or []:
        if kind == "line":
            draw_polyline(
                draw,
                path,
                z=base_z + 0.09,
                color=color,
                width=float(visual["line_width"]) * 0.8,
            )
        else:
            # Nested sizes preserve a visible saved-B halo when nominal B is
            # exactly co-located, while retaining both poses at their true XY.
            size_scale = max(0.7, 1.6 - 0.3 * marker_index)
            heading_length = max(0.14, 0.30 - 0.06 * marker_index)
            draw_pose_points(
                draw,
                path,
                z=base_z + 0.12,
                color=color,
                size=float(visual["marker_size"]) * size_scale,
            )
            draw_heading_markers(
                draw,
                path,
                z=base_z + 0.12,
                color=color,
                width=3.0,
                length_m=heading_length,
            )
            marker_index += 1


def primitive_suite(
    runtime: GuiRuntime,
    config: Mapping[str, Any],
    parameters: Mapping[str, Any],
    draw,
    output: Path,
) -> list[str]:
    configured = config["gui"]["primitive_scenarios"]
    scenarios = configured if ARGS.scenario == "all" else [
        row for row in configured if row["id"] == ARGS.scenario
    ]
    completed = []
    for scenario in scenarios:
        identifier = str(scenario["id"])
        v = float(scenario["linear_velocity_mps"])
        omega = float(scenario["angular_velocity_rps"])
        phase_counts = [
            int(round(float(config["simulation"][key]) / runtime.control_dt))
            for key in (
                "initial_stop_duration_s",
                "active_duration_s",
                "final_stop_duration_s",
            )
        ]
        commands = (
            [np.zeros(2)] * phase_counts[0]
            + [np.array([v, omega])] * phase_counts[1]
            + [np.zeros(2)] * phase_counts[2]
        )
        segments = [
            {
                "name": "initial_stop",
                "duration_s": float(config["simulation"]["initial_stop_duration_s"]),
                "linear_velocity_mps": 0.0,
                "angular_velocity_rps": 0.0,
            },
            {
                "name": "active",
                "duration_s": float(config["simulation"]["active_duration_s"]),
                "linear_velocity_mps": v,
                "angular_velocity_rps": omega,
            },
            {
                "name": "final_stop",
                "duration_s": float(config["simulation"]["final_stop_duration_s"]),
                "linear_velocity_mps": 0.0,
                "angular_velocity_rps": 0.0,
            },
        ]
        reference = generate_reference_trajectory(
            runtime.initial_pose, segments_from_config(segments), runtime.control_dt
        ).poses
        configure_camera([reference], float(config["gui"]["visualization"]["camera_height_m"]))
        nominal: list[np.ndarray] = []
        calibrated: list[np.ndarray] = []

        def source(index: int, _pose: np.ndarray) -> tuple[np.ndarray, bool]:
            return (commands[index], False) if index < len(commands) else (np.zeros(2), True)

        print(f"STAGE0D_GUI_PHASE=PRIMITIVE_NOMINAL scenario={identifier}", flush=True)
        nominal = execute(
            runtime,
            config,
            mode="nominal",
            pose=runtime.initial_pose,
            command_source=source,
            maximum_controls=len(commands) + 1,
            parameters=None,
            scenario=identifier,
            on_path=lambda path: redraw(draw, config, reference, path, calibrated),
        )
        print(f"STAGE0D_GUI_PHASE=RESET_BEFORE_CALIBRATED scenario={identifier}", flush=True)
        calibrated = execute(
            runtime,
            config,
            mode="calibrated",
            pose=runtime.initial_pose,
            command_source=source,
            maximum_controls=len(commands) + 1,
            parameters=parameters,
            scenario=identifier,
            on_path=lambda path: redraw(draw, config, reference, nominal, path),
        )
        redraw(draw, config, reference, nominal, calibrated)
        scenario_dir = output / identifier
        scenario_dir.mkdir(parents=True, exist_ok=False)
        np.save(scenario_dir / "reference.npy", reference)
        np.save(scenario_dir / "nominal_actual.npy", nominal)
        np.save(scenario_dir / "calibrated_actual.npy", calibrated)
        completed.append(identifier)
    return completed


def composite_suite(
    runtime: GuiRuntime,
    config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    parameters: Mapping[str, Any],
    draw,
    output: Path,
) -> list[str]:
    reference = generate_reference_trajectory(
        runtime.initial_pose,
        segments_from_config(stage0b["composite_motion_profile"]),
        runtime.control_dt,
    ).poses
    configure_camera([reference], float(config["gui"]["visualization"]["camera_height_m"]))
    nominal: list[np.ndarray] = []
    calibrated: list[np.ndarray] = []

    def run(mode: str, prior: list[np.ndarray]) -> list[np.ndarray]:
        follower = TrajectoryFollower(reference, follower_config(stage0b))

        def source(_index: int, pose: np.ndarray) -> tuple[np.ndarray, bool]:
            command = follower.forward(pose)
            return (
                np.array([command.linear_velocity_mps, command.angular_velocity_rps]),
                command.goal_reached,
            )

        current: list[np.ndarray] = []
        return execute(
            runtime,
            config,
            mode=mode,
            pose=runtime.initial_pose,
            command_source=source,
            maximum_controls=int(
                round(float(stage0b["closed_loop"]["maximum_duration_s"]) / runtime.control_dt)
            ),
            parameters=(parameters if mode == "calibrated" else None),
            scenario="stage0b_composite",
            on_path=lambda path: redraw(
                draw, config, reference, path if mode == "nominal" else prior, path if mode == "calibrated" else current
            ),
        )

    print("STAGE0D_GUI_PHASE=COMPOSITE_NOMINAL", flush=True)
    nominal = run("nominal", [])
    print("STAGE0D_GUI_PHASE=RESET_BEFORE_COMPOSITE_CALIBRATED", flush=True)
    calibrated = run("calibrated", nominal)
    redraw(draw, config, reference, nominal, calibrated)
    np.save(output / "reference.npy", reference)
    np.save(output / "nominal_actual.npy", nominal)
    np.save(output / "calibrated_actual.npy", calibrated)
    return ["stage0b_composite"]


def exp02b_suite(
    runtime: GuiRuntime,
    config: Mapping[str, Any],
    exp02b: Mapping[str, Any],
    parameters: Mapping[str, Any],
    draw,
    output: Path,
) -> list[str]:
    exp02b_run = resolve_path(config["paths"]["exp02b_run"])
    selection = load_strict_json(exp02b_run / "source_selection.json")
    verify_frozen_source_selection(selection["source_root"], exp02b)
    source = load_source_case(ARGS.case, selection["cases"][ARGS.case]["trial_directory"])
    with (source.trial_directory / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    old_rows = [row for row in rows if row["reference_source"] == "OLD"]
    fresh_first = next(row for row in rows if row["reference_source"] == "FRESH")
    commands: list[tuple[np.ndarray, float]] = []
    for index, row in enumerate(old_rows):
        next_time = (
            float(old_rows[index + 1]["sim_time_s"])
            if index + 1 < len(old_rows)
            else float(fresh_first["sim_time_s"])
        )
        duration = next_time - float(row["sim_time_s"])
        count = int(round(duration / runtime.physics_dt))
        if count < 1 or not math.isclose(count * runtime.physics_dt, duration, abs_tol=1e-6):
            raise ValueError("EXP-02B GUI replay requires physics-step-aligned source intervals")
        command = np.array([float(row["v_command_mps"]), float(row["omega_command_rps"])])
        commands.append((command, duration))

    fresh = source.fresh_world
    configure_camera(
        [source.old_world, fresh], float(config["gui"]["visualization"]["camera_height_m"])
    )
    visual = config["gui"]["visualization"]
    markers = [
        (np.asarray([source.boundary_pose_world]), visual["saved_boundary_color_rgba"], "marker")
    ]
    nominal: list[np.ndarray] = []
    calibrated: list[np.ndarray] = []
    extras = [(fresh, visual["fresh_color_rgba"], "line"), *markers]

    def run(mode: str, prior: list[np.ndarray]) -> list[np.ndarray]:
        initial = np.asarray(source.source_metadata["initial_pose_se2"])
        runtime.reset(initial, float(config["simulation"]["settling_duration_s"]))
        correction = runtime.correction(parameters, config) if mode == "calibrated" else None
        actual = [se2_from_world_pose(runtime.robot)]
        measured = np.zeros(2)
        for command_index, (desired, duration) in enumerate(commands):
            executed, target, saturated = runtime.apply(
                desired,
                mode,
                correction,
                float(measured[1]),
                config,
                duration,
            )
            previous = actual[-1]
            for _ in range(int(round(duration / runtime.physics_dt))):
                runtime.paced_step()
                actual.append(se2_from_world_pose(runtime.robot))
            measured = body_motion(previous, actual[-1], duration)
            measured_wheels = canonical_wheel_values(
                runtime.robot.get_joint_velocities(), runtime.wheels
            )
            print(
                "STAGE0D_GUI_TELEMETRY="
                + json.dumps(
                    {
                        "phase": "EXP02B_OLD_REPLAY",
                        "scenario": ARGS.case,
                        "k": ARGS.k,
                        "method": ARGS.method,
                        "source_command_index": command_index,
                        "mode": mode.upper(),
                        "desired_v_mps": float(desired[0]),
                        "desired_omega_rps": float(desired[1]),
                        "executed_v_mps": float(executed[0]),
                        "executed_omega_rps": float(executed[1]),
                        "measured_v_mps": float(measured[0]),
                        "measured_omega_rps": float(measured[1]),
                        "yaw_rate_error_rps": float(desired[1] - measured[1]),
                        "wheel_targets_rad_s": canonical_wheel_values(
                            target, runtime.wheels
                        ).tolist(),
                        "wheel_measured_rad_s": measured_wheels.tolist(),
                        "effective_track_width_m": (
                            None
                            if correction is None
                            else parameters["calibrated_effective_wheel_separation_m"]
                        ),
                        "saturated": saturated,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            current_extras = [
                (fresh, visual["fresh_color_rgba"], "line"),
                *markers,
            ]
            redraw(
                draw,
                config,
                source.old_world,
                actual if mode == "nominal" else prior,
                actual if mode == "calibrated" else [],
                current_extras,
            )
        runtime.articulation.apply_action(
            ArticulationAction(joint_velocities=np.zeros(runtime.robot.num_dof))
        )
        return actual

    print(
        f"STAGE0D_GUI_PHASE=EXP02B_NOMINAL case={ARGS.case} "
        f"k={ARGS.k} method={ARGS.method}",
        flush=True,
    )
    nominal = run("nominal", [])
    markers.append(
        (np.asarray([nominal[-1]]), visual["nominal_boundary_color_rgba"], "marker")
    )
    print(
        f"STAGE0D_GUI_PHASE=RESET_BEFORE_EXP02B_CALIBRATED case={ARGS.case} "
        f"k={ARGS.k} method={ARGS.method}",
        flush=True,
    )
    calibrated = run("calibrated", nominal)
    markers.append(
        (np.asarray([calibrated[-1]]), visual["calibrated_boundary_color_rgba"], "marker")
    )
    redraw(
        draw,
        config,
        source.old_world,
        nominal,
        calibrated,
        [(fresh, visual["fresh_color_rgba"], "line"), *markers],
    )
    np.save(output / "planned_old.npy", source.old_world)
    np.save(output / "raw_fresh.npy", fresh)
    np.save(output / "nominal_actual.npy", nominal)
    np.save(output / "calibrated_actual.npy", calibrated)
    return [ARGS.case]


def hold() -> None:
    print("STAGE0D_GUI_PHASE=FINISHED; close Isaac Sim to exit", flush=True)
    while SIMULATION_APP.is_running():
        SIMULATION_APP.update()
        time.sleep(0.02)


def capture_final_viewport(output: Path) -> Path:
    path = output / "final_viewport.png"
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport is unavailable for GUI evidence capture")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 10.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("viewport capture did not complete")
    return path


def main() -> None:
    run = ARGS.calibration_run.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    model = load_strict_json(run / "calibration_model.json")
    if load_strict_json(run / "metadata.json").get("smoke"):
        raise ValueError("GUI validation requires a completed primary calibration run")
    stage0b = load_yaml(resolve_path(config["paths"]["stage0b_config"]))
    exp02b = load_yaml(resolve_path(config["paths"]["exp02b_config"]))
    replay_context = config["exp02b_replay_validation"]
    if ARGS.suite == "exp02b" and (
        ARGS.k != int(replay_context["entry_index"])
        or ARGS.method != str(replay_context["method"])
    ):
        raise ValueError(
            "Stage 0-D GUI only supports its frozen EXP-02B context: "
            f"k={replay_context['entry_index']}, method={replay_context['method']}"
        )
    parameters = model["selected_parameters"]
    output = run / "gui_metadata" / datetime.now(timezone.utc).strftime(
        f"{ARGS.suite}-%Y%m%dT%H%M%SZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    runtime = GuiRuntime(config, stage0b)
    draw = _debug_draw.acquire_debug_draw_interface()
    visual = config["gui"]["visualization"]
    print("[Stage 0-D GUI] legend:", flush=True)
    print(f"  reference/planned OLD = blue {visual['reference_color_rgba']}", flush=True)
    print(f"  nominal actual = red/orange {visual['nominal_color_rgba']}", flush=True)
    print(f"  calibrated actual = green {visual['calibrated_color_rgba']}", flush=True)
    if ARGS.suite == "primitive":
        completed = primitive_suite(runtime, config, parameters, draw, output)
    elif ARGS.suite == "composite":
        completed = composite_suite(runtime, config, stage0b, parameters, draw, output)
    else:
        completed = exp02b_suite(runtime, config, exp02b, parameters, draw, output)
    viewport_capture = capture_final_viewport(output)
    metadata = {
        "diagnostic_only": True,
        "quantitative_primary": False,
        "suite": ARGS.suite,
        "case": ARGS.case if ARGS.suite == "exp02b" else None,
        "k": ARGS.k if ARGS.suite == "exp02b" else None,
        "method": ARGS.method if ARGS.suite == "exp02b" else None,
        "completed_scenarios": completed,
        "calibration_run": str(run),
        "calibration_model_sha256": sha256_file(run / "calibration_model.json"),
        "selected_parameters": parameters,
        "real_time_factor": runtime.real_time_factor,
        "slowdown_changes_simulation_timing": False,
        "legend": {
            "reference": visual["reference_color_rgba"],
            "nominal_actual": visual["nominal_color_rgba"],
            "calibrated_actual": visual["calibrated_color_rgba"],
        },
        "jackal_asset": runtime.asset,
        "runtime_physics_overrides": {},
        "viewport_capture": str(viewport_capture),
    }
    with (output / "metadata.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(f"STAGE0D_GUI_OUTPUT={output}", flush=True)
    if ARGS.hold or (not ARGS.no_hold and bool(config["gui"]["final_hold"])):
        hold()


try:
    main()
finally:
    SIMULATION_APP.close()
