#!/usr/bin/env python3
"""Visualize and safely execute one derived LightNav chunk in Isaac Sim GUI."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_lightnav_single_chunk.yaml",
    )
    parser.add_argument("--visualize-only", action="store_true")
    parser.add_argument(
        "--replay",
        action="store_true",
        help="execute again for GUI inspection without overwriting recorded artifacts",
    )
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true", help="explicit diagnostic override")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.util.debug_draw import _debug_draw
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from lightnav_stage0c_runtime import (
    canonical_wheel_names,
    canonical_wheel_values,
    discover_wheels,
    find_articulation_root,
    finite_positive,
    load_config,
    quaternion_from_yaw,
    require_mapping,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.controller_validation import (
    ControllerTelemetry,
    compute_controller_metrics,
    estimate_body_velocities,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.lightnav_adapter import (
    save_json_exclusive,
    save_npy_exclusive,
    validate_single_chunk_run,
)
from reconciliation.trajectory import validate_se2_trajectory


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def add_static_box(path: str, center, size, color) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    xform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def follower_config(config) -> FollowerConfig:
    values = require_mapping(config, "closed_loop")
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


def rgba(value) -> list[float]:
    result = [float(item) for item in value]
    if len(result) != 4:
        raise ValueError("DebugDraw colors must be RGBA")
    return result


def draw_paths(draw, reference: np.ndarray, actual: np.ndarray, config) -> None:
    z = float(config["z_offset_m"])
    reference_points = [[float(x), float(y), z] for x, y in reference[:, :2]]
    actual_points = [[float(x), float(y), z * 1.15] for x, y in actual[:, :2]]
    draw.clear_lines()
    draw.clear_points()
    if len(reference_points) > 1:
        draw.draw_lines(
            reference_points[:-1],
            reference_points[1:],
            [rgba(config["trajectory_color_rgba"])] * (len(reference_points) - 1),
            [float(config["line_width"])] * (len(reference_points) - 1),
        )
    if len(actual_points) > 1:
        draw.draw_lines(
            actual_points[:-1],
            actual_points[1:],
            [rgba(config["actual_color_rgba"])] * (len(actual_points) - 1),
            [float(config["line_width"])] * (len(actual_points) - 1),
        )
    starts = reference_points
    ends = [
        [
            point[0] + float(config["heading_marker_length_m"]) * math.cos(float(pose[2])),
            point[1] + float(config["heading_marker_length_m"]) * math.sin(float(pose[2])),
            z,
        ]
        for point, pose in zip(starts, reference, strict=True)
    ]
    draw.draw_lines(
        starts,
        ends,
        [rgba(config["heading_color_rgba"])] * len(starts),
        [float(config["heading_line_width"])] * len(starts),
    )
    points = [reference_points[0], reference_points[-1], actual_points[0], actual_points[-1]]
    draw.draw_points(
        points,
        [rgba(config["heading_color_rgba"])] * len(points),
        [float(config["endpoint_point_size"])] * len(points),
    )


def run() -> dict:
    if ARGS.visualize_only and ARGS.replay:
        raise ValueError("--visualize-only and --replay cannot be combined")
    config = load_config(ARGS.config)
    run_dir = ARGS.run_directory.resolve()
    validation = validate_single_chunk_run(run_dir)
    recorded_validation = load_json(run_dir / "results/validation.json")
    if not recorded_validation.get("safe_for_execution") and not ARGS.visualize_only:
        raise RuntimeError("derived LightNav path failed execution safety checks")
    world_path = validate_se2_trajectory(
        np.load(run_dir / "derived/lightnav_world_path.npy", allow_pickle=False),
        name="LightNav world path",
    )
    observation = load_json(run_dir / "raw/observation_metadata.json")
    anchor = np.asarray(observation["robot_pose_at_observation"], dtype=np.float64)
    execution_reference = np.vstack((anchor, world_path))
    recorded_outputs = (
        run_dir / "derived/execution_reference.npy",
        run_dir / "derived/jackal_actual_trajectory.npy",
        run_dir / "derived/execution_samples.csv",
        run_dir / "results/execution_metrics.json",
        run_dir / "results/execution_metadata.json",
    )
    existing_outputs = [path for path in recorded_outputs if path.exists()]
    if existing_outputs and not ARGS.visualize_only and not ARGS.replay:
        raise FileExistsError(
            "this immutable run already has execution output; use --replay to watch the "
            "Jackal execute again without overwriting it"
        )
    if not ARGS.visualize_only and not ARGS.replay:
        save_npy_exclusive(run_dir / "derived/execution_reference.npy", execution_reference)

    simulation = require_mapping(config, "simulation")
    robot_config = require_mapping(config, "robot")
    scene = require_mapping(config, "scene")
    closed_loop = require_mapping(config, "closed_loop")
    visualization = require_mapping(config, "visualization")
    physics_dt = finite_positive(simulation, "physics_dt")
    sample_dt = finite_positive(closed_loop, "sample_dt")
    pre_execution_pause_s = float(visualization["pre_execution_pause_s"])
    playback_real_time_factor = finite_positive(visualization, "playback_real_time_factor")
    if not math.isfinite(pre_execution_pause_s) or pre_execution_pause_s < 0.0:
        raise ValueError("visualization.pre_execution_pause_s must be finite and non-negative")
    sample_steps = int(round(sample_dt / physics_dt))
    if not math.isclose(sample_dt / physics_dt, sample_steps, abs_tol=1e-9):
        raise ValueError("closed-loop sample_dt must be divisible by physics_dt")
    max_duration = finite_positive(closed_loop, "maximum_duration_s")
    max_samples = int(round(max_duration / sample_dt))

    asset = resolve_jackal_asset(robot_config)
    world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Stage0CPlaybackDomeLight")
    dome.CreateIntensityAttr(1000.0)
    length = float(scene["corridor_length_m"])
    width = float(scene["corridor_width_m"])
    height = float(scene["wall_height_m"])
    thickness = float(scene["wall_thickness_m"])
    center_x = float(scene["wall_center_x_m"])
    for side, y in (("Left", width / 2.0), ("Right", -width / 2.0)):
        add_static_box(
            f"/World/Corridor{side}Wall",
            [center_x, y, height / 2.0],
            [length, thickness, height],
            [0.72, 0.74, 0.78],
        )
    reference_prim = str(robot_config["reference_prim_path"])
    add_reference_to_stage(str(asset["resolved_path"]), reference_prim)
    while is_stage_loading():
        SIMULATION_APP.update()
    articulation_root = find_articulation_root(reference_prim)
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="stage0c_playback_jackal",
            position=np.array([anchor[0], anchor[1], simulation["spawn_height_m"]]),
            orientation=quaternion_from_yaw(float(anchor[2])),
        )
    )
    world.reset()
    set_camera_view(
        eye=[float(value) for value in visualization["camera_eye"]],
        target=[float(value) for value in visualization["camera_target"]],
        camera_prim_path="/OmniverseKit_Persp",
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    saved_actual_path = run_dir / "derived/jackal_actual_trajectory.npy"
    if ARGS.visualize_only and saved_actual_path.is_file():
        actual = validate_se2_trajectory(
            np.load(saved_actual_path, allow_pickle=False),
            name="saved Jackal actual trajectory",
        ).tolist()
    else:
        actual = [se2_from_world_pose(robot)]
    draw_paths(draw, execution_reference, np.asarray(actual), visualization)
    print(f"[Stage 0-C] raw action: {run_dir / 'raw/lightnav_actions.npy'}")
    print(f"[Stage 0-C] derived world path: {run_dir / 'derived/lightnav_world_path.npy'}")
    print(f"[Stage 0-C] world path start/end: {world_path[0].tolist()} -> {world_path[-1].tolist()}")
    if ARGS.visualize_only:
        print(
            "[Stage 0-C] reference/actual paths drawn: "
            f"{execution_reference.shape[0]}/{len(actual)} poses"
        )
        return {
            "visualized": True,
            "executed": False,
            "saved_actual_drawn": saved_actual_path.is_file(),
            "validation": validation,
        }

    wheels = discover_wheels(robot, articulation_root)
    differential = DifferentialController(
        wheel_radius=float(wheels["radius_m"]),
        wheel_base=float(wheels["separation_m"]),
    )
    controller = robot.get_articulation_controller()
    follower = TrajectoryFollower(execution_reference, follower_config(config))
    settling_steps = int(round(float(simulation["settling_duration_s"]) / physics_dt))
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    for _ in range(settling_steps):
        world.step(render=True)
    actual = [se2_from_world_pose(robot)]
    draw_paths(draw, execution_reference, np.asarray(actual), visualization)
    if pre_execution_pause_s > 0.0:
        print(
            f"[Stage 0-C] Jackal motion starts in {pre_execution_pause_s:.1f} s; "
            f"GUI playback is paced at {playback_real_time_factor:.2f}x simulation time"
        )
        time.sleep(pre_execution_pause_s)
    command = follower.forward(actual[0])
    times = [0.0]
    commands = [[0.0, 0.0]]
    targets = [np.zeros(4)]
    wheel_actual = [canonical_wheel_values(robot.get_joint_velocities(), wheels)]
    reference_indices = [command.nearest_index]
    origin_time = float(world.current_time)
    for _ in range(max_samples):
        if command.goal_reached:
            break
        side_speeds = differential.forward(
            np.asarray([command.linear_velocity_mps, command.angular_velocity_rps])
        )
        runtime_target = runtime_wheel_command(side_speeds, wheels)
        controller.apply_action(ArticulationAction(joint_velocities=runtime_target))
        for _ in range(sample_steps):
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim closed before LightNav playback completed")
            wall_physics_step_start = time.monotonic()
            world.step(render=True)
            target_wall_period = physics_dt / playback_real_time_factor
            remaining_wall_time = target_wall_period - (
                time.monotonic() - wall_physics_step_start
            )
            if remaining_wall_time > 0.0:
                time.sleep(remaining_wall_time)
        pose = se2_from_world_pose(robot)
        next_command = follower.forward(pose)
        actual.append(pose)
        times.append(float(world.current_time) - origin_time)
        commands.append([command.linear_velocity_mps, command.angular_velocity_rps])
        targets.append(canonical_wheel_values(runtime_target, wheels))
        wheel_actual.append(canonical_wheel_values(robot.get_joint_velocities(), wheels))
        reference_indices.append(next_command.nearest_index)
        command = next_command
        draw_paths(draw, execution_reference, np.asarray(actual), visualization)
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))

    telemetry = ControllerTelemetry(
        execution_reference,
        np.asarray(actual),
        np.asarray(times),
        np.asarray(commands),
        np.asarray(targets),
        np.asarray(wheel_actual),
        np.asarray(reference_indices),
        canonical_wheel_names(wheels),
    )
    metrics = compute_controller_metrics(telemetry)
    metrics.update(
        {
            "goal_reached": bool(command.goal_reached),
            "execution_duration_s": float(times[-1]),
            "maximum_duration_reached": not command.goal_reached,
            "metric_scope": "controller integration only; not LightNav navigation quality",
        }
    )
    measured_v, measured_w = estimate_body_velocities(
        telemetry.actual_trajectory, telemetry.sim_times_s
    )
    if not ARGS.replay:
        save_npy_exclusive(
            run_dir / "derived/jackal_actual_trajectory.npy",
            telemetry.actual_trajectory,
        )
        with (run_dir / "derived/execution_samples.csv").open(
            "x", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "sample_index", "sim_time_s", "reference_index", "ref_x", "ref_y",
                    "ref_yaw", "actual_x", "actual_y", "actual_yaw", "commanded_v_mps",
                    "commanded_omega_rps", "measured_v_mps", "measured_omega_rps",
                )
            )
            for index in range(telemetry.sample_count):
                writer.writerow(
                    (
                        index,
                        telemetry.sim_times_s[index],
                        telemetry.reference_indices[index],
                        *telemetry.reference_trajectory[telemetry.reference_indices[index]],
                        *telemetry.actual_trajectory[index],
                        *telemetry.commanded_body[index],
                        measured_v[index],
                        measured_w[index],
                    )
                )
        save_json_exclusive(run_dir / "results/execution_metrics.json", metrics)
        save_json_exclusive(
            run_dir / "results/execution_metadata.json",
            {
                "creation_time": datetime.now().astimezone().isoformat(timespec="seconds"),
                "controller": "Stage 0-B TrajectoryFollower + Isaac DifferentialController",
                "reference": (
                    "observation anchor prepended to derived LightNav world future poses"
                ),
                "no_interpolation": True,
                "robot_prim_path": articulation_root,
                "actual_dof_names": list(robot.dof_names or []),
                "wheel_parameters": wheels,
                "visualized_in_gui": not ARGS.headless,
                "pre_execution_pause_s": pre_execution_pause_s,
                "playback_real_time_factor": playback_real_time_factor,
                "research_evidence": False,
            },
        )
    else:
        print("[Stage 0-C] replay mode: existing execution artifacts were not modified")
    result = {
        "visualized": True,
        "executed": True,
        "recorded": not ARGS.replay,
        "metrics": metrics,
    }
    print("STAGE0C_PLAYBACK_JSON=" + json.dumps(result, sort_keys=True))
    return result


try:
    RESULT = run()
    if not ARGS.no_hold and not ARGS.headless:
        print("[Stage 0-C] GUI ready; close the window to exit")
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
finally:
    SIMULATION_APP.close()
