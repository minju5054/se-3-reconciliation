#!/usr/bin/env python3
"""Run the GUI Stage 0 trajectory smoke test with Isaac Sim's Jackal asset."""

from __future__ import annotations

import argparse
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
        default=REPOSITORY_ROOT / "configs/stage0_jackal_trajectory.yaml",
    )
    parser.add_argument("--run-id", help="explicit immutable run directory name")
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="close after saving; default keeps the GUI open for inspection",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="explicit diagnostics mode only; GUI is the default",
    )
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
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from pxr import Usd, UsdGeom, UsdPhysics

from reconciliation.se2 import wrap_angle
from reconciliation.stage0_jackal import (
    Stage0Recording,
    compute_stage0_metrics,
    save_stage0_run,
    validate_stage0_output,
)
from reconciliation.trajectory import (
    MotionSegment,
    generate_reference_trajectory,
    segments_from_config,
    validate_pose_se2,
)


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Stage 0 config must contain an object")
    return config


def require_mapping(config: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = config.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"config field {key!r} must contain an object")
    return value


def finite_positive(config: Mapping[str, Any], key: str) -> float:
    value = float(config[key])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"config field {key!r} must be finite and greater than zero")
    return value


def quaternion_from_yaw(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def se2_from_world_pose(robot: SingleArticulation) -> np.ndarray:
    position, orientation = robot.get_world_pose()
    w, x, y, z = (float(value) for value in orientation)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([float(position[0]), float(position[1]), wrap_angle(yaw)], dtype=np.float64)


def find_articulation_root(reference_prim_path: str) -> str:
    stage = omni.usd.get_context().get_stage()
    reference = stage.GetPrimAtPath(reference_prim_path)
    if not reference.IsValid():
        raise RuntimeError(f"robot reference prim was not created: {reference_prim_path}")
    roots = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(reference)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected exactly one Jackal articulation root, found: {roots}")
    return roots[0]


def collision_cylinder_radius(stage, wheel_body_prim) -> float:
    radii = []
    for prim in Usd.PrimRange(wheel_body_prim):
        if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
            radius = UsdGeom.Cylinder(prim).GetRadiusAttr().Get()
            if radius is not None:
                radii.append(float(radius))
    if len(radii) != 1 or not math.isfinite(radii[0]) or radii[0] <= 0.0:
        raise RuntimeError(
            f"expected one collision cylinder below {wheel_body_prim.GetPath()}, found {radii}"
        )
    return radii[0]


def discover_wheels(robot: SingleArticulation, articulation_root: str) -> dict[str, Any]:
    """Discover all wheel mapping/parameters from the loaded USD and articulation runtime."""

    dof_names = list(robot.dof_names or [])
    if robot.num_dof != 4 or len(dof_names) != 4:
        raise RuntimeError(
            f"Stage 0 requires the official 4-wheel Jackal articulation; "
            f"runtime reported num_dof={robot.num_dof}, dof_names={dof_names}"
        )
    if len(set(dof_names)) != 4:
        raise RuntimeError(f"Jackal DOF names must be unique: {dof_names}")

    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(articulation_root)
    joint_by_name = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(root_prim)
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in dof_names
    }
    if set(joint_by_name) != set(dof_names):
        raise RuntimeError(
            "every runtime DOF must resolve to a revolute joint; "
            f"DOFs={dof_names}, joints={sorted(joint_by_name)}"
        )

    wheel_records = []
    for index, dof_name in enumerate(dof_names):
        joint = UsdPhysics.RevoluteJoint(joint_by_name[dof_name])
        local_position = joint.GetLocalPos0Attr().Get()
        body_targets = joint.GetBody1Rel().GetTargets()
        if local_position is None or len(body_targets) != 1:
            raise RuntimeError(f"cannot resolve wheel joint geometry for {dof_name}")
        lateral_y = float(local_position[1])
        if math.isclose(lateral_y, 0.0, abs_tol=1e-6):
            raise RuntimeError(f"wheel joint {dof_name} has ambiguous lateral position {lateral_y}")
        body_prim = stage.GetPrimAtPath(body_targets[0])
        radius = collision_cylinder_radius(stage, body_prim)
        wheel_records.append(
            {
                "dof_index": index,
                "dof_name": dof_name,
                "joint_prim_path": str(joint.GetPrim().GetPath()),
                "wheel_body_prim_path": str(body_targets[0]),
                "lateral_position_m": lateral_y,
                "side": "left" if lateral_y > 0.0 else "right",
                "radius_m": radius,
            }
        )

    left = [record for record in wheel_records if record["side"] == "left"]
    right = [record for record in wheel_records if record["side"] == "right"]
    if len(left) != 2 or len(right) != 2:
        raise RuntimeError(f"expected two runtime wheels per side, found: {wheel_records}")
    radii = np.asarray([record["radius_m"] for record in wheel_records], dtype=np.float64)
    if not np.allclose(radii, radii[0], rtol=0.0, atol=1e-6):
        raise RuntimeError(f"Jackal wheel collision radii differ: {radii.tolist()}")
    separation = float(
        np.mean([record["lateral_position_m"] for record in left])
        - np.mean([record["lateral_position_m"] for record in right])
    )
    if separation <= 0.0:
        raise RuntimeError(f"invalid discovered wheel separation: {separation}")

    return {
        "radius_m": float(radii[0]),
        "separation_m": separation,
        "radius_source": "USD collision cylinder radius",
        "separation_source": "USD revolute-joint localPos0 lateral spacing",
        "wheels": wheel_records,
    }


def wheel_velocity_command(
    linear_velocity_mps: float,
    angular_velocity_rps: float,
    wheel_parameters: Mapping[str, Any],
    dof_count: int,
) -> np.ndarray:
    radius = float(wheel_parameters["radius_m"])
    separation = float(wheel_parameters["separation_m"])
    left_velocity = (linear_velocity_mps - angular_velocity_rps * separation / 2.0) / radius
    right_velocity = (linear_velocity_mps + angular_velocity_rps * separation / 2.0) / radius
    command = np.zeros(dof_count, dtype=np.float64)
    for wheel in wheel_parameters["wheels"]:
        command[int(wheel["dof_index"])] = left_velocity if wheel["side"] == "left" else right_velocity
    return command


def rgba(value: Sequence[float], name: str) -> list[float]:
    result = [float(component) for component in value]
    if len(result) != 4 or not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} must be finite RGBA with four components")
    return result


def xyz_points(trajectory: np.ndarray, z_offset: float) -> list[list[float]]:
    return [[float(pose[0]), float(pose[1]), z_offset] for pose in trajectory]


def draw_polyline(draw, points, color, width: float) -> None:
    if len(points) < 2:
        return
    starts = points[:-1]
    ends = points[1:]
    draw.draw_lines(starts, ends, [color] * len(starts), [width] * len(starts))


def draw_trajectories(
    draw,
    reference: np.ndarray,
    actual: np.ndarray,
    visualization: Mapping[str, Any],
) -> None:
    z_offset = float(visualization["z_offset_m"])
    reference_points = xyz_points(reference, z_offset)
    actual_points = xyz_points(actual, z_offset * 1.15)
    reference_color = rgba(visualization["reference_color_rgba"], "reference_color_rgba")
    actual_color = rgba(visualization["actual_color_rgba"], "actual_color_rgba")
    start_color = rgba(visualization["start_color_rgba"], "start_color_rgba")
    end_color = rgba(visualization["end_color_rgba"], "end_color_rgba")

    draw.clear_lines()
    draw.clear_points()
    draw_polyline(draw, reference_points, reference_color, float(visualization["reference_line_width"]))
    draw_polyline(draw, actual_points, actual_color, float(visualization["actual_line_width"]))

    stride = int(visualization["waypoint_point_stride"])
    if stride < 1:
        raise ValueError("waypoint_point_stride must be at least one")
    visible_reference = reference_points[::stride]
    draw.draw_points(
        visible_reference,
        [reference_color] * len(visible_reference),
        [float(visualization["waypoint_point_size"])] * len(visible_reference),
    )
    endpoint_points = [reference_points[0], reference_points[-1], actual_points[0], actual_points[-1]]
    endpoint_colors = [start_color, end_color, start_color, end_color]
    endpoint_size = float(visualization["endpoint_point_size"])
    draw.draw_points(endpoint_points, endpoint_colors, [endpoint_size] * len(endpoint_points))


def isaac_sim_version() -> str:
    root = Path(os.environ.get("ISAAC_SIM_ROOT", ""))
    version_file = root / "VERSION"
    if not version_file.is_file():
        raise RuntimeError(f"Isaac Sim VERSION file not found: {version_file}")
    return version_file.read_text(encoding="utf-8").strip()


def actual_segment_summaries(reference, actual: np.ndarray) -> list[dict[str, Any]]:
    """Measure actual motion over each commanded segment without interpolation."""

    summaries = []
    command_names = reference.segment_names[:-1]
    start = 0
    while start < len(command_names):
        name = command_names[start]
        end = start + 1
        while end < len(command_names) and command_names[end] == name:
            end += 1
        start_pose = actual[start]
        end_pose = actual[end]
        summaries.append(
            {
                "name": name,
                "start_sample_index": start,
                "end_sample_index": end,
                "commanded_linear_velocity_mps": float(reference.linear_velocity_mps[start]),
                "commanded_angular_velocity_rps": float(reference.angular_velocity_rps[start]),
                "actual_displacement_m": float(np.linalg.norm(end_pose[:2] - start_pose[:2])),
                "actual_yaw_change_rad": float(wrap_angle(end_pose[2] - start_pose[2])),
            }
        )
        start = end
    return summaries


def evaluate_motion_checks(
    segment_summaries: Sequence[Mapping[str, Any]],
    success_config: Mapping[str, Any],
) -> dict[str, Any]:
    min_straight = float(success_config["minimum_straight_segment_displacement_m"])
    min_turn_yaw = float(success_config["minimum_turn_segment_abs_yaw_change_rad"])
    if min_straight <= 0.0 or min_turn_yaw <= 0.0:
        raise ValueError("Stage 0 success thresholds must be greater than zero")
    straight_values = [
        float(segment["actual_displacement_m"])
        for segment in segment_summaries
        if abs(float(segment["commanded_linear_velocity_mps"])) > 0.0
        and math.isclose(float(segment["commanded_angular_velocity_rps"]), 0.0, abs_tol=1e-12)
    ]
    turn_values = [
        abs(float(segment["actual_yaw_change_rad"]))
        for segment in segment_summaries
        if abs(float(segment["commanded_angular_velocity_rps"])) > 0.0
    ]
    max_straight = max(straight_values, default=0.0)
    max_turn_yaw = max(turn_values, default=0.0)
    straight_passed = max_straight >= min_straight
    turn_passed = max_turn_yaw >= min_turn_yaw
    return {
        "passed": straight_passed and turn_passed,
        "straight_motion_passed": straight_passed,
        "turning_motion_passed": turn_passed,
        "maximum_straight_segment_displacement_m": max_straight,
        "maximum_turn_segment_abs_yaw_change_rad": max_turn_yaw,
        "minimum_straight_segment_displacement_m": min_straight,
        "minimum_turn_segment_abs_yaw_change_rad": min_turn_yaw,
    }


def git_commit_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"],
        text=True,
    ).strip()


def run() -> dict[str, Any]:
    config = load_config(ARGS.config)
    simulation = require_mapping(config, "simulation")
    robot_config = require_mapping(config, "robot")
    visualization = require_mapping(config, "visualization")
    output_config = require_mapping(config, "output")
    success_config = require_mapping(config, "success_checks")
    creation_time = datetime.now().astimezone().isoformat(timespec="seconds")
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_root = Path(str(output_config["root"]))
    if not output_root.is_absolute():
        output_root = REPOSITORY_ROOT / output_root
    output_dir = output_root / run_id
    if output_dir.exists():
        raise FileExistsError(f"Stage 0 output already exists and will not be overwritten: {output_dir}")
    physics_dt = finite_positive(simulation, "physics_dt")
    sample_dt = finite_positive(simulation, "sample_dt")
    sample_steps_float = sample_dt / physics_dt
    sample_steps = int(round(sample_steps_float))
    if sample_steps < 1 or not math.isclose(sample_steps_float, sample_steps, abs_tol=1e-9):
        raise ValueError("sample_dt must be an integer multiple of physics_dt")
    settling_duration = float(simulation["settling_duration_s"])
    settling_steps_float = settling_duration / physics_dt
    settling_steps = int(round(settling_steps_float))
    if settling_duration < 0.0 or not math.isclose(
        settling_steps_float,
        settling_steps,
        abs_tol=1e-9,
    ):
        raise ValueError("settling_duration_s must be non-negative and divisible by physics_dt")

    initial_pose = validate_pose_se2(simulation["initial_robot_pose_se2"], name="initial_robot_pose_se2")
    segments: tuple[MotionSegment, ...] = segments_from_config(config["motion_profile"])
    reference = generate_reference_trajectory(initial_pose, segments, sample_dt)

    assets_root = get_assets_root_path()
    collection_path = str(robot_config["asset_collection_path"]).rstrip("/")
    relative_path = str(robot_config["asset_relative_path"]).lstrip("/")
    asset_query_path = f"{collection_path}/{relative_path}"
    resolved_asset_path = resolve_asset_path(asset_query_path)
    if not resolved_asset_path:
        raise RuntimeError(f"official Jackal asset could not be resolved: {asset_query_path}")

    reference_prim_path = str(robot_config["reference_prim_path"])
    world = World(
        physics_dt=physics_dt,
        rendering_dt=physics_dt,
        stage_units_in_meters=1.0,
    )
    world.scene.add_default_ground_plane()
    add_reference_to_stage(resolved_asset_path, reference_prim_path)
    while is_stage_loading():
        SIMULATION_APP.update()

    articulation_root = find_articulation_root(reference_prim_path)
    spawn_height = float(simulation["spawn_height_m"])
    if not math.isfinite(spawn_height) or spawn_height < 0.0:
        raise ValueError("spawn_height_m must be finite and non-negative")
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="stage0_jackal",
            position=np.array([initial_pose[0], initial_pose[1], spawn_height]),
            orientation=quaternion_from_yaw(float(initial_pose[2])),
        )
    )
    world.reset()
    for _ in range(settling_steps):
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim GUI closed before Stage 0 settling completed")
        world.step(render=True)

    wheel_parameters = discover_wheels(robot, articulation_root)
    dof_names = list(robot.dof_names or [])
    print(f"[Stage 0] articulation prim path: {articulation_root}")
    print(f"[Stage 0] DOF count: {robot.num_dof}")
    print(f"[Stage 0] DOF names: {dof_names}")
    print(f"[Stage 0] wheel parameters: {json.dumps(wheel_parameters, sort_keys=True)}")

    set_camera_view(
        eye=[float(value) for value in visualization["camera_eye"]],
        target=[float(value) for value in visualization["camera_target"]],
        camera_prim_path="/OmniverseKit_Persp",
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    draw_trajectories(draw, reference.poses, np.asarray([se2_from_world_pose(robot)]), visualization)
    print(f"[Stage 0] REFERENCE START: {reference.poses[0].tolist()}")
    print(f"[Stage 0] REFERENCE END: {reference.poses[-1].tolist()}")

    controller = robot.get_articulation_controller()
    actual_poses = [se2_from_world_pose(robot)]
    actual_times = [0.0]
    execution_time_origin = float(world.current_time)
    for sample_index in range(reference.poses.shape[0] - 1):
        command = wheel_velocity_command(
            float(reference.linear_velocity_mps[sample_index]),
            float(reference.angular_velocity_rps[sample_index]),
            wheel_parameters,
            robot.num_dof,
        )
        controller.apply_action(ArticulationAction(joint_velocities=command))
        for _ in range(sample_steps):
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim GUI closed before Stage 0 execution completed")
            world.step(render=True)
        actual_poses.append(se2_from_world_pose(robot))
        actual_times.append(float(world.current_time) - execution_time_origin)
        draw_trajectories(
            draw,
            reference.poses,
            np.asarray(actual_poses, dtype=np.float64),
            visualization,
        )

    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    actual = np.asarray(actual_poses, dtype=np.float64)
    times = np.asarray(actual_times, dtype=np.float64)
    if not np.allclose(times, reference.times_s, rtol=0.0, atol=physics_dt * 1e-3):
        raise RuntimeError(
            "physics sample times diverged from reference sample times; no interpolation was applied"
        )
    recording = Stage0Recording(reference.poses, actual, times)
    metrics = compute_stage0_metrics(recording)
    segment_summaries = actual_segment_summaries(reference, actual)
    smoke_success_checks = evaluate_motion_checks(segment_summaries, success_config)
    if not smoke_success_checks["passed"]:
        raise RuntimeError(
            "Jackal motion smoke checks failed: " + json.dumps(smoke_success_checks, sort_keys=True)
        )

    metadata = {
        "stage": str(config["stage"]),
        "run_id": run_id,
        "creation_time": creation_time,
        "git_commit_sha": git_commit_sha(),
        "isaac_sim_version": isaac_sim_version(),
        "robot_asset": {
            "relative_path": relative_path,
            "asset_collection_path": collection_path,
            "assets_root_runtime": assets_root,
            "resolved_path": resolved_asset_path,
        },
        "robot_prim_path": articulation_root,
        "actual_dof_names": dof_names,
        "physics_dt": physics_dt,
        "sample_dt": sample_dt,
        "trajectory_convention": (
            "pose k and actual pose k are sampled together at execution-relative k*sample_dt; "
            "command k advances the interval to k+1; no interpolation"
        ),
        "pose_frame": "Isaac Sim world frame; canonical [world_x, world_y, world_yaw]",
        "yaw_convention": "radians about world +Z, counter-clockwise positive, wrapped [-pi, pi)",
        "motion_profile": [segment.to_dict() for segment in segments],
        "wheel_parameters": wheel_parameters,
        "wheel_command_relation": {
            "left_rad_s": "(v - omega * separation / 2) / radius",
            "right_rad_s": "(v + omega * separation / 2) / radius",
        },
        "visualization_height_m": float(visualization["z_offset_m"]),
        "visualization": {
            "reference": "DebugDraw connected line plus waypoint/start/end points",
            "actual": "DebugDraw accumulated connected line plus start/end points",
        },
        "smoke_metrics_not_research_results": metrics,
        "actual_motion_segments": segment_summaries,
        "smoke_success_checks": smoke_success_checks,
    }
    save_stage0_run(output_dir, recording, metadata)
    validation = validate_stage0_output(output_dir)
    validation["output_directory"] = str(output_dir)
    validation["articulation_prim_path"] = articulation_root
    validation["actual_dof_names"] = dof_names
    validation["viewport_visualization"] = {
        "reference_drawn": True,
        "actual_drawn": True,
        "debug_draw_only": True,
    }
    print(f"[Stage 0] ACTUAL START: {actual[0].tolist()}")
    print(f"[Stage 0] ACTUAL END: {actual[-1].tolist()}")
    print(f"[Stage 0] output validation: {json.dumps(validation, sort_keys=True)}")
    print("STAGE0_RESULT_JSON=" + json.dumps(validation, sort_keys=True))
    return validation


try:
    RESULT = run()
    if not ARGS.no_hold and not ARGS.headless:
        print("[Stage 0] GUI inspection ready. Close the Isaac Sim window to exit.")
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
finally:
    SIMULATION_APP.close()
