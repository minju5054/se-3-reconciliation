#!/usr/bin/env python3
"""Run Stage 0-B Jackal controller diagnostics in Isaac Sim GUI mode."""

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
        default=REPOSITORY_ROOT / "configs/stage0_jackal_controller_validation.yaml",
    )
    parser.add_argument("--session-id", help="explicit immutable session directory name")
    parser.add_argument(
        "--scenarios",
        nargs="+",
        choices=("straight", "rotate_left", "rotate_right", "arc", "composite"),
        default=("straight", "rotate_left", "rotate_right", "arc"),
    )
    parser.add_argument(
        "--controller",
        choices=("official", "custom", "closed_loop"),
        default="official",
        help="open-loop conversion implementation, or pose-feedback follower",
    )
    parser.add_argument("--no-hold", action="store_true", help="close after saving")
    parser.add_argument("--headless", action="store_true", help="diagnostic override only")
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
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from reconciliation.controller_validation import (
    ControllerTelemetry,
    WHEEL_LABELS,
    compute_controller_metrics,
    save_controller_run,
    save_session_summary,
    validate_controller_run,
)
from reconciliation.controllers.differential import (
    differential_wheel_speeds,
    map_wheel_speeds_by_side,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import (
    MotionSegment,
    generate_reference_trajectory,
    segments_from_config,
    validate_pose_se2,
)


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("controller validation config must contain an object")
    return value


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
    roots = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(reference)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one Jackal articulation root, found: {roots}")
    return roots[0]


def collision_cylinder_radius(stage, wheel_body_prim) -> float:
    radii = []
    for prim in Usd.PrimRange(wheel_body_prim):
        if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
            value = UsdGeom.Cylinder(prim).GetRadiusAttr().Get()
            if value is not None:
                radii.append(float(value))
    if len(radii) != 1 or not math.isfinite(radii[0]) or radii[0] <= 0.0:
        raise RuntimeError(f"cannot identify one collision radius below {wheel_body_prim.GetPath()}")
    return radii[0]


def discover_wheels(robot: SingleArticulation, articulation_root: str) -> dict[str, Any]:
    """Resolve four Jackal wheel DOFs and geometry without guessing joint names."""

    dof_names = list(robot.dof_names or [])
    if robot.num_dof != 4 or len(set(dof_names)) != 4:
        raise RuntimeError(f"expected four unique Jackal DOFs, got {dof_names}")
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(articulation_root)
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(root_prim)
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in dof_names
    }
    if set(joints) != set(dof_names):
        raise RuntimeError("runtime wheel DOFs did not match USD revolute joints")

    records = []
    for index, name in enumerate(dof_names):
        joint = UsdPhysics.RevoluteJoint(joints[name])
        local_position = joint.GetLocalPos0Attr().Get()
        targets = joint.GetBody1Rel().GetTargets()
        if local_position is None or len(targets) != 1:
            raise RuntimeError(f"wheel geometry is unavailable for {name}")
        body = stage.GetPrimAtPath(targets[0])
        records.append(
            {
                "dof_index": index,
                "dof_name": name,
                "joint_prim_path": str(joint.GetPrim().GetPath()),
                "wheel_body_prim_path": str(targets[0]),
                "longitudinal_position_m": float(local_position[0]),
                "lateral_position_m": float(local_position[1]),
                "side": "left" if float(local_position[1]) > 0.0 else "right",
                "axle": "front" if float(local_position[0]) > 0.0 else "rear",
                "radius_m": collision_cylinder_radius(stage, body),
            }
        )
    corners = [f"{record['axle']}_{record['side']}" for record in records]
    if set(corners) != set(WHEEL_LABELS):
        raise RuntimeError(f"runtime wheel geometry did not form four Jackal corners: {records}")
    radii = np.asarray([record["radius_m"] for record in records], dtype=np.float64)
    if not np.allclose(radii, radii[0], rtol=0.0, atol=1e-6):
        raise RuntimeError(f"wheel radii differ: {radii.tolist()}")
    left_y = [record["lateral_position_m"] for record in records if record["side"] == "left"]
    right_y = [record["lateral_position_m"] for record in records if record["side"] == "right"]
    separation = float(np.mean(left_y) - np.mean(right_y))
    if separation <= 0.0:
        raise RuntimeError(f"invalid wheel separation: {separation}")
    return {
        "radius_m": float(radii[0]),
        "separation_m": separation,
        "radius_source": "USD collision cylinder",
        "separation_source": "USD revolute-joint localPos0 lateral spacing",
        "wheels": records,
    }


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if hasattr(value, "pathString"):
        return str(value)
    try:
        return [_json_value(item) for item in value]
    except TypeError:
        return str(value)


def authored_attributes(prim, keywords: Sequence[str]) -> dict[str, Any]:
    result = {}
    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        if attribute.HasAuthoredValueOpinion() and any(word in name.lower() for word in keywords):
            result[name] = _json_value(attribute.Get())
    return result


def bound_physics_material(prim) -> dict[str, Any] | None:
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if not material:
        return None
    material_prim = material.GetPrim()
    return {
        "prim_path": str(material_prim.GetPath()),
        "attributes": authored_attributes(
            material_prim,
            ("friction", "restitution", "density", "material"),
        ),
    }


def inspect_articulation(articulation_root: str, wheels: Mapping[str, Any]) -> dict[str, Any]:
    """Record authored drive, effort, velocity, mass, inertia, and contact properties."""

    stage = omni.usd.get_context().get_stage()
    records = []
    keywords = (
        "drive",
        "damping",
        "stiffness",
        "force",
        "effort",
        "velocity",
        "mass",
        "inertia",
        "friction",
        "restitution",
        "collision",
    )
    for wheel in wheels["wheels"]:
        joint_prim = stage.GetPrimAtPath(wheel["joint_prim_path"])
        body_prim = stage.GetPrimAtPath(wheel["wheel_body_prim_path"])
        colliders = [
            prim
            for prim in Usd.PrimRange(body_prim)
            if prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        drive = UsdPhysics.DriveAPI(joint_prim, "angular")
        records.append(
            {
                "dof_name": wheel["dof_name"],
                "joint_prim_path": wheel["joint_prim_path"],
                "drive": {
                    "type": _json_value(drive.GetTypeAttr().Get()),
                    "target_velocity": _json_value(drive.GetTargetVelocityAttr().Get()),
                    "damping": _json_value(drive.GetDampingAttr().Get()),
                    "stiffness": _json_value(drive.GetStiffnessAttr().Get()),
                    "max_force": _json_value(drive.GetMaxForceAttr().Get()),
                },
                "joint_authored_attributes": authored_attributes(joint_prim, keywords),
                "wheel_body_authored_attributes": authored_attributes(body_prim, keywords),
                "collision_prims": [
                    {
                        "prim_path": str(prim.GetPath()),
                        "authored_attributes": authored_attributes(prim, keywords),
                        "bound_physics_material": bound_physics_material(prim),
                    }
                    for prim in colliders
                ],
            }
        )
    root_prim = stage.GetPrimAtPath(articulation_root)
    return {
        "articulation_root_authored_attributes": authored_attributes(root_prim, keywords),
        "wheel_records": records,
        "official_asset_modified": False,
        "runtime_overrides_applied": False,
    }


def primitive_reference(
    initial_pose: np.ndarray,
    scenario: str,
    config: Mapping[str, Any],
    sample_dt: float,
):
    timing = require_mapping(config, "primitive_timing")
    primitive = require_mapping(require_mapping(config, "primitives"), scenario)
    segments = (
        MotionSegment("initial_stop", float(timing["initial_stop_duration_s"]), 0.0, 0.0),
        MotionSegment(
            scenario,
            float(timing["active_duration_s"]),
            float(primitive["linear_velocity_mps"]),
            float(primitive["angular_velocity_rps"]),
        ),
        MotionSegment("final_stop", float(timing["final_stop_duration_s"]), 0.0, 0.0),
    )
    return generate_reference_trajectory(initial_pose, segments, sample_dt), segments


def composite_reference(
    initial_pose: np.ndarray,
    config: Mapping[str, Any],
    sample_dt: float,
):
    segments = segments_from_config(config["composite_motion_profile"])
    return generate_reference_trajectory(initial_pose, segments, sample_dt), segments


def follower_config(config: Mapping[str, Any]) -> FollowerConfig:
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


def official_comparison(radius: float, separation: float) -> list[dict[str, Any]]:
    controller = DifferentialController(wheel_radius=radius, wheel_base=separation)
    cases = (
        ("stop", 0.0, 0.0),
        ("straight", 0.5, 0.0),
        ("pure_rotation", 0.0, 0.25),
        ("arc", 0.4, 0.25),
        ("opposite_rotation", 0.0, -0.25),
    )
    rows = []
    for name, linear, angular in cases:
        custom = differential_wheel_speeds(linear, angular, radius, separation)
        official = controller.forward(np.asarray([linear, angular], dtype=np.float64))
        rows.append(
            {
                "case": name,
                "linear_velocity_mps": linear,
                "angular_velocity_rps": angular,
                "custom_left_right_rad_s": custom.tolist(),
                "official_left_right_rad_s": official.tolist(),
                "max_abs_difference_rad_s": float(np.max(np.abs(custom - official))),
            }
        )
    return rows


def rgba(values: Sequence[float]) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError("visualization colors must be finite RGBA")
    return result


def draw_trajectories(draw, reference, actual, visualization: Mapping[str, Any]) -> None:
    z = float(visualization["z_offset_m"])
    reference_points = [[float(p[0]), float(p[1]), z] for p in reference]
    actual_points = [[float(p[0]), float(p[1]), z * 1.15] for p in actual]
    draw.clear_lines()
    draw.clear_points()
    for points, color_key, width_key in (
        (reference_points, "reference_color_rgba", "reference_line_width"),
        (actual_points, "actual_color_rgba", "actual_line_width"),
    ):
        if len(points) > 1:
            draw.draw_lines(
                points[:-1],
                points[1:],
                [rgba(visualization[color_key])] * (len(points) - 1),
                [float(visualization[width_key])] * (len(points) - 1),
            )
    stride = int(visualization["heading_marker_stride"])
    marker_length = float(visualization["heading_marker_length_m"])
    heading_starts = reference_points[::stride]
    heading_ends = [
        [
            point[0] + marker_length * math.cos(float(reference[index, 2])),
            point[1] + marker_length * math.sin(float(reference[index, 2])),
            z,
        ]
        for point, index in zip(heading_starts, range(0, len(reference), stride), strict=True)
    ]
    if heading_starts:
        draw.draw_lines(
            heading_starts,
            heading_ends,
            [rgba(visualization["heading_color_rgba"])] * len(heading_starts),
            [float(visualization["heading_line_width"])] * len(heading_starts),
        )
    endpoints = [reference_points[0], reference_points[-1], actual_points[0], actual_points[-1]]
    colors = [rgba(visualization["heading_color_rgba"])] * len(endpoints)
    draw.draw_points(
        endpoints,
        colors,
        [float(visualization["endpoint_point_size"])] * len(endpoints),
    )


def canonical_wheel_values(runtime_values: np.ndarray, wheels: Mapping[str, Any]) -> np.ndarray:
    values = np.asarray(runtime_values, dtype=np.float64)
    by_corner = {
        f"{wheel['axle']}_{wheel['side']}": float(values[int(wheel["dof_index"])])
        for wheel in wheels["wheels"]
    }
    return np.asarray([by_corner[label] for label in WHEEL_LABELS], dtype=np.float64)


def canonical_wheel_names(wheels: Mapping[str, Any]) -> tuple[str, ...]:
    by_corner = {
        f"{wheel['axle']}_{wheel['side']}": str(wheel["dof_name"])
        for wheel in wheels["wheels"]
    }
    return tuple(by_corner[label] for label in WHEEL_LABELS)


def runtime_wheel_command(
    side_speeds: np.ndarray,
    wheels: Mapping[str, Any],
) -> np.ndarray:
    sides = [str(wheel["side"]) for wheel in wheels["wheels"]]
    return map_wheel_speeds_by_side(side_speeds, sides)


def run_scenario(
    *,
    world: World,
    robot: SingleArticulation,
    wheels: Mapping[str, Any],
    scenario: str,
    config: Mapping[str, Any],
    controller_kind: str,
    session_dir: Path,
    articulation_inspection: Mapping[str, Any],
    asset_metadata: Mapping[str, Any],
    articulation_root: str,
    initial_pose: np.ndarray,
    physics_dt: float,
    sample_dt: float,
    sample_steps: int,
    settling_steps: int,
    draw,
) -> dict[str, Any]:
    reference, segments = primitive_reference(initial_pose, scenario, config, sample_dt)
    visualization = require_mapping(config, "visualization")
    articulation_controller = robot.get_articulation_controller()
    official_controller = DifferentialController(
        wheel_radius=float(wheels["radius_m"]),
        wheel_base=float(wheels["separation_m"]),
    )

    world.reset()
    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )
    for _ in range(settling_steps):
        world.step(render=True)

    actual_poses = [se2_from_world_pose(robot)]
    times = [0.0]
    commanded_body = [[0.0, 0.0]]
    target_wheels = [np.zeros(4, dtype=np.float64)]
    actual_wheels = [canonical_wheel_values(robot.get_joint_velocities(), wheels)]
    origin = float(world.current_time)
    draw_trajectories(draw, reference.poses, np.asarray(actual_poses), visualization)

    for index in range(reference.poses.shape[0] - 1):
        linear = float(reference.linear_velocity_mps[index])
        angular = float(reference.angular_velocity_rps[index])
        if controller_kind == "official":
            side_speeds = official_controller.forward(np.asarray([linear, angular]))
        else:
            side_speeds = differential_wheel_speeds(
                linear,
                angular,
                float(wheels["radius_m"]),
                float(wheels["separation_m"]),
            )
        runtime_target = runtime_wheel_command(side_speeds, wheels)
        articulation_controller.apply_action(ArticulationAction(joint_velocities=runtime_target))
        for _ in range(sample_steps):
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim closed before controller validation completed")
            world.step(render=True)
        actual_poses.append(se2_from_world_pose(robot))
        times.append(float(world.current_time) - origin)
        commanded_body.append([linear, angular])
        target_wheels.append(canonical_wheel_values(runtime_target, wheels))
        actual_wheels.append(canonical_wheel_values(robot.get_joint_velocities(), wheels))
        draw_trajectories(
            draw,
            reference.poses,
            np.asarray(actual_poses, dtype=np.float64),
            visualization,
        )

    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )
    telemetry = ControllerTelemetry(
        reference.poses,
        np.asarray(actual_poses, dtype=np.float64),
        np.asarray(times, dtype=np.float64),
        np.asarray(commanded_body, dtype=np.float64),
        np.asarray(target_wheels, dtype=np.float64),
        np.asarray(actual_wheels, dtype=np.float64),
        np.arange(reference.poses.shape[0], dtype=np.int64),
        canonical_wheel_names(wheels),
    )
    if not np.allclose(telemetry.sim_times_s, reference.times_s, atol=physics_dt * 1e-3):
        raise RuntimeError("saved sample times diverged from configured reference timing")
    metrics = compute_controller_metrics(telemetry)
    commanded_linear = metrics["active_commanded_linear_velocity_mean_mps"]
    commanded_angular = metrics["active_commanded_angular_velocity_mean_rps"]
    measured_linear = metrics["active_measured_linear_velocity_mean_mps"]
    measured_angular = metrics["active_measured_angular_velocity_mean_rps"]
    if commanded_linear is not None and commanded_angular is not None:
        metrics["expected_kinematic_radius_m"] = abs(commanded_linear / commanded_angular)
    if measured_linear is not None and measured_angular is not None and abs(measured_angular) > 1e-9:
        metrics["measured_effective_radius_m"] = abs(measured_linear / measured_angular)
    metadata = {
        "stage": str(config["stage"]),
        "scenario": scenario,
        "controller": f"{controller_kind}_differential_open_loop",
        "creation_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit_sha": git_commit_sha(),
        "isaac_sim_version": isaac_sim_version(),
        "robot_asset": dict(asset_metadata),
        "robot_prim_path": articulation_root,
        "actual_dof_names": list(robot.dof_names or []),
        "canonical_wheel_column_order": list(WHEEL_LABELS),
        "physics_dt": physics_dt,
        "sample_dt": sample_dt,
        "trajectory_convention": (
            "reference and actual pose row k are at k*sample_dt; telemetry command row k>0 "
            "is the command applied over the interval ending at row k; row 0 is zero; no interpolation"
        ),
        "pose_frame": "Isaac Sim world; [world_x, world_y, world_yaw]",
        "yaw_convention": "world +Z, counter-clockwise positive, radians, wrapped [-pi, pi)",
        "motion_profile": [segment.to_dict() for segment in segments],
        "wheel_parameters": dict(wheels),
        "wheel_command_api": (
            "isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController"
            if controller_kind == "official"
            else "reconciliation.controllers.differential.differential_wheel_speeds"
        ),
        "four_wheel_mapping": "official [left,right] mapped by runtime USD joint geometry",
        "articulation_inspection": dict(articulation_inspection),
        "runtime_asset_overrides": {},
        "visualization_height_m": float(visualization["z_offset_m"]),
        "research_evidence": False,
    }
    destination = session_dir / scenario
    save_controller_run(destination, telemetry, metrics, metadata)
    validation = validate_controller_run(destination)
    print(f"[Stage 0-B] {scenario}: {json.dumps(metrics, sort_keys=True)}")
    return validation


def run_closed_loop_composite(
    *,
    world: World,
    robot: SingleArticulation,
    wheels: Mapping[str, Any],
    config: Mapping[str, Any],
    session_dir: Path,
    articulation_inspection: Mapping[str, Any],
    asset_metadata: Mapping[str, Any],
    articulation_root: str,
    initial_pose: np.ndarray,
    physics_dt: float,
    sample_dt: float,
    sample_steps: int,
    settling_steps: int,
    draw,
) -> dict[str, Any]:
    """Track the composite path using measured pose, not a time-advanced waypoint index."""

    reference, segments = composite_reference(initial_pose, config, sample_dt)
    settings = follower_config(config)
    closed_loop = require_mapping(config, "closed_loop")
    maximum_duration_s = finite_positive(closed_loop, "maximum_duration_s")
    maximum_samples_float = maximum_duration_s / sample_dt
    maximum_samples = int(round(maximum_samples_float))
    if not math.isclose(maximum_samples_float, maximum_samples, abs_tol=1e-9):
        raise ValueError("closed-loop maximum_duration_s must be divisible by sample_dt")
    visualization = require_mapping(config, "visualization")
    articulation_controller = robot.get_articulation_controller()
    differential_controller = DifferentialController(
        wheel_radius=float(wheels["radius_m"]),
        wheel_base=float(wheels["separation_m"]),
    )
    follower = TrajectoryFollower(reference.poses, settings)

    world.reset()
    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )
    for _ in range(settling_steps):
        world.step(render=True)

    current_pose = se2_from_world_pose(robot)
    command = follower.forward(current_pose)
    actual_poses = [current_pose]
    times = [0.0]
    commanded_body = [[0.0, 0.0]]
    target_wheels = [np.zeros(4, dtype=np.float64)]
    actual_wheels = [canonical_wheel_values(robot.get_joint_velocities(), wheels)]
    reference_indices = [command.nearest_index]
    origin = float(world.current_time)
    draw_trajectories(draw, reference.poses, np.asarray(actual_poses), visualization)

    for _ in range(maximum_samples):
        if command.goal_reached:
            break
        side_speeds = differential_controller.forward(
            np.asarray(
                [command.linear_velocity_mps, command.angular_velocity_rps],
                dtype=np.float64,
            )
        )
        runtime_target = runtime_wheel_command(side_speeds, wheels)
        articulation_controller.apply_action(ArticulationAction(joint_velocities=runtime_target))
        for _ in range(sample_steps):
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim closed before closed-loop validation completed")
            world.step(render=True)
        current_pose = se2_from_world_pose(robot)
        next_command = follower.forward(current_pose)
        actual_poses.append(current_pose)
        times.append(float(world.current_time) - origin)
        commanded_body.append([command.linear_velocity_mps, command.angular_velocity_rps])
        target_wheels.append(canonical_wheel_values(runtime_target, wheels))
        actual_wheels.append(canonical_wheel_values(robot.get_joint_velocities(), wheels))
        reference_indices.append(next_command.nearest_index)
        command = next_command
        draw_trajectories(
            draw,
            reference.poses,
            np.asarray(actual_poses, dtype=np.float64),
            visualization,
        )

    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )
    telemetry = ControllerTelemetry(
        reference.poses,
        np.asarray(actual_poses, dtype=np.float64),
        np.asarray(times, dtype=np.float64),
        np.asarray(commanded_body, dtype=np.float64),
        np.asarray(target_wheels, dtype=np.float64),
        np.asarray(actual_wheels, dtype=np.float64),
        np.asarray(reference_indices, dtype=np.int64),
        canonical_wheel_names(wheels),
    )
    metrics = compute_controller_metrics(telemetry)
    acceptance = require_mapping(config, "engineering_acceptance")
    acceptance_results = {
        key: metrics[key] < float(limit)
        for key, limit in (
            ("final_position_error_m", acceptance["final_position_error_m"]),
            ("position_rmse_m", acceptance["position_rmse_m"]),
            ("final_yaw_error_rad", acceptance["final_yaw_error_rad"]),
            ("yaw_rmse_rad", acceptance["yaw_rmse_rad"]),
        )
    }
    metrics["goal_reached"] = bool(command.goal_reached)
    metrics["maximum_duration_reached"] = not command.goal_reached
    metrics["engineering_acceptance"] = acceptance_results
    metrics["engineering_acceptance_passed"] = all(acceptance_results.values())
    metadata = {
        "stage": str(config["stage"]),
        "scenario": "composite",
        "controller": "closed_loop_nearest_lookahead",
        "creation_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit_sha": git_commit_sha(),
        "isaac_sim_version": isaac_sim_version(),
        "robot_asset": dict(asset_metadata),
        "robot_prim_path": articulation_root,
        "actual_dof_names": list(robot.dof_names or []),
        "canonical_wheel_column_order": list(WHEEL_LABELS),
        "physics_dt": physics_dt,
        "sample_dt": sample_dt,
        "trajectory_convention": (
            "reference is an immutable N x 3 path; actual row k is associated with the "
            "monotonic nearest reference_index chosen from pose feedback; reference and actual "
            "lengths may differ; no interpolation and no time-forced waypoint advancement"
        ),
        "pose_frame": "Isaac Sim world; [world_x, world_y, world_yaw]",
        "yaw_convention": "world +Z, counter-clockwise positive, radians, wrapped [-pi, pi)",
        "motion_profile": [segment.to_dict() for segment in segments],
        "follower": {
            key: getattr(settings, key) for key in settings.__dataclass_fields__
        },
        "maximum_duration_s": maximum_duration_s,
        "wheel_parameters": dict(wheels),
        "wheel_command_api": (
            "isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController"
        ),
        "four_wheel_mapping": "official [left,right] mapped by runtime USD joint geometry",
        "articulation_inspection": dict(articulation_inspection),
        "runtime_asset_overrides": {},
        "visualization_height_m": float(visualization["z_offset_m"]),
        "research_evidence": False,
    }
    destination = session_dir / "closed_loop"
    save_controller_run(destination, telemetry, metrics, metadata)
    validation = validate_controller_run(destination)
    print(f"[Stage 0-B] closed_loop: {json.dumps(metrics, sort_keys=True)}")
    return validation


def git_commit_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()


def isaac_sim_version() -> str:
    path = Path(os.environ.get("ISAAC_SIM_ROOT", "")) / "VERSION"
    if not path.is_file():
        raise RuntimeError(f"Isaac Sim VERSION file not found: {path}")
    return path.read_text(encoding="utf-8").strip()


def run() -> dict[str, Any]:
    config = load_config(ARGS.config)
    if ARGS.controller == "closed_loop" and tuple(ARGS.scenarios) != ("composite",):
        raise ValueError("--controller closed_loop requires --scenarios composite")
    if ARGS.controller != "closed_loop" and "composite" in ARGS.scenarios:
        raise ValueError("the composite scenario is reserved for --controller closed_loop")
    simulation = require_mapping(config, "simulation")
    robot_config = require_mapping(config, "robot")
    visualization = require_mapping(config, "visualization")
    output_config = require_mapping(config, "output")
    physics_dt = finite_positive(simulation, "physics_dt")
    sample_dt = finite_positive(simulation, "sample_dt")
    sample_steps_float = sample_dt / physics_dt
    sample_steps = int(round(sample_steps_float))
    if sample_steps < 1 or not math.isclose(sample_steps_float, sample_steps, abs_tol=1e-9):
        raise ValueError("sample_dt must be an integer multiple of physics_dt")
    settling_duration = float(simulation["settling_duration_s"])
    settling_steps = int(round(settling_duration / physics_dt))
    if settling_duration < 0.0 or not math.isclose(
        settling_duration / physics_dt, settling_steps, abs_tol=1e-9
    ):
        raise ValueError("settling duration must be non-negative and divisible by physics_dt")
    initial_pose = validate_pose_se2(simulation["initial_robot_pose_se2"])

    output_root = Path(str(output_config["root"]))
    if not output_root.is_absolute():
        output_root = REPOSITORY_ROOT / output_root
    session_id = ARGS.session_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_dir = output_root / session_id
    session_dir.mkdir(parents=True, exist_ok=False)

    collection = str(robot_config["asset_collection_path"]).rstrip("/")
    relative = str(robot_config["asset_relative_path"]).lstrip("/")
    query = f"{collection}/{relative}"
    resolved = resolve_asset_path(query)
    if not resolved:
        raise RuntimeError(f"official Jackal asset could not be resolved: {query}")
    asset_metadata = {
        "relative_path": relative,
        "asset_collection_path": collection,
        "assets_root_runtime": get_assets_root_path(),
        "resolved_path": resolved,
    }

    world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    reference_prim = str(robot_config["reference_prim_path"])
    add_reference_to_stage(resolved, reference_prim)
    while is_stage_loading():
        SIMULATION_APP.update()
    articulation_root = find_articulation_root(reference_prim)
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="stage0b_jackal",
            position=np.array(
                [initial_pose[0], initial_pose[1], float(simulation["spawn_height_m"])]
            ),
            orientation=quaternion_from_yaw(float(initial_pose[2])),
        )
    )
    world.reset()
    wheels = discover_wheels(robot, articulation_root)
    inspection = inspect_articulation(articulation_root, wheels)
    comparisons = official_comparison(float(wheels["radius_m"]), float(wheels["separation_m"]))
    print(f"[Stage 0-B] articulation prim: {articulation_root}")
    print(f"[Stage 0-B] DOF count: {robot.num_dof}")
    print(f"[Stage 0-B] DOF names: {list(robot.dof_names or [])}")
    print("[Stage 0-B] controller comparison: " + json.dumps(comparisons, sort_keys=True))
    print("[Stage 0-B] articulation inspection: " + json.dumps(inspection, sort_keys=True))

    set_camera_view(
        eye=[float(value) for value in visualization["camera_eye"]],
        target=[float(value) for value in visualization["camera_target"]],
        camera_prim_path="/OmniverseKit_Persp",
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    validations = []
    for scenario in ARGS.scenarios:
        if ARGS.controller == "closed_loop":
            validations.append(
                run_closed_loop_composite(
                    world=world,
                    robot=robot,
                    wheels=wheels,
                    config=config,
                    session_dir=session_dir,
                    articulation_inspection=inspection,
                    asset_metadata=asset_metadata,
                    articulation_root=articulation_root,
                    initial_pose=initial_pose,
                    physics_dt=physics_dt,
                    sample_dt=sample_dt,
                    sample_steps=sample_steps,
                    settling_steps=settling_steps,
                    draw=draw,
                )
            )
        else:
            validations.append(run_scenario(
                world=world,
                robot=robot,
                wheels=wheels,
                scenario=scenario,
                config=config,
                controller_kind=ARGS.controller,
                session_dir=session_dir,
                articulation_inspection=inspection,
                asset_metadata=asset_metadata,
                articulation_root=articulation_root,
                initial_pose=initial_pose,
                physics_dt=physics_dt,
                sample_dt=sample_dt,
                sample_steps=sample_steps,
                settling_steps=settling_steps,
                draw=draw,
            ))
    formula_equivalent = all(row["max_abs_difference_rad_s"] <= 1e-12 for row in comparisons)
    summary = {
        "stage": str(config["stage"]),
        "session_id": session_id,
        "creation_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "git_commit_sha": git_commit_sha(),
        "controller": (
            "closed_loop_nearest_lookahead"
            if ARGS.controller == "closed_loop"
            else f"{ARGS.controller}_differential_open_loop"
        ),
        "official_controller_api": (
            "isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController"
        ),
        "custom_vs_official": comparisons,
        "formula_equivalent_within_tolerance": formula_equivalent,
        "formula_conclusion": (
            "wheel conversion formula bug is not supported"
            if formula_equivalent
            else "custom and official wheel conversion differ"
        ),
        "runs": validations,
        "research_evidence": False,
    }
    save_session_summary(session_dir, summary)
    result = {"session_directory": str(session_dir), **summary}
    print("STAGE0B_RESULT_JSON=" + json.dumps(result, sort_keys=True))
    return result


try:
    RESULT = run()
    if not ARGS.no_hold and not ARGS.headless:
        print("[Stage 0-B] GUI inspection ready. Close the Isaac Sim window to exit.")
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
finally:
    SIMULATION_APP.close()
