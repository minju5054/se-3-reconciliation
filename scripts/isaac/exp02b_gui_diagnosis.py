#!/usr/bin/env python3
"""Diagnose one frozen EXP-02B branch with GUI geometry and execution telemetry."""

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
import time
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02b_gui_diagnosis.yaml",
    )
    parser.add_argument(
        "--case",
        choices=("case_high_delta_v", "case_high_delta_omega", "case_benign_delayed"),
        default="case_high_delta_omega",
    )
    parser.add_argument("--k", type=int, default=3)
    parser.add_argument(
        "--method",
        choices=("raw_f0", "raw_k", "pose_anchor", "rigid", "graph"),
        default="raw_k",
    )
    parser.add_argument("--exp02b-run", type=Path)
    parser.add_argument("--run-id", help="explicit immutable diagnostic output directory name")
    parser.add_argument("--real-time-factor", type=float)
    parser.add_argument("--pre-reset-hold-s", type=float)
    parser.add_argument("--no-hold", action="store_true", help="close after saving")
    parser.add_argument("--headless", action="store_true", help="automated diagnostic override")
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
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from debug_draw_trajectories import (
    draw_heading_markers,
    draw_polyline,
    draw_pose_points,
)
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
from reconciliation.controller_switch_metrics import controller_switch_metrics
from reconciliation.controller_validation import estimate_body_velocities
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02b import (
    METHODS,
    comparability_gate,
    load_source_case,
    verify_frozen_source_selection,
)
from reconciliation.exp02b_diagnosis import (
    DIAGNOSTIC_PHASES,
    DiagnosticTelemetry,
    execution_tracking_metrics,
    interpretation_status,
    save_diagnostic_output,
    spatial_reference_metrics,
    validate_diagnostic_output,
)
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_se2_trajectory


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def finite_nonnegative(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result


def finite_positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def source_command_rows(source_dir: Path) -> tuple[list[dict[str, str]], dict[str, str]]:
    with (source_dir / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    split = next(index for index, row in enumerate(rows) if row["reference_source"] == "FRESH")
    if split < 3:
        raise ValueError("source needs at least three OLD commands")
    return rows[:split], rows[split]


def follower_config(config: Mapping[str, Any]) -> FollowerConfig:
    values = config["closed_loop"]
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


def add_static_box(path: str, center, size, color) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    transform = UsdGeom.XformCommonAPI(cube)
    transform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    transform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def suppress_lidar_viewport_visualization(reference_path: str) -> dict[str, list[str]]:
    """Suppress lidar-only viewport clutter without touching articulation physics."""

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(reference_path)
    sensor_roots = [
        prim
        for prim in Usd.PrimRange(root)
        if "lidar" in prim.GetName().lower() or "laser" in prim.GetName().lower()
    ]
    hidden: list[str] = []
    disabled_sensors: list[str] = []
    for sensor_root in sensor_roots:
        for prim in Usd.PrimRange(sensor_root):
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
                hidden.append(str(prim.GetPath()))
            if prim.GetName().lower() == "lidar":
                disabled_sensors.append(str(prim.GetPath()))
                prim.SetActive(False)
    return {
        "hidden_imageable_prim_paths": sorted(set(hidden)),
        "disabled_sensor_prim_paths": sorted(set(disabled_sensors)),
    }


def phase_message(phase: str, *, sim_time_s: float, case_id: str, k: int, method: str) -> None:
    if phase not in DIAGNOSTIC_PHASES:
        raise ValueError(f"unknown diagnostic phase: {phase}")
    print(
        "EXP02B_DIAG_PHASE="
        + json.dumps(
            {
                "phase": phase,
                "simulation_time_s": float(sim_time_s),
                "case_id": case_id,
                "method": method,
                "k": k,
            },
            sort_keys=True,
        ),
        flush=True,
    )


def measured_interval_body(previous: np.ndarray, current: np.ndarray, dt: float) -> list[float]:
    delta = current[:2] - previous[:2]
    forward = np.array([math.cos(float(previous[2])), math.sin(float(previous[2]))])
    return [
        float(delta @ forward / dt),
        float(wrap_angle(current[2] - previous[2]) / dt),
    ]


def main() -> dict[str, Any]:
    diagnostic_started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    phase_sim_times_s: dict[str, float] = {}
    config_path = ARGS.config.resolve()
    diagnosis_config = load_yaml(config_path)
    if diagnosis_config.get("experiment") != "EXP-02B GUI Execution Diagnosis":
        raise ValueError("diagnostic config has the wrong experiment")
    paths = diagnosis_config["paths"]
    exp02b_config_path = resolve_path(paths["exp02b_config"])
    exp01b_config_path = resolve_path(paths["exp01b_config"])
    exp02b_config = load_yaml(exp02b_config_path)
    exp01b_config = load_yaml(exp01b_config_path)
    if ARGS.k not in exp02b_config["entry_indices"]:
        raise ValueError(f"k must be one of the frozen EXP-02B entries: {exp02b_config['entry_indices']}")
    if ARGS.method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}")
    exp02b_run = (
        ARGS.exp02b_run.resolve()
        if ARGS.exp02b_run is not None
        else resolve_path(paths["exp02b_run"])
    )
    source_selection_path = exp02b_run / "source_selection.json"
    source_selection = load_strict_json(source_selection_path)
    verify_frozen_source_selection(source_selection["source_root"], exp02b_config)
    source_record = source_selection["cases"][ARGS.case]
    source = load_source_case(ARGS.case, source_record["trial_directory"])
    selected_suffix = source.fresh_world[ARGS.k :].copy()
    candidate_path = exp02b_run / ARGS.case / f"k_{ARGS.k}" / ARGS.method / "candidate.npy"
    candidate = validate_se2_trajectory(
        np.load(candidate_path, allow_pickle=False), name="frozen EXP-02B candidate"
    )
    expected_shape = source.fresh_world.shape if ARGS.method == "raw_f0" else selected_suffix.shape
    if candidate.shape != expected_shape:
        raise ValueError("candidate shape differs from the frozen method contract")
    if ARGS.method == "raw_f0" and not np.array_equal(candidate, source.fresh_world):
        raise ValueError("raw_f0 candidate is not exact FRESH[0:]")
    if ARGS.method == "raw_k" and not np.array_equal(candidate, selected_suffix):
        raise ValueError("raw_k candidate is not exact FRESH[k:]")

    output_root = resolve_path(paths["output_root"])
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("exp02b-gui-%Y%m%dT%H%M%SZ")
    destination = output_root / run_id
    if destination.exists():
        raise FileExistsError(f"diagnostic output already exists: {destination}")

    execution = exp02b_config["execution"]
    playback = diagnosis_config["diagnostic_playback"]
    visualization = diagnosis_config["visualization"]
    colors = visualization["colors"]
    physics_dt = finite_positive(execution["physics_dt"], "execution.physics_dt")
    control_dt = finite_positive(execution["control_dt"], "execution.control_dt")
    control_steps = int(round(control_dt / physics_dt))
    if not math.isclose(control_steps * physics_dt, control_dt, abs_tol=1e-9):
        raise ValueError("control_dt must be an integer multiple of physics_dt")
    real_time_factor = finite_positive(
        ARGS.real_time_factor if ARGS.real_time_factor is not None else playback["real_time_factor"],
        "real_time_factor",
    )
    pre_reset_hold_s = finite_nonnegative(
        ARGS.pre_reset_hold_s
        if ARGS.pre_reset_hold_s is not None
        else playback["pre_reset_hold_wall_s"],
        "pre_reset_hold_s",
    )
    exact_reset_hold_s = finite_nonnegative(
        playback["exact_reset_hold_wall_s"], "exact_reset_hold_wall_s"
    )

    robot_config = exp01b_config["robot"]
    asset = resolve_jackal_asset(robot_config)
    world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Exp02BDiagnosisDomeLight")
    dome.CreateIntensityAttr(1000.0)
    for box in exp01b_config["scene"]["static_boxes"]:
        add_static_box(f"/World/{box['id']}", box["center"], box["size"], box["color"])
    reference_path = str(robot_config["reference_prim_path"])
    add_reference_to_stage(str(asset["resolved_path"]), reference_path)
    while is_stage_loading():
        SIMULATION_APP.update()
    lidar_visualization_override = suppress_lidar_viewport_visualization(reference_path)
    print(
        "[EXP-02B diagnosis] lidar viewport-only override: "
        + json.dumps(lidar_visualization_override, sort_keys=True),
        flush=True,
    )
    articulation_root = find_articulation_root(reference_path)
    initial = np.asarray(source.source_metadata["initial_pose_se2"], dtype=np.float64)
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="exp02b_diagnostic_jackal",
            position=np.array([0.0, 0.0, float(execution["spawn_height_m"])]),
        )
    )
    world.reset()
    wheels = discover_wheels(robot, articulation_root)
    differential = DifferentialController(
        wheel_radius=float(wheels["radius_m"]), wheel_base=float(wheels["separation_m"])
    )
    articulation_controller = robot.get_articulation_controller()
    draw = _debug_draw.acquire_debug_draw_interface()

    visible_xy = np.vstack((source.old_world[:, :2], source.fresh_world[:, :2], candidate[:, :2]))
    center = np.mean(np.vstack((np.min(visible_xy, axis=0), np.max(visible_xy, axis=0))), axis=0)
    camera_offset = np.asarray(visualization["camera_xy_offset_m"], dtype=np.float64)
    set_camera_view(
        eye=[
            float(center[0] + camera_offset[0]),
            float(center[1] + camera_offset[1]),
            float(visualization["camera_height_m"]),
        ],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )

    fresh_observation = np.asarray(
        source.source_metadata["robot_pose_at_fresh_observation"], dtype=np.float64
    )
    marker_poses: dict[str, np.ndarray] = {
        "FRESH_observation": fresh_observation,
        "B_saved": source.boundary_pose_world.copy(),
        "F_k": source.fresh_world[ARGS.k].copy(),
        "X_k": candidate[0].copy(),
    }
    old_actual: list[np.ndarray] = []
    post_actual: list[np.ndarray] = []
    base_z = float(visualization["base_z_m"])
    line_width = float(visualization["line_width"])

    def redraw() -> None:
        draw.clear_lines()
        draw.clear_points()
        for path, z_scale, key, width_scale in (
            (source.old_world, 1.00, "planned_old", 1.25),
            (source.fresh_world, 1.15, "raw_fresh", 0.75),
            (selected_suffix, 1.30, "selected_suffix", 1.00),
            (candidate, 1.45, "candidate", 1.35),
        ):
            draw_polyline(
                draw, path, z=base_z * z_scale, color=colors[key], width=line_width * width_scale
            )
        if len(old_actual) > 1:
            draw_polyline(
                draw,
                np.asarray(old_actual),
                z=base_z * 1.60,
                color=colors["old_actual"],
                width=line_width * 1.4,
            )
        if len(post_actual) > 1:
            draw_polyline(
                draw,
                np.asarray(post_actual),
                z=base_z * 1.75,
                color=colors["post_actual"],
                width=line_width * 1.4,
            )
        marker_style = {
            "FRESH_observation": ("fresh_observation", 2.2, 1.00),
            "B_saved": ("saved_boundary", 2.5, 1.45),
            "B_reproduced": ("reproduced_boundary", 2.8, 1.15),
            "B_exact_reset": ("exact_reset_boundary", 3.1, 0.85),
            "F_k": ("selected_entry", 2.0, 1.05),
            "X_k": ("candidate_entry", 2.3, 0.80),
        }
        for name, pose in marker_poses.items():
            color_key, z_scale, size_scale = marker_style[name]
            pose_array = np.asarray([pose])
            draw_pose_points(
                draw,
                pose_array,
                z=base_z * z_scale,
                color=colors[color_key],
                size=float(visualization["marker_size"]) * size_scale,
            )
            draw_heading_markers(
                draw,
                pose_array,
                z=base_z * z_scale,
                color=colors[color_key],
                width=float(visualization["heading_line_width"]),
                length_m=float(visualization["heading_length_m"]),
            )

    def paced_step() -> None:
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed before diagnosis completed")
        started = time.monotonic()
        world.step(render=not ARGS.headless)
        remaining = physics_dt / real_time_factor - (time.monotonic() - started)
        if remaining > 0.0:
            time.sleep(remaining)

    def hold_view(seconds: float) -> None:
        world.pause()
        try:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if not SIMULATION_APP.is_running():
                    raise RuntimeError("Isaac Sim closed during diagnostic inspection hold")
                SIMULATION_APP.update()
                time.sleep(0.01)
        finally:
            world.play()

    print("[EXP-02B diagnosis] terminal legend (logical markers remain separate even if coincident):")
    for key, description in (
        ("planned_old", "planned OLD world trajectory"),
        ("old_actual", "actual Jackal history during OLD command replay"),
        ("raw_fresh", "raw full FRESH world trajectory"),
        ("selected_suffix", "selected FRESH[k:] suffix"),
        ("candidate", f"current {ARGS.method} candidate"),
        ("post_actual", "actual Jackal history after exact reset and switch"),
    ):
        print(f"  {key}: RGBA={colors[key]} — {description}")
    for name, pose in marker_poses.items():
        print(f"  marker {name}: {pose.tolist()}")
    redraw()

    # Match the frozen quantitative replay setup exactly: reset the initialized
    # articulation, then explicitly restore pose and zero wheel state before settling.
    world.reset()
    robot.set_world_pose(
        position=np.array([initial[0], initial[1], float(execution["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(initial[2])),
    )
    robot.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float64))
    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )
    phase_sim_times_s["SETTLING"] = float(world.current_time)
    phase_message(
        "SETTLING",
        sim_time_s=phase_sim_times_s["SETTLING"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    settling_steps = int(round(float(execution["settling_duration_s"]) / physics_dt))
    for _ in range(settling_steps):
        paced_step()

    old_rows, fresh_first = source_command_rows(source.trial_directory)
    old_poses = [se2_from_world_pose(robot)]
    old_actual[:] = [old_poses[0]]
    old_times = [0.0]
    old_commands = [[0.0, 0.0]]
    old_targets = [np.zeros(4, dtype=np.float64)]
    old_measured_wheels = [canonical_wheel_values(robot.get_joint_velocities(), wheels)]
    old_command_indices = [-1]
    old_origin = float(world.current_time)
    phase_sim_times_s["OLD_REPLAY"] = old_origin
    phase_message(
        "OLD_REPLAY",
        sim_time_s=phase_sim_times_s["OLD_REPLAY"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    for command_index, row in enumerate(old_rows):
        next_time = (
            float(old_rows[command_index + 1]["sim_time_s"])
            if command_index + 1 < len(old_rows)
            else float(fresh_first["sim_time_s"])
        )
        duration = next_time - float(row["sim_time_s"])
        steps = int(round(duration / physics_dt))
        if steps < 1 or not math.isclose(steps * physics_dt, duration, abs_tol=1e-6):
            raise ValueError(f"source command interval is not physics-step aligned: {duration}")
        command = np.asarray(
            [float(row["v_command_mps"]), float(row["omega_command_rps"])], dtype=np.float64
        )
        runtime_target = runtime_wheel_command(differential.forward(command), wheels)
        canonical_target = canonical_wheel_values(runtime_target, wheels)
        articulation_controller.apply_action(
            ArticulationAction(joint_velocities=runtime_target)
        )
        interval_start_pose = old_poses[-1]
        interval_start_time = old_times[-1]
        for _ in range(steps):
            paced_step()
            pose = se2_from_world_pose(robot)
            old_poses.append(pose)
            old_actual.append(pose)
            old_times.append(float(world.current_time) - old_origin)
            old_commands.append(command.tolist())
            old_targets.append(canonical_target.copy())
            old_measured_wheels.append(
                canonical_wheel_values(robot.get_joint_velocities(), wheels)
            )
            old_command_indices.append(command_index)
        redraw()
        body = measured_interval_body(
            interval_start_pose, old_poses[-1], old_times[-1] - interval_start_time
        )
        print(
            "EXP02B_DIAG_TELEMETRY="
            + json.dumps(
                {
                    "phase": "OLD_REPLAY",
                    "simulation_time_s": float(world.current_time),
                    "case_id": ARGS.case,
                    "method": ARGS.method,
                    "k": ARGS.k,
                    "actual_se2": old_poses[-1].tolist(),
                    "commanded_v_mps": float(command[0]),
                    "commanded_omega_rps": float(command[1]),
                    "measured_body_v_mps": body[0],
                    "measured_body_omega_rps": body[1],
                    "wheel_targets_rad_s": canonical_target.tolist(),
                    "wheel_measured_rad_s": old_measured_wheels[-1].tolist(),
                },
                sort_keys=True,
            ),
            flush=True,
        )

    reproduced = old_poses[-1].copy()
    marker_poses["B_reproduced"] = reproduced.copy()
    redraw()
    gate_values = execution["comparability_gate"]
    replay_command = np.asarray(old_commands[-1], dtype=np.float64)
    gate = comparability_gate(
        source_boundary=source.boundary_pose_world,
        reproduced_boundary=reproduced,
        source_pre_switch_command=source.old_commands[-1],
        reproduced_pre_switch_command=replay_command,
        translation_tolerance_m=float(gate_values["translation_tolerance_m"]),
        yaw_tolerance_rad=float(gate_values["yaw_tolerance_rad"]),
        command_tolerance=float(gate_values["command_tolerance"]),
    )
    phase_sim_times_s["PRE_RESET_INSPECTION"] = float(world.current_time)
    phase_message(
        "PRE_RESET_INSPECTION",
        sim_time_s=phase_sim_times_s["PRE_RESET_INSPECTION"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    print(
        "[EXP-02B diagnosis] B_saved={} B_reproduced={} translation={:.9f} m yaw={:.9f} rad; "
        "simulation is held before reset for {:.2f} wall s".format(
            source.boundary_pose_world.tolist(),
            reproduced.tolist(),
            gate["translation_error_m"],
            gate["yaw_error_rad"],
            pre_reset_hold_s,
        ),
        flush=True,
    )
    if not gate["valid"]:
        raise RuntimeError(f"frozen pre-switch comparability gate failed: {gate}")
    hold_view(pre_reset_hold_s)

    phase_sim_times_s["EXACT_BOUNDARY_RESET"] = float(world.current_time)
    phase_message(
        "EXACT_BOUNDARY_RESET",
        sim_time_s=phase_sim_times_s["EXACT_BOUNDARY_RESET"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    robot.set_world_pose(
        position=np.array(
            [
                source.boundary_pose_world[0],
                source.boundary_pose_world[1],
                float(execution["spawn_height_m"]),
            ]
        ),
        orientation=quaternion_from_yaw(float(source.boundary_pose_world[2])),
    )
    restored_runtime_target = runtime_wheel_command(
        differential.forward(source.old_commands[-1]), wheels
    )
    robot.set_joint_velocities(restored_runtime_target)
    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=restored_runtime_target)
    )
    exact_reset_pose = se2_from_world_pose(robot)
    marker_poses["B_exact_reset"] = exact_reset_pose.copy()
    post_actual[:] = [exact_reset_pose.copy()]
    redraw()
    print(
        "[EXP-02B diagnosis] EXACT RESET occurred; OLD actual remains a separate cyan history, "
        "post-switch actual starts a new green history.",
        flush=True,
    )
    hold_view(exact_reset_hold_s)

    follower = TrajectoryFollower(candidate, follower_config(exp01b_config))
    post_poses = [exact_reset_pose.copy()]
    post_times = [0.0]
    post_commands = [source.old_commands[-1].copy()]
    post_targets = [canonical_wheel_values(restored_runtime_target, wheels)]
    post_measured_wheels = [canonical_wheel_values(robot.get_joint_velocities(), wheels)]
    post_command_indices = [-1]
    first_command = follower.forward(exact_reset_pose)
    post_progress: list[tuple[int, int, bool]] = [
        (first_command.nearest_index, first_command.target_index, first_command.goal_reached)
    ]
    control_commands: list[list[float]] = []
    post_origin = float(world.current_time)
    phase_sim_times_s["POST_SWITCH_EXECUTION"] = post_origin
    phase_message(
        "POST_SWITCH_EXECUTION",
        sim_time_s=phase_sim_times_s["POST_SWITCH_EXECUTION"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    command_count = int(round(float(execution["branch_duration_s"]) / control_dt)) + 1
    command = first_command
    for command_index in range(command_count):
        if command_index > 0:
            command = follower.forward(post_poses[-1])
        control_commands.append([command.linear_velocity_mps, command.angular_velocity_rps])
        print(
            "EXP02B_DIAG_TELEMETRY="
            + json.dumps(
                {
                    "phase": "POST_SWITCH_EXECUTION",
                    "simulation_time_s": float(world.current_time),
                    "case_id": ARGS.case,
                    "method": ARGS.method,
                    "k": ARGS.k,
                    "actual_se2": post_poses[-1].tolist(),
                    "commanded_v_mps": command.linear_velocity_mps,
                    "commanded_omega_rps": command.angular_velocity_rps,
                    "nearest_index": command.nearest_index,
                    "target_index": command.target_index,
                    "goal_reached": command.goal_reached,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        if command_index == command_count - 1 or command.goal_reached:
            break
        command_array = np.asarray(
            [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
        )
        runtime_target = runtime_wheel_command(differential.forward(command_array), wheels)
        canonical_target = canonical_wheel_values(runtime_target, wheels)
        articulation_controller.apply_action(
            ArticulationAction(joint_velocities=runtime_target)
        )
        for _ in range(control_steps):
            paced_step()
            pose = se2_from_world_pose(robot)
            post_poses.append(pose)
            post_actual.append(pose)
            post_times.append(float(world.current_time) - post_origin)
            post_commands.append(command_array.copy())
            post_targets.append(canonical_target.copy())
            post_measured_wheels.append(
                canonical_wheel_values(robot.get_joint_velocities(), wheels)
            )
            post_command_indices.append(command_index)
            post_progress.append(
                (command.nearest_index, command.target_index, command.goal_reached)
            )
        redraw()
    articulation_controller.apply_action(
        ArticulationAction(joint_velocities=np.zeros(robot.num_dof, dtype=np.float64))
    )

    old_telemetry = DiagnosticTelemetry(
        np.asarray(old_poses),
        np.asarray(old_times),
        np.asarray(old_commands),
        np.asarray(old_targets),
        np.asarray(old_measured_wheels),
    )
    post_telemetry = DiagnosticTelemetry(
        np.asarray(post_poses),
        np.asarray(post_times),
        np.asarray(post_commands),
        np.asarray(post_targets),
        np.asarray(post_measured_wheels),
    )
    old_spatial = spatial_reference_metrics(source.old_world, old_telemetry.actual_trajectory)
    old_execution = execution_tracking_metrics(old_telemetry)
    post_execution = execution_tracking_metrics(post_telemetry)
    post_control_array = np.asarray(control_commands, dtype=np.float64)
    window = min(
        int(execution["command_window"]), len(source.old_commands), len(post_control_array)
    )
    post_controller = controller_switch_metrics(
        source.old_commands,
        post_control_array,
        control_dt_s=control_dt,
        window_size=window,
    )
    post_controller["executed_command_count"] = len(control_commands)
    post_controller["goal_reached_within_branch"] = bool(command.goal_reached)

    reset_translation = float(np.linalg.norm(exact_reset_pose[:2] - reproduced[:2]))
    reset_yaw = abs(float(wrap_angle(exact_reset_pose[2] - reproduced[2])))
    status = interpretation_status(
        old_spatial=old_spatial,
        old_execution=old_execution,
        reset_translation_m=reset_translation,
        reset_yaw_rad=reset_yaw,
        thresholds=diagnosis_config["diagnostic_thresholds"],
    )
    hashes = {
        relative: sha256_file(source.trial_directory / relative)
        for relative in source_record["sha256"]
    }
    source_provenance = {
        "source_trial_sha256": hashes,
        "expected_source_trial_sha256": dict(source_record["sha256"]),
        "source_selection_json": str(source_selection_path),
        "source_selection_sha256": sha256_file(source_selection_path),
        "candidate_path": str(candidate_path),
        "candidate_sha256": sha256_file(candidate_path),
        "exp02b_config_path": str(exp02b_config_path),
        "exp02b_config_sha256": sha256_file(exp02b_config_path),
        "exp01b_config_path": str(exp01b_config_path),
        "exp01b_config_sha256": sha256_file(exp01b_config_path),
    }
    source_timing = source.source_attempt["timing"]
    source_event_timestamps = {
        "observation": {
            "sim_time_s": float(source_timing["observation_sim_time_s"]),
            "host_monotonic_ns": int(source_timing["request_host_monotonic_ns"]),
        },
        "model_ready": {
            "sim_time_s": float(source_timing["model_ready_sim_time_s"]),
            "host_monotonic_ns": int(source_timing["model_ready_host_monotonic_ns"]),
        },
        "execution_fresh_usable": {
            "sim_time_s": float(source_timing["fresh_usable_sim_time_s"]),
            "host_monotonic_ns": int(source_timing["fresh_usable_host_monotonic_ns"]),
        },
        "semantics": (
            "timestamps copied from the immutable redesigned EXP-01B source; host values use "
            "that run's monotonic clock and are not comparable with this diagnostic wall clock"
        ),
    }
    old_body = {
        key: old_execution[key]
        for key in (
            "evaluation_sample_count",
            "v_command_vs_measured_rmse_mps",
            "omega_command_vs_measured_rmse_rps",
            "commanded_v_mean_mps",
            "measured_v_mean_mps",
            "commanded_omega_mean_rps",
            "measured_omega_mean_rps",
            "body_motion_semantics",
        )
    }
    old_wheel = {
        key: old_execution[key]
        for key in (
            "evaluation_sample_count",
            "wheel_target_vs_measured_rmse_rad_s",
            "wheel_target_vs_measured_rmse_by_corner_rad_s",
            "wheel_motion_semantics",
        )
    }
    summary = {
        "experiment": "EXP-02B GUI execution diagnosis",
        "source_trial_path": str(source.trial_directory),
        "source_provenance": source_provenance,
        "case_id": ARGS.case,
        "entry_index": ARGS.k,
        "method": ARGS.method,
        "saved_boundary_se2": source.boundary_pose_world.tolist(),
        "reproduced_boundary_se2": reproduced.tolist(),
        "pre_reset_error": {
            "translation_m": gate["translation_error_m"],
            "yaw_rad": gate["yaw_error_rad"],
            "comparability_gate_valid": gate["valid"],
        },
        "exact_reset": {
            "occurred": True,
            "pose_after_reset_se2": exact_reset_pose.tolist(),
            "translation_jump_from_reproduced_m": reset_translation,
            "yaw_jump_from_reproduced_rad": reset_yaw,
            "last_old_wheel_target_restored": True,
        },
        "old_reference_spatial_metrics": old_spatial,
        "old_body_command_vs_measured_metrics": old_body,
        "old_wheel_target_vs_measured_metrics": old_wheel,
        "post_switch_controller_metrics": post_controller,
        "post_switch_execution_metrics": post_execution,
        "interpretation_status": status,
    }
    git_sha = subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()
    worktree_dirty = subprocess.run(
        ["git", "-C", str(REPOSITORY_ROOT), "diff", "--quiet"], check=False
    ).returncode != 0
    phase_sim_times_s["FINISHED"] = float(world.current_time)
    diagnostic_finished_at = datetime.now().astimezone().isoformat(timespec="seconds")
    metadata = {
        "experiment": "EXP-02B GUI execution diagnosis",
        "diagnostic_only": True,
        "algorithm_or_objective_changed": False,
        "creation_time": diagnostic_finished_at,
        "diagnostic_execution_timestamps": {
            "host_started_at": diagnostic_started_at,
            "host_finished_at": diagnostic_finished_at,
            "phase_sim_times_s": phase_sim_times_s,
        },
        "source_observation_readiness_execution_timestamps": source_event_timestamps,
        "git_commit_sha": git_sha,
        "git_worktree_dirty": worktree_dirty,
        "case_id": ARGS.case,
        "entry_index": ARGS.k,
        "method": ARGS.method,
        "source_provenance": source_provenance,
        "coordinate_frame": "Isaac Sim world SE(2): [world_x_m, world_y_m, yaw_CCW_about_+Z_rad]",
        "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
        "wheel_runtime_dof_names": list(canonical_wheel_names(wheels)),
        "wheel_parameters": wheels,
        "physics_dt_s": physics_dt,
        "control_dt_s": control_dt,
        "playback_real_time_factor": real_time_factor,
        "phase_sequence": list(DIAGNOSTIC_PHASES),
        "pre_reset_hold_wall_s": pre_reset_hold_s,
        "exact_reset_hold_wall_s": exact_reset_hold_s,
        "waypoint_timing": (
            "OLD/FRESH LightNav rows have no intrinsic timestamp; reference metrics are spatial"
        ),
        "telemetry_timing": (
            "row 0 is the pre-interval state; rows 1+ store the command applied over the "
            "physics interval ending at that row"
        ),
        "visualization": {
            "trajectory_geometry_backend": "Isaac DebugDraw",
            "lidar_viewport_override": lidar_visualization_override,
            "lidar_visibility_override_semantics": (
                "diagnostic visual clarity only; lidar imageables hidden and the lidar sensor "
                "prim disabled; no articulation, contact, controller, or command setting changed"
            ),
            "legend_rgba": dict(colors),
            "marker_poses": {name: pose.tolist() for name, pose in marker_poses.items()},
            "logical_marker_names": [
                "FRESH_observation",
                "B_saved",
                "B_reproduced",
                "B_exact_reset",
                "F_k",
                "X_k",
            ],
            "paths": {
                "planned_old": str(source.trial_directory / "derived/old_world.npy"),
                "raw_fresh": str(source.trial_directory / "derived/fresh_world.npy"),
                "selected_suffix": f"raw_fresh[{ARGS.k}:]",
                "candidate": str(candidate_path),
                "old_actual": "old_replay_actual.npy",
                "post_actual": "post_switch_actual.npy",
            },
            "labels": "terminal legend/marker names; DebugDraw supplies colored geometry",
        },
        "isaac_sim_version": (
            Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION"
        ).read_text(encoding="utf-8").strip(),
        "robot_asset": asset,
        "articulation_prim_path": articulation_root,
        "frozen_quantitative_output_modified": False,
    }
    save_diagnostic_output(
        destination,
        metadata=metadata,
        summary=summary,
        old_telemetry=old_telemetry,
        old_command_indices=old_command_indices,
        post_telemetry=post_telemetry,
        post_command_indices=post_command_indices,
        post_progress=post_progress,
    )
    validation = validate_diagnostic_output(destination)
    phase_message(
        "FINISHED",
        sim_time_s=phase_sim_times_s["FINISHED"],
        case_id=ARGS.case,
        k=ARGS.k,
        method=ARGS.method,
    )
    result = {
        "output_directory": str(destination),
        "validation": validation,
        "diagnosis_summary": summary,
    }
    print("EXP02B_GUI_DIAGNOSIS=" + json.dumps(result, sort_keys=True), flush=True)
    if not ARGS.no_hold and not ARGS.headless:
        print("[EXP-02B diagnosis] FINISHED; close the Isaac Sim window to exit.", flush=True)
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
    return result


try:
    RESULT = main()
finally:
    SIMULATION_APP.close()
