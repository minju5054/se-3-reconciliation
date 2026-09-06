#!/usr/bin/env python3
"""Replay frozen OLD commands and execute every EXP-02B candidate branch in Isaac."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp02b_controller_aware.yaml",
    )
    parser.add_argument(
        "--qualification",
        action="store_true",
        help="measure deterministic OLD replay noise only; do not execute branches",
    )
    parser.add_argument("--smoke", action="store_true", help="turning k=3 raw/rigid/graph only")
    parser.add_argument("--gui", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": not ARGS.gui})

import numpy as np
import omni.usd
import yaml
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from lightnav_stage0c_runtime import (
    discover_wheels,
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.controller_switch_metrics import controller_switch_metrics
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02b import (
    CASE_IDS,
    METHODS,
    comparability_gate,
    controller_improvement,
    load_source_case,
)
from reconciliation.online_switch import load_strict_json, save_json_exclusive, save_npy_exclusive


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def isaac_sim_version() -> str:
    version_file = Path(os.environ.get("ISAAC_SIM_ROOT", "")) / "VERSION"
    if not version_file.is_file():
        raise RuntimeError(f"Isaac Sim VERSION file not found: {version_file}")
    return version_file.read_text(encoding="utf-8").strip()


def add_static_box(path: str, center, size, color) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    transform = UsdGeom.XformCommonAPI(cube)
    transform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    transform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


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


def source_command_rows(source_dir: Path) -> tuple[list[dict[str, str]], dict[str, str]]:
    with (source_dir / "derived/controller_commands.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    split = next(index for index, row in enumerate(rows) if row["reference_source"] == "FRESH")
    if split < 3:
        raise ValueError("source needs at least three OLD commands")
    return rows[:split], rows[split]


def replay_old(
    *,
    world: World,
    robot: SingleArticulation,
    controller,
    differential,
    wheels: Mapping[str, Any],
    source,
    exp01b_config: Mapping[str, Any],
    execution: Mapping[str, Any],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    physics_dt = float(execution["physics_dt"])
    initial = np.asarray(source.source_metadata["initial_pose_se2"], dtype=np.float64)
    world.reset()
    robot.set_world_pose(
        position=np.array([initial[0], initial[1], float(execution["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(initial[2])),
    )
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    settle_steps = int(round(float(execution["settling_duration_s"]) / physics_dt))
    for _ in range(settle_steps):
        world.step(render=ARGS.gui)

    old_rows, fresh_first = source_command_rows(source.trial_directory)
    replayed_steps = 0
    for index, row in enumerate(old_rows):
        next_time = float(old_rows[index + 1]["sim_time_s"]) if index + 1 < len(old_rows) else float(
            fresh_first["sim_time_s"]
        )
        duration = next_time - float(row["sim_time_s"])
        steps = int(round(duration / physics_dt))
        if steps < 1 or not math.isclose(steps * physics_dt, duration, abs_tol=1e-6):
            raise ValueError(f"source command interval is not physics-step aligned: {duration}")
        command = np.array(
            [float(row["v_command_mps"]), float(row["omega_command_rps"])], dtype=np.float64
        )
        side = differential.forward(command)
        controller.apply_action(
            ArticulationAction(joint_velocities=runtime_wheel_command(side, wheels))
        )
        for _ in range(steps):
            world.step(render=ARGS.gui)
        replayed_steps += steps
    reproduced = se2_from_world_pose(robot)
    pre_switch = np.asarray(
        [float(old_rows[-1]["v_command_mps"]), float(old_rows[-1]["omega_command_rps"])],
        dtype=np.float64,
    )
    return reproduced, pre_switch, {
        "old_command_count": len(old_rows),
        "physics_step_count": replayed_steps,
        "source_switch_sim_time_s": float(fresh_first["sim_time_s"]),
        "source_first_old_sim_time_s": float(old_rows[0]["sim_time_s"]),
        "replay_duration_s": replayed_steps * physics_dt,
        "scene_id": source.source_metadata["scene_id"],
        "controller_config": dict(exp01b_config["closed_loop"]),
    }


def exact_boundary_reset(
    *,
    robot: SingleArticulation,
    controller,
    differential,
    wheels: Mapping[str, Any],
    boundary: np.ndarray,
    pre_switch_command: np.ndarray,
    spawn_height_m: float,
) -> None:
    """Remove replay pose scatter only after the comparability gate has passed."""

    robot.set_world_pose(
        position=np.array([boundary[0], boundary[1], spawn_height_m]),
        orientation=quaternion_from_yaw(float(boundary[2])),
    )
    side = differential.forward(pre_switch_command)
    runtime = runtime_wheel_command(side, wheels)
    robot.set_joint_velocities(runtime)
    controller.apply_action(ArticulationAction(joint_velocities=runtime))


def execute_branch(
    *,
    candidate: np.ndarray,
    world: World,
    robot: SingleArticulation,
    controller,
    differential,
    wheels: Mapping[str, Any],
    follower_values: Mapping[str, Any],
    execution: Mapping[str, Any],
    source_old_commands: np.ndarray,
) -> tuple[np.ndarray, list[dict[str, Any]], dict[str, Any]]:
    follower = TrajectoryFollower(candidate, follower_config({"closed_loop": follower_values}))
    physics_dt = float(execution["physics_dt"])
    control_dt = float(execution["control_dt"])
    control_steps = int(round(control_dt / physics_dt))
    command_count = int(round(float(execution["branch_duration_s"]) / control_dt)) + 1
    actual: list[np.ndarray] = []
    rows: list[dict[str, Any]] = []
    for index in range(command_count):
        pose = se2_from_world_pose(robot)
        actual.append(pose.copy())
        command = follower.forward(pose)
        rows.append(
            {
                "command_index": index,
                "time_from_switch_s": index * control_dt,
                "v_command_mps": float(command.linear_velocity_mps),
                "omega_command_rps": float(command.angular_velocity_rps),
                "nearest_index": int(command.nearest_index),
                "target_index": int(command.target_index),
                "goal_reached": bool(command.goal_reached),
                "actual_x": float(pose[0]),
                "actual_y": float(pose[1]),
                "actual_yaw": float(pose[2]),
            }
        )
        if index == command_count - 1 or command.goal_reached:
            break
        side = differential.forward(
            np.array([command.linear_velocity_mps, command.angular_velocity_rps])
        )
        controller.apply_action(
            ArticulationAction(joint_velocities=runtime_wheel_command(side, wheels))
        )
        for _ in range(control_steps):
            world.step(render=ARGS.gui)
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    fresh_commands = np.asarray(
        [[row["v_command_mps"], row["omega_command_rps"]] for row in rows], dtype=np.float64
    )
    window = min(int(execution["command_window"]), len(source_old_commands), len(fresh_commands))
    metrics = controller_switch_metrics(
        source_old_commands,
        fresh_commands,
        control_dt_s=control_dt,
        window_size=window,
    )
    metrics["executed_command_count"] = len(rows)
    metrics["goal_reached_within_branch"] = bool(rows[-1]["goal_reached"])
    return np.asarray(actual), rows, metrics


def main() -> None:
    run_dir = ARGS.run_dir.resolve()
    config = load_yaml(ARGS.config.resolve())
    exp01b_config = load_yaml(resolve_path(config["paths"]["exp01b_config"]))
    execution = config["execution"]
    source_selection = load_strict_json(run_dir / "source_selection.json")

    physics_dt = float(execution["physics_dt"])
    robot_config = exp01b_config["robot"]
    asset = resolve_jackal_asset(robot_config)
    world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Exp02BDomeLight")
    dome.CreateIntensityAttr(1000.0)
    for box in exp01b_config["scene"]["static_boxes"]:
        add_static_box(f"/World/{box['id']}", box["center"], box["size"], box["color"])
    reference_path = str(robot_config["reference_prim_path"])
    add_reference_to_stage(str(asset["resolved_path"]), reference_path)
    while is_stage_loading():
        SIMULATION_APP.update()
    articulation_root = find_articulation_root(reference_path)
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="exp02b_jackal",
            position=np.array([0.0, 0.0, float(execution["spawn_height_m"])]),
        )
    )
    world.reset()
    wheels = discover_wheels(robot, articulation_root)
    differential = DifferentialController(
        wheel_radius=float(wheels["radius_m"]), wheel_base=float(wheels["separation_m"])
    )
    controller = robot.get_articulation_controller()

    if ARGS.qualification:
        case_id = "case_high_delta_omega"
        source = load_source_case(case_id, source_selection["cases"][case_id]["trial_directory"])
        records = []
        for repetition in range(int(execution["comparability_gate"]["qualification_repetitions"])):
            reproduced, pre_switch, replay = replay_old(
                world=world,
                robot=robot,
                controller=controller,
                differential=differential,
                wheels=wheels,
                source=source,
                exp01b_config=exp01b_config,
                execution=execution,
            )
            translation = float(np.linalg.norm(reproduced[:2] - source.boundary_pose_world[:2]))
            yaw = abs(float(((reproduced[2] - source.boundary_pose_world[2] + np.pi) % (2*np.pi)) - np.pi))
            records.append(
                {
                    "repetition": repetition,
                    "source_boundary": source.boundary_pose_world.tolist(),
                    "reproduced_boundary": reproduced.tolist(),
                    "translation_error_m": translation,
                    "yaw_error_rad": yaw,
                    "pre_switch_command": pre_switch.tolist(),
                    "replay": replay,
                }
            )
        recommendation = {
            "translation_tolerance_m": max(row["translation_error_m"] for row in records)
            + float(execution["comparability_gate"]["margin_translation_m"]),
            "yaw_tolerance_rad": max(row["yaw_error_rad"] for row in records)
            + float(execution["comparability_gate"]["margin_yaw_rad"]),
            "basis": "maximum of three deterministic turning-case OLD replays plus frozen margin",
        }
        save_json_exclusive(
            run_dir / "comparability_qualification.json",
            {"case_id": case_id, "records": records, "recommended_gate": recommendation},
        )
        print("EXP02B_COMPARABILITY=" + json.dumps(recommendation, sort_keys=True))
        return

    gate_values = execution["comparability_gate"]
    if gate_values.get("frozen_after_qualification") is not True:
        raise ValueError("comparability gate must be frozen after qualification before execution")
    if gate_values.get("translation_tolerance_m") is None or gate_values.get("yaw_tolerance_rad") is None:
        raise ValueError("comparability tolerances are missing")

    plan = []
    for case_id in CASE_IDS:
        for k in config["entry_indices"]:
            for method in METHODS:
                if ARGS.smoke and not (
                    case_id == "case_high_delta_omega"
                    and int(k) == 3
                    and method in ("raw_k", "rigid", "graph")
                ):
                    continue
                plan.append((case_id, int(k), method))
    branch_summaries = []
    for case_id, k, method in plan:
        source = load_source_case(case_id, source_selection["cases"][case_id]["trial_directory"])
        reproduced, replay_command, replay = replay_old(
            world=world,
            robot=robot,
            controller=controller,
            differential=differential,
            wheels=wheels,
            source=source,
            exp01b_config=exp01b_config,
            execution=execution,
        )
        gate = comparability_gate(
            source_boundary=source.boundary_pose_world,
            reproduced_boundary=reproduced,
            source_pre_switch_command=source.old_commands[-1],
            reproduced_pre_switch_command=replay_command,
            translation_tolerance_m=float(gate_values["translation_tolerance_m"]),
            yaw_tolerance_rad=float(gate_values["yaw_tolerance_rad"]),
            command_tolerance=float(gate_values["command_tolerance"]),
        )
        gate["replay"] = replay
        if not gate["valid"]:
            raise RuntimeError(f"pre-switch comparability failed: {case_id}/k{k}/{method}: {gate}")
        exact_boundary_reset(
            robot=robot,
            controller=controller,
            differential=differential,
            wheels=wheels,
            boundary=source.boundary_pose_world,
            pre_switch_command=source.old_commands[-1],
            spawn_height_m=float(execution["spawn_height_m"]),
        )
        candidate = np.load(
            run_dir / case_id / f"k_{k}" / method / "candidate.npy", allow_pickle=False
        )
        actual, command_rows, metrics = execute_branch(
            candidate=candidate,
            world=world,
            robot=robot,
            controller=controller,
            differential=differential,
            wheels=wheels,
            follower_values=exp01b_config["closed_loop"],
            execution=execution,
            source_old_commands=source.old_commands,
        )
        branch_root = run_dir / case_id / f"k_{k}" / method / "execution"
        branch_root.mkdir(parents=True, exist_ok=False)
        save_npy_exclusive(branch_root / "actual_trajectory.npy", actual)
        with (branch_root / "controller_commands.csv").open(
            "x", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(command_rows[0]))
            writer.writeheader()
            writer.writerows(command_rows)
        save_json_exclusive(branch_root / "controller_metrics.json", metrics)
        save_json_exclusive(branch_root / "comparability.json", gate)
        branch_summaries.append(
            {
                "case_id": case_id,
                "entry_index": k,
                "method": method,
                "execution_valid": True,
                "comparability": gate,
                "controller_metrics": metrics,
            }
        )
        print(
            "EXP02B_BRANCH="
            + json.dumps(
                {
                    "case": case_id,
                    "k": k,
                    "method": method,
                    "delta_v": metrics["delta_v_abs_mps"],
                    "delta_omega": metrics["delta_omega_abs_rps"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

    if ARGS.smoke:
        save_json_exclusive(
            run_dir / "execution_smoke_summary.json",
            {"experiment": "EXP-02B", "branch_count": len(branch_summaries), "branches": branch_summaries},
        )
    else:
        by_key = {(row["case_id"], row["entry_index"], row["method"]): row for row in branch_summaries}
        epsilon = float(config["metrics"]["improvement_denominator_epsilon"])
        for row in branch_summaries:
            if row["method"] == "raw_f0":
                row["same_k_improvement"] = {"delta_v": None, "delta_omega": None}
                continue
            raw = by_key[(row["case_id"], row["entry_index"], "raw_k")]["controller_metrics"]
            metrics = row["controller_metrics"]
            row["same_k_improvement"] = {
                "delta_v": controller_improvement(
                    metrics["delta_v_abs_mps"], raw["delta_v_abs_mps"], epsilon=epsilon
                ),
                "delta_omega": controller_improvement(
                    metrics["delta_omega_abs_rps"], raw["delta_omega_abs_rps"], epsilon=epsilon
                ),
            }
        save_json_exclusive(
            run_dir / "execution_summary.json",
            {
                "experiment": "EXP-02B",
                "branch_count": len(branch_summaries),
                "all_branches_valid": all(row["execution_valid"] for row in branch_summaries),
                "runtime": {
                    "isaac_sim_version": isaac_sim_version(),
                    "headless": not ARGS.gui,
                    "robot_asset": asset,
                    "articulation_prim_path": articulation_root,
                    "dof_count": len(robot.dof_names or []),
                    "dof_names": list(robot.dof_names or []),
                    "controller_api": (
                        "isaacsim.robot.experimental.wheeled_robots.controllers."
                        "DifferentialController"
                    ),
                    "wheel_radius_m": float(wheels["radius_m"]),
                    "wheel_separation_m": float(wheels["separation_m"]),
                    "physics_dt_s": physics_dt,
                    "control_dt_s": float(execution["control_dt"]),
                },
                "comparability_gate": dict(gate_values),
                "state_reset_policy": (
                    "replay source OLD and pass frozen gate, then reset pose exactly to saved B and "
                    "restore last OLD wheel target before each post-switch branch"
                ),
                "branches": branch_summaries,
            },
        )
        print("EXP02B_EXECUTION_BRANCH_COUNT=" + str(len(branch_summaries)))


try:
    main()
finally:
    SIMULATION_APP.close()
