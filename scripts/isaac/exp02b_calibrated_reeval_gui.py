#!/usr/bin/env python3
"""Small GUI diagnosis for one frozen EXP-02B-R branch."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--case",
        choices=("case_high_delta_v", "case_high_delta_omega", "case_benign_delayed"),
        required=True,
    )
    parser.add_argument("--k", type=int, choices=(0, 3, 6), required=True)
    parser.add_argument("--method", choices=("raw_k", "rigid", "graph"), required=True)
    parser.add_argument("--real-time-factor", type=float)
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.usd
import yaml
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom, UsdPhysics

from closed_loop_execution_runtime import ClosedLoopRuntime
from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    canonical_wheel_values,
    quaternion_from_yaw,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.closed_loop_execution_validation import load_frozen_stage0d_candidate
from reconciliation.controllers.trajectory_follower import TrajectoryFollower
from reconciliation.exp02b import comparability_gate, load_source_case
from reconciliation.exp02b_calibrated_reeval import (
    body_interval_motion,
    first_command_invariant,
    follower_config,
    validate_reeval_config,
    verify_file_hash,
    write_json_exclusive,
)
from reconciliation.online_switch import load_strict_json


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
    return rows[:split], rows[split]


def hold(world, seconds: float) -> None:
    """Refresh the viewport without advancing the diagnostic physics state."""

    world.pause()
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim closed during EXP-02B-R GUI hold")
            SIMULATION_APP.update()
            time.sleep(0.02)
    finally:
        world.play()


def capture(output: Path) -> Path:
    path = output / "final_viewport.png"
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 10.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("EXP-02B-R viewport capture failed")
    return path


def main() -> None:
    run = ARGS.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    validate_reeval_config(config)
    preparation = load_strict_json(run / "metadata.json")
    verify_file_hash(
        run / "historical_metric_snapshot.json",
        preparation["historical_metric_snapshot_sha256"],
        "historical metric snapshot",
    )
    snapshot = load_strict_json(run / "historical_metric_snapshot.json")
    branch = next(
        row
        for row in snapshot["branches"]
        if row["case_id"] == ARGS.case
        and int(row["entry_index"]) == ARGS.k
        and row["method"] == ARGS.method
    )
    candidate_path = Path(branch["candidate_path"])
    verify_file_hash(candidate_path, branch["candidate_sha256"], "GUI candidate")
    candidate_trajectory = np.load(candidate_path, allow_pickle=False)
    historical_actual = np.load(branch["historical_actual_path"], allow_pickle=False)
    exp01b_config = load_yaml(resolve_path(config["paths"]["source_exp01b_config"]))
    exp02b_config = load_yaml(resolve_path(config["paths"]["historical_exp02b_config"]))
    source = load_source_case(ARGS.case, branch["source_trial_directory"])
    raw_fresh = source.fresh_world
    controller_candidate = load_frozen_stage0d_candidate(
        resolve_path(config["paths"]["stage0d_run"]),
        config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    factor = float(
        ARGS.real_time_factor
        if ARGS.real_time_factor is not None
        else config["gui"]["real_time_factor"]
    )
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError("real-time factor must be finite and positive")
    runtime = ClosedLoopRuntime(
        SIMULATION_APP,
        {"simulation": config["simulation"]},
        exp01b_config,
        controller_candidate,
        render=not ARGS.headless,
        real_time_factor=factor,
    )
    for box in exp01b_config["scene"]["static_boxes"]:
        add_static_box(f"/World/{box['id']}", box["center"], box["size"], box["color"])
    runtime.world.reset()
    visual = config["gui"]["visualization"]
    draw = _debug_draw.acquire_debug_draw_interface()
    calibrated_actual: list[np.ndarray] = []

    def redraw() -> None:
        draw.clear_lines()
        draw.clear_points()
        z = float(visual["z_offset_m"])
        width = float(visual["line_width"])
        draw_polyline(draw, raw_fresh, z=z, color=visual["fresh_context_color_rgba"], width=width * 0.7)
        draw_polyline(draw, candidate_trajectory, z=z + 0.04, color=visual["candidate_color_rgba"], width=width)
        draw_heading_markers(draw, candidate_trajectory, z=z + 0.04, color=visual["candidate_color_rgba"], width=2.0, length_m=0.12, stride=2)
        draw_polyline(draw, historical_actual, z=z + 0.08, color=visual["historical_actual_color_rgba"], width=width)
        if len(calibrated_actual) > 1:
            draw_polyline(draw, np.asarray(calibrated_actual), z=z + 0.12, color=visual["calibrated_actual_color_rgba"], width=width * 1.2)
        for pose, color, offset in (
            (source.boundary_pose_world, visual["boundary_color_rgba"], 0.18),
            (candidate_trajectory[0], visual["candidate_entry_color_rgba"], 0.22),
            (raw_fresh[ARGS.k], visual["raw_entry_color_rgba"], 0.26),
        ):
            draw_pose_points(draw, np.asarray([pose]), z=z + offset, color=color, size=float(visual["marker_size"]))
            draw_heading_markers(draw, np.asarray([pose]), z=z + offset, color=color, width=3.0, length_m=0.18)

    visible = np.vstack((raw_fresh[:, :2], candidate_trajectory[:, :2], historical_actual[:, :2]))
    center = (np.min(visible, axis=0) + np.max(visible, axis=0)) / 2.0
    extent = float(np.max(np.ptp(visible, axis=0)))
    set_camera_view(
        eye=[float(center[0]), float(center[1] - 0.3), max(float(visual["camera_height_m"]), 2.0 * extent + 1.5)],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )
    redraw()
    print(
        "[EXP-02B-R GUI] BLUE=candidate, GREEN=calibrated actual, ORANGE=historical nominal actual, "
        "GREY=raw full FRESH; YELLOW=B_saved, CYAN=candidate X_k, MAGENTA=raw F_k",
        flush=True,
    )
    print(f"EXP02B_R_GUI_PHASE=SETTLING case={ARGS.case} k={ARGS.k} method={ARGS.method}", flush=True)
    initial = np.asarray(source.source_metadata["initial_pose_se2"], dtype=np.float64)
    runtime.reset(initial, float(config["simulation"]["settling_duration_s"]))
    old_rows, fresh_first = source_command_rows(source.trial_directory)
    last_start = se2_from_world_pose(runtime.robot)
    last_duration = 0.0
    print("EXP02B_R_GUI_PHASE=OLD_REPLAY", flush=True)
    for index, row in enumerate(old_rows):
        next_time = float(old_rows[index + 1]["sim_time_s"]) if index + 1 < len(old_rows) else float(fresh_first["sim_time_s"])
        duration = next_time - float(row["sim_time_s"])
        steps = int(round(duration / runtime.physics_dt))
        desired_old = np.asarray([float(row["v_command_mps"]), float(row["omega_command_rps"])])
        target_runtime = runtime_wheel_command(runtime.differential.forward(desired_old), runtime.wheels)
        runtime.articulation.apply_action(ArticulationAction(joint_velocities=target_runtime))
        last_start = se2_from_world_pose(runtime.robot)
        last_duration = steps * runtime.physics_dt
        for _ in range(steps):
            runtime.step()
    reproduced = se2_from_world_pose(runtime.robot)
    old_measured = body_interval_motion(last_start, reproduced, last_duration)
    gate_values = exp02b_config["execution"]["comparability_gate"]
    gate = comparability_gate(
        source_boundary=source.boundary_pose_world,
        reproduced_boundary=reproduced,
        source_pre_switch_command=source.old_commands[-1],
        reproduced_pre_switch_command=desired_old,
        translation_tolerance_m=float(gate_values["translation_tolerance_m"]),
        yaw_tolerance_rad=float(gate_values["yaw_tolerance_rad"]),
        command_tolerance=float(gate_values["command_tolerance"]),
    )
    if not gate["valid"]:
        raise RuntimeError(f"GUI nominal replay comparability failed: {gate}")
    print("EXP02B_R_GUI_PHASE=EXACT_RESET", flush=True)
    runtime.robot.set_world_pose(
        position=np.asarray([*source.boundary_pose_world[:2], runtime.spawn_height]),
        orientation=quaternion_from_yaw(float(source.boundary_pose_world[2])),
    )
    restored_runtime = runtime_wheel_command(
        runtime.differential.forward(source.old_commands[-1]), runtime.wheels
    )
    runtime.robot.set_joint_velocities(restored_runtime)
    runtime.articulation.apply_action(ArticulationAction(joint_velocities=restored_runtime))
    exact = se2_from_world_pose(runtime.robot)
    if not np.allclose(exact, branch["historical_follower_effective_boundary_se2"], rtol=0.0, atol=1e-12):
        raise RuntimeError("GUI exact-reset follower state differs from historical EXP-02B")
    calibrated_actual.append(exact.copy())
    redraw()
    print("EXP02B_R_GUI_PHASE=CALIBRATED_CONTROLLER_RESET", flush=True)
    correction = runtime.correction()
    if correction.integral_error_rad != 0.0:
        raise RuntimeError("GUI calibrated controller integral was not reset")
    hold(runtime.world, float(config["gui"]["exact_reset_hold_wall_s"]))
    follower = TrajectoryFollower(candidate_trajectory, follower_config(exp01b_config["closed_loop"]))
    command = follower.forward(exact)
    invariant = first_command_invariant(
        branch["historical_controller_metrics"]["fresh_first_command_v_omega"],
        [command.linear_velocity_mps, command.angular_velocity_rps],
        tolerance=float(config["protocol"]["first_desired_invariant_tolerance"]),
    )
    if not invariant["passed"]:
        raise RuntimeError(f"PROTOCOL_INVARIANT_FAILURE in GUI: {invariant}")
    print("EXP02B_R_GUI_PHASE=POST_SWITCH_EXECUTION", flush=True)
    measured = old_measured.copy()
    telemetry_rows = []
    control_count = int(round(config["protocol"]["branch_duration_s"] / runtime.control_dt))
    for control_index in range(control_count):
        desired = np.asarray([command.linear_velocity_mps, command.angular_velocity_rps])
        executed, target_runtime, state = runtime.apply(desired, "calibrated", correction, float(measured[1]))
        start = calibrated_actual[-1].copy()
        for _ in range(runtime.physics_steps):
            runtime.step()
            calibrated_actual.append(se2_from_world_pose(runtime.robot))
        measured = body_interval_motion(start, calibrated_actual[-1], runtime.control_dt)
        command = follower.forward(calibrated_actual[-1])
        target = canonical_wheel_values(target_runtime, runtime.wheels)
        wheel_measured = canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
        row = {
            "case_id": ARGS.case,
            "entry_index": ARGS.k,
            "method": ARGS.method,
            "phase": "POST_SWITCH_EXECUTION",
            "control_index": control_index,
            "desired_v_omega": desired.tolist(),
            "executed_v_omega": executed.tolist(),
            "measured_v_omega": measured.tolist(),
            "desired_delta_from_old": (desired - source.old_commands[-1]).tolist(),
            "measured_delta_from_old": (measured - old_measured).tolist(),
            "nearest_index": command.nearest_index,
            "target_index": command.target_index,
            "goal_reached": command.goal_reached,
            "wheel_target_rad_s": target.tolist(),
            "wheel_measured_rad_s": wheel_measured.tolist(),
            **state,
        }
        telemetry_rows.append(row)
        print("EXP02B_R_GUI_TELEMETRY=" + json.dumps(row, sort_keys=True), flush=True)
        redraw()
    runtime.stop()
    runtime.world.pause()
    print("EXP02B_R_GUI_PHASE=FINISHED", flush=True)
    output = run / "gui_metadata" / datetime.now(timezone.utc).strftime(
        f"{ARGS.case}-k{ARGS.k}-{ARGS.method}-%Y%m%dT%H%M%SZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    np.save(output / "candidate.npy", candidate_trajectory)
    np.save(output / "historical_nominal_actual.npy", historical_actual)
    np.save(output / "calibrated_actual.npy", np.asarray(calibrated_actual))
    np.save(output / "raw_full_fresh.npy", raw_fresh)
    viewport = capture(output) if not ARGS.headless else None
    write_json_exclusive(
        output / "metadata.json",
        {
            "experiment": "EXP-02B-R GUI",
            "diagnostic_only": True,
            "case_id": ARGS.case,
            "entry_index": ARGS.k,
            "method": ARGS.method,
            "candidate_sha256": branch["candidate_sha256"],
            "candidate_regenerated": False,
            "saved_boundary_se2": source.boundary_pose_world.tolist(),
            "reproduced_boundary_se2": reproduced.tolist(),
            "exact_reset_boundary_se2": exact.tolist(),
            "comparability_gate": gate,
            "first_desired_invariant": invariant,
            "calibrated_controller_state_at_switch": "RESET",
            "old_final_measured_v_omega": old_measured.tolist(),
            "phases": ["SETTLING", "OLD_REPLAY", "EXACT_RESET", "CALIBRATED_CONTROLLER_RESET", "POST_SWITCH_EXECUTION", "FINISHED"],
            "legend": {
                "blue": "same frozen candidate trajectory",
                "green": "calibrated actual post-switch",
                "orange": "historical nominal post-switch actual",
                "grey": "raw full FRESH context",
                "yellow_marker": "B_saved",
                "cyan_marker": "candidate first pose X_k",
                "magenta_marker": "raw F_k",
            },
            "telemetry": telemetry_rows,
            "real_time_factor": factor,
            "physics_paused_during_exact_reset_hold": True,
            "viewport_capture": None if viewport is None else str(viewport),
        },
    )
    print(f"EXP02B_R_GUI_OUTPUT={output}", flush=True)
    if not ARGS.no_hold and not ARGS.headless and bool(config["gui"]["final_hold"]):
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            time.sleep(0.02)


try:
    main()
finally:
    SIMULATION_APP.close()
