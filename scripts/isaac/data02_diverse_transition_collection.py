#!/usr/bin/env python3
"""Collect real same-episode successive LightNav OLD/FRESH pairs for DATA-02."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--phase", choices=("qualification", "primary", "smoke"), required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/data02_diverse_lightnav_transitions.yaml",
    )
    parser.add_argument("--scenario", default="all")
    parser.add_argument("--max-attempts", type=int)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": not ARGS.gui})

import numpy as np
import omni.replicator.core as rep
import omni.usd
from PIL import Image
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from closed_loop_execution_runtime import ClosedLoopRuntime, follower_config
from debug_draw_trajectories import draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    canonical_wheel_names,
    canonical_wheel_values,
    load_config,
    se2_from_world_pose,
)
from reconciliation.closed_loop_execution_validation import load_frozen_stage0d_candidate
from reconciliation.controllers.trajectory_follower import TrajectoryFollower
from reconciliation.data02_diverse_transitions import (
    SCENARIO_IDS,
    apply_variation,
    anchor_successive_paths,
    classify_attempt,
    descriptive_geometry_label,
    execution_diagnostics,
    previous_control_pose,
    qualify_geometry,
    raw_pair_id,
    transition_context_id,
    transition_descriptor,
    validate_data02_config,
    validate_timing,
    variation_schedule,
    write_json_exclusive,
    write_npy_exclusive,
)
from reconciliation.exp01b_extension import is_stop_actions
from reconciliation.lightnav_adapter import (
    DECODED_OUTPUT_SEMANTICS,
    raw_actions_to_local_path,
)
from reconciliation.lightnav_execution_envelope import geometry_descriptor
from reconciliation.online_ipc import OnlineLightNavClient, validate_rgb_frame
from reconciliation.online_switch import sha256_file
from reconciliation.se2 import wrap_angle


TELEMETRY_COLUMNS = (
    "sample_index",
    "sim_time_s",
    "host_monotonic_ns",
    "phase",
    "control_index",
    "nearest_index",
    "target_index",
    "goal_reached",
    "fresh_inference_in_flight",
    "desired_v_mps",
    "desired_omega_rps",
    "executed_v_mps",
    "executed_omega_rps",
    "measured_v_mps",
    "measured_omega_rps",
    "target_front_left_rad_s",
    "target_front_right_rad_s",
    "target_rear_left_rad_s",
    "target_rear_right_rad_s",
    "measured_front_left_rad_s",
    "measured_front_right_rad_s",
    "measured_rear_left_rad_s",
    "measured_rear_right_rad_s",
    "actual_x",
    "actual_y",
    "actual_yaw",
    "pi_correction_rps",
    "integral_error_rad",
    "saturated",
    "sign_protection_event",
)


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def git_sha(root: Path) -> str:
    return subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()


def git_status(root: Path) -> list[str]:
    return subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True).splitlines()


def gpu_snapshot() -> dict[str, Any]:
    line = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,memory.used,memory.free,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        text=True,
    ).strip().splitlines()[0]
    name, total, used, free, utilization = [value.strip() for value in line.split(",")]
    return {
        "name": name,
        "memory_total_mib": int(total),
        "memory_used_mib": int(used),
        "memory_free_mib": int(free),
        "utilization_percent": int(utilization),
    }


def add_static_box(path: str, box: Mapping[str, Any]) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in box["color"]])])
    transform = UsdGeom.XformCommonAPI(cube)
    transform.SetTranslate(Gf.Vec3d(*[float(value) for value in box["center"]]))
    transform.SetScale(Gf.Vec3f(*[float(value) for value in box["size"]]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def capture_rgb(annotator, resolution: tuple[int, int], render_subframes: int) -> np.ndarray:
    rep.orchestrator.step(rt_subframes=render_subframes, delta_time=0.0, pause_timeline=False)
    rgba = np.asarray(annotator.get_data())
    expected = (resolution[1], resolution[0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"unexpected Isaac RGB output: {rgba.shape} {rgba.dtype}")
    return validate_rgb_frame(rgba[:, :, :3]).copy()


def pace(origin_wall_s: float, origin_sim_s: float, current_sim_s: float, factor: float) -> None:
    deadline = origin_wall_s + (current_sim_s - origin_sim_s) / factor
    remaining = deadline - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def write_csv_exclusive(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TELEMETRY_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def save_frames(directory: Path, frames: list[tuple[int, float, np.ndarray]]) -> list[dict[str, Any]]:
    directory.mkdir(parents=True, exist_ok=False)
    manifest = []
    for frame_index, sim_time_s, rgb in frames:
        path = directory / f"frame_{frame_index:04d}.png"
        with path.open("xb") as stream:
            Image.fromarray(rgb, mode="RGB").save(stream, format="PNG")
        manifest.append(
            {
                "frame_index": frame_index,
                "sim_time_s": sim_time_s,
                "relative_path": str(path.relative_to(directory.parent.parent)),
                "sha256": sha256_file(path),
            }
        )
    return manifest


def body_velocity(previous: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    delta = current[:2] - previous[:2]
    forward = np.asarray([math.cos(float(previous[2])), math.sin(float(previous[2]))])
    return np.asarray(
        [float(delta @ forward / dt), float(wrap_angle(current[2] - previous[2]) / dt)]
    )


def telemetry_row(
    *,
    sample_index: int,
    sim_time_s: float,
    phase: str,
    control_index: int,
    command,
    in_flight: bool,
    desired: np.ndarray,
    executed: np.ndarray,
    measured: np.ndarray,
    target_wheels: np.ndarray,
    measured_wheels: np.ndarray,
    pose: np.ndarray,
    state: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "sample_index": sample_index,
        "sim_time_s": sim_time_s,
        "host_monotonic_ns": time.monotonic_ns(),
        "phase": phase,
        "control_index": control_index,
        "nearest_index": int(command.nearest_index),
        "target_index": int(command.target_index),
        "goal_reached": int(command.goal_reached),
        "fresh_inference_in_flight": int(in_flight),
        "desired_v_mps": float(desired[0]),
        "desired_omega_rps": float(desired[1]),
        "executed_v_mps": float(executed[0]),
        "executed_omega_rps": float(executed[1]),
        "measured_v_mps": float(measured[0]),
        "measured_omega_rps": float(measured[1]),
        "target_front_left_rad_s": float(target_wheels[0]),
        "target_front_right_rad_s": float(target_wheels[1]),
        "target_rear_left_rad_s": float(target_wheels[2]),
        "target_rear_right_rad_s": float(target_wheels[3]),
        "measured_front_left_rad_s": float(measured_wheels[0]),
        "measured_front_right_rad_s": float(measured_wheels[1]),
        "measured_rear_left_rad_s": float(measured_wheels[2]),
        "measured_rear_right_rad_s": float(measured_wheels[3]),
        "actual_x": float(pose[0]),
        "actual_y": float(pose[1]),
        "actual_yaw": float(pose[2]),
        "pi_correction_rps": float(state["pi_correction_rps"]),
        "integral_error_rad": float(state["integral_error_rad"]),
        "saturated": int(state["saturated"]),
        "sign_protection_event": int(state["sign_protection_event"]),
    }


def redraw(draw, visual: Mapping[str, Any], old_world: np.ndarray, actual: list[np.ndarray], fresh_world=None, markers=None) -> None:
    if draw is None:
        return
    draw.clear_lines()
    draw.clear_points()
    z = float(visual["z_offset_m"])
    width = float(visual["line_width"])
    draw_polyline(draw, old_world, z=z, color=visual["old_reference_color_rgba"], width=width)
    if len(actual) > 1:
        draw_polyline(
            draw,
            np.asarray(actual),
            z=z + 0.02,
            color=visual["old_actual_color_rgba"],
            width=width,
        )
    if fresh_world is not None:
        draw_polyline(
            draw,
            fresh_world,
            z=z + 0.04,
            color=visual["fresh_reference_color_rgba"],
            width=width,
        )
    if markers:
        for pose, color in markers:
            draw_pose_points(
                draw,
                np.asarray([pose]),
                z=z + 0.08,
                color=color,
                size=float(visual["marker_size"]),
            )


def capture_viewport(path: Path) -> None:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport unavailable")
    path.parent.mkdir(parents=True, exist_ok=True)
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 10.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("viewport capture did not complete")


def failure_attempt(
    attempt_dir: Path,
    *,
    scenario: Mapping[str, Any],
    variant: Mapping[str, Any],
    attempt_index: int,
    classification: str,
    stage: str,
    error: str,
    relative_path: str,
) -> dict[str, Any]:
    record = {
        "scenario_id": scenario["id"],
        "variant_id": variant["id"],
        "attempt_index": attempt_index,
        "classification": classification,
        "failure_stage": stage,
        "error": error,
        "raw_pair_id": "",
        "transition_context_id": "",
        "geometry_label": "",
        "attempt_relative_path": relative_path,
    }
    write_json_exclusive(attempt_dir / "attempt.json", record)
    return record


def collect_attempt(
    *,
    attempt_dir: Path,
    relative_path: str,
    scenario: Mapping[str, Any],
    variant: Mapping[str, Any],
    attempt_index: int,
    episode_index: int,
    config: Mapping[str, Any],
    runtime: ClosedLoopRuntime,
    client: OnlineLightNavClient,
    rgb_annotator,
    resolution: tuple[int, int],
    server_info: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    candidate: Mapping[str, Any],
    draw,
) -> dict[str, Any]:
    for name in ("raw", "derived", "actual", "telemetry"):
        (attempt_dir / name).mkdir(parents=True, exist_ok=False)
    protocol = config["protocol"]
    simulation = config["simulation"]
    camera = config["camera"]
    lightnav = config["lightnav"]
    initial = apply_variation(scenario["initial_pose_se2"], variant)
    runtime.reset(initial, float(simulation["settling_duration_s"]))
    if ARGS.gui:
        set_camera_view(
            eye=[float(initial[0]) + 1.5, float(initial[1]), float(config["gui"]["visualization"]["camera_height_m"])],
            target=[float(initial[0]) + 1.5, float(initial[1]), 0.0],
            camera_prim_path="/OmniverseKit_Persp",
        )
    client.reset_episode(str(scenario["instruction"]), episode_index)
    physics_dt = float(simulation["physics_dt_s"])
    control_dt = float(simulation["control_dt_s"])
    physics_steps = int(round(control_dt / physics_dt))
    rgb_period = 1.0 / float(lightnav["video_fps"])
    rgb_steps = int(round(rgb_period / physics_dt))
    frames: list[tuple[int, float, np.ndarray]] = []
    frame_index = 0
    for _ in range(int(lightnav["expected_history_frames"])):
        for step in range(rgb_steps):
            runtime.world.step(render=ARGS.gui or step == rgb_steps - 1)
        rgb = capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
        frames.append((frame_index, float(runtime.world.current_time), rgb.copy()))
        client.observe(rgb, frame_index=frame_index, sim_time_s=float(runtime.world.current_time))
        frame_index += 1

    old_observation_pose = se2_from_world_pose(runtime.robot)
    old_observation_sim = float(runtime.world.current_time)
    old_observation_host = time.monotonic_ns()
    old_request_host = time.monotonic_ns()
    old_actions, old_response = client.predict(prediction_kind="old")
    old_ready_host = time.monotonic_ns()
    write_npy_exclusive(attempt_dir / "raw/old_lightnav.npy", old_actions)
    with (attempt_dir / "raw/old_raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(str(old_response["raw_text"]))
    if is_stop_actions(old_actions, absolute_tolerance=float(protocol["stop_action_absolute_tolerance"])):
        frame_manifest = save_frames(attempt_dir / "raw/rgb", frames)
        write_json_exclusive(attempt_dir / "raw/history_manifest.json", {"frames": frame_manifest})
        write_json_exclusive(
            attempt_dir / "provenance.json",
            {
                "research_repo_commit": git_sha(REPOSITORY_ROOT),
                "server_info": dict(server_info),
                "failure_stage": "old_output",
                "raw_old_sha256": sha256_file(attempt_dir / "raw/old_lightnav.npy"),
            },
        )
        return failure_attempt(
            attempt_dir,
            scenario=scenario,
            variant=variant,
            attempt_index=attempt_index,
            classification="MODEL_STOP_OUTPUT",
            stage="old_output",
            error="OLD was a model STOP output; no successive executing pair can be formed",
            relative_path=relative_path,
        )

    old_local = raw_actions_to_local_path(
        old_actions,
        decoded_output_semantics=DECODED_OUTPUT_SEMANTICS,
        expected_horizon=int(lightnav["expected_horizon"]),
    )
    old_world, _ = anchor_successive_paths(
        old_local, old_local, old_observation_pose, old_observation_pose
    )
    write_npy_exclusive(attempt_dir / "derived/old_world.npy", old_world)
    follower = TrajectoryFollower(old_world, follower_config(stage0b))
    correction = runtime.correction()
    pose = se2_from_world_pose(runtime.robot)
    command = follower.forward(pose)
    old_execution_start = float(runtime.world.current_time)
    control_samples: list[dict[str, Any]] = []
    actual = [pose.copy()]
    sample_times = [0.0]
    telemetry: list[dict[str, Any]] = []
    saturated_rows = [False]
    measured_body = np.zeros(2, dtype=np.float64)
    previous_pose = pose.copy()
    desired = np.asarray([command.linear_velocity_mps, command.angular_velocity_rps])
    executed = np.zeros(2, dtype=np.float64)
    target = np.zeros(4, dtype=np.float64)
    measured_wheels = canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
    neutral_state = {
        "pi_correction_rps": 0.0,
        "integral_error_rad": 0.0,
        "saturated": False,
        "sign_protection_event": False,
    }
    telemetry.append(
        telemetry_row(
            sample_index=0,
            sim_time_s=0.0,
            phase="OLD_EXECUTION",
            control_index=-1,
            command=command,
            in_flight=False,
            desired=np.zeros(2),
            executed=np.zeros(2),
            measured=np.zeros(2),
            target_wheels=target,
            measured_wheels=measured_wheels,
            pose=pose,
            state=neutral_state,
        )
    )
    control_samples.append(
        {"sim_time_s": old_execution_start, "actual_pose_world": pose.tolist(), "control_index": 0}
    )
    fresh_future = None
    fresh_actions = None
    fresh_response = None
    fresh_observation_pose = None
    fresh_observation_sim = None
    fresh_observation_host = None
    fresh_request_host = None
    fresh_ready_host = None
    fresh_ready_sim = None
    fresh_detected_host = None
    request_clock: dict[str, int] = {}
    queued_frame_count = 0
    old_goal_during_inference = False
    old_nonzero_during_inference = True
    control_index = 0
    live_step = 0
    origin_wall = time.monotonic()
    origin_sim = float(runtime.world.current_time)
    last_state = neutral_state
    last_old_desired = desired.copy()
    visual = config["gui"]["visualization"]
    redraw(draw, visual, old_world, actual)

    with ThreadPoolExecutor(max_workers=1) as executor:
        while fresh_actions is None:
            if live_step % physics_steps == 0:
                pose = se2_from_world_pose(runtime.robot)
                command = follower.forward(pose)
                desired = np.asarray(
                    [command.linear_velocity_mps, command.angular_velocity_rps], dtype=np.float64
                )
                last_old_desired = desired.copy()
                control_samples.append(
                    {
                        "sim_time_s": float(runtime.world.current_time),
                        "actual_pose_world": pose.tolist(),
                        "control_index": control_index,
                    }
                )
                if command.goal_reached:
                    runtime.stop()
                    executed = np.zeros(2)
                    runtime_target = np.zeros(runtime.robot.num_dof)
                    last_state = neutral_state
                    if fresh_future is not None:
                        old_goal_during_inference = True
                else:
                    executed, runtime_target, last_state = runtime.apply(
                        desired, "calibrated", correction, float(measured_body[1])
                    )
                target = canonical_wheel_values(runtime_target, runtime.wheels)
                control_index += 1
                if ARGS.gui:
                    print(
                        "DATA02_LIVE="
                        + json.dumps(
                            {
                                "scenario": scenario["id"],
                                "variant": variant["id"],
                                "phase": "OLD_EXECUTION_FRESH_IN_FLIGHT" if fresh_future else "OLD_EXECUTION",
                                "sim_time_s": float(runtime.world.current_time) - old_execution_start,
                                "actual": pose.tolist(),
                                "desired": desired.tolist(),
                                "executed": executed.tolist(),
                                "measured": measured_body.tolist(),
                                "target_wheels": target.tolist(),
                                "measured_wheels": measured_wheels.tolist(),
                                "nearest_index": command.nearest_index,
                                "target_index": command.target_index,
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
            live_step += 1
            capture_due = live_step % rgb_steps == 0
            runtime.world.step(render=ARGS.gui or capture_due)
            pace(origin_wall, origin_sim, float(runtime.world.current_time), float(protocol["pace_real_time_factor"]))
            current = se2_from_world_pose(runtime.robot)
            measured_body = body_velocity(previous_pose, current, physics_dt)
            previous_pose = current.copy()
            measured_wheels = canonical_wheel_values(runtime.robot.get_joint_velocities(), runtime.wheels)
            actual.append(current.copy())
            sample_times.append(float(runtime.world.current_time) - old_execution_start)
            saturated_rows.append(bool(last_state["saturated"]))
            if fresh_future is not None and (abs(desired[0]) <= 1e-9 and abs(desired[1]) <= 1e-9):
                old_nonzero_during_inference = False
            telemetry.append(
                telemetry_row(
                    sample_index=len(actual) - 1,
                    sim_time_s=sample_times[-1],
                    phase="OLD_EXECUTION_FRESH_IN_FLIGHT" if fresh_future is not None else "OLD_EXECUTION",
                    control_index=control_index - 1,
                    command=command,
                    in_flight=fresh_future is not None and not fresh_future.done(),
                    desired=desired,
                    executed=executed,
                    measured=measured_body,
                    target_wheels=target,
                    measured_wheels=measured_wheels,
                    pose=current,
                    state=last_state,
                )
            )
            elapsed = float(runtime.world.current_time) - old_execution_start
            if capture_due:
                rgb = capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
                frames.append((frame_index, float(runtime.world.current_time), rgb.copy()))
                if fresh_future is None and elapsed + 1e-9 < float(protocol["fresh_observation_delay_s"]):
                    client.observe(rgb, frame_index=frame_index, sim_time_s=float(runtime.world.current_time))
                elif fresh_future is None:
                    client.observe(rgb, frame_index=frame_index, sim_time_s=float(runtime.world.current_time))
                    fresh_observation_pose = current.copy()
                    fresh_observation_sim = float(runtime.world.current_time)
                    fresh_observation_host = time.monotonic_ns()
                    started = threading.Event()

                    def request_fresh():
                        request_clock["start"] = time.monotonic_ns()
                        started.set()
                        result = client.predict(prediction_kind="new")
                        request_clock["end"] = time.monotonic_ns()
                        return result

                    fresh_future = executor.submit(request_fresh)
                    if not started.wait(timeout=1.0):
                        raise RuntimeError("FRESH request thread did not start")
                    fresh_request_host = request_clock["start"]
                else:
                    queued_frame_count += 1
                frame_index += 1
            if fresh_future is not None and fresh_future.done():
                fresh_actions, fresh_response = fresh_future.result()
                fresh_ready_host = request_clock["end"]
                fresh_ready_sim = float(runtime.world.current_time)
                fresh_detected_host = time.monotonic_ns()
                break
            if fresh_observation_sim is not None and (
                float(runtime.world.current_time) - fresh_observation_sim
                > float(protocol["maximum_fresh_wait_sim_s"]) + 30.0
            ):
                raise TimeoutError("FRESH inference exceeded emergency timeout")
            if ARGS.gui and live_step % physics_steps == 0:
                redraw(draw, visual, old_world, actual)

    runtime.stop()
    boundary = se2_from_world_pose(runtime.robot)
    if fresh_observation_pose is None or fresh_observation_sim is None or fresh_observation_host is None:
        raise RuntimeError("FRESH observation was not recorded")
    if fresh_request_host is None or fresh_ready_host is None or fresh_ready_sim is None:
        raise RuntimeError("FRESH timing was not recorded")
    write_npy_exclusive(attempt_dir / "raw/fresh_lightnav.npy", fresh_actions)
    with (attempt_dir / "raw/fresh_raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(str(fresh_response["raw_text"]))
    fresh_local = raw_actions_to_local_path(
        fresh_actions,
        decoded_output_semantics=DECODED_OUTPUT_SEMANTICS,
        expected_horizon=int(lightnav["expected_horizon"]),
    )
    _, fresh_world = anchor_successive_paths(
        old_local, fresh_local, old_observation_pose, fresh_observation_pose
    )
    write_npy_exclusive(attempt_dir / "derived/fresh_world.npy", fresh_world)
    p_pose = previous_control_pose(control_samples, fresh_ready_sim)
    fresh_follower = TrajectoryFollower(fresh_world, follower_config(stage0b))
    fresh_command = fresh_follower.forward(boundary)
    first_fresh_desired = np.asarray(
        [fresh_command.linear_velocity_mps, fresh_command.angular_velocity_rps]
    )
    timing = {
        "old_observation_sim_time_s": old_observation_sim,
        "old_observation_host_monotonic_ns": old_observation_host,
        "old_request_host_monotonic_ns": old_request_host,
        "old_ready_host_monotonic_ns": old_ready_host,
        "fresh_observation_sim_time_s": fresh_observation_sim,
        "fresh_observation_host_monotonic_ns": fresh_observation_host,
        "fresh_request_host_monotonic_ns": fresh_request_host,
        "fresh_model_ready_sim_time_s": fresh_ready_sim,
        "fresh_model_ready_host_monotonic_ns": fresh_ready_host,
        "fresh_usable_sim_time_s": fresh_ready_sim,
        "fresh_usable_host_monotonic_ns": fresh_ready_host,
        "fresh_ready_detected_host_monotonic_ns": fresh_detected_host,
        "natural_inference_host_latency_s": (fresh_ready_host - fresh_request_host) / 1e9,
        "effective_inference_sim_latency_s": fresh_ready_sim - fresh_observation_sim,
        "artificial_added_delay_s": 0.0,
    }
    timing_checks = validate_timing(timing)
    host_latency = float(timing["natural_inference_host_latency_s"])
    rtf = float(timing["effective_inference_sim_latency_s"]) / host_latency
    low_rtf, high_rtf = [float(value) for value in protocol["acceptable_inference_real_time_factor"]]
    timing_checks.update(
        {
            "natural_latency_no_added_delay": timing["artificial_added_delay_s"] == 0.0,
            "inference_real_time_factor_in_range": low_rtf <= rtf <= high_rtf,
            "fresh_observed_while_old_active": not command.goal_reached,
            "old_active_during_inference": old_nonzero_during_inference and not old_goal_during_inference,
            "response_within_configured_wait": float(timing["effective_inference_sim_latency_s"])
            <= float(protocol["maximum_fresh_wait_sim_s"]),
        }
    )
    actual_array = np.asarray(actual)
    desired_rows = np.asarray(
        [[row["desired_v_mps"], row["desired_omega_rps"]] for row in telemetry]
    )
    executed_rows = np.asarray(
        [[row["executed_v_mps"], row["executed_omega_rps"]] for row in telemetry]
    )
    measured_rows = np.asarray(
        [[row["measured_v_mps"], row["measured_omega_rps"]] for row in telemetry]
    )
    target_rows = np.asarray(
        [
            [
                row["target_front_left_rad_s"],
                row["target_front_right_rad_s"],
                row["target_rear_left_rad_s"],
                row["target_rear_right_rad_s"],
            ]
            for row in telemetry
        ]
    )
    wheel_rows = np.asarray(
        [
            [
                row["measured_front_left_rad_s"],
                row["measured_front_right_rad_s"],
                row["measured_rear_left_rad_s"],
                row["measured_rear_right_rad_s"],
            ]
            for row in telemetry
        ]
    )
    diagnostics = execution_diagnostics(
        old_world,
        actual_array,
        np.asarray(saturated_rows),
        desired_body=desired_rows,
        executed_body=executed_rows,
        measured_body=measured_rows,
        target_wheels=target_rows,
        measured_wheels=wheel_rows,
    )
    travel_translation = float(np.linalg.norm(boundary[:2] - fresh_observation_pose[:2]))
    travel_yaw = abs(float(wrap_angle(boundary[2] - fresh_observation_pose[2])))
    execution_checks = {
        "old_reference_distance_rms_in_envelope": float(
            diagnostics["spatial_reference"]["old_reference_distance_rms_m"]
        )
        <= float(protocol["old_reference_distance_rms_max_m"]),
        "old_reference_distance_max_in_envelope": float(
            diagnostics["spatial_reference"]["old_reference_distance_max_m"]
        )
        <= float(protocol["old_reference_distance_max_max_m"]),
        "active_saturation_fraction_in_envelope": float(diagnostics["active_saturation_fraction"])
        <= float(protocol["active_saturation_fraction_max"]),
        "robot_moved_during_inference": travel_translation
        > float(protocol["minimum_observation_to_boundary_motion_m"])
        or travel_yaw > 1e-3,
    }
    classification = classify_attempt(
        fresh_raw=fresh_actions,
        old_goal_reached=old_goal_during_inference,
        timing_checks=timing_checks,
        execution_checks=execution_checks,
        stop_tolerance=float(protocol["stop_action_absolute_tolerance"]),
    )
    old_geometry = geometry_descriptor(old_world)
    fresh_geometry = geometry_descriptor(fresh_world)
    geometry_label = descriptive_geometry_label(fresh_geometry, config["qualification_thresholds"])
    qualification = qualify_geometry(
        fresh_geometry, str(scenario["qualification_rule"]), config["qualification_thresholds"]
    )
    transition = transition_descriptor(
        p_pose,
        boundary,
        fresh_world,
        last_old_desired=last_old_desired,
        first_fresh_desired=first_fresh_desired,
        fresh_observation_pose=fresh_observation_pose,
        natural_latency_s=host_latency,
    )
    old_raw_hash = sha256_file(attempt_dir / "raw/old_lightnav.npy")
    fresh_raw_hash = sha256_file(attempt_dir / "raw/fresh_lightnav.npy")
    pair_id = raw_pair_id(old_raw_hash, fresh_raw_hash)
    context = {
        "raw_pair_id": pair_id,
        "scenario_id": scenario["id"],
        "variant_id": variant["id"],
        "instruction": scenario["instruction"],
        "initial_pose_world": initial.tolist(),
        "old_observation_pose_world": old_observation_pose.tolist(),
        "fresh_observation_pose_world": fresh_observation_pose.tolist(),
        "previous_control_pose_world": p_pose.tolist(),
        "boundary_pose_world": boundary.tolist(),
        "timing": timing,
        "frames": dict(config["frames"]),
        "p_semantics": "last actual control-update pose strictly preceding B",
        "b_semantics": "actual Jackal pose when FRESH response became usable",
        "fresh_anchor_semantics": "FRESH remains anchored at its observation pose, never B",
    }
    context_id = transition_context_id(context)
    write_npy_exclusive(attempt_dir / "actual/old_execution.npy", actual_array)
    write_csv_exclusive(attempt_dir / "telemetry/old_execution.csv", telemetry)
    write_json_exclusive(attempt_dir / "context.json", context)
    write_json_exclusive(
        attempt_dir / "descriptors.json",
        {
            "old_geometry": old_geometry,
            "fresh_geometry": fresh_geometry,
            "fresh_descriptive_geometry_label": geometry_label,
            "scenario_qualification": qualification,
            "transition": transition,
            "execution": diagnostics,
            "execution_checks": execution_checks,
            "timing_checks": timing_checks,
            "inference_real_time_factor": rtf,
        },
    )
    frame_manifest = save_frames(attempt_dir / "raw/rgb", frames)
    write_json_exclusive(
        attempt_dir / "raw/history_manifest.json",
        {
            "frame_count": len(frame_manifest),
            "old_observation_frame_index": int(lightnav["expected_history_frames"]) - 1,
            "fresh_observation_frame_index": next(
                item["frame_index"]
                for item in frame_manifest
                if math.isclose(item["sim_time_s"], fresh_observation_sim, abs_tol=1e-12)
            ),
            "queued_during_fresh_inference_count": queued_frame_count,
            "frames": frame_manifest,
        },
    )
    provenance = {
        "research_repo_commit": git_sha(REPOSITORY_ROOT),
        "research_repo_status": git_status(REPOSITORY_ROOT),
        "lightnav_repo_commit": git_sha(resolve_path(config["paths"]["lightnav_checkout"])),
        "server_info": dict(server_info),
        "stage0d_candidate": dict(candidate),
        "stage0b_config_sha256": sha256_file(resolve_path(config["paths"]["stage0b_config"])),
        "data02_config_sha256": sha256_file(ARGS.config.resolve()),
        "raw_old_sha256": old_raw_hash,
        "raw_fresh_sha256": fresh_raw_hash,
        "old_world_sha256": sha256_file(attempt_dir / "derived/old_world.npy"),
        "fresh_world_sha256": sha256_file(attempt_dir / "derived/fresh_world.npy"),
        "canonical_wheel_names": list(canonical_wheel_names(runtime.wheels)),
        "wheel_radius_m": float(runtime.wheels["radius_m"]),
        "physical_wheel_separation_m": float(runtime.wheels["separation_m"]),
        "gpu": gpu_snapshot(),
        "isaac_runtime": "Isaac Sim 6.0.1 via configured ISAACSIM_ROOT python.sh",
        "no_optimizer_or_reconciliation_executed": True,
        "fresh_executed": False,
    }
    write_json_exclusive(attempt_dir / "provenance.json", provenance)
    record = {
        "scenario_id": scenario["id"],
        "scene_id": scenario["scene_id"],
        "instruction": scenario["instruction"],
        "variant_id": variant["id"],
        "attempt_index": attempt_index,
        "classification": classification,
        "raw_pair_id": pair_id,
        "transition_context_id": context_id,
        "geometry_label": geometry_label,
        "qualification_geometry_pass": bool(qualification["passed"]),
        "old_raw_sha256": old_raw_hash,
        "fresh_raw_sha256": fresh_raw_hash,
        "old_world_sha256": provenance["old_world_sha256"],
        "fresh_world_sha256": provenance["fresh_world_sha256"],
        "fresh_geometry": fresh_geometry,
        "transition": transition,
        "attempt_relative_path": relative_path,
    }
    write_json_exclusive(attempt_dir / "attempt.json", record)
    markers = [
        (old_observation_pose, visual["old_reference_color_rgba"]),
        (fresh_observation_pose, visual["observation_color_rgba"]),
        (p_pose, visual["previous_pose_color_rgba"]),
        (boundary, visual["boundary_color_rgba"]),
    ]
    redraw(draw, visual, old_world, actual, fresh_world, markers)
    if ARGS.gui:
        print(
            "DATA02_BOUNDARY="
            + json.dumps(
                {
                    "scenario": scenario["id"],
                    "variant": variant["id"],
                    "classification": classification,
                    "P": p_pose.tolist(),
                    "B": boundary.tolist(),
                    "fresh_observation": fresh_observation_pose.tolist(),
                    "first_raw_fresh_desired": first_fresh_desired.tolist(),
                    "fresh_executed": False,
                },
                sort_keys=True,
            ),
            flush=True,
        )
    return record


def main() -> Path:
    config = load_config(ARGS.config.resolve())
    scenarios = validate_data02_config(config)
    if ARGS.phase == "primary" and config["protocol"].get("primary_config_frozen") is not True:
        raise ValueError("primary collection requires primary_config_frozen=true")
    if ARGS.scenario != "all":
        scenarios = [item for item in scenarios if item["id"] == ARGS.scenario]
        if not scenarios:
            raise ValueError(f"unknown DATA-02 scenario: {ARGS.scenario}")
    output_root = resolve_path(config["paths"]["output_root"])
    run_root = output_root / ARGS.run_id
    run_root.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(ARGS.config.resolve(), run_root / "config_snapshot.yaml")
    stage0e = load_config(resolve_path(config["paths"]["stage0e_config"]))
    stage0b = load_config(resolve_path(config["paths"]["stage0b_config"]))
    candidate = load_frozen_stage0d_candidate(
        resolve_path(stage0e["paths"]["stage0d_run"]),
        stage0e["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    client = OnlineLightNavClient(ARGS.socket, timeout_s=float(config["ipc"]["request_timeout_s"]))
    server_info = client.server_info()
    runtime = ClosedLoopRuntime(
        SIMULATION_APP,
        config,
        stage0b,
        candidate,
        render=False,
        real_time_factor=None,
    )
    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, "/World/Data02DomeLight")
    light.CreateIntensityAttr(1000.0)
    for scenario in validate_data02_config(config):
        for box in scenario["boxes"]:
            add_static_box(f"/World/Data02/{scenario['id']}/{box['id']}", box)
    camera = config["camera"]
    aperture = float(camera["horizontal_aperture"])
    focal_length = aperture / (
        2.0 * math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0)
    )
    camera_prim = rep.functional.create.camera(
        position=tuple(float(value) for value in camera["relative_translation_m"]),
        rotation=tuple(float(value) for value in camera["relative_rotation_xyz_deg"]),
        relative_to=runtime.articulation_root,
        focal_length=focal_length,
        horizontal_aperture=aperture,
        clipping_range=tuple(float(value) for value in camera["clipping_range_m"]),
        parent=runtime.articulation_root,
        name=str(camera["name"]),
    )
    resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
    render_product = rep.create.render_product(str(camera_prim.GetPath()), resolution=resolution)
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(render_product)
    rep.orchestrator.set_capture_on_play(False)
    runtime.world.step(render=True)
    capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
    draw = None
    if ARGS.gui:
        from isaacsim.util.debug_draw import _debug_draw

        draw = _debug_draw.acquire_debug_draw_interface()
        print("DATA02_GUI_LEGEND=BLUE OLD reference; GREEN actual OLD; ORANGE FRESH; MAGENTA FRESH observation; BLACK P; RED B", flush=True)
    phase_directory = "qualification" if ARGS.phase == "qualification" else "primary" if ARGS.phase == "primary" else "smoke"
    attempts_root = run_root / phase_directory / "attempts"
    attempts_root.mkdir(parents=True, exist_ok=False)
    target = (
        int(config["protocol"]["qualification_valid_outputs_per_family"])
        if ARGS.phase == "qualification"
        else int(config["protocol"]["primary_valid_moving_per_family"])
        if ARGS.phase == "primary"
        else 1
    )
    cap = (
        int(config["protocol"]["qualification_max_attempts_per_family"])
        if ARGS.phase == "qualification"
        else int(config["protocol"]["primary_max_attempts_per_family"])
        if ARGS.phase == "primary"
        else 1
    )
    if ARGS.max_attempts is not None:
        if ARGS.max_attempts < 1:
            raise ValueError("--max-attempts must be positive")
        cap = min(cap, ARGS.max_attempts)
    records = []
    episode_index = 0
    for scenario in scenarios:
        valid_count = 0
        for attempt_index in range(cap):
            if valid_count >= target:
                break
            variant = variation_schedule(config, attempt_index)
            dirname = f"{scenario['id']}__{variant['id']}__attempt_{attempt_index:02d}"
            attempt_dir = attempts_root / dirname
            attempt_dir.mkdir(parents=True, exist_ok=False)
            relative = str(attempt_dir.relative_to(run_root))
            try:
                record = collect_attempt(
                    attempt_dir=attempt_dir,
                    relative_path=relative,
                    scenario=scenario,
                    variant=variant,
                    attempt_index=attempt_index,
                    episode_index=episode_index,
                    config=config,
                    runtime=runtime,
                    client=client,
                    rgb_annotator=rgb_annotator,
                    resolution=resolution,
                    server_info=server_info,
                    stage0b=stage0b,
                    candidate=candidate,
                    draw=draw,
                )
            except Exception as error:
                classification = "INFERENCE_ERROR" if "LightNav server" in str(error) or isinstance(error, TimeoutError) else "OTHER_FAILURE"
                if not (attempt_dir / "provenance.json").exists():
                    write_json_exclusive(
                        attempt_dir / "provenance.json",
                        {
                            "research_repo_commit": git_sha(REPOSITORY_ROOT),
                            "server_info": dict(server_info),
                            "exception_type": type(error).__name__,
                        },
                    )
                record = failure_attempt(
                    attempt_dir,
                    scenario=scenario,
                    variant=variant,
                    attempt_index=attempt_index,
                    classification=classification,
                    stage="collector_exception",
                    error=f"{type(error).__name__}: {error}",
                    relative_path=relative,
                )
                print(f"DATA02_ATTEMPT_ERROR={record}", file=sys.stderr, flush=True)
            records.append(record)
            valid_count += int(record["classification"] == "VALID_MOVING")
            episode_index += 1
            if ARGS.gui:
                capture_viewport(run_root / "gui" / f"{dirname}.png")
                runtime.world.pause()
                hold_until = time.monotonic() + float(config["gui"]["inter_episode_hold_wall_s"])
                while time.monotonic() < hold_until and SIMULATION_APP.is_running():
                    SIMULATION_APP.update()
                    time.sleep(0.02)
                runtime.world.play()
    client.shutdown_server()
    metadata = {
        "dataset": config["dataset"],
        "run_id": ARGS.run_id,
        "phase": ARGS.phase,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "research_repo_commit": git_sha(REPOSITORY_ROOT),
        "research_repo_status": git_status(REPOSITORY_ROOT),
        "config_snapshot_sha256": sha256_file(run_root / "config_snapshot.yaml"),
        "primary_config_frozen": bool(config["protocol"]["primary_config_frozen"]),
        "scenario_ids": [item["id"] for item in scenarios],
        "attempt_count": len(records),
        "classification_counts": dict(sorted(__import__("collections").Counter(item["classification"] for item in records).items())),
        "natural_latency_only": True,
        "fresh_executed": False,
        "no_optimizer_or_reconciliation_executed": True,
        "server_info_start": server_info,
    }
    write_json_exclusive(run_root / "metadata.json", metadata)
    write_json_exclusive(run_root / "attempt_index.json", {"attempts": records})
    print(f"DATA02_OUTPUT={run_root}", flush=True)
    if ARGS.gui and not ARGS.no_hold:
        print("DATA02_GUI_PHASE=FINISHED_HOLDING; close Isaac Sim to exit", flush=True)
        runtime.world.pause()
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            time.sleep(0.02)
    return run_root


try:
    main()
finally:
    SIMULATION_APP.close()
