#!/usr/bin/env python3
"""Run live EXP-01B OLD/NEW transitions against a persistent LightNav server."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
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
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp01b_online_raw_switch.yaml",
    )
    parser.add_argument("--gui", action="store_true", help="diagnostic display, not timing replay")
    parser.add_argument("--hold", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": not ARGS.gui})

import numpy as np
import omni.replicator.core as rep
import omni.usd
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from lightnav_stage0c_runtime import (
    canonical_wheel_names,
    discover_wheels,
    find_articulation_root,
    load_config,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.lightnav_adapter import (
    DECODED_OUTPUT_SEMANTICS,
    lightnav_local_to_world,
    raw_actions_to_local_path,
)
from reconciliation.online_ipc import OnlineLightNavClient, validate_rgb_frame
from reconciliation.online_switch import (
    MeasuredReadyTiming,
    aggregate_experiment,
    analyze_online_ready_switch,
    apply_reporting_thresholds,
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
    timing_validity,
    validate_experiment_output,
    validate_trial_output,
)
from reconciliation.se2 import wrap_angle
from reconciliation.trajectory import validate_pose_se2


TIMELINE_COLUMNS = (
    "sim_time_s",
    "host_monotonic_ns",
    "phase",
    "actual_x",
    "actual_y",
    "actual_yaw",
    "commanded_v",
    "commanded_omega",
    "active_chunk",
    "active_reference_index",
    "new_inference_in_flight",
    "rgb_frame_index",
)


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def git_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()


def git_status() -> list[str]:
    output = subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "status", "--porcelain"], text=True
    )
    return output.splitlines()


def gpu_snapshot() -> dict[str, Any]:
    output = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,memory.used,memory.free,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        text=True,
    ).strip()
    name, total, used, free, utilization = [part.strip() for part in output.split(",")]
    return {
        "gpu_name": name,
        "memory_total_mib": int(total),
        "memory_used_mib": int(used),
        "memory_free_mib": int(free),
        "utilization_percent": int(utilization),
    }


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


def capture_rgb(annotator, resolution: tuple[int, int], render_subframes: int) -> np.ndarray:
    rep.orchestrator.step(
        rt_subframes=render_subframes,
        delta_time=0.0,
        pause_timeline=False,
    )
    rgba = np.asarray(annotator.get_data())
    expected = (resolution[1], resolution[0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"unexpected Isaac RGB output: {rgba.shape} {rgba.dtype}")
    return validate_rgb_frame(rgba[:, :, :3]).copy()


def add_event(
    events: list[dict[str, Any]],
    name: str,
    world: World,
    pose: np.ndarray,
    **fields: Any,
) -> None:
    events.append(
        {
            "event": name,
            "sim_time_s": float(world.current_time),
            "host_monotonic_ns": time.monotonic_ns(),
            "robot_pose_world": pose.tolist(),
            **fields,
        }
    )


def timeline_row(
    world: World,
    pose: np.ndarray,
    *,
    phase: str,
    command,
    active_chunk: str,
    in_flight: bool,
    rgb_frame_index: int,
) -> dict[str, Any]:
    return {
        "sim_time_s": float(world.current_time),
        "host_monotonic_ns": time.monotonic_ns(),
        "phase": phase,
        "actual_x": float(pose[0]),
        "actual_y": float(pose[1]),
        "actual_yaw": float(pose[2]),
        "commanded_v": float(command.linear_velocity_mps),
        "commanded_omega": float(command.angular_velocity_rps),
        "active_chunk": active_chunk,
        "active_reference_index": int(command.target_index),
        "new_inference_in_flight": int(in_flight),
        "rgb_frame_index": int(rgb_frame_index),
    }


def pace_to_simulation_deadline(
    *,
    origin_wall_s: float,
    origin_sim_s: float,
    current_sim_s: float,
    real_time_factor: float,
) -> None:
    """Pace against one absolute deadline so render stalls can be recovered.

    A per-step sleep permanently carries each RGB render delay into wall time.
    Absolute simulation-time deadlines instead let following non-render steps
    catch up while still preventing the simulation from running ahead.
    """

    target_wall_s = origin_wall_s + (current_sim_s - origin_sim_s) / real_time_factor
    remaining = target_wall_s - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def pose_one_control_interval_before(
    pose_history: list[tuple[float, np.ndarray]], ready_time: float, control_dt: float
) -> np.ndarray:
    target = ready_time - control_dt
    candidates = [pose for sample_time, pose in pose_history if sample_time <= target + 1e-9]
    return (candidates[-1] if candidates else pose_history[0][1]).copy()


def run_trial(
    *,
    trial_index: int,
    trial_dir: Path,
    config: Mapping[str, Any],
    world: World,
    robot: SingleArticulation,
    controller,
    differential,
    wheels: Mapping[str, Any],
    rgb_annotator,
    resolution: tuple[int, int],
    client: OnlineLightNavClient,
) -> dict[str, Any]:
    for folder in ("raw", "derived", "results"):
        (trial_dir / folder).mkdir(parents=True, exist_ok=False)
    simulation = config["simulation"]
    protocol = config["experiment_protocol"]
    camera = config["camera"]
    lightnav = config["lightnav"]
    instruction = str(config["instruction"])
    initial_pose = validate_pose_se2(simulation["initial_robot_pose_se2"])
    physics_dt = float(simulation["physics_dt"])
    control_dt = float(simulation["control_dt"])
    control_steps = int(round(control_dt / physics_dt))
    rgb_period = 1.0 / float(lightnav["video_fps"])
    rgb_steps = int(round(rgb_period / physics_dt))
    if not math.isclose(control_steps * physics_dt, control_dt, abs_tol=1e-9):
        raise ValueError("control_dt must be divisible by physics_dt")
    if not math.isclose(rgb_steps * physics_dt, rgb_period, abs_tol=1e-9):
        raise ValueError("RGB period must be divisible by physics_dt")

    world.reset()
    robot.set_world_pose(
        position=np.array([initial_pose[0], initial_pose[1], simulation["spawn_height_m"]]),
        orientation=quaternion_from_yaw(float(initial_pose[2])),
    )
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    settling_steps = int(round(float(simulation["settling_duration_s"]) / physics_dt))
    for step in range(settling_steps):
        world.step(render=ARGS.gui or step == settling_steps - 1)

    events: list[dict[str, Any]] = []
    timeline: list[dict[str, Any]] = []
    actual: list[np.ndarray] = []
    frame_index = 0
    start_pose = se2_from_world_pose(robot)
    add_event(events, "episode_start", world, start_pose, trial_index=trial_index)
    client.reset_episode(instruction, trial_index)

    expected_history = int(lightnav["expected_history_frames"])
    for prime_index in range(expected_history):
        for step in range(rgb_steps):
            world.step(render=ARGS.gui or step == rgb_steps - 1)
        rgb = capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
        client.observe(rgb, frame_index=frame_index, sim_time_s=float(world.current_time))
        frame_index += 1
    old_observation_pose = se2_from_world_pose(robot)
    old_observation_sim_time = float(world.current_time)
    add_event(events, "old_observation", world, old_observation_pose, frame_index=frame_index - 1)
    old_request_start_ns = time.monotonic_ns()
    old_actions, old_response = client.predict(prediction_kind="old")
    old_response_ns = time.monotonic_ns()
    add_event(
        events,
        "old_ready",
        world,
        se2_from_world_pose(robot),
        request_response_wall_s=(old_response_ns - old_request_start_ns) / 1e9,
        lightnav_predict_ms=float(old_response["server_predict_host_latency_ms"]),
    )
    old_local = raw_actions_to_local_path(
        old_actions,
        decoded_output_semantics=DECODED_OUTPUT_SEMANTICS,
        expected_horizon=int(lightnav["expected_horizon"]),
    )
    old_world = lightnav_local_to_world(old_local, old_observation_pose)
    old_reference = np.vstack((old_observation_pose, old_world))
    old_follower = TrajectoryFollower(old_reference, follower_config(config))
    command = old_follower.forward(se2_from_world_pose(robot))
    old_execution_start_time = float(world.current_time)
    add_event(events, "old_execution_start", world, se2_from_world_pose(robot))

    new_delay = float(protocol["new_observation_delay_s"])
    max_wait = float(protocol["maximum_new_wait_sim_s"])
    pacing = float(protocol["pace_real_time_factor"])
    new_future = None
    new_observation_pose = None
    new_observation_rgb = None
    new_observation_sim_time = None
    new_request_start_ns = None
    new_progress_observation = None
    new_target_observation = None
    queued_frames: list[tuple[int, float, np.ndarray]] = []
    old_exhausted = False
    wait_limit_exceeded = False
    pose_history: list[tuple[float, np.ndarray]] = [
        (float(world.current_time), se2_from_world_pose(robot))
    ]
    actual.append(pose_history[-1][1].copy())
    pacing_origin_wall_s = time.monotonic()
    pacing_origin_sim_s = float(world.current_time)

    with ThreadPoolExecutor(max_workers=1) as executor:
        live_step = 0
        while True:
            if command.goal_reached:
                old_exhausted = new_future is not None
                runtime_target = np.zeros(robot.num_dof)
            else:
                side = differential.forward(
                    np.asarray([command.linear_velocity_mps, command.angular_velocity_rps])
                )
                runtime_target = runtime_wheel_command(side, wheels)
            controller.apply_action(ArticulationAction(joint_velocities=runtime_target))
            live_step += 1
            capture_due = live_step % rgb_steps == 0
            world.step(render=ARGS.gui or capture_due)
            rgb = (
                capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
                if capture_due
                else None
            )
            pace_to_simulation_deadline(
                origin_wall_s=pacing_origin_wall_s,
                origin_sim_s=pacing_origin_sim_s,
                current_sim_s=float(world.current_time),
                real_time_factor=pacing,
            )
            pose = se2_from_world_pose(robot)
            pose_history.append((float(world.current_time), pose.copy()))
            actual.append(pose.copy())

            elapsed_old = float(world.current_time) - old_execution_start_time
            if capture_due and rgb is not None:
                if new_future is None and elapsed_old + 1e-9 < new_delay:
                    client.observe(rgb, frame_index=frame_index, sim_time_s=float(world.current_time))
                elif new_future is None:
                    client.observe(rgb, frame_index=frame_index, sim_time_s=float(world.current_time))
                    new_observation_pose = pose.copy()
                    new_observation_rgb = rgb.copy()
                    new_observation_sim_time = float(world.current_time)
                    new_progress_observation = old_follower.progress_index
                    new_target_observation = command.target_index
                    add_event(
                        events,
                        "new_observation",
                        world,
                        pose,
                        old_progress_index=new_progress_observation,
                        old_target_index=new_target_observation,
                        frame_index=frame_index,
                    )
                    request_started = threading.Event()
                    request_clock: dict[str, int] = {}

                    def request_new():
                        request_clock["start"] = time.monotonic_ns()
                        request_started.set()
                        actions, response = client.predict(prediction_kind="new")
                        request_clock["end"] = time.monotonic_ns()
                        return actions, response

                    new_future = executor.submit(request_new)
                    if not request_started.wait(timeout=1.0):
                        raise RuntimeError("NEW request worker did not start")
                    new_request_start_ns = request_clock["start"]
                    add_event(
                        events,
                        "new_request_sent",
                        world,
                        pose,
                        client_request_host_monotonic_ns=new_request_start_ns,
                    )
                else:
                    queued_frames.append((frame_index, float(world.current_time), rgb.copy()))
                frame_index += 1

            if live_step % control_steps == 0:
                command = old_follower.forward(pose)
            timeline.append(
                timeline_row(
                    world,
                    pose,
                    phase="old_execution_new_inference" if new_future else "old_execution",
                    command=command,
                    active_chunk="OLD",
                    in_flight=new_future is not None and not new_future.done(),
                    rgb_frame_index=frame_index - 1,
                )
            )
            if new_future is not None and new_future.done():
                new_actions, new_response = new_future.result()
                new_response_ns = request_clock["end"]
                ready_pose = pose.copy()
                ready_sim_time = float(world.current_time)
                break
            if (
                new_future is not None
                and new_observation_sim_time is not None
                and float(world.current_time) - new_observation_sim_time > max_wait
            ):
                wait_limit_exceeded = True
            if live_step * physics_dt > new_delay + max_wait + 30.0:
                raise RuntimeError("NEW inference did not return within the emergency limit")

    if new_observation_pose is None or new_observation_rgb is None:
        raise RuntimeError("NEW observation was not captured")
    if new_observation_sim_time is None or new_request_start_ns is None:
        raise RuntimeError("NEW timing was not initialized")
    add_event(
        events,
        "new_ready",
        world,
        ready_pose,
        server_predict_host_latency_ms=float(new_response["server_predict_host_latency_ms"]),
        lightnav_reported_latency_ms=float(new_response["lightnav_reported_latency_ms"]),
        client_response_host_monotonic_ns=new_response_ns,
        queued_rgb_frames=len(queued_frames),
    )
    for queued_index, queued_time, queued_rgb in queued_frames:
        client.observe(queued_rgb, frame_index=queued_index, sim_time_s=queued_time)

    new_local = raw_actions_to_local_path(
        new_actions,
        decoded_output_semantics=DECODED_OUTPUT_SEMANTICS,
        expected_horizon=int(lightnav["expected_horizon"]),
    )
    new_world = lightnav_local_to_world(new_local, new_observation_pose)
    previous_pose = pose_one_control_interval_before(pose_history, ready_sim_time, control_dt)
    measured_timing = MeasuredReadyTiming(
        observation_sim_time_s=new_observation_sim_time,
        ready_sim_time_s=ready_sim_time,
        request_host_monotonic_ns=new_request_start_ns,
        response_host_monotonic_ns=new_response_ns,
    )
    analysis = analyze_online_ready_switch(
        actual_pose_before_ready=previous_pose,
        actual_pose_at_ready=ready_pose,
        robot_pose_at_new_observation=new_observation_pose,
        new_world_trajectory=new_world,
        timing=measured_timing,
        old_progress_at_observation=int(new_progress_observation),
        old_progress_at_ready=old_follower.progress_index,
        old_exhausted_before_new_ready=old_exhausted,
    )
    validity = timing_validity(
        analysis,
        acceptable_rtf_range=protocol["acceptable_rtf_range"],
        motion_noise_floor_m=float(protocol["motion_noise_floor_m"]),
    )
    if wait_limit_exceeded:
        validity["checks"]["within_configured_new_wait"] = False
        validity["valid"] = False
    else:
        validity["checks"]["within_configured_new_wait"] = True
    switch_payload = analysis.to_dict()
    switch_payload["valid"] = bool(validity["valid"])
    switch_payload["validity_checks"] = validity["checks"]
    switch_payload["threshold_exceedance"] = apply_reporting_thresholds(
        analysis, config["reporting_thresholds"]
    )
    switch_payload["lightnav_predict_host_latency_ms"] = float(
        new_response["server_predict_host_latency_ms"]
    )
    switch_payload["lightnav_reported_latency_ms"] = float(
        new_response["lightnav_reported_latency_ms"]
    )
    add_event(events, "raw_switch", world, ready_pose, metrics=switch_payload["metrics"])

    new_controller_reference = np.vstack((ready_pose, new_world))
    new_follower = TrajectoryFollower(new_controller_reference, follower_config(config))
    command = new_follower.forward(ready_pose)
    add_event(events, "new_execution_start", world, ready_pose)
    new_execution_origin_wall_s = time.monotonic()
    new_execution_origin_sim_s = float(world.current_time)
    maximum_steps = int(round(float(protocol["maximum_new_execution_s"]) / physics_dt))
    for execution_step in range(maximum_steps):
        if command.goal_reached:
            break
        side = differential.forward(
            np.asarray([command.linear_velocity_mps, command.angular_velocity_rps])
        )
        controller.apply_action(
            ArticulationAction(joint_velocities=runtime_wheel_command(side, wheels))
        )
        world.step(render=ARGS.gui)
        pace_to_simulation_deadline(
            origin_wall_s=new_execution_origin_wall_s,
            origin_sim_s=new_execution_origin_sim_s,
            current_sim_s=float(world.current_time),
            real_time_factor=pacing,
        )
        pose = se2_from_world_pose(robot)
        actual.append(pose.copy())
        if (execution_step + 1) % control_steps == 0:
            command = new_follower.forward(pose)
        timeline.append(
            timeline_row(
                world,
                pose,
                phase="new_execution",
                command=command,
                active_chunk="NEW",
                in_flight=False,
                rgb_frame_index=frame_index - 1,
            )
        )
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    final_pose = se2_from_world_pose(robot)
    add_event(
        events,
        "episode_end",
        world,
        final_pose,
        new_goal_reached=bool(command.goal_reached),
    )

    save_npy_exclusive(trial_dir / "raw/old_actions.npy", old_actions)
    save_npy_exclusive(trial_dir / "raw/new_actions.npy", new_actions)
    with (trial_dir / "raw/old_raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(str(old_response["raw_text"]))
    with (trial_dir / "raw/new_raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(str(new_response["raw_text"]))
    with (trial_dir / "raw/new_observation_rgb.png").open("xb") as stream:
        Image.fromarray(new_observation_rgb, mode="RGB").save(stream, format="PNG")
    save_json_exclusive(trial_dir / "raw/event_log.json", events)
    save_npy_exclusive(trial_dir / "derived/old_world.npy", old_world)
    save_npy_exclusive(trial_dir / "derived/new_world.npy", new_world)
    save_npy_exclusive(
        trial_dir / "derived/new_controller_reference.npy", new_controller_reference
    )
    save_npy_exclusive(trial_dir / "derived/actual_trajectory.npy", np.asarray(actual))
    with (trial_dir / "derived/timeline.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TIMELINE_COLUMNS)
        writer.writeheader()
        writer.writerows(timeline)
    save_json_exclusive(trial_dir / "results/switch_metrics.json", switch_payload)
    save_json_exclusive(
        trial_dir / "results/timing.json",
        {
            "valid": bool(validity["valid"]),
            "new_observation_sim_time_s": new_observation_sim_time,
            "new_ready_sim_time_s": ready_sim_time,
            "new_request_host_monotonic_ns": new_request_start_ns,
            "new_response_host_monotonic_ns": new_response_ns,
            "new_ready_first_control_opportunity_host_monotonic_ns": events[-4][
                "host_monotonic_ns"
            ],
            "request_response_wall_s": measured_timing.request_response_wall_s,
            "simulation_observation_to_ready_s": measured_timing.simulation_latency_s,
            "real_time_factor": measured_timing.real_time_factor,
            "lightnav_predict_host_latency_ms": float(
                new_response["server_predict_host_latency_ms"]
            ),
            "lightnav_reported_latency_ms": float(
                new_response["lightnav_reported_latency_ms"]
            ),
            "old_request_response_wall_s": (old_response_ns - old_request_start_ns) / 1e9,
            "old_lightnav_predict_host_latency_ms": float(
                old_response["server_predict_host_latency_ms"]
            ),
            "timing_validity_criterion": {
                "acceptable_rtf_range": list(protocol["acceptable_rtf_range"]),
                "motion_noise_floor_m": float(protocol["motion_noise_floor_m"]),
            },
        },
    )
    metadata = {
        "experiment": "EXP-01B",
        "trial_index": trial_index,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction": instruction,
        "action_horizon": int(lightnav["expected_horizon"]),
        "decoded_action_semantics": DECODED_OUTPUT_SEMANTICS,
        "intrinsic_waypoint_time_base": False,
        "raw_switch_policy": "NEW row 0 at measured ready; no stale-row deletion",
        "robot_pose_at_old_observation": old_observation_pose.tolist(),
        "robot_pose_at_new_observation": new_observation_pose.tolist(),
        "robot_pose_at_new_ready": ready_pose.tolist(),
        "new_world_anchor": "robot pose at NEW observation, never NEW ready pose",
        "controller_reference_policy": "prepend ready pose only; preserve all NEW world rows",
        "old_response": old_response,
        "new_response": new_response,
        "raw_sha256": {},
    }
    metadata["raw_sha256"] = {
        "old_actions.npy": sha256_file(trial_dir / "raw/old_actions.npy"),
        "new_actions.npy": sha256_file(trial_dir / "raw/new_actions.npy"),
    }
    save_json_exclusive(trial_dir / "metadata.json", metadata)
    output_validation = validate_trial_output(trial_dir)
    save_json_exclusive(trial_dir / "results/validation.json", output_validation)
    print(
        "EXP01B_TRIAL="
        + json.dumps(
            {
                "trial_index": trial_index,
                "valid": switch_payload["valid"],
                "rtf": measured_timing.real_time_factor,
                "translation_gap_m": switch_payload["metrics"]["translation_gap_m"],
                "yaw_gap_rad": switch_payload["metrics"]["yaw_gap_rad"],
                "motion_m": switch_payload["observation_to_ready_translation_m"],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return switch_payload


def run() -> Path:
    config = load_config(ARGS.config)
    lightnav = config["lightnav"]
    simulation = config["simulation"]
    robot_config = config["robot"]
    scene = config["scene"]
    camera = config["camera"]
    protocol = config["experiment_protocol"]
    if bool(lightnav["intrinsic_waypoint_time_base"]):
        raise ValueError("EXP-01B must not fabricate a waypoint time base")
    output_root = resolve_path(config["paths"]["output_root"])
    experiment_dir = output_root / ARGS.experiment_id
    experiment_dir.mkdir(parents=True, exist_ok=False)

    client = OnlineLightNavClient(
        ARGS.socket, timeout_s=float(config["ipc"]["request_timeout_s"])
    )
    try:
        server_info_start = client.server_info()
        physics_dt = float(simulation["physics_dt"])
        asset = resolve_jackal_asset(robot_config)
        world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, "/World/Exp01BDomeLight")
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
        add_static_box(
            "/World/CorridorEndMarker",
            [center_x + length / 2.0, 0.0, 0.75],
            [0.08, width * 0.45, 1.5],
            [0.2, 0.45, 0.8],
        )
        reference_path = str(robot_config["reference_prim_path"])
        add_reference_to_stage(str(asset["resolved_path"]), reference_path)
        while is_stage_loading():
            SIMULATION_APP.update()
        articulation_root = find_articulation_root(reference_path)
        initial = validate_pose_se2(simulation["initial_robot_pose_se2"])
        robot = world.scene.add(
            SingleArticulation(
                articulation_root,
                name="exp01b_jackal",
                position=np.array([initial[0], initial[1], simulation["spawn_height_m"]]),
                orientation=quaternion_from_yaw(float(initial[2])),
            )
        )
        aperture = float(camera["horizontal_aperture"])
        hfov = float(camera["horizontal_fov_deg"])
        focal_length = aperture / (2.0 * math.tan(math.radians(hfov) / 2.0))
        camera_prim = rep.functional.create.camera(
            position=tuple(float(value) for value in camera["relative_translation_m"]),
            rotation=tuple(float(value) for value in camera["relative_rotation_xyz_deg"]),
            relative_to=articulation_root,
            focal_length=focal_length,
            horizontal_aperture=aperture,
            clipping_range=tuple(float(value) for value in camera["clipping_range_m"]),
            parent=articulation_root,
            name=str(camera["name"]),
        )
        camera_path = str(camera_prim.GetPath())
        resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
        render_product = rep.create.render_product(camera_path, resolution=resolution)
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annotator.attach(render_product)
        rep.orchestrator.set_capture_on_play(False)
        world.reset()
        if ARGS.gui:
            set_camera_view(
                eye=[1.6, 2.4, 3.5],
                target=[0.4, 0.0, 0.0],
                camera_prim_path="/OmniverseKit_Persp",
            )
        world.step(render=True)
        gate_rgb = capture_rgb(rgb_annotator, resolution, int(camera["render_subframes"]))
        if gate_rgb.shape != (resolution[1], resolution[0], 3):
            raise RuntimeError("coexistence camera gate returned an invalid RGB frame")
        coexistence_gpu = gpu_snapshot()
        coexistence = {
            "passed": True,
            "isaac_camera_rgb_shape": list(gate_rgb.shape),
            "isaac_camera_rgb_dtype": str(gate_rgb.dtype),
            "gpu_after_isaac_and_lightnav_load": coexistence_gpu,
            "oom": False,
        }
        wheels = discover_wheels(robot, articulation_root)
        differential = DifferentialController(
            wheel_radius=float(wheels["radius_m"]),
            wheel_base=float(wheels["separation_m"]),
        )
        controller = robot.get_articulation_controller()
        trials = []
        required_valid = int(protocol["required_valid_trial_count"])
        maximum_trials = int(protocol["maximum_trial_count"])
        if required_valid <= 0 or maximum_trials < required_valid:
            raise ValueError("trial-count protocol is invalid")
        for trial_index in range(maximum_trials):
            trial = run_trial(
                    trial_index=trial_index,
                    trial_dir=experiment_dir / f"trial_{trial_index:03d}",
                    config=config,
                    world=world,
                    robot=robot,
                    controller=controller,
                    differential=differential,
                    wheels=wheels,
                    rgb_annotator=rgb_annotator,
                    resolution=resolution,
                    client=client,
                )
            trials.append(trial)
            if sum(bool(item["valid"]) for item in trials) >= required_valid:
                break
        server_info_end = client.server_info()
        aggregate = aggregate_experiment(trials)
        summary = {
            "experiment": "EXP-01B",
            "experiment_id": ARGS.experiment_id,
            "aggregate": aggregate,
            "claim_boundary": (
                "Measured warmed LightNav + Isaac raw-switch discontinuity only; no "
                "reconciliation method or navigation-improvement claim."
            ),
        }
        save_json_exclusive(experiment_dir / "summary.json", summary)
        save_json_exclusive(
            experiment_dir / "metadata.json",
            {
                "experiment": "EXP-01B",
                "experiment_id": ARGS.experiment_id,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "research_git_commit_sha_at_run": git_sha(),
                "research_git_status_at_run": git_status(),
                "attempted_trial_count": len(trials),
                "required_valid_trial_count": required_valid,
                "maximum_trial_count": maximum_trials,
                "instruction": str(config["instruction"]),
                "intrinsic_waypoint_time_base": False,
                "new_observation_trigger": (
                    "simulation-time delay from OLD execution start; not a waypoint duration"
                ),
                "lightnav_server_start": server_info_start,
                "lightnav_server_end": server_info_end,
                "coexistence_gate": coexistence,
                "isaac": {
                    "version": "6.0.1",
                    "headless_measurement": not ARGS.gui,
                    "physics_dt": physics_dt,
                    "control_dt": float(simulation["control_dt"]),
                    "robot_asset": asset,
                    "robot_prim_path": articulation_root,
                    "actual_dof_names": list(robot.dof_names or []),
                    "canonical_wheel_names": list(canonical_wheel_names(wheels)),
                    "camera_prim_path": camera_path,
                    "camera_resolution": list(resolution),
                },
                "lightnav_gpu_memory_change_reason": (
                    "Reduced configured utilization from 0.90 to 0.65 and explicit KV cache "
                    "from 2 GiB to 1 GiB to leave memory for concurrent Isaac rendering; "
                    "the installed vLLM path notes explicit KV bytes supersede utilization."
                ),
                "protocol": protocol,
                "reporting_thresholds": config["reporting_thresholds"],
            },
        )
        validation = validate_experiment_output(experiment_dir)
        save_json_exclusive(experiment_dir / "validation.json", validation)
        print("EXP01B_OUTPUT_DIR=" + str(experiment_dir), flush=True)
        print("EXP01B_SUMMARY=" + json.dumps(summary, sort_keys=True), flush=True)
        print("EXP01B_VALIDATION=" + json.dumps(validation, sort_keys=True), flush=True)
        return experiment_dir
    finally:
        try:
            client.shutdown_server()
        except Exception as error:
            print(f"[EXP-01B] server shutdown warning: {error}", file=sys.stderr)
        client.close()


try:
    OUTPUT = run()
    if ARGS.gui and ARGS.hold:
        print("[EXP-01B] measurement complete; close the GUI to exit")
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
finally:
    SIMULATION_APP.close()
