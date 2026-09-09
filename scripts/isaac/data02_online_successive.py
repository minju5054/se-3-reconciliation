#!/usr/bin/env python3
"""Collect or replay genuine online-successive LightNav/Jackal transitions."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/data02_online_successive_v1.yaml")
    parser.add_argument("--socket")
    parser.add_argument("--run-id")
    parser.add_argument("--phase", choices=("preview", "smoke", "primary"), default="primary")
    parser.add_argument("--max-episodes", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--replay-run", type=Path)
    parser.add_argument("--episode")
    parser.add_argument("--transition", type=int)
    parser.add_argument("--no-hold", action="store_true")
    return parser.parse_args()


ARGS = arguments()

from isaacsim import SimulationApp


APP = SimulationApp({"headless": not ARGS.gui})

import carb
import numpy as np
import omni.physx
import omni.replicator.core as rep
import omni.ui as ui
import omni.usd
import yaml
from PIL import Image, ImageDraw
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom, UsdLux

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    canonical_wheel_values,
    discover_wheels,
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    runtime_wheel_command,
    se2_from_world_pose,
    suppress_sensor_viewport_visualization,
)
from reconciliation.closed_loop_execution_validation import load_frozen_stage0d_candidate
from reconciliation.controllers.jackal_execution_controller import (
    JackalExecutionController,
    JackalExecutionControllerConfig,
)
from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.data02_online_successive import (
    SuccessiveEpisodeState,
    TRANSITION_STATUSES,
    TransitionTiming,
    chunk_content_sha256,
    classify_transition,
    observation_anchored_paths,
    ordered_pair_sha256,
    strict_json,
    transition_metrics,
    validate_raw_chunk,
    variant_pose,
)
from reconciliation.data02_v2 import validate_independent_template_bank
from reconciliation.exp01b_extension import is_stop_actions
from reconciliation.lightnav_adapter import save_json_exclusive, save_npy_exclusive
from reconciliation.online_ipc import OnlineLightNavClient, validate_rgb_frame
from reconciliation.online_switch import sha256_file
from reconciliation.se2 import wrap_angle


TELEMETRY_COLUMNS = (
    "sample_index", "sim_time_s", "host_monotonic_ns", "phase",
    "actual_x", "actual_y", "actual_yaw", "measured_v_mps", "measured_omega_rps",
    "desired_v_mps", "desired_omega_rps", "executed_v_mps", "executed_omega_rps",
    "target_front_left_rad_s", "target_front_right_rad_s", "target_rear_left_rad_s", "target_rear_right_rad_s",
    "measured_front_left_rad_s", "measured_front_right_rad_s", "measured_rear_left_rad_s", "measured_rear_right_rad_s",
    "nearest_index", "target_index", "goal_reached", "active_chunk_id",
    "fresh_request_in_flight", "collision_detected", "rgb_frame_index",
)
RGB_INDEX_COLUMNS = (
    "frame_index", "sim_time_s", "actual_x", "actual_y", "actual_yaw",
    "server_observed_frames", "history_buffer_length", "rgb_relative_path", "rgb_file_sha256",
    "capture_host_duration_ms", "observe_host_duration_ms", "rgb_persist_host_duration_ms",
    "inference_queue_depth_at_capture", "persistence_deferred",
)


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("DATA-02 config must be a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def git(command: str) -> str:
    return subprocess.check_output(["git", "-C", str(ROOT), *command.split()], text=True).strip()


def source_status() -> list[str]:
    value = subprocess.check_output(["git", "-C", str(ROOT), "status", "--porcelain"], text=True)
    return value.splitlines()


def validate_config(config: Mapping[str, Any]) -> None:
    if config.get("stage") not in ("data02-online-successive-v1", "data02-online-successive-v2"):
        raise ValueError("wrong DATA-02 identity")
    if len(config.get("episode_templates", [])) < 10:
        raise RuntimeError("DATA02_EPISODE_TEMPLATE_INSUFFICIENT")
    if len(config.get("variants", [])) != 7:
        raise ValueError("DATA-02 requires the frozen seven variants")
    lightnav = config["lightnav"]
    if lightnav["intrinsic_waypoint_time_base"] is not False:
        raise ValueError("LightNav waypoints must remain untimed")
    protocol = config["protocol"]
    if protocol["waypoint_dt"] is not None or protocol["prepend_switch_boundary"] is not False:
        raise ValueError("DATA-02 forbids waypoint timing and boundary prepending")
    if float(protocol["fresh_trigger_delay_sim_s"]) != 0.5:
        raise ValueError("frozen FRESH trigger must be 0.50 sim seconds")
    camera = config["camera"]
    if (int(camera["resolution_width"]), int(camera["resolution_height"]), float(camera["horizontal_fov_deg"])) != (480, 270, 112.0):
        raise ValueError("Stage 0-G2 camera contract changed")
    if config.get("stage") == "data02-online-successive-v2":
        independence = config.get("template_independence", {})
        v1_config = load_yaml(resolve_path(independence["immutable_v1_template_config"]))
        result = validate_independent_template_bank(
            config["episode_templates"],
            v1_config["episode_templates"],
            required_per_family=int(independence["templates_per_family"]),
            translation_threshold_m=float(independence["translation_threshold_m"]),
            yaw_threshold_rad=float(independence["wrapped_yaw_threshold_rad"]),
        )
        if not result["valid"]:
            raise RuntimeError(f"DATA02V2_TEMPLATE_BANK_INSUFFICIENT: {result['checks']}")


def verify_sources(config: Mapping[str, Any]) -> dict[str, Any]:
    checkout = resolve_path(config["paths"]["lightnav_checkout"])
    checkpoint = checkout / str(config["paths"]["checkpoint_relative_path"])
    lightnav_sha = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    if lightnav_sha != config["baseline"]["expected_lightnav_git_sha"]:
        raise ValueError("LightNav git SHA changed")
    hashes = {}
    for relative, expected in config["baseline"]["expected_file_sha256"].items():
        digest = sha256_file(checkpoint / relative)
        if digest != expected:
            raise ValueError(f"checkpoint hash changed: {relative}")
        hashes[relative] = digest
    revision = config["baseline"]["expected_checkpoint_revision"]
    if not (checkpoint / ".cache/huggingface/trees" / f"{revision}.json").is_file():
        raise ValueError("checkpoint revision marker is unavailable")
    stage0d = resolve_path(config["paths"]["stage0d_run"])
    candidate = load_frozen_stage0d_candidate(stage0d, config["stage0d_candidate_provenance"], repository_root=ROOT)
    decision_path = resolve_path(config["paths"]["stage0f_run"]) / "summary/final_lightnav_execution_envelope_decision.json"
    if sha256_file(decision_path) != config["stage0f_provenance"]["decision_sha256"]:
        raise ValueError("Stage 0-F decision hash changed")
    decision = json.loads(decision_path.read_text(encoding="utf-8"))
    if decision["status"] != config["stage0f_provenance"]["status"] or decision["execution_platform_validated"] is not False:
        raise ValueError("Stage 0-F limitation/status changed")
    isaac_version = (Path(os.environ["ISAAC_SIM_ROOT"]) / "VERSION").read_text(encoding="utf-8").strip()
    if not isaac_version.startswith(str(config["baseline"].get("expected_isaac_sim_version_prefix", "6.0.1"))):
        raise ValueError(f"Isaac Sim version changed: {isaac_version}")
    return {
        "isaac_sim_version": isaac_version,
        "lightnav_git_sha": lightnav_sha,
        "lightnav_checkout_clean": subprocess.check_output(["git", "-C", str(checkout), "status", "--porcelain"], text=True).strip() == "",
        "checkpoint_identifier": config["baseline"]["checkpoint_identifier"],
        "checkpoint_revision": revision,
        "checkpoint_file_sha256": hashes,
        "stage0d_candidate": candidate,
        "stage0f_decision": decision,
        "stage0f_decision_sha256": sha256_file(decision_path),
    }


def resolved_asset(relative: str) -> str:
    root = str(get_assets_root_path()).rstrip("/")
    resolved = resolve_asset_path(relative) or resolve_asset_path(root + relative)
    if not resolved:
        raise RuntimeError(f"Isaac asset unavailable: {relative}")
    return str(resolved)


def create_runtime(config: Mapping[str, Any]):
    dt = float(config["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    hospital = resolved_asset(str(config["environment"]["asset_relative_path"]))
    add_reference_to_stage(hospital, str(config["environment"]["reference_prim_path"]))
    robot_asset = resolve_jackal_asset(config["robot"])
    robot_reference = str(config["robot"]["reference_prim_path"])
    add_reference_to_stage(str(robot_asset["resolved_path"]), robot_reference)
    while is_stage_loading():
        APP.update()
    articulation_root = find_articulation_root(robot_reference)
    first = np.asarray(config["episode_templates"][0]["initial_pose_se2"], dtype=float)
    robot = world.scene.add(SingleArticulation(
        articulation_root, name="data02_jackal",
        position=np.array([first[0], first[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(first[2])),
    ))
    camera = config["camera"]
    aperture = float(camera["horizontal_aperture"])
    focal = aperture / (2.0 * math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0))
    camera_prim = rep.functional.create.camera(
        position=tuple(camera["relative_translation_m"]), rotation=tuple(camera["relative_rotation_xyz_deg"]),
        focal_length=focal, horizontal_aperture=aperture,
        clipping_range=tuple(camera["clipping_range_m"]), name=str(camera["name"]),
    )
    resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
    product = rep.create.render_product(str(camera_prim.GetPath()), resolution=resolution)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(product)
    rep.orchestrator.set_capture_on_play(False)
    world.reset()
    wheels = discover_wheels(robot, articulation_root)
    differential = DifferentialController(wheel_radius=float(wheels["radius_m"]), wheel_base=float(wheels["separation_m"]))
    return {
        "world": world, "robot": robot, "articulation": robot.get_articulation_controller(),
        "wheels": wheels, "differential": differential, "annotator": annotator,
        "resolution": resolution, "camera_path": str(camera_prim.GetPath()),
        "hospital": hospital, "robot_asset": robot_asset,
    }


def sync_camera(runtime: Mapping[str, Any], config: Mapping[str, Any], pose: np.ndarray) -> None:
    mount = np.asarray(config["camera"]["relative_translation_m"], dtype=float)
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    position = Gf.Vec3d(float(pose[0] + c * mount[0] - s * mount[1]), float(pose[1] + s * mount[0] + c * mount[1]), float(mount[2]))
    rotation = list(config["camera"]["relative_rotation_xyz_deg"])
    rotation[2] += math.degrees(float(pose[2]))
    transform = UsdGeom.XformCommonAPI(omni.usd.get_context().get_stage().GetPrimAtPath(runtime["camera_path"]))
    transform.SetTranslate(position)
    transform.SetRotate(Gf.Vec3f(*rotation))


def capture_rgb(runtime: Mapping[str, Any], config: Mapping[str, Any], pose: np.ndarray) -> np.ndarray:
    sync_camera(runtime, config, pose)
    rep.orchestrator.step(rt_subframes=int(config["camera"]["render_subframes"]), delta_time=0.0, pause_timeline=False)
    rgba = np.asarray(runtime["annotator"].get_data())
    expected = (runtime["resolution"][1], runtime["resolution"][0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"invalid camera RGB {rgba.shape}/{rgba.dtype}, expected {expected}/uint8")
    return validate_rgb_frame(rgba[:, :, :3]).copy()


def save_png_exclusive(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        Image.fromarray(image, mode="RGB").save(stream, format="PNG")


def persist_buffered_rgb(
    episode: Path,
    buffered: list[tuple[int, np.ndarray]],
    frame_rows: list[dict[str, Any]],
) -> dict[str, float]:
    """Persist exact captured pixels after online execution has finished.

    PNG encoding, hashing, and disk I/O are deliberately excluded from the
    online physics/inference loop.  Capture and server observation order are
    unchanged; only persistence is deferred.
    """

    rows_by_index = {int(row["frame_index"]): row for row in frame_rows}
    durations_ms: list[float] = []
    for frame_index, image in buffered:
        path = episode / "rgb" / f"frame_{frame_index:06d}.png"
        started = time.monotonic_ns()
        save_png_exclusive(path, image)
        row = rows_by_index[frame_index]
        row["rgb_file_sha256"] = sha256_file(path)
        duration_ms = (time.monotonic_ns() - started) / 1e6
        row["rgb_persist_host_duration_ms"] = duration_ms
        durations_ms.append(duration_ms)
    return {
        "frame_count": float(len(durations_ms)),
        "total_host_s": float(sum(durations_ms) / 1000.0),
        "mean_host_ms": float(np.mean(durations_ms)) if durations_ms else 0.0,
        "maximum_host_ms": float(max(durations_ms, default=0.0)),
    }


def save_csv_exclusive(path: Path, rows: list[Mapping[str, Any]], columns: tuple[str, ...]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def collision_paths(runtime: Mapping[str, Any], config: Mapping[str, Any], pose: np.ndarray) -> list[str]:
    half = carb.Float3(
        float(config["robot"]["approximate_footprint_length_m"]) / 2.0,
        float(config["robot"]["approximate_footprint_width_m"]) / 2.0,
        0.10,
    )
    hits: set[str] = set()
    robot_path = str(config["robot"]["reference_prim_path"])
    environment_path = str(config["environment"]["reference_prim_path"])
    def report(hit) -> bool:
        path = str(hit.collision)
        if path.startswith(environment_path) and not path.startswith(robot_path):
            hits.add(path)
        return True
    yaw = float(pose[2])
    rotation = carb.Float4(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    omni.physx.get_physx_scene_query_interface().overlap_box(
        half, carb.Float3(float(pose[0]), float(pose[1]), 0.27), rotation, report, False
    )
    return sorted(hits)


def reset_robot(runtime: Mapping[str, Any], config: Mapping[str, Any], pose: np.ndarray) -> np.ndarray:
    world, robot = runtime["world"], runtime["robot"]
    world.reset()
    robot.set_world_pose(
        position=np.array([pose[0], pose[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(pose[2])),
    )
    zeros = np.zeros(robot.num_dof)
    robot.set_joint_velocities(zeros)
    runtime["articulation"].apply_action(ArticulationAction(joint_velocities=zeros))
    steps = int(round(float(config["simulation"]["settling_duration_s"]) / float(config["simulation"]["physics_dt"])))
    for index in range(steps):
        world.step(render=ARGS.gui or index == steps - 1)
    return se2_from_world_pose(robot)


def follower_config(config: Mapping[str, Any]) -> FollowerConfig:
    values = config["follower"]
    return FollowerConfig(**{key: values[key] for key in (
        "lookahead_distance_m", "position_gain", "heading_gain", "cross_track_gain",
        "max_linear_velocity_mps", "max_angular_velocity_rps", "goal_position_tolerance_m",
        "goal_yaw_tolerance_rad", "rotate_in_place_threshold_rad", "nearest_search_window",
    )})


def correction(config: Mapping[str, Any], sources: Mapping[str, Any], runtime: Mapping[str, Any]) -> JackalExecutionController:
    candidate = sources["stage0d_candidate"]
    parameters = candidate["selected_parameters"]
    controls = candidate["runtime_controller_config"]
    return JackalExecutionController(JackalExecutionControllerConfig(
        physical_wheel_separation_m=float(runtime["wheels"]["separation_m"]),
        calibrated_effective_wheel_separation_m=float(parameters["calibrated_effective_wheel_separation_m"]),
        yaw_rate_kp=float(parameters["yaw_rate_kp"]), yaw_rate_ki=float(parameters["yaw_rate_ki"]),
        maximum_abs_executed_omega_rps=float(controls["maximum_abs_executed_omega_rps"]),
        sign_protection=bool(controls["sign_protection"]),
    ))


def apply_command(runtime: Mapping[str, Any], low_level: JackalExecutionController, desired: np.ndarray, measured_omega: float, control_dt: float, maximum_wheel: float):
    result = low_level.forward(
        desired_v_mps=float(desired[0]), desired_omega_rps=float(desired[1]),
        measured_omega_rps=float(measured_omega), control_dt_s=control_dt,
    )
    executed = np.array([result.executed_v_mps, result.executed_omega_rps])
    side = np.asarray(runtime["differential"].forward(executed), dtype=float)
    side = np.clip(side, -maximum_wheel, maximum_wheel)
    target = runtime_wheel_command(side, runtime["wheels"])
    runtime["articulation"].apply_action(ArticulationAction(joint_velocities=target))
    return executed, target, result


def pace(origin_wall: float, origin_sim: float, current_sim: float, rtf: float) -> None:
    remaining = origin_wall + (current_sim - origin_sim) / rtf - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def render_template_previews(config: Mapping[str, Any], runtime: Mapping[str, Any], run: Path) -> dict[str, Any]:
    destination = run / "template_bank"
    destination.mkdir(parents=True, exist_ok=False)
    thumbs = []
    feasibility = []
    for template in config["episode_templates"]:
        pose = np.asarray(template["initial_pose_se2"], dtype=float)
        actual = reset_robot(runtime, config, pose)
        image = capture_rgb(runtime, config, actual)
        paths = collision_paths(runtime, config, actual)
        record = {
            "template_id": template["id"], "class": template["class"], "physical_region": template["region"],
            "initial_pose_se2": pose.tolist(), "settled_pose_se2": actual.tolist(),
            "instruction": template["instruction"], "visible_affordance": template["visible_affordance"],
            "expected_route": template["expected_route"], "minimum_intended_clearance_m": template["minimum_intended_clearance_m"],
            "independence_note": template.get("independence_note"),
            "preview": f"{template['id']}.png", "collision_paths_at_v0": paths,
            "settling_translation_m": float(np.linalg.norm(actual[:2] - pose[:2])),
            "selected_before_primary_inference": True,
        }
        save_png_exclusive(destination / record["preview"], image)
        feasibility.append(record)
        canvas = Image.new("RGB", (480, 305), "white")
        canvas.paste(Image.fromarray(image), (0, 35))
        ImageDraw.Draw(canvas).text((6, 8), f"{template['id']} | {template['class']} | {template['region']}", fill="black")
        thumbs.append(canvas)
    cols = 3
    rows = math.ceil(len(thumbs) / cols)
    overview = Image.new("RGB", (cols * 480, rows * 305), "white")
    for index, image in enumerate(thumbs):
        overview.paste(image, ((index % cols) * 480, (index // cols) * 305))
    with (destination / "overview.png").open("xb") as stream:
        overview.save(stream, format="PNG")
    # Test all frozen variants by settling, before any primary inference.
    variant_checks = []
    for template in config["episode_templates"]:
        for variant in config["variants"]:
            requested = variant_pose(template["initial_pose_se2"], variant)
            settled = reset_robot(runtime, config, requested)
            hits = collision_paths(runtime, config, settled)
            variant_checks.append({
                "template_id": template["id"], "variant_id": variant["id"],
                "requested_pose_se2": requested.tolist(), "settled_pose_se2": settled.tolist(),
                "settling_translation_m": float(np.linalg.norm(settled[:2] - requested[:2])),
                "collision_paths": hits, "feasible": not hits and float(np.linalg.norm(settled[:2] - requested[:2])) <= 0.05,
            })
    independence = None
    if config.get("stage") == "data02-online-successive-v2":
        contract = config["template_independence"]
        v1_config = load_yaml(resolve_path(contract["immutable_v1_template_config"]))
        independence = validate_independent_template_bank(
            config["episode_templates"],
            v1_config["episode_templates"],
            required_per_family=int(contract["templates_per_family"]),
            translation_threshold_m=float(contract["translation_threshold_m"]),
            yaw_threshold_rad=float(contract["wrapped_yaw_threshold_rad"]),
        )
    all_variants_feasible = all(item["feasible"] for item in variant_checks)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "template_count": len(feasibility),
        "variant_count": len(config["variants"]), "overview": "overview.png", "templates": feasibility,
        "variant_feasibility": variant_checks,
        "all_variants_feasible": all_variants_feasible,
        "independence_validation": independence,
        "template_bank_valid": all_variants_feasible and (independence is None or bool(independence["valid"])),
        "selection_used_lightnav_outputs": False,
    }
    save_json_exclusive(destination / "manifest.json", manifest)
    return manifest


def save_chunk(episode: Path, episode_id: str, chunk_index: int, raw: np.ndarray, response: Mapping[str, Any], observation_pose: np.ndarray, observation_sim_s: float, observation_frame: int, *, ready_sim_s: float, host_latency_s: float, history_first_sim_s: float, initial_history_frames: int) -> dict[str, Any]:
    chunk_id = f"chunk_{chunk_index:02d}"
    directory = episode / "chunks" / chunk_id
    directory.mkdir(parents=True, exist_ok=False)
    raw_copy = validate_raw_chunk(raw)
    local, world = observation_anchored_paths(raw_copy, observation_pose)
    save_npy_exclusive(directory / "raw_actions.npy", raw_copy)
    save_npy_exclusive(directory / "local_path.npy", local)
    save_npy_exclusive(directory / "world_path.npy", world)
    raw_text = str(response.get("raw_text", ""))
    with (directory / "raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(raw_text)
    raw_hash = chunk_content_sha256(raw_copy)
    world_hash = chunk_content_sha256(world)
    metadata = {
        "chunk_id": chunk_id, "episode_id": episode_id, "transition_index_when_created": max(0, chunk_index - 1), "trajectory_chunk_schema": "DATA02_TrajectoryChunk_v1",
        "raw_actions": "raw_actions.npy", "local_path": "local_path.npy", "world_path": "world_path.npy",
        "shape": list(raw_copy.shape), "dtype": str(raw_copy.dtype), "columns": ["x", "y", "yaw"],
        "local_axes": ["forward_m", "lateral_m_left_positive", "yaw_rad_ccw_positive"],
        "world_frame": "Isaac world x/y metres, yaw CCW about +Z radians",
        "observation_pose_world_se2": observation_pose.tolist(), "observation_sim_time_s": float(observation_sim_s),
        "observation_rgb_frame_index": int(observation_frame), "waypoint_dt": None,
        "ready_sim_time_s": float(ready_sim_s), "host_inference_latency_s": float(host_latency_s),
        "total_observed_frame_count": int(observation_frame + 1),
        "live_frame_count": max(0, int(observation_frame + 1 - initial_history_frames)),
        "history_oldest_frame_sim_time_s": float(history_first_sim_s),
        "history_age_at_observation_s": float(observation_sim_s - history_first_sim_s),
        "intrinsic_waypoint_time_base": False, "boundary_inserted": False,
        "raw_sha256": raw_hash, "world_sha256": world_hash,
        "raw_text": "raw_text.txt", "raw_text_sha256": sha256_file(directory / "raw_text.txt"),
        "raw_actions_file_sha256": sha256_file(directory / "raw_actions.npy"),
        "response": {key: response.get(key) for key in (
            "request_id", "prediction_kind", "server_predict_start_monotonic_ns", "server_predict_end_monotonic_ns",
            "server_predict_host_latency_ms", "lightnav_reported_latency_ms", "observed_frames", "history_buffer_length", "internal_timings_ms",
        )},
    }
    save_json_exclusive(directory / "metadata.json", metadata)
    return {"id": chunk_id, "raw": raw_copy, "local": local, "world": world, "metadata": metadata, "directory": directory}


def copy_exclusive(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as incoming, destination.open("xb") as outgoing:
        shutil.copyfileobj(incoming, outgoing)


def save_transition(episode: Path, episode_id: str, template: Mapping[str, Any], variant: Mapping[str, Any], index: int, old: Mapping[str, Any], fresh: Mapping[str, Any], timing: Mapping[str, Any], observation_pose: np.ndarray, model_ready_pose: np.ndarray, pre_pose: np.ndarray, pre_pose_sim_s: float, boundary: np.ndarray, old_command: np.ndarray, fresh_command: np.ndarray, actual: np.ndarray, telemetry_rows: list[Mapping[str, Any]], status: str, collision: list[str]) -> dict[str, Any]:
    transition_id = f"{episode_id}_transition_{index:02d}"
    directory = episode / "transitions" / f"transition_{index:02d}"
    (directory / "raw").mkdir(parents=True, exist_ok=False)
    (directory / "derived").mkdir(parents=True, exist_ok=False)
    copy_exclusive(old["directory"] / "raw_actions.npy", directory / "raw/old_actions.npy")
    copy_exclusive(fresh["directory"] / "raw_actions.npy", directory / "raw/fresh_actions.npy")
    copy_exclusive(old["directory"] / "world_path.npy", directory / "derived/old_world.npy")
    copy_exclusive(fresh["directory"] / "world_path.npy", directory / "derived/fresh_world.npy")
    save_npy_exclusive(directory / "actual.npy", actual)
    save_csv_exclusive(directory / "telemetry.csv", telemetry_rows, TELEMETRY_COLUMNS)
    metrics = transition_metrics(
        old["world"], fresh["world"], observation_pose, pre_pose, boundary,
        old_command, fresh_command, model_ready_pose=model_ready_pose,
    )
    old_hash = old["metadata"]["raw_sha256"]
    fresh_hash = fresh["metadata"]["raw_sha256"]
    pair_hash = ordered_pair_sha256(old_hash, fresh_hash)
    world_pair = ordered_pair_sha256(old["metadata"]["world_sha256"], fresh["metadata"]["world_sha256"])
    artifact_hashes = {relative: sha256_file(directory / relative) for relative in (
        "raw/old_actions.npy", "raw/fresh_actions.npy", "derived/old_world.npy", "derived/fresh_world.npy", "actual.npy", "telemetry.csv",
    )}
    document = {
        "schema": "DATA02_OnlineSuccessiveTransition_v1", "transition_id": transition_id,
        "episode_id": episode_id, "transition_index": index, "template_id": template["id"],
        "variant_id": variant["id"], "status": status, "waypoint_dt": None,
        "old_chunk_id": old["id"], "fresh_chunk_id": fresh["id"],
        "successive_semantics": "FRESH_i is promoted unchanged to OLD_(i+1); no LightNav reset between chunks",
        "fresh_anchoring": "raw FRESH transformed only by robot pose at its final input observation; B is not inserted",
        "old_observation_pose_world_se2": old["metadata"]["observation_pose_world_se2"],
        "old_observation_sim_time_s": old["metadata"]["observation_sim_time_s"],
        "fresh_observation_pose_world_se2": observation_pose.tolist(), "fresh_observation_sim_time_s": timing["t_obs_sim_s"],
        "model_ready_pose_world_se2": model_ready_pose.tolist(),
        "observation_pose_world_se2": observation_pose.tolist(), "pose_immediately_before_switch_P_world_se2": pre_pose.tolist(),
        "pose_immediately_before_switch_P_sim_time_s": float(pre_pose_sim_s),
        "switch_boundary_B_world_se2": boundary.tolist(), "timing": timing, "metrics": metrics,
        "collision_paths": collision,
        "hashes": {"old_raw_sha256": old_hash, "fresh_raw_sha256": fresh_hash, "ordered_raw_pair_sha256": pair_hash,
                   "old_world_sha256": old["metadata"]["world_sha256"], "fresh_world_sha256": fresh["metadata"]["world_sha256"], "ordered_world_pair_sha256": world_pair},
        "artifact_sha256": artifact_hashes,
    }
    save_json_exclusive(directory / "transition.json", document)
    return document


def run_episode(index: int, template: Mapping[str, Any], variant: Mapping[str, Any], config: Mapping[str, Any], sources: Mapping[str, Any], runtime: Mapping[str, Any], client: OnlineLightNavClient, run: Path, *, transition_limit: int) -> dict[str, Any]:
    episode_id = f"episode_{index:06d}"
    episode = run / "episodes" / episode_id
    episode.mkdir(parents=True, exist_ok=False)
    for name in ("rgb", "chunks", "transitions"):
        (episode / name).mkdir()
    initial = variant_pose(template["initial_pose_se2"], variant)
    settled = reset_robot(runtime, config, initial)
    state = SuccessiveEpisodeState(episode_id)
    state.reset()
    client.reset_episode(str(template["instruction"]), index)
    world, robot = runtime["world"], runtime["robot"]
    physics_dt = float(config["simulation"]["physics_dt"])
    control_dt = float(config["simulation"]["control_dt"])
    control_steps = int(round(control_dt / physics_dt))
    rgb_steps = int(round((1.0 / float(config["lightnav"]["video_fps"])) / physics_dt))
    if not math.isclose(control_steps * physics_dt, control_dt, abs_tol=1e-9) or not math.isclose(rgb_steps * physics_dt, 1.0 / float(config["lightnav"]["video_fps"]), abs_tol=1e-9):
        raise ValueError("control/RGB periods must be divisible by physics dt")
    origin_wall, origin_sim = time.monotonic(), float(world.current_time)
    frame_rows: list[dict[str, Any]] = []
    buffered_rgb: list[tuple[int, np.ndarray]] = []
    defer_rgb_persistence = config["protocol"].get("rgb_persistence_mode") == "buffered_episode_end"
    frame_index = 0
    for _ in range(int(config["protocol"]["initial_history_frames"])):
        for step in range(rgb_steps):
            world.step(render=ARGS.gui or step == rgb_steps - 1)
            pace(origin_wall, origin_sim, float(world.current_time), float(config["simulation"]["pace_real_time_factor"]))
        pose = se2_from_world_pose(robot)
        capture_started = time.monotonic_ns()
        rgb = capture_rgb(runtime, config, pose)
        capture_ms = (time.monotonic_ns() - capture_started) / 1e6
        rgb_hash = ""
        persist_ms = 0.0
        if defer_rgb_persistence:
            buffered_rgb.append((frame_index, rgb))
        else:
            persist_started = time.monotonic_ns()
            rgb_path = episode / "rgb" / f"frame_{frame_index:06d}.png"
            save_png_exclusive(rgb_path, rgb)
            rgb_hash = sha256_file(rgb_path)
            persist_ms = (time.monotonic_ns() - persist_started) / 1e6
        observe_started = time.monotonic_ns()
        response = client.observe(rgb, frame_index=frame_index, sim_time_s=float(world.current_time))
        observe_ms = (time.monotonic_ns() - observe_started) / 1e6
        frame_rows.append({
            "frame_index": frame_index, "sim_time_s": float(world.current_time),
            "actual_x": pose[0], "actual_y": pose[1], "actual_yaw": pose[2],
            "server_observed_frames": response["observed_frames"], "history_buffer_length": response["history_buffer_length"],
            "rgb_relative_path": f"rgb/frame_{frame_index:06d}.png", "rgb_file_sha256": rgb_hash,
            "capture_host_duration_ms": capture_ms, "observe_host_duration_ms": observe_ms,
            "rgb_persist_host_duration_ms": persist_ms, "inference_queue_depth_at_capture": 0,
            "persistence_deferred": int(defer_rgb_persistence),
        })
        frame_index += 1
    initial_observation = se2_from_world_pose(robot)
    start_ns = time.monotonic_ns()
    actions, response = client.predict(prediction_kind="old")
    initial_ready_ns = time.monotonic_ns()
    old = save_chunk(
        episode, episode_id, 0, actions, response, initial_observation, float(world.current_time), frame_index - 1,
        ready_sim_s=float(world.current_time), host_latency_s=(initial_ready_ns - start_ns) / 1e9,
        history_first_sim_s=float(frame_rows[0]["sim_time_s"]), initial_history_frames=int(config["protocol"]["initial_history_frames"]),
    )
    # Bootstrap inference is intentionally outside online transition timing.
    origin_wall, origin_sim = time.monotonic(), float(world.current_time)
    state.bootstrap(old["id"])
    follower = TrajectoryFollower(old["world"], follower_config(config))
    command = follower.forward(se2_from_world_pose(robot))
    low_level = correction(config, sources, runtime)
    maximum_wheel = float(sources["stage0d_candidate"]["runtime_controller_config"]["maximum_abs_wheel_target_rad_s"])
    measured_body = np.zeros(2)
    executed = np.zeros(2)
    runtime_target = np.zeros(robot.num_dof)
    current_desired = np.array([command.linear_velocity_mps, command.angular_velocity_rps])
    all_telemetry: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    collision_all: set[str] = set()
    episode_start_sim = float(world.current_time)
    chunk_start_sim = float(world.current_time)
    transition_start_row = 0
    last_control_pose = se2_from_world_pose(robot)
    last_control_sim = float(world.current_time)
    pose_samples: list[tuple[float, np.ndarray]] = [(float(world.current_time), last_control_pose.copy())]
    queued_frames: list[tuple[int, float, np.ndarray]] = []
    request_future = None
    obs_pose = None
    obs_sim = None
    request_ns = None
    obs_frame = None
    old_command_at_request = None
    old_exhausted = False
    fresh_response_ns = None
    model_ready_sim = None
    model_ready_pose = None
    fresh_actions = None
    fresh_response = None
    live_step = 0
    termination = "MAXIMUM_TRANSITIONS"
    wait_exceeded = False
    executor = ThreadPoolExecutor(max_workers=1)
    try:
        while len(transitions) < transition_limit:
            if float(world.current_time) - episode_start_sim > float(config["protocol"]["maximum_episode_sim_s"]):
                termination = "EPISODE_TIMEOUT"
                break
            if command.goal_reached and request_future is None:
                termination = "CHUNK_EXHAUSTED_BEFORE_TRIGGER"
                break
            current_desired = np.array([command.linear_velocity_mps, command.angular_velocity_rps]) if not command.goal_reached else np.zeros(2)
            if command.goal_reached:
                old_exhausted = True
            executed, runtime_target, _ = apply_command(runtime, low_level, current_desired, float(measured_body[1]), control_dt, maximum_wheel)
            canonical_target = canonical_wheel_values(runtime_target, runtime["wheels"])
            for _ in range(control_steps):
                live_step += 1
                capture_due = live_step % rgb_steps == 0
                world.step(render=ARGS.gui or capture_due)
                pace(origin_wall, origin_sim, float(world.current_time), float(config["simulation"]["pace_real_time_factor"]))
                pose = se2_from_world_pose(robot)
                pose_samples.append((float(world.current_time), pose.copy()))
                if request_future is not None and fresh_response is None and request_future.done():
                    fresh_response_ns = time.monotonic_ns()
                    fresh_actions, fresh_response = request_future.result()
                    model_ready_sim = float(world.current_time)
                    model_ready_pose = pose.copy()
                hits = collision_paths(runtime, config, pose)
                collision_all.update(hits)
                measured_wheels = canonical_wheel_values(robot.get_joint_velocities(), runtime["wheels"])
                all_telemetry.append({
                    "sample_index": len(all_telemetry), "sim_time_s": float(world.current_time), "host_monotonic_ns": time.monotonic_ns(),
                    "phase": ("FRESH_READY_PENDING_SWITCH" if fresh_response is not None else "FRESH_IN_FLIGHT") if request_future is not None else "ACTIVE_OLD",
                    "actual_x": pose[0], "actual_y": pose[1], "actual_yaw": pose[2],
                    "measured_v_mps": measured_body[0], "measured_omega_rps": measured_body[1],
                    "desired_v_mps": current_desired[0], "desired_omega_rps": current_desired[1],
                    "executed_v_mps": executed[0], "executed_omega_rps": executed[1],
                    **{f"target_{label}_rad_s": canonical_target[i] for i, label in enumerate(("front_left", "front_right", "rear_left", "rear_right"))},
                    **{f"measured_{label}_rad_s": measured_wheels[i] for i, label in enumerate(("front_left", "front_right", "rear_left", "rear_right"))},
                    "nearest_index": command.nearest_index, "target_index": command.target_index, "goal_reached": int(command.goal_reached),
                    "active_chunk_id": old["id"], "fresh_request_in_flight": int(request_future is not None and fresh_response is None),
                    "collision_detected": int(bool(hits)), "rgb_frame_index": frame_index if capture_due else -1,
                })
                if capture_due:
                    capture_started = time.monotonic_ns()
                    rgb = capture_rgb(runtime, config, pose)
                    capture_ms = (time.monotonic_ns() - capture_started) / 1e6
                    observe_ms = 0.0
                    rgb_hash = ""
                    persist_ms = 0.0
                    if defer_rgb_persistence:
                        buffered_rgb.append((frame_index, rgb))
                    else:
                        persist_started = time.monotonic_ns()
                        rgb_path = episode / "rgb" / f"frame_{frame_index:06d}.png"
                        save_png_exclusive(rgb_path, rgb)
                        rgb_hash = sha256_file(rgb_path)
                        persist_ms = (time.monotonic_ns() - persist_started) / 1e6
                    if request_future is None:
                        observe_started = time.monotonic_ns()
                        response_observe = client.observe(rgb, frame_index=frame_index, sim_time_s=float(world.current_time))
                        observe_ms = (time.monotonic_ns() - observe_started) / 1e6
                        observed_frames = response_observe["observed_frames"]
                        history_length = response_observe["history_buffer_length"]
                    else:
                        queued_frames.append((frame_index, float(world.current_time), rgb))
                        observed_frames = -1
                        history_length = -1
                    frame_rows.append({
                        "frame_index": frame_index, "sim_time_s": float(world.current_time),
                        "actual_x": pose[0], "actual_y": pose[1], "actual_yaw": pose[2],
                        "server_observed_frames": observed_frames, "history_buffer_length": history_length,
                        "rgb_relative_path": f"rgb/frame_{frame_index:06d}.png", "rgb_file_sha256": rgb_hash,
                        "capture_host_duration_ms": capture_ms, "observe_host_duration_ms": observe_ms,
                        "rgb_persist_host_duration_ms": persist_ms,
                        "inference_queue_depth_at_capture": len(queued_frames),
                        "persistence_deferred": int(defer_rgb_persistence),
                    })
                    frame_index += 1
                    if request_future is None and float(world.current_time) - chunk_start_sim >= float(config["protocol"]["fresh_trigger_delay_sim_s"]) - 1e-9:
                        obs_pose = pose.copy(); obs_sim = float(world.current_time); obs_frame = frame_index - 1
                        request_ns = time.monotonic_ns(); old_command_at_request = current_desired.copy()
                        state.start_request(); request_future = executor.submit(client.predict, prediction_kind="new")
                    # Capture/rendering is one measured host-side blocking region.
                    # Poll again without advancing simulation so a response that
                    # completed during capture is not delayed to the next step.
                    if (
                        config.get("stage") == "data02-online-successive-v2"
                        and request_future is not None
                        and fresh_response is None
                        and request_future.done()
                    ):
                        fresh_response_ns = time.monotonic_ns()
                        fresh_actions, fresh_response = request_future.result()
                        model_ready_sim = float(world.current_time)
                        model_ready_pose = pose.copy()
                if hits:
                    termination = "EXECUTION_COLLISION"
                if float(np.linalg.norm(pose[:2] - initial[:2])) > float(config["protocol"]["out_of_envelope_translation_from_start_m"]):
                    termination = "EXECUTION_OUT_OF_ENVELOPE"
                if request_future is not None and obs_sim is not None and float(world.current_time) - obs_sim > float(config["protocol"]["maximum_fresh_wait_sim_s"]):
                    wait_exceeded = True
            control_pose = se2_from_world_pose(robot)
            pre_control_pose = last_control_pose.copy()
            pre_control_sim = last_control_sim
            delta = control_pose[:2] - last_control_pose[:2]
            forward = np.array([math.cos(float(last_control_pose[2])), math.sin(float(last_control_pose[2]))])
            measured_body = np.array([float(delta @ forward / control_dt), float(wrap_angle(control_pose[2] - last_control_pose[2]) / control_dt)])
            last_control_pose = control_pose.copy()
            last_control_sim = float(world.current_time)
            command = follower.forward(control_pose)
            if request_future is not None and fresh_response is not None:
                assert fresh_response_ns is not None and model_ready_sim is not None and model_ready_pose is not None
                ready_sim = model_ready_sim
                queued_count_at_ready = len(queued_frames)
                flush_started_ns = time.monotonic_ns()
                rows_by_index = {int(row["frame_index"]): row for row in frame_rows}
                for queued_index, queued_time, queued_rgb in queued_frames:
                    observe_started = time.monotonic_ns()
                    observed = client.observe(queued_rgb, frame_index=queued_index, sim_time_s=queued_time)
                    row = rows_by_index[queued_index]
                    row["server_observed_frames"] = observed["observed_frames"]
                    row["history_buffer_length"] = observed["history_buffer_length"]
                    row["observe_host_duration_ms"] = (time.monotonic_ns() - observe_started) / 1e6
                queued_flush_s = (time.monotonic_ns() - flush_started_ns) / 1e9
                queued_frames.clear()
                assert obs_pose is not None and obs_sim is not None and request_ns is not None and obs_frame is not None and old_command_at_request is not None
                history_first_index = max(0, obs_frame - int(config["lightnav"]["expected_history_frames"]) + 1)
                fresh = save_chunk(
                    episode, episode_id, len(transitions) + 1, fresh_actions, fresh_response, obs_pose, obs_sim, obs_frame,
                    ready_sim_s=ready_sim, host_latency_s=(fresh_response_ns - request_ns) / 1e9,
                    history_first_sim_s=float(frame_rows[history_first_index]["sim_time_s"]),
                    initial_history_frames=int(config["protocol"]["initial_history_frames"]),
                )
                boundary = se2_from_world_pose(robot)
                # P is the actual robot state at the previous controller boundary,
                # not an arbitrary physics sample one nominal interval earlier.
                pre_pose = pre_control_pose
                fresh_follower = TrajectoryFollower(fresh["world"], follower_config(config))
                first_fresh = fresh_follower.forward(boundary)
                first_fresh_command = np.array([first_fresh.linear_velocity_mps, first_fresh.angular_velocity_rps])
                host_s = (fresh_response_ns - request_ns) / 1e9
                rtf = (ready_sim - obs_sim) / host_s if host_s > 0 else math.inf
                timing_object = TransitionTiming(
                    t_obs_sim_s=obs_sim, t_request_host_ns=request_ns, t_model_ready_host_ns=fresh_response_ns,
                    t_ready_sim_s=ready_sim, t_switch_sim_s=float(world.current_time), host_request_response_s=host_s,
                    model_reported_s=float(fresh_response["lightnav_reported_latency_ms"]) / 1000.0, real_time_factor=rtf,
                )
                timing = timing_object.validate(config["simulation"]["acceptable_rtf_range"])
                server_start_ns = int(fresh_response["server_predict_start_monotonic_ns"])
                server_end_ns = int(fresh_response["server_predict_end_monotonic_ns"])
                timing["technical_profile"] = {
                    "server_prediction_s": (server_end_ns - server_start_ns) / 1e9,
                    "request_dispatch_s": (server_start_ns - request_ns) / 1e9,
                    "main_loop_detection_delay_s": (fresh_response_ns - server_end_ns) / 1e9,
                    "queued_frame_count_at_ready": queued_count_at_ready,
                    "queued_observe_flush_host_s": queued_flush_s,
                    "rgb_persistence_during_online_transition_host_s": 0.0,
                    "response_polled_after_rgb_capture": config.get("stage") == "data02-online-successive-v2",
                }
                stop = is_stop_actions(fresh_actions, absolute_tolerance=float(config["protocol"]["stop_action_absolute_tolerance"]))
                status = classify_transition(
                    model_stop=stop, old_exhausted=old_exhausted or command.goal_reached, timing_valid=bool(timing["valid"]),
                    collision=bool(collision_all), out_of_envelope=termination == "EXECUTION_OUT_OF_ENVELOPE",
                    technical_valid=not wait_exceeded,
                )
                segment_rows = all_telemetry[transition_start_row:]
                actual = np.array([[row["actual_x"], row["actual_y"], row["actual_yaw"]] for row in segment_rows], dtype=float)
                transition = save_transition(
                    episode, episode_id, template, variant, len(transitions), old, fresh, timing,
                    obs_pose, model_ready_pose, pre_pose, pre_control_sim, boundary, current_desired.copy(), first_fresh_command,
                    actual, segment_rows, status, sorted(collision_all),
                )
                transitions.append(transition)
                print(f"DATA02_TRANSITION episode={episode_id} index={len(transitions)-1} status={status} tau={timing['tau_effective_s']:.3f} pair={transition['hashes']['ordered_raw_pair_sha256'][:12]}", flush=True)
                state.promote_fresh(fresh["id"])
                if status not in ("ELIGIBLE_MOVING", "TIMING_INVALID"):
                    termination = status
                    break
                old = fresh; follower = fresh_follower; command = first_fresh
                chunk_start_sim = float(world.current_time); transition_start_row = len(all_telemetry)
                request_future = None; fresh_actions = None; fresh_response = None; model_ready_sim = None; model_ready_pose = None
                obs_pose = None; obs_sim = None; request_ns = None; obs_frame = None
                old_command_at_request = None; old_exhausted = False; collision_all.clear()
                wait_exceeded = False
                pose_samples = [(float(world.current_time), boundary.copy())]
            if termination in ("EXECUTION_COLLISION", "EXECUTION_OUT_OF_ENVELOPE") and request_future is None:
                break
        runtime["articulation"].apply_action(ArticulationAction(joint_velocities=np.zeros(robot.num_dof)))
    finally:
        executor.shutdown(wait=True)
    if defer_rgb_persistence:
        rgb_persistence = persist_buffered_rgb(episode, buffered_rgb, frame_rows)
    else:
        persist_durations = [float(row["rgb_persist_host_duration_ms"]) for row in frame_rows]
        rgb_persistence = {
            "frame_count": float(len(persist_durations)),
            "total_host_s": float(sum(persist_durations) / 1000.0),
            "mean_host_ms": float(np.mean(persist_durations)) if persist_durations else 0.0,
            "maximum_host_ms": float(max(persist_durations, default=0.0)),
        }
    save_csv_exclusive(episode / "telemetry.csv", all_telemetry, TELEMETRY_COLUMNS)
    save_csv_exclusive(episode / "rgb_frames.csv", frame_rows, RGB_INDEX_COLUMNS)
    metadata = {
        "schema": "DATA02_OnlineSuccessiveEpisode_v1", "episode_id": episode_id, "episode_index": index,
        "template_id": template["id"], "template_class": template["class"], "physical_region": template["region"],
        "variant_id": variant["id"], "instruction": template["instruction"],
        "requested_initial_pose_se2": initial.tolist(), "settled_initial_pose_se2": settled.tolist(),
        "lightnav_episode_reset_count": state.reset_count, "transition_count": len(transitions),
        "raw_chunk_count": state.transition_index + 1, "rgb_frame_count": len(frame_rows),
        "initial_history_frame_count": int(config["protocol"]["initial_history_frames"]),
        "live_frame_count": max(0, len(frame_rows) - int(config["protocol"]["initial_history_frames"])),
        "initial_old_request_host_s": (initial_ready_ns - start_ns) / 1e9, "transition_limit": transition_limit,
        "termination_reason": termination, "no_reset_between_successive_chunks": True,
        "cohort_id": str(config.get("cohort_id", "v1")),
        "rgb_persistence_mode": str(config["protocol"].get("rgb_persistence_mode", "synchronous_online")),
        "rgb_persistence_profile": rgb_persistence,
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    save_json_exclusive(episode / "metadata.json", metadata)
    save_json_exclusive(episode / "attempt_complete.json", {"complete": True, "transition_count": len(transitions)})
    return metadata


def initialize_run(config_path: Path, config: Mapping[str, Any], sources: Mapping[str, Any], run: Path, phase: str) -> dict[str, Any]:
    if run.exists():
        if not ARGS.resume:
            raise FileExistsError(f"run already exists: {run}")
        snapshot = run / "config_snapshot.yaml"
        if sha256_file(snapshot) != json.loads((run / "metadata.json").read_text())["config_snapshot_sha256"]:
            raise ValueError("resume config snapshot hash mismatch")
        return json.loads((run / "metadata.json").read_text())
    run.mkdir(parents=True)
    copy_exclusive(config_path, run / "config_snapshot.yaml")
    protocol = {
        "schema": "DATA02_OnlineSuccessiveProtocol_v1",
        "identity": config["stage"],
        "scope": "coverage-oriented data collection only; no reconciliation or optimization",
        "system": {
            "robot": config["robot"], "environment": config["environment"],
            "camera": config["camera"], "lightnav": config["lightnav"],
            "follower": config["follower"], "stage0d_candidate_provenance": config["stage0d_candidate_provenance"],
        },
        "online_architecture": {
            "persistent_model_process": True, "model_built_and_warmed_once_per_invocation": True,
            "lightnav_reset_only_at_episode_start": True, "one_fresh_request_in_flight": True,
            "old_execution_continues_during_fresh_inference": True,
            "fresh_i_becomes_old_i_plus_1_unchanged": True,
        },
        "timing": {
            **config["simulation"], **config["protocol"],
            "clock_domains": ["Isaac simulation seconds", "host monotonic nanoseconds", "LightNav reported milliseconds"],
        },
        "coordinates": {
            "raw_local_axes": config["lightnav"]["axes"],
            "world_frame": "Isaac world x/y metres, yaw CCW about +Z radians",
            "fresh_anchor": "robot world pose at final FRESH input observation frame",
            "boundary_inserted": False, "waypoint_dt": None,
        },
        "transition_statuses": list(TRANSITION_STATUSES),
        "episode_templates": config["episode_templates"], "variants": config["variants"],
        "cohort_id": str(config.get("cohort_id", "v1")),
        "template_independence": config.get("template_independence"),
        "readiness": config["readiness"],
    }
    save_json_exclusive(run / "protocol.json", protocol)
    metadata = {
        "schema": "DATA02_OnlineSuccessiveRun_v1", "run_id": run.name, "phase": phase,
        "cohort_id": str(config.get("cohort_id", "v1")),
        "created_utc": datetime.now(timezone.utc).isoformat(), "repository_root": str(ROOT),
        "collector_git_sha": git("rev-parse HEAD"), "collector_git_status": source_status(),
        "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
        "protocol_sha256": sha256_file(run / "protocol.json"), "sources": sources,
        "research_scope": "data collection only; no reconciliation or optimization",
        "runtime_only_cleanup": {
            "rgb_persistence": str(config["protocol"].get("rgb_persistence_mode", "synchronous_online")),
            "response_poll_after_capture": config.get("stage") == "data02-online-successive-v2",
            "scientific_protocol_changed": False,
        },
        "stage0g_g2_g3_results_rewritten": False,
    }
    if config.get("stage") == "data02-online-successive-v2":
        metadata["data02_v2_collector_git_sha"] = metadata["collector_git_sha"]
    save_json_exclusive(run / "metadata.json", metadata)
    return metadata


def collection_progress(run: Path, completed: int, planned: int) -> str:
    documents = []
    for path in sorted((run / "episodes").glob("episode_*/transitions/transition_*/transition.json")):
        documents.append(json.loads(path.read_text(encoding="utf-8")))
    counts = Counter(str(document["status"]) for document in documents)
    eligible = [document for document in documents if document["status"] == "ELIGIBLE_MOVING"]
    pairs = {document["hashes"]["ordered_raw_pair_sha256"] for document in eligible}
    recent = [float(document["timing"]["real_time_factor"]) for document in documents[-12:]]
    try:
        gpu = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used,memory.free", "--format=csv,noheader,nounits"],
            text=True, timeout=5,
        ).strip().splitlines()[0].replace(", ", "/") + "MiB_used/free"
    except Exception:
        gpu = "unavailable"
    status_text = ",".join(f"{key}:{counts.get(key, 0)}" for key in TRANSITION_STATUSES)
    rtf_text = f"{min(recent):.3f}-{max(recent):.3f}" if recent else "n/a"
    return (
        f"DATA02_PROGRESS episode={completed}/{planned} transitions={len(documents)} "
        f"unique_eligible_pairs={len(pairs)} recent_rtf={rtf_text} gpu={gpu} statuses={status_text}"
    )


def collection_manifest(run: Path) -> dict[str, Any]:
    episodes = []
    for episode in sorted((run / "episodes").glob("episode_*")):
        metadata = strict_json(episode / "metadata.json")
        chunks = sorted((episode / "chunks").glob("chunk_*"))
        transitions = sorted((episode / "transitions").glob("transition_*"))
        episodes.append({
            "episode_id": episode.name, "template_id": metadata["template_id"], "variant_id": metadata["variant_id"],
            "transition_count": len(transitions), "chunk_count": len(chunks), "rgb_frame_count": metadata["rgb_frame_count"],
            "episode_metadata_sha256": sha256_file(episode / "metadata.json"),
            "attempt_complete_sha256": sha256_file(episode / "attempt_complete.json"),
            "telemetry_sha256": sha256_file(episode / "telemetry.csv"),
            "rgb_index_sha256": sha256_file(episode / "rgb_frames.csv"),
            "chunk_metadata_sha256": [sha256_file(path / "metadata.json") for path in chunks],
            "transition_metadata_sha256": [sha256_file(path / "transition.json") for path in transitions],
        })
    return {
        "schema": "DATA02_OnlineSuccessiveCollectionManifest_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "episode_count": len(episodes), "episodes": episodes,
        "template_bank_manifest_sha256": sha256_file(run / "template_bank/manifest.json"),
        "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
        "protocol_sha256": sha256_file(run / "protocol.json"),
    }


def collect(config_path: Path, config: Mapping[str, Any], sources: Mapping[str, Any], runtime: Mapping[str, Any]) -> None:
    if not ARGS.socket:
        raise ValueError("collection requires --socket")
    if ARGS.phase == "primary" and source_status():
        raise RuntimeError("primary collection requires a clean collector worktree")
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime(f"data02-{ARGS.phase}-%Y%m%dT%H%M%SZ")
    run = resolve_path(config["paths"]["output_root"]) / run_id
    metadata = initialize_run(config_path, config, sources, run, ARGS.phase)
    if not (run / "template_bank/manifest.json").is_file():
        manifest = render_template_previews(config, runtime, run)
        if not manifest["template_bank_valid"]:
            status = str(config["protocol"].get("template_bank_insufficient_status", "DATA02_EPISODE_TEMPLATE_INSUFFICIENT"))
            save_json_exclusive(run / "final_status.json", {
                "status": status,
                "reason": "one or more pre-inference feasibility or independence checks failed",
                "all_variants_feasible": manifest["all_variants_feasible"],
                "independence_valid": None if manifest["independence_validation"] is None else manifest["independence_validation"]["valid"],
            })
            raise RuntimeError(status)
    client = OnlineLightNavClient(ARGS.socket, timeout_s=float(config["ipc"]["request_timeout_s"]))
    server_info = client.server_info()
    invocation = run / "invocations" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    invocation.mkdir(parents=True, exist_ok=False)
    save_json_exclusive(invocation / "server_info.json", server_info)
    planned = [(template, variant) for template in config["episode_templates"] for variant in config["variants"]]
    if ARGS.phase == "smoke":
        planned = planned[:1]
    if ARGS.max_episodes is not None:
        planned = planned[:ARGS.max_episodes]
    completed = []
    final_server_info = None
    try:
        for index, (template, variant) in enumerate(planned):
            episode = run / "episodes" / f"episode_{index:06d}"
            if episode.exists():
                print(f"DATA02_RESUME_SKIP episode={episode.name} reason=already_attempted", flush=True)
                continue
            completed.append(run_episode(index, template, variant, config, sources, runtime, client, run, transition_limit=(3 if ARGS.phase == "smoke" else int(config["protocol"]["maximum_transitions_per_episode"]))))
            print(collection_progress(run, index + 1, len(planned)), flush=True)
        final_server_info = client.server_info()
    finally:
        try:
            client.shutdown_server()
        finally:
            client.close()
    save_json_exclusive(invocation / "completion.json", {
        "completed_utc": datetime.now(timezone.utc).isoformat(), "episodes_completed_this_invocation": len(completed),
        "planned_episode_count": len(planned), "final_server_info": final_server_info,
    })
    attempted = list((run / "episodes").glob("episode_*"))
    if len(attempted) == len(planned) and all((path / "attempt_complete.json").is_file() for path in attempted):
        maximum_transition_count = sum(
            int(strict_json(path / "metadata.json")["transition_limit"])
            for path in attempted
        )
        save_json_exclusive(run / "collection_complete.json", {
            "complete": True, "episode_count": len(attempted), "maximum_transition_count": maximum_transition_count,
            "full_primary_grid": ARGS.phase == "primary" and len(attempted) == len(config["episode_templates"]) * len(config["variants"]),
        })
        save_json_exclusive(run / "collection_manifest.json", collection_manifest(run))
    print(f"DATA02_RUN_DIRECTORY={run}", flush=True)


def replay(config: Mapping[str, Any], runtime: Mapping[str, Any]) -> None:
    if not ARGS.gui or ARGS.replay_run is None or ARGS.episode is None or ARGS.transition is None:
        raise ValueError("saved replay requires --replay-run, --episode, --transition, and --gui")
    run = ARGS.replay_run.resolve()
    episode = run / "episodes" / ARGS.episode
    transition_dir = episode / "transitions" / f"transition_{ARGS.transition:02d}"
    transition = json.loads((transition_dir / "transition.json").read_text())
    actual = np.load(transition_dir / "actual.npy", allow_pickle=False)
    old = np.load(transition_dir / "derived/old_world.npy", allow_pickle=False)
    fresh = np.load(transition_dir / "derived/fresh_world.npy", allow_pickle=False)
    telemetry = list(csv.DictReader((transition_dir / "telemetry.csv").open(encoding="utf-8")))
    initial = actual[0]
    reset_robot(runtime, config, initial)
    suppress_sensor_viewport_visualization(str(config["robot"]["reference_prim_path"]))
    visual = config["visualization"]
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Data02SavedReplayDomeLight")
    dome.CreateIntensityAttr(float(visual["gui_replay_dome_light_intensity"]))
    draw = _debug_draw.acquire_debug_draw_interface(); draw.clear_lines(); draw.clear_points()
    z = float(visual["z_offset_m"])
    draw_polyline(draw, old, z=z, color=visual["old_color_rgba"], width=float(visual["line_width"]))
    draw_polyline(draw, fresh, z=z + .03, color=visual["fresh_color_rgba"], width=float(visual["line_width"]))
    for key, color in (("observation_pose_world_se2", visual["observation_color_rgba"]), ("pose_immediately_before_switch_P_world_se2", visual["p_color_rgba"]), ("switch_boundary_B_world_se2", visual["boundary_color_rgba"])):
        draw_pose_points(draw, np.asarray([transition[key]]), z=z + .1, color=color, size=float(visual["marker_size"]))
    # The Hospital ceiling occludes a high top-down camera. A low fixed chase
    # view keeps both the official Jackal mesh and its short transition motion in
    # frame without turning the saved replay into a physics re-execution.
    yaw = float(initial[2])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-forward[1], forward[0]])
    eye = initial[:2] - float(visual["gui_replay_camera_back_m"]) * forward + float(visual["gui_replay_camera_left_m"]) * left
    target = initial[:2] + float(visual["gui_replay_camera_lookahead_m"]) * forward
    key_light = UsdLux.SphereLight.Define(stage, "/World/Data02SavedReplayKeyLight")
    key_light.CreateIntensityAttr(float(visual["gui_replay_sphere_light_intensity"]))
    key_light.CreateRadiusAttr(0.35)
    UsdGeom.XformCommonAPI(key_light).SetTranslate(Gf.Vec3d(float(eye[0]), float(eye[1]), 2.0))
    set_camera_view(
        eye=[float(eye[0]), float(eye[1]), float(visual["gui_replay_camera_height_m"])],
        target=[float(target[0]), float(target[1]), .22],
        camera_prim_path="/OmniverseKit_Persp",
    )
    panel = ui.Window("DATA-02 saved transition replay", width=520, height=155)
    with panel.frame:
        with ui.VStack(spacing=4):
            ui.Label(f"{ARGS.episode} / transition {ARGS.transition:02d}  |  SAVED-ONLY (no LightNav inference)")
            phase_label = ui.Label("phase=INITIALIZING")
            pose_label = ui.Label("pose=[]")
            ui.Label("blue OLD | magenta FRESH | green actual | yellow obs | orange P | red B/switch")
    for _ in range(30):
        APP.update()
    previous = float(telemetry[0]["sim_time_s"])
    print("DATA02_GUI_LEGEND blue=OLD magenta=raw_FRESH green=actual yellow=observation orange=P red=B Jackal=saved_physical_execution_replay", flush=True)
    for index, pose in enumerate(actual):
        runtime["robot"].set_world_pose(position=np.array([pose[0], pose[1], float(config["simulation"]["spawn_height_m"])]), orientation=quaternion_from_yaw(float(pose[2])))
        runtime["robot"].set_joint_velocities(np.zeros(runtime["robot"].num_dof))
        if index > 1:
            draw_polyline(draw, actual[:index + 1], z=z + .06, color=visual["actual_color_rgba"], width=float(visual["line_width"]))
        current = float(telemetry[index]["sim_time_s"])
        phase_label.text = f"phase={telemetry[index]['phase']}  sample={index}/{len(actual)-1}  sim_time={current:.3f} s"
        pose_label.text = f"Jackal world SE(2)=[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]"
        APP.update()
        time.sleep(max(0.0, (current - previous) / float(visual["replay_real_time_factor"])))
        previous = current
        if index % 6 == 0:
            print(f"DATA02_GUI phase={telemetry[index]['phase']} sample={index}/{len(actual)-1} pose={[round(float(x),3) for x in pose]}", flush=True)
    phase_label.text = "phase=FINISHED | B/switch reached | saved_only=true"
    print("DATA02_GUI phase=FINISHED saved_only=true no_inference=true", flush=True)
    if ARGS.no_hold:
        output = run / "gui_replays" / f"{ARGS.episode}_transition_{ARGS.transition:02d}.png"
        output.parent.mkdir(parents=True, exist_ok=True)
        suffix = 1
        while output.exists():
            output = output.with_name(f"{ARGS.episode}_transition_{ARGS.transition:02d}_{suffix}.png")
            suffix += 1
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("active viewport unavailable")
        capture_viewport_to_file(viewport, file_path=str(output))
        deadline = time.monotonic() + 20.0
        minimum_refresh = time.monotonic() + 1.0
        while (not output.is_file() or time.monotonic() < minimum_refresh) and time.monotonic() < deadline:
            APP.update(); time.sleep(.03)
        if not output.is_file():
            raise RuntimeError("GUI replay screenshot failed")
        with Image.open(output) as screenshot:
            extrema = screenshot.convert("RGB").getextrema()
        if all(maximum == 0 for _, maximum in extrema):
            raise RuntimeError("GUI replay screenshot is fully black; Hospital/Jackal were not visible")
        print(f"DATA02_GUI_CAPTURE={output}", flush=True)
    else:
        while APP.is_running(): APP.update()


def main() -> None:
    config_path = (
        ARGS.replay_run.resolve() / "config_snapshot.yaml"
        if ARGS.replay_run is not None
        else ARGS.config.resolve()
    )
    config = load_yaml(config_path)
    validate_config(config)
    sources = verify_sources(config)
    runtime = create_runtime(config)
    sources = dict(sources)
    sources["resolved_assets"] = {
        "hospital": runtime["hospital"], "jackal": runtime["robot_asset"],
    }
    if ARGS.replay_run is not None:
        replay(config, runtime)
    elif ARGS.phase == "preview":
        run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("data02-preview-%Y%m%dT%H%M%SZ")
        run = resolve_path(config["paths"]["output_root"]) / run_id
        initialize_run(config_path, config, sources, run, "preview")
        manifest = render_template_previews(config, runtime, run)
        print(
            f"DATA02_TEMPLATE_PREVIEW={run} feasible={manifest['all_variants_feasible']} "
            f"template_bank_valid={manifest['template_bank_valid']}",
            flush=True,
        )
    else:
        collect(config_path, config, sources, runtime)


try:
    main()
finally:
    APP.close()
