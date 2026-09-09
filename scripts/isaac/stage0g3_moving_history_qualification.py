#!/usr/bin/env python3
"""Check, capture, and replay Stage 0-G3 scripted moving visual histories."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g3_moving_history_qualification.yaml")
    parser.add_argument("--mode", choices=("feasibility", "capture", "gui"), required=True)
    parser.add_argument("--scenario")
    parser.add_argument("--variant", default="V0")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--remaining", action="store_true", help="capture cases without completed capture metadata")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true", help="play once, save a screenshot, and exit")
    parser.add_argument("--replay-hz", type=float, help="GUI pose replay rate; default is frozen 4 Hz")
    return parser.parse_args()


ARGS = arguments()

from isaacsim import SimulationApp


APP = SimulationApp({"headless": ARGS.headless})

import carb
import matplotlib.pyplot as plt
import numpy as np
import omni.physx
import omni.replicator.core as rep
import omni.usd
import yaml
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    find_articulation_root,
    quaternion_from_yaw,
    resolve_jackal_asset,
    se2_from_world_pose,
    suppress_sensor_viewport_visualization,
)
from reconciliation.lightnav_adapter import save_json_exclusive, save_npy_exclusive
from reconciliation.stage0g3_moving_history_qualification import (
    SCENARIO_IDS,
    VARIANT_IDS,
    approach_history,
    load_yaml,
    resolved_scientific_config,
    sha256_file,
    validate_config,
    validate_history_poses,
    validate_timestamps,
)


def load_run_config(run: Path) -> tuple[dict, dict]:
    source = run / "config_snapshot.yaml" if ARGS.mode in ("capture", "gui") else ARGS.config.resolve()
    config = load_yaml(source)
    validate_config(config, ROOT)
    if ARGS.mode in ("capture", "gui"):
        freeze = json.loads((run / "config_freeze.json").read_text(encoding="utf-8"))
        if sha256_file(source) != freeze.get("config_sha256"):
            raise ValueError("frozen Stage 0-G3 config hash mismatch")
    return config, resolved_scientific_config(config, ROOT)


def resolved_environment(scientific: dict) -> dict:
    root = str(get_assets_root_path()).rstrip("/")
    relative = str(scientific["environment"]["asset_relative_path"])
    resolved = resolve_asset_path(relative) or resolve_asset_path(root + relative)
    if not resolved:
        raise RuntimeError(f"frozen Hospital asset is unavailable: {relative}")
    return {"assets_root_runtime": root, "relative_path": relative, "resolved_path": str(resolved)}


def create_runtime(scientific: dict):
    dt = float(scientific["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    environment = resolved_environment(scientific)
    add_reference_to_stage(environment["resolved_path"], str(scientific["environment"]["reference_prim_path"]))
    robot_asset = resolve_jackal_asset(scientific["robot"])
    reference = str(scientific["robot"]["reference_prim_path"])
    add_reference_to_stage(str(robot_asset["resolved_path"]), reference)
    while is_stage_loading():
        APP.update()
    articulation = find_articulation_root(reference)
    robot = world.scene.add(SingleArticulation(
        articulation,
        name="stage0g3_jackal",
        position=np.array([19.0, 23.5, float(scientific["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(-math.pi / 2.0),
    ))
    camera = scientific["camera"]
    aperture = float(camera["horizontal_aperture"])
    focal = aperture / (2.0 * math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0))
    camera_prim = rep.functional.create.camera(
        position=tuple(camera["relative_translation_m"]),
        rotation=tuple(camera["relative_rotation_xyz_deg"]),
        focal_length=focal,
        horizontal_aperture=aperture,
        clipping_range=tuple(camera["clipping_range_m"]),
        name="Stage0G3EgocentricCamera",
    )
    resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
    product = rep.create.render_product(str(camera_prim.GetPath()), resolution=resolution)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(product)
    rep.orchestrator.set_capture_on_play(False)
    world.reset()
    return world, robot, annotator, resolution, str(camera_prim.GetPath()), environment, robot_asset


def synchronize_camera(camera_path: str, scientific: dict, pose: np.ndarray) -> None:
    mount = np.asarray(scientific["camera"]["relative_translation_m"], dtype=float)
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    position = Gf.Vec3d(float(pose[0] + c * mount[0] - s * mount[1]), float(pose[1] + s * mount[0] + c * mount[1]), float(mount[2]))
    rotation = list(scientific["camera"]["relative_rotation_xyz_deg"])
    rotation[2] += math.degrees(float(pose[2]))
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(camera_path)
    transform = UsdGeom.XformCommonAPI(prim)
    transform.SetTranslate(position)
    transform.SetRotate(Gf.Vec3f(*rotation))


def set_robot_pose(robot, scientific: dict, pose: np.ndarray) -> np.ndarray:
    robot.set_world_pose(
        position=np.array([pose[0], pose[1], float(scientific["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(pose[2])),
    )
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    return se2_from_world_pose(robot)


def rgb_frame(annotator, resolution: tuple[int, int], subframes: int) -> np.ndarray:
    rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0, pause_timeline=False)
    rgba = np.asarray(annotator.get_data())
    expected = (resolution[1], resolution[0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"unexpected RGB output {rgba.shape}/{rgba.dtype}; expected {expected}/uint8")
    return np.ascontiguousarray(rgba[:, :, :3])


def save_png_exclusive(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        Image.fromarray(rgb, mode="RGB").save(stream, format="PNG")


def control_cases(run: Path) -> dict:
    value = json.loads((run / "stage0g2_control_reference.json").read_text(encoding="utf-8"))
    if value.get("valid") is not True or value.get("case_count") != 30:
        raise ValueError("Stage 0-G2 control reference is not valid")
    return value["cases"]


def query_collisions(poses: np.ndarray, config: dict, scientific: dict) -> dict:
    history = config["history"]
    margin = float(history["footprint_clearance_margin_m"])
    half = carb.Float3(
        float(scientific["robot"]["approximate_footprint_length_m"]) / 2.0 + margin,
        float(scientific["robot"]["approximate_footprint_width_m"]) / 2.0 + margin,
        float(history["feasibility_box_half_height_m"]),
    )
    z = float(history["feasibility_box_height_center_m"])
    interface = omni.physx.get_physx_scene_query_interface()
    collision_frames = []
    all_hits: set[str] = set()
    for index, pose in enumerate(poses):
        hits: set[str] = set()

        def report(hit) -> bool:
            path = str(hit.collision)
            if not path.startswith(str(scientific["robot"]["reference_prim_path"])):
                hits.add(path)
            return True

        yaw = float(pose[2])
        rotation = carb.Float4(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        interface.overlap_box(half, carb.Float3(float(pose[0]), float(pose[1]), z), rotation, report, False)
        if hits:
            collision_frames.append({"frame_index": index, "collision_paths": sorted(hits)})
            all_hits.update(hits)
    return {
        "feasible": not collision_frames,
        "checked_pose_count": int(len(poses)),
        "collision_frame_count": len(collision_frames),
        "collision_paths": sorted(all_hits),
        "collision_frames": collision_frames,
        "query_semantics": "oriented Jackal footprint box against authored PhysX collision shapes; no controller execution or path planning",
    }


def candidate_distances(config: dict) -> list[float]:
    history = config["history"]
    maximum = int(round(float(history["maximum_common_distance_m"]) * 10))
    minimum = int(round(float(history["minimum_common_distance_m"]) * 10))
    step = int(round(float(history["distance_search_step_m"]) * 10))
    return [value / 10.0 for value in range(maximum, minimum - 1, -step)]


def save_feasibility_plot(path: Path, records: dict, selected: float) -> None:
    figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
    for key, record in records.items():
        poses = np.asarray(record["history_pose_se2"], dtype=float)
        axis.plot(poses[:, 0], poses[:, 1], linewidth=1.5, label=key if key.endswith("/V0") else None)
        axis.scatter(poses[-1, 0], poses[-1, 1], s=12)
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("world x [m]"); axis.set_ylabel("world y [m]")
    axis.grid(True, alpha=.3); axis.legend(fontsize=7)
    axis.set_title(f"Stage 0-G3 proposed scripted histories; common D={selected:.3f} m")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: figure.savefig(stream, format="png", dpi=150)
    plt.close(figure)


def feasibility(config: dict, scientific: dict, run: Path, runtime) -> None:
    if (run / "config_snapshot.yaml").exists():
        raise ValueError("feasibility must precede Stage 0-G3 config freeze")
    world, robot, _, _, _, _, _ = runtime
    robot.set_world_pose(position=np.array([1000.0, 1000.0, 5.0]), orientation=quaternion_from_yaw(0.0))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    for _ in range(5): world.step(render=False)
    controls = control_cases(run)
    evaluations = []
    selected_distance = None
    selected_records = None
    for distance in candidate_distances(config):
        records = {}
        all_feasible = True
        for scenario_id in SCENARIO_IDS:
            for variant_id in VARIANT_IDS:
                key = f"{scenario_id}/{variant_id}"
                final = np.asarray(controls[key]["final_pose_se2"], dtype=np.float64)
                poses = approach_history(final, distance)
                pose_check = validate_history_poses(poses, final, distance)
                collision = query_collisions(poses, config, scientific)
                records[key] = {**pose_check, **collision, "history_pose_se2": poses.tolist()}
                all_feasible = all_feasible and bool(collision["feasible"])
        evaluations.append({
            "distance_m": distance,
            "all_30_histories_feasible": all_feasible,
            "infeasible_cases": sorted(key for key, item in records.items() if not item["feasible"]),
        })
        print(f"STAGE0G3_FEASIBILITY distance_m={distance:.3f} all_30={str(all_feasible).lower()}", flush=True)
        if all_feasible:
            selected_distance, selected_records = distance, records
            break
    if selected_distance is None or selected_records is None:
        save_json_exclusive(run / "approach_feasibility.json", {
            "status": "STAGE0G3_APPROACH_PATH_NOT_FEASIBLE",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "all_30_histories_feasible": False,
            "candidate_evaluations": evaluations,
            "minimum_required_distance_m": config["history"]["minimum_common_distance_m"],
        })
        print("STAGE0G3_APPROACH_PATH_NOT_FEASIBLE", flush=True)
        return
    selected_root = run / "selected_histories"
    for key, record in selected_records.items():
        scenario_id, variant_id = key.split("/")
        save_npy_exclusive(selected_root / scenario_id / variant_id / "history_pose_se2.npy", np.asarray(record["history_pose_se2"], dtype=np.float64))
    save_feasibility_plot(run / "approach_feasibility_paths.png", selected_records, selected_distance)
    serialized = {key: {name: value for name, value in record.items() if name != "history_pose_se2"} for key, record in selected_records.items()}
    save_json_exclusive(run / "approach_feasibility.json", {
        "status": "FEASIBLE",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "selected_common_distance_m": selected_distance,
        "all_30_histories_feasible": True,
        "largest_common_distance_policy": True,
        "candidate_evaluations": evaluations,
        "selected_case_results": serialized,
        "collision_check_is_not_controller_execution": True,
    })
    print(f"STAGE0G3_SELECTED_COMMON_DISTANCE_M={selected_distance:.3f}", flush=True)


def capture(config: dict, scientific: dict, run: Path, runtime) -> None:
    world, robot, annotator, resolution, camera_path, environment, robot_asset = runtime
    freeze = json.loads((run / "config_freeze.json").read_text(encoding="utf-8"))
    feasibility_value = json.loads((run / "approach_feasibility.json").read_text(encoding="utf-8"))
    if sha256_file(run / "approach_feasibility.json") != freeze.get("approach_feasibility_sha256"):
        raise ValueError("frozen approach feasibility hash mismatch")
    selected_distance = float(freeze["selected_common_distance_m"])
    controls = control_cases(run)
    if ARGS.all and ARGS.remaining:
        raise ValueError("choose only one of --all or --remaining")
    all_cases = [(q, v) for q in SCENARIO_IDS for v in VARIANT_IDS]
    if ARGS.all:
        requested = all_cases
    elif ARGS.remaining:
        requested = [(q, v) for q, v in all_cases if not (run / "qualification" / q / v / "input/capture_metadata.json").is_file()]
    else:
        requested = [(ARGS.scenario, ARGS.variant)]
    if any(q not in SCENARIO_IDS or v not in VARIANT_IDS for q, v in requested):
        raise ValueError("invalid requested Stage 0-G3 case")
    dt = float(scientific["simulation"]["physics_dt"])
    steps_per_sample = int(round(1.0 / float(config["history"]["fps"]) / dt))
    subframes = int(scientific["capture"]["render_subframes"])
    scenarios = {item["id"]: item for item in scientific["scenarios"]}
    for scenario_id, variant_id in requested:
        key = f"{scenario_id}/{variant_id}"
        case = run / "qualification" / scenario_id / variant_id
        history_dir = case / "input/history"
        history_dir.mkdir(parents=True, exist_ok=False)
        requested_poses = np.load(run / "selected_histories" / scenario_id / variant_id / "history_pose_se2.npy", allow_pickle=False)
        final = np.asarray(controls[key]["final_pose_se2"], dtype=np.float64)
        validate_history_poses(requested_poses, final, selected_distance)
        # Prime Fabric/RTX with Jackal outside the Hospital before frame 0.
        # This discarded render prevents the preceding case pose from appearing
        # in the next egocentric history and advances no simulation time.
        set_robot_pose(robot, scientific, np.array([1000.0, 1000.0, 0.0]))
        world.render()
        rgb_frame(annotator, resolution, subframes)
        rows = []
        actual_poses = []
        rgb_values = []
        for frame_index, requested_pose in enumerate(requested_poses):
            for _ in range(steps_per_sample): world.step(render=False)
            actual = set_robot_pose(robot, scientific, requested_pose)
            synchronize_camera(camera_path, scientific, actual)
            # Flush the articulation teleport into the render scene without
            # advancing simulation time. Otherwise frame 0 can contain the
            # Jackal at the preceding case's pose while the camera is already
            # at the new pose.
            world.render()
            rgb = rgb_frame(annotator, resolution, subframes)
            actual = se2_from_world_pose(robot)
            save_png_exclusive(history_dir / f"frame_{frame_index:06d}.png", rgb)
            actual_poses.append(actual.copy()); rgb_values.append(rgb)
            rows.append((frame_index, float(world.current_time), *actual.tolist(), *requested_pose.tolist()))
        actual_array = np.asarray(actual_poses, dtype=np.float64)
        motion = validate_history_poses(
            actual_array, final, selected_distance,
            float(config["history"]["final_translation_tolerance_m"]),
            float(config["history"]["final_yaw_tolerance_rad"]),
        )
        timestamps = np.asarray([row[1] for row in rows], dtype=np.float64)
        timing = validate_timestamps(timestamps, float(config["history"]["fps"]))
        latest = rgb_values[-1]
        save_png_exclusive(case / "input/latest_rgb.png", latest)
        save_npy_exclusive(case / "input/history_pose_se2.npy", actual_array)
        save_npy_exclusive(case / "input/requested_history_pose_se2.npy", requested_poses)
        header = ("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw", "requested_x", "requested_y", "requested_yaw")
        for filename in ("frame_samples.csv", "history_pose.csv"):
            with (case / "input" / filename).open("x", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream); writer.writerow(header); writer.writerows(rows)
        consecutive_mae = [float(np.mean(np.abs(rgb_values[i].astype(np.float32) - rgb_values[i - 1].astype(np.float32)))) for i in range(1, len(rgb_values))]
        rgb_diagnostics = {
            "mean_consecutive_rgb_mae_uint8": float(np.mean(consecutive_mae)),
            "max_consecutive_rgb_mae_uint8": float(np.max(consecutive_mae)),
            "first_to_final_rgb_mae_uint8": float(np.mean(np.abs(rgb_values[-1].astype(np.float32) - rgb_values[0].astype(np.float32)))),
            "final_g2_to_g3_rgb_mae_uint8": float(np.mean(np.abs(np.asarray(Image.open(Path(json.loads((run / "stage0g2_control_reference.json").read_text())["primary_run"]) / "qualification" / scenario_id / variant_id / "input/latest_rgb.png").convert("RGB"), dtype=np.float32) - latest.astype(np.float32)))),
        }
        scenario = scenarios[scenario_id]
        capture_metadata = {
            "stage": config["stage"], "scenario_id": scenario_id, "variant_id": variant_id,
            "instruction": scenario["instruction"], "stationary_history": False,
            "history_mode": "moving_scripted", "scripted_history_is_not_controller_execution": True,
            "frame_count": len(rows), "capture_fps": config["history"]["fps"],
            "observation_time_s": rows[-1][1], "robot_pose_at_observation": actual_array[-1].tolist(),
            "frozen_g2_robot_pose_at_observation": final.tolist(),
            "observation_pose_definition": "world pose at final inference-input RGB frame",
            "final_camera_synchronized_from_robot_pose_se2": actual_array[-1].tolist(),
            "camera_robot_final_pose_mismatch": {"translation_m": 0.0, "yaw_rad": 0.0},
            "selected_common_approach_distance_m": selected_distance,
            "motion_diagnostics": motion, "timing_diagnostics": timing, "rgb_diagnostics": rgb_diagnostics,
            "intrinsic_waypoint_time_base": False, "frame_timestamps_file": "frame_samples.csv",
            "camera": scientific["camera"], "robot_asset": robot_asset, "environment_asset": environment,
            "collision_feasibility": feasibility_value["selected_case_results"][key],
        }
        save_json_exclusive(case / "input/capture_metadata.json", capture_metadata)
        print(
            f"STAGE0G3_CAPTURE case={key} distance_m={motion['history_translation_m']:.6f} "
            f"final_translation_error_m={motion['final_translation_mismatch_m']:.3e} rgb_mae={rgb_diagnostics['first_to_final_rgb_mae_uint8']:.3f}",
            flush=True,
        )


def viewport_capture(path: Path) -> None:
    viewport = get_active_viewport()
    if not viewport: raise RuntimeError("active viewport unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 20.0
    while not path.is_file() and time.monotonic() < deadline:
        APP.update(); time.sleep(.03)
    if not path.is_file(): raise RuntimeError("viewport capture failed")


def chase_camera(pose: np.ndarray) -> None:
    yaw = float(pose[2]); forward = np.array([math.cos(yaw), math.sin(yaw)]); left = np.array([-forward[1], forward[0]])
    eye = pose[:2] - 1.25 * forward + .85 * left
    target = pose[:2] + .45 * forward
    set_camera_view(eye=[float(eye[0]), float(eye[1]), 1.15], target=[float(target[0]), float(target[1]), .20], camera_prim_path="/OmniverseKit_Persp")


def gui(config: dict, scientific: dict, run: Path, runtime) -> None:
    if ARGS.scenario not in SCENARIO_IDS or ARGS.variant not in VARIANT_IDS:
        raise ValueError("GUI requires a valid --scenario and --variant")
    world, robot, _, _, _, _, _ = runtime
    case = run / "qualification" / ARGS.scenario / ARGS.variant
    poses = np.load(case / "input/history_pose_se2.npy", allow_pickle=False)
    moving = np.load(case / "derived/trajectory_world.npy", allow_pickle=False)
    control = json.loads((run / "stage0g2_control_reference.json").read_text(encoding="utf-8"))
    g2_run = Path(control["primary_run"])
    stationary = np.load(g2_run / "qualification" / ARGS.scenario / ARGS.variant / "derived/trajectory_world.npy", allow_pickle=False)
    suppress_sensor_viewport_visualization(str(scientific["robot"]["reference_prim_path"]))
    visual = config["visualization"]; z = float(visual["z_offset_m"])
    draw = _debug_draw.acquire_debug_draw_interface(); draw.clear_lines(); draw.clear_points()
    draw_polyline(draw, poses, z=z, color=visual["history_color_rgba"], width=float(visual["line_width"]))
    draw_pose_points(draw, poses[[0, -1]], z=z + .04, color=visual["history_color_rgba"], size=float(visual["point_size"]))
    draw_polyline(draw, moving, z=z + .03, color=visual["moving_prediction_color_rgba"], width=float(visual["line_width"]))
    draw_heading_markers(draw, moving, z=z + .06, color=visual["moving_prediction_color_rgba"], width=2.0, length_m=.18, stride=2)
    draw_polyline(draw, stationary, z=z + .01, color=visual["stationary_prediction_color_rgba"], width=4.0)
    draw_pose_points(draw, poses[[-1]], z=z + .08, color=visual["final_pose_color_rgba"], size=float(visual["point_size"]) + 4.0)
    replay_hz = float(ARGS.replay_hz or visual["gui_replay_hz"])
    if replay_hz <= 0.0: raise ValueError("--replay-hz must be positive")
    print("STAGE0G3_GUI_MODE=SCRIPTED HISTORY REPLAY (visualization only; no controller commands)", flush=True)
    print("STAGE0G3_GUI_LEGEND green=moving_history magenta=G3_moving_history_prediction cyan=G2_stationary_history_prediction yellow=shared_final_observation_pose Jackal=official_scripted_pose", flush=True)
    print(f"STAGE0G3_GUI case={ARGS.scenario}/{ARGS.variant} replay_hz={replay_hz:.3f} persistent={str(not ARGS.no_hold).lower()}", flush=True)

    def replay_once(sleep: bool) -> None:
        for index, pose in enumerate(poses):
            if not APP.is_running():
                break
            actual = set_robot_pose(robot, scientific, pose); chase_camera(actual)
            APP.update()
            print(f"SCRIPTED_HISTORY_REPLAY frame={index:02d}/63 pose=[{actual[0]:.4f},{actual[1]:.4f},{actual[2]:.4f}]", flush=True)
            if sleep: time.sleep(1.0 / replay_hz)

    if ARGS.no_hold:
        replay_once(False)
        for _ in range(30): APP.update()
        destination = run / "overview" / f"gui_scripted_replay_{ARGS.scenario}_{ARGS.variant}.png"
        suffix = 1
        while destination.exists():
            destination = run / "overview" / f"gui_scripted_replay_{ARGS.scenario}_{ARGS.variant}_{suffix}.png"; suffix += 1
        viewport_capture(destination); print(f"STAGE0G3_GUI_CAPTURE={destination}", flush=True)
    else:
        print("STAGE0G3_GUI_HOLD=replay loops until Isaac Sim is closed", flush=True)
        while APP.is_running():
            replay_once(True)
            deadline = time.monotonic() + 1.0
            while APP.is_running() and time.monotonic() < deadline: APP.update(); time.sleep(.03)


def main() -> None:
    run = ARGS.run_directory.resolve()
    config, scientific = load_run_config(run)
    runtime = create_runtime(scientific)
    if ARGS.mode == "feasibility": feasibility(config, scientific, run, runtime)
    elif ARGS.mode == "capture": capture(config, scientific, run, runtime)
    else: gui(config, scientific, run, runtime)


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    APP.close()
