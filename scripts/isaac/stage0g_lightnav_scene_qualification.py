#!/usr/bin/env python3
"""Render/capture Stage 0-G scenes and inspect saved predictions in Isaac GUI."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g_lightnav_scene_qualification.yaml")
    parser.add_argument("--mode", choices=("preview", "capture", "gui"), required=True)
    parser.add_argument("--scenario", choices=("Q0_STRAIGHT", "Q1_LEFT_TURN", "Q2_RIGHT_TURN", "Q3_DOORWAY", "Q4_DETOUR_LEFT", "Q5_DETOUR_RIGHT"))
    parser.add_argument("--variant", choices=("V0", "V1", "V2", "V3", "V4"), default="V0")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.replicator.core as rep
import omni.usd
import yaml
from PIL import Image, ImageDraw
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import find_articulation_root, quaternion_from_yaw, resolve_jackal_asset, se2_from_world_pose, suppress_sensor_viewport_visualization
from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import SCENARIO_IDS, VARIANT_IDS, sha256_file, validate_config, variant_pose


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("config must be a mapping")
    validate_config(value)
    return value


def box(path: str, center, size, color, collision: bool = True) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    xform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def wall(path: str, x: float, y: float, sx: float, sy: float, scene: dict) -> None:
    height = float(scene["wall_height_m"])
    box(path, [x, y, height / 2.0], [sx, sy, height], scene["wall_color_rgb"])


def build_map(config: dict) -> None:
    scene = config["scene"]
    half = float(scene["corridor_half_width_m"])
    thick = float(scene["wall_thickness_m"])
    accent = scene["accent_color_rgb"]
    obstacle = scene["obstacle_color_rgb"]
    # Colored floors only separate six bays; navigability comes from physical walls/openings.
    for scenario in config["scenarios"]:
        q = str(scenario["id"])
        y0 = float(scenario["baseline_pose_se2"][1])
        safe = q.replace("_", "")
        box(f"/World/Stage0G/{safe}/Floor", [3.0, y0, 0.012], [11.0, 9.0, 0.024], scene["floor_color_rgb"], collision=False)
        # Small overhead sign provides a shared visual style, never the only task cue.
        box(f"/World/Stage0G/{safe}/Header", [0.9, y0, 2.02], [0.9, 0.10, 0.14], accent, collision=False)
        if q == "Q0_STRAIGHT":
            wall(f"/World/Stage0G/{safe}/Left", 3.5, y0 + half, 9.0, thick, scene)
            wall(f"/World/Stage0G/{safe}/Right", 3.5, y0 - half, 9.0, thick, scene)
        elif q in ("Q1_LEFT_TURN", "Q2_RIGHT_TURN"):
            branch_sign = 1.0 if q == "Q1_LEFT_TURN" else -1.0
            wall(f"/World/Stage0G/{safe}/IncomingLeft", 0.4, y0 + half, 5.0, thick, scene)
            wall(f"/World/Stage0G/{safe}/IncomingRight", 0.4, y0 - half, 5.0, thick, scene)
            # A solid forward wall makes the T decision visible.
            wall(f"/World/Stage0G/{safe}/ForwardStop", 3.6, y0 - branch_sign * 0.25, thick, 3.5, scene)
            branch_center_y = y0 + branch_sign * 3.6
            wall(f"/World/Stage0G/{safe}/BranchInner", 2.1, branch_center_y, thick, 4.3, scene)
            wall(f"/World/Stage0G/{safe}/BranchOuter", 5.1, branch_center_y, thick, 4.3, scene)
            box(f"/World/Stage0G/{safe}/BranchEndAccent", [3.6, y0 + branch_sign * 5.5, 0.7], [1.0, 0.08, 1.4], accent)
        elif q == "Q3_DOORWAY":
            wall(f"/World/Stage0G/{safe}/NearLeft", 0.7, y0 + half, 5.5, thick, scene)
            wall(f"/World/Stage0G/{safe}/NearRight", 0.7, y0 - half, 5.5, thick, scene)
            wall(f"/World/Stage0G/{safe}/DoorWallLeft", 3.0, y0 + 1.12, thick, 0.78, scene)
            wall(f"/World/Stage0G/{safe}/DoorWallRight", 3.0, y0 - 1.12, thick, 0.78, scene)
            wall(f"/World/Stage0G/{safe}/DoorLintel", 3.0, y0, thick, 1.45, {**scene, "wall_height_m": 0.22, "wall_color_rgb": accent})
            # Lintel is raised to a real doorway height.
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(f"/World/Stage0G/{safe}/DoorLintel")
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(3.0, y0, 1.95))
            wall(f"/World/Stage0G/{safe}/RoomLeft", 5.2, y0 + 2.2, 4.4, thick, scene)
            wall(f"/World/Stage0G/{safe}/RoomRight", 5.2, y0 - 2.2, 4.4, thick, scene)
            box(f"/World/Stage0G/{safe}/RoomAccent", [6.6, y0, 0.8], [0.08, 1.0, 1.6], accent)
        else:
            wall(f"/World/Stage0G/{safe}/Left", 3.5, y0 + half, 9.0, thick, scene)
            wall(f"/World/Stage0G/{safe}/Right", 3.5, y0 - half, 9.0, thick, scene)
            obstacle_y = y0 - 0.32 if q == "Q4_DETOUR_LEFT" else y0 + 0.32
            box(f"/World/Stage0G/{safe}/Obstacle", [2.35, obstacle_y, 0.65], [0.95, 1.15, 1.30], obstacle)
            box(f"/World/Stage0G/{safe}/BeyondAccent", [5.9, y0, 0.7], [0.08, 0.9, 1.4], accent)


def create_runtime(config: dict):
    dt = float(config["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    dome = UsdLux.DomeLight.Define(omni.usd.get_context().get_stage(), "/World/Stage0GDome")
    dome.CreateIntensityAttr(1300.0)
    build_map(config)
    asset = resolve_jackal_asset(config["robot"])
    reference = str(config["robot"]["reference_prim_path"])
    add_reference_to_stage(str(asset["resolved_path"]), reference)
    while is_stage_loading():
        APP.update()
    articulation = find_articulation_root(reference)
    initial = variant_pose(config, SCENARIO_IDS[0], "V0")
    robot = world.scene.add(SingleArticulation(
        articulation, name="stage0g_jackal",
        position=np.array([initial[0], initial[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(initial[2])),
    ))
    camera = config["camera"]
    aperture = float(camera["horizontal_aperture"])
    focal = aperture / (2.0 * math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0))
    camera_prim = rep.functional.create.camera(
        position=tuple(camera["relative_translation_m"]), rotation=tuple(camera["relative_rotation_xyz_deg"]),
        focal_length=focal, horizontal_aperture=aperture,
        clipping_range=tuple(camera["clipping_range_m"]), name="Stage0GEgocentricCamera",
    )
    resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
    product = rep.create.render_product(str(camera_prim.GetPath()), resolution=resolution)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(product)
    rep.orchestrator.set_capture_on_play(False)
    world.reset()
    return world, robot, annotator, resolution, str(camera_prim.GetPath()), asset


def synchronize_camera(camera_path: str, config: dict, pose: np.ndarray) -> None:
    mount = np.asarray(config["camera"]["relative_translation_m"], dtype=float)
    cosine, sine = math.cos(float(pose[2])), math.sin(float(pose[2]))
    position = Gf.Vec3d(
        float(pose[0] + cosine * mount[0] - sine * mount[1]),
        float(pose[1] + sine * mount[0] + cosine * mount[1]),
        float(mount[2]),
    )
    rotation = list(config["camera"]["relative_rotation_xyz_deg"])
    rotation[2] += math.degrees(float(pose[2]))
    camera = UsdGeom.XformCommonAPI(omni.usd.get_context().get_stage().GetPrimAtPath(camera_path))
    camera.SetTranslate(position)
    camera.SetRotate(Gf.Vec3f(*[float(value) for value in rotation]))


def place(world, robot, config: dict, scenario_id: str, variant_id: str, camera_path: str, settle: bool = True) -> np.ndarray:
    pose = variant_pose(config, scenario_id, variant_id)
    robot.set_world_pose(
        position=np.array([pose[0], pose[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(pose[2])),
    )
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    steps = int(round(float(config["simulation"]["settling_duration_s"]) / float(config["simulation"]["physics_dt"]))) if settle else 3
    for index in range(steps):
        world.step(render=index == steps - 1)
    actual = se2_from_world_pose(robot)
    synchronize_camera(camera_path, config, actual)
    world.step(render=True)
    return actual


def rgb_frame(annotator, resolution, subframes: int) -> np.ndarray:
    rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0, pause_timeline=False)
    rgba = np.asarray(annotator.get_data())
    expected = (resolution[1], resolution[0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"unexpected RGB annotator output {rgba.shape} {rgba.dtype}, expected {expected} uint8")
    return np.ascontiguousarray(rgba[:, :, :3])


def save_png_exclusive(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        Image.fromarray(image, mode="RGB").save(stream, format="PNG")


def preview(config: dict, run: Path, world, robot, annotator, resolution, camera_path: str) -> None:
    if (run / "config_snapshot.yaml").exists():
        raise ValueError("scene preview must precede the primary config freeze")
    thumbnails = []
    for scenario_id in SCENARIO_IDS:
        actual = place(world, robot, config, scenario_id, "V0", camera_path)
        image = rgb_frame(annotator, resolution, int(config["capture"]["render_subframes"]))
        destination = run / "scene_previews" / scenario_id / "V0"
        save_png_exclusive(destination / "latest_rgb.png", image)
        scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
        save_json_exclusive(destination / "preview_metadata.json", {
            "scenario_id": scenario_id, "variant_id": "V0", "instruction": scenario["instruction"],
            "geometry": scenario["geometry"], "actual_robot_pose_se2": actual.tolist(),
            "camera_prim_path": camera_path, "rgb_shape_hwc": list(image.shape), "rgb_dtype": str(image.dtype),
            "rendered_before_primary_inference": True,
        })
        thumb = Image.fromarray(image).resize((480, 270))
        canvas = Image.new("RGB", (480, 305), "white")
        canvas.paste(thumb, (0, 35))
        ImageDraw.Draw(canvas).text((8, 9), f"{scenario_id}: {scenario['instruction']}", fill="black")
        thumbnails.append(canvas)
        print(f"STAGE0G_PREVIEW scenario={scenario_id} image={destination / 'latest_rgb.png'}", flush=True)
    overview = Image.new("RGB", (960, 915), "white")
    for index, image in enumerate(thumbnails):
        overview.paste(image, ((index % 2) * 480, (index // 2) * 305))
    overview_path = run / "scene_previews/all_scenarios_v0.png"
    overview_path.parent.mkdir(parents=True, exist_ok=True)
    with overview_path.open("xb") as stream:
        overview.save(stream, format="PNG")
    save_json_exclusive(run / "scene_previews/preview_manifest.json", {
        "scenario_ids": list(SCENARIO_IDS), "variant": "V0", "preview_count": 6,
        "visual_inspection_status": "PENDING_HUMAN_OR_CODEX_INSPECTION", "created_utc": datetime.now(timezone.utc).isoformat(),
    })


def capture(config: dict, run: Path, world, robot, annotator, resolution, camera_path: str, asset: dict) -> None:
    snapshot = run / "config_snapshot.yaml"
    freeze = run / "config_freeze.json"
    if not snapshot.is_file() or not freeze.is_file():
        raise ValueError("capture requires a frozen post-preview config snapshot")
    freeze_data = json.loads(freeze.read_text())
    if sha256_file(snapshot) != freeze_data["config_sha256"]:
        raise ValueError("frozen config hash mismatch")
    requested = [(q, v) for q in SCENARIO_IDS for v in VARIANT_IDS] if ARGS.all else [(ARGS.scenario, ARGS.variant)]
    if any(q is None for q, _ in requested):
        raise ValueError("capture requires --all or --scenario")
    dt = float(config["simulation"]["physics_dt"])
    fps = float(config["capture"]["fps"])
    steps_per_sample = int(round(1.0 / fps / dt))
    frame_count = int(config["capture"]["frame_count"])
    for scenario_id, variant_id in requested:
        case = run / "qualification" / str(scenario_id) / str(variant_id)
        history = case / "input/history"
        history.mkdir(parents=True, exist_ok=False)
        place(world, robot, config, str(scenario_id), str(variant_id), camera_path)
        rows = []
        latest = None
        for frame_index in range(frame_count):
            for _ in range(steps_per_sample):
                world.step(render=False)
            latest = rgb_frame(annotator, resolution, int(config["capture"]["render_subframes"]))
            pose = se2_from_world_pose(robot)
            save_png_exclusive(history / f"frame_{frame_index:06d}.png", latest)
            rows.append((frame_index, float(world.current_time), *pose.tolist()))
        assert latest is not None
        save_png_exclusive(case / "input/latest_rgb.png", latest)
        with (case / "input/frame_samples.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw"))
            writer.writerows(rows)
        observation_pose = rows[-1][2:5]
        scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
        save_json_exclusive(case / "input/capture_metadata.json", {
            "stage": config["stage"], "scenario_id": scenario_id, "variant_id": variant_id,
            "instruction": scenario["instruction"], "stationary_history": True,
            "frame_count": frame_count, "capture_fps": fps, "frame_period_s": 1.0 / fps,
            "observation_time_s": rows[-1][1], "robot_pose_at_observation": list(observation_pose),
            "observation_pose_definition": "world pose at the final LightNav inference-input RGB frame",
            "frame_timestamps_file": "frame_samples.csv", "intrinsic_waypoint_time_base": False,
            "camera": {"prim_path": camera_path, "resolution_wh": list(resolution), "rgb_format": "HWC uint8 RGB", "horizontal_fov_deg": config["camera"]["horizontal_fov_deg"], "relative_translation_m": config["camera"]["relative_translation_m"], "relative_rotation_xyz_deg": config["camera"]["relative_rotation_xyz_deg"]},
            "robot_asset": asset,
        })
        print(f"STAGE0G_CAPTURE case={scenario_id}/{variant_id} observation_pose={list(observation_pose)}", flush=True)


def viewport_capture(path: Path) -> None:
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport unavailable")
    path.parent.mkdir(parents=True, exist_ok=True)
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 20.0
    while not path.is_file() and time.monotonic() < deadline:
        APP.update()
        time.sleep(0.03)
    if not path.is_file():
        raise RuntimeError("viewport capture failed")


def gui(config: dict, run: Path, world, robot) -> None:
    if ARGS.scenario is None:
        raise ValueError("GUI requires --scenario")
    case = run / "qualification" / ARGS.scenario / ARGS.variant
    sensor_override = suppress_sensor_viewport_visualization(str(config["robot"]["reference_prim_path"]))
    world_path = case / "derived/trajectory_world.npy"
    if not world_path.is_file():
        raise FileNotFoundError(f"prediction not found: {world_path}")
    trajectory = np.load(world_path, allow_pickle=False)
    capture_metadata = json.loads((case / "input/capture_metadata.json").read_text())
    observation = np.asarray(capture_metadata["robot_pose_at_observation"], dtype=float)
    robot.set_world_pose(
        position=np.array([observation[0], observation[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(observation[2])),
    )
    for _ in range(20):
        world.step(render=True)
    visual = config["visualization"]
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines(); draw.clear_points()
    z = float(visual["z_offset_m"])
    draw_polyline(draw, trajectory, z=z, color=visual["trajectory_color_rgba"], width=float(visual["line_width"]))
    draw_heading_markers(draw, trajectory, z=z + 0.02, color=visual["heading_color_rgba"], width=2.0, length_m=float(visual["heading_length_m"]), stride=2)
    draw_pose_points(draw, trajectory[[0, -1]], z=z + 0.04, color=visual["trajectory_color_rgba"], size=float(visual["point_size"]))
    # Keep the diagnostic viewport inside the corridor; a lateral exterior view is
    # commonly occluded by the qualification walls. This camera never enters inference.
    yaw = float(observation[2])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    eye_xy = observation[:2] - 1.8 * forward
    target_xy = observation[:2] + 1.3 * forward
    set_camera_view(
        eye=[float(eye_xy[0]), float(eye_xy[1]), 1.6],
        target=[float(target_xy[0]), float(target_xy[1]), 0.22],
        camera_prim_path="/OmniverseKit_Persp",
    )
    print("STAGE0G_GUI_LEGEND cyan=LightNav_world yellow=heading_markers cyan_points=first_and_endpoint", flush=True)
    print("STAGE0G_GUI_SENSOR_VIEW_OVERRIDE=" + json.dumps(sensor_override, sort_keys=True), flush=True)
    print(f"STAGE0G_GUI case={ARGS.scenario}/{ARGS.variant} robot=stationary prediction_only=true", flush=True)
    for _ in range(30):
        APP.update()
    if ARGS.no_hold:
        stem = f"gui_smoke_{ARGS.scenario}_{ARGS.variant}"
        destination = run / "overview" / f"{stem}.png"
        suffix = 1
        while destination.exists():
            destination = run / "overview" / f"{stem}_{suffix}.png"
            suffix += 1
        viewport_capture(destination)
        print(f"STAGE0G_GUI_CAPTURE={destination}", flush=True)
    else:
        print("STAGE0G_GUI_HOLD=close Isaac Sim to exit", flush=True)
        while APP.is_running():
            APP.update()


def main() -> None:
    run = ARGS.run_directory.resolve()
    config_path = (run / "config_snapshot.yaml") if ARGS.mode in ("capture", "gui") else ARGS.config.resolve()
    config = load_yaml(config_path)
    world, robot, annotator, resolution, camera_path, asset = create_runtime(config)
    if ARGS.mode == "preview":
        preview(config, run, world, robot, annotator, resolution, camera_path)
    elif ARGS.mode == "capture":
        capture(config, run, world, robot, annotator, resolution, camera_path, asset)
    else:
        gui(config, run, world, robot)


try:
    main()
finally:
    APP.close()
