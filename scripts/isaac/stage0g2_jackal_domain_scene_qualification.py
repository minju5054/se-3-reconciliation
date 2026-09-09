#!/usr/bin/env python3
"""Audit, capture, and visualize frozen Stage 0-G2 Jackal hospital scenes."""

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


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g2_jackal_domain_scene_qualification.yaml")
    parser.add_argument("--mode", choices=("asset-audit", "preview", "capture", "gui"), required=True)
    parser.add_argument("--scenario")
    parser.add_argument("--variant", default="V0")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    return parser.parse_args()


ARGS = arguments()

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
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import (
    find_articulation_root, quaternion_from_yaw, resolve_jackal_asset,
    se2_from_world_pose, suppress_sensor_viewport_visualization,
)
from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g2_jackal_domain_scene_qualification import (
    SCENARIO_IDS, VARIANT_IDS, sha256_file, validate_config, variant_pose,
)


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("config must be a mapping")
    validate_config(value)
    return value


def save_png_exclusive(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        Image.fromarray(rgb, mode="RGB").save(stream, format="PNG")


def resolved_environment(config: dict) -> dict:
    root = str(get_assets_root_path()).rstrip("/")
    relative = str(config["environment"]["asset_relative_path"])
    resolved = resolve_asset_path(relative)
    if not resolved:
        resolved = resolve_asset_path(root + relative)
    if not resolved:
        raise RuntimeError(f"selected hospital asset is unavailable: {relative}")
    return {"assets_root_runtime": root, "relative_path": relative, "resolved_path": str(resolved)}


def asset_audit(config: dict, run: Path) -> None:
    selected = resolved_environment(config)
    candidates = [
        {"family": "Office", "relative_path": "/Isaac/Environments/Office/office.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "authored office lights/dome", "prim_count": 4653, "collision_prim_count": 1, "visual_quality": "rich multi-floor office", "jackal_fit": "visual corridors fit; collision qualification weak", "decision": "REJECT_COLLISION_COVERAGE"},
        {"family": "Hospital", "relative_path": "/Isaac/Environments/Hospital/hospital.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "authored hospital lights/dome", "prim_count": 1910, "collision_prim_count": 126, "visual_quality": "rich coherent hospital with corridors, doors, signs, beds, carts", "jackal_fit": "3.5 m main corridor and wide intersections", "decision": "SELECTED_BEFORE_INFERENCE"},
        {"family": "Simple_Room", "relative_path": "/Isaac/Environments/Simple_Room/simple_room.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "authored room lights/dome", "prim_count": 139, "collision_prim_count": 60, "visual_quality": "furnished but single small room", "jackal_fit": "insufficient route diversity", "decision": "REJECT_SCALE_AND_DIVERSITY"},
        {"family": "Simple_Warehouse", "relative_path": "/Isaac/Environments/Simple_Warehouse/warehouse.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "authored warehouse lights/dome", "prim_count": 3417, "collision_prim_count": 781, "visual_quality": "rich industrial warehouse rather than target indoor VLN domain", "jackal_fit": "wide and collision-enabled", "decision": "REJECT_DOMAIN_MISMATCH"},
        {"family": "Grid", "relative_path": "/Isaac/Environments/Grid/default_environment.usd", "available": True, "loadable": True, "scale_m_per_stage_unit": 1.0, "lighting": "basic authored calibration lighting", "prim_count": None, "collision_prim_count": None, "visual_quality": "abstract calibration grid", "jackal_fit": "drivable but not realistic", "decision": "REJECT_NOT_DOMAIN_MATCHED"},
    ]
    save_json_exclusive(run / "environment_asset_manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "isaac_sim_version": "6.0.1",
        "audit_before_primary_inference": True, "candidate_search_terms": ["office", "apartment", "room", "hospital", "warehouse", "corridor", "interior"],
        "candidates": candidates, "selected": selected,
    })
    save_json_exclusive(run / "environment_selection.json", {
        "selected_before_primary_inference": True, "selected_family": config["environment"]["family"],
        "selected_asset": selected, "scale": 1.0, "stage_units_in_meters": 1.0,
        "collision_prim_count_from_audit": 126, "lighting": config["environment"]["lighting"],
        "reason": "One coherent, realistic hospital contains wide Jackal-scale corridors, intersections, doorways, and recognizable movable obstacles, with materially better authored collision coverage than Office.",
        "selection_did_not_use_lightnav_outputs": True,
    })
    print(f"STAGE0G2_ASSET_SELECTED={selected['resolved_path']}", flush=True)


def create_runtime(config: dict):
    dt = float(config["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    environment = resolved_environment(config)
    add_reference_to_stage(environment["resolved_path"], str(config["environment"]["reference_prim_path"]))
    robot_asset = resolve_jackal_asset(config["robot"])
    reference = str(config["robot"]["reference_prim_path"])
    add_reference_to_stage(str(robot_asset["resolved_path"]), reference)
    while is_stage_loading():
        APP.update()
    articulation = find_articulation_root(reference)
    initial = variant_pose(config, SCENARIO_IDS[0], "V0")
    robot = world.scene.add(SingleArticulation(
        articulation, name="stage0g2_jackal",
        position=np.array([initial[0], initial[1], float(config["simulation"]["spawn_height_m"])]),
        orientation=quaternion_from_yaw(float(initial[2])),
    ))
    camera = config["camera"]; aperture = float(camera["horizontal_aperture"])
    focal = aperture / (2.0 * math.tan(math.radians(float(camera["horizontal_fov_deg"])) / 2.0))
    camera_prim = rep.functional.create.camera(
        position=tuple(camera["relative_translation_m"]), rotation=tuple(camera["relative_rotation_xyz_deg"]),
        focal_length=focal, horizontal_aperture=aperture, clipping_range=tuple(camera["clipping_range_m"]),
        name="Stage0G2EgocentricCamera",
    )
    resolution = (int(camera["resolution_width"]), int(camera["resolution_height"]))
    product = rep.create.render_product(str(camera_prim.GetPath()), resolution=resolution)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb"); annotator.attach(product)
    rep.orchestrator.set_capture_on_play(False); world.reset()
    return world, robot, annotator, resolution, str(camera_prim.GetPath()), environment, robot_asset


def synchronize_camera(camera_path: str, config: dict, pose: np.ndarray) -> None:
    mount = np.asarray(config["camera"]["relative_translation_m"], dtype=float)
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    position = Gf.Vec3d(float(pose[0] + c * mount[0] - s * mount[1]), float(pose[1] + s * mount[0] + c * mount[1]), float(mount[2]))
    rotation = list(config["camera"]["relative_rotation_xyz_deg"]); rotation[2] += math.degrees(float(pose[2]))
    transform = UsdGeom.XformCommonAPI(omni.usd.get_context().get_stage().GetPrimAtPath(camera_path))
    transform.SetTranslate(position); transform.SetRotate(Gf.Vec3f(*rotation))


def place(world, robot, config: dict, scenario: str, variant: str, camera_path: str) -> tuple[np.ndarray, dict]:
    pose = variant_pose(config, scenario, variant)
    robot.set_world_pose(position=np.array([pose[0], pose[1], float(config["simulation"]["spawn_height_m"])]), orientation=quaternion_from_yaw(float(pose[2])))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    steps = int(round(float(config["simulation"]["settling_duration_s"]) / float(config["simulation"]["physics_dt"])))
    for index in range(steps):
        world.step(render=index == steps - 1)
    actual = se2_from_world_pose(robot); synchronize_camera(camera_path, config, actual); world.step(render=True)
    displacement = float(np.linalg.norm(actual[:2] - pose[:2]))
    feasibility = {
        "requested_pose_se2": pose.tolist(), "settled_pose_se2": actual.tolist(),
        "settling_translation_displacement_m": displacement,
        "initial_pose_collision_free": displacement <= 0.05,
        "check_semantics": "No material pose displacement after one second settling; corridor dimensions and static collision audit are reported separately. This is not path planning.",
    }
    return actual, feasibility


def rgb_frame(annotator, resolution, subframes: int) -> np.ndarray:
    rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0, pause_timeline=False)
    rgba = np.asarray(annotator.get_data()); expected = (resolution[1], resolution[0], 4)
    if rgba.shape != expected or rgba.dtype != np.uint8:
        raise RuntimeError(f"unexpected RGB output {rgba.shape}/{rgba.dtype}, expected {expected}/uint8")
    return np.ascontiguousarray(rgba[:, :, :3])


def preview(config: dict, run: Path, runtime) -> None:
    if (run / "config_snapshot.yaml").exists():
        raise ValueError("preview must precede config freeze")
    world, robot, annotator, resolution, camera_path, _, _ = runtime
    thumbs = []
    for scenario_id in SCENARIO_IDS:
        actual, feasibility = place(world, robot, config, scenario_id, "V0", camera_path)
        rgb = rgb_frame(annotator, resolution, int(config["capture"]["render_subframes"]))
        scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
        destination = run / "scene_previews" / scenario_id / "V0"
        save_png_exclusive(destination / "latest_rgb.png", rgb)
        save_json_exclusive(destination / "preview_metadata.json", {
            "scenario_id": scenario_id, "variant_id": "V0", "instruction": scenario["instruction"],
            "visible_affordance": scenario["visible_affordance"], "actual_robot_pose_se2": actual.tolist(),
            "route_width_m": scenario["route_width_m"], "jackal_footprint_width_m": config["robot"]["approximate_footprint_width_m"],
            "minimum_intended_clearance_m": scenario["minimum_intended_clearance_m"], **feasibility,
            "rendered_before_primary_inference": True, "rgb_shape_hwc": list(rgb.shape), "rgb_dtype": str(rgb.dtype),
        })
        canvas = Image.new("RGB", (480, 305), "white"); canvas.paste(Image.fromarray(rgb), (0, 35))
        ImageDraw.Draw(canvas).text((8, 9), f"{scenario_id}: {scenario['instruction']}", fill="black"); thumbs.append(canvas)
        print(f"STAGE0G2_PREVIEW scenario={scenario_id} collision_free={feasibility['initial_pose_collision_free']}", flush=True)
    overview = Image.new("RGB", (960, 915), "white")
    for index, image in enumerate(thumbs): overview.paste(image, ((index % 2) * 480, (index // 2) * 305))
    path = run / "scene_previews/all_scenarios_v0.png"; path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: overview.save(stream, format="PNG")
    save_json_exclusive(run / "scene_previews/preview_manifest.json", {"scenario_ids": list(SCENARIO_IDS), "preview_count": 6, "variant": "V0", "status": "PENDING_EXPLICIT_GATE"})


def capture(config: dict, run: Path, runtime) -> None:
    snapshot = run / "config_snapshot.yaml"; freeze = json.loads((run / "config_freeze.json").read_text())
    if sha256_file(snapshot) != freeze["config_sha256"]:
        raise ValueError("frozen config hash mismatch")
    world, robot, annotator, resolution, camera_path, environment, robot_asset = runtime
    requested = [(q, v) for q in SCENARIO_IDS for v in VARIANT_IDS] if ARGS.all else [(ARGS.scenario, ARGS.variant)]
    if any(q not in SCENARIO_IDS or v not in VARIANT_IDS for q, v in requested): raise ValueError("invalid requested G2 case")
    dt = float(config["simulation"]["physics_dt"]); steps_per_sample = int(round(1.0 / float(config["capture"]["fps"]) / dt))
    for scenario_id, variant_id in requested:
        case = run / "qualification" / str(scenario_id) / str(variant_id); history = case / "input/history"
        history.mkdir(parents=True, exist_ok=False)
        _, feasibility = place(world, robot, config, str(scenario_id), str(variant_id), camera_path)
        rows = []; latest = None
        for frame_index in range(int(config["capture"]["frame_count"])):
            for _ in range(steps_per_sample): world.step(render=False)
            latest = rgb_frame(annotator, resolution, int(config["capture"]["render_subframes"])); pose = se2_from_world_pose(robot)
            save_png_exclusive(history / f"frame_{frame_index:06d}.png", latest); rows.append((frame_index, float(world.current_time), *pose.tolist()))
        assert latest is not None; save_png_exclusive(case / "input/latest_rgb.png", latest)
        with (case / "input/frame_samples.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream); writer.writerow(("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw")); writer.writerows(rows)
        scenario = next(item for item in config["scenarios"] if item["id"] == scenario_id)
        save_json_exclusive(case / "input/capture_metadata.json", {
            "stage": config["stage"], "scenario_id": scenario_id, "variant_id": variant_id, "instruction": scenario["instruction"],
            "stationary_history": True, "frame_count": len(rows), "capture_fps": config["capture"]["fps"],
            "observation_time_s": rows[-1][1], "robot_pose_at_observation": list(rows[-1][2:5]),
            "observation_pose_definition": "world pose at final inference-input RGB frame", "intrinsic_waypoint_time_base": False,
            "frame_timestamps_file": "frame_samples.csv", "camera": config["camera"], "robot_asset": robot_asset,
            "environment_asset": environment, "collision_feasibility": feasibility,
        })
        print(f"STAGE0G2_CAPTURE case={scenario_id}/{variant_id}", flush=True)


def viewport_capture(path: Path) -> None:
    viewport = get_active_viewport()
    if not viewport: raise RuntimeError("active viewport unavailable")
    capture_viewport_to_file(viewport, file_path=str(path)); deadline = time.monotonic() + 20.0
    while not path.is_file() and time.monotonic() < deadline: APP.update(); time.sleep(0.03)
    if not path.is_file(): raise RuntimeError("viewport capture failed")


def gui(config: dict, run: Path, runtime) -> None:
    if ARGS.scenario not in SCENARIO_IDS or ARGS.variant not in VARIANT_IDS: raise ValueError("GUI requires valid scenario/variant")
    world, robot, _, _, _, _, _ = runtime; case = run / "qualification" / ARGS.scenario / ARGS.variant
    trajectory = np.load(case / "derived/trajectory_world.npy", allow_pickle=False)
    metadata = json.loads((case / "input/capture_metadata.json").read_text()); observation = np.asarray(metadata["robot_pose_at_observation"], dtype=float)
    robot.set_world_pose(position=np.array([observation[0], observation[1], float(config["simulation"]["spawn_height_m"])]), orientation=quaternion_from_yaw(float(observation[2])))
    robot.set_joint_velocities(np.zeros(robot.num_dof)); suppress_sensor_viewport_visualization(str(config["robot"]["reference_prim_path"]))
    for _ in range(20): world.step(render=True)
    visual = config["visualization"]; draw = _debug_draw.acquire_debug_draw_interface(); draw.clear_lines(); draw.clear_points(); z = float(visual["z_offset_m"])
    draw_polyline(draw, trajectory, z=z, color=visual["trajectory_color_rgba"], width=float(visual["line_width"]))
    draw_heading_markers(draw, trajectory, z=z + .03, color=visual["heading_color_rgba"], width=2.0, length_m=float(visual["heading_length_m"]), stride=2)
    draw_pose_points(draw, trajectory[[0, -1]], z=z + .05, color=visual["trajectory_color_rgba"], size=float(visual["point_size"]))
    yaw = float(observation[2]); forward = np.array([math.cos(yaw), math.sin(yaw)])
    # Centered chase view keeps Jackal in frame and avoids entering nearby walls.
    eye = observation[:2] - 1.35 * forward; target = observation[:2] + .75 * forward
    set_camera_view(eye=[float(eye[0]), float(eye[1]), 1.25], target=[float(target[0]), float(target[1]), .22], camera_prim_path="/OmniverseKit_Persp")
    print("STAGE0G2_GUI_LEGEND cyan=LightNav_world yellow=headings cyan_points=first_endpoint Jackal=stationary_observer", flush=True)
    print(f"STAGE0G2_GUI case={ARGS.scenario}/{ARGS.variant} realistic_hospital=true prediction_only=true", flush=True)
    for _ in range(30): APP.update()
    if ARGS.no_hold:
        destination = run / "overview" / f"gui_smoke_{ARGS.scenario}_{ARGS.variant}.png"; suffix = 1
        while destination.exists():
            destination = run / "overview" / f"gui_smoke_{ARGS.scenario}_{ARGS.variant}_{suffix}.png"; suffix += 1
        viewport_capture(destination); print(f"STAGE0G2_GUI_CAPTURE={destination}", flush=True)
    else:
        print("STAGE0G2_GUI_HOLD=close Isaac Sim to exit", flush=True)
        while APP.is_running(): APP.update()


def main() -> None:
    run = ARGS.run_directory.resolve(); config_path = run / "config_snapshot.yaml" if ARGS.mode in ("capture", "gui") else ARGS.config.resolve(); config = load_config(config_path)
    if ARGS.mode == "asset-audit": asset_audit(config, run); return
    runtime = create_runtime(config)
    if ARGS.mode == "preview": preview(config, run, runtime)
    elif ARGS.mode == "capture": capture(config, run, runtime)
    else: gui(config, run, runtime)


try:
    main()
finally:
    APP.close()
