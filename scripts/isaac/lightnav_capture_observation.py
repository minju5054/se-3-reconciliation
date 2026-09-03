#!/usr/bin/env python3
"""Capture one stationary Jackal egocentric RGB history for Stage 0-C."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_lightnav_single_chunk.yaml",
    )
    parser.add_argument("--run-id")
    parser.add_argument("--hold", action="store_true", help="keep GUI open after capture")
    parser.add_argument("--headless", action="store_true", help="explicit diagnostic override")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.replicator.core as rep
import omni.usd
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from lightnav_stage0c_runtime import (
    camera_prims_below,
    find_articulation_root,
    finite_positive,
    load_config,
    quaternion_from_yaw,
    require_mapping,
    resolve_jackal_asset,
    se2_from_world_pose,
)
from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.trajectory import validate_pose_se2


def add_static_box(path: str, center, size, color) -> None:
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*[float(value) for value in color])])
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*[float(value) for value in center]))
    xform.SetScale(Gf.Vec3f(*[float(value) for value in size]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def git_sha() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], text=True
    ).strip()


def run() -> Path:
    config = load_config(ARGS.config)
    simulation = require_mapping(config, "simulation")
    robot_config = require_mapping(config, "robot")
    scene = require_mapping(config, "scene")
    camera_config = require_mapping(config, "camera")
    capture = require_mapping(config, "capture")
    paths = require_mapping(config, "paths")
    physics_dt = finite_positive(simulation, "physics_dt")
    fps = finite_positive(capture, "fps")
    frame_count = int(capture["frame_count"])
    if frame_count < 1:
        raise ValueError("capture.frame_count must be positive")
    sample_steps_float = 1.0 / fps / physics_dt
    sample_steps = int(round(sample_steps_float))
    if not math.isclose(sample_steps_float, sample_steps, abs_tol=1e-9):
        raise ValueError("capture FPS period must be divisible by physics_dt")
    initial_pose = validate_pose_se2(simulation["initial_robot_pose_se2"])
    output_root = Path(str(paths["output_root"]))
    if not output_root.is_absolute():
        output_root = REPOSITORY_ROOT / output_root
    run_id = ARGS.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = output_root / run_id
    rgb_dir = run_dir / "raw/rgb"
    rgb_dir.mkdir(parents=True, exist_ok=False)

    asset = resolve_jackal_asset(robot_config)
    world = World(physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Stage0CDomeLight")
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
    official_cameras = camera_prims_below(reference_path)
    articulation_root = find_articulation_root(reference_path)
    robot = world.scene.add(
        SingleArticulation(
            articulation_root,
            name="stage0c_jackal",
            position=np.array([initial_pose[0], initial_pose[1], simulation["spawn_height_m"]]),
            orientation=quaternion_from_yaw(float(initial_pose[2])),
        )
    )

    aperture = float(camera_config["horizontal_aperture"])
    hfov_deg = float(camera_config["horizontal_fov_deg"])
    focal_length = aperture / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    camera_prim = rep.functional.create.camera(
        position=tuple(float(value) for value in camera_config["relative_translation_m"]),
        rotation=tuple(float(value) for value in camera_config["relative_rotation_xyz_deg"]),
        relative_to=articulation_root,
        focal_length=focal_length,
        horizontal_aperture=aperture,
        clipping_range=tuple(float(value) for value in camera_config["clipping_range_m"]),
        parent=articulation_root,
        name=str(camera_config["name"]),
    )
    camera_path = str(camera_prim.GetPath())
    resolution = (int(camera_config["resolution_width"]), int(camera_config["resolution_height"]))
    render_product = rep.create.render_product(camera_path, resolution=resolution)
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(render_product)
    rep.orchestrator.set_capture_on_play(False)

    world.reset()
    print(
        "[Stage 0-C] CAPTURE ONLY: Jackal intentionally remains stationary while the "
        "LightNav RGB history is recorded. Motion occurs in the final playback step."
    )
    settling_steps = int(round(float(simulation["settling_duration_s"]) / physics_dt))
    for _ in range(settling_steps):
        world.step(render=True)
    frame_rows = []
    for frame_index in range(frame_count):
        for _ in range(sample_steps):
            if not SIMULATION_APP.is_running():
                raise RuntimeError("Isaac Sim closed before RGB capture completed")
            world.step(render=True)
        rep.orchestrator.step(
            rt_subframes=int(capture["render_subframes"]),
            delta_time=0.0,
            pause_timeline=False,
        )
        rgba = np.asarray(rgb_annotator.get_data())
        if rgba.shape != (resolution[1], resolution[0], 4) or rgba.dtype != np.uint8:
            raise RuntimeError(f"unexpected RGB annotator output: {rgba.shape} {rgba.dtype}")
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        image_path = rgb_dir / f"frame_{frame_index:06d}.png"
        with image_path.open("xb") as stream:
            Image.fromarray(rgb, mode="RGB").save(stream, format="PNG")
        pose = se2_from_world_pose(robot)
        frame_rows.append((frame_index, float(world.current_time), *pose.tolist()))

    observation_pose = se2_from_world_pose(robot)
    observation_time = float(frame_rows[-1][1])
    with (run_dir / "raw/frame_samples.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("frame_index", "sim_time_s", "robot_x", "robot_y", "robot_yaw"))
        writer.writerows(frame_rows)
    observation_metadata = {
        "stage": str(config["stage"]),
        "run_id": run_id,
        "creation_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "instruction": str(config["instruction"]),
        "frame_count": frame_count,
        "capture_fps": fps,
        "frame_period_s": 1.0 / fps,
        "observation_time_s": observation_time,
        "robot_pose_at_observation": observation_pose.tolist(),
        "observation_pose_definition": "world pose at the last LightNav input RGB frame",
        "robot_asset": asset,
        "robot_prim_path": articulation_root,
        "actual_dof_names": list(robot.dof_names or []),
        "official_asset_camera_prims": official_cameras,
        "camera": {
            "source": str(camera_config["source"]),
            "prim_path": camera_path,
            "parent": articulation_root,
            "relative_translation_m": list(camera_config["relative_translation_m"]),
            "relative_rotation_xyz_deg": list(camera_config["relative_rotation_xyz_deg"]),
            "camera_axes": "USD camera -Z forward, +Y up; mapped to robot +X forward, +Z up",
            "resolution_width": resolution[0],
            "resolution_height": resolution[1],
            "horizontal_fov_deg": hfov_deg,
            "horizontal_aperture": aperture,
            "focal_length": focal_length,
            "rgb_format": "H x W x 3 uint8 RGB sliced from Isaac rgb annotator RGBA",
        },
        "physics_dt": physics_dt,
        "research_evidence": False,
    }
    save_json_exclusive(run_dir / "raw/observation_metadata.json", observation_metadata)
    save_json_exclusive(
        run_dir / "metadata.json",
        {
            "stage": str(config["stage"]),
            "run_id": run_id,
            "research_git_commit_sha_at_capture": git_sha(),
            "instruction": str(config["instruction"]),
            "pipeline": "sequential capture -> inference -> derive -> playback",
            "research_evidence": False,
        },
    )
    print(f"[Stage 0-C] official Jackal camera prims: {official_cameras}")
    print(f"[Stage 0-C] experiment camera: {camera_path}")
    print(f"[Stage 0-C] RGB: {frame_count} frames, shape {(resolution[1], resolution[0], 3)}")
    print(f"[Stage 0-C] observation time: {observation_time}")
    print(f"[Stage 0-C] observation pose: {observation_pose.tolist()}")
    print(f"STAGE0C_RUN_DIRECTORY={run_dir.resolve()}")
    return run_dir


try:
    RUN_DIRECTORY = run()
    if ARGS.hold and not ARGS.headless:
        print("[Stage 0-C] capture complete; close the GUI to exit")
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
finally:
    SIMULATION_APP.close()
