#!/usr/bin/env python3
"""Static Isaac RGB/file handoff and passive visualization; no robot or execution."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "visualize"))
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/robotless_lightnav_single_chunk.yaml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--view-only", action="store_true", help="Reopen saved trajectory without writing new evidence")
    return parser.parse_args()


ARGS = arguments()
from isaacsim import SimulationApp

APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from reconciliation.robotless_single_chunk import (
    FRAME_CONVENTIONS, frame_sanity_fixtures, load_config, make_observation_time,
    observation_to_world, save_json_exclusive, sha256_file, validate_observation_metadata,
    validate_scene_frame, validate_waypoints,
)


def git_output(directory: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(directory), *args], text=True).strip()


def runtime_scene(config: dict, pose: np.ndarray) -> tuple:
    validate_scene_frame(config["scene"])
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, float(config["scene"]["stage_units_in_meters"]))
    UsdGeom.Xform.Define(stage, "/World")
    relative = str(config["scene"]["asset_relative_path"])
    root = str(get_assets_root_path()).rstrip("/")
    resolved = resolve_asset_path(relative) or resolve_asset_path(root + relative)
    if not resolved:
        raise RuntimeError(f"Hospital USD is unavailable: {relative}")
    scene_root = stage.DefinePrim(str(config["scene"]["reference_prim_path"]), "Xform")
    if not scene_root.GetReferences().AddReference(str(resolved)):
        raise RuntimeError("Hospital USD reference failed")
    while is_stage_loading():
        APP.update()
    agent = UsdGeom.Xform.Define(stage, str(config["agent"]["prim_path"]))
    xform = UsdGeom.XformCommonAPI(agent)
    xform.SetTranslate(Gf.Vec3d(float(pose[0]), float(pose[1]), float(config["agent"]["z_m"])))
    xform.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(float(pose[2]))))
    cfg = config["camera"]
    camera = UsdGeom.Camera.Define(stage, str(cfg["prim_path"]))
    if camera.GetPrim().GetParent().GetPath() != agent.GetPath():
        raise ValueError("Camera must be a direct child of the logical agent Xform")
    rotation = np.asarray(cfg["rotation_agent_from_camera"], dtype=float)
    if rotation.shape != (3, 3) or not np.allclose(rotation.T @ rotation, np.eye(3)) or not np.isclose(np.linalg.det(rotation), 1.0):
        raise ValueError("Camera rotation must be a proper SO(3) rotation")
    local = np.eye(4)
    local[:3, :3] = rotation
    local[:3, 3] = cfg["relative_translation_m"]
    # USD Gf uses row vectors; transpose the documented column-vector T_agent_camera.
    camera.AddTransformOp().Set(Gf.Matrix4d(*local.T.reshape(-1).tolist()))
    width, height = int(cfg["resolution_width"]), int(cfg["resolution_height"])
    # USD camera lens properties use tenths of a stage unit, not millimetres.
    millimetres_per_usd_camera_unit = 100.0 * float(config["scene"]["stage_units_in_meters"])
    aperture = float(cfg["horizontal_aperture_mm"]) / millimetres_per_usd_camera_unit
    focal = aperture / (2.0 * math.tan(math.radians(float(cfg["horizontal_fov_deg"])) / 2.0))
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(aperture * height / width)
    camera.CreateFocalLengthAttr(focal)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*cfg["clipping_range_m"]))
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    timeline.set_current_time(0.0)
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()), resolution=(width, height))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(product)
    for _ in range(int(config["capture"]["warmup_updates"])):
        APP.update()
    inventory = scene_inventory(stage, config)
    if not inventory["no_robot_model"]:
        raise RuntimeError(f"Unexpected robot or articulation in scene: {inventory}")
    return stage, agent, camera, annotator, {
        **config["scene"], "resolved_asset_path": str(resolved), "assets_root": root,
        "runtime_inventory": inventory,
    }


def scene_inventory(stage, config: dict) -> dict:
    prims = list(stage.Traverse())
    articulations = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    robot_paths = [str(p.GetPath()) for p in prims if any(term in str(p.GetPath()).lower() for term in ("jackal", "turtlebot", "/robot"))]
    return {
        "prim_count": len(prims), "articulation_roots": articulations,
        "robot_named_paths": robot_paths,
        "rigid_body_count": sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),
        "physics_scene_count": sum(p.IsA(UsdPhysics.Scene) for p in prims),
        "collision_prim_count": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),
        "authored_asset_references": [config["scene"]["asset_relative_path"]],
        "created_runtime_geometry": "logical Xform and child Camera only; visualization uses DebugDraw",
        "no_robot_model": not articulations and not robot_paths,
        "dynamics_advanced": False,
    }


def actual_pose(agent) -> np.ndarray:
    matrix = UsdGeom.Xformable(agent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = matrix.ExtractTranslation()
    forward = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
    return np.array([position[0], position[1], math.atan2(forward[1], forward[0])], dtype=float)


def camera_metadata(camera, agent, config: dict) -> dict:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_camera = np.asarray(cache.GetLocalToWorldTransform(camera.GetPrim()), dtype=float).T
    world_agent = np.asarray(cache.GetLocalToWorldTransform(agent.GetPrim()), dtype=float).T
    agent_camera = np.linalg.inv(world_agent) @ world_camera
    width, height = int(config["camera"]["resolution_width"]), int(config["camera"]["resolution_height"])
    ha = float(camera.GetHorizontalApertureAttr().Get())
    va = float(camera.GetVerticalApertureAttr().Get())
    focal = float(camera.GetFocalLengthAttr().Get())
    checks = {
        "optical_forward_maps_to_agent_positive_x": np.allclose(agent_camera[:3, :3] @ [0, 0, -1], [1, 0, 0]),
        "image_right_maps_to_agent_negative_y": np.allclose(agent_camera[:3, :3] @ [1, 0, 0], [0, -1, 0]),
        "image_up_maps_to_agent_positive_z": np.allclose(agent_camera[:3, :3] @ [0, 1, 0], [0, 0, 1]),
        "proper_rotation_determinant_positive_one": np.isclose(np.linalg.det(agent_camera[:3, :3]), 1.0),
        "translation_matches_configuration": np.allclose(agent_camera[:3, 3], config["camera"]["relative_translation_m"]),
    }
    if not all(checks.values()):
        raise RuntimeError(f"Actual USD camera basis check failed: {checks}")
    return {
        "configured": config["camera"],
        "camera_prim_path": str(camera.GetPath()), "parent_prim_path": str(agent.GetPath()),
        "usd_camera_axes": "+X=image right, +Y=image up, -Z=optical forward",
        "matrix_convention": "column vectors; T_target_source; translation in metres",
        "T_agent_camera": agent_camera.tolist(), "T_world_camera": world_camera.tolist(),
        "T_world_agent": world_agent.tolist(),
        "basis_checks": {key: bool(value) for key, value in checks.items()},
        "actual_intrinsics": {
            "resolution_width_height": [width, height],
            "K_pixel_center_convention": "continuous pixel coordinates, principal point=(width/2,height/2)",
            "K": [[width * focal / ha, 0.0, width / 2], [0.0, height * focal / va, height / 2], [0.0, 0.0, 1.0]],
            "horizontal_fov_deg": math.degrees(2 * math.atan(ha / (2 * focal))),
            "vertical_fov_deg": math.degrees(2 * math.atan(va / (2 * focal))),
            "usd_horizontal_aperture": ha, "usd_vertical_aperture": va,
            "usd_focal_length": focal,
            "horizontal_aperture_mm": ha * 100.0 * float(config["scene"]["stage_units_in_meters"]),
            "vertical_aperture_mm": va * 100.0 * float(config["scene"]["stage_units_in_meters"]),
            "focal_length_mm": focal * 100.0 * float(config["scene"]["stage_units_in_meters"]),
            "usd_aperture_focal_units": "tenths of a stage unit; ratios determine pinhole intrinsics",
            "clipping_range_m": list(camera.GetClippingRangeAttr().Get()),
            "distortion": "none; USD perspective pinhole",
        },
    }


def capture(config: dict, run: Path, config_path: Path) -> None:
    run.mkdir(parents=True, exist_ok=True)
    for reserved in ("metadata.json", "config_snapshot.yaml", "capture_validation.json", "raw", "derived"):
        if (run / reserved).exists():
            raise FileExistsError(f"Capture output already exists: {run / reserved}")
    with (run / "config_snapshot.yaml").open("xb") as stream:
        stream.write(config_path.read_bytes())
    external = (ROOT / config["paths"]["lightnav_checkout"]).resolve()
    lightnav_sha = git_output(external, "rev-parse", "HEAD")
    if lightnav_sha != config["lightnav"]["expected_git_sha"] or git_output(external, "status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError("LightNav checkout does not match the configured unmodified source")
    pose = np.asarray(config["agent"]["pose_world"], dtype=float)
    stage, agent, camera, annotator, scene = runtime_scene(config, pose)
    before = actual_pose(agent)
    camera_info = camera_metadata(camera, agent, config)
    capture_start = make_observation_time(0.0)
    rep.orchestrator.step(rt_subframes=int(config["capture"]["render_subframes"]), delta_time=0.0, pause_timeline=True)
    rgba = np.asarray(annotator.get_data())
    observation_time = make_observation_time(float(omni.timeline.get_timeline_interface().get_current_time()))
    after = actual_pose(agent)
    width, height = int(config["camera"]["resolution_width"]), int(config["camera"]["resolution_height"])
    if rgba.shape != (height, width, 4) or rgba.dtype != np.uint8:
        raise RuntimeError(f"Actual renderer output invalid: {rgba.shape} {rgba.dtype}")
    if not np.array_equal(before, after) or not np.allclose(after, pose, rtol=0, atol=1e-7):
        raise RuntimeError("Static logical agent pose changed or was not set correctly")
    rgb = np.ascontiguousarray(rgba[:, :, :3])
    if float(rgb.std()) < 1.0:
        raise RuntimeError("Captured RGB appears blank")
    (run / "raw").mkdir(exist_ok=False)
    with (run / "raw/observation.jpg").open("xb") as stream:
        Image.fromarray(rgb).save(stream, format="JPEG", quality=int(config["capture"]["jpeg_quality"]))
    source_paths = [Path(__file__).resolve(), ROOT / "src/reconciliation/robotless_single_chunk.py", ROOT / "src/reconciliation/se2.py", config_path]
    metadata = {
        "schema_version": 1, "stage": config["stage"], "observation_time": observation_time,
        "capture_started_time": capture_start,
        "observation_time_semantics": "host UTC and process monotonic readback-completion after one static render; capture_started_time brackets render; simulation timeline held at zero",
        "agent_pose_world": after.tolist(), "agent_prim_path": str(agent.GetPath()),
        "agent_z_m": float(config["agent"]["z_m"]), "agent_pose_before_capture": before.tolist(),
        "agent_pose_after_capture": after.tolist(), "camera": camera_info,
        "instruction": config["instruction"], "scene": scene,
        "coordinate_conventions": FRAME_CONVENTIONS,
        "research_git_sha": git_output(ROOT, "rev-parse", "HEAD"),
        "research_git_status": git_output(ROOT, "status", "--short"),
        "research_source_sha256": {str(p.relative_to(ROOT)): sha256_file(p) for p in source_paths},
        "lightnav_git_sha": lightnav_sha,
        "lightnav_checkout": str(external), "lightnav_tracked_source_clean": True,
        "model_checkpoint_identifier": config["lightnav"]["checkpoint_identifier"],
        "model_checkpoint_revision": config["lightnav"]["checkpoint_revision"],
        "model_checkpoint_path": str((ROOT / config["paths"]["checkpoint_path"]).resolve()),
        "rgb": {"path": "raw/observation.jpg", "sha256": sha256_file(run / "raw/observation.jpg"),
                "resolution_width_height": [width, height], "dtype": "uint8", "channels": "RGB"},
        "config": {"path": "config_snapshot.yaml", "sha256": sha256_file(run / "config_snapshot.yaml")},
        "execution_time": None, "agent_motion": False,
    }
    validate_observation_metadata(metadata)
    save_json_exclusive(run / "metadata.json", metadata)
    save_json_exclusive(run / "capture_validation.json", {
        "scene_loaded": True, "no_robot_model": scene["runtime_inventory"]["no_robot_model"],
        "logical_agent_pose_set": True, "logical_agent_static": True,
        "rgb_captured": True, "rgb_shape_hwc": list(rgb.shape), "rgb_standard_deviation": float(rgb.std()),
        "camera_basis_checks": camera_info["basis_checks"], "timeline_time_s": observation_time["simulation_time_s"],
        "no_simulation_dynamics": True, "source_metadata_sha256": sha256_file(run / "metadata.json"),
    })
    print(f"ROBOTLESS_CAPTURE_SAVED={run}", flush=True)


def viewport_capture(path: Path) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Active Isaac viewport unavailable")
    for _ in range(30):
        APP.update()
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        APP.update()
        if path.is_file() and path.stat().st_size > 0:
            try:
                with Image.open(path) as image:
                    image.verify()
                return
            except (OSError, SyntaxError):
                pass
        time.sleep(0.03)
    raise RuntimeError(f"Isaac viewport screenshot failed: {path}")


def draw_frame(draw, pose: np.ndarray, config: dict, *, draw_left: bool = True) -> None:
    visual = config["visualization"]
    z = float(visual["z_offset_m"])
    points = observation_to_world(pose, np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]))
    draw_pose_points(draw, points[:1], z=z + .05, color=visual["origin_color_rgba"], size=float(visual["point_size"]))
    draw_polyline(draw, points[[0, 1]], z=z, color=visual["forward_color_rgba"], width=float(visual["line_width"]))
    draw_pose_points(draw, points[1:2], z=z, color=visual["forward_color_rgba"], size=float(visual["point_size"]))
    if draw_left:
        draw_polyline(draw, points[[0, 2]], z=z, color=visual["left_color_rgba"], width=float(visual["line_width"]))
        draw_pose_points(draw, points[2:3], z=z, color=visual["left_color_rgba"], size=float(visual["point_size"]))


def visualize(config: dict, run: Path) -> None:
    inference = json.loads((run / "inference_metadata.json").read_text())
    for relative, recorded in (
        ("metadata.json", inference["raw_inputs"]["capture_metadata"]),
        ("raw/lightnav_waypoints.npy", inference["raw_inputs"]["waypoints"]),
        ("derived/world_waypoints.npy", inference["world_waypoints"]),
    ):
        if recorded["path"] != relative or sha256_file(run / relative) != recorded["sha256"]:
            raise ValueError(f"Saved inference input hash mismatch: {relative}")
    metadata = json.loads((run / "metadata.json").read_text())
    validate_observation_metadata(metadata)
    if sha256_file(run / "config_snapshot.yaml") != metadata["config"]["sha256"]:
        raise ValueError("Frozen capture configuration hash mismatch")
    if sha256_file(run / metadata["rgb"]["path"]) != metadata["rgb"]["sha256"]:
        raise ValueError("Raw observation image hash mismatch")
    if not ARGS.view_only and (run / "visualization_validation.json").exists():
        raise FileExistsError("Visualization validation already exists")
    local = validate_waypoints(np.load(run / "raw/lightnav_waypoints.npy", allow_pickle=False))
    world = validate_waypoints(np.load(run / "derived/world_waypoints.npy", allow_pickle=False))
    pose = np.asarray(metadata["agent_pose_world"], dtype=float)
    recomputed = observation_to_world(pose, local)
    if world.shape != recomputed.shape or not np.allclose(world, recomputed, rtol=0, atol=1e-12):
        raise ValueError("Derived trajectory does not match observation-pose transform")
    stage, agent, camera, _, scene = runtime_scene(config, pose)
    before = actual_pose(agent)
    actual_camera = camera_metadata(camera, agent, config)
    if not np.allclose(actual_camera["T_world_camera"], metadata["camera"]["T_world_camera"], rtol=0, atol=1e-7):
        raise ValueError("Visualization camera no longer matches observation camera")
    draw = _debug_draw.acquire_debug_draw_interface()
    visual = config["visualization"]
    screenshots = []
    fixtures = frame_sanity_fixtures()
    fixture_records = []
    # Synthetic frame glyphs are independent test geometry. The real agent stays fixed.
    for fixture in ([] if ARGS.view_only else fixtures):
        fixture_pose = np.asarray(fixture["agent_pose_world"], dtype=float)
        display_pose = fixture_pose.copy()
        display_pose[:2] += pose[:2]
        fixture_world = observation_to_world(display_pose, np.asarray(fixture["local_waypoints"]))
        expected = np.asarray(fixture["expected_world_waypoints"], dtype=float).copy()
        expected[:, :2] += pose[:2]
        passed = bool(fixture["passed"] and np.allclose(fixture_world, expected, rtol=0, atol=1e-12))
        if not passed:
            raise RuntimeError("Synthetic frame visualization fixture failed")
        draw.clear_lines(); draw.clear_points()
        draw_frame(draw, display_pose, config)
        set_camera_view(
            eye=(np.r_[pose[:2], 0.0] + visual["fixture_eye_world_offset_m"]).tolist(),
            target=(np.r_[pose[:2], 0.0] + visual["fixture_target_world_offset_m"]).tolist(),
            camera_prim_path="/OmniverseKit_Persp",
        )
        degrees = int(round(math.degrees(float(fixture_pose[2]))))
        path = run / "evidence" / f"synthetic_frame_yaw_{degrees:03d}.png"
        viewport_capture(path)
        screenshots.append(str(path.relative_to(run)))
        fixture_records.append({
            **fixture, "display_translation_world_m": pose[:2].tolist(),
            "display_agent_pose_world": display_pose.tolist(),
            "display_world_waypoints": fixture_world.tolist(), "display_expected_world_waypoints": expected.tolist(),
            "display_transform": "add documented world XY offset to test fixture and expected positions; yaw unchanged",
            "screenshot": str(path.relative_to(run)), "passed": passed, "synthetic_only": True,
        })
    draw.clear_lines(); draw.clear_points()
    draw_frame(draw, pose, config)
    z = float(visual["z_offset_m"])
    # Include the observation origin as the visual start, without altering saved waypoints.
    draw_polyline(draw, np.vstack([pose, world]), z=z + .04, color=visual["trajectory_color_rgba"], width=float(visual["line_width"]))
    draw_pose_points(draw, world, z=z + .04, color=visual["trajectory_color_rgba"], size=float(visual["point_size"]) * .65)
    draw_heading_markers(draw, world, z=z + .07, color=visual["heading_color_rgba"], width=3.0, length_m=float(visual["heading_length_m"]))
    offsets = np.asarray([visual["overview_eye_agent_offset_m"], visual["overview_target_agent_offset_m"]], dtype=float)
    xy = observation_to_world(pose, np.c_[offsets[:, :2], np.zeros(2)])
    set_camera_view(eye=[*xy[0, :2], offsets[0, 2]], target=[*xy[1, :2], offsets[1, 2]], camera_prim_path="/OmniverseKit_Persp")
    path = run / "evidence/lightnav_world_trajectory.png"
    if not ARGS.view_only:
        viewport_capture(path)
        screenshots.append(str(path.relative_to(run)))
    after = actual_pose(agent)
    if not np.array_equal(before, after) or not np.allclose(before, pose, rtol=0, atol=1e-7):
        raise RuntimeError("Logical agent moved during passive visualization")
    if ARGS.view_only:
        print("ROBOTLESS_VIEW_ONLY=existing capture and raw trajectory unchanged", flush=True)
        if not ARGS.no_hold and not ARGS.headless:
            while APP.is_running():
                APP.update()
        return
    save_json_exclusive(run / "visualization_validation.json", {
        "created_time": make_observation_time(float(omni.timeline.get_timeline_interface().get_current_time())),
        "isaac_viewport_rendered": True, "no_robot_model": scene["runtime_inventory"]["no_robot_model"],
        "agent_pose_world_used": pose.tolist(), "agent_pose_before_visualization": before.tolist(),
        "agent_pose_after_visualization": after.tolist(), "logical_agent_static": True,
        "world_shape": list(world.shape), "matches_observation_pose_transform": True,
        "camera_basis_checks": actual_camera["basis_checks"], "synthetic_fixtures": fixture_records,
        "fixtures_are_research_evidence": False, "passive_visualization": "Isaac DebugDraw only; no USD physical bodies; timeline remains stopped",
        "scene_visibility_modifications": [], "waypoint_heading_count": int(len(world)),
        "legend": {"magenta": "logical origin", "green": "local +X forward (1m)", "red": "local +Y left (1m)", "cyan": "LightNav world trajectory and waypoints", "yellow": "world waypoint headings"},
        "screenshots": [{"path": p, "sha256": sha256_file(run / p)} for p in screenshots],
        "raw_inputs": {"metadata.json": sha256_file(run / "metadata.json"), "raw/lightnav_waypoints.npy": sha256_file(run / "raw/lightnav_waypoints.npy")},
        "derived_input": {"path": "derived/world_waypoints.npy", "sha256": sha256_file(run / "derived/world_waypoints.npy")},
        "config_sha256": metadata["config"]["sha256"],
        "research_source_sha256": {str(Path(__file__).resolve().relative_to(ROOT)): sha256_file(Path(__file__))},
    })
    print(f"ROBOTLESS_VISUALIZATION_SAVED={run}", flush=True)
    if not ARGS.no_hold and not ARGS.headless:
        print("ROBOTLESS_VIEWPORT_HOLD=close Isaac Sim to exit", flush=True)
        while APP.is_running():
            APP.update()


def main() -> None:
    run = ARGS.run_directory.resolve()
    config_path = ARGS.config.resolve() if ARGS.mode == "capture" else run / "config_snapshot.yaml"
    config = load_config(config_path)
    if ARGS.view_only and ARGS.mode != "visualize":
        raise ValueError("--view-only requires visualize mode")
    if ARGS.mode == "capture":
        capture(config, run, config_path)
    else:
        visualize(config, run)


try:
    main()
finally:
    APP.close()
