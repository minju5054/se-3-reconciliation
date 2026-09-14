"""Shared robotless scene, camera, pose assignment, and viewport readback.

Import only after creating Isaac SimulationApp. No robot model, physics world,
controller, simulation stepping, or model inference is constructed here.
"""

from __future__ import annotations

import math
from pathlib import Path
import subprocess
import time

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from reconciliation.robotless_single_chunk import validate_scene_frame


def set_agent_pose(agent, pose: np.ndarray, *, z_m: float) -> None:
    """Assign an already-composed world pose directly; do not advance dynamics."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)) or not math.isfinite(z_m):
        raise ValueError("logical agent pose and z must be finite")
    xform = UsdGeom.XformCommonAPI(agent)
    xform.SetTranslate(Gf.Vec3d(float(pose[0]), float(pose[1]), float(z_m)))
    xform.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(float(pose[2]))))


def git_output(directory: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(directory), *args], text=True).strip()


def runtime_scene(config: dict, pose: np.ndarray, *, app) -> tuple:
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
        app.update()
    agent = UsdGeom.Xform.Define(stage, str(config["agent"]["prim_path"]))
    set_agent_pose(agent, pose, z_m=float(config["agent"]["z_m"]))
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
        app.update()
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



def viewport_capture(path: Path, *, app) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Active Isaac viewport unavailable")
    for _ in range(30):
        app.update()
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline:
        app.update()
        if path.is_file() and path.stat().st_size > 0:
            try:
                with Image.open(path) as image:
                    image.verify()
                return
            except (OSError, SyntaxError):
                pass
        time.sleep(0.03)
    raise RuntimeError(f"Isaac viewport screenshot failed: {path}")
