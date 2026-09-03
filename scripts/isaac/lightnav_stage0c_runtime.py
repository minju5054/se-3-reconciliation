"""Shared Isaac-only Jackal helpers for Stage 0-C capture and playback."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import omni.usd
import yaml
from isaacsim.core.prims import SingleArticulation
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path
from pxr import Usd, UsdGeom, UsdPhysics

from reconciliation.controllers.differential import map_wheel_speeds_by_side
from reconciliation.se2 import wrap_angle
from reconciliation.controller_validation import WHEEL_LABELS


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("Stage 0-C config must contain a mapping")
    return value


def require_mapping(config: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = config.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"config field {key!r} must contain a mapping")
    return value


def finite_positive(config: Mapping[str, Any], key: str) -> float:
    value = float(config[key])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"config field {key!r} must be finite and positive")
    return value


def quaternion_from_yaw(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def se2_from_world_pose(robot: SingleArticulation) -> np.ndarray:
    position, orientation = robot.get_world_pose()
    w, x, y, z = (float(value) for value in orientation)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([position[0], position[1], wrap_angle(yaw)], dtype=np.float64)


def resolve_jackal_asset(robot_config: Mapping[str, Any]) -> dict[str, str | None]:
    collection = str(robot_config["asset_collection_path"]).rstrip("/")
    relative = str(robot_config["asset_relative_path"]).lstrip("/")
    query = f"{collection}/{relative}"
    resolved = resolve_asset_path(query)
    if not resolved:
        raise RuntimeError(f"official Jackal asset could not be resolved: {query}")
    return {
        "relative_path": relative,
        "asset_collection_path": collection,
        "assets_root_runtime": get_assets_root_path(),
        "resolved_path": resolved,
    }


def find_articulation_root(reference_prim_path: str) -> str:
    stage = omni.usd.get_context().get_stage()
    reference = stage.GetPrimAtPath(reference_prim_path)
    roots = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(reference)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one Jackal articulation root, found: {roots}")
    return roots[0]


def camera_prims_below(reference_prim_path: str) -> list[str]:
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(reference_prim_path)
    return [str(prim.GetPath()) for prim in Usd.PrimRange(root) if prim.IsA(UsdGeom.Camera)]


def _collision_cylinder_radius(stage, wheel_body_prim) -> float:
    radii = []
    for prim in Usd.PrimRange(wheel_body_prim):
        if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
            value = UsdGeom.Cylinder(prim).GetRadiusAttr().Get()
            if value is not None:
                radii.append(float(value))
    if len(radii) != 1 or radii[0] <= 0.0:
        raise RuntimeError(f"cannot identify wheel collision radius below {wheel_body_prim.GetPath()}")
    return radii[0]


def discover_wheels(robot: SingleArticulation, articulation_root: str) -> dict[str, Any]:
    dof_names = list(robot.dof_names or [])
    if robot.num_dof != 4 or len(set(dof_names)) != 4:
        raise RuntimeError(f"expected four unique Jackal wheel DOFs, got {dof_names}")
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(articulation_root)
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(root_prim)
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in dof_names
    }
    records = []
    for index, name in enumerate(dof_names):
        joint = UsdPhysics.RevoluteJoint(joints[name])
        local_position = joint.GetLocalPos0Attr().Get()
        targets = joint.GetBody1Rel().GetTargets()
        if local_position is None or len(targets) != 1:
            raise RuntimeError(f"wheel geometry is unavailable for {name}")
        body = stage.GetPrimAtPath(targets[0])
        records.append(
            {
                "dof_index": index,
                "dof_name": name,
                "side": "left" if float(local_position[1]) > 0.0 else "right",
                "axle": "front" if float(local_position[0]) > 0.0 else "rear",
                "radius_m": _collision_cylinder_radius(stage, body),
                "lateral_position_m": float(local_position[1]),
            }
        )
    corners = {f"{item['axle']}_{item['side']}" for item in records}
    if corners != set(WHEEL_LABELS):
        raise RuntimeError(f"runtime wheel geometry did not form four corners: {records}")
    radii = np.asarray([item["radius_m"] for item in records])
    if not np.allclose(radii, radii[0], atol=1e-6, rtol=0.0):
        raise RuntimeError(f"wheel radii differ: {radii.tolist()}")
    left = [item["lateral_position_m"] for item in records if item["side"] == "left"]
    right = [item["lateral_position_m"] for item in records if item["side"] == "right"]
    separation = float(np.mean(left) - np.mean(right))
    return {"radius_m": float(radii[0]), "separation_m": separation, "wheels": records}


def canonical_wheel_values(runtime_values: np.ndarray, wheels: Mapping[str, Any]) -> np.ndarray:
    values = np.asarray(runtime_values, dtype=np.float64)
    by_corner = {
        f"{wheel['axle']}_{wheel['side']}": float(values[int(wheel["dof_index"])])
        for wheel in wheels["wheels"]
    }
    return np.asarray([by_corner[label] for label in WHEEL_LABELS], dtype=np.float64)


def canonical_wheel_names(wheels: Mapping[str, Any]) -> tuple[str, ...]:
    by_corner = {
        f"{wheel['axle']}_{wheel['side']}": str(wheel["dof_name"])
        for wheel in wheels["wheels"]
    }
    return tuple(by_corner[label] for label in WHEEL_LABELS)


def runtime_wheel_command(side_speeds: np.ndarray, wheels: Mapping[str, Any]) -> np.ndarray:
    return map_wheel_speeds_by_side(
        side_speeds,
        [str(wheel["side"]) for wheel in wheels["wheels"]],
    )
