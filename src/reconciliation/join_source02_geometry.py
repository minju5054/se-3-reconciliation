"""Source-only runtime mesh geometry using the unchanged Hospital oracle rules.

The complete authored prop is clipped to the original height band. Polygon
surfaces and closed solid sections are retained; nonclosing section interiors
remain unknown workspace. No box, convex hull, or inflation replaces the mesh.
This is an authored polygon oracle, not real-robot or continuous-curve safety.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib

import numpy as np
import shapely
from shapely.geometry import GeometryCollection, box
from shapely.ops import unary_union

from .gp_se2_environment import (
    HospitalEnvironment, clip_height, filled_cross_section, project_surface,
)

HEIGHT_BAND_M = (.05, .65)


def project_prop(triangles, *, height_band=HEIGHT_BAND_M):
    """Project finite world-metre ``(N,3,3)`` triangles without changing them.

    Geometry objects serve live checks; ``metadata`` contains complete WKB and
    hash provenance for saved-record reconstruction. The caller supplies actual
    USD triangles and separately authenticates the world transform/source asset.
    """
    triangles = np.asarray(triangles, dtype=np.float64)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not len(triangles):
        raise ValueError("Nonempty actual world triangles must have shape (N,3,3)")
    if not np.all(np.isfinite(triangles)):
        raise ValueError("Nonfinite mesh geometry is unsupported, never free space")
    band = np.asarray(height_band, dtype=float)
    if band.shape != (2,) or not np.array_equal(band, HEIGHT_BAND_M):
        raise ValueError("JOIN-SOURCE-02 preserves the original [.05,.65]m height band")
    surfaces = []
    counts = dict(below_band=0, above_band=0, intersects_band=0)
    for triangle in triangles:
        if triangle[:, 2].max() < band[0]:
            counts["below_band"] += 1
            continue
        if triangle[:, 2].min() > band[1]:
            counts["above_band"] += 1
            continue
        projected = project_surface(clip_height(triangle, *band))
        if projected is not None and not projected.is_empty:
            surfaces.append(projected)
        counts["intersects_band"] += 1
    surface = unary_union(surfaces) if surfaces else GeometryCollection()
    occupied, unknown, section = filled_cross_section(triangles, float(band[0]) + 1e-9)
    obstacles = unary_union([surface, occupied])
    bounds = np.stack([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))])
    # Bounds are descriptive only. They never enter the collision oracle.
    bounding_box = box(*bounds[:, :2].reshape(-1))
    excess = bounding_box.difference(obstacles)
    metadata = dict(
        schema="JOIN_SOURCE_02_ACTUAL_MESH_PROJECTION_V1",
        frame="world XY metres from world Z-up triangles; no re-anchoring",
        height_band_world_m=band.tolist(), ground_z_m=0.,
        triangle_count=len(triangles), triangle_height_counts=counts,
        triangles_float64_c_sha256=hashlib.sha256(np.ascontiguousarray(triangles).tobytes()).hexdigest(),
        triangle_shape=list(triangles.shape), triangle_dtype="float64",
        world_bounds_xyz_m=bounds.tolist(),
        obstacle_wkb_hex=obstacles.wkb_hex, unknown_wkb_hex=unknown.wkb_hex,
        surface_projection_area_m2=float(surface.area),
        closed_section_area_m2=float(occupied.area),
        obstacle_area_m2=float(obstacles.area), unknown_area_m2=float(unknown.area),
        lower_plane_cross_section=section,
        bbox_area_m2=float(bounding_box.area),
        bbox_extra_area_m2=float(excess.area),
        bbox_is_oracle=False,
        mesh_outside_bbox_area_m2=float(obstacles.difference(bounding_box).area),
        footprint_radius_applied_in_projection=False,
        clearance_applied_in_projection=False,
        method="unchanged clip_height/project_surface plus filled_cross_section at .05+1e-9; open section interiors UNKNOWN",
        uncertainty="authored polygon oracle; no subdivision or real-world perception uncertainty model",
        shapely_version=shapely.__version__, geos_version=shapely.geos_version_string,
    )
    return dict(obstacle_geometry=obstacles, unknown_geometry=unknown, metadata=metadata)


def projection_from_metadata(metadata):
    """Load saved geometry only; no render, model, controller, or optimizer call."""
    if metadata.get("schema") != "JOIN_SOURCE_02_ACTUAL_MESH_PROJECTION_V1":
        raise ValueError("Unsupported runtime prop projection schema")
    if metadata.get("height_band_world_m") != list(HEIGHT_BAND_M) or metadata.get("bbox_is_oracle") is not False:
        raise ValueError("Saved runtime projection violates frozen geometry semantics")
    return dict(obstacle_geometry=shapely.from_wkb(bytes.fromhex(metadata["obstacle_wkb_hex"])),
                unknown_geometry=shapely.from_wkb(bytes.fromhex(metadata["unknown_wkb_hex"])),
                metadata=deepcopy(metadata))


def compose_environment(base, projection, present):
    """Return direct-query environment with occupancy matching runtime presence.

    Original Hospital and its map are never mutated. No approximate grid is
    rebuilt or retained as an apparently combined map; this source experiment
    uses the inherited independent point/swept-polyline checker directly.
    """
    if not isinstance(present, (bool, np.bool_)):
        raise TypeError("Runtime obstacle presence must be explicit boolean")
    if projection["metadata"]["height_band_world_m"] != list(HEIGHT_BAND_M):
        raise ValueError("Runtime height band mismatch")
    base_band = base.metadata.get("height_band_world_m", list(HEIGHT_BAND_M))
    if base_band != list(HEIGHT_BAND_M):
        raise ValueError("Original Hospital height band differs from frozen source protocol")
    parts = list(base.parts)
    if present:
        runtime = projection["obstacle_geometry"]
        if not runtime.is_empty:
            parts.append(runtime)
        obstacles = unary_union([base.obstacles, runtime])
        workspace = base.workspace.difference(projection["unknown_geometry"])
    else:
        obstacles, workspace = base.obstacles, base.workspace
    metadata = deepcopy(base.metadata)
    metadata.update(runtime_obstacle_present=bool(present),
        runtime_projection=deepcopy(projection["metadata"]),
        original_hospital_obstacles_wkb_sha256=hashlib.sha256(base.obstacles.wkb).hexdigest(),
        original_hospital_workspace_wkb_sha256=hashlib.sha256(base.workspace.wkb).hexdigest(),
        query_policy="DIRECT_GEOMETRY_ONLY; runtime mesh occupancy iff present; no distance-grid use")
    result = HospitalEnvironment(obstacles, workspace, parts=parts, grid=None, metadata=metadata)
    result.gates = deepcopy(base.gates)
    return result
