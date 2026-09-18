"""Static Hospital oracle projection, approximate queries, and exact XY checking.

Geometry is derived from fully transformed USD mesh faces, clipped to the chosen
ground-relative volume. This module makes no assertion about a physical robot.
The optimizer's bilinear distance grid is distinct from direct GEOS geometry
queries and swept circular-footprint checking used for final validation.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def transform_points(points, column_matrix, metres_per_unit=1.):
    points = np.asarray(points, dtype=float)
    matrix = np.asarray(column_matrix, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or matrix.shape != (4, 4):
        raise ValueError("expected N by 3 points and 4 by 4 column-vector transform")
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(matrix)) or metres_per_unit <= 0:
        raise ValueError("finite transform and positive stage unit scale required")
    result = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    return result[:, :3] / result[:, 3, None] * metres_per_unit


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def triangulate_face(vertices):
    """Ear clipping respects nonconvex authored polygons instead of filling AABBs."""
    vertices = np.asarray(vertices, dtype=float)
    if len(vertices) < 3:
        return []
    if len(vertices) == 3:
        return [vertices]
    normal = np.sum(np.cross(vertices, np.roll(vertices, -1, axis=0)), axis=0)
    if np.linalg.norm(normal) < 1e-13:
        # Degenerate faces can still represent a thin projected obstacle surface.
        return [vertices[[0, i, i + 1]] for i in range(1, len(vertices) - 1)]
    xy = np.delete(vertices, np.argmax(np.abs(normal)), axis=1)
    signed_area = sum(_cross2(xy[i], xy[(i + 1) % len(xy)]) for i in range(len(xy)))
    orientation = 1. if signed_area >= 0 else -1.
    remaining, output = list(range(len(vertices))), []
    while len(remaining) > 3:
        found = False
        for j in range(len(remaining)):
            indices = [remaining[(j - 1) % len(remaining)], remaining[j], remaining[(j + 1) % len(remaining)]]
            a, b, c = xy[indices]
            if orientation * _cross2(b - a, c - b) <= 1e-13:
                continue
            other = [k for k in remaining if k not in indices]
            if any(min(orientation * _cross2(b - a, xy[k] - a),
                       orientation * _cross2(c - b, xy[k] - b),
                       orientation * _cross2(a - c, xy[k] - c)) >= -1e-12 for k in other):
                continue
            output.append(vertices[indices])
            remaining.pop(j)
            found = True
            break
        if not found:
            # Collinear boundary vertices can be removed without changing the face.
            for j in range(len(remaining)):
                a, b, c = [xy[remaining[k % len(remaining)]] for k in (j - 1, j, j + 1)]
                if abs(_cross2(b - a, c - b)) <= 1e-12:
                    remaining.pop(j)
                    found = True
                    break
            if not found:
                raise ValueError("Self-intersecting or unsupported nonplanar mesh face")
    if len(remaining) == 3:
        output.append(vertices[remaining])
    return output


def clip_height(vertices, z_min, z_max):
    """Clip a convex 3D polygon to a closed horizontal slab, retaining vertical faces."""
    result = np.asarray(vertices, dtype=float)
    for boundary, keep_above in ((float(z_min), True), (float(z_max), False)):
        if not len(result):
            break
        output = []
        for a, b in zip(result, np.roll(result, -1, axis=0)):
            inside_a = a[2] >= boundary if keep_above else a[2] <= boundary
            inside_b = b[2] >= boundary if keep_above else b[2] <= boundary
            if inside_a:
                output.append(a)
            if inside_a != inside_b:
                output.append(a + (b - a) * ((boundary - a[2]) / (b[2] - a[2])))
        result = np.asarray(output, dtype=float).reshape((-1, 3))
    return result


def project_surface(vertices):
    """Return a polygon, line, or point; never discard zero-area vertical walls."""
    from shapely.geometry import MultiPoint
    vertices = np.asarray(vertices, dtype=float)
    if not len(vertices):
        return None
    # Input is a clipped triangle and therefore convex. Convex hull preserves
    # its exact projection, including degenerate line/point projections.
    return MultiPoint(vertices[:, :2]).convex_hull


def filled_cross_section(triangles, height, *, snap_tolerance=1e-8):
    """Even-odd solid occupancy at a horizontal plane, plus unresolved regions.

    Filling each closed loop independently would fill doorway/cavity holes.
    Instead parity of original section segments classifies polygonized cells.
    Nonclosing section edges with positive-area hulls are explicitly unknown.
    """
    import shapely
    from shapely.geometry import GeometryCollection, LineString
    from shapely.ops import polygonize_full, unary_union
    segments = []
    for triangle in np.asarray(triangles):
        if triangle[:, 2].min() > height or triangle[:, 2].max() < height:
            continue
        points = []
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            if (a[2] <= height < b[2]) or (b[2] <= height < a[2]):
                points.append((a + (b - a) * ((height - a[2]) / (b[2] - a[2])))[:2])
        if len(points) == 2 and np.linalg.norm(points[1] - points[0]) > snap_tolerance:
            segments.append(points)
    if not segments:
        return GeometryCollection(), GeometryCollection(), {"segment_count": 0, "filled_area_m2": 0., "unknown_area_m2": 0.}
    segments = np.asarray(segments)
    network = shapely.set_precision(unary_union([LineString(x) for x in segments]), snap_tolerance)
    polygons, cuts, dangles, invalid = polygonize_full(network)
    filled = []
    a, b = segments[:, 0], segments[:, 1]
    for polygon in polygons.geoms:
        point = polygon.representative_point()
        crosses = (a[:, 1] > point.y) != (b[:, 1] > point.y)
        active_a, active_b = a[crosses], b[crosses]
        x_cross = active_a[:, 0] + (point.y - active_a[:, 1]) * (active_b[:, 0] - active_a[:, 0]) / (active_b[:, 1] - active_a[:, 1])
        if np.count_nonzero(x_cross > point.x) % 2:
            filled.append(polygon)
    occupied = unary_union(filled)
    unresolved = unary_union([cuts, dangles, invalid])
    # A genuinely open sheet has no solid volume; a bent/open shell may enclose
    # unknown volume. Do not promote such a shell's hull to a filled obstacle.
    lines = list(unresolved.geoms) if hasattr(unresolved, "geoms") else ([] if unresolved.is_empty else [unresolved])
    parents = list(range(len(lines)))
    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index
    endpoints = {}
    for index, line in enumerate(lines):
        if not hasattr(line, "coords"):
            continue
        for endpoint in (line.coords[0], line.coords[-1]):
            key = tuple(np.round(np.asarray(endpoint) / snap_tolerance).astype(np.int64))
            if key in endpoints:
                parents[root(index)] = root(endpoints[key])
            endpoints[key] = index
    components = {}
    for index, line in enumerate(lines):
        components.setdefault(root(index), []).append(line)
    hulls = [unary_union(lines).convex_hull for lines in components.values()]
    unknown = unary_union([hull for hull in hulls if hull.area > snap_tolerance**2])
    return occupied, unknown, {"segment_count": len(segments), "filled_area_m2": occupied.area,
                               "unknown_area_m2": unknown.area, "snap_tolerance_m": snap_tolerance,
                               "closed_cell_count": len(filled), "unresolved_edge_length_m": unresolved.length}


def _json_save(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def build_environment(export_directory, *, grid_resolution=.05, ground_tolerance=.03):
    """Create an exclusive derived map beside the unmodified exported triangles."""
    import shapely
    from shapely import STRtree
    from shapely.geometry import GeometryCollection
    from shapely.ops import unary_union

    root = Path(export_directory)
    target = root / "geometry/projected"
    target.mkdir(exist_ok=False)
    provenance = json.loads((root / "scene_provenance.json").read_text())
    if any(provenance.get(k) for k in ("composition_errors", "missing_geometry", "unsupported_geometry")):
        raise ValueError("Unresolved or unsupported scene geometry prohibits validated environment")
    ground = float(provenance["ground_z_m"])
    band = np.asarray(provenance["height_band_ground_relative_m"]) + ground
    raw = np.load(root / "geometry/world_triangles.npz", allow_pickle=False)
    triangles, mesh_ids = raw["triangles"], raw["mesh_ids"]
    obstacles_by_mesh, floors, height_counts = {}, [], {"below_band": 0, "above_band": 0, "intersects_band": 0}
    unknown_parts, sections = [], []
    floor_mesh_ids = set()
    for triangle, mesh_id in zip(triangles, mesh_ids):
        z = triangle[:, 2]
        if np.max(np.abs(z - ground)) <= ground_tolerance:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            if np.linalg.norm(normal) > 1e-14 and abs(normal[2]) / np.linalg.norm(normal) >= .999:
                floor = project_surface(triangle)
                if floor.area > 1e-12:
                    floors.append(floor)
                    floor_mesh_ids.add(int(mesh_id))
        if z.max() < band[0]:
            height_counts["below_band"] += 1
            continue
        if z.min() > band[1]:
            height_counts["above_band"] += 1
            continue
        clipped = clip_height(triangle, *band)
        obstacle = project_surface(clipped)
        if obstacle is not None and not obstacle.is_empty:
            obstacles_by_mesh.setdefault(int(mesh_id), []).append(obstacle)
            height_counts["intersects_band"] += 1
    meshes = json.loads((root / "geometry/meshes.json").read_text())
    # A referenced asset can divide a watertight solid across Section0/Section1
    # material meshes. Rejoin only sibling meshes under the actual asset root.
    asset_groups = {}
    for mesh_id in obstacles_by_mesh:
        prim = meshes[mesh_id]["prim_path"] if meshes else f"/synthetic/{mesh_id}"
        asset = "/".join(prim.split("/")[:4])
        asset_groups.setdefault(asset, []).append(mesh_id)
    for asset, group in asset_groups.items():
        occupied, unknown, section = filled_cross_section(triangles[np.isin(mesh_ids, group)], band[0] + 1e-9)
        obstacles_by_mesh[group[0]].append(occupied)
        if not unknown.is_empty:
            unknown_parts.append(unknown)
        sections.append({"asset_root": asset, "mesh_ids": group, **section})
    obstacle_parts, obstacle_ids = [], []
    for mesh_id, parts in obstacles_by_mesh.items():
        obstacle_parts.append(unary_union(parts))
        obstacle_ids.append(mesh_id)
    obstacles = unary_union(obstacle_parts) if obstacle_parts else GeometryCollection()
    unknown = unary_union(unknown_parts)
    workspace = unary_union(floors).difference(unknown)
    if workspace.is_empty:
        raise ValueError("No actual horizontal ground geometry supports the evaluated workspace")
    for name, geometry in (("obstacles", obstacles), ("workspace", workspace), ("unknown_interiors", unknown)):
        with (target / f"{name}.wkb").open("xb") as stream:
            stream.write(geometry.wkb)
    # Per-mesh parts retain source attribution and accelerate independent queries.
    with (target / "obstacle_parts.json").open("x") as stream:
        json.dump([{"mesh_id": mesh_id, "wkb_hex": item.wkb_hex}
                   for mesh_id, item in zip(obstacle_ids, obstacle_parts)], stream)
    bounds = np.asarray(workspace.bounds)
    origin = np.floor(bounds[:2] / grid_resolution) * grid_resolution
    size = np.ceil((bounds[2:] - origin) / grid_resolution).astype(int) + 1
    xs = origin[0] + grid_resolution * np.arange(size[0])
    ys = origin[1] + grid_resolution * np.arange(size[1])
    points = shapely.points(*np.meshgrid(xs, ys))
    tree = STRtree(obstacle_parts)
    flat = points.reshape(-1)
    distances = np.empty(len(flat))
    for offset in range(0, len(flat), 10000):
        batch = flat[offset:offset + 10000]
        nearest = tree.nearest(batch)
        distances[offset:offset + len(batch)] = shapely.distance(batch, np.asarray(obstacle_parts, dtype=object)[nearest])
    with (target / "distance_grid.npz").open("xb") as stream:
        np.savez_compressed(stream, origin=origin, resolution=grid_resolution,
                            distances=distances.reshape((len(ys), len(xs))))
    record = {"frame": "world XY, metres; Z-up", "obstacle_projection": "height-clipped actual mesh triangle projection; line walls retained",
              "ground_z_m": ground, "height_band_world_m": band.tolist(),
              "ground_tolerance_m": ground_tolerance,
              "ground_semantics": "union of actual near-horizontal mesh faces within ground tolerance; footprint must lie wholly within union",
              "floor_mesh_ids": sorted(floor_mesh_ids), "obstacle_mesh_ids": obstacle_ids,
              "obstacle_part_count": len(obstacle_parts), "workspace_bounds_m": bounds.tolist(),
              "workspace_area_m2": workspace.area, "projected_obstacle_area_m2": obstacles.area,
              "triangle_height_counts": height_counts,
              "solid_cross_sections": sections,
              "solid_occupancy_rule": "surface slab projection union lower-plane section odd-parity interiors; section holes preserved",
              "unknown_interior_area_m2": unknown.area,
              "optimizer_query": "bilinear interpolation of exact center-to-obstacle distances on grid",
              "grid_resolution_m": grid_resolution,
              "grid_distance_absolute_error_bound_m": grid_resolution / np.sqrt(2),
              "grid_bound_reason": "1-Lipschitz Euclidean distance; weighted distance to four interpolation corners <= cell diagonal / 2",
              "independent_checker": "direct GEOS Euclidean distance to projected mesh polygons/lines; swept XY LineString distance",
              "numerical_tolerance_m": 1e-7,
              "geometry_representation_limitations": ["authored static polygon surface oracle, not real robot safety", "nonclosing positive-area section shells are UNKNOWN workspace", "USD polygon tessellation is authoritative; no subdivision-surface smoothing"],
              "shapely_version": shapely.__version__, "geos_version": shapely.geos_version_string,
              "source_sha256": {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (root / "geometry/world_triangles.npz", root / "geometry/meshes.json", root / "scene_provenance.json")}}
    _json_save(target / "projection.json", record)
    return HospitalEnvironment.load(root)


def derive_doorway_gates(export_directory, *, radius=.20, required_clearance=.05):
    """Catalog real authored door-frame openings; raw route crossing is separate.

    AABB is used only to propose an oriented frame-plane line, never as an
    obstacle. The allowed center interval is verified against exact geometry.
    Gates are not automatically required for any event.
    """
    from shapely.geometry import LineString
    root = Path(export_directory)
    env = HospitalEnvironment.load(root)
    meshes = json.loads((root / "geometry/meshes.json").read_text())
    gates = []
    for mesh_id, mesh in enumerate(meshes):
        if "doorframe" not in mesh["prim_path"].lower():
            continue
        bounds = np.asarray(mesh["world_bounds_m"])
        spans = bounds[1] - bounds[0]
        normal_axis = int(np.argmin(spans[:2]))
        tangent_axis = 1 - normal_axis
        if spans[normal_axis] > .3 or spans[tangent_axis] < .7 or spans[2] < 1.8:
            continue
        center = bounds[:, :2].mean(axis=0)
        tangent, normal = np.eye(2)[tangent_axis], np.eye(2)[normal_axis]
        samples = np.linspace(-spans[tangent_axis] / 2, spans[tangent_axis] / 2,
                              int(np.ceil(spans[tangent_axis] / .001)) + 1)
        xy = center + samples[:, None] * tangent
        good = (env.clearance(xy, radius) >= required_clearance + 1e-5) & env.workspace_status(xy, radius)
        transitions = np.diff(np.r_[False, good, False].astype(int))
        for start, stop in zip(np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)):
            endpoints = xy[[start, stop - 1]]
            width = float(np.linalg.norm(endpoints[1] - endpoints[0]))
            if width < .10 or not env.check_polyline(endpoints, radius=radius, required_clearance=required_clearance)["clearance_valid"]:
                continue
            gate_center = endpoints.mean(axis=0)
            cross = np.array([gate_center - .30 * normal, gate_center + .30 * normal])
            crossing = env.check_polyline(cross, radius=radius, required_clearance=required_clearance)
            if not crossing["clearance_valid"]:
                continue
            gates.append({"gate_id": f"doorframe_{mesh_id:04d}_{start:04d}",
                          "source_mesh_id": mesh_id, "source_prim_path": mesh["prim_path"],
                          "center_world_xy_m": gate_center.tolist(), "normal_world_xy": normal.tolist(),
                          "tangent_world_xy": tangent.tolist(), "center_interval_endpoints_world_m": endpoints.tolist(),
                          "center_interval_width_m": width, "half_width_m": width / 2,
                          "footprint_radius_m": radius, "required_clearance_m": required_clearance,
                          "interval_query_step_m_max": .001, "checked_crossing_segment_world_m": cross.tolist(),
                          "checked_crossing": crossing,
                          "direction": "unassigned; event raw crossing must determine sign and necessity",
                          "source_rule": "actual named door-frame plane plus direct obstacle/workspace queries; no wall AABB filling"})
    return gates


def derive_corner_side_gate(export_directory, boundary, fresh, *, corner_world_xy,
                            tangent_world_xy, source_mesh_ids, ray_length=12.,
                            radius=.20, required_clearance=.05):
    """Validate a scene-identified corner-to-opposite-obstacle passage witness.

    Caller supplies a real convex corner and outward bisector justified by
    adjacent scene walls. This function does no global route planning. It
    requires an exact safe interval bounded by actual obstacles and one
    positive raw crossing; unknown workspace cannot terminate the interval.
    """
    from shapely.geometry import Point
    root = Path(export_directory)
    env = HospitalEnvironment.load(root)
    corner = np.asarray(corner_world_xy, dtype=float)
    tangent = np.asarray(tangent_world_xy, dtype=float)
    tangent /= np.linalg.norm(tangent)
    normal = np.array([tangent[1], -tangent[0]])
    if env.obstacle_distance(corner) > 1e-6:
        raise ValueError("Corner must belong to actual obstacle geometry")
    distances = np.arange(0., ray_length + .0001, .001)
    xy = corner + distances[:, None] * tangent
    good = (env.clearance(xy, radius) >= required_clearance + 1e-5) & env.workspace_status(xy, radius)
    changes = np.diff(np.r_[False, good, False].astype(int))
    starts, stops = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
    if not len(starts):
        raise ValueError("No safe known passage on proposed corner ray")
    start, stop = starts[0], stops[0]
    if stop >= len(xy) or not env.workspace_status(xy[stop], radius) or env.clearance(xy[stop], radius) >= required_clearance:
        raise ValueError("Opposite gate end not established by an actual obstacle")
    # Exclude one extraction cell from each end. This is a fixed geometric
    # precision rule, not case/result-dependent gate narrowing.
    lo, hi = float(distances[start] + .001), float(distances[stop - 1] - .001)
    endpoints = corner + np.array([lo, hi])[:, None] * tangent
    sweep = env.check_polyline(endpoints, radius=radius, required_clearance=required_clearance)
    if not sweep["clearance_valid"]:
        raise ValueError("Gate interior does not pass independent continuous segment check")
    raw = np.asarray(fresh, dtype=float)[:, :2]
    crossings = []
    for index, (a, b) in enumerate(zip(raw[:-1], raw[1:])):
        sa, sb = (a - corner) @ normal, (b - corner) @ normal
        if sa < 0. <= sb:
            alpha = sa / (sa - sb)
            p = a + alpha * (b - a)
            distance = float((p - corner) @ tangent)
            if lo <= distance <= hi:
                crossings.append({"raw_segment_index": index, "segment_fraction": float(alpha),
                                  "position_world_xy_m": p.tolist(), "corner_ray_distance_m": distance,
                                  "clearance_m": float(env.clearance(p, radius))})
    if len(crossings) != 1 or (np.asarray(boundary)[:2] - corner) @ normal >= 0:
        raise ValueError("One raw positive crossing from a pre-gate boundary is required")
    part_index = int(env.tree.nearest(Point(xy[stop])))
    part_records = json.loads((root / "geometry/projected/obstacle_parts.json").read_text())
    opposite_id = part_records[part_index]["mesh_id"]
    meshes = json.loads((root / "geometry/meshes.json").read_text())
    return {"gate_id": "actual_corner_to_stairs_passage", "center_world_xy_m": endpoints.mean(axis=0).tolist(),
            "normal_world_xy": normal.tolist(), "tangent_world_xy": tangent.tolist(),
            "center_interval_endpoints_world_m": endpoints.tolist(),
            "center_interval_width_m": float(np.linalg.norm(endpoints[1] - endpoints[0])),
            "half_width_m": float(np.linalg.norm(endpoints[1] - endpoints[0]) / 2),
            "corner_world_xy_m": corner.tolist(), "ray_interval_m": [lo, hi],
            "source_mesh_ids": list(source_mesh_ids), "source_prim_paths": [meshes[i]["prim_path"] for i in source_mesh_ids],
            "opposite_obstacle_mesh_id": opposite_id, "opposite_obstacle_prim_path": meshes[opposite_id]["prim_path"],
            "positive_raw_crossing": crossings[0], "checked_gate_interval": sweep,
            "direction": "normal_world_xy positive crossing required", "interval_query_step_m_max": .001,
            "extraction_endpoint_shrink_m": .001,
            "footprint_radius_m": radius, "required_clearance_m": required_clearance,
            "route_semantics": "local corner-side passage preservation proxy from original scene and raw FRESH; not a global route or instruction label"}


def _transform_validation_path(export_directory):
    """Accept preserved technical records and current capture-prefixed records."""
    evidence = Path(export_directory) / "evidence"
    for name in ("transform_and_layer_validation.json",
                 "actual_corner_passage_transform_and_layer_validation.json",
                 "hospital_corridor_checkpoints_transform_and_layer_validation.json"):
        path = evidence / name
        if path.is_file():
            return path
    raise FileNotFoundError("No independent scene transform/layer validation record")


def validate_actual_hospital_environment(export_directory):
    """Freeze real-scene geometry evidence after technical corrections, before primary."""
    import shapely
    root = Path(export_directory)
    if (root / "validation.json").exists():
        raise FileExistsError(root / "validation.json")
    env = HospitalEnvironment.load(root)
    provenance = json.loads((root / "scene_provenance.json").read_text())
    meshes = json.loads((root / "geometry/meshes.json").read_text())
    transform_path = _transform_validation_path(root)
    transform = json.loads(transform_path.read_text())
    points = {"free_floor": [19., 26.7], "wall_overlap": [17.2, 27.9],
              "actual_doorway": [17.2, 26.9], "open_door_furniture_overlap": [16.6, 26.1],
              "unknown_outside_workspace": [100., 100.]}
    results = {name: env.query(xy) for name, xy in points.items()}
    doorway = env.check_polyline([[18., 26.9], [16.3, 26.9]])
    bounds = np.asarray(env.workspace.bounds)
    random_xy = np.random.default_rng(20260918).uniform(bounds[:2] + .1, bounds[2:] - .1, (2000, 2))
    exact = env.obstacle_distance(random_xy)
    grid = env.optimizer_distance(random_xy, conservative=False)
    query_error = float(np.max(np.abs(exact - grid)))
    # Direct full-union GEOS calls are independent of STRtree nearest-part selection.
    direct = shapely.distance(shapely.points(random_xy[:200]), env.obstacles)
    index_error = float(np.max(np.abs(exact[:200] - direct)))
    floor_id = next(i for i, mesh in enumerate(meshes) if "floor" in mesh["prim_path"].lower() and "DoorFloor" not in mesh["prim_path"] and mesh["retained_triangle_count"] > 0)
    handrail_id = 881
    checks = {
        "resolved_composition": not provenance["composition_errors"] and not provenance["missing_geometry"] and not provenance["unsupported_geometry"],
        "free_floor_valid": results["free_floor"]["status"] == "CLEARANCE_VALID",
        "wall_overlap_detected": results["wall_overlap"]["clearance_m"] < 0,
        "doorway_preserved_and_known": results["actual_doorway"]["status"] == "CLEARANCE_VALID" and doorway["clearance_valid"],
        "furniture_overlap_detected": results["open_door_furniture_overlap"]["clearance_m"] < 0,
        "unknown_not_free": results["unknown_outside_workspace"]["status"] == "UNKNOWN_WORKSPACE",
        "floor_is_ground_not_obstacle": floor_id in env.metadata["floor_mesh_ids"] and floor_id not in env.metadata["obstacle_mesh_ids"],
        "overhead_handrail_excluded_by_height": meshes[handrail_id]["world_bounds_m"][0][2] > .65 and meshes[handrail_id]["retained_triangle_count"] == 0,
        "independent_transform_checks": transform["maximum_error_m"] <= 1e-9 and transform["all_nonanonymous_layer_hashes_unchanged"],
        "grid_error_within_lipschitz_bound": query_error <= env.metadata["grid_distance_absolute_error_bound_m"] + 1e-9,
        "independent_full_union_distance_matches_index": index_error <= 1e-7,
        "all_gate_intervals_checked": all(g.get("checked_gate_interval", g.get("checked_crossing", {})).get("clearance_valid", False) for g in env.gates),
        "actual_scene_screenshots_present": all((root / f"evidence/{name}.png").is_file() for name in ("hospital_scene_context", "hospital_corridor_checkpoints", "actual_corner_passage")),
    }
    # Scientific map, with all point queries and original obstacle shapes visible.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from shapely.plotting import plot_polygon, plot_line
    def draw_geometry(ax, geometry, color, alpha):
        if geometry.is_empty:
            return
        if geometry.geom_type == "Polygon":
            plot_polygon(geometry, ax=ax, add_points=False, color=color, alpha=alpha)
        elif geometry.geom_type in ("LineString", "LinearRing"):
            plot_line(geometry, ax=ax, add_points=False, color=color, linewidth=1.)
        elif hasattr(geometry, "geoms"):
            for item in geometry.geoms:
                draw_geometry(ax, item, color, alpha)
    figure, axes = plt.subplots(1, 2, figsize=(13., 5.4))
    ax = axes[0]
    clip = shapely.box(15.5, 25.1, 20.0, 28.7)
    draw_geometry(ax, env.workspace.intersection(clip), "#dfeadf", .6)
    draw_geometry(ax, env.obstacles.intersection(clip), "#333333", .85)
    for name, color in [("free_floor", "green"), ("wall_overlap", "red"), ("actual_doorway", "deepskyblue"), ("open_door_furniture_overlap", "darkorange")]:
        xy = points[name]
        ax.plot(*xy, "o", color=color, markersize=5.)
        ax.add_patch(Circle(xy, .20, edgecolor=color, facecolor="none", linewidth=1.2))
        ax.add_patch(Circle(xy, .25, edgecolor=color, facecolor="none", linestyle="--", linewidth=.8))
        ax.annotate(name.replace("_", " "), xy, xytext=(4, 5), textcoords="offset points", fontsize=7.5)
    ax.plot([18., 16.3], [26.9, 26.9], color="deepskyblue", linewidth=2., label="validated doorway sweep")
    ax.set(xlim=(15.5, 20.), ylim=(25.1, 28.7), xlabel="world X [m]", ylabel="world Y [m]", title="Actual Hospital oracle geometry and footprint queries")
    ax.set_aspect("equal"); ax.grid(alpha=.15)
    ax = axes[1]
    ax.axhspan(.05, .65, color="gold", alpha=.3, label="obstacle height band [.05,.65] m")
    ax.axhline(0., color="green", linewidth=2., label="ground reference z=0 m")
    ax.axhline(.001, color="darkgreen", linestyle=":", label="authored floor approx z=.001 m")
    rail = meshes[handrail_id]["world_bounds_m"]
    ax.plot([.5, .5], [rail[0][2], rail[1][2]], linewidth=10., color="grey", label="actual handrail excluded above band")
    ax.set(xlim=(0., 1.), ylim=(-.05, 1.12), ylabel="world Z [m]", xticks=[], title="Height selection and transform validation")
    ax.text(.03, .68, f'{provenance["instance_proxy_mesh_count"]} instance-proxy meshes\n{len(transform["transform_checks"])} independent Gf transform checks\nmax error {transform["maximum_error_m"]:.1e} m\n{transform["checked_layer_count"]} unchanged layer hashes', fontsize=9)
    ax.legend(loc="upper right", fontsize=7.)
    figure.suptitle("GP-SE2-01 static oracle validation | radius .20 m + required clearance .05 m")
    figure.text(.02, .015, f'Raw geometry SHA256 {env.metadata["source_sha256"]["geometry/world_triangles.npz"][:20]} | no scene visibility modifications | independent exact checker', fontsize=8.)
    figure.tight_layout(rect=(0., .04, 1., .94))
    plot_path = root / "evidence/environment_validation_map.png"
    if plot_path.exists():
        raise FileExistsError(plot_path)
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)
    report = {"valid": all(checks.values()), "checks": checks, "point_checks": results,
              "doorway_swept_check": doorway, "height_checks": {"floor_mesh_id": floor_id, "overhead_handrail_mesh_id": handrail_id},
              "transform_check_file": str(transform_path.relative_to(root)),
              "independent_query_random_seed": 20260918, "grid_validation_points": 2000,
              "maximum_observed_grid_error_m": query_error,
              "grid_absolute_error_bound_m": env.metadata["grid_distance_absolute_error_bound_m"],
              "independent_full_union_check_points": 200, "maximum_index_distance_error_m": index_error,
              "direct_checker_tolerance_m": env.numerical_tolerance_m,
              "workspace_area_m2": env.workspace.area, "unknown_interior_area_m2": env.metadata["unknown_interior_area_m2"],
              "footprint_radius_m": .20, "required_clearance_m": .05, "gate_catalog_count": len(env.gates),
              "checker_level": "exact projected polygon/line point and swept-polyline circle geometry; curved GP uses explicit separate refinement/bounds",
              "scope": "static authored Hospital oracle and selected experimental footprint; not physical robot safety",
              "technical_trials_preserved": ["environment_technical_20260918", "environment_technical_20260918_retry02", "environment_technical_20260918_retry03"],
              "processing_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "artifact_sha256": {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in sorted(root.rglob("*")) if path.is_file()}}
    _json_save(root / "validation.json", report)
    return report


class HospitalEnvironment:
    """Immutable environment query object with explicit unknown-workspace states."""

    def __init__(self, obstacles, workspace, *, parts=None, grid=None, metadata=None):
        from shapely import STRtree
        self.obstacles = obstacles
        self.workspace = workspace
        self.parts = list(parts) if parts is not None else [obstacles]
        self.tree = STRtree(self.parts)
        self.grid = grid
        self.metadata = metadata or {}
        self.numerical_tolerance_m = float(self.metadata.get("numerical_tolerance_m", 1e-7))
        self.obstacle_uncertainty_m = self.numerical_tolerance_m
        self.gates = []  # Added only by validated scene/raw-route gate extraction.

    @classmethod
    def load(cls, export_directory):
        import shapely
        root = Path(export_directory) / "geometry/projected"
        obstacles = shapely.from_wkb((root / "obstacles.wkb").read_bytes())
        workspace = shapely.from_wkb((root / "workspace.wkb").read_bytes())
        parts = [shapely.from_wkb(bytes.fromhex(x["wkb_hex"])) for x in json.loads((root / "obstacle_parts.json").read_text())]
        with np.load(root / "distance_grid.npz", allow_pickle=False) as values:
            grid = {key: values[key].copy() for key in values.files}
        result = cls(obstacles, workspace, parts=parts, grid=grid,
                     metadata=json.loads((root / "projection.json").read_text()))
        gate_path = Path(export_directory) / "gate_catalog.json"
        if not gate_path.exists():
            gate_path = Path(export_directory) / "doorway_gates.json"
        if gate_path.exists():
            result.gates = json.loads(gate_path.read_text())["gates"]
        return result

    def obstacle_distance(self, xy):
        """Independent exact center-to-projected-obstacle distance, in metres."""
        import shapely
        xy = np.asarray(xy, dtype=float)
        scalar = xy.shape == (2,)
        points = shapely.points(xy.reshape((-1, 2)))
        nearest = self.tree.nearest(points)
        values = shapely.distance(points, np.asarray(self.parts, dtype=object)[nearest])
        return float(values[0]) if scalar else values.reshape(xy.shape[:-1])

    def clearance(self, xy, radius=.20):
        """Footprint clearance = center distance - radius, applied exactly once."""
        return self.obstacle_distance(xy) - radius

    def optimizer_distance(self, xy, *, conservative=True):
        """Grid approximation; subtract its absolute error bound by default."""
        xy = np.asarray(xy, dtype=float)
        scalar = xy.shape == (2,)
        if self.grid is None:
            return self.obstacle_distance(xy)
        q = (xy.reshape((-1, 2)) - self.grid["origin"]) / self.grid["resolution"]
        indices = np.floor(q).astype(int)
        values = np.full(len(q), np.nan)
        z = self.grid["distances"]
        valid = (indices[:, 0] >= 0) & (indices[:, 1] >= 0) & (indices[:, 0] < z.shape[1] - 1) & (indices[:, 1] < z.shape[0] - 1)
        i, j = indices[valid].T
        a, b = (q[valid] - indices[valid]).T
        values[valid] = (1-a)*(1-b)*z[j, i] + a*(1-b)*z[j, i+1] + (1-a)*b*z[j+1, i] + a*b*z[j+1, i+1]
        if conservative:
            values -= float(self.metadata.get("grid_distance_absolute_error_bound_m", 0.))
        return float(values[0]) if scalar else values.reshape(xy.shape[:-1])

    def optimizer_clearance(self, xy, radius=.20):
        return self.optimizer_distance(xy) - radius

    def workspace_status(self, xy, radius=.20):
        values = self.workspace_margin(xy, radius)
        return values >= self.numerical_tolerance_m

    def workspace_margin(self, xy, radius=.20):
        """Signed center-to-known-ground boundary distance minus footprint radius."""
        import shapely
        xy = np.asarray(xy, dtype=float)
        scalar = xy.shape == (2,)
        points = shapely.points(xy.reshape((-1, 2)))
        inside = shapely.covers(self.workspace, points)
        margin = shapely.distance(points, self.workspace.boundary)
        values = np.where(inside, margin, -margin) - radius
        return float(values[0]) if scalar else values.reshape(xy.shape[:-1])

    def query(self, xy, radius=.20, required_clearance=.05):
        clearance = float(self.clearance(xy, radius))
        known = bool(self.workspace_status(xy, radius))
        eps = self.numerical_tolerance_m
        status = "UNKNOWN_WORKSPACE" if not known else (
            "PHYSICAL_OVERLAP" if clearance < -eps else (
                "CLEARANCE_VIOLATION" if clearance < required_clearance - eps else (
                    "BORDERLINE" if clearance < required_clearance + eps else "CLEARANCE_VALID")))
        return {"position_world_xy_m": np.asarray(xy).tolist(), "workspace_known": known,
                "center_obstacle_distance_m": clearance + radius,
                "clearance_m": clearance, "footprint_radius_m": radius,
                "required_clearance_m": required_clearance, "status": status,
                "checker": "DIRECT_GEOMETRY_POINT_QUERY"}

    def check_polyline(self, points, *, radius=.20, required_clearance=.05, max_step=.01):
        """Exact circular sweep of a piecewise-linear XY path, including thin walls.

        ``max_step`` is recorded for caller compatibility but GEOS checks entire
        segments; it does not certify a distinct curved GP interpolation.
        """
        from shapely.geometry import LineString, Point
        xy = np.asarray(points, dtype=float)[:, :2]
        if not len(xy) or not np.all(np.isfinite(xy)):
            raise ValueError("finite, nonempty polyline required")
        path = Point(xy[0]) if len(xy) == 1 else LineString(xy)
        known = self.workspace.covers(path) and path.distance(self.workspace.boundary) >= radius + self.numerical_tolerance_m
        clearance = float(path.distance(self.parts[self.tree.nearest(path)]) - radius)
        eps = self.numerical_tolerance_m
        status = "UNKNOWN_WORKSPACE" if not known else (
            "PHYSICAL_OVERLAP" if clearance < -eps else (
                "CLEARANCE_VIOLATION" if clearance < required_clearance - eps else (
                    "BORDERLINE" if clearance < required_clearance + eps else "SWEPT_POLYLINE_CLEARANCE_VALID")))
        first_collision_segment = None
        if clearance < -eps:
            for index in range(max(0, len(xy) - 1)):
                segment = LineString(xy[index:index + 2])
                if segment.distance(self.parts[self.tree.nearest(segment)]) < radius - eps:
                    first_collision_segment = index
                    break
        return {"status": status, "workspace_known": bool(known), "unknown": not bool(known), "minimum_clearance_m": clearance,
                "physical_overlap": clearance < -eps, "clearance_valid": bool(known and clearance >= required_clearance + eps),
                "footprint_radius_m": radius, "required_clearance_m": required_clearance,
                "checker": "DIRECT_GEOMETRY_SWEPT_CIRCLE_ALONG_PIECEWISE_LINEAR_XY",
                "sample_count": len(xy), "sampling_hint_m": max_step,
                "first_collision_segment_index": first_collision_segment,
                "continuous_time_claim": "only the supplied piecewise-linear path; callers must bound curved-path deviation"}

    def check_trajectory(self, times, poses, *, radius=.20, required_clearance=.05, curved_path_error_bound_m=0.):
        times, poses = np.asarray(times), np.asarray(poses)
        if times.ndim != 1 or len(times) != len(poses) or len(times) < 2 or np.any(np.diff(times) <= 0):
            raise ValueError("strictly increasing timestamps matching poses required")
        report = self.check_polyline(poses, radius=radius, required_clearance=required_clearance + curved_path_error_bound_m)
        if curved_path_error_bound_m < 0:
            raise ValueError("curve deviation bound must be nonnegative")
        if curved_path_error_bound_m > 0:
            from shapely.geometry import LineString
            line = LineString(poses[:, :2])
            known = self.workspace.covers(line) and line.distance(self.workspace.boundary) >= radius + curved_path_error_bound_m + self.numerical_tolerance_m
            report["workspace_known"] = bool(known)
            report["unknown"] = not bool(known)
            if not known:
                report["status"] = "UNKNOWN_WORKSPACE"
                report["clearance_valid"] = False
        report.update(max_time_step_s=float(np.max(np.diff(times))), curved_path_error_bound_m=float(curved_path_error_bound_m),
                      minimum_clearance_lower_bound_m=report["minimum_clearance_m"] - curved_path_error_bound_m,
                      validation_level="SWEPT_POLYLINE_WITH_EXPLICIT_CURVE_BOUND" if curved_path_error_bound_m > 0 else "SAMPLED_CLEARANCE_VALIDATION")
        return report
