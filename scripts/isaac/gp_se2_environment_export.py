#!/usr/bin/env python3
"""Export the actual, composed Hospital mesh surfaces in metre world coordinates.

Run with Isaac's Python launcher. No robot, physics stepping, or VLA inference.
This exports surfaces, not collider bounds. Offline projection/checking lives in
reconciliation.gp_se2_environment and uses an independent exact geometry checker.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def save_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False,
                  default=lambda item: item.item() if hasattr(item, "item") else str(item))


def export(args, app):
    import numpy as np
    import yaml
    from pxr import Usd, UsdGeom
    from robotless_runtime import runtime_scene
    from reconciliation.gp_se2_environment import triangulate_face, transform_points

    config = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "geometry").mkdir()
    (args.output / "evidence").mkdir()
    stage, _, _, _, scene = runtime_scene(config, np.array(config["agent"]["pose_world"]), app=app)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    scale = float(UsdGeom.GetStageMetersPerUnit(stage))
    if str(UsdGeom.GetStageUpAxis(stage)).upper() != "Z":
        raise ValueError("Only explicitly Z-up stages are supported")
    root_path = config["scene"]["reference_prim_path"]
    records, triangle_blocks, ids, unsupported, missing = [], [], [], [], []
    analytic_ground_planes = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not str(prim.GetPath()).startswith(root_path + "/"):
            continue
        if prim.HasPayload() and not prim.IsLoaded():
            missing.append({"prim": str(prim.GetPath()), "reason": "unloaded payload"})
        if prim.IsA(UsdGeom.PointInstancer):
            unsupported.append({"prim": str(prim.GetPath()), "type": "PointInstancer"})
        if not prim.IsA(UsdGeom.Mesh):
            if prim.GetTypeName() == "Plane":
                axis = str(prim.GetAttribute("axis").Get())
                transform = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=np.float64).T
                if axis.upper() in ("X", "Y", "Z"):
                    normal = np.linalg.inv(transform[:3, :3]).T @ np.eye(3)[("X", "Y", "Z").index(axis.upper())]
                    normal /= np.linalg.norm(normal)
                    z = float(transform[2, 3] * scale)
                    if abs(abs(normal[2]) - 1.) < 1e-10 and z < args.ground_z + .05:
                        analytic_ground_planes.append({"prim_path": str(prim.GetPath()), "world_normal": normal.tolist(),
                                                      "world_z_m": z, "axis": axis,
                                                      "T_world_local_column_vector_stage_units": transform.tolist(),
                                                      "reason": "exact horizontal analytic plane lies below obstacle slab; finite workspace still requires actual mesh ground"})
                        continue
            if prim.IsA(UsdGeom.Gprim):
                unsupported.append({"prim": str(prim.GetPath()), "type": prim.GetTypeName()})
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
        holes = set(mesh.GetHoleIndicesAttr().Get() or [])
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=np.float64).T
        world = transform_points(points, matrix, scale)
        if points.ndim != 2 or points.shape[1] != 3 or counts.sum() != len(indices):
            missing.append({"prim": str(prim.GetPath()), "reason": "invalid mesh topology"})
            continue
        lo, hi = world.min(axis=0), world.max(axis=0)
        record = {"prim_path": str(prim.GetPath()), "instance_proxy": prim.IsInstanceProxy(),
                  "point_count": len(points), "face_count": len(counts), "holes": sorted(holes),
                  "T_world_local_column_vector_stage_units": matrix.tolist(),
                  "world_bounds_m": [lo.tolist(), hi.tolist()],
                  "authored_subdivision_scheme": str(mesh.GetSubdivisionSchemeAttr().Get()),
                  "purpose": str(UsdGeom.Imageable(prim).ComputePurpose()),
                  "visibility": str(UsdGeom.Imageable(prim).ComputeVisibility()),
                  "source_layers": sorted({x.layer.identifier for x in prim.GetPrimStack()}),
                  "retained_triangle_count": 0}
        if hi[2] >= args.ground_z - .03 and lo[2] <= args.ground_z + .65:
            selected = []
            offset = 0
            for face_index, count in enumerate(counts):
                face = world[indices[offset:offset + count]]
                offset += count
                if face_index in holes or face[:, 2].max() < args.ground_z - .03 or face[:, 2].min() > args.ground_z + .65:
                    continue
                selected.extend(triangulate_face(face))
            if selected:
                triangles = np.asarray(selected, dtype=np.float64)
                triangle_blocks.append(triangles)
                ids.append(np.full(len(triangles), len(records), dtype=np.int32))
                record["retained_triangle_count"] = len(triangles)
        records.append(record)
    errors = [str(x) for x in stage.GetCompositionErrors()]
    # A serialized composed stage snapshot detects resolved layer or instance changes.
    flat_text = stage.Flatten().ExportToString()
    flat_hash = hashlib.sha256(flat_text.encode()).hexdigest()
    with (args.output / "geometry/world_triangles.npz").open("xb") as stream:
        np.savez_compressed(stream, triangles=np.concatenate(triangle_blocks),
                            mesh_ids=np.concatenate(ids))
    save_json(args.output / "geometry/meshes.json", records)
    layers = [{"identifier": layer.identifier, "resolved_path": layer.resolvedPath,
               "anonymous": layer.anonymous,
               "composed_layer_text_sha256": hashlib.sha256(layer.ExportToString().encode()).hexdigest()}
              for layer in stage.GetUsedLayers()]
    provenance = {"schema_version": 1, "scene": scene, "stage_units_in_meters": scale,
                  "source_coordinates": "mesh local USD stage units; nested transforms and instance proxies resolved",
                  "target_coordinates": "world, Z-up, X/Y metres, double precision",
                  "matrix_convention": "column vectors; transpose USD row-vector matrix then multiply stage metres/unit",
                  "ground_z_m": args.ground_z, "height_band_ground_relative_m": [.05, .65],
                  "retained_z_range_world_m": [args.ground_z - .03, args.ground_z + .65],
                  "mesh_count": len(records), "triangle_count": sum(x["retained_triangle_count"] for x in records),
                  "instance_proxy_mesh_count": sum(x["instance_proxy"] for x in records),
                  "composition_errors": errors, "unsupported_geometry": unsupported, "missing_geometry": missing,
                  "analytic_horizontal_planes_excluded_below_band": analytic_ground_planes,
                  "used_layers": layers, "flattened_composed_stage_text_sha256": flat_hash,
                  "config_path": str(args.config), "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  "export_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  "geometry_module_sha256": hashlib.sha256((ROOT / "src/reconciliation/gp_se2_environment.py").read_bytes()).hexdigest(),
                  "export_wall_time_unix_s": time.time(), "new_lightnav_inference_count": 0,
                  "dynamics_advanced": False,
                  "surface_semantics": "authored mesh polygon surfaces; no collider omission; holes respected; no AABB obstacle filling"}
    save_json(args.output / "scene_provenance.json", provenance)
    print(json.dumps({k: provenance[k] for k in ("mesh_count", "triangle_count", "instance_proxy_mesh_count", "unsupported_geometry", "missing_geometry", "composition_errors")}), flush=True)
    if errors or unsupported or missing:
        raise RuntimeError("Incomplete geometry: explicit unresolved/unsupported records prohibit declaring free space")
    capture(args.output, app, args.check_points)


def capture(output, app, check_points=None, capture_name="hospital_scene_context"):
    import numpy as np
    from robotless_runtime import viewport_capture
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.util.debug_draw import _debug_draw
    import omni.ui as ui
    draw = _debug_draw.acquire_debug_draw_interface()
    specification = json.loads(check_points.read_text()) if check_points else {}
    checkpoints = specification.get("points", []) if isinstance(specification, dict) else specification
    if checkpoints:
        for point in checkpoints:
            p = point["position_world_m"]
            draw.draw_points([[p[0], p[1], .15]], [point.get("color", [.1, 1., .1, 1.])], [14.])
        center = np.mean([x["position_world_m"][:2] for x in checkpoints], axis=0)
    else:
        center = np.array([19., 26.7])
    panel = ui.Window("GP-SE2-01 oracle environment validation", width=680, height=160)
    with panel.frame:
        with ui.VStack():
            ui.Label("STATIC HOSPITAL ORACLE GEOMETRY VALIDATION — NO NEW VLA")
            ui.Label("World XY metres; circular radius .20 m; required clearance .05 m")
            for point in checkpoints:
                ui.Label(f'{point["id"]}: {point["position_world_m"]}')
    eye = specification.get("camera_eye_world_m", [center[0] - 2.8, center[1] - 1.8, 2.3]) if isinstance(specification, dict) else [center[0]-2.8, center[1]-1.8, 2.3]
    target = specification.get("camera_target_world_m", [center[0], center[1], .1]) if isinstance(specification, dict) else [center[0], center[1], .1]
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    path = output / f"evidence/{capture_name}.png"
    viewport_capture(path, app=app)
    save_json(output / f"evidence/{capture_name}.json", {"check_points": checkpoints,
              "camera_eye_world_m": eye,
              "camera_target_world_m": target,
              "geometry_visibility_modifications": [], "marker_only_z_m": .15,
              "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest()})


def validate_reloaded_stage(output, stage, capture_name=""):
    """Independent Gf.Transform versus exported column-matrix checks."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    from reconciliation.gp_se2_environment import transform_points
    old = json.loads((output / "scene_provenance.json").read_text())
    expected = {x["identifier"]: x["composed_layer_text_sha256"] for x in old["used_layers"] if not x["anonymous"]}
    actual = {layer.identifier: hashlib.sha256(layer.ExportToString().encode()).hexdigest()
              for layer in stage.GetUsedLayers() if not layer.anonymous}
    if expected != actual:
        raise ValueError("Reloaded static scene's non-anonymous layer hashes changed")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    checks = []
    meshes = json.loads((output / "geometry/meshes.json").read_text())
    for index in sorted(set(list(range(0, len(meshes), 83)) + [220, 1106, 1415])):
        record = meshes[index]
        prim = stage.GetPrimAtPath(record["prim_path"])
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        local = np.asarray([points[0], points[len(points) // 2], points[-1]], dtype=float)
        usd_matrix = cache.GetLocalToWorldTransform(prim)
        scale = float(UsdGeom.GetStageMetersPerUnit(stage))
        gf_world = np.asarray([usd_matrix.Transform(Gf.Vec3d(*point)) for point in local]) * scale
        exported_world = transform_points(local, record["T_world_local_column_vector_stage_units"], scale)
        error = float(np.max(np.abs(gf_world - exported_world)))
        if error > 1e-9:
            raise ValueError("Export transform differs from direct Gf.Transform")
        checks.append({"mesh_id": index, "prim_path": record["prim_path"], "instance_proxy": prim.IsInstanceProxy(),
                       "local_points_stage_units": local.tolist(), "Gf_world_points_m": gf_world.tolist(),
                       "export_column_transform_world_points_m": exported_world.tolist(), "max_absolute_error_m": error})
    save_json(output / f"evidence/{capture_name + '_' if capture_name else ''}transform_and_layer_validation.json", {
              "all_nonanonymous_layer_hashes_unchanged": True, "checked_layer_count": len(expected),
              "transform_checks": checks, "maximum_error_m": max(x["max_absolute_error_m"] for x in checks),
              "processing_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/robotless_online_handoffs.yaml")
    parser.add_argument("--ground-z", type=float, default=0.)
    parser.add_argument("--check-points", type=Path)
    parser.add_argument("--evidence-only", action="store_true", help="Reload unchanged scene and add exclusive validation screenshot to existing export")
    parser.add_argument("--capture-name", default="hospital_scene_context")
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False})
    try:
        if args.evidence_only:
            import numpy as np
            import yaml
            from robotless_runtime import runtime_scene
            config = yaml.safe_load(args.config.read_text())
            stage, *_ = runtime_scene(config, np.array(config["agent"]["pose_world"]), app=app)
            validate_reloaded_stage(args.output, stage, args.capture_name)
            capture(args.output, app, args.check_points, args.capture_name)
        else:
            export(args, app)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
