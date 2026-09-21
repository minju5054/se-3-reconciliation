#!/usr/bin/env python3
"""JOIN-SOURCE-02 renderer/geometry preflight; zero model/controller calls.

Run with the installed Isaac Python launcher. Source scene and external assets
are read-only. Each mode exclusively creates its own technical_preflight folder.
Inspection images establish actual target visibility before instruction freeze.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts/isaac")]

LOCATIONS = [
    ("location_01", [19.2, 28.0, -math.pi / 2]),
    ("location_02", [19.2, 24.0, -math.pi / 2]),
]
PROP_SOURCE = "/World/Hospital/SM_SupplyCart_02a7"
PROP_ROOT = "/World/JOINSource02Obstacle"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False,
                  default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x))


def collect_meshes(stage, prefix):
    import numpy as np
    from pxr import Usd, UsdGeom
    from reconciliation.gp_se2_environment import triangulate_face, transform_points
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    scale = float(UsdGeom.GetStageMetersPerUnit(stage))
    blocks, records = [], []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not (str(prim.GetPath()) == prefix or str(prim.GetPath()).startswith(prefix + "/")):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
        holes = set(mesh.GetHoleIndicesAttr().Get() or [])
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=np.float64).T
        world = transform_points(points, matrix, scale)
        triangles, cursor = [], 0
        for i, count in enumerate(counts):
            face = world[indices[cursor:cursor + count]]
            cursor += count
            if i not in holes:
                triangles.extend(triangulate_face(face))
        if cursor != len(indices):
            raise ValueError("Invalid mesh topology")
        block = np.asarray(triangles, dtype=np.float64)
        blocks.append(block)
        records.append(dict(prim_path=str(prim.GetPath()), instance_proxy=prim.IsInstanceProxy(),
            point_count=len(points), triangle_count=len(block), holes=sorted(holes),
            source_layers=sorted({x.layer.identifier for x in prim.GetPrimStack()}),
            world_bounds_m=[world.min(axis=0).tolist(), world.max(axis=0).tolist()],
            T_world_mesh_column=matrix.tolist()))
    if not blocks:
        raise ValueError(f"No actual meshes below {prefix}")
    return np.concatenate(blocks), records


def instance_data(annotator, prefix=None):
    import numpy as np
    result = annotator.get_data()
    if not isinstance(result, dict) or "data" not in result:
        raise RuntimeError("Instance annotator returned no structured mask")
    mask = np.asarray(result["data"])
    labels = result.get("info", {}).get("idToLabels", {})
    if isinstance(labels, str):
        labels = json.loads(labels)
    labels = {str(k): str(v) for k, v in labels.items()}
    if mask.ndim != 2 or mask.dtype != np.uint32:
        raise RuntimeError(f"Unexpected instance mask: {mask.shape}/{mask.dtype}")
    ids = [int(k) for k, path in labels.items()
           if prefix and (path == prefix or path.startswith(prefix + "/"))]
    selected = np.isin(mask, ids)
    ys, xs = np.nonzero(selected)
    return mask, dict(idToLabels=labels, matched_instance_ids=ids,
        visible_pixels=int(selected.sum()), image_fraction=float(selected.mean()),
        bbox_xyxy=None if not len(xs) else [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())])


def create_prop(stage, source_root, desired_center_xy, desired_yaw=None, *, runtime_root=PROP_ROOT):
    """Make initially hidden internal reference and authenticate exact rigid mesh relocation.

    ``desired_yaw`` is world yaw of source root local +X, or None to preserve it.
    Returned ``record`` and triangles are serializable; ``prim`` is runtime-only.
    """
    import numpy as np
    from pxr import Gf, Usd, UsdGeom
    if stage.GetPrimAtPath(runtime_root).IsValid():
        raise FileExistsError(runtime_root)
    original, original_meshes = collect_meshes(stage, source_root)
    original_root = stage.GetPrimAtPath(source_root)
    original_matrix = np.asarray(UsdGeom.Xformable(original_root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
    wrapper = UsdGeom.Xform.Define(stage, runtime_root)
    clone = stage.DefinePrim(runtime_root + "/Prop")
    if not clone.GetReferences().AddInternalReference(source_root):
        raise RuntimeError("Internal reference failed")
    before, _ = collect_meshes(stage, runtime_root)
    if original.shape != before.shape:
        raise RuntimeError("Internal reference changed source topology")
    source_to_before = before.reshape(-1, 3).mean(axis=0) - original.reshape(-1, 3).mean(axis=0)
    if not np.allclose(before, original + source_to_before, atol=1e-8, rtol=0):
        raise RuntimeError("Internal reference changed source geometry/orientation")
    center = .5 * (before.min(axis=(0, 1)) + before.max(axis=(0, 1)))
    source_yaw = math.atan2(original_matrix[1, 0], original_matrix[0, 0])
    yaw_delta = 0.0 if desired_yaw is None else float(desired_yaw) - source_yaw
    c, s = math.cos(yaw_delta), math.sin(yaw_delta)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    destination = np.r_[np.asarray(desired_center_xy, dtype=float), center[2]]
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = destination - rotation @ center
    wrapper.AddTransformOp().Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))
    from omni.replicator.core import functional as F
    F.modify.semantics(wrapper.GetPrim(), {"class": ["JOINSource02Obstacle"]}, mode="replace")
    actual, actual_meshes = collect_meshes(stage, runtime_root)
    predicted = before @ rotation.T + matrix[:3, 3]
    parity = np.allclose(actual, predicted, atol=1e-8, rtol=0)
    if not parity:
        raise RuntimeError("Actual cloned mesh disagrees with declared rigid transform")
    source_delta = matrix.copy()
    source_delta[:3, 3] += rotation @ source_to_before
    record = dict(source_prim=source_root, runtime_prim=runtime_root,
        source_meshes=original_meshes, actual_meshes=actual_meshes,
        original_root_world_matrix_column=original_matrix.tolist(),
        source_to_actual_world_matrix_column=source_delta.tolist(),
        wrapper_matrix_column=matrix.tolist(), source_mesh_parity=bool(parity),
        source_root_yaw_rad=source_yaw, desired_root_yaw_rad=desired_yaw,
        desired_center_xy_m=np.asarray(desired_center_xy).tolist(),
        actual_center_xy_m=(.5 * (actual.min(axis=(0,1)) + actual.max(axis=(0,1))))[:2].tolist(),
        actual_world_bounds_m=[actual.min(axis=(0,1)).tolist(), actual.max(axis=(0,1)).tolist()],
        frame="world Z-up metres; column-vector transforms", source_geometry_unchanged=True,
        renderer_mesh_replaced_with_box=False, radius_m=.20, required_edge_clearance_m=.05,
        evaluation_height_band_m=[.05, .65])
    prop = dict(prim=wrapper.GetPrim(), record=record, triangles=actual,
                source_triangles=original, present=None)
    set_present(prop, False)
    return prop


def set_present(prop, present):
    """Set USD visibility and logical presence together; oracle union uses this flag."""
    from pxr import UsdGeom
    present = bool(present)
    imageable = UsdGeom.Imageable(prop["prim"])
    imageable.MakeVisible() if present else imageable.MakeInvisible()
    prop["present"] = present
    prop["record"]["present"] = present
    prop["record"]["computed_visibility"] = str(imageable.ComputeVisibility())


def same_product_annotators(rgb):
    """Attach masks to the exact RGB product on this pinned Replicator version."""
    import omni.replicator.core as rep
    products = list(rgb._render_products)
    if len(products) != 1:
        raise RuntimeError(f"Expected one RGB render product: {products}")
    instance = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False})
    semantic = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": False})
    instance.attach(products[0]); semantic.attach(products[0])
    return instance, semantic, products[0]


def snapshot(out, name, config, app, agent, camera, rgb, instance, semantic, *, prefix=None):
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    from PIL import Image
    from robotless_runtime import actual_pose, camera_metadata
    timeline = omni.timeline.get_timeline_interface()
    pose = actual_pose(agent).copy()
    camera_before = camera_metadata(camera, agent, config)
    time_before = float(timeline.get_current_time())
    capture_started_ns = time.monotonic_ns()
    # Same held camera/state, no physics timeline advance. Warm rendering of the
    # visibility edit is technical setup, never fabricated history delivery.
    for _ in range(4):
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
    rgba = np.asarray(rgb.get_data()).copy()
    mask, metadata = instance_data(instance, prefix)
    capture_completed_ns = time.monotonic_ns()
    capture_completed_utc = datetime.now(timezone.utc).isoformat()
    sem = semantic.get_data()
    shape = (int(config["camera"]["resolution_height"]), int(config["camera"]["resolution_width"]))
    stable = (np.array_equal(pose, actual_pose(agent)) and camera_before == camera_metadata(camera, agent, config)
              and time_before == float(timeline.get_current_time()))
    if not stable or rgba.shape != (*shape, 4) or mask.shape != shape:
        raise RuntimeError("RGB/mask camera or state synchronization failed")
    with (out / (name + ".jpg")).open("xb") as stream:
        Image.fromarray(rgba[:, :, :3]).save(stream, format="JPEG", quality=int(config["capture"]["jpeg_quality"]))
    with (out / (name + "_instance.npz")).open("xb") as stream:
        np.savez_compressed(stream, mask=mask)
    if isinstance(sem, dict) and "data" in sem:
        with (out / (name + "_semantic.npz")).open("xb") as stream:
            np.savez_compressed(stream, mask=np.asarray(sem["data"]))
    visible_objects = []
    for label_id, path in metadata["idToLabels"].items():
        n = int(np.count_nonzero(mask == int(label_id)))
        if n and path.startswith("/World/Hospital/"):
            visible_objects.append(dict(prim_path=path, instance_id=int(label_id), pixels=n))
    record = dict(name=name, agent_pose_world=pose.tolist(), camera=camera_before,
        capture_started_monotonic_ns=capture_started_ns,
        capture_completed_monotonic_ns=capture_completed_ns,
        capture_completed_utc=capture_completed_utc,
        stable_camera_pose_and_simulation_time=stable, simulation_time_s=time_before,
        render_step_count=4, render_delta_time_s=0.0, rgba_sha256=hashlib.sha256(rgba.tobytes()).hexdigest(),
        rgb_file=name + ".jpg", rgb_jpeg_sha256=sha(out / (name + ".jpg")),
        mask_file=name + "_instance.npz", mask_sha256=sha(out / (name + "_instance.npz")),
        instance=metadata, semantic_info=sem.get("info", {}) if isinstance(sem, dict) else None,
        visible_scene_objects=sorted(visible_objects, key=lambda x: (-x["pixels"], x["prim_path"])),
        same_render_product=True, diagnostic_only=True, model_input_annotations=False,
        model_calls=0, MPC_calls=0, source_history_frame=False)
    save(out / (name + ".json"), record)
    return record


capture_sameproduct = snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--mode", choices=["inspect", "visibility"], required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/robotless_online_handoffs.yaml")
    parser.add_argument("--location", choices=[x[0] for x in LOCATIONS], default="location_01")
    parser.add_argument("--prop-source", default=PROP_SOURCE)
    parser.add_argument("--ahead-m", type=float, default=1.0)
    args = parser.parse_args()
    out = args.run.resolve() / "technical_preflight" / args.mode
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    import numpy as np
    import yaml
    config = yaml.safe_load(args.config.read_text())
    save(out / "invocation.json", dict(mode=args.mode, config=str(args.config.resolve()),
        config_sha256=sha(args.config), script_sha256=sha(__file__), locations=LOCATIONS,
        prop_source=args.prop_source, ahead_m=args.ahead_m, external_source_modified=False,
        new_inference_calls=0, new_MPC_calls=0))
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, renderer="RayTracedLighting", anti_aliasing=0, display_options=0))
    try:
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        from robotless_runtime import runtime_scene, set_agent_pose
        first = dict(LOCATIONS)[args.location]
        stage, agent, camera, rgb, scene = runtime_scene(config, np.asarray(first), app=app)
        # Installed Replicator 1.13.27 explicitly stores actual attached paths
        # here. Use the RGB product itself, not another same-camera product.
        instance, semantic, product = same_product_annotators(rgb)
        save(out / "scene.json", dict(scene=scene, shared_render_product=product,
            instance_annotator="instance_id_segmentation", semantic_annotator="semantic_segmentation",
            installed_isaac_version=Path("/home/gpuadmin/isaacsim/VERSION").read_text().strip()))
        records = []
        if args.mode == "inspect":
            for name, pose in LOCATIONS:
                set_agent_pose(agent, np.asarray(pose), z_m=float(config["agent"]["z_m"]))
                records.append(snapshot(out, name, config, app, agent, camera, rgb, instance, semantic))
            save(out / "validation.json", dict(valid=all(r["stable_camera_pose_and_simulation_time"] for r in records),
                inspected_locations=2, target_identity_requires_actual_RGB_review=True,
                obstacle_visibility_preflight_completed=False, model_calls=0, MPC_calls=0,
                elapsed_wall_s=time.monotonic() - started))
            return

        pose = np.asarray(first)
        desired_xy = pose[:2] + args.ahead_m * np.array([np.cos(pose[2]), np.sin(pose[2])])
        prop = create_prop(stage, args.prop_source, desired_xy)
        with (out / "actual_prop_triangles.npz").open("xb") as stream:
            np.savez_compressed(stream, triangles=prop["triangles"], source_triangles=prop["source_triangles"])
        save(out / "prop_geometry.json", dict(prop["record"], triangles_sha256=sha(out / "actual_prop_triangles.npz")))
        for name, visible in [("obstacle_off", False), ("obstacle_on", True), ("obstacle_off_sham", False)]:
            set_present(prop, visible)
            records.append(snapshot(out, name, config, app, agent, camera, rgb, instance, semantic, prefix=PROP_ROOT))
        counts = [r["instance"]["visible_pixels"] for r in records]
        state_parity = all(r["agent_pose_world"] == records[0]["agent_pose_world"] and r["camera"] == records[0]["camera"]
                           and r["simulation_time_s"] == records[0]["simulation_time_s"] for r in records)
        checks = dict(hidden_zero=counts[0] == 0, visible_at_least_20=counts[1] >= 20,
            hidden_again_zero=counts[2] == 0, same_pose_camera_time=state_parity,
            source_mesh_parity=prop["record"]["source_mesh_parity"], same_render_product=True,
            no_other_scene_object_matches_runtime_root=all(not str(v).startswith("/World/Hospital/")
                for k, v in records[1]["instance"]["idToLabels"].items()
                if int(k) in records[1]["instance"]["matched_instance_ids"]))
        save(out / "validation.json", dict(valid=all(checks.values()), checks=checks,
            visible_pixels_off_on_off=counts, interpretation="Instrument validation, not model obstacle recognition",
            runtime_geometry_present_follows_visibility=True, historical_environment_not_mutated=True,
            evaluator_union_must_activate_only_when_visible=True, model_calls=0, MPC_calls=0,
            elapsed_wall_s=time.monotonic() - started))
    except BaseException as exc:
        save(out / "failure.json", dict(type=type(exc).__name__, reason=str(exc),
            model_calls=0, MPC_calls=0, elapsed_wall_s=time.monotonic() - started))
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
