#!/usr/bin/env python3
"""Static Isaac RGB/file handoff and passive visualization; no robot or execution."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

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
from PIL import Image
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from reconciliation.robotless_single_chunk import (
    FRAME_CONVENTIONS, frame_sanity_fixtures, load_config, make_observation_time,
    observation_to_world, save_json_exclusive, sha256_file, validate_observation_metadata,
    validate_waypoints,
)


from robotless_runtime import actual_pose, camera_metadata, git_output, runtime_scene, viewport_capture


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
    stage, agent, camera, annotator, scene = runtime_scene(config, pose, app=APP)
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
    source_paths = [Path(__file__).resolve(), ROOT / "scripts/isaac/robotless_runtime.py", ROOT / "src/reconciliation/robotless_single_chunk.py", ROOT / "src/reconciliation/se2.py", config_path]
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
    stage, agent, camera, _, scene = runtime_scene(config, pose, app=APP)
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
        viewport_capture(path, app=APP)
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
        viewport_capture(path, app=APP)
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
        "research_source_sha256": {str(p.relative_to(ROOT)): sha256_file(p) for p in (Path(__file__).resolve(), ROOT / "scripts/isaac/robotless_runtime.py", ROOT / "scripts/isaac/debug_draw_trajectories.py", ROOT / "src/reconciliation/robotless_single_chunk.py", ROOT / "src/reconciliation/se2.py")},
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
