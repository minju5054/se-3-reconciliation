#!/usr/bin/env python3
"""Two robotless Isaac observations and passive OLD/FRESH world visualization."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "visualize"))
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/robotless_lightnav_successive_chunks.yaml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--view-only", action="store_true", help="Hash-check and reopen saved trajectories without writing evidence")
    return parser.parse_args()


ARGS = arguments()
from isaacsim import SimulationApp

APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.ui as ui
from PIL import Image
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from reconciliation.robotless_single_chunk import (
    load_config, make_observation_time, observation_to_world, save_json_exclusive,
    sha256_file, validate_waypoints,
)
from reconciliation.robotless_successive_chunks import (
    SUCCESSIVE_FRAME_CONVENTIONS, successive_observation_poses, successive_to_world,
    validate_successive_observation_metadata,
)
from reconciliation.se2 import relative_pose
from robotless_runtime import (
    actual_pose, camera_metadata, capture_observation, git_output, runtime_scene, set_agent_pose, stopped_time,
    viewport_capture,
)


def source_hashes(config_path: Path) -> dict:
    paths = [
        Path(__file__).resolve(), ROOT / "scripts/isaac/robotless_runtime.py",
        ROOT / "scripts/isaac/debug_draw_trajectories.py",
        ROOT / "src/reconciliation/robotless_successive_chunks.py",
        ROOT / "src/reconciliation/robotless_single_chunk.py",
        ROOT / "src/reconciliation/se2.py", config_path,
    ]
    return {str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path): sha256_file(path) for path in paths}


def capture(config: dict, run: Path, config_path: Path) -> None:
    run.mkdir(parents=True, exist_ok=True)
    unexpected = [path.name for path in run.iterdir() if path.name not in {"logs", "source_provenance_before.json"}]
    if unexpected:
        raise FileExistsError(f"Capture requires a new run directory; found {unexpected}")
    initial, target = successive_observation_poses(config["agent"]["pose_world"], config["agent"]["local_displacement"])
    if np.allclose(initial, target, rtol=0, atol=1e-12):
        raise ValueError("Configured displacement must produce a distinct observation pose")
    external = (ROOT / config["paths"]["lightnav_checkout"]).resolve()
    lightnav_sha = git_output(external, "rev-parse", "HEAD")
    if lightnav_sha != config["lightnav"]["expected_git_sha"] or git_output(external, "status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError("LightNav checkout does not match the configured unmodified source")
    with (run / "config_snapshot.yaml").open("xb") as stream:
        stream.write(config_path.read_bytes())
    (run / "raw").mkdir(exist_ok=False)
    _, agent, camera, annotator, scene = runtime_scene(config, initial, app=APP)
    inventory = scene["runtime_inventory"]
    if inventory["rigid_body_count"] or inventory["physics_scene_count"]:
        raise RuntimeError("Unexpected rigid bodies or physics scene in robotless runtime")
    old, old_check = capture_observation(config, run, agent, camera, annotator, initial, 0)
    before = actual_pose(agent)
    displacement_start = stopped_time()
    set_agent_pose(agent, target, z_m=float(config["agent"]["z_m"]))
    displacement_end = stopped_time()
    after = actual_pose(agent)
    if not np.allclose(after, target, rtol=0, atol=1e-7):
        raise RuntimeError("Actual scripted displacement does not match composed R1")
    # Only the second camera render follows assignment. There is no physics step,
    # trajectory following, or model inference in this Isaac capture process.
    fresh, fresh_check = capture_observation(config, run, agent, camera, annotator, target, 1)
    actual_local = relative_pose(before, after)
    if not np.allclose(actual_local, config["agent"]["local_displacement"], rtol=0, atol=1e-7):
        raise RuntimeError("Measured local displacement differs from configuration")
    metadata = {
        "schema_version": 1, "stage": config["stage"], "instruction": config["instruction"],
        "R0": old["agent_pose_world"], "R1": fresh["agent_pose_world"],
        "configured_local_displacement": list(config["agent"]["local_displacement"]),
        "observations": [old, fresh], "agent_prim_path": str(agent.GetPath()),
        "agent_z_m": float(config["agent"]["z_m"]), "scene": scene,
        "scripted_displacement": {
            "configured_local_se2": list(config["agent"]["local_displacement"]),
            "actual_local_se2": actual_local.tolist(),
            "actual_world_delta_xy_m": (after[:2] - before[:2]).tolist(),
            "agent_pose_before": before.tolist(), "agent_pose_after": after.tolist(),
            "started_time": displacement_start, "completed_time": displacement_end,
            "source_frame": "logical_agent_at_observation_000", "target_frame": "isaac_world",
            "operation": "T_world_R1 = T_world_R0 @ Delta_local",
            "assignment": "direct USD logical Xform pose assignment between captures",
            "dynamics_advanced": False,
        },
        "observation_time_semantics": "host UTC and host monotonic readback-completion after each static render; capture_started_time brackets render; simulation timeline stays zero across both observations and pose assignment",
        "coordinate_conventions": SUCCESSIVE_FRAME_CONVENTIONS,
        "research_git_sha": git_output(ROOT, "rev-parse", "HEAD"),
        "research_git_status": git_output(ROOT, "status", "--short"),
        "research_source_sha256": source_hashes(config_path),
        "lightnav_git_sha": lightnav_sha, "lightnav_checkout": str(external),
        "lightnav_tracked_source_clean": True,
        "model_checkpoint_identifier": config["lightnav"]["checkpoint_identifier"],
        "model_checkpoint_revision": config["lightnav"]["checkpoint_revision"],
        "model_checkpoint_path": str((ROOT / config["paths"]["checkpoint_path"]).resolve()),
        "config": {"path": "config_snapshot.yaml", "sha256": sha256_file(run / "config_snapshot.yaml")},
        "execution_time": None, "motion_during_capture": False,
        "inference_concurrent_with_capture": False, "isaac_scene_load_count": 1,
    }
    validate_successive_observation_metadata(metadata)
    save_json_exclusive(run / "metadata.json", metadata)
    save_json_exclusive(run / "capture_validation.json", {
        "scene_loaded": True, "scene_load_count": 1, "no_robot_model": inventory["no_robot_model"],
        "logical_agent_poses_set": True, "rgb_captured": True, "camera_unchanged": True,
        "scripted_displacement_matches_config": True, "no_simulation_dynamics": True,
        "timeline_time_s": 0.0, "observations": [old_check, fresh_check],
        "source_metadata_sha256": sha256_file(run / "metadata.json"),
    })
    print(f"ROBOTLESS_SUCCESSIVE_CAPTURE_SAVED={run}", flush=True)


def check_artifact(run: Path, record: dict, relative: str) -> None:
    if record.get("path") != relative or sha256_file(run / relative) != record.get("sha256"):
        raise ValueError(f"Saved artifact hash mismatch: {relative}")


def legend_window() -> ui.Window:
    window = ui.Window("Robotless successive OLD / FRESH", width=470, height=205)
    with window.frame:
        with ui.VStack(spacing=6):
            ui.Label("Same session: reset once, seq 0 then seq 1", height=28)
            ui.Label("BLUE = OLD chunk   |   MAGENTA = FRESH chunk", height=24)
            ui.Label("YELLOW = R0   |   ORANGE = R1", height=24)
            ui.Label("GREEN = configured local pose displacement", height=24)
            ui.Label("Small lines show world waypoint headings", height=24)
            ui.Label("Passive raw chunks; no trajectory execution", height=24)
    return window


def visualize(config: dict, run: Path) -> None:
    inference = json.loads((run / "inference_metadata.json").read_text())
    check_artifact(run, inference["raw_inputs"]["capture_metadata"], "metadata.json")
    check_artifact(run, inference["config"], "config_snapshot.yaml")
    check_artifact(run, inference["raw_inputs"]["protocol"], "raw/lightnav_protocol.jsonl")
    metadata = json.loads((run / "metadata.json").read_text())
    validate_successive_observation_metadata(metadata)
    check_artifact(run, metadata["config"], "config_snapshot.yaml")
    if not ARGS.view_only and (run / "visualization_validation.json").exists():
        raise FileExistsError("Visualization evidence already exists; use --view-only")
    chunks = inference["chunks"]
    if len(chunks) != 2:
        raise ValueError("Exactly two saved chunks are required")
    local, world = [], []
    for seq, (chunk, observation) in enumerate(zip(chunks, metadata["observations"], strict=True)):
        if chunk["seq"] != seq or chunk["observation_id"] != observation["observation_id"] or chunk["label"] != observation["label"]:
            raise ValueError("Saved chunk is not bound to its own observation")
        if chunk["agent_pose_world_at_observation"] != observation["agent_pose_world"]:
            raise ValueError("Saved chunk used a different observation pose")
        for key, relative in (("observation", f"raw/observation_{seq:03d}.jpg"),
                              ("request", f"raw/request_{seq:03d}.json"),
                              ("response", f"raw/response_{seq:03d}.json"),
                              ("waypoints", f"raw/chunk_{seq:03d}.npy")):
            check_artifact(run, chunk["raw_inputs"][key], relative)
        check_artifact(run, observation["rgb"], f"raw/observation_{seq:03d}.jpg")
        check_artifact(run, chunk["world_waypoints"], f"derived/chunk_{seq:03d}_world.npy")
        local.append(validate_waypoints(np.load(run / f"raw/chunk_{seq:03d}.npy", allow_pickle=False)))
        world.append(validate_waypoints(np.load(run / f"derived/chunk_{seq:03d}_world.npy", allow_pickle=False)))
    poses = [np.asarray(metadata["R0"], dtype=float), np.asarray(metadata["R1"], dtype=float)]
    recomputed = successive_to_world(*poses, *local)
    for seq in range(2):
        if world[seq].shape != recomputed[seq].shape or not np.allclose(world[seq], recomputed[seq], rtol=0, atol=1e-12):
            raise ValueError(f"Derived chunk {seq} differs from its observation-pose transform")
    _, agent, camera, _, scene = runtime_scene(config, poses[1], app=APP)
    before = actual_pose(agent)
    actual_camera = camera_metadata(camera, agent, config)
    if not np.allclose(actual_camera["T_world_camera"], metadata["observations"][1]["camera"]["T_world_camera"], rtol=0, atol=1e-7):
        raise ValueError("Visualization camera differs from saved R1 observation camera")
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines()
    draw.clear_points()
    visual = config["visualization"]
    z = float(visual["z_offset_m"])
    width = float(visual["line_width"])
    size = float(visual["point_size"])
    draw_polyline(draw, np.asarray(poses), z=z, color=visual["displacement_color_rgba"], width=width)
    for seq, (pose, trajectory) in enumerate(zip(poses, world, strict=True)):
        pose_color = visual[("r0_color_rgba", "r1_color_rgba")[seq]]
        color = visual[("old_color_rgba", "fresh_color_rgba")[seq]]
        draw_pose_points(draw, pose[None, :], z=z + .05, color=pose_color, size=size)
        draw_heading_markers(draw, pose[None, :], z=z + .05, color=pose_color, width=width,
                             length_m=float(visual["observation_heading_length_m"]))
        # Start the visual line at its observation origin without changing raw rows.
        draw_polyline(draw, np.vstack([pose, trajectory]), z=z + .04, color=color, width=width)
        draw_pose_points(draw, trajectory, z=z + .04, color=color, size=size * .65)
        draw_heading_markers(draw, trajectory, z=z + .07, color=color, width=3.0,
                             length_m=float(visual["heading_length_m"]))
    offsets = np.asarray([visual["overview_eye_agent_offset_m"], visual["overview_target_agent_offset_m"]], dtype=float)
    xy = observation_to_world(poses[0], np.c_[offsets[:, :2], np.zeros(2)])
    set_camera_view(eye=[*xy[0, :2], offsets[0, 2]], target=[*xy[1, :2], offsets[1, 2]], camera_prim_path="/OmniverseKit_Persp")
    panel = legend_window()
    screenshot = "evidence/old_fresh_world_trajectories.png"
    if not ARGS.view_only:
        viewport_capture(run / screenshot, app=APP)
    else:
        for _ in range(30):
            APP.update()
    after = actual_pose(agent)
    stamp = stopped_time()
    if not np.array_equal(before, after) or not np.allclose(after, poses[1], rtol=0, atol=1e-7):
        raise RuntimeError("Agent moved during passive OLD/FRESH visualization")
    if not ARGS.view_only:
        save_json_exclusive(run / "visualization_validation.json", {
            "created_time": stamp, "isaac_viewport_rendered": True,
            "no_robot_model": scene["runtime_inventory"]["no_robot_model"],
            "old_fresh_simultaneously_displayed": True,
            "matches_observation_pose_transforms": True, "scripted_displacement_displayed": True,
            "agent_pose_before_visualization": before.tolist(), "agent_pose_after_visualization": after.tolist(),
            "logical_agent_static": True, "timeline_time_s": 0.0,
            "observations": [{"seq": seq, "label": ("OLD", "FRESH")[seq],
                "agent_pose_world_used": poses[seq].tolist(), "world_shape": list(world[seq].shape),
                "camera_basis_checks": metadata["observations"][seq]["camera"]["basis_checks"],
                "waypoint_heading_count": len(world[seq])} for seq in range(2)],
            "legend": {"blue": "OLD world trajectory and waypoint headings", "magenta": "FRESH world trajectory and waypoint headings",
                       "yellow": "R0 observation pose and heading", "orange": "R1 observation pose and heading",
                       "green": "R0 to R1 scripted displacement"},
            "legend_window_title": "Robotless successive OLD / FRESH",
            "screenshot_scope": "Isaac rendered viewport; UI legend window is separate",
            "passive_visualization": "DebugDraw only; no physical bodies; stopped simulation timeline",
            "scene_visibility_modifications": [], "geometry_scaling": 1.0,
            "screenshots": [{"path": screenshot, "sha256": sha256_file(run / screenshot)}],
            "raw_inputs": {p: sha256_file(run / p) for p in ("metadata.json", "raw/chunk_000.npy", "raw/chunk_001.npy")},
            "derived_inputs": [chunk["world_waypoints"] for chunk in chunks],
            "inference_metadata_sha256": sha256_file(run / "inference_metadata.json"),
            "config_sha256": metadata["config"]["sha256"],
            "research_source_sha256": source_hashes(run / "config_snapshot.yaml"),
        })
        print(f"ROBOTLESS_SUCCESSIVE_VISUALIZATION_SAVED={run}", flush=True)
    else:
        print("ROBOTLESS_SUCCESSIVE_VIEW_ONLY=hash-checked inputs; no artifacts written", flush=True)
    if not ARGS.no_hold and not ARGS.headless:
        while APP.is_running() and panel.visible:
            APP.update()


def main() -> None:
    if ARGS.view_only and ARGS.mode != "visualize":
        raise ValueError("--view-only requires visualize mode")
    run = ARGS.run_directory.resolve()
    config_path = ARGS.config.resolve() if ARGS.mode == "capture" else run / "config_snapshot.yaml"
    config = load_config(config_path)
    if ARGS.mode == "capture":
        capture(config, run, config_path)
    else:
        visualize(config, run)


try:
    main()
finally:
    APP.close()
