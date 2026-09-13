#!/usr/bin/env python3
"""Present the archived EXP-02B OLD execution mismatch, with replay controls."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from reconciliation.exp02b_failure_demo import checked_file, load_failure_evidence, saved_sample_index
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.online_switch import sha256_file

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, default=ROOT / "configs/exp02b_failure_demo.yaml")
parser.add_argument("--duration", type=float, help="wall seconds for the archived OLD interval")
parser.add_argument("--no-hold", action="store_true", help="close after one replay and captures")
ARGS = parser.parse_args()
EVIDENCE = load_failure_evidence(ROOT, ARGS.config)
VIEW = EVIDENCE.config["presentation"]
DURATION = ARGS.duration if ARGS.duration is not None else float(VIEW["duration_wall_s"])
saved_sample_index(EVIDENCE.telemetry.sim_times_s, 0, DURATION)
OUTPUT = ROOT / VIEW["output_root"] / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
OUTPUT.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp
APP = SimulationApp({"headless": False, "hide_ui": False, "width": 1800, "height": 1100})

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom, UsdLux
from debug_draw_trajectories import draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import suppress_sensor_viewport_visualization

BLUE = (0.10, 0.45, 1.0, 1.0)
CYAN = (0.0, 0.95, 1.0, 1.0)
RED = (1.0, 0.15, 0.12, 1.0)


def plot_evidence() -> Path:
    """Publish exact recorded measurements; no smoothing or time-aligned waypoint error."""
    t = EVIDENCE.telemetry
    spatial = nearest_polyline_samples(EVIDENCE.old_reference, t.actual_trajectory)
    fig, axes = plt.subplots(2, 1, figsize=(7.5, 5.3), layout="constrained")
    axes[0].step(t.sim_times_s[1:], t.commanded_body[1:, 1], where="pre",
                 color="#db7415", label="Commanded omega")
    axes[0].plot(t.sim_times_s[1:], EVIDENCE.measured_body[1:, 1], color="#009db5",
                 label="Measured omega")
    axes[0].set_ylabel("Angular velocity [rad/s]")
    axes[0].legend(loc="upper left", fontsize=9)
    axes[1].plot(t.sim_times_s, spatial["distance_m"] * 100, color="#008caa")
    rms = EVIDENCE.summary["old_reference_spatial_metrics"]["old_reference_distance_rms_m"] * 100
    axes[1].axhline(rms, color="#db7415", linestyle="--", label=f"Full OLD RMS {rms:.3f} cm")
    axes[1].set_ylabel("Nearest OLD distance [cm]")
    axes[1].set_xlabel("Recorded OLD elapsed simulation time [s]")
    axes[1].legend(loc="upper right", fontsize=9)
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.set_xlim(0, t.sim_times_s[-1])
    fig.suptitle("Archived 2026-09-07 measurements | full OLD interval", fontsize=12)
    path = OUTPUT / "old_evidence.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def main() -> None:
    telemetry = EVIDENCE.telemetry
    poses = telemetry.actual_trajectory
    summary = EVIDENCE.summary
    plot_path = plot_evidence()
    world = World(physics_dt=1/60, rendering_dt=1/60, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DomeLight.Define(stage, "/World/FailureDemoLight").CreateIntensityAttr(1200.0)
    source_config = yaml.safe_load(Path(summary["source_provenance"]["exp01b_config_path"]).read_text())
    for box in source_config["scene"]["static_boxes"]:
        cube = UsdGeom.Cube.Define(stage, f"/World/{box['id']}")
        cube.CreateSizeAttr(1.0)
        transform = UsdGeom.XformCommonAPI(cube.GetPrim())
        transform.SetTranslate(Gf.Vec3d(*box["center"]))
        transform.SetScale(Gf.Vec3f(*box["size"]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*box["color"])])
    display_root = UsdGeom.Xform.Define(stage, "/World/ArchivedJackalPose")
    robot_reference = "/World/ArchivedJackalPose/Jackal"
    add_reference_to_stage(EVIDENCE.metadata["robot_asset"]["resolved_path"], robot_reference)
    while is_stage_loading():
        APP.update()
    # Initialize render resources once. No articulation/controller/physics stepping in playback.
    world.reset()
    world.pause()
    frozen_sim_time = float(world.current_time)
    suppress_sensor_viewport_visualization(robot_reference)
    robot_transform = UsdGeom.XformCommonAPI(display_root.GetPrim())
    set_camera_view(eye=VIEW["camera_eye_world_m"], target=VIEW["camera_target_world_m"],
                    camera_prim_path="/OmniverseKit_Persp")
    draw = _debug_draw.acquire_debug_draw_interface()
    spatial = nearest_polyline_samples(EVIDENCE.old_reference, poses)
    state = {"elapsed": 0.0, "paused": False, "restart": False, "index": 0,
             "intro": float(VIEW["intro_hold_wall_s"])}

    def restart():
        state.update(elapsed=0.0, paused=False, restart=True, index=0, intro=1.0)
        print("EXP02B_FAILURE_CONTROL=restart", flush=True)

    def pause():
        state["paused"] = not state["paused"]
        print(f"EXP02B_FAILURE_CONTROL=paused:{state['paused']} saved_index:{state['index']}", flush=True)

    body = summary["old_body_command_vs_measured_metrics"]
    panel = ui.Window("EXP-02B | OLD execution mismatch", width=550, height=980,
                      position_x=15, position_y=45)
    panel.visible = True
    with panel.frame:
        with ui.VStack(spacing=4):
            ui.Label("EXP-02B: OLD execution mismatch", height=35, style={"font_size": 25})
            ui.Label("Archived measured replay | 2026-09-07 | no new physics", height=22)
            ui.Label("case_high_delta_omega  /  k=3  /  raw_k", height=22)
            phase_label = ui.Label("READY", height=34, style={"font_size": 23})
            time_label = ui.Label("", height=25)
            with ui.HStack(height=38, spacing=8):
                ui.Button("Restart OLD replay", clicked_fn=restart)
                ui.Button("Pause / Resume", clicked_fn=pause)
            ui.Label("FULL RECORDED OLD INTERVAL", height=25, style={"font_size": 18})
            rms = summary["old_reference_spatial_metrics"]["old_reference_distance_rms_m"]
            ui.Label(f"Spatial deviation RMS: {100*rms:.3f} cm", height=36, style={"font_size": 25})
            ui.Label(f"Mean commanded omega:  {body['commanded_omega_mean_rps']:.5f} rad/s", height=26)
            ui.Label(f"Mean measured omega:     {body['measured_omega_mean_rps']:.5f} rad/s", height=26)
            ui.Image(str(plot_path), height=285)
            current_label = ui.Label("", height=46, word_wrap=True)
            distance_label = ui.Label("", height=25)
            ui.Label("BLUE = planned OLD | CYAN = measured OLD", height=24)
            ui.Label("RED = saved B | CYAN point = reproduced B", height=24)
            ui.Label("B reproduction error: 0 m / 0 rad (saved result)", height=24)
            ui.Label("Stops BEFORE exact reset. No post-switch history.", height=24)
            ui.Label("Both lines at visual Z=0.72 m; saved world XY unchanged.", height=22)
            ui.Label("Spatial distance, not time-aligned waypoint error.", height=22)
            ui.Label("Observed mismatch; the physical cause is not established.", height=22)

    # Dock explicitly so a previously saved editor layout cannot hide the floating panel.
    for _ in range(3):
        APP.update()
    stage_window = ui.Workspace.get_window("Stage")
    if stage_window is not None:
        panel.dock_in(stage_window, ui.DockPosition.SAME)
    for name in ("Property", "Content", "Console", "Stage", "Render Settings"):
        editor_window = ui.Workspace.get_window(name)
        if editor_window is not None:
            editor_window.visible = False
    panel.focus()
    print(f"EXP02B_FAILURE_PANEL=visible:{panel.visible} size:{panel.width}x{panel.height}", flush=True)

    def display(index: int):
        if world.is_playing():
            world.pause()
        pose = poses[index]
        robot_transform.SetTranslate(Gf.Vec3d(float(pose[0]), float(pose[1]), float(VIEW["robot_display_z_m"])))
        robot_transform.SetRotate(Gf.Vec3f(0, 0, math.degrees(float(pose[2]))))
        draw.clear_lines()
        draw.clear_points()
        z = float(VIEW["path_visual_z_m"])
        # Both curves share exactly the same visual Z; no apparent gap from height offsets.
        draw_polyline(draw, EVIDENCE.old_reference, z=z, color=BLUE, width=5)
        if index:
            draw_polyline(draw, poses[:index+1], z=z, color=CYAN, width=6)
        draw_pose_points(draw, np.asarray([summary["saved_boundary_se2"]]), z=z, color=RED, size=20)
        if index == len(poses)-1:
            draw_pose_points(draw, np.asarray([summary["reproduced_boundary_se2"]]), z=z, color=CYAN, size=10)
        # Nearest spatial segment, not a correspondence to a timed waypoint.
        nearest = spatial["nearest_xy"][index]
        draw.draw_lines([(float(pose[0]), float(pose[1]), z)],
                        [(float(nearest[0]), float(nearest[1]), z)], [(1.0, 0.8, 0.15, 1.0)], [3])
        ended = index == len(poses)-1
        phase_label.text = "PRE-RESET INSPECTION" if ended else ("PAUSED" if state["paused"] else "OLD REPLAY")
        origin = EVIDENCE.metadata["diagnostic_execution_timestamps"]["phase_sim_times_s"]["OLD_REPLAY"]
        time_label.text = f"Saved OLD {telemetry.sim_times_s[index]:.3f} / {telemetry.sim_times_s[-1]:.3f} s | sim {origin+telemetry.sim_times_s[index]:.3f} s"
        current_label.text = ("Initial pre-interval sample; velocity not yet measured" if index == 0 else
            f"This ending interval: omega command {telemetry.commanded_body[index,1]:.3f} / measured {EVIDENCE.measured_body[index,1]:.3f} rad/s")
        distance_label.text = f"Current nearest OLD distance: {100*spatial['distance_m'][index]:.2f} cm"
        world.render()
        # Pump native UI callbacks while the simulation timeline remains paused.
        APP.update()
        if float(world.current_time) != frozen_sim_time:
            raise RuntimeError("presentation must not advance the physics clock")

    captures = []
    def capture(name, index):
        for _ in range(5):
            display(index)
        path = OUTPUT / f"{name}.png"
        capture_viewport_to_file(get_active_viewport(), file_path=str(path))
        deadline = time.monotonic() + 15
        while not path.exists() and time.monotonic() < deadline:
            display(index)
        if not path.exists():
            raise RuntimeError("viewport capture did not complete")
        captures.append({"file": path.name, "sha256": sha256_file(path), "sample_index": index,
                         "saved_old_elapsed_s": float(telemetry.sim_times_s[index]), "pose_world_se2": poses[index].tolist()})

    display(0)
    for _ in range(40):
        display(0)
    capture("01_old_start", 0)
    print(f"EXP02B_FAILURE_DEMO_READY={OUTPUT}", flush=True)
    last = time.monotonic()
    milestones = set()
    manifest_written = False
    while APP.is_running():
        now = time.monotonic()
        dt = now-last
        last = now
        if state["restart"]:
            state["restart"] = False
        elif state["intro"] > 0:
            state["intro"] -= dt
        elif not state["paused"]:
            state["elapsed"] = min(DURATION, state["elapsed"]+dt)
        index = saved_sample_index(telemetry.sim_times_s, state["elapsed"], DURATION)
        state["index"] = index
        display(index)
        if index >= len(poses)//2 and "mid" not in milestones:
            capture("02_old_motion", index)
            milestones.add("mid")
            last = time.monotonic()
        if index == len(poses)-1 and not manifest_written:
            capture("03_before_reset", index)
            for source_path, digest in EVIDENCE.hashes.items():
                checked_file(Path(source_path), digest)
            files = [Path(__file__), ROOT/"src/reconciliation/exp02b_failure_demo.py",
                     ROOT/"scripts/isaac/run_exp02b_failure_demo.sh", ROOT/"scripts/isaac/debug_draw_trajectories.py"]
            manifest = {
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "source_run": EVIDENCE.config["source_run"], "source_sha256": EVIDENCE.hashes,
                "code_sha256": {str(p):sha256_file(p) for p in files},
                "presentation": {**VIEW, "duration_wall_s":DURATION},
                "coordinate_frame": EVIDENCE.metadata["coordinate_frame"],
                "source_observation_readiness_execution_timestamps": EVIDENCE.metadata["source_observation_readiness_execution_timestamps"],
                "diagnostic_execution_timestamps": EVIDENCE.metadata["diagnostic_execution_timestamps"],
                "sampling": "last saved sample at/before mapped elapsed time; no pose interpolation",
                "telemetry_timing": EVIDENCE.metadata["telemetry_timing"],
                "saved_actual_replay":True, "physics_reexecuted":False, "controller_executed":False,
                "xy_transform":"identity", "xy_scaling":False, "post_switch_actual_displayed":False,
                "reset_executed_in_presentation":False, "last_sample":"B_reproduced before exact reset",
                "sample_count":len(poses), "summary_recomputed_and_verified":True,
                "presentation_physics_clock_held_s":frozen_sim_time,
                "old_reference_spatial_metrics":summary["old_reference_spatial_metrics"],
                "old_body_command_vs_measured_metrics":body,
                "captures":captures, "chart_sha256":sha256_file(plot_path),
            }
            with (OUTPUT/"capture_manifest.json").open("x") as stream:
                json.dump(manifest,stream,indent=2,allow_nan=False)
                stream.write("\n")
            print(f"EXP02B_FAILURE_DEMO_COMPLETE={OUTPUT}", flush=True)
            manifest_written = True
            if ARGS.no_hold:
                break
        time.sleep(0.01)
    panel.visible = False


try:
    main()
finally:
    APP.close()
