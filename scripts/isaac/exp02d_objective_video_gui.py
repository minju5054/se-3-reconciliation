#!/usr/bin/env python3
"""Record real Hospital physics for matched historical/Exp02D objective videos."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts")]
from summarize_exp02d_physical_execution import validate_run
from reconciliation.exp02d_gui import load_exp02d_gui_case, compute_exp02d_camera_plan
from reconciliation.online_switch import sha256_file
from reconciliation.physics_display import render_paused
from reconciliation.video_timing import validate_video_frames
from reconciliation.closed_loop_execution_validation import (
    compute_closed_loop_metrics, save_closed_loop_trial, validate_closed_loop_trial,
)
from exp02d_physical_execution import resolve, utc, write_json

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--cases", nargs="+", help="case labels from the supplied physical protocol")
parser.add_argument("--methods", nargs="+", choices=["M0_RAW", "M1_HISTORICAL_M4", "M3_LOOKAHEAD"],
                    default=["M1_HISTORICAL_M4", "M3_LOOKAHEAD"])
parser.add_argument("--stride", type=int, default=3, help="one actual frame per N physics steps")
args = parser.parse_args()
if args.stride not in (1, 2, 3, 6):
    raise ValueError("stride must divide the existing six-step control interval")
run = args.run.resolve()
protocol, summary = validate_run(run)
config = protocol["config"]
for name, digest in protocol["code_sha256"].items():
    if sha256_file(ROOT / name) != digest:
        raise ValueError(f"primary execution code changed: {name}")
if "transition_ids" in config:
    from reconciliation.exp02d_turning_search import FrozenArchive
    archive = FrozenArchive(protocol["source_run"])
    cases = {case: archive.load(config["transition_ids"][case], case) for case in config["cases"]}
else:
    cases = {case: load_exp02d_gui_case(protocol["source_run"], case) for case in config["cases"]}
args.cases = args.cases or config["cases"]
if not args.cases or any(case not in cases for case in args.cases):
    raise ValueError("requested video case is not in the physical protocol")
for case, evidence in cases.items():
    expected = protocol["input_hashes"][case]
    if dict(evidence.source_sha256) != expected["source"] or dict(evidence.result_sha256) != expected["result"]:
        raise ValueError("source/candidate hash differs from matched primary")

from isaacsim import SimulationApp
APP = SimulationApp({"headless": False, "width": 1700, "height": 1000})
import numpy as np
import yaml
import omni.ui as ui
import omni.usd
from pxr import Gf, UsdGeom, UsdLux
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import get_active_viewport
from exp02d_execution_runtime import HospitalExecutionRuntime, loop_configs
from closed_loop_execution_runtime import run_closed_loop, se2_from_world_pose, canonical_wheel_values
from debug_draw_trajectories import draw_polyline
from x11_window_capture import IsaacWindowCapture


class RecordingRuntime(HospitalExecutionRuntime):
    """Only add render/read operations around the unchanged physical loop."""
    frame_callback = None
    recording = False

    def reset(self, pose, settling_s):
        self.recording = False
        super().reset(pose, settling_s)
        self.recording_origin = float(self.world.current_time)
        self.recording_index = 0
        self.current_desired = np.zeros(2)
        self.recording = True
        self.frame_callback(self)

    def apply(self, desired, mode, correction, measured_omega):
        self.current_desired = desired.copy()
        return super().apply(desired, mode, correction, measured_omega)

    def step(self):
        super().step()
        if self.recording:
            self.recording_index += 1
            if self.recording_index % args.stride == 0:
                self.frame_callback(self)


def camera_for(runtime, evidence, directory):
    source = resolve(config["gui"]["camera_config"])
    gui = yaml.safe_load(source.read_text())["gui"]
    view = config["gui"].get("camera_case_view_sectors", {}).get(evidence.case)
    if view is None:
        view = gui["camera_case_view_sectors"].get(evidence.case, {
            "preferred_eye_direction_world_xy": gui["camera_preferred_eye_direction_world_xy"],
            "preferred_eye_half_angle_deg": gui["camera_preferred_eye_half_angle_deg"]})
    actual = [np.load(run / "trials" / evidence.case / method /
                     "repetition_00/raw/actual_trajectory.npy", allow_pickle=False) for method in config["methods"]]
    references = [evidence.method_candidates[method] for method in config["methods"]]
    z = float(config["gui"]["line_height_m"])
    plan = compute_exp02d_camera_plan(evidence.active_old.old_world, np.vstack(actual), references,
        viewport_aspect_ratio=1.60, old_path_z_m=z, actual_path_visual_z_m=z,
        candidate_path_z_m=[z] * len(references),
        observation_pose_world_se2=evidence.active_old.boundary_pose_world_se2,
        observation_marker_visual_z_m=z, preferred_eye_direction_xy=view["preferred_eye_direction_world_xy"],
        preferred_eye_half_angle_deg=view["preferred_eye_half_angle_deg"])
    stage = omni.usd.get_context().get_stage()
    path = "/World/Exp02DObjectiveVideoCamera"
    camera = UsdGeom.Camera.Define(stage, path)
    transform = UsdGeom.Xformable(camera.GetPrim())
    if not transform.GetOrderedXformOps():
        transform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        transform.AddOrientOp().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
    aperture = 20.955
    camera.CreateHorizontalApertureAttr().Set(aperture)
    camera.CreateVerticalApertureAttr().Set(aperture / 1.60)
    camera.CreateFocalLengthAttr().Set(aperture / (2 * math.tan(math.radians(plan.horizontal_fov_deg) / 2)))
    viewport = get_active_viewport()
    viewport.camera_path = camera.GetPath()
    viewport.set_texture_resolution((1440, 900))
    set_camera_view(eye=list(plan.eye_xyz), target=list(plan.target_xyz), camera_prim_path=path, viewport_api=viewport)
    light = UsdLux.SphereLight.Define(stage, "/World/Exp02DObjectiveVideoFill")
    light.CreateIntensityAttr(18000.)
    light.CreateRadiusAttr(.3)
    UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(Gf.Vec3d(plan.target_xyz[0], plan.target_xyz[1], 1.6))
    write_json(directory / "camera.json", {"plan": asdict(plan), "camera_config_sha256": sha256_file(source),
               "coordinate_transform": "identity world XY metres; visual line Z only", "line_height_m": z,
               "identical_camera_for_all_methods_in_case": True})


def main():
    output = run / "video_gui" / utc().replace(":", "").replace("+", "_")
    output.mkdir(parents=True, exist_ok=False)
    source = protocol["source_system"]
    runtime = RecordingRuntime(APP, source, protocol["candidate"],
        np.asarray(protocol.get("scene_initialization_pose_world_se2",
            cases[config["cases"][0]].active_old.boundary_pose_world_se2)), render=True)
    runtime.world.pause()
    loop, follower = loop_configs(source, config["maximum_duration_s"])
    draw = _debug_draw.acquire_debug_draw_interface()
    panel = ui.Window("EXP02D matched objective recording", width=530, height=850)
    with panel.frame:
        with ui.VStack(spacing=7, style={"margin": 12}):
            ui.Label("SAME EPISODE / SAME CONTROLLER", height=35, style={"font_size": 19})
            case_label = ui.Label("", height=48, word_wrap=True, style={"font_size": 17})
            method_label = ui.Label("", height=55, word_wrap=True, style={"font_size": 22})
            primary_label = ui.Label("", height=120, word_wrap=True, style={"font_size": 16})
            live_label = ui.Label("", height=150, word_wrap=True, style={"font_size": 17})
            ui.Label("ORANGE: historical objective path\nMAGENTA: Exp02D path\nGREEN: live measured motion\nGRAY: raw FRESH / YELLOW: start B",
                     height=100, word_wrap=True, style={"font_size": 16})
            ui.Label("Real physics, recorded at sampled steps.\nNo saved-pose playback or interpolation.\nVideo playback is 2x slow motion.",
                     height=75, word_wrap=True)
            ui.Label("Reset at saved B. Prior velocity and PI reset.\nPASS/FAIL: tracking gates, not navigation.\nFinal frame is held after each run ends.",
                     height=75, word_wrap=True)
    for _ in range(3):
        APP.update()
    stage_window = ui.Workspace.get_window("Stage")
    if stage_window:
        panel.dock_in(stage_window, ui.DockPosition.SAME)
    for name in ("Property", "Content", "Console", "Stage", "Render Settings"):
        window = ui.Workspace.get_window(name)
        if window:
            window.visible = False
    panel.focus()
    window_capture = IsaacWindowCapture()
    scripts = [Path(__file__), ROOT / "scripts/isaac/x11_window_capture.py",
               ROOT / "src/reconciliation/video_timing.py", ROOT / "src/reconciliation/physics_display.py"]
    write_json(output / "provenance.json", {"primary_protocol_sha256": sha256_file(run / "protocol.json"),
        "primary_summary_sha256": sha256_file(run / "summary.json"),
        "code_sha256": {str(p.relative_to(ROOT)): sha256_file(p) for p in scripts},
        "physics_dt": runtime.physics_dt, "control_dt": runtime.control_dt, "capture_stride": args.stride,
        "capture": "unaltered XComposite Isaac client PNGs; physics paused during render/read and clock checked",
        "physics_execution": True, "saved_pose_replay": False, "selection": vars(args) | {"run": str(run)},
        "coordinate_frame": protocol["coordinate_frame"], "timing": protocol["timing"]})
    results = []
    names = {"M0_RAW": "RAW FRESH", "M1_HISTORICAL_M4": "BEFORE: historical objective", "M3_LOOKAHEAD": "AFTER: Exp02D objective"}
    colors = {"M0_RAW": (.65, .65, .65, 1), "M1_HISTORICAL_M4": (1, .5, .05, 1), "M3_LOOKAHEAD": (.9, .1, .85, 1)}
    for case in args.cases:
        evidence = cases[case]
        case_dir = output / case
        case_dir.mkdir()
        camera_for(runtime, evidence, case_dir)
        for method in args.methods:
            directory = case_dir / method
            frames_dir = directory / "raw_frames"
            frames_dir.mkdir(parents=True)
            frames, trail = [], []
            primary = summary["outcomes"][case][method]
            case_label.text = f"{case}: {evidence.corpus_transition_id}"
            method_label.text = names[method]
            primary_label.text = (f"Primary tracking: {primary['physical_tracking_status']}\n"
                f"Goal: {primary['goal_successes']}/3\n"
                f"Yaw RMS: {primary['repeatability']['yaw_rmse_rad']['mean']:.4f} rad\n"
                f"Yaw RMS gate: < {protocol['acceptance_criteria']['yaw_rmse_rad']:.4f} rad\n"
                f"Failed gates: {', '.join(primary['failed_checks']) or 'none'}")
            def frame(rt):
                t = float(rt.world.current_time) - rt.recording_origin
                pose = se2_from_world_pose(rt.robot)
                wheels = canonical_wheel_values(rt.robot.get_joint_velocities(), rt.wheels)
                trail.append(pose.copy())
                draw.clear_lines()
                draw.clear_points()
                z = float(config["gui"]["line_height_m"])
                for ref_method in config["methods"]:
                    draw_polyline(draw, evidence.method_candidates[ref_method], z=z,
                        color=colors[ref_method], width=6 if ref_method == method else 2)
                if len(trail) > 1:
                    draw_polyline(draw, np.asarray(trail), z=z, color=(.05, 1, .18, 1), width=5)
                b = evidence.active_old.boundary_pose_world_se2
                draw.draw_points([(float(b[0]), float(b[1]), z)], [(1., .85, .1, 1)], [16.])
                live_label.text = (f"LIVE PHYSICS | t = {t:.2f} s\n"
                    f"Desired v: {rt.current_desired[0]:.3f} m/s\nDesired yaw rate: {rt.current_desired[1]:.3f} rad/s\n"
                    f"Wheel speeds: {wheels[0]:.1f}, {wheels[1]:.1f},\n                         {wheels[2]:.1f}, {wheels[3]:.1f} rad/s")
                path = frames_dir / f"{len(frames):06d}.png"
                render_paused(rt.world, 4)
                size = window_capture.capture(path)
                if float(rt.world.current_time) - rt.recording_origin != t:
                    raise RuntimeError("window recording changed physics time")
                frames.append({"physics_index": rt.recording_index, "sim_time_s": t,
                    "pose_world_se2": pose.tolist(), "wheel_velocities_rad_s": wheels.tolist(),
                    "file": str(path.relative_to(directory)), "sha256": sha256_file(path),
                    "size": size, "captured_utc": utc()})
                rt.world.play()
            runtime.frame_callback = frame
            started = utc()
            print(f"OBJECTIVE_VIDEO_START={case} {method}", flush=True)
            telemetry = run_closed_loop(runtime, loop, follower, evidence.method_candidates[method],
                evidence.active_old.boundary_pose_world_se2, "calibrated")
            runtime.recording = False
            runtime.world.pause()
            metrics = compute_closed_loop_metrics(telemetry)
            trial = directory / "trial"
            save_closed_loop_trial(trial, telemetry, metrics, {
                "evaluation_semantics": "reference-matched closed-loop evaluation", "mode": "calibrated",
                "scenario": f"{case}_{method}", "repetition": 0, "case": case, "method": method,
                "quantitative_primary": False, "started_utc": started, "completed_utc": utc(),
                "provenance_path": str(output / "provenance.json"), "provenance_sha256": sha256_file(output / "provenance.json"),
                "source_observation_sim_time_s": evidence.active_old.observation_sim_time_s,
                "source_readiness_sim_time_s": evidence.active_old.model_ready_sim_time_s,
                "source_switch_sim_time_s": evidence.active_old.switch_sim_time_s,
                "requested_initial_pose_world_se2": evidence.active_old.boundary_pose_world_se2.tolist(),
                "settled_initial_pose_world_se2": telemetry.actual_trajectory[0].tolist(),
                "coordinate_frame": protocol["coordinate_frame"], "timing": protocol["timing"]})
            validate_closed_loop_trial(trial)
            timing = validate_video_frames(frames, runtime.physics_dt, args.stride,
                                            telemetry.sim_times_s, telemetry.actual_trajectory)
            primary_poses = np.load(run / "trials" / case / method / "repetition_00/raw/actual_trajectory.npy", allow_pickle=False)
            equal = bool(np.array_equal(primary_poses, telemetry.actual_trajectory))
            live_label.text = (f"RUN ENDED | t = {metrics['time_to_goal_or_timeout_s']:.2f} s\n"
                f"Goal reached: {metrics['goal_reached']}\nPath RMS: {metrics['position_rmse_m']*100:.2f} cm\n"
                f"Yaw RMS: {metrics['yaw_rmse_rad']:.3f} rad\nMatches primary motion: {equal}")
            render_paused(runtime.world, 4)
            window_capture.capture(directory / "end_card.png")
            result = {"case": case, "method": method, "metrics": metrics, "timing": timing,
                "frames": frames, "primary_poses_exactly_equal": equal,
                "end_card_sha256": sha256_file(directory / "end_card.png"), "directory": str(directory)}
            write_json(directory / "recording.json", result)
            results.append({key: value for key, value in result.items() if key != "frames"})
            print(f"OBJECTIVE_VIDEO_COMPLETE={case} {method} frames={len(frames)} primary_equal={equal}", flush=True)
    window_capture.close()
    write_json(output / "completed.json", {"results": results, "completed_utc": utc()})
    print(f"OBJECTIVE_VIDEO_READY={output}", flush=True)


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    print("OBJECTIVE_VIDEO_FAILED", flush=True)
    raise
finally:
    APP.close()
