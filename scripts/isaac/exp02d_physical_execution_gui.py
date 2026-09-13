#!/usr/bin/env python3
"""Run frozen EXP-02D RAW/M3 paths with live Jackal physics and measured trails."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from summarize_exp02d_physical_execution import validate_run
from reconciliation.exp02d_gui import load_exp02d_gui_case, compute_exp02d_camera_plan, validate_gui_capture_rgb
from reconciliation.closed_loop_execution_validation import (
    compute_closed_loop_metrics, save_closed_loop_trial, validate_closed_loop_trial,
)
from reconciliation.online_switch import sha256_file
from reconciliation.physics_display import render_paused
from exp02d_physical_execution import resolve, utc, write_json

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--case", choices=["success", "failure", "S1", "S2", "F1"], default="success")
parser.add_argument("--both", action="store_true", help="run selected physical success then failure")
parser.add_argument("--all-cases", action="store_true", help="run S1, S2 and F1 in protocol order")
parser.add_argument("--case-hold-seconds", type=float, default=0.0, help="paused display between automatic cases")
parser.add_argument("--no-hold", action="store_true")
parser.add_argument("--real-time-factor", type=float)
parser.add_argument("--compare-raw", action="store_true", help="also physically run RAW before M3")
parser.add_argument("--line-height", type=float, default=0.035, help="visual-only world Z in metres; floor overlay by default")
args = parser.parse_args()
if not math.isfinite(args.line_height):
    raise ValueError("line height must be finite")
if not math.isfinite(args.case_hold_seconds) or args.case_hold_seconds < 0:
    raise ValueError("case hold duration must be finite and nonnegative")
run = args.run.resolve()
protocol, summary = validate_run(run)
config = protocol["config"]
plot_manifest = json.loads((run / "gui_plots/manifest.json").read_text())
if plot_manifest["source_summary_sha256"] != sha256_file(run / "summary.json"):
    raise ValueError("GUI plots do not identify the primary physical result")
for name, digest in plot_manifest["plots"].items():
    if sha256_file(run / "gui_plots" / name) != digest:
        raise ValueError("GUI plot hash mismatch")
for name, digest in protocol["code_sha256"].items():
    if sha256_file(ROOT / name) != digest:
        raise ValueError(f"primary execution code changed: {name}")
cases = {case: load_exp02d_gui_case(protocol["source_run"], case) for case in config["cases"]}
for case, evidence in cases.items():
    expected = protocol["input_hashes"][case]
    if dict(evidence.source_sha256) != expected["source"] or dict(evidence.result_sha256) != expected["result"]:
        raise ValueError("saved source differs from physical primary")
selection = summary["gui_selection"]
initial_case = selection.get(args.case, args.case)
if initial_case not in cases:
    raise ValueError(f"physical {args.case} example is unavailable under the frozen gates")

from isaacsim import SimulationApp
APP = SimulationApp({"headless": False, "width": 1700, "height": 1000})
import numpy as np
import yaml
import omni.ui as ui
import omni.usd
from pxr import Gf, UsdGeom, UsdLux
from PIL import Image
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from exp02d_execution_runtime import HospitalExecutionRuntime, loop_configs
from closed_loop_execution_runtime import run_closed_loop
from debug_draw_trajectories import draw_polyline


def capture(path, runtime):
    """Capture actual current physics state while paused; do not teleport or step."""
    before = float(runtime.world.current_time)
    render_paused(runtime.world, 4)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active viewport is unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    started = time.monotonic()
    while (not path.exists() or time.monotonic() - started < .25) and time.monotonic() - started < 20:
        render_paused(runtime.world)
        time.sleep(.02)
    if float(runtime.world.current_time) != before:
        raise RuntimeError("capture advanced the physics clock")
    if not path.exists():
        raise RuntimeError("viewport capture did not complete")
    with Image.open(path) as im:
        pixels = np.asarray(im.convert("RGB"))
    content = validate_gui_capture_rgb(pixels, require_jackal=True)
    return {"sha256": sha256_file(path), "sim_time_s": before, "visual_content": content,
            "capture_does_not_step_physics": True}


def configure_camera(runtime, evidence, output):
    gui_path = resolve(config["gui"]["camera_config"])
    gui = yaml.safe_load(gui_path.read_text())["gui"]
    sector = gui["camera_case_view_sectors"][evidence.case]
    primary_actual = [np.load(run / "trials" / evidence.case / method /
                         "repetition_00/raw/actual_trajectory.npy", allow_pickle=False)
                      for method in config["methods"]]
    z = args.line_height
    plan = compute_exp02d_camera_plan(
        evidence.active_old.old_world, np.vstack(primary_actual),
        [evidence.method_candidates[method] for method in config["methods"]],
        viewport_aspect_ratio=1.60, old_path_z_m=z, actual_path_visual_z_m=z,
        candidate_path_z_m=[z, z], observation_pose_world_se2=evidence.active_old.boundary_pose_world_se2,
        observation_marker_visual_z_m=z, preferred_eye_direction_xy=sector["preferred_eye_direction_world_xy"],
        preferred_eye_half_angle_deg=sector["preferred_eye_half_angle_deg"],
    )
    stage = omni.usd.get_context().get_stage()
    path = "/World/Exp02DPhysicalCamera"
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
    light = UsdLux.SphereLight.Define(stage, "/World/Exp02DPhysicalFill")
    light.CreateIntensityAttr(18000.0)
    light.CreateRadiusAttr(.3)
    UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(Gf.Vec3d(plan.target_xyz[0], plan.target_xyz[1], 1.6))
    write_json(output / "camera.json", {"plan": asdict(plan), "gui_source_sha256": sha256_file(gui_path),
                "line_height_m": z, "light_only_no_physics_change": True,
                "coordinate_transform": "identity XY; line display Z only raised; no scaling"})


def main():
    output = run / "gui" / utc().replace(":", "").replace("+", "_")
    output.mkdir(parents=True, exist_ok=False)
    source = protocol["source_system"]
    speed = args.real_time_factor or float(config["gui"]["real_time_factor"])
    runtime = HospitalExecutionRuntime(APP, source, protocol["candidate"],
                  cases[config["cases"][0]].active_old.boundary_pose_world_se2, render=True, real_time_factor=speed)
    loop, follower = loop_configs(source, config["maximum_duration_s"])
    runtime.world.pause()
    draw = _debug_draw.acquire_debug_draw_interface()
    state = {"pending": None, "last_case": initial_case}
    panel = ui.Window("EXP02D physical path execution", width=500, height=900)
    def request(case):
        state["pending"] = case
    with panel.frame:
        with ui.VStack(spacing=3, style={"margin": 8}):
            ui.Label("EXP02D: JACKAL PHYSICS", height=28, style={"font_size": 20})
            case_label = ui.Label("Preparing", height=32, word_wrap=True, style={"font_size": 16})
            status_label = ui.Label("", height=28, word_wrap=True, style={"font_size": 16})
            for label, case in selection.items():
                if case is not None:
                    ui.Button(f"Run {label.upper()} case ({case})", height=24, clicked_fn=lambda c=case: request(c))
            for case in cases:
                if case not in selection.values():
                    ui.Button(f"Run TIMEOUT case ({case})", height=24, clicked_fn=lambda c=case: request(c))
            phase_label = ui.Label("", height=26, word_wrap=True, style={"font_size": 16})
            live_label = ui.Label("", height=56, word_wrap=True)
            outcome_label = ui.Label("", height=34, word_wrap=True)
            primary_label = ui.Label("", height=78, word_wrap=True)
            ui.Label("MAGENTA: M3 path | GREEN: new actual\nGRAY: RAW path | YELLOW: saved switch B\nFloor overlay; unchanged world XY.", height=46, word_wrap=True)
            xy_image = ui.Image(str(run / "gui_plots" / f"{initial_case}_xy.png"), height=180)
            ui.Label("Inset: primary measured path, not live telemetry.", height=22, word_wrap=True)
            ui.Label("Current controller / Hospital\nReset at B; prior velocity and PI history reset", height=30, word_wrap=True)
            ui.Label("Buttons start new physical runs.\nPASS/FAIL means tracking, not navigation success.", height=30, word_wrap=True)
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
    processing = {"primary_summary_sha256": sha256_file(run / "summary.json"),
                  "primary_protocol_sha256": sha256_file(run / "protocol.json"),
                  "runner_sha256": sha256_file(Path(__file__)), "real_time_factor": speed,
                  "effective_line_height_m": args.line_height, "compare_raw": args.compare_raw,
                  "gui_plot_manifest_sha256": sha256_file(run / "gui_plots/manifest.json"),
                  "scene_initialization_case_matches_primary": config["cases"][0],
                  "between_case_hold_wall_s": args.case_hold_seconds,
                  "paused_display_helper_sha256": sha256_file(ROOT / "src/reconciliation/physics_display.py"),
                  "simulation_timing_unchanged": True, "physics_execution": True,
                  "saved_pose_replay": False, "gui_selection": selection}
    write_json(output / "provenance.json", processing)
    completed = []

    def execute_case(case):
        evidence = cases[case]
        state["last_case"] = case
        directory = output / f"{len(completed):02d}_{case}"
        directory.mkdir()
        configure_camera(runtime, evidence, directory)
        xy_image.source_url = str(run / "gui_plots" / f"{case}_xy.png")
        primary = summary["outcomes"][case]["M3_LOOKAHEAD"]
        case_label.text = f"{case}: {evidence.corpus_transition_id}"
        status_label.text = f"PRIMARY M3 TRACKING: {primary['physical_tracking_status']} (3 runs)"
        friendly = {"goal_success_rate": "goal", "position_rmse": "path RMS", "final_position_error": "endpoint",
                    "yaw_rmse": "heading RMS", "final_yaw_error": "final heading", "saturation": "command limit",
                    "no_pathological_stuck": "progress"}
        reasons = [friendly.get(key, key) for key in primary["failed_checks"]]
        primary_label.text = (f"Primary M3: goal {primary['goal_successes']}/3\n"
            f"Position RMS: {100*primary['repeatability']['position_rmse_m']['mean']:.2f} cm\n"
            f"Yaw RMS: {primary['repeatability']['yaw_rmse_rad']['mean']:.3f} rad\n"
            f"Failed gates: {', '.join(reasons) or 'none'}")
        outcome_label.text = "New execution pending"
        trails = {"M0_RAW": [], "M3_LOOKAHEAD": []}
        def redraw():
            draw.clear_lines()
            draw.clear_points()
            z = args.line_height
            for method, key in (("M0_RAW", "raw_reference_color"), ("M3_LOOKAHEAD", "reference_color")):
                draw_polyline(draw, evidence.method_candidates[method], z=z, color=tuple(config["gui"][key]), width=5.0)
            for method, key in (("M0_RAW", "raw_actual_color"), ("M3_LOOKAHEAD", "actual_color")):
                if len(trails[method]) > 1:
                    draw_polyline(draw, np.asarray(trails[method]), z=z, color=tuple(config["gui"][key]), width=5.0)
            b = evidence.active_old.boundary_pose_world_se2
            draw.draw_points([(float(b[0]), float(b[1]), z)], [tuple(config["gui"]["boundary_color"])], [16.0])
        results = {}
        for method in (config["methods"] if args.compare_raw else ["M3_LOOKAHEAD"]):
            phase_label.text = f"LIVE PHYSICS: {method}"
            print(f"EXP02D_PHYSICAL_GUI_PHASE={case} {method}", flush=True)
            def callback(info, poses):
                trails[method] = poses
                redraw()
                live_label.text = (f"t={(len(poses)-1)*runtime.physics_dt:.2f} s\n"
                    f"Desired v/w: {info['desired'][0]:.3f} / {info['desired'][1]:.3f}\n"
                    f"Measured v/w: {info['measured'][0]:.3f} / {info['measured'][1]:.3f}\n"
                    f"Goal reached: {info['goal_reached']}")
            started = utc()
            telemetry = run_closed_loop(runtime, loop, follower, evidence.method_candidates[method],
                      evidence.active_old.boundary_pose_world_se2, "calibrated", on_control=callback)
            runtime.world.pause()
            trails[method] = telemetry.actual_trajectory
            redraw()
            metrics = compute_closed_loop_metrics(telemetry)
            trial = directory / method
            hashes = save_closed_loop_trial(trial, telemetry, metrics, {
                "evaluation_semantics": "reference-matched closed-loop evaluation", "mode": "calibrated",
                "scenario": f"{case}_{method}", "repetition": 0, "case": case, "method": method,
                "started_utc": started, "completed_utc": utc(), "quantitative_primary": False,
                "provenance_path": str(output / "provenance.json"), "provenance_sha256": sha256_file(output / "provenance.json"),
                "reference_sha256": sha256_file(run / "references" / case / f"{method}.npy"),
                "coordinate_frame": protocol["coordinate_frame"], "timing": protocol["timing"],
                "source_switch_sim_time_s": evidence.active_old.switch_sim_time_s,
                "requested_initial_pose_world_se2": evidence.active_old.boundary_pose_world_se2.tolist(),
                "settled_initial_pose_world_se2": telemetry.actual_trajectory[0].tolist()})
            validate_closed_loop_trial(trial)
            primary_actual = np.load(run / "trials" / case / method / "repetition_00/raw/actual_trajectory.npy", allow_pickle=False)
            same_shape = primary_actual.shape == telemetry.actual_trajectory.shape
            comparison = {"same_shape": same_shape, "exactly_equal": bool(same_shape and np.array_equal(primary_actual, telemetry.actual_trajectory))}
            if same_shape:
                comparison["maximum_abs_pose_component_difference"] = float(np.max(np.abs(primary_actual - telemetry.actual_trajectory)))
            outcome_label.text = f"New {method}: goal={metrics['goal_reached']}\nRMS={metrics['position_rmse_m']*100:.2f} cm; yaw RMS={metrics['yaw_rmse_rad']:.3f} rad"
            phase_label.text = f"{method} FINISHED | physics paused"
            capture_result = capture(directory / f"{method}_final.png", runtime)
            results[method] = {"metrics": metrics, "raw_sha256": hashes, "capture": capture_result,
                               "comparison_to_primary_repetition_00": comparison}
        write_json(directory / "result.json", {"case": case, "primary_status": primary["physical_tracking_status"],
                    "methods": results, "completed_utc": utc()})
        completed.append(str(directory.relative_to(output)))
        print(f"EXP02D_PHYSICAL_GUI_CASE_COMPLETE={directory}", flush=True)

    requested = [initial_case]
    if args.both:
        requested = [case for case in (selection["success"], selection["failure"]) if case is not None]
    if args.all_cases:
        requested = config["cases"]
    for case in requested:
        execute_case(case)
        until = time.monotonic() + args.case_hold_seconds
        while APP.is_running() and time.monotonic() < until:
            if not render_paused(runtime.world, allow_editor_stop=True):
                phase_label.text = "Editor stopped/reset. Next case will reset at B."
                print("EXP02D_PHYSICAL_GUI_EDITOR_STOP; saved trials unchanged", flush=True)
            time.sleep(.02)
    print(f"EXP02D_PHYSICAL_GUI_READY={output}", flush=True)
    while APP.is_running() and not args.no_hold:
        if state["pending"] is not None:
            case, state["pending"] = state["pending"], None
            execute_case(case)
        if not render_paused(runtime.world, allow_editor_stop=True):
            phase_label.text = "Editor stopped/reset. Use Run buttons to restart."
            print("EXP02D_PHYSICAL_GUI_EDITOR_STOP; saved trials unchanged", flush=True)
        time.sleep(.02)
    write_json(output / "completed.json", {"runs": completed, "closed_utc": utc()})


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    print("EXP02D_PHYSICAL_GUI_FAILED", flush=True)
    raise
finally:
    APP.close()
