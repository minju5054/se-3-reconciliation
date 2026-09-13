"""Read/render-only presentation capture for the frozen EXP-02B-R GUI loop."""

import csv
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import omni.ui as ui
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import get_active_viewport

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from x11_window_capture import IsaacWindowCapture
from reconciliation.exp02b_calibrated_reeval import historical_nominal_post_metrics, verify_file_hash, write_json_exclusive
from reconciliation.online_switch import sha256_file
from reconciliation.physics_display import render_paused
from reconciliation.video_timing import validate_video_frames


class PresentationRecording:
    stride = 3

    def __init__(self, output, run, source, branch, snapshot, runtime):
        if branch["method"] not in ("raw_k", "graph"):
            raise ValueError("presentation comparison supports RAW and graph")
        self.output, self.run, self.source, self.branch, self.runtime = output, run, source, branch, runtime
        output.mkdir(parents=True, exist_ok=False)
        (output / "raw_frames").mkdir()
        self.frames = []
        self.case, self.k, self.method = branch["case_id"], branch["entry_index"], branch["method"]
        self.pair = {b["method"]: b for b in snapshot["branches"]
                     if b["case_id"] == self.case and b["entry_index"] == self.k and b["method"] in ("raw_k", "graph")}
        self.paths = {}
        input_files = [run / "config_snapshot.yaml", run / "historical_metric_snapshot.json",
                       run / "summary/historical_vs_calibrated.csv"]
        for method, item in self.pair.items():
            verify_file_hash(Path(item["candidate_path"]), item["candidate_sha256"], "presentation candidate")
            self.paths[method] = np.load(item["candidate_path"], allow_pickle=False)
            input_files.append(Path(item["candidate_path"]))
        for name, digest in branch["source_trial_sha256"].items():
            path = source.trial_directory / name
            verify_file_hash(path, digest, "presentation raw source")
            input_files.append(path)
        self.primary = run / "branch_results" / self.case / f"k_{self.k}" / self.method
        self.old_actual = np.load(self.primary / "old_replay_actual.npy", allow_pickle=False)
        input_files.extend([self.primary / "actual_trajectory.npy", self.primary / "old_replay_actual.npy",
                            self.primary / "desired_commands.csv", self.primary / "raw_provenance.json"])
        with (run / "summary/historical_vs_calibrated.csv").open() as stream:
            self.stats = next(row for row in csv.DictReader(stream) if row["case_id"] == self.case
                              and int(row["entry_index"]) == self.k and row["method"] == self.method)
        # Turn off only lidar display flags; sensor, collision and dynamics remain active.
        changed = []
        for prim in omni.usd.get_context().get_stage().Traverse():
            if "lidar" in str(prim.GetPath()).lower():
                for name in ("drawLines", "drawPoints"):
                    attr = prim.GetAttribute(name)
                    if attr:
                        changed.append({"prim": str(prim.GetPath()), "attribute": name, "previous": attr.Get()})
                        attr.Set(False)
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.z = .55  # Display lift only; all XY/yaw data stay in saved world coordinates.
        graph = self.pair["graph"]["geometric_metrics"]
        self.panel = ui.Window("Exp02B-R | Presentation evidence", width=550, height=940)
        with self.panel.frame:
            with ui.VStack(spacing=3, style={"margin": 4}):
                name = "B: rotation-command jump" if self.case == "case_high_delta_omega" else "C: benign command switch"
                ui.Label(f"EXP02B-R | Case {name}\nk = {self.k}", height=48, word_wrap=True, style={"font_size": 20})
                ui.Label("RAW suffix" if self.method == "raw_k" else "GRAPH | previous objective", height=36, style={"font_size": 26})
                ui.Label("Post-switch controller: pi_strong (ON)\nSame saved B, follower, and frozen paths", height=50, style={"font_size": 19})
                metric = "delta_omega_abs_rps" if self.case == "case_high_delta_omega" else "delta_v_abs_mps"
                symbol, unit = ("dw", "rad/s") if self.case == "case_high_delta_omega" else ("dv", "m/s")
                raw = self.pair["raw_k"]["historical_controller_metrics"][metric]
                corrected = self.pair["graph"]["historical_controller_metrics"][metric]
                ui.Label(f"FIRST DESIRED COMMAND CHANGE\nRAW |{symbol}| = {raw:.4f} {unit}\nGRAPH |{symbol}| = {corrected:.4f} {unit}",
                         height=80, style={"font_size": 23})
                ui.Label("New first command minus last OLD command.\nThis is not measured body-motion error.", height=44, style={"font_size": 17})
                ui.Label(f"Graph entry moved: {graph['selected_entry_displacement']['translation_m'] * 100:.2f} cm\nRelative path motion / turn sign preserved",
                         height=48, style={"font_size": 19})
                a = float(self.stats["historical_desired_omega_tracking_rmse_rps"])
                b = float(self.stats["calibrated_desired_omega_tracking_rmse_rps"])
                ui.Label(f"ACTUAL EXECUTION | this method\nYaw-rate tracking RMSE: {a:.3f} -> {b:.3f} rad/s\nHistorical nominal -> corrected\nCorrected path RMS: {float(self.stats['calibrated_position_rmse_m']) * 100:.2f} cm\n2 s window, samples every 0.1 s, includes B",
                         height=112, style={"font_size": 19})
                self.live = ui.Label("Preparing replay", height=108, word_wrap=True, style={"font_size": 20})
                ui.Label("BLUE: planned OLD / CYAN: saved actual OLD\nGRAY: original FRESH / ORANGE: graph\nGREEN: live corrected-controller motion\nYELLOW: saved B / PINK: raw F_k\nWHITE: active entry (may overlap raw F_k)",
                         height=104, style={"font_size": 18})
                ui.Label("OLD history is static, from nominal execution.\nLive clip: exact reset at B, PI reset to zero.\nFixed 2 s diagnostic; no navigation pass/fail.\n4x slow motion in exported video.",
                         height=88, style={"font_size": 17})
        stage = ui.Workspace.get_window("Stage")
        if stage:
            self.panel.dock_in(stage, ui.DockPosition.SAME)
        for name in ("Property", "Content", "Console", "Stage", "Render Settings"):
            window = ui.Workspace.get_window(name)
            if window:
                window.visible = False
        self.panel.focus()
        self.capture = IsaacWindowCapture()
        files = [Path(__file__), Path(__file__).with_name("exp02b_calibrated_reeval_gui.py"),
                 Path(__file__).with_name("closed_loop_execution_runtime.py"),
                 Path(__file__).with_name("x11_window_capture.py")]
        write_json_exclusive(output / "provenance.json", {
            "primary_run": str(run), "primary_branch": str(self.primary),
            "input_sha256": {str(p): sha256_file(p) for p in input_files},
            "code_sha256": {str(p): sha256_file(p) for p in files},
            "source_trial": str(source.trial_directory), "source_timing": source.source_attempt["timing"],
            "source_coordinate_metadata": {key: source.source_metadata.get(key) for key in (
                "decoded_action_semantics", "new_world_anchor", "controller_reference_policy", "raw_switch_policy")},
            "coordinate_frame": "identity saved Isaac world XY metres; yaw radians, +Z CCW; no alignment/reanchoring",
            "visual_line_z_m": self.z, "display_only_lidar_flags": changed,
            "physics_dt_s": runtime.physics_dt, "control_dt_s": runtime.control_dt, "capture_stride": self.stride,
            "capture": "real physics rerun; XComposite Isaac window; paused render and unchanged clock check; no pose interpolation",
            "old_semantics": "static saved nominal OLD replay; post-switch correction ON, exact B reset, restored OLD wheel target, zero PI",
        })
        write_json_exclusive(output / "context.json", {"planned_old_world_se2": source.old_world.tolist(),
            "saved_actual_old_world_se2": self.old_actual.tolist(), "raw_full_fresh_world_se2": source.fresh_world.tolist(),
            "P_world_se2": source.previous_pose_world.tolist(), "B_world_se2": source.boundary_pose_world.tolist(),
            "method_paths_world_se2": {k: v.tolist() for k, v in self.paths.items()},
            "source_timing": source.source_attempt["timing"], "paired_frozen_metrics": self.pair,
            "primary_comparable_metrics": self.stats})

    def set_camera(self):
        actuals = [np.load(self.run / "branch_results" / self.case / f"k_{self.k}" / method / "actual_trajectory.npy", allow_pickle=False)
                   for method in self.paths]
        visible = np.vstack([self.source.old_world, self.source.fresh_world, self.old_actual, *self.paths.values(), *actuals])[:, :2]
        center = (visible.min(axis=0) + visible.max(axis=0)) / 2
        extent = float(np.max(np.ptp(visible, axis=0)))
        eye = [float(center[0]), float(center[1] - .35), max(3.2, 2 * extent + 1.0)]
        target = [float(center[0]), float(center[1]), 0.0]
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1440, 1000))
        set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
        write_json_exclusive(self.output / "camera.json", {"eye_xyz": eye, "target_xyz": target,
            "same_camera_for_raw_and_graph": True, "bounds_include_both_primary_actuals": True})

    def redraw(self, actual):
        draw, z = self.draw, self.z
        draw.clear_lines()
        draw.clear_points()
        draw_polyline(draw, self.source.old_world, z=z, color=(.15, .4, 1., 1), width=3)
        draw_polyline(draw, self.old_actual, z=z, color=(.05, .95, 1., 1), width=3)
        draw_polyline(draw, self.source.fresh_world, z=z, color=(.65, .65, .65, 1), width=2)
        for method, color in [("raw_k", (.65, .65, .65, 1)), ("graph", (1., .45, .05, 1))]:
            draw_polyline(draw, self.paths[method], z=z, color=color, width=6 if self.method == method else 3)
            draw_heading_markers(draw, self.paths[method], z=z, color=color, width=2, length_m=.10, stride=2)
        if len(actual) > 1:
            draw_polyline(draw, np.asarray(actual), z=z, color=(.05, 1., .18, 1), width=5)
        for pose, color, size in [(self.source.boundary_pose_world, (1., .9, .05, 1), 17),
                                  (self.source.fresh_world[self.k], (1., .15, .8, 1), 14),
                                  (self.paths[self.method][0], (1., 1., 1., 1), 8)]:
            draw_pose_points(draw, np.asarray([pose]), z=z, color=color, size=size)

    def frame(self, actual, desired, executed, measured):
        index = len(actual) - 1
        if not self.frames:
            self.origin = float(self.runtime.world.current_time)
            # World.reset / asynchronous Kit layout restore can hide early UI.
            # Reassert the evidence panel after OLD replay and exact reset.
            self.panel.visible = True
            viewport_window = ui.Workspace.get_window("Viewport")
            if viewport_window:
                self.panel.dock_in(viewport_window, ui.DockPosition.RIGHT, .33)
            self.panel.focus()
        world_time = float(self.runtime.world.current_time)
        self.redraw(actual)
        self.live.text = (f"LIVE PHYSICS | t = {index * self.runtime.physics_dt:.2f} s\n"
            f"                   v (m/s)       w (rad/s)\n"
            f"Desired:      {desired[0]: .3f}          {desired[1]: .3f}\n"
            f"Executed:   {executed[0]: .3f}          {executed[1]: .3f}\n"
            f"Measured:  {measured[0]: .3f}          {measured[1]: .3f}")
        file = self.output / "raw_frames" / f"{len(self.frames):06d}.png"
        render_paused(self.runtime.world, 4)
        size = self.capture.capture(file)
        if float(self.runtime.world.current_time) != world_time:
            raise RuntimeError("capture advanced physics time")
        self.frames.append({"physics_index": index, "sim_time_s": index * self.runtime.physics_dt,
            "simulator_world_time_s": world_time, "pose_world_se2": actual[-1].tolist(),
            "desired_v_omega": desired.tolist(), "executed_v_omega": executed.tolist(),
            "measured_v_omega": measured.tolist(),
            "measured_interval": "since current 0.1 s control start; t=0 uses final OLD interval",
            "file": str(file.relative_to(self.output)), "sha256": sha256_file(file), "size": size,
            "captured_utc": datetime.now(timezone.utc).isoformat()})
        self.runtime.world.play()

    def finish(self, actual, telemetry, gate, invariant):
        actual = np.asarray(actual)
        primary = np.load(self.primary / "actual_trajectory.npy", allow_pickle=False)
        equal = bool(np.array_equal(actual, primary))
        timing = validate_video_frames(self.frames, self.runtime.physics_dt, self.stride,
                                       np.arange(len(actual)) * self.runtime.physics_dt, actual)
        comparable = historical_nominal_post_metrics(candidate=self.paths[self.method],
            actual=actual[::self.runtime.physics_steps], desired=np.asarray([r["desired_v_omega"] for r in telemetry]),
            control_dt_s=self.runtime.control_dt)
        self.live.text = ("2 s WINDOW COMPLETE\n"
            f"Actual yaw-rate RMSE: {comparable['desired_omega_vs_measured_omega_rmse_rps']:.3f} rad/s\n"
            f"Path RMS (includes B): {comparable['position_rmse_m'] * 100:.2f} cm\n"
            f"Matches recorded primary motion: {equal}\nNo navigation success/failure label")
        render_paused(self.runtime.world, 4)
        self.capture.capture(self.output / "end_card.png")
        self.capture.close()
        write_json_exclusive(self.output / "recording.json", {"case": self.case, "k": self.k, "method": self.method,
            "frames": self.frames, "timing": timing, "primary_poses_exactly_equal": equal,
            "maximum_absolute_pose_difference": float(np.max(np.abs(actual - primary))),
            "comparable_control_time_metrics": comparable, "comparability_gate": gate, "first_desired_invariant": invariant,
            "provenance_sha256": sha256_file(self.output / "provenance.json"),
            "end_card_sha256": sha256_file(self.output / "end_card.png")})
        if not equal:
            raise RuntimeError("recorded physical motion differs from frozen primary; do not present as primary reproduction")
