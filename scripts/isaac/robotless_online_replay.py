#!/usr/bin/env python3
"""Replay saved online execution samples and raw chunks; never run inference.

Replay selects recorded states by their simulation timestamps without generating
intermediate poses. Display wall time and saved simulation time remain distinct.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts")]

import numpy as np
import yaml

from reconciliation.online_handoff_analysis import metrics_from_records
from reconciliation.robotless_online import dump_new, verify_resume
from reconciliation.se2 import local_trajectory_to_world, wrap_angle

LABEL = "RECORDED ONLINE EPISODE REPLAY"
COLORS = {"old": (.1, .35, 1., 1.), "fresh": (1., .05, .75, 1.),
          "past": (.22, .22, .22, 1.), "inference": (.15, .85, .3, 1.),
          "post": (1., .4, .05, 1.), "state": (1., .95, .05, 1.),
          "q": (1., 1., 1., 1.)}


def read_json(path):
    return json.loads(Path(path).read_text())


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def overview_aperture(poses, aspect):
    """Fit unchanged fixed-world XY at equal scale, below the Hospital ceiling."""
    xy = np.asarray(poses, dtype=float)[:, :2]
    if not len(xy) or not np.isfinite(xy).all() or not np.isfinite(aspect) or aspect <= 0:
        raise ValueError("finite geometry and positive viewport aspect required")
    lower, upper = xy.min(axis=0), xy.max(axis=0)
    extent = upper - lower
    vertical_span = max(2.4, float(extent[1]) + .6, (float(extent[0]) + .6) / aspect)
    return (lower + upper) / 2, [vertical_span * aspect, vertical_span]


class SavedEpisode:
    """Read-only loader and sampled replay clock, independent of Isaac imports."""
    def __init__(self, run, episode_id):
        self.run = Path(run).resolve()
        self.path = (self.run / "episodes" / episode_id).resolve()
        if self.path.parent != self.run / "episodes" or not episode_id:
            raise ValueError("episode ID must identify one direct episode directory")
        if not verify_resume(self.path):
            raise ValueError("completed, hash-verified saved episode is required")
        self.episode_id = episode_id
        self.config = yaml.safe_load((self.run / "config_snapshot.yaml").read_text())
        self.metadata = read_json(self.path / "metadata.json")
        self.completion = read_json(self.path / "completion.json")
        self.execution = read_csv(self.path / "execution.csv")
        self.commands = read_csv(self.path / "commands.csv")
        if not self.execution:
            raise ValueError("saved episode has no actual execution samples")
        self.poses = np.array([[float(row[k]) for k in ("x", "y", "yaw")] for row in self.execution])
        self.times = np.array([float(row["sim_time_s"]) for row in self.execution])
        self.ids = np.array([int(row["state_id"]) for row in self.execution])
        if (not np.isfinite(self.poses).all() or not np.isfinite(self.times).all()
                or np.any(np.diff(self.times) <= 0) or len(set(self.ids)) != len(self.ids)):
            raise ValueError("invalid actual state/timestamp stream")
        self.lookup = {int(state_id): index for index, state_id in enumerate(self.ids)}
        self.applied = {int(row["application_state_id"]): row for row in self.commands}
        if len(self.applied) != len(self.commands):
            raise ValueError("duplicate command application state")
        self.sources = {p: file_hash(p) for p in (
            self.run / "config_snapshot.yaml", self.path / "metadata.json",
            self.path / "completion.json", self.path / "execution.csv", self.path / "commands.csv")}
        self.chunks = {}
        for world_path in sorted((self.path / "chunks").glob("*/world.npy")):
            chunk_path = world_path.parent
            raw_path = chunk_path / "raw_local.npy"
            meta_path = chunk_path / "metadata.json"
            raw, world = (np.load(p, allow_pickle=False) for p in (raw_path, world_path))
            meta = read_json(meta_path)
            anchor = meta.get("observation_pose_world", meta.get("R_obs", meta.get("observation", {}).get("pose_world")))
            if anchor is None:
                raise ValueError("chunk agent capture pose missing")
            expected = local_trajectory_to_world(anchor, raw)
            if (world.shape != expected.shape or not np.allclose(world[:, :2], expected[:, :2], rtol=0, atol=1e-10)
                    or not np.allclose(wrap_angle(world[:, 2] - expected[:, 2]), 0, rtol=0, atol=1e-10)):
                raise ValueError("raw/world capture-anchor mismatch")
            world.flags.writeable = False
            self.chunks[chunk_path.name] = world
            self.sources.update({p: file_hash(p) for p in (raw_path, world_path, meta_path)})
        self.events, self.unrenderable_events = [], []
        required = {"old_chunk_id", "fresh_chunk_id", "old_raw_local_ref", "fresh_raw_local_ref",
                    "switch_state_id", "ready_state_id", "obs_state_id", "B", "R_obs", "R_ready"}
        for path in sorted((self.path / "handoffs").glob("*/context.json")):
            context = read_json(path)
            self.sources[path] = file_hash(path)
            if not required.issubset(context) or context.get("switch_state_id") is None:
                self.unrenderable_events.append({"path": str(path), "status": context.get("status"),
                    "reason": "attempt has no complete activated handoff context"})
                continue
            for key in ("old_raw_local_ref", "old_world_ref", "fresh_raw_local_ref", "fresh_world_ref"):
                ref = context[key]
                target = (self.path / ref["path"]).resolve()
                if not target.is_relative_to(self.path) or file_hash(target) != ref["sha256"]:
                    raise ValueError("handoff raw/world reference hash mismatch")
            old, fresh = self.chunks[context["old_chunk_id"]], self.chunks[context["fresh_chunk_id"]]
            metrics = metrics_from_records(context, self.execution, self.commands, old, fresh)
            if not np.array_equal(np.asarray(context["B"]), self.poses[metrics["B_stream_row"]]):
                raise ValueError("saved handoff boundary is not the exact actual stream state")
            self.events.append({"context": context, "metrics": metrics})
        self.events.sort(key=lambda event: event["metrics"]["B_stream_row"])
        self.rgb = read_csv(self.path / "rgb_index.csv") if (self.path / "rgb_index.csv").exists() else []
        if self.rgb:
            self.sources[self.path / "rgb_index.csv"] = file_hash(self.path / "rgb_index.csv")
        self.poses.flags.writeable = self.times.flags.writeable = self.ids.flags.writeable = False
        self.reset()

    def reset(self):
        self.index = 0
        self.replay_sim_s = float(self.times[0])
        self.playing = False
        self.speed = 1.

    def seek_row(self, index):
        if not isinstance(index, (int, np.integer)) or not 0 <= index < len(self.times):
            raise ValueError("saved execution row outside episode")
        self.index = int(index)
        self.replay_sim_s = float(self.times[self.index])

    def advance(self, display_elapsed_s):
        dt = float(display_elapsed_s)
        if not np.isfinite(dt) or dt < 0 or self.speed not in (.5, 1., 2., 4.):
            raise ValueError("invalid display replay clock/speed")
        if self.playing:
            self.replay_sim_s = min(float(self.times[-1]), self.replay_sim_s + dt * self.speed)
            self.index = max(0, int(np.searchsorted(self.times, self.replay_sim_s, side="right") - 1))
            if self.index == len(self.times) - 1:
                self.playing = False

    def event_at_row(self):
        if not self.events:
            return None
        ready = [event for event in self.events
                 if self.lookup[int(event["context"]["ready_state_id"])] <= self.index]
        return ready[-1] if ready else self.events[0]

    def latest_rgb(self):
        eligible = [row for row in self.rgb if int(row["rendered_state_id"]) in self.lookup
                    and self.lookup[int(row["rendered_state_id"])] <= self.index]
        if not eligible:
            return None
        row = max(eligible, key=lambda item: self.lookup[int(item["rendered_state_id"])] )
        rel = row.get("path") or row.get("file")
        if not rel:
            raise ValueError("saved RGB index has no file path")
        path = (self.path / rel).resolve()
        if not path.is_relative_to(self.path) or file_hash(path) != row["sha256"]:
            raise ValueError("saved RGB source hash mismatch")
        return row, path

    def verify_unchanged(self):
        if any(file_hash(path) != digest for path, digest in self.sources.items()):
            raise ValueError("replay source changed")
        verify_resume(self.path)


class ReplayGui:
    def __init__(self, saved, args, app, output):
        import carb
        import omni.ui as ui
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene
        self.saved, self.args, self.app, self.output = saved, args, app, output
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.stage, self.agent, _, _, self.scene = runtime_scene(saved.config, saved.poses[0], app=app)
        self.clock, self.samples, self.actions = time.monotonic(), [], []
        self.camera_mode = "handoff"
        carb.settings.get_settings().set_float("/app/window/dpiScaleOverride", 1.)
        self.panel = ui.Window(LABEL, width=640, height=1050)
        with self.panel.frame:
            with ui.VStack(spacing=5):
                ui.Label(LABEL, height=35, style={"font_size": 21})
                ui.Label(saved.episode_id, height=28, style={"font_size": 20})
                ui.Label("Saved actual E(t), chunks, images and timestamps.", height=23)
                ui.Label("Display playback only. No new prediction or execution.", height=28, word_wrap=True)
                with ui.HStack(height=38):
                    for name in ("Play / Pause", "Reset", "Next Handoff"):
                        ui.Button(name, clicked_fn=lambda name=name: self.action(name))
                with ui.HStack(height=38):
                    for name in ("Speed", "Overview", "Handoff Camera"):
                        ui.Button(name, clicked_fn=lambda name=name: self.action(name))
                self.state_label = ui.Label("", height=100, style={"font_size": 17})
                self.event_label = ui.Label("", height=165, word_wrap=True, style={"font_size": 16})
                ui.Label("BLUE: selected event OLD raw predicted path", height=23)
                ui.Label("MAGENTA: selected event FRESH (after saved readiness)", height=23)
                ui.Label("DARK GRAY: actual executed past E(t)", height=23)
                ui.Label("GREEN: saved inference interval execution", height=23)
                ui.Label("ORANGE: this FRESH's actual execution only", height=23)
                ui.Label("YELLOW: sampled agent state / B; WHITE: projection Q", height=23)
                ui.Label("All XY coordinates unchanged; glyph height is display only.", height=26, word_wrap=True)
                ui.Label("SAVED CAMERA FRAME (never resubmitted to model)", height=27)
                self.rgb_image = ui.Image("", height=230, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                self.rgb_label = ui.Label("", height=50, word_wrap=True)
                ui.Label("This replay is separate from live online collection.", height=28)
        for _ in range(4):
            app.update()
        stage_panel = ui.Workspace.get_window("Stage")
        if stage_panel:
            self.panel.dock_in(stage_panel, ui.DockPosition.SAME)
        for name in ("Property", "Content", "Console", "Stage", "Render Settings"):
            window = ui.Workspace.get_window(name)
            if window:
                window.visible = False
        for _ in range(4):
            app.update()
        ui.Workspace.set_dock_id_width(self.panel.dock_id, 650)
        self.panel.focus()
        self.set_camera()
        self.refresh()

    def action(self, name):
        s = self.saved
        if name == "Play / Pause":
            s.playing = not s.playing
        elif name == "Reset":
            s.reset()
        elif name == "Speed":
            s.speed = {.5: 1., 1.: 2., 2.: 4., 4.: .5}[s.speed]
        elif name == "Next Handoff":
            indices = [event["metrics"]["B_stream_row"] for event in s.events]
            if indices:
                s.seek_row(next((index for index in indices if index > s.index), indices[0]))
                s.playing = False
                self.set_camera()
        elif name in ("Overview", "Handoff Camera"):
            self.camera_mode = "overview" if name == "Overview" else "handoff"
            self.set_camera()
        else:
            raise ValueError(name)
        self.clock = time.monotonic()
        self.actions.append({"action": name, "saved_state_id": int(s.ids[s.index]),
                             "display_host_monotonic_s": self.clock, "playing": s.playing})
        self.refresh()

    def set_camera(self):
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import UsdGeom
        s = self.saved
        event = s.event_at_row()
        resolution = get_active_viewport().resolution
        aspect = float(resolution[0]) / float(resolution[1])
        if self.camera_mode == "overview":
            geometry = np.vstack([s.poses, *s.chunks.values()])
            xy, spans = overview_aperture(geometry, aspect)
        else:
            xy = np.asarray(event["context"]["B"][:2] if event else s.poses[s.index, :2])
            spans = [2.4 * aspect, 2.4]
        eye, target = [float(xy[0]), float(xy[1]), 2.4], [float(xy[0]), float(xy[1]), .14]
        self.camera = {"mode": self.camera_mode, "eye_world_m": eye, "target_world_m": target,
                       "geometry_scaling": False, "projection": "orthographic",
                       "viewport_resolution": list(resolution), "world_span_xy_m": spans,
                       "equal_xy_scale": True, "aperture_units": "tenths of scene unit; one scene unit is one metre"}
        set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
        camera = UsdGeom.Camera(self.stage.GetPrimAtPath("/OmniverseKit_Persp"))
        camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        camera.CreateHorizontalApertureAttr(float(spans[0]) * 10.)
        camera.CreateVerticalApertureAttr(float(spans[1]) * 10.)

    def refresh(self):
        from robotless_old_consistent_observation import set_precise_agent_pose
        from robotless_runtime import actual_pose
        s, draw = self.saved, self.draw
        pose, state_id = s.poses[s.index], int(s.ids[s.index])
        set_precise_agent_pose(self.agent, pose, z_m=float(s.config["agent"]["z_m"]))
        if not np.allclose(actual_pose(self.agent)[:2], pose[:2], atol=1e-10, rtol=0):
            raise ValueError("replay USD readback differs from actual saved state")
        if abs(wrap_angle(actual_pose(self.agent)[2] - pose[2])) > 1e-10:
            raise ValueError("replay USD yaw differs from actual saved state")
        draw.clear_lines(); draw.clear_points()

        def path(values, color, width=4.):
            values = np.asarray(values)
            if len(values) > 1:
                points = [[float(row[0]), float(row[1]), .14] for row in values]
                draw.draw_lines(points[:-1], points[1:], [COLORS[color]]*(len(points)-1), [width]*(len(points)-1))

        def point(value, color, size=12.):
            draw.draw_points([[float(value[0]), float(value[1]), .17]], [COLORS[color]], [size])

        def yaws(values, color):
            for p in values:
                start = [float(p[0]), float(p[1]), .15]
                end = [float(p[0]+.08*np.cos(p[2])), float(p[1]+.08*np.sin(p[2])), .15]
                draw.draw_lines([start], [end], [COLORS[color]], [2.])

        path(s.poses[:s.index+1], "past", 5.)
        event = s.event_at_row()
        if event:
            context, metrics = event["context"], event["metrics"]
            old, fresh = s.chunks[context["old_chunk_id"]], s.chunks[context["fresh_chunk_id"]]
            # An event can be selected before its readiness in recorded playback.
            # FRESH is revealed only at the actual saved first-ready-seen state.
            if any(row["chunk_id"] == context["old_chunk_id"] and s.lookup[int(row["application_state_id"])] <= s.index for row in s.commands):
                path(old, "old"); yaws(old, "old")
            fresh_visible = s.index >= s.lookup[int(context["ready_state_id"])]
            if fresh_visible:
                path(fresh, "fresh"); yaws(fresh, "fresh")
                point(metrics["fresh_projection"]["Q_xy_world_m"], "q", 9.)
            for key, color in (("inference_execution", "inference"), ("post_switch_execution", "post")):
                limits = metrics[key]["stream_rows_inclusive"]
                if limits is None:
                    continue
                begin, end = limits
                stop = min(s.index, end)
                if stop >= begin:
                    path(s.poses[begin:stop+1], color, 7.)
            if s.index >= metrics["B_stream_row"]:
                point(context["B"], "state", 10.)
            self.event_label.text = (f'Selected {context["event_id"]}: {context["old_chunk_id"]} -> {context["fresh_chunk_id"]}\n'
                f'{context["status"]}\nSaved RTT {metrics["client_rtt_s"]:.3f} s\n'
                f'Saved obs-to-switch sim age {metrics["observation_to_switch_sim_s"]:.3f} s\n'
                f'Actual history {context["actual_history_count"]}; timing valid {metrics["timing_valid"]}\n'
                f'FRESH ready in replay: {fresh_visible}; B state ID {context["switch_state_id"]}')
        else:
            command = s.applied.get(state_id)
            if command and command["chunk_id"] in s.chunks:
                path(s.chunks[command["chunk_id"]], "old")
            self.event_label.text = "No complete activated handoff in this recorded episode.\n" + str(s.completion.get("status", s.metadata.get("status", "see completion record")))
        point(pose, "state", 16.)
        command = s.applied.get(state_id)
        command_text = "bootstrap hold" if command is None or not command["chunk_id"] else f'{command["chunk_id"]} / version {command["reference_version"]}'
        self.state_label.text = (f'Saved simulation time {s.times[s.index]:.3f} s\n'
            f'State ID {state_id}; row {s.index}/{len(s.times)-1}\n'
            f'Display speed {s.speed:g}x; {"PLAYING" if s.playing else "PAUSED"}\nCommand attached to pre-step state: {command_text}')
        rgb = s.latest_rgb()
        if rgb:
            row, image = rgb
            url = f"file:{image}"
            if self.rgb_image.source_url != url:
                self.rgb_image.source_url = url
            self.rgb_label.text = f'Saved frame {row["frame_id"]}; rendered state ID {row["rendered_state_id"]}'
        else:
            self.rgb_image.source_url = ""
            self.rgb_label.text = "No captured frame at or before this saved state."
        self.samples.append({"display_host_monotonic_s": time.monotonic(), "saved_state_id": state_id,
                             "saved_sim_time_s": float(s.times[s.index]), "saved_row": s.index,
                             "pose_world": pose.tolist(), "usd_pose_readback": actual_pose(self.agent).tolist()})

    def tick(self):
        now = time.monotonic()
        previous = self.saved.index
        self.saved.advance(now - self.clock)
        self.clock = now
        if previous != self.saved.index:
            self.refresh()
        self.app.update()

    def capture(self, name):
        import omni.kit.renderer.capture as capture
        from PIL import Image
        path = self.output / f"{name}.png"
        for _ in range(10):
            self.app.update()
        capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(path))
        deadline = time.monotonic() + 25.
        while time.monotonic() < deadline:
            self.app.update()
            if path.exists() and path.stat().st_size:
                try:
                    with Image.open(path) as image:
                        image.load()
                        if np.asarray(image).std() < 1:
                            raise ValueError("blank replay GUI screenshot")
                    break
                except OSError:
                    pass
        else:
            raise RuntimeError("replay GUI screenshot unavailable")
        dump_new(path.with_suffix(".json"), {"label": LABEL, "episode_id": self.saved.episode_id,
            "image_sha256": file_hash(path), "source_run": str(self.saved.run),
            "saved_state": self.samples[-1], "camera": self.camera,
            "display_playback_only": True, "new_inference_count": 0,
            "capture_utc": datetime.now(timezone.utc).isoformat()})
        self.clock = time.monotonic()

    def verify(self):
        self.action("Reset")
        self.capture("replay_start")
        self.action("Play / Pause")
        deadline = time.monotonic() + .35
        while self.app.is_running() and time.monotonic() < deadline:
            self.tick()
        if self.saved.playing:
            self.action("Play / Pause")
        if len(self.saved.times) > 1 and self.saved.index == 0:
            raise ValueError("replay did not advance to another actual saved sample")
        if self.saved.events:
            self.action("Reset")
            self.action("Next Handoff")
            self.capture("replay_handoff")
            event = self.saved.event_at_row()
            begin, end = event["metrics"]["post_switch_execution"]["stream_rows_inclusive"]
            self.saved.seek_row(min(end, begin + 30))
            self.refresh()
            self.capture("replay_post_switch")
        self.action("Overview")
        self.capture("replay_overview")
        self.action("Handoff Camera")
        self.action("Speed"); self.action("Speed"); self.action("Speed"); self.action("Speed")
        self.saved.verify_unchanged()
        dump_new(self.output / "runtime_validation.json", {
            "status": "RECORDED_ONLINE_REPLAY_RUNTIME_VALIDATED", "label": LABEL,
            "source_run": str(self.saved.run), "episode_id": self.saved.episode_id,
            "method": "actual Isaac renderer with same callbacks as GUI controls; no OS mouse-click claim",
            "sources": [{"path": str(path), "sha256": digest} for path, digest in self.saved.sources.items()],
            "source_unchanged": True, "display_playback_only": True,
            "new_inference_count": 0, "generated_execution_states": 0,
            "sampling": "last actual state at or before display replay simulation time; no interpolation",
            "controls": self.actions, "displayed_actual_samples": self.samples,
            "unrenderable_attempts": self.saved.unrenderable_events, "scene": self.scene})
        print(f"RECORDED_ONLINE_REPLAY_RUNTIME_VALIDATED {self.output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--auto-replay", action="store_true")
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    args = parser.parse_args()
    saved = SavedEpisode(args.run, args.episode)
    output = (args.output or ROOT / "data/robotless_online_replay" /
              datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")).resolve()
    if output.is_relative_to(saved.run):
        raise ValueError("replay evidence must be outside the source collection run")
    output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False, "window_width": 2400, "window_height": 1320,
                         "width": 1600, "height": 1100, "renderer": "RayTracedLighting"})
    try:
        gui = ReplayGui(saved, args, app, output)
        if args.verify:
            gui.verify()
        if args.auto_replay:
            gui.action("Reset"); gui.action("Play / Pause")
        started = time.monotonic()
        while app.is_running():
            gui.tick()
            if args.no_hold and not saved.playing and time.monotonic() - started > 2.:
                break
        saved.verify_unchanged()
    finally:
        app.close()


if __name__ == "__main__":
    main()
