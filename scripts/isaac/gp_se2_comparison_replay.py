#!/usr/bin/env python3
"""Display saved GP-SE2-01 counterfactuals in the unchanged Isaac Hospital.

This viewer never runs inference, an optimizer, an MPC solve or integration.
It assigns only saved counterfactual samples to a logical display marker.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/"src"), str(ROOT/"scripts"), str(Path(__file__).resolve().parent)]

import numpy as np
import yaml

from reconciliation.robotless_online import dump_new
from reconciliation.se2 import wrap_angle
from robotless_online_replay import overview_aperture

LABEL = "OFFLINE COUNTERFACTUAL HANDOFF COMPARISON"
HARD_LABEL = "OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON"
METHODS = ("M0_NATIVE", "M0_ADAPTER", "M1_RIGID", "M2_GP_NO_OBSTACLE", "M3_GP_CONSTRAINED")
COLORS = {"old": (.15, .38, 1., 1.), "fresh": (1., .05, .7, 1.),
    "candidate": (.05, .95, .95, 1.), "execution": (1., .5, .02, 1.),
    "boundary": (1., .9, .1, 1.), "goal": (.1, 1., .2, 1.),
    "past": (.25, .25, .25, 1.), "seed": (.65, .35, .95, 1.),
    "margin": (1., 1., 1., 1.), "gate": (.7, .25, 1., 1.), "workspace": (.65, .65, .65, 1.)}


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _poses(value, name):
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1:] != (3,) or len(array) == 0 or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain nonempty finite world poses")
    array.flags.writeable = False
    return array


def _inside(path, root):
    path = Path(path).resolve()
    if not path.is_relative_to(root):
        raise ValueError("saved replay path escaped experiment run")
    return path


def gate_segment(gate):
    """World gate display only; crossing results come from independent metrics."""
    if all(key in gate for key in ("center_xy", "normal_xy", "half_width_m")):
        center, normal = np.asarray(gate["center_xy"], float), np.asarray(gate["normal_xy"], float)
        half_width = float(gate["half_width_m"])
        if center.shape != (2,) or normal.shape != (2,) or not np.isfinite([center, normal]).all() or not np.isfinite(half_width) or half_width <= 0 or not np.isclose(np.linalg.norm(normal), 1., atol=1e-8, rtol=0):
            raise ValueError("invalid frozen gate world geometry")
        tangent = np.array([-normal[1], normal[0]])
        return np.array([center-half_width*tangent, center+half_width*tangent])
    for name in ("endpoints_world", "endpoints", "segment_world", "segment"):
        if name in gate:
            points = np.asarray(gate[name], dtype=float)
            if points.shape == (2, 2) and np.isfinite(points).all():
                return points
    if "a" in gate and "b" in gate:
        points = np.asarray([gate["a"], gate["b"]], dtype=float)
        if points.shape == (2, 2) and np.isfinite(points).all():
            return points
    if "center" in gate and "normal" in gate and "half_width" in gate:
        center, normal = np.asarray(gate["center"], float), np.asarray(gate["normal"], float)
        if center.shape == (2,) and normal.shape == (2,) and np.linalg.norm(normal) > 0:
            tangent = np.array([-normal[1], normal[0]])/np.linalg.norm(normal)
            return np.array([center-tangent*gate["half_width"], center+tangent*gate["half_width"]])
    raise ValueError("gate world geometry missing or unrecognized; refusing invented display geometry")


class SavedComparison:
    """Immutable artifact loader with a sample-selecting display clock."""
    def __init__(self, run):
        self.run = Path(run).resolve()
        self.sources = {}
        self.config = self.read_yaml(self.run/"config_snapshot.yaml")
        source = Path(self.config["source_run"])
        self.source_run = source.resolve() if source.is_absolute() else (ROOT/source).resolve()
        self.scene_config = self.read_yaml(self.source_run/"config_snapshot.yaml", external=True)
        manifest = self.read_json(self.run/"case_manifest.json")
        protocol_path = self.run/"protocol.json"
        protocol = self.read_json(protocol_path) if protocol_path.is_file() else {}
        self.hard_transfer = protocol.get("experiment") == "GP-SE2-02"
        self.methods = METHODS + ("SEED_ONLY",) if self.hard_transfer else METHODS
        self.label = HARD_LABEL if self.hard_transfer else LABEL
        self.runtime_status = ("GP_SE2_02_COMPARISON_REPLAY_RUNTIME_VALIDATED" if self.hard_transfer
                               else "GP_SE2_01_COMPARISON_REPLAY_RUNTIME_VALIDATED")
        self.cases = []
        for row in manifest["selected"]:
            directory = _inside(self.run/"cases"/row["case_directory"], self.run/"cases")
            context = self.read_json(directory/"input_context.json")
            goal_route = self.read_json(directory/"goal_route.json")
            goal = np.asarray(goal_route["goal_world"], dtype=float)
            boundary = np.asarray(context["B_world"], dtype=float)
            if goal.shape != (3,) or boundary.shape != (3,) or not np.isfinite([goal, boundary]).all():
                raise ValueError("invalid frozen goal/boundary")
            methods = {}
            for name in self.methods:
                folder = directory/"methods"/name
                metrics = self.read_json(folder/"metrics.json")
                candidate_path, rollout_path = folder/"candidate_world.npy", folder/"rollout/rollout.json"
                candidate = None
                if candidate_path.exists():
                    self.record(candidate_path)
                    candidate = _poses(np.load(candidate_path, allow_pickle=False), "candidate")
                rollout = None
                poses, times = None, None
                if rollout_path.exists():
                    rollout = self.read_json(rollout_path)
                    poses = _poses([state["pose_world"] for state in rollout["states"]], "saved execution")
                    times = np.asarray([state["time_s"] for state in rollout["states"]], dtype=float)
                    if len(times) != len(poses) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or times[0] != 0:
                        raise ValueError("invalid saved counterfactual time grid")
                    if not np.array_equal(poses[0], boundary):
                        raise ValueError("counterfactual must begin at exact frozen B")
                    if candidate is None:
                        raise ValueError("rollout exists without its candidate")
                    if not np.array_equal(np.asarray(rollout["candidate_world"]), candidate):
                        raise ValueError("candidate differs from the actual controller input")
                    times.flags.writeable = False
                for filename in ("solver_result.json", "status.json"):
                    if (folder/filename).is_file():
                        self.record(folder/filename)
                methods[name] = {"candidate": candidate, "rollout": rollout, "poses": poses, "times": times,
                    "metrics": metrics, "metric_path": folder/"metrics.json",
                    "display_status": "SAVED ROLLOUT" if rollout is not None else "NO CANDIDATE / NO ROLLOUT" if candidate is None else "CANDIDATE ONLY / NO ROLLOUT"}
            case = {"manifest": row, "directory": directory, "context": context, "goal_route": goal_route,
                "old": _poses(context["old_world"], "OLD"), "fresh": _poses(context["fresh_world"], "FRESH"),
                "B": boundary, "goal": goal, "gates": [gate_segment(g) for g in goal_route.get("gates", [])], "methods": methods}
            past_path = directory/"actual_past_execution.json"
            case["past"] = None
            if past_path.is_file():
                past = self.read_json(past_path)
                case["past"] = _poses(past["poses_world"], "recorded past")
                if not np.array_equal(case["past"][-1], boundary):
                    raise ValueError("recorded past does not end at exact B")
            self.cases.append(case)
        if not self.cases:
            raise ValueError("no selected cases available for replay")
        ids = [case["manifest"]["case_id"] for case in self.cases]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate selected case")
        for filename in ("source.json", "protocol.json", "environment/scene_provenance.json", "environment/validation.json"):
            if (self.run/filename).is_file():
                self.record(self.run/filename)
        render_geometry = self.run/"environment/geometry/render_geometry.json"
        external_geometry = False
        if self.hard_transfer and not render_geometry.is_file():
            source = self.read_json(self.run/"source.json")
            environment = Path(source["environment_path"])
            environment = environment if environment.is_absolute() else ROOT/environment
            render_geometry = environment/"geometry/render_geometry.json"
            external_geometry = True
        self.workspace_rings = []
        if render_geometry.is_file():
            if external_geometry:
                # Only the explicit provenance environment is allowed externally.
                self.record(render_geometry)
                geometry = json.loads(render_geometry.read_text())
            else:
                geometry = self.read_json(render_geometry)
            for ring in geometry.get("workspace_polygons", []):
                ring = np.asarray(ring, float)
                if ring.ndim != 2 or ring.shape[1:] != (2,) or len(ring) < 4 or not np.isfinite(ring).all() or not np.array_equal(ring[0], ring[-1]):
                    raise ValueError("invalid closed evaluated-workspace ring")
                self.workspace_rings.append(ring)
        self.cases.sort(key=lambda case: case["manifest"]["case_id"])
        self.representative_indices, self.representative_report = self.select_representatives()
        self.case_index = self.representative_indices[0]
        self.method_index = 0
        self.reset()

    def record(self, path):
        path = Path(path).resolve()
        self.sources[path] = file_hash(path)

    def read_json(self, path):
        path = _inside(path, self.run)
        self.record(path)
        return json.loads(path.read_text())

    def read_yaml(self, path, external=False):
        path = Path(path).resolve() if external else _inside(path, self.run)
        self.record(path)
        return yaml.safe_load(path.read_text())

    @property
    def case(self):
        return self.cases[self.case_index]

    @property
    def method_name(self):
        return self.methods[self.method_index]

    @property
    def method(self):
        return self.case["methods"][self.method_name]

    def select_representatives(self):
        if self.hard_transfer:
            required = (("BENIGN_CONTROL", "episode_001_repeat_01/handoff_002"),
                        ("HARD_POSITION_AND_DIRECTION", "episode_013_repeat_01/handoff_024"))
            indices, report = [], []
            for role, case_id in required:
                matches = [i for i,c in enumerate(self.cases) if c["manifest"]["case_id"] == case_id]
                if not matches:
                    raise ValueError("required fixed hard-transfer GUI case missing: " + case_id)
                indices.append(matches[0])
                report.append({"reason": role, "available": True, "selected_case_id": case_id,
                               "selection_rule": "user-fixed benign and first hard case; independent of outcomes"})
            return indices, report
        predicates = (
            ("RAW_success_to_M3_failure", lambda c: c["methods"]["M0_NATIVE"]["metrics"].get("primary_success") is True
                and c["methods"]["M3_GP_CONSTRAINED"]["metrics"].get("primary_success") is not True),
            ("obstacle_sensitive_D", lambda c: str(c["manifest"].get("selected_group", "")).startswith("D")),
            ("benign_A", lambda c: str(c["manifest"].get("selected_group", "")).startswith("A")),
        )
        indices, report = [], []
        for reason, predicate in predicates:
            matching = [index for index, case in enumerate(self.cases) if predicate(case)]
            index = matching[0] if matching else None
            report.append({"reason": reason, "available": bool(matching), "selected_case_id": None if index is None else self.cases[index]["manifest"]["case_id"], "selection_rule": "first lexical case_id satisfying recorded outcome/group"})
            if index is not None and index not in indices:
                indices.append(index)
        if not indices:
            indices = [0]
            report.append({"reason": "no_requested_representative_category_available", "selected_case_id": self.cases[0]["manifest"]["case_id"], "selection_rule": "first lexical selected case; does not imply benign or obstacle-sensitive"})
        return indices, report

    def reset(self):
        self.index, self.replay_time_s, self.playing = 0, 0., False

    def select(self, case_index=None, method_index=None):
        if case_index is not None:
            if not 0 <= case_index < len(self.cases):
                raise ValueError("unknown case index")
            self.case_index = case_index
        if method_index is not None:
            if not 0 <= method_index < len(self.methods):
                raise ValueError("unknown method index")
            self.method_index = method_index
        self.reset()

    def advance(self, elapsed):
        if not np.isfinite(elapsed) or elapsed < 0:
            raise ValueError("invalid display elapsed time")
        times = self.method["times"]
        if self.playing and times is not None:
            self.replay_time_s = min(float(times[-1]), self.replay_time_s+elapsed)
            self.index = int(np.searchsorted(times, self.replay_time_s, side="right")-1)
            if self.index == len(times)-1:
                self.playing = False
        elif times is None:
            self.playing = False

    def seek_end(self):
        if self.method["times"] is not None:
            self.index = len(self.method["times"])-1
            self.replay_time_s = float(self.method["times"][-1])
        self.playing = False

    def overview_geometry(self):
        case = self.case
        arrays = [case["old"], case["fresh"], np.array([case["B"], case["goal"]])]
        if case.get("past") is not None:
            arrays.append(case["past"])
        for method in case["methods"].values():
            arrays.extend(method[k] for k in ("candidate", "poses") if method[k] is not None)
        arrays.extend(np.column_stack([gate, np.zeros(2)]) for gate in case["gates"])
        return np.vstack(arrays)

    def verify_unchanged(self):
        if any(file_hash(path) != digest for path, digest in self.sources.items()):
            raise ValueError("saved experiment source changed during replay")


class ComparisonGui:
    def __init__(self, saved, app, output):
        import carb
        import omni.ui as ui
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene
        self.saved, self.app, self.output = saved, app, output
        self.stage, self.agent, _, _, self.scene = runtime_scene(saved.scene_config, saved.case["B"], app=app)
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.actions, self.samples, self.captures = [], [], []
        self.clock = time.monotonic()
        carb.settings.get_settings().set_float("/app/window/dpiScaleOverride", 1.)
        self.panel = ui.Window(saved.label, width=700, height=1000)
        with self.panel.frame:
            with ui.VStack(spacing=7):
                ui.Label("OFFLINE COUNTERFACTUAL", height=32, style={"font_size": 26})
                ui.Label("HARD-HANDOFF COMPARISON" if saved.hard_transfer else "HANDOFF COMPARISON", height=32, style={"font_size": 26})
                ui.Label("Saved 3 s idealized synchronous MPC outcomes.", height=25)
                ui.Label("Display only: no new inference, solve or execution.", height=25)
                with ui.HStack(height=38):
                    for label in ("Play / Pause", "Reset", "End"):
                        ui.Button(label, clicked_fn=lambda name=label: self.action(name))
                with ui.HStack(height=38):
                    ui.Button("Next Case", clicked_fn=lambda: self.action("Next Case"))
                    ui.Button("Next Method", clicked_fn=lambda: self.action("Next Method"))
                self.case_label = ui.Label("", height=80, word_wrap=True, style={"font_size": 19})
                self.method_label = ui.Label("", height=92, word_wrap=True, style={"font_size": 20})
                self.metrics_label = ui.Label("", height=155, word_wrap=True, style={"font_size": 17})
                self.state_label = ui.Label("", height=98, word_wrap=True, style={"font_size": 16})
                legends = ("BLUE: original OLD prediction", "MAGENTA: original FRESH prediction",
                           "CYAN: this method candidate reference", "ORANGE: this method saved execution",
                           "YELLOW: B and sampled footprint", "WHITE RING: required clearance margin",
                           "GREEN: common goal / tolerance", "PURPLE: frozen directed passage gates",
                           "GRAY: evaluated workspace boundary (where visible)")
                if saved.hard_transfer:
                    legends = ("BLUE: original OLD prediction", "MAGENTA: original FRESH prediction",
                               "CYAN: candidate reference; PURPLE: SEED_ONLY",
                               "DARK GRAY: recorded actual past through B", "ORANGE: saved counterfactual execution",
                               "YELLOW: B / footprint; WHITE: clearance margin",
                               "GREEN: original goal and tolerance",
                               "GRAY: evaluated workspace boundary")
                for legend in legends:
                    ui.Label(legend, height=25, style={"font_size": 17})
                ui.Label("Same world coordinates and camera scale for all methods.", height=45, word_wrap=True)
                ui.Label("Static Hospital geometry; circular evaluation footprint only.\nNo physical robot safety claim.", height=55, word_wrap=True)
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
        ui.Workspace.set_dock_id_width(self.panel.dock_id, 710)
        self.panel.focus()
        self.set_camera()
        self.refresh()

    def action(self, name):
        saved = self.saved
        if name == "Play / Pause":
            saved.playing = not saved.playing if saved.method["times"] is not None else False
        elif name == "Reset":
            saved.reset()
        elif name == "End":
            saved.seek_end()
        elif name == "Next Method":
            saved.select(method_index=(saved.method_index+1)%len(saved.methods))
        elif name == "Next Case":
            saved.select(case_index=(saved.case_index+1)%len(saved.cases))
            self.set_camera()
        else:
            raise ValueError(name)
        self.clock = time.monotonic()
        self.actions.append({"action": name, "case_id": saved.case["manifest"]["case_id"],
            "method": saved.method_name, "saved_row": saved.index, "display_host_monotonic_s": self.clock})
        self.refresh()

    def set_camera(self):
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import UsdGeom
        resolution = get_active_viewport().resolution
        aspect = float(resolution[0])/float(resolution[1])
        xy, spans = overview_aperture(self.saved.overview_geometry(), aspect)
        # .6m padding in overview_aperture contains the .25m footprint+margin.
        eye, target = [*map(float, xy), 2.4], [*map(float, xy), .14]
        set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
        camera = UsdGeom.Camera(self.stage.GetPrimAtPath("/OmniverseKit_Persp"))
        camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
        camera.CreateHorizontalApertureAttr(float(spans[0])*10.)
        camera.CreateVerticalApertureAttr(float(spans[1])*10.)
        self.camera = {"eye_world_m": eye, "target_world_m": target, "projection": "orthographic",
            "world_span_xy_m": spans, "viewport_resolution": list(resolution), "equal_xy_scale": True,
            "method_independent_bounds": True, "geometry_scaling": False}

    def refresh(self):
        from robotless_old_consistent_observation import set_precise_agent_pose
        from robotless_runtime import actual_pose
        saved, case, method = self.saved, self.saved.case, self.saved.method
        pose = case["B"] if method["poses"] is None else method["poses"][saved.index]
        set_precise_agent_pose(self.agent, pose, z_m=float(saved.scene_config["agent"]["z_m"]))
        readback = actual_pose(self.agent)
        if not np.allclose(readback[:2], pose[:2], atol=1e-10, rtol=0) or abs(wrap_angle(readback[2]-pose[2])) > 1e-10:
            raise ValueError("USD display pose differs from exact saved sample")
        self.draw.clear_lines()
        self.draw.clear_points()

        def path(points, color, width=4., height=.14):
            points = np.asarray(points)
            if len(points) > 1:
                p = [[float(v[0]), float(v[1]), height] for v in points]
                self.draw.draw_lines(p[:-1], p[1:], [COLORS[color]]*(len(p)-1), [width]*(len(p)-1))

        def ring(center, radius, color, width=2.):
            angle = np.linspace(0, 2*np.pi, 65)
            path(np.column_stack([center[0]+radius*np.cos(angle), center[1]+radius*np.sin(angle)]), color, width)

        def arrow(pose, color, length=.2):
            end = [pose[0]+length*np.cos(pose[2]), pose[1]+length*np.sin(pose[2])]
            path([pose[:2], end], color, 4., .17)

        if case.get("past") is not None:
            path(case["past"], "past", 4.)
        path(case["old"], "old", 3.)
        path(case["fresh"], "fresh", 4.)
        if method["candidate"] is not None:
            path(method["candidate"], "seed" if saved.method_name == "SEED_ONLY" else "candidate", 5., .16)
        if method["poses"] is not None:
            path(method["poses"][:saved.index+1], "execution", 7., .18)
        self.draw.draw_points([[*map(float, case["B"][:2]), .19]], [COLORS["boundary"]], [12.])
        arrow(case["B"], "boundary", .24)
        arrow(pose, "boundary", .20)
        radius = float(saved.config["footprint"]["radius_m"])
        clearance = float(saved.config["footprint"]["required_clearance_m"])
        ring(pose, radius, "boundary", 3.)
        ring(pose, radius+clearance, "margin")
        tolerance = float(saved.config["formulation"]["goal_position_tolerance"])
        ring(case["goal"], tolerance, "goal")
        arrow(case["goal"], "goal")
        for ring_points in saved.workspace_rings:
            path(ring_points, "workspace", 2.)
        for gate, record in zip(case["gates"], case["goal_route"].get("gates", [])):
            path(gate, "gate", 6.)
            normal_value = record.get("normal_xy", record.get("normal"))
            if normal_value is not None:
                normal = np.asarray(normal_value, float)
                center = gate.mean(axis=0)
                arrow([*center, np.arctan2(normal[1], normal[0])], "gate", .15)
        self.case_label.text = f'{case["manifest"]["case_id"]}\nGroup: {case["manifest"].get("selected_group", "see manifest")}'
        self.method_label.text = f'{saved.method_name}\n{method["display_status"]}'
        metrics = method["metrics"]
        reasons = metrics.get("failure_reasons", metrics.get("failures", metrics.get("status", "See saved metrics")))
        self.metrics_label.text = f'PRIMARY LOCAL SUCCESS: {metrics.get("primary_success", "UNKNOWN")}\nSaved outcome: {str(reasons)[:260]}\nSafety / goal / route use identical frozen evaluation.'
        if saved.hard_transfer:
            reasons = metrics.get("termination_reasons", reasons)
            self.metrics_label.text = (f'PLAN VALID: {metrics.get("plan_valid", "UNKNOWN")} | '
                f'LOCAL SUCCESS: {metrics.get("primary_success", "UNKNOWN")}\n'
                f'Diagnostic invalid-plan rollout: {metrics.get("diagnostic_rollout_plan_invalid", False)}\n'
                f'Saved outcome: {str(reasons)[:210]}')
        sample_time = None if method["times"] is None else float(method["times"][saved.index])
        self.state_label.text = ("NO EXECUTION SAMPLES; boundary marker only.\n" if sample_time is None else
            f'Saved t={sample_time:.3f} s / {method["times"][-1]:.3f} s; sample {saved.index}\n') + f'{"PLAYING" if saved.playing else "PAUSED"}; world metres / yaw radians\nDisplay wall time is not counterfactual simulation time.'
        self.samples.append({"case_id": case["manifest"]["case_id"], "method": saved.method_name,
            "saved_row": None if sample_time is None else saved.index, "saved_time_s": sample_time,
            "pose_world": pose.tolist(), "usd_pose_readback": readback.tolist(),
            "is_execution_sample": sample_time is not None, "display_host_monotonic_s": time.monotonic()})

    def tick(self):
        now = time.monotonic()
        previous = self.saved.index
        self.saved.advance(now-self.clock)
        self.clock = now
        if previous != self.saved.index:
            self.refresh()
        self.app.update()

    def capture(self, name):
        import omni.kit.renderer.capture as capture
        from PIL import Image
        path = self.output/f"{name}.png"
        if path.exists():
            raise FileExistsError(path)
        for _ in range(10):
            self.app.update()
        capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(path))
        deadline = time.monotonic()+25.
        while time.monotonic() < deadline:
            self.app.update()
            if path.exists() and path.stat().st_size:
                try:
                    with Image.open(path) as image:
                        image.load()
                        if np.asarray(image).std() < 1:
                            raise ValueError("blank comparison screenshot")
                    break
                except OSError:
                    pass
        else:
            raise RuntimeError("Isaac application screenshot unavailable")
        method = self.saved.method
        sidecar = {"label": self.saved.label, "case_id": self.saved.case["manifest"]["case_id"], "method": self.saved.method_name,
            "image_sha256": file_hash(path), "experiment_run": str(self.saved.run), "source_dataset_run": str(self.saved.source_run),
            "config_sha256": self.saved.sources[self.saved.run/"config_snapshot.yaml"],
            "metric_path": str(method["metric_path"]), "metric_sha256": file_hash(method["metric_path"]),
            "source_hashes": [{"path": str(p), "sha256": h} for p, h in self.saved.sources.items()],
            "saved_state": self.samples[-1], "camera": self.camera, "legend": COLORS,
            "display_status": method["display_status"], "display_playback_only": True,
            "evaluated_workspace_overlay_available": bool(self.saved.workspace_rings),
            "new_inference_count": 0, "new_mpc_solves": 0, "generated_execution_states": 0,
            "capture_utc": datetime.now(timezone.utc).isoformat()}
        dump_new(path.with_suffix(".json"), sidecar)
        self.captures.append({"path": path.name, "case_id": sidecar["case_id"], "method": sidecar["method"], "sha256": sidecar["image_sha256"]})
        self.clock = time.monotonic()

    def verify(self):
        for case_index in self.saved.representative_indices:
            self.saved.select(case_index=case_index, method_index=0)
            self.set_camera()
            for method_index, method in enumerate(self.saved.methods):
                self.saved.select(method_index=method_index)
                self.refresh()
                self.action("End")
                name = self.saved.case["manifest"]["case_directory"]+"__"+method
                self.capture(name)
        self.saved.select(case_index=self.saved.representative_indices[0], method_index=0)
        self.set_camera()
        self.action("Reset")
        self.action("Play / Pause")
        deadline = time.monotonic()+.35
        while self.app.is_running() and time.monotonic() < deadline:
            self.tick()
        if self.saved.playing:
            self.action("Play / Pause")
        if self.saved.method["poses"] is not None and self.saved.index == 0:
            raise ValueError("saved counterfactual playback did not advance")
        self.action("Next Method")
        self.action("Next Case")
        self.saved.select(case_index=self.saved.representative_indices[0], method_index=0)
        self.set_camera()
        self.action("End")
        self.saved.verify_unchanged()
        expected = len(self.saved.representative_indices)*len(self.saved.methods)
        if len(self.captures) != expected:
            raise ValueError("representative/method screenshot coverage incomplete")
        dump_new(self.output/"runtime_validation.json", {"status": self.saved.runtime_status,
            "label": self.saved.label, "experiment_run": str(self.saved.run), "scene": self.scene,
            "representatives": self.saved.representative_report, "expected_screenshot_count": expected,
            "captures": self.captures, "controls": self.actions, "displayed_saved_samples": self.samples,
            "source_hashes": [{"path": str(p), "sha256": h} for p, h in self.saved.sources.items()],
            "source_unchanged": True, "display_playback_only": True, "new_inference_count": 0,
            "new_mpc_solves": 0, "generated_execution_states": 0,
            "sampling": "last saved 60Hz state at or before replay time; no interpolation or integration",
            "verification_method": "actual Isaac renderer and same callbacks as GUI controls; no OS mouse-click claim"})
        print(f"{self.saved.runtime_status} {self.output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    args = parser.parse_args()
    saved = SavedComparison(args.run)
    output = (args.output or ROOT/"data/robotless_gp_se2_comparison_replay"/datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")).resolve()
    if output.is_relative_to(saved.run) or output.is_relative_to(saved.source_run):
        raise ValueError("GUI evidence must use a separate new output directory")
    output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False, "window_width": 2400, "window_height": 1320,
        "width": 1600, "height": 1100, "renderer": "RayTracedLighting"})
    try:
        gui = ComparisonGui(saved, app, output)
        if args.verify:
            gui.verify()
        started = time.monotonic()
        while app.is_running():
            gui.tick()
            if args.no_hold and not saved.playing and time.monotonic()-started > 2.:
                break
        saved.verify_unchanged()
    finally:
        app.close()


if __name__ == "__main__":
    main()
