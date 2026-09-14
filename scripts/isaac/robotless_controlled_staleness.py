#!/usr/bin/env python3
"""Static Isaac display of fixed saved paths and controlled boundary conditions."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

import numpy as np

from robotless_controlled_staleness_artifacts import file_record, load_inputs, verify_source
from reconciliation.robotless_single_chunk import load_config, make_observation_time, observation_to_world, save_json_exclusive, sha256_file


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--view-only", action="store_true")
    args = parser.parse_args()
    run = args.run_directory.resolve()
    config, record, source, metadata, arrays, boundaries = load_inputs(run)
    R_obs = np.asarray(metadata["R1"], dtype=float)
    motion, visual = config["motion"], config["visualization"]
    # Independent scalar translation check; metrics.json is never opened here.
    times = np.asarray(motion["tau_s"])
    expected = np.tile(R_obs, (len(times), 1))
    expected[:, 0] += motion["v_mps"] * times * np.cos(R_obs[2])
    expected[:, 1] += motion["v_mps"] * times * np.sin(R_obs[2])
    if not np.allclose(boundaries, expected, rtol=0, atol=1e-12) or not np.array_equal(boundaries[0], R_obs):
        raise ValueError("saved boundaries disagree with independent controlled-motion check")
    if not args.view_only and ((run / "visualization.json").exists() or (run / "evidence/isaac_handoff_overview.png").exists()):
        raise FileExistsError("existing visualization is immutable; use --view-only")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless})
    try:
        import omni.timeline
        import omni.ui as ui
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.util.debug_draw import _debug_draw
        from debug_draw_trajectories import draw_polyline, draw_pose_points
        from robotless_runtime import actual_pose, runtime_scene, viewport_capture

        scene_config = load_config(source / "config_snapshot.yaml")
        _, agent, _, _, scene = runtime_scene(scene_config, R_obs, app=app)
        before = actual_pose(agent)
        draw = _debug_draw.acquire_debug_draw_interface()
        draw.clear_lines()
        draw.clear_points()
        z_path, z_boundary = float(visual["trajectory_z_m"]), float(visual["boundary_z_m"])
        width, size = float(visual["line_width"]), float(visual["point_size"])
        for name, color_key in (("OLD_world", "old_color_rgba"), ("FRESH_world", "fresh_color_rgba")):
            # Only original rows form the polyline. No observation-to-entry connector.
            draw_polyline(draw, arrays[name], z=z_path, color=visual[color_key], width=width)
            draw_pose_points(draw, arrays[name], z=z_path, color=visual[color_key], size=size * .6)
        draw_polyline(draw, boundaries, z=z_boundary, color=visual["motion_color_rgba"], width=width)
        left = np.array([-np.sin(R_obs[2]), np.cos(R_obs[2])])
        half_tick = float(visual["boundary_tick_half_length_m"])
        for boundary, color in zip(boundaries, visual["boundary_colors_rgba"], strict=True):
            tick = np.tile(boundary, (2, 1))
            tick[:, :2] += np.array([-1, 1])[:, None] * half_tick * left
            draw_polyline(draw, tick, z=z_boundary, color=color, width=width)
            draw_pose_points(draw, boundary[None, :], z=z_boundary, color=color, size=size)
        offsets = np.asarray([visual["overview_eye_agent_offset_m"], visual["overview_target_agent_offset_m"]])
        view_xy = observation_to_world(R_obs, np.column_stack((offsets[:, :2], np.zeros(2))))
        set_camera_view(eye=[*view_xy[0, :2], offsets[0, 2]], target=[*view_xy[1, :2], offsets[1, 2]], camera_prim_path="/OmniverseKit_Persp")
        panel = ui.Window("Controlled staleness: fixed FRESH / moving B", width=540, height=280)
        with panel.frame:
            with ui.VStack(spacing=5):
                ui.Label("BLUE: saved OLD context | MAGENTA: fixed raw FRESH", height=25)
                ui.Label("GREEN: controlled body-forward path; not OLD execution", height=25)
                for tau, color in zip(times, visual["boundary_color_names"], strict=True):
                    text = f"{color.upper()}: B(tau={tau:g} s)" + (" = R_obs" if tau == 0 else "")
                    ui.Label(text, height=25)
                ui.Label(f"v={motion['v_mps']:g} m/s, omega=0 | tau is not measured RTT", height=25)
                ui.Label("No robot, controller or new model inference", height=25)
        image_path = run / "evidence/isaac_handoff_overview.png"
        if args.view_only:
            for _ in range(30):
                app.update()
        else:
            viewport_capture(image_path, app=app)
        after = actual_pose(agent)
        timeline = omni.timeline.get_timeline_interface()
        if not np.array_equal(before, after) or not np.allclose(after, R_obs, rtol=0, atol=1e-7) or timeline.is_playing() or timeline.get_current_time() != 0:
            raise RuntimeError("static logical agent or stopped timeline changed")
        verify_source(record)
        if not args.view_only:
            save_json_exclusive(run / "visualization.json", {
                "created_time": make_observation_time(0.0), "isaac_viewport_rendered": True,
                "source_json_sha256": sha256_file(run / "source.json"),
                "config_sha256": sha256_file(run / "config_snapshot.yaml"),
                "boundaries_input": file_record(run / "derived/boundaries.npy", run),
                "R_obs_world": R_obs.tolist(), "boundaries_world": boundaries.tolist(), "tau_s": times.tolist(),
                "agent_pose_before": before.tolist(), "agent_pose_after": after.tolist(), "timeline_time_s": 0.0,
                "trajectory_inputs": {k: record["array_inputs"][k] for k in ("OLD_world", "FRESH_world")},
                "drawn_trajectories": {k: arrays[k].tolist() for k in ("OLD_world", "FRESH_world")},
                "fresh_world_instances": 1, "observation_to_entry_connector": False, "metrics_file_used": False,
                "new_lightnav_inference_count": 0, "scene": scene, "geometry_scaling": 1.0,
                "scene_visibility_modifications": [], "display_z_offsets_m": {"trajectories": z_path, "boundaries": z_boundary},
                "display_offsets_scope": "height only for visibility; source XY/yaw unchanged",
                "boundary_colors": visual["boundary_color_names"], "boundary_marker_count": len(boundaries),
                "legend": "separate Isaac UI window; viewport PNG has colored geometry only",
                "screenshot": file_record(image_path, run),
                "research_source_sha256": {str(p.relative_to(ROOT)): sha256_file(p) for p in (
                    Path(__file__), ROOT / "scripts/isaac/robotless_runtime.py", ROOT / "scripts/isaac/debug_draw_trajectories.py",
                    ROOT / "scripts/robotless_controlled_staleness_artifacts.py", ROOT / "src/reconciliation/se2.py")},
            })
        print("CONTROLLED_STALENESS_VIEW_ONLY_OK" if args.view_only else "CONTROLLED_STALENESS_ISAAC_VISUALIZATION_SAVED", flush=True)
        if not args.no_hold and not args.headless:
            while app.is_running() and panel.visible:
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
