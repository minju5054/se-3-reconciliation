#!/usr/bin/env python3
"""Overview and per-tau Isaac projection views of immutable saved geometry."""

import argparse
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
import numpy as np

import robotless_projection_artifacts as artifacts
from reconciliation.robotless_projection_handoff import characterize_projection
from reconciliation.robotless_single_chunk import load_config, make_observation_time, observation_to_world, save_json_exclusive, sha256_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--view-only", action="store_true")
    args = parser.parse_args()
    run = args.run_directory.resolve()
    config, record, loaded = artifacts.load_inputs(run)
    _, previous_config, _, successive, metadata, arrays, boundaries, previous_metrics = loaded
    rows = characterize_projection(boundaries, arrays["FRESH_world"], previous_metrics["conditions"], previous_config["motion"],
        distance_atol_m=config["numerical_validation"]["previous_d_poly_atol_m"])
    metrics_path = run / "derived/projection_metrics.json"
    metrics = artifacts.read_json(metrics_path)
    if metrics["conditions"] != rows or metrics["source_json_sha256"] != sha256_file(run / "source.json"):
        raise ValueError("saved projection metrics disagree with frozen-source recomputation")
    image_paths = ["evidence/isaac_projection_overview.png"] + [f"evidence/tau_{i:03d}_detail.png" for i in range(len(rows))]
    if not args.view_only and any((run / p).exists() for p in ["visualization.json", *image_paths]):
        raise FileExistsError("existing evidence is immutable; use --view-only")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless})
    try:
        import omni.timeline
        import omni.ui as ui
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.util.debug_draw import _debug_draw
        from debug_draw_trajectories import draw_polyline, draw_pose_points
        from robotless_runtime import actual_pose, runtime_scene, viewport_capture

        R_obs = np.asarray(metadata["R1"])
        _, agent, _, _, scene = runtime_scene(load_config(successive / "config_snapshot.yaml"), R_obs, app=app)
        before = actual_pose(agent)
        visual = config["visualization"]
        draw = _debug_draw.acquire_debug_draw_interface()
        z, width, size = visual["path_z_m"], visual["line_width"], visual["point_size"]

        def arrow(xy, angle, prefix):
            height, length = visual[f"{prefix}_z_m"], visual[f"{prefix}_length_m"]
            color = visual[{"incoming": "incoming_color", "tangent": "tangent_color", "pose_yaw": "pose_yaw_color"}[prefix]]
            start = [*xy, height]
            end = [xy[0] + length * math.cos(angle), xy[1] + length * math.sin(angle), height]
            head = visual["arrowhead_length_m"]
            ends = [[end[0] + head * math.cos(angle + sign * 5 * math.pi / 6),
                     end[1] + head * math.sin(angle + sign * 5 * math.pi / 6), height] for sign in (-1, 1)]
            draw.draw_lines([start, end, end], [end, *ends], [color] * 3, [width] * 3)

        def show(index=None):
            draw.clear_lines()
            draw.clear_points()
            for name, color in (("OLD_world", "old_color"), ("FRESH_world", "fresh_color")):
                draw_polyline(draw, arrays[name], z=z, color=visual[color], width=width)
            selected = rows if index is None else [rows[index]]
            for row in selected:
                b, q = row["B_world"], row["Q_xy_world_m"]
                draw_polyline(draw, np.array([b[:2], q]), z=z+.01, color=visual["connector_color"], width=width)
                draw_pose_points(draw, np.array([b]), z=z+.01, color=visual["boundary_color"], size=size)
                draw_pose_points(draw, np.array([q]), z=z+.01, color=visual["projection_color"], size=size)
                if index is not None:
                    # Display-only vertical stems expose common XY origins of layered arrows.
                    draw.draw_lines([[*b[:2], z], [*q, z]], [[*b[:2], visual["incoming_z_m"]], [*q, visual["pose_yaw_z_m"]]],
                                    [visual["boundary_color"], visual["projection_color"]], [1.5, 1.5])
                    arrow(b[:2], row["phi_in_rad"], "incoming")
                    if row["tangent_available"]:
                        arrow(q, row["phi_F_rad"], "tangent")
                    arrow(q, row["theta_Q_rad"], "pose_yaw")
            prefix = "overview" if index is None else "detail"
            offsets = np.array([visual[f"{prefix}_eye_local_m"], visual[f"{prefix}_target_local_m"]])
            anchor = R_obs if index is None else np.asarray(rows[index]["B_world"])
            xy = observation_to_world(anchor, np.column_stack((offsets[:, :2], np.zeros(2))))
            set_camera_view(eye=[*xy[0, :2], offsets[0, 2]], target=[*xy[1, :2], offsets[1, 2]], camera_prim_path="/OmniverseKit_Persp")

        panel = ui.Window("Projection handoff geometry", width=530, height=305)
        with panel.frame:
            with ui.VStack(spacing=5):
                ui.Label("MAGENTA: fixed FRESH | BLUE: OLD context", height=24)
                ui.Label("YELLOW: B / incoming | WHITE: Q | GREEN: B to Q", height=24)
                ui.Label("CYAN: path tangent | ORANGE: interpolated pose yaw", height=24)
                ui.Label("Arrows use height layers; XY and angles are unchanged", height=24)
                ui.Label("Controlled delay, not actual LightNav latency", height=24)
                ui.Button("Overview (all B and Q)", clicked_fn=lambda: show(None), height=28)
                with ui.HStack(height=30):
                    for i, row in enumerate(rows):
                        ui.Button(f"tau={row['tau_s']:g} s", clicked_fn=lambda index=i: show(index))
                ui.Label("Detail views show three direction arrows for the selected tau", height=25)
        if not args.view_only:
            for index, relative in zip([None, *range(len(rows))], image_paths, strict=True):
                show(index)
                viewport_capture(run / relative, app=app)
        show(None)
        for _ in range(30):
            app.update()
        after = actual_pose(agent)
        timeline = omni.timeline.get_timeline_interface()
        if not np.array_equal(before, after) or timeline.is_playing() or timeline.get_current_time() != 0:
            raise RuntimeError("logical agent or stopped timeline changed")
        artifacts.load_source(record)
        if not args.view_only:
            save_json_exclusive(run / "visualization.json", {
                "created_time": make_observation_time(0), "isaac_viewport_rendered": True,
                "source_json_sha256": sha256_file(run / "source.json"), "metrics_sha256": sha256_file(metrics_path),
                "config_sha256": sha256_file(run / "config_snapshot.yaml"), "new_lightnav_inference_count": 0,
                "scene": scene, "agent_pose_before": before.tolist(), "agent_pose_after": after.tolist(), "timeline_time_s": 0,
                "fresh_world_drawn": arrays["FRESH_world"].tolist(), "boundaries_world": boundaries.tolist(),
                "geometry_scaling": 1.0, "scene_visibility_modifications": [], "fresh_reanchored": False,
                "display_layers": {k: visual[k] for k in ("path_z_m", "incoming_z_m", "tangent_z_m", "pose_yaw_z_m")},
                "display_convention": "Only drawing heights differ; all arrow origins retain B/Q XY and actual angles; no angular magnification",
                "screenshots": [artifacts.file_record(run / p, run) for p in image_paths],
                "details": [{k: row[k] for k in ("tau_s", "B_world", "Q_xy_world_m", "phi_in_rad", "phi_F_rad", "theta_Q_rad", "tangent_available")} for row in rows],
                "legend_scope": "Separate UI panel with overview and tau detail buttons; viewport images contain geometry only",
                "research_source_sha256": {str(p.relative_to(ROOT)): sha256_file(p) for p in (Path(__file__), ROOT / "scripts/isaac/robotless_runtime.py", ROOT / "scripts/isaac/debug_draw_trajectories.py")},
            })
        print("PROJECTION_VIEW_ONLY_OK" if args.view_only else "PROJECTION_ISAAC_OVERVIEW_AND_DETAILS_SAVED", flush=True)
        if not args.no_hold and not args.headless:
            while app.is_running() and panel.visible:
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
