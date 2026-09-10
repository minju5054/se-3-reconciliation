#!/usr/bin/env python3
"""Show a high-motion DATA-02 transition by replaying saved poses only."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.data02_high_motion_demo import (  # noqa: E402
    DEFAULT_DEMO_OUTPUT_RELATIVE,
    DEFAULT_DURATION_S,
    DEFAULT_SELECTION_OUTPUT_RELATIVE,
    ReplayEvidence,
    compute_camera_framing,
    interpolated_pose_at,
    load_replay_evidence,
    phase_at,
    phase_schedule,
    saved_only_contract,
    saved_time_for_presentation,
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--episode")
    parser.add_argument("--transition", type=int)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--hold", action="store_true", help="hold final view until Isaac closes")
    parser.add_argument("--show-rgb", action="store_true", help="show saved observation RGB inset")
    return parser.parse_args()


ARGS = arguments()


def _default_selection() -> tuple[Path, str, int]:
    path = ROOT / DEFAULT_SELECTION_OUTPUT_RELATIVE / "selected_transition.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("clearly_higher_motion_than_previous_default") is not True:
        raise RuntimeError("default high-motion selection did not pass its comparison gate")
    selected = document["selected"]
    return Path(selected["source_run_path"]), str(selected["episode_id"]), int(
        selected["transition_index"]
    )


default_run, default_episode, default_transition = _default_selection()
RUN = (ARGS.run or default_run).expanduser().resolve()
EPISODE = ARGS.episode or default_episode
TRANSITION = ARGS.transition if ARGS.transition is not None else default_transition

from isaacsim import SimulationApp  # noqa: E402


APP = SimulationApp({"headless": False, "width": 1440, "height": 900})

import numpy as np  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path  # noqa: E402
from isaacsim.util.debug_draw import _debug_draw  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

from debug_draw_trajectories import (  # noqa: E402
    draw_heading_markers,
    draw_polyline,
    draw_pose_points,
    rgba,
)
from lightnav_stage0c_runtime import (  # noqa: E402
    resolve_jackal_asset,
    suppress_sensor_viewport_visualization,
)


OLD_COLOR = (0.10, 0.45, 1.0, 1.0)
FRESH_COLOR = (0.95, 0.15, 0.85, 1.0)
ACTUAL_COLOR = (0.05, 1.0, 0.18, 1.0)
OBSERVATION_COLOR = (1.0, 0.90, 0.05, 1.0)
P_COLOR = (1.0, 0.42, 0.05, 1.0)
BOUNDARY_COLOR = (1.0, 0.05, 0.05, 1.0)


def load_config(run: Path) -> dict[str, Any]:
    value = yaml.safe_load((run / "config_snapshot.yaml").read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("DATA-02 config snapshot must be a mapping")
    return value


def resolved_asset(relative: str) -> str:
    assets_root = str(get_assets_root_path()).rstrip("/")
    result = resolve_asset_path(relative) or resolve_asset_path(assets_root + relative)
    if not result:
        raise RuntimeError(f"Isaac asset unavailable: {relative}")
    return str(result)


def create_saved_display(
    config: Mapping[str, Any], evidence: ReplayEvidence
) -> tuple[World, UsdGeom.XformCommonAPI]:
    """Load Hospital and Jackal once; replay itself never advances physics."""

    dt = float(config["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    hospital = resolved_asset(str(config["environment"]["asset_relative_path"]))
    add_reference_to_stage(hospital, str(config["environment"]["reference_prim_path"]))
    jackal = resolve_jackal_asset(config["robot"])
    stage = omni.usd.get_context().get_stage()
    display_root_path = "/World/Data02HighMotionJackalPose"
    display_root = UsdGeom.Xform.Define(stage, display_root_path)
    robot_reference = f"{display_root_path}/JackalReference"
    add_reference_to_stage(str(jackal["resolved_path"]), robot_reference)
    while is_stage_loading():
        APP.update()
    # Initialize rendering once. The replay moves a parent USD transform only;
    # no articulation state, wheel command, or simulation step is used.
    world.reset()
    suppress_sensor_viewport_visualization(robot_reference)
    return world, UsdGeom.XformCommonAPI(display_root.GetPrim())


def configure_view(config: Mapping[str, Any], evidence: ReplayEvidence):
    framing = compute_camera_framing(
        evidence.actual_poses, evidence.old_world, evidence.fresh_world
    )
    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Data02HighMotionDomeLight")
    dome.CreateIntensityAttr(1450.0)
    key = UsdLux.SphereLight.Define(stage, "/World/Data02HighMotionKeyLight")
    key.CreateIntensityAttr(34000.0)
    key.CreateRadiusAttr(0.45)
    UsdGeom.XformCommonAPI(key).SetTranslate(Gf.Vec3d(*framing.eye_xyz))
    set_camera_view(
        eye=list(framing.eye_xyz),
        target=list(framing.target_xyz),
        camera_prim_path="/OmniverseKit_Persp",
    )
    camera = UsdGeom.Camera(stage.GetPrimAtPath("/OmniverseKit_Persp"))
    aperture = 20.955
    focal = aperture / (2.0 * math.tan(math.radians(framing.horizontal_fov_deg) / 2.0))
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateFocalLengthAttr(focal)
    print(
        "DATA02_HIGH_MOTION_CAMERA="
        f"eye={list(framing.eye_xyz)} target={list(framing.target_xyz)} "
        f"actual_fraction={framing.estimated_actual_viewport_fraction:.3f} "
        f"all_geometry_fraction={framing.estimated_all_geometry_viewport_fraction:.3f}",
        flush=True,
    )
    return framing


def _footprint_segments(pose: np.ndarray) -> tuple[list[list[float]], list[list[float]]]:
    half_length, half_width = 0.31, 0.26
    local = np.array(
        [
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ]
    )
    c, s = math.cos(float(pose[2])), math.sin(float(pose[2]))
    rotation = np.array([[c, -s], [s, c]])
    world_xy = local @ rotation.T + pose[:2]
    points = [[float(x), float(y), 0.31] for x, y in world_xy]
    return points, points[1:] + points[:1]


def draw_scene(
    draw,
    evidence: ReplayEvidence,
    display_pose: np.ndarray,
    lower_index: int,
    phase_key: str,
) -> None:
    draw.clear_lines()
    draw.clear_points()
    draw_polyline(draw, evidence.old_world, z=0.19, color=OLD_COLOR, width=8.0)
    trail = np.vstack((evidence.actual_poses[: lower_index + 1], display_pose))
    draw_polyline(draw, trail, z=0.25, color=ACTUAL_COLOR, width=9.0)
    draw_pose_points(draw, display_pose[None, :], z=0.29, color=ACTUAL_COLOR, size=17.0)

    if phase_key != "OLD_EXECUTING":
        draw_pose_points(
            draw,
            evidence.observation_pose[None, :],
            z=0.32,
            color=OBSERVATION_COLOR,
            size=27.0,
        )
        draw_heading_markers(
            draw,
            evidence.observation_pose[None, :],
            z=0.34,
            color=OBSERVATION_COLOR,
            width=8.0,
            length_m=0.48,
        )
        starts, ends = _footprint_segments(evidence.observation_pose)
        draw.draw_lines(
            starts,
            ends,
            [rgba(OBSERVATION_COLOR)] * len(starts),
            [7.0] * len(starts),
        )
        draw.draw_lines(
            [[float(evidence.observation_pose[0]), float(evidence.observation_pose[1]), 0.30]],
            [[float(display_pose[0]), float(display_pose[1]), 0.30]],
            [rgba((1.0, 0.9, 0.05, 0.85))],
            [4.0],
        )

    if phase_key in ("FRESH_READY_SWITCH", "FRESH_ACTIVE"):
        draw_polyline(draw, evidence.fresh_world, z=0.22, color=FRESH_COLOR, width=9.0)
        draw_heading_markers(
            draw,
            evidence.fresh_world,
            z=0.25,
            color=FRESH_COLOR,
            width=4.0,
            length_m=0.20,
        )
        for pose, color in ((evidence.p_pose, P_COLOR), (evidence.boundary_pose, BOUNDARY_COLOR)):
            draw_pose_points(draw, pose[None, :], z=0.36, color=color, size=28.0)
            draw_heading_markers(
                draw, pose[None, :], z=0.38, color=color, width=7.0, length_m=0.42
            )


def set_robot_pose(
    robot_transform: UsdGeom.XformCommonAPI, config: Mapping[str, Any], pose: np.ndarray
) -> None:
    robot_transform.SetTranslate(
        Gf.Vec3d(
            float(pose[0]),
            float(pose[1]),
            float(config["simulation"]["spawn_height_m"]),
        )
    )
    robot_transform.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(float(pose[2]))))


def create_output_directory() -> Path:
    root = ROOT / DEFAULT_DEMO_OUTPUT_RELATIVE
    root.mkdir(parents=True, exist_ok=True)
    stem = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = root / stem
    output.mkdir(exist_ok=False)
    return output


def capture(
    path: Path,
    world: World,
    robot_transform: UsdGeom.XformCommonAPI,
    config: Mapping[str, Any],
    evidence: ReplayEvidence,
    draw,
    display,
    phase_key: str,
) -> None:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    started = time.monotonic()
    deadline = started + 20.0
    while (not path.is_file() or time.monotonic() - started < 0.25) and time.monotonic() < deadline:
        set_robot_pose(robot_transform, config, display.pose)
        draw_scene(draw, evidence, display.pose, display.lower_index, phase_key)
        world.render()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError(f"viewport capture failed: {path}")
    with Image.open(path) as image:
        extrema = image.convert("RGB").getextrema()
    if all(maximum == 0 for _, maximum in extrema):
        raise RuntimeError(f"viewport capture is fully black: {path}")


def run_demo(config: Mapping[str, Any], evidence: ReplayEvidence) -> None:
    contract = saved_only_contract()
    if not all(
        (
            contract["saved_only"],
            not contract["lightnav_inference"],
            not contract["controller_execution"],
            not contract["physics_reexecution"],
            not contract["spatial_scaling"],
        )
    ):
        raise RuntimeError("saved-only execution contract changed")
    world, robot_transform = create_saved_display(config, evidence)
    framing = configure_view(config, evidence)
    draw = _debug_draw.acquire_debug_draw_interface()
    output = create_output_directory()

    panel = ui.Window(
        "DATA-02 saved high-motion example", width=690, height=300, position_x=16, position_y=46
    )
    with panel.frame:
        with ui.VStack(spacing=7, style={"margin": 11}):
            ui.Label("DATA-02 COLLECTION — SAVED HIGH-MOTION EXAMPLE", style={"font_size": 23})
            ui.Label("SAVED-DATA REPLAY  |  PRESENTATION SPEED: PHASE-MAPPED")
            phase_label = ui.Label("PHASE 1 — OLD EXECUTING", style={"font_size": 27})
            detail_label = ui.Label("saved pre-observation approach", style={"font_size": 17})
            moved_label = ui.Label("MOVED DURING FRESH INFERENCE: 0.000 m", style={"font_size": 28})
            displacement_label = ui.Label("observation → current: Δ = 0.000 m", style={"font_size": 19})
            ui.Label("BLUE OLD  |  MAGENTA raw FRESH  |  GREEN saved actual")
            ui.Label("YELLOW FRESH OBSERVATION  |  ORANGE P  |  RED B")
            timing_label = ui.Label("presentation 0.0 s  |  saved sim time initializing")
            ui.Label("Saved replay — no LightNav inference / no controller / no physics re-execution")

    rgb_panel = None
    if ARGS.show_rgb:
        if evidence.observation_rgb_path is None:
            raise ValueError("--show-rgb requested but saved observation RGB is unavailable")
        rgb_panel = ui.Window(
            "SAVED FRESH OBSERVATION RGB", width=350, height=250, position_x=1060, position_y=48
        )
        rgb_panel.visible = False
        with rgb_panel.frame:
            with ui.VStack(spacing=4, style={"margin": 8}):
                ui.Label("Exact saved FRESH observation", style={"font_size": 18})
                ui.Image(
                    f"file:{evidence.observation_rgb_path}",
                    width=320,
                    height=180,
                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                )
                ui.Label(f"saved frame {evidence.observation_rgb_frame_index}")

    initial = evidence.actual_poses[0]
    set_robot_pose(robot_transform, config, initial)
    draw_scene(draw, evidence, initial, 0, "OLD_EXECUTING")
    for _ in range(35):
        set_robot_pose(robot_transform, config, initial)
        world.render()

    capture_plan = (
        ("01_before_observation.png", 0, 0.75),
        ("02_during_inference.png", 1, 0.70),
        ("03_after_switch.png", 3, 0.60),
    )
    phases = phase_schedule(ARGS.duration)
    capture_targets = {
        name: phases[phase_index].start_s
        + fraction * (phases[phase_index].end_s - phases[phase_index].start_s)
        for name, phase_index, fraction in capture_plan
    }
    captures: list[dict[str, Any]] = []
    last_phase = ""
    started = time.monotonic()
    capture_pause_s = 0.0
    while APP.is_running():
        elapsed = min(time.monotonic() - started - capture_pause_s, float(ARGS.duration))
        phase = phase_at(elapsed, ARGS.duration)
        saved_time = saved_time_for_presentation(elapsed, ARGS.duration, evidence.candidate)
        display = interpolated_pose_at(evidence, saved_time)
        set_robot_pose(robot_transform, config, display.pose)
        draw_scene(draw, evidence, display.pose, display.lower_index, phase.key)
        if phase.key != last_phase:
            print(f"DATA02_HIGH_MOTION_PHASE={phase.key}", flush=True)
            last_phase = phase.key
        if rgb_panel is not None and phase.key != "OLD_EXECUTING":
            rgb_panel.visible = True

        displacement = float(np.linalg.norm(display.pose[:2] - evidence.observation_pose[:2]))
        inference_motion = (
            0.0
            if phase.key == "OLD_EXECUTING"
            else displacement
            if phase.key == "FRESH_INFERENCE"
            else evidence.candidate.inference_translation_m
        )
        phase_label.text = phase.title
        detail_label.text = phase.subtitle
        moved_label.text = f"MOVED DURING FRESH INFERENCE: {inference_motion:.3f} m"
        displacement_label.text = f"observation → current: Δ = {displacement:.3f} m"
        timing_label.text = (
            f"presentation {elapsed:4.1f}/{ARGS.duration:.1f} s  |  "
            f"saved sim time {saved_time:.3f} s  |  DISPLAY INTERPOLATION"
        )
        world.render()

        pending = next(
            (
                (name, target)
                for name, target in capture_targets.items()
                if target <= elapsed and not any(row["file"] == name for row in captures)
            ),
            None,
        )
        if pending is not None:
            name, target = pending
            pause_started = time.monotonic()
            capture(
                output / name,
                world,
                robot_transform,
                config,
                evidence,
                draw,
                display,
                phase.key,
            )
            capture_pause_s += time.monotonic() - pause_started
            captures.append(
                {
                    "file": name,
                    "target_presentation_time_s": target,
                    "actual_presentation_time_s": elapsed,
                    "saved_sim_time_s": saved_time,
                    "saved_lower_index": display.lower_index,
                    "saved_upper_index": display.upper_index,
                    "saved_lower_pose_world_se2": evidence.actual_poses[
                        display.lower_index
                    ].tolist(),
                    "saved_upper_pose_world_se2": evidence.actual_poses[
                        display.upper_index
                    ].tolist(),
                    "display_interpolation_alpha": display.alpha,
                    "displayed_pose_world_se2": display.pose.tolist(),
                }
            )
            print(
                f"DATA02_HIGH_MOTION_CAPTURE={output / name} pose={display.pose.tolist()}",
                flush=True,
            )

        if elapsed >= float(ARGS.duration):
            break
        time.sleep(0.01)

    if len(captures) != 3:
        raise RuntimeError(f"expected three phase captures, got {len(captures)}")
    pose_xy = [np.asarray(row["displayed_pose_world_se2"][:2]) for row in captures]
    d_before_during = float(np.linalg.norm(pose_xy[1] - pose_xy[0]))
    d_during_after = float(np.linalg.norm(pose_xy[2] - pose_xy[1]))
    changed = max(d_before_during, d_during_after) > 1e-6
    substantial_threshold = 0.20 * evidence.candidate.full_demo_path_length_m
    substantial = max(d_before_during, d_during_after) >= substantial_threshold
    manifest = {
        "schema": "DATA02HighMotionDemoCapture_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "SAVED-DATA REPLAY",
        "presentation_speed": "piecewise mapping over four presentation phases",
        "display_interpolation": "linear XY and shortest-angle yaw between adjacent saved samples",
        "no_physics_no_inference": True,
        "source": asdict(evidence.candidate),
        "source_sha256": dict(evidence.source_sha256),
        "duration_s": float(ARGS.duration),
        "replay_window_sim_s": [
            evidence.candidate.replay_start_sim_s,
            evidence.candidate.replay_end_sim_s,
        ],
        "camera": asdict(framing),
        "captures": captures,
        "d_before_during_m": d_before_during,
        "d_during_after_m": d_during_after,
        "displayed_robot_pose_changed": changed,
        "substantial_stage_displacement_threshold_m": substantial_threshold,
        "substantial_stage_displacement_verified": substantial,
    }
    (output / "capture_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not changed or not substantial:
        raise RuntimeError("captured displayed robot motion was not substantial")
    print(f"DATA02_HIGH_MOTION_D_BEFORE_DURING_M={d_before_during:.6f}", flush=True)
    print(f"DATA02_HIGH_MOTION_D_DURING_AFTER_M={d_during_after:.6f}", flush=True)
    print("DATA02_HIGH_MOTION_DISPLAYED_POSE_CHANGED=true", flush=True)
    print(f"DATA02_HIGH_MOTION_OUTPUT={output}", flush=True)

    if ARGS.hold:
        print("DATA02_HIGH_MOTION_HOLD=close Isaac Sim to exit", flush=True)
        final_display = interpolated_pose_at(evidence, evidence.actual_sim_times_s[-1])
        while APP.is_running():
            set_robot_pose(robot_transform, config, final_display.pose)
            draw_scene(
                draw, evidence, final_display.pose, final_display.lower_index, "FRESH_ACTIVE"
            )
            world.render()


def main() -> None:
    config = load_config(RUN)
    evidence = load_replay_evidence(RUN, EPISODE, TRANSITION)
    print("DATA02_HIGH_MOTION_MODE=SAVED-DATA REPLAY", flush=True)
    print("DATA02_HIGH_MOTION_PRESENTATION_SPEED=piecewise four-phase", flush=True)
    print("DATA02_HIGH_MOTION_EXECUTION=NO PHYSICS / NO INFERENCE / NO CONTROLLER", flush=True)
    print(
        f"DATA02_HIGH_MOTION_SELECTION={RUN} {EPISODE} transition={TRANSITION}", flush=True
    )
    print(
        "DATA02_HIGH_MOTION_LEGEND=blue OLD; magenta raw FRESH; green saved actual; "
        "yellow FRESH observation; orange P; red B",
        flush=True,
    )
    print(
        f"DATA02_HIGH_MOTION_INFERENCE_MOVEMENT_M="
        f"{evidence.candidate.inference_translation_m:.6f}",
        flush=True,
    )
    run_demo(config, evidence)


try:
    main()
finally:
    APP.close()
