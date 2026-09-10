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
    hold = parser.add_mutually_exclusive_group()
    hold.add_argument(
        "--hold",
        dest="hold",
        action="store_true",
        help="hold the final B view until Isaac closes (default)",
    )
    hold.add_argument(
        "--no-hold",
        dest="hold",
        action="store_false",
        help="automation only: close after captures",
    )
    parser.set_defaults(hold=True)
    parser.add_argument("--show-rgb", action="store_true", help="show saved observation RGB inset")
    return parser.parse_args()


ARGS = arguments()


def _default_selection() -> tuple[Path, str, int]:
    path = ROOT / DEFAULT_SELECTION_OUTPUT_RELATIVE / "selected_transition.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("more_active_old_motion_than_pre_correction_default") is not True:
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
    draw_polyline,
    draw_pose_points,
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

    if phase_key != "OLD_ACTIVE":
        draw_polyline(draw, evidence.fresh_world, z=0.22, color=FRESH_COLOR, width=9.0)
        draw_pose_points(
            draw,
            evidence.observation_pose[None, :],
            z=0.32,
            color=OBSERVATION_COLOR,
            size=20.0,
        )

    if phase_key == "AT_B":
        for pose, color in ((evidence.p_pose, P_COLOR), (evidence.boundary_pose, BOUNDARY_COLOR)):
            draw_pose_points(draw, pose[None, :], z=0.36, color=color, size=22.0)


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
            ui.Label("SAVED ACTUAL OLD-ACTIVE REPLAY  |  PRESENTATION SPEED != SCIENTIFIC TIME")
            phase_label = ui.Label("PHASE 1 — CURRENT OLD ACTIVE", style={"font_size": 27})
            detail_label = ui.Label("saved motion after this OLD became active", style={"font_size": 17})
            moved_label = ui.Label("MOVED SINCE FRESH OBSERVATION: 0.000 m", style={"font_size": 28})
            displacement_label = ui.Label("OLD-active path shown: 0.000 m", style={"font_size": 19})
            ui.Label("BLUE current OLD  |  MAGENTA raw FRESH  |  GREEN current-OLD actual")
            ui.Label("YELLOW FRESH OBSERVATION  |  ORANGE P  |  RED B")
            timing_label = ui.Label("presentation 0.0 s  |  saved sim time initializing")
            ui.Label("No previous/post-switch actual | no inference/controller/physics re-execution")

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
    draw_scene(draw, evidence, initial, 0, "OLD_ACTIVE")
    for _ in range(35):
        set_robot_pose(robot_transform, config, initial)
        world.render()

    phases = phase_schedule(ARGS.duration)
    capture_targets = {
        "01_old_active.png": {
            "presentation_time_s": phases[0].start_s
            + 0.65 * (phases[0].end_s - phases[0].start_s),
            "saved_time_s": None,
            "phase_key": "OLD_ACTIVE",
        },
        "02_at_observation.png": {
            "presentation_time_s": phases[1].start_s,
            "saved_time_s": evidence.candidate.t_obs_sim_s,
            "phase_key": "FRESH_INFERENCE_OLD_ACTIVE",
        },
        "03_at_B.png": {
            "presentation_time_s": phases[2].start_s,
            "saved_time_s": evidence.candidate.t_switch_sim_s,
            "phase_key": "AT_B",
        },
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
        if rgb_panel is not None and phase.key != "OLD_ACTIVE":
            rgb_panel.visible = True

        displacement = float(np.linalg.norm(display.pose[:2] - evidence.observation_pose[:2]))
        motion_since_observation = 0.0 if saved_time < evidence.candidate.t_obs_sim_s else displacement
        phase_label.text = phase.title
        detail_label.text = (
            "FRESH ready; current OLD remains active until B"
            if phase.key == "FRESH_INFERENCE_OLD_ACTIVE"
            and saved_time > evidence.candidate.t_ready_sim_s + 1e-9
            else phase.subtitle
        )
        moved_label.text = f"MOVED SINCE FRESH OBSERVATION: {motion_since_observation:.3f} m"
        shown_path = evidence.actual_poses[: display.lower_index + 1]
        displacement_label.text = (
            f"OLD-active path shown: "
            f"{float(np.linalg.norm(np.diff(shown_path[:, :2], axis=0), axis=1).sum()):.3f} m"
        )
        timing_label.text = (
            f"presentation {elapsed:4.1f}/{ARGS.duration:.1f} s  |  "
            f"saved sim time {saved_time:.3f} s  |  DISPLAY INTERPOLATION"
        )
        world.render()

        pending = next(
            (
                (name, target)
                for name, target in capture_targets.items()
                if float(target["presentation_time_s"]) <= elapsed
                and not any(row["file"] == name for row in captures)
            ),
            None,
        )
        if pending is not None:
            name, target = pending
            capture_saved_time = (
                saved_time
                if target["saved_time_s"] is None
                else float(target["saved_time_s"])
            )
            capture_display = interpolated_pose_at(evidence, capture_saved_time)
            set_robot_pose(robot_transform, config, capture_display.pose)
            draw_scene(
                draw,
                evidence,
                capture_display.pose,
                capture_display.lower_index,
                str(target["phase_key"]),
            )
            pause_started = time.monotonic()
            capture(
                output / name,
                world,
                robot_transform,
                config,
                evidence,
                draw,
                capture_display,
                str(target["phase_key"]),
            )
            capture_pause_s += time.monotonic() - pause_started
            captures.append(
                {
                    "file": name,
                    "target_presentation_time_s": target["presentation_time_s"],
                    "actual_presentation_time_s": elapsed,
                    "saved_sim_time_s": capture_saved_time,
                    "saved_lower_index": capture_display.lower_index,
                    "saved_upper_index": capture_display.upper_index,
                    "saved_lower_pose_world_se2": evidence.actual_poses[
                        capture_display.lower_index
                    ].tolist(),
                    "saved_upper_pose_world_se2": evidence.actual_poses[
                        capture_display.upper_index
                    ].tolist(),
                    "display_interpolation_alpha": capture_display.alpha,
                    "displayed_pose_world_se2": capture_display.pose.tolist(),
                    "phase": target["phase_key"],
                }
            )
            print(
                f"DATA02_HIGH_MOTION_CAPTURE={output / name} "
                f"pose={capture_display.pose.tolist()}",
                flush=True,
            )

        if elapsed >= float(ARGS.duration):
            break
        time.sleep(0.01)

    if len(captures) != 3:
        raise RuntimeError(f"expected three phase captures, got {len(captures)}")
    pose_xy = [np.asarray(row["displayed_pose_world_se2"][:2]) for row in captures]
    d_old_to_observation = float(np.linalg.norm(pose_xy[1] - pose_xy[0]))
    d_observation_to_b = float(np.linalg.norm(pose_xy[2] - pose_xy[1]))
    changed = max(d_old_to_observation, d_observation_to_b) > 1e-6
    substantial_threshold = 0.20 * evidence.candidate.active_old_path_length_m
    substantial = max(d_old_to_observation, d_observation_to_b) >= substantial_threshold
    manifest = {
        "schema": "DATA02HighMotionDemoCapture_v2",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "SAVED-DATA REPLAY",
        "presentation_speed": "piecewise mapping over current-OLD active/inference/B phases",
        "display_interpolation": "linear XY and shortest-angle yaw between adjacent saved samples",
        "no_physics_no_inference": True,
        "source": asdict(evidence.candidate),
        "source_sha256": dict(evidence.source_sha256),
        "duration_s": float(ARGS.duration),
        "active_old_interval_sim_s": [
            evidence.candidate.old_active_start_sim_s,
            evidence.candidate.old_active_end_sim_s,
        ],
        "actual_primary_source": "selected transition actual.npy and telemetry.csv",
        "activation_source": evidence.candidate.old_activation_source,
        "activation_boundary_prepended": evidence.candidate.activation_boundary_prepended,
        "display_contains_previous_chunk_actual": False,
        "display_contains_post_switch_actual": False,
        "every_telemetry_row_active_chunk_id": evidence.candidate.old_chunk_id,
        "display_ends_at_saved_B": bool(
            np.allclose(evidence.actual_poses[-1], evidence.boundary_pose, rtol=0.0, atol=1e-6)
        ),
        "camera": asdict(framing),
        "captures": captures,
        "d_old_capture_to_observation_m": d_old_to_observation,
        "d_observation_to_B_m": d_observation_to_b,
        "displayed_robot_pose_changed": changed,
        "substantial_stage_displacement_threshold_m": substantial_threshold,
        "substantial_stage_displacement_verified": substantial,
    }
    (output / "capture_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not changed or not substantial:
        raise RuntimeError("captured displayed robot motion was not substantial")
    print(f"DATA02_HIGH_MOTION_D_OLD_TO_OBSERVATION_M={d_old_to_observation:.6f}", flush=True)
    print(f"DATA02_HIGH_MOTION_D_OBSERVATION_TO_B_M={d_observation_to_b:.6f}", flush=True)
    print("DATA02_HIGH_MOTION_DISPLAYED_POSE_CHANGED=true", flush=True)
    print(f"DATA02_HIGH_MOTION_OUTPUT={output}", flush=True)

    if ARGS.hold:
        print("DATA02_HIGH_MOTION_HOLD=close Isaac Sim to exit", flush=True)
        final_display = interpolated_pose_at(evidence, evidence.actual_sim_times_s[-1])
        while APP.is_running():
            set_robot_pose(robot_transform, config, final_display.pose)
            draw_scene(
                draw, evidence, final_display.pose, final_display.lower_index, "AT_B"
            )
            world.render()


def main() -> None:
    config = load_config(RUN)
    evidence = load_replay_evidence(RUN, EPISODE, TRANSITION)
    print("DATA02_HIGH_MOTION_MODE=SAVED ACTUAL OLD-ACTIVE REPLAY", flush=True)
    print("DATA02_HIGH_MOTION_PRESENTATION_SPEED=not scientific time", flush=True)
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
        "DATA02_HIGH_MOTION_ACTIVE_OLD="
        + json.dumps(
            {
                "old_chunk_id": evidence.candidate.old_chunk_id,
                "start_sim_time_s": evidence.candidate.old_active_start_sim_s,
                "observation_sim_time_s": evidence.candidate.t_obs_sim_s,
                "switch_sim_time_s": evidence.candidate.t_switch_sim_s,
                "path_length_m": evidence.candidate.active_old_path_length_m,
                "previous_chunk_actual_displayed": False,
                "post_switch_actual_displayed": False,
            },
            sort_keys=True,
        ),
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
