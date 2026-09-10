#!/usr/bin/env python3
"""Explain a frozen EXP-02D representative with saved Jackal motion in Isaac."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import textwrap
import time
import traceback
from typing import Any, Mapping, Sequence

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.exp02d_gui import (  # noqa: E402
    PHASE_A_FRACTION,
    REPRESENTATIVE_CASES,
    Exp02DGuiCase,
    compute_exp02d_camera_plan,
    gui_phase_and_saved_time,
    latest_completed_run,
    load_exp02d_gui_case,
    saved_only_gui_contract,
    saved_pose_at,
    validate_gui_capture_rgb,
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        type=Path,
        help="completed EXP-02D primary run (default: latest completed run)",
    )
    parser.add_argument("--case", required=True, choices=REPRESENTATIVE_CASES)
    parser.add_argument(
        "--duration",
        type=float,
        help="presentation duration in seconds; does not alter saved scientific time",
    )
    parser.add_argument(
        "--show-m2",
        action="store_true",
        help="also show the cyan M2 no-direction diagnostic candidate",
    )
    parser.add_argument(
        "--show-entry-vector",
        action="store_true",
        help="also show the dashed B-to-F_k historical entry vector",
    )
    hold = parser.add_mutually_exclusive_group()
    hold.add_argument(
        "--hold",
        dest="hold",
        action="store_true",
        help="hold the final comparison until Isaac closes (default from config)",
    )
    hold.add_argument(
        "--no-hold",
        dest="hold",
        action="store_false",
        help="automation only: close after all four captures",
    )
    parser.set_defaults(hold=None)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="render/capture without opening a window; normally pair with --no-hold",
    )
    return parser.parse_args()


ARGS = arguments()
RUN = (
    ARGS.run.expanduser().resolve()
    if ARGS.run is not None
    else latest_completed_run(REPOSITORY_ROOT / "data/exp02d_lookahead_direction")
)
EVIDENCE = load_exp02d_gui_case(RUN, ARGS.case)
GUI_RENDER_CONFIG_PATH = REPOSITORY_ROOT / "configs/exp02d_lookahead_direction.yaml"
GUI_RENDER_DOCUMENT = yaml.safe_load(GUI_RENDER_CONFIG_PATH.read_text(encoding="utf-8"))
if (
    not isinstance(GUI_RENDER_DOCUMENT, Mapping)
    or GUI_RENDER_DOCUMENT.get("experiment") != "EXP-02D"
    or not isinstance(GUI_RENDER_DOCUMENT.get("gui"), Mapping)
):
    raise ValueError("repository EXP-02D GUI render config is invalid")
GUI_CONFIG = dict(GUI_RENDER_DOCUMENT["gui"])
DURATION_S = float(
    GUI_CONFIG["presentation_duration_s"] if ARGS.duration is None else ARGS.duration
)
if not math.isfinite(DURATION_S) or DURATION_S <= 0.0:
    raise ValueError("--duration must be finite and positive")
HOLD = bool(GUI_CONFIG["hold_by_default"]) if ARGS.hold is None else bool(ARGS.hold)


from isaacsim import SimulationApp  # noqa: E402


APP = SimulationApp(
    {"headless": ARGS.headless, "width": 1440, "height": 900}
)

import numpy as np  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402
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
    rgba,
)
from lightnav_stage0c_runtime import (  # noqa: E402
    resolve_jackal_asset,
    suppress_sensor_viewport_visualization,
)
from reconciliation.online_switch import (  # noqa: E402
    save_json_exclusive,
    sha256_file,
)


MODE_ACTUAL = "PHASE_A_ACTUAL_OLD_ACTIVE"
MODE_RAW = "PHASE_B_RAW_AT_B"
MODE_COMPARISON = "PHASE_B_OFFLINE_CANDIDATES_AT_B"


def strict_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def gui_processing_provenance(evidence: Exp02DGuiCase) -> dict[str, Any]:
    """Record exact post-primary renderer inputs independently of Git state."""

    artifacts = {
        "gui_runner": Path(__file__).resolve(),
        "gui_helper": REPOSITORY_ROOT / "src/reconciliation/exp02d_gui.py",
        "gui_launcher": REPOSITORY_ROOT
        / "scripts/isaac/run_exp02d_success_failure_gui.sh",
    }
    for name, path in artifacts.items():
        if not path.is_file():
            raise RuntimeError(f"EXP-02D GUI processing artifact is missing: {name}: {path}")
    quantitative_config = evidence.run / "config_snapshot.yaml"
    quantitative_config_hash = sha256_file(quantitative_config)
    if quantitative_config_hash != evidence.result_sha256["config_snapshot.yaml"]:
        raise RuntimeError("EXP-02D primary config changed before GUI rendering")
    return {
        "derivation": "post-primary saved-evidence rendering only",
        "processing_artifact_sha256": {
            name: sha256_file(path) for name, path in sorted(artifacts.items())
        },
        "quantitative_config_source": str(quantitative_config),
        "quantitative_config_source_sha256": quantitative_config_hash,
        "gui_render_config_source": str(GUI_RENDER_CONFIG_PATH),
        "gui_render_config_source_sha256": sha256_file(GUI_RENDER_CONFIG_PATH),
        "runtime_gui_mapping": json.loads(json.dumps(GUI_CONFIG, sort_keys=True)),
        "effective_presentation_duration_s": DURATION_S,
        "effective_hold": HOLD,
        "cli_flags": {
            "headless": bool(ARGS.headless),
            "show_m2": bool(ARGS.show_m2),
            "show_entry_vector": bool(ARGS.show_entry_vector),
        },
        "note": (
            "quantitative evidence comes from the immutable primary snapshot; "
            "camera-only post-processing comes from the separately hashed repository config"
        ),
    }


def source_scene_config(evidence: Exp02DGuiCase) -> dict[str, Any]:
    path = evidence.active_old.run / "config_snapshot.yaml"
    digest = sha256_file(path)
    if digest != evidence.source_sha256["config_snapshot.yaml"]:
        raise ValueError("source DATA-02 scene config changed after GUI evidence loading")
    value = strict_yaml(path)
    for key in ("simulation", "robot", "environment"):
        if not isinstance(value.get(key), Mapping):
            raise ValueError(f"source DATA-02 config lacks {key}")
    return value


def resolved_asset(relative: str) -> str:
    assets_root = str(get_assets_root_path()).rstrip("/")
    result = resolve_asset_path(relative) or resolve_asset_path(assets_root + relative)
    if not result:
        raise RuntimeError(f"Isaac asset unavailable: {relative}")
    return str(result)


def create_saved_display(
    config: Mapping[str, Any], evidence: Exp02DGuiCase
) -> tuple[World, UsdGeom.XformCommonAPI, dict[str, Any]]:
    """Create a visual-only Hospital/Jackal stage; saved replay never steps physics."""

    dt = float(config["simulation"]["physics_dt"])
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    hospital = resolved_asset(str(config["environment"]["asset_relative_path"]))
    add_reference_to_stage(hospital, str(config["environment"]["reference_prim_path"]))

    jackal = resolve_jackal_asset(config["robot"])
    stage = omni.usd.get_context().get_stage()
    display_root_path = "/World/Exp02DSavedActualJackal"
    display_root = UsdGeom.Xform.Define(stage, display_root_path)
    reference_path = display_root_path + "/JackalReference"
    add_reference_to_stage(str(jackal["resolved_path"]), reference_path)

    dome = UsdLux.DomeLight.Define(stage, "/World/Exp02DSavedGuiDomeLight")
    dome.CreateIntensityAttr(1450.0)
    key = UsdLux.SphereLight.Define(stage, "/World/Exp02DSavedGuiKeyLight")
    key.CreateIntensityAttr(34000.0)
    key.CreateRadiusAttr(0.45)

    while is_stage_loading():
        if not APP.is_running():
            raise RuntimeError("Isaac Sim closed while loading EXP-02D assets")
        APP.update()
    world.reset()
    sensor_override = suppress_sensor_viewport_visualization(reference_path)
    return (
        world,
        UsdGeom.XformCommonAPI(display_root.GetPrim()),
        {
            "environment_asset": hospital,
            "environment_reference_prim_path": str(
                config["environment"]["reference_prim_path"]
            ),
            "jackal_asset": str(jackal["resolved_path"]),
            "jackal_reference_prim_path": reference_path,
            "sensor_viewport_override": sensor_override,
            "visual_only_parent_transform": True,
            "articulation_initialized": False,
            "physics_stepped": False,
        },
    )


def configure_camera(evidence: Exp02DGuiCase) -> dict[str, Any]:
    base_z = float(GUI_CONFIG["z_offset_m"])
    old_path_z = float(GUI_CONFIG.get("old_path_visual_z_m", 0.78))
    actual_ground_z = base_z + 0.035
    actual_visual_z = float(GUI_CONFIG.get("actual_path_visual_z_m", 0.80))
    candidate_z = [
        float(GUI_CONFIG.get("raw_path_visual_z_m", 0.84)),
        float(GUI_CONFIG.get("historical_m4_path_visual_z_m", 0.88)),
        float(GUI_CONFIG.get("lookahead_path_visual_z_m", 0.92)),
    ]
    candidate_paths = [
        evidence.selected_raw_fresh,
        evidence.method_candidates["M1_HISTORICAL_M4"],
        evidence.method_candidates["M3_LOOKAHEAD"],
    ]
    if ARGS.show_m2:
        candidate_paths.append(evidence.method_candidates["M2_NO_DIRECTION"])
        candidate_z.append(float(GUI_CONFIG.get("no_direction_path_visual_z_m", 0.90)))
    sectors = GUI_CONFIG.get("camera_case_view_sectors", {})
    sector = sectors.get(evidence.case, {}) if isinstance(sectors, Mapping) else {}
    if not isinstance(sector, Mapping):
        raise ValueError(f"EXP-02D camera sector for {evidence.case} must be a mapping")
    fov_deg = float(GUI_CONFIG["camera_horizontal_fov_deg"])
    target_fraction = float(GUI_CONFIG["camera_geometry_fraction"])
    minimum_slant_distance = float(
        GUI_CONFIG.get("camera_minimum_slant_distance_m", 1.80)
    )
    maximum_eye_height = float(
        GUI_CONFIG.get("camera_indoor_max_eye_height_m", 1.85)
    )
    plan = compute_exp02d_camera_plan(
        evidence.active_old.old_world,
        evidence.active_old.display_actual_poses,
        candidate_paths,
        horizontal_fov_deg=fov_deg,
        candidate_geometry_fraction=target_fraction,
        viewport_aspect_ratio=float(
            GUI_CONFIG.get("camera_viewport_aspect_ratio", 1.60)
        ),
        minimum_slant_distance_m=minimum_slant_distance,
        minimum_back_distance_m=float(
            GUI_CONFIG.get("camera_minimum_back_distance_m", 1.55)
        ),
        eye_height_m=float(GUI_CONFIG.get("camera_eye_height_m", 1.45)),
        minimum_eye_height_m=float(
            GUI_CONFIG.get("camera_indoor_min_eye_height_m", 1.15)
        ),
        maximum_eye_height_m=maximum_eye_height,
        target_z_m=float(GUI_CONFIG.get("camera_target_z_m", 0.25)),
        lateral_offset_m=float(GUI_CONFIG.get("camera_lateral_offset_m", 0.25)),
        azimuth_coarse_step_deg=float(
            GUI_CONFIG.get("camera_azimuth_coarse_step_deg", 5.0)
        ),
        azimuth_fine_step_deg=float(
            GUI_CONFIG.get("camera_azimuth_fine_step_deg", 0.25)
        ),
        containment_limit_ndc=float(
            GUI_CONFIG.get("camera_containment_limit_ndc", 0.95)
        ),
        minimum_depth_m=float(GUI_CONFIG.get("camera_minimum_depth_m", 0.15)),
        old_path_z_m=old_path_z,
        actual_path_ground_z_m=actual_ground_z,
        actual_path_visual_z_m=actual_visual_z,
        candidate_path_z_m=candidate_z,
        observation_pose_world_se2=(
            evidence.active_old.observation_pose_world_se2
        ),
        observation_marker_visual_z_m=float(
            GUI_CONFIG.get("observation_marker_visual_z_m", 0.88)
        ),
        preferred_eye_direction_xy=GUI_CONFIG.get(
            "camera_preferred_eye_direction_world_xy", [1.0, -1.0]
        )
        if "preferred_eye_direction_world_xy" not in sector
        else sector["preferred_eye_direction_world_xy"],
        preferred_eye_half_angle_deg=float(
            sector.get(
                "preferred_eye_half_angle_deg",
                GUI_CONFIG.get("camera_preferred_eye_half_angle_deg", 90.0),
            )
        ),
    )
    eye = list(plan.eye_xyz)
    target = list(plan.target_xyz)
    stage = omni.usd.get_context().get_stage()
    camera_path = "/World/Exp02DPresentationCamera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_xform = UsdGeom.Xformable(camera.GetPrim())
    camera_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    camera_xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    aperture = 20.955
    vertical_aperture = aperture / plan.viewport_aspect_ratio
    focal = aperture / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    horizontal_set = camera.CreateHorizontalApertureAttr().Set(aperture)
    vertical_set = camera.CreateVerticalApertureAttr().Set(vertical_aperture)
    focal_set = camera.CreateFocalLengthAttr().Set(focal)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable for camera setup")
    viewport.camera_path = camera.GetPath()
    set_camera_view(
        eye=eye,
        target=target,
        camera_prim_path=camera_path,
        viewport_api=viewport,
    )
    read_horizontal_aperture = float(camera.GetHorizontalApertureAttr().Get())
    read_vertical_aperture = float(camera.GetVerticalApertureAttr().Get())
    read_focal = float(camera.GetFocalLengthAttr().Get())
    read_horizontal_fov = math.degrees(
        2.0 * math.atan(read_horizontal_aperture / (2.0 * read_focal))
    )
    read_vertical_fov = math.degrees(
        2.0 * math.atan(read_vertical_aperture / (2.0 * read_focal))
    )
    print(
        "EXP02D_GUI_CAMERA_LENS="
        f"authored={horizontal_set}/{vertical_set}/{focal_set} "
        f"apertures={read_horizontal_aperture:.9f}/{read_vertical_aperture:.9f} "
        f"focal_length={read_focal:.9f} "
        f"horizontal_fov_deg={read_horizontal_fov:.9f} "
        f"vertical_fov_deg={read_vertical_fov:.9f}",
        flush=True,
    )
    if not math.isclose(
        read_horizontal_fov, plan.horizontal_fov_deg, rel_tol=0.0, abs_tol=1e-5
    ) or not math.isclose(
        read_vertical_fov, plan.vertical_fov_deg, rel_tol=0.0, abs_tol=1e-5
    ):
        raise RuntimeError(
            "USD camera lens readback differs from EXP-02D projection plan: "
            f"{read_horizontal_fov}/{read_vertical_fov} deg"
        )
    camera_projection = {
        "model": "USD_pinhole_fixed_apertures",
        "camera_prim_path": camera_path,
        "viewport_resolution_px": [1440, 900],
        "horizontal_aperture": read_horizontal_aperture,
        "vertical_aperture": read_vertical_aperture,
        "focal_length": read_focal,
        "derived_horizontal_fov_deg": read_horizontal_fov,
        "derived_vertical_fov_deg": read_vertical_fov,
        "matches_projection_plan": True,
    }
    light = stage.GetPrimAtPath("/World/Exp02DSavedGuiKeyLight")
    if light.IsValid():
        UsdGeom.XformCommonAPI(light).SetTranslate(Gf.Vec3d(*eye))
    fill_path = "/World/Exp02DSavedGuiFillLight"
    fill = UsdLux.SphereLight.Define(stage, fill_path)
    fill_intensity = float(GUI_CONFIG.get("camera_target_fill_intensity", 18000.0))
    fill_height = float(GUI_CONFIG.get("camera_target_fill_height_m", 1.60))
    fill.CreateIntensityAttr(fill_intensity)
    fill.CreateRadiusAttr(0.40)
    fill_xyz = [float(target[0]), float(target[1]), fill_height]
    UsdGeom.XformCommonAPI(fill.GetPrim()).SetTranslate(Gf.Vec3d(*fill_xyz))
    legacy_height_min = GUI_CONFIG.get("camera_height_min_m")
    if legacy_height_min is not None:
        print(
            "EXP02D_GUI_CAMERA_CORRECTION="
            "legacy 3-D minimum height is not applied because it placed the camera "
            "above the Hospital ceiling; indoor eye-height cap is active",
            flush=True,
        )
    result = {
        "eye_xyz": eye,
        "target_xyz": target,
        "horizontal_fov_deg": plan.horizontal_fov_deg,
        "vertical_fov_deg": plan.vertical_fov_deg,
        "viewport_aspect_ratio": plan.viewport_aspect_ratio,
        "elevation_deg": plan.elevation_deg,
        "slant_distance_m": plan.slant_distance_m,
        "back_distance_m": plan.back_distance_m,
        "lateral_offset_m": plan.lateral_offset_m,
        "nominal_view_direction_xy": list(plan.nominal_view_direction_xy),
        "eye_direction_xy": list(plan.eye_direction_xy),
        "preferred_eye_direction_world_xy": (
            None
            if plan.preferred_eye_direction_xy is None
            else list(plan.preferred_eye_direction_xy)
        ),
        "preferred_eye_half_angle_deg": plan.preferred_eye_half_angle_deg,
        "case_view_sector": json.loads(json.dumps(sector, sort_keys=True)),
        "azimuth_world_deg": plan.azimuth_world_deg,
        "azimuth_offset_from_initial_heading_deg": (
            plan.azimuth_offset_from_initial_heading_deg
        ),
        "azimuth_search_coarse_step_deg": plan.azimuth_search_coarse_step_deg,
        "azimuth_search_fine_step_deg": plan.azimuth_search_fine_step_deg,
        "azimuth_candidates_evaluated": plan.azimuth_candidates_evaluated,
        "minimum_slant_distance_m": plan.minimum_slant_distance_m,
        "minimum_back_distance_m": plan.minimum_back_distance_m,
        "indoor_maximum_eye_height_m": plan.maximum_eye_height_m,
        "containment_limit_ndc": plan.containment_limit_ndc,
        "maximum_projected_ndc": plan.maximum_projected_ndc,
        "minimum_projected_depth_m": plan.minimum_projected_depth_m,
        "projection": camera_projection,
        "requested_candidate_geometry_fraction": plan.requested_candidate_geometry_fraction,
        "projected_candidate_geometry_fraction": plan.projected_candidate_geometry_fraction,
        "projected_focus_geometry_fraction": plan.projected_focus_geometry_fraction,
        "candidate_fraction_target_achieved": plan.candidate_fraction_target_achieved,
        "full_context_and_jackal_contained": plan.full_context_and_jackal_contained,
        "limiting_constraint": plan.limiting_constraint,
        "candidate_bounds_xy_m": [list(row) for row in plan.candidate_bounds_xy],
        "focus_bounds_xy_m": [list(row) for row in plan.focus_bounds_xy],
        "drawn_bounds_xy_m": [list(row) for row in plan.drawn_bounds_xy],
        "framing_scope": "saved active-OLD interval plus offline candidates",
        "full_old_still_drawn": True,
        "rendered_path_z_m": {
            "old": plan.old_path_z_m,
            "actual_ground": plan.actual_path_ground_z_m,
            "actual_visual_duplicate": plan.actual_path_visual_z_m,
            "observation_marker": plan.observation_marker_visual_z_m,
            "candidate_path_order": [
                "M0_RAW",
                "M1_HISTORICAL_M4",
                "M3_LOOKAHEAD",
            ]
            + (["M2_NO_DIRECTION"] if ARGS.show_m2 else []),
            "candidate_paths_in_projection_order": list(plan.candidate_path_z_m),
        },
        "legacy_camera_height_min_m": legacy_height_min,
        "legacy_height_min_applied": False,
        "legacy_override_reason": (
            "3-D minimum put the viewport above/inside the Hospital ceiling"
            if legacy_height_min is not None
            else None
        ),
        "stable_elevated_oblique": True,
        "spatial_scaling": False,
        "target_fill_light": {
            "prim_path": fill_path,
            "intensity": fill_intensity,
            "position_world_xyz": fill_xyz,
            "visualization_only": True,
        },
    }
    print("EXP02D_GUI_CAMERA=" + json.dumps(result, sort_keys=True), flush=True)
    return result


def set_robot_pose(
    transform: UsdGeom.XformCommonAPI,
    source_config: Mapping[str, Any],
    pose: np.ndarray,
) -> None:
    transform.SetTranslate(
        Gf.Vec3d(
            float(pose[0]),
            float(pose[1]),
            float(source_config["simulation"]["spawn_height_m"]),
        )
    )
    transform.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(float(pose[2]))))


def draw_arrow(
    draw,
    start: Sequence[float],
    end: Sequence[float],
    *,
    z: float,
    color: Sequence[float],
    width: float,
    dashed: bool = False,
) -> None:
    start_xy = np.asarray(start[:2], dtype=np.float64)
    end_xy = np.asarray(end[:2], dtype=np.float64)
    delta = end_xy - start_xy
    length = float(np.linalg.norm(delta))
    if not math.isfinite(length) or length <= 1e-9:
        return
    direction = delta / length
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    starts: list[list[float]] = []
    ends: list[list[float]] = []
    if dashed:
        dash_count = max(2, int(math.ceil(length / 0.10)))
        for index in range(dash_count):
            first = index / dash_count
            second = min(first + 0.55 / dash_count, 1.0)
            a = start_xy + first * delta
            b = start_xy + second * delta
            starts.append([float(a[0]), float(a[1]), float(z)])
            ends.append([float(b[0]), float(b[1]), float(z)])
    else:
        starts.append([float(start_xy[0]), float(start_xy[1]), float(z)])
        ends.append([float(end_xy[0]), float(end_xy[1]), float(z)])
    head_length = min(0.16, max(0.06, 0.28 * length))
    head_width = 0.55 * head_length
    for sign in (-1.0, 1.0):
        tip_base = end_xy - head_length * direction + sign * head_width * normal
        starts.append([float(end_xy[0]), float(end_xy[1]), float(z)])
        ends.append([float(tip_base[0]), float(tip_base[1]), float(z)])
    draw.draw_lines(
        starts,
        ends,
        [rgba(color)] * len(starts),
        [float(width)] * len(starts),
    )


def actual_trail(evidence: Exp02DGuiCase, display) -> np.ndarray:
    values = evidence.active_old.display_actual_poses[: display.lower_index + 1]
    if not np.allclose(values[-1], display.pose, rtol=0.0, atol=1e-12):
        values = np.vstack((values, display.pose))
    return values


def draw_observation_pin(draw, pose: np.ndarray, *, z: float, color, size: float) -> None:
    """Draw one compact visual-only cross centered on the exact observation XY."""

    x, y = (float(pose[0]), float(pose[1]))
    arm = 0.025
    starts = [
        [x - arm, y, z],
        [x, y - arm, z],
    ]
    ends = [
        [x + arm, y, z],
        [x, y + arm, z],
    ]
    draw.draw_lines(
        starts,
        ends,
        [rgba(color)] * len(starts),
        [max(4.0, 0.35 * float(size))] * len(starts),
    )
    draw.draw_points([[x, y, z]], [rgba(color)], [float(size)])


def draw_scene(
    draw,
    evidence: Exp02DGuiCase,
    display,
    mode: str,
    saved_time_s: float,
) -> None:
    visual = GUI_CONFIG
    colors = visual["colors"]
    z = float(visual["z_offset_m"])
    old_z = float(visual.get("old_path_visual_z_m", 0.78))
    actual_visual_z = float(visual.get("actual_path_visual_z_m", 0.80))
    raw_z = float(visual.get("raw_path_visual_z_m", 0.84))
    m1_z = float(visual.get("historical_m4_path_visual_z_m", 0.88))
    m2_z = float(visual.get("no_direction_path_visual_z_m", 0.90))
    m3_z = float(visual.get("lookahead_path_visual_z_m", 0.92))
    width = float(visual["line_width"])
    marker = float(visual["marker_size"])
    draw.clear_lines()
    draw.clear_points()
    draw_polyline(
        draw,
        evidence.active_old.old_world,
        z=old_z,
        color=colors["old"],
        width=width,
    )
    draw_polyline(
        draw,
        actual_trail(evidence, display),
        z=z + 0.035,
        color=colors["actual"],
        width=width + 1.0,
    )
    # The final part of a short truthful centerline lies beneath the opaque
    # Jackal.  Repeat the exact same XY as a thin elevated halo; no XY scaling
    # or shift is applied, and the GUI/manifest label it visual-only.
    draw_polyline(
        draw,
        actual_trail(evidence, display),
        z=actual_visual_z,
        color=colors["actual"],
        width=max(3.0, 0.45 * width),
    )
    if saved_time_s >= evidence.active_old.observation_sim_time_s - 1e-9:
        draw_observation_pin(
            draw,
            evidence.active_old.observation_pose_world_se2,
            z=float(visual.get("observation_marker_visual_z_m", 0.88)),
            color=colors["observation"],
            size=marker,
        )
    if mode not in (MODE_RAW, MODE_COMPARISON):
        return

    draw_polyline(
        draw,
        evidence.method_candidates["M0_RAW"],
        z=raw_z,
        color=colors["raw"],
        width=width,
    )
    if mode == MODE_COMPARISON:
        if ARGS.show_m2:
            draw_polyline(
                draw,
                evidence.method_candidates["M2_NO_DIRECTION"],
                z=m2_z,
                color=colors["no_direction"],
                width=width,
            )
        # M1 is intentionally wider beneath the thinner M3 so both colors
        # remain visible where paths overlap; neither path is shifted/scaled.
        draw_polyline(
            draw,
            evidence.method_candidates["M1_HISTORICAL_M4"],
            z=m1_z,
            color=colors["historical_m4"],
            width=width
            * float(visual.get("historical_m4_line_width_scale", 1.50)),
        )
        draw_polyline(
            draw,
            evidence.method_candidates["M3_LOOKAHEAD"],
            z=m3_z,
            color=colors["lookahead"],
            width=width * float(visual.get("lookahead_line_width_scale", 0.75)),
        )

    point_rows = (
        (evidence.active_old.p_pose_world_se2, colors["p"], 1.00),
        (evidence.active_old.boundary_pose_world_se2, colors["boundary"], 1.25),
        (evidence.f_k_world_se2, colors["entry"], 0.90),
        (evidence.f_q_world_se2, colors["lookahead_target"], 0.90),
    )
    for pose, color, scale in point_rows:
        draw_pose_points(
            draw,
            pose[None, :],
            z=z + 0.18,
            color=color,
            size=marker * scale,
        )
    draw_arrow(
        draw,
        evidence.active_old.p_pose_world_se2,
        evidence.active_old.boundary_pose_world_se2,
        z=z + 0.23,
        color=colors["p"],
        width=max(3.0, 0.65 * width),
    )
    draw_arrow(
        draw,
        evidence.active_old.boundary_pose_world_se2,
        evidence.f_q_world_se2,
        z=z + 0.25,
        color=colors["lookahead_target"],
        width=max(3.0, 0.65 * width),
    )
    if ARGS.show_entry_vector:
        draw_arrow(
            draw,
            evidence.active_old.boundary_pose_world_se2,
            evidence.f_k_world_se2,
            z=z + 0.27,
            color=colors["entry"],
            width=max(2.0, 0.48 * width),
            dashed=True,
        )


def mode_at(presentation_time_s: float) -> tuple[str, float]:
    phase, saved_time = gui_phase_and_saved_time(
        presentation_time_s, DURATION_S, EVIDENCE.active_old
    )
    if phase == MODE_ACTUAL:
        return MODE_ACTUAL, saved_time
    if presentation_time_s < 0.84 * DURATION_S:
        return MODE_RAW, saved_time
    return MODE_COMPARISON, saved_time


def create_panel(evidence: Exp02DGuiCase):
    metrics = evidence.representative["metrics"]
    window = ui.Window(
        "EXP-02D saved success/failure explanation",
        width=720,
        height=610,
        position_x=14,
        position_y=42,
    )
    with window.frame:
        with ui.VStack(spacing=5, style={"margin": 10}):
            ui.Label(f"EXP-02D {evidence.case}: {evidence.outcome_title}", style={"font_size": 24})
            ui.Label(
                "SAVED ACTUAL OLD-ACTIVE REPLAY  |  PRESENTATION SPEED != SCIENTIFIC TIME"
            )
            phase_label = ui.Label("PHASE A — WHAT ACTUALLY HAPPENED", style={"font_size": 25})
            execution_label = ui.Label(
                "Only saved physical OLD-active motion is moving", style={"font_size": 17}
            )
            timing_label = ui.Label("saved time initializing")
            pose_label = ui.Label("Jackal world SE(2) initializing")
            ui.Label("Command discontinuity at exact saved B", style={"font_size": 18})
            for method, title in (
                ("M0_RAW", "RAW"),
                ("M1_HISTORICAL_M4", "M1 HISTORICAL M4"),
                ("M3_LOOKAHEAD", "M3 LOOKAHEAD"),
            ):
                command = evidence.method_metrics[method]["command"]
                ui.Label(
                    f"{title:<18} |dv|={float(command['delta_v_abs_mps']):.3f} m/s  "
                    f"|dw|={float(command['delta_omega_abs_rps']):.3f} rad/s  "
                    f"J={float(command['J_cmd']):.3f}"
                )
            if ARGS.show_m2:
                command = evidence.method_metrics["M2_NO_DIRECTION"]["command"]
                ui.Label(
                    f"{'M2 NO-DIRECTION':<18} |dv|={float(command['delta_v_abs_mps']):.3f} m/s  "
                    f"|dw|={float(command['delta_omega_abs_rps']):.3f} rad/s  "
                    f"J={float(command['J_cmd']):.3f}"
                )
            ui.Label(
                "Geometry: "
                f"alpha_entry={math.degrees(float(metrics['alpha_entry_rad'])):.1f} deg  "
                f"alpha_look={math.degrees(float(metrics['alpha_look_rad'])):.1f} deg"
            )
            ui.Label(
                f"B->F_k={float(metrics['b_to_fresh_k_translation_m']):.3f} m  "
                f"B->F_q={float(metrics['b_to_fresh_q_translation_m']):.3f} m  "
                f"k={evidence.k_fresh}  q={evidence.q_fresh}"
            )
            ui.Label("BLUE OLD reference  |  GREEN saved OLD-active actual")
            ui.Label("raised line heights repeat exact SE(2) XY (visibility only; no scaling)")
            ui.Label("GRAY RAW[k:]  |  ORANGE M1  |  MAGENTA M3" + ("  |  CYAN M2" if ARGS.show_m2 else ""))
            ui.Label("points: YELLOW observation | ORANGE P | RED B | GRAY F_k | MAGENTA F_q")
            ui.Label("arrows: ORANGE P->B INCOMING ACTUAL MOTION | MAGENTA B->F_q RAW FOLLOWER LOOKAHEAD")
            if ARGS.show_entry_vector:
                ui.Label("optional dashed GRAY B->F_k HISTORICAL ENTRY VECTOR")
            ui.Label("Interpretation (frozen case rule):", style={"font_size": 17})
            for line in textwrap.wrap(evidence.interpretation, width=91):
                ui.Label(line)
            ui.Label("M1/M2/M3 are offline candidates — NOT PHYSICALLY EXECUTED")
            ui.Label("No previous-chunk actual | no post-switch actual | no controller/physics replay")
    return window, phase_label, execution_label, timing_label, pose_label


def update_panel(
    labels,
    mode: str,
    presentation_time_s: float,
    saved_time_s: float,
    pose: np.ndarray,
) -> None:
    phase_label, execution_label, timing_label, pose_label = labels
    if mode == MODE_ACTUAL:
        phase_label.text = "PHASE A — WHAT ACTUALLY HAPPENED"
        if saved_time_s < EVIDENCE.active_old.observation_sim_time_s - 1e-9:
            detail = "saved Jackal follows current OLD before FRESH observation"
        elif saved_time_s < EVIDENCE.active_old.model_ready_sim_time_s - 1e-9:
            detail = "FRESH inference in flight; saved Jackal remains on current OLD"
        else:
            detail = "FRESH ready; saved Jackal remains on current OLD until B"
        execution_label.text = detail
    elif mode == MODE_RAW:
        phase_label.text = "PHASE B1 — STOPPED AT B / RAW FRESH[k:]"
        execution_label.text = "Robot frozen at saved B; gray RAW is offline and not executed"
    else:
        phase_label.text = "PHASE B2 — WHAT THE OPTIMIZER WOULD DO"
        execution_label.text = "Robot frozen at saved B; M1/M3 candidates are offline and not executed"
    timing_label.text = (
        f"presentation {presentation_time_s:.2f}/{DURATION_S:.2f} s  |  "
        f"saved sim time {saved_time_s:.6f} s"
    )
    pose_label.text = (
        f"Jackal world SE(2)=[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]"
    )


def capture_view(
    path: Path,
    world: World,
    robot_transform: UsdGeom.XformCommonAPI,
    source_config: Mapping[str, Any],
    draw,
    display,
    mode: str,
    saved_time_s: float,
) -> dict[str, Any]:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    # Make the requested saved pose/phase visible to Hydra before asking the
    # asynchronous viewport capture service for an image.  Without this
    # warm-up, a fast capture can save the immediately preceding animation
    # frame (notably hiding the marker at exact t_obs).
    for _ in range(3):
        set_robot_pose(robot_transform, source_config, display.pose)
        draw_scene(draw, EVIDENCE, display, mode, saved_time_s)
        world.render()
    capture_viewport_to_file(viewport, file_path=str(path))
    started = time.monotonic()
    deadline = started + 20.0
    while (
        (not path.is_file() or time.monotonic() - started < 0.25)
        and time.monotonic() < deadline
    ):
        set_robot_pose(robot_transform, source_config, display.pose)
        draw_scene(draw, EVIDENCE, display, mode, saved_time_s)
        world.render()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError(f"EXP-02D viewport capture failed: {path}")
    with Image.open(path) as image:
        rgb = np.asarray(image.convert("RGB"))
    try:
        content = validate_gui_capture_rgb(
            rgb,
            required_palette=GUI_CONFIG["colors"],
            require_jackal=True,
            minimum_palette_pixels=0,
        )
        counts = content["palette_pixel_counts"]
        required = ["old", "actual"]
        if saved_time_s >= EVIDENCE.active_old.observation_sim_time_s - 1e-9:
            required.append("observation")
        missing = [name for name in required if counts[name] < 16]
        if missing:
            raise ValueError(
                "GUI capture lacks required visible path colors: "
                + ", ".join(missing)
            )
        return content
    except ValueError as error:
        raise RuntimeError(f"EXP-02D viewport capture failed visibility check: {path}: {error}") from error


def create_output_directory(run: Path, case: str) -> Path:
    root = run / "gui"
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = root / f"{case}_{stamp}"
    output.mkdir(exist_ok=False)
    return output


def capture_schedule(evidence: Exp02DGuiCase) -> list[dict[str, Any]]:
    active = evidence.active_old
    span = active.switch_sim_time_s - active.activation_sim_time_s
    obs_fraction = (
        (active.observation_sim_time_s - active.activation_sim_time_s) / span
        if span > 0.0
        else 0.0
    )
    observation_presentation = min(
        0.66 * DURATION_S,
        max(0.14 * DURATION_S, PHASE_A_FRACTION * obs_fraction * DURATION_S),
    )
    old_presentation = max(0.05 * DURATION_S, 0.50 * observation_presentation)
    old_saved_time = active.activation_sim_time_s + 0.50 * (
        active.observation_sim_time_s - active.activation_sim_time_s
    )
    return [
        {
            "file": "01_old_active.png",
            "presentation_time_s": old_presentation,
            "saved_time_s": old_saved_time,
            "mode": MODE_ACTUAL,
        },
        {
            "file": "02_at_observation.png",
            "presentation_time_s": observation_presentation,
            "saved_time_s": active.observation_sim_time_s,
            "mode": MODE_ACTUAL,
        },
        {
            "file": "03_at_B_raw.png",
            "presentation_time_s": 0.76 * DURATION_S,
            "saved_time_s": active.switch_sim_time_s,
            "mode": MODE_RAW,
        },
        {
            "file": "04_method_comparison.png",
            "presentation_time_s": 0.92 * DURATION_S,
            "saved_time_s": active.switch_sim_time_s,
            "mode": MODE_COMPARISON,
        },
    ]


def verify_display_contract(evidence: Exp02DGuiCase) -> None:
    active = evidence.active_old
    contract = saved_only_gui_contract()
    if not all(
        (
            contract["saved_primary_results_only"],
            not contract["lightnav_invoked"],
            not contract["optimizer_invoked"],
            not contract["controller_invoked"],
            not contract["physics_reexecution"],
            not contract["candidate_execution"],
            not contract["previous_chunk_actual_displayed"],
            not contract["post_switch_actual_displayed"],
        )
    ):
        raise RuntimeError("EXP-02D saved-only GUI contract changed")
    if any(row["active_chunk_id"] != active.old_chunk_id for row in active.telemetry_rows):
        raise RuntimeError("display buffer contains a different active chunk")
    if active.display_sim_times_s[0] != active.activation_sim_time_s:
        raise RuntimeError("display begins before/after the current OLD activation")
    if active.display_sim_times_s[-1] != active.switch_sim_time_s:
        raise RuntimeError("display does not end at the selected switch")
    if not np.allclose(
        active.display_actual_poses[-1],
        active.boundary_pose_world_se2,
        rtol=0.0,
        atol=1e-6,
    ):
        raise RuntimeError("displayed actual path does not end at saved B")


def run_gui(evidence: Exp02DGuiCase) -> None:
    verify_display_contract(evidence)
    source_config = source_scene_config(evidence)
    world, robot_transform, scene = create_saved_display(source_config, evidence)
    camera = configure_camera(evidence)
    output = create_output_directory(evidence.run, evidence.case)
    draw = _debug_draw.acquire_debug_draw_interface()
    panel, *labels = create_panel(evidence)
    _ = panel

    initial = saved_pose_at(evidence.active_old, evidence.active_old.activation_sim_time_s)
    set_robot_pose(robot_transform, source_config, initial.pose)
    draw_scene(
        draw,
        evidence,
        initial,
        MODE_ACTUAL,
        evidence.active_old.activation_sim_time_s,
    )
    update_panel(
        labels,
        MODE_ACTUAL,
        0.0,
        evidence.active_old.activation_sim_time_s,
        initial.pose,
    )
    for _ in range(35):
        set_robot_pose(robot_transform, source_config, initial.pose)
        world.render()

    targets = capture_schedule(evidence)
    captures: list[dict[str, Any]] = []
    started = time.monotonic()
    capture_pause_s = 0.0
    last_mode = ""
    while APP.is_running():
        elapsed = min(time.monotonic() - started - capture_pause_s, DURATION_S)
        mode, saved_time = mode_at(elapsed)
        display = saved_pose_at(evidence.active_old, saved_time)
        set_robot_pose(robot_transform, source_config, display.pose)
        draw_scene(draw, evidence, display, mode, saved_time)
        update_panel(labels, mode, elapsed, saved_time, display.pose)
        world.render()
        if mode != last_mode:
            print(f"EXP02D_GUI_PHASE={mode}", flush=True)
            last_mode = mode

        pending = next(
            (
                target
                for target in targets
                if float(target["presentation_time_s"]) <= elapsed
                and not any(row["file"] == target["file"] for row in captures)
            ),
            None,
        )
        if pending is not None:
            capture_saved_time = float(pending["saved_time_s"])
            capture_mode = str(pending["mode"])
            capture_display = saved_pose_at(evidence.active_old, capture_saved_time)
            set_robot_pose(robot_transform, source_config, capture_display.pose)
            draw_scene(
                draw,
                evidence,
                capture_display,
                capture_mode,
                capture_saved_time,
            )
            update_panel(
                labels,
                capture_mode,
                elapsed,
                capture_saved_time,
                capture_display.pose,
            )
            path = output / str(pending["file"])
            pause_started = time.monotonic()
            content_check = capture_view(
                path,
                world,
                robot_transform,
                source_config,
                draw,
                capture_display,
                capture_mode,
                capture_saved_time,
            )
            capture_pause_s += time.monotonic() - pause_started
            row = {
                **pending,
                "actual_presentation_time_s": elapsed,
                "displayed_pose_world_se2": capture_display.pose.tolist(),
                "saved_lower_index": capture_display.lower_index,
                "saved_upper_index": capture_display.upper_index,
                "display_interpolation_alpha": capture_display.alpha,
                "viewport_content_check": content_check,
                "sha256": sha256_file(path),
            }
            captures.append(row)
            print(f"EXP02D_GUI_CAPTURE={path}", flush=True)

        if elapsed >= DURATION_S:
            break
        time.sleep(0.01)

    if len(captures) != len(targets):
        raise RuntimeError(
            f"EXP-02D GUI closed before all captures: {len(captures)}/{len(targets)}"
        )
    raw_capture = next(row for row in captures if row["file"] == "03_at_B_raw.png")
    comparison_capture = next(
        row for row in captures if row["file"] == "04_method_comparison.png"
    )
    raw_palette = raw_capture["viewport_content_check"]["palette_pixel_counts"]
    comparison_palette = comparison_capture["viewport_content_check"][
        "palette_pixel_counts"
    ]
    for name in ("historical_m4", "lookahead"):
        gained = int(comparison_palette[name]) - int(raw_palette[name])
        if gained < 24:
            raise RuntimeError(
                f"EXP-02D final comparison does not visibly add {name}: "
                f"palette gain {gained} < 24 pixels"
            )
    active = evidence.active_old
    manifest = {
        "schema": "EXP02D_SavedSuccessFailureGuiCapture_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "case": evidence.case,
        "outcome_title": evidence.outcome_title,
        "corpus_transition_id": evidence.corpus_transition_id,
        "primary_run": str(evidence.run),
        "primary_result_manifest_sha256": sha256_file(
            evidence.run / "result_manifest.json"
        ),
        "processing_provenance": gui_processing_provenance(evidence),
        "source_run": str(active.run),
        "source_transition": str(active.transition_directory),
        "source_sha256": dict(evidence.source_sha256),
        "result_sha256": dict(evidence.result_sha256),
        "saved_primary_representative_only": True,
        "presentation_speed_is_scientific_time": False,
        "duration_s": DURATION_S,
        "old_chunk_id": active.old_chunk_id,
        "fresh_chunk_id": active.fresh_chunk_id,
        "active_old_interval_sim_s": [
            active.activation_sim_time_s,
            active.switch_sim_time_s,
        ],
        "activation_source": active.activation_source,
        "activation_boundary_prepended": active.activation_boundary_prepended,
        "active_old_telemetry_row_count": len(active.telemetry_rows),
        "active_old_display_pose_count": len(active.display_actual_poses),
        "active_old_path_length_m": active.active_old_path_length_m,
        "every_telemetry_row_active_chunk_id": all(
            row["active_chunk_id"] == active.old_chunk_id
            for row in active.telemetry_rows
        ),
        "display_starts_at_current_old_activation": True,
        "display_ends_at_saved_B": True,
        "previous_chunk_actual_displayed": False,
        "post_switch_actual_displayed": False,
        "candidate_execution": False,
        "lightnav_invoked": False,
        "optimizer_invoked": False,
        "controller_invoked": False,
        "physics_reexecution": False,
        "candidate_paths": {
            "M0_RAW": "offline/not executed",
            "M1_HISTORICAL_M4": "offline/not executed",
            "M2_NO_DIRECTION": "offline/not executed",
            "M3_LOOKAHEAD": "offline/not executed",
        },
        "default_visible_methods": ["M0_RAW", "M1_HISTORICAL_M4", "M3_LOOKAHEAD"],
        "m2_visible": bool(ARGS.show_m2),
        "entry_vector_visible": bool(ARGS.show_entry_vector),
        "k_fresh": evidence.k_fresh,
        "q_fresh": evidence.q_fresh,
        "P_world_se2": active.p_pose_world_se2.tolist(),
        "B_world_se2": active.boundary_pose_world_se2.tolist(),
        "F_k_world_se2": evidence.f_k_world_se2.tolist(),
        "F_q_world_se2": evidence.f_q_world_se2.tolist(),
        "camera": camera,
        "visualization": {
            "trajectory_xy_spatial_scaling": False,
            "rendered_path_z_m": camera["rendered_path_z_m"],
            "all_path_z_lifts_are_visual_only": True,
            "observation_marker_visual_z_m": float(
                GUI_CONFIG.get("observation_marker_visual_z_m", 0.88)
            ),
            "observation_marker_z_lift_is_visual_only": True,
            "actual_path_z_lift_is_visual_only": True,
            "method_widths_preserve_overlap_without_xy_shift": True,
        },
        "scene": scene,
        "captures": captures,
        "interpretation": evidence.interpretation,
        "claim_boundary": (
            "candidate desired-command/geometric evidence only; no physical candidate "
            "execution or navigation-success claim"
        ),
    }
    save_json_exclusive(output / "capture_manifest.json", manifest)
    print(f"EXP02D_GUI_OUTPUT={output}", flush=True)

    if HOLD:
        print("EXP02D_GUI_HOLD=close Isaac Sim to exit", flush=True)
        final = saved_pose_at(active, active.switch_sim_time_s)
        while APP.is_running():
            set_robot_pose(robot_transform, source_config, final.pose)
            draw_scene(
                draw,
                evidence,
                final,
                MODE_COMPARISON,
                active.switch_sim_time_s,
            )
            update_panel(
                labels,
                MODE_COMPARISON,
                DURATION_S,
                active.switch_sim_time_s,
                final.pose,
            )
            world.render()
            time.sleep(0.01)


def main() -> None:
    print(f"EXP02D_GUI_RUN={RUN}", flush=True)
    print(
        f"EXP02D_GUI_CASE={EVIDENCE.case} {EVIDENCE.corpus_transition_id}",
        flush=True,
    )
    print("EXP02D_GUI_MODE=SAVED ACTUAL OLD-ACTIVE REPLAY", flush=True)
    print("EXP02D_GUI_PRESENTATION_SPEED=not scientific time", flush=True)
    print(
        "EXP02D_GUI_EXECUTION=NO LIGHTNAV / NO OPTIMIZER / NO CONTROLLER / "
        "NO PHYSICS RE-EXECUTION / NO CANDIDATE EXECUTION",
        flush=True,
    )
    print(
        "EXP02D_GUI_LEGEND=blue OLD; green saved active-OLD actual; gray RAW; "
        "orange M1; magenta M3; yellow observation; orange P; red B; gray F_k; "
        "magenta F_q",
        flush=True,
    )
    run_gui(EVIDENCE)


try:
    main()
except BaseException:
    print("EXP02D_GUI_FATAL=see traceback below", file=sys.stderr, flush=True)
    traceback.print_exc()
    raise
finally:
    APP.close()
