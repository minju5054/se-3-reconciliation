#!/usr/bin/env python3
"""Isaac GUI for saved EXP-02C factor-isolation artifacts and a static Jackal reference."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--view",
        choices=("real_benign_k0", "synthetic_s4"),
        default="real_benign_k0",
    )
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="automation only: close after saving the viewport capture",
    )
    parser.add_argument("--headless", action="store_true", help="render validation only")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import omni.usd
import yaml
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, UsdGeom, UsdLux

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from lightnav_stage0c_runtime import resolve_jackal_asset
from reconciliation.online_switch import load_strict_json, save_json_exclusive, sha256_file


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_repository_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def add_static_boundary_jackal(
    config: dict[str, Any], boundary: np.ndarray, boundary_kind: str
) -> dict[str, Any]:
    """Load the official Jackal as a non-executing visual reference at view boundary B."""

    exp01b_config_path = resolve_repository_path(config["paths"]["exp01b_config"])
    exp01b_config = load_yaml(exp01b_config_path)
    robot_config = exp01b_config["robot"]
    asset = resolve_jackal_asset(robot_config)
    spawn_height = float(exp01b_config["simulation"]["spawn_height_m"])
    wrapper_path = "/World/Exp02CStaticBoundaryJackal"
    reference_path = wrapper_path + "/Asset"
    stage = omni.usd.get_context().get_stage()
    wrapper = UsdGeom.Xform.Define(stage, wrapper_path)
    wrapper.AddTranslateOp().Set(
        Gf.Vec3d(float(boundary[0]), float(boundary[1]), spawn_height)
    )
    wrapper.AddRotateZOp().Set(math.degrees(float(boundary[2])))
    add_reference_to_stage(str(asset["resolved_path"]), reference_path)
    dome = UsdLux.DomeLight.Define(stage, "/World/Exp02CStaticViewLight")
    dome.CreateIntensityAttr(1000.0)
    while is_stage_loading():
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed while loading the Jackal visual")
        SIMULATION_APP.update()
    return {
        "asset": asset,
        "reference_prim_path": reference_path,
        "pose_se2": [float(value) for value in boundary],
        "pose_frame": "world",
        "translation_unit": "m",
        "yaw_unit": "rad",
        "boundary_kind": boundary_kind,
        "spawn_height_m": spawn_height,
        "visual_only": True,
        "articulation_initialized": False,
        "physics_executed": False,
    }


def capture(path: Path) -> None:
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active Isaac viewport unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 15.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("EXP-02C viewport capture failed")


def configure_camera(paths: list[np.ndarray], height: float) -> None:
    xy = np.vstack([path[:, :2] for path in paths])
    center = (np.min(xy, axis=0) + np.max(xy, axis=0)) / 2.0
    extent = float(np.max(np.ptp(xy, axis=0)))
    set_camera_view(
        eye=[float(center[0]), float(center[1] - 0.2), max(height, 2.0 * extent + 1.5)],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )


def print_variant(root: Path, variant: str) -> None:
    costs = load_strict_json(root / variant / "factor_costs_initial.json")
    gradients = load_strict_json(root / variant / "factor_gradients_initial.json")
    controller = load_strict_json(root / variant / "controller_desired_metrics.json")
    geometry = load_strict_json(root / variant / "geometry_metrics.json")
    rigid = load_strict_json(root / variant / "rigid_fit_metrics.json")
    payload = {
        "variant": variant,
        "active_factor_costs": {
            name: item["objective_weighted_cost"] for name, item in costs.items()
        },
        "weighted_gradient_norms": {
            name: item["weighted"]["gradient_norm"]
            for name, item in gradients["factors"].items()
        },
        "delta_v_des_abs_mps": controller["delta_v_des_abs_mps"],
        "delta_omega_des_abs_rps": controller["delta_omega_des_abs_rps"],
        "entry_displacement_m": geometry["entry"]["translation_displacement_m"],
        "endpoint_displacement_m": geometry["endpoint"]["translation_displacement_m"],
        "rigid_fit_translation_rms_m": rigid["translation_rms_m"],
    }
    print("EXP02C_GUI_VARIANT=" + json.dumps(payload, sort_keys=True), flush=True)


def main() -> None:
    run = ARGS.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    metadata = load_strict_json(run / "metadata.json")
    if metadata.get("experiment") != "EXP-02C" or metadata.get("artifact_variant_count") != 126:
        raise ValueError("GUI requires a complete EXP-02C run")
    visual = config["gui"]
    colors = visual["colors"]
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines()
    draw.clear_points()
    paths: list[np.ndarray] = []
    legend: dict[str, Any]
    terminal_variants: tuple[str, ...]
    z = float(visual["z_offset_m"])
    width = float(visual["line_width"])
    marker_size = float(visual["marker_size"])

    if ARGS.view == "real_benign_k0":
        root = run / "real/case_benign_delayed/k_0"
        provenance = load_strict_json(run / "source_provenance.json")
        trial = Path(provenance["real_cases"]["case_benign_delayed"]["trial_directory"])
        old = np.load(trial / "derived/old_world.npy", allow_pickle=False)
        attempt = load_strict_json(trial / "results/attempt.json")
        boundary = np.asarray(attempt["robot_pose_at_new_ready"], dtype=float)
        boundary_kind = "saved_source_boundary"
        raw = np.load(root / "V0_RAW/raw_fresh.npy", allow_pickle=False)
        draw_polyline(draw, old, z=z, color=colors["old"], width=width)
        draw_polyline(draw, raw, z=z + 0.02, color=colors["raw"], width=width)
        paths.extend((old, raw))
        terminal_variants = (
            "V1_ENTRY_FRESH",
            "V2_ENTRY_DIRECTION_FRESH",
            "V3_ENTRY_YAW_FRESH",
            "V4_FULL_CURRENT_M4",
            "V5_NO_ENTRY",
        )
        for offset, variant in enumerate(terminal_variants, start=1):
            poses = np.load(root / variant / "optimized.npy", allow_pickle=False)
            paths.append(poses)
            draw_polyline(
                draw,
                poses,
                z=z + 0.04 * (offset + 1),
                color=colors[variant],
                width=width,
            )
            draw_pose_points(
                draw,
                poses[:1],
                z=z + 0.04 * (offset + 1) + 0.02,
                color=colors[variant],
                size=marker_size,
            )
        draw_pose_points(draw, boundary[None, :], z=z + 0.32, color=colors["boundary"], size=marker_size * 1.4)
        draw_heading_markers(draw, boundary[None, :], z=z + 0.32, color=colors["boundary"], width=3.0, length_m=0.18)
        draw_pose_points(draw, raw[:1], z=z + 0.36, color=colors["raw"], size=marker_size * 1.3)
        legend = {
            "BLUE": "OLD",
            "GREY": "raw full FRESH and F_k marker",
            "CYAN": "V1 ENTRY+FRESH (raw no-op)",
            "ORANGE": "V2 ENTRY+DIRECTION+FRESH",
            "PURPLE": "V3 ENTRY+YAW+FRESH",
            "RED": "V4 FULL current M4",
            "GREEN": "V5 NO ENTRY",
            "WHITE_STAR": "saved B",
            "colored_first_pose_points": "each variant X_k",
        }
    else:
        root = run / "synthetic/S4_DOWNSTREAM_CONFLICT"
        raw = np.load(root / "raw_fresh.npy", allow_pickle=False)
        desired = np.load(root / "desired_diagnostic_target.npy", allow_pickle=False)
        full = np.load(root / "V4_FULL_CURRENT_M4/optimized.npy", allow_pickle=False)
        no_prop = np.load(root / "V8_DIAGNOSTIC_NO_PROPAGATION/optimized.npy", allow_pickle=False)
        boundary = np.asarray([0.0, 0.0, 0.0])
        boundary_kind = "declared_synthetic_boundary"
        for offset, (poses, color) in enumerate(
            (
                (raw, colors["raw"]),
                (desired, colors["desired_target"]),
                (full, colors["V4_FULL_CURRENT_M4"]),
                (no_prop, colors["V8_DIAGNOSTIC_NO_PROPAGATION"]),
            )
        ):
            draw_polyline(draw, poses, z=z + 0.05 * offset, color=color, width=width)
            draw_pose_points(draw, poses[:1], z=z + 0.05 * offset + 0.02, color=color, size=marker_size)
        draw_pose_points(draw, boundary[None, :], z=z + 0.24, color=colors["boundary"], size=marker_size * 1.4)
        paths.extend((raw, desired, full, no_prop))
        terminal_variants = ("V0_RAW", "V4_FULL_CURRENT_M4", "V8_DIAGNOSTIC_NO_PROPAGATION")
        legend = {
            "GREY": "raw FRESH",
            "YELLOW": "desired synthetic diagnostic target (not an objective factor)",
            "RED": "V4 FULL current M4",
            "MAGENTA": "V8 DIAGNOSTIC_NO_PROPAGATION",
            "WHITE_STAR": "B",
        }

    print("EXP02C_GUI_PHASE=LOADING_STATIC_JACKAL", flush=True)
    robot_visual = add_static_boundary_jackal(config, boundary, boundary_kind)
    configure_camera(paths, float(visual["camera_height_m"]))
    print(f"EXP02C_GUI_PHASE=STATIC_FACTOR_DIAGNOSIS view={ARGS.view}", flush=True)
    print(
        "EXP02C_GUI_ROBOT="
        + json.dumps(
            {
                "meaning": "static visual reference at view boundary B; not an execution trace",
                "boundary_kind": robot_visual["boundary_kind"],
                "pose_se2": robot_visual["pose_se2"],
                "reference_prim_path": robot_visual["reference_prim_path"],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    print("EXP02C_GUI_LEGEND=" + json.dumps(legend, sort_keys=True), flush=True)
    for variant in terminal_variants:
        print_variant(root, variant)
    for _ in range(120):
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed before EXP-02C view was rendered")
        SIMULATION_APP.update()
        time.sleep(0.01)

    output = run / "gui_metadata" / datetime.now(timezone.utc).strftime(
        f"{ARGS.view}-%Y%m%dT%H%M%SZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    screenshot = output / "viewport.png"
    if not ARGS.headless:
        capture(screenshot)
    save_json_exclusive(
        output / "metadata.json",
        {
            "experiment": "EXP-02C GUI",
            "view": ARGS.view,
            "diagnostic_only": True,
            "isaac_debug_draw": True,
            "physics_executed": False,
            "quantitative_evidence": False,
            "robot_visual": robot_visual,
            "legend": legend,
            "terminal_variants": list(terminal_variants),
            "screenshot": str(screenshot) if screenshot.is_file() else None,
            "screenshot_sha256": sha256_file(screenshot) if screenshot.is_file() else None,
        },
    )
    print("EXP02C_GUI_PHASE=FINISHED output=" + str(output), flush=True)
    if not ARGS.no_hold:
        print(
            "EXP02C_GUI_PHASE=READY_AND_HOLDING; close Isaac Sim or press Ctrl-C to exit.",
            flush=True,
        )
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            time.sleep(0.02)


try:
    main()
finally:
    SIMULATION_APP.close()
