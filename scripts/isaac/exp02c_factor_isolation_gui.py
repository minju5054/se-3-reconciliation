#!/usr/bin/env python3
"""Isaac DebugDraw-only GUI for saved EXP-02C factor-isolation artifacts."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
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
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true", help="render validation only")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import yaml
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from reconciliation.online_switch import load_strict_json, save_json_exclusive, sha256_file


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


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

    configure_camera(paths, float(visual["camera_height_m"]))
    print(f"EXP02C_GUI_PHASE=STATIC_FACTOR_DIAGNOSIS view={ARGS.view}", flush=True)
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
            "legend": legend,
            "terminal_variants": list(terminal_variants),
            "screenshot": str(screenshot) if screenshot.is_file() else None,
            "screenshot_sha256": sha256_file(screenshot) if screenshot.is_file() else None,
        },
    )
    print("EXP02C_GUI_PHASE=FINISHED output=" + str(output), flush=True)
    if bool(config["gui"]["hold"]) and not ARGS.no_hold:
        print("Close Isaac Sim to exit the static diagnostic view.", flush=True)
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            time.sleep(0.02)


try:
    main()
finally:
    SIMULATION_APP.close()
