#!/usr/bin/env python3
"""Run one selected Stage 0-F LightNav reference with qualitative GUI telemetry."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--selection",
        choices=("low", "median", "high", "strong_like", "direction_change", "first_fail"),
        default="high",
    )
    parser.add_argument("--real-time-factor", type=float)
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})

import numpy as np
import yaml
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.util.debug_draw import _debug_draw
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

from closed_loop_execution_runtime import ClosedLoopRuntime, run_closed_loop
from debug_draw_trajectories import draw_heading_markers, draw_polyline, draw_pose_points
from reconciliation.closed_loop_execution_validation import (
    compute_closed_loop_metrics,
    load_frozen_stage0d_candidate,
)
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.lightnav_execution_envelope import severity_key
from reconciliation.online_switch import load_strict_json, sha256_file


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def selected_row(
    rows: list[dict[str, Any]],
    selection: str,
    severity_order: list[str],
    decision: Mapping[str, Any],
) -> tuple[dict[str, Any], int]:
    ordered = sorted(
        rows,
        key=lambda row: (
            severity_key(row["descriptor"], severity_order),
            row["kind"],
            row["reference_id"],
        ),
    )
    if selection == "low":
        row = ordered[0]
    elif selection == "median":
        row = ordered[(len(ordered) - 1) // 2]
    elif selection == "high":
        row = ordered[-1]
    elif selection == "strong_like":
        row = min(
            rows,
            key=lambda value: (
                value["fixture_comparison"]["nearest_failing_distance"],
                value["kind"],
                value["reference_id"],
            ),
        )
    elif selection == "direction_change":
        candidates = [
            value
            for value in rows
            if value["descriptor"]["curvature_sign_change_count"] > 0
        ]
        if not candidates:
            raise ValueError("no LightNav direction-change reference exists")
        row = max(
            candidates,
            key=lambda value: (
                value["descriptor"]["curvature_sign_change_count"],
                severity_key(value["descriptor"], severity_order),
                value["reference_id"],
            ),
        )
    else:
        failed = decision["failed_references"]
        if not failed:
            raise ValueError("no deterministic Stage 0-F execution failure exists")
        failed_id = sorted(
            failed, key=lambda value: (value["kind"], value["reference_id"])
        )[0]["reference_id"]
        row = next(value for value in rows if value["reference_id"] == failed_id)
    return row, ordered.index(row) + 1


def configure_camera(reference: np.ndarray, height: float) -> None:
    center = (
        np.min(reference[:, :2], axis=0) + np.max(reference[:, :2], axis=0)
    ) / 2.0
    extent = float(np.max(np.ptp(reference[:, :2], axis=0)))
    set_camera_view(
        eye=[
            float(center[0]),
            float(center[1] - 0.3),
            max(height, extent * 2.0 + 1.5),
        ],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )


def redraw(
    draw,
    visual: Mapping[str, Any],
    reference: np.ndarray,
    start: np.ndarray,
    nominal: list[np.ndarray] | np.ndarray,
    calibrated: list[np.ndarray] | np.ndarray,
    *,
    show_start: bool,
) -> None:
    z = float(visual["z_offset_m"])
    draw.clear_lines()
    draw.clear_points()
    draw_polyline(
        draw,
        reference,
        z=z,
        color=visual["reference_color_rgba"],
        width=float(visual["line_width"]),
    )
    draw_heading_markers(
        draw,
        reference[:: int(visual["heading_marker_stride"])],
        z=z,
        color=visual["reference_color_rgba"],
        width=2.0,
        length_m=float(visual["heading_marker_length_m"]),
    )
    if show_start:
        draw_pose_points(
            draw,
            np.asarray([start]),
            z=z + 0.12,
            color=visual["observation_marker_color_rgba"],
            size=float(visual["marker_size"]),
        )
    if len(nominal) > 1:
        draw_polyline(
            draw,
            np.asarray(nominal),
            z=z + 0.04,
            color=visual["nominal_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )
    if len(calibrated) > 1:
        draw_polyline(
            draw,
            np.asarray(calibrated),
            z=z + 0.08,
            color=visual["calibrated_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )


def callback(
    *,
    row: Mapping[str, Any],
    mode: str,
    severity_rank: int,
    reference: np.ndarray,
    runtime: ClosedLoopRuntime,
    draw,
    visual: Mapping[str, Any],
    nominal: list[np.ndarray] | np.ndarray,
    start: np.ndarray,
):
    def update(state: dict[str, Any], path: list[np.ndarray]) -> None:
        sample = nearest_polyline_samples(reference, np.asarray([path[-1]]))
        redraw(
            draw,
            visual,
            reference,
            start,
            path if mode == "nominal" else nominal,
            path if mode == "calibrated" else [],
            show_start=row["kind"] == "FRESH",
        )
        print(
            "STAGE0F_GUI_TELEMETRY="
            + json.dumps(
                {
                    "source_trials": row["source_trial_paths"],
                    "kind": row["kind"],
                    "unique_hash": row["world_trajectory_sha256"],
                    "geometry_severity_rank": severity_rank,
                    "mode": mode.upper(),
                    "phase": f"{mode.upper()}_CLOSED_LOOP_EXECUTION",
                    "actual": state["actual"],
                    "nearest_index": state["nearest_index"],
                    "target_index": state["target_index"],
                    "goal_reached": state["goal_reached"],
                    "desired_v_omega": state["desired"],
                    "executed_v_omega": state["executed"],
                    "measured_v_omega": state["measured"],
                    "spatial_position_error_m": float(sample["distance_m"][0]),
                    "spatial_yaw_error_rad": float(sample["yaw_error_rad"][0]),
                    "wheel_targets_rad_s": state["target_wheels"],
                    "wheel_measured_rad_s": state["measured_wheels"],
                    "pi_correction_rps": state["pi_correction_rps"],
                    "integral_error_rad": state["integral_error_rad"],
                    "saturated": state["saturated"],
                    "sign_protection_event": state["sign_protection_event"],
                },
                sort_keys=True,
            ),
            flush=True,
        )

    return update


def wall_hold(seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed during Stage 0-F GUI hold")
        SIMULATION_APP.update()
        time.sleep(0.02)


def capture(output: Path) -> Path:
    path = output / "final_viewport.png"
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport unavailable")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 10.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("Stage 0-F viewport capture failed")
    return path


def main() -> None:
    run = ARGS.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    preparation = load_strict_json(run / "preparation_metadata.json")
    if sha256_file(run / "config_snapshot.yaml") != preparation[
        "config_snapshot_sha256"
    ]:
        raise ValueError("prepared Stage 0-F config snapshot changed")
    comparison_path = run / "summary/geometry_fixture_comparison.json"
    if sha256_file(comparison_path) != preparation[
        "geometry_fixture_comparison_sha256"
    ]:
        raise ValueError("prepared Stage 0-F geometry comparison changed")
    comparison = load_strict_json(run / "summary/geometry_fixture_comparison.json")
    decision = load_strict_json(
        run / "summary/final_lightnav_execution_envelope_decision.json"
    )
    rows = comparison["references"]
    row, rank = selected_row(
        rows, ARGS.selection, config["geometry"]["severity_order"], decision
    )
    reference_path = (
        run
        / "geometry"
        / row["kind"].lower()
        / row["reference_id"]
        / "reference_world.npy"
    )
    reference = np.load(reference_path, allow_pickle=False)
    provenance = load_strict_json(reference_path.parent / "source_provenance.json")
    if sha256_file(reference_path) != provenance["derived_copy_sha256"]:
        raise ValueError("prepared Stage 0-F GUI reference changed")
    if row["world_trajectory_sha256"] != provenance["world_trajectory_sha256"]:
        raise ValueError("Stage 0-F GUI source provenance binding changed")
    start = np.asarray(row["execution_start_pose_se2"], dtype=np.float64)
    stage0e_config = load_yaml(
        resolve_path(config["paths"]["stage0e_run"]) / "config_snapshot.yaml"
    )
    stage0b = load_yaml(resolve_path(stage0e_config["paths"]["stage0b_config"]))
    candidate = load_frozen_stage0d_candidate(
        resolve_path(stage0e_config["paths"]["stage0d_run"]),
        stage0e_config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    factor = float(
        ARGS.real_time_factor
        if ARGS.real_time_factor is not None
        else config["gui"]["real_time_factor"]
    )
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError("real-time factor must be finite and positive")
    output = run / "gui_metadata" / datetime.now(timezone.utc).strftime(
        f"{ARGS.selection}-%Y%m%dT%H%M%SZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    runtime = ClosedLoopRuntime(
        SIMULATION_APP,
        stage0e_config,
        stage0b,
        candidate,
        render=not ARGS.headless,
        real_time_factor=factor,
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    visual = config["gui"]["visualization"]
    configure_camera(reference, float(visual["camera_height_m"]))
    descriptor = row["descriptor"]
    print("[Stage 0-F GUI] BLUE=LightNav reference, GREEN=calibrated actual, ORANGE/RED=nominal actual", flush=True)
    print(
        "STAGE0F_GUI_GEOMETRY="
        + json.dumps(
            {
                "reference_id": row["reference_id"],
                "kind": row["kind"],
                "source_trials": row["source_trial_paths"],
                "unique_hash": row["world_trajectory_sha256"],
                "severity_rank": rank,
                "path_length_m": descriptor["path_length_m"],
                "cumulative_pose_yaw_rad": descriptor[
                    "cumulative_abs_pose_yaw_change_rad"
                ],
                "tangent_curvature_p95_abs_per_m": descriptor[
                    "tangent_curvature_p95_abs_per_m"
                ],
                "tangent_curvature_max_abs_per_m": descriptor[
                    "tangent_curvature_max_abs_per_m"
                ],
                "nearest_stage0e_fixture": row["fixture_comparison"][
                    "nearest_fixture"
                ],
                "nearest_fixture_passed": row["fixture_comparison"][
                    "nearest_fixture_passed"
                ],
                "fixture_label": row["fixture_comparison"]["label"],
                "fresh_observation_start_marker": row["kind"] == "FRESH",
            },
            sort_keys=True,
        ),
        flush=True,
    )
    redraw(
        draw,
        visual,
        reference,
        start,
        [],
        [],
        show_start=row["kind"] == "FRESH",
    )
    frozen_representatives = load_strict_json(
        run / "summary/representative_selection.json"
    )
    nominal_selected = any(
        value["kind"] == row["kind"]
        and value["world_trajectory_sha256"] == row["world_trajectory_sha256"]
        for value in frozen_representatives["references"]
    )
    nominal: list[np.ndarray] | np.ndarray = []
    nominal_metrics = None
    if nominal_selected:
        print("STAGE0F_GUI_PHASE=NOMINAL_CLOSED_LOOP_EXECUTION", flush=True)
        nominal_telemetry = run_closed_loop(
            runtime,
            stage0e_config,
            stage0b,
            reference,
            start,
            "nominal",
            on_control=callback(
                row=row,
                mode="nominal",
                severity_rank=rank,
                reference=reference,
                runtime=runtime,
                draw=draw,
                visual=visual,
                nominal=[],
                start=start,
            ),
        )
        nominal = nominal_telemetry.actual_trajectory
        nominal_metrics = compute_closed_loop_metrics(nominal_telemetry)
        redraw(
            draw,
            visual,
            reference,
            start,
            nominal,
            [],
            show_start=row["kind"] == "FRESH",
        )
        print("STAGE0F_GUI_PHASE=RESET_BEFORE_CALIBRATED", flush=True)
        wall_hold(float(config["gui"]["inter_mode_hold_wall_s"]))
    print("STAGE0F_GUI_PHASE=CALIBRATED_CLOSED_LOOP_EXECUTION", flush=True)
    calibrated_telemetry = run_closed_loop(
        runtime,
        stage0e_config,
        stage0b,
        reference,
        start,
        "calibrated",
        on_control=callback(
            row=row,
            mode="calibrated",
            severity_rank=rank,
            reference=reference,
            runtime=runtime,
            draw=draw,
            visual=visual,
            nominal=nominal,
            start=start,
        ),
    )
    calibrated = calibrated_telemetry.actual_trajectory
    redraw(
        draw,
        visual,
        reference,
        start,
        nominal,
        calibrated,
        show_start=row["kind"] == "FRESH",
    )
    np.save(output / "reference.npy", reference)
    if len(nominal) > 1:
        np.save(output / "nominal_actual.npy", nominal)
    np.save(output / "calibrated_actual.npy", calibrated)
    viewport = capture(output)
    with (output / "metadata.json").open("x", encoding="utf-8") as stream:
        json.dump(
            {
                "stage": config["stage"],
                "diagnostic_only": True,
                "selection": ARGS.selection,
                "reference_id": row["reference_id"],
                "kind": row["kind"],
                "source_world_sha256": row["world_trajectory_sha256"],
                "reference_copy_sha256": sha256_file(reference_path),
                "severity_rank": rank,
                "descriptor": descriptor,
                "fixture_comparison": row["fixture_comparison"],
                "nominal_executed": nominal_selected,
                "nominal_metrics": nominal_metrics,
                "calibrated_metrics": compute_closed_loop_metrics(
                    calibrated_telemetry
                ),
                "fresh_observation_start_pose_marker": (
                    start.tolist() if row["kind"] == "FRESH" else None
                ),
                "real_time_factor": factor,
                "slowdown_changes_simulation_timing": False,
                "viewport_capture": str(viewport),
            },
            stream,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        stream.write("\n")
    print(f"STAGE0F_GUI_OUTPUT={output}", flush=True)
    if ARGS.hold or (not ARGS.no_hold and bool(config["gui"]["final_hold"])):
        print("STAGE0F_GUI_PHASE=FINISHED; close Isaac Sim to exit", flush=True)
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            time.sleep(0.02)


try:
    main()
finally:
    SIMULATION_APP.close()
