#!/usr/bin/env python3
"""Qualitative Stage 0-E nominal/calibrated same-reference GUI comparison."""

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
    parser.add_argument("validation_run", type=Path)
    parser.add_argument(
        "--suite", choices=("controlled", "composite", "exp02b"), default="controlled"
    )
    parser.add_argument("--scenario", default="s_curve")
    parser.add_argument("--case", default="case_high_delta_omega")
    parser.add_argument("--real-time-factor", type=float)
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--headless", action="store_true", help="automation-only render check")
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
    generate_controlled_references,
    load_frozen_stage0d_candidate,
)
from reconciliation.exp02b import CASE_IDS, load_source_case, verify_frozen_source_selection
from reconciliation.exp02b_diagnosis import nearest_polyline_samples
from reconciliation.online_switch import load_strict_json, sha256_file
from reconciliation.trajectory import generate_reference_trajectory, segments_from_config


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def configure_camera(paths: list[np.ndarray], height: float) -> None:
    xy = np.vstack([path[:, :2] for path in paths])
    center = (np.min(xy, axis=0) + np.max(xy, axis=0)) / 2.0
    extent = np.max(np.ptp(xy, axis=0))
    camera_height = max(height, float(extent) * 2.0 + 1.5)
    set_camera_view(
        eye=[float(center[0]), float(center[1] - 0.3), camera_height],
        target=[float(center[0]), float(center[1]), 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )


def redraw(
    draw,
    visual: Mapping[str, Any],
    reference: np.ndarray,
    nominal: list[np.ndarray] | np.ndarray,
    calibrated: list[np.ndarray] | np.ndarray,
    fresh: np.ndarray | None = None,
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
    if fresh is not None:
        draw_polyline(
            draw,
            fresh,
            z=z + 0.02,
            color=visual["fresh_context_color_rgba"],
            width=float(visual["line_width"]) * 0.7,
        )
    if len(nominal) > 1:
        draw_polyline(
            draw,
            np.asarray(nominal),
            z=z + 0.04,
            color=visual["nominal_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )
        draw_pose_points(
            draw,
            np.asarray([nominal[-1]]),
            z=z + 0.05,
            color=visual["nominal_color_rgba"],
            size=float(visual["marker_size"]),
        )
    if len(calibrated) > 1:
        draw_polyline(
            draw,
            np.asarray(calibrated),
            z=z + 0.08,
            color=visual["calibrated_color_rgba"],
            width=float(visual["line_width"]) * 1.2,
        )
        draw_pose_points(
            draw,
            np.asarray([calibrated[-1]]),
            z=z + 0.09,
            color=visual["calibrated_color_rgba"],
            size=float(visual["marker_size"]),
        )


def live_callback(
    *,
    suite: str,
    scenario: str,
    mode: str,
    reference: np.ndarray,
    runtime: ClosedLoopRuntime,
    candidate: Mapping[str, Any],
    draw,
    visual: Mapping[str, Any],
    nominal: list[np.ndarray] | np.ndarray,
    calibrated: list[np.ndarray] | np.ndarray,
    fresh: np.ndarray | None,
):
    def callback(state: dict[str, Any], path: list[np.ndarray]) -> None:
        current_nominal = path if mode == "nominal" else nominal
        current_calibrated = path if mode == "calibrated" else calibrated
        sample = nearest_polyline_samples(reference, np.asarray([path[-1]]))
        print(
            "STAGE0E_GUI_TELEMETRY="
            + json.dumps(
                {
                    "suite": suite,
                    "scenario": scenario,
                    "mode": mode.upper(),
                    "phase": f"{mode.upper()}_CLOSED_LOOP_EXECUTION",
                    "sim_time_s": float(runtime.world.current_time),
                    "actual_x": state["actual"][0],
                    "actual_y": state["actual"][1],
                    "actual_yaw": state["actual"][2],
                    "nearest_index": state["nearest_index"],
                    "target_index": state["target_index"],
                    "goal_reached": state["goal_reached"],
                    "desired_v_mps": state["desired"][0],
                    "desired_omega_rps": state["desired"][1],
                    "executed_v_mps": state["executed"][0],
                    "executed_omega_rps": state["executed"][1],
                    "measured_v_mps": state["measured"][0],
                    "measured_omega_rps": state["measured"][1],
                    "nearest_position_error_m": float(sample["distance_m"][0]),
                    "nearest_yaw_error_rad": float(sample["yaw_error_rad"][0]),
                    "wheel_targets_rad_s": state["target_wheels"],
                    "wheel_measured_rad_s": state["measured_wheels"],
                    "feedforward_scale": (
                        1.0
                        if mode == "nominal"
                        else candidate["selected_parameters"]["feedforward_scale"]
                    ),
                    "pi_correction_rps": state["pi_correction_rps"],
                    "integral_error_rad": state["integral_error_rad"],
                    "saturated": state["saturated"],
                    "sign_protection_event": state["sign_protection_event"],
                },
                sort_keys=True,
            ),
            flush=True,
        )
        redraw(
            draw,
            visual,
            reference,
            current_nominal,
            current_calibrated,
            fresh,
        )

    return callback


def wall_hold(seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not SIMULATION_APP.is_running():
            raise RuntimeError("Isaac Sim closed during GUI inter-mode hold")
        SIMULATION_APP.update()
        time.sleep(0.02)


def run_pair(
    runtime: ClosedLoopRuntime,
    config: Mapping[str, Any],
    stage0b: Mapping[str, Any],
    candidate: Mapping[str, Any],
    draw,
    output: Path,
    *,
    suite: str,
    scenario: str,
    reference: np.ndarray,
    initial_pose: np.ndarray,
    fresh: np.ndarray | None = None,
) -> dict[str, Any]:
    visual = config["gui"]["visualization"]
    configure_camera(
        [reference] + ([] if fresh is None else [fresh]),
        float(visual["camera_height_m"]),
    )
    redraw(draw, visual, reference, [], [], fresh)
    nominal: list[np.ndarray] | np.ndarray = []
    calibrated: list[np.ndarray] | np.ndarray = []
    print(
        f"STAGE0E_GUI_PHASE=NOMINAL_CLOSED_LOOP_EXECUTION suite={suite} scenario={scenario}",
        flush=True,
    )
    nominal_telemetry = run_closed_loop(
        runtime,
        config,
        stage0b,
        reference,
        initial_pose,
        "nominal",
        on_control=live_callback(
            suite=suite,
            scenario=scenario,
            mode="nominal",
            reference=reference,
            runtime=runtime,
            candidate=candidate,
            draw=draw,
            visual=visual,
            nominal=[],
            calibrated=[],
            fresh=fresh,
        ),
    )
    nominal = nominal_telemetry.actual_trajectory
    redraw(draw, visual, reference, nominal, [], fresh)
    print(
        f"STAGE0E_GUI_PHASE=RESET_BEFORE_CALIBRATED suite={suite} scenario={scenario}",
        flush=True,
    )
    wall_hold(float(config["gui"]["inter_mode_hold_wall_s"]))
    calibrated_telemetry = run_closed_loop(
        runtime,
        config,
        stage0b,
        reference,
        initial_pose,
        "calibrated",
        on_control=live_callback(
            suite=suite,
            scenario=scenario,
            mode="calibrated",
            reference=reference,
            runtime=runtime,
            candidate=candidate,
            draw=draw,
            visual=visual,
            nominal=nominal,
            calibrated=[],
            fresh=fresh,
        ),
    )
    calibrated = calibrated_telemetry.actual_trajectory
    redraw(draw, visual, reference, nominal, calibrated, fresh)
    scenario_dir = output / scenario
    scenario_dir.mkdir(parents=True, exist_ok=False)
    np.save(scenario_dir / "reference.npy", reference)
    np.save(scenario_dir / "nominal_actual.npy", nominal)
    np.save(scenario_dir / "calibrated_actual.npy", calibrated)
    if fresh is not None:
        np.save(scenario_dir / "fresh_context.npy", fresh)
    return {
        "scenario": scenario,
        "nominal": compute_closed_loop_metrics(nominal_telemetry),
        "calibrated": compute_closed_loop_metrics(calibrated_telemetry),
    }


def capture_viewport(output: Path) -> Path:
    path = output / "final_viewport.png"
    viewport = get_active_viewport()
    if not viewport:
        raise RuntimeError("active viewport is unavailable for Stage 0-E GUI capture")
    capture_viewport_to_file(viewport, file_path=str(path))
    deadline = time.monotonic() + 10.0
    while not path.is_file() and time.monotonic() < deadline:
        SIMULATION_APP.update()
        time.sleep(0.02)
    if not path.is_file():
        raise RuntimeError("Stage 0-E GUI viewport capture did not complete")
    return path


def hold() -> None:
    print("STAGE0E_GUI_PHASE=FINISHED; close Isaac Sim to exit", flush=True)
    while SIMULATION_APP.is_running():
        SIMULATION_APP.update()
        time.sleep(0.02)


def main() -> None:
    run = ARGS.validation_run.resolve()
    config = load_yaml(run / "config_snapshot.yaml")
    primary_metadata = load_strict_json(run / "metadata.json")
    load_strict_json(run / "summary/final_execution_platform_decision.json")
    stage0b = load_yaml(resolve_path(config["paths"]["stage0b_config"]))
    candidate = load_frozen_stage0d_candidate(
        resolve_path(config["paths"]["stage0d_run"]),
        config["stage0d_candidate_provenance"],
        repository_root=REPOSITORY_ROOT,
    )
    real_time_factor = float(
        ARGS.real_time_factor
        if ARGS.real_time_factor is not None
        else config["gui"]["real_time_factor"]
    )
    if not math.isfinite(real_time_factor) or real_time_factor <= 0.0:
        raise ValueError("real-time factor must be finite and positive")
    output = run / "gui_metadata" / datetime.now(timezone.utc).strftime(
        f"{ARGS.suite}-%Y%m%dT%H%M%SZ"
    )
    output.mkdir(parents=True, exist_ok=False)
    runtime = ClosedLoopRuntime(
        SIMULATION_APP,
        config,
        stage0b,
        candidate,
        render=not ARGS.headless,
        real_time_factor=real_time_factor,
    )
    draw = _debug_draw.acquire_debug_draw_interface()
    visual = config["gui"]["visualization"]
    print("[Stage 0-E GUI] legend:", flush=True)
    print(f"  reference/planned OLD = BLUE {visual['reference_color_rgba']}", flush=True)
    print(f"  nominal closed-loop actual = ORANGE/RED {visual['nominal_color_rgba']}", flush=True)
    print(f"  calibrated closed-loop actual = GREEN {visual['calibrated_color_rgba']}", flush=True)
    print(
        f"  raw FRESH context (EXP-02B only; excluded from metrics) = GREY "
        f"{visual['fresh_context_color_rgba']}",
        flush=True,
    )
    results = []
    if ARGS.suite == "controlled":
        references = generate_controlled_references(config)
        configured = list(config["gui"]["controlled_scenarios"])
        scenarios = configured if ARGS.scenario == "all" else [ARGS.scenario]
        if any(value not in configured for value in scenarios):
            raise ValueError(f"controlled GUI scenario must be one of {configured} or all")
        for scenario in scenarios:
            results.append(
                run_pair(
                    runtime,
                    config,
                    stage0b,
                    candidate,
                    draw,
                    output,
                    suite="controlled",
                    scenario=scenario,
                    reference=references[scenario],
                    initial_pose=np.asarray(config["simulation"]["initial_pose_se2"]),
                )
            )
    elif ARGS.suite == "composite":
        reference = generate_reference_trajectory(
            config["simulation"]["initial_pose_se2"],
            segments_from_config(stage0b["composite_motion_profile"]),
            float(config["simulation"]["control_dt_s"]),
        ).poses
        results.append(
            run_pair(
                runtime,
                config,
                stage0b,
                candidate,
                draw,
                output,
                suite="composite",
                scenario="stage0b_composite",
                reference=reference,
                initial_pose=np.asarray(config["simulation"]["initial_pose_se2"]),
            )
        )
    else:
        exp02b = load_yaml(resolve_path(config["paths"]["exp02b_config"]))
        selection = load_strict_json(resolve_path(config["paths"]["exp02b_run"]) / "source_selection.json")
        verify_frozen_source_selection(selection["source_root"], exp02b)
        cases = list(CASE_IDS) if ARGS.case == "all" else [ARGS.case]
        if any(value not in CASE_IDS for value in cases):
            raise ValueError(f"EXP-02B GUI case must be one of {CASE_IDS} or all")
        for case_id in cases:
            source = load_source_case(
                case_id, selection["cases"][case_id]["trial_directory"]
            )
            results.append(
                run_pair(
                    runtime,
                    config,
                    stage0b,
                    candidate,
                    draw,
                    output,
                    suite="exp02b",
                    scenario=case_id,
                    reference=source.old_world,
                    initial_pose=np.asarray(source.source_metadata["initial_pose_se2"]),
                    fresh=source.fresh_world,
                )
            )
    viewport = capture_viewport(output)
    metadata = {
        "stage": config["stage"],
        "diagnostic_only": True,
        "quantitative_primary": False,
        "suite": ARGS.suite,
        "requested_scenario": ARGS.scenario,
        "requested_case": ARGS.case,
        "completed": results,
        "primary_validation_run": str(run),
        "primary_metadata_sha256": sha256_file(run / "metadata.json"),
        "primary_run_id": primary_metadata["run_id"],
        "stage0d_candidate_model_sha256": candidate["model_sha256"],
        "real_time_factor": real_time_factor,
        "slowdown_changes_simulation_timing": False,
        "legend": {
            "reference_or_planned_old": visual["reference_color_rgba"],
            "nominal_actual": visual["nominal_color_rgba"],
            "calibrated_actual": visual["calibrated_color_rgba"],
            "fresh_context_not_in_metrics": visual["fresh_context_color_rgba"],
        },
        "viewport_capture": str(viewport),
        "runtime_physics_overrides": {},
    }
    with (output / "metadata.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(f"STAGE0E_GUI_OUTPUT={output}", flush=True)
    if ARGS.hold or (not ARGS.no_hold and bool(config["gui"]["final_hold"])):
        hold()


try:
    main()
finally:
    SIMULATION_APP.close()
