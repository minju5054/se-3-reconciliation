#!/usr/bin/env python3
"""Validate descriptive projection artifacts and reviewed actual Isaac images."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image

import robotless_projection_artifacts as artifacts
from characterize_robotless_projection_handoff import PLOTS
from reconciliation.robotless_projection_handoff import DEFINITIONS, characterize_projection, csv_row
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file, validate_observation_time
from validate_robotless_controlled_staleness import hash_check, require, same

VALIDATED = "ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY_VALIDATED"
FAILED = "ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY_VALIDATION_FAILED"
NOT_VALIDATED = "ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY_RUNTIME_NOT_VALIDATED"
ANALYSIS_FILES = ("source.json", "config_snapshot.yaml", "derived/projection_metrics.json", "derived/projection_metrics.csv", "plot_manifest.json", "analysis_validation.json")
REVIEW_FLAGS = ("fixed_FRESH", "B_and_Q", "connectors", "incoming_arrows", "tangent_arrows", "pose_yaw_arrows", "all_tau_details", "no_robot", "four_plots")


def compare_value(actual, expected, name):
    if expected is None or isinstance(expected, (str, bool)):
        require(actual == expected and type(actual) is type(expected), name)
    elif isinstance(expected, int):
        require(type(actual) is int and actual == expected, name)
    else:
        same(actual, expected, name)


def validate_analysis(run: Path) -> dict:
    config, record, loaded = artifacts.load_inputs(run)
    source, previous_config, previous_record, successive, metadata, arrays, boundaries, previous_metrics = loaded
    require(artifacts.validate_source(source) == record["source_validation"], "source validator result changed")
    require(record["R_obs"] == metadata["R1"] and record["motion"] == previous_config["motion"], "source pose/motion changed")
    require(record["observation_time"] == previous_record["source_observation_time"], "observation time changed")
    validate_observation_time(record["created_time"])
    require(record["new_lightnav_inference_count"] == 0 and record["actual_execution_time"] is None
            and record["new_inference_readiness_time"] is None and record["controlled_motion_is_surrogate"] is True,
            "unexpected inference/execution interpretation")
    require(record["incoming_direction_definition"] == DEFINITIONS["phi_in_rad"], "incoming direction meaning changed")
    require(record["trajectory_inputs"] == previous_record["array_inputs"], "trajectory input provenance changed")
    for item in record["source_inputs"].values():
        hash_check(source, item)
    rows = characterize_projection(boundaries, arrays["FRESH_world"], previous_metrics["conditions"], previous_config["motion"],
        distance_atol_m=config["numerical_validation"]["previous_d_poly_atol_m"])
    metrics = artifacts.read_json(run / "derived/projection_metrics.json")
    require(metrics["definitions"] == DEFINITIONS and metrics["fresh_reanchored"] is False, "projection definitions changed")
    require(metrics["source_json_sha256"] == sha256_file(run / "source.json"), "metrics source hash mismatch")
    require(metrics["fixed_fresh_world_input"] == record["trajectory_inputs"]["FRESH_world"], "FRESH was replaced")
    hash_check(run, metrics["configuration"])
    require(len(metrics["conditions"]) == len(rows), "wrong number of tau conditions")
    for actual, expected in zip(metrics["conditions"], rows, strict=True):
        require(set(actual) == set(expected), "metric fields differ")
        for key, value in expected.items():
            compare_value(actual[key], value, f"metric {key}")
    with (run / "derived/projection_metrics.csv").open(newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    require(len(csv_rows) == len(rows), "CSV condition count")
    for actual, expected in zip(csv_rows, rows, strict=True):
        flat = csv_row(expected)
        require(set(actual) == set(flat), "CSV fields differ")
        for key, value in flat.items():
            if value is None or isinstance(value, (str, bool)):
                require(actual[key] == ("" if value is None else str(value)), f"CSV {key}")
            else:
                same(float(actual[key]), value, f"CSV {key}")
    analysis = artifacts.read_json(run / "analysis_validation.json")
    require(analysis["source_hashes_unchanged"] is True and analysis["new_lightnav_inference_count"] == 0, "analysis failed")
    require(analysis["source_json_sha256"] == sha256_file(run / "source.json"), "analysis source hash")
    for item in analysis["outputs"]:
        hash_check(run, item)
    plots = artifacts.read_json(run / "plot_manifest.json")
    hash_check(run, plots["metrics_input"])
    hash_check(run, plots["configuration"])
    require(len(plots["plots"]) == len(PLOTS), "four plots required")
    for item, (key, name, _) in zip(plots["plots"], PLOTS, strict=True):
        require(item["path"] == f"evidence/tau_vs_{name}.png" and item["metric"] == key, "wrong plot metric")
        hash_check(run, item)
        require(item["tau_s"] == [r["tau_s"] for r in rows] and item["values"] == [r[key] for r in rows], "plotted data changed")
    return {"source_run": str(source), "R_obs": record["R_obs"], "motion": record["motion"], "conditions": rows,
            "max_abs_previous_d_poly_difference_m": max(abs(r["e_perp_minus_previous_d_poly_m"]) for r in rows)}


def validate_visual(run: Path, details: dict) -> None:
    config, record, loaded = artifacts.load_inputs(run)
    _, _, _, successive, _, arrays, boundaries, _ = loaded
    visual = artifacts.read_json(run / "visualization.json")
    require(visual["source_json_sha256"] == sha256_file(run / "source.json"), "visual source mismatch")
    require(visual["metrics_sha256"] == sha256_file(run / "derived/projection_metrics.json"), "visual metric hash mismatch")
    require(visual["config_sha256"] == sha256_file(run / "config_snapshot.yaml"), "visual config mismatch")
    require(visual["isaac_viewport_rendered"] is True and visual["new_lightnav_inference_count"] == 0, "actual viewport missing")
    require(visual["timeline_time_s"] == 0 and visual["geometry_scaling"] == 1
            and visual["scene_visibility_modifications"] == [] and visual["fresh_reanchored"] is False, "unexpected scene/motion alteration")
    inventory = visual["scene"]["runtime_inventory"]
    source_asset = artifacts.previous.load_config(successive / "config_snapshot.yaml")["scene"]["asset_relative_path"]
    require(any(reference.endswith(source_asset) for reference in inventory["authored_asset_references"]), "wrong scene asset")
    require(inventory["no_robot_model"] is True and not inventory["articulation_roots"] and not inventory["robot_named_paths"]
            and inventory["rigid_body_count"] == 0 and inventory["physics_scene_count"] == 0 and inventory["dynamics_advanced"] is False, "robot/dynamics present")
    same(visual["agent_pose_before"], details["R_obs"], "visual observation pose", tolerance=1e-7)
    same(visual["agent_pose_after"], details["R_obs"], "agent moved", tolerance=1e-7)
    same(visual["fresh_world_drawn"], arrays["FRESH_world"], "visual FRESH differs")
    same(visual["boundaries_world"], boundaries, "visual B differs")
    validate_observation_time(visual["created_time"])
    require(len(visual["details"]) == len(details["conditions"]), "all detail views required")
    for rendered, row in zip(visual["details"], details["conditions"], strict=True):
        require(rendered["tau_s"] == row["tau_s"], "detail tau differs")
        same(rendered["B_world"], row["B_world"], "drawn B mismatch")
        same(rendered["Q_xy_world_m"], row["Q_xy_world_m"], "drawn Q mismatch")
        for key in ("phi_in_rad", "phi_F_rad", "theta_Q_rad", "tangent_available"):
            compare_value(rendered[key], row[key], f"rendered {key}")
    review = artifacts.read_json(run / "visual_review.json")
    require(review["reviewed"] is True, "explicit visual review missing")
    for key in REVIEW_FLAGS:
        require(review["checks"][key] is True, f"visual review failed: {key}")
    images = ["evidence/isaac_projection_overview.png"] + [f"evidence/tau_{i:03d}_detail.png" for i in range(len(boundaries))]
    require([item["path"] for item in visual["screenshots"]] == images, "wrong screenshot set")
    for item in visual["screenshots"]:
        hash_check(run, item)
    images += [f"evidence/tau_vs_{name}.png" for _, name, _ in PLOTS]
    for rel in images:
        require(review["image_sha256"][rel] == sha256_file(run / rel), "review image changed")
        with Image.open(run / rel) as image:
            require(image.format == "PNG" and min(image.size) >= 100, "invalid PNG")
            image.verify()
        with Image.open(run / rel) as image:
            require(float(np.asarray(image).std()) > 1, "blank image")


def validate_run(run: Path) -> dict:
    run = run.resolve()
    missing = [p for p in (*ANALYSIS_FILES, "visualization.json", "visual_review.json") if not (run / p).is_file()]
    details, failures = {}, []
    try:
        if all((run / p).is_file() for p in ANALYSIS_FILES):
            details = validate_analysis(run)
        if not missing:
            validate_visual(run, details)
    except FileNotFoundError as error:
        missing.append(str(error.filename))
    except (ValueError, KeyError, TypeError, IndexError, OSError, AttributeError, OverflowError) as error:
        failures.append(f"{type(error).__name__}: {error}")
    return {"schema_version": 1, "run_directory": str(run), "status": FAILED if failures else NOT_VALIDATED if missing else VALIDATED,
            "missing_artifacts": missing, "failures": failures, **details}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    result = validate_run(args.run_directory)
    if args.write:
        save_json_exclusive(args.run_directory / "validation.json", result)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == VALIDATED else 1


if __name__ == "__main__":
    raise SystemExit(main())
