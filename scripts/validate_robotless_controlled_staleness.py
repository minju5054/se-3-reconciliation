#!/usr/bin/env python3
"""Audit frozen-source controlled geometry and actual reviewed Isaac evidence."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image

import robotless_controlled_staleness_artifacts as artifacts
from characterize_robotless_controlled_staleness import METRIC_DEFINITIONS
from reconciliation.robotless_controlled_staleness import characterize, csv_row
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file, validate_observation_time


VALIDATED = "ROBOTLESS_CONTROLLED_STALENESS_VALIDATED"
FAILED = "ROBOTLESS_CONTROLLED_STALENESS_VALIDATION_FAILED"
NOT_VALIDATED = "ROBOTLESS_CONTROLLED_STALENESS_RUNTIME_NOT_VALIDATED"
IMAGES = ("evidence/isaac_handoff_overview.png", "evidence/latency_vs_entry_gap.png", "evidence/latency_vs_polyline_gap.png")
ANALYSIS_FILES = ("source.json", "config_snapshot.yaml", "derived/boundaries.npy", "derived/metrics.json",
                  "derived/metrics.csv", "plot_manifest.json", "analysis_validation.json")
REVIEW_CHECKS = ("fixed_FRESH", "OLD_context", "R_obs_equals_B_zero", "all_four_boundaries",
                 "controlled_motion_path", "no_robot", "entry_plot", "polyline_plot")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def same(actual, expected, name: str, *, tolerance=1e-12) -> None:
    a, b = np.asarray(actual), np.asarray(expected)
    require(a.shape == b.shape and np.all(np.isfinite(a)) and np.allclose(a, b, rtol=0, atol=tolerance), name)


def hash_check(run: Path, record: dict) -> None:
    path = (run / record["path"]).resolve()
    require(path.is_relative_to(run.resolve()), "artifact escaped run")
    require(sha256_file(path) == record["sha256"], f"artifact hash mismatch: {record['path']}")


def validate_analysis(run: Path) -> dict:
    config, source_record, source, metadata, arrays, boundaries = artifacts.load_inputs(run)
    source_result = artifacts.validate_source(source)
    require(source_result["status"] == artifacts.SOURCE_VALIDATED, "source validator no longer passes")
    require(source_result == source_record["source_validation"], "source validation result differs from saved result")
    for key, path in artifacts.SOURCE_ARRAYS.items():
        require(source_record["array_inputs"][key]["path"] == path, "wrong source array path")
        hash_check(source, source_record["array_inputs"][key])
    require(source_record["new_lightnav_inference_count"] == 0
            and source_record["actual_robot_execution_time"] is None
            and source_record["inference_readiness_time"] is None
            and source_record["actual_old_execution"] is False
            and source_record["controlled_motion_is_surrogate"] is True, "incorrect execution/inference interpretation")
    validate_observation_time(source_record["created_time"])
    require(source_record["source_observation_time"] == metadata["observations"][1]["observation_time"], "observation clock changed")
    motion = config["motion"]
    expected_b, expected_rows = characterize(metadata["R1"], arrays["FRESH_world"], motion["tau_s"], motion["v_mps"], motion["omega_radps"])
    require(np.array_equal(boundaries, expected_b), "saved boundary differs from configured body-forward motion")
    require(np.array_equal(boundaries[0], metadata["R1"]), "B(0) differs from R_obs")
    same(np.linalg.norm(boundaries[:, :2] - boundaries[0, :2], axis=1),
         np.asarray(motion["tau_s"]) * motion["v_mps"], "unexpected boundary translation")
    metrics = artifacts.read_json(run / "derived/metrics.json")
    require(metrics["definitions"] == METRIC_DEFINITIONS, "metric definitions changed")
    require(metrics["motion"] == motion and metrics["R_obs"] == metadata["R1"], "metric configuration mismatch")
    require(metrics["fresh_reanchored"] is False, "FRESH must not be reanchored")
    require(metrics["source_json_sha256"] == sha256_file(run / "source.json"), "metrics source hash mismatch")
    hash_check(run, metrics["boundaries_input"])
    hash_check(run, metrics["configuration"])
    require(len(metrics["conditions"]) == len(expected_rows), "condition count mismatch")
    for actual, expected in zip(metrics["conditions"], expected_rows, strict=True):
        require(actual["fixed_fresh_world_input"] == source_record["array_inputs"]["FRESH_world"], "condition reanchors or replaces FRESH")
        for key, value in expected.items():
            if key == "nearest_polyline":
                require(actual[key]["segment_index"] == value["segment_index"], "nearest segment differs")
                for field in ("distance_m", "nearest_xy_world_m", "segment_fraction"):
                    same(actual[key][field], value[field], f"nearest {field}")
            else:
                same(actual[key], value, f"metric mismatch: {key}")
    for key in ("delta_d_entry_m", "delta_d_poly_m"):
        require(metrics["conditions"][0][key] == 0.0, "baseline increment must be exactly zero")
    with (run / "derived/metrics.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        csv_rows = list(reader)
    require(len(csv_rows) == len(expected_rows), "CSV condition count mismatch")
    for actual, expected in zip(csv_rows, expected_rows, strict=True):
        flat = csv_row(expected)
        require(set(actual) == set(flat), "CSV columns differ")
        for key, value in flat.items():
            if value is None:
                require(actual[key] == "", "CSV singleton segment should be blank")
            else:
                same(float(actual[key]), value, f"CSV {key}")
    analysis = artifacts.read_json(run / "analysis_validation.json")
    require(analysis["source_json_sha256"] == sha256_file(run / "source.json"), "analysis source changed")
    for flag in ("source_hashes_unchanged", "no_robot_controller", "fresh_fixed_all_conditions"):
        require(analysis[flag] is True, f"analysis {flag} failed")
    require(analysis["new_lightnav_inference_count"] == 0 and analysis["condition_count"] == len(expected_rows), "analysis counts differ")
    for output in analysis["outputs"]:
        hash_check(run, output)
    plots = artifacts.read_json(run / "plot_manifest.json")
    hash_check(run, plots["metrics_input"])
    require(plots["config_sha256"] == sha256_file(run / "config_snapshot.yaml"), "plot config mismatch")
    require([entry["path"] for entry in plots["plots"]] == list(IMAGES[1:]), "missing expected plots")
    for entry in plots["plots"]:
        hash_check(run, entry)
        same(entry["tau_s"], motion["tau_s"], "plot tau mismatch")
        for field, key in (("distance_m", entry["metric"]), ("increment_m", "delta_" + entry["metric"])):
            same(entry[field], [r[key] for r in expected_rows], "plot data mismatch")
    return {"R_obs": metadata["R1"], "motion": motion, "conditions": metrics["conditions"],
            "source_arrays": source_record["array_inputs"], "source_run": str(source), "source_validated": True}


def validate_visual(run: Path, details: dict) -> None:
    visual = artifacts.read_json(run / "visualization.json")
    require(visual["source_json_sha256"] == sha256_file(run / "source.json"), "visual source hash mismatch")
    require(visual["config_sha256"] == sha256_file(run / "config_snapshot.yaml"), "visual config mismatch")
    hash_check(run, visual["boundaries_input"])
    require(visual["isaac_viewport_rendered"] is True and visual["metrics_file_used"] is False, "viewport must independently read geometry")
    require(visual["new_lightnav_inference_count"] == 0 and visual["timeline_time_s"] == 0.0, "visual execution not allowed")
    require(visual["geometry_scaling"] == 1 and visual["scene_visibility_modifications"] == [], "geometry/scene altered")
    validate_observation_time(visual["created_time"])
    inventory = visual["scene"]["runtime_inventory"]
    require(inventory["no_robot_model"] is True and not inventory["articulation_roots"]
            and not inventory["robot_named_paths"] and inventory["rigid_body_count"] == 0
            and inventory["physics_scene_count"] == 0 and inventory["dynamics_advanced"] is False, "robot/dynamics present")
    source = Path(details["source_run"])
    scene_config = artifacts.load_config(source / "config_snapshot.yaml")["scene"]
    require(inventory["authored_asset_references"] == [scene_config["asset_relative_path"]], "visual scene differs")
    for key in ("R_obs_world", "agent_pose_before", "agent_pose_after"):
        same(visual[key], details["R_obs"], f"visual {key}", tolerance=1e-7)
    same(visual["boundaries_world"], [r["B_world"] for r in details["conditions"]], "visual boundaries differ")
    same(visual["tau_s"], details["motion"]["tau_s"], "visual tau differs")
    require(visual["fresh_world_instances"] == 1 and visual["observation_to_entry_connector"] is False, "FRESH polyline must be fixed source rows only")
    for name in ("OLD_world", "FRESH_world"):
        hash_check(source, visual["trajectory_inputs"][name])
        require(visual["trajectory_inputs"][name] == details["source_arrays"][name], "visual trajectory provenance mismatch")
        same(visual["drawn_trajectories"][name], np.load(source / artifacts.SOURCE_ARRAYS[name], allow_pickle=False), "drawn trajectory differs")
    hash_check(run, visual["screenshot"])
    review = artifacts.read_json(run / "visual_review.json")
    require(review["reviewed"] is True, "explicit image review missing")
    for key in REVIEW_CHECKS:
        require(review["checks"][key] is True, f"visual review failed: {key}")
    for relative in IMAGES:
        require(review["image_sha256"][relative] == sha256_file(run / relative), "review image hash mismatch")
        with Image.open(run / relative) as image:
            require(image.format == "PNG" and min(image.size) >= 100, "invalid evidence image")
            image.verify()
        with Image.open(run / relative) as image:
            require(float(np.asarray(image).std()) > 1, "blank evidence image")


def validate_run(run: Path) -> dict:
    run = run.resolve()
    required = (*ANALYSIS_FILES, "visualization.json", "visual_review.json", *IMAGES)
    missing = [p for p in required if not (run / p).is_file()]
    failures, details = [], {}
    try:
        if all((run / path).is_file() for path in ANALYSIS_FILES):
            details = validate_analysis(run)
        if not missing:
            validate_visual(run, details)
    except (ValueError, KeyError, TypeError, IndexError, OSError, AttributeError, OverflowError) as error:
        failures.append(f"{type(error).__name__}: {error}")
    return {"schema_version": 1, "run_directory": str(run),
            "status": FAILED if failures else NOT_VALIDATED if missing else VALIDATED,
            "missing_artifacts": missing, "failures": failures,
            "interpretation": "saved actual FRESH plus controlled motion surrogate; not actual asynchronous execution or latency causation",
            **details}


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
