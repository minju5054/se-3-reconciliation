#!/usr/bin/env python3
"""Strictly validate a completed paired Stage 0-G3 moving-history run."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import lightnav_local_to_world, save_json_exclusive
from reconciliation.stage0g3_moving_history_qualification import (
    FINAL_STATUSES,
    QUALIFICATION_CONDITION_KEYS,
    SCENARIO_IDS,
    VARIANT_IDS,
    class_transition,
    dominant_exact_raw_fraction,
    load_yaml,
    paired_trajectory_metrics,
    qualification_status,
    resolved_scientific_config,
    sha256_file,
    validate_config,
    validate_frozen_control,
    validate_history_poses,
    validate_timestamps,
)


def close(actual: float, expected: float, tolerance: float = 1e-12) -> None:
    if not np.isclose(float(actual), float(expected), atol=tolerance, rtol=0.0):
        raise ValueError(f"metric mismatch: {actual} != {expected}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--check-only", action="store_true")
    options = parser.parse_args(); run = options.run_directory.resolve()
    config = load_yaml(run / "config_snapshot.yaml"); validate_config(config, ROOT)
    scientific = resolved_scientific_config(config, ROOT)
    freeze = json.loads((run / "config_freeze.json").read_text(encoding="utf-8"))
    if sha256_file(run / "config_snapshot.yaml") != freeze.get("config_sha256"):
        raise ValueError("Stage 0-G3 frozen config hash mismatch")
    if sha256_file(run / "approach_feasibility.json") != freeze.get("approach_feasibility_sha256"):
        raise ValueError("Stage 0-G3 frozen feasibility hash mismatch")
    feasibility = json.loads((run / "approach_feasibility.json").read_text(encoding="utf-8"))
    if feasibility.get("status") != "FEASIBLE" or feasibility.get("all_30_histories_feasible") is not True:
        raise ValueError("Stage 0-G3 common approach is not feasible")
    control = validate_frozen_control(config, ROOT); g2_run = Path(control["primary_run"])
    inspection = json.loads((run / "history_previews/visual_inspection.json").read_text(encoding="utf-8"))
    if inspection.get("all_six_v0_contact_sheets_inspected") is not True or inspection.get("inspection_before_any_stage0g3_inference") is not True:
        raise ValueError("pre-inference six-V0 visual inspection is missing")
    if sha256_file(run / "stage0g2_control_reference.json") != freeze.get("g2_control_reference_sha256"):
        raise ValueError("Stage 0-G2 control reference changed after freeze")
    selected = float(freeze["selected_common_distance_m"])
    if selected != float(feasibility["selected_common_distance_m"]):
        raise ValueError("selected common distance mismatch")
    if scientific["robot"]["required_embodiment"] != "Jackal" or scientific["robot"]["asset_relative_path"] != "Clearpath/Jackal/jackal.usd":
        raise ValueError("official Jackal contract changed")
    if scientific["environment"]["asset_relative_path"] != "/Isaac/Environments/Hospital/hospital.usd":
        raise ValueError("Hospital contract changed")
    if scientific["camera"] != load_yaml(ROOT / config["frozen_control"]["config_path"])["camera"]:
        raise ValueError("camera differs from frozen Stage 0-G2")
    if scientific["baseline"]["expected_lightnav_git_sha"] != "a645828d81a8439651172197ca80a75dc1377977" or scientific["baseline"]["expected_checkpoint_revision"] != "7221d418bfff55cfcbadd09f7a26aaab81e1f8a6":
        raise ValueError("LightNav baseline changed")
    named = {}; records = []; final_translation = []; final_yaw = []; history_nonzero = []
    for scenario_id in SCENARIO_IDS:
        for variant_id in VARIANT_IDS:
            name = f"{scenario_id}/{variant_id}"; case = run / "qualification" / scenario_id / variant_id
            g2_case = g2_run / "qualification" / scenario_id / variant_id
            required = (
                "input/history_pose_se2.npy", "input/requested_history_pose_se2.npy", "input/history_pose.csv",
                "input/frame_samples.csv", "input/latest_rgb.png", "input/capture_metadata.json",
                "raw/lightnav.npy", "raw/lightnav_raw_text.txt", "derived/trajectory_world.npy",
                "metadata.json", "descriptors.json", "provenance.json", "paired_control_provenance.json", "paired_metrics.json",
                "plots/contact_sheet.png", "plots/local_trajectory.png", "plots/stationary_vs_moving.png",
            )
            missing = [value for value in required if not (case / value).is_file()]
            if missing: raise ValueError(f"missing Stage 0-G3 case artifacts {name}: {missing}")
            frame_paths = sorted((case / "input/history").glob("frame_*.png"))
            if len(frame_paths) != 64: raise ValueError(f"wrong RGB history frame count: {name}")
            for path in frame_paths:
                with Image.open(path) as image:
                    if image.size != (480, 270) or image.mode != "RGB": raise ValueError(f"invalid RGB frame: {path}")
                    image.verify()
            if sha256_file(frame_paths[-1]) != sha256_file(case / "input/latest_rgb.png"):
                raise ValueError(f"latest RGB is not final moving-history frame: {name}")
            with (case / "input/frame_samples.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            if len(rows) != 64: raise ValueError(f"wrong frame_samples row count: {name}")
            validate_timestamps(np.asarray([float(row["sim_time_s"]) for row in rows]), 4.0)
            poses = np.load(case / "input/history_pose_se2.npy", allow_pickle=False)
            final = np.asarray(control["cases"][name]["final_pose_se2"], dtype=float)
            motion = validate_history_poses(
                poses, final, selected, float(config["history"]["final_translation_tolerance_m"]), float(config["history"]["final_yaw_tolerance_rad"]),
            )
            capture = json.loads((case / "input/capture_metadata.json").read_text(encoding="utf-8"))
            if capture.get("history_mode") != "moving_scripted" or capture.get("scripted_history_is_not_controller_execution") is not True:
                raise ValueError(f"moving-history identity missing: {name}")
            if capture.get("camera") != scientific["camera"] or capture.get("instruction") != control["cases"][name]["instruction"]:
                raise ValueError(f"paired camera/instruction mismatch: {name}")
            camera_error = capture.get("camera_robot_final_pose_mismatch", {})
            if float(camera_error.get("translation_m", 1.0)) > 1e-6 or float(camera_error.get("yaw_rad", 1.0)) > 1e-6:
                raise ValueError(f"final camera/robot sync mismatch: {name}")
            moving = np.load(case / "raw/lightnav.npy", allow_pickle=False); stationary = np.load(g2_case / "raw/lightnav.npy", allow_pickle=False)
            metrics = paired_trajectory_metrics(stationary, moving); saved = json.loads((case / "paired_metrics.json").read_text(encoding="utf-8"))
            for key, value in metrics.items():
                if isinstance(value, bool):
                    if saved.get(key) is not value: raise ValueError(f"paired boolean mismatch {name}/{key}")
                else: close(saved[key], value)
            g2_descriptor = json.loads((g2_case / "descriptors.json").read_text(encoding="utf-8"))
            g3_descriptor = json.loads((case / "descriptors.json").read_text(encoding="utf-8"))
            transition = class_transition(g2_descriptor["geometry_signature"], g3_descriptor["geometry_signature"])
            if (
                saved.get("stationary_raw_sha256") != sha256_file(g2_case / "raw/lightnav.npy")
                or saved.get("moving_raw_sha256") != sha256_file(case / "raw/lightnav.npy")
                or saved.get("stationary_geometry_class") != transition["stationary_class"]
                or saved.get("moving_geometry_class") != transition["moving_class"]
                or saved.get("class_transition") != transition["class_transition"]
                or saved.get("stationary_intended_match") is not bool(g2_descriptor["matches_intended_geometry"])
                or saved.get("moving_intended_match") is not bool(g3_descriptor["matches_intended_geometry"])
            ):
                raise ValueError(f"paired class/hash metadata mismatch: {name}")
            world = np.load(case / "derived/trajectory_world.npy", allow_pickle=False)
            if not np.array_equal(world, lightnav_local_to_world(moving, capture["robot_pose_at_observation"])):
                raise ValueError(f"G3 output was not anchored only at final observation pose: {name}")
            provenance = json.loads((case / "provenance.json").read_text(encoding="utf-8"))
            paired_provenance = json.loads((case / "paired_control_provenance.json").read_text(encoding="utf-8"))
            if provenance.get("raw_lightnav_sha256") != sha256_file(case / "raw/lightnav.npy"):
                raise ValueError(f"G3 raw hash mismatch: {name}")
            if paired_provenance.get("stage0g2_raw_lightnav_sha256") != control["cases"][name]["raw_sha256"] or control["cases"][name]["raw_sha256"] != sha256_file(g2_case / "raw/lightnav.npy"):
                raise ValueError(f"G2 source provenance mismatch: {name}")
            named[name] = moving; records.append(saved)
            final_translation.append(motion["final_translation_mismatch_m"]); final_yaw.append(motion["final_yaw_mismatch_rad"]); history_nonzero.append(motion["nonzero_motion"])
    if len(records) != 30: raise ValueError("expected exactly 30 paired primary cases")
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    comparison = json.loads((run / "stage0g2_vs_stage0g3_summary.json").read_text(encoding="utf-8"))
    result = json.loads((run / "qualification_result.json").read_text(encoding="utf-8"))
    if summary.get("primary_output_count") != 30 or comparison.get("case_count") != 30 or result.get("status") not in FINAL_STATUSES:
        raise ValueError("summary/comparison/result schema invalid")
    if set(result.get("conditions", {})) != QUALIFICATION_CONDITION_KEYS:
        raise ValueError("qualification result does not contain the exact frozen A-J conditions")
    close(summary["max_final_translation_mismatch_m"], max(final_translation)); close(summary["max_final_yaw_mismatch_rad"], max(final_yaw))
    if not all(history_nonzero): raise ValueError("one or more moving histories did not move")
    close(summary["dominant_exact_raw_fraction"], dominant_exact_raw_fraction(named))
    changed_count = 0
    for name, moving in named.items():
        scenario_id, variant_id = name.split("/")
        stationary = np.load(g2_run / "qualification" / scenario_id / variant_id / "raw/lightnav.npy", allow_pickle=False)
        changed_count += int(not paired_trajectory_metrics(stationary, moving)["exact_raw_equal"])
    if comparison.get("exact_raw_changed_count") != changed_count:
        raise ValueError("paired raw change count mismatch")
    expected_status = qualification_status(result["conditions"])
    if result["status"] != expected_status: raise ValueError("qualification decision mismatch")
    for name in ("history_examples.png", "all_scenarios_stationary_vs_moving.png", "all_variants_by_scenario.png", "intended_match_comparison.png"):
        with Image.open(run / "overview" / name) as image: image.verify()
    if summary.get("visual_inspection") != {"all_six_v0_contact_sheets": True, "all_six_v0_stationary_vs_moving": True, "all_overviews": True}:
        raise ValueError("required visual inspection was not recorded")
    validation = {
        "valid": True, "stage": config["stage"], "primary_output_count": 30,
        "g2_control_valid": True, "g2_config_sha256": control["config_sha256"],
        "selected_common_distance_m": selected, "all_final_poses_equal": True,
        "maximum_final_translation_mismatch_m": max(final_translation), "maximum_final_yaw_mismatch_rad": max(final_yaw),
        "all_histories_nonzero_and_64_frames": True, "camera_frozen": True, "lightnav_baseline_frozen": True,
        "official_jackal_required": True, "hospital_required": True, "no_alternate_embodiment": True,
        "raw_outputs_hash_verified": True, "source_g2_immutable_and_hash_verified": True,
        "no_waypoint_intrinsic_time_base": True, "summary_status": result["status"],
    }
    if not options.check_only: save_json_exclusive(run / "validation.json", validation)
    print(f"STAGE0G3_VALIDATION=PASS cases=30 status={result['status']} check_only={str(options.check_only).lower()}")


if __name__ == "__main__": main()
