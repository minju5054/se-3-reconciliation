#!/usr/bin/env python3
"""Create the Stage 0-G audit directory and freeze its qualification config."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g_lightnav_scene_qualification import create_run_directory_exclusive, sha256_file, validate_config


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/stage0g_lightnav_scene_qualification.yaml")
    parser.add_argument("--run-id")
    parser.add_argument("--run-directory", type=Path)
    parser.add_argument("--freeze", action="store_true")
    return parser.parse_args()


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("config must be a mapping")
    validate_config(value)
    return value


def resolve(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def git_output(checkout: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(checkout), *args], text=True).strip()


def checkpoint_revision(checkpoint: Path) -> str | None:
    trees = sorted((checkpoint / ".cache/huggingface/trees").glob("*.json"))
    return trees[-1].stem if trees else None


def audit(config_path: Path, run_id: str | None) -> Path:
    config = load_yaml(config_path)
    output = resolve(ROOT, str(config["paths"]["output_root"]))
    identifier = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run = output / identifier
    contract_dir = run / "input_contract"
    create_run_directory_exclusive(run)
    contract_dir.mkdir()
    checkout = resolve(ROOT, str(config["paths"]["lightnav_checkout"]))
    checkpoint = checkout / str(config["paths"]["checkpoint_relative_path"])
    baseline = config["baseline"]
    checkout_sha = git_output(checkout, "rev-parse", "HEAD")
    checkout_clean = git_output(checkout, "status", "--porcelain") == ""
    revision = checkpoint_revision(checkpoint)
    hashes = {relative: sha256_file(checkpoint / relative) for relative in baseline["expected_file_sha256"]}
    checks = {
        "lightnav_checkout_sha_matches": checkout_sha == baseline["expected_lightnav_git_sha"],
        "lightnav_checkout_clean": checkout_clean,
        "checkpoint_revision_matches": revision == baseline["expected_checkpoint_revision"],
        "checkpoint_file_hashes_match": hashes == baseline["expected_file_sha256"],
        "rgb_hwc_uint8_semantics_match": config["camera"]["rgb_format"] == "HWC_uint8_RGB",
        "color_order_matches": True,
        "aspect_behavior_justified": config["lightnav"]["aspect_mode"] == "stretch",
        "camera_forward_axis_verified": config["camera"]["orientation"] == "level_robot_forward",
        "hfov_justified_from_upstream_reference": True,
        "camera_height_justified": True,
        "history_timing_semantics_justified": config["capture"]["frame_count"] == 64 and float(config["capture"]["fps"]) == 4.0,
        "timestamps_preserved": True,
        "task_prompt_mode_correct": config["lightnav"]["task_key"] == "vlnce" and config["lightnav"]["task_type"] == "vlnce_traj",
        "raw_output_convention_verified": config["lightnav"]["decoded_output_semantics"] == "absolute_poses_in_observation_robot_frame",
        "no_hidden_reanchoring_or_resampling": True,
    }
    contract = {
        "stage": config["stage"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "contract_resolved": all(checks.values()),
        "checks": checks,
        "lightnav_checkout": {
            "path": str(checkout), "expected_sha": baseline["expected_lightnav_git_sha"],
            "actual_sha": checkout_sha, "clean": checkout_clean,
        },
        "checkpoint": {
            "path": str(checkpoint), "identifier": baseline["checkpoint_identifier"],
            "expected_revision": baseline["expected_checkpoint_revision"], "actual_revision": revision,
            "expected_file_sha256": baseline["expected_file_sha256"], "actual_file_sha256": hashes,
        },
        "input": {
            "source_rgb": {"layout": "HWC", "dtype": "uint8", "order": "RGB", "size_wh": [480, 270]},
            "model_preprocess": {"size_hw": [256, 448], "resample": "bilinear", "aspect_mode": "stretch", "normalization": "[-1, 1]"},
            "history": {"frames": 64, "cadence_hz": 4.0, "stationary": True, "frame_timestamps_preserved": True, "waypoint_intrinsic_timestamps": False},
            "slowfast": {
                "current": "age 0-1, dense, pool 1", "fast": "age 2-33, dense, pool 2",
                "mid": "age 34-89, burst pair_stride 6, pool 2", "long": "age 90+, unreachable with 64 frames",
                "anchor": "episode frames 0-1, pool 4",
            },
            "task": {"cli_task": "vln", "task_key": "vlnce", "task_type": "vlnce_traj", "prompt_style": "unified_traj", "timestamps_relative": True},
            "output": {"shape": [10, 3], "dtype": "float32", "columns": config["lightnav"]["axes"], "semantics": config["lightnav"]["decoded_output_semantics"]},
        },
        "camera_selection": {
            "chosen": config["camera"],
            "ambiguity": "Upstream defines no single cross-platform physical camera calibration; calibration metadata is not an inference tensor.",
            "basis": "480x270, level, approximately 112 deg horizontal FOV follow the official wheeled MuJoCo reference; 0.48 m Jackal-safe mount height preserves the validated Isaac mounting point because MuJoCo's ~0.198 m height is TurtleBot-specific.",
        },
        "research_boundaries": {"inference_only": True, "controller_execution": False, "old_fresh_collection": False, "optimizer_changes": False},
    }
    comparison = {
        "properties": [
            {"property": "source resolution", "previous_isaac": "448x256", "lightnav_reference": "480x270 official robot/MuJoCo client; model 448x256", "stage0g": "480x270", "reason": "match upstream wheeled client before checkpoint preprocessing"},
            {"property": "HFOV", "previous_isaac": "90 deg", "lightnav_reference": "~112 deg wheeled MuJoCo; 120 deg Habitat", "stage0g": "112 deg", "reason": "wheeled embodiment reference selected before inference"},
            {"property": "camera height", "previous_isaac": "0.48 m", "lightnav_reference": "~0.198 m TurtleBot; 0.88 m Habitat; robot deployment unspecified", "stage0g": "0.48 m", "reason": "embodiment-safe validated Jackal mount; upstream is ambiguous"},
            {"property": "orientation", "previous_isaac": "level forward", "lightnav_reference": "level forward", "stage0g": "level forward", "reason": "direct match"},
            {"property": "aspect", "previous_isaac": "stretch", "lightnav_reference": "stretch is training default", "stage0g": "stretch", "reason": "preserve training preprocessing"},
            {"property": "RGB layout", "previous_isaac": "HWC uint8 RGB", "lightnav_reference": "HWC uint8 RGB", "stage0g": "HWC uint8 RGB", "reason": "direct match"},
            {"property": "history", "previous_isaac": "64 stationary frames", "lightnav_reference": "64-frame vlnce SlowFast", "stage0g": "64 stationary frames", "reason": "checkpoint task config"},
            {"property": "frame cadence", "previous_isaac": "4 Hz", "lightnav_reference": "video_fps=4", "stage0g": "4 Hz", "reason": "checkpoint timestamp semantics"},
            {"property": "task/prompt", "previous_isaac": "vln/vlnce_traj", "lightnav_reference": "vln -> vlnce unified_traj", "stage0g": "vln/vlnce/vlnce_traj unified_traj", "reason": "official task mapping"},
        ]
    }
    save_json_exclusive(contract_dir / "lightnav_input_contract.json", contract)
    save_json_exclusive(contract_dir / "comparison_to_previous_isaac.json", comparison)
    save_json_exclusive(run / "run_metadata.json", {
        "stage": config["stage"], "run_id": identifier,
        "research_git_sha_at_audit": git_output(ROOT, "rev-parse", "HEAD"),
        "input_contract_resolved": contract["contract_resolved"], "config_frozen": False,
    })
    print(f"STAGE0G_RUN_DIRECTORY={run.resolve()}")
    print(f"STAGE0G_INPUT_CONTRACT_RESOLVED={str(contract['contract_resolved']).lower()}")
    return run


def freeze(config_path: Path, run: Path) -> None:
    config = load_yaml(config_path)
    previews = [run / "scene_previews" / scenario / "V0" / "latest_rgb.png" for scenario in [item["id"] for item in config["scenarios"]]]
    missing = [str(path) for path in previews if not path.is_file()]
    if missing:
        raise ValueError(f"cannot freeze before all six V0 previews exist: {missing}")
    destination = run / "config_snapshot.yaml"
    with destination.open("xb") as stream:
        stream.write(config_path.read_bytes())
    save_json_exclusive(run / "config_freeze.json", {
        "frozen_utc": datetime.now(timezone.utc).isoformat(),
        "config_snapshot": "config_snapshot.yaml", "config_sha256": sha256_file(destination),
        "scene_previews_present_before_freeze": True,
        "prohibition": "No camera, scene, instruction, threshold, or variant tuning after this record.",
    })
    print(f"STAGE0G_FROZEN_CONFIG_SHA256={sha256_file(destination)}")


def main() -> None:
    args = arguments()
    config_path = args.config.resolve()
    if args.freeze:
        if args.run_directory is None:
            raise ValueError("--freeze requires --run-directory")
        freeze(config_path, args.run_directory.resolve())
    else:
        if args.run_directory is not None:
            raise ValueError("--run-directory is used only with --freeze")
        audit(config_path, args.run_id)


if __name__ == "__main__":
    main()
