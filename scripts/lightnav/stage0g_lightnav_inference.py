#!/usr/bin/env python3
"""Run one primary LightNav inference for each frozen Stage 0-G Q/V case."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from PIL import Image
import torch
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import lightnav_local_to_world, save_json_exclusive, save_npy_exclusive, validate_raw_lightnav_actions
from reconciliation.stage0g_lightnav_scene_qualification import (
    SCENARIO_IDS, VARIANT_IDS, classify_descriptors, geometry_descriptors,
    geometry_signature, sha256_file, validate_config,
)


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--smoke", action="store_true", help="Q0/Q1/Q2 V0 only")
    group.add_argument("--remaining", action="store_true", help="all not-yet-inferred frozen cases")
    group.add_argument("--all", action="store_true", help="all 30 cases in a fresh run")
    group.add_argument("--cases", help="comma-separated Q/V pairs")
    return parser.parse_args()


def load_config(run: Path) -> dict:
    with (run / "config_snapshot.yaml").open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("frozen config must be a mapping")
    validate_config(value)
    freeze = json.loads((run / "config_freeze.json").read_text())
    if sha256_file(run / "config_snapshot.yaml") != freeze["config_sha256"]:
        raise ValueError("frozen Stage 0-G config hash mismatch")
    return value


def resolve(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def parse_cases(options: argparse.Namespace, run: Path) -> list[tuple[str, str]]:
    all_cases = [(q, v) for q in SCENARIO_IDS for v in VARIANT_IDS]
    if options.smoke:
        selected = [(SCENARIO_IDS[index], "V0") for index in range(3)]
    elif options.remaining:
        selected = [(q, v) for q, v in all_cases if not (run / "qualification" / q / v / "raw/lightnav.npy").exists()]
    elif options.all:
        selected = all_cases
    else:
        selected = []
        for token in str(options.cases).split(","):
            pieces = token.strip().split("/")
            if len(pieces) != 2 or pieces[0] not in SCENARIO_IDS or pieces[1] not in VARIANT_IDS:
                raise ValueError(f"invalid Q/V case: {token}")
            selected.append((pieces[0], pieces[1]))
    if len(set(selected)) != len(selected):
        raise ValueError("duplicate requested Q/V case")
    existing = list(run.glob("qualification/Q*/V*/raw/lightnav.npy"))
    if len(existing) + len(selected) > 30:
        raise ValueError("Stage 0-G primary inference budget would exceed 30 outputs")
    for q, v in selected:
        if (run / "qualification" / q / v / "raw/lightnav.npy").exists():
            raise FileExistsError(f"primary output already exists for {q}/{v}")
    return selected


def checkpoint_revision(checkpoint: Path) -> str | None:
    trees = sorted((checkpoint / ".cache/huggingface/trees").glob("*.json"))
    return trees[-1].stem if trees else None


def main() -> None:
    options = args()
    run = options.run_directory.resolve()
    config = load_config(run)
    selected = parse_cases(options, run)
    if not selected:
        print("STAGE0G_INFERENCE nothing_to_do=true")
        return
    checkout = resolve(str(config["paths"]["lightnav_checkout"]))
    checkpoint = checkout / str(config["paths"]["checkpoint_relative_path"])
    sha = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    status = subprocess.check_output(["git", "-C", str(checkout), "status", "--porcelain"], text=True).strip()
    if sha != config["baseline"]["expected_lightnav_git_sha"] or status:
        raise ValueError("LightNav checkout differs from audited clean baseline")
    if checkpoint_revision(checkpoint) != config["baseline"]["expected_checkpoint_revision"]:
        raise ValueError("checkpoint revision differs from frozen input contract")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in the isolated LightNav environment")
    from lightnav.tracking import build_tracking_agent

    load_start = time.monotonic_ns()
    agent = build_tracking_agent(
        model_path=str(checkpoint), backend=str(config["lightnav"]["backend"]),
        gpu_memory_utilization=float(config["lightnav"]["gpu_memory_utilization"]),
        aspect_mode=str(config["lightnav"]["aspect_mode"]), task_key=str(config["lightnav"]["task_key"]),
    )
    model_load_ms = (time.monotonic_ns() - load_start) / 1_000_000.0
    scenarios = {item["id"]: item for item in config["scenarios"]}
    for number, (scenario_id, variant_id) in enumerate(selected, start=1):
        case = run / "qualification" / scenario_id / variant_id
        capture = json.loads((case / "input/capture_metadata.json").read_text())
        instruction = str(scenarios[scenario_id]["instruction"])
        if capture["instruction"] != instruction:
            raise ValueError(f"capture/inference instruction mismatch for {scenario_id}/{variant_id}")
        paths = sorted((case / "input/history").glob("frame_*.png"))
        if len(paths) != int(config["capture"]["frame_count"]):
            raise ValueError(f"wrong frame count for {scenario_id}/{variant_id}")
        frames = []
        for path in paths:
            frame = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
            expected_shape = (int(config["camera"]["resolution_height"]), int(config["camera"]["resolution_width"]), 3)
            if frame.shape != expected_shape or frame.dtype != np.uint8:
                raise ValueError(f"invalid HWC uint8 RGB frame: {path} {frame.shape} {frame.dtype}")
            frames.append(frame)
        agent.reset(instruction=instruction)
        for frame in frames:
            agent.observe(frame)
        started_wall = datetime.now(timezone.utc).isoformat()
        started_ns = time.monotonic_ns()
        actions, raw_text, reported_ms = agent.predict_waypoints(instruction, task_type=str(config["lightnav"]["task_type"]))
        ended_ns = time.monotonic_ns()
        raw = validate_raw_lightnav_actions(np.asarray(actions), expected_horizon=int(config["lightnav"]["expected_horizon"]))
        raw_dir = case / "raw"
        derived = case / "derived"
        raw_dir.mkdir(exist_ok=False)
        derived.mkdir(exist_ok=False)
        save_npy_exclusive(raw_dir / "lightnav.npy", raw)
        with (raw_dir / "lightnav_raw_text.txt").open("x", encoding="utf-8") as stream:
            stream.write(str(raw_text))
        observation_pose = capture["robot_pose_at_observation"]
        world = lightnav_local_to_world(raw, observation_pose)
        save_npy_exclusive(derived / "trajectory_world.npy", world)
        descriptors = geometry_descriptors(raw, stop_path_length_m=float(config["qualification"]["stop_path_length_m"]))
        descriptors.update(classify_descriptors(descriptors, str(scenarios[scenario_id]["intended_class"]), config["qualification"]))
        descriptors["geometry_signature"] = geometry_signature(descriptors)
        metadata = {
            "stage": config["stage"], "scenario_id": scenario_id, "variant_id": variant_id,
            "instruction": instruction, "intended_class": scenarios[scenario_id]["intended_class"],
            "raw_shape": list(raw.shape), "raw_dtype": str(raw.dtype),
            "decoded_action_semantics": config["lightnav"]["decoded_output_semantics"],
            "axis_convention": config["lightnav"]["axes"], "intrinsic_waypoint_time_base": False,
            "world_transform": "T_world_i = T_world_robot_at_observation * T_robot_i",
            "robot_pose_at_observation": observation_pose,
            "host_inference_start_utc": started_wall, "host_inference_latency_ms": (ended_ns - started_ns) / 1_000_000.0,
            "lightnav_reported_latency_ms": float(reported_ms), "model_load_ms_for_this_process": model_load_ms,
            "primary_output_ordinal_in_process": number,
        }
        save_json_exclusive(case / "metadata.json", metadata)
        save_json_exclusive(case / "descriptors.json", descriptors)
        save_json_exclusive(case / "provenance.json", {
            "raw_lightnav_sha256": sha256_file(raw_dir / "lightnav.npy"),
            "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
            "latest_rgb_sha256": sha256_file(case / "input/latest_rgb.png"),
            "lightnav_checkout_sha": sha, "lightnav_checkout_clean": True,
            "checkpoint_identifier": config["baseline"]["checkpoint_identifier"],
            "checkpoint_revision": checkpoint_revision(checkpoint),
            "checkpoint_file_sha256": config["baseline"]["expected_file_sha256"],
            "lightnav_package_version": importlib.metadata.version("lightnav"),
            "torch_version": torch.__version__, "torch_cuda_version": torch.version.cuda,
            "vllm_version": importlib.metadata.version("vllm"), "gpu_name": torch.cuda.get_device_name(0),
        })
        print(
            f"STAGE0G_INFERENCE case={scenario_id}/{variant_id} match={descriptors['matches_intended_geometry']} "
            f"signature={descriptors['geometry_signature']} latency_ms={(ended_ns-started_ns)/1_000_000.0:.1f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
