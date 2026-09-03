#!/usr/bin/env python3
"""Run one real LightNav checkpoint inference in the external Python 3.11 environment."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image
import torch
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_lightnav_single_chunk.yaml",
    )
    parser.add_argument("--checkpoint", type=Path)
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def resolve_from_repository(path: str) -> Path:
    value = Path(path).expanduser()
    return value if value.is_absolute() else (REPOSITORY_ROOT / value).resolve()


def checkpoint_revision(checkpoint: Path) -> str | None:
    tree_dir = checkpoint / ".cache/huggingface/trees"
    trees = sorted(tree_dir.glob("*.json")) if tree_dir.is_dir() else []
    return trees[-1].stem if trees else None


def save_npy_exclusive(path: Path, value: np.ndarray) -> None:
    with path.open("xb") as stream:
        np.save(stream, value, allow_pickle=False)


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config must contain a mapping")
    lightnav_cfg = config["lightnav"]
    paths = config["paths"]
    checkout = resolve_from_repository(str(paths["lightnav_checkout"]))
    checkpoint = args.checkpoint or checkout / str(paths["checkpoint_relative_path"])
    checkpoint = checkpoint.expanduser().resolve()
    if not (checkpoint / "config.json").is_file():
        raise FileNotFoundError(f"LightNav checkpoint is incomplete: {checkpoint}")
    eval_config = load_json(checkpoint / "eval_config.json")
    task_key = str(lightnav_cfg["task_key"])
    task_config = eval_config["tasks"][task_key]
    manifest = load_json(checkpoint / task_config["action_tokenizer"]["bundle_path"] / "manifest.json")
    expected_horizon = int(lightnav_cfg["expected_horizon"])
    expected_history = int(lightnav_cfg["expected_history_frames"])
    if int(task_config["predict_horizon"]) != expected_horizon:
        raise ValueError("configured horizon differs from checkpoint eval_config.json")
    if int(manifest["horizon"]) != expected_horizon:
        raise ValueError("checkpoint decoder horizon differs from eval_config.json")
    if int(task_config["num_history_frames"]) != expected_history:
        raise ValueError("configured history length differs from checkpoint eval_config.json")
    if bool(lightnav_cfg["intrinsic_waypoint_time_base"]):
        raise ValueError("LightNav actions do not have an intrinsic waypoint time base")

    raw_dir = args.run_directory.resolve() / "raw"
    image_paths = sorted((raw_dir / "rgb").glob("frame_*.png"))
    if len(image_paths) != expected_history:
        raise ValueError(f"expected {expected_history} RGB frames, found {len(image_paths)}")
    frames = []
    for path in image_paths:
        frame = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError(f"invalid HWC uint8 RGB frame: {path} {frame.shape} {frame.dtype}")
        frames.append(frame)
    if len({frame.shape for frame in frames}) != 1:
        raise ValueError("all LightNav input frames must have the same HWC shape")
    observation = load_json(raw_dir / "observation_metadata.json")
    instruction = str(config["instruction"])
    if observation.get("instruction") != instruction:
        raise ValueError("capture instruction differs from inference instruction")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in the LightNav environment")

    from lightnav.tracking import build_tracking_agent

    load_start = time.monotonic_ns()
    agent = build_tracking_agent(
        model_path=str(checkpoint),
        backend=str(lightnav_cfg["backend"]),
        gpu_memory_utilization=float(lightnav_cfg["gpu_memory_utilization"]),
        aspect_mode=str(lightnav_cfg["aspect_mode"]),
        task_key=task_key,
    )
    model_load_ms = (time.monotonic_ns() - load_start) / 1_000_000.0
    agent.reset(instruction=instruction)
    for frame in frames:
        agent.observe(frame)

    host_start_wall = datetime.now(timezone.utc).isoformat()
    host_start_ns = time.monotonic_ns()
    waypoints, raw_text, reported_latency_ms = agent.predict_waypoints(
        instruction,
        task_type=str(lightnav_cfg["task_type"]),
    )
    host_end_ns = time.monotonic_ns()
    host_end_wall = datetime.now(timezone.utc).isoformat()
    actions = np.asarray(waypoints)
    if actions.shape != (expected_horizon, 3) or not np.issubdtype(actions.dtype, np.floating):
        raise ValueError(f"unexpected LightNav action shape/dtype: {actions.shape} {actions.dtype}")
    if not np.all(np.isfinite(actions)):
        raise ValueError("LightNav returned NaN or Inf actions")

    checkout_sha = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
    ).strip()
    checkout_status = subprocess.check_output(
        ["git", "-C", str(checkout), "status", "--porcelain"], text=True
    ).strip()
    metadata = {
        "lightnav_checkout_sha": checkout_sha,
        "lightnav_checkout_clean": checkout_status == "",
        "lightnav_package_version": importlib.metadata.version("lightnav"),
        "checkpoint_path": str(checkpoint),
        "checkpoint_identifier": "LightOriginsHQ/LightNav-0",
        "checkpoint_revision": checkpoint_revision(checkpoint),
        "backend": str(lightnav_cfg["backend"]),
        "instruction": instruction,
        "task": str(lightnav_cfg["task"]),
        "task_key": task_key,
        "horizon": expected_horizon,
        "raw_shape": list(actions.shape),
        "raw_dtype": str(actions.dtype),
        "host_inference_monotonic_start_ns": host_start_ns,
        "host_inference_monotonic_end_ns": host_end_ns,
        "host_inference_start_utc": host_start_wall,
        "host_inference_end_utc": host_end_wall,
        "latency_ms": (host_end_ns - host_start_ns) / 1_000_000.0,
        "lightnav_reported_latency_ms": float(reported_latency_ms),
        "model_load_and_warmup_ms": model_load_ms,
        "decoded_action_semantics": "absolute_poses_in_observation_robot_frame",
        "decoder_internal_representation": str(manifest["representation"]),
        "decoder_conversion": (
            "official RVQ decoder SE(2)-composes se2_diff features before public API return"
        ),
        "axis_convention": {
            "columns": ["forward_m", "lateral_m", "yaw_rad"],
            "forward_positive": "robot forward",
            "lateral_positive": "robot left",
            "yaw_positive": "counter-clockwise",
        },
        "identity_pose_stored": False,
        "intrinsic_waypoint_time_base": False,
        "time_base_statement": (
            "video_fps governs observation history; decoded waypoint rows have no intrinsic timestamps"
        ),
        "video_fps": float(task_config["video_fps"]),
        "num_history_frames": int(task_config["num_history_frames"]),
        "input_frame_shape_hwc": list(frames[0].shape),
        "input_frame_dtype": str(frames[0].dtype),
        "torch_version": torch.__version__,
        "torch_cuda_version": torch.version.cuda,
        "vllm_version": importlib.metadata.version("vllm"),
        "gpu_name": torch.cuda.get_device_name(0),
        "gpu_compute_capability": list(torch.cuda.get_device_capability(0)),
        "observation_time_s": observation["observation_time_s"],
        "robot_pose_at_observation": observation["robot_pose_at_observation"],
    }
    save_npy_exclusive(raw_dir / "lightnav_actions.npy", actions)
    with (raw_dir / "lightnav_raw_text.txt").open("x", encoding="utf-8") as stream:
        stream.write(raw_text)
    with (raw_dir / "lightnav_inference.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print("LIGHTNAV_SINGLE_CHUNK_JSON=" + json.dumps(metadata, sort_keys=True))
    np.set_printoptions(precision=6, suppress=True)
    print(actions)


if __name__ == "__main__":
    main()
