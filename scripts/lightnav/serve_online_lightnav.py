#!/usr/bin/env python3
"""Serve one persistent, preloaded, warmed LightNav agent over a Unix socket."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Any

import numpy as np
from PIL import Image
import torch
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.latency_benchmark import validate_action_array  # noqa: E402
from reconciliation.online_ipc import (  # noqa: E402
    array_payload,
    decode_rgb_message,
    receive_message,
    send_message,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, type=Path)
    parser.add_argument("--ready-file", required=True, type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp01b_online_raw_switch.yaml",
    )
    return parser.parse_args()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        result = json.load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return result


def load_warmup_frames(source: Path, expected: int) -> list[np.ndarray]:
    paths = sorted((source / "raw/rgb").glob("frame_*.png"))
    if len(paths) != expected:
        raise ValueError(f"expected {expected} warm-up frames, found {len(paths)}")
    frames = []
    for path in paths:
        with Image.open(path) as image:
            frame = np.asarray(image.convert("RGB"), dtype=np.uint8)
        if frame.shape[2:] != (3,) or frame.dtype != np.uint8:
            raise ValueError(f"invalid warm-up RGB frame: {path}")
        frames.append(np.ascontiguousarray(frame))
    return frames


def checkpoint_revision(checkpoint: Path) -> str | None:
    tree_dir = checkpoint / ".cache/huggingface/trees"
    trees = sorted(tree_dir.glob("*.json")) if tree_dir.is_dir() else []
    return trees[-1].stem if trees else None


def gpu_snapshot() -> dict[str, Any]:
    query = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.total,memory.used,memory.free,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        text=True,
    ).strip()
    name, total, used, free, utilization = [item.strip() for item in query.split(",")]
    return {
        "gpu_name": name,
        "memory_total_mib": int(total),
        "memory_used_mib": int(used),
        "memory_free_mib": int(free),
        "utilization_percent": int(utilization),
    }


def prefix_caching(agent: Any) -> bool | None:
    lightnav_llm = getattr(agent.engine, "vllm_engine", None)
    config = getattr(getattr(lightnav_llm, "llm_engine", None), "vllm_config", None)
    value = getattr(getattr(config, "cache_config", None), "enable_prefix_caching", None)
    return None if value is None else bool(value)


def send_error(connection: socket.socket, request_id: Any, error: Exception) -> None:
    send_message(
        connection,
        {
            "type": "error",
            "request_id": request_id,
            "error": f"{type(error).__name__}: {error}",
        },
    )


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config must contain a mapping")
    paths = config["paths"]
    lightnav = config["lightnav"]
    instruction = str(config["instruction"])
    if bool(lightnav["intrinsic_waypoint_time_base"]):
        raise ValueError("EXP-01B must not fabricate a LightNav waypoint time base")
    checkout = resolve_path(paths["lightnav_checkout"])
    checkpoint = (checkout / str(paths["checkpoint_relative_path"])).resolve()
    warmup_source = resolve_path(paths["warmup_source_run"])
    expected_history = int(lightnav["expected_history_frames"])
    expected_horizon = int(lightnav["expected_horizon"])
    prior = load_json(warmup_source / "raw/lightnav_inference.json")
    if prior.get("instruction") != instruction:
        raise ValueError("warm-up input instruction differs from EXP-01B instruction")
    eval_config = load_json(checkpoint / "eval_config.json")
    task = eval_config["tasks"][str(lightnav["task_key"])]
    if int(task["num_history_frames"]) != expected_history:
        raise ValueError("checkpoint history contract differs from EXP-01B config")
    if int(task["predict_horizon"]) != expected_horizon:
        raise ValueError("checkpoint horizon differs from EXP-01B config")
    frames = load_warmup_frames(warmup_source, expected_history)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in the LightNav environment")

    from lightnav.tracking import build_tracking_agent

    load_start = time.monotonic_ns()
    agent = build_tracking_agent(
        model_path=str(checkpoint),
        backend=str(lightnav["backend"]),
        gpu_memory_utilization=float(lightnav["gpu_memory_utilization"]),
        aspect_mode=str(lightnav["aspect_mode"]),
        task_key=str(lightnav["task_key"]),
    )
    model_load_ms = (time.monotonic_ns() - load_start) / 1e6
    agent.reset(instruction=instruction)
    for frame in frames:
        agent.observe(frame)
    warm_start = time.monotonic_ns()
    warm_actions_raw, warm_text, warm_reported_ms = agent.predict_waypoints(
        instruction, task_type=str(lightnav["task_type"])
    )
    warm_host_ms = (time.monotonic_ns() - warm_start) / 1e6
    warm_actions = validate_action_array(warm_actions_raw, expected_horizon=expected_horizon)
    checkout_sha = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
    ).strip()
    server_info = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "server_pid": os.getpid(),
        "model_build_call_count": 1,
        "model_load_ms": model_load_ms,
        "warmup": {
            "method": "saved validated Stage 0-C 64-frame RGB history",
            "source_run": str(warmup_source),
            "host_latency_ms": warm_host_ms,
            "lightnav_reported_latency_ms": float(warm_reported_ms),
            "action_shape": list(warm_actions.shape),
            "action_dtype": str(warm_actions.dtype),
            "raw_text_length": len(warm_text),
        },
        "lightnav_sha": checkout_sha,
        "lightnav_checkout_clean": subprocess.check_output(
            ["git", "-C", str(checkout), "status", "--porcelain"], text=True
        ).strip()
        == "",
        "lightnav_package_version": importlib.metadata.version("lightnav"),
        "checkpoint": str(checkpoint),
        "checkpoint_identifier": str(lightnav["checkpoint_identifier"]),
        "checkpoint_revision": checkpoint_revision(checkpoint),
        "backend": str(lightnav["backend"]),
        "vllm_version": importlib.metadata.version("vllm"),
        "torch_version": torch.__version__,
        "gpu_compute_capability": list(torch.cuda.get_device_capability(0)),
        "gpu_memory_utilization_config": float(lightnav["gpu_memory_utilization"]),
        "prior_gpu_memory_utilization_config": float(lightnav["prior_gpu_memory_utilization"]),
        "kv_cache_gib": float(os.environ.get("VLN_KV_CACHE_GIB", lightnav["kv_cache_gib"])),
        "vllm_prefix_caching": prefix_caching(agent),
        "gpu_after_warmup": gpu_snapshot(),
        "episode_reset_count": 0,
        "prediction_count": 0,
    }

    socket_path = args.socket.resolve()
    if socket_path.exists():
        raise FileExistsError(f"socket already exists: {socket_path}")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(socket_path))
    listener.listen(1)
    with args.ready_file.open("x", encoding="utf-8") as stream:
        json.dump(server_info, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print("EXP01B_LIGHTNAV_READY=" + json.dumps(server_info, sort_keys=True), flush=True)

    try:
        connection, _ = listener.accept()
        with connection:
            observed_frames = 0
            while True:
                header, payload = receive_message(connection)
                request_id = header.get("request_id")
                message_type = header["type"]
                try:
                    if message_type == "server_info":
                        server_info["gpu_current"] = gpu_snapshot()
                        send_message(
                            connection,
                            {
                                "type": "server_info_response",
                                "request_id": request_id,
                                "server_info": server_info,
                            },
                        )
                    elif message_type == "episode_reset":
                        episode_instruction = str(header["instruction"])
                        if episode_instruction != instruction:
                            raise ValueError("episode instruction differs from server instruction")
                        agent.reset(instruction=instruction)
                        observed_frames = 0
                        server_info["episode_reset_count"] += 1
                        send_message(
                            connection,
                            {
                                "type": "episode_reset_response",
                                "request_id": request_id,
                                "episode_index": int(header["episode_index"]),
                            },
                        )
                    elif message_type == "observe_rgb":
                        frame = decode_rgb_message(header, payload)
                        agent.observe(frame)
                        observed_frames += 1
                        send_message(
                            connection,
                            {
                                "type": "observe_rgb_response",
                                "request_id": request_id,
                                "observed_frames": observed_frames,
                                "history_buffer_length": int(getattr(agent, "_buffer_len", 0)),
                            },
                        )
                    elif message_type == "predict":
                        if observed_frames < expected_history:
                            raise RuntimeError("episode history is not primed")
                        start_ns = time.monotonic_ns()
                        actions_raw, raw_text, reported_ms = agent.predict_waypoints(
                            instruction, task_type=str(lightnav["task_type"])
                        )
                        end_ns = time.monotonic_ns()
                        actions = validate_action_array(
                            actions_raw, expected_horizon=expected_horizon
                        )
                        server_info["prediction_count"] += 1
                        array_header, action_bytes = array_payload(actions)
                        send_message(
                            connection,
                            {
                                "type": "predict_response",
                                "request_id": request_id,
                                "prediction_kind": str(header["prediction_kind"]),
                                "server_predict_start_monotonic_ns": start_ns,
                                "server_predict_end_monotonic_ns": end_ns,
                                "server_predict_host_latency_ms": (end_ns - start_ns) / 1e6,
                                "lightnav_reported_latency_ms": float(reported_ms),
                                "raw_text": raw_text,
                                "observed_frames": observed_frames,
                                "history_buffer_length": int(getattr(agent, "_buffer_len", 0)),
                                "internal_timings_ms": dict(
                                    getattr(agent.engine, "_last_generate_timings", {}) or {}
                                ),
                                **array_header,
                            },
                            action_bytes,
                        )
                    elif message_type == "shutdown":
                        send_message(
                            connection,
                            {"type": "shutdown_response", "request_id": request_id},
                        )
                        break
                    else:
                        raise ValueError(f"unsupported message type: {message_type}")
                except Exception as error:
                    send_error(connection, request_id, error)
    finally:
        listener.close()
        socket_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
