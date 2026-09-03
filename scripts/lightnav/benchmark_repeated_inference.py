#!/usr/bin/env python3
"""Benchmark first and warm LightNav predictions with one model build."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
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

from reconciliation.latency_benchmark import (  # noqa: E402
    build_summary,
    make_trial_record,
    save_json_exclusive,
    save_trials_csv_exclusive,
    validate_action_array,
    validate_benchmark_output,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/exp01a_lightnav_latency.yaml",
    )
    parser.add_argument("--source-run", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--benchmark-id")
    return parser.parse_args()


def load_json_object(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(
            stream,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def checkpoint_revision(checkpoint: Path) -> str | None:
    tree_dir = checkpoint / ".cache/huggingface/trees"
    trees = sorted(tree_dir.glob("*.json")) if tree_dir.is_dir() else []
    return trees[-1].stem if trees else None


def load_frames(source_run: Path, expected_count: int) -> list[np.ndarray]:
    paths = sorted((source_run / "raw/rgb").glob("frame_*.png"))
    if len(paths) != expected_count:
        raise ValueError(f"expected {expected_count} RGB frames, found {len(paths)}")
    frames: list[np.ndarray] = []
    for path in paths:
        with Image.open(path) as image:
            frame = np.asarray(image.convert("RGB"), dtype=np.uint8)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError(f"invalid HWC uint8 RGB frame: {path}")
        frames.append(frame)
    if len({frame.shape for frame in frames}) != 1:
        raise ValueError("all RGB frames must have one HWC shape")
    return frames


def save_npy_exclusive(path: Path, actions: np.ndarray) -> None:
    with path.open("xb") as stream:
        np.save(stream, actions, allow_pickle=False)


def vllm_prefix_caching(agent: Any) -> bool | None:
    lightnav_llm = getattr(agent.engine, "vllm_engine", None)
    candidates = (
        getattr(getattr(lightnav_llm, "llm_engine", None), "vllm_config", None),
        getattr(lightnav_llm, "vllm_config", None),
    )
    for config in candidates:
        cache_config = getattr(config, "cache_config", None)
        value = getattr(cache_config, "enable_prefix_caching", None)
        if value is not None:
            return bool(value)
    return None


def vit_cache_entry_count(agent: Any) -> int | None:
    # Direct TrackingAgent.predict_waypoints() uses the engine cache. The agent-owned
    # cache is reserved for the batched service path and remains empty here.
    cache = getattr(agent.engine, "_vit_cache", None)
    if cache is None or not hasattr(cache, "cached_keys"):
        return None
    return len(cache.cached_keys())


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config must contain a mapping")
    input_cfg = config["input"]
    lightnav_cfg = config["lightnav"]
    benchmark_cfg = config["benchmark"]
    warm_trials = int(benchmark_cfg["warm_trials"])
    if warm_trials < 1:
        raise ValueError("warm_trials must be positive")
    if bool(benchmark_cfg["intrinsic_waypoint_time_base"]):
        raise ValueError("EXP-01A must not fabricate a waypoint time base")

    source_run = resolve_path(args.source_run or input_cfg["source_run_directory"])
    checkout = resolve_path(lightnav_cfg["checkout"])
    checkpoint = (
        args.checkpoint.expanduser().resolve()
        if args.checkpoint
        else (checkout / str(lightnav_cfg["checkpoint_relative_path"])).resolve()
    )
    output_root = resolve_path(args.output_root or benchmark_cfg["output_root"])
    metrics_path = resolve_path(benchmark_cfg["stage0c_execution_metrics"])
    if not (checkpoint / "config.json").is_file():
        raise FileNotFoundError(f"LightNav checkpoint is incomplete: {checkpoint}")

    observation = load_json_object(source_run / "raw/observation_metadata.json")
    prior_inference = load_json_object(source_run / "raw/lightnav_inference.json")
    execution_metrics = load_json_object(metrics_path)
    instruction = str(input_cfg["instruction"])
    if prior_inference.get("instruction") != instruction:
        raise ValueError("configured instruction differs from validated Stage 0-C inference")
    expected_history = int(input_cfg["expected_history_frames"])
    expected_horizon = int(lightnav_cfg["expected_horizon"])
    eval_config = load_json_object(checkpoint / "eval_config.json")
    task_config = eval_config["tasks"][str(lightnav_cfg["task_key"])]
    if int(task_config["num_history_frames"]) != expected_history:
        raise ValueError("history length differs from checkpoint eval_config.json")
    if int(task_config["predict_horizon"]) != expected_horizon:
        raise ValueError("horizon differs from checkpoint eval_config.json")
    if float(task_config["video_fps"]) != float(input_cfg["expected_video_fps"]):
        raise ValueError("video FPS differs from checkpoint eval_config.json")
    frames = load_frames(source_run, expected_history)
    stage0c_duration_s = float(execution_metrics["execution_duration_s"])
    if not np.isfinite(stage0c_duration_s) or stage0c_duration_s <= 0.0:
        raise ValueError("Stage 0-C execution duration must be finite and positive")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in the LightNav environment")

    benchmark_id = args.benchmark_id or datetime.now(timezone.utc).strftime(
        "exp01a-%Y%m%dT%H%M%SZ"
    )
    run_dir = output_root / benchmark_id
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "actions").mkdir()
    (run_dir / "raw_text").mkdir()

    from lightnav.tracking import build_tracking_agent

    benchmark_start_ns = time.monotonic_ns()
    load_start_ns = time.monotonic_ns()
    agent = build_tracking_agent(
        model_path=str(checkpoint),
        backend=str(lightnav_cfg["backend"]),
        gpu_memory_utilization=float(lightnav_cfg["gpu_memory_utilization"]),
        aspect_mode=str(lightnav_cfg["aspect_mode"]),
        task_key=str(lightnav_cfg["task_key"]),
    )
    model_load_ms = (time.monotonic_ns() - load_start_ns) / 1_000_000.0
    prefix_caching = vllm_prefix_caching(agent)

    trials: list[dict[str, Any]] = []
    for trial_index in range(warm_trials + 1):
        reset_start_ns = time.monotonic_ns()
        agent.reset(instruction=instruction)
        reset_ms = (time.monotonic_ns() - reset_start_ns) / 1_000_000.0
        cache_entries_after_reset = vit_cache_entry_count(agent)
        if cache_entries_after_reset not in (None, 0):
            raise RuntimeError("agent.reset() did not clear the direct inference ViT cache")

        observe_start_ns = time.monotonic_ns()
        for frame in frames:
            agent.observe(frame)
        observe_ms = (time.monotonic_ns() - observe_start_ns) / 1_000_000.0

        predict_start_ns = time.monotonic_ns()
        actions_raw, raw_text, reported_ms = agent.predict_waypoints(
            instruction,
            task_type=str(lightnav_cfg["task_type"]),
        )
        predict_host_ms = (time.monotonic_ns() - predict_start_ns) / 1_000_000.0
        actions = validate_action_array(actions_raw, expected_horizon=expected_horizon)
        internal_timings = dict(getattr(agent.engine, "_last_generate_timings", {}) or {})
        record = make_trial_record(
            trial_index=trial_index,
            trial_kind="first" if trial_index == 0 else "warm",
            reset_ms=reset_ms,
            observe_history_ms=observe_ms,
            predict_host_latency_ms=predict_host_ms,
            lightnav_reported_latency_ms=float(reported_ms),
            actions=actions,
            expected_horizon=expected_horizon,
            raw_text=raw_text,
            input_source_run=str(source_run),
            internal_timings_ms=internal_timings,
            vit_cache_entries_after_predict=vit_cache_entry_count(agent),
        )
        save_npy_exclusive(run_dir / "actions" / f"trial_{trial_index:03d}.npy", actions)
        with (run_dir / "raw_text" / f"trial_{trial_index:03d}.txt").open(
            "x", encoding="utf-8"
        ) as stream:
            stream.write(raw_text)
        trials.append(record)
        print(
            "EXP01A_TRIAL="
            + json.dumps(
                {
                    "trial_index": trial_index,
                    "trial_kind": record["trial_kind"],
                    "host_ms": predict_host_ms,
                    "reported_ms": float(reported_ms),
                    "shape": list(actions.shape),
                },
                sort_keys=True,
            ),
            flush=True,
        )

    total_wall_ms = (time.monotonic_ns() - benchmark_start_ns) / 1_000_000.0
    checkout_sha = subprocess.check_output(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True
    ).strip()
    checkout_status = subprocess.check_output(
        ["git", "-C", str(checkout), "status", "--porcelain"], text=True
    ).strip()
    environment = {
        "backend": str(lightnav_cfg["backend"]),
        "gpu": torch.cuda.get_device_name(0),
        "gpu_compute_capability": list(torch.cuda.get_device_capability(0)),
        "lightnav_sha": checkout_sha,
        "lightnav_checkout_clean": checkout_status == "",
        "lightnav_package_version": importlib.metadata.version("lightnav"),
        "vllm_version": importlib.metadata.version("vllm"),
        "torch_version": torch.__version__,
        "torch_cuda_version": torch.version.cuda,
        "checkpoint": str(checkpoint),
        "checkpoint_identifier": str(lightnav_cfg["checkpoint_identifier"]),
        "checkpoint_revision": checkpoint_revision(checkpoint),
    }
    caching_settings = {
        "vllm_prefix_caching": prefix_caching,
        "vllm_enforce_eager": os.environ.get("VLN_VLLM_ENFORCE_EAGER", "0").lower()
        in ("1", "true", "yes"),
        "vllm_kv_cache_gib": float(os.environ.get("VLN_KV_CACHE_GIB", "2")),
        "vllm_chunked_prefill": False,
        "vllm_multimodal_embeddings": True,
        "lightnav_vit_cache_enabled": bool(getattr(agent.engine, "enable_vit_cache", False)),
        "agent_reset_clears_session_and_engine_vit_cache": True,
        "vllm_video_embedding_hash": "fresh time.monotonic-derived hash per request",
        "cross_trial_image_embedding_cache_expected": False,
        "evidence": [
            "lightnav/inference/policies.py NavigationPolicy.reset clears both caches",
            "lightnav/inference/vllm_utils.py embedding patch creates fresh per-call video hashes",
        ],
        "interpretation": (
            "Identical input is a controlled workload, but reset prevents LightNav ViT cache "
            "reuse and fresh multimodal hashes prevent vLLM image embedding cache reuse."
        ),
    }
    metadata = {
        "schema_version": 1,
        "experiment": str(config["experiment"]),
        "benchmark_id": benchmark_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "controlled_repeated_input": True,
        "source_run_directory": str(source_run),
        "source_observation_time_s": float(observation["observation_time_s"]),
        "instruction": instruction,
        "input_history_shape": [len(frames), *frames[0].shape],
        "input_dtype": str(frames[0].dtype),
        "input_video_fps": float(input_cfg["expected_video_fps"]),
        "expected_horizon": expected_horizon,
        "warm_trials": warm_trials,
        "model_build_call_count": 1,
        "model_load_ms": model_load_ms,
        "total_benchmark_wall_ms": total_wall_ms,
        "latency_definition": "host monotonic_ns around predict_waypoints call",
        "reported_latency_definition": (
            "LightNav vllm_local generation llm_ms returned by predict_waypoints"
        ),
        "gpu_synchronization_added": False,
        "gpu_synchronization_reason": (
            "No extra synchronize: predict_waypoints synchronously returns decoded text/actions "
            "after blocking vLLM LLM.generate."
        ),
        "intrinsic_waypoint_time_base": False,
        "time_base_statement": (
            "Input video_fps is not a decoded action waypoint time base; no waypoint_dt or "
            "ready_time is assigned in EXP-01A."
        ),
        "stage0c_execution_metrics": str(metrics_path),
        "stage0c_execution_duration_s": stage0c_duration_s,
        "environment": environment,
        "caching_settings": caching_settings,
    }
    summary = build_summary(
        model_load_ms=model_load_ms,
        total_benchmark_wall_ms=total_wall_ms,
        trials=trials,
        expected_warm_trials=warm_trials,
        expected_horizon=expected_horizon,
        stage0c_execution_duration_s=stage0c_duration_s,
        environment=environment,
        caching_settings=caching_settings,
    )
    save_trials_csv_exclusive(run_dir / "trials.csv", trials)
    save_json_exclusive(run_dir / "trials.json", {"schema_version": 1, "trials": trials})
    save_json_exclusive(run_dir / "metadata.json", metadata)
    save_json_exclusive(run_dir / "summary.json", summary)
    validation = validate_benchmark_output(run_dir)
    print("EXP01A_OUTPUT_DIR=" + str(run_dir), flush=True)
    print("EXP01A_SUMMARY=" + json.dumps(summary, sort_keys=True), flush=True)
    print("EXP01A_VALIDATION=" + json.dumps(validation, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
