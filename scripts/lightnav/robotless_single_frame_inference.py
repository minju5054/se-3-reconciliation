#!/usr/bin/env python3
"""Send one captured JPEG to an isolated official LightNav WebSocket server.

This client imports no LightNav or Isaac modules. Run it with the existing
external LightNav Python environment for numpy, Pillow, PyYAML and websockets.
It never retries inference: a run directory accepts one request and response.
"""

from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import io
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import numpy as np
from PIL import Image

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.robotless_single_chunk import (  # noqa: E402
    load_config,
    observation_to_world,
    save_json_exclusive,
    save_npy_exclusive,
    sha256_file,
    validate_observation_metadata,
    validate_waypoints,
)


def host_event() -> dict[str, Any]:
    """Host receipt/send audit times; these assign no time to waypoint rows."""
    return {
        "utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "monotonic_ns": time.monotonic_ns(),
    }


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def write_bytes_exclusive(path: Path, value: bytes) -> None:
    with path.open("xb") as stream:
        stream.write(value)


def response_data(raw: str | bytes, expected_action: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict) or value.get("action") != expected_action:
        raise ValueError(f"unexpected {expected_action} response envelope")
    data = value.get("data")
    if not isinstance(data, dict) or type(data.get("rc")) is not int or data["rc"] != 0:
        raise ValueError(f"LightNav {expected_action} failed: {data!r}")
    return data


def parse_prediction(raw: str | bytes, resolution: list[int]) -> tuple[np.ndarray, dict]:
    """Parse only the pinned official nested action protocol, without reshaping."""
    data = response_data(raw, "next")
    if type(data.get("seq")) is not int or data["seq"] != 0:
        raise ValueError("prediction sequence does not match the only request, seq=0")
    actions = data.get("actions")
    if not isinstance(actions, dict) or "actions" not in actions:
        raise ValueError("prediction must contain data.actions.actions")
    if type(data.get("stop")) is not bool:
        raise ValueError("prediction must contain a boolean stop flag")
    pointing = data.get("pointing")
    if pointing is not None and (
        not isinstance(pointing, dict) or pointing.get("frame_size") != resolution
    ):
        raise ValueError("response pointing frame_size differs from original observation")
    # STOP (all-zero or otherwise finite output) is not an interface failure.
    return validate_waypoints(actions["actions"]), data


def exchange_one_frame(
    ws: Any, raw_dir: Path, jpeg: bytes, instruction: str, timeout_s: float
) -> tuple[str | bytes, dict[str, Any]]:
    """One synchronous login/reset/next exchange; persist before payload parsing."""
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("instruction must be nonempty to request actual inference")
    for name in ("lightnav_response.json", "lightnav_request.json", "lightnav_protocol.jsonl"):
        if (raw_dir / name).exists():
            raise FileExistsError(raw_dir / name)
    events: dict[str, Any] = {}
    with (raw_dir / "lightnav_protocol.jsonl").open("x", encoding="utf-8") as protocol:
        def record(direction: str, action: str, wire: str | bytes,
                   event: dict[str, Any] | None = None) -> dict[str, Any]:
            if event is None:
                event = host_event()
            # Preserve wire text rather than reserialize the server's JSON payload.
            record_value = {
                "direction": direction, "action": action, "time": event,
                "wire_text": wire if isinstance(wire, str) else None,
                "wire_bytes_base64": base64.b64encode(wire).decode("ascii")
                if isinstance(wire, bytes) else None,
            }
            protocol.write(json.dumps(record_value, ensure_ascii=False) + "\n")
            protocol.flush()
            return event

        for action, data in (
            ("login", {"clientId": "robotless-single-chunk"}),
            ("reset", {}),
            ("next", {"seq": 0, "image": base64.b64encode(jpeg).decode("ascii"),
                      "instruction": instruction}),
        ):
            request = json.dumps({"action": action, "data": data}, ensure_ascii=False)
            if action == "next":
                write_bytes_exclusive(raw_dir / "lightnav_request.json", request.encode("utf-8"))
            events[f"{action}_request_time"] = record("request", action, request)
            ws.send(request)
            raw = ws.recv(timeout=timeout_s)
            response_time = host_event()
            if action == "next":
                # This is deliberately BEFORE any JSON, envelope, sequence, or shape validation.
                write_bytes_exclusive(
                    raw_dir / "lightnav_response.json",
                    raw.encode("utf-8") if isinstance(raw, str) else raw,
                )
            events[f"{action}_response_time"] = record("response", action, raw, response_time)
            if action != "next":
                response_data(raw, action)
    return raw, events


def git_state(checkout: Path) -> dict[str, Any]:
    def git(*args: str) -> str:
        return subprocess.check_output(["git", "-C", str(checkout), *args], text=True).strip()
    status = git("status", "--porcelain")
    return {"path": str(checkout), "git_sha": git("rev-parse", "HEAD"),
            "clean": status == "", "status_porcelain": status}


def checkpoint_manifest(checkpoint: Path) -> dict[str, Any]:
    """Read and hash model, decoder and configuration artifacts; never modify them."""
    if not (checkpoint / "config.json").is_file() or not (checkpoint / "eval_config.json").is_file():
        raise FileNotFoundError(f"checkpoint config/eval_config missing: {checkpoint}")
    suffixes = {".json", ".safetensors", ".bin", ".pt", ".npy", ".model"}
    files = [path for path in sorted(checkpoint.rglob("*"))
             if path.is_file() and path.suffix in suffixes
             and not any(part.startswith(".") for part in path.relative_to(checkpoint).parts)]
    if not any(path.suffix in {".safetensors", ".bin"} for path in files):
        raise FileNotFoundError(f"checkpoint model weights missing: {checkpoint}")
    entries = []
    for path in files:
        before = path.stat()
        digest = sha256_file(path)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError(f"checkpoint changed while hashing: {path}")
        entries.append({"path": str(path.relative_to(checkpoint)), "sha256": digest,
                        "bytes": after.st_size, "mtime_ns": after.st_mtime_ns})
    return {"path": str(checkpoint), "files": entries}


def verify_checkpoint_unchanged(manifest: dict[str, Any]) -> None:
    for entry in manifest["files"]:
        stat = (Path(manifest["path"]) / entry["path"]).stat()
        if (stat.st_size, stat.st_mtime_ns) != (entry["bytes"], entry["mtime_ns"]):
            raise ValueError(f"checkpoint file changed during inference: {entry['path']}")


def verify_expected_checkpoint_hashes(manifest: dict[str, Any], expected: dict[str, str]) -> None:
    if not isinstance(expected, dict) or not expected:
        raise ValueError("configuration must declare expected checkpoint SHA-256 hashes")
    actual = {entry["path"]: entry["sha256"] for entry in manifest["files"]}
    for relative, digest in expected.items():
        if actual.get(relative) != digest:
            raise ValueError(f"checkpoint SHA-256 mismatch: {relative}")


def resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPOSITORY_ROOT / path).resolve()


def run_inference(run_directory: Path, timeout_s: float = 120.0) -> dict[str, Any]:
    from websockets.sync.client import connect

    if not np.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout must be finite and positive")
    run = run_directory.resolve()
    raw_dir, derived_dir = run / "raw", run / "derived"
    config_path, metadata_path = run / "config_snapshot.yaml", run / "metadata.json"
    server_process_path = run / "server_process.json"
    server_process_sha = sha256_file(server_process_path)
    config = load_config(config_path)
    metadata = read_json(metadata_path)
    validate_observation_metadata(metadata)
    metadata_sha = sha256_file(metadata_path)
    config_sha = sha256_file(config_path)
    if metadata.get("config", {}).get("sha256") != config_sha:
        raise ValueError("capture configuration snapshot hash mismatch")
    if metadata["instruction"] != config["instruction"]:
        raise ValueError("captured instruction differs from configuration snapshot")
    for path in (raw_dir / "lightnav_response.json", raw_dir / "lightnav_request.json",
                 raw_dir / "lightnav_protocol.jsonl", raw_dir / "lightnav_waypoints.npy",
                 derived_dir / "world_waypoints.npy", run / "inference_metadata.json"):
        if path.exists():
            raise FileExistsError(f"one inference per run; existing artifact: {path}")

    image_path = run / metadata["rgb"]["path"]
    if image_path.resolve() != (raw_dir / "observation.jpg").resolve():
        raise ValueError("observation must be raw/observation.jpg in this run")
    jpeg = image_path.read_bytes()
    if hashlib.sha256(jpeg).hexdigest() != metadata["rgb"]["sha256"]:
        raise ValueError("observation JPEG SHA-256 mismatch")
    with Image.open(io.BytesIO(jpeg)) as image:
        if image.format != "JPEG" or image.mode != "RGB":
            raise ValueError("observation must already be an RGB JPEG; no client conversions")
        resolution = list(image.size)
        image.verify()
    if resolution != metadata["rgb"]["resolution_width_height"]:
        raise ValueError("observation JPEG resolution differs from capture metadata")

    lightnav_config = config["lightnav"]
    checkout = resolve_path(config["paths"]["lightnav_checkout"])
    checkpoint = resolve_path(config["paths"]["checkpoint_path"])
    source_before = git_state(checkout)
    if not source_before["clean"] or source_before["git_sha"] != lightnav_config["expected_git_sha"]:
        raise ValueError("LightNav source must be clean and at configured official SHA")
    if metadata["lightnav_git_sha"] != source_before["git_sha"]:
        raise ValueError("capture and client LightNav source SHA differ")
    if metadata["model_checkpoint_identifier"] != lightnav_config["checkpoint_identifier"]:
        raise ValueError("capture and client checkpoint identifiers differ")
    checkpoint_info = checkpoint_manifest(checkpoint)
    verify_expected_checkpoint_hashes(checkpoint_info, lightnav_config.get("checkpoint_sha256"))
    protocol_sources = {
        relative: sha256_file(checkout / relative) for relative in (
            "docs/PROTOCOL.md", "src/lightnav/cli/ws_client.py",
            "src/lightnav/serving/ws_server.py", "src/lightnav/serving/protocol.py",
        )
    }
    derived_dir.mkdir(exist_ok=True)
    with connect(lightnav_config["server_url"], max_size=64 * 1024 * 1024,
                 open_timeout=timeout_s, close_timeout=10) as ws:
        raw, events = exchange_one_frame(ws, raw_dir, jpeg, metadata["instruction"], timeout_s)
    local, response = parse_prediction(raw, resolution)
    # Serialized JSON numbers are preserved as float64, without yaw wrapping locally.
    save_npy_exclusive(raw_dir / "lightnav_waypoints.npy", local)
    world = observation_to_world(metadata["agent_pose_world"], local)
    save_npy_exclusive(derived_dir / "world_waypoints.npy", world)
    verify_checkpoint_unchanged(checkpoint_info)
    source_after = git_state(checkout)
    if source_after != source_before:
        raise ValueError("LightNav source changed during inference")
    if sha256_file(metadata_path) != metadata_sha or sha256_file(config_path) != config_sha:
        raise ValueError("capture metadata or configuration changed during inference")
    if sha256_file(server_process_path) != server_process_sha:
        raise ValueError("recorded server process provenance changed during inference")
    times = [metadata["observation_time"]["monotonic_ns"]]
    times.extend(event["monotonic_ns"] for event in events.values())
    if times != sorted(times):
        raise ValueError("host monotonic event order is inconsistent with observation-first inference")
    result = {
        "schema_version": 1,
        "single_frame_single_request": True,
        "server_url": lightnav_config["server_url"],
        "protocol": "official JSON text WebSocket: login, reset, next(seq=0)",
        "protocol_source_sha256": protocol_sources,
        "research_source_sha256": {
            relative: sha256_file(REPOSITORY_ROOT / relative) for relative in (
                "scripts/lightnav/robotless_single_frame_inference.py",
                "src/reconciliation/robotless_single_chunk.py",
                "src/reconciliation/se2.py",
            )
        },
        "lightnav_source_before": source_before, "lightnav_source_after": source_after,
        "server_process": {"path": "server_process.json", "sha256": server_process_sha},
        "checkpoint_identifier": lightnav_config["checkpoint_identifier"],
        "checkpoint_revision": lightnav_config["checkpoint_revision"],
        "checkpoint": checkpoint_info,
        "provenance_scope": "Local server checkout/checkpoint audit; wire protocol does not attest model identity.",
        "client_python_executable": sys.executable,
        "client_packages": {name: importlib.metadata.version(name)
                            for name in ("numpy", "Pillow", "PyYAML", "websockets")},
        "raw_inputs": {
            "observation": {"path": "raw/observation.jpg", "sha256": metadata["rgb"]["sha256"],
                            "resolution_width_height": resolution},
            "capture_metadata": {"path": "metadata.json", "sha256": metadata_sha},
            "request": {"path": "raw/lightnav_request.json",
                        "sha256": sha256_file(raw_dir / "lightnav_request.json")},
            "response": {"path": "raw/lightnav_response.json",
                         "sha256": sha256_file(raw_dir / "lightnav_response.json")},
            "protocol": {"path": "raw/lightnav_protocol.jsonl",
                         "sha256": sha256_file(raw_dir / "lightnav_protocol.jsonl")},
            "waypoints": {"path": "raw/lightnav_waypoints.npy",
                          "sha256": sha256_file(raw_dir / "lightnav_waypoints.npy")},
        },
        "config": {"path": "config_snapshot.yaml", "sha256": config_sha},
        "processing_source_sha256": {
            relative: sha256_file(REPOSITORY_ROOT / relative) for relative in (
                "scripts/lightnav/robotless_single_frame_inference.py",
                "src/reconciliation/robotless_single_chunk.py", "src/reconciliation/se2.py",
            )
        },
        "instruction": metadata["instruction"],
        "observation_time": metadata["observation_time"],
        "agent_pose_world_at_observation": metadata["agent_pose_world"],
        "events": events, "execution_time": None,
        "event_convention": "Requests stamped before send, responses immediately after recv; UTC wall and host monotonic_ns; no latency benchmark or waypoint time base.",
        "raw_shape": list(local.shape), "raw_dtype": str(local.dtype),
        "stop": response["stop"], "response_seq": response["seq"],
        "response_pointing_frame_size": (response.get("pointing") or {}).get("frame_size"),
        "world_waypoints": {"path": "derived/world_waypoints.npy",
                            "sha256": sha256_file(derived_dir / "world_waypoints.npy")},
        "transform": {
            "source_frame": "logical_agent_at_observation",
            "target_frame": "Isaac_world_z_up", "translation_unit": "meter", "angle_unit": "radian",
            "source_columns": ["forward_m", "left_m", "yaw_ccw_rad"],
            "target_columns": ["world_x_m", "world_y_m", "world_yaw_ccw_rad"],
            "equation": "T_world_waypoint = T_world_agent(t_obs) @ T_agent_waypoint",
            "yaw_wrap": "[-pi, pi)", "raw_array_unchanged": True,
            "waypoint_time_base": None,
        },
        "example_raw_local_waypoint": local[0].tolist(),
        "example_world_waypoint": world[0].tolist(),
        "interface_inference_valid": True,
    }
    save_json_exclusive(run / "inference_metadata.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--timeout-s", type=float, default=120.0,
                        help="Per-operation failure timeout; never triggers an inference retry.")
    args = parser.parse_args()
    result = run_inference(args.run_directory, args.timeout_s)
    print(json.dumps({"raw_shape": result["raw_shape"], "stop": result["stop"],
                      "example_raw_local_waypoint": result["example_raw_local_waypoint"],
                      "example_world_waypoint": result["example_world_waypoint"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
