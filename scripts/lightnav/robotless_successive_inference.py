#!/usr/bin/env python3
"""Two captured observations through one synchronous official LightNav session.

One connection, one login, one reset and next(seq=0), next(seq=1). There is no
retry, reconnect, movement during inference, execution, alignment or correction.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.metadata
import io
import json
from pathlib import Path
import sys
from typing import Any
import uuid

import numpy as np
from PIL import Image

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robotless_single_frame_inference import (
    checkpoint_manifest, git_state, host_event, parse_prediction, read_json,
    resolve_path, response_data, verify_checkpoint_unchanged,
    verify_expected_checkpoint_hashes, write_bytes_exclusive,
)
from reconciliation.robotless_single_chunk import load_config, save_json_exclusive, save_npy_exclusive, sha256_file
from reconciliation.robotless_successive_chunks import (
    round_trip_latency_ms, successive_to_world, validate_successive_observation_metadata,
)


def exchange_successive_frames(
    server_url: str, raw_dir: Path, frames: list[bytes], resolutions: list[list[int]],
    instruction: str, timeout_s: float, *, connect_fn=None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Preserve exact wire bytes before validating each reply in one connection.

    The injected connector is used only by offline protocol tests. On failure,
    all already-created raw artifacts remain and this run cannot be retried.
    """
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("instruction must be nonempty")
    if len(frames) != 2 or len(resolutions) != 2:
        raise ValueError("exactly two captured observations are required")
    if not np.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout must be finite and positive")
    reserved = ["lightnav_protocol.jsonl"] + [
        f"{kind}_{seq:03d}.{suffix}" for seq in (0, 1)
        for kind, suffix in (("request", "json"), ("response", "json"), ("chunk", "npy"))
    ]
    for name in reserved:
        if (raw_dir / name).exists():
            raise FileExistsError(raw_dir / name)
    if connect_fn is None:
        from websockets.sync.client import connect
        connect_fn = connect
    connection_id = str(uuid.uuid4())
    events: dict[str, Any] = {}
    chunks = []
    with (raw_dir / "lightnav_protocol.jsonl").open("x", encoding="utf-8") as protocol:
        def record(direction: str, action: str, wire: str | bytes, event: dict, seq=None) -> None:
            value = {
                "connection_id": connection_id, "direction": direction, "action": action,
                "time": event, "seq": seq,
                "wire_text": wire if isinstance(wire, str) else None,
                "wire_bytes_base64": base64.b64encode(wire).decode("ascii") if isinstance(wire, bytes) else None,
            }
            protocol.write(json.dumps(value, ensure_ascii=False) + "\n")
            protocol.flush()

        with connect_fn(server_url, max_size=64 * 1024 * 1024,
                        open_timeout=timeout_s, close_timeout=10) as ws:
            def exchange(action: str, data: dict, seq=None) -> tuple[str | bytes, dict, dict]:
                request = json.dumps({"action": action, "data": data}, ensure_ascii=False)
                if seq is not None:
                    write_bytes_exclusive(raw_dir / f"request_{seq:03d}.json", request.encode("utf-8"))
                send_time = host_event()
                ws.send(request)
                record("request", action, request, send_time, seq)
                response = ws.recv(timeout=timeout_s)
                receive_time = host_event()
                if seq is not None:
                    write_bytes_exclusive(raw_dir / f"response_{seq:03d}.json",
                                          response.encode("utf-8") if isinstance(response, str) else response)
                record("response", action, response, receive_time, seq)
                prefix = action if seq is None else f"next_{seq:03d}"
                events[f"{prefix}_request_time"] = send_time
                events[f"{prefix}_response_time"] = receive_time
                return response, send_time, receive_time

            login, _, _ = exchange("login", {"clientId": "robotless-successive-chunks"})
            response_data(login, "login")
            reset, _, _ = exchange("reset", {})
            response_data(reset, "reset")
            for seq in (0, 1):
                response, send_time, receive_time = exchange("next", {
                    "seq": seq, "image": base64.b64encode(frames[seq]).decode("ascii"),
                    "instruction": instruction,
                }, seq)
                local, data = parse_prediction(response, resolutions[seq], expected_seq=seq)
                save_npy_exclusive(raw_dir / f"chunk_{seq:03d}.npy", local)
                chunks.append({
                    "seq": seq, "local": local, "response_data": data,
                    "request_send_time": send_time, "response_receive_time": receive_time,
                    "round_trip_latency_ms": round_trip_latency_ms(send_time, receive_time),
                })
    return chunks, {
        "events": events,
        "session": {
            "connection_count": 1, "login_count": 1, "reset_count": 1,
            "next_count": 2, "request_sequence": [0, 1], "connection_id": connection_id,
            "instruction_constant": True, "reconnect_count": 0, "retry_count": 0,
        },
    }


def verify_input_image(run: Path, observation: dict, seq: int) -> tuple[bytes, list[int]]:
    image_path = run / observation["rgb"]["path"]
    if image_path.resolve() != (run / f"raw/observation_{seq:03d}.jpg").resolve():
        raise ValueError("observation must use the matching raw/observation_NNN.jpg path")
    jpeg = image_path.read_bytes()
    if hashlib.sha256(jpeg).hexdigest() != observation["rgb"]["sha256"]:
        raise ValueError("observation JPEG SHA-256 mismatch")
    with Image.open(io.BytesIO(jpeg)) as image:
        if image.format != "JPEG" or image.mode != "RGB":
            raise ValueError("observation must already be an RGB JPEG; no client conversions")
        resolution = list(image.size)
        image.verify()
    if resolution != observation["rgb"]["resolution_width_height"]:
        raise ValueError("observation JPEG resolution differs from capture metadata")
    return jpeg, resolution


def run_inference(run_directory: Path, timeout_s: float = 120.0) -> dict[str, Any]:
    if not np.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout must be finite and positive")
    run = run_directory.resolve()
    raw_dir, derived_dir = run / "raw", run / "derived"
    config_path, metadata_path = run / "config_snapshot.yaml", run / "metadata.json"
    server_process_path = run / "server_process.json"
    config, metadata = load_config(config_path), read_json(metadata_path)
    validate_successive_observation_metadata(metadata)
    metadata_sha, config_sha = sha256_file(metadata_path), sha256_file(config_path)
    server_process_sha = sha256_file(server_process_path)
    if metadata.get("config", {}).get("sha256") != config_sha:
        raise ValueError("capture configuration snapshot hash mismatch")
    if metadata["instruction"] != config["instruction"]:
        raise ValueError("captured instruction differs from configuration snapshot")
    if not np.allclose(metadata["R0"], config["agent"]["pose_world"], rtol=0, atol=1e-7):
        raise ValueError("R0 differs from configuration")
    if not np.array_equal(metadata["configured_local_displacement"], config["agent"]["local_displacement"]):
        raise ValueError("local displacement differs from configuration")
    for path in [run / "inference_metadata.json", raw_dir / "lightnav_protocol.jsonl"] + [
        directory / filename for seq in (0, 1) for directory, filename in (
            (raw_dir, f"request_{seq:03d}.json"), (raw_dir, f"response_{seq:03d}.json"),
            (raw_dir, f"chunk_{seq:03d}.npy"), (derived_dir, f"chunk_{seq:03d}_world.npy"),
        )
    ]:
        if path.exists():
            raise FileExistsError(f"one successive session per run; existing artifact: {path}")
    observations = metadata["observations"]
    verified = [verify_input_image(run, observation, seq) for seq, observation in enumerate(observations)]
    frames, resolutions = [item[0] for item in verified], [item[1] for item in verified]
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
    protocol_sources = {relative: sha256_file(checkout / relative) for relative in (
        "docs/PROTOCOL.md", "src/lightnav/cli/ws_client.py",
        "src/lightnav/serving/ws_server.py", "src/lightnav/serving/protocol.py",
    )}
    decoded, exchange_metadata = exchange_successive_frames(
        lightnav_config["server_url"], raw_dir, frames, resolutions, metadata["instruction"], timeout_s,
    )
    worlds = successive_to_world(
        observations[0]["agent_pose_world"], observations[1]["agent_pose_world"],
        decoded[0]["local"], decoded[1]["local"],
    )
    chunks = []
    for seq, (observation, decoded_chunk, world) in enumerate(zip(observations, decoded, worlds, strict=True)):
        save_npy_exclusive(derived_dir / f"chunk_{seq:03d}_world.npy", world)
        local, response = decoded_chunk["local"], decoded_chunk["response_data"]
        def artifact(relative: str) -> dict:
            return {"path": relative, "sha256": sha256_file(run / relative)}
        chunks.append({
            "seq": seq, "label": observation["label"], "observation_id": observation["observation_id"],
            "observation_time": observation["observation_time"],
            "agent_pose_world_at_observation": observation["agent_pose_world"],
            "raw_shape": list(local.shape), "raw_dtype": str(local.dtype),
            "stop": response["stop"], "response_seq": response["seq"],
            "response_actions_step": response["actions"].get("step"),
            "request_send_time": decoded_chunk["request_send_time"],
            "response_receive_time": decoded_chunk["response_receive_time"],
            "round_trip_latency_ms": decoded_chunk["round_trip_latency_ms"],
            "raw_inputs": {
                "observation": {**artifact(observation["rgb"]["path"]), "resolution_width_height": resolutions[seq]},
                "request": artifact(f"raw/request_{seq:03d}.json"),
                "response": artifact(f"raw/response_{seq:03d}.json"),
                "waypoints": artifact(f"raw/chunk_{seq:03d}.npy"),
            },
            "world_waypoints": artifact(f"derived/chunk_{seq:03d}_world.npy"),
            "example_raw_local_waypoint": local[0].tolist(), "example_world_waypoint": world[0].tolist(),
        })
    verify_checkpoint_unchanged(checkpoint_info)
    source_after = git_state(checkout)
    if source_before != source_after:
        raise ValueError("LightNav source changed during inference")
    for path, expected in ((metadata_path, metadata_sha), (config_path, config_sha), (server_process_path, server_process_sha)):
        if sha256_file(path) != expected:
            raise ValueError(f"capture/server/config provenance changed during inference: {path}")
    for observation in observations:
        if sha256_file(run / observation["rgb"]["path"]) != observation["rgb"]["sha256"]:
            raise ValueError("observation image changed during inference")
    times = [observation["observation_time"]["monotonic_ns"] for observation in observations]
    times.extend(event["monotonic_ns"] for event in exchange_metadata["events"].values())
    if times != sorted(times):
        raise ValueError("host monotonic event order inconsistent with both captures preceding inference")
    sources = {relative: sha256_file(REPOSITORY_ROOT / relative) for relative in (
        "scripts/lightnav/robotless_successive_inference.py", "scripts/lightnav/robotless_single_frame_inference.py",
        "src/reconciliation/robotless_successive_chunks.py", "src/reconciliation/robotless_single_chunk.py", "src/reconciliation/se2.py",
    )}
    result = {
        "schema_version": 1, "stage": config["stage"], "interface_inference_valid": True,
        "server_url": lightnav_config["server_url"], "protocol": "official WebSocket: login, reset, next(seq=0), next(seq=1)",
        "protocol_source_sha256": protocol_sources, "research_source_sha256": sources, "processing_source_sha256": sources,
        "lightnav_source_before": source_before, "lightnav_source_after": source_after,
        "server_process": {"path": "server_process.json", "sha256": server_process_sha},
        "checkpoint_identifier": lightnav_config["checkpoint_identifier"], "checkpoint_revision": lightnav_config["checkpoint_revision"],
        "checkpoint": checkpoint_info,
        "provenance_scope": "Local server checkout/checkpoint audit; wire protocol does not attest model identity.",
        "client_python_executable": sys.executable,
        "client_packages": {name: importlib.metadata.version(name) for name in ("numpy", "Pillow", "PyYAML", "websockets")},
        "raw_inputs": {
            "capture_metadata": {"path": "metadata.json", "sha256": metadata_sha},
            "protocol": {"path": "raw/lightnav_protocol.jsonl", "sha256": sha256_file(raw_dir / "lightnav_protocol.jsonl")},
        },
        "config": {"path": "config_snapshot.yaml", "sha256": config_sha},
        "instruction": metadata["instruction"], "chunks": chunks, **exchange_metadata,
        "event_convention": "Send stamped immediately before ws.send; receive immediately after ws.recv. UTC wall and same-host monotonic_ns are distinct clocks.",
        "round_trip_latency_convention": "(response_receive_monotonic_ns-request_send_monotonic_ns)/1e6 milliseconds; recorded only, not agent motion, switch latency, model-only latency or a latency experiment.",
        "execution_time": None, "agent_motion_during_inference": False,
        "transform": {
            "equations": ["T_world_OLD = T_world_R0(t_obs0) @ T_R0_OLD", "T_world_FRESH = T_world_R1(t_obs1) @ T_R1_FRESH"],
            "source_frames": ["logical_agent_at_observation_000", "logical_agent_at_observation_001"],
            "target_frame": "Isaac_world_z_up", "translation_unit": "meter", "angle_unit": "radian",
            "source_columns": ["forward_m", "left_m", "yaw_ccw_rad"], "target_columns": ["world_x_m", "world_y_m", "world_yaw_ccw_rad"],
            "row_semantics": "cumulative local poses; not increments to integrate", "yaw_wrap": "[-pi, pi)",
            "raw_arrays_unchanged": True, "correction_applied": False, "waypoint_time_base": None,
        },
    }
    save_json_exclusive(run / "inference_metadata.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--timeout-s", type=float, default=120.0, help="Per-operation failure timeout; no retries.")
    args = parser.parse_args()
    result = run_inference(args.run_directory, args.timeout_s)
    print(json.dumps({"session": result["session"], "chunks": [{
        key: chunk[key] for key in ("seq", "label", "raw_shape", "stop", "round_trip_latency_ms", "example_raw_local_waypoint", "example_world_waypoint")
    } for chunk in result["chunks"]]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
