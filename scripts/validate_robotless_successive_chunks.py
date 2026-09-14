#!/usr/bin/env python3
"""Audit successive robotless runtime artifacts; fixtures cannot establish PASS."""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from reconciliation.robotless_single_chunk import (  # noqa: E402
    load_config, observation_to_world, save_json_exclusive, sha256_file,
    validate_observation_time, validate_waypoints,
)
from reconciliation.robotless_successive_chunks import (  # noqa: E402
    round_trip_latency_ms, successive_observation_poses,
    validate_successive_observation_metadata,
)
from reconciliation.se2 import relative_pose  # noqa: E402
from validate_robotless_single_chunk import (  # noqa: E402
    artifact, check_camera, check_external_provenance, check_hash,
    protocol_wire, read_json, require, same_array, true_flags,
)


VALIDATED = "ROBOTLESS_SUCCESSIVE_CHUNKS_VALIDATED"
FAILED = "ROBOTLESS_SUCCESSIVE_CHUNKS_VALIDATION_FAILED"
NOT_VALIDATED = "ROBOTLESS_SUCCESSIVE_CHUNKS_RUNTIME_NOT_VALIDATED"
SCREENSHOTS = ("evidence/old_fresh_world_trajectories.png",)
CAPTURE_FLAGS = (
    "scene_loaded", "no_robot_model", "logical_agent_poses_set", "rgb_captured",
    "no_simulation_dynamics", "camera_unchanged", "scripted_displacement_matches_config",
)
VISUAL_FLAGS = (
    "isaac_viewport_rendered", "no_robot_model", "old_fresh_simultaneously_displayed",
    "matches_observation_pose_transforms", "scripted_displacement_displayed",
)
VISUAL_REVIEW_FLAGS = (
    "no_robot", "R0_observation_pose", "R1_observation_pose", "scripted_displacement",
    "OLD_world_trajectory", "FRESH_world_trajectory", "OLD_waypoint_headings",
    "FRESH_waypoint_headings",
)
REQUIRED_ARTIFACTS = (
    "config_snapshot.yaml", "metadata.json", "capture_validation.json",
    "inference_metadata.json", "visualization_validation.json", "server_process.json",
    "visual_review.json", "raw/lightnav_protocol.jsonl", *SCREENSHOTS,
    *(f"{folder}/{name}_{seq:03d}.{extension}" for seq in range(2)
      for folder, name, extension in (
          ("raw", "observation", "jpg"), ("raw", "request", "json"),
          ("raw", "response", "json"), ("raw", "chunk", "npy"))),
    "derived/chunk_000_world.npy", "derived/chunk_001_world.npy",
)


def check_protocol(run: Path, metadata: dict, inference: dict, locals_: list) -> None:
    """Require exactly login/reset/next0/next1 on one recorded connection."""

    session = inference["session"]
    for name, count in (("connection_count", 1), ("login_count", 1),
                        ("reset_count", 1), ("next_count", 2)):
        require(type(session.get(name)) is int and session[name] == count, f"session {name} mismatch")
    require(session["request_sequence"] == [0, 1], "session sequences must be [0, 1]")
    connection = session.get("connection_id")
    require(isinstance(connection, str) and bool(connection), "missing connection identity")
    records = [json.loads(line) for line in (run / "raw/lightnav_protocol.jsonl").read_text().splitlines()]
    expected = [(direction, action) for action in ("login", "reset", "next", "next")
                for direction in ("request", "response")]
    require([(r["direction"], r["action"]) for r in records] == expected,
            "protocol must be exactly login/reset/next0/next1 request-response pairs")
    previous_ns = metadata["observations"][1]["observation_time"]["monotonic_ns"]
    for index, (record, (direction, action)) in enumerate(zip(records, expected, strict=True)):
        require(record.get("connection_id") == connection, "protocol reconnect or missing connection identity")
        wire = protocol_wire(record)
        envelope = json.loads(wire)
        require(envelope["action"] == action, "protocol action differs from stored wire")
        event = record["time"]
        validate_observation_time({**event, "simulation_time_s": 0.0})
        require(event["monotonic_ns"] >= previous_ns, "protocol event precedes both captures or reverses time")
        previous_ns = event["monotonic_ns"]
        prefix = action if index < 4 else f"next_{(index - 4) // 2:03d}"
        require(event == inference["events"][f"{prefix}_{direction}_time"], "protocol/inference event mismatch")
        if direction == "response":
            require(type(envelope["data"]["rc"]) is int and envelope["data"]["rc"] == 0,
                    f"{action} response failed")
        if action == "next":
            seq = (index - 4) // 2
            require(type(envelope["data"]["seq"]) is int and envelope["data"]["seq"] == seq,
                    "next sequence differs from successive order")
            preserved = run / f"raw/{direction}_{seq:03d}.json"
            require(wire == preserved.read_bytes(), f"seq{seq} {direction} differs from preserved wire")
            observation = metadata["observations"][seq]
            chunk = inference["chunks"][seq]
            if direction == "request":
                require(envelope["data"]["instruction"] == metadata["instruction"], "instruction changed within session")
                require(base64.b64decode(envelope["data"]["image"], validate=True)
                        == (run / observation["rgb"]["path"]).read_bytes(),
                        "request JPEG differs from captured observation bytes")
                require(event == chunk["request_send_time"], "chunk request timestamp mismatch")
            else:
                data = envelope["data"]
                require(type(data["stop"]) is bool and data["stop"] == chunk["stop"], "stop flag mismatch")
                require(np.array_equal(validate_waypoints(data["actions"]["actions"]), locals_[seq]),
                        "raw NPY differs from preserved response values")
                require(event == chunk["response_receive_time"], "chunk response timestamp mismatch")
                if data.get("pointing") is not None:
                    require(data["pointing"]["frame_size"] == observation["rgb"]["resolution_width_height"],
                            "response image dimensions mismatch")


def check_visual_review(run: Path, visual: dict) -> None:
    records = {entry["path"]: entry for entry in visual["screenshots"]}
    require(set(SCREENSHOTS).issubset(records), "missing simultaneous viewport screenshot")
    review = read_json(run / "visual_review.json")
    true_flags(review, ("reviewed",), "visual_review")
    true_flags(review["checks"], VISUAL_REVIEW_FLAGS, "visual_review.checks")
    for path, entry in records.items():
        check_hash(run, entry)
        require(review["screenshots"][path] == entry["sha256"], "visual review is not bound to screenshot bytes")
        with Image.open(artifact(run, path)) as image:
            require(image.format == "PNG" and min(image.size) > 1, "invalid viewport PNG")
            image.verify()
        with Image.open(artifact(run, path)) as image:
            require(float(np.asarray(image).std()) > 1.0, "blank viewport PNG")


def validate_complete(run: Path) -> dict[str, Any]:
    config = load_config(run / "config_snapshot.yaml")
    metadata = read_json(run / "metadata.json")
    capture = read_json(run / "capture_validation.json")
    inference = read_json(run / "inference_metadata.json")
    visual = read_json(run / "visualization_validation.json")
    server = read_json(run / "server_process.json")
    validate_successive_observation_metadata(metadata)
    require(metadata["instruction"] == config["instruction"] == inference["instruction"], "instruction provenance mismatch")
    require(inference["execution_time"] is None, "no trajectory execution is permitted")
    require(metadata["motion_during_capture"] is False and metadata["inference_concurrent_with_capture"] is False
            and inference["agent_motion_during_inference"] is False, "captures and inference must be separate static phases")
    require(inference["transform"]["raw_arrays_unchanged"] is True
            and inference["transform"]["correction_applied"] is False
            and inference["transform"]["waypoint_time_base"] is None, "raw chunks must remain untimed and uncorrected")
    r0, r1 = successive_observation_poses(config["agent"]["pose_world"], config["agent"]["local_displacement"])
    for key, expected in (("R0", r0), ("R1", r1), ("configured_local_displacement", config["agent"]["local_displacement"])):
        same_array(metadata[key], expected, key, 1e-7)
    check_hash(run, metadata["config"], "config_snapshot.yaml")
    check_hash(run, inference["config"], "config_snapshot.yaml")
    check_hash(run, inference["raw_inputs"]["capture_metadata"], "metadata.json")
    check_hash(run, inference["raw_inputs"]["protocol"], "raw/lightnav_protocol.jsonl")
    check_hash(run, inference["server_process"], "server_process.json")
    require(capture["source_metadata_sha256"] == sha256_file(run / "metadata.json"), "capture metadata hash mismatch")
    require(visual["config_sha256"] == sha256_file(run / "config_snapshot.yaml"), "visualization configuration hash mismatch")
    require(visual["inference_metadata_sha256"] == sha256_file(run / "inference_metadata.json"), "visualization inference hash mismatch")
    for path in ("metadata.json", "raw/chunk_000.npy", "raw/chunk_001.npy"):
        check_hash(run, {"path": path, "sha256": visual["raw_inputs"][path]})
    derived_records = {entry["path"]: entry for entry in visual["derived_inputs"]}
    for seq in range(2):
        path = f"derived/chunk_{seq:03d}_world.npy"
        check_hash(run, derived_records[path], path)
    true_flags(capture, CAPTURE_FLAGS, "capture")
    true_flags(visual, VISUAL_FLAGS, "visualization")
    require(visual["logical_agent_static"] is True and visual["timeline_time_s"] == 0, "visualization must remain static")
    for key in ("agent_pose_before_visualization", "agent_pose_after_visualization"):
        same_array(visual[key], metadata["R1"], key, 1e-7)
    validate_observation_time(visual["created_time"])
    require(visual["created_time"]["simulation_time_s"] == 0, "visualization timeline advanced")
    true_flags(inference, ("interface_inference_valid",), "inference")
    inventory = metadata["scene"]["runtime_inventory"]
    require(inventory["no_robot_model"] is True and inventory["articulation_roots"] == []
            and inventory["robot_named_paths"] == [] and inventory["dynamics_advanced"] is False,
            "runtime inventory contains robot/articulation or advancing dynamics")
    require(inventory["authored_asset_references"] == [config["scene"]["asset_relative_path"]], "scene asset mismatch")
    require(metadata["scene"]["stage_units_in_meters"] == 1.0 and metadata["scene"]["up_axis"] == "Z",
            "world must explicitly use meters and Z up")
    displacement = metadata["scripted_displacement"]
    same_array(displacement["actual_local_se2"], relative_pose(metadata["R0"], metadata["R1"]), "actual local displacement", 1e-7)
    same_array(displacement["actual_world_delta_xy_m"], r1[:2] - r0[:2], "actual world displacement", 1e-7)
    time_order = [metadata["observations"][0]["capture_started_time"], metadata["observations"][0]["observation_time"],
                  displacement["started_time"], displacement["completed_time"],
                  metadata["observations"][1]["capture_started_time"], metadata["observations"][1]["observation_time"]]
    previous_ns = -1
    for stamp in time_order:
        validate_observation_time(stamp)
        require(stamp["simulation_time_s"] == 0, "simulation timeline advanced")
        require(stamp["monotonic_ns"] >= previous_ns, "capture/displacement order mismatch")
        previous_ns = stamp["monotonic_ns"]
    locals_, examples = [], []
    require(len(inference["chunks"]) == len(capture["observations"]) == len(visual["observations"]) == 2,
            "exactly two capture/inference/visualization records required")
    for seq, observation in enumerate(metadata["observations"]):
        chunk = inference["chunks"][seq]
        captured, viewed = capture["observations"][seq], visual["observations"][seq]
        pose = observation["agent_pose_world"]
        require(type(chunk["seq"]) is int and chunk["seq"] == seq
                and chunk["label"] == observation["label"] and chunk["observation_id"] == observation["observation_id"],
                "chunk observation identity mismatch")
        require(chunk["observation_time"] == observation["observation_time"], "chunk observation timestamp mismatch")
        for key in ("agent_pose_before_capture", "agent_pose_after_capture"):
            same_array(observation[key], pose, key, 1e-7)
        same_array(chunk["agent_pose_world_at_observation"], pose, "chunk observation anchor", 0)
        same_array(viewed["agent_pose_world_used"], pose, "visualization observation anchor", 1e-7)
        check_camera({**metadata, **observation}, config, captured, viewed)
        camera_pose = np.eye(4)
        c, s = np.cos(pose[2]), np.sin(pose[2])
        camera_pose[:2, :2] = [[c, -s], [s, c]]
        camera_pose[:3, 3] = [pose[0], pose[1], config["agent"]["z_m"]]
        same_array(observation["camera"]["T_world_agent"], camera_pose, "camera observation world anchor", 1e-7)
        check_hash(run, observation["rgb"], f"raw/observation_{seq:03d}.jpg")
        for field, path in (("observation", f"raw/observation_{seq:03d}.jpg"),
                            ("request", f"raw/request_{seq:03d}.json"),
                            ("response", f"raw/response_{seq:03d}.json"),
                            ("waypoints", f"raw/chunk_{seq:03d}.npy")):
            check_hash(run, chunk["raw_inputs"][field], path)
        check_hash(run, chunk["world_waypoints"], f"derived/chunk_{seq:03d}_world.npy")
        resolution = observation["rgb"]["resolution_width_height"]
        with Image.open(run / observation["rgb"]["path"]) as rgb:
            require(rgb.format == "JPEG" and rgb.mode == "RGB" and list(rgb.size) == resolution, "RGB format/resolution mismatch")
            require(float(np.asarray(rgb).std()) > 1.0, "captured RGB is blank")
            rgb.verify()
        require(captured["rgb_shape_hwc"] == [resolution[1], resolution[0], 3], "capture shape mismatch")
        local = validate_waypoints(np.load(run / f"raw/chunk_{seq:03d}.npy", allow_pickle=False))
        world = validate_waypoints(np.load(run / f"derived/chunk_{seq:03d}_world.npy", allow_pickle=False))
        same_array(world, observation_to_world(pose, local), f"{observation['label']} observation-only transform")
        require(chunk["raw_shape"] == list(local.shape) == viewed["world_shape"], "recorded waypoint shape mismatch")
        require(viewed["waypoint_heading_count"] == len(local), "missing waypoint headings")
        latency = round_trip_latency_ms(chunk["request_send_time"], chunk["response_receive_time"])
        require(type(chunk["round_trip_latency_ms"]) in (float, int)
                and np.isclose(chunk["round_trip_latency_ms"], latency, atol=1e-9, rtol=0), "host round-trip milliseconds mismatch")
        locals_.append(local)
        examples.append({"label": observation["label"], "raw_shape": list(local.shape),
                         "example_raw_local_waypoint": local[0].tolist(), "example_world_waypoint": world[0].tolist()})
    check_protocol(run, metadata, inference, locals_)
    check_external_provenance(metadata, config, inference, server)
    check_visual_review(run, visual)
    return {"R0": metadata["R0"], "R1": metadata["R1"], "configured_local_displacement": metadata["configured_local_displacement"],
            "actual_local_displacement": displacement["actual_local_se2"], "request_sequence": [0, 1], "chunks": examples,
            "checks": {"immutable_raw_hashes": True, "two_actual_rgb_captures": True, "same_camera": True,
                       "local_scripted_displacement": True, "one_connection_one_reset_successive_protocol": True,
                       "respective_observation_pose_transforms": True, "robotless_no_execution": True,
                       "external_source_checkpoint_provenance": True, "simultaneous_isaac_viewport_and_explicit_review": True}}


def validate_run(run_directory: str | Path) -> dict[str, Any]:
    """Missing evidence yields NOT_VALIDATED; explicit failed evidence yields FAILED."""

    run = Path(run_directory).resolve()
    missing = [name for name in REQUIRED_ARTIFACTS if not (run / name).is_file()]
    failures = []
    for name, flags in (("capture_validation.json", CAPTURE_FLAGS), ("visualization_validation.json", VISUAL_FLAGS),
                        ("inference_metadata.json", ("interface_inference_valid",)),
                        ("server_process.json", ("ready_confirmed", "source_status_clean")),
                        ("visual_review.json", ("reviewed",))):
        if (run / name).is_file():
            try:
                partial = read_json(run / name)
                failures.extend(f"{name}: {key} is false" for key in flags if partial.get(key) is False)
                for group in ("camera_basis_checks", "checks"):
                    if isinstance(partial.get(group), dict):
                        failures.extend(f"{name}: {group}.{key} is false" for key, value in partial[group].items() if value is False)
            except (ValueError, OSError, TypeError, AttributeError) as error:
                failures.append(f"{name}: {error}")
    details = {}
    if not missing and not failures:
        try:
            details = validate_complete(run)
        except FileNotFoundError as error:
            missing.append(str(error.filename))
        except (ValueError, KeyError, TypeError, IndexError, OSError, AttributeError, OverflowError,
                RuntimeError, SyntaxError, EOFError, yaml.YAMLError) as error:
            failures.append(f"{type(error).__name__}: {error}")
    return {"schema_version": 1, "run_directory": str(run), "status": FAILED if failures else NOT_VALIDATED if missing else VALIDATED,
            "missing_artifacts": missing, "failures": failures,
            "interpretation": "static successive interface only; no latency/discontinuity, correspondence, reconciliation or navigation claim",
            **details}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--write", action="store_true", help="Create validation.json exclusively.")
    args = parser.parse_args()
    result = validate_run(args.run_directory)
    if args.write:
        save_json_exclusive(args.run_directory / "validation.json", result)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == VALIDATED else 1


if __name__ == "__main__":
    raise SystemExit(main())
