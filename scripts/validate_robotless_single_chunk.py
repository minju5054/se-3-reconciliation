#!/usr/bin/env python3
"""Audit one robotless runtime handoff; never infer runtime success from fixtures."""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.robotless_single_chunk import (  # noqa: E402
    FRAME_CONVENTIONS, frame_sanity_fixtures, load_config, observation_to_world,
    save_json_exclusive, sha256_file, validate_observation_metadata,
    validate_observation_time, validate_waypoints,
)


VALIDATED = "ROBOTLESS_SINGLE_CHUNK_VALIDATED"
FAILED = "ROBOTLESS_SINGLE_CHUNK_VALIDATION_FAILED"
NOT_VALIDATED = "ROBOTLESS_SINGLE_CHUNK_RUNTIME_NOT_VALIDATED"
SCREENSHOTS = (
    "evidence/synthetic_frame_yaw_000.png", "evidence/synthetic_frame_yaw_090.png",
    "evidence/lightnav_world_trajectory.png",
)
REQUIRED_ARTIFACTS = (
    "config_snapshot.yaml", "metadata.json", "capture_validation.json",
    "inference_metadata.json", "visualization_validation.json", "server_process.json",
    "visual_review.json", "raw/observation.jpg", "raw/lightnav_request.json",
    "raw/lightnav_response.json", "raw/lightnav_protocol.jsonl",
    "raw/lightnav_waypoints.npy", "derived/world_waypoints.npy", *SCREENSHOTS,
)
CAPTURE_FLAGS = (
    "scene_loaded", "no_robot_model", "logical_agent_pose_set", "logical_agent_static",
    "rgb_captured", "no_simulation_dynamics",
)
VISUAL_FLAGS = (
    "isaac_viewport_rendered", "no_robot_model", "logical_agent_static",
    "matches_observation_pose_transform",
)
VISUAL_REVIEW_FLAGS = (
    "no_robot", "logical_agent_origin_forward", "world_trajectory", "waypoint_headings",
    "yaw0_forward_left", "yaw90_forward_left",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"{path.name} must contain an object")
    return value


def artifact(run: Path, relative: str) -> Path:
    path = run / relative
    require(not Path(relative).is_absolute() and path.resolve().is_relative_to(run.resolve()),
            f"artifact path escapes run directory: {relative}")
    return path


def check_hash(run: Path, record: dict, expected_path: str | None = None) -> None:
    if expected_path is not None:
        require(record["path"] == expected_path, f"unexpected artifact path: {record['path']}")
    path = artifact(run, record["path"])
    require(sha256_file(path) == record["sha256"], f"SHA-256 mismatch: {record['path']}")


def same_array(actual: Any, expected: Any, name: str, tolerance: float = 1e-12) -> None:
    a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    require(a.shape == b.shape and np.all(np.isfinite(a)) and np.all(np.isfinite(b))
            and np.allclose(a, b, rtol=0, atol=tolerance), f"{name} mismatch")


def true_flags(mapping: dict, fields: tuple[str, ...], name: str) -> None:
    for field in fields:
        require(mapping.get(field) is True, f"{name}.{field} must be true")


def check_camera(metadata: dict, config: dict, capture: dict, visual: dict) -> None:
    camera, configured = metadata["camera"], config["camera"]
    require(camera["configured"] == configured, "camera configured metadata mismatch")
    require(camera["camera_prim_path"] == configured["prim_path"]
            and camera["parent_prim_path"] == config["agent"]["prim_path"],
            "camera must be the configured logical agent child")
    transform = np.asarray(camera["T_agent_camera"], dtype=float)
    require(transform.shape == (4, 4), "camera extrinsic must be 4x4")
    same_array(transform[3], [0, 0, 0, 1], "camera homogeneous row")
    rotation = transform[:3, :3]
    same_array(rotation, configured["rotation_agent_from_camera"], "configured camera rotation")
    same_array(rotation.T @ rotation, np.eye(3), "orthonormal camera rotation")
    require(np.isclose(np.linalg.det(rotation), 1, atol=1e-12), "camera rotation handedness")
    for source, target, label in (([0, 0, -1], [1, 0, 0], "optical forward"),
                                  ([1, 0, 0], [0, -1, 0], "image right"),
                                  ([0, 1, 0], [0, 0, 1], "image up")):
        same_array(rotation @ source, target, label, 1e-7)
    same_array(transform[:3, 3], configured["relative_translation_m"], "camera translation", 1e-7)
    same_array(np.asarray(camera["T_world_agent"]) @ transform, camera["T_world_camera"],
               "camera world extrinsic composition", 1e-7)
    basis_fields = (
        "optical_forward_maps_to_agent_positive_x", "image_right_maps_to_agent_negative_y",
        "image_up_maps_to_agent_positive_z", "proper_rotation_determinant_positive_one",
        "translation_matches_configuration",
    )
    for label, checks in (("camera", camera["basis_checks"]),
                          ("capture", capture["camera_basis_checks"]),
                          ("visualization", visual["camera_basis_checks"])):
        true_flags(checks, basis_fields, label)
    intrinsics = camera["actual_intrinsics"]
    resolution = [configured["resolution_width"], configured["resolution_height"]]
    require(intrinsics["resolution_width_height"] == resolution, "actual camera resolution mismatch")
    horizontal = float(intrinsics["usd_horizontal_aperture"])
    vertical = float(intrinsics["usd_vertical_aperture"])
    focal = float(intrinsics["usd_focal_length"])
    require(all(np.isfinite(x) and x > 0 for x in (horizontal, vertical, focal)), "invalid camera pinhole values")
    fov = np.degrees(2 * np.arctan(horizontal / (2 * focal)))
    require(np.isclose(fov, configured["horizontal_fov_deg"], atol=1e-5), "actual camera FOV mismatch")
    require(np.isclose(fov, intrinsics["horizontal_fov_deg"], atol=1e-12), "recorded horizontal FOV mismatch")
    width, height = resolution
    same_array(intrinsics["K"], [[width*focal/horizontal, 0, width/2],
                                [0, height*focal/vertical, height/2], [0, 0, 1]], "intrinsic K")


def protocol_wire(record: dict) -> bytes:
    text, binary = record.get("wire_text"), record.get("wire_bytes_base64")
    require((text is None) != (binary is None), "protocol record must preserve exactly one wire form")
    return text.encode("utf-8") if text is not None else base64.b64decode(binary, validate=True)


def check_protocol(run: Path, metadata: dict, inference: dict, local: np.ndarray) -> None:
    request_bytes = (run / "raw/lightnav_request.json").read_bytes()
    response_bytes = (run / "raw/lightnav_response.json").read_bytes()
    request, response = json.loads(request_bytes), json.loads(response_bytes)
    require(request.get("action") == "next", "single request action must be next")
    data = request["data"]
    require(type(data["seq"]) is int and data["seq"] == 0, "single request must use seq=0")
    require(data["instruction"] == metadata["instruction"], "request instruction changed")
    require(base64.b64decode(data["image"], validate=True) == (run / "raw/observation.jpg").read_bytes(),
            "request JPEG differs from captured raw observation")
    require(response.get("action") == "next", "response action must be next")
    data = response["data"]
    require(type(data["rc"]) is int and data["rc"] == 0, "LightNav response failed")
    require(type(data["seq"]) is int and data["seq"] == 0, "response must match seq=0")
    require(type(data["stop"]) is bool and data["stop"] == inference["stop"], "stop flag mismatch")
    response_local = validate_waypoints(data["actions"]["actions"])
    require(np.array_equal(local, response_local), "raw NPY differs from untouched nested response actions")
    if data.get("pointing") is not None:
        require(data["pointing"]["frame_size"] == metadata["rgb"]["resolution_width_height"],
                "response pointing dimensions differ from observation")
    records = [json.loads(line) for line in (run / "raw/lightnav_protocol.jsonl").read_text().splitlines()]
    expected = [(direction, action) for action in ("login", "reset", "next")
                for direction in ("request", "response")]
    require([(r["direction"], r["action"]) for r in records] == expected,
            "protocol must contain exactly one login/reset/next request and response")
    previous_ns = metadata["observation_time"]["monotonic_ns"]
    for record, (direction, action) in zip(records, expected, strict=True):
        wire = protocol_wire(record)
        envelope = json.loads(wire)
        require(envelope["action"] == action, "protocol action differs from stored wire")
        event = record["time"]
        validate_observation_time({**event, "simulation_time_s": 0.0})
        require(event["monotonic_ns"] >= previous_ns, "protocol precedes observation or reverses host event order")
        previous_ns = event["monotonic_ns"]
        require(event == inference["events"][f"{action}_{direction}_time"], "event differs from protocol record")
        if direction == "response":
            require(type(envelope["data"]["rc"]) is int and envelope["data"]["rc"] == 0,
                    f"{action} protocol response failed")
        if action == "next":
            require(wire == (request_bytes if direction == "request" else response_bytes),
                    f"next {direction} differs from preserved raw wire")


def check_fixtures(visual: dict, observation_pose: list) -> None:
    records = visual["synthetic_fixtures"]
    expected = frame_sanity_fixtures()
    require(len(records) == len(expected), "both yaw 0 and yaw 90 fixtures are required")
    require(visual["fixtures_are_research_evidence"] is False, "synthetic fixtures mislabeled as evidence")
    for actual, reference in zip(records, expected, strict=True):
        require(actual["name"] == reference["name"], "fixture identity mismatch")
        require(actual["passed"] is True and actual["synthetic_only"] is True,
                "fixture failed or not marked synthetic")
        for key in ("agent_pose_world", "local_waypoints", "expected_world_waypoints", "actual_world_waypoints"):
            same_array(actual[key], reference[key], f"fixture {key}")
        same_array(actual["display_translation_world_m"], observation_pose[:2], "fixture display offset")
        display_pose = np.array(reference["agent_pose_world"], dtype=float)
        display_pose[:2] += observation_pose[:2]
        same_array(actual["display_agent_pose_world"], display_pose, "fixture display pose")
        derived = observation_to_world(display_pose, reference["local_waypoints"])
        same_array(actual["display_world_waypoints"], derived, "display fixture transform")
        same_array(actual["display_expected_world_waypoints"], derived, "display fixture expectation")


def check_external_provenance(metadata: dict, config: dict, inference: dict, server: dict) -> None:
    expected_sha = config["lightnav"]["expected_git_sha"]
    before, after = inference["lightnav_source_before"], inference["lightnav_source_after"]
    require(before == after and before["clean"] is True and before["status_porcelain"] == "",
            "LightNav source was not clean and unchanged")
    require(before["git_sha"] == expected_sha == metadata["lightnav_git_sha"] == server["lightnav_git_sha"],
            "LightNav source SHA differs from configured provenance")
    checkout = str((ROOT / config["paths"]["lightnav_checkout"]).resolve())
    checkpoint = str((ROOT / config["paths"]["checkpoint_path"]).resolve())
    require(before["path"] == server["lightnav_checkout"] == metadata["lightnav_checkout"] == checkout,
            "LightNav source paths differ")
    require(inference["checkpoint"]["path"] == server["checkpoint_path"] == metadata["model_checkpoint_path"] == checkpoint,
            "checkpoint paths differ")
    identifier = config["lightnav"]["checkpoint_identifier"]
    require(identifier == metadata["model_checkpoint_identifier"] == inference["checkpoint_identifier"]
            == server["checkpoint_identifier"], "checkpoint identifiers differ")
    require(inference["checkpoint_revision"] == metadata["model_checkpoint_revision"]
            == config["lightnav"]["checkpoint_revision"], "checkpoint revision mismatch")
    true_flags(server, ("ready_confirmed", "source_status_clean"), "server")
    require(isinstance(server["process_id"], int) and server["process_id"] > 0, "missing server process ID")
    require(isinstance(server["argv"], list) and server["argv"], "missing concrete server argv")
    checkpoint_files = {entry["path"]: entry for entry in inference["checkpoint"]["files"]}
    server_files = {entry["path"]: entry for entry in server["checkpoint_files"]}
    expected_hashes = config["lightnav"]["checkpoint_sha256"]
    require(bool(expected_hashes), "configuration must pin checkpoint artifact hashes")
    for relative, digest in expected_hashes.items():
        require(checkpoint_files[relative]["sha256"] == digest == server_files[relative]["sha256"],
                f"checkpoint SHA-256 mismatch: {relative}")
    for relative, entry in checkpoint_files.items():
        server_entry = server_files[relative]
        require(entry["sha256"] == server_entry["sha256"]
                and entry["bytes"] == server_entry["size"] and entry["mtime_ns"] == server_entry["mtime_ns"],
                f"server/client checkpoint artifact mismatch: {relative}")


def validate_complete(run: Path) -> dict[str, Any]:
    config = load_config(run / "config_snapshot.yaml")
    metadata = read_json(run / "metadata.json")
    capture = read_json(run / "capture_validation.json")
    inference = read_json(run / "inference_metadata.json")
    visual = read_json(run / "visualization_validation.json")
    server = read_json(run / "server_process.json")
    validate_observation_metadata(metadata)
    require(metadata["coordinate_conventions"] == FRAME_CONVENTIONS, "coordinate convention differs from implemented transform")
    require(metadata["instruction"] == config["instruction"] == inference["instruction"], "instruction provenance mismatch")
    require(metadata["observation_time"] == inference["observation_time"], "inference observation event mismatch")
    require(metadata["agent_motion"] is False and metadata["execution_time"] is None
            and inference["execution_time"] is None, "static task must not contain motion or execution")
    validate_observation_time(metadata["capture_started_time"])
    require(metadata["capture_started_time"]["monotonic_ns"] <= metadata["observation_time"]["monotonic_ns"],
            "observation timestamp must follow static render start")
    for stamp in (metadata["capture_started_time"], metadata["observation_time"], visual["created_time"]):
        validate_observation_time(stamp)
        require(stamp["simulation_time_s"] == 0, "simulation timeline advanced in static validation")
    check_hash(run, metadata["config"], "config_snapshot.yaml")
    check_hash(run, metadata["rgb"], "raw/observation.jpg")
    check_hash(run, inference["config"], "config_snapshot.yaml")
    expected_inputs = {
        "observation": "raw/observation.jpg", "capture_metadata": "metadata.json",
        "request": "raw/lightnav_request.json", "response": "raw/lightnav_response.json",
        "protocol": "raw/lightnav_protocol.jsonl", "waypoints": "raw/lightnav_waypoints.npy",
    }
    for field, path in expected_inputs.items():
        check_hash(run, inference["raw_inputs"][field], path)
    check_hash(run, inference["server_process"], "server_process.json")
    check_hash(run, inference["world_waypoints"], "derived/world_waypoints.npy")
    require(capture["source_metadata_sha256"] == sha256_file(run / "metadata.json"), "capture metadata hash mismatch")
    require(visual["config_sha256"] == sha256_file(run / "config_snapshot.yaml"), "visualization configuration hash mismatch")
    for path in ("metadata.json", "raw/lightnav_waypoints.npy"):
        check_hash(run, {"path": path, "sha256": visual["raw_inputs"][path]})
    check_hash(run, visual["derived_input"], "derived/world_waypoints.npy")
    true_flags(capture, CAPTURE_FLAGS, "capture")
    true_flags(visual, VISUAL_FLAGS, "visualization")
    inventory = metadata["scene"]["runtime_inventory"]
    require(inventory["no_robot_model"] is True and inventory["articulation_roots"] == []
            and inventory["robot_named_paths"] == [] and inventory["dynamics_advanced"] is False,
            "runtime inventory contains robot/articulation or advancing dynamics")
    require(inventory["authored_asset_references"] == [config["scene"]["asset_relative_path"]],
            "runtime referenced a scene other than the configured asset")
    require(metadata["scene"]["stage_units_in_meters"] == 1.0
            and metadata["scene"]["up_axis"] == "Z", "world must explicitly use meters and Z up")
    pose = metadata["agent_pose_world"]
    for key in ("agent_pose_before_capture", "agent_pose_after_capture"):
        same_array(metadata[key], pose, key, 0)
    same_array(pose, config["agent"]["pose_world"], "configured observation pose", 1e-7)
    same_array(inference["agent_pose_world_at_observation"], pose, "inference pose", 0)
    for key in ("agent_pose_world_used", "agent_pose_before_visualization", "agent_pose_after_visualization"):
        same_array(visual[key], pose, key, 1e-7)
    check_camera(metadata, config, capture, visual)
    resolution = metadata["rgb"]["resolution_width_height"]
    with Image.open(run / "raw/observation.jpg") as rgb:
        require(rgb.format == "JPEG" and rgb.mode == "RGB" and list(rgb.size) == resolution,
                "saved RGB format/resolution mismatch")
        require(float(np.asarray(rgb).std()) > 1.0, "captured RGB is blank")
        rgb.verify()
    require(capture["rgb_shape_hwc"] == [resolution[1], resolution[0], 3], "capture shape mismatch")
    local = validate_waypoints(np.load(run / "raw/lightnav_waypoints.npy", allow_pickle=False))
    world = validate_waypoints(np.load(run / "derived/world_waypoints.npy", allow_pickle=False))
    same_array(world, observation_to_world(pose, local), "observation-only world transform")
    require(inference["raw_shape"] == list(local.shape) == visual["world_shape"], "recorded waypoint shape mismatch")
    require(visual["waypoint_heading_count"] == len(local), "missing waypoint heading markers")
    true_flags(inference, ("single_frame_single_request", "interface_inference_valid"), "inference")
    check_protocol(run, metadata, inference, local)
    check_external_provenance(metadata, config, inference, server)
    check_fixtures(visual, pose)
    screenshot_records = {entry["path"]: entry for entry in visual["screenshots"]}
    require(set(SCREENSHOTS).issubset(screenshot_records), "missing required viewport screenshot records")
    review = read_json(run / "visual_review.json")
    true_flags(review, ("reviewed",), "visual_review")
    true_flags(review["checks"], VISUAL_REVIEW_FLAGS, "visual_review.checks")
    for path, entry in screenshot_records.items():
        check_hash(run, entry)
        require(review["screenshots"][path] == entry["sha256"], "visual review is not bound to screenshot bytes")
        with Image.open(artifact(run, path)) as image:
            require(image.format == "PNG" and min(image.size) > 1, "invalid viewport PNG")
            image.verify()
    return {"raw_shape": list(local.shape), "example_raw_local_waypoint": local[0].tolist(),
            "example_world_waypoint": world[0].tolist(),
            "checks": {"immutable_input_hashes": True, "single_real_inference_protocol": True,
                       "observation_only_se2_transform": True, "static_robotless_capture": True,
                       "camera_optical_basis": True, "synthetic_frame_fixtures": True,
                       "external_source_checkpoint_provenance": True,
                       "isaac_viewport_screenshots_and_explicit_visual_review": True}}


def validate_run(run_directory: str | Path) -> dict[str, Any]:
    """Return exactly one status; missing runtime artifacts can never yield PASS."""
    run = Path(run_directory).resolve()
    missing = [name for name in REQUIRED_ARTIFACTS if not (run / name).is_file()]
    failures: list[str] = []
    # Even an incomplete run may contain an explicit failed runtime check.
    for name, flags in (("capture_validation.json", CAPTURE_FLAGS),
                        ("visualization_validation.json", VISUAL_FLAGS),
                        ("inference_metadata.json", ("interface_inference_valid",)),
                        ("server_process.json", ("ready_confirmed", "source_status_clean")),
                        ("visual_review.json", ("reviewed",))):
        if (run / name).is_file():
            try:
                partial = read_json(run / name)
                failures.extend(f"{name}: {key} is false" for key in flags if partial.get(key) is False)
                for group in ("camera_basis_checks", "checks"):
                    if isinstance(partial.get(group), dict):
                        failures.extend(f"{name}: {group}.{key} is false"
                                        for key, value in partial[group].items() if value is False)
                for fixture in partial.get("synthetic_fixtures", []):
                    if fixture.get("passed") is False:
                        failures.append(f"{name}: synthetic frame fixture failed")
            except (ValueError, OSError, TypeError, AttributeError) as error:
                failures.append(f"{name}: {error}")
    details = {}
    if not missing and not failures:
        try:
            details = validate_complete(run)
        except FileNotFoundError as error:
            missing.append(str(error.filename))
        except (ValueError, KeyError, TypeError, IndexError, OSError, AttributeError, OverflowError) as error:
            failures.append(f"{type(error).__name__}: {error}")
    status = FAILED if failures else NOT_VALIDATED if missing else VALIDATED
    return {"schema_version": 1, "run_directory": str(run), "status": status,
            "missing_artifacts": missing, "failures": failures,
            "synthetic_fixture_claim": "coordinate tests only; never experimental evidence", **details}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--write", action="store_true", help="Create validation.json exclusively; never overwrite an audit.")
    args = parser.parse_args()
    result = validate_run(args.run_directory)
    if args.write:
        save_json_exclusive(args.run_directory / "validation.json", result)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == VALIDATED else 1


if __name__ == "__main__":
    raise SystemExit(main())
