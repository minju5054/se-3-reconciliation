"""Synthetic offline auditor fixtures; these tests are never runtime evidence."""

import base64
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/validate_robotless_successive_chunks.py"
SPEC = importlib.util.spec_from_file_location("validate_robotless_successive_chunks", SCRIPT)
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def test_missing_runtime_evidence_cannot_pass(tmp_path):
    result = validator.validate_run(tmp_path)
    assert result["status"] == validator.NOT_VALIDATED
    assert "raw/observation_001.jpg" in result["missing_artifacts"]
    assert "visual_review.json" in result["missing_artifacts"]


@pytest.mark.parametrize("filename,key", [
    ("capture_validation.json", "scripted_displacement_matches_config"),
    ("visualization_validation.json", "old_fresh_simultaneously_displayed"),
    ("inference_metadata.json", "interface_inference_valid"),
    ("server_process.json", "source_status_clean"), ("visual_review.json", "reviewed"),
])
def test_failed_evidence_dominates_missing_evidence(tmp_path, filename, key):
    save_json_exclusive(tmp_path / filename, {key: False})
    result = validator.validate_run(tmp_path)
    assert result["status"] == validator.FAILED
    assert any(key in failure for failure in result["failures"])


def test_available_corrupt_runtime_audit_fails_closed(tmp_path):
    (tmp_path / "capture_validation.json").write_text("not JSON")
    assert validator.validate_run(tmp_path)["status"] == validator.FAILED


@pytest.mark.parametrize("configuration", ["{}", "[unterminated", "scene:\n  - mismatched: [\n"])
def test_all_present_placeholder_or_corrupt_config_files_cannot_pass(tmp_path, configuration):
    for relative in validator.REQUIRED_ARTIFACTS:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    (tmp_path / "config_snapshot.yaml").write_text(configuration)
    assert validator.validate_run(tmp_path)["status"] == validator.FAILED


def protocol_fixture(tmp_path):
    raw = tmp_path / "raw"
    raw.mkdir()
    frames = [b"synthetic-image-zero", b"synthetic-image-one"]
    local = [np.array([[1., 0., 0.]]), np.array([[2., 0., .5], [3., .1, .6]])]
    observations = []
    inference = {"events": {}, "chunks": [{"stop": False}, {"stop": True}],
                 "session": {"connection_count": 1, "login_count": 1, "reset_count": 1,
                             "next_count": 2, "request_sequence": [0, 1], "connection_id": "test-one-connection"}}
    for seq in range(2):
        image_path = f"raw/observation_{seq:03d}.jpg"
        (tmp_path / image_path).write_bytes(frames[seq])
        observations.append({"observation_time": {"monotonic_ns": seq + 1},
                             "rgb": {"path": image_path, "resolution_width_height": [480, 270]}})
    metadata = {"instruction": "Fixed instruction", "observations": observations}
    records = []
    for index, action in enumerate(("login", "reset", "next", "next")):
        seq = index - 2 if index >= 2 else None
        request = {"action": action, "data": {} if seq is None else {
            "seq": seq, "instruction": metadata["instruction"], "image": base64.b64encode(frames[seq]).decode()}}
        response = {"action": action, "data": {"rc": 0}}
        if seq is not None:
            response["data"].update(seq=seq, stop=inference["chunks"][seq]["stop"], actions={"actions": local[seq].tolist()})
        prefix = action if seq is None else f"next_{seq:03d}"
        for offset, (direction, envelope) in enumerate((("request", request), ("response", response))):
            wire = json.dumps(envelope)
            event = {"utc": "2026-09-14T12:00:00Z", "monotonic_ns": 10 + index * 2 + offset}
            record = {"connection_id": "test-one-connection", "direction": direction, "action": action,
                      "time": event, "wire_text": wire, "wire_bytes_base64": None}
            records.append(record)
            inference["events"][f"{prefix}_{direction}_time"] = event
            if seq is not None:
                (raw / f"{direction}_{seq:03d}.json").write_bytes(wire.encode())
                inference["chunks"][seq]["request_send_time" if direction == "request" else "response_receive_time"] = event
    (raw / "lightnav_protocol.jsonl").write_text("\n".join(map(json.dumps, records)) + "\n")
    return metadata, inference, local, records


def test_protocol_accepts_one_connection_one_reset_two_different_length_chunks(tmp_path):
    metadata, inference, local, _ = protocol_fixture(tmp_path)
    validator.check_protocol(tmp_path, metadata, inference, local)


@pytest.mark.parametrize("mutation", [
    lambda records: records.insert(6, deepcopy(records[2])),
    lambda records: records.pop(),
    lambda records: records[6].update(connection_id="reconnected"),
    lambda records: records[6].update(action="reset"),
    lambda records: records[6]["time"].update(monotonic_ns=0),
    lambda records: records[6].update(wire_text=records[4]["wire_text"]),
])
def test_protocol_rejects_extra_reset_reconnect_wrong_order_and_wrong_seq(tmp_path, mutation):
    metadata, inference, local, records = protocol_fixture(tmp_path)
    mutation(records)
    (tmp_path / "raw/lightnav_protocol.jsonl").write_text("\n".join(map(json.dumps, records)))
    with pytest.raises(ValueError):
        validator.check_protocol(tmp_path, metadata, inference, local)


@pytest.mark.parametrize("part", ["image", "instruction", "raw_array", "response", "connection_count", "reset_count"])
def test_protocol_rejects_mismatched_capture_wire_raw_output_and_counts(tmp_path, part):
    metadata, inference, local, _ = protocol_fixture(tmp_path)
    if part == "image":
        (tmp_path / "raw/observation_001.jpg").write_bytes(b"changed")
    elif part == "instruction":
        metadata["instruction"] = "Changed instruction"
    elif part == "raw_array":
        local[1][0, 0] = 200
    elif part == "response":
        (tmp_path / "raw/response_001.json").write_bytes(b"changed")
    else:
        inference["session"][part] = 2
    with pytest.raises(ValueError):
        validator.check_protocol(tmp_path, metadata, inference, local)


def test_visual_review_binds_every_required_check_and_image_bytes(tmp_path):
    path = tmp_path / validator.SCREENSHOTS[0]
    path.parent.mkdir()
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    image[:4] = 255
    Image.fromarray(image).save(path)
    record = {"path": validator.SCREENSHOTS[0], "sha256": sha256_file(path)}
    visual = {"screenshots": [record]}
    review = {"reviewed": True, "checks": dict.fromkeys(validator.VISUAL_REVIEW_FLAGS, True),
              "screenshots": {record["path"]: record["sha256"]}}
    save_json_exclusive(tmp_path / "visual_review.json", review)
    validator.check_visual_review(tmp_path, visual)
    review["checks"]["FRESH_waypoint_headings"] = False
    (tmp_path / "visual_review.json").write_text(json.dumps(review))
    with pytest.raises(ValueError, match="FRESH_waypoint_headings"):
        validator.check_visual_review(tmp_path, visual)


def test_blank_viewport_screenshot_cannot_establish_visual_validation(tmp_path):
    path = tmp_path / validator.SCREENSHOTS[0]
    path.parent.mkdir()
    Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(path)
    record = {"path": validator.SCREENSHOTS[0], "sha256": sha256_file(path)}
    save_json_exclusive(tmp_path / "visual_review.json", {
        "reviewed": True, "checks": dict.fromkeys(validator.VISUAL_REVIEW_FLAGS, True),
        "screenshots": {record["path"]: record["sha256"]},
    })
    with pytest.raises(ValueError, match="blank viewport"):
        validator.check_visual_review(tmp_path, {"screenshots": [record]})
