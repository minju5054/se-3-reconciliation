"""Synthetic protocol tests: no simulator, model inference, or network access."""

from __future__ import annotations

import base64
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/lightnav/robotless_single_frame_inference.py"
SPEC = importlib.util.spec_from_file_location("robotless_client", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


def prediction(waypoints=None, **overrides):
    data = {"rc": 0, "seq": 0, "stop": False,
            "actions": {"step": 1, "actions": [[1.0, 0.0, 0.0]] if waypoints is None else waypoints}}
    data.update(overrides)
    return json.dumps({"action": "next", "data": data})


class FakeSocket:
    def __init__(self, final):
        self.sent = []
        self.responses = iter([
            '{"action":"login","data":{"rc":0,"msg":"ok"}}',
            '{"action":"reset","data":{"rc":0,"msg":"ok"}}',
            final,
        ])

    def send(self, payload):
        self.sent.append(payload)

    def recv(self, *, timeout):
        assert timeout == 30.0
        return next(self.responses)


def test_exact_original_jpeg_single_request_and_response_are_preserved(tmp_path):
    jpeg = b"synthetic JPEG bytes deliberately not decoded by transport\x00\xff"
    wire = '{ "action": "next", "data": {"rc": 0, "seq": 0, "stop": false, "actions": {"actions": [[1,0,0]]}} }\n'
    socket = FakeSocket(wire)
    raw, events = client.exchange_one_frame(socket, tmp_path, jpeg, "go left", 30.0)
    requests = [json.loads(item) for item in socket.sent]
    assert [item["action"] for item in requests] == ["login", "reset", "next"]
    assert requests[1]["data"] == {}
    assert requests[2]["data"]["seq"] == 0
    assert requests[2]["data"]["instruction"] == "go left"
    assert base64.b64decode(requests[2]["data"]["image"]) == jpeg
    assert raw == wire
    assert (tmp_path / "lightnav_response.json").read_bytes() == wire.encode()
    assert (tmp_path / "lightnav_request.json").read_text() == socket.sent[-1]
    records = [json.loads(line) for line in (tmp_path / "lightnav_protocol.jsonl").read_text().splitlines()]
    assert len(records) == 6
    assert sum(record["direction"] == "request" and record["action"] == "next" for record in records) == 1
    assert records[-1]["wire_text"] == wire
    assert list(events) == [f"{action}_{direction}_time" for action in ("login", "reset", "next")
                            for direction in ("request", "response")]
    times = [event["monotonic_ns"] for event in events.values()]
    assert times == sorted(times)
    assert all(event["utc"].endswith("Z") for event in events.values())


@pytest.mark.parametrize("wire", ["invalid-json", prediction(seq=3), prediction([[float("nan"), 0, 0]]),
                                   '{"action":"next","data":{"rc":500,"msg":"decode error"}}'])
def test_failed_prediction_is_saved_before_any_validation(tmp_path, wire):
    socket = FakeSocket(wire)
    raw, _ = client.exchange_one_frame(socket, tmp_path, b"fixture", "go left", 30.0)
    assert (tmp_path / "lightnav_response.json").read_bytes() == wire.encode()
    with pytest.raises(ValueError):
        client.parse_prediction(raw, [480, 270])
    assert (tmp_path / "lightnav_response.json").read_bytes() == wire.encode()


@pytest.mark.parametrize("name", ["lightnav_response.json", "lightnav_request.json", "lightnav_protocol.jsonl"])
def test_existing_protocol_artifact_refuses_all_network_requests(tmp_path, name):
    path = tmp_path / name
    path.write_bytes(b"immutable raw fixture")
    socket = FakeSocket(prediction())
    with pytest.raises(FileExistsError):
        client.exchange_one_frame(socket, tmp_path, b"fixture", "go left", 30.0)
    assert socket.sent == []
    assert path.read_bytes() == b"immutable raw fixture"


@pytest.mark.parametrize("length", [1, 2, 7, 13, 101])
def test_prediction_length_is_unrestricted_and_no_local_yaw_wrapping(length):
    local = np.tile([0.125, -0.25, 4.0], (length, 1))
    parsed, data = client.parse_prediction(prediction(local.tolist()), [480, 270])
    assert parsed.shape == (length, 3)
    np.testing.assert_array_equal(parsed, local)
    assert data["seq"] == 0


def test_stop_is_valid_interface_output():
    local, data = client.parse_prediction(prediction([[0.0, 0.0, 0.0]], stop=True), [480, 270])
    assert data["stop"] is True
    np.testing.assert_array_equal(local, np.zeros((1, 3)))


@pytest.mark.parametrize("overrides", [
    {"seq": 1}, {"seq": False}, {"seq": 0.0}, {"rc": False}, {"rc": 500},
    {"stop": 0}, {"actions": [[1, 0, 0]]}, {"actions": {"actions": [1, 0, 0]}},
    {"pointing": {"frame_size": [270, 480]}},
])
def test_prediction_rejects_protocol_mismatch(overrides):
    with pytest.raises(ValueError):
        client.parse_prediction(prediction(**overrides), [480, 270])


def test_pointing_resolution_matches_original_observation():
    raw, _ = client.parse_prediction(prediction(pointing={"frame_size": [480, 270]}), [480, 270])
    assert raw.shape == (1, 3)


def test_empty_instruction_does_not_send_buffer_only_frame(tmp_path):
    socket = FakeSocket(prediction())
    with pytest.raises(ValueError, match="nonempty"):
        client.exchange_one_frame(socket, tmp_path, b"fixture", "  ", 30.0)
    assert socket.sent == []
    assert list(tmp_path.iterdir()) == []


def test_login_failure_prevents_reset_and_inference(tmp_path):
    socket = FakeSocket(prediction())
    socket.responses = iter(['{"action":"login","data":{"rc":500}}'])
    with pytest.raises(ValueError, match="login failed"):
        client.exchange_one_frame(socket, tmp_path, b"fixture", "go left", 30.0)
    assert len(socket.sent) == 1
    assert not (tmp_path / "lightnav_response.json").exists()
    assert len((tmp_path / "lightnav_protocol.jsonl").read_text().splitlines()) == 2


def test_checkpoint_hash_mismatch_is_rejected():
    manifest = {"files": [{"path": "model.safetensors", "sha256": "a" * 64}]}
    client.verify_expected_checkpoint_hashes(manifest, {"model.safetensors": "a" * 64})
    with pytest.raises(ValueError, match="mismatch"):
        client.verify_expected_checkpoint_hashes(manifest, {"model.safetensors": "b" * 64})
    with pytest.raises(ValueError, match="mismatch"):
        client.verify_expected_checkpoint_hashes(manifest, {"missing.safetensors": "a" * 64})
    with pytest.raises(ValueError, match="declare expected"):
        client.verify_expected_checkpoint_hashes(manifest, {})
