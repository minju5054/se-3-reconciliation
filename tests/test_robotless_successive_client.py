"""Synthetic protocol/file-handoff tests; no actual model, network or Isaac."""

from __future__ import annotations

import base64
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/lightnav/robotless_successive_inference.py"
SPEC = importlib.util.spec_from_file_location("robotless_successive_client", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)


def prediction(seq, waypoints=None, **overrides):
    data = {"rc": 0, "seq": seq, "stop": False,
            "actions": {"actions": [[1., 0., 0.]] if waypoints is None else waypoints}}
    data.update(overrides)
    return json.dumps({"action": "next", "data": data})


class FakeSocket:
    def __init__(self, first=None, second=None):
        self.sent = []
        self.responses = iter([
            '{ "action": "login", "data": {"rc": 0} }\n',
            '{ "action": "reset", "data": {"rc": 0} }\n',
            prediction(0) if first is None else first,
            prediction(1) if second is None else second,
        ])
        self.enters = 0
        self.exits = 0

    def __enter__(self):
        self.enters += 1
        return self

    def __exit__(self, *_):
        self.exits += 1

    def send(self, wire):
        self.sent.append(wire)

    def recv(self, *, timeout):
        assert timeout == 30.0
        item = next(self.responses)
        if isinstance(item, Exception):
            raise item
        return item


class FakeConnector:
    def __init__(self, socket):
        self.socket = socket
        self.calls = []

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.socket


def exchange(tmp_path, socket, frames=None, **kwargs):
    connector = FakeConnector(socket)
    chunks, metadata = client.exchange_successive_frames(
        "ws://synthetic.invalid", tmp_path,
        [b"synthetic JPEG zero\x00\xff", b"synthetic JPEG one\x00\xfe"] if frames is None else frames,
        [[480, 270], [480, 270]], "turn left", 30.0, connect_fn=connector, **kwargs,
    )
    return chunks, metadata, connector


def test_one_connection_one_login_one_reset_two_ordered_predictions(tmp_path):
    first = '{ "action":"next","data":{"rc":0,"seq":0,"stop":false,"actions":{"actions":[[1,0,0],[2,0,4]]}}}\n'
    second = prediction(1, [[.25, -.1, 1.2]])
    socket = FakeSocket(first, second)
    chunks, metadata, connector = exchange(tmp_path, socket)
    assert len(connector.calls) == 1
    assert socket.enters == socket.exits == 1
    requests = [json.loads(wire) for wire in socket.sent]
    assert [item["action"] for item in requests] == ["login", "reset", "next", "next"]
    assert requests[1]["data"] == {}
    assert [item["data"]["seq"] for item in requests[2:]] == [0, 1]
    assert [item["data"]["instruction"] for item in requests[2:]] == ["turn left", "turn left"]
    assert [base64.b64decode(item["data"]["image"]) for item in requests[2:]] == [
        b"synthetic JPEG zero\x00\xff", b"synthetic JPEG one\x00\xfe",
    ]
    assert [chunk["seq"] for chunk in chunks] == [0, 1]
    for seq, wire in enumerate((first, second)):
        assert (tmp_path / f"response_{seq:03d}.json").read_bytes() == wire.encode()
        assert (tmp_path / f"request_{seq:03d}.json").read_text() == socket.sent[seq + 2]
        np.testing.assert_array_equal(np.load(tmp_path / f"chunk_{seq:03d}.npy"), chunks[seq]["local"])
    records = [json.loads(line) for line in (tmp_path / "lightnav_protocol.jsonl").read_text().splitlines()]
    assert len(records) == 8
    assert {record["connection_id"] for record in records} == {metadata["session"]["connection_id"]}
    assert [r["action"] for r in records if r["direction"] == "request"] == ["login", "reset", "next", "next"]
    assert metadata["session"]["request_sequence"] == [0, 1]
    assert metadata["session"]["login_count"] == metadata["session"]["reset_count"] == 1
    times = [event["monotonic_ns"] for event in metadata["events"].values()]
    assert times == sorted(times)
    assert all(event["utc"].endswith("Z") for event in metadata["events"].values())
    for chunk in chunks:
        assert chunk["round_trip_latency_ms"] == (
            chunk["response_receive_time"]["monotonic_ns"] - chunk["request_send_time"]["monotonic_ns"]
        ) / 1e6


@pytest.mark.parametrize("n0,n1", [(1, 1), (7, 2), (13, 101)])
def test_arbitrary_independent_horizons_and_cumulative_local_yaw_preserved(tmp_path, n0, n1):
    old = np.tile([.1, -.2, 9.], (n0, 1))
    fresh = np.tile([.3, .4, -8.], (n1, 1))
    chunks, _, _ = exchange(tmp_path, FakeSocket(prediction(0, old.tolist()), prediction(1, fresh.tolist())))
    np.testing.assert_array_equal(chunks[0]["local"], old)
    np.testing.assert_array_equal(chunks[1]["local"], fresh)
    assert chunks[0]["local"].shape == (n0, 3)
    assert chunks[1]["local"].shape == (n1, 3)


@pytest.mark.parametrize("wire", ["bad JSON", prediction(1), prediction(0, []),
    prediction(0, [[float("nan"), 0, 0]]), prediction(0, [[0, float("inf"), 0]]),
    prediction(0, [[1, 2]]), prediction(0, stop=0)])
def test_invalid_old_reply_preserved_and_prevents_second_prediction(tmp_path, wire):
    socket = FakeSocket(wire)
    connector = FakeConnector(socket)
    with pytest.raises((ValueError, TypeError)):
        client.exchange_successive_frames("ws://fixture", tmp_path, [b"a", b"b"], [[480, 270]] * 2,
                                           "left", 30., connect_fn=connector)
    assert len(connector.calls) == 1
    assert len(socket.sent) == 3
    assert (tmp_path / "response_000.json").read_bytes() == wire.encode()
    assert not (tmp_path / "chunk_000.npy").exists()
    assert not (tmp_path / "request_001.json").exists()


@pytest.mark.parametrize("wire", ["bad JSON", prediction(0), prediction(1, []),
    prediction(1, [[0, 0, float("nan")]])])
def test_invalid_fresh_reply_preserves_old_and_exact_fresh_response(tmp_path, wire):
    socket = FakeSocket(second=wire)
    with pytest.raises((ValueError, TypeError)):
        exchange(tmp_path, socket)
    assert len(socket.sent) == 4
    np.testing.assert_array_equal(np.load(tmp_path / "chunk_000.npy"), [[1, 0, 0]])
    assert (tmp_path / "response_001.json").read_bytes() == wire.encode()
    assert not (tmp_path / "chunk_001.npy").exists()


@pytest.mark.parametrize("name", ["lightnav_protocol.jsonl", "request_000.json", "request_001.json",
    "response_000.json", "response_001.json", "chunk_000.npy", "chunk_001.npy"])
def test_any_existing_raw_output_prevents_network_and_overwrite(tmp_path, name):
    path = tmp_path / name
    path.write_bytes(b"immutable existing raw")
    socket = FakeSocket()
    connector = FakeConnector(socket)
    with pytest.raises(FileExistsError):
        client.exchange_successive_frames("ws://fixture", tmp_path, [b"a", b"b"], [[480, 270]] * 2,
                                           "left", 30., connect_fn=connector)
    assert connector.calls == []
    assert socket.sent == []
    assert path.read_bytes() == b"immutable existing raw"


def test_completed_exchange_cannot_be_retried(tmp_path):
    exchange(tmp_path, FakeSocket())
    original = {p.name: p.read_bytes() for p in tmp_path.iterdir()}
    with pytest.raises(FileExistsError):
        exchange(tmp_path, FakeSocket())
    assert original == {p.name: p.read_bytes() for p in tmp_path.iterdir()}


def test_network_timeout_does_not_retry_reset_or_reconnect(tmp_path):
    socket = FakeSocket(second=TimeoutError("synthetic timeout"))
    connector = FakeConnector(socket)
    with pytest.raises(TimeoutError):
        client.exchange_successive_frames("ws://fixture", tmp_path, [b"a", b"b"], [[480, 270]] * 2,
                                           "left", 30., connect_fn=connector)
    assert len(connector.calls) == 1
    assert [json.loads(wire)["action"] for wire in socket.sent] == ["login", "reset", "next", "next"]
    assert (tmp_path / "chunk_000.npy").exists()
    assert not (tmp_path / "response_001.json").exists()


@pytest.mark.parametrize("action,index", [("login", 0), ("reset", 1)])
def test_handshake_failure_prevents_predictions_and_is_preserved(tmp_path, action, index):
    socket = FakeSocket()
    replies = ['{"action":"login","data":{"rc":0}}', '{"action":"reset","data":{"rc":0}}']
    replies[index] = json.dumps({"action": action, "data": {"rc": 500}})
    socket.responses = iter(replies[:index + 1])
    with pytest.raises(ValueError):
        exchange(tmp_path, socket)
    assert len(socket.sent) == index + 1
    records = [json.loads(line) for line in (tmp_path / "lightnav_protocol.jsonl").read_text().splitlines()]
    assert records[-1]["wire_text"] == replies[index]
    assert not (tmp_path / "request_000.json").exists()


def test_binary_response_bytes_are_preserved_verbatim(tmp_path):
    wire = prediction(1).encode() + b"\n\t"
    chunks, _, _ = exchange(tmp_path, FakeSocket(second=wire))
    assert chunks[1]["seq"] == 1
    assert (tmp_path / "response_001.json").read_bytes() == wire
    last = json.loads((tmp_path / "lightnav_protocol.jsonl").read_text().splitlines()[-1])
    assert last["wire_text"] is None
    assert base64.b64decode(last["wire_bytes_base64"]) == wire


def test_stop_flag_does_not_cancel_required_second_observation(tmp_path):
    chunks, _, _ = exchange(tmp_path, FakeSocket(prediction(0, [[0, 0, 0]], stop=True)))
    assert len(chunks) == 2
    assert chunks[0]["response_data"]["stop"] is True


@pytest.mark.parametrize("instruction", ["", " ", None])
def test_empty_instruction_prevents_all_network(tmp_path, instruction):
    socket = FakeSocket()
    connector = FakeConnector(socket)
    with pytest.raises(ValueError):
        client.exchange_successive_frames("ws://fixture", tmp_path, [b"a", b"b"], [[480, 270]] * 2,
                                           instruction, 30., connect_fn=connector)
    assert connector.calls == []
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("timeout", [0., -1., float("nan"), float("inf")])
def test_invalid_timeout_prevents_all_network(tmp_path, timeout):
    connector = FakeConnector(FakeSocket())
    with pytest.raises(ValueError):
        client.exchange_successive_frames("ws://fixture", tmp_path, [b"a", b"b"], [[480, 270]] * 2,
                                           "left", timeout, connect_fn=connector)
    assert connector.calls == []


def test_parse_default_still_requires_seq_zero_and_explicit_seq_one_works():
    with pytest.raises(ValueError):
        client.parse_prediction(prediction(1), [480, 270])
    local, response = client.parse_prediction(prediction(1), [480, 270], expected_seq=1)
    assert local.shape == (1, 3)
    assert response["seq"] == 1


def test_image_input_is_verified_without_reencoding(tmp_path):
    (tmp_path / "raw").mkdir()
    data = io.BytesIO()
    Image.new("RGB", (8, 4), (17, 29, 51)).save(data, format="JPEG", quality=91)
    payload = data.getvalue()
    path = tmp_path / "raw/observation_000.jpg"
    path.write_bytes(payload)
    observation = {"rgb": {"path": "raw/observation_000.jpg", "sha256": client.sha256_file(path),
                             "resolution_width_height": [8, 4]}}
    verified, resolution = client.verify_input_image(tmp_path, observation, 0)
    assert verified == payload
    assert resolution == [8, 4]
    observation["rgb"]["sha256"] = "f" * 64
    with pytest.raises(ValueError, match="SHA-256"):
        client.verify_input_image(tmp_path, observation, 0)


def test_full_handoff_anchors_each_saved_raw_chunk_to_its_own_observation(tmp_path, monkeypatch):
    """Synthetic end-to-end file fixture exercises persistence and observation binding."""
    import yaml
    from reconciliation.robotless_successive_chunks import SUCCESSIVE_FRAME_CONVENTIONS

    raw = tmp_path / "raw"
    raw.mkdir()
    checkout = tmp_path / "external"
    for relative in ("docs/PROTOCOL.md", "src/lightnav/cli/ws_client.py",
                     "src/lightnav/serving/ws_server.py", "src/lightnav/serving/protocol.py"):
        path = checkout / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("synthetic provenance fixture")
    config = {
        "stage": "synthetic-successive-test", "instruction": "turn left",
        "agent": {"pose_world": [2., 3., np.pi / 2], "local_displacement": [.3, 0., 0.]},
        "paths": {"lightnav_checkout": str(checkout), "checkpoint_path": str(tmp_path / "checkpoint")},
        "lightnav": {"expected_git_sha": "a" * 40, "checkpoint_identifier": "synthetic/model",
                     "checkpoint_revision": "b" * 40, "server_url": "ws://synthetic.invalid",
                     "checkpoint_sha256": {"model.safetensors": "c" * 64}},
    }
    (tmp_path / "config_snapshot.yaml").write_text(yaml.safe_dump(config))
    (tmp_path / "server_process.json").write_text('{"synthetic_only":true}')
    observations = []
    poses = [[2., 3., np.pi / 2], [2., 3.3, np.pi / 2]]
    for seq in (0, 1):
        path = raw / f"observation_{seq:03d}.jpg"
        Image.new("RGB", (8, 4), (17 + seq, 29, 51)).save(path, format="JPEG")
        observations.append({
            "seq": seq, "label": ("OLD", "FRESH")[seq], "observation_id": f"observation_{seq:03d}",
            "instruction": "turn left", "agent_pose_world": poses[seq],
            "observation_time": {"utc": "2026-09-14T00:00:00Z", "monotonic_ns": seq + 1, "simulation_time_s": 0.},
            "camera": {"configured": {"synthetic": True}, "actual_intrinsics": {"resolution": [8, 4]}, "T_agent_camera": np.eye(4).tolist()},
            "rgb": {"path": f"raw/observation_{seq:03d}.jpg", "sha256": client.sha256_file(path), "resolution_width_height": [8, 4]},
        })
    metadata = {
        "schema_version": 1, "R0": poses[0], "R1": poses[1], "configured_local_displacement": [.3, 0., 0.],
        "observations": observations, "instruction": "turn left", "scene": {"synthetic_only": True},
        "coordinate_conventions": SUCCESSIVE_FRAME_CONVENTIONS, "execution_time": None,
        "research_git_sha": "d" * 40, "lightnav_git_sha": "a" * 40,
        "model_checkpoint_identifier": "synthetic/model",
        "config": {"path": "config_snapshot.yaml", "sha256": client.sha256_file(tmp_path / "config_snapshot.yaml")},
    }
    (tmp_path / "metadata.json").write_text(json.dumps(metadata))
    source = {"path": str(checkout), "git_sha": "a" * 40, "clean": True, "status_porcelain": ""}
    monkeypatch.setattr(client, "git_state", lambda _: source.copy())
    monkeypatch.setattr(client, "checkpoint_manifest", lambda _: {
        "path": "synthetic", "files": [{"path": "model.safetensors", "sha256": "c" * 64}],
    })
    monkeypatch.setattr(client, "verify_checkpoint_unchanged", lambda _: None)
    monkeypatch.setattr(client.importlib.metadata, "version", lambda _: "synthetic")
    # The transport implementation itself still executes its complete session.
    connector = FakeConnector(FakeSocket(prediction(0, [[1., 0., 0.]]), prediction(1, [[1., 0., 0.], [2., 0., 0.]])))
    original_exchange = client.exchange_successive_frames
    monkeypatch.setattr(client, "exchange_successive_frames", lambda *args: original_exchange(*args, connect_fn=connector))
    before_images = [p.read_bytes() for p in sorted(raw.glob("*.jpg"))]
    result = client.run_inference(tmp_path, timeout_s=30.)
    assert len(connector.calls) == 1
    assert [chunk["raw_shape"] for chunk in result["chunks"]] == [[1, 3], [2, 3]]
    old_world = np.load(tmp_path / "derived/chunk_000_world.npy")
    fresh_world = np.load(tmp_path / "derived/chunk_001_world.npy")
    np.testing.assert_allclose(old_world, [[2., 4., np.pi / 2]], atol=1e-12)
    np.testing.assert_allclose(fresh_world, [[2., 4.3, np.pi / 2], [2., 5.3, np.pi / 2]], atol=1e-12)
    assert before_images == [p.read_bytes() for p in sorted(raw.glob("*.jpg"))]
    raw_bytes = {p.name: p.read_bytes() for p in raw.iterdir()}
    with pytest.raises(FileExistsError):
        client.run_inference(tmp_path, timeout_s=30.)
    assert len(connector.calls) == 1
    assert raw_bytes == {p.name: p.read_bytes() for p in raw.iterdir()}
    for seq, chunk in enumerate(result["chunks"]):
        assert chunk["seq"] == seq
        assert chunk["agent_pose_world_at_observation"] == poses[seq]
        assert chunk["raw_inputs"]["waypoints"]["sha256"] == client.sha256_file(raw / f"chunk_{seq:03d}.npy")
        assert chunk["world_waypoints"]["sha256"] == client.sha256_file(tmp_path / f"derived/chunk_{seq:03d}_world.npy")
    assert json.loads((tmp_path / "inference_metadata.json").read_text()) == result
