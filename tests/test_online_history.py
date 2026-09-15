"""Pure/mock protocol tests; these fixtures are not experimental evidence."""

from __future__ import annotations

import base64
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from reconciliation.online_history import CaptureQueue, SessionHistory, validate_frame


def frame(index=0, **kw):
    return {"frame_id": f"frame_{index:06d}", "path": f"rgb/frame_{index:06d}.jpg",
            "sha256": "0" * 64, "capture_sim_time_s": index / 4,
            "capture_monotonic_ns": index * 250_000_000,
            "capture_host_utc": "2026-09-15T00:00:00Z", "episode_time_s": index / 4,
            "pose_world": [1.0, 2.0, 0.25], "rendered_state_id": index * 15,
            "bootstrap": index < 4, "stationary": index < 4, **kw}


def contract(capacity=4, storage="ring"):
    return {"num_history_frames": capacity, "history_storage": storage,
            "slowfast_tiers": [], "pool_enable": True, "pool_spatial": 2,
            "pool_mode": "avg", "video_fps": 4.0}


def session(capacity=4, storage="ring", sampler=None):
    history = SessionHistory("Take the left corridor.", contract(capacity, storage), sampler)
    history.login()
    history.reset()
    return history


def send(h, i, *, predict=False):
    pending = h.begin(frame(i), predict=predict, send_monotonic_ns=i * 250_000_000 + 10)
    count = i + 1 if h.contract["history_storage"] == "full_episode_slowfast" else min(i + 1, h.contract["num_history_frames"])
    return h.complete(pending["seq"], server_actions_step=count if predict else None)


def test_one_login_one_reset_and_no_successive_reset():
    h = session()
    for method in (h.login, h.reset):
        with pytest.raises(ValueError):
            method()
    send(h, 0)
    with pytest.raises(ValueError, match="initial reset"):
        h.reset()


def test_login_reset_are_required_before_any_frame():
    h = SessionHistory("Forward", contract())
    with pytest.raises(ValueError, match="login"):
        h.begin(frame(), predict=True, send_monotonic_ns=1)
    with pytest.raises(ValueError, match="initial reset"):
        h.reset()


def test_buffer_only_consumes_seq_and_prediction_index_is_distinct():
    h = session()
    for i in range(3):
        result = send(h, i)
        assert result["wire_seq"] == i
        assert result["prediction_index"] is None
    result = send(h, 3, predict=True)
    assert result["wire_seq"] == 3 and result["prediction_index"] == 0
    assert h.next_seq == 4 and h.prediction_count == 1


def test_ring_history_rollover_and_actual_count():
    h = session(3)
    for i in range(5):
        snapshot = send(h, i, predict=True)
        assert snapshot["actual_history_count"] == min(3, i + 1)
        assert snapshot["history_full"] == (i >= 2)
    assert snapshot["chronological_history_frame_ids"] == [frame(i)["frame_id"] for i in (2, 3, 4)]
    assert snapshot["server_actions_step_observed"] == 3
    assert snapshot["capture_sim_span_s"] == 0.5


def test_full_episode_history_is_not_a_nominal_capacity_ring():
    calls = []
    def sampler(current, count, tiers):
        calls.append((current, count, tiers))
        return [{"frame_ids": [0, current], "pool_spatial": 1, "tier": 0}]
    h = session(3, "full_episode_slowfast", sampler)
    for i in range(6):
        snapshot = send(h, i, predict=True)
    assert snapshot["actual_history_count"] == 6
    assert snapshot["history_full"]
    assert calls[-1] == (5, 6, [])
    assert snapshot["model_input_unique_frame_count_reconstructed"] == 2
    assert not snapshot["internal_selection_directly_observed"]


def test_capacity_is_not_claimed_as_actual_history_length():
    h = session(64)
    for i in range(7):
        snapshot = send(h, i, predict=True)
    assert snapshot["actual_history_count"] == 7
    assert not snapshot["history_full"]


def test_model_padding_does_not_append_fabricated_camera_observations():
    h = session(64)
    snapshot = send(h, 0, predict=True)
    assert snapshot["actual_history_count"] == 1
    assert snapshot["model_input_slots_including_processor_padding_reconstructed"] == 2
    assert snapshot["model_input_unique_frame_count_reconstructed"] == 1
    assert len(h.frames) == 1


def test_pending_inference_keeps_later_captures_out_of_frozen_history():
    h, queue = session(), CaptureQueue(3)
    h.begin(frame(0), predict=True, send_monotonic_ns=1)
    for i in (1, 2, 3):
        queue.append(frame(i))
    assert h.snapshot()["chronological_history_frame_ids"] == [frame()["frame_id"]]
    assert len(queue) == 3
    with pytest.raises(ValueError, match="outstanding"):
        h.begin(queue.pop(), predict=False, send_monotonic_ns=1_000_000_000)
    h.complete(0, server_actions_step=1)
    assert h.pending is None


def test_queued_transmission_uses_original_capture_pose_and_time():
    h, queue = session(), CaptureQueue(3)
    queue.append(frame(1, pose_world=[9, 10, 0.9]))
    queued = queue.pop()
    p = h.begin(queued, predict=True, send_monotonic_ns=9_000_000_000)
    assert p["frame"]["capture_sim_time_s"] == 0.25
    assert p["frame"]["pose_world"] == [9, 10, 0.9]


def test_buffer_and_predict_cannot_append_same_frame_twice():
    h = session()
    send(h, 0)
    with pytest.raises(ValueError, match="duplicate"):
        h.begin(frame(0), predict=True, send_monotonic_ns=1)


@pytest.mark.parametrize("change", [
    {"capture_monotonic_ns": 500}, {"pose_world": [0, float("nan"), 0]},
    {"sha256": "wrong"}, {"rendered_state_id": None}, {"bootstrap": None},
])
def test_invalid_or_future_capture_rejected(change):
    h = session()
    with pytest.raises(ValueError):
        h.begin(frame(0, **change), predict=True, send_monotonic_ns=10)


def test_navigation_instruction_is_immutable():
    h = session()
    with pytest.raises(ValueError, match="cannot change"):
        h.begin(frame(), predict=True, send_monotonic_ns=1, instruction="Turn right")


def test_out_of_order_capture_is_not_silently_sorted():
    h = session()
    send(h, 1)
    with pytest.raises(ValueError, match="chronological"):
        h.begin(frame(0), predict=False, send_monotonic_ns=500_000_000)


def test_simulation_and_host_capture_clocks_cannot_run_backwards():
    h = session()
    send(h, 1)
    with pytest.raises(ValueError, match="backwards"):
        h.begin(frame(2, capture_sim_time_s=0.01), predict=False, send_monotonic_ns=600_000_000)


def test_queue_overflow_is_explicit_and_keeps_older_frames():
    queue = CaptureQueue(1)
    queue.append(frame(0))
    with pytest.raises(OverflowError, match="overflow"):
        queue.append(frame(1))
    assert queue.overflow_count == 1
    assert queue.pop()["frame_id"] == frame(0)["frame_id"]


def test_queue_refuses_duplicates_after_pop():
    queue = CaptureQueue()
    queue.append(frame(0))
    queue.pop()
    with pytest.raises(ValueError, match="already captured"):
        queue.append(frame(0))


def test_bad_server_count_or_seq_poison_session():
    for seq, count in ((3, 1), (0, 99), (0, None)):
        h = session()
        h.begin(frame(0), predict=True, send_monotonic_ns=1)
        with pytest.raises(ValueError):
            h.complete(seq, server_actions_step=count)
        assert h.failed
        with pytest.raises(ValueError, match="failed session"):
            h.begin(frame(1), predict=True, send_monotonic_ns=1_000_000_000)


def test_snapshots_and_source_capture_metadata_are_independent_copies():
    h, captured = session(), frame(0)
    original = deepcopy(captured)
    h.begin(captured, predict=True, send_monotonic_ns=1)
    snapshot = h.snapshot()
    snapshot["frames"][0]["pose_world"][0] = 999
    assert h.frames[0]["pose_world"][0] == 1
    assert captured == original


def test_reconstructed_sampler_cannot_select_unavailable_future_frame():
    h = session(4, "full_episode_slowfast", lambda *a: [{"frame_ids": [0, 1]}])
    h.begin(frame(0), predict=True, send_monotonic_ns=1)
    with pytest.raises(ValueError, match="future"):
        h.snapshot()


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("test_online_worker", ROOT / "scripts/online_lightnav_worker.py")
worker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(worker)


class FakeSocket:
    def __init__(self, final=None):
        self.sent = []
        self.final = final
        self.closed = False

    def send(self, wire):
        self.sent.append(wire)

    def recv(self, **kw):
        request = json.loads(self.sent[-1])
        action, data = request["action"], request["data"]
        if action != "next":
            return json.dumps({"action": action, "data": {"rc": 0}})
        if self.final is not None and data["instruction"]:
            return self.final
        result = {"rc": 0, "seq": data["seq"]}
        if data["instruction"]:
            result.update(stop=False, actions={"step": data["seq"] + 1,
                                               "actions": [[0.125, -0.25, 4.0]]})
        return json.dumps({"action": "next", "data": result})

    def close(self):
        self.closed = True


def setup_worker(tmp_path, final=None):
    (tmp_path / "rgb").mkdir()
    sock, notifications = FakeSocket(final), []
    epoch = 10_000_000_000
    def event():
        nonlocal epoch
        epoch += 1_000
        return {"utc": "2026-09-15T00:00:00Z", "monotonic_ns": epoch}
    obj = worker.Session({"lightnav": {"server_url": "ws://unused.invalid"}},
                         {"episode_id": "fixture", "episode_dir": str(tmp_path),
                          "instruction": "Take the left corridor."}, contract(64), None,
                         connect_fn=lambda *a, **kw: sock, emit_fn=notifications.append, event_fn=event)
    def captured(i):
        path = tmp_path / "rgb" / f"frame_{i:06d}.jpg"
        Image.new("RGB", (8, 6), (i, 20, 30)).save(path, "JPEG")
        return frame(i, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return obj, sock, captured, notifications


def test_streaming_one_session_chronological_buffer_upload_and_raw_bytes(tmp_path):
    obj, sock, captured, notifications = setup_worker(tmp_path)
    for i in range(3):
        assert obj.process_frame({"frame": captured(i), "predict": False})["status"] == "BUFFERED"
    result = obj.process_frame({"frame": captured(3), "predict": True, "chunk_id": "chunk_000"})
    result2 = obj.process_frame({"frame": captured(4), "predict": True, "chunk_id": "chunk_001"})
    assert result["status"] == result2["status"] == "PREDICTION_READY"
    assert result["prediction_index"] == 0 and result["seq"] == 3
    wire = [json.loads(x) for x in sock.sent]
    assert [x["action"] for x in wire] == ["login", "reset"] + ["next"] * 5
    assert [x["data"]["seq"] for x in wire[2:]] == list(range(5))
    assert [x["data"]["instruction"] for x in wire[2:]] == [""] * 3 + ["Take the left corridor."] * 2
    assert base64.b64decode(wire[-2]["data"]["image"]) == (tmp_path / "rgb/frame_000003.jpg").read_bytes()
    assert result["observation"]["capture_sim_time_s"] == 0.75
    assert result["observation"]["pose_world"] == [1, 2, 0.25]
    np.testing.assert_array_equal(np.load(tmp_path / result["raw_local_ref"]["path"]), [[0.125, -0.25, 4]])
    assert len(notifications) == 5 and notifications[-1]["type"] == "request_sent"
    assert result["t_ready_host"]["monotonic_ns"] > result["t_request_host"]["monotonic_ns"]
    closed = obj.close()
    assert closed["wire_login_count"] == closed["wire_reset_count"] == 1
    assert closed["prediction_count"] == 2 and sock.closed


@pytest.mark.parametrize("rows", [1, 7, 13, 101])
def test_arbitrary_raw_n_by_three_keeps_numeric_values_and_own_anchor(tmp_path, rows):
    values = np.tile([0.123456789, -0.25, 4.0], (rows, 1))
    raw = json.dumps({"action": "next", "data": {"rc": 0, "seq": 0, "stop": False,
                                                  "actions": {"step": 1, "actions": values.tolist()}}})
    obj, _, captured, _ = setup_worker(tmp_path, raw)
    result = obj.process_frame({"frame": captured(0), "predict": True, "chunk_id": "chunk_000"})
    assert result["status"] == "PREDICTION_READY"
    np.testing.assert_array_equal(np.load(tmp_path / result["raw_local_ref"]["path"]), values)
    assert (tmp_path / result["response_ref"]["path"]).read_bytes() == raw.encode()
    obj.close()


@pytest.mark.parametrize("payload,status", [
    ({"rc": 500, "seq": 0, "msg": "decode failure"}, "MODEL_ERROR"),
    ({"rc": 0, "seq": 99}, "PROTOCOL_ERROR"),
    ({"rc": 0, "seq": 0, "stop": False, "actions": {"step": 1, "actions": [[float("nan"), 0, 0]]}}, "NONFINITE_OUTPUT"),
    ({"rc": 0, "seq": 0, "stop": False, "actions": {"step": 1, "actions": [[1, 2]]}}, "PROTOCOL_ERROR"),
    ({"rc": 0, "seq": 0, "stop": True, "actions": {"step": 1, "actions": [[0, 0, 0]]}}, "MODEL_STOP"),
])
def test_error_stop_nonfinite_preserved_before_filter(tmp_path, payload, status):
    raw = json.dumps({"action": "next", "data": payload})
    obj, _, captured, _ = setup_worker(tmp_path, raw)
    result = obj.process_frame({"frame": captured(0), "predict": True, "chunk_id": "chunk_000"})
    assert result["status"] == status
    assert (tmp_path / "chunks/chunk_000/response.json").read_bytes() == raw.encode()
    if "actions" in payload:
        assert (tmp_path / "chunks/chunk_000/raw_local.npy").is_file()
    obj.close()


def test_resume_rejects_existing_session_artifacts_before_wire_connect(tmp_path):
    obj, _, _, _ = setup_worker(tmp_path)
    obj.close()
    def forbidden(*a, **kw):
        raise AssertionError("must reject before connection")
    with pytest.raises(FileExistsError):
        worker.Session({"lightnav": {"server_url": "unused"}},
                       {"episode_id": "fixture", "episode_dir": str(tmp_path), "instruction": "go"},
                       contract(), None, connect_fn=forbidden)


def test_captured_file_mismatch_prevents_transmission(tmp_path):
    obj, sock, captured, _ = setup_worker(tmp_path)
    f = captured(0)
    (tmp_path / f["path"]).write_bytes(b"changed capture")
    with pytest.raises(ValueError, match="hash changed"):
        obj.process_frame({"frame": f, "predict": True, "chunk_id": "chunk_000"})
    assert len(sock.sent) == 2
    obj.close()
