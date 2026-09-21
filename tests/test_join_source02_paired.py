"""Synthetic protocol/checker fixtures only; no real LightNav predictions."""
import base64
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("join_source02_paired_test", ROOT / "scripts/lightnav/join_source02_paired.py")
paired = importlib.util.module_from_spec(spec)
spec.loader.exec_module(paired)


@pytest.fixture
def inputs(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    frames = []
    for idx in range(17):
        path = source / f"{idx}.jpg"
        Image.fromarray(np.full((8, 12, 3), idx * 10, dtype=np.uint8)).save(path, "JPEG")
        frames.append({"frame_id": f"capture_{idx}", "path": str(path),
                       "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                       "rendered_state_id": f"render_{idx}", "capture_sim_time_s": idx * .25,
                       "capture_monotonic_ns": idx + 1, "pose_world": [1., 2., 0.3],
                       "bootstrap": False, "stationary": False,
                       "resolution_width_height": [12, 8], "camera_prim_path": "/World/Camera"})
    frames[16]["capture_sim_time_s"] = frames[15]["capture_sim_time_s"]
    manifest = {"condition_id": "condition_01", "instruction": "Go to the chair and stop.",
                "history": frames[:15], "final_frames": {"A": frames[15], "B": frames[16], "SHAM": frames[15]},
                "config_path": str(tmp_path / "config.yaml")}
    protocol = {"development": {"condition_order": ["condition_01"]}}
    return manifest, protocol


class FakeSocket:
    def __init__(self, stop=False, fail=False):
        self.stop = stop
        self.fail = fail
        self.requests = []
        self.frames = 0
        self.closed = False

    def send(self, wire):
        self.requests.append(json.loads(wire))

    def recv(self, **kwargs):
        request = self.requests[-1]
        action, data = request["action"], request["data"]
        if action in ("login", "reset"):
            return json.dumps({"action": action, "data": {"rc": 0}})
        self.frames += 1
        response = {"rc": 0, "seq": data["seq"]}
        if data["instruction"]:
            if self.fail:
                response.update(rc=500, msg="fixture decoder failure")
            else:
                response.update(actions={"step": self.frames, "actions": [[0., 0., 0.]] if self.stop else [[.3, .1, .2], [.6, .2, .3]]},
                                stop=self.stop, visible=None, raw_text="fixture")
        return json.dumps({"action": action, "data": response})

    def close(self):
        self.closed = True


def run_fixture(tmp_path, manifest, *, fail_branch=None):
    sockets = []
    def factory(config, command, contract, sampler, **kwargs):
        name = command["episode_id"].rsplit("_", 1)[-1]
        ws = FakeSocket(stop=name == "B", fail=name == fail_branch)
        sockets.append(ws)
        return paired.Session(config, command, contract, sampler, connect_fn=lambda *a, **k: ws, **kwargs)
    config = {"lightnav": {"server_url": "ws://fixture"}}
    contract = {"num_history_frames": 64, "history_storage": "ring", "pool_enable": False}
    results = paired.run_branches(tmp_path / "out", manifest, config, contract, None, session_factory=factory)
    return results, sockets


def test_exact_independent_session_order_and_sham(inputs, tmp_path):
    manifest, protocol = inputs
    paired.validate_manifest(manifest, protocol, now_ns=100)
    before = [Path(f["path"]).read_bytes() for f in manifest["history"]]
    results, sockets = run_fixture(tmp_path, manifest)
    assert len(sockets) == 3
    assert len({r["session_close"]["connection_id"] for r in results.values()}) == 3
    for branch, ws in zip(paired.BRANCHES, sockets):
        assert [r["action"] for r in ws.requests] == ["login", "reset"] + ["next"] * 16
        assert all(not r["data"]["instruction"] for r in ws.requests[2:-1])
        assert ws.requests[-1]["data"]["instruction"] == manifest["instruction"]
        assert results[branch]["wire_parity"]["valid"]
        assert results[branch]["wire_parity"]["actual_terminal_predictions_sent"] == 1
        assert results[branch]["session_close"]["prediction_count"] == 1
        assert results[branch]["session_close"]["retry_count"] == 0
        assert ws.closed
    for a, b, sham in zip(sockets[0].requests[2:-1], sockets[1].requests[2:-1], sockets[2].requests[2:-1]):
        assert a["data"]["image"] == b["data"]["image"] == sham["data"]["image"]
    assert sockets[0].requests[-1]["data"]["image"] == sockets[2].requests[-1]["data"]["image"]
    assert sockets[0].requests[-1]["data"]["image"] != sockets[1].requests[-1]["data"]["image"]
    assert before == [Path(f["path"]).read_bytes() for f in manifest["history"]]


def test_stop_optional_pointing_and_variable_horizon_preserved(inputs, tmp_path):
    results, _ = run_fixture(tmp_path, inputs[0])
    assert results["B"]["status"] == "MODEL_STOP"
    assert results["B"]["terminal_prediction"]["stop"] is True
    assert "pointing" not in results["B"]["terminal_prediction"]["response"]
    assert len(results["B"]["terminal_prediction"]["raw_local"]) == 1
    assert len(results["A"]["terminal_prediction"]["raw_local"]) == 2
    assert results["A"]["terminal_prediction"]["transform"]["anchor_pose_world"] == [1., 2., .3]


def test_failure_preserved_no_retry_and_sham_still_attempted(inputs, tmp_path):
    results, sockets = run_fixture(tmp_path, inputs[0], fail_branch="B")
    assert results["B"]["status"] == "MODEL_ERROR"
    assert results["SHAM"]["status"] == "PREDICTION_READY"
    assert sum(bool(r["data"].get("instruction")) for ws in sockets for r in ws.requests) == 3
    assert (tmp_path / "out/branches/B/chunks/terminal/response.json").is_file()


@pytest.mark.parametrize("mutation,match", [
    (lambda m: m.update(condition_id="not_declared"), "undeclared"),
    (lambda m: m["history"].reverse(), "chronological"),
    (lambda m: m["history"].__setitem__(1, deepcopy(m["history"][0])), "duplicated"),
    (lambda m: m["history"][0].update(capture_monotonic_ns=1000), "future"),
    (lambda m: m["final_frames"]["B"].update(pose_world=[2., 2., .3]), "pose"),
    (lambda m: m["final_frames"]["B"].update(capture_sim_time_s=4.0), "simulation"),
    (lambda m: m["final_frames"].update(SHAM=deepcopy(m["final_frames"]["B"])), "sham"),
    (lambda m: m["history"].pop(), "H includes"),
])
def test_invalid_pair_rejected_before_requests(inputs, mutation, match):
    manifest, protocol = deepcopy(inputs)
    mutation(manifest)
    with pytest.raises(ValueError, match=match):
        paired.validate_manifest(manifest, protocol, now_ns=100)


def test_hash_change_and_no_overwrite(inputs, tmp_path):
    manifest, protocol = inputs
    run_fixture(tmp_path, manifest)
    with pytest.raises(FileExistsError):
        run_fixture(tmp_path, manifest)
    path = Path(manifest["history"][0]["path"])
    path.write_bytes(path.read_bytes() + b"changed")
    with pytest.raises(ValueError, match="hash mismatch"):
        paired.validate_manifest(manifest, protocol, now_ns=100)


def test_source_wire_parity_detects_corruption(inputs, tmp_path):
    manifest = inputs[0]
    run_fixture(tmp_path, manifest)
    branch = tmp_path / "out/branches/A"
    frames = paired.read_json(branch / "replayed_inputs.json")["frames"]
    p = branch / "requests/wire_000017_next_request.json"
    wire = paired.read_json(p)
    wire["data"]["image"] = base64.b64encode(b"not the source JPEG").decode()
    p.write_text(json.dumps(wire))
    assert not paired.wire_parity(branch, frames)["valid"]
