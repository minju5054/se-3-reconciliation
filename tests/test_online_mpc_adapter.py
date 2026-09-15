"""Synthetic unit/mock tests. These are not online experiment evidence."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import json

import numpy as np
import pytest

from reconciliation.online_mpc_adapter import (
    OfficialMpcAdapter, PINNED_LIGHTNAV_SHA, audited_tracker, load_official,
    selection_audit, sha256,
)
from reconciliation.se2 import local_trajectory_to_world


class MockTracker:
    def __init__(self):
        self._generation = 0
        self._future = None
        self._trajectory = None
        self.previous_command = self.command = (0., 0.)
        self.error = ""
        self.solve_timing = {}
        self.submissions = []

    def reset(self):
        self._generation += 1
        self._trajectory = None
        self.previous_command = self.command = (0., 0.)
        self.error = ""

    def set_body_path(self, raw, anchor):
        self._trajectory = local_trajectory_to_world(anchor, raw)
        self._generation += 1
        self.error = ""

    def submit(self, pose):
        if self._future is not None:
            return
        self._future = Future()
        self.submissions.append((self._generation, pose.copy(), self.previous_command))

    def complete(self, reference, command=(.1, .2), *, generation=None):
        generation = self.submissions[-1][0] if generation is None else generation
        self.solve_timing[generation] = {
            "solve_start_host_monotonic_s": 101., "solve_end_host_monotonic_s": 102.}
        self._future.set_result(SimpleNamespace(
            generation=generation, command=command, reference=np.asarray(reference),
            prediction=np.zeros((6, 3)), solve_ms=3.))

    def poll(self):
        if self._future is None or not self._future.done():
            return None
        future, self._future = self._future, None
        try:
            result = future.result()
        except Exception as exc:
            self.error = str(exc)
            self.previous_command = self.command = (0., 0.)
            return self.command
        if result.generation != self._generation:
            return None
        self.previous_command = self.command = result.command
        return result.command

    def close(self):
        self._generation += 1


@pytest.fixture
def configured(tmp_path):
    tracker = MockTracker()
    module = SimpleNamespace(HORIZON=5, Q_WEIGHTS=(10., 10., 1.))
    adapter = OfficialMpcAdapter(module, tracker, monotonic=lambda: 100.)
    adapter.reset("ep0")
    raw = np.array([[.1, 0., 0.], [.2, 0., 0.], [.3, 0., 0.]], dtype=np.float32)
    path = tmp_path / "raw.npy"
    np.save(path, raw)
    adapter.install(episode_id="ep0", chunk_id="chunk0", reference_version=0,
                    raw_local_path=path, capture_pose=[0, 0, 0], raw_sha256=sha256(path))
    return adapter, tracker, path


def submit(adapter, solve_id="s0"):
    return adapter.submit(solve_id=solve_id, pose=[0., 0., 0.], input_state_id=8,
                          input_sim_time_s=.5, input_host_monotonic_s=99.)


def reference(adapter):
    return adapter.world[[1, 2, 2, 2, 2]].copy()


def test_install_retains_capture_anchor_raw_dtype_and_previous_command(configured):
    adapter, tracker, path = configured
    before = path.read_bytes()
    tracker.previous_command = tracker.command = (.2, -.5)
    result = adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                             raw_local_path=path, capture_pose=[2., 3., np.pi/2])
    assert result["command_activated"] is False
    assert tracker.command == (.2, -.5)
    np.testing.assert_allclose(adapter.world[:, 0], 2., atol=1e-15)
    np.testing.assert_allclose(adapter.world[:, 1], 3. + np.load(path)[:, 0], atol=2e-7)
    assert path.read_bytes() == before
    assert np.load(path).dtype == np.float32


def test_submit_is_nonblocking_single_pending_and_preserves_input_state(configured):
    adapter, tracker, _ = configured
    result = submit(adapter)
    assert result["status"] == "submitted"
    assert result["input_state_id"] == 8
    assert result["input_sim_time_s"] == .5
    assert result["input_host_monotonic_s"] == 99.
    assert adapter.poll() is None
    assert submit(adapter, "s1")["status"] == "busy"
    assert len(tracker.submissions) == 1


def test_current_generation_result_logs_actual_selection_and_timing(configured):
    adapter, tracker, _ = configured
    submit(adapter)
    tracker.complete(reference(adapter))
    result = adapter.poll()
    assert result["status"] == "command"
    assert result["command_available_for_application"] is True
    assert result["chunk_id"] == "chunk0"
    assert result["solve_id"] == "s0"
    assert result["selection"]["indices"] == [1, 2, 2, 2, 2]
    assert result["selection"]["nearest_index"] == 0
    assert result["solve_start_host_monotonic_s"] == 101.
    assert result["solve_end_host_monotonic_s"] == 102.
    assert result["official_solve_ms"] == 3.
    assert adapter.poll() is None
    json.dumps(result, allow_nan=False)


def test_late_old_result_after_fresh_install_keeps_old_ids_and_is_rejected(configured):
    adapter, tracker, path = configured
    submit(adapter)
    old_reference = reference(adapter)
    adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                    raw_local_path=path, capture_pose=[1, 2, 1])
    tracker.complete(old_reference)
    result = adapter.poll()
    assert result["status"] == "stale_rejected"
    assert result["command_available_for_application"] is False
    assert result["reference_version"] == 0
    assert result["chunk_id"] == "chunk0"
    assert tracker.command == (0., 0.)
    assert submit(adapter, "s1")["reference_version"] == 1


def test_late_result_after_episode_reset_keeps_original_episode(configured):
    adapter, tracker, _ = configured
    submit(adapter)
    ref = reference(adapter)
    adapter.reset("ep1")
    tracker.complete(ref)
    result = adapter.poll()
    assert result["episode_id"] == "ep0"
    assert result["status"] == "stale_rejected"


@pytest.mark.parametrize("stale", [False, True])
def test_solver_exception_never_becomes_an_activatable_fresh_command(configured, stale):
    adapter, tracker, path = configured
    submit(adapter)
    if stale:
        adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                        raw_local_path=path, capture_pose=[1, 0, 0])
    tracker._future.set_exception(RuntimeError("mock solver failure"))
    result = adapter.poll()
    assert result["status"] == ("stale_rejected" if stale else "controller_error")
    assert result["command_available_for_application"] is False
    assert "mock solver failure" in result["error"]


def test_same_numerical_command_has_new_reference_identity(configured):
    adapter, tracker, path = configured
    submit(adapter)
    tracker.complete(reference(adapter))
    old = adapter.poll()
    adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                    raw_local_path=path, capture_pose=[0, 0, 0])
    submit(adapter, "s1")
    tracker.complete(reference(adapter))
    fresh = adapter.poll()
    assert old["command"] == fresh["command"]
    assert old["reference_version"] != fresh["reference_version"]
    assert fresh["previous_command"] == old["command"]


@pytest.mark.parametrize("n", [1, 2, 9, 17])
def test_arbitrary_row_count_and_endpoint_repetition(configured, n):
    adapter, _, path = configured
    np.save(path, np.column_stack([np.arange(n), np.zeros(n), np.zeros(n)]))
    adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                    raw_local_path=path, capture_pose=[0, 0, 0])
    indices = [min(i, n-1) for i in range(1, 6)]
    result = selection_audit(adapter.world, [0, 0, 0], adapter.world[indices],
                             horizon=5, weights=(10, 10, 1))
    assert result["indices"] == indices


def test_rotation_only_rows_are_pose_selected_not_discarded_by_xy_arc():
    path = np.array([[0, 0, 0], [0, 0, .5], [0, 0, 1.], [0, 0, 1.5]])
    selected = path[[2, 3, 3, 3, 3]]
    audit = selection_audit(path, [0, 0, .51], selected, horizon=5, weights=(10, 10, 1))
    assert audit["nearest_index"] == 1
    assert audit["indices"] == [2, 3, 3, 3, 3]


def test_selection_yaw_periodicity_and_unwrapped_reference_are_preserved():
    path = np.array([[0, 0, 3.1], [0, 0, -3.1], [0, 0, -3.]])
    ref = path[[1, 2, 2, 2, 2]].copy()
    ref[:, 2] += 2*np.pi
    result = selection_audit(path, [0, 0, 3.1], ref, horizon=5, weights=(10, 10, 1))
    assert result["reference_world"] == ref.tolist()


@pytest.mark.parametrize("column", [0, 1, 2])
def test_reference_audit_mismatch_is_controller_error(configured, column):
    adapter, tracker, _ = configured
    submit(adapter)
    ref = reference(adapter)
    ref[0, column] += .1
    tracker.complete(ref)
    result = adapter.poll()
    assert result["status"] == "controller_error"
    assert result["command_available_for_application"] is False


@pytest.mark.parametrize("bad_command", [(np.nan, 0), (0, np.inf)])
def test_nonfinite_controller_result_is_json_serializable_error(configured, bad_command):
    adapter, tracker, _ = configured
    submit(adapter)
    tracker.complete(reference(adapter), command=bad_command)
    result = adapter.poll()
    assert result["status"] == "controller_error"
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("field,value", [("input_sim_time_s", np.nan),
                                         ("input_sim_time_s", -.1),
                                         ("input_host_monotonic_s", np.inf),
                                         ("pose", [0, np.nan, 0])])
def test_invalid_state_time_fails_before_official_submission(configured, field, value):
    adapter, tracker, _ = configured
    payload = dict(solve_id="s", pose=[0, 0, 0], input_state_id=2,
                   input_sim_time_s=1., input_host_monotonic_s=5.)
    payload[field] = value
    with pytest.raises(ValueError):
        adapter.submit(**payload)
    assert tracker._future is None


@pytest.mark.parametrize("bad", [np.empty((0, 3)), np.ones((3, 2)),
                                 np.array([[np.nan, 0, 0]])])
def test_invalid_raw_fails_before_install(configured, bad):
    adapter, tracker, path = configured
    old_generation = tracker._generation
    np.save(path, bad)
    with pytest.raises(ValueError):
        adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=1,
                        raw_local_path=path, capture_pose=[0, 0, 0])
    assert tracker._generation == old_generation


def test_hash_failure_and_wrong_episode_fail_closed(configured):
    adapter, _, path = configured
    base = dict(episode_id="ep0", chunk_id="fresh", reference_version=1,
                raw_local_path=path, capture_pose=[0, 0, 0])
    with pytest.raises(ValueError, match="hash"):
        adapter.install(**base, raw_sha256="wrong")
    base["episode_id"] = "different"
    with pytest.raises(ValueError, match="matching episode"):
        adapter.install(**base)


def test_ids_versions_and_closed_adapter_are_checked(configured):
    adapter, tracker, path = configured
    with pytest.raises(ValueError, match="increase"):
        adapter.install(episode_id="ep0", chunk_id="fresh", reference_version=0,
                        raw_local_path=path, capture_pose=[0, 0, 0])
    with pytest.raises(ValueError, match="chunk_id"):
        adapter.install(episode_id="ep0", chunk_id="chunk0", reference_version=1,
                        raw_local_path=path, capture_pose=[0, 0, 0])
    submit(adapter)
    tracker.complete(reference(adapter))
    adapter.poll()
    with pytest.raises(ValueError, match="solve_id"):
        submit(adapter)
    adapter.close()
    with pytest.raises(ValueError, match="closed"):
        adapter.poll()


@pytest.mark.parametrize("git_sha,status", [("wrong", ""), (PINNED_LIGHTNAV_SHA, " M mpc.py")])
def test_external_source_pin_rejects_before_import(monkeypatch, tmp_path, git_sha, status):
    answers = iter([git_sha, status])
    monkeypatch.setattr("reconciliation.online_mpc_adapter.subprocess.check_output",
                        lambda *a, **kw: next(answers))
    with pytest.raises(ValueError, match="clean and at the pinned"):
        load_official(tmp_path)


def test_external_mpc_hash_rejects_before_import(monkeypatch, tmp_path):
    source = tmp_path / "mujoco_demo/vln_mujoco/mpc.py"
    source.parent.mkdir(parents=True)
    source.write_text("raise RuntimeError('must not import')")
    answers = iter([PINNED_LIGHTNAV_SHA, ""])
    monkeypatch.setattr("reconciliation.online_mpc_adapter.subprocess.check_output",
                        lambda *a, **kw: next(answers))
    with pytest.raises(ValueError, match="source hash"):
        load_official(tmp_path)


def test_timing_subclass_delegates_unchanged_inputs_and_result():
    sentinel = object()
    class Base:
        def _solve(self, *values):
            self.values = values
            return sentinel
    clock = iter([20., 23.])
    tracker = audited_tracker(SimpleNamespace(MpcTracker=Base), monotonic=lambda: next(clock))
    args = (7, np.zeros(3), np.ones((5, 3)), (.2, .3))
    assert tracker._solve(*args) is sentinel
    assert tracker.values == args
    assert tracker.solve_timing[7] == {"solve_start_host_monotonic_s": 20.,
                                       "solve_end_host_monotonic_s": 23.}


def test_worker_dispatch_preserves_install_activation_distinction(configured):
    adapter, _, path = configured
    script = Path(__file__).parents[1] / "scripts/online_mpc_worker.py"
    spec = importlib.util.spec_from_file_location("online_mpc_worker_test", script)
    worker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(worker)
    request = dict(request_id=1, op="install", episode_id="ep0", chunk_id="fresh",
                   reference_version=1, raw_local_path=str(path), capture_pose=[0, 0, 0])
    assert worker.dispatch(adapter, request)["command_activated"] is False
    assert request["op"] == "install"
    with pytest.raises(ValueError, match="unknown"):
        worker.dispatch(adapter, {"request_id": 2, "op": "retune"})
