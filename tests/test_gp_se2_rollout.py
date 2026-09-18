"""Synthetic semantics tests plus a separately labelled historical runtime audit.

Mock candidates/rollouts are implementation fixtures, never research evidence.
"""
from concurrent.futures import Future
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from reconciliation.gp_se2_rollout import (
    candidate_in_capture_frame, counterfactual_rollout, dense_rollout_samples,
    execution_metrics, historical_solve_audit, load_frozen_context, save_rollout,
    verify_source_records,
)
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import local_trajectory_to_world, wrap_angle


def mock_module(command=(.2, .3), failure_at=None):
    instances = []

    def reference(path, pose, horizon, weights):
        errors = path-pose
        errors[:, 2] = wrap_angle(errors[:, 2])
        nearest = np.argmin(np.sum(errors*errors*np.array(weights), axis=1))
        selected = path[np.minimum(nearest+1+np.arange(horizon), len(path)-1)].copy()
        yaw = pose[2]
        for p in selected:
            p[2] = yaw+wrap_angle(p[2]-yaw)
            yaw = p[2]
        return selected

    class Tracker:
        def __init__(self):
            self.previous_command = self.command = (0., 0.)
            self.error = ""
            self._future = None
            self.calls = []
            self.closed = False
            instances.append(self)

        def set_body_path(self, path, anchor):
            self._trajectory = local_trajectory_to_world(anchor, path)

        def submit(self, pose):
            self.calls.append((pose.copy(), self.previous_command))
            self._future = Future()
            if len(self.calls)-1 == failure_at:
                self._future.set_exception(RuntimeError("synthetic numerical failure"))
            else:
                ref = reference(self._trajectory, pose, 5, (10., 10., 1.))
                self._future.set_result(SimpleNamespace(command=tuple(command), reference=ref,
                    prediction=np.vstack([pose, ref]), solve_ms=100000.))

        def poll(self):
            future, self._future = self._future, None
            try:
                result = future.result()
            except Exception as exc:
                self.error = str(exc)
                self.previous_command = self.command = (0., 0.)
                return self.command
            self.previous_command = self.command = result.command
            self.error = ""
            return self.command

        def close(self):
            self.closed = True

    return SimpleNamespace(MpcTracker=Tracker, build_pose_aligned_reference=reference,
        HORIZON=5, CONTROL_RATE_HZ=10., Q_WEIGHTS=(10., 10., 1.), R_WEIGHTS=(.1, .1),
        OBJNAV_V_MAX=.8, W_MAX=3., A_MAX_V=2., A_MAX_W=5., instances=instances)


@pytest.fixture
def frozen():
    return {"case_id": "synthetic/not_evidence", "B_world": [3., -2., 2.7],
        "u_minus": [.1, .2], "previous_control": [.3, -.1],
        "original_capture_pose_world": [2., -3., -2.9], "source_files": []}


@pytest.fixture
def candidate():
    return np.array([[3.1, -1.9, 2.8], [3.4, -1.8, -3.1], [3.5, -1.7, -2.9]])


def test_original_capture_transform_and_wrapped_yaw_are_not_reanchoring(frozen, candidate):
    original = candidate.copy()
    local, report = candidate_in_capture_frame(candidate, frozen["original_capture_pose_world"])
    back = local_trajectory_to_world(frozen["original_capture_pose_world"], local)
    np.testing.assert_allclose(back[:, :2], original[:, :2], atol=1e-12)
    np.testing.assert_allclose(wrap_angle(back[:, 2]-original[:, 2]), 0, atol=1e-12)
    assert report["B_reanchoring"] is False
    assert report["capture_pose_world"] != frozen["B_world"]
    np.testing.assert_array_equal(original, candidate)


def test_synchronous_grid_no_snap_no_wall_time_motion(frozen, candidate):
    module = mock_module()
    result = counterfactual_rollout(module, frozen, candidate)
    assert len(result["states"]) == 181
    assert len(result["commands"]) == 180
    assert len(result["controller_reference_selections"]) == 30
    assert result["states"][0]["pose_world"] == frozen["B_world"]
    assert result["controller_reference_selections"][0]["time_s"] == 0
    assert result["controller_reference_selections"][-1]["time_s"] == 2.9
    assert result["states"][-1]["time_s"] == 3
    np.testing.assert_allclose(result["states"][-1]["pose_world"], integrate_unicycle(frozen["B_world"], [.2, .3], 3.), atol=1e-12)
    assert result["official_mpc_solve_wall_s"] == [100.]*30  # simulation still exactly3s
    assert all(s["simulation_time_advanced_during_solve_s"] == 0 for s in result["controller_reference_selections"])
    assert result["pose_snaps"] == 0
    assert result["new_lightnav_updates"] == 0
    assert module.instances[0].closed


def test_physical_command_and_controller_memory_remain_distinct(frozen, candidate):
    module = mock_module()
    result = counterfactual_rollout(module, frozen, candidate)
    first = result["controller_reference_selections"][0]
    assert first["previous_control"] == frozen["previous_control"]
    assert result["initial_physical_command"] == frozen["u_minus"]
    assert first["previous_control"] != result["initial_physical_command"]
    metrics = execution_metrics(result, candidate[-1])
    assert metrics["first_command_delta_v_mps"] == pytest.approx(.1)
    assert metrics["first_command_delta_omega_radps"] == pytest.approx(.1)
    assert metrics["control_grid_max_abs_acceleration_v_mps2"] == pytest.approx(1.)


def test_each_candidate_has_new_tracker_and_independent_state(frozen, candidate):
    module = mock_module()
    a = counterfactual_rollout(module, frozen, candidate)
    b = counterfactual_rollout(module, frozen, candidate + [5, 4, 0])
    assert len(module.instances) == 2
    assert a["states"][0] == b["states"][0]
    assert module.instances[0].calls[0][1] == module.instances[1].calls[0][1]
    np.testing.assert_array_equal(module.instances[0].calls[0][0], frozen["B_world"])
    assert a["candidate_world"] != b["candidate_world"]
    # This is a mocked identical-command fixture, not reused recorded execution.
    a["states"][1]["pose_world"][0] = -900
    assert b["states"][1]["pose_world"][0] != -900


def test_reference_rows_logged_from_each_actual_controller_solve(frozen, candidate):
    result = counterfactual_rollout(mock_module(), frozen, candidate)
    for solve in result["controller_reference_selections"]:
        indices = solve["selection"]["indices"]
        ref = np.array(solve["selection"]["reference_world"])
        np.testing.assert_allclose(ref[:, :2], candidate[indices, :2], atol=1e-12)
        assert len(indices) == 5
        assert solve["selection_source"] == "actual official solve result"
        assert solve["selection"]["endpoint_repeated"]


def test_failure_uses_official_reset_continues_horizon_and_remains_failure(frozen, candidate):
    module = mock_module(command=(.8, 2.), failure_at=1)
    result = counterfactual_rollout(module, frozen, candidate)
    assert result["controller_failure_count"] == 1
    failed = result["controller_reference_selections"][1]
    assert failed["command"] == [0., 0.]
    assert not failed["success"]
    assert result["controller_reference_selections"][2]["previous_control"] == [0., 0.]
    assert result["states"][-1]["time_s"] == 3.
    assert result["commands"][6]["command"] == [0., 0.]
    assert result["commands"][12]["command"] == [.8, 2.]
    metric = execution_metrics(result, candidate[-1])
    assert not metric["controller_validity_pass"]
    assert metric["motion_limit_violations"]["linear_acceleration"]
    assert metric["motion_limit_violations"]["angular_acceleration"]


def test_dense_query_exact_held_unicycle_and_does_not_interpolate_chords(frozen, candidate):
    result = counterfactual_rollout(mock_module(command=(.7, 2.5)), frozen, candidate)
    times, poses = dense_rollout_samples(result, .007)
    assert np.max(np.diff(times)) <= .007+1e-12
    assert times[0] == 0 and times[-1] == 3.
    for t, pose in zip(times[1:], poses[1:]):
        np.testing.assert_allclose(pose, integrate_unicycle(frozen["B_world"], [.7, 2.5], t), atol=1e-12)


def test_stationary_smooth_solution_fails_goal(frozen, candidate):
    result = counterfactual_rollout(mock_module(command=(0., 0.)), frozen, candidate)
    metric = execution_metrics(result, candidate[-1])
    assert not metric["goal_reached"]
    assert not metric["terminal_goal_dwell_pass"]
    assert metric["time_to_goal_s"] is None
    assert metric["path_length_m"] == 0
    assert metric["clearance_validity"].startswith("N/A")
    assert metric["route_validity"].startswith("N/A")


def test_historical_solve_uses_recorded_input_not_boundary(frozen, candidate):
    module = mock_module(command=(.2, .3))
    frozen = dict(frozen)
    historical_input = [1., 2., .4]
    local, _ = candidate_in_capture_frame(candidate, frozen["original_capture_pose_world"])
    reference = module.build_pose_aligned_reference(candidate, np.array(historical_input), 5, (10., 10., 1.))
    from reconciliation.online_mpc_adapter import selection_audit
    selected = selection_audit(candidate, historical_input, reference, horizon=5, weights=(10., 10., 1.))
    frozen.update(fresh_raw_local=local, historical_first_fresh_solve={"solve_id": "old",
        "input_pose": historical_input, "previous_command": [.6, .7], "command": [.2, .3], "selection": selected})
    audit = historical_solve_audit(module, frozen)
    assert audit["passed"]
    assert not audit["input_pose_equals_B"]
    assert not audit["B_counterfactual_equality_required"]
    np.testing.assert_array_equal(module.instances[0].calls[0][0], historical_input)
    assert module.instances[0].calls[0][1] == (.6, .7)


def test_output_refuses_overwrite_and_contains_all_streams(tmp_path, frozen, candidate):
    result = counterfactual_rollout(mock_module(), frozen, candidate)
    destination = tmp_path/"method"
    save_rollout(destination, result)
    expected = {"rollout_states.json", "rollout_commands.json", "controller_reference_selections.json",
        "rollout.json", "rollout_times.npy", "rollout_poses_world.npy", "sampled_mpc_reference.npy", "derived_capture_local.npy", "hashes.json"}
    assert {p.name for p in destination.iterdir()} == expected
    verify_source_records(destination, json.loads((destination/"hashes.json").read_text())["files"])
    with pytest.raises(FileExistsError):
        save_rollout(destination, result)


@pytest.mark.parametrize("kwargs", [{"control_hz": 20.}, {"integration_hz": 59.}, {"horizon_s": 3.01}, {"horizon_s": 0.}])
def test_unaligned_or_changed_control_schedule_rejected(frozen, candidate, kwargs):
    with pytest.raises(ValueError):
        counterfactual_rollout(mock_module(), frozen, candidate, **kwargs)


def test_modified_source_hash_rejected(tmp_path):
    path = tmp_path/"immutable.json"
    path.write_text("original")
    import hashlib
    record = {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "bytes": path.stat().st_size}
    verify_source_records(tmp_path, [record])
    path.write_text("tampered")
    with pytest.raises(ValueError, match="integrity"):
        verify_source_records(tmp_path, [record])


def test_real_official_historical_solve_reproducibility_when_available(tmp_path):
    """Existing real recorded solve audit, NOT new counterfactual performance."""
    repo = Path(__file__).resolve().parents[1]
    source = repo/"data/robotless_online_handoffs_v1/primary_20260915T091900Z"
    external = Path("/home/gpuadmin/Workspace/external/LightNav-0-official-demo")
    python = external/"mujoco_demo/.venv/bin/python"
    if not source.is_dir() or not python.is_file():
        pytest.skip("retained real source or official isolated environment unavailable")
    request = tmp_path/"request.json"
    request.write_text(json.dumps({"source_root": str(source), "audit_only": True,
        "cases": [{"episode_id": "episode_008_repeat_01", "handoff_id": "handoff_013"}]}))
    import os
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", PYTHONDONTWRITEBYTECODE="1")
    completed = subprocess.run([str(python), str(repo/"scripts/lightnav/gp_se2_mpc_rollout.py"),
        "--request", str(request), "--output", str(tmp_path/"audit"), "--lightnav-checkout", str(external)],
        env=env, text=True, capture_output=True, timeout=45)
    assert completed.returncode == 0, completed.stderr
    audit = json.loads((tmp_path/"audit/episode_008_repeat_01__handoff_013/historical_solve_audit.json").read_text())
    assert audit["passed"]
    assert not audit["input_pose_equals_B"]
    assert audit["command_error"] == [0., 0.]


def test_progress_and_mpc_compute_are_measured_in_separate_units(frozen, candidate):
    result = counterfactual_rollout(mock_module(command=(.2, 0.)), frozen, candidate)
    metrics = execution_metrics(result, candidate[-1])
    initial = np.linalg.norm(np.asarray(frozen["B_world"])[:2]-candidate[-1, :2])
    terminal = np.linalg.norm(np.asarray(result["states"][-1]["pose_world"])[:2]-candidate[-1, :2])
    assert metrics["initial_goal_distance_m"] == pytest.approx(initial)
    assert metrics["goal_distance_improvement_m"] == pytest.approx(initial-terminal)
    assert metrics["net_displacement_m"] == pytest.approx(.6)
    assert metrics["mpc_solve_count_total"] == 30
    assert metrics["mpc_official_solve_count_available"] == 30
    assert metrics["mpc_official_solve_total_s"] == 3000.
    assert metrics["mpc_official_solve_median_s"] == metrics["mpc_official_solve_max_s"] == 100.
    assert metrics["mpc_submit_wait_poll_count_available"] == 30
    assert result["states"][-1]["time_s"] == 3.


def test_missing_failed_official_compute_is_null_and_counted_separately(frozen, candidate):
    result = counterfactual_rollout(mock_module(failure_at=0), frozen, candidate)
    metrics = execution_metrics(result, candidate[-1])
    assert metrics["mpc_solve_count_total"] == 30
    assert metrics["mpc_official_solve_count_available"] == 29
    assert metrics["mpc_submit_wait_poll_count_available"] == 30
    for row in result["controller_reference_selections"]:
        row["official_solve_wall_s"] = None
    metrics = execution_metrics(result, candidate[-1])
    assert metrics["mpc_official_solve_count_available"] == 0
    assert metrics["mpc_official_solve_total_s"] is None
    assert metrics["mpc_official_solve_max_s"] is None
