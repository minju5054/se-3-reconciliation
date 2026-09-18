"""Synthetic end-to-end artifact/phase fixtures; not experimental evidence.

No optimizer or official MPC is executed here: those interfaces are replaced
with deterministic fakes so these tests isolate freeze and output accounting.
"""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import LineString, box
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.robotless_online import integrate_unicycle

SCRIPT = Path(__file__).resolve().parents[1]/"scripts/run_gp_se2_01.py"
SPEC = importlib.util.spec_from_file_location("gp_se2_runner_test", SCRIPT)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)
REAL_ROOT = SCRIPT.parents[1]


def put_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def prepared_inputs(tmp_path, monkeypatch):
    repo = tmp_path/"synthetic_repo"
    repo.mkdir()
    cfg = yaml.safe_load((REAL_ROOT/"configs/gp_se2_01.yaml").read_text())
    cfg.update(source_run="source", shape_analysis="shape")
    cfg["selection"]["priority_events"] = []
    config = repo/"configs/gp_se2_01.yaml"
    config.parent.mkdir()
    config.write_text(yaml.safe_dump(cfg))
    (repo/"pyproject.toml").write_text("# synthetic source snapshot\n")
    (repo/"src/reconciliation").mkdir(parents=True)
    (repo/"src/reconciliation/se2.py").write_text("# synthetic source snapshot\n")
    environment = repo/"environment"
    (environment/"geometry").mkdir(parents=True)
    put_json(environment/"validation.json", {"valid": True, "fixture_only": True})
    audit = repo/"audit"
    for name in ("source_validation.json", "source_hash_validation.json"):
        put_json(audit/name, {"valid": True, "fixture_only": True})
    put_json(audit/"historical_mpc_audit/provenance.json", {"official_settings": {
        "Q_WEIGHTS": [10., 10., 1.], "CONTROL_RATE_HZ": 10., "OBJNAV_V_MAX": .8,
        "W_MAX": 3., "A_MAX_V": 2., "A_MAX_W": 5.}, "fixture_only": True})
    raw = np.column_stack([np.linspace(.05, .6, 10), np.zeros(10), np.zeros(10)])
    frozen_context = dict(case_id="episode_fixture/handoff_000", episode_id="episode_fixture", handoff_id="handoff_000",
        B_world=[0., 0., 0.], u_minus=[.2, 0.], previous_control=[.2, 0.],
        old_world=raw.tolist(), fresh_world=raw.tolist(), source_files=[], source_root=str(repo/"source"),
        original_capture_pose_world=[0., 0., 0.])
    shapes = []
    shape = dict(arc_m=.55, chord_over_arc=1., yaw_travel_from_observation_deg=0., max_forward_direction_deviation_deg=0.)
    for event_id, timing_pass in (("handoff_000", True), ("handoff_001", False)):
        episode = repo/"source/episodes/episode_fixture"
        path = episode/f"chunks/{event_id}/world.npy"
        path.parent.mkdir(parents=True)
        np.save(path, raw)
        put_json(episode/f"handoffs/{event_id}/context.json", {"B": [0., 0., 0.],
            "fresh_world_ref": {"path": str(path.relative_to(episode)), "sha256": runner.digest(path)}})
        metrics = {"interval_timing": {"inference_client_host_interval": {"capture_count": 2},
            "real_time_pacing_valid": timing_pass, "causal_execution_overlap_observed": True,
            "actual_post_switch_execution_available": True}, "old_raw_sha256": "a"*64,
            "fresh_raw_sha256": "b"*64, "e_perp_m": .01, "abs_e_dir_window_deg": 0., "abs_e_yaw_deg": 0.,
            "fresh_projection": {"window_0_10m": {"available": True, "chord_m": .1}},
            "incoming_execution": {"available": True, "chord_m": .05}}
        put_json(episode/f"handoffs/{event_id}/metrics.json", metrics)
        shapes.append({"episode_id": "episode_fixture", "event_id": event_id, "raw_shape": shape})
    put_json(repo/"shape/events.json", shapes)
    env = HospitalEnvironment(LineString([(8., -5.), (8., 5.)]), box(-2., -2., 5., 5.))
    monkeypatch.setattr(runner, "ROOT", repo)
    monkeypatch.setattr(runner.HospitalEnvironment, "load", lambda path: env)
    monkeypatch.setattr(runner, "load_frozen_context", lambda source, episode, event: deepcopy(frozen_context))
    monkeypatch.setattr(runner.subprocess, "check_output", lambda *a, **kw: "f"*40+"\n")
    return dict(repo=repo, config=config, environment=environment, audit=audit, run=repo/"run", env=env,
                context=frozen_context, cfg=cfg)


def prepare(fixture):
    runner.prepare(fixture["run"], fixture["config"], fixture["environment"], fixture["audit"])
    return fixture["run"]


def test_prepare_freezes_only_eligible_source_based_selection_and_preserves_raw(prepared_inputs):
    f = prepared_inputs
    before = runner.file_records(f["repo"]/"source")
    run = prepare(f)
    manifest = runner.read(run/"case_manifest.json")
    assert manifest["planned_count"] == 12
    assert manifest["source_event_count"] == 2
    assert manifest["eligible_count"] == manifest["selected_count"] == 1
    assert manifest["selected"][0]["selected_group"] == "A_SMALL_STRAIGHT"
    assert not manifest["selection_used_new_optimization_or_rollouts"]
    rejected = [r for r in manifest["all_source_decisions"] if not r["eligible"]]
    assert rejected[0]["rejection_reasons"] == ["exact_timing_or_overlap_failed"]
    case = run/"cases/episode_fixture__handoff_000"
    native, common = np.load(case/"F_native.npy"), np.load(case/"F_common.npy")
    assert len(native) == 10 and len(common) == 30
    assert not np.array_equal(common[0], f["context"]["B_world"])
    assert runner.read(case/"goal_route.json")["goal_world"] == native[-1].tolist()
    assert runner.file_records(f["repo"]/"source") == before
    assert (run/"environment/geometry/render_geometry.json").is_file()
    runner.verify_frozen(run)


def test_prepare_and_atomic_writers_refuse_all_overwrites(prepared_inputs):
    run = prepare(prepared_inputs)
    with pytest.raises(FileExistsError):
        prepare(prepared_inputs)
    with pytest.raises(FileExistsError):
        runner.write(run/"protocol.json", {})
    with pytest.raises(FileExistsError):
        runner.array(run/"cases/episode_fixture__handoff_000/F_common.npy", np.zeros((30, 3)))


@pytest.mark.parametrize("where", ["source", "hash", "environment"])
def test_source_and_environment_validation_are_gates_before_output(prepared_inputs, where):
    f = prepared_inputs
    path = {"source": f["audit"]/"source_validation.json", "hash": f["audit"]/"source_hash_validation.json",
            "environment": f["environment"]/"validation.json"}[where]
    put_json(path, {"valid": False})
    with pytest.raises(ValueError, match="PASS"):
        prepare(f)
    assert not f["run"].exists()


@pytest.mark.parametrize("relative", ["config_snapshot.yaml", "case_manifest.json", "environment/validation.json",
    "cases/episode_fixture__handoff_000/input_context.json", "cases/episode_fixture__handoff_000/goal_route.json",
    "cases/episode_fixture__handoff_000/F_common.npy", "cases/episode_fixture__handoff_000/F_native.npy",
    "cases/episode_fixture__handoff_000/reference_preparation.json"])
def test_every_frozen_case_input_mutation_is_detected(prepared_inputs, relative):
    run = prepare(prepared_inputs)
    path = run/relative
    path.write_bytes(path.read_bytes()+b" ")
    with pytest.raises((ValueError, AssertionError)):
        runner.verify_frozen(run)


def test_code_change_after_freeze_is_detected(prepared_inputs):
    run = prepare(prepared_inputs)
    (prepared_inputs["repo"]/"src/reconciliation/se2.py").write_text("# changed\n")
    with pytest.raises(ValueError, match="implementation changed"):
        runner.verify_frozen(run)


def fake_optimizer(common, *, failed=False):
    return dict(status="NO_FEASIBLE_CANDIDATE_FOUND" if failed else "CANDIDATE_FOUND",
        candidate_world=None if failed else common.copy(), support_poses=None, support_twists=None,
        factor_costs=None, constraint_report=None, attempts=[dict(initialization="synthetic_mock_only",
        termination="MOCK_NUMERICAL_FAILURE" if failed else "MOCK_SUCCESS", solver_success=not failed,
        iterations=0, wall_time_s=.001)], infeasibility_proven=False)


def fake_external_batch(run, f):
    output = f["repo"]/"mock_mpc_output"
    request = runner.read(run/"mpc_request.json")
    for case in request["cases"]:
        target = output/(case["episode_id"]+"__"+case["handoff_id"])
        runner.write(target/"historical_solve_audit.json", {"passed": True, "fixture_only": True})
        for method, candidate in case["methods"].items():
            if candidate is None:
                runner.write(target/method/"status.json", {"status": "NO_CANDIDATE", "rollout_performed": False})
                continue
            poses = [np.array([0., 0., 0.])]
            commands, solves = [], []
            for tick in range(180):
                command = [.2, 0.]
                if tick % 6 == 0:
                    solves.append({"time_s": tick/60, "command": command, "success": True,
                        "official_solve_wall_s": .001, "submit_wait_poll_wall_s": .002})
                commands.append({"time_s": tick/60, "end_time_s": (tick+1)/60, "command": command})
                poses.append(integrate_unicycle(poses[-1], command, 1/60))
            rollout = dict(case_id="episode_fixture/handoff_000", horizon_s=3., control_hz=10., integration_hz=60.,
                candidate_world=np.load(candidate).tolist(), candidate_source={"path": candidate, "sha256": runner.digest(candidate)},
                initial_physical_command=[.2, 0.], initial_previous_control=[.2, 0.], initial_pose_world=[0., 0., 0.],
                states=[{"tick": i, "time_s": i/60, "pose_world": p.tolist()} for i, p in enumerate(poses)],
                commands=commands, controller_reference_selections=solves, controller_failure_count=0,
                resolved_limits={"v_max": .8, "omega_max": 3., "a_v_max": 2., "a_omega_max": 5.})
            runner.write(target/method/"rollout.json", rollout)
    runner.write(output/"provenance.json", {"request_sha256": runner.digest(run/"mpc_request.json"), "fixture_only": True})
    runner.write(output/"summary.json", {"fixture_only": True, "source_files_preserved": True})
    runner.write(output/"output_hashes.json", {"files": runner.file_records(output)})
    return output


def mock_optimization(f, monkeypatch):
    monkeypatch.setattr(runner, "solve_rigid", lambda b, v, common, goal, config, **kw: fake_optimizer(common))
    monkeypatch.setattr(runner, "solve_gp", lambda b, v, common, goal, config, **kw: fake_optimizer(common, failed=True))
    run = prepare(f)
    runner.optimize(run)
    return run


def test_fake_full_phases_keep_failed_methods_and_real_vs_plan_outcomes_separate(prepared_inputs, monkeypatch):
    f = prepared_inputs
    run = mock_optimization(f, monkeypatch)
    request = runner.read(run/"mpc_request.json")
    assert request["cases"][0]["methods"]["M3_GP_CONSTRAINED"] is None
    with pytest.raises(FileExistsError):
        runner.optimize(run)
    output = fake_external_batch(run, f)
    runner.evaluate(run, output)
    summary = runner.read(run/"aggregate/summary.json")
    assert summary["methods"]["M0_NATIVE"]["primary_success"] == 1
    assert summary["methods"]["M3_GP_CONSTRAINED"]["candidates"] == 0
    assert summary["native_success_to_gp_failure"] == 1
    assert summary["adapter_success_to_gp_failure"] == 1
    rows = runner.read(run/"aggregate/method_results.json")
    assert len(rows) == 5
    failed = next(row for row in rows if row["method"] == "M3_GP_CONSTRAINED")
    assert failed["failure_reasons"] == ["no_candidate"]
    assert failed["rollout_metrics"]["execution"] is None
    assert failed["deformation"] is None
    for method in runner.METHODS:
        target = run/"cases/episode_fixture__handoff_000/methods"/method
        assert (target/"solver_result.json").is_file()
        assert (target/"plan_validation.json").is_file()
        assert (target/"metrics.json").is_file()
        assert (target/"hashes.json").is_file()
    with pytest.raises(FileExistsError):
        runner.evaluate(run, output)


def test_modified_external_rollout_is_rejected_before_evaluation(prepared_inputs, monkeypatch):
    f = prepared_inputs
    run = mock_optimization(f, monkeypatch)
    output = fake_external_batch(run, f)
    path = output/"episode_fixture__handoff_000/M0_NATIVE/rollout.json"
    data = runner.read(path)
    data["states"][1]["pose_world"][0] += 100.
    put_json(path, data)
    with pytest.raises(ValueError):
        runner.evaluate(run, output)


@pytest.mark.parametrize("mutation", ["request", "historical_audit", "candidate", "physical_initial", "memory_initial"])
def test_rehashed_unrelated_or_invalid_mpc_batch_still_rejected(prepared_inputs, monkeypatch, mutation):
    f = prepared_inputs
    run = mock_optimization(f, monkeypatch)
    output = fake_external_batch(run, f)
    base = output/"episode_fixture__handoff_000"
    if mutation == "request":
        path = output/"provenance.json"
        data = runner.read(path)
        data["request_sha256"] = "0"*64
    elif mutation == "historical_audit":
        path = base/"historical_solve_audit.json"
        data = runner.read(path)
        data["passed"] = False
    else:
        path = base/"M0_NATIVE/rollout.json"
        data = runner.read(path)
        if mutation == "candidate":
            data["candidate_world"][0][0] += .01
        elif mutation == "physical_initial":
            data["initial_physical_command"][0] += .01
        else:
            data["initial_previous_control"][0] += .01
    put_json(path, data)
    hashes = output/"output_hashes.json"
    hashes.unlink()
    runner.write(hashes, {"files": runner.file_records(output)})
    with pytest.raises(ValueError):
        runner.evaluate(run, output)


def test_primary_protocol_values_cannot_be_silently_changed(prepared_inputs):
    f = prepared_inputs
    cfg = yaml.safe_load(f["config"].read_text())
    cfg["footprint"]["radius_m"] = .1
    f["config"].write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="protocol"):
        prepare(f)
    assert not f["run"].exists()
