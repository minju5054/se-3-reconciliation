"""Synthetic restoration/audit tests; these are not actual saved-event evidence."""
from __future__ import annotations

import copy

import numpy as np
import pytest

import reconciliation.gp_se2_diag03_audit as audit
from reconciliation.gp_se2_formulation import GPProblem, _constraint_report
from reconciliation.se2 import compose_poses, se2_exp


def synthetic_saved_record(phase="final"):
    boundary = np.array([2., -3., 0.7])
    twist = np.array([0.3, 0., 0.2])
    times = np.arange(31)*0.1
    poses = compose_poses(boundary, se2_exp(times[:, None]*twist))
    twists = np.tile(twist, (31, 1))
    problem = GPProblem(boundary, twist, poses[1:], poses[-1], {},
                        workspace_margin=lambda xy: np.full(xy.shape[:-1], 2.),
                        include_obstacles=False)
    vector = problem.vector(poses, twists)
    saved_poses, saved_twists = problem.unpack(vector)
    solver = dict(initial_vector=vector.tolist(), latest_iterate=vector.tolist(),
                  latest_support_poses=saved_poses.tolist(), latest_support_twists=saved_twists.tolist(),
                  termination="CONVERGED", solver_success=True, candidate_found=False,
                  candidate_vector=None, selected_iterate=None)
    initial_metadata = dict(chart_reconstructed_poses=saved_poses.tolist(),
                            chart_reconstructed_twists=saved_twists.tolist())
    spec = next(r for r in audit.record_specs() if r["phase"] == phase)
    return spec, {"problem": problem}, solver, vector, initial_metadata


def test_rejected_candidate_null_does_not_hide_saved_latest_iterate():
    args = synthetic_saved_record()
    result = audit.restore_record(*args)
    assert result["available"]
    assert not result["candidate_available"]
    assert result["source_vector_key"] == "latest_iterate"
    assert result["solver_converged"]
    assert result["reconstruction"]["consistent"]
    np.testing.assert_array_equal(result["vector"], args[3])
    assert result["selected_iterate"] is None


@pytest.mark.parametrize("phase,key", [("initial", "initial_vector"), ("final", "latest_iterate")])
def test_missing_saved_vectors_are_explicit_and_never_filled(phase, key):
    spec, case, solver, initial, metadata = synthetic_saved_record(phase)
    solver[key] = None
    result = audit.restore_record(spec, case, solver, initial, metadata)
    assert result["available"] is False
    assert result["status"] == "MISSING_SAVED_VECTOR"
    assert key in result["reason"]
    assert "vector" not in result
    assert result["record_id"] == spec["record_id"]


@pytest.mark.parametrize("bad", [np.zeros(149), np.zeros((30, 5)), np.full(150, np.nan), np.full(150, np.inf)])
def test_invalid_final_vector_is_not_interpreted_as_valid_or_zero(bad):
    spec, case, solver, initial, metadata = synthetic_saved_record()
    solver["latest_iterate"] = bad
    with pytest.raises(ValueError, match="shape/nonfinite"):
        audit.restore_record(spec, case, solver, initial, metadata)


def test_seed_must_match_saved_file_exactly_even_below_reproduction_tolerance():
    spec, case, solver, initial, metadata = synthetic_saved_record()
    altered = initial.copy()
    altered[0] += 1e-12
    with pytest.raises(ValueError, match="initialization"):
        audit.restore_record(spec, case, solver, altered, metadata)


def test_seed_file_shape_is_part_of_saved_input_integrity():
    spec, case, solver, initial, metadata = synthetic_saved_record()
    with pytest.raises(ValueError):
        audit.restore_record(spec, case, solver, initial.reshape(75, 2), metadata)


@pytest.mark.parametrize("key,bad", [("latest_support_poses", np.zeros(3)),
                                     ("latest_support_twists", np.zeros((30, 3))),
                                     ("latest_support_poses", np.full((31, 3), np.nan)),
                                     ("latest_support_twists", np.full((31, 3), np.inf))])
def test_saved_support_shape_and_finiteness_are_explicit_integrity_checks(key, bad):
    spec, case, solver, initial, metadata = synthetic_saved_record()
    solver[key] = bad
    with pytest.raises(ValueError):
        audit.restore_record(spec, case, solver, initial, metadata)


def test_periodic_pose_agreement_is_not_literal_array_agreement():
    spec, case, solver, initial, metadata = synthetic_saved_record()
    poses = np.asarray(solver["latest_support_poses"])
    poses[:, 2] += 2*np.pi
    solver["latest_support_poses"] = poses.tolist()
    result = audit.restore_record(spec, case, solver, initial, metadata)
    assert not result["reconstruction"]["pose_literal_equal"]
    assert result["reconstruction"]["consistent"]
    assert result["reconstruction"]["pose_periodic_max_error"] < 1e-10
    assert result["reconstruction"]["twist_literal_equal"]


def test_missing_support_arrays_remain_declared_missing():
    spec, case, solver, initial, metadata = synthetic_saved_record()
    solver["latest_support_poses"] = None
    result = audit.restore_record(spec, case, solver, initial, metadata)
    assert result["available"]
    assert not result["reconstruction"]["consistent"]
    assert result["reconstruction"]["reason"] == "MISSING_SAVED_SUPPORT_ARRAYS"


def test_initial_phase_uses_saved_seed_metadata_not_latest_support():
    spec, case, solver, initial, metadata = synthetic_saved_record("initial")
    solver["latest_support_poses"] = None
    result = audit.restore_record(spec, case, solver, initial, metadata)
    assert result["source_vector_key"] == "initial_vector"
    assert result["reconstruction"]["consistent"]


def test_nine_provenance_records_survive_duplicate_vector_hashes():
    specs = audit.record_specs()
    assert len(specs) == len({r["record_id"] for r in specs}) == 9
    assert sum(r["role"] == "hard_initial" for r in specs) == 4
    assert sum(r["role"] == "hard_final" for r in specs) == 4
    assert sum(r["role"] == "benign_final" for r in specs) == 1
    _, case, solver, initial, metadata = synthetic_saved_record()
    rows = [audit.restore_record(spec, case, solver, initial, metadata) for spec in specs]
    assert len({r["vector_sha256"] for r in rows}) == 1
    assert len(rows) == 9
    assert {r["case_id"] for r in rows} == {audit.HARD, audit.BENIGN}
    assert {r["method"] for r in rows} == set(audit.METHODS)
    assert {r["initialization"] for r in rows} == set(audit.INITIALIZATIONS)


def test_numeric_comparison_retains_literal_numerical_and_shape_distinctions():
    same = audit.numeric_comparison([1., 2.], [1., 2.])
    nearby = audit.numeric_comparison([1.+1e-9, 2.], [1., 2.])
    outside = audit.numeric_comparison([1.+1e-6, 2.], [1., 2.])
    wrong_shape = audit.numeric_comparison([[1., 2.]], [1., 2.])
    assert same["literal_equal"] and same["numerical_agreement"]
    assert not nearby["literal_equal"] and nearby["numerical_agreement"]
    assert not outside["numerical_agreement"]
    assert not wrong_shape["shape_equal"]
    assert audit.PROTOCOL["original_reproduction_absolute_tolerance"] == 1e-8
    assert audit.PROTOCOL["original_reproduction_relative_tolerance"] == 1e-10


def test_compare_tree_does_not_replace_missing_records_or_boolean_failure():
    rows = audit.compare_tree({"full": False, "goal": {"valid": False}},
                              {"full": True, "goal": {"valid": False}, "missing": 1.})
    mapped = {r["field"]: r for r in rows}
    assert not mapped["full"]["numerical_agreement"]
    assert mapped["goal.valid"]["numerical_agreement"]
    assert mapped["missing"]["reason"] == "missing"


def test_actual_audit_queries_do_not_invoke_optimization_rollout_or_official_runtime(monkeypatch):
    import scipy.optimize
    import reconciliation.gp_se2_formulation as formulation
    import reconciliation.gp_se2_diag_solver as diag_solver
    import reconciliation.gp_se2_diag02_solver as diag02_solver
    import reconciliation.gp_se2_diag02_derivatives as supplied
    import reconciliation.gp_se2_rollout as rollout
    import reconciliation.online_mpc_adapter as adapter

    def forbidden(*args, **kwargs):
        raise AssertionError("read-only audit invoked a forbidden solver/runtime entry")
    for module, name in ((scipy.optimize, "minimize"), (formulation, "minimize"),
                         (formulation, "solve_gp"), (formulation, "solve_rigid"),
                         (diag_solver, "run_instrumented"), (diag02_solver, "run_instrumented"),
                         (diag02_solver, "minimize"), (supplied, "DerivativeProvider"),
                         (rollout, "counterfactual_rollout"), (rollout, "_synchronous_solve"),
                         (rollout, "historical_solve_audit"), (adapter, "load_official")):
        monkeypatch.setattr(module, name, forbidden)
    spec, case, solver, initial, metadata = synthetic_saved_record()
    record = audit.restore_record(spec, case, solver, initial, metadata)
    problem = case["problem"]
    value = problem.evaluate(initial)
    dense = problem.dense_report(initial)
    full = dict(original_dense_report=dense, original_plan_valid=True, full_feasible=True,
                additional_grid={"flags": {"lateral_velocity": True, "linear_speed": True}})
    # Environment checking has separate unchanged-core tests. This fixture
    # isolates restoration plus real GP interval/derivative queries.
    monkeypatch.setattr(audit, "check_full_candidate", lambda *args, **kwargs: copy.deepcopy(full))
    solver["candidate_checks"] = [dict(iterate="latest_iterate", objective=value["objective"],
        collocation=_constraint_report(value["equality"], value["inequality"], problem.config),
        constraint_report=dense)]
    post = {"rows": [{"iterate": "latest_iterate", "full_feasible": True}]}
    before_vector = initial.copy()
    before_config = copy.deepcopy(problem.config)
    result = audit.audit_record(record, case, solver, post, None)
    assert result["original_reproduction"]["reproduced"]
    assert result["acceptance"]["full_feasible"]
    assert result["derivatives"]["summary"]["interpolation_derivative_consistent"]
    assert result["intervals"]["optimizer_calls"] == 0
    np.testing.assert_array_equal(before_vector, initial)
    assert problem.config == before_config


def test_unavailable_record_never_calls_original_checker(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("missing vector must not be replaced with a synthetic path")
    monkeypatch.setattr(audit, "check_full_candidate", forbidden)
    record = dict(available=False, status="MISSING_SAVED_VECTOR", record_id="missing")
    assert audit.audit_record(record, None, None, None, None) is record


def test_summary_finds_knot_failure_even_if_interval_worst_is_interior():
    record = dict(available=True, role="hard_final", record_id="synthetic", vector_sha256="same",
        original_reproduction={"reproduced": True}, reconstruction={"consistent": True},
        acceptance={"collocation_feasible": False, "dense_feasible": False},
        derivatives={"summary": {"interpolation_derivative_consistent": True,
                                  "coefficient_cross_check_consistent": True}},
        original_full={"original_config": {"formulation": {"a_v_max": 2., "a_w_max": 5., "inequality_tolerance": 1e-5}}},
        intervals={"knot_limits": [{"time_s": .1, "body_acceleration_left": [-2.001, 0, 0],
                                     "body_acceleration_right": [0, 0, 0]}]},
        interval_extrema=[{"grid": "SUPPLEMENTAL_0.001_OFFSET", "quantity": "ax",
                           "worst_knot_side": None, "maximum_tolerance_excess": .1}],
        grid_detection=[])
    result = audit.summarize([record])
    assert result["one_sided_knot_violation_observed"]
    assert result["insufficient_saved_evidence"]
    assert not result["original_results_reproduced"]
    assert not result["implementation_inconsistency_found"]
