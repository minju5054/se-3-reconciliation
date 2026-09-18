"""Saved plot numbers, null semantics, and provenance are independently checkable."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.gp_se2 import sample_gp
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.se2 import compose_poses, se2_exp

PATH = Path(__file__).resolve().parents[1] / "scripts/plot_gp_se2_diag_01.py"
SPEC = importlib.util.spec_from_file_location("plot_gp_diagnostic_test", PATH)
plots = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plots)


def fixture_result():
    times = np.linspace(0, 3., 31)
    b = np.array([2., 3., .7]); velocity = np.array([.3, 0., .4])
    p = compose_poses(b, se2_exp(times[:, None]*velocity))
    v = np.tile(velocity, (31, 1))
    problem = GPProblem(b, velocity, p[1:], p[-1], {}, include_obstacles=False)
    x = problem.vector(p, v)
    original = problem.evaluate(x)
    result = dict(config=problem.config, variable_count=150, candidate_found=False,
                  support_poses=None, support_twists=None, latest_support_poses=p.tolist(), latest_support_twists=v.tolist(),
                  candidate_checks=[dict(iterate="initial", objective=original["objective"], constraint_report=problem.dense_report(x))],
                  callback_snapshots=[dict(iteration=1, elapsed_s=.2, objective=original["objective"],
                                           equality_residuals=original["equality"].tolist(), inequality_margins=original["inequality"].tolist())])
    return problem, result


def test_callback_plot_uses_actual_first_block_and_never_invents_missing_samples():
    _, result = fixture_result()
    result["callback_snapshots"][0]["equality_residuals"][4] = .003
    result["callback_snapshots"][0]["inequality_margins"][4*90+7] = -.6
    result["callback_snapshots"].append(dict(iteration=2, elapsed_s=.3, objective=None,
                                             equality_residuals=None, inequality_margins=None))
    history = plots.history_data(result)
    assert history["elapsed_s"] == [0., .2, .3]
    assert history["lateral_max_abs_m_s"][1:] == [.003, None]
    assert history["linear_acceleration_excess_m_s2"][1:] == [.6, None]
    assert history["angular_acceleration_excess_rad_s2"][-1] is None
    assert history["objective"][-1] is None


def test_rejected_curve_is_labelled_and_numeric_trace_matches_original_gp():
    problem, result = fixture_result()
    trace = plots.trajectory_data(result)
    assert trace["kind"] == "rejected_latest_iterate"
    assert "REJECTED" in trace["label"]
    assert not trace["execution_available"]
    expected = sample_gp(problem.times, np.asarray(result["latest_support_poses"]),
                         np.asarray(result["latest_support_twists"]), np.asarray(trace["times_s"]))
    for key, value in zip(("poses_world", "body_twists", "body_accelerations"), expected):
        np.testing.assert_array_equal(trace[key], value)
    assert set(trace["one_sided_side"]) == {"left", "right"}
    assert len(trace["one_sided_accelerations"]) == 60


def test_sampled_candidate_is_not_full_candidate_without_independent_acceptance():
    _, result = fixture_result()
    result.update(candidate_found=True, support_poses=result["latest_support_poses"], support_twists=result["latest_support_twists"])
    assert plots.trajectory_data(result)["kind"] == "rejected_sampled_candidate"
    accepted = plots.trajectory_data(result, full_accepted=True)
    assert accepted["kind"] == "accepted_candidate"
    assert "not executed" in accepted["label"]
    result.update(candidate_found=False, latest_support_poses=None, latest_support_twists=None)
    missing = plots.trajectory_data(result)
    assert not missing["available"] and missing["poses_world"] is None


def test_profiling_stacks_disjoint_components_not_inclusive_levels():
    result = dict(profiling=dict(numerical_component_seconds=dict(gp_prior_residual=1., gp_prior_whitening=.5,
                      collocation_gp_interpolation=2., gate_gp_interpolation=.5, environment_query=3.),
                  other_evaluate_inclusive_seconds=1., evaluate_inclusive_seconds=8.,
                  unique_vector_evaluations=27, measured_instrumentation_bookkeeping_seconds=.01,
                  calibrated_timer_call_estimate_seconds=.001, instrumentation_overhead_scope="measured plus timer estimate"),
                  solve_wall_time_s=10., setup_wall_time_s=.2, post_solve_validation_time_s=.8,
                  total_setup_solve_post_wall_time_s=11., objective_evaluations=10, equality_evaluations=20, inequality_evaluations=30)
    profile = plots.profiling_parts(result)
    assert profile["disjoint_total_seconds"] == profile["measured_total_seconds"] == 11.
    assert profile["disjoint_wall_seconds"]["solver_outside_evaluate"] == 2.
    assert profile["function_counts"] == dict(objective=10, equality=20, inequality=30, unique_vectors=27)
    assert "measured_bookkeeping_seconds" not in profile["disjoint_wall_seconds"]
    result.update(pre_solve_budget_cost_s=.15, total_pre_setup_solve_post_wall_time_s=11.15)
    charged = plots.profiling_parts(result)
    assert charged["disjoint_wall_seconds"]["seed_or_restoration"] == .15
    assert charged["disjoint_total_seconds"] == pytest.approx(11.15)
    assert charged["measured_total_seconds"] == 11.15


def test_plot_sidecar_copies_exact_numbers_and_detects_source_change(tmp_path):
    source = tmp_path / "source.json"
    source.write_text('{"values":[0,0.002,null]}\n')
    numbers = json.loads(source.read_text())
    fig, ax = plots.plt.subplots()
    ax.plot([0, 1, 2], numbers["values"])
    path = tmp_path / "graph.png"
    record = plots.save_plot(fig, path, "test diagnostic", numbers, [source], evidence_kind="UNIT_TEST")
    sidecar = plots.read(record["sidecar"])
    assert sidecar["numeric_data"] == numbers
    assert sidecar["numeric_data"]["values"][-1] is None
    assert plots.verify_plot_sidecar(record["sidecar"])["valid"]
    source.write_text('{"values":[0,0,null]}\n')
    assert not plots.verify_plot_sidecar(record["sidecar"])["valid"]
    fig, _ = plots.plt.subplots()
    with pytest.raises(FileExistsError):
        plots.save_plot(fig, path, "cannot overwrite", numbers, [source], evidence_kind="UNIT_TEST")
    plots.plt.close(fig)


def test_selected_start_and_missing_phase_do_not_fabricate_success(tmp_path):
    assert plots._is_selected(dict(selected_start=1), Path("start_01.json"))
    assert not plots._is_selected(dict(selected_start=None), Path("start_01.json"))
    assert plots._is_selected(dict(selected_start="start_00.json"), Path("start_00.json"))
    with pytest.raises(FileNotFoundError):
        plots.plot_run(tmp_path)
    assert not (tmp_path / "plots").exists()


def test_serialization_nonfinite_is_null_never_success_zero():
    assert plots.plain(dict(metric=np.nan, series=np.array([1., np.inf]))) == dict(metric=None, series=[1., None])
