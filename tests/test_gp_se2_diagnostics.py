"""Diagnostic classification must not redefine physical acceptance."""
import json

import numpy as np
import pytest
import yaml

from reconciliation.gp_se2_diagnostics import (
    audit_saved_attempts, classify_iterate, classify_saved_attempt,
    detailed_residuals, file_sha256, load_frozen_case,
)
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.se2 import compose_poses, se2_exp


def constant_problem(twist=(.3, 0., .4), horizon=3.):
    times = np.linspace(0, horizon, round(horizon/.1)+1)
    boundary = np.array([2.3, -1.2, .7])
    twist = np.asarray(twist)
    poses = compose_poses(boundary, se2_exp(times[:, None]*twist))
    velocity = np.tile(twist, (len(times), 1))
    problem = GPProblem(boundary, twist, poses[1:], poses[-1], {"horizon_s": horizon},
                        include_obstacles=False)
    return problem, problem.vector(poses, velocity)


@pytest.mark.parametrize("twist", [(.3, 0., 0.), (.3, 0., .4)])
def test_dense_feasible_initial_is_distinct_from_generated_iterate(twist):
    problem, vector = constant_problem(twist)
    assert classify_iterate(problem, vector, initial=True)["classification"] == "INITIAL_ALREADY_FEASIBLE"
    assert classify_iterate(problem, vector)["classification"] == "DENSE_FEASIBLE"
    assert problem.dense_report(vector)["feasible"]


def test_unknown_history_never_fabricates_iterates_or_zero_metrics():
    problem, vector = constant_problem()
    attempt = dict(initial_vector=vector, latest_iterate=vector, termination="TIMEOUT", iterations=7)
    result = classify_saved_attempt(problem, attempt)
    assert result["original_termination"] == "TIMEOUT"
    assert result["classification"] == "INITIAL_ALREADY_FEASIBLE"
    assert result["iteration_history_status"] == "UNKNOWN_NOT_SAVED"
    assert result["best_feasible_intermediate"]["classification"] == "INSUFFICIENT_SAVED_EVIDENCE"
    assert "dense" not in result["best_feasible_intermediate"]
    assert not result["original_acceptance_rewritten"]


def test_nonfinite_saved_vector_is_numerical_error():
    problem, vector = constant_problem()
    vector[2] = np.nan
    result = classify_iterate(problem, vector)
    assert result["classification"] == "NUMERICAL_EVALUATION_ERROR"
    assert "dense" not in result


def test_midpoint_pass_off_collocation_fail_and_original_acceptance_unchanged():
    problem = GPProblem([0, 0, 0], [.3, 0, .08], [[.03, 0, 0]], [.03, 0, 0],
                        {"horizon_s": .1}, include_obstacles=False)
    vector = problem.vector(np.array([[0, 0, 0], [.03, 0, 0]]), np.array([[.3, 0, .08], [.3, 0, .08]]))
    before = problem.dense_report(vector)
    result = classify_iterate(problem, vector)
    after = problem.dense_report(vector)
    assert result["classification"] == "COLLOCATION_PASS_DENSE_FAIL"
    assert result["collocation"]["feasible"]
    assert not result["dense"]["feasible"]
    lateral = [r for r in result["residuals"] if r["family"] == "lateral_velocity"]
    assert all(r["passed"] for r in lateral if r["site"] in ("support", "midpoint"))
    violating = [r for r in lateral if not r["passed"]]
    assert {r["site"] for r in violating} == {"off-collocation"}
    assert all(r["physical_unit"] == "m/s" and r["interval_index"] == 0 for r in violating)
    assert all(r["allowed_minimum"] == r["allowed_maximum"] == 0 for r in violating)
    assert before["maximum_equality_residual"] == after["maximum_equality_residual"]


def test_acceleration_has_both_derivative_sides_and_units():
    problem, vector = constant_problem()
    vector[3] += .2
    result = detailed_residuals(problem, vector)
    acceleration = [r for r in result["residuals"] if r["family"] == "linear_acceleration"
                    and r["phase"] == "dense_one_sided_knots"]
    assert {r["derivative_side"] for r in acceleration} == {"left", "right"}
    assert all(r["physical_unit"] == "m/s^2" for r in acceleration)
    assert all(r["allowed_minimum"] == -2 and r["allowed_maximum"] == 2 for r in acceleration)
    assert all(0 <= r["violation_time_s"] <= 3 and 0 <= r["interval_index"] < 30 for r in acceleration)
    assert any(not r["passed"] for r in acceleration)


def test_goal_position_retains_original_squared_constraint_units():
    problem, vector = constant_problem()
    problem.goal_pose[0] += 1
    result = detailed_residuals(problem, vector)
    row = next(r for r in result["residuals"] if r["family"] == "goal_position_squared_distance")
    assert row["physical_unit"] == "m^2"
    assert row["actual_value_at_maximum_violation"] == pytest.approx(1.)
    assert row["allowed_maximum"] == pytest.approx(.15**2)
    assert row["applicable_tolerance"] == 1e-5


def test_saved_report_reproduction_retains_termination():
    problem, vector = constant_problem()
    report = problem.dense_report(vector)
    attempt = dict(initial_vector=vector, latest_iterate=vector, termination="SOLVER_FAILURE",
                   candidate_checks=[dict(iterate="initial", constraint_report=report)])
    result = classify_saved_attempt(problem, attempt)
    assert result["saved_report_reproduction"][0]["matches"]
    assert result["original_termination"] == "SOLVER_FAILURE"


def test_frozen_loader_uses_manifest_path_physical_twist_and_preserves_hashes(tmp_path):
    class Environment:
        def optimizer_clearance(self, xy, radius):
            return np.ones(len(xy))
        def workspace_margin(self, xy, radius):
            return np.ones(len(xy))
    primary = tmp_path / "primary"
    case = primary / "cases" / "not_guessed_from_case_id"
    case.mkdir(parents=True)
    problem, vector = constant_problem(horizon=.1)
    config = dict(formulation=problem.config, footprint=dict(radius_m=.2))
    (primary / "config_snapshot.yaml").write_text(yaml.safe_dump(config))
    (primary / "case_manifest.json").write_text(json.dumps(dict(selected=[dict(case_id="actual/event", case_directory=case.name)])))
    (case / "input_context.json").write_text(json.dumps(dict(B_world=[2.3, -1.2, .7], u_minus=[.3, .4], previous_control=[.1, .2])))
    (case / "goal_route.json").write_text(json.dumps(dict(goal_world=problem.goal_pose.tolist(), gates=[])))
    for name in ("F_common.npy", "F_native.npy"):
        np.save(case / name, problem.common_reference)
    loaded = load_frozen_case(primary, "actual/event", environment=Environment())
    assert loaded["case_directory"] == case
    assert loaded["problem"].initial_twist.tolist() == [.3, 0., .4]
    assert loaded["context"]["previous_control"] == [.1, .2]
    assert all(file_sha256(path) == digest for path, digest in loaded["hashes"].items())
    with pytest.raises(ValueError, match="manifest"):
        load_frozen_case(primary, "missing/event", environment=Environment())


def test_audit_refuses_historical_output_or_overwrite(tmp_path):
    primary = tmp_path / "historical"
    primary.mkdir()
    with pytest.raises(ValueError, match="historical"):
        audit_saved_attempts(primary, primary / "overwrite")
    assert not (primary / "overwrite").exists()
    output = tmp_path / "existing"
    output.mkdir()
    with pytest.raises(FileExistsError):
        audit_saved_attempts(primary, output)
