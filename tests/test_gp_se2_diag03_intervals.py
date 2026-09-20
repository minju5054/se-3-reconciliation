"""Synthetic checker tests, not hard-handoff experimental evidence."""
import numpy as np
import pytest

from reconciliation.gp_se2_diag03_intervals import (
    audit_intervals, constraint_time_map, motion_definitions,
    refine_transition, sample_interval_grid,
)
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.se2 import compose_poses, se2_exp


def constant_problem(twist=(.3, 0., .4), horizon=.3, obstacles=False, gates=None):
    times = np.linspace(0., horizon, round(horizon/.1)+1)
    boundary = np.array([2.3, -1.2, .7])
    twist = np.asarray(twist)
    poses = compose_poses(boundary, se2_exp(times[:, None]*twist))
    velocity = np.tile(twist, (len(times), 1))
    problem = GPProblem(boundary, twist, poses[1:], poses[-1], {"horizon_s": horizon},
                        workspace_margin=lambda xy: np.full(xy.shape[:-1], 2.),
                        obstacle_clearance=lambda xy: np.full(xy.shape[:-1], .3),
                        include_obstacles=obstacles, gates=gates)
    return problem, problem.vector(poses, velocity)


@pytest.mark.parametrize("obstacles,inequalities", [(False, 813), (True, 903)])
def test_row_map_matches_original_values_and_order(obstacles, inequalities):
    problem, vector = constant_problem(horizon=3., obstacles=obstacles)
    result = constraint_time_map(problem, vector)
    assert result["equality_count"] == 30
    assert result["inequality_count"] == inequalities
    assert max(result["comparison_maximum_absolute_error"].values()) < 1e-12
    inequalities = [r for r in result["rows"] if r["constraint_kind"] == "inequality"]
    assert inequalities[0]["family"] == "linear_speed_lower"
    assert inequalities[89]["interval_index"] == 29
    assert inequalities[89]["local_fraction"] == 1.
    assert inequalities[90]["family"] == "linear_speed_upper"
    assert inequalities[720]["physical_unit"] == "m^2"
    assert inequalities[720]["actual_numerical_tolerance"] == problem.config["inequality_tolerance"]
    assert inequalities[723]["family"] == "workspace"
    assert inequalities[0]["knot_side"] == "right"
    assert inequalities[2]["knot_side"] == "left"
    assert result["support_lateral_zero"]["explicit_constraint_rows"] == 0


def test_row_map_retains_gate_order_and_unknown_environment_penalty():
    gate = dict(center_xy=[2.33, -1.16], normal_xy=[1., 0.], half_width_m=.4,
                time_s=.15, crossing_offset_s=.02, minimum_progress_m=1e-4)
    problem, vector = constant_problem(gates=[gate], obstacles=True)
    problem.workspace_margin = lambda xy: np.full(xy.shape[:-1], np.nan)
    result = constraint_time_map(problem, vector)
    eq = [r for r in result["rows"] if r["constraint_kind"] == "equality"]
    assert eq[-1]["family"] == "gate_plane"
    assert eq[-1]["trajectory_time_s"] == .15
    workspace = [r for r in result["rows"] if r["family"] == "workspace"]
    assert all(r["nonfinite_query_mapped"] and r["actual_value"] == -1e6 for r in workspace)
    assert max(result["comparison_maximum_absolute_error"].values()) < 1e-12


@pytest.mark.parametrize("twist", [(.3, 0., 0.), (.3, 0., .4), (0., 0., .4)])
def test_constant_twist_grid_and_no_solve(monkeypatch, twist):
    import reconciliation.gp_se2_formulation as formulation
    def forbidden(*args, **kwargs):
        raise AssertionError("a stored-curve audit must not optimize")
    monkeypatch.setattr(formulation, "minimize", forbidden)
    problem, vector = constant_problem(twist)
    before = vector.copy()
    result = audit_intervals(problem, vector)
    np.testing.assert_array_equal(before, vector)
    assert result["optimizer_calls"] == 0
    assert not result["primary_acceptance_modified"]
    assert not result["continuous_time_feasibility_proven"]
    assert len(result["interval_extrema"]) == 6*6*3
    assert np.max(np.abs(result["finest_trace"]["body_twists"]-np.asarray(twist))) < 1e-11
    assert all(r["passed"] for r in result["grid_detection_comparison"])
    assert result["diagnostic_witnesses"] == []


def test_midpoint_pass_interior_fail_fixture_and_detection_only():
    problem = GPProblem([0, 0, 0], [.3, 0, .08], [[.03, 0, 0]], [.03, 0, 0],
                        {"horizon_s": .1}, include_obstacles=False)
    vector = problem.vector(np.array([[0, 0, 0], [.03, 0, 0]]),
                            np.array([[.3, 0, .08], [.3, 0, .08]]))
    before = problem.dense_report(vector)
    result = audit_intervals(problem, vector)
    after = problem.dense_report(vector)
    assert before["feasible"] == after["feasible"] == False
    assert before["maximum_equality_residual"] == after["maximum_equality_residual"]
    lateral = {r["grid"]: r for r in result["grid_detection_comparison"] if r["quantity"] == "vy"}
    assert lateral["G0_ORIGINAL"]["passed"]
    assert not lateral["G1_QUARTERS"]["passed"]
    assert not lateral["G2_WITNESS"]["passed"]
    assert lateral["G1_QUARTERS"]["additional_lateral_equalities"] == 2
    assert lateral["G1_QUARTERS"]["additional_motion_evaluations"] == 2
    assert all(r["detection_only"] and not r["constraints_actually_added"] for r in lateral.values())
    brackets = [r for r in result["violation_brackets"] if r["quantity"] == "vy"]
    assert brackets
    assert all(r["bracket_width_s"] <= 1e-4 for r in brackets)
    assert all(r["interval_index"] == 0 for r in brackets)
    assert all(0 <= r["bracket_s"][0] <= r["bracket_s"][1] <= .1 for r in brackets)


def test_left_right_acceleration_preserved_and_ay_has_no_acceptance_limit():
    problem, vector = constant_problem()
    vector[3] += .2
    trace = sample_interval_grid(problem, vector, np.array([0., .1, .2, .3]))
    at_knot = np.flatnonzero(np.isclose(trace["times_s"], .1))
    assert len(at_knot) == 2
    assert {trace["knot_side"][i] for i in at_knot} == {"left", "right"}
    assert np.max(np.abs(trace["body_accelerations"][at_knot[0]]-trace["body_accelerations"][at_knot[1]])) > 1
    result = audit_intervals(problem, vector)
    assert len(result["knot_limits"]) == 2
    assert "ay" not in {r["quantity"] for r in result["grid_detection_comparison"]}
    ay = [r for r in result["interval_extrema"] if r["quantity"] == "ay"]
    assert all(r["maximum_tolerance_excess"] is None and not r["enforced"] for r in ay)


def test_units_and_tolerance_excess_are_not_a_new_acceptance():
    defs = motion_definitions(constant_problem()[0].config)
    assert defs["vy"]["lower"] == defs["vy"]["upper"] == 0
    assert defs["vy"]["unit"] == "m/s"
    assert defs["alpha"]["unit"] == "rad/s^2"
    assert defs["ay"]["tolerance"] is None
    bracket = refine_transition(lambda t: t-.12345, .12, .13)
    assert bracket["bracket_s"][0] <= .12345 <= bracket["bracket_s"][1]
    assert bracket["bracket_width_s"] <= 1e-4
    assert not bracket["continuous_root_uniqueness_proven"]
    with pytest.raises(ValueError, match="safe/violating"):
        refine_transition(lambda t: 1e-5-(t-.5)**2, 0., 1.)


def test_missing_or_nonfinite_vector_and_out_of_domain_queries_are_not_zeroed():
    problem, vector = constant_problem()
    with pytest.raises(ValueError, match="saved vector"):
        audit_intervals(problem, None)
    vector[0] = np.nan
    with pytest.raises(ValueError, match="saved vector"):
        audit_intervals(problem, vector)
    problem, vector = constant_problem()
    with pytest.raises(ValueError, match="support domain"):
        sample_interval_grid(problem, vector, [-.01])
