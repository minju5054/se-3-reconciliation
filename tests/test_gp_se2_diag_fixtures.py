"""Independent synthetic diagnostics; no optimization or performance claims."""
import numpy as np
import pytest

from reconciliation.gp_se2_diag_fixtures import (
    BOUNDARY, PERTURBATION_SEED, closed_form_pose, evaluate_blind_spot,
    evaluate_fixture, independent_pose_derivatives, make_fixture_problem,
    truth_acceleration, truth_twist, write_optimizer_free_fixtures,
)


@pytest.fixture(scope="module", params=["S0", "S1", "S2", "S3"])
def evaluated(request):
    return evaluate_fixture(request.param)


def test_full_size_support_reproduction_and_nontrivial_boundary(evaluated):
    f, s = evaluated["fixture_input"], evaluated["summary"]
    assert len(f["support_times"]) == 31
    assert f["support_times"][-1] == 3
    assert f["variable_count"] == 150
    assert np.linalg.norm(BOUNDARY[:2]) > 0 and BOUNDARY[2] != 0
    assert s["support_pose_max_absolute_error"] < 1e-12
    assert s["support_twist_max_absolute_error"] < 1e-11
    assert s["ode_vs_tighter_position_error_max_m"] < 2e-11
    assert s["ode_vs_tighter_yaw_error_max_rad"] < 2e-12


def test_gp_returned_derivatives_against_pose_only_world_fd(evaluated):
    s = evaluated["summary"]
    assert np.max(s["independent_interior_twist_error_max_by_component"]) < 3e-8
    assert np.max(s["independent_interior_acceleration_error_max_by_component"]) < 1e-8
    assert np.max(s["independent_one_sided_twist_error_max_by_component"]) < 5e-8
    assert np.max(s["independent_one_sided_acceleration_error_max_by_component"]) < 5e-8


def test_known_motion_family_reconstructed_with_original_acceptance(evaluated):
    s = evaluated["summary"]
    if s["fixture"] == "S3":
        pytest.skip("S3 diagnoses representation error, not exact reproduction")
    assert s["known_seed_original_dense_feasible"]
    assert s["position_error_max_m"] < 2e-12
    assert s["yaw_error_max_rad"] < 2e-12
    assert s["lateral_velocity_max_m_s"] < 2e-11
    assert s["known_analytic_gp_residual_error_max"] < 2e-12
    assert np.max(s["body_acceleration_error_max_by_component"]) < 5e-11


def test_s2_time_units_and_nonzero_prior_residual():
    t = np.array([0, 1.5, 3.])
    np.testing.assert_allclose(truth_twist("S2", t), [[.3, 0, .4], [.15, 0, .2], [0, 0, 0]])
    np.testing.assert_allclose(truth_acceleration("S2", t), np.tile([-.1, 0, -.4/3], (3, 1)))
    report = evaluate_fixture("S2")
    assert np.max(np.abs(report["gp_residuals"])) > 1e-3
    assert report["fixture_input"]["q_units"] == "s"
    assert report["fixture_input"]["q_dot_units"] == "dimensionless"
    assert report["fixture_input"]["q_ddot_units"] == "1/s"


def test_s3_sampled_unicycle_and_gp_reconstruction_are_distinct():
    r = evaluate_fixture("S3")
    s = r["summary"]
    assert s["position_error_max_m"] > 100*s["ode_vs_tighter_position_error_max_m"]
    assert 1e-11 < s["lateral_velocity_max_m_s"] < s["equality_tolerance_m_s"]
    assert np.max(s["body_acceleration_error_max_by_component"]) > 1e-8
    assert s["known_seed_original_dense_feasible"]
    np.testing.assert_array_equal(r["truth_body_twists"][:, 1], 0)


@pytest.mark.parametrize("name", ["S0", "S1", "S2"])
def test_deterministic_perturbation_preserves_fixed_state_and_requires_recovery(name):
    problem, f = make_fixture_problem(name)
    problem2, f2 = make_fixture_problem(name)
    assert f["perturbation_seed"] == PERTURBATION_SEED
    np.testing.assert_array_equal(f["perturbation_vector"], f2["perturbation_vector"])
    assert f["perturbation_vector"].shape == (150,)
    assert problem.dense_report(f["known_vector"])["feasible"]
    assert not problem.dense_report(f["perturbed_vector"])["feasible"]
    poses, twists = problem.unpack(f["perturbed_vector"])
    np.testing.assert_array_equal(poses[0], f["boundary_pose"])
    np.testing.assert_array_equal(twists[0], f["initial_twist"])
    assert problem.config == problem2.config
    assert problem.config["equality_tolerance"] == 1e-5
    assert problem.config["a_v_max"] == 2


def test_blind_spot_endpoint_midpoint_pass_but_quarters_dense_fail():
    r = evaluate_blind_spot()
    s = r["summary"]
    assert s["endpoint_midpoint_lateral_pass"]
    assert s["quarter_point_lateral_fail"]
    assert not s["dense_lateral_pass"]
    assert s["maximum_violation_m_s"] > 1e-5
    assert s["independent_pose_derivative_velocity_error_max"] < 1e-8
    assert s["independent_pose_derivative_acceleration_error_max"] < 1e-5
    np.testing.assert_array_equal(s["prescribed_point_pass"], [True, False, True, False, True])
    i = np.argmax(np.abs(r["body_twists"][:, 1]))
    assert s["maximum_violation_time_s"] == r["times_s"][i]
    assert s["maximum_absolute_lateral_velocity_m_s"] == abs(r["body_twists"][i, 1])


@pytest.mark.parametrize("stencil,t", [((-2,-1,0,1,2), .4), ((0,1,2,3,4), 0.), ((-4,-3,-2,-1,0), 3.)])
def test_independent_fd_matches_analytic_world_solution_interior_and_sides(stencil, t):
    v, a = independent_pose_derivatives(lambda q: closed_form_pose("S2", q), t, offsets=stencil)
    np.testing.assert_allclose(v, truth_twist("S2", t), atol=2e-10)
    np.testing.assert_allclose(a, truth_acceleration("S2", t), atol=1e-6)


def test_fixture_writer_refuses_existing_output_before_work(tmp_path):
    with pytest.raises(FileExistsError):
        write_optimizer_free_fixtures(tmp_path)


def test_reduced_problem_is_not_silently_substituted():
    with pytest.raises(ValueError, match="actual 3 s"):
        make_fixture_problem("S0", {"horizon_s": .5})
