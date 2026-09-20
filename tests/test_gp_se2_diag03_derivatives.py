"""Synthetic derivative-checker tests, never actual handoff evidence."""
from __future__ import annotations

import json

import numpy as np
import pytest

import reconciliation.gp_se2_diag03_derivatives as audit
from reconciliation.gp_se2 import interpolate_interval
from reconciliation.se2 import compose_poses, se2_exp


def constant_supports(twist, yaw=0.7):
    times = np.arange(4)*0.1
    origin = np.array([12., -23., yaw])
    v = np.broadcast_to(twist, (len(times), 3)).copy()
    poses = compose_poses(origin, se2_exp(times[:, None]*v))
    return times, poses, v


@pytest.mark.parametrize("twist", [[0.5, 0., 0.], [0.5, 0., 0.8],
                                   [0., 0., 1.2], [0.4, 0., 1e-9]])
def test_constant_straight_turning_rotation_and_small_angle(twist):
    t, p, v = constant_supports(twist)
    saved = [x.copy() for x in (t, p, v)]
    checked = audit.check_interpolation_consistency(t, p, v)
    assert checked["summary"]["coefficient_cross_check_consistent"]
    assert checked["summary"]["interpolation_derivative_consistent"]
    assert checked["summary"]["fixed_interior_probe_count"] == 15
    for record in checked["derivative_checks"]:
        np.testing.assert_allclose(record["analytic_twist"], twist, atol=1e-10)
        np.testing.assert_allclose(record["analytic_acceleration"], 0., atol=1e-9)
    for original, after in zip(saved, (t, p, v)):
        np.testing.assert_array_equal(original, after)
    json.dumps(checked, allow_nan=False)


@pytest.mark.parametrize("omega", [0., 0.0000999, 0.0001001, 0.0009999, 0.0010001, -0.3, 0.7])
def test_nonzero_local_chart_coefficients_match_both_small_angle_branches(omega):
    p0 = np.array([7.3, -9.1, -2.7])
    p1 = compose_poses(p0, se2_exp([0.07, 0.002, omega]))
    v0 = np.array([0.7, 0., omega*10+0.03])
    v1 = np.array([0.55, 0., omega*10-0.02])
    u = np.array([0., 0.125, 0.25, 0.5, 0.75, 0.875, 1.])
    c = audit.hermite_coefficients(p0, v0, p1, v1, 0.1)
    independent = audit.coefficient_interpolation(p0, c, 0.1, u)
    production = interpolate_interval(p0, v0, p1, v1, 0.1, u)
    for a, b in zip(independent, production):
        np.testing.assert_allclose(a, b, atol=1e-8, rtol=0)
    np.testing.assert_allclose(c[0], 0.1*v0, atol=0, rtol=0)


def test_coefficient_evaluation_does_not_reuse_production_interpolator(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Production interpolator must not be used on independent side")
    monkeypatch.setattr(audit, "interpolate_interval", forbidden)
    t, p, v = constant_supports([0.3, 0., 0.7])
    c = audit.hermite_coefficients(p[0], v[0], p[1], v[1], 0.1)
    _, body_twist, _ = audit.coefficient_interpolation(p[0], c, 0.1, 0.4)
    np.testing.assert_allclose(body_twist, v[0], atol=1e-11)


def test_time_finite_difference_does_not_use_analytic_jacobian_derivative(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("FD derivative cannot depend on analytic D Jr")
    monkeypatch.setattr(audit, "right_jacobian_directional", forbidden)
    t, p, v = constant_supports([0.3, 0., 0.7])
    c = audit.hermite_coefficients(p[0], v[0], p[1], v[1], 0.1)
    fd = audit.finite_difference_at(p[0], c, 0.1, 0.5, 1e-5)
    assert fd["available"]
    np.testing.assert_allclose(fd["twist_fd"], v[0], atol=1e-8)
    np.testing.assert_allclose(fd["acceleration_fd"], 0, atol=1e-8)


def test_wrapped_world_yaw_boundary_is_not_false_derivative_failure():
    t, p, v = constant_supports([0.5, 0., 0.8], yaw=np.pi-0.04)
    assert p[0, 2] > 0 and p[1, 2] < 0
    checked = audit.check_interpolation_consistency(t, p, v)
    assert checked["summary"]["interpolation_derivative_consistent"]
    assert checked["summary"]["coefficient_cross_check_consistent"]


@pytest.mark.parametrize("fraction", [0., 1., 0.0001, 0.9999])
def test_no_finite_difference_touches_or_crosses_knots(fraction):
    fd = audit.finite_difference_at([0, 0, 0], np.zeros((3, 3)), 0.1, fraction, 1e-4)
    assert not fd["available"]
    assert fd["reason"] == "PROBE_WOULD_TOUCH_OR_CROSS_KNOT"
    assert "twist_fd" not in fd


def test_knot_side_acceleration_is_kept_separately_and_witness_is_guarded():
    t = np.array([0., 0.1, 0.2])
    p = np.array([[0., 0., 0.], [0.05, 0., 0.], [0.13, 0., 0.]])
    v = np.array([[0.4, 0., 0.], [0.5, 0., 0.], [0.6, 0., 0.]])
    checked = audit.check_interpolation_consistency(t, p, v,
        witness_points=[{"interval_index": 0, "local_fraction": 1., "label": "left_knot"},
                        {"interval_index": 1, "local_fraction": 0., "label": "right_knot"}])
    knot = checked["knot_limits"][0]
    assert knot["time_s"] == 0.1
    np.testing.assert_allclose(knot["pose_periodic_absolute_difference"], 0., atol=1e-12)
    np.testing.assert_allclose(knot["twist_absolute_difference"], 0., atol=1e-12)
    assert abs(knot["right_minus_left_acceleration"][0]) > 1.
    assert not knot["finite_difference_across_knot_performed"]
    assert checked["summary"]["unverified_boundary_or_branch_probe_count"] == 2
    assert checked["summary"]["unverified_fixed_probe_count"] == 0
    assert checked["summary"]["interpolation_derivative_consistent"]


def test_post_hoc_witness_preserves_source_and_deduplicates_fraction():
    t, p, v = constant_supports([0.4, 0, 0.2])
    checked = audit.check_interpolation_consistency(t, p, v, witness_points=[
        {"interval_index": 1, "local_fraction": 0.5, "label": "lateral"},
        {"interval_index": 1, "local_fraction": 0.5, "label": "speed"}])
    row = [r for r in checked["derivative_checks"] if r["interval_index"] == 1 and r["local_fraction"] == 0.5][0]
    assert row["probe_sources"] == ["FIXED_INTERIOR", "POST_HOC_WITNESS:lateral", "POST_HOC_WITNESS:speed"]
    assert checked["summary"]["total_unique_derivative_probes"] == 15


def test_error_trend_and_units_stay_separate_from_physical_acceptance():
    t, p, v = constant_supports([0.8, 0., 2.0])
    checked = audit.check_interpolation_consistency(t, p, v)
    assert checked["protocol"]["units"]["body_twist"] == ["m/s", "m/s", "rad/s"]
    assert checked["protocol"]["units"]["body_acceleration"] == ["m/s^2", "m/s^2", "rad/s^2"]
    assert not checked["protocol"]["physical_acceptance_modified"]
    assert not checked["summary"]["new_optimization"]
    row = checked["derivative_checks"][0]
    assert [r["epsilon_s"] for r in row["epsilon_checks"]] == [1e-4, 3e-5, 1e-5]
    assert all(r["available"] for r in row["epsilon_checks"])
    assert row["epsilon_checks"][-1]["twist_absolute_error"][0] < row["epsilon_checks"][0]["twist_absolute_error"][0]


def test_protocol_copy_is_independent_and_tolerances_are_fixed():
    protocol = audit.derivative_protocol()
    protocol["tolerances"]["coefficient_pose_absolute"][0] = 100.
    assert audit.derivative_protocol()["tolerances"]["coefficient_pose_absolute"][0] == 1e-9
    assert audit.DERIVATIVE_PROTOCOL["tolerances"]["finite_difference_acceleration_absolute"] == [2e-5]*3


@pytest.mark.parametrize("witness", [{"interval_index": -1, "local_fraction": 0.5},
                                      {"interval_index": 0, "local_fraction": -0.1},
                                      {"interval_index": 0.3, "local_fraction": 0.5},
                                      {"interval_index": 3, "local_fraction": 0.5}])
def test_invalid_witness_is_not_silently_shifted(witness):
    t, p, v = constant_supports([0.4, 0, 0.2])
    with pytest.raises(ValueError, match="witness"):
        audit.check_interpolation_consistency(t, p, v, witness_points=[witness])


def test_checker_detects_analytic_acceleration_mismatch_without_changing_tolerance(monkeypatch):
    original = audit.interpolate_interval
    def incorrect_acceleration(*args, **kwargs):
        pose, twist, acceleration = original(*args, **kwargs)
        return pose, twist, acceleration+np.array([1e-3, 0, 0])
    monkeypatch.setattr(audit, "interpolate_interval", incorrect_acceleration)
    t, p, v = constant_supports([0.4, 0, 0.2])
    checked = audit.check_interpolation_consistency(t, p, v)
    assert not checked["summary"]["coefficient_cross_check_consistent"]
    assert not checked["summary"]["interpolation_derivative_consistent"]
    assert checked["summary"]["maximum_fd_errors_all_epsilons"]["acceleration"][0] > 9e-4
