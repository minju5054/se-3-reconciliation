import numpy as np
import pytest

from reconciliation.graph_metrics import correction_profile, evaluate_graph_state, trajectory_error
from reconciliation.graph_optimizer import (
    OptimizationError,
    SolverConfig,
    numerical_jacobian,
    solve_graph,
)
from reconciliation.se2 import compose_poses, relative_pose
from reconciliation.se2_graph import (
    GraphProblem,
    GraphWeights,
    OracleCorrespondence,
    ResidualScales,
    boundary_residual,
    correspondence_residual,
    graph_cost,
    new_motion_residual,
    validate_correspondences,
)


def curved_ground_truth(count: int = 9) -> np.ndarray:
    poses = [np.zeros(3)]
    step = np.array([0.22, 0.0, 0.045])
    for _ in range(count - 1):
        poses.append(compose_poses(poses[-1], step))
    return np.asarray(poses)


def known_problem(count: int = 9):
    ground_truth = curved_ground_truth(count)
    old = ground_truth[:5]
    expected = ground_truth[2:]
    perturbation = np.array([0.20, 0.15, 0.10])
    raw = compose_poses(perturbation, expected)
    correspondences = tuple(
        OracleCorrespondence(i, i - 2, [0.0, 0.0, 0.0], "same synthetic pose")
        for i in (2, 3, 4)
    )
    problem = GraphProblem(
        fixed_old=old,
        raw_new=raw,
        fixed_boundary=expected[0],
        correspondences=correspondences,
        scales=ResidualScales(0.1, 0.1),
        weights=GraphWeights(4.0, 1.0, 1.0),
    )
    return problem, expected, perturbation


def test_factor_zero_cases_and_correspondence_direction() -> None:
    old = np.array([1.0, 2.0, 0.3])
    measurement = np.array([0.4, -0.1, 0.2])
    new = compose_poses(old, measurement)
    assert boundary_residual(old, old) == pytest.approx(np.zeros(3), abs=1e-12)
    assert correspondence_residual(old, new, measurement) == pytest.approx(
        np.zeros(3), abs=1e-12
    )
    wrong_direction = relative_pose(new, old)
    assert np.linalg.norm(correspondence_residual(old, new, wrong_direction)) > 0.1
    assert new_motion_residual(old, new, old, new) == pytest.approx(np.zeros(3), abs=1e-12)


def test_arbitrary_horizon_and_monotonic_correspondence_validation() -> None:
    problem, expected, _ = known_problem(count=13)
    assert problem.raw_new.shape == (11, 3)
    assert graph_cost(problem, expected) == pytest.approx(0.0, abs=1e-20)
    with pytest.raises(ValueError, match="monotonic"):
        validate_correspondences(
            (
                OracleCorrespondence(1, 2, [0, 0, 0]),
                OracleCorrespondence(2, 1, [0, 0, 0]),
            ),
            old_count=4,
            new_count=4,
        )
    with pytest.raises(ValueError, match="bounds"):
        validate_correspondences(
            (OracleCorrespondence(5, 0, [0, 0, 0]),), old_count=4, new_count=4
        )


def test_known_synthetic_recovery_and_downstream_propagation() -> None:
    problem, expected, _ = known_problem()
    boundary_copy = problem.fixed_boundary.copy()
    initial_cost = graph_cost(problem, problem.raw_new)
    result = solve_graph(problem, SolverConfig())
    error = trajectory_error(expected, result.optimized)
    evaluated = evaluate_graph_state(problem, result.optimized)
    profile = correction_profile(problem.raw_new, result.optimized)
    assert result.converged
    assert result.final_cost < initial_cost * 1e-12
    assert error["translation_max_m"] < 1e-7
    assert error["yaw_max_rad"] < 1e-7
    assert evaluated["correspondence"]["translation_rms_m"] < 1e-8
    assert evaluated["new_motion_distortion"]["translation_rms_m"] < 1e-8
    assert np.array_equal(problem.fixed_boundary, boundary_copy)
    assert profile[-1]["translation_correction_m"] > 0.1


def test_solver_is_deterministic_and_cost_history_decreases() -> None:
    problem, _, _ = known_problem()
    first = solve_graph(problem, SolverConfig())
    second = solve_graph(problem, SolverConfig())
    assert np.array_equal(first.optimized, second.optimized)
    assert all(a > b for a, b in zip(first.cost_history, first.cost_history[1:]))


def test_numerical_jacobian_and_invalid_solver_inputs_fail_explicitly() -> None:
    problem, _, _ = known_problem()
    jacobian = numerical_jacobian(lambda x: x.reshape(-1), problem.raw_new, 1e-6)
    assert jacobian.shape == (problem.raw_new.size, problem.raw_new.size)
    with pytest.raises(ValueError, match="positive"):
        SolverConfig(finite_difference_epsilon=0.0)
    with pytest.raises(FloatingPointError, match="finite"):
        numerical_jacobian(
            lambda _: np.array([np.nan]), problem.raw_new, 1e-6
        )


def test_singular_linear_solve_failure_is_explicit(monkeypatch) -> None:
    problem, _, _ = known_problem()

    def singular(*_args, **_kwargs):
        raise np.linalg.LinAlgError("fixture singular")

    monkeypatch.setattr(np.linalg, "solve", singular)
    with pytest.raises(OptimizationError, match="singular"):
        solve_graph(problem, SolverConfig())
