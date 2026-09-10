from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.controllers.trajectory_follower import FollowerConfig, TrajectoryFollower
from reconciliation.exp02b import load_source_case, transition_input
from reconciliation.exp02c_factor_isolation import synthetic_conditions
from reconciliation.exp02d_lookahead_direction import (
    HISTORICAL_ENTRY_DIRECTION_UNDEFINED,
    INCOMING_DIRECTION_UNDEFINED,
    INPUT_RECONSTRUCTION_FAILED,
    LOOKAHEAD_DIRECTION_UNDEFINED,
    METHOD_FACTORS,
    GeometryUndefinedError,
    InputReconstructionError,
    candidate_command_metrics,
    candidate_deformation_metrics,
    lookahead_direction_residual,
    method_residual_vector,
    physical_method_residuals,
    problem_from_config,
    reconstruct_raw_follower_decision,
    solve_method,
    transition_geometry_metrics,
    undefined_geometry_statuses,
    validate_reconstructed_first_command,
)
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput
from reconciliation.transition_graph import (
    problem_from_config as historical_problem_from_config,
    transition_residual_vector,
)


ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_VALUES = {
    "lookahead_distance_m": 0.25,
    "position_gain": 1.0,
    "heading_gain": 1.5,
    "cross_track_gain": 0.8,
    "max_linear_velocity_mps": 1.0,
    "max_angular_velocity_rps": 2.0,
    "goal_position_tolerance_m": 0.02,
    "goal_yaw_tolerance_rad": 0.05,
    "rotate_in_place_threshold_rad": 1.2,
    "nearest_search_window": 10,
}


def graph_config():
    with (ROOT / "configs/exp02a_spatial_entry.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def make_inputs(*, fresh=None, k=0, previous=(-0.2, 0.0, 0.0), boundary=(0, 0, 0)):
    if fresh is None:
        fresh = np.asarray(
            [
                [0.10, 0.05, 0.02],
                [0.20, 0.06, 0.04],
                [0.32, 0.07, 0.06],
                [0.46, 0.08, 0.08],
                [0.61, 0.09, 0.10],
                [0.77, 0.10, 0.12],
                [0.94, 0.11, 0.14],
            ],
            dtype=np.float64,
        )
    return TransitionReconciliationInput(
        old_poses_world=np.asarray([[-0.5, 0.0, 0.0], [-0.3, 0.0, 0.0]]),
        fresh_poses_world=np.asarray(fresh, dtype=np.float64),
        committed_pose_world=np.asarray(boundary, dtype=np.float64),
        actual_pose_before_committed=np.asarray(previous, dtype=np.float64),
        entry_context=SpatialEntryContext(entry_index=k, source="exp02d_test"),
    )


def make_problem(*, inputs=None, q=3):
    config = graph_config()
    return problem_from_config(
        make_inputs() if inputs is None else inputs,
        q,
        residual_scales=config["residual_scales"],
        weights=config["variants"]["incoming_motion_aware"],
        minimum_translation_m=config["minimum_direction_translation_m"],
    )


def test_lookahead_residual_zero_antiparallel_and_perpendicular():
    kwargs = {
        "previous": [-1.0, 0.0, 0.0],
        "boundary": [0.0, 0.0, 0.0],
        "raw_lookahead": [2.0, 0.0, 0.0],
        "minimum_translation_m": 1e-6,
    }
    assert lookahead_direction_residual(
        lookahead_candidate=[2.0, 0.0, 0.0], **kwargs
    ) == pytest.approx([0.0, 0.0])
    assert lookahead_direction_residual(
        lookahead_candidate=[-2.0, 0.0, 0.0], **kwargs
    ) == pytest.approx([-2.0, 0.0])
    assert lookahead_direction_residual(
        lookahead_candidate=[0.0, 2.0, 0.0], **kwargs
    ) == pytest.approx([-1.0, 1.0])


def test_lookahead_residual_is_finite_near_boundary_and_uses_fixed_raw_radius():
    near = lookahead_direction_residual(
        [-1, 0, 0],
        [0, 0, 0],
        [1e-15, -1e-15, 0],
        [1, 0, 0],
        minimum_translation_m=1e-6,
    )
    assert np.all(np.isfinite(near))
    assert near == pytest.approx([-1.0, -1e-15])
    doubled = lookahead_direction_residual(
        [-1, 0, 0],
        [0, 0, 0],
        [2, 0, 0],
        [1, 0, 0],
        minimum_translation_m=1e-6,
    )
    assert doubled == pytest.approx([1.0, 0.0])


@pytest.mark.parametrize("count", [2, 3, 11])
def test_lookahead_problem_supports_arbitrary_horizon(count):
    fresh = np.column_stack(
        (
            np.linspace(0.2, 1.0, count),
            np.linspace(0.0, 0.2, count),
            np.linspace(0.0, 0.3, count),
        )
    )
    problem = make_problem(inputs=make_inputs(fresh=fresh), q=count - 1)
    residual = method_residual_vector(problem, problem.raw, "M3_LOOKAHEAD")
    assert residual.shape == (3 + 3 * (count - 1) + 2 + 1,)
    assert np.all(np.isfinite(residual))


def test_q_relative_maps_direction_to_absolute_raw_q():
    inputs = make_inputs(k=2)
    problem = make_problem(inputs=inputs, q=5)
    assert problem.q_relative == 3
    baseline = problem.raw
    changed_wrong_node = baseline.copy()
    changed_wrong_node[2, 1] += 0.4
    changed_q_node = baseline.copy()
    changed_q_node[3, 1] += 0.4
    initial = physical_method_residuals(problem, baseline, "M3_LOOKAHEAD")[
        "lookahead_direction"
    ]
    wrong = physical_method_residuals(problem, changed_wrong_node, "M3_LOOKAHEAD")[
        "lookahead_direction"
    ]
    at_q = physical_method_residuals(problem, changed_q_node, "M3_LOOKAHEAD")[
        "lookahead_direction"
    ]
    assert wrong == pytest.approx(initial)
    assert not np.allclose(at_q, initial)


def test_q_equal_k_makes_m3_vector_identical_to_historical_m1():
    problem = make_problem(q=0)
    state = problem.raw
    state[0] += [0.03, -0.01, 0.08]
    state[-1] += [0.02, 0.04, -0.03]
    assert np.array_equal(
        method_residual_vector(problem, state, "M3_LOOKAHEAD"),
        method_residual_vector(problem, state, "M1_HISTORICAL_M4"),
    )


def test_m2_has_no_direction_and_retains_entry_yaw_and_fresh_motion():
    problem = make_problem()
    assert METHOD_FACTORS["M2_NO_DIRECTION"] == (
        "entry_preservation",
        "fresh_motion",
        "incoming_yaw",
    )
    state = problem.raw
    state[0, 0] += 0.03
    state[0, 2] += 0.2
    groups = physical_method_residuals(problem, state, "M2_NO_DIRECTION")
    assert tuple(groups) == METHOD_FACTORS["M2_NO_DIRECTION"]
    assert all(np.linalg.norm(groups[name]) > 0.0 for name in groups)


def test_historical_m1_residual_and_output_are_production_identical():
    config = graph_config()
    problem = make_problem()
    production = historical_problem_from_config(
        problem.inputs,
        "incoming_motion_aware",
        residual_scales=config["residual_scales"],
        weights=config["variants"]["incoming_motion_aware"],
        minimum_translation_m=config["minimum_direction_translation_m"],
    )
    perturbed = problem.raw
    perturbed[0] += [0.02, -0.03, 0.07]
    perturbed[-1] += [-0.04, 0.01, -0.02]
    assert np.array_equal(
        method_residual_vector(problem, perturbed, "M1_HISTORICAL_M4"),
        transition_residual_vector(production, perturbed),
    )
    solver = SolverConfig(**config["solver"])
    expected = solve_least_squares(
        production.raw_new,
        lambda candidate: transition_residual_vector(production, candidate),
        solver,
    )
    actual = solve_method(problem, "M1_HISTORICAL_M4", solver)
    assert np.array_equal(actual.candidate, expected.optimized)


def test_exp02c_benign_historical_m4_still_matches_frozen_candidate():
    with (ROOT / "configs/exp02b_controller_aware.yaml").open(encoding="utf-8") as stream:
        exp02b = yaml.safe_load(stream)
    source = ROOT / exp02b["paths"]["source_root"] / exp02b["frozen_source_cases"][
        "case_benign_delayed"
    ]["relative_trial_path"]
    inputs = transition_input(load_source_case("case_benign_delayed", source), 0)
    problem = make_problem(inputs=inputs, q=0)
    result = solve_method(
        problem,
        "M1_HISTORICAL_M4",
        SolverConfig(**graph_config()["solver"]),
    )
    frozen = np.load(
        ROOT
        / "data/exp02b/exp02b-controller-aware-20260906T150400Z"
        / "case_benign_delayed/k_0/graph/candidate.npy",
        allow_pickle=False,
    )
    difference = result.candidate - frozen
    assert np.max(np.linalg.norm(difference[:, :2], axis=1)) <= 1e-9
    assert np.max(np.abs((difference[:, 2] + np.pi) % (2 * np.pi) - np.pi)) <= 1e-9


def test_named_undefined_geometry_rejections_and_statuses():
    incoming_problem = make_problem(
        inputs=make_inputs(previous=(0, 0, 0), boundary=(0, 0, 0)), q=3
    )
    assert undefined_geometry_statuses(incoming_problem) == (
        INCOMING_DIRECTION_UNDEFINED,
    )
    with pytest.raises(GeometryUndefinedError) as incoming_error:
        method_residual_vector(incoming_problem, incoming_problem.raw, "M3_LOOKAHEAD")
    assert incoming_error.value.status == INCOMING_DIRECTION_UNDEFINED

    fresh = make_inputs().fresh_poses_world.copy()
    fresh[3, :2] = 0.0
    lookahead_problem = make_problem(inputs=make_inputs(fresh=fresh), q=3)
    assert undefined_geometry_statuses(lookahead_problem) == (
        LOOKAHEAD_DIRECTION_UNDEFINED,
    )
    with pytest.raises(GeometryUndefinedError) as lookahead_error:
        method_residual_vector(lookahead_problem, lookahead_problem.raw, "M3_LOOKAHEAD")
    assert lookahead_error.value.status == LOOKAHEAD_DIRECTION_UNDEFINED

    entry_zero = make_inputs().fresh_poses_world.copy()
    entry_zero[0, :2] = 0.0
    historical_problem = make_problem(inputs=make_inputs(fresh=entry_zero), q=3)
    with pytest.raises(GeometryUndefinedError) as historical_error:
        method_residual_vector(
            historical_problem, historical_problem.raw, "M1_HISTORICAL_M4"
        )
    assert historical_error.value.status == HISTORICAL_ENTRY_DIRECTION_UNDEFINED


def test_invalid_q_and_nonfinite_state_are_rejected():
    with pytest.raises(ValueError, match="k <= q"):
        make_problem(q=99)
    with pytest.raises(ValueError, match="k <= q"):
        make_problem(inputs=make_inputs(k=2), q=1)
    problem = make_problem()
    invalid = problem.raw
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        method_residual_vector(problem, invalid, "M3_LOOKAHEAD")


def test_raw_follower_oracle_is_one_exact_frozen_decision_with_bounded_k_q():
    fresh = np.column_stack(
        (
            np.arange(8, dtype=np.float64) * 0.1,
            np.zeros(8),
            np.zeros(8),
        )
    )
    boundary = np.asarray([0.11, 0.0, 0.0])
    expected = TrajectoryFollower(fresh, FollowerConfig(**FOLLOWER_VALUES)).forward(boundary)
    oracle = reconstruct_raw_follower_decision(
        fresh, boundary=boundary, follower_values=FOLLOWER_VALUES
    )
    assert oracle["k_fresh"] == expected.nearest_index == 1
    assert oracle["q_fresh"] == expected.target_index == 4
    assert oracle["q_minus_k"] == 3
    assert 0 <= oracle["k_fresh"] <= oracle["q_fresh"] < len(fresh)
    assert oracle["lookahead_distance_m"] == 0.25
    assert oracle["first_command_v_omega"] == pytest.approx(
        [expected.linear_velocity_mps, expected.angular_velocity_rps]
    )


def test_reconstructed_command_validation_is_tight_and_named():
    valid = validate_reconstructed_first_command(
        [0.2, -0.1], [0.2 + 1e-13, -0.1], tolerance=1e-12
    )
    assert valid["consistent"]
    with pytest.raises(InputReconstructionError) as error:
        validate_reconstructed_first_command(
            [0.2, -0.1], [0.2 + 1e-6, -0.1], tolerance=1e-12
        )
    assert error.value.status == INPUT_RECONSTRUCTION_FAILED


def test_candidate_geometry_deformation_and_command_diagnostics():
    problem = make_problem()
    geometry = transition_geometry_metrics(problem, observation_pose=[-0.1, 0.0, 0.0])
    assert geometry["entry_index_k"] == 0
    assert geometry["target_index_q"] == 3
    assert geometry["entry_to_lookahead_arc_length_m"] > 0.0
    assert geometry["undefined_geometry_statuses"] == []

    deformation = candidate_deformation_metrics(problem, problem.raw)
    assert deformation["entry_displacement_from_raw"]["translation_m"] == pytest.approx(
        0.0, abs=1e-15
    )
    assert deformation["q_node_displacement_from_raw"]["translation_m"] == pytest.approx(
        0.0, abs=1e-15
    )
    assert deformation["endpoint_displacement_from_raw"]["translation_m"] == pytest.approx(
        0.0, abs=1e-15
    )
    assert deformation["translation_deformation_rms_m"] == pytest.approx(0.0, abs=1e-15)
    assert deformation["fresh_relative_motion"]["translation_rms_m"] == pytest.approx(
        0.0, abs=1e-15
    )
    assert deformation["best_fit_single_rigid_transform"]["translation_rms_m"] < 1e-12

    command = candidate_command_metrics(
        problem.raw,
        boundary=problem.inputs.committed_pose_world,
        old_last_command=[0.1, -0.2],
        follower_values=FOLLOWER_VALUES,
        original_entry_index=problem.entry_index,
        raw_target_index=problem.raw_target_index,
    )
    expected_j = np.hypot(
        command["delta_v_abs_mps"] / 0.15,
        command["delta_omega_abs_rps"] / 0.30,
    )
    assert command["J_cmd"] == pytest.approx(expected_j)
    assert 0 <= command["candidate_nearest_index_relative"] < len(problem.raw)
    assert 0 <= command["candidate_target_index_relative"] < len(problem.raw)


def test_input_arrays_remain_read_only_and_raw_candidate_is_independent():
    problem = make_problem()
    original = problem.inputs.fresh_poses_world.copy()
    result = solve_method(problem, "M0_RAW", SolverConfig(**graph_config()["solver"]))
    assert not result.candidate.flags.writeable
    assert not problem.inputs.fresh_poses_world.flags.writeable
    assert np.array_equal(problem.inputs.fresh_poses_world, original)
