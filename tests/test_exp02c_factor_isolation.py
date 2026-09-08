from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.exp02b import CASE_IDS, load_source_case, transition_input
from reconciliation.exp02c_factor_isolation import (
    FACTOR_ORDER,
    best_fit_left_se2,
    factor_residual_vector,
    gradient_cosines,
    gradient_diagnostics,
    physical_factor_residuals,
    problem_for_variant,
    residual_diagnostics,
    solve_variant,
    synthetic_conditions,
)
from reconciliation.graph_optimizer import SolverConfig
from reconciliation.online_switch import save_json_exclusive, sha256_file
from reconciliation.se2 import compose_poses
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput
from reconciliation.transition_graph import problem_from_config, transition_residual_vector


ROOT = Path(__file__).resolve().parents[1]


def graph_config():
    with (ROOT / "configs/exp02a_spatial_entry.yaml").open() as stream:
        return yaml.safe_load(stream)


def make_input(case_name="S0_PERFECTLY_BENIGN"):
    values = synthetic_conditions()[case_name]
    return TransitionReconciliationInput(
        old_poses_world=values["old"],
        fresh_poses_world=values["fresh"],
        committed_pose_world=values["boundary"],
        actual_pose_before_committed=np.asarray(values["old"])[-1],
        entry_context=SpatialEntryContext(entry_index=0, source="test"),
    )


def problem(inputs, variant):
    config = graph_config()
    return problem_for_variant(
        inputs,
        variant,
        residual_scales=config["residual_scales"],
        weights={name: 1.0 for name in FACTOR_ORDER},
        minimum_translation_m=config["minimum_direction_translation_m"],
    )


def solver():
    return SolverConfig(**graph_config()["solver"])


def test_raw_fresh_entry_residual_is_zero():
    inputs = make_input("S1_DIRECTION_ONLY")
    residuals = physical_factor_residuals(inputs, inputs.selected_suffix, minimum_translation_m=1e-6)
    assert np.array_equal(residuals["entry_preservation"], np.zeros((1, 3)))


def test_raw_fresh_motion_residual_is_zero():
    inputs = make_input("S2_YAW_ONLY")
    residuals = physical_factor_residuals(inputs, inputs.selected_suffix, minimum_translation_m=1e-6)
    assert np.allclose(residuals["fresh_motion"], 0.0, atol=1e-15)


def test_s0_all_factors_zero():
    inputs = make_input()
    residuals = physical_factor_residuals(inputs, inputs.selected_suffix, minimum_translation_m=1e-6)
    assert all(np.linalg.norm(value) < 1e-12 for value in residuals.values())


def test_s1_only_direction_nonzero():
    inputs = make_input("S1_DIRECTION_ONLY")
    residuals = physical_factor_residuals(inputs, inputs.selected_suffix, minimum_translation_m=1e-6)
    nonzero = {name for name, value in residuals.items() if np.linalg.norm(value) > 1e-12}
    assert nonzero == {"incoming_direction"}


def test_s2_only_yaw_nonzero():
    inputs = make_input("S2_YAW_ONLY")
    residuals = physical_factor_residuals(inputs, inputs.selected_suffix, minimum_translation_m=1e-6)
    nonzero = {name for name, value in residuals.items() if np.linalg.norm(value) > 1e-12}
    assert nonzero == {"incoming_yaw"}


def test_per_factor_jacobian_and_gradients_are_finite_and_sized():
    inputs = make_input("S1_DIRECTION_ONLY")
    graph = problem(inputs, "V4_FULL_CURRENT_M4")
    assert graph is not None
    arrays, payload = gradient_diagnostics(graph, inputs.selected_suffix, epsilon=1e-6)
    assert arrays.shape == (4, 2, inputs.selected_suffix.size)
    assert np.all(np.isfinite(arrays))
    assert all(
        item["weighted"]["jacobian_shape"][1] == inputs.selected_suffix.size
        for item in payload["factors"].values()
    )


def test_zero_gradient_cosine_is_null():
    gradients = np.zeros((4, 2, 9))
    value = gradient_cosines(
        gradients,
        active_factors=("entry_preservation", "incoming_direction"),
        zero_norm_tolerance=1e-12,
    )
    assert value["pairs"]["entry_preservation__vs__incoming_direction"] is None


def test_factor_subset_cost_composes_from_active_factors():
    inputs = make_input("S2_YAW_ONLY")
    graph = problem(inputs, "V3_ENTRY_YAW_FRESH")
    assert graph is not None
    residual = factor_residual_vector(graph, inputs.selected_suffix)
    diagnostics = residual_diagnostics(graph, inputs.selected_suffix)
    expected = sum(
        item["diagnostic_weighted_cost"]
        for name, item in diagnostics.items()
        if name in graph.active_factors
    )
    assert residual @ residual == pytest.approx(expected, abs=1e-12)


def test_v1_is_exact_raw_noop():
    inputs = make_input("S1_DIRECTION_ONLY")
    result = solve_variant(problem(inputs, "V1_ENTRY_FRESH"), inputs.selected_suffix, solver())
    assert np.array_equal(result.optimized, inputs.selected_suffix)
    assert result.initial_cost == 0.0
    assert result.final_cost == 0.0


def test_v4_residual_vector_matches_production_transition_problem():
    inputs = make_input("S1_DIRECTION_ONLY")
    config = graph_config()
    diagnostic = problem(inputs, "V4_FULL_CURRENT_M4")
    production = problem_from_config(
        inputs,
        "incoming_motion_aware",
        residual_scales=config["residual_scales"],
        weights=config["variants"]["incoming_motion_aware"],
        minimum_translation_m=config["minimum_direction_translation_m"],
    )
    assert diagnostic is not None
    assert np.array_equal(
        factor_residual_vector(diagnostic, inputs.selected_suffix),
        transition_residual_vector(production, inputs.selected_suffix),
    )


@pytest.mark.parametrize("case", CASE_IDS)
@pytest.mark.parametrize("k", (0, 3, 6))
def test_v4_matches_frozen_historical_candidates(case, k):
    with (ROOT / "configs/exp02b_controller_aware.yaml").open() as stream:
        config = yaml.safe_load(stream)
    trial = ROOT / config["paths"]["source_root"] / config["frozen_source_cases"][case]["relative_trial_path"]
    inputs = transition_input(load_source_case(case, trial), k)
    result = solve_variant(problem(inputs, "V4_FULL_CURRENT_M4"), inputs.selected_suffix, solver())
    frozen = np.load(
        ROOT / "data/exp02b/exp02b-controller-aware-20260906T150400Z" / case / f"k_{k}/graph/candidate.npy",
        allow_pickle=False,
    )
    with (ROOT / "configs/exp02b_calibrated_reeval.yaml").open() as stream:
        manifest = yaml.safe_load(stream)
    frozen_path = ROOT / "data/exp02b/exp02b-controller-aware-20260906T150400Z" / case / f"k_{k}/graph/candidate.npy"
    assert sha256_file(frozen_path) == manifest["candidate_sha256"][case][f"k_{k}"]["graph"]
    difference = result.optimized - frozen
    assert np.max(np.linalg.norm(difference[:, :2], axis=1)) <= 1e-9
    assert np.max(np.abs((difference[:, 2] + np.pi) % (2 * np.pi) - np.pi)) <= 1e-9


def test_no_entry_ablation_is_finite():
    inputs = make_input("S1_DIRECTION_ONLY")
    result = solve_variant(problem(inputs, "V5_NO_ENTRY"), inputs.selected_suffix, solver())
    assert result.converged
    assert np.all(np.isfinite(result.optimized))


def test_no_propagation_changes_only_first_node():
    inputs = make_input("S4_DOWNSTREAM_CONFLICT")
    result = solve_variant(
        problem(inputs, "V8_DIAGNOSTIC_NO_PROPAGATION"), inputs.selected_suffix, solver()
    )
    assert not np.allclose(result.optimized[0], inputs.selected_suffix[0])
    assert np.array_equal(result.optimized[1:], inputs.selected_suffix[1:])


def test_rigid_fit_exact_left_transform_has_zero_residual():
    raw = synthetic_conditions()["S0_PERFECTLY_BENIGN"]["fresh"]
    candidate = compose_poses([0.5, -0.2, 0.3], raw)
    fit = best_fit_left_se2(raw, candidate)
    assert fit["translation_max_m"] < 1e-12
    assert fit["yaw_max_rad"] < 1e-12


def test_identity_correction_profile_is_zero():
    raw = synthetic_conditions()["S0_PERFECTLY_BENIGN"]["fresh"]
    fit = best_fit_left_se2(raw, raw)
    assert fit["correction_variation_translation_max_m"] < 1e-12
    assert fit["correction_variation_yaw_max_rad"] < 1e-12


def test_input_arrays_remain_immutable_after_solve():
    inputs = make_input("S1_DIRECTION_ONLY")
    original_old = inputs.old_poses_world.copy()
    original_fresh = inputs.fresh_poses_world.copy()
    solve_variant(problem(inputs, "V4_FULL_CURRENT_M4"), inputs.selected_suffix, solver())
    assert np.array_equal(inputs.old_poses_world, original_old)
    assert np.array_equal(inputs.fresh_poses_world, original_fresh)
    assert not inputs.old_poses_world.flags.writeable
    assert not inputs.fresh_poses_world.flags.writeable


def test_diagnostic_output_is_exclusive(tmp_path):
    path = tmp_path / "summary.json"
    save_json_exclusive(path, {"valid": True})
    with pytest.raises(FileExistsError):
        save_json_exclusive(path, {"valid": False})


def test_nan_and_inf_are_rejected():
    inputs = make_input()
    invalid = inputs.selected_suffix.copy()
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError):
        physical_factor_residuals(inputs, invalid, minimum_translation_m=1e-6)
    invalid[0, 0] = np.inf
    with pytest.raises(ValueError):
        best_fit_left_se2(inputs.selected_suffix, invalid)
