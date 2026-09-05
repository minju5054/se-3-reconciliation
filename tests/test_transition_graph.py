import numpy as np
import pytest

from reconciliation.exp02a import synthetic_inputs
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares
from reconciliation.se2 import wrap_angle
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput
from reconciliation.transition_graph import (
    TransitionGraphProblem,
    TransitionResidualScales,
    TransitionWeights,
    entry_preservation_residual,
    incoming_motion_residual,
    physical_transition_residuals,
    pose_anchor_residual,
    transition_residual_vector,
    world_translation_direction,
)
from reconciliation.transition_metrics import evaluate_transition_state, inter_k_separation


SCALES = TransitionResidualScales(0.1, 0.1, 0.1)
SOLVER = SolverConfig()


def synthetic_config():
    contexts = {}
    for label, k in (("early", 0), ("middle", 3), ("late", 6)):
        contexts[label] = {
            "entry_index": k,
            "evidence": {"candidate": float(k)},
            "evidence_status": {"candidate": "candidate"},
            "source": "oracle_exp02a_test",
        }
    return {
        "old_start_pose": [-0.6, 0, 0],
        "old_local_step": [0.2, 0, 0],
        "old_pose_count": 3,
        "committed_pose": [0, 0, 0],
        "fresh_initial_pose": [0.18, 0.12, 0.10],
        "fresh_local_step": [0.17, 0, 0.055],
        "fresh_pose_count": 10,
        "contexts": contexts,
    }


def make_problem(inputs, variant):
    weights = {
        "pose_anchor": TransitionWeights(pose_anchor=1, fresh_motion=1),
        "entry_preservation": TransitionWeights(entry_preservation=1, fresh_motion=1),
        "incoming_motion_aware": TransitionWeights(
            entry_preservation=1, incoming_direction=1, incoming_yaw=1, fresh_motion=1
        ),
    }[variant]
    return TransitionGraphProblem(inputs, variant, SCALES, weights, 1e-6)


def solve(problem):
    return solve_least_squares(
        problem.raw_new,
        lambda state: transition_residual_vector(problem, state),
        SOLVER,
    )


def test_transition_zero_residuals_and_angle_wrap() -> None:
    assert pose_anchor_residual([1, 2, 0.3], [1, 2, 0.3]) == pytest.approx([0, 0, 0])
    assert entry_preservation_residual([1, 2, -3.13], [1, 2, -3.13]) == pytest.approx([0, 0, 0])
    residual = incoming_motion_residual(
        [-0.2, 0, np.pi - 0.01],
        [0, 0, -np.pi + 0.01],
        [0.2, 0, -np.pi + 0.03],
        [0.2, 0, -np.pi + 0.03],
        minimum_translation_m=1e-6,
    )
    assert residual[:2] == pytest.approx([0.0, 0.0])
    assert residual[2] == pytest.approx(0.0, abs=1e-12)
    assert wrap_angle(residual[2]) == pytest.approx(residual[2])


def test_transition_vector_distinguishes_antiparallel_motion() -> None:
    residual = incoming_motion_residual(
        [-0.2, 0, 0],
        [0, 0, 0],
        [-0.2, 0, 0],
        [-0.2, 0, 0],
        minimum_translation_m=1e-6,
    )
    assert residual == pytest.approx([-2.0, 0.0, 0.0])


def test_near_zero_direction_is_rejected_explicitly() -> None:
    with pytest.raises(ValueError, match="undefined"):
        world_translation_direction([0, 0, 0], [1e-8, 0, 0], minimum_translation_m=1e-6, name="test")
    inputs = next(iter(synthetic_inputs(synthetic_config()).values()))
    bad = TransitionReconciliationInput(
        inputs.old_poses_world,
        inputs.fresh_poses_world,
        inputs.committed_pose_world,
        inputs.entry_context,
        inputs.committed_pose_world,
    )
    with pytest.raises(ValueError, match="incoming OLD motion"):
        make_problem(bad, "incoming_motion_aware")


def test_complete_fresh_motion_chain_is_present_for_arbitrary_suffix() -> None:
    inputs = synthetic_inputs(synthetic_config())["middle"]
    problem = make_problem(inputs, "entry_preservation")
    groups = physical_transition_residuals(problem, problem.raw_new)
    assert groups["fresh_motion"].shape == (problem.raw_new.shape[0] - 1, 3)
    assert groups["fresh_motion"] == pytest.approx(0.0, abs=1e-12)


def test_evidence_never_changes_residual_or_optimizer_output() -> None:
    inputs = synthetic_inputs(synthetic_config())["middle"]
    changed_context = SpatialEntryContext(
        entry_index=inputs.entry_context.entry_index,
        evidence={"arbitrary_unvalidated_feature": 999.0},
        evidence_status={"arbitrary_unvalidated_feature": "candidate"},
        source="different_evidence",
    )
    changed = TransitionReconciliationInput(
        inputs.old_poses_world,
        inputs.fresh_poses_world,
        inputs.committed_pose_world,
        changed_context,
        inputs.actual_pose_before_committed,
    )
    first = make_problem(inputs, "incoming_motion_aware")
    second = make_problem(changed, "incoming_motion_aware")
    assert transition_residual_vector(first, first.raw_new) == pytest.approx(
        transition_residual_vector(second, second.raw_new)
    )
    assert np.array_equal(solve(first).optimized, solve(second).optimized)


def test_variants_are_deterministic_and_have_expected_tradeoff() -> None:
    inputs = synthetic_inputs(synthetic_config())["middle"]
    raw_metrics = evaluate_transition_state(
        inputs, inputs.selected_suffix, translation_scale_m=0.1, yaw_scale_rad=0.1, minimum_translation_m=1e-6
    )
    pose = solve(make_problem(inputs, "pose_anchor"))
    entry = solve(make_problem(inputs, "entry_preservation"))
    incoming_problem = make_problem(inputs, "incoming_motion_aware")
    incoming = solve(incoming_problem)
    incoming_again = solve(incoming_problem)
    pose_metrics = evaluate_transition_state(
        inputs, pose.optimized, translation_scale_m=0.1, yaw_scale_rad=0.1, minimum_translation_m=1e-6
    )
    incoming_metrics = evaluate_transition_state(
        inputs, incoming.optimized, translation_scale_m=0.1, yaw_scale_rad=0.1, minimum_translation_m=1e-6
    )
    assert np.array_equal(entry.optimized, inputs.selected_suffix)
    assert pose_metrics["entry_preservation"]["translation_displacement_m"] > 0.2
    assert incoming_metrics["transition_motion"]["incoming_to_transition_direction_jump_rad"] < raw_metrics["transition_motion"]["incoming_to_transition_direction_jump_rad"]
    assert incoming_metrics["entry_preservation"]["translation_displacement_m"] < pose_metrics["entry_preservation"]["translation_displacement_m"]
    assert np.array_equal(incoming.optimized, incoming_again.optimized)
    assert incoming.converged
    assert incoming.final_cost < incoming.initial_cost


def test_inter_k_distinction_detects_pose_anchor_collapse() -> None:
    inputs = synthetic_inputs(synthetic_config())
    raw_states = {label: item.selected_suffix for label, item in inputs.items()}
    pose_states = {label: solve(make_problem(item, "pose_anchor")).optimized for label, item in inputs.items()}
    incoming_states = {
        label: solve(make_problem(item, "incoming_motion_aware")).optimized
        for label, item in inputs.items()
    }
    raw = inter_k_separation(inputs, raw_states)
    pose = inter_k_separation(inputs, pose_states)
    incoming = inter_k_separation(inputs, incoming_states)
    assert raw["pair_count"] == 3
    assert max(row["entry_separation_retention_ratio"] for row in pose["pairs"]) < 1e-6
    assert min(row["entry_separation_retention_ratio"] for row in incoming["pairs"]) > 0.5


def test_invalid_variant_weights_are_rejected() -> None:
    inputs = synthetic_inputs(synthetic_config())["early"]
    with pytest.raises(ValueError, match="required"):
        TransitionGraphProblem(
            inputs,
            "incoming_motion_aware",
            SCALES,
            TransitionWeights(entry_preservation=1, fresh_motion=1),
        )
