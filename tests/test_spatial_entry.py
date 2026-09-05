import copy

import numpy as np
import pytest

from reconciliation.se2 import compose_poses
from reconciliation.spatial_entry import SpatialEntryContext, TransitionReconciliationInput


def trajectories(old_count: int = 4, fresh_count: int = 8):
    old = np.column_stack((np.linspace(-0.6, -0.1, old_count), np.zeros(old_count), np.zeros(old_count)))
    fresh = np.asarray(
        [compose_poses([0.2, 0.1, 0.1], [0.15 * index, 0.0, 0.04 * index]) for index in range(fresh_count)]
    )
    return old, fresh


def context(k: int = 2, **overrides) -> SpatialEntryContext:
    values = {
        "entry_index": k,
        "evidence": {"candidate_score": 0.25, "unknown_nested": {"values": [1, 2]}},
        "evidence_status": {"candidate_score": "candidate"},
        "source": "oracle_exp02a_test",
        "metadata": {"note": "fixture"},
    }
    values.update(overrides)
    return SpatialEntryContext(**values)


def test_context_round_trip_preserves_unknown_optional_evidence() -> None:
    original = context().to_dict()
    recovered = SpatialEntryContext.from_mapping(copy.deepcopy(original))
    assert recovered.to_dict() == original
    assert recovered.to_dict()["evidence"]["unknown_nested"]["values"] == [1, 2]


def test_missing_optional_evidence_is_allowed() -> None:
    value = SpatialEntryContext(entry_index=0, source="manual")
    assert value.to_dict()["evidence"] == {}
    assert value.to_dict()["evidence_status"] == {}


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_context_rejects_nonfinite_evidence(bad: float) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        context(evidence={"candidate": bad})


def test_arbitrary_lengths_k_zero_and_exact_suffix() -> None:
    old, fresh = trajectories(old_count=7, fresh_count=13)
    inputs = TransitionReconciliationInput(old, fresh, [0, 0, 0], context(0), old[-1])
    assert inputs.old_poses_world.shape == (7, 3)
    assert inputs.fresh_poses_world.shape == (13, 3)
    assert np.array_equal(inputs.selected_suffix, fresh)


def test_suffix_is_exact_fresh_k_and_fixed_inputs_are_read_only_copies() -> None:
    old, fresh = trajectories()
    old_original = old.copy()
    fresh_original = fresh.copy()
    committed = np.array([0.0, 0.0, 0.0])
    inputs = TransitionReconciliationInput(old, fresh, committed, context(3), old[-1])
    assert np.array_equal(inputs.selected_suffix, fresh[3:])
    assert not inputs.old_poses_world.flags.writeable
    assert not inputs.fresh_poses_world.flags.writeable
    with pytest.raises(ValueError):
        inputs.fresh_poses_world[0, 0] = 99.0
    assert np.array_equal(old, old_original)
    assert np.array_equal(fresh, fresh_original)
    assert np.array_equal(committed, [0.0, 0.0, 0.0])


def test_entry_index_bounds_and_final_pose_rejection_are_explicit() -> None:
    old, fresh = trajectories()
    with pytest.raises(ValueError, match="bounds"):
        TransitionReconciliationInput(old, fresh, [0, 0, 0], context(8), old[-1])
    with pytest.raises(ValueError, match="following pose"):
        TransitionReconciliationInput(old, fresh, [0, 0, 0], context(7), old[-1])


def test_actual_incoming_pose_is_preferred_but_planned_relation_is_retained() -> None:
    old, fresh = trajectories()
    measured = np.array([-0.05, 0.02, 0.01])
    inputs = TransitionReconciliationInput(old, fresh, [0, 0, 0], context(), measured)
    assert np.array_equal(inputs.incoming_previous_pose, measured)
    assert inputs.planned_old_to_boundary.shape == (3,)
