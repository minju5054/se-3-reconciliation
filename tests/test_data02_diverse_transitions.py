from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.data02_diverse_transitions import (
    SCENARIO_IDS,
    apply_variation,
    anchor_successive_paths,
    canonicalize_at_boundary,
    classify_attempt,
    dataset_acceptance,
    descriptive_geometry_label,
    grouped_split,
    previous_control_pose,
    qualify_geometry,
    raw_pair_id,
    transition_context_id,
    validate_data02_config,
    validate_timing,
    variation_schedule,
    write_npy_exclusive,
)
from reconciliation.lightnav_execution_envelope import geometry_descriptor


ROOT = Path(__file__).resolve().parents[1]


def config():
    with (ROOT / "configs/data02_diverse_lightnav_transitions.yaml").open(
        "r", encoding="utf-8"
    ) as stream:
        return yaml.safe_load(stream)


def timing():
    return {
        "old_observation_sim_time_s": 1.0,
        "old_observation_host_monotonic_ns": 100,
        "old_request_host_monotonic_ns": 101,
        "old_ready_host_monotonic_ns": 110,
        "fresh_observation_sim_time_s": 2.0,
        "fresh_observation_host_monotonic_ns": 200,
        "fresh_request_host_monotonic_ns": 201,
        "fresh_model_ready_sim_time_s": 2.5,
        "fresh_model_ready_host_monotonic_ns": 250,
        "fresh_usable_sim_time_s": 2.5,
        "fresh_usable_host_monotonic_ns": 250,
    }


def context():
    return {
        "raw_pair_id": "rawpair_1234567890abcdef",
        "scenario_id": "D0_STRAIGHT",
        "variant_id": "V0_BASELINE",
        "old_observation_pose_world": [0.0, 0.0, 0.0],
        "fresh_observation_pose_world": [0.2, 0.0, 0.0],
        "previous_control_pose_world": [0.25, 0.0, 0.0],
        "boundary_pose_world": [0.3, 0.0, 0.0],
        "timing": timing(),
    }


def test_scenario_config_and_frozen_order():
    scenarios = validate_data02_config(config())
    assert tuple(item["id"] for item in scenarios) == SCENARIO_IDS
    assert all(item["boxes"] for item in scenarios)


def test_config_rejects_intrinsic_waypoint_time():
    value = config()
    value["lightnav"]["intrinsic_waypoint_time_base"] = True
    with pytest.raises(ValueError, match="waypoint timestamps"):
        validate_data02_config(value)


def test_deterministic_variation_schedule_and_body_lateral():
    value = config()
    assert [variation_schedule(value, index)["id"] for index in range(6)] == [
        "V0_BASELINE",
        "V1_YAW_POS",
        "V2_YAW_NEG",
        "V3_LATERAL_POS",
        "V4_LATERAL_NEG",
        "V0_BASELINE",
    ]
    pose = apply_variation([1.0, 2.0, np.pi / 2], value["variations"][3])
    np.testing.assert_allclose(pose, [0.9, 2.0, np.pi / 2], atol=1e-12)


def test_raw_world_separation_and_observation_anchors_without_mutation():
    old = np.asarray([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]])
    fresh = np.asarray([[0.1, 0.0, 0.1], [0.2, 0.05, 0.2]])
    old_before = old.copy()
    fresh_before = fresh.copy()
    old_world, fresh_world = anchor_successive_paths(
        old, fresh, [1.0, 2.0, 0.0], [3.0, 4.0, np.pi / 2]
    )
    np.testing.assert_array_equal(old, old_before)
    np.testing.assert_array_equal(fresh, fresh_before)
    np.testing.assert_allclose(old_world[:, :2], [[1.1, 2.0], [1.2, 2.0]])
    np.testing.assert_allclose(fresh_world[0], [3.0, 4.1, np.pi / 2 + 0.1])


def test_fresh_is_observation_anchored_not_boundary():
    raw = np.asarray([[0.2, 0.0, 0.0], [0.3, 0.0, 0.0]])
    _, fresh = anchor_successive_paths(raw, raw, [0, 0, 0], [1, 0, 0])
    _, wrong = anchor_successive_paths(raw, raw, [0, 0, 0], [2, 0, 0])
    assert fresh[0, 0] == pytest.approx(1.2)
    assert not np.array_equal(fresh, wrong)


def test_pair_hash_determinism_and_order():
    a = "a" * 64
    b = "b" * 64
    assert raw_pair_id(a, b) == raw_pair_id(a, b)
    assert raw_pair_id(a, b) != raw_pair_id(b, a)


def test_context_id_changes_with_boundary():
    original = context()
    changed = deepcopy(original)
    changed["boundary_pose_world"][0] += 0.01
    assert transition_context_id(original) != transition_context_id(changed)


def test_stop_and_failure_classification_precedence():
    checks = {"ok": True}
    stop = np.zeros((10, 3))
    assert classify_attempt(
        fresh_raw=stop,
        old_goal_reached=False,
        timing_checks=checks,
        execution_checks=checks,
    ) == "MODEL_STOP_OUTPUT"
    assert classify_attempt(
        fresh_raw=np.ones((10, 3)),
        old_goal_reached=True,
        timing_checks=checks,
        execution_checks=checks,
    ) == "OLD_EXHAUSTED"


def test_timing_order_and_usable_equals_ready():
    assert all(validate_timing(timing()).values())
    invalid = timing()
    invalid["fresh_request_host_monotonic_ns"] = 199
    assert not validate_timing(invalid)["fresh_request_after_observation"]
    invalid = timing()
    invalid["fresh_usable_sim_time_s"] = 2.6
    assert not validate_timing(invalid)["fresh_usable_equals_model_ready"]


def test_previous_control_pose_is_strictly_before_boundary():
    rows = [
        {"sim_time_s": 1.0, "actual_pose_world": [0.0, 0.0, 0.0]},
        {"sim_time_s": 1.1, "actual_pose_world": [0.1, 0.0, 0.0]},
        {"sim_time_s": 1.2, "actual_pose_world": [0.2, 0.0, 0.0]},
    ]
    np.testing.assert_array_equal(previous_control_pose(rows, 1.2), [0.1, 0.0, 0.0])


def test_left_right_and_straight_geometry_rules():
    thresholds = config()["qualification_thresholds"]
    straight = geometry_descriptor(np.asarray([[0, 0, 0], [0.5, 0, 0], [1, 0, 0]]))
    left = geometry_descriptor(np.asarray([[0, 0, 0], [0.4, 0.05, 0.12], [0.75, 0.25, 0.28]]))
    right = geometry_descriptor(np.asarray([[0, 0, 0], [0.4, -0.05, -0.12], [0.75, -0.25, -0.28]]))
    assert qualify_geometry(straight, "straight", thresholds)["passed"]
    assert qualify_geometry(left, "gentle_left", thresholds)["passed"]
    assert qualify_geometry(right, "gentle_right", thresholds)["passed"]
    assert not qualify_geometry(right, "gentle_left", thresholds)["passed"]


def test_s_curve_sign_change_descriptor_and_label():
    path = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [0.3, 0.0, 0.2],
            [0.55, 0.18, 0.4],
            [0.85, 0.18, 0.1],
            [1.10, 0.0, -0.1],
        ]
    )
    descriptor = geometry_descriptor(path)
    assert descriptor["curvature_sign_change_count"] >= 1
    thresholds = config()["qualification_thresholds"]
    assert qualify_geometry(descriptor, "s_curve", thresholds)["passed"]
    assert descriptive_geometry_label(descriptor, thresholds) == "S/compound-curvature-like"


def test_canonical_boundary_transform_maps_b_to_zero():
    boundary = np.asarray([2.0, 3.0, np.pi / 2])
    result = canonicalize_at_boundary(
        np.asarray([boundary, [2.0, 4.0, np.pi / 2]]), boundary
    )
    np.testing.assert_allclose(result[0], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(result[1], [1.0, 0.0, 0.0], atol=1e-12)


def test_grouped_split_prevents_duplicate_raw_pair_leakage():
    records = [
        {
            "transition_context_id": f"ctx_{index}",
            "raw_pair_id": "same" if index < 3 else f"pair_{index}",
        }
        for index in range(10)
    ]
    split = grouped_split(records, heldout_fraction=0.3, seed="fixed")
    assert split["raw_pair_leakage_count"] == 0
    assert not set(split["development_raw_pair_ids"]) & set(split["heldout_raw_pair_ids"])


def test_exclusive_output_rejects_overwrite(tmp_path):
    path = tmp_path / "raw.npy"
    source = np.asarray([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    write_npy_exclusive(path, source)
    with pytest.raises(FileExistsError):
        write_npy_exclusive(path, source)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_nan_inf_rejected(bad):
    with pytest.raises(ValueError):
        apply_variation([0.0, 0.0, 0.0], {"lateral_offset_m": bad, "yaw_offset_rad": 0.0})


def test_dataset_acceptance_logic():
    value = config()
    records = []
    for index in range(50):
        scenario = SCENARIO_IDS[index % len(SCENARIO_IDS)]
        records.append(
            {
                "classification": "VALID_MOVING",
                "scenario_id": scenario,
                "raw_pair_id": f"rawpair_{index:016x}",
            }
        )
    assert dataset_acceptance(records, value)["status"] == "DATA02_READY_FOR_FORMULATION_RESEARCH"
    assert dataset_acceptance(records[:20], value)["status"] == "DATA02_DIVERSITY_TARGET_NOT_MET"
