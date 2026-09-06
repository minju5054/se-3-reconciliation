import copy

import numpy as np
import pytest

from reconciliation.controller_switch_metrics import (
    align_command_rows_to_raw_switch,
    command_rows_to_array,
    controlled_switch_geometry,
    controller_switch_metrics,
    validate_command_array,
)


def test_immediate_signed_absolute_and_short_window_metrics() -> None:
    old = [[0.1, -0.1], [0.2, -0.2], [0.3, -0.3]]
    fresh = [[0.5, 0.2], [0.4, 0.1], [0.35, 0.0]]
    result = controller_switch_metrics(old, fresh, control_dt_s=0.1)
    assert result["delta_v_signed_mps"] == pytest.approx(0.2)
    assert result["delta_omega_signed_rps"] == pytest.approx(0.5)
    assert result["delta_v_abs_mps"] == pytest.approx(0.2)
    assert result["delta_omega_abs_rps"] == pytest.approx(0.5)
    assert result["post_switch_max_abs_delta_v_mps"] == pytest.approx(0.2)
    assert result["post_switch_mean_abs_delta_v_mps"] == pytest.approx((0.2 + 0.1 + 0.05) / 3)
    assert result["immediate_command_slew_v_mps2"] == pytest.approx(2.0)
    assert "not physical acceleration" in result["slew_semantics"]


def test_command_validation_rejects_invalid_and_preserves_inputs() -> None:
    commands = np.zeros((5, 2), dtype=np.float32)
    before = commands.copy()
    assert validate_command_array(commands).shape == (5, 2)
    assert np.array_equal(commands, before)
    for bad in (np.zeros((2, 3)), np.array([[np.nan, 0.0]]), np.empty((0, 2))):
        with pytest.raises(ValueError):
            validate_command_array(bad)
    with pytest.raises(ValueError):
        controller_switch_metrics(np.zeros((2, 2)), np.zeros((3, 2)), control_dt_s=0.1)


def test_geometry_uses_first_meaningful_tangent_and_wraps_yaw() -> None:
    fresh = np.array(
        [
            [0.2, 0.0, np.pi - 0.01],
            [0.2, 0.0, -np.pi + 0.01],
            [0.2, 0.2, -np.pi + 0.02],
            [0.1, 0.4, -np.pi + 0.03],
        ]
    )
    result = controlled_switch_geometry(
        actual_pose_before_switch=[0.0, 0.0, np.pi - 0.01],
        actual_pose_at_switch=[0.1, 0.0, -np.pi + 0.01],
        robot_pose_at_fresh_observation=[0.0, 0.0, np.pi - 0.01],
        fresh_world=fresh,
        zero_motion_tolerance_m=1e-6,
    )
    assert result["fresh_first_meaningful_edge_index"] == 1
    assert result["fresh_first_meaningful_heading_rad"] == pytest.approx(np.pi / 2)
    assert result["old_fresh_tangent_disagreement_abs_rad"] == pytest.approx(np.pi / 2)
    assert result["old_incoming_yaw_increment_rad"] == pytest.approx(0.02)
    assert "not velocity" in result["metric_semantics"]["local_spatial_step_magnitude_mismatch_m"]


def test_near_zero_tangents_are_undefined() -> None:
    result = controlled_switch_geometry(
        actual_pose_before_switch=[0.0, 0.0, 0.0],
        actual_pose_at_switch=[0.0, 0.0, 0.0],
        robot_pose_at_fresh_observation=[0.0, 0.0, 0.0],
        fresh_world=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        zero_motion_tolerance_m=1e-6,
    )
    assert result["old_incoming_heading_defined"] is False
    assert result["fresh_first_meaningful_heading_defined"] is False
    assert result["old_fresh_tangent_disagreement_rad"] is None
    assert result["boundary_to_fresh_entry_direction_defined"] is False


def test_arbitrary_horizon_geometry_does_not_mutate_fresh() -> None:
    for horizon in (2, 10, 24):
        fresh = np.column_stack((np.linspace(0.1, 1.0, horizon), np.zeros(horizon), np.zeros(horizon)))
        before = fresh.copy()
        controlled_switch_geometry(
            actual_pose_before_switch=[0.0, 0.0, 0.0],
            actual_pose_at_switch=[0.05, 0.0, 0.0],
            robot_pose_at_fresh_observation=[0.0, 0.0, 0.0],
            fresh_world=fresh,
            zero_motion_tolerance_m=1e-8,
        )
        assert np.array_equal(fresh, before)


def test_command_rows_filter_exact_reference_source() -> None:
    rows = [
        {"reference_source": "OLD", "v_command_mps": 0.2, "omega_command_rps": -0.1},
        {"reference_source": "FRESH", "v_command_mps": 0.3, "omega_command_rps": 0.2},
    ]
    before = copy.deepcopy(rows)
    result = command_rows_to_array(rows, source="OLD")
    assert result.tolist() == [[0.2, -0.1]]
    assert rows == before


def test_command_trace_alignment_places_first_fresh_command_at_zero() -> None:
    rows = [
        {"sim_time_s": "1.9", "reference_source": "OLD"},
        {"sim_time_s": "2.0", "reference_source": "FRESH"},
        {"sim_time_s": "2.1", "reference_source": "FRESH"},
    ]
    before = copy.deepcopy(rows)
    aligned = align_command_rows_to_raw_switch(rows)
    assert [row["time_from_raw_switch_s"] for row in aligned] == pytest.approx([-0.1, 0.0, 0.1])
    assert rows == before
