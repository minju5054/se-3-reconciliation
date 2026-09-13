"""Synthetic SE(2) geometry fixtures; never experimental evidence."""

import numpy as np
from reconciliation.exp02d_turning_search import path_turning, reference_is_turning, actual_is_turning


def test_yaw_wrap_does_not_create_a_false_full_turn():
    result = path_turning([[0, 0, np.deg2rad(179)], [.5, 0, np.deg2rad(-179)], [1, 0, np.deg2rad(-177)]])
    assert np.isclose(result["yaw_excursion_deg"], 4)
    assert result["tangent_excursion_deg"] == 0


def test_curved_path_geometry_is_invariant_to_world_rotation_and_translation():
    t = np.linspace(0, np.pi / 2, 11)
    path = np.column_stack([np.sin(t), 1 - np.cos(t), t])
    angle = 2.9
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    moved = path.copy()
    moved[:, :2] = path[:, :2] @ rotation.T + [12, -4]
    moved[:, 2] = (path[:, 2] + angle + np.pi) % (2 * np.pi) - np.pi
    a, b = path_turning(path), path_turning(moved)
    for key in a:
        assert np.isclose(a[key], b[key])
    assert np.isclose(a["yaw_excursion_deg"], 90)
    assert np.isclose(a["tangent_excursion_deg"], 81)


def test_straight_line_with_rotating_yaw_is_not_a_curved_reference():
    config = dict(minimum_reference_arc_m=.5, minimum_reference_yaw_excursion_deg=30,
                  minimum_reference_tangent_excursion_deg=30, minimum_actual_arc_m=.5,
                  minimum_actual_yaw_excursion_deg=30)
    features = path_turning([[0, 0, 0], [.5, 0, .4], [1, 0, .8]])
    assert not reference_is_turning(features, config)
    assert actual_is_turning(features, config)
    assert not actual_is_turning(path_turning([[0, 0, 0], [0, 0, 1]]), config)
