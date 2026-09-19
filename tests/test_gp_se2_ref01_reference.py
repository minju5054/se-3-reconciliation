"""Synthetic factor/lineage semantics only; none are navigation evidence."""
import copy

import numpy as np
import pytest

from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.gp_se2_ref01_reference import (
    VARIANTS, array_value_hash, build_variants, enrich_selection, sampled_polyline_distance,
)
from reconciliation.se2 import wrap_angle


W = (10., 10., 1.)


def make(rows, boundary=None):
    source = np.asarray(rows, dtype=np.float64)
    boundary = source[0] if boundary is None else boundary
    prep = prepare_reference(source, boundary, W)
    result = build_variants(source, boundary, W, expected_common=prep['common_world'],
                            preparation_metadata=prep)
    return source, prep, result


def test_exact_two_factors_no_padding_second_suffix_or_reanchoring():
    native = np.array([[5., -2., .1], [6., -2., .1], [7., -2., .1], [8., -2., .1]])
    saved = native.copy()
    source, prep, result = make(native, [6., -2., .1])
    assert result['suffix_selection']['first_future_row_index'] == 2
    assert tuple(result['variants']) == VARIANTS
    variants = result['variants']
    assert [len(variants[name]['reference_world']) for name in VARIANTS] == [4, 2, 30, 30]
    np.testing.assert_array_equal(variants[VARIANTS[0]]['reference_world'], source)
    np.testing.assert_array_equal(variants[VARIANTS[1]]['reference_world'], source[2:])
    np.testing.assert_array_equal(variants[VARIANTS[3]]['reference_world'], prep['common_world'])
    np.testing.assert_array_equal(saved, native)
    assert not np.shares_memory(variants[VARIANTS[0]]['reference_world'], native)
    assert result['conventions']['timestamps_passed_to_mpc'] is False
    assert result['conventions']['intrinsic_lightnav_waypoint_dt_s'] is None
    for variant in variants.values():
        np.testing.assert_array_equal(variant['reference_world'][-1, :2], source[-1, :2])
        assert abs(wrap_angle(variant['reference_world'][-1, 2]-source[-1, 2])) < 1e-12


def test_original_preparation_and_saved_common_authentication():
    rows, prep, _ = make([[0., 0., 0.], [1., 0., .1], [2., 1., .2]])
    bad = prep['common_world'].copy()
    bad[3, 0] += 1e-15
    with pytest.raises(ValueError, match='exactly equal'):
        build_variants(rows, rows[0], W, expected_common=bad)
    changed = copy.deepcopy(prep)
    changed['first_future_row_index'] = 2
    with pytest.raises(ValueError, match='metadata mismatch'):
        build_variants(rows, rows[0], W, expected_common=prep['common_world'], preparation_metadata=changed)
    changed = copy.deepcopy(prep)
    changed['row_times_s'][0] += 1e-8
    with pytest.raises(ValueError, match='array mismatch'):
        build_variants(rows, rows[0], W, expected_common=prep['common_world'], preparation_metadata=changed)


def test_weighted_yaw_selection_uses_original_native_and_original_boundary():
    native = [[0., 0., 0.], [.01, 0., 2.], [.5, 0., 0.], [1., 0., 0.]]
    _, _, result = make(native, [.01, 0., 0.])
    assert result['suffix_selection']['nearest_row_index'] == 0
    assert result['suffix_selection']['first_future_row_index'] == 1
    # Resampling changes nearest-grid identity but must never recompute external k.
    assert result['variants']['R11_CURRENT_ADAPTER']['row_provenance'][0]['original_fractional_row_coordinate'] == 1.


def test_resampling_lineage_source_identity_not_xy_geometry():
    native, _, result = make([[0., 0., 0.], [0., 0., .5], [0., 0., 1.], [1., 0., 1.]])
    rows = result['variants']['R01_RESAMPLE_ONLY']['row_provenance']
    for row in rows:
        left, right, alpha = row['original_left_row_index'], row['original_right_row_index'], row['interpolation_alpha']
        assert row['original_fractional_row_coordinate'] == left + alpha * (right-left)
        np.testing.assert_allclose(row['world_xy'], native[left, :2] + alpha*(native[right, :2]-native[left, :2]))
    # Equal XY does not erase multiple distinct original yaw identities.
    assert rows[0]['world_xy'] == rows[5]['world_xy']
    assert rows[0]['original_fractional_row_coordinate'] != rows[5]['original_fractional_row_coordinate']
    assert rows[0]['unwrapped_yaw'] != rows[5]['unwrapped_yaw']
    assert rows[0]['original_row'] and rows[-1]['original_row']
    assert rows[5]['interpolated']
    raw = result['variants']['R00_NATIVE']['geometry_audit']
    assert raw['duplicate_xy_segments'] == [0, 1]
    assert raw['rotation_only_segments'] == [0, 1]


def test_wrap_crossing_unwrap_stays_on_original_branch_after_suffix():
    _, _, result = make([[2., 3., 3.0], [2., 3., -3.1], [2., 3., -2.9]])
    for variant in result['variants'].values():
        yaw = np.array([row['unwrapped_yaw'] for row in variant['row_provenance']])
        assert np.max(np.abs(np.diff(yaw))) < .4
        assert yaw[-1] > np.pi
        assert np.allclose(wrap_angle(yaw), wrap_angle(variant['reference_world'][:, 2]), atol=1e-12)
    assert result['variants']['R10_SUFFIX_ONLY']['row_provenance'][0]['unwrapped_yaw'] > np.pi


@pytest.mark.parametrize('rows,boundary', [([[1., 2., .3]], [0., 0., 0.]),
                                          ([[0., 0., 0.], [1., 2., .3]], [1., 2., .3])])
def test_one_row_suffix_original_constant_helper(rows, boundary):
    native, _, result = make(rows, boundary)
    assert len(result['variants']['R10_SUFFIX_ONLY']['reference_world']) == 1
    common = result['variants']['R11_CURRENT_ADAPTER']
    assert common['reference_world'].shape == (30, 3)
    assert all(row['original_fractional_row_coordinate'] == len(native)-1 for row in common['row_provenance'])
    assert all(row['original_row'] for row in common['row_provenance'])
    assert common['geometry_audit']['total_xy_arc_length_m'] == 0


def test_resampling_can_cut_corner_and_skip_original_knot():
    _, _, result = make([[0., 0., 0.], [1., 0., 0.], [1., 1., .5]])
    variant = result['variants']['R01_RESAMPLE_ONLY']
    audit = variant['geometry_audit']
    assert audit['total_xy_arc_length_m'] < 2.
    assert audit['original_knots'][1]['is_corner']
    assert audit['original_knots'][1]['included_derived_row_indices'] == []
    assert audit['comparison_to_retained_source']['source_to_derived_max_sampled_m'] > .02
    assert audit['comparison_to_retained_source']['derived_to_source_max_sampled_m'] > .01
    assert audit['comparison_to_retained_source']['maximum_source_sampling_spacing_m'] == .001
    assert 'sampled' in audit['comparison_to_retained_source']['limitation']


def test_sampled_distance_handles_zero_segments_and_singleton():
    audit = sampled_polyline_distance([[1., 1.], [1., 1.]], [[2., 1.]])
    assert audit['source_to_derived_max_sampled_m'] == 1.
    assert audit['derived_to_source_max_sampled_m'] == 1.


def test_prefix_audit_records_removed_turn_and_edge_to_first_retained():
    native, _, result = make([[0., 0., 0.], [0., 0., .5], [1., 0., 1.]])
    suffix = result['variants']['R10_SUFFIX_ONLY']['geometry_audit']
    assert suffix['removed_prefix_rows'][0]['world_pose'] == native[0].tolist()
    assert suffix['removed_prefix_including_edge_to_first_retained_yaw_variation_rad'] == pytest.approx(.5)
    assert result['variants']['R00_NATIVE']['geometry_audit']['removed_prefix_rows'] == []


def selection_fixture():
    _, _, result = make([[1., 0., 0.], [2., 0., 0.], [3., 0., 0.], [4., 0., 0.]])
    variant = result['variants']['R00_NATIVE']
    return variant, np.array([[2., 0., 0.], [3., 0., 0.], [4., 0., 0.], [4., 0., 0.], [4., 0., 0.]])


def test_selection_physical_horizon_source_mapping_and_costs():
    variant, ref = selection_fixture()
    out = enrich_selection(variant['reference_world'], [0., 0., 0.], ref, variant['row_provenance'],
                           weights=W, horizon=5, original_goal_row_index=3)
    assert out['indices'] == [1, 2, 3, 3, 3]
    assert out['selected_original_fractional_row_coordinates'] == [1., 2., 3., 3., 3.]
    assert out['weighted_xy_contributions'] == [10., 40., 90., 160.]
    assert out['weighted_yaw_contributions'] == [0.]*4
    assert out['nearest_tie_margin'] == 30.
    assert out['selected_first_target_distance_m'] == 2.
    assert out['selected_last_target_distance_m'] == 4.
    assert out['horizon_xy_arc_length_m'] == 2.
    assert out['current_through_horizon_xy_arc_m'] == 4.
    assert out['final_goal_row_in_horizon'] is True
    assert out['endpoint_repetition_count'] == 2


def test_selection_exact_tie_preserves_first_argmin():
    _, _, result = make([[0., 0., -.1], [0., 0., .1], [1., 0., .2]])
    variant = result['variants']['R00_NATIVE']
    reference = np.array([[0., 0., .1], [1., 0., .2], [1., 0., .2]])
    out = enrich_selection(variant['reference_world'], [0., 0., 0.], reference, variant['row_provenance'],
                           weights=W, horizon=3, original_goal_row_index=2)
    assert out['nearest_index'] == 0
    assert out['exact_tied_nearest_indices'] == [0, 1]
    assert out['nearest_tie_margin'] == 0.
    assert out['indices'] == [1, 2, 2]


def test_selection_near_tie_is_reported_not_used_to_override_argmin():
    _, _, result = make([[0., 0., -.1000000001], [0., 0., .1], [1., 0., .2]])
    variant = result['variants']['R00_NATIVE']
    reference = np.tile([1., 0., .2], (3, 1))
    out = enrich_selection(variant['reference_world'], [0., 0., 0.], reference, variant['row_provenance'],
                           weights=W, horizon=3, original_goal_row_index=2)
    assert out['nearest_index'] == 1
    assert out['near_tied_nearest_indices'] == [0, 1]
    assert out['exact_tied_nearest_indices'] == [1]
    assert out['indices'] == [2, 2, 2]


def test_sequential_yaw_is_unwrapped_from_probe_state_not_visualization_branch():
    _, _, result = make([[0., 0., 3.1], [1., 0., -3.1], [2., 0., -3.0]])
    variant = result['variants']['R00_NATIVE']
    reference = np.array([[1., 0., 2*np.pi-3.1], [2., 0., 2*np.pi-3.]])
    out = enrich_selection(variant['reference_world'], [0., 0., 3.1], reference, variant['row_provenance'],
                           weights=W, horizon=2, original_goal_row_index=2)
    assert out['official_reference_unwrapped_yaw'][0] > np.pi
    assert abs(out['horizon_yaw_span_rad'] - .1) < 1e-12
    with pytest.raises(ValueError, match='sequential yaw unwrap'):
        enrich_selection(variant['reference_world'], [0., 0., 3.1], reference - [0., 0., 2*np.pi],
                         variant['row_provenance'], weights=W, horizon=2, original_goal_row_index=2)


def test_mismatched_actual_selection_or_lineage_fails():
    variant, ref = selection_fixture()
    args = (variant['reference_world'], [0., 0., 0.], ref, variant['row_provenance'])
    kw = dict(weights=W, horizon=5, original_goal_row_index=3)
    with pytest.raises(ValueError, match='provided selection audit'):
        enrich_selection(*args, **kw, audit={'nearest_index': 9})
    corrupted = copy.deepcopy(variant['row_provenance'])
    corrupted[2]['world_xy'][0] += .1
    with pytest.raises(ValueError, match='provenance geometry'):
        enrich_selection(*args[:3], corrupted, **kw)
    with pytest.raises(ValueError, match='provenance count'):
        enrich_selection(*args[:3], variant['row_provenance'][:-1], **kw)
    corrupted_reference = ref.copy()
    corrupted_reference[0, 0] += .01
    with pytest.raises(ValueError, match='selected XY'):
        enrich_selection(args[0], args[1], corrupted_reference, args[3], **kw)


def test_value_hash_distinguishes_order_and_shape():
    original = np.arange(12., dtype=float).reshape(4, 3)
    assert array_value_hash(original) == array_value_hash(original.copy())
    assert array_value_hash(original) != array_value_hash(original[::-1])
    assert array_value_hash(original) != array_value_hash(original.reshape(2, 6))
