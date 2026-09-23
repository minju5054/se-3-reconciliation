"""Synthetic checker fixtures, not additional scientific observations."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_join_online02_stop import point_relation, stop_evidence, write_new


def pointing(pixel=(245., 155.), clamped=False):
    return dict(mode='grid', frame_size=[480, 270], opos_px=pixel,
                opos_state='point' if pixel else 'not_visible', opos_clamped=clamped)


def instance():
    return dict(matched_instance_ids=[7], idToLabels={'7': '/cart', '0': '/floor'})


def test_quantized_cell_and_center_hit_are_distinct():
    mask = np.zeros((270, 480), dtype=np.uint32)
    mask[155, 245] = 7
    r = point_relation(pointing(), 'opos', mask, instance())
    assert r['raster_on_cart'] and r['grid_cell_cart_fraction'] == .01
    assert r['grid_cell_xyxy'] == [240, 150, 250, 160]
    mask[150:160, 240:250] = 7
    assert point_relation(pointing(), 'opos', mask, instance())['grid_cell_cart_fraction'] == 1.


def test_absent_pixel_or_cart_remains_missing_not_false_grounding():
    mask = np.zeros((270, 480), dtype=np.uint32)
    absent = point_relation(pointing(None), 'opos', mask, instance())
    assert absent['raster_on_cart'] is None and absent['grid_cell_cart_fraction'] is None
    r = point_relation(pointing(), 'opos', mask, instance())
    assert r['pixel_to_cart_distance'] is None and not r['raster_on_cart']


def test_clamping_and_resolution_checked():
    mask = np.zeros((270, 480), dtype=np.uint32)
    r = point_relation(pointing((245, 265), True), 'opos', mask, instance())
    assert r['interpretation'] == 'CENSORED_BY_CLAMPING'
    with pytest.raises(ValueError, match='resolution'):
        point_relation(pointing(), 'opos', mask[:100], instance())


def response(codes, n=10, apos='point', stop=False, value=1.):
    return dict(raw_text=''.join(f'<act_l{i}_{c}>' for i, c in enumerate(codes)),
                actions=dict(actions=np.full((n, 3), value).tolist()),
                pointing=dict(apos_state=apos), stop=stop)


MANIFEST = dict(levels=[256]*3, stop=dict(l0=6, codes=[6, 122, 174]))


@pytest.mark.parametrize('n', [1, 10, 17])
def test_generic_N_and_explicit_stop(n):
    r = stop_evidence(response([6, 122, 174], n=n, apos='stop', stop=True, value=0.), MANIFEST)
    assert r['rows'] == n and r['explicit_stop_l0'] and r['full_manifest_stop_tuple']
    assert r['decoded_all_zero'] and r['wire_stop']


def test_l0_stop_predicate_is_not_APOS_or_exact_tuple():
    r = stop_evidence(response([0, 30, 85], apos='stop'), MANIFEST)
    assert not r['explicit_stop_l0'] and not r['wire_stop']
    r = stop_evidence(response([6, 0, 1]), MANIFEST)
    assert r['explicit_stop_l0'] and not r['full_manifest_stop_tuple']


def test_missing_action_codes_are_an_error():
    with pytest.raises(ValueError, match='levels'):
        stop_evidence(response([0, 30]), MANIFEST)


def test_no_overwrite(tmp_path):
    p = tmp_path / 'raw.json'
    write_new(p, {'v': 1})
    before = p.read_bytes()
    with pytest.raises(FileExistsError):
        write_new(p, {'v': 2})
    assert p.read_bytes() == before
