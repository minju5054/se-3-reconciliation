"""Rendering-only correction tests; synthetic inputs are not experimental evidence."""
import importlib.util
import json
from pathlib import Path

import pytest
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


fixture_module = module('ref01_synthetic_presentation_fixture', ROOT/'tests/test_gp_se2_ref01_visuals.py')
render = module('ref01_presentation_renderer_test', ROOT/'scripts/render_gp_se2_ref01_detail.py')


def test_additive_rendering_preserves_primary_and_all_row_identities(tmp_path, monkeypatch):
    run, case, _ = fixture_module.fixture_run.__wrapped__(tmp_path)
    env = HospitalEnvironment(box(2, -2, 2.2, 2), box(-3, -3, 3, 3))
    monkeypatch.setattr(render.HospitalEnvironment, 'load', lambda path:env)
    fixture_module.plots.main(run)
    original = {str(p):p.read_bytes() for p in run.rglob('*') if p.is_file()}
    output = tmp_path/'presentation_synthetic'
    result = render.main(run, output)
    assert result['image_count'] == 4 and result['primary_files_unchanged']
    assert result['new_controller_solves'] == 0
    assert original == {str(p):p.read_bytes() for p in run.rglob('*') if p.is_file()}
    correction = render.read(output/'correction_provenance.json')
    assert not correction['scientific_run_repeated'] and not correction['primary_artifacts_overwritten']
    common_zoom = []
    for image in correction['images']:
        path = output/image['path']; side = render.read(path.with_suffix('.json'))
        data = side['numeric_data']; name = image['variant']
        reference = render.read(case/'variants'/name/'row_provenance.json')
        assert data['row_provenance'] == reference
        assert len(data['formatted_table']) == len(reference)
        assert [r[0] for r in data['formatted_table']] == [str(i) for i in range(len(reference))]
        assert side['new_controller_solves'] == 0 and not side['numerical_results_changed']
        assert all(render.file_sha256(p) == h for p, h in side['source_hashes'].items())
        common_zoom.append(data['input_zoom_axes_world_m'])
    assert all(x == common_zoom[0] for x in common_zoom)
    assert common_zoom[0][1]-common_zoom[0][0] == pytest.approx(common_zoom[0][3]-common_zoom[0][2])
    with pytest.raises(FileExistsError):
        render.main(run, output)


def test_row_table_keeps_duplicate_positions_and_distinct_yaw():
    rows = [dict(derived_row_index=i, original_fractional_row_coordinate=float(i), original_left_row_index=i,
                 original_right_row_index=i, interpolation_alpha=0., world_xy=[1., 2.], wrapped_yaw=y,
                 unwrapped_yaw=y if i == 0 else y+6.283185307179586) for i, y in enumerate((3.12, -3.12))]
    table = render.table_rows(rows)
    assert len(table) == 2
    assert table[0][4:6] == table[1][4:6]
    assert table[0][0] != table[1][0] and table[0][6] != table[1][6]
    assert float(table[1][7]) > 180.
