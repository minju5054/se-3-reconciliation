"""Synthetic rendering fixtures only; not saved hard-event research evidence."""
from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import re
from types import SimpleNamespace
import zipfile

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import box

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('diag03_plots', ROOT / 'scripts/plot_gp_se2_diag03.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)


def fixture(tmp_path):
    u = np.tile([0., .25, .5, .75, 1.], 30)
    interval = np.repeat(np.arange(30), 5)
    times = (interval+u)*.1
    zeros = np.zeros_like(times)
    config = dict(equality_tolerance=1e-5, inequality_tolerance=1e-5, v_max=.8,
        w_max=3., a_v_max=2., a_w_max=5.)
    records = []
    for index in range(5):
        hard = index < 4
        method = 'M2_GP_NO_OBSTACLE' if index < 2 else 'M3_GP_CONSTRAINED'
        initialization = 'I0_FRESH' if index % 2 == 0 else 'I1_DECEL'
        if not hard:
            initialization = 'I1_DECEL'
        velocity = np.column_stack([.2+zeros, (4e-5 if hard else 1e-7)*np.sin(2*np.pi*u), zeros])
        acceleration = np.column_stack([np.where(u == 0., 2.00002 if hard else .2, .1), zeros, zeros])
        samples = dict(times_s=times, poses_world=np.column_stack([.2*times, zeros, zeros]),
            body_twists=velocity, body_accelerations=acceleration,
            interval_index=interval, local_u=u)
        ext = []
        for quantity in ('vx', 'vy', 'omega', 'ax', 'alpha'):
            for i in range(30):
                ext.append(dict(grid='SUPPLEMENTAL_0.001_OFFSET', quantity=quantity, interval_index=i,
                    worst_time_s=(i+.25)*.1, maximum_tolerance_excess=(1e-5 if hard else -1e-7) if i in (2, 3) else -1.))
        detection = [dict(grid=grid, quantity=q, passed=(not hard or grid == 'G0_ORIGINAL' or q not in ('vy', 'ax')))
            for grid in ('G0_ORIGINAL', 'G1_QUARTERS', 'G2_WITNESS') for q in ('vx', 'vy', 'omega', 'ax', 'alpha')]
        record = dict(record_id=f'synthetic_{index}', role='hard_final' if hard else 'benign_final',
            method=method, initialization=initialization, available=True, samples=samples,
            acceptance=dict(collocation_feasible=True, dense_feasible=not hard, full_feasible=not hard),
            interval_extrema=ext, grid_detection=detection, vector_sha256=f'fixture_vector_{index}',
            source_record='SYNTHETIC_UNIT_FIXTURE_ONLY')
        if hard:
            seed = copy.deepcopy(record)
            seed.update(role='hard_initial', record_id=f'synthetic_initial_{index}')
            records.append(seed)
        records.append(record)
    context = dict(B_world=[0., 0., 0.], old_world=[[-.1, 0., 0.], [0., 0., 0.]],
        fresh_world=[[.1, 0., 0.], [.6, 0., 0.]], goal_world=[.6, 0., 0.])
    audit = p.plain(dict(records=records, contexts=dict(hard=context, benign=context),
        formulation_config=config, summary=dict(synthetic_only=True)))
    for name, value in [('source.json', {'synthetic_only': True}), ('protocol.json', {'synthetic_only': True}),
            ('record_manifest.json', {'count': 9}), ('audit.json', audit)]:
        (tmp_path / name).write_text(json.dumps(value))
    (tmp_path / 'config_snapshot.yaml').write_text(p.yaml.safe_dump(config))
    (tmp_path / 'aggregate').mkdir()
    (tmp_path / 'aggregate/summary.json').write_text('{"synthetic_only":true}')
    (tmp_path / 'aggregate/original_reproduction.csv').write_text('record,synthetic_only\nfixture,true\n')
    environment = SimpleNamespace(workspace=box(-1, -1, 2, 1), obstacles=box(.1, .5, .4, .8))
    return audit, environment


def test_representative_rule_per_quantity_earliest_interval_and_unavailable(tmp_path):
    audit, _ = fixture(tmp_path)
    rows = p.representative_intervals(audit)
    assert {r['quantity'] for r in rows} == set(p.QUANTITIES)
    assert {r['interval_index'] for r in rows} == {2}
    assert {r['record_id'] for r in rows} == {'synthetic_0'}
    for record in audit['records']:
        record['interval_extrema'] = []
    assert all(r['record_id'] is None for r in p.representative_intervals(audit))


def test_acceleration_sides_are_not_joined_and_signed_values_preserved():
    fig, ax = p.plt.subplots()
    p._line(ax, [0., .1, .1, .2], [-2., -1., 1., 2.], label='synthetic sides', color='black', interval_index=[0, 0, 1, 1])
    assert len(ax.lines) == 2
    assert list(ax.lines[0].get_ydata()) == [-2., -1.]
    assert list(ax.lines[1].get_ydata()) == [1., 2.]
    p.plt.close(fig)


def test_numeric_records_include_context_but_do_not_count_initials_as_finals(tmp_path):
    audit, _ = fixture(tmp_path)
    numeric = p.expected_numeric(audit)
    assert tuple(numeric) == p.PLOT_NAMES
    for values in numeric.values():
        assert len(values['records']) == 5
        assert len(values['initial_records_context']) == 4
        assert values['no_execution']
    assert numeric['world_xy_and_heading']['aspect'] == 'equal'
    assert numeric['grid_detection_summary']['detection_only']
    assert 'post-hoc' in numeric['grid_detection_summary']['G2_semantics']
    values = numeric['lateral_velocity_vs_time']['records'][0]['samples']['body_twists']
    assert np.max(np.asarray(values)[:, 1]) > 1e-5
    assert np.min(np.asarray(values)[:, 1]) < -1e-5


def test_all_figures_exclusive_hashes_links_bundle_and_no_new_queries(tmp_path, monkeypatch):
    audit, environment = fixture(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError('plotting must not query or optimize a GP')
    import reconciliation.gp_se2 as gp
    import reconciliation.gp_se2_formulation as formulation
    monkeypatch.setattr(gp, 'sample_gp', forbidden)
    monkeypatch.setattr(gp, 'interpolate_interval', forbidden)
    monkeypatch.setattr(formulation, 'minimize', forbidden)
    result = p.generate(tmp_path, audit=audit, environment=environment)
    assert result['plot_count'] == 7
    assert result['source_record_count'] == 9
    assert result['plotted_record_count'] == 5
    assert result['context_initial_record_count'] == 4
    expected = p.expected_numeric(audit)
    for row in result['images']:
        image = Path(row['path'])
        side = p.read(Path(row['sidecar']))
        assert side['numeric_data'] == expected[image.stem]
        assert side['image_sha256'] == row['sha256'] == p.digest(image)
        assert side['new_gp_or_rigid_optimization'] == side['new_mpc_solve'] == side['new_closed_loop_rollout'] == 0
        assert all(p.digest(path) == value for path, value in side['source_hashes'].items())
        with Image.open(image) as opened:
            assert min(opened.info['dpi']) >= 159.9
    for directory in (tmp_path, tmp_path / 'review_bundle'):
        index = (directory / 'index.html').read_text()
        assert index.count('<tr><td>') == 9
        assert 'Rejected GP iterate' in index
        for relative in re.findall(r'(?:href|src)="([^"]+)"', index):
            assert (directory / relative).is_file()
    with zipfile.ZipFile(tmp_path / 'review_bundle.zip') as z:
        assert z.testzip() is None
        assert sum(name.endswith('.png') for name in z.namelist()) == 7
        assert not any(Path(name).suffix in ('.npy', '.npz', '.pt', '.wkb', '.mp4', '.usd') for name in z.namelist())
        inventory = json.loads(z.read('manifest.json'))
        assert set(z.namelist()) == {r['path'] for r in inventory['files']} | {'manifest.json'}
    with pytest.raises(FileExistsError):
        p.generate(tmp_path, audit=audit, environment=environment)


def test_missing_saved_vector_retains_na_and_low_dpi_refused(tmp_path):
    audit, env = fixture(tmp_path)
    final = next(r for r in audit['records'] if r['role'] == 'hard_final')
    final.update(available=False, samples=None, acceptance=None, interval_extrema=[], grid_detection=[])
    values = p.expected_numeric(audit)['world_xy_and_heading']['records'][0]
    assert not values['available'] and values['samples'] is None
    with pytest.raises(ValueError, match='160 dpi'):
        p.generate(tmp_path, audit=audit, environment=env, dpi=90)
    assert not (tmp_path / 'plots').exists()
