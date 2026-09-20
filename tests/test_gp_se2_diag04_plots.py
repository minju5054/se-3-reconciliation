"""Synthetic rendering and provenance tests; not hard-handoff experiment evidence."""
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
SPEC = importlib.util.spec_from_file_location('diag04_plots', ROOT/'scripts/plot_gp_se2_diag04.py')
p = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(p)


def fixture(tmp_path):
    fractions = np.tile([0., .125, .25, .5, .75, .875, 1.], 30)
    intervals = np.repeat(np.arange(30), 7)
    times = (intervals+fractions)*.1
    zeros = np.zeros_like(times)
    cfg = dict(equality_tolerance=1e-5, inequality_tolerance=1e-5, v_max=.8, w_max=3.,
        a_v_max=2., a_w_max=5.)
    keys = [('HARD', method, seed) for method in ('M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')
        for seed in ('I0_FRESH', 'I1_DECEL')]+[('BENIGN_CONTROL', 'M3_GP_CONSTRAINED', 'I1_DECEL')]
    solves = []
    for index, key in enumerate(keys):
        for grid in p.GRIDS:
            full = key[0] == 'BENIGN_CONTROL'
            samples = dict(times_s=times, poses_world=np.column_stack([.2*times, zeros, zeros]),
                body_twists=np.column_stack([.2+zeros, (1e-7 if full else 4e-5)*np.sin(2*np.pi*fractions), zeros]),
                body_accelerations=np.column_stack([np.where(fractions == 0., 2.00002 if not full else .2, .1), zeros, zeros]),
                interval_index=intervals, local_u=fractions)
            result = dict(samples=samples, full_feasible=full, objective=12.+index,
                vector_sha256=f'synthetic_{index}_{grid}', source='latest')
            selected = copy.deepcopy(result) if full else None
            conditioning = {role: {g: dict(raw_singular_values=[2., .5, 1e-14, 0.], scaled_singular_values=[3., .4, 3e-15, 0.])
                for g in p.GRIDS} for role in ('initial', 'latest', 'selected')}
            if selected is None:
                conditioning['selected'] = None
            solves.append(dict(solve_id=f'fixture_{index}_{grid}', case_role=key[0], method=key[1], initialization=key[2],
                grid=grid, termination='CONVERGED' if grid == 'G0_ORIGINAL' else 'SINGULAR_LSQ',
                latest=result, selected=selected, history=[
                    dict(elapsed_s=0., objective=30., equality_residual=.1, inequality_violation=.4, full_feasible=full, source='initial'),
                    dict(elapsed_s=.3, objective=20., equality_residual=.01, inequality_violation=.04, full_feasible=None, source='callback_0'),
                    dict(elapsed_s=.5, objective=result['objective'], equality_residual=1e-7, inequality_violation=0., full_feasible=full, source='latest')],
                timing=dict(prepared_solve_s=.5, compile_warmup_s=1., candidate_postcheck_s=.2, rank_diagnostics_s=.1),
                counts=dict(objective_calls=5, constraint_calls=10, derivative_calls=6, primal_cache_misses=4), conditioning=conditioning))
    context = dict(B_world=[0., 0., 0.], old_world=[[-.1, 0., 0.], [0., 0., 0.]],
        fresh_world=[[.1, 0., 0.], [.6, 0., 0.]], goal_world=[.6, 0., 0.])
    audit = p.plain(dict(solves=solves, contexts={key: context for key in ('HARD', 'BENIGN_CONTROL')},
        formulation_config=cfg, summary=dict(synthetic_only=True)))
    for name, value in [('source.json', {'synthetic_only': True}), ('protocol.json', {'synthetic_only': True}),
            ('experiment_manifest.json', {'planned_solves': 10}), ('plot_input.json', audit)]:
        (tmp_path/name).write_text(json.dumps(value))
    (tmp_path/'config_snapshot.yaml').write_text('synthetic_only: true\n')
    (tmp_path/'aggregate').mkdir()
    (tmp_path/'aggregate/summary.json').write_text('{"synthetic_only":true}')
    (tmp_path/'aggregate/all_starts.csv').write_text('solve_id,synthetic_only\nfixture,true\n')
    environment = SimpleNamespace(workspace=box(-1, -1, 2, 1), obstacles=box(.1, .5, .4, .8))
    return audit, environment


def test_expected_numeric_full_ledger_and_exact_curves(tmp_path):
    audit, _ = fixture(tmp_path)
    numeric = p.expected_numeric(audit)
    assert tuple(numeric) == p.PLOT_NAMES
    for value in numeric.values():
        assert len(value['solves']) == 10
        assert value['new_execution'] is False
        assert value['finite_grid_is_continuous_proof'] is False
    assert numeric['lateral_velocity_vs_time']['solves'][0]['latest']['samples']['body_twists'] == audit['solves'][0]['latest']['samples']['body_twists']
    assert numeric['world_xy_and_heading']['aspect'] == 'equal'
    assert numeric['world_xy_and_heading']['no_coordinate_offsets']
    audit['solves'].pop()
    with pytest.raises(ValueError, match='ten planned'):
        p.expected_numeric(audit)


def test_identical_latest_selected_is_one_curve_and_na_full_history(tmp_path):
    audit, _ = fixture(tmp_path)
    benign = audit['solves'][-1]
    assert len(list(p.visible_results(benign))) == 1
    assert list(p.visible_results(benign))[0][0] == 'latest = selected'
    benign['selected']['vector_sha256'] = 'different_synthetic_vector'
    assert len(list(p.visible_results(benign))) == 2
    hard = audit['solves'][0]
    assert p.full_history(hard)['best_full_feasible_objective'] == [None, None, None]
    assert p.full_history(benign)['best_full_feasible_objective'] == [30., 30., 16.]
    hard['history'][1]['full_feasible'] = True
    assert p.full_history(hard)['best_full_feasible_objective'] == [None, 20., 20.]


def test_worst_interval_uses_new_result_unitwise_and_earliest_tie(tmp_path):
    audit, _ = fixture(tmp_path)
    rows = p.representatives(audit)
    assert len(rows) == 50
    assert {r['quantity'] for r in rows} == set(p.QUANTITIES)
    assert {r['interval_index'] for r in rows} == {0}
    record = audit['solves'][0]['latest']['samples']
    k = record['interval_index'].index(20)+3
    record['body_twists'][k][1] = .0009
    selected = [r for r in p.representatives(audit) if r['solve_id'] == audit['solves'][0]['solve_id'] and r['quantity'] == 'v_y'][0]
    assert selected['interval_index'] == 20
    assert selected['maximum_tolerance_excess'] == pytest.approx(.00089)
    assert selected['units'] == 'm/s'


def test_all_figures_source_hashes_bundle_numbers_and_no_queries(tmp_path, monkeypatch):
    audit, environment = fixture(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError('plotting must not query or optimize GP')
    import reconciliation.gp_se2 as gp
    import reconciliation.gp_se2_formulation as formulation
    monkeypatch.setattr(gp, 'sample_gp', forbidden)
    monkeypatch.setattr(gp, 'interpolate_interval', forbidden)
    monkeypatch.setattr(formulation, 'minimize', forbidden)
    result = p.generate(tmp_path, audit=audit, environment=environment)
    assert result['plot_count'] == 10 and result['source_solve_count'] == 10
    assert result['new_queries_from_plotter'] == result['new_solves_from_plotter'] == result['new_execution'] == 0
    expected = p.expected_numeric(audit)
    for row in result['images']:
        path = Path(row['path'])
        sidecar = p.read(Path(row['sidecar']))
        assert sidecar['numeric_data'] == expected[path.stem]
        assert sidecar['image_sha256'] == p.digest(path)
        assert all(p.digest(source) == sha for source, sha in sidecar['source_hashes'].items())
        with Image.open(path) as opened:
            assert min(opened.info['dpi']) >= 159.9
    for root in (tmp_path, tmp_path/'review_bundle'):
        index = (root/'index.html').read_text()
        assert index.count('<tr><td>') == 10
        assert 'Rejected / not executed' in index
        assert 'callback discovery time' in index
        for link in re.findall(r'(?:href|src)="([^"]+)"', index):
            assert (root/link).is_file()
    with zipfile.ZipFile(tmp_path/'review_bundle.zip') as archive:
        assert archive.testzip() is None
        assert sum(name.endswith('.png') for name in archive.namelist()) == 10
        assert not any(Path(name).suffix in ('.npy', '.npz', '.wkb', '.pt', '.mp4', '.usd') for name in archive.namelist())
        manifest = json.loads(archive.read('manifest.json'))
        assert set(archive.namelist()) == {r['path'] for r in manifest['files']} | {'manifest.json'}
    with pytest.raises(FileExistsError):
        p.generate(tmp_path, audit=audit, environment=environment)


def test_missing_result_keeps_na_and_low_resolution_refused(tmp_path):
    audit, environment = fixture(tmp_path)
    audit['solves'][0].update(latest=None, selected=None, history=[], termination='DERIVATIVE_UNSUPPORTED')
    numeric = p.expected_numeric(audit)
    assert numeric['world_xy_and_heading']['solves'][0]['latest'] is None
    assert list(p.visible_results(audit['solves'][0])) == []
    assert p.full_history(audit['solves'][0])['elapsed_s'] == []
    with pytest.raises(ValueError, match='160 dpi'):
        p.generate(tmp_path, audit=audit, environment=environment, dpi=100)
    assert not (tmp_path/'plots').exists()
