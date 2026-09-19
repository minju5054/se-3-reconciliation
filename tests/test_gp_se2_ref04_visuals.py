"""Synthetic REF-04 plotting checks; fixtures are not experimental evidence."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('plot_gp_se2_ref04', ROOT / 'scripts/plot_gp_se2_ref04.py')
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def test_exclusive_numeric_sidecar_and_bounded_title(tmp_path):
    source = tmp_path / 'audit.json'
    source.write_text('{"synthetic_test_only":true}')
    fig, ax = p.plt.subplots(figsize=(10, 5))
    p._time(ax, 'saved values')
    assert ax.get_xlim() == (0., 3.)
    target = tmp_path / 'figure.png'
    title = 'An explicit synthetic test title which is long enough to require a line break without changing the recorded values or any original experimental evidence'
    rec = p._save(fig, target, {'missing': None, 'actual': [0., 1.]}, [source], title)
    side = json.loads(target.with_suffix('.json').read_text())
    assert side['numeric_data'] == {'missing': None, 'actual': [0., 1.]}
    assert side['source_hashes'][str(source.resolve())] == p.digest(source)
    assert side['image_sha256'] == rec['sha256'] == p.digest(target)
    assert not side['new_optimization'] and not side['new_rollout']
    assert '\n' in fig._suptitle.get_text()
    with pytest.raises(FileExistsError):
        p._save(fig, target, {}, [source], title)


def test_missing_xy_is_legend_only_not_fabricated_execution():
    fig, ax = p.plt.subplots()
    p._xy_line(ax, None, label='missing saved prediction', color='blue')
    assert len(ax.lines) == 1
    assert len(ax.lines[0].get_xdata()) == 0
    assert ax.lines[0].get_label().endswith('N/A')
    p.plt.close(fig)


def fixture(tmp_path):
    from copy import deepcopy
    from types import SimpleNamespace
    import numpy as np
    from shapely.geometry import box
    context = dict(B_world=[0., 0., 0.], old_world=[[-.1, 0., 0.], [0., 0., 0.]],
        fresh_world=[[.03, 0., 0.], [.3, .03, .2]], u_minus=[.1, .2], previous_control=[.2, .3])
    route = dict(goal_world=[.3, .03, .2], route_status='REQUIRED_ORDERED_GATES',
        gates=[dict(gate_id='SYNTHETIC_GATE_ONLY', center_xy=[.15, .0], normal_xy=[1., 0.], half_width_m=.2)])
    config = dict(footprint=dict(radius_m=.2, required_clearance_m=.05),
        formulation=dict(goal_position_tolerance=.15, goal_yaw_tolerance=.26))
    ref = np.array([[.03 * i, .003 * i, .02 * i] for i in range(10)])
    def geometry(points, times):
        points = np.asarray(points).tolist()
        return dict(points_world=points, times_s=times, node_clearance_m=[.06] * len(times), minimum_clearance_m=.059,
            effective_threshold_m=.0500001, first_sampled_violation_time_s=None,
            first_swept_violation_bracket_s=None, first_refined_violation_bracket_s=None,
            minimum_location_world=points[0][:2], minimum_time_s=times[0],
            segments=[dict(index=i, start_time_s=times[i], end_time_s=times[i+1], minimum_clearance_m=.059,
                minimum_location_world=points[i][:2], minimum_time_s=times[i], segment_fraction=.5,
                clearance_valid=True, physical_overlap=False, workspace_known=True) for i in range(len(times)-1)])
    audit = dict(representative_intervals=[dict(solve_index=0, reasons=['FIRST_SOLVE', 'SYNTHETIC_EARLY_WARNING']),
        dict(solve_index=3, reasons=['SYNTHETIC_MINIMUM'])], methods={}, timeline=[])
    for mi, name in enumerate(p.METHODS):
        solves = []
        for i in range(30):
            issue = i * .1; start = np.array([i * .01, 0., 0.])
            nodes = [start + [h * .01, 0., 0.] for h in range(6)]
            times = [issue + h * .1 for h in range(6)]
            g = geometry(nodes, times)
            actual = [start.tolist(), (start + [.01, .0001, .002]).tolist()]
            solves.append(dict(solve_index=i, issue_time_s=issue, input_pose_world=start.tolist(),
                selected_reference=dict(poses_world=ref[:5].tolist(), indices=list(range(5)), source_progress=list(range(5)),
                    entry_connector=geometry([start, ref[0]], [issue, issue])),
                prediction=dict(available=True, poses_world=np.asarray(nodes).tolist(), node_times_s=times, geometry=g,
                    first_interval=geometry(nodes[:2], times[:2]), tail=geometry(nodes[1:], times[1:]),
                    prospective_warning=False, start_status='VALID'),
                applied_prefix=dict(times_s=[issue, issue+.1], poses_world=actual,
                    geometry=geometry(actual, [issue, issue+.1]), saved_prediction_endpoint=np.asarray(nodes[1]).tolist(),
                    euler_endpoint=np.asarray(nodes[1]).tolist(), exact_endpoint=actual[-1], actual_endpoint=actual[-1],
                    model_discrepancy=dict(xy_m=.0001))))
        full = geometry([[i*.01, 0., 0.] for i in range(31)], [i*.1 for i in range(31)])
        if mi == 1:
            full['node_clearance_m'][3] = .04
            full['first_swept_violation_bracket_s'] = [.2, .3]
            full['first_refined_violation_bracket_s'] = [.26, .265]
        audit['methods'][name] = dict(per_solve=solves, full_execution=full,
            original_outcome=dict(primary_success=mi == 0, failure_reasons=[] if mi == 0 else ['CLEARANCE_VIOLATION']),
            gate_crossings=[dict(classification='VALID_CROSSING' if mi == 0 else 'INVALID_INTERVAL_CROSSING',
                crossing_time_s=.72 if mi == 0 else .4, location_world=[.15, .19 + .02 * mi],
                tangent_offset_m=.19+.02*mi, interval_margin_m=.01-.02*mi)])
        audit['timeline'].append(dict(method=name, event='synthetic first event', issue_time_s=0.,
            event_time_s=.3, event_time_bracket_s=[.2, .3], layer='prediction', interpretation='fixture only'))
    for name, value in [('source.json', {}), ('input_context.json', context), ('goal_route.json', route), ('audit.json', audit),
            ('protocol.json', {'synthetic_only': True}), ('data_availability.json', {}), ('original_outcome_recheck.json', {})]:
        (tmp_path / name).write_text(json.dumps(value))
    (tmp_path / 'aggregate').mkdir()
    (tmp_path / 'aggregate/summary.json').write_text('{}')
    (tmp_path / 'aggregate/example.csv').write_text('synthetic_only,value\ntrue,1\n')
    (tmp_path / 'config_snapshot.yaml').write_text(p.yaml.safe_dump(config))
    np.save(tmp_path / 'reference_world.npy', ref)
    env = SimpleNamespace(workspace=box(-1, -1, 2, 2), obstacles=box(.5, -.5, 1., -.3))
    return audit, context, route, ref, config, env


def test_numeric_layers_and_shared_representative_indices_are_preserved(tmp_path):
    audit, context, route, ref, config, _ = fixture(tmp_path)
    result = p.expected_numeric(audit, context, route, ref, config)
    assert tuple(result) == p.PLOT_NAMES
    data = result['selected_targets_and_predictions']
    assert not data['matched_input_pose_claim']
    assert data['representative_intervals'] == audit['representative_intervals']
    for name in p.METHODS:
        assert [s['solve_index'] for s in data['methods'][name]] == [0, 3]
        s = data['methods'][name][0]
        assert len(s['selected_reference']['poses_world']) == 5
        assert len(s['prediction']['poses_world']) == 6
        assert s['prediction']['poses_world'] != s['applied_prefix']['poses_world']
    assert result['gate_crossing_zoom']['no_second_footprint_subtraction']
    assert result['stress_world_overlay']['shared_dense_reference_world'] == ref.tolist()
    assert result['prediction_vs_applied_prefix']['future_optimized_controls_logged'] is False


def test_missing_prediction_is_nan_not_zero_and_no_representative_substitution(tmp_path):
    audit, context, route, ref, config, _ = fixture(tmp_path)
    s = audit['methods'][p.METHODS[1]]['per_solve'][0]
    s['prediction'] = dict(available=False, poses_world=None, geometry=None)
    out = p.expected_numeric(audit, context, route, ref, config)
    assert p._prediction_values(out['predicted_clearance_heatmap']['methods'][p.METHODS[1]][0], 'nodes') is None
    assert out['selected_targets_and_predictions']['methods'][p.METHODS[1]][0]['prediction']['poses_world'] is None
    audit['representative_intervals'].append(dict(solve_index=0, reasons=['duplicate']))
    with pytest.raises(ValueError, match='deduplicated'):
        p.expected_numeric(audit, context, route, ref, config)


def test_all_seven_renderers_index_and_small_allowlisted_bundle(tmp_path):
    import zipfile
    audit, context, route, ref, config, env = fixture(tmp_path)
    result = p.render(tmp_path, audit, context, route, ref, config, env)
    assert result['image_count'] == 7
    for row in result['images']:
        image = tmp_path / row['path']
        side = json.loads(image.with_suffix('.json').read_text())
        assert image.exists() and side['image_sha256'] == p.digest(image)
        assert side['numeric_data'] == p.plain(p.expected_numeric(audit, context, route, ref, config)[image.stem])
    index = (tmp_path / 'index.html').read_text()
    assert 'FAIL' in index and 'PASS' in index and 'CLEARANCE_VIOLATION' in index
    assert all(name + '.png' in index for name in p.PLOT_NAMES)
    (tmp_path / 'raw_rgb.png').write_bytes(b'forbidden packaging fixture')
    bundle = p.package(tmp_path, result['images'])
    with zipfile.ZipFile(tmp_path / 'review_bundle.zip') as z:
        assert len(z.namelist()) == bundle['file_count']
        assert not any(x.endswith('.npy') for x in z.namelist())
        assert 'raw_rgb.png' not in z.namelist()
        assert sum(x.endswith('.png') for x in z.namelist()) == 7
        assert {'protocol.json','data_availability.json','original_outcome_recheck.json','aggregate/summary.json','aggregate/example.csv'} <= set(z.namelist())
    with pytest.raises(FileExistsError):
        p.render(tmp_path, audit, context, route, ref, config, env)
    with pytest.raises(FileExistsError):
        p.package(tmp_path, result['images'])


def test_node_and_swept_clearance_are_different_explicit_layers(tmp_path):
    import numpy as np
    audit, _, _, _, _, _ = fixture(tmp_path)
    row = audit['methods'][p.METHODS[0]]['per_solve'][0]
    nodes = p._prediction_values(row, 'nodes')
    segments = p._prediction_values(row, 'segments')
    assert nodes.shape == (6,) and segments.shape == (5,)
    np.testing.assert_allclose(nodes, .06-.0500001)
    np.testing.assert_allclose(segments, .059-.0500001)
    assert nodes.min() > segments.min()


def test_main_loads_environment_export_and_passes_saved_past(tmp_path, monkeypatch):
    from types import SimpleNamespace
    audit, context, route, ref, config, env = fixture(tmp_path)
    export = tmp_path / 'validated_environment_export'
    (tmp_path / 'source.json').write_text(json.dumps(dict(environment_path=str(export))))
    past = dict(poses_world=[[-.1, 0., 0.], context['B_world']], times_relative_to_B_s=[-.1, 0.])
    (tmp_path / 'actual_past_execution.json').write_text(json.dumps(past))
    loaded = []
    def load(path):
        loaded.append(path)
        return env
    monkeypatch.setattr(p, 'HospitalEnvironment', SimpleNamespace(load=load))
    actual_render = p.render
    def render(*args, **kwargs):
        assert kwargs['past'] == past
        return actual_render(*args, **kwargs)
    monkeypatch.setattr(p, 'render', render)
    result = p.main(tmp_path)
    assert loaded == [export]
    assert result['plots']['image_count'] == result['review']['image_count'] == 7
    side = json.loads((tmp_path / 'plots/stress_world_overlay.json').read_text())
    assert side['numeric_data']['actual_past_world'] == past['poses_world']
    assert str(tmp_path / 'actual_past_execution.json') in side['source_hashes']
