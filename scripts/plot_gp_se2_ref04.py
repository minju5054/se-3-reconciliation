#!/usr/bin/env python3
"""Plot a saved REF-04 prediction/execution audit without performing solves."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import sys
import textwrap
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_02 import base_xy, environment_path, plain, read, write
from plot_gp_se2_ref02 import square_bounds
from plot_gp_se2_ref03 import gate_geometry, _gates

LABEL = 'SAVED MPC PREDICTION / EXECUTION AUDIT'
METHODS = ('B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS')
SHORT = dict(zip(METHODS, ('B · dense row step', 'C · source progress')))
COLORS = dict(zip(METHODS, ('#d55e00', '#0072b2')))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _save(fig, target, numeric, sources, title):
    """Exclusive figure writes, bounded title width, and exact numeric sidecars."""
    target = Path(target)
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError('refusing plot overwrite')
    fig.suptitle(textwrap.fill(title, width=86), fontsize=12)
    fig.text(.015, .014, LABEL + ' | saved records only | adjacent JSON: numbers and hashes', fontsize=8)
    fig.tight_layout(rect=(0, .055, 1, .91))
    fig.savefig(target, dpi=140)
    plt.close(fig)
    write(target.with_suffix('.json'), dict(
        image=target.name, image_sha256=digest(target), label=LABEL,
        numeric_data=plain(numeric), source_hashes={str(Path(p).resolve()): digest(p) for p in sources},
        plotter_sha256=digest(__file__),
        new_inference=False, new_optimization=False, new_rollout=False, gui_runtime_validated=False,
        semantics='selected reference, saved MPC prediction, applied prefix and full execution are distinct',
        missing_values='null, never fabricated or replaced by zero'))
    return dict(path=str(target), sha256=digest(target), sidecar_sha256=digest(target.with_suffix('.json')))


def _time(ax, ylabel, limit=(0., 3.)):
    ax.set(xlabel='time after original B [s]', ylabel=ylabel, xlim=limit)
    ax.grid(alpha=.2)


def _xy_line(ax, poses, *, label, color, style='-', width=1.8, marker=None):
    if poses is None or len(poses) == 0:
        ax.plot([], [], style, color=color, label=label + ' N/A')
        return
    p = np.asarray(poses, dtype=float)
    ax.plot(p[:, 0], p[:, 1], style, color=color, lw=width, marker=marker, ms=4, label=label)


def _outside_legend(ax):
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.))


def write_index(run, images, outcome_rows):
    """A complete static index; outcomes are already audited, not recomputed here."""
    run = Path(run)
    target = run / 'index.html'
    if target.exists():
        raise FileExistsError('refusing index overwrite')
    lines = ['<!doctype html><html><head><meta charset="utf-8"><title>REF-04 saved audit</title>',
        '<style>body{font:16px system-ui;max-width:1500px;margin:25px auto;padding:15px}img{max-width:100%}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #bbb}article{margin:35px 0}</style></head><body>',
        '<h1>Saved MPC prediction / execution audit</h1>',
        '<p>One previously saved obstacle stress event. B and C use the same dense reference geometry. '
        'Selected reference targets, saved MPC predictions, actually applied 0.1 s prefixes and full saved execution are distinct. '
        'No new MPC solve, GP optimization, rollout, inference or GUI runtime is performed.</p>',
        '<h2>Original outcomes</h2><table><tr><th>Method</th><th>Original result</th><th>Reasons</th></tr>']
    for row in outcome_rows:
        result = row['success']
        label = 'N/A' if result is None else ('PASS' if result else 'FAIL')
        lines.append('<tr><td>' + html.escape(row['method']) + '</td><td>' + label + '</td><td>' + html.escape(', '.join(row.get('failure_reasons') or [])) + '</td></tr>')
    lines += ['</table>', '<p>The existing official MPC has no obstacle or gate constraint. An unsafe prediction '
        'is therefore evidence about this controller/reference combination, not a violated MPC obstacle constraint. '
        'Prediction polylines use forward Euler states; actual motion uses exact held-command unicycle integration. '
        'Refined violation brackets are diagnostic and do not change original acceptance.</p>',
        '<p><a href="audit.json">Full numeric audit</a> · <a href="source.json">Sources and hashes</a></p>']
    lines.append('<h2>Protocol and core tables</h2><ul>')
    for path in review_metadata(run):
        if path.name not in ('index.html', 'plot_manifest.json', 'audit.json'):
            lines.append(f'<li><a href="{html.escape(str(path))}">{html.escape(str(path))}</a></li>')
    lines.append('</ul>')
    for row in images:
        relative = str(Path(row['path']).relative_to(run)) if Path(row['path']).is_absolute() else row['path']
        label = Path(relative).stem.replace('_', ' ')
        lines.append(f'<article><h2>{html.escape(label)}</h2><img loading="lazy" src="{html.escape(relative)}"><p><a href="{html.escape(str(Path(relative).with_suffix(".json")))}">Exact plotted values and source hashes</a></p></article>')
    target.write_text('\n'.join(lines + ['</body></html>']))


def review_metadata(run):
    run = Path(run)
    names = [Path(name) for name in ('index.html', 'source.json', 'config_snapshot.yaml', 'audit.json',
        'plot_manifest.json', 'protocol.json', 'data_availability.json', 'original_outcome_recheck.json',
        'aggregate/summary.json')]
    names += [p.relative_to(run) for p in sorted((run / 'aggregate').glob('*.csv'))]
    return names


def package(run, images):
    """Small explicit allowlist; original arrays/environment/external files stay out."""
    run = Path(run).resolve()
    bundle, archive = run / 'review_bundle', run / 'review_bundle.zip'
    if bundle.exists() or archive.exists() or (run / 'review_bundle_manifest.json').exists():
        raise FileExistsError('refusing bundle overwrite')
    names = review_metadata(run)
    for row in images:
        path = Path(row['path'])
        if path.is_absolute():
            path = path.relative_to(run)
        names += [path, path.with_suffix('.json')]
    if len(names) != len(set(names)) or any(p.is_absolute() or '..' in p.parts for p in names):
        raise ValueError('unsafe or duplicate review allowlist')
    for path in names:
        if not (run / path).is_file():
            raise FileNotFoundError(run / path)
    bundle.mkdir()
    entries = []
    for path in names:
        destination = bundle / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run / path, destination)
        entries.append(dict(path=str(path), sha256=digest(destination), bytes=destination.stat().st_size))
    (bundle / 'README.md').write_text('''# REF-04 saved MPC prediction / execution audit

Open index.html. This package contains a saved-data diagnostic for the fixed obstacle stress event. No new controller solve, trajectory optimizer, state integration rollout or inference was executed. Prediction nodes are not actual execution; only the recorded first command was applied before each next solve. The original safety and route failures remain failures. Refined brackets are separate diagnostics. The full 0.5 s saved prediction is forward Euler, while applied execution uses exact held commands. Comparisons beyond the saved 3 s execution remain unavailable.

Every PNG has a numerical/source-hash sidecar. Only explicit derived audit metadata, plots and their numbers are included. Original binary arrays, RGB, full environment export, external source, checkpoint and caches are excluded. No GUI validation or general navigation improvement is claimed.
''')
    entries.append(dict(path='README.md', sha256=digest(bundle / 'README.md'), bytes=(bundle / 'README.md').stat().st_size))
    write(bundle / 'manifest.json', dict(files=entries, self_excluded='manifest.json', image_count=len(images), new_solves=0, new_rollouts=0))
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as output:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():
                output.write(path, str(path.relative_to(bundle)))
    result = dict(bundle_sha256=digest(archive), bundle_bytes=archive.stat().st_size, image_count=len(images),
        file_count=len(entries) + 1, plotter_sha256=digest(__file__), gui_runtime_validated=False)
    write(run / 'review_bundle_manifest.json', result)
    return result


PLOT_NAMES = ('stress_world_overlay', 'selected_targets_and_predictions',
    'predicted_clearance_heatmap', 'executed_clearance_timeline', 'gate_crossing_zoom',
    'prediction_vs_applied_prefix', 'failure_timeline')
TITLES = dict(zip(PLOT_NAMES, (
    'Shared input and saved B/C execution', 'Selected targets, predictions and applied prefixes',
    'Saved prediction clearance: nodes and swept segments', 'Saved execution clearance and first violation brackets',
    'Original directed gate and actual crossing geometry', 'First interval: saved prediction versus applied motion',
    'Issue times, forecast warnings and actual failures')))


def expected_numeric(audit, context, route, reference, config, *, past=None):
    """Export only saved audit quantities; never choose snapshots from image appearance."""
    reps = audit['representative_intervals']
    indices = [r['solve_index'] for r in reps if r.get('solve_index') is not None]
    if len(indices) != len(set(indices)):
        raise ValueError('representative issue indices must be deduplicated by the audit')
    geometry = dict(B_world=context['B_world'], goal_world=route['goal_world'],
        old_world=context['old_world'], fresh_world=context['fresh_world'],
        actual_past_world=None if past is None else past['poses_world'],
        actual_past_times_relative_to_B_s=None if past is None else past['times_relative_to_B_s'],
        shared_dense_reference_world=np.asarray(reference).tolist(),
        required_gates=gate_geometry(route), footprint_radius_m=config['footprint']['radius_m'],
        required_clearance_m=config['footprint']['required_clearance_m'],
        original_route=route, methods={})
    representatives = dict(representative_intervals=reps,
        representative_unavailable=audit.get('representative_unavailable', []),
        same_absolute_issue_indices_for_B_C=True,
        matched_input_pose_claim=False,
        note='Same issue times, distinct recorded closed-loop input poses; these are not matched-state probes',
        methods={})
    heatmap, actual, crossings, outcomes = {}, {}, {}, {}
    for name in METHODS:
        item = audit['methods'][name]
        by_index = {s['solve_index']: s for s in item['per_solve']}
        if len(by_index) != len(item['per_solve']):
            raise ValueError('duplicate solve indices in audit')
        representatives['methods'][name] = [by_index.get(i) for i in indices]
        if any(r is None for r in representatives['methods'][name]):
            raise ValueError('audit representative index absent from method evidence')
        geometry['methods'][name] = dict(full_execution=item['full_execution'], original_outcome=item['original_outcome'])
        heatmap[name] = [dict(solve_index=s['solve_index'], issue_time_s=s['issue_time_s'],
            prediction=s['prediction']) for s in item['per_solve']]
        actual[name] = dict(geometry=item['full_execution'], original_outcome=item['original_outcome'])
        crossings[name] = item['gate_crossings']
        outcomes[name] = item['original_outcome']
    common = dict(required_clearance_m=config['footprint']['required_clearance_m'],
        unchanged_original_acceptance=True)
    return dict(stress_world_overlay=geometry,
        selected_targets_and_predictions=dict(geometry=geometry, **representatives),
        predicted_clearance_heatmap=dict(methods=heatmap, **common,
            node_and_swept_segment_values_distinct=True, first_interval_s=.1, prediction_horizon_s=.5,
            missing_prediction_is_NA=True),
        executed_clearance_timeline=dict(methods=actual, **common, full_time_range_s=[0., 3.]),
        gate_crossing_zoom=dict(geometry=geometry, methods=crossings, original_outcomes=outcomes,
            gate_half_width_already_safe_center_interval=True, no_second_footprint_subtraction=True),
        prediction_vs_applied_prefix=dict(**representatives,
            saved_prediction='forward Euler MPC nodes, not execution',
            applied_prefix='recorded first held command, exact unicycle integration',
            future_optimized_controls_logged=False),
        failure_timeline=dict(events=audit['timeline'], original_outcomes=outcomes,
            issue_and_event_times_are_distinct=True, full_time_range_s=[0., 3.]))


def _background(ax, env, bounds):
    from shapely.geometry import box
    from plot_gp_se2_02 import geometry as draw_geometry
    clip = box(bounds[0], bounds[2], bounds[1], bounds[3])
    ax.set_facecolor('#eee9e2')
    draw_geometry(ax, env.workspace.intersection(clip), facecolor='#f5faf2', edgecolor='#899b7a')
    draw_geometry(ax, env.obstacles.intersection(clip), facecolor='#a3a3a3', edgecolor='#555555')
    ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='world x [m]', ylabel='world y [m]', aspect='equal')
    ax.grid(alpha=.2)


def _world(ax, env, data, bounds, *, footprint=True):
    from matplotlib.patches import Circle
    _background(ax, env, bounds)
    _gates(ax, data['required_gates'])
    _xy_line(ax, data['old_world'], label='original OLD prediction', color='#2c7fb8', style='--', width=1.)
    _xy_line(ax, data['fresh_world'], label='original FRESH prediction', color='#b32183', style='--', width=1.)
    _xy_line(ax, data['actual_past_world'], label='recorded actual past through B', color='#777777', width=1.)
    ref = data['shared_dense_reference_world']
    _xy_line(ax, ref, label='B/C shared dense reference', color='#984ea3', style=':', marker='.')
    boundary, goal = np.asarray(data['B_world']), np.asarray(data['goal_world'])
    ax.scatter(*boundary[:2], s=35, color='black', label='original B', zorder=7)
    ax.scatter(*goal[:2], s=90, color='#238b45', marker='*', label='original goal', zorder=7)
    ax.arrow(*boundary[:2], *.14 * np.array([np.cos(boundary[2]), np.sin(boundary[2])]),
        color='black', width=.002, head_width=.025, length_includes_head=True)
    if footprint:
        for radius, style in ((data['footprint_radius_m'], '-'),
                (data['footprint_radius_m'] + data['required_clearance_m'], ':')):
            ax.add_patch(Circle(boundary[:2], radius, fill=False, color='black', ls=style, lw=.8))
    for name, row in data['methods'].items():
        _xy_line(ax, row['full_execution'].get('points_world'), label=SHORT[name] + ' actual', color=COLORS[name],
            style='--' if name == METHODS[0] else '-', width=2.)


def _bounds(parts, *, padding=.12, minimum_span=.3):
    valid = [p for p in parts if p is not None and len(p)]
    if not valid:
        raise ValueError('no saved world points available')
    return square_bounds(valid, padding=padding, minimum_span=minimum_span)


def _prediction_values(row, kind):
    prediction = row['prediction']
    if not prediction.get('available', False):
        return None
    geometry = prediction.get('geometry')
    if geometry is None:
        return None
    threshold = geometry.get('effective_threshold_m')
    if threshold is None:
        return None
    if kind == 'nodes':
        values = geometry.get('node_clearance_m')
    else:
        values = [s.get('minimum_clearance_m') for s in geometry.get('segments', [])]
    if values is None or any(v is None for v in values):
        return None
    return np.asarray(values, dtype=float) - threshold


def _summary(row):
    if 'summary' in row:
        return row['summary']
    return row


def render(run, audit, context, route, reference, config, environment, *, past=None):
    """Render all seven audit views once; every curve comes from audit.json."""
    run = Path(run).resolve()
    out = run / 'plots'
    if out.exists() or (run / 'plot_manifest.json').exists() or (run / 'index.html').exists():
        raise FileExistsError('refusing plot/index overwrite')
    data = expected_numeric(audit, context, route, reference, config, past=past)
    sources = [run / p for p in ('source.json', 'config_snapshot.yaml', 'input_context.json',
        'goal_route.json', 'reference_world.npy', 'audit.json')]
    if past is not None:
        sources.append(run / 'actual_past_execution.json')
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError(source)
    out.mkdir()
    images = []
    def save(name, fig):
        record = _save(fig, out / (name + '.png'), data[name], sources, TITLES[name])
        record['path'] = str(Path(record['path']).relative_to(run))
        images.append(record)
    xy = data['stress_world_overlay']
    paths = [reference, [context['B_world'], route['goal_world']]]
    paths += [m['full_execution']['points_world'] for m in xy['methods'].values()]
    local = _bounds(paths, padding=.25, minimum_span=1.2)
    complete = _bounds(paths + [g['segment_world_xy'] for g in xy['required_gates']], padding=.25)
    if past is not None:
        complete = _bounds(paths + [context['old_world'], context['fresh_world'], past['poses_world']]
            + [g['segment_world_xy'] for g in xy['required_gates']], padding=.25)
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    for ax, bounds, title in zip(axes, (complete, local), ('Original gate extent', 'Local B/goal execution detail')):
        _world(ax, environment, xy, bounds)
        ax.set_title(title, fontsize=11)
    _outside_legend(axes[1])
    save('stress_world_overlay', fig)

    representative = data['selected_targets_and_predictions']
    valid_reps = [r for r in representative['representative_intervals'] if r.get('solve_index') is not None]
    fig, axes = plt.subplots(max(1, len(valid_reps)), 2, figsize=(14, 4.8 * max(1, len(valid_reps))), squeeze=False)
    for ri, rep in enumerate(valid_reps):
        pair = [representative['methods'][name][ri] for name in METHODS]
        bounds = _bounds(paths + [s['prediction'].get('poses_world') for s in pair], padding=.2, minimum_span=1.)
        for mi, (name, solve) in enumerate(zip(METHODS, pair)):
            ax = axes[ri, mi]
            _background(ax, environment, bounds)
            _gates(ax, xy['required_gates'])
            _xy_line(ax, reference, label='same stored dense path', color='#aaaaaa', style=':')
            _xy_line(ax, solve['selected_reference'].get('poses_world'), label='five selected targets', color='#984ea3', style=':', marker='o')
            connector = solve['selected_reference'].get('entry_connector')
            _xy_line(ax, None if connector is None else connector.get('points_world'),
                label='current-to-first target (diagnostic connector)', color='#a6761d', style=':', width=1.8)
            _xy_line(ax, solve['prediction'].get('poses_world') if solve['prediction'].get('available') else None,
                label='saved MPC prediction', color=COLORS[name], style='--', marker='.')
            _xy_line(ax, solve['applied_prefix'].get('poses_world'), label='actual applied 0.1 s prefix', color='black', width=3.)
            ax.scatter(*solve['input_pose_world'][:2], color='black', s=25, marker='x', label='actual solve input')
            ax.set_title(f'{SHORT[name]} | solve {rep["solve_index"]}, t={solve["issue_time_s"]:.1f} s', fontsize=10)
            if ri == 0:
                ax.legend(fontsize=7, loc='best')
        axes[ri, 0].text(.01, -.23, '\n'.join(textwrap.wrap('; '.join(rep.get('reasons', [])), 55)), transform=axes[ri, 0].transAxes, fontsize=8)
    if not valid_reps:
        for ax in axes.flat:
            ax.text(.5, .5, 'N/A: no representative interval', ha='center'); ax.set_axis_off()
    save('selected_targets_and_predictions', fig)

    from matplotlib.colors import TwoSlopeNorm
    fig, axes = plt.subplots(2, 2, figsize=(13, 11), squeeze=False)
    values = [v for name in METHODS for row in data['predicted_clearance_heatmap']['methods'][name]
        for kind in ('nodes', 'segments') if (v := _prediction_values(row, kind)) is not None]
    values = np.concatenate(values) if values else np.array([-.01, .01])
    norm = TwoSlopeNorm(vmin=min(float(values.min()), -.01), vcenter=0., vmax=max(float(values.max()), .01))
    cmap = plt.get_cmap('RdYlGn').with_extremes(bad='#cccccc')
    for mi, name in enumerate(METHODS):
        rows = data['predicted_clearance_heatmap']['methods'][name]
        for ki, kind in enumerate(('nodes', 'segments')):
            ax = axes[mi, ki]
            size = 6 if kind == 'nodes' else 5
            matrix = np.full((len(rows), size), np.nan)
            for ri, row in enumerate(rows):
                v = _prediction_values(row, kind)
                if v is not None:
                    if len(v) != size:
                        raise ValueError('unexpected saved MPC prediction length')
                    matrix[ri] = v
            extent = (-.05, .55, 2.95, -.05) if kind == 'nodes' else (0., .5, 2.95, -.05)
            im = ax.imshow(np.ma.masked_invalid(matrix), extent=extent, aspect='auto', cmap=cmap, norm=norm, interpolation='nearest')
            ax.axvline(.1, ls='--', color='black', lw=1.1)
            ax.set(xlabel='offset from prediction issue [s]', ylabel='saved prediction issue time [s]',
                title=SHORT[name] + (' | node clearance' if kind == 'nodes' else ' | swept segment minimum'))
            ax.text(.02, -.14, 'First 0.1 s | later prediction tail; grey = N/A', transform=ax.transAxes, fontsize=8)
            fig.colorbar(im, ax=ax, label='clearance − effective threshold [m]', shrink=.85)
    save('predicted_clearance_heatmap', fig)

    fig, ax = plt.subplots(figsize=(12, 6))
    for name, item in data['executed_clearance_timeline']['methods'].items():
        geom = item['geometry']
        times, clearance = geom.get('times_s'), geom.get('node_clearance_m')
        if times is None or clearance is None:
            ax.plot([], [], color=COLORS[name], label=SHORT[name] + ' N/A')
        else:
            ax.plot(times, clearance, color=COLORS[name], label=SHORT[name] + ' actual sampled clearance')
        threshold = geom.get('effective_threshold_m')
        if threshold is not None:
            ax.axhline(threshold, color=COLORS[name], ls='--', lw=1., label=SHORT[name] + ' effective threshold')
        for key, style in (('first_swept_violation_bracket_s', ':'), ('first_refined_violation_bracket_s', '-')):
            bracket = geom.get(key)
            if bracket is not None:
                for bound in bracket:
                    ax.axvline(bound, color=COLORS[name], ls=style, lw=.9)
                ax.axvspan(*bracket, color=COLORS[name], alpha=.15,
                    label=SHORT[name] + (' first swept interval' if style == ':' else ' refined diagnostic bracket'))
    ax.axhline(config['footprint']['required_clearance_m'], color='red', ls=':', label='nominal required clearance')
    ax.axhline(0., color='#555555', lw=.7)
    _time(ax, 'footprint edge clearance [m]')
    ax.legend(fontsize=7, ncol=2)
    save('executed_clearance_timeline', fig)

    from matplotlib.patches import Circle
    gate_data = data['gate_crossing_zoom']
    cross_points = [c['location_world'] for cs in gate_data['methods'].values() for c in cs if c.get('location_world') is not None]
    near_endpoints = []
    for gate in xy['required_gates']:
        endpoints = gate['segment_world_xy']
        near_endpoints.append(min(endpoints, key=lambda q: np.linalg.norm(np.asarray(q) - np.asarray(context['B_world'][:2]))))
    bounds = _bounds([cross_points, near_endpoints, [context['B_world']]], padding=.3, minimum_span=.7)
    fig, ax = plt.subplots(figsize=(12, 8))
    _world(ax, environment, xy, bounds, footprint=False)
    for gate in xy['required_gates']:
        end = min(gate['segment_world_xy'], key=lambda q: np.linalg.norm(np.asarray(q) - np.asarray(context['B_world'][:2])))
        normal = np.asarray(gate['normal_world_xy'])
        ax.arrow(*end, *(normal * .12), width=.002, head_width=.02, color='#a65628', length_includes_head=True)
        ax.scatter(*end, s=45, marker='|', color='#a65628', label='original safe-center interval endpoint')
    for name, crossings in gate_data['methods'].items():
        for crossing in crossings:
            point = crossing.get('location_world')
            if point is None:
                continue
            valid = crossing['classification'] == 'VALID_CROSSING'
            ax.scatter(*point, s=65, color=COLORS[name], marker='o' if valid else 'x', zorder=9,
                label=SHORT[name] + ' ' + crossing['classification'])
            ax.add_patch(Circle(point, config['footprint']['radius_m'], fill=False, color=COLORS[name], lw=1.1, alpha=.7))
    _outside_legend(ax)
    save('gate_crossing_zoom', fig)

    prefix_data = data['prediction_vs_applied_prefix']
    fig, axes = plt.subplots(max(1, len(valid_reps)), 2, figsize=(13, 4.5 * max(1, len(valid_reps))), squeeze=False)
    for ri, rep in enumerate(valid_reps):
        pair = [prefix_data['methods'][name][ri] for name in METHODS]
        pieces = [s['applied_prefix'].get('poses_world') for s in pair]
        pieces += [[s['input_pose_world'], s['applied_prefix'].get('saved_prediction_endpoint')]
            for s in pair if s['applied_prefix'].get('saved_prediction_endpoint') is not None]
        bounds = _bounds(pieces, padding=.012, minimum_span=.05)
        for mi, (name, solve) in enumerate(zip(METHODS, pair)):
            ax = axes[ri, mi]; pre = solve['applied_prefix']
            _background(ax, environment, bounds)
            first = solve['input_pose_world']
            endpoint = pre.get('saved_prediction_endpoint')
            _xy_line(ax, [first, endpoint] if endpoint is not None else None,
                label='saved prediction first Euler segment', color=COLORS[name], style='--')
            _xy_line(ax, pre.get('poses_world'), label='saved actual applied prefix', color='black', width=2.2)
            for key, marker, color, label in (
                    ('euler_endpoint', 's', '#984ea3', 'Euler endpoint from applied command'),
                    ('exact_endpoint', '+', '#238b45', 'exact held-command endpoint'),
                    ('actual_endpoint', 'x', 'black', 'saved actual endpoint')):
                point = pre.get(key)
                if point is not None:
                    ax.scatter(*point[:2], s=60, marker=marker, color=color, facecolors='none' if marker == 's' else color, label=label, zorder=8)
            ax.set_title(f'{SHORT[name]} | solve {rep["solve_index"]}, first 0.1 s', fontsize=10)
            if ri == 0:
                ax.legend(fontsize=7, loc='best')
    if not valid_reps:
        for ax in axes.flat:
            ax.text(.5, .5, 'N/A: no saved prefix', ha='center'); ax.set_axis_off()
    save('prediction_vs_applied_prefix', fig)

    events = data['failure_timeline']['events']
    fig, ax = plt.subplots(figsize=(13, max(5., .6 * len(events) + 2.)))
    labels = []
    for i, event in enumerate(events):
        name = event.get('method')
        color = COLORS.get(name, '#333333')
        issue, when = event.get('issue_time_s'), event.get('event_time_s')
        bracket = event.get('event_time_bracket_s')
        if issue is not None:
            ax.scatter([issue], [i], marker='o', facecolors='none', edgecolors=color, s=45)
        if when is not None:
            ax.scatter([when], [i], marker='D', color=color, s=35)
        if issue is not None and when is not None:
            ax.plot([issue, when], [i, i], ':', color=color)
        if bracket is not None:
            ax.plot(bracket, [i, i], color=color, lw=4, alpha=.55)
        label = (SHORT.get(name, str(name) if name else 'B/C') + ' [' + event.get('layer', '') + '] | ' + event['event']).replace('_', ' ')
        if issue is None and when is None and bracket is None:
            label += ' [N/A]'
        labels.append('\n'.join(textwrap.wrap(label, 48)))
    ax.set_yticks(range(len(events)), labels, fontsize=8)
    ax.invert_yaxis(); _time(ax, '')
    ax.plot([], [], 'o', markerfacecolor='none', color='#333333', label='issue / observed difference time')
    ax.plot([], [], 'D', color='#333333', label='forecast or actual event time; see row layer')
    ax.plot([], [], '-', color='#333333', lw=4, alpha=.55, label='saved/refined event bracket')
    ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(.5, 1.13), ncol=3)
    save('failure_timeline', fig)

    manifest = dict(images=images, image_count=len(images), plot_names=list(PLOT_NAMES),
        representative_intervals=audit['representative_intervals'], plotter_sha256=digest(__file__),
        new_solves=0, new_rollouts=0, gui_runtime_validated=False)
    write(run / 'plot_manifest.json', manifest)
    outcome_rows = []
    for name in METHODS:
        row = _summary(audit['methods'][name]['original_outcome'])
        outcome_rows.append(dict(method=name, success=row.get('primary_success'), failure_reasons=row.get('failure_reasons')))
    write_index(run, images, outcome_rows)
    return manifest


def main(run):
    run = Path(run).resolve()
    audit = read(run / 'audit.json')
    context = read(run / 'input_context.json')
    route = read(run / 'goal_route.json')
    reference = np.load(run / 'reference_world.npy', allow_pickle=False)
    config = yaml.safe_load((run / 'config_snapshot.yaml').read_text())
    env = HospitalEnvironment.load(environment_path(run))
    past = read(run / 'actual_past_execution.json')
    manifest = render(run, audit, context, route, reference, config, env, past=past)
    return dict(plots=manifest, review=package(run, manifest['images']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    print(json.dumps(main(parser.parse_args().run), indent=2))
