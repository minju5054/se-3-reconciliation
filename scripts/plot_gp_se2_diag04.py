#!/usr/bin/env python3
"""Render saved DIAG-04 solve records without querying, optimizing or executing GP paths."""
from __future__ import annotations

import argparse
import copy
import html
import json
from pathlib import Path
import shutil
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import box

from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_01 import geometry
from plot_gp_se2_diag03 import plain, read, write, digest, _line, _quantity, _bands

LABEL = 'Quarter-point GP constraint experiment — plans only; no MPC execution'
GRIDS = ('G0_ORIGINAL', 'G1_QUARTER')
COLORS = dict(G0_ORIGINAL='#0072b2', G1_QUARTER='#d55e00')
PLOT_NAMES = ('world_xy_and_heading', 'lateral_velocity_vs_time', 'forward_speed_vs_time',
    'linear_acceleration_vs_time', 'angular_acceleration_vs_time', 'worst_interval_comparison',
    'feasible_objective_history', 'constraint_violation_history', 'solve_cost_and_evaluations',
    'equality_singular_values')
TITLES = ('Fixed inputs and newly optimized GP plans', 'Body lateral velocity and original tolerance',
    'Body forward speed and original bounds', 'Linear body acceleration and original bounds',
    'Angular body acceleration and original bounds', 'Each saved result at its own maximum observed tolerance-excess interval',
    'All callback objectives versus post-hoc full-feasible retained objectives',
    'Actual solver-grid residuals at recorded callbacks', 'Prepared solve, separate setup/check costs and evaluation counts',
    'Equality Jacobian singular values: diagnostic scaling only')
QUANTITIES = ('v_y', 'v_x', 'omega', 'a_x', 'a_omega')
UNITS = dict(v_y='m/s', v_x='m/s', omega='rad/s', a_x='m/s²', a_omega='rad/s²')


def pair_key(solve):
    return solve['case_role'], solve['method'], solve['initialization']


def pair_label(pair):
    case, method, initialization = pair
    return ('Hard' if case == 'HARD' else 'Benign control') + ' · ' + method.split('_')[0] + ' / ' + initialization.split('_')[0]


def pairs(solves):
    keys = list(dict.fromkeys(pair_key(s) for s in solves))
    return [(key, [s for s in solves if pair_key(s) == key]) for key in keys]


def visible_results(solve):
    """Identical selected/latest vectors are one curve, never an invented improvement."""
    latest, selected = solve.get('latest'), solve.get('selected')
    if latest is not None and latest.get('samples') is not None:
        same = selected is not None and latest.get('vector_sha256') is not None and latest.get('vector_sha256') == selected.get('vector_sha256')
        yield ('latest = selected' if same else 'latest'), latest, '-'
        if same:
            return
    if selected is not None and selected.get('samples') is not None:
        yield 'selected', selected, '--'


def full_history(solve):
    """Unknown/unchecked points remain NaN; only post-checked points establish a best value."""
    history = solve.get('history', [])
    times, bests, best = [], [], np.inf
    for row in sorted(history, key=lambda r: r['elapsed_s']):
        times.append(row['elapsed_s'])
        if row.get('full_feasible') is True and row.get('objective') is not None:
            best = min(best, row['objective'])
        bests.append(float(best) if np.isfinite(best) else None)
    return dict(elapsed_s=times, best_full_feasible_objective=bests,
        meaning='full checks performed after solve; x is saved discovery/callback time, not certification time')


def residual(values, quantity, cfg):
    values = np.asarray(values, float)
    if quantity == 'v_y':
        return np.abs(values) - cfg['equality_tolerance']
    upper = cfg[dict(v_x='v_max', omega='w_max', a_x='a_v_max', a_omega='a_w_max')[quantity]]
    lower = 0. if quantity == 'v_x' else -upper
    return np.maximum(lower-values, values-upper)-cfg['inequality_tolerance']


def representatives(audit):
    """Use every new result's saved samples; tie-break by earliest interval and time."""
    rows = []
    for solve in audit['solves']:
        for role, result, _ in visible_results(solve):
            samples = result['samples']
            for quantity in QUANTITIES:
                excess = residual(_quantity(samples, quantity), quantity, audit['formulation_config'])
                intervals, times = np.asarray(samples['interval_index']), np.asarray(samples['times_s'])
                order = np.lexsort((times, intervals, -excess))
                k = int(order[0])
                interval = int(intervals[k])
                mask = intervals == interval
                rows.append(dict(solve_id=solve['solve_id'], result=role, quantity=quantity,
                    interval_index=interval, observed_time_s=float(times[k]),
                    maximum_tolerance_excess=float(excess[k]), units=UNITS[quantity],
                    local_u=np.asarray(samples['local_u'])[mask].tolist(),
                    times_s=times[mask].tolist(), values=_quantity(samples, quantity)[mask].tolist(),
                    tolerance_excess=excess[mask].tolist()))
    return rows


def expected_numeric(audit):
    solves = audit['solves']
    if len(solves) != 10 or len({s['solve_id'] for s in solves}) != 10:
        raise ValueError('retain all ten planned solve records, including unavailable/failing starts')
    grouped = pairs(solves)
    if len(grouped) != 5 or any([s['grid'] for s in group] != list(GRIDS) for _, group in grouped):
        raise ValueError('five frozen G0/G1 pairs in planned order required')
    common = dict(solves=copy.deepcopy(solves), formulation_config=audit['formulation_config'],
        interpretation='one hard event with four method/seed comparisons and one benign control; not independent hard episodes',
        new_execution=False, full_acceptance_unchanged=True,
        finite_grid_is_continuous_proof=False, label=LABEL)
    output = {name: copy.deepcopy(common) for name in PLOT_NAMES}
    output['world_xy_and_heading'].update(contexts=audit['contexts'], aspect='equal',
        no_coordinate_offsets=True)
    output['worst_interval_comparison']['representatives'] = representatives(audit)
    output['feasible_objective_history']['posthoc_full_history'] = {s['solve_id']: full_history(s) for s in solves}
    sample_fields = {
        'world_xy_and_heading': ('poses_world',),
        'lateral_velocity_vs_time': ('times_s', 'body_twists', 'interval_index', 'local_u'),
        'forward_speed_vs_time': ('times_s', 'body_twists', 'interval_index', 'local_u'),
        'linear_acceleration_vs_time': ('times_s', 'body_accelerations', 'interval_index', 'local_u'),
        'angular_acceleration_vs_time': ('times_s', 'body_accelerations', 'interval_index', 'local_u'),
    }
    for name, data in output.items():
        for solve in data['solves']:
            for role in ('latest', 'selected'):
                value = solve.get(role)
                if value is not None:
                    if name in sample_fields and value.get('samples') is not None:
                        value['samples'] = {key: value['samples'][key] for key in sample_fields[name]}
                    else:
                        value.pop('samples', None)
            if name not in ('feasible_objective_history', 'constraint_violation_history'):
                solve.pop('history', None)
            if name != 'equality_singular_values':
                solve.pop('conditioning', None)
            if name != 'solve_cost_and_evaluations':
                solve.pop('timing', None)
                solve.pop('counts', None)
    return plain(output)


def _status(value):
    return 'N/A' if value is None else 'PASS' if value else 'FAIL'


def _panel_title(key, group):
    states = []
    for solve in group:
        latest, selected = solve.get('latest') or {}, solve.get('selected') or {}
        states.append(solve['grid'][:2] + ': latest ' + _status(latest.get('full_feasible')) +
            ' / selected ' + _status(selected.get('full_feasible')))
    return pair_label(key) + '\n' + ' | '.join(states)


def _layout(ncols=1, height=2.7, width=12):
    return plt.subplots(5, ncols, figsize=(width, height*5), squeeze=False)


def _axis(ax, ylabel, *, time=True):
    ax.set_ylabel(ylabel)
    if time:
        ax.set(xlabel='trajectory time [s]', xlim=(0., 3.))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3), useOffset=False)
    ax.grid(alpha=.2)


def _figure_key(fig):
    handles = [Line2D([], [], color=COLORS[g], label=g) for g in GRIDS]
    handles.extend([Line2D([], [], color='black', ls='-', label='latest (or latest = selected)'),
        Line2D([], [], color='black', ls='--', label='distinct selected candidate'),
        Line2D([], [], color='black', marker='o', ls='', fillstyle='none', label='support / midpoint'),
        Line2D([], [], color='black', marker='x', ls='', label='quarter points (G1 enforcement)')])
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .035), ncol=3, fontsize=8, frameon=False)


def world_plot(data, environment):
    fig, axes = _layout(width=11, height=3.4)
    for ax, (key, group) in zip(axes[:, 0], pairs(data['solves'])):
        context = data['contexts'][key[0]]
        pieces = [np.asarray(context[k], float) for k in ('old_world', 'fresh_world')]
        for solve in group:
            pieces.extend(np.asarray(r['samples']['poses_world']) for _, r, _ in visible_results(solve))
        xy = np.vstack([p[:, :2] for p in pieces if len(p)])
        lo, hi = xy.min(axis=0)-.35, xy.max(axis=0)+.35
        clip = box(lo[0], lo[1], hi[0], hi[1])
        if environment is not None:
            geometry(ax, environment.workspace.intersection(clip), facecolor='#f2f7eb', edgecolor='#718768')
            geometry(ax, environment.obstacles.intersection(clip), facecolor='#aaa', edgecolor='#555')
        for field, color, label in [('old_world', '#888', 'original OLD'), ('fresh_world', '#ac66ac', 'original FRESH')]:
            path = np.asarray(context[field], float)
            ax.plot(path[:, 0], path[:, 1], '.:', color=color, lw=.9, ms=3, label=label)
        for solve in group:
            for role, result, style in visible_results(solve):
                poses = np.asarray(result['samples']['poses_world'], float)
                color = COLORS[solve['grid']]
                ax.plot(poses[:, 0], poses[:, 1], color=color, ls=style, lw=1.3,
                    label=solve['grid'][:2] + ' ' + role)
                indices = np.linspace(0, len(poses)-1, min(9, len(poses)), dtype=int)
                p = poses[indices]
                ax.quiver(p[:, 0], p[:, 1], np.cos(p[:, 2]), np.sin(p[:, 2]),
                    color=color, angles='xy', scale_units='xy', scale=12, width=.003)
        b, goal = np.asarray(context['B_world']), np.asarray(context['goal_world'])
        ax.scatter(*b[:2], s=25, color='black', zorder=8, label='fixed B')
        ax.scatter(*goal[:2], s=80, marker='*', color='#238647', zorder=8, label='original goal')
        radius = context.get('footprint_radius_m', .20)
        ax.add_patch(Circle(b[:2], radius, fill=False, color='black', lw=.7))
        ax.add_patch(Circle(b[:2], radius+context.get('required_clearance_m', .05), fill=False, color='black', ls=':', lw=.7))
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='world x [m]', ylabel='world y [m]')
        ax.set_aspect('equal')
        ax.set_title(_panel_title(key, group), fontsize=10)
        ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=7, frameon=False)
        ax.grid(alpha=.15)
    return fig


def motion_plot(data, quantity):
    fig, axes = _layout(ncols=2 if quantity == 'v_x' else 1, width=14 if quantity == 'v_x' else 12)
    for row, (key, group) in enumerate(pairs(data['solves'])):
        for ax in axes[row]:
            for solve in group:
                for role, result, style in visible_results(solve):
                    samples = result['samples']
                    color = COLORS[solve['grid']]
                    values = _quantity(samples, quantity)
                    _line(ax, samples['times_s'], values, label=solve['grid'][:2]+' '+role,
                        color=color, linestyle=style, interval_index=samples['interval_index'], lw=1.1)
                    u = np.asarray(samples['local_u'])
                    times = np.asarray(samples['times_s'])
                    for points, marker in [((0., .5, 1.), 'o'), ((.25, .75), 'x')]:
                        mask = np.any(np.isclose(u[:, None], points, atol=1e-10, rtol=0), axis=1)
                        ax.scatter(times[mask], values[mask], s=9, color=color, marker=marker,
                            linewidths=.5, **({'facecolors': 'none'} if marker == 'o' else {}))
            _bands(ax, quantity, data['formulation_config'], label=False)
            _axis(ax, f'{quantity} [{UNITS[quantity]}]')
            ax.set_title(_panel_title(key, group), fontsize=10)
        if quantity == 'v_x':
            all_values = [_quantity(r['samples'], quantity) for s in group for _, r, _ in visible_results(s)]
            minimum = min([float(np.min(v)) for v in all_values] or [0.])
            span = max(abs(minimum), data['formulation_config']['inequality_tolerance']*5)
            axes[row, 1].set_ylim(-1.2*span, 2.5*span)
            axes[row, 1].set_title('Zero-speed detail — same curves and original tolerance', fontsize=9)
    _figure_key(fig)
    return fig


def worst_plot(data):
    fig, axes = _layout(ncols=5, width=24, height=3.3)
    cfg = data['formulation_config']
    for row, (key, group) in enumerate(pairs(data['solves'])):
        for column, quantity in enumerate(QUANTITIES):
            ax = axes[row, column]
            relevant = [r for r in data['representatives'] if r['solve_id'] in {s['solve_id'] for s in group} and r['quantity'] == quantity]
            max_excess = max([r['maximum_tolerance_excess'] for r in relevant] or [0.])
            for item in relevant:
                solve = next(s for s in group if s['solve_id'] == item['solve_id'])
                ax.plot(item['local_u'], item['tolerance_excess'], color=COLORS[solve['grid']],
                    ls='--' if item['result'] == 'selected' else '-', lw=1.2,
                    label=f"{solve['grid'][:2]} {item['result']} · i={item['interval_index']}")
            ax.axhline(0, color='black', ls='--', lw=.8)
            for u in (0., .5, 1.):
                ax.axvline(u, color='#aaa', ls=':', lw=.7)
            for u in (.25, .75):
                ax.axvline(u, color=COLORS['G1_QUARTER'], ls=':', lw=.7)
            tolerance = cfg['equality_tolerance' if quantity == 'v_y' else 'inequality_tolerance']
            all_excess = [value for item in relevant for value in item['tolerance_excess']]
            minimum = min(all_excess or [0.])
            span = max(max_excess, tolerance*.1)
            ax.set_ylim(min(minimum*1.1, -3*span), max(1.3*span, tolerance))
            ax.set(xlim=(0., 1.), xlabel='local interval fraction u', ylabel=f'tolerance excess [{UNITS[quantity]}]')
            ax.set_title(pair_label(key)+'\n'+quantity+'; each curve uses its own worst interval', fontsize=8)
            if abs(minimum) > 100*max(span, tolerance):
                ax.set_yscale('symlog', linthresh=tolerance)
                ax.set_ylabel(f'signed tolerance excess [{UNITS[quantity]}] (symlog)')
            else:
                ax.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3), useOffset=False)
            ax.grid(alpha=.15)
            ax.legend(loc='lower left', fontsize=6, frameon=False)
    return fig


def objective_plot(data):
    fig, axes = _layout(ncols=2, width=15)
    for row, (key, group) in enumerate(pairs(data['solves'])):
        for solve in group:
            history = solve.get('history', [])
            color = COLORS[solve['grid']]
            times = [r['elapsed_s'] for r in history]
            costs = [r.get('objective') for r in history]
            axes[row, 0].plot(times, costs, color=color, label=solve['grid'][:2], marker='.', ms=2, lw=.9)
            full = data['posthoc_full_history'][solve['solve_id']]
            values = [np.nan if v is None else v for v in full['best_full_feasible_objective']]
            axes[row, 1].step(full['elapsed_s'], values, where='post', color=color, label=solve['grid'][:2], lw=1.3)
            initial = next((r for r in history if r.get('source') == 'initial'), None)
            if initial is not None and initial.get('full_feasible') is True:
                axes[row, 1].axhline(initial['objective'], color=color, ls=':', lw=.8)
        for col, ax in enumerate(axes[row]):
            ax.set(xlabel='elapsed prepared solve time [s]', ylabel='original objective')
            ax.set_title(pair_label(key) + (' · all recorded callbacks' if col == 0 else ' · post-hoc full-feasible best'), fontsize=10)
            ax.grid(alpha=.2)
            ax.legend(loc='upper right', fontsize=8)
        if not any(any(v is not None for v in data['posthoc_full_history'][s['solve_id']]['best_full_feasible_objective']) for s in group):
            axes[row, 1].text(.5, .5, 'N/A — no post-checked full-feasible record', ha='center', transform=axes[row, 1].transAxes, fontsize=9)
    return fig


def constraint_plot(data):
    fig, axes = _layout(ncols=2, width=15)
    for row, (key, group) in enumerate(pairs(data['solves'])):
        for col, field in enumerate(('equality_residual', 'inequality_violation')):
            ax = axes[row, col]
            for solve in group:
                history = solve.get('history', [])
                ax.plot([h['elapsed_s'] for h in history], [h.get(field) for h in history],
                    color=COLORS[solve['grid']], label=solve['grid'][:2], lw=1, marker='.', ms=2)
            tol = data['formulation_config']['equality_tolerance' if col == 0 else 'inequality_tolerance']
            ax.axhline(tol, color='black', ls='--', lw=.8, label='original numerical tolerance')
            ax.set_yscale('symlog', linthresh=tol)
            ax.set(xlabel='elapsed prepared solve time [s]', ylabel='max |h| [m/s]' if col == 0 else 'max inequality violation [mixed original row units]')
            ax.set_title(pair_label(key) + (' · solver lateral equalities' if col == 0 else ' · solver inequality rows'), fontsize=10)
            ax.grid(alpha=.2)
            ax.legend(loc='upper right', fontsize=7)
    return fig


def costs_plot(data):
    fig, axes = plt.subplots(1, 3, figsize=(18, 8))
    labels = [pair_label(pair_key(s))+' '+s['grid'][:2] for s in data['solves']]
    yy = np.arange(len(labels))
    colors = [COLORS[s['grid']] for s in data['solves']]
    solve_times = [(s.get('timing') or {}).get('prepared_solve_s') for s in data['solves']]
    for k, value in enumerate(solve_times):
        if value is None:
            axes[0].text(0, k, 'N/A', va='center', fontsize=8)
        else:
            axes[0].barh(k, value, color=colors[k])
    axes[0].set_yticks(yy, labels, fontsize=8)
    axes[0].set(xlabel='prepared solve [s]', title='Equal configured solve budget; actual time shown')
    fields = [('compile_warmup_s', 'compile / warmup'), ('candidate_postcheck_s', 'candidate post-check'), ('rank_diagnostics_s', 'rank diagnostics')]
    for j, (field, label) in enumerate(fields):
        values = [(s.get('timing') or {}).get(field) for s in data['solves']]
        for k, value in enumerate(values):
            if value is not None:
                axes[1].barh(k+(j-1)*.22, value, height=.21, color=('#999', '#6bb4b5', '#b0a0d5')[j], label=label if k == 0 else None)
    axes[1].set(xlabel='separate stage wall time [s]', title='Stages shown separately; no overlapping totals')
    axes[1].set_yticks(yy, [])
    axes[1].legend(loc='upper right', fontsize=8)
    count_fields = [('objective_calls', 'objective'), ('constraint_calls', 'constraints'), ('derivative_calls', 'gradient / Jacobians'), ('primal_cache_misses', 'primal cache misses')]
    for j, (field, label) in enumerate(count_fields):
        for k, solve in enumerate(data['solves']):
            value = (solve.get('counts') or {}).get(field)
            if value is not None:
                axes[2].barh(k+(j-1.5)*.19, value, height=.18, color=('#555', '#88aacc', '#ddaa77', '#88bbaa')[j], label=label if k == 0 else None)
    axes[2].set(xlabel='calls / actual evaluations', title='Counts keep their inclusion relationships')
    axes[2].set_yticks(yy, [])
    axes[2].legend(loc='upper right', fontsize=8)
    for ax in axes:
        ax.invert_yaxis()
        ax.grid(axis='x', alpha=.2)
    return fig


def singular_plot(data):
    fig, axes = _layout(ncols=2, width=15)
    for row, (key, group) in enumerate(pairs(data['solves'])):
        for column, field in enumerate(('raw_singular_values', 'scaled_singular_values')):
            ax = axes[row, column]
            spectra = []
            for solve in group:
                for role, style in [('initial', ':'), ('latest', '-'), ('selected', '--')]:
                    value = (solve.get('conditioning') or {}).get(role) or {}
                    singular = (value.get(solve['grid']) or {}).get(field)
                    if singular is None:
                        continue
                    values = np.asarray(singular, float)
                    spectra.append(values)
                    positive = values > 0
                    ax.plot(np.flatnonzero(positive)+1, values[positive], ls=style, color=COLORS[solve['grid']],
                        label=solve['grid'][:2]+' '+role, lw=1)
                    if not np.all(positive):
                        ax.text(.02, .05+.06*len(spectra), f"{solve['grid'][:2]} {role}: {np.sum(~positive)} literal zero(s)", transform=ax.transAxes, fontsize=7)
            if not spectra:
                ax.text(.5, .5, 'N/A — conditioning unavailable', ha='center', transform=ax.transAxes)
            else:
                ax.set_yscale('log')
                ax.legend(loc='upper right', fontsize=7, ncol=2)
            ax.set(xlabel="singular-value index (solve's own equality grid)", ylabel='singular value')
            ax.set_title(pair_label(key)+(' · raw Jacobian' if column == 0 else ' · diagnostic column-scaled Jacobian'), fontsize=10)
            ax.grid(alpha=.2)
    return fig


def _save(fig, target, numeric, sources, title, dpi):
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError('refusing plot overwrite')
    fig.suptitle(title, fontsize=13, y=.996)
    fig.text(.01, .008, LABEL+' | Invalid latest: rejected / not executed. Full-valid candidate: plan only.', fontsize=8)
    extra = .075 if target.stem in ('lateral_velocity_vs_time', 'forward_speed_vs_time', 'linear_acceleration_vs_time', 'angular_acceleration_vs_time') else .035
    fig.tight_layout(rect=(0, extra, 1, .976))
    fig.savefig(target, dpi=dpi)
    plt.close(fig)
    write(target.with_suffix('.json'), dict(image=target.name, image_sha256=digest(target), numeric_data=numeric,
        source_hashes={str(path.resolve()): digest(path) for path in sources}, plotter_sha256=digest(__file__),
        label=LABEL, dpi=dpi, new_queries_from_plotter=0, new_solves_from_plotter=0, new_execution=0,
        objective_history_semantics='full feasibility checked after solve; callback timestamp is discovery, not certification'))
    return dict(path=str(target), sidecar=str(target.with_suffix('.json')), sha256=digest(target), sidecar_sha256=digest(target.with_suffix('.json')))


def metadata(run):
    paths = [Path(name) for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'experiment_manifest.json', 'aggregate/summary.json') if (run/name).is_file()]
    paths.extend(p.relative_to(run) for p in sorted((run/'aggregate').glob('*.csv')))
    return paths


def write_index(run, audit, images):
    lines = ['<!doctype html><html><head><meta charset="utf-8"><title>GP-SE2-DIAG-04</title><style>body{font:16px system-ui;max-width:1500px;margin:25px auto;padding:15px}img{max-width:100%}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:8px}article{margin:35px 0}</style></head><body>',
        '<h1>Quarter-point motion constraint refinement</h1>',
        '<p>Five paired G0/G1 comparisons: four method/seed pairs from one hard event and one benign control. These are ten planned GP solves, not ten independent handoffs. '
        'Both grids use the same saved initial vectors, objective and physical acceptance. Quarter lateral equalities and motion inequalities are the only additional constraints.</p>',
        '<p><strong>Rejected / not executed</strong> applies to invalid results. Full-valid candidates are also plans only. No MPC solve, rollout, VLA inference or GUI runtime is represented. '
        'Latest and selected vectors are separate; identical vectors share a single curve. Finite-grid success is not continuous-time proof.</p>',
        '<p>Objective histories distinguish all recorded callbacks from post-hoc full-feasible retained records. Their time coordinate is callback discovery time, not real-time feasibility certification. '
        'Body acceleration is the time derivative of body-twist components. Rank scaling is diagnostic only and does not change the solver.</p>',
        '<table><tr><th>Planned solve</th><th>Termination</th><th>Latest full</th><th>Selected full</th><th>Selected source</th><th>Objective latest / selected</th></tr>']
    for solve in audit['solves']:
        latest, selected = solve.get('latest') or {}, solve.get('selected') or {}
        cells = [solve['solve_id'], str(solve.get('termination', 'N/A')), _status(latest.get('full_feasible')),
            _status(selected.get('full_feasible')), str(selected.get('source', 'N/A')),
            str(latest.get('objective', 'N/A'))+' / '+str(selected.get('objective', 'N/A'))]
        lines.append('<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>')
    lines.append('</table><h2>Protocol, source hashes and complete result tables</h2><ul>')
    for name in metadata(run):
        lines.append(f'<li><a href="{name}">{name}</a></li>')
    lines.append('</ul>')
    for row in images:
        path = Path(row['path']).relative_to(run)
        lines.append(f'<article><h2>{html.escape(path.stem.replace("_", " "))}</h2><img loading="lazy" src="{path}"><p><a href="{path.with_suffix(".json")}">Exact plotted numbers and hashes</a></p></article>')
    with (run/'index.html').open('x') as stream:
        stream.write('\n'.join(lines+['</body></html>']))


def package(run, images):
    target, archive = run/'review_bundle', run/'review_bundle.zip'
    target.mkdir()
    paths = metadata(run)+[Path('index.html'), Path('plot_manifest.json')]
    for row in images:
        path = Path(row['path']).relative_to(run)
        paths.extend([path, path.with_suffix('.json')])
    if len(set(paths)) != len(paths) or any(p.is_absolute() or '..' in p.parts for p in paths):
        raise ValueError('unsafe review allowlist')
    inventory = []
    for path in paths:
        out = target/path
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run/path, out)
        inventory.append(dict(path=str(path), sha256=digest(out), bytes=out.stat().st_size))
    notes = target/'README.md'
    notes.write_text('# GP-SE2-DIAG-04 review\n\nOpen index.html. All ten planned solves, including failures and N/A results, remain in the ledger. '
        'Quarter-point constraints change motion enforcement, not support nodes, objective, environment rows or original acceptance. '
        'Plots use saved records only and perform no queries or solves. Invalid paths are rejected / not executed; valid candidates are also unexecuted plans. '
        'One hard event contributes four method/seed pairs, not four independent episodes. Full-feasible history is checked after solving. '
        'No continuous-time, navigation or actual execution improvement is established by this experiment. '
        'Sidecars preserve plotted numbers and source hashes. Raw source arrays, RGB, whole environment, checkpoints, external source and caches are excluded.\n')
    inventory.append(dict(path='README.md', sha256=digest(notes), bytes=notes.stat().st_size))
    write(target/'manifest.json', dict(files=inventory, image_count=len(images), self_excluded='manifest.json'))
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as stream:
        for path in sorted(target.rglob('*')):
            if path.is_file():
                stream.write(path, str(path.relative_to(target)))
    result = dict(path=str(archive), sha256=digest(archive), bytes=archive.stat().st_size, image_count=len(images))
    write(run/'review_bundle_manifest.json', result)
    return result


def generate(run, *, audit=None, environment=None, dpi=160):
    run = Path(run).resolve()
    if dpi < 160:
        raise ValueError('at least 160 dpi required')
    for name in ('plots', 'plot_manifest.json', 'index.html', 'review_bundle', 'review_bundle.zip', 'review_bundle_manifest.json'):
        if (run/name).exists():
            raise FileExistsError('refusing plotting output overwrite: '+str(run/name))
    audit = read(run/'plot_input.json') if audit is None else audit
    sources = [run/name for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'experiment_manifest.json', 'plot_input.json')]
    sources.extend(sorted((run/'aggregate').glob('*.csv')))
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)
    numeric = expected_numeric(audit)
    if environment is None:
        source = read(run/'source.json')
        value = source.get('environment_path', source.get('environment'))
        value = value.get('path') if isinstance(value, dict) else value
        if not isinstance(value, str):
            raise ValueError('source must identify preserved environment export')
        path = Path(value)
        environment = HospitalEnvironment.load(path if path.is_absolute() else ROOT/path)
    (run/'plots').mkdir()
    functions = (lambda d: world_plot(d, environment), lambda d: motion_plot(d, 'v_y'),
        lambda d: motion_plot(d, 'v_x'), lambda d: motion_plot(d, 'a_x'), lambda d: motion_plot(d, 'a_omega'),
        worst_plot, objective_plot, constraint_plot, costs_plot, singular_plot)
    images = [_save(plot(numeric[name]), run/'plots'/(name+'.png'), numeric[name], sources, title, dpi)
        for name, title, plot in zip(PLOT_NAMES, TITLES, functions)]
    manifest = dict(images=images, plot_count=len(images), source_solve_count=len(audit['solves']), pair_count=5,
        plotter_sha256=digest(__file__), new_solves_from_plotter=0, new_queries_from_plotter=0,
        new_execution=0, gui_runtime_validated=False)
    write(run/'plot_manifest.json', manifest)
    write_index(run, audit, images)
    manifest['review_bundle'] = package(run, images)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(generate(args.run), indent=2))


if __name__ == '__main__':
    main()
