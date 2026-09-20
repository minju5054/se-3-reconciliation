#!/usr/bin/env python3
"""Plot saved GP interpolation audit values; never solve or execute a path.

The audit owns sampling, acceptance and representative interval selection. This
entry point only renders those saved numbers, retaining each record's provenance.
"""
from __future__ import annotations

import argparse
import copy
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
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import box
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_01 import geometry

LABEL = 'Saved GP interpolation audit — no optimization or execution'
REJECTED_LABEL = 'Rejected GP iterate — not executed'
PLOT_NAMES = ('world_xy_and_heading', 'lateral_velocity_vs_time', 'forward_speed_vs_time',
    'body_acceleration_vs_time', 'violation_by_interval', 'worst_interval_zoom', 'grid_detection_summary')
TITLES = ('Stored GP geometry and headings', 'Body lateral velocity: enforced points and interval interiors',
    'Body forward speed: full range and zero-speed detail', 'Body-twist component derivatives: one-sided knots retained',
    'Observed violations by support interval and constraint family',
    'Worst tolerance-excess intervals: enforced points versus the saved interpolant',
    'Detection only: three query grids on identical saved vectors')
COLORS = ('#0072b2', '#d55e00', '#009e73', '#cc79a7', '#555555')
FAMILIES = ('lateral', 'speed_lower', 'speed_upper', 'angular_speed', 'linear_acceleration', 'angular_acceleration')
FAMILY_LABELS = ('lateral [m/s]', 'speed lower [m/s]', 'speed upper [m/s]',
    'angular speed [rad/s]', 'linear accel. [m/s²]', 'angular accel. [rad/s²]')
QUANTITIES = ('v_y', 'v_x', 'omega', 'a_x', 'a_omega')
QUANTITY_ALIASES = dict(vx='v_x', vy='v_y', omega='omega', ax='a_x', ay='a_y', alpha='a_omega')
UNITS = dict(v_y='m/s', v_x='m/s', omega='rad/s', a_x='m/s²', a_y='m/s²', a_omega='rad/s²')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def _save(fig, target, numeric, sources, title, *, dpi=160):
    target = Path(target)
    if dpi < 160:
        raise ValueError('at least 160 dpi required')
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError('refusing plot overwrite')
    fig.suptitle(textwrap.fill(title, width=92), fontsize=13)
    fig.text(.012, .009, LABEL + ' | adjacent JSON: exact numbers and hashes', fontsize=8)
    fig.tight_layout(rect=(0, .035, 1, .935))
    fig.savefig(target, dpi=dpi)
    plt.close(fig)
    write(target.with_suffix('.json'), dict(image=target.name, image_sha256=digest(target),
        numeric_data=numeric, source_hashes={str(Path(p).resolve()): digest(p) for p in sources},
        plotter_sha256=digest(__file__), dpi=dpi, label=LABEL,
        new_inference=0, new_gp_or_rigid_optimization=0, new_mpc_solve=0,
        new_closed_loop_rollout=0, new_gui_runtime=0,
        missing_values='null / explicitly unavailable; never fabricated zero',
        interpretation='sampled diagnostics preserve original acceptance; no continuous feasibility proof'))
    return dict(path=str(target), sidecar=str(target.with_suffix('.json')),
        sha256=digest(target), sidecar_sha256=digest(target.with_suffix('.json')))


def _legend(ax, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc='upper left', bbox_to_anchor=(1.02, 1),
            fontsize=7, frameon=False, **kwargs)


def _time(ax, label):
    ax.set(xlabel='trajectory time [s]', ylabel=label, xlim=(0., 3.))
    ax.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3), useOffset=False)
    ax.grid(alpha=.2)


def _line(ax, times, values, *, label, color, linestyle='-', interval_index=None, **kwargs):
    """Never connect the two sides of an acceleration jump at a support knot."""
    times = np.asarray(times, float)
    values = np.asarray([np.nan if v is None else v for v in values], float)
    if interval_index is None:
        ax.plot(times, values, color=color, linestyle=linestyle, label=label, **kwargs)
        return
    index = np.asarray(interval_index)
    for k, interval in enumerate(dict.fromkeys(index.tolist())):
        mask = index == interval
        ax.plot(times[mask], values[mask], color=color, linestyle=linestyle,
            label=label if k == 0 else None, **kwargs)


def _short(record):
    if record.get('role') == 'benign_final':
        return 'Benign M3 / I1 · checker reference'
    return record['method'].replace('M2_GP_NO_OBSTACLE', 'M2').replace('M3_GP_CONSTRAINED', 'M3') + ' / ' + record['initialization'].replace('I0_FRESH', 'I0').replace('I1_DECEL', 'I1')


def _quantity(samples, quantity):
    if quantity in ('v_x', 'v_y', 'omega'):
        return np.asarray(samples['body_twists'], float)[:, ('v_x', 'v_y', 'omega').index(quantity)]
    return np.asarray(samples['body_accelerations'], float)[:, ('a_x', 'a_y', 'a_omega').index(quantity)]


def _bands(ax, quantity, config, *, label=True):
    eq, iq = config['equality_tolerance'], config['inequality_tolerance']
    if quantity == 'v_y':
        limits = [-eq, eq]
        ax.axhspan(*limits, color='#009e73', alpha=.08)
        for i, limit in enumerate(limits):
            ax.axhline(limit, color='#333333', ls='--', lw=.9, label='original ± lateral tolerance' if label and i == 0 else None)
        return
    if quantity == 'a_y':
        return
    maximum = config[dict(v_x='v_max', omega='w_max', a_x='a_v_max', a_omega='a_w_max')[quantity]]
    lower = 0. if quantity == 'v_x' else -maximum
    for i, limit in enumerate((lower, maximum)):
        ax.axhline(limit, color='#777777', ls=':', lw=.9, label='nominal physical bounds' if label and i == 0 else None)
    for i, limit in enumerate((lower - iq, maximum + iq)):
        ax.axhline(limit, color='#333333', ls='--', lw=.9, label='bounds + original numerical tolerance' if label and i == 0 else None)


def _enforced(ax, samples, quantity, color):
    u = np.asarray(samples['local_u'], float)
    mask = np.isclose(u, 0., atol=1e-12) | np.isclose(u, .5, atol=1e-12) | np.isclose(u, 1., atol=1e-12)
    values = _quantity(samples, quantity)
    ax.scatter(np.asarray(samples['times_s'])[mask], values[mask], s=10, color=color,
        marker='o', facecolors='none', linewidths=.6, zorder=5)


def expected_numeric(audit):
    """Exact plotted payload, kept separate so the artifact validator can compare it."""
    records = audit['records']
    finals = [r for r in records if r['role'] in ('hard_final', 'benign_final')]
    if len(finals) != 5:
        raise ValueError('all four hard final records and the benign final reference must remain in the plot ledger')
    views = []
    for index, record in enumerate(finals):
        views.append(dict(record_id=record['record_id'], role=record['role'], method=record['method'],
            initialization=record['initialization'], label=_short(record), color=COLORS[index],
            available=record.get('available', True), samples=record.get('samples'),
            acceptance=record.get('acceptance'), vector_sha256=record.get('vector_sha256'),
            source_record=record.get('source_record'), interval_extrema=record.get('interval_extrema', []),
            grid_detection=record.get('grid_detection', [])))
    common = dict(records=views, formulation_config=audit['formulation_config'],
        no_execution=True, primary='four stored hard final iterates; one event, not four independent samples',
        benign_role='saved full-valid reconstruction reference, not an additional performance event',
        initial_records_context=[dict(record_id=r['record_id'], method=r['method'], initialization=r['initialization'],
            available=r.get('available', True), acceptance=r.get('acceptance'),
            vector_sha256=r.get('vector_sha256')) for r in records if r['role'] == 'hard_initial'])
    result = {}
    for name in PLOT_NAMES:
        result[name] = copy.deepcopy(common)
    result['world_xy_and_heading'].update(contexts=audit['contexts'], aspect='equal',
        heading_semantics='body heading at saved interpolant samples; no MPC execution line')
    result['violation_by_interval']['families'] = list(FAMILIES)
    result['worst_interval_zoom']['representative_intervals'] = representative_intervals(audit)
    result['grid_detection_summary']['detection_only'] = True
    result['grid_detection_summary']['G2_semantics'] = 'post-hoc diagnostic witnesses; not independent validation or a solved refined problem'
    # A small review bundle contains the arrays actually used by each image,
    # not seven copies of all pose, twist, acceleration and interval tables.
    sample_fields = {
        'world_xy_and_heading': {'poses_world'},
        'lateral_velocity_vs_time': {'times_s', 'body_twists', 'interval_index', 'local_u'},
        'forward_speed_vs_time': {'times_s', 'body_twists', 'interval_index', 'local_u'},
        'body_acceleration_vs_time': {'times_s', 'body_accelerations', 'interval_index', 'local_u'},
        'violation_by_interval': {'times_s', 'body_twists', 'body_accelerations', 'interval_index'},
        'worst_interval_zoom': {'times_s', 'body_twists', 'body_accelerations', 'interval_index', 'local_u'},
        'grid_detection_summary': set(),
    }
    for name, payload in result.items():
        for record in payload['records']:
            record.pop('interval_extrema', None)
            if name != 'grid_detection_summary':
                record.pop('grid_detection', None)
            if record.get('samples') is not None:
                record['samples'] = {key: value for key, value in record['samples'].items() if key in sample_fields[name]}
    return plain(result)


def representative_intervals(audit):
    """Maximum tolerance-excess per physical quantity; earliest interval breaks ties."""
    rows = []
    for quantity in QUANTITIES:
        candidates = []
        for record in audit['records']:
            if record['role'] != 'hard_final':
                continue
            for item in record.get('interval_extrema', []):
                if item.get('grid') != 'SUPPLEMENTAL_0.001_OFFSET' or QUANTITY_ALIASES.get(item['quantity'], item['quantity']) != quantity:
                    continue
                excess = item.get('maximum_tolerance_excess')
                if excess is not None:
                    candidates.append(dict(quantity=quantity, record_id=record['record_id'],
                        interval_index=item['interval_index'], time_s=item['worst_time_s'],
                        tolerance_excess=excess))
        if candidates:
            rows.append(min(candidates, key=lambda r: (-r['tolerance_excess'], r['interval_index'], r['time_s'], r['record_id'])))
        else:
            rows.append(dict(quantity=quantity, record_id=None, interval_index=None, time_s=None,
                tolerance_excess=None, reason='No available saved finest-grid extrema'))
    return rows


def _records(data, role):
    return [r for r in data['records'] if r['role'] == role]


def _plot_trajectories(ax, rows, quantity):
    for r in rows:
        if not r['available'] or not r.get('samples'):
            ax.plot([], [], label=r['label'] + ' · N/A', color=r['color'])
            continue
        samples = r['samples']
        _line(ax, samples['times_s'], _quantity(samples, quantity), label=r['label'], color=r['color'],
            interval_index=samples['interval_index'] if quantity.startswith('a_') else None,
            lw=1.4, alpha=.9)
        _enforced(ax, samples, quantity, r['color'])


def _bounds(context, records):
    parts = [np.asarray(context[k], float)[:, :2] for k in ('old_world', 'fresh_world')]
    parts += [np.asarray([context['B_world'], context['goal_world']], float)[:, :2]]
    parts += [np.asarray(r['samples']['poses_world'], float)[:, :2] for r in records if r.get('samples')]
    points = np.vstack(parts)
    lower, upper = points.min(axis=0) - .45, points.max(axis=0) + .45
    centre, span = (lower + upper) / 2, max(float(np.max(upper - lower)), 1.2)
    return [centre[0]-span/2, centre[0]+span/2, centre[1]-span/2, centre[1]+span/2]


def world_plot(data, environments):
    fig, axes = plt.subplots(1, 2, figsize=(18, 7))
    for ax, role, context_key in zip(axes, ('hard_final', 'benign_final'), ('hard', 'benign')):
        rows = _records(data, role)
        context = data['contexts'][context_key]
        env = environments[context_key]
        bounds = _bounds(context, rows)
        clip = box(bounds[0], bounds[2], bounds[1], bounds[3])
        ax.set_facecolor('#eeebe4')
        geometry(ax, env.workspace.intersection(clip), facecolor='#f4faf0', edgecolor='#91a481')
        geometry(ax, env.obstacles.intersection(clip), facecolor='#aaaaaa', edgecolor='#666666')
        for key, color, label in (('old_world', '#92a6b2', 'original OLD'), ('fresh_world', '#a86b92', 'original FRESH')):
            p = np.asarray(context[key], float)
            ax.plot(p[:, 0], p[:, 1], '.--', lw=1., ms=3, c=color, label=label)
        for r in rows:
            if not r.get('samples'):
                ax.plot([], [], color=r['color'], label=r['label'] + ' · N/A')
                continue
            p = np.asarray(r['samples']['poses_world'], float)
            ax.plot(p[:, 0], p[:, 1], color=r['color'], lw=1.6, label=r['label'])
            idx = np.unique(np.linspace(0, len(p)-1, 11, dtype=int))
            ax.quiver(p[idx, 0], p[idx, 1], np.cos(p[idx, 2]), np.sin(p[idx, 2]), color=r['color'],
                angles='xy', scale_units='xy', scale=14., width=.003, alpha=.7)
        b, goal = np.asarray(context['B_world']), np.asarray(context['goal_world'])
        radius = context.get('footprint_radius_m', .2)
        clearance = context.get('required_clearance_m', .05)
        ax.add_patch(Circle(b[:2], radius, fill=False, edgecolor='#111111', lw=.8))
        ax.add_patch(Circle(b[:2], radius+clearance, fill=False, edgecolor='#111111', lw=.8, ls=':'))
        ax.scatter(*b[:2], s=35, color='black', zorder=7, label='original B / footprint / margin')
        ax.quiver(*b[:2], np.cos(b[2]), np.sin(b[2]), angles='xy', scale_units='xy', scale=5., color='black', zorder=8)
        ax.scatter(*goal[:2], s=90, marker='*', color='#278245', zorder=8, label='original goal')
        ax.quiver(*goal[:2], np.cos(goal[2]), np.sin(goal[2]), angles='xy', scale_units='xy', scale=6., color='#278245')
        for gate in context.get('gates', []):
            center, normal = np.asarray(gate['center_xy']), np.asarray(gate['normal_xy'])
            tangent = np.array([-normal[1], normal[0]])
            endpoints = center + np.array([[-1], [1]])*gate['half_width_m']*tangent
            ax.plot(endpoints[:, 0], endpoints[:, 1], '-.', color='#80551a', lw=1.4, label='original centre-safe gate')
        ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='world x [m]', ylabel='world y [m]', aspect='equal',
            title=REJECTED_LABEL if role == 'hard_final' else 'Benign saved reference — not executed here')
        ax.grid(alpha=.18)
        _legend(ax)
    return fig


def lateral_plot(data):
    fig, axes = plt.subplots(2, 1, figsize=(15, 9))
    for ax, role in zip(axes, ('hard_final', 'benign_final')):
        _plot_trajectories(ax, _records(data, role), 'v_y')
        _bands(ax, 'v_y', data['formulation_config'])
        _time(ax, 'body lateral velocity $v_y$ [m/s]')
        ax.set_title(REJECTED_LABEL if role == 'hard_final' else 'Benign reference context; circles = support / midpoint')
        _legend(ax)
    return fig


def speed_plot(data):
    fig, axes = plt.subplots(2, 2, figsize=(19, 9))
    config = data['formulation_config']
    for row, role in enumerate(('hard_final', 'benign_final')):
        records = _records(data, role)
        for column, ax in enumerate(axes[row]):
            _plot_trajectories(ax, records, 'v_x')
            _bands(ax, 'v_x', config)
            _time(ax, 'body forward velocity $v_x$ [m/s]')
            ax.set_title(('Hard finals' if row == 0 else 'Benign reference') + (' · full range' if column == 0 else ' · signed zero-speed detail'))
            if column == 1:
                available = [_quantity(r['samples'], 'v_x') for r in records if r.get('samples')]
                values = np.concatenate(available) if available else np.array([-config['inequality_tolerance']])
                minimum = min(float(values.min()), -config['inequality_tolerance'])
                ax.set_ylim(minimum - max(abs(minimum)*.15, 2e-5), max(abs(minimum)*3, 5e-4))
            _legend(ax)
    return fig


def acceleration_plot(data):
    fig, axes = plt.subplots(3, 2, figsize=(19, 13))
    for row, quantity in enumerate(('a_x', 'a_omega', 'a_y')):
        for column, role in enumerate(('hard_final', 'benign_final')):
            ax = axes[row, column]
            _plot_trajectories(ax, _records(data, role), quantity)
            _bands(ax, quantity, data['formulation_config'])
            _time(ax, f'{quantity} [{UNITS[quantity]}]')
            ax.set_title(('Hard finals' if column == 0 else 'Benign reference') + (' · lateral derivative is diagnostic only' if quantity == 'a_y' else ' · interval sides plotted separately'))
            _legend(ax)
    return fig


def _family_excess(samples, config):
    v = np.asarray(samples['body_twists'], float)
    a = np.asarray(samples['body_accelerations'], float)
    eq, iq = config['equality_tolerance'], config['inequality_tolerance']
    return np.column_stack((abs(v[:, 1])-eq, -v[:, 0]-iq, v[:, 0]-config['v_max']-iq,
        abs(v[:, 2])-config['w_max']-iq, abs(a[:, 0])-config['a_v_max']-iq,
        abs(a[:, 2])-config['a_w_max']-iq))


def interval_plot(data):
    records = data['records']
    fig, axes = plt.subplots(len(records), 1, figsize=(15, 3*len(records)), squeeze=False)
    cmap = ListedColormap(['#eff4f7', '#c7483b'])
    for ax, record in zip(axes[:, 0], records):
        if not record.get('samples'):
            ax.text(.5, .5, record['label']+' · N/A', ha='center', transform=ax.transAxes)
            continue
        s = record['samples']
        indices = np.asarray(s['interval_index'])
        excess = _family_excess(s, data['formulation_config'])
        observed = np.array([(excess[indices == i] > 0).any(axis=0) for i in range(30)]).T.astype(int)
        ax.imshow(observed, origin='upper', aspect='auto', interpolation='none', vmin=0, vmax=1, cmap=cmap)
        ax.set(yticks=np.arange(len(FAMILIES)), yticklabels=FAMILY_LABELS, xticks=np.arange(30),
            xlabel='support interval index (0.1 s each)', title=record['label'])
        ax.text(1.015, .5, 'red: observed\ntolerance exceedance\n\nblank: none observed\non sampled grid\n(not a continuous proof)',
            transform=ax.transAxes, fontsize=8, va='center')
    return fig


def _quantity_excess(values, quantity, config):
    if quantity == 'v_y':
        return abs(values)-config['equality_tolerance']
    maximum = config[dict(v_x='v_max', omega='w_max', a_x='a_v_max', a_omega='a_w_max')[quantity]]
    lower = 0. if quantity == 'v_x' else -maximum
    return np.maximum(lower-values, values-maximum)-config['inequality_tolerance']


def worst_plot(data):
    reps = data['representative_intervals']
    fig, axes = plt.subplots(len(reps), 2, figsize=(20, 3.7*len(reps)), squeeze=False)
    mapping = {r['record_id']: r for r in data['records']}
    config = data['formulation_config']
    for row, rep in enumerate(reps):
        quantity = rep['quantity']
        if rep.get('record_id') is None:
            for ax in axes[row]:
                ax.text(.5, .5, f'{quantity}: N/A — no available saved vector', ha='center', transform=ax.transAxes)
            continue
        selected = mapping[rep['record_id']]
        interval = rep['interval_index']
        lo, hi = interval*.1, (interval+1)*.1
        excess_curves = []
        for r in _records(data, 'hard_final'):
            s = r.get('samples')
            if not s:
                continue
            mask = np.asarray(s['interval_index']) == interval
            partial = {key: np.asarray(value)[mask].tolist() for key, value in s.items() if key in
                ('times_s', 'body_twists', 'body_accelerations', 'local_u', 'interval_index', 'poses_world')}
            values = _quantity(partial, quantity)
            excess = _quantity_excess(values, quantity, config)
            excess_curves.extend(excess)
            _line(axes[row, 0], partial['times_s'], values, label=r['label'], color=r['color'],
                lw=2.3 if r['record_id'] == selected['record_id'] else 1.0,
                interval_index=partial['interval_index'])
            _enforced(axes[row, 0], partial, quantity, r['color'])
            _line(axes[row, 1], partial['times_s'], excess, label=r['label'], color=r['color'],
                lw=2.3 if r['record_id'] == selected['record_id'] else 1.0,
                interval_index=partial['interval_index'])
            u = np.asarray(partial['local_u'])
            enforced = np.isclose(u, 0., atol=1e-12) | np.isclose(u, .5, atol=1e-12) | np.isclose(u, 1., atol=1e-12)
            axes[row, 1].scatter(np.asarray(partial['times_s'])[enforced], excess[enforced], s=14,
                marker='o', facecolors='none', edgecolors=r['color'], linewidths=.6, zorder=5)
        _bands(axes[row, 0], quantity, config)
        axes[row, 0].set_ylabel(f'{quantity} [{UNITS[quantity]}]')
        axes[row, 0].set_title(f"{quantity}: interval {interval}; {selected['label']} has largest excess")
        axes[row, 1].set_title('Signed tolerance-excess: detail near the acceptance boundary')
        axes[row, 1].set_ylabel(f'max(lower − value, value − upper) − tolerance [{UNITS[quantity]}]' if quantity != 'v_y'
            else f'|v_y| − lateral tolerance [{UNITS[quantity]}]')
        axes[row, 1].axhline(0., color='black', ls='--', lw=.9, label='original acceptance boundary')
        tolerance = config['equality_tolerance' if quantity == 'v_y' else 'inequality_tolerance']
        axes[row, 1].axhline(-tolerance, color='#777777', ls=':', lw=.8, label='nominal bound')
        maximum = float(max(excess_curves))
        if maximum > 0.:
            span = max(maximum, 1e-9)
            axes[row, 1].set_ylim(-3*span, 1.3*span)
        for ax in axes[row]:
            ax.set(xlim=(lo, hi), xlabel='trajectory time [s]')
            ax.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3), useOffset=False)
            ax.grid(alpha=.2)
            for t in (lo, (lo+hi)/2, hi):
                ax.axvline(t, color='#777777', ls=':', lw=.7)
            if rep.get('time_s') is not None:
                ax.axvline(rep['time_s'], color=selected['color'], ls='--', lw=.9, label='observed maximum-excess time')
            _legend(ax)
        axes[row, 1].text(.015, .94, f"sampled maximum = {rep['tolerance_excess']:.9g} {UNITS[quantity]}",
            transform=axes[row, 1].transAxes, va='top', fontsize=9, bbox=dict(facecolor='white', alpha=.8, edgecolor='none'))
    return fig


def detection_plot(data):
    records = data['records']
    grid_names = ('G0_ORIGINAL', 'G1_QUARTERS', 'G2_WITNESS')
    table = []
    for r in records:
        by_grid = {grid: [row for row in r['grid_detection'] if row['grid'] == grid] for grid in grid_names}
        values = []
        for grid in grid_names:
            rows = by_grid[grid]
            if not rows:
                values.append('N/A')
            else:
                hits = [QUANTITY_ALIASES.get(row.get('quantity'), row.get('quantity', 'unknown')) for row in rows if not row.get('passed', True)]
                values.append('No observed violation' if not hits else 'Detected:\n'+'\n'.join(hits))
        table.append([r['label'], *values])
    fig, ax = plt.subplots(figsize=(16, 8))
    ax.axis('off')
    tab = ax.table(cellText=table, colLabels=['Saved final record', 'G0: u = 0, 0.5, 1',
        'G1: G0 + quarters', 'G2: G0 + observed witnesses'], cellLoc='left', colLoc='left',
        bbox=[0, .18, 1, .77], colWidths=[.25, .25, .25, .25])
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    for (row, column), cell in tab.get_celld().items():
        cell.set_edgecolor('#bbbbbb')
        if row == 0:
            cell.set_facecolor('#dae4ed')
            cell.set_text_props(weight='bold')
        elif column > 0:
            cell.set_facecolor('#f8e7e4' if 'Detected:' in cell.get_text().get_text() else '#f1f5f7')
    ax.text(0, .11, 'Same stored vectors in every column. G1/G2 only detect violations; no refined optimization was solved.', fontsize=11)
    ax.text(0, .065, 'G2 uses post-hoc violation witnesses. Missing violations on a finite grid is not continuous feasibility.', fontsize=10)
    ax.text(0, .02, 'Original collocation, dense and full acceptance remain unchanged and are listed in the index and source tables.', fontsize=10)
    return fig


def _status(value):
    return 'N/A' if value is None else 'PASS' if value else 'FAIL'


def review_metadata(run):
    run = Path(run)
    result = [Path(name) for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'record_manifest.json',
        'aggregate/summary.json') if (run / name).is_file()]
    result.extend(p.relative_to(run) for p in sorted((run / 'aggregate').glob('*.csv')))
    return result


def write_index(run, audit, images):
    run = Path(run)
    path = run / 'index.html'
    if path.exists():
        raise FileExistsError('refusing index overwrite')
    text = ['<!doctype html><html><head><meta charset="utf-8"><title>GP-SE2-DIAG-03 saved audit</title>',
        '<style>body{font:16px system-ui;max-width:1500px;margin:25px auto;padding:15px}img{max-width:100%}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #bbb}article{margin:35px 0}th{background:#edf2f6}</style></head><body>',
        '<h1>Saved GP interpolation feasibility audit</h1>',
        '<p><strong>Rejected GP iterate — not executed.</strong> Four final stored iterates from one hard handoff are the primary comparison. '
        'Their four stored initial vectors provide context. One benign final vector is a reconstruction/checker reference. '
        'These records are not independent performance episodes. No GP/rigid optimization, VLA inference, MPC solve, closed-loop rollout or GUI runtime was performed.</p>',
        '<p>Every panel is computed from the saved fixed trajectory. Extra query points, interval refinement and derivative checks do not change its support states or original acceptance. '
        'Sampled extrema are observed extrema, not a continuous feasibility proof. Acceleration is the time derivative of body-twist components.</p>',
        '<h2>Original acceptance and record coverage</h2><table><tr><th>Record</th><th>Role</th><th>Available</th><th>Collocation</th><th>Dense</th><th>Independent full</th></tr>']
    for record in audit['records']:
        acceptance = record.get('acceptance') or {}
        text.append('<tr><td>' + html.escape(record['record_id']) + '</td><td>' + html.escape(record['role']) +
            '</td><td>' + ('yes' if record.get('available', True) else 'N/A') + '</td><td>' +
            _status(acceptance.get('collocation_feasible')) + '</td><td>' + _status(acceptance.get('dense_feasible')) +
            '</td><td>' + _status(acceptance.get('full_feasible')) + '</td></tr>')
    text.extend(['</table>', '<h2>Protocol, hashes and complete numeric tables</h2><ul>'])
    for file in review_metadata(run):
        text.append(f'<li><a href="{html.escape(str(file))}">{html.escape(str(file))}</a></li>')
    text.append('</ul>')
    for row in images:
        file = Path(row['path'])
        if file.is_absolute():
            file = file.relative_to(run.resolve())
        text.append(f'<article><h2>{html.escape(file.stem.replace("_", " "))}</h2><img loading="lazy" src="{html.escape(str(file))}"><p><a href="{html.escape(str(file.with_suffix(".json")))}">Exact plotted values and source hashes</a></p></article>')
    path.write_text('\n'.join(text + ['</body></html>']))


def package(run, images):
    run = Path(run).resolve()
    bundle, archive = run / 'review_bundle', run / 'review_bundle.zip'
    if bundle.exists() or archive.exists() or (run / 'review_bundle_manifest.json').exists():
        raise FileExistsError('refusing review bundle overwrite')
    names = review_metadata(run) + [Path('index.html'), Path('plot_manifest.json')]
    for row in images:
        p = Path(row['path'])
        if p.is_absolute():
            p = p.relative_to(run)
        names.extend([p, p.with_suffix('.json')])
    if len(names) != len(set(names)) or any(p.is_absolute() or '..' in p.parts for p in names):
        raise ValueError('unsafe or duplicate review allowlist')
    if any(not (run / p).is_file() for p in names):
        raise FileNotFoundError('review allowlist input missing')
    bundle.mkdir()
    inventory = []
    for name in names:
        target = bundle / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(run / name, target)
        inventory.append(dict(path=str(name), sha256=digest(target), bytes=target.stat().st_size))
    notes = bundle / 'README.md'
    notes.write_text('''# GP-SE2-DIAG-03 saved interpolation feasibility audit

Open index.html. All figures concern fixed saved GP support states and velocities. Hard final iterates remain rejected and were not executed. Four hard initial records are context; the benign saved final is a checker reference. No new optimization, controller solve, inference, rollout or GUI runtime was performed. Every image has a numeric/hash sidecar. The original physical acceptance is unchanged. Additional samples and post-hoc witnesses only locate or detect violations, without establishing that a newly refined optimization would find a valid solution. Reported maxima are sampled maxima. Body acceleration means the time derivative of body-twist components, not world or wheel acceleration.

The allowlisted package excludes raw RGB, source arrays, original solver histories, external source, environment exports, checkpoints, videos and caches. It contains small numeric tables, derived plotting arrays, protocol/source information and images. Source hashes identify the original records outside this package.
''')
    inventory.append(dict(path='README.md', sha256=digest(notes), bytes=notes.stat().st_size))
    write(bundle / 'manifest.json', dict(files=inventory, image_count=len(images),
        excludes='raw arrays, RGB, whole environment, checkpoints, original solver histories, external source',
        self_excluded='manifest.json'))
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED) as stream:
        for path in sorted(bundle.rglob('*')):
            if path.is_file():
                stream.write(path, str(path.relative_to(bundle)))
    result = dict(path=str(archive), sha256=digest(archive), bytes=archive.stat().st_size,
        image_count=len(images), files=len(inventory)+1, new_solves=0, new_rollouts=0, new_gui_runtime=0)
    write(run / 'review_bundle_manifest.json', result)
    return result


def generate(run, *, audit=None, environment=None, environments=None, dpi=160):
    run = Path(run).resolve()
    if dpi < 160:
        raise ValueError('at least 160 dpi required')
    for relative in ('plots', 'plot_manifest.json', 'index.html', 'review_bundle', 'review_bundle.zip', 'review_bundle_manifest.json'):
        if (run / relative).exists():
            raise FileExistsError('refusing plotting output overwrite: ' + str(run / relative))
    if audit is None:
        audit = read(run / 'audit.json')
    sources = [run / name for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'record_manifest.json', 'audit.json')]
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)
    sources.extend(sorted((run / 'aggregate').glob('*.csv')))
    numeric = expected_numeric(audit)
    if environment is not None:
        if environments is not None:
            raise ValueError('provide either environment or environments, not both')
        environments = dict(hard=environment, benign=environment)
    if environments is None:
        source = read(run / 'source.json')
        value = source.get('environment_path', source.get('environment'))
        if isinstance(value, dict):
            value = value.get('path')
        if not isinstance(value, str):
            raise ValueError('source must identify the preserved environment export')
        path = Path(value)
        if not path.is_absolute():
            path = ROOT / path
        environment = HospitalEnvironment.load(path)
        environments = dict(hard=environment, benign=environment)
    (run / 'plots').mkdir()
    functions = (lambda d: world_plot(d, environments), lateral_plot, speed_plot,
        acceleration_plot, interval_plot, worst_plot, detection_plot)
    images = []
    for name, title, plot in zip(PLOT_NAMES, TITLES, functions):
        fig = plot(numeric[name])
        images.append(_save(fig, run / 'plots' / (name+'.png'), numeric[name], sources, title, dpi=dpi))
    manifest = dict(images=images, plot_count=len(images), plotted_record_count=5,
        context_initial_record_count=4, source_record_count=len(audit['records']),
        label=LABEL, no_execution=True, gui_runtime_validated=False,
        plotter_sha256=digest(__file__))
    write(run / 'plot_manifest.json', manifest)
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
