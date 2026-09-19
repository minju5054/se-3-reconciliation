#!/usr/bin/env python3
"""Render saved REF-01 factor-isolation results; never execute an MPC solve.

Each PNG has a JSON sidecar with its plotted numbers, units and source hashes.
The four interventions retain their different row counts and source identities.
"""
from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diagnostics import file_sha256
from reconciliation.se2 import wrap_angle
from plot_gp_se2_02 import base_xy, environment_path, past_execution, plain, read, write

LABEL = 'OFFLINE REFERENCE-PREPARATION DIAGNOSTIC'
VARIANTS = ('R00_NATIVE', 'R10_SUFFIX_ONLY', 'R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER')
SHORT = dict(zip(VARIANTS, ('R00 Native', 'R10 Suffix only', 'R01 Resample only', 'R11 Current adapter')))
COLORS = dict(zip(VARIANTS, ('#333333', '#0072b2', '#d55e00', '#009e73')))
STYLES = dict(zip(VARIANTS, ('-', '--', '-.', ':')))
PLOT_NAMES = ('input_rows_world', 'input_yaw_vs_original_progress', 'matched_state_reference_targets',
              'selected_original_progress_vs_time', 'selected_reference_yaw_vs_time', 'actual_trajectory_overlay',
              'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
              'clearance_vs_time', 'outcome_summary')


def load_variants(case):
    out = {}
    for name in VARIANTS:
        p = Path(case) / 'variants' / name
        out[name] = dict(folder=p, reference=np.load(p / 'reference_world.npy', allow_pickle=False),
                         provenance=read(p / 'row_provenance.json'), geometry=read(p / 'geometry_audit.json'),
                         rollout=read(p / 'rollout.json'), metrics=read(p / 'metrics.json'))
        if len(out[name]['reference']) != len(out[name]['provenance']):
            raise ValueError('reference/lineage row count mismatch')
    return out


def square_bounds(parts, padding=.35):
    xy = np.vstack([np.asarray(p, float)[:, :2] for p in parts])
    middle = (np.min(xy, axis=0) + np.max(xy, axis=0))/2
    span = max(float(np.ptp(xy, axis=0).max()) + 2*padding, .7)
    return [middle[0]-span/2, middle[0]+span/2, middle[1]-span/2, middle[1]+span/2]


def geometry_numeric(context, route, variants, past):
    data = dict(old_world=context['old_world'], fresh_world=context['fresh_world'],
                actual_past_world=past['poses_world'], actual_past_times_relative_to_B_s=past['times_relative_to_B_s'],
                B_world=context['B_world'], goal_world=route['goal_world'], variants={})
    for name, item in variants.items():
        data['variants'][name] = dict(reference_world=item['reference'].tolist(), row_provenance=item['provenance'],
            actual_poses_world=[r['pose_world'] for r in item['rollout']['states']],
            actual_times_s=[r['time_s'] for r in item['rollout']['states']],
            geometry_audit=item['geometry'])
    parts = [data[k] for k in ('old_world', 'fresh_world', 'actual_past_world')]
    detail = [data['fresh_world'], [data['B_world'], data['goal_world']]]
    for item in data['variants'].values():
        parts.extend([item['reference_world'], item['actual_poses_world']])
        detail.extend([item['reference_world'], item['actual_poses_world']])
    data['axes_world_m'] = square_bounds(parts)
    data['detail_axes_world_m'] = square_bounds(detail, .12)
    return data


def selection_series(rows, matched=False):
    """Preserve exact official yaw and lineage; visualization unwrap is separate."""
    out = {}
    for name in VARIANTS:
        source = rows if matched else rows[name]['rollout']['controller_reference_selections']
        diagnostics = [r['variants'][name] if matched else r['selection_diagnostic'] for r in source]
        refs = np.asarray([d['reference_world'] for d in diagnostics])
        # Across control instants, remove only representational +/-2pi cuts.
        # This is NOT the controller's sequential unwrap inside one horizon.
        yaw_visual = np.unwrap(refs[:, :, 2], axis=0)
        out[name] = dict(times_s=[r['time_s'] for r in source],
            input_poses_world=[r['input_pose_world'] for r in source],
            selected_reference_world=refs.tolist(),
            official_reference_unwrapped_yaw_rad=refs[:, :, 2].tolist(),
            reference_yaw_visual_continuous_rad=yaw_visual.tolist(),
            nearest_original_progress=[d['nearest_original_fractional_row_coordinate'] for d in diagnostics],
            selected_original_progress=[d['selected_original_fractional_row_coordinates'] for d in diagnostics],
            first_target_distance_m=[d['selected_first_target_distance_m'] for d in diagnostics],
            last_target_distance_m=[d['selected_last_target_distance_m'] for d in diagnostics],
            horizon_xy_arc_length_m=[d['horizon_xy_arc_length_m'] for d in diagnostics],
            horizon_yaw_span_rad=[d['horizon_yaw_span_rad'] for d in diagnostics],
            final_goal_row_in_horizon=[d['final_goal_row_in_horizon'] for d in diagnostics],
            endpoint_repeated=[d['endpoint_repeated'] for d in diagnostics],
            nearest_tie_margin=[d['nearest_tie_margin'] for d in diagnostics])
    return out


def outcome_rows(variants):
    rows = []
    for name, item in variants.items():
        m = item['metrics']; e = m['execution']
        rows.append(dict(variant=name, success=m['primary_success'],
            position_error_m=e['terminal_position_error_m'], yaw_error_abs_deg=np.rad2deg(e['terminal_yaw_error_rad']),
            terminal_goal_dwell_pass=e['terminal_goal_dwell_pass'], required_goal_dwell_s=e['terminal_goal_dwell_s'],
            time_to_goal_s=e['time_to_goal_s'], minimum_clearance_m=m['minimum_clearance_m'],
            motion_valid=e['motion_limits_pass'], controller_failure_count=e['controller_failure_count'],
            failure_reasons=m['failure_reasons']))
    return plain(rows)


def save(fig, target, numeric, sources, title, notes=()):
    target = Path(target)
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError(target)
    fig.suptitle(title, fontsize=14)
    fig.text(.012, .008, LABEL + ' | saved results only | adjacent JSON contains exact values and source hashes', fontsize=8, color='#444444')
    fig.tight_layout(rect=(0, .035, 1, .94))
    fig.savefig(target, dpi=160)
    plt.close(fig)
    write(target.with_suffix('.json'), dict(image=target.name, image_sha256=file_sha256(target),
        label=LABEL, numeric_data=plain(numeric),
        source_hashes={str(Path(p).resolve()): file_sha256(p) for p in sources},
        plotter_sha256=file_sha256(__file__), dpi=160, missing_values='null; no fabricated values',
        notes=list(notes), new_inference=False, new_optimization=False, new_execution=False,
        gui_runtime_validated=False))
    return dict(path=str(target), sha256=file_sha256(target), sidecar_sha256=file_sha256(target.with_suffix('.json')))


def arrows(ax, poses, color, length=.07):
    a = np.asarray(poses)
    ax.quiver(a[:, 0], a[:, 1], np.cos(a[:, 2])*length, np.sin(a[:, 2])*length,
              angles='xy', scale_units='xy', scale=1, color=color, width=.004, zorder=7)


def decorate(ax, ylabel, xlabel='simulation time after B [s]'):
    ax.set(xlabel=xlabel, ylabel=ylabel); ax.grid(alpha=.2)


def plot_case(run, row, environment, config):
    case = run / 'cases' / row['case_directory']; folder = case / 'plots'
    if folder.exists():
        raise FileExistsError(folder)
    variants = load_variants(case)
    context, route = read(case / 'input_context.json'), read(case / 'goal_route.json')
    matched = read(case / 'matched_state_probes.json')
    if len(matched['states']) != 30:
        raise ValueError('all 30 matched states are required')
    past = past_execution(case)
    xy = geometry_numeric(context, route, variants, past)
    xy['reference_factorization'] = read(case / 'reference_factorization.json')
    closed = selection_series(variants)
    probes = selection_series(matched['states'], matched=True)
    sources = [run / name for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'case_manifest.json')]
    sources.extend(case / name for name in ('input_context.json', 'goal_route.json', 'reference_factorization.json',
                                           'matched_state_probes.json', 'reproduction_check.json', 'actual_past_execution.json'))
    for item in variants.values():
        sources.extend(item['folder'] / name for name in ('reference_world.npy', 'row_provenance.json',
                                                         'geometry_audit.json', 'rollout.json', 'metrics.json'))
    folder.mkdir(); images = []
    title = row['case_id']
    def output(name, fig, numeric, subtitle, notes=()):
        images.append(save(fig, folder / (name+'.png'), numeric, sources, title+' | '+subtitle, notes))

    fig, axes = plt.subplots(2, 4, figsize=(19, 10))
    for j, name in enumerate(VARIANTS):
        item = variants[name]; a = item['reference']; color = COLORS[name]
        for i in range(2):
            ax = axes[i, j]; bounds = xy['axes_world_m'] if i == 0 else xy['detail_axes_world_m']
            base_xy(ax, xy, environment, bounds, config)
            ax.plot(a[:, 0], a[:, 1], STYLES[name], color=color, lw=1.2)
            ax.scatter(a[:, 0], a[:, 1], color=color, s=15)
            first_source = min(r['original_fractional_row_coordinate'] for r in item['provenance'])
            removed = np.asarray(context['fresh_world'])[:int(first_source)]
            if len(removed):
                ax.scatter(removed[:, 0], removed[:, 1], color='#777777', marker='x', s=65,
                           label='removed original prefix; explanatory only', zorder=8)
            arrows(ax, a, color, .09 if i == 0 else .045)
            if i == 1:
                for n, p in enumerate(a):
                    ax.annotate(str(n), p[:2], xytext=(4, 4 if n % 2 == 0 else -10), textcoords='offset points', fontsize=7, color=color)
            ax.set_title(SHORT[name]+f' | {len(a)} rows'+ (' | common context' if i == 0 else ' | common detail scale'), fontsize=10)
    axes[0, 0].legend(fontsize=6, loc='lower left')
    output('input_rows_world', fig, xy, 'Input rows and headings; labels are derived row indices',
           ['Both context and detail bounds are shared by all four variants. Original past/OLD/FRESH remain context only.',
            'Row identities remain separate even at duplicate XY. World axes are metres; no geometry exaggeration.'])

    numeric = dict(variants={}, original_native_world=context['fresh_world'])
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for name, item in variants.items():
        lineage = item['provenance']; x = [r['original_fractional_row_coordinate'] for r in lineage]
        yaw = [r['unwrapped_yaw'] for r in lineage]
        numeric['variants'][name] = dict(original_fractional_row_coordinate=x, wrapped_yaw_rad=[r['wrapped_yaw'] for r in lineage],
                                         original_branch_unwrapped_yaw_rad=yaw, row_provenance=lineage)
        axes[0].plot(x, np.rad2deg(yaw), STYLES[name], marker='o', ms=3, color=COLORS[name], label=SHORT[name])
        axes[1].plot(x, np.arange(len(x)), STYLES[name], marker='o', ms=3, color=COLORS[name], label=SHORT[name])
    decorate(axes[0], 'yaw on original continuous branch [deg]', 'original fractional row coordinate')
    decorate(axes[1], 'derived row index', 'original fractional row coordinate (row order; not physical time or distance)')
    axes[0].legend(ncol=2)
    output('input_yaw_vs_original_progress', fig, numeric, 'Input yaw and row density', ['Wrapped yaw is retained in JSON; plotted yaw uses the original native unwrap branch.'])

    snapshot = [matched['states'][i] for i in (0, 10, 20)]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, state in zip(axes, snapshot):
        base_xy(ax, xy, environment, xy['detail_axes_world_m'], config)
        p = np.asarray(state['input_pose_world']); ax.scatter(*p[:2], marker='D', s=70, color='#763393', zorder=10)
        arrows(ax, [p], '#763393', .1)
        for name in VARIANTS:
            r = np.asarray(state['variants'][name]['reference_world'])
            ax.plot(r[:, 0], r[:, 1], STYLES[name], marker='o', ms=4, color=COLORS[name], label=SHORT[name])
            arrows(ax, r, COLORS[name], .045)
        ax.set_title(f"Matched Native input pose, t = {state['time_s']:.1f} s")
    axes[0].legend(fontsize=7, loc='lower left')
    output('matched_state_reference_targets', fig, dict(world_geometry=xy, snapshots=snapshot),
           'Same-state five-target horizons; no solve or integration in probes', ['Snapshot input poses come from the historical GP-SE2-02 Native rollout.'])

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    for col, (label, series) in enumerate((('Matched historical Native states', probes), ('Independent closed-loop states', closed))):
        for name, s in series.items():
            progress = np.asarray(s['selected_original_progress']); t = s['times_s']
            axes[0, col].plot(t, progress[:, 0], STYLES[name], color=COLORS[name], label=SHORT[name]+' first')
            axes[0, col].fill_between(t, progress[:, 0], progress[:, -1], color=COLORS[name], alpha=.09)
            axes[1, col].plot(t, s['last_target_distance_m'], STYLES[name], color=COLORS[name], label=SHORT[name])
        axes[0, col].set_title(label)
        decorate(axes[0, col], 'selected original row progress; shading first–last')
        decorate(axes[1, col], 'distance from input pose to fifth target [m]')
    axes[0, 0].legend(fontsize=8, ncol=2)
    output('selected_original_progress_vs_time', fig, dict(matched_state=probes, closed_loop=closed), 'Selection progress and physical lookahead',
           ['Original fractional coordinates compare source identity, not local row numbers.', 'Shading spans first to fifth selected row; endpoint repetition is retained.'])

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    for col, (label, series) in enumerate((('Matched historical Native states', probes), ('Independent closed-loop states', closed))):
        for name, s in series.items():
            r = np.rad2deg(s['reference_yaw_visual_continuous_rad']); t = s['times_s']
            axes[0, col].plot(t, r[:, 0], STYLES[name], color=COLORS[name], label=SHORT[name]+' first')
            axes[0, col].plot(t, r[:, -1], STYLES[name], color=COLORS[name], alpha=.45, lw=2.4)
            axes[1, col].plot(t, np.rad2deg(s['horizon_yaw_span_rad']), STYLES[name], color=COLORS[name], label=SHORT[name])
        axes[0, col].set_title(label)
        decorate(axes[0, col], 'selected yaw [deg]; pale line = fifth target')
        decorate(axes[1, col], 'official sequential horizon yaw span [deg]')
    axes[0, 0].legend(fontsize=8, ncol=2)
    output('selected_reference_yaw_vs_time', fig, dict(matched_state=probes, closed_loop=closed), 'Official reference yaw and remaining horizon turn',
           ['Exact official sequentially unwrapped yaw is stored. Across-time curves separately remove representation cuts by numpy.unwrap, never modify MPC inputs.'])

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, key in zip(axes, ('axes_world_m', 'detail_axes_world_m')):
        base_xy(ax, xy, environment, xy[key], config)
        for name, item in xy['variants'].items():
            a = np.asarray(item['actual_poses_world']); ax.plot(a[:, 0], a[:, 1], STYLES[name], color=COLORS[name], lw=2.2, label=SHORT[name]+' new execution')
            arrows(ax, a[::30], COLORS[name], .1 if key == 'axes_world_m' else .06)
        ax.set_title('Common context' if key == 'axes_world_m' else 'Common detail scale')
    axes[0].legend(fontsize=7)
    output('actual_trajectory_overlay', fig, xy, 'Eight fresh counterfactuals; this case has four independent executions')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True); numeric = {}
    for name, item in variants.items():
        m = item['metrics']; a = np.asarray(m['dense_poses_world']); t = np.asarray(m['dense_times_s'])
        raw = a[:, 2]; continuous = np.unwrap(raw); signed = wrap_angle(raw-route['goal_world'][2]); distance = np.linalg.norm(a[:, :2]-np.asarray(route['goal_world'])[:2], axis=1)
        numeric[name] = dict(times_s=t, actual_wrapped_yaw_rad=raw, actual_visual_unwrapped_yaw_rad=continuous,
                             goal_relative_wrapped_yaw_error_rad=signed, goal_position_error_m=distance,
                             goal_world=route['goal_world'])
        axes[0].plot(t, np.rad2deg(continuous), STYLES[name], color=COLORS[name], label=SHORT[name])
        axes[1].plot(t, np.rad2deg(signed), STYLES[name], color=COLORS[name])
        axes[2].plot(t, distance, STYLES[name], color=COLORS[name])
    yaw_limit = np.rad2deg(config['formulation']['goal_yaw_tolerance'])
    for value in (-yaw_limit, yaw_limit): axes[1].axhline(value, color='#555555', ls='--', lw=.8)
    axes[2].axhline(config['formulation']['goal_position_tolerance'], color='#555555', ls='--', lw=.8)
    for ax in axes:
        ax.axvspan(3-config['evaluation']['terminal_goal_dwell_s'], 3, color='#aaaaaa', alpha=.15)
    axes[0].legend(ncol=2)
    for ax, ylabel in zip(axes, ('actual continuous yaw [deg]', 'wrapped yaw error to original goal [deg]', 'position error to original goal [m]')): decorate(ax, ylabel)
    output('actual_yaw_and_goal_error', fig, dict(variants=numeric, config_thresholds=config['formulation'], required_dwell_s=config['evaluation']['terminal_goal_dwell_s']),
           'Actual yaw, original-goal error and required terminal dwell window', ['Grey band is the required final dwell interval, not an achieved-duration measurement.'])

    for component, name, label in ((0, 'linear_command_vs_time', 'applied linear velocity [m/s]'), (1, 'angular_command_vs_time', 'applied angular velocity [rad/s]')):
        fig, ax = plt.subplots(figsize=(12, 5)); numeric = {}
        for variant, item in variants.items():
            rows = item['rollout']['controller_reference_selections']; ts = [r['time_s'] for r in rows]+[3.]
            commands = [r['command'][component] for r in rows]+[rows[-1]['command'][component]]
            numeric[variant] = dict(times_s=ts, applied_commands=commands,
                initial_physical_command=context['u_minus'][component], initial_controller_memory=context['previous_control'][component])
            ax.step(ts, commands, where='post', color=COLORS[variant], ls=STYLES[variant], label=SHORT[variant])
        ax.scatter([0], [context['u_minus'][component]], marker='x', color='black', s=80, label='physical incoming command')
        ax.scatter([0], [context['previous_control'][component]], marker='o', facecolors='none', edgecolors='black', s=80, label='controller memory')
        decorate(ax, label); ax.legend(ncol=3, fontsize=8)
        output(name, fig, numeric, 'Actual applied MPC commands; 0.1 s holds', ['No reference derivative or planned GP velocity is used as an applied command.'])

    fig, ax = plt.subplots(figsize=(12, 5)); numeric = {}
    for name, item in variants.items():
        m = item['metrics']; numeric[name] = dict(times_s=m['dense_times_s'], footprint_clearance_m=m['environment']['clearance_samples_m'])
        ax.plot(m['dense_times_s'], m['environment']['clearance_samples_m'], STYLES[name], color=COLORS[name], label=SHORT[name])
    ax.axhline(config['footprint']['required_clearance_m'], color='red', ls='--', label='required edge clearance')
    ax.axhline(0, color='#555555', lw=.8, label='footprint overlap boundary')
    decorate(ax, 'footprint edge clearance [m]'); ax.legend(ncol=3, fontsize=8)
    output('clearance_vs_time', fig, dict(variants=numeric, required_edge_clearance_m=config['footprint']['required_clearance_m']), 'Actual footprint clearance in unchanged environment')

    outcomes = outcome_rows(variants); fig, axes = plt.subplots(1, 3, figsize=(17, 5), gridspec_kw={'width_ratios':[1, 1, 2.1]})
    x = np.arange(4)
    for ax, key, threshold, ylabel in ((axes[0], 'yaw_error_abs_deg', yaw_limit, 'terminal |yaw error| [deg]'),
                                     (axes[1], 'position_error_m', config['formulation']['goal_position_tolerance'], 'terminal position error [m]')):
        ax.bar(x, [r[key] for r in outcomes], color=[COLORS[n] for n in VARIANTS]); ax.axhline(threshold, color='red', ls='--')
        ax.set_xticks(x, ['R00', 'R10', 'R01', 'R11']); decorate(ax, ylabel, 'variant')
    axes[2].axis('off')
    cells = [[SHORT[r['variant']], 'PASS' if r['success'] else 'FAIL', 'PASS' if r['terminal_goal_dwell_pass'] else 'FAIL',
              'N/A' if r['time_to_goal_s'] is None else f"{r['time_to_goal_s']:.3f}", 'PASS' if r['motion_valid'] else 'FAIL'] for r in outcomes]
    table = axes[2].table(cellText=cells, colLabels=['Variant', 'Success', 'Dwell', 'Goal time [s]', 'Motion'], loc='center', cellLoc='center', colWidths=[.35,.16,.16,.18,.16])
    table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1., 2.)
    output('outcome_summary', fig, dict(rows=outcomes), 'Original success predicates; failures and absent goal times retained', ['Dwell is the original final 0.20 s predicate. No arbitrary aggregate score.'])
    write(folder / 'plot_provenance.json', dict(case_id=row['case_id'], variants=list(VARIANTS), images=images,
        axes_world_m=xy['axes_world_m'], detail_axes_world_m=xy['detail_axes_world_m'], gui_runtime_validated=False))
    return images


def main(run):
    run = Path(run).resolve()
    if (run / 'plot_manifest.json').exists() or (run / 'index.html').exists():
        raise FileExistsError('refusing plot/index overwrite')
    rows = read(run / 'case_manifest.json')['selected']
    config = yaml.safe_load((run / 'config_snapshot.yaml').read_text())
    environment = HospitalEnvironment.load(environment_path(run)); images = []
    for row in rows:
        images.extend(plot_case(run, row, environment, config))
    for image in images: image['path'] = str(Path(image['path']).relative_to(run))
    write(run / 'plot_manifest.json', dict(experiment='GP-SE2-REF-01', label=LABEL,
        variants=list(VARIANTS), images=images, case_count=len(rows), image_count=len(images),
        gui_runtime_validated=False, plotter_sha256=file_sha256(__file__)))
    lines = ['<!doctype html><html><head><meta charset="utf-8"><title>REF-01 factor isolation</title>',
        '<style>body{font:16px system-ui;max-width:1500px;margin:25px auto;padding:15px}img{max-width:100%;border:1px solid #ddd}article{margin:30px 0}nav a{margin-right:18px}code{font-size:14px}</style></head><body>',
        '<h1>'+LABEL+'</h1>', '<p>Two fixed events, four input variants, eight independent saved MPC rollouts. No new GP optimization or VLA inference. Plots are numerical evidence; this run does not claim GUI runtime validation.</p>',
        '<p>R00 = Native; R10 = suffix only; R01 = resample only; R11 = current adapter. Source row progress is not physical time. Success uses the original goal, motion, clearance and final dwell predicates.</p>',
        '<p><a href="aggregate/summary.json">Diagnosis summary</a> · <a href="aggregate/rollout_outcomes.csv">All outcomes</a> · <a href="aggregate/factor_contrasts.csv">Factor contrasts</a> · <a href="source.json">Source hashes</a> · <a href="protocol.json">Frozen protocol</a></p>', '<nav>']
    for row in rows: lines.append(f'<a href="#{html.escape(row["case_directory"])}">{html.escape(row["case_id"])}</a>')
    lines.append('</nav>')
    for row in rows:
        lines.append(f'<h2 id="{html.escape(row["case_directory"])}">{html.escape(row["case_id"])}</h2>')
        for name in PLOT_NAMES:
            p = f'cases/{row["case_directory"]}/plots/{name}'
            lines.append(f'<article><h3>{html.escape(name.replace("_", " "))}</h3><a href="{p}.png"><img loading="lazy" src="{p}.png" alt="{name}"></a><p><a href="{p}.json">Plotted numbers and source hashes</a></p></article>')
    (run / 'index.html').write_text('\n'.join(lines+['</body></html>']))
    return dict(case_count=len(rows), image_count=len(images), gui_runtime_validated=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--run', type=Path, required=True)
    print(json.dumps(main(parser.parse_args().run), indent=2))
