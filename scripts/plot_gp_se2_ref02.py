#!/usr/bin/env python3
"""Plot saved REF-02 lookahead-selection diagnostics; never run a controller.

B and C share one byte-identical input. Their curves are never spatially offset
for presentation. Yaw visualization branches are distinct from solver records.
"""
from __future__ import annotations
import argparse
import html
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from reconciliation.gp_se2_diagnostics import file_sha256
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.se2 import wrap_angle
from plot_gp_se2_02 import base_xy, environment_path, past_execution, plain, read, write

LABEL = 'OFFLINE LOOKAHEAD-SELECTION DIAGNOSTIC'
METHODS = ('A_NATIVE', 'B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS')
SHORT = dict(zip(METHODS, ('A Native / next row', 'B Dense / next row', 'C Dense / source progress')))
COLORS = dict(zip(METHODS, ('#333333', '#d55e00', '#0072b2')))
STYLES = dict(zip(METHODS, ('-', '--', '-.')))
PLOT_NAMES = ('fixed_input_world', 'matched_state_targets', 'selected_source_progress_vs_time',
              'selected_target_yaw_vs_time', 'goal_in_horizon_vs_time', 'actual_trajectory_overlay',
              'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
              'clearance_vs_time', 'outcome_summary')


def load_methods(case):
    result = {}
    for name in METHODS:
        folder = Path(case)/'methods'/name
        result[name] = dict(folder=folder, reference=np.load(folder/'reference_world.npy', allow_pickle=False),
            provenance=read(folder/'row_provenance.json'), rollout=read(folder/'rollout.json'), metrics=read(folder/'metrics.json'))
        if len(result[name]['reference']) != len(result[name]['provenance']):
            raise ValueError('reference and row identity coverage mismatch')
    identity = input_identity(result)
    if not all(identity[k] for k in ('byte_identical', 'array_identical', 'lineage_identical')):
        raise ValueError('B/C must have byte-identical references and equal source lineage')
    return result


def input_identity(methods):
    b, c = [methods[n] for n in METHODS[1:]]
    paths = [p['folder']/'reference_world.npy' for p in (b, c)]
    hashes = [file_sha256(p) for p in paths]
    return dict(methods=list(METHODS[1:]), file_sha256=dict(zip(METHODS[1:], hashes)),
                byte_identical=paths[0].read_bytes() == paths[1].read_bytes(),
                array_identical=bool(b['reference'].dtype == c['reference'].dtype
                    and np.array_equal(b['reference'], c['reference'])),
                lineage_identical=b['provenance'] == c['provenance'],
                representation='one shared dense input; only target selector differs; no spatial offsets',
                original_fractional_coordinate_units='source row order, not physical time or distance')


def square_bounds(parts, *, padding=.15, minimum_span=.7):
    xy = np.vstack([np.asarray(p, float)[:, :2] for p in parts])
    center = (xy.min(0)+xy.max(0))/2
    span = max(float(np.ptp(xy, axis=0).max())+2*padding, minimum_span)
    return [float(center[0]-span/2), float(center[0]+span/2), float(center[1]-span/2), float(center[1]+span/2)]


def geometry_numeric(context, route, methods, past):
    result = dict(old_world=context['old_world'], fresh_world=context['fresh_world'],
        actual_past_world=past['poses_world'], actual_past_times_relative_to_B_s=past['times_relative_to_B_s'],
        B_world=context['B_world'], goal_world=route['goal_world'], methods={})
    parts = [result[k] for k in ('old_world', 'fresh_world', 'actual_past_world')]
    detail = [result['fresh_world'], [result['B_world'], result['goal_world']]]
    for name, item in methods.items():
        row = dict(reference_world=item['reference'].tolist(), row_provenance=item['provenance'],
            actual_poses_world=[r['pose_world'] for r in item['rollout']['states']],
            actual_times_s=[r['time_s'] for r in item['rollout']['states']])
        result['methods'][name] = row
        parts.extend([row['reference_world'], row['actual_poses_world']])
        detail.extend([row['reference_world'], row['actual_poses_world']])
    result['axes_world_m'] = square_bounds(parts, padding=.35)
    result['detail_axes_world_m'] = square_bounds(detail)
    result['input_axes_world_m'] = square_bounds([m['reference'] for m in methods.values()], padding=.008, minimum_span=.04)
    return result


def selection_series(records, *, matched):
    result = {}
    for name in METHODS:
        source = records if matched else records[name]['rollout']['controller_reference_selections']
        diagnostics = [r['methods'][name] if matched else r['selection_diagnostic'] for r in source]
        refs = np.asarray([r['reference_world'] for r in diagnostics])
        result[name] = dict(times_s=[r['time_s'] for r in source], input_poses_world=[r['input_pose_world'] for r in source],
            selected_reference_world=refs.tolist(), official_reference_unwrapped_yaw_rad=refs[:, :, 2].tolist(),
            display_unwrapped_reference_yaw_rad=np.unwrap(refs[:, :, 2], axis=0).tolist(),
            nearest_original_progress=[r['nearest_original_fractional_row_coordinate'] for r in diagnostics],
            selected_original_progress=[r['selected_original_fractional_row_coordinates'] for r in diagnostics],
            first_target_distance_m=[r['selected_first_target_distance_m'] for r in diagnostics],
            last_target_distance_m=[r['selected_last_target_distance_m'] for r in diagnostics],
            horizon_xy_arc_length_m=[r['horizon_xy_arc_length_m'] for r in diagnostics],
            horizon_yaw_span_rad=[r['horizon_yaw_span_rad'] for r in diagnostics],
            final_goal_row_in_horizon=[r['final_goal_row_in_horizon'] for r in diagnostics],
            endpoint_repeated=[r['endpoint_repeated'] for r in diagnostics],
            requested_q_h=[r.get('q_h') for r in diagnostics],
            progress_overshoot=[r.get('progress_overshoot') for r in diagnostics],
            # Full records retain exact q/ceiling/overshoot names and branch metadata.
            selection_diagnostics=diagnostics)
    return result


def wrapped_curve_segments(times, angles):
    """Break a wrapped-error display at a representation cut; preserve samples."""
    t, a = np.asarray(times), np.asarray(angles)
    if t.shape != a.shape or t.ndim != 1:
        raise ValueError('aligned one-dimensional time/angle samples required')
    cuts = np.r_[0, np.flatnonzero(np.abs(np.diff(a)) > np.pi)+1, len(a)]
    return [dict(times_s=t[l:r].tolist(), angles_rad=a[l:r].tolist()) for l, r in zip(cuts[:-1], cuts[1:]) if r > l]


def outcome_rows(methods):
    rows = []
    for name, item in methods.items():
        m = item['metrics']; e = m['execution']
        rows.append(dict(method=name, success=bool(m['primary_success']),
            position_error_m=e['terminal_position_error_m'], yaw_error_abs_deg=float(np.rad2deg(e['terminal_yaw_error_rad'])),
            terminal_goal_dwell_pass=e['terminal_goal_dwell_pass'], required_goal_dwell_s=e['terminal_goal_dwell_s'],
            time_to_goal_s=e['time_to_goal_s'], minimum_clearance_m=m['minimum_clearance_m'],
            motion_valid=e['motion_limits_pass'], controller_failure_count=e['controller_failure_count'], failure_reasons=m['failure_reasons']))
    return rows


def expected_numeric(context, route, methods, past, probes, config):
    """Shared numeric export entry point, allowing artifact-only recomputation."""
    xy = geometry_numeric(context, route, methods, past)
    matched, closed = selection_series(probes['states'], matched=True), selection_series(methods, matched=False)
    series = dict(matched_state=matched, closed_loop=closed)
    result = dict(fixed_input_world=xy, actual_trajectory_overlay=xy,
        matched_state_targets=dict(world_geometry=xy, snapshots=[probes['states'][i] for i in (0, 10, 20)]),
        selected_source_progress_vs_time=series, selected_target_yaw_vs_time=series,
        goal_in_horizon_vs_time=series, outcome_summary=dict(rows=outcome_rows(methods)))
    yaw, clearance = {}, {}
    for name, item in methods.items():
        m = item['metrics']; a = np.asarray(m['dense_poses_world']); times = m['dense_times_s']
        signed = wrap_angle(a[:, 2]-route['goal_world'][2])
        yaw[name] = dict(times_s=times, actual_wrapped_yaw_rad=a[:, 2].tolist(),
            actual_visual_unwrapped_yaw_rad=np.unwrap(a[:, 2]).tolist(), goal_relative_wrapped_yaw_error_rad=signed.tolist(),
            goal_error_display_segments=wrapped_curve_segments(times, signed),
            goal_position_error_m=np.linalg.norm(a[:, :2]-np.asarray(route['goal_world'])[:2], axis=1).tolist(), goal_world=route['goal_world'])
        clearance[name] = dict(times_s=times, footprint_clearance_m=m['environment']['clearance_samples_m'])
    result['actual_yaw_and_goal_error'] = dict(methods=yaw, config_thresholds=config['formulation'], required_dwell_s=config['evaluation']['terminal_goal_dwell_s'])
    result['clearance_vs_time'] = dict(methods=clearance, required_edge_clearance_m=config['footprint']['required_clearance_m'])
    for component, name in enumerate(('linear_command_vs_time', 'angular_command_vs_time')):
        result[name] = {}
        for method, item in methods.items():
            solves = item['rollout']['controller_reference_selections']
            result[name][method] = dict(times_s=[r['time_s'] for r in solves]+[3.],
                applied_commands=[r['command'][component] for r in solves]+[solves[-1]['command'][component]],
                initial_physical_command=context['u_minus'][component], initial_controller_memory=context['previous_control'][component])
    return result


def arrows(ax, poses, color, length):
    p = np.asarray(poses)
    ax.quiver(p[:, 0], p[:, 1], np.cos(p[:, 2])*length, np.sin(p[:, 2])*length,
              angles='xy', scale_units='xy', scale=1, color=color, width=.0035, zorder=7)


def decorate_time(ax, ylabel):
    ax.set(xlim=(0., 3.), xlabel='simulation time after original B [s]', ylabel=ylabel)
    ax.grid(alpha=.2)


def save(fig, target, numeric, sources, identity, title, notes=()):
    target = Path(target)
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError('refusing plot or sidecar overwrite')
    fig.suptitle(title, fontsize=15)
    fig.text(.012, .008, LABEL+' | saved results only | B/C input byte-identical | adjacent JSON: exact values/hashes', fontsize=8)
    fig.tight_layout(rect=(0, .035, 1, .94))
    fig.savefig(target, dpi=160); plt.close(fig)
    write(target.with_suffix('.json'), dict(image=target.name, image_sha256=file_sha256(target), label=LABEL,
        numeric_data=plain(numeric), input_identity=identity,
        source_hashes={str(Path(p).resolve()):file_sha256(p) for p in sources},
        plotter_sha256=file_sha256(__file__), dpi=160, notes=list(notes),
        missing_values='null; never filled with zero or horizon', new_inference=False, new_optimization=False,
        new_execution=False, gui_runtime_validated=False))
    return dict(path=str(target), sha256=file_sha256(target), sidecar_sha256=file_sha256(target.with_suffix('.json')))


def plot_case(run, row, env, config):
    case = run/'cases'/row['case_directory']; output = case/'plots'
    if output.exists():
        raise FileExistsError('refusing plot directory overwrite')
    methods = load_methods(case); identity = input_identity(methods)
    context, route, probes = [read(case/p) for p in ('input_context.json', 'goal_route.json', 'matched_state_probes.json')]
    if len(probes['states']) != 30:
        raise ValueError('all 30 matched historical input poses required')
    past = past_execution(case); numeric = expected_numeric(context, route, methods, past, probes, config)
    xy = numeric['fixed_input_world']; sources = [run/p for p in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'case_manifest.json', 'selector_definition.json')]
    sources += [case/p for p in ('input_context.json', 'goal_route.json', 'matched_state_probes.json', 'actual_past_execution.json')]
    for item in methods.values():
        sources += [item['folder']/p for p in ('reference_world.npy', 'row_provenance.json', 'rollout.json', 'metrics.json')]
    output.mkdir(); images = []
    def finish(name, fig, title, notes=()):
        images.append(save(fig, output/(name+'.png'), numeric[name], sources, identity, row['case_id']+' | '+title, notes))

    # Explicitly two input curves: B and C share one panel and complete row table.
    fig, axes = plt.subplots(2, 3, figsize=(24, 16), gridspec_kw={'width_ratios':[1, 1, 1.55]})
    for k, name in enumerate((METHODS[0], METHODS[1])):
        item = methods[name]; ref = item['reference']; color = COLORS[name]
        label = SHORT[name] if k == 0 else 'B and C: same dense input bytes'
        base_xy(axes[k, 0], xy, env, xy['axes_world_m'], config)
        axes[k, 0].plot(ref[:, 0], ref[:, 1], color=color, marker='o', ms=3)
        axes[k, 0].set_title(label+'\nUnchanged world context', fontsize=11)
        axes[k, 0].legend(fontsize=7, loc='lower left')
        ax = axes[k, 1]; bounds = xy['input_axes_world_m']; side = bounds[1]-bounds[0]
        native = np.asarray(context['fresh_world'])
        ax.plot(native[:, 0], native[:, 1], ':', color='#888888', lw=1, label='original FRESH context')
        ax.plot(ref[:, 0], ref[:, 1], color=color, marker='o', ms=4, label=label)
        arrows(ax, ref, color, side*.06)
        ax.scatter(*ref[0, :2], facecolors='none', edgecolors=color, s=95)
        ax.scatter(*ref[-1, :2], marker='*', color=color, s=90)
        ax.annotate('first #0', ref[0, :2], xytext=(7, -15), textcoords='offset points', fontsize=8)
        ax.annotate(f'last #{len(ref)-1}', ref[-1, :2], xytext=(7, 12), textcoords='offset points', fontsize=8)
        ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='world x [m]', ylabel='world y [m]', aspect='equal')
        ax.ticklabel_format(useOffset=False); ax.grid(alpha=.2); ax.legend(fontsize=7, loc='lower left')
        ax.set_title(f'Common input zoom; {len(ref)} rows\nHeading glyph length {side*.06:.4f} m; direction only', fontsize=11)
        ax = axes[k, 2]; ax.axis('off')
        cells = [[str(r['derived_row_index']), f"{r['original_fractional_row_coordinate']:.4f}",
                  f"{r['world_xy'][0]:.6f}", f"{r['world_xy'][1]:.6f}", f"{np.rad2deg(r['wrapped_yaw']):.3f}"] for r in item['provenance']]
        table = ax.table(cellText=cells, colLabels=['Row #','Source s','World x [m]','World y [m]','Yaw [deg]'],
                         loc='center', cellLoc='right', colLoc='center', colWidths=[.12,.17,.245,.245,.20])
        table.auto_set_font_size(False); table.set_fontsize(8); table.scale(1., 1.38)
        ax.set_title('All row identities; duplicate positions remain distinct\nSource s is row order, not physical time or distance', fontsize=11)
    finish('fixed_input_world', fig, 'Fixed inputs: native versus one shared B/C dense array',
           ['B/C are drawn once because arrays and file hashes are identical; no visual offsets.',
            'Only first/last XY labels are annotated; complete source-row identities appear in the separate table and JSON.',
            'Heading glyphs indicate direction only, not motion or speed.'])

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, snapshot in zip(axes, numeric['matched_state_targets']['snapshots']):
        base_xy(ax, xy, env, xy['detail_axes_world_m'], config)
        pose = np.asarray(snapshot['input_pose_world']); ax.scatter(*pose[:2], marker='D', color='#763393', s=65, zorder=9)
        arrows(ax, [pose], '#763393', .07)
        for name in METHODS:
            ref = np.asarray(snapshot['methods'][name]['reference_world'])
            ax.plot(ref[:, 0], ref[:, 1], STYLES[name], color=COLORS[name], marker='o', ms=4, label=SHORT[name])
            arrows(ax, ref, COLORS[name], .035)
        ax.set_title(f"Same Native probe pose, t={snapshot['time_s']:.1f} s")
    axes[0].legend(fontsize=7, loc='lower left')
    finish('matched_state_targets', fig, 'Five-target horizons at identical states; no probe solve')

    series = numeric['selected_source_progress_vs_time']
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    for col, (mode, title) in enumerate((('matched_state','Matched historical Native poses'), ('closed_loop','Independent closed-loop poses'))):
        for name, record in series[mode].items():
            progress = np.asarray(record['selected_original_progress']); ts = record['times_s']
            axes[0, col].plot(ts, progress[:, 0], STYLES[name], color=COLORS[name], label=SHORT[name]+' first')
            axes[0, col].plot(ts, progress[:, -1], STYLES[name], color=COLORS[name], alpha=.45, lw=2)
            axes[0, col].fill_between(ts, progress[:, 0], progress[:, -1], color=COLORS[name], alpha=.07)
            if name == 'C_DENSE_SOURCE_PROGRESS' and all(q is not None for q in record['requested_q_h']):
                q = np.asarray(record['requested_q_h'])
                axes[0, col].plot(ts, q[:, 0], ':', color='#254e70', lw=.9, label='C requested q (first)')
                axes[0, col].plot(ts, q[:, -1], ':', color='#254e70', lw=.9, alpha=.5)
            axes[1, col].plot(ts, record['last_target_distance_m'], STYLES[name], color=COLORS[name], label=SHORT[name])
        axes[0, col].set_title(title)
        decorate_time(axes[0, col], 'selected source s; first / pale fifth target')
        decorate_time(axes[1, col], 'input-pose to fifth target distance [m]')
    axes[0, 0].legend(fontsize=8)
    finish('selected_source_progress_vs_time', fig, 'Source progress and physical lookahead', ['Full selector diagnostics preserve C requested q and ceiling overshoot; selected s is not physical time.'])

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    for col, mode in enumerate(('matched_state','closed_loop')):
        for name, record in series[mode].items():
            angles = np.rad2deg(record['display_unwrapped_reference_yaw_rad']); ts = record['times_s']
            axes[0, col].plot(ts, angles[:, 0], STYLES[name], color=COLORS[name], label=SHORT[name])
            axes[0, col].plot(ts, angles[:, -1], STYLES[name], color=COLORS[name], alpha=.45, lw=2)
            axes[1, col].plot(ts, np.rad2deg(record['horizon_yaw_span_rad']), STYLES[name], color=COLORS[name])
        axes[0, col].set_title('Matched states' if col == 0 else 'Independent closed loop')
        decorate_time(axes[0, col], 'selected yaw [deg]; first / pale fifth target')
        decorate_time(axes[1, col], 'sequential selected horizon yaw span [deg]')
    axes[0, 0].legend(fontsize=8)
    finish('selected_target_yaw_vs_time', fig, 'Actual selected target yaw', ['Stored official sequential yaw is preserved; plotting removes only across-time representation cuts.'])

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for ax, mode in zip(axes, ('matched_state','closed_loop')):
        for name, record in series[mode].items():
            ax.step(record['times_s'], np.asarray(record['final_goal_row_in_horizon'], int), where='post',
                    color=COLORS[name], ls=STYLES[name], label=SHORT[name])
        decorate_time(ax, 'original goal row in selected horizon')
        ax.set(ylim=(-.1,1.1), yticks=[0,1], yticklabels=['No','Yes'], title='Matched states' if mode == 'matched_state' else 'Independent closed loop')
    axes[0].legend(ncol=3, fontsize=8)
    finish('goal_in_horizon_vs_time', fig, 'When the original goal enters the selected horizon', ['Final-row retention in the input alone does not imply horizon inclusion. Last selector call is at 2.9 seconds.'])

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, key in zip(axes, ('axes_world_m','detail_axes_world_m')):
        base_xy(ax, xy, env, xy[key], config)
        for name, item in xy['methods'].items():
            a = np.asarray(item['actual_poses_world'])
            ax.plot(a[:, 0], a[:, 1], STYLES[name], color=COLORS[name], lw=2, label=SHORT[name]+' execution')
            arrows(ax, a[::30], COLORS[name], .07)
        ax.set_title('Common world context' if key == 'axes_world_m' else 'Common detail scale')
    axes[0].legend(fontsize=7)
    finish('actual_trajectory_overlay', fig, 'Three newly computed independent executions')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    for name, record in numeric['actual_yaw_and_goal_error']['methods'].items():
        axes[0].plot(record['times_s'], np.rad2deg(record['actual_visual_unwrapped_yaw_rad']), STYLES[name], color=COLORS[name], label=SHORT[name])
        for segment in record['goal_error_display_segments']:
            axes[1].plot(segment['times_s'], np.rad2deg(segment['angles_rad']), STYLES[name], color=COLORS[name])
        axes[2].plot(record['times_s'], record['goal_position_error_m'], STYLES[name], color=COLORS[name])
    yaw_limit = float(np.rad2deg(config['formulation']['goal_yaw_tolerance']))
    for sign in (-1, 1): axes[1].axhline(sign*yaw_limit, color='#555555', ls='--', lw=.8)
    axes[2].axhline(config['formulation']['goal_position_tolerance'], color='#555555', ls='--', lw=.8)
    for ax, label in zip(axes, ('actual continuous yaw [deg]', 'wrapped error to original goal [deg]', 'position error to original goal [m]')):
        decorate_time(ax, label); ax.axvspan(3-config['evaluation']['terminal_goal_dwell_s'], 3, color='#aaaaaa', alpha=.15)
    axes[0].legend(ncol=3, fontsize=8)
    finish('actual_yaw_and_goal_error', fig, 'Actual yaw, goal errors and original terminal dwell', ['Wrapped-error representation cuts are disconnected; no fabricated turns.', 'Grey region is the required dwell window, not achieved dwell duration.'])

    for component, name, label in ((0,'linear_command_vs_time','applied linear velocity [m/s]'), (1,'angular_command_vs_time','applied angular velocity [rad/s]')):
        fig, ax = plt.subplots(figsize=(12,5))
        for method, record in numeric[name].items():
            ax.step(record['times_s'], record['applied_commands'], where='post', ls=STYLES[method], color=COLORS[method], label=SHORT[method])
        ax.scatter([0], [context['u_minus'][component]], color='black', marker='x', s=70, label='physical incoming command')
        ax.scatter([0], [context['previous_control'][component]], facecolors='none', edgecolors='black', s=70, label='controller memory')
        decorate_time(ax, label); ax.legend(ncol=3, fontsize=8)
        finish(name, fig, 'Actual applied MPC command with original 0.1 s holds', ['The final held command is shown through 3.0 seconds; no additional solve at 3.0 seconds.'])

    fig, ax = plt.subplots(figsize=(12,5))
    for name, record in numeric['clearance_vs_time']['methods'].items():
        ax.plot(record['times_s'], record['footprint_clearance_m'], STYLES[name], color=COLORS[name], label=SHORT[name])
    ax.axhline(config['footprint']['required_clearance_m'], color='red', ls='--', label='required edge clearance')
    ax.axhline(0, color='#555555', lw=.8); decorate_time(ax, 'actual footprint edge clearance [m]'); ax.legend(ncol=4, fontsize=8)
    finish('clearance_vs_time', fig, 'Clearance under unchanged environment and footprint')

    rows = numeric['outcome_summary']['rows']; fig, axes = plt.subplots(1,3,figsize=(17,5), gridspec_kw={'width_ratios':[1,1,2]})
    for ax,key,threshold,label in ((axes[0],'yaw_error_abs_deg',yaw_limit,'terminal |yaw error| [deg]'),
                                 (axes[1],'position_error_m',config['formulation']['goal_position_tolerance'],'terminal position error [m]')):
        ax.bar(np.arange(3), [r[key] for r in rows], color=[COLORS[n] for n in METHODS]); ax.axhline(threshold,color='red',ls='--')
        ax.set(xticks=np.arange(3),xticklabels=['A','B','C'],ylabel=label); ax.grid(axis='y',alpha=.2)
    axes[2].axis('off')
    cells = [[r['method'][0], 'PASS' if r['success'] else 'FAIL', 'PASS' if r['terminal_goal_dwell_pass'] else 'FAIL',
              'N/A' if r['time_to_goal_s'] is None else f"{r['time_to_goal_s']:.3f}", 'PASS' if r['motion_valid'] else 'FAIL'] for r in rows]
    table = axes[2].table(cellText=cells,colLabels=['Method','Success','Dwell','Goal time [s]','Motion'],loc='center',cellLoc='center')
    table.auto_set_font_size(False);table.set_fontsize(9);table.scale(1.,2.)
    finish('outcome_summary',fig,'Original success predicates; failures and absent goal times retained')
    write(output/'plot_provenance.json',dict(case_id=row['case_id'],images=images,input_identity=identity,
        axes_world_m=xy['axes_world_m'],detail_axes_world_m=xy['detail_axes_world_m'],input_axes_world_m=xy['input_axes_world_m'],gui_runtime_validated=False))
    return images


def main(run):
    run = Path(run).resolve()
    if (run/'plot_manifest.json').exists() or (run/'index.html').exists():
        raise FileExistsError('refusing plot/index overwrite')
    rows = read(run/'case_manifest.json')['selected']; config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    env = HospitalEnvironment.load(environment_path(run)); images = []
    for row in rows: images.extend(plot_case(run,row,env,config))
    for item in images:item['path']=str(Path(item['path']).relative_to(run))
    write(run/'plot_manifest.json',dict(experiment='GP-SE2-REF-02',label=LABEL,methods=list(METHODS),images=images,
        case_count=len(rows),image_count=len(images),gui_runtime_validated=False,plotter_sha256=file_sha256(__file__)))
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>REF-02 lookahead isolation</title>',
        '<style>body{font:16px system-ui;max-width:1600px;margin:25px auto;padding:15px}img{max-width:100%}article{margin:35px 0}</style></head><body>',
        '<h1>'+LABEL+'</h1><p>Two fixed events × three selectors. B and C use byte-identical dense input arrays. Only C target selection differs. Saved numerical results, no GUI runtime claim.</p>',
        '<p><a href="aggregate/summary.json">Diagnosis</a> · <a href="aggregate/outcomes.csv">All outcomes</a> · <a href="aggregate/paired_comparison.csv">Paired comparison</a> · <a href="aggregate/selection_mechanism.csv">Selection mechanism</a> · <a href="aggregate/timing.csv">Timing</a> · <a href="source.json">Source hashes</a></p>']
    for row in rows:
        lines.append('<h2>'+html.escape(row['case_id'])+'</h2>')
        for name in PLOT_NAMES:
            path=f'cases/{row["case_directory"]}/plots/{name}'
            lines.append(f'<article><h3>{html.escape(name.replace("_"," "))}</h3><a href="{path}.png"><img loading="lazy" src="{path}.png" alt="{name}"></a><p><a href="{path}.json">Exact plotted numbers and source/config hashes</a></p></article>')
    (run/'index.html').write_text('\n'.join(lines+['</body></html>']))
    return dict(case_count=len(rows),image_count=len(images),gui_runtime_validated=False)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(main(parser.parse_args().run),indent=2))
