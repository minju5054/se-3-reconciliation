#!/usr/bin/env python3
"""Saved GP-SE2-02 comparison figures; no optimizer, inference, or MPC execution.

Every planned method and GP start remains visible, including missing candidates.
Adjacent sidecars contain exactly the plotted arrays and their source hashes.
"""
from __future__ import annotations
import argparse
import csv
import html
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import box
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import goal_trace
from reconciliation.gp_se2_diagnostics import file_sha256
from plot_gp_se2_01 import geometry
from plot_gp_se2_diag_01 import plain
from plot_gp_se2_diag_02 import best_full_history, sample_support

LABEL = 'OFFLINE COUNTERFACTUAL HARD-HANDOFF COMPARISON'
METHODS = ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID', 'M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED', 'SEED_ONLY')
GP_METHODS = METHODS[3:5]
SEEDS = ('I0_FRESH', 'I1_DECEL')
PLOT_NAMES = ('candidate_world_overlay', 'actual_rollout_overlay', 'boundary_zoom',
              'clearance_vs_time', 'linear_command_vs_time', 'angular_command_vs_time',
              'goal_error_vs_time', 'candidate_feasibility_summary', 'feasible_objective_history',
              'lateral_velocity', 'planned_acceleration', 'solver_cost')
COLORS = dict(zip(METHODS, ('#777777', '#0072b2', '#cc79a7', '#e69f00', '#009e73', '#7e57c2')))
SHORT = {'M0_NATIVE': 'Native', 'M0_ADAPTER': 'Adapter', 'M1_RIGID': 'Rigid',
         'M2_GP_NO_OBSTACLE': 'GP M2', 'M3_GP_CONSTRAINED': 'GP M3', 'SEED_ONLY': 'Seed only'}


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def environment_path(run):
    local = Path(run) / 'environment'
    if local.is_dir():
        return local.resolve()
    source = read(Path(run) / 'source.json')
    value = source.get('environment_path', source.get('environment'))
    if isinstance(value, dict):
        value = value.get('path')
    if not isinstance(value, str):
        raise ValueError('source must identify original environment_path')
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def past_execution(case):
    """Recorded last three seconds through exact B, never counterfactual future."""
    case = Path(case)
    target = case / 'actual_past_execution.json'
    if target.exists():
        return read(target)
    context = read(case / 'input_context.json')
    relative = f"episodes/{context['episode_id']}/execution.csv"
    source = Path(context['source_root']) / relative
    entry = next(r for r in context['source_files'] if r['path'] == relative)
    if file_sha256(source) != entry['sha256']:
        raise ValueError('historical execution hash changed')
    with source.open() as stream:
        rows = list(csv.DictReader(stream))
    boundary = float(context['switch_sim_time_s'])
    rows = [r for r in rows if boundary - 3. <= float(r['sim_time_s']) <= boundary
            and int(r['state_id']) <= int(context['switch_state_id'])]
    poses = np.asarray([[float(r[k]) for k in ('x', 'y', 'yaw')] for r in rows])
    if not len(rows) or int(rows[-1]['state_id']) != int(context['switch_state_id']):
        raise ValueError('historical B row missing')
    if not np.array_equal(poses[-1], context['B_world']):
        raise ValueError('historical boundary does not equal preserved B')
    payload = dict(source_path=str(source), source_sha256=file_sha256(source),
                   context_sha256=file_sha256(case / 'input_context.json'),
                   selection='recorded simulation times in [B time - 3 seconds, B time], through original switch_state_id',
                   frame='unchanged Isaac Hospital world XY metres, yaw radians CCW',
                   times_relative_to_B_s=[float(r['sim_time_s']) - boundary for r in rows],
                   state_ids=[int(r['state_id']) for r in rows], poses_world=poses,
                   historical=True, newly_executed=False, future_samples_used=False)
    write(target, payload)
    return plain(payload)


def load_methods(case):
    output = {}
    for method in METHODS:
        folder = Path(case) / 'methods' / method
        item = dict(folder=folder)
        for name in ('solver_result', 'plan_validation', 'metrics'):
            item[name] = read(folder / f'{name}.json')
        candidate = folder / 'candidate_world.npy'
        item['candidate'] = np.load(candidate, allow_pickle=False) if candidate.is_file() else None
        rollout = folder / 'rollout/rollout.json'
        item['rollout'] = read(rollout) if rollout.is_file() else None
        item['starts'] = {}
        if method in GP_METHODS:
            for seed in SEEDS:
                p = folder / 'starts' / seed
                start = {name: read(p / f'{name}.json') for name in
                         ('solver_result', 'result_summary', 'full_acceptance', 'post_full_checks', 'setup')}
                start['folder'] = p
                start['seed_metadata'] = read(Path(case) / 'initializations' / f'{seed}.json')
                item['starts'][seed] = start
        output[method] = item
    return output


def availability(item):
    m = item['metrics']
    return dict(candidate_available=bool(m.get('candidate_available', item['candidate'] is not None)),
                plan_valid=bool(m.get('plan_valid', item['plan_validation'].get('plan_valid', False))),
                rollout_performed=item['rollout'] is not None,
                rollout_success=bool(m.get('rollout_success', m.get('primary_success', False))),
                candidate_status=item['solver_result'].get('status', item['solver_result'].get('termination')),
                termination_reasons=m.get('termination_reasons', m.get('status_reasons', m.get('failure_reasons', []))))


def xy_numeric(context, route, methods, past):
    result = dict(old_world=context['old_world'], fresh_world=context['fresh_world'],
                  B_world=context['B_world'], goal_world=route['goal_world'],
                  actual_past_world=past['poses_world'], actual_past_times_relative_to_B_s=past['times_relative_to_B_s'], methods={})
    for name, item in methods.items():
        s = item['solver_result']
        result['methods'][name] = dict(status=availability(item),
            candidate_world=plain(item['candidate']),
            actual_rollout_world=item['metrics'].get('dense_poses_world'),
            rejected_final_world=s.get('diagnostic_candidate_world'),
            candidate_kind='fixed deceleration seed' if name == 'SEED_ONLY' else
                'original spatial reference' if name.startswith('M0') else 'optimized reference')
    return result


def common_limits(numeric):
    parts = [np.asarray(numeric[k])[:, :2] for k in ('old_world', 'fresh_world', 'actual_past_world')]
    parts.append(np.asarray([numeric['B_world'], numeric['goal_world']])[:, :2])
    for item in numeric['methods'].values():
        parts.extend(np.asarray(item[k])[:, :2] for k in ('candidate_world', 'actual_rollout_world', 'rejected_final_world')
                     if item[k] is not None)
    xy = np.vstack(parts)
    center = (xy.min(0) + xy.max(0)) / 2
    span = max(float(np.ptp(xy, axis=0).max()) + .8, 2.)
    return [center[0] - span / 2, center[0] + span / 2, center[1] - span / 2, center[1] + span / 2]


def base_xy(ax, data, environment, bounds, config):
    clip = box(bounds[0], bounds[2], bounds[1], bounds[3])
    ax.set_facecolor('#eee9e2')
    geometry(ax, environment.workspace.intersection(clip), facecolor='#f5faf2', edgecolor='#899b7a')
    geometry(ax, environment.obstacles.intersection(clip), facecolor='#a3a3a3', edgecolor='#555555')
    for key, color, style, label in (
        ('old_world', '#2269b1', '--', 'original OLD prediction'),
        ('fresh_world', '#b32183', '--', 'original FRESH prediction'),
        ('actual_past_world', '#262626', '-', 'recorded past through B')):
        points = np.asarray(data[key])
        ax.plot(points[:, 0], points[:, 1], style, color=color, lw=1.2, alpha=.7, label=label)
    boundary, goal = np.asarray(data['B_world']), np.asarray(data['goal_world'])
    radius = config['footprint']['radius_m']
    for r, style in ((radius, '-'), (radius + config['footprint']['required_clearance_m'], ':')):
        ax.add_patch(Circle(boundary[:2], r, fill=False, color='black', ls=style, lw=.9))
    ax.scatter(*boundary[:2], color='black', s=30, zorder=8, label='B; footprint / margin')
    ax.arrow(*boundary[:2], .25*np.cos(boundary[2]), .25*np.sin(boundary[2]), width=.006,
             head_width=.05, length_includes_head=True, color='black', zorder=9)
    ax.add_patch(Circle(goal[:2], config['formulation']['goal_position_tolerance'], color='#239447', alpha=.10))
    ax.scatter(*goal[:2], color='#239447', marker='*', s=100, zorder=9, label='original common goal')
    ax.arrow(*goal[:2], .2*np.cos(goal[2]), .2*np.sin(goal[2]), width=.005,
             head_width=.04, length_includes_head=True, color='#239447', zorder=9)
    ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='world x [m]', ylabel='world y [m]', aspect='equal')
    ax.grid(alpha=.15)


def save(fig, target, numeric, sources, title, notes=()):
    target = Path(target)
    if target.exists() or target.with_suffix('.json').exists():
        raise FileExistsError(target)
    fig.suptitle(title, fontsize=12)
    fig.text(.015, .008, 'OFFLINE counterfactual | static Hospital | adjacent JSON: exact plotted values and source hashes', fontsize=8, color='#555555')
    fig.tight_layout(rect=(0, .04, 1, .94))
    fig.savefig(target, dpi=160)
    plt.close(fig)
    payload = dict(image=target.name, image_sha256=file_sha256(target), label=LABEL,
        numeric_data=plain(numeric), source_hashes={str(Path(p).resolve()): file_sha256(p) for p in sources},
        plotter_sha256=file_sha256(__file__), dpi=160, missing_values='null; never filled with zero',
        notes=list(notes), new_inference=False, new_solver=False, new_execution=False)
    write(target.with_suffix('.json'), payload)
    return dict(path=str(target), sha256=file_sha256(target), sidecar_sha256=file_sha256(target.with_suffix('.json')))


def legend(ax, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.), fontsize=8, frameon=False, **kwargs)


def command_numeric(item, context, component):
    roll = item['rollout']
    if roll is None:
        return None
    selections = roll.get('controller_reference_selections', [])
    if not selections:
        return None
    return dict(times_s=[float(row['time_s']) for row in selections] + [3.],
                commands=[float(row['command'][component]) for row in selections] + [float(selections[-1]['command'][component])],
                initial_physical_command=float(context['u_minus'][component]),
                controller_memory=float(context['previous_control'][component]),
                meaning='newly computed official MPC command held on [t_k,t_k+0.1); GP twist is not supplied as feed-forward')


def plot_case(run, row, environment, config):
    case = run / 'cases' / row['case_directory']
    output = case / 'plots'
    output.mkdir(exist_ok=False)
    context, route = read(case / 'input_context.json'), read(case / 'goal_route.json')
    past, methods = past_execution(case), load_methods(case)
    xy = xy_numeric(context, route, methods, past)
    bounds = common_limits(xy)
    sources = [run / p for p in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'case_manifest.json')]
    sources += [case / p for p in ('input_context.json', 'goal_route.json', 'actual_past_execution.json')]
    for item in methods.values():
        sources += [item['folder'] / f'{name}.json' for name in ('solver_result', 'plan_validation', 'metrics')]
        for name in ('candidate_world.npy', 'rollout/rollout.json'):
            if (item['folder'] / name).is_file(): sources.append(item['folder'] / name)
        for start in item['starts'].values():
            sources += [start['folder'] / f'{name}.json' for name in ('solver_result', 'result_summary', 'full_acceptance', 'post_full_checks', 'setup')]
            sources.append(case / 'initializations' / f"{start['solver_result']['initialization']}.json")
    images = []
    title = f"{row['case_id']} | {row.get('case_role', row.get('role', row.get('selected_group', 'fixed case')))}"
    for name in PLOT_NAMES[:3]:
        fig, ax = plt.subplots(figsize=(12, 7))
        used_bounds = bounds
        if name == 'boundary_zoom':
            b = np.asarray(context['B_world']); used_bounds = [b[0]-.75, b[0]+.75, b[1]-.75, b[1]+.75]
        base_xy(ax, xy, environment, used_bounds, config)
        missing = []
        for method, item in xy['methods'].items():
            key = 'actual_rollout_world' if name in ('actual_rollout_overlay', 'boundary_zoom') else 'candidate_world'
            points = item[key]
            if points is None:
                missing.append(SHORT[method])
                if key == 'candidate_world' and item['rejected_final_world'] is not None:
                    rejected=np.asarray(item['rejected_final_world'])
                    ax.plot(rejected[:,0],rejected[:,1],color=COLORS[method],ls=':',lw=1.,alpha=.7,
                            label=SHORT[method]+' REJECTED final; no candidate')
                continue
            points = np.asarray(points)
            status = item['status']
            label = SHORT[method] + (' execution' if key == 'actual_rollout_world' else ' reference')
            if key == 'candidate_world' and not status['plan_valid']: label += ' [PLAN INVALID]'
            if key == 'actual_rollout_world' and not status['rollout_success']: label += ' [FAILED]'
            ax.plot(points[:, 0], points[:, 1], color=COLORS[method], ls=':' if method == 'SEED_ONLY' else '-', lw=1.8, label=label)
        if missing: ax.text(.02, .02, 'Unavailable: ' + ', '.join(missing), fontsize=8, transform=ax.transAxes)
        legend(ax)
        images.append(save(fig, output / f'{name}.png', dict(**xy, axes_world_m=used_bounds), sources,
                           title + '\n' + name.replace('_', ' '), ['Same metric bounds for all methods within this panel; zoom is fixed ±0.75 m about B.']))
    fig, ax = plt.subplots(figsize=(12, 6)); numeric = {}
    for method, item in methods.items():
        metric = item['metrics']; environment_trace = metric.get('environment')
        trace = None if environment_trace is None else dict(times_s=metric['dense_times_s'], clearance_m=environment_trace['clearance_samples_m'])
        numeric[method] = trace
        if trace: ax.plot(trace['times_s'], trace['clearance_m'], color=COLORS[method], label=SHORT[method])
    ax.axhline(.05, color='red', ls='--', label='required clearance'); ax.axhline(0, color='black', ls=':')
    ax.set(xlabel='counterfactual time [s]', ylabel='actual footprint-edge clearance [m]', xlim=(0, 3));ax.grid(alpha=.2);legend(ax)
    images.append(save(fig, output / 'clearance_vs_time.png', numeric, sources, title + '\nactual rollout clearance'))
    for component, name, label, limit in ((0, 'linear_command_vs_time', 'actual linear command [m/s]', (0, .8)),
                                         (1, 'angular_command_vs_time', 'actual angular command [rad/s]', (-3, 3))):
        fig, ax = plt.subplots(figsize=(12, 6)); numeric = {}
        for method, item in methods.items():
            trace = command_numeric(item, context, component); numeric[method] = trace
            if trace: ax.step(trace['times_s'], trace['commands'], where='post', color=COLORS[method], label=SHORT[method])
        ax.scatter([0], [context['u_minus'][component]], marker='o', color='black', label='physical u_minus', zorder=8)
        ax.scatter([0], [context['previous_control'][component]], marker='x', color='red', label='controller previous_control', zorder=8)
        for bound in limit: ax.axhline(bound, color='red', ls='--', lw=.8)
        ax.set(xlabel='counterfactual time [s]', ylabel=label, xlim=(0, 3)); ax.grid(alpha=.2); legend(ax)
        images.append(save(fig, output / f'{name}.png', numeric, sources, title + '\nnew official MPC held commands', ['Missing rollout has no line; planned body velocities are never plotted as applied commands.']))
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True); numeric = {}
    for method, item in methods.items():
        metric = item['metrics']; points = metric.get('dense_poses_world')
        trace = None
        if points:
            dist, yaw, _ = goal_trace(points, route['goal_world'])
            trace = dict(times_s=metric['dense_times_s'], position_error_m=dist, yaw_error_deg=np.degrees(yaw))
            axes[0].plot(trace['times_s'], dist, color=COLORS[method], label=SHORT[method]); axes[1].plot(trace['times_s'], np.degrees(yaw), color=COLORS[method])
        numeric[method] = plain(trace)
    for ax, tol, label in zip(axes, (.15, 15), ('actual goal position error [m]', 'actual goal yaw error [deg]')):
        ax.axhline(tol, color='red', ls='--'); ax.axvspan(2.8, 3., color='#009e73', alpha=.08)
        ax.set_ylabel(label); ax.grid(alpha=.2)
    axes[-1].set(xlabel='counterfactual time [s]; shade = original terminal 0.20 s dwell', xlim=(0, 3)); legend(axes[0])
    images.append(save(fig, output / 'goal_error_vs_time.png', numeric, sources, title + '\noriginal common goal; rollout and dwell'))
    fig, ax = plt.subplots(figsize=(13, 6)); ax.axis('off')
    numeric = {method: availability(item) for method, item in methods.items()}
    rows = [[SHORT[method], str(s['candidate_available']), str(s['plan_valid']), str(s['rollout_performed']), str(s['rollout_success']), '\n'.join(s['termination_reasons']) or 'none'] for method, s in numeric.items()]
    table = ax.table(cellText=rows, colLabels=['method', 'candidate', 'plan valid', 'rollout', 'local success', 'recorded reasons'], cellLoc='left', loc='center', colWidths=[.13,.10,.10,.10,.11,.46]); table.auto_set_font_size(False); table.set_fontsize(8); table.scale(1, 3.)
    images.append(save(fig, output / 'candidate_feasibility_summary.png', numeric, sources, title + '\nCandidate / plan / actual rollout are separate outcomes'))
    images.extend(plot_gp(output, methods, config, sources, title))
    write(output / 'plot_provenance.json', dict(case_id=row['case_id'], axes_world_m=bounds, methods=METHODS,
        images=[dict(path=Path(r['path']).name, sha256=r['sha256'], sidecar_sha256=r['sidecar_sha256']) for r in images],
        source_sha256=file_sha256(run / 'source.json'), config_sha256=file_sha256(run / 'config_snapshot.yaml')))
    return images, numeric


def plot_gp(output, methods, config, sources, title):
    images = []
    fig, axes = plt.subplots(2, 2, figsize=(13, 9)); numeric = {}
    for i, method in enumerate(GP_METHODS):
        for j, seed in enumerate(SEEDS):
            start = methods[method]['starts'][seed]; ax = axes[i,j]
            history = best_full_history(start); numeric[f'{method}/{seed}'] = history
            points = history['points']; t = [p['discovery_time_s'] for p in points]; y = [p['best_full_objective'] if p['best_full_objective'] is not None else np.nan for p in points]
            if np.isfinite(y).any(): ax.step(t, y, where='post', color=COLORS[method], label='post-certified full-feasible best')
            else: ax.text(.5, .5, 'N/A: no full-feasible candidate', ha='center', transform=ax.transAxes)
            if history['seed_full_feasible']: ax.axhline(history['seed_objective'], color='gray', ls=':', label='feasible seed')
            result = start['solver_result']; ax.set_title(f"{SHORT[method]} / {seed}: {result.get('termination', 'UNSUPPORTED')}", fontsize=9)
            ax.set(xlabel='discovery solve wall time [s]', ylabel='original GP objective'); ax.set_ylim(bottom=0); ax.grid(alpha=.2)
            if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=7)
    images.append(save(fig, output/'feasible_objective_history.png', numeric, sources, title+'\nFull feasibility is certified after solving; blank intervals are N/A'))
    traces = {}
    for method in GP_METHODS:
        for seed, start in methods[method]['starts'].items():
            r = start['solver_result']
            traces[f'{method}/{seed}'] = dict(
                seed=sample_support(start['seed_metadata']['chart_reconstructed_poses'], start['seed_metadata']['chart_reconstructed_twists'], config['formulation']),
                final_full_feasible=start['result_summary'].get('final_full_feasible'),
                seed_full_feasible=start['result_summary'].get('initial_full_feasible'),
                final=sample_support(r.get('latest_support_poses'),r.get('latest_support_twists'), config['formulation']),
                selected=sample_support(r.get('support_poses'),r.get('support_twists'), config['formulation']),
                selected_full_feasible=bool(start['full_acceptance'].get('full_feasible')))
    for name, acceleration in (('lateral_velocity', False), ('planned_acceleration', True)):
        fig, axes = plt.subplots(2,2, figsize=(13,9))
        for i, method in enumerate(GP_METHODS):
            for j, seed in enumerate(SEEDS):
                ax=axes[i,j]; record=traces[f'{method}/{seed}']
                for kind, style in (('seed',':'),('final','--'),('selected','-')):
                    trace=record[kind]
                    if trace is None or (kind=='selected' and not record['selected_full_feasible']):continue
                    t=trace['times_s']; values=np.asarray(trace['body_accelerations'] if acceleration else trace['body_twists'])
                    label=('selected full-accepted' if kind=='selected' else
                           ('initial seed [FULL VALID]' if record['seed_full_feasible'] else 'initial seed [INVALID]') if kind=='seed' else
                           ('final [FULL VALID]' if record['final_full_feasible'] else 'final [REJECTED / UNVERIFIED]'))
                    if acceleration:
                        ax.plot(t,values[:,0],style,c=COLORS[method],label=label+' linear [m/s²]')
                        ax.plot(t,values[:,2],style,c='#b32183',label=label+' angular [rad/s²]')
                    else:ax.plot(t,values[:,1],style,c=COLORS[method],label=label)
                for bound in ((-5,-2,2,5) if acceleration else (-1e-5,1e-5)):ax.axhline(bound,c='gray',ls=':',lw=.6)
                ax.set(title=f'{SHORT[method]} / {seed}',xlabel='GP trajectory time [s]',ylabel='body acceleration; units in legend' if acceleration else 'planned lateral body velocity [m/s]')
                ax.grid(alpha=.2)
                if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=6)
                else:ax.text(.5,.5,'No supported trajectory',ha='center',transform=ax.transAxes)
        images.append(save(fig,output/f'{name}.png',traces,sources,title+'\nGP plan states; not actual MPC commands'))
    fig, axes=plt.subplots(2,1,figsize=(13,9)); numeric={}; labels=[]; costs=[]; counts=[]
    for method in GP_METHODS:
        for seed,start in methods[method]['starts'].items():
            r,s=start['solver_result'],start['setup']; key=f'{method}/{seed}'; labels.append(SHORT[method]+'\n'+seed)
            parts=dict(case_seed_loading=s.get('case_loading_seed_preparation_s',0.),
                solver_setup=r.get('setup_wall_time_s',0.),graph=s.get('derivative_graph_construction_s',0.),warmup=s.get('compilation_first_call_warmup_s',0.),
                solve=r.get('solve_wall_time_s',0.),original_post=r.get('post_solve_validation_time_s',0.),
                full_validation=start['post_full_checks'].get('full_validation_wall_time_s',0.))
            count=dict(objective=r.get('objective_evaluations'),primal_misses=r.get('profiling',{}).get('evaluator_cache_misses'),
                objective_jac=r.get('derivative_calls',{}).get('objective_gradient'))
            numeric[key]=dict(disjoint_seconds=parts,counts=count,setup=s,termination=r.get('termination'))
            costs.append(parts);counts.append(count)
    bottom=np.zeros(4)
    for key in costs[0]:
        vals=np.array([float(p[key] or 0.) for p in costs]);axes[0].bar(labels,vals,bottom=bottom,label=key);bottom+=vals
    axes[0].set_ylabel('measured wall time [s]');legend(axes[0]);axes[0].grid(axis='y',alpha=.2)
    for k,key in enumerate(counts[0]):
        vals=[np.nan if p[key] is None else p[key] for p in counts];axes[1].bar(np.arange(4)+(k-1)*.24,vals,width=.24,label=key)
    axes[1].set_xticks(np.arange(4),labels);axes[1].set_ylabel('callback / actual evaluator counts');axes[1].set_yscale('symlog',linthresh=1);legend(axes[1]);axes[1].grid(axis='y',alpha=.2)
    images.append(save(fig,output/'solver_cost.png',numeric,sources,title+'\nBoth planned starts included; disjoint setup/solve/check costs', ['Missing counts remain null; no unsupported start is reported as a zero-cost successful solve.']))
    return images


def main(run):
    run=Path(run).resolve()
    if (run/'index.html').exists():raise FileExistsError(run/'index.html')
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());manifest=read(run/'case_manifest.json')
    environment=HospitalEnvironment.load(environment_path(run));images=[]
    # Validate all expected source records before opening any figure output.
    for row in manifest['selected']:
        case=run/'cases'/row['case_directory']; load_methods(case); past_execution(case)
        if (case/'plots').exists():raise FileExistsError(case/'plots')
    sections=['<!doctype html><html><head><meta charset="utf-8"><title>GP-SE2-02</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:20px}img{width:49%;vertical-align:top}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}a{color:#145a90}</style></head><body>',f'<h1>{LABEL}</h1>',
      '<p>Four fixed cases from the development corpus. Static Hospital oracle, original official MPC, independent 3 s offline counterfactuals. Candidate validity, numerical convergence and rollout success are separate outcomes. Missing execution is never fabricated.</p>',
      '<p><a href="case_manifest.json">Frozen cases</a> · <a href="protocol.json">Protocol</a> · <a href="source.json">Sources and hashes</a></p>']
    for filename in ('primary_outcomes.csv','regressions.csv','all_starts.csv','candidate_acceptance.csv','paired_metrics.csv','timing.csv','summary.json'):
        if (run/'aggregate'/filename).is_file(): sections.append(f'<p><a href="aggregate/{filename}">{filename}</a></p>')
    for row in manifest['selected']:
        records,statuses=plot_case(run,row,environment,config);images+=records
        sections.append(f'<h2>{html.escape(row["case_id"])} — {html.escape(row.get("case_role",row.get("role",row.get("selected_group",""))))}</h2>')
        sections.append('<table><tr><th>Method</th><th>Candidate</th><th>Plan valid</th><th>Rollout</th><th>Local success</th><th>Reasons</th></tr>')
        for method,s in statuses.items():sections.append('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in (method,s['candidate_available'],s['plan_valid'],s['rollout_performed'],s['rollout_success'],s['termination_reasons']))+'</tr>')
        sections.append('</table>')
        for image in records:
            relative=str(Path(image['path']).relative_to(run));sections.append(f'<a href="{relative}"><img loading="lazy" src="{relative}" alt="{Path(relative).stem}"></a>')
    sections.append('</body></html>')
    with (run/'index.html').open('x') as stream:stream.write('\n'.join(sections))
    for record in images:record['path']=str(Path(record['path']).relative_to(run))
    write(run/'plot_manifest.json',dict(case_count=len(manifest['selected']),methods=METHODS,required_images_per_case=PLOT_NAMES,
        images=images,index_sha256=file_sha256(run/'index.html'),source_sha256=file_sha256(run/'source.json'),config_sha256=file_sha256(run/'config_snapshot.yaml')))
    return dict(case_count=len(manifest['selected']),image_count=len(images))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(main(parser.parse_args().run),indent=2))
