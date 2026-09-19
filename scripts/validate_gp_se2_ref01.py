#!/usr/bin/env python3
"""Independently audit saved REF-01 evidence; never rerun MPC or optimization.

Scientific failure is a valid outcome. Validation rejects inconsistent evidence,
not a failed goal/dwell predicate or an unreproduced historical result.
"""
from __future__ import annotations
import argparse
import ast
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
from reconciliation.gp_se2_ref01_reference import VARIANTS, build_variants, enrich_selection
from reconciliation.gp_se2_ref01_evaluation import (
    FIXED_CASES, CONTRAST_METRICS, evaluate_variant, reproduction_check, selection_diagnostics,
)
from reconciliation.gp_se2_02_evaluation import validate_counterfactual
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_rollout import load_frozen_context
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.se2 import wrap_angle

PLOTS = ('input_rows_world', 'input_yaw_vs_original_progress', 'matched_state_reference_targets',
         'selected_original_progress_vs_time', 'selected_reference_yaw_vs_time', 'actual_trajectory_overlay',
         'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
         'clearance_vs_time', 'outcome_summary')


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


class Audit:
    def __init__(self):
        self.checks = 0
        self.errors = []

    def check(self, condition, message):
        self.checks += 1
        if not condition:
            self.errors.append(message)
        return bool(condition)

    def same(self, actual, expected, label, atol=1e-12):
        if isinstance(expected, np.ndarray):
            expected = expected.tolist()
        if isinstance(actual, np.ndarray):
            actual = actual.tolist()
        if isinstance(expected, np.generic):
            expected = expected.item()
        if isinstance(actual, np.generic):
            actual = actual.item()
        if isinstance(expected, dict):
            if not self.check(isinstance(actual, dict), label+': expected mapping'):
                return
            self.check(set(actual) == set(expected), label+': mapping keys')
            for key in set(actual) & set(expected):
                self.same(actual[key], expected[key], label+'/'+str(key), atol)
        elif isinstance(expected, (list, tuple)):
            if not self.check(isinstance(actual, (list, tuple)), label+': expected sequence'):
                return
            self.check(len(actual) == len(expected), label+': sequence length')
            for i, (a, b) in enumerate(zip(actual, expected)):
                self.same(a, b, label+'/'+str(i), atol)
        elif isinstance(expected, (float, int)) and not isinstance(expected, bool):
            self.check(isinstance(actual, (float, int)) and not isinstance(actual, bool)
                       and math.isfinite(actual) and math.isfinite(expected)
                       and abs(actual-expected) <= atol, label+': numerical value')
        else:
            self.check(type(actual) is type(expected) and actual == expected, label+': value/type')

    def capture(self, label, function):
        try:
            return function()
        except Exception as exc:
            self.check(False, f'{label}: {type(exc).__name__}: {exc}')
            return None


def official_selector(source_path):
    """Execute only three unchanged pure functions from hash-pinned MPC source.

    AST extraction deliberately excludes CasADi, tracker construction and solves.
    This validates the saved selector output using the actual official function.
    """
    path = Path(source_path)
    if digest(path) != PINNED_MPC_SHA256:
        raise ValueError('official MPC source hash mismatch')
    tree = ast.parse(path.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('wrap_angle', 'build_pose_aligned_reference', 'project_body_to_world')]
    if len(functions) != 3:
        raise ValueError('official pure selector/projection function coverage mismatch')
    namespace = dict(np=np, math=math, Sequence=list)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def audit_lineage(audit, native, rows, lineage, label):
    """Independent affine reconstruction, not just reusing the generating helper."""
    audit.check(len(rows) == len(lineage), label+'/lineage length')
    unwrapped = np.r_[native[0, 2], native[0, 2]+np.cumsum(wrap_angle(np.diff(native[:, 2])))]
    for i, (point, row) in enumerate(zip(rows, lineage)):
        left, right, alpha = row['original_left_row_index'], row['original_right_row_index'], row['interpolation_alpha']
        audit.check(row['derived_row_index'] == i, label+f'/row {i} identity')
        if not audit.check(0 <= left <= right < len(native) and right-left <= 1 and 0 <= alpha <= 1,
                           label+f'/row {i} source interval'):
            continue
        audit.same(row['original_fractional_row_coordinate'], left+alpha*(right-left), label+f'/row {i} progress')
        expected_xy = (1-alpha)*native[left, :2] + alpha*native[right, :2]
        expected_yaw = (1-alpha)*unwrapped[left] + alpha*unwrapped[right]
        audit.check(np.max(np.abs(point[:2]-expected_xy)) <= 1e-12, label+f'/row {i} affine XY')
        audit.check(abs(float(wrap_angle(point[2]-expected_yaw))) <= 1e-12, label+f'/row {i} yaw')
        audit.same(row['unwrapped_yaw'], float(expected_yaw), label+f'/row {i} original yaw branch')
        audit.same(row['original_row'], left == right, label+f'/row {i} original identity')
        audit.same(row['interpolated'], left != right, label+f'/row {i} interpolation identity')


def audit_selection(audit, pure, path, pose, lineage, saved, weights, label):
    reference = pure['build_pose_aligned_reference'](path, np.asarray(pose), horizon=5, weights=weights)
    expected = enrich_selection(path, pose, reference, lineage, weights=weights, horizon=5,
                                original_goal_row_index=saved['original_goal_row_index'])
    audit.same(saved, expected, label)
    # Independent row-cost and physical-horizon facts, including repeated endpoint.
    err = np.asarray(path)-pose
    err[:, 2] = [math.atan2(math.sin(float(a)), math.cos(float(a))) for a in err[:, 2]]
    costs = np.sum(err*err*np.asarray(weights), axis=1)
    nearest = int(np.argmin(costs))
    ids = np.minimum(nearest+1+np.arange(5), len(path)-1)
    audit.same(saved['nearest_index'], nearest, label+'/independent argmin')
    audit.same(saved['indices'], ids.tolist(), label+'/independent next rows')
    audit.check(np.max(np.abs(reference[:, :2]-path[ids, :2])) <= 1e-12, label+'/actual targets')
    audit.same(saved['final_goal_row_in_horizon'], bool(np.any(ids == len(path)-1)), label+'/endpoint inclusion')


def audit_rollout(audit, pure, rollout, context, reference, lineage, weights, ordinal, label):
    audit.capture(label+'/original counterfactual checker', lambda: validate_counterfactual(rollout, context, reference))
    audit.same(rollout['primary_execution_ordinal'], ordinal, label+'/primary order')
    audit.same(rollout['independent_tracker_instance'], True, label+'/independent instance')
    audit.same(rollout['GP_or_rigid_optimization_performed'], False, label+'/no optimization')
    installed = pure['project_body_to_world'](rollout['candidate_capture_local'], context['original_capture_pose_world'])
    audit.same(rollout['installed_reference_world'], installed, label+'/actual installed path')
    expected_hash = hashlib.sha256(np.asarray(installed, dtype='<f8').tobytes(order='C')).hexdigest()
    audit.same(rollout['installed_path_sha256'], expected_hash, label+'/installed hash')
    solves = rollout['controller_reference_selections']
    for i, solve in enumerate(solves):
        audit.same(solve['input_pose_world'], rollout['states'][i*6]['pose_world'], label+f'/input state {i}', 0.)
        audit.same(solve['previous_control'], context['previous_control'] if i == 0 else solves[i-1]['previous_control_after'],
                   label+f'/memory {i}', 0.)
        audit.same(solve['installed_path_sha256'], expected_hash, label+f'/installed hash {i}')
        audit.same(solve['previous_control_after'], solve['command'], label+f'/official memory after {i}', 0.)
        if solve['success']:
            prediction = np.asarray(solve['prediction_world'])
            audit.check(prediction.shape == (6, 3) and np.isfinite(prediction).all(), label+f'/finite prediction {i}')
        else:
            audit.same(solve['command'], [0., 0.], label+f'/original failed solve policy {i}', 0.)
        audit_selection(audit, pure, installed, solve['input_pose_world'], lineage, solve['selection_diagnostic'], weights,
                        label+f'/actual selection {i}')
        for key in ('nearest_index', 'indices', 'reference_world', 'endpoint_repeated', 'nearest_is_final_row'):
            audit.same(solve['selection'][key], solve['selection_diagnostic'][key], label+f'/actual recorded audit {i}/{key}')
    audit.same(rollout['controller_failure_count'], sum(not row['success'] for row in solves), label+'/failure count')


def independent_contrasts(rows):
    by = {(r['case_id'], r['variant']): r for r in rows}
    if len(by) != len(rows):
        raise ValueError('duplicate outcome row')
    output = []
    for case in FIXED_CASES:
        for metric in CONTRAST_METRICS:
            values = [by[case, v].get(metric) for v in VARIANTS]
            def diff(i, j):
                return None if values[i] is None or values[j] is None else values[i]-values[j]
            interaction = None if any(v is None for v in values) else values[3]-values[1]-values[2]+values[0]
            output.append(dict(case_id=case, metric=metric, **dict(zip(VARIANTS, values)),
                suffix_without_resampling=diff(1, 0), resampling_without_suffix=diff(2, 0),
                suffix_with_resampling=diff(3, 2), resampling_with_suffix=diff(3, 1), interaction=interaction,
                interpretation='fixed-event descriptive contrast; no population significance'))
    return output



def independent_pairwise(case_id, records, *, matched):
    pairs = [('R00_NATIVE', 'R10_SUFFIX_ONLY'), ('R00_NATIVE', 'R01_RESAMPLE_ONLY'),
             ('R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER'), ('R10_SUFFIX_ONLY', 'R11_CURRENT_ADAPTER')]
    rows = []
    for base, other in pairs:
        for i in range(30):
            if matched:
                a,b = records[i]['variants'][base], records[i]['variants'][other]
                at = records[i]['time_s']; command_difference = pose_difference = None
            else:
                left,right = records[base][i], records[other][i]
                a,b = left['selection_diagnostic'],right['selection_diagnostic']; at = left['time_s']
                command_difference = (np.asarray(right['command'])-left['command']).tolist()
                pose_difference = np.asarray(right['input_pose_world'])-left['input_pose_world']
                pose_difference[2] = wrap_angle(pose_difference[2]); pose_difference = pose_difference.tolist()
            delta = np.asarray(b['reference_world'])-a['reference_world']; delta[:,2] = wrap_angle(delta[:,2])
            progress = np.asarray(b['selected_original_fractional_row_coordinates'])-a['selected_original_fractional_row_coordinates']
            rows.append(dict(case_id=case_id, baseline=base, variant=other, time_s=at, same_input_pose=matched,
                max_selected_xy_difference_m=float(max(np.linalg.norm(delta[:,:2],axis=1))),
                max_selected_yaw_difference_rad=float(max(abs(delta[:,2]))),
                selected_original_progress_difference=progress.tolist(),
                selection_changed=bool(np.max(np.abs(delta)) > 1e-12 or np.max(np.abs(progress)) > 1e-12),
                baseline_goal_in_horizon=a['final_goal_row_in_horizon'], variant_goal_in_horizon=b['final_goal_row_in_horizon'],
                command_difference=command_difference, input_pose_difference=pose_difference,
                interpretation='matched-state input effect' if matched else 'closed-loop difference includes feedback state divergence'))
    return rows


def audit_csv(audit, path, expected, label):
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    audit.check(len(rows) == len(expected), label+'/row count')
    for i, (actual, wanted) in enumerate(zip(rows, expected)):
        for key, value in wanted.items():
            if not audit.check(key in actual, label+f'/{i}/{key} column'):
                continue
            text = actual[key]
            if value is None:
                audit.same(text, '', label+f'/{i}/{key} missing preserved')
            elif isinstance(value, (dict, list)):
                audit.same(json.loads(text), value, label+f'/{i}/{key}')
            elif isinstance(value, bool):
                audit.same(text, str(value), label+f'/{i}/{key}')
            elif isinstance(value, (int, float)):
                audit.same(float(text), float(value), label+f'/{i}/{key}')
            else:
                audit.same(text, str(value), label+f'/{i}/{key}')


def audit_bundle(audit, run):
    bundle = run/'review_bundle'
    manifest = read(bundle/'manifest.json')
    allowed = {r['path']: r for r in manifest['allowlisted_files']}
    expected = {name for name in ('source.json', 'protocol.json', 'config_snapshot.yaml', 'case_manifest.json', 'plot_manifest.json', 'README.md', 'index.html', 'manifest.json')}
    expected.update('aggregate/'+name for name in ('input_audit.csv', 'rollout_outcomes.csv', 'factor_contrasts.csv', 'selection_diagnostics.csv', 'summary.json'))
    for case in read(run/'case_manifest.json')['selected']:
        base = 'cases/'+case['case_directory']+'/'
        expected.update(base+name for name in ('reference_factorization.json', 'reproduction_check.json'))
        expected.update(base+'plots/'+name+extension for name in PLOTS for extension in ('.png', '.json'))
    audit.same(sorted(set(allowed) | {'manifest.json'}), sorted(expected), 'review protocol allowlist')
    actual = {str(p.relative_to(bundle)) for p in bundle.rglob('*') if p.is_file()}
    audit.same(sorted(actual), sorted(expected), 'review bundle exact allowlist')
    for relative, record in allowed.items():
        path = Path(relative)
        audit.check(not path.is_absolute() and '..' not in path.parts, 'review path safe '+relative)
        audit.check(path.suffix in ('.png', '.json', '.csv', '.yaml', '.html', '.md'), 'review extension '+relative)
        audit.same(digest(bundle/path), record['sha256'], 'review hash '+relative)
        if record['source'] != 'generated review text':
            audit.same(digest(record['source']), record['sha256'], 'review original hash '+relative)
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        audit.same(sorted(archive.namelist()), sorted(expected), 'ZIP exact allowlist')
        audit.check(len(archive.namelist()) == len(set(archive.namelist())), 'ZIP no duplicates')
        for name in archive.namelist():
            audit.same(hashlib.sha256(archive.read(name)).hexdigest(), digest(bundle/name), 'ZIP member '+name)
    outer = read(run/'review_bundle_manifest.json')
    audit.same(outer['bundle_sha256'], digest(run/'review_bundle.zip'), 'ZIP outer hash')
    audit.same(outer['image_count'], 22, 'ZIP image coverage')
    audit.same(outer['gui_runtime_validated'], False, 'no synthetic GUI claim')


def audit_plots(audit, run, case, context, route, variants, matched, config):
    from plot_gp_se2_ref01 import geometry_numeric, selection_series, outcome_rows
    past = read(case/'actual_past_execution.json')
    audit.same(past['source_sha256'], digest(past['source_path']), 'past original source')
    audit.same(past['poses_world'][-1], context['B_world'], 'past ends at original B', 0.)
    with Path(past['source_path']).open() as stream:
        source_rows = {int(r['state_id']): r for r in csv.DictReader(stream)}
    for i, state_id in enumerate(past['state_ids']):
        r = source_rows[state_id]
        audit.same(past['poses_world'][i], [float(r[k]) for k in ('x', 'y', 'yaw')], f'past actual state {i}', 0.)
        audit.same(past['times_relative_to_B_s'][i], float(r['sim_time_s'])-context['switch_sim_time_s'], f'past actual time {i}', 0.)
    xy = geometry_numeric(context, route, variants, past)
    xy['reference_factorization'] = read(case/'reference_factorization.json')
    closed, probes = selection_series(variants), selection_series(matched['states'], matched=True)
    expected = {'input_rows_world': xy, 'actual_trajectory_overlay': xy,
                'matched_state_reference_targets': dict(world_geometry=xy, snapshots=[matched['states'][i] for i in (0, 10, 20)]),
                'selected_original_progress_vs_time': dict(matched_state=probes, closed_loop=closed),
                'selected_reference_yaw_vs_time': dict(matched_state=probes, closed_loop=closed),
                'outcome_summary': dict(rows=outcome_rows(variants))}
    expected['input_yaw_vs_original_progress'] = dict(original_native_world=context['fresh_world'], variants={})
    yaw = {}
    clearance = {}
    for name, value in variants.items():
        lineage, m = value['provenance'], value['metrics']
        expected['input_yaw_vs_original_progress']['variants'][name] = dict(
            original_fractional_row_coordinate=[r['original_fractional_row_coordinate'] for r in lineage],
            wrapped_yaw_rad=[r['wrapped_yaw'] for r in lineage],
            original_branch_unwrapped_yaw_rad=[r['unwrapped_yaw'] for r in lineage], row_provenance=lineage)
        a, t = np.asarray(m['dense_poses_world']), m['dense_times_s']
        yaw[name] = dict(times_s=t, actual_wrapped_yaw_rad=a[:, 2].tolist(),
            actual_visual_unwrapped_yaw_rad=np.unwrap(a[:, 2]).tolist(),
            goal_relative_wrapped_yaw_error_rad=wrap_angle(a[:, 2]-route['goal_world'][2]).tolist(),
            goal_position_error_m=np.linalg.norm(a[:, :2]-np.asarray(route['goal_world'])[:2], axis=1).tolist(), goal_world=route['goal_world'])
        clearance[name] = dict(times_s=t, footprint_clearance_m=m['environment']['clearance_samples_m'])
    expected['actual_yaw_and_goal_error'] = dict(variants=yaw, config_thresholds=config['formulation'], required_dwell_s=config['evaluation']['terminal_goal_dwell_s'])
    expected['clearance_vs_time'] = dict(variants=clearance, required_edge_clearance_m=config['footprint']['required_clearance_m'])
    for component, name in enumerate(('linear_command_vs_time', 'angular_command_vs_time')):
        expected[name] = {}
        for variant, value in variants.items():
            rows = value['rollout']['controller_reference_selections']
            expected[name][variant] = dict(times_s=[r['time_s'] for r in rows]+[3.],
                applied_commands=[r['command'][component] for r in rows]+[rows[-1]['command'][component]],
                initial_physical_command=context['u_minus'][component], initial_controller_memory=context['previous_control'][component])
    for name in PLOTS:
        path = case/'plots'/f'{name}.png'; side = read(path.with_suffix('.json'))
        audit.same(digest(path), side['image_sha256'], name+'/PNG hash')
        audit.same(side['numeric_data'], expected[name], name+'/plotted values')
        audit.same(side['plotter_sha256'], digest(ROOT/'scripts/plot_gp_se2_ref01.py'), name+'/plotter hash')
        for p, h in side['source_hashes'].items():
            audit.same(digest(p), h, name+'/source '+p)
        for field in ('new_inference', 'new_optimization', 'new_execution', 'gui_runtime_validated'):
            audit.same(side[field], False, name+'/'+field)
        audit.same(side['label'], 'OFFLINE REFERENCE-PREPARATION DIAGNOSTIC', name+'/label')
    return len(expected)


def validate(run, *, write_output=True, output=None, require_finalized=True):
    run = Path(run).resolve(); target = Path(output) if output is not None else (run/'validation.json' if require_finalized else run/'verification/preflight_validation.json')
    if write_output and target.exists():
        raise FileExistsError('refusing authoritative validation overwrite: '+str(target))
    begin = time.perf_counter(); audit = Audit(); cases_report = []; deferred = []
    def global_checks():
        source, protocol = read(run/'source.json'), read(run/'protocol.json')
        config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
        from run_gp_se2_ref01 import validate_config
        validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()))
        audit.same(source['original_config_sha256'], digest(run/'config_snapshot.yaml'), 'original config hash')
        audit.same(source['experiment_config_sha256'], digest(run/'experiment_config.yaml'), 'experiment config hash')
        audit.same(digest(run/'config_snapshot.yaml'), digest(Path(source['reference_source'])/'config_snapshot.yaml'), 'authoritative config unchanged')
        for record, h in source['authoritative_validations'].items():
            audit.same(digest(record), h, 'authoritative validator '+record)
            audit.check(read(record)['valid'], 'authoritative prior valid '+record)
        for mapping, prefix in (('copied_source_sha256', None), ('preserved_core_sha256', ROOT)):
            for p, h in source[mapping].items():
                audit.same(digest((prefix/Path(p)) if prefix else Path(p)), h, mapping+'/'+p)
        freeze = read(run/'experiment_freeze.json')
        for p, h in freeze['source_sha256'].items():
            audit.same(digest(ROOT/p), h, 'frozen code '+p)
        for p, h in freeze['input_sha256'].items():
            audit.same(digest(run/p), h, 'frozen input '+p)
        before = read(run/'preservation_before.json')
        if (run/'preservation_after.json').exists() or require_finalized:
            after = read(run/'preservation_after.json')
            for key in ('files', 'user_config_hashes', 'external_git_sha', 'external_git_status'):
                audit.same(before[key], after[key], 'preservation '+key)
        else:
            after = before
            deferred.append('final preservation_after.json equality')
        for p, h in after['files'].items():
            audit.same(digest(p), h, 'original current '+p)
        for p, h in after['user_config_hashes'].items():
            audit.same(digest(ROOT/p), h, 'user current '+p)
        if (run/'artifact_manifest.json').exists() or require_finalized:
            artifact = read(run/'artifact_manifest.json')['files']
            for p, h in artifact.items():
                audit.same(digest(run/p), h, 'artifact '+p)
        else:
            deferred.append('final artifact_manifest.json hashes')
        manifest = read(run/'case_manifest.json')['selected']
        audit.same([r['case_id'] for r in manifest], list(FIXED_CASES), 'fixed cases/order')
        audit.same(sorted(p.name for p in (run/'cases').iterdir() if p.is_dir()), sorted(r['case_directory'] for r in manifest), 'case folder coverage')
        audit.same(protocol['execution_order'], [dict(case_id=c, variant=v) for c in FIXED_CASES for v in VARIANTS], 'primary fixed order')
        mpc = read(run/'mpc_output/provenance.json')
        audit.same(mpc['official_settings'], source['official_mpc']['official_settings'], 'official settings unchanged')
        settings_hash = hashlib.sha256(json.dumps(mpc['official_settings'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        audit.same(mpc['official_settings_sha256'], settings_hash, 'official settings hash')
        audit.same(mpc['request_sha256'], digest(run/'mpc_request.json'), 'actual worker request')
        audit.same(read(run/'mpc_output/request.json'), read(run/'mpc_request.json'), 'actual worker request values')
        for r in read(run/'mpc_output/output_hashes.json')['files']:
            audit.same(digest(run/'mpc_output'/r['path']), r['sha256'], 'worker output '+r['path'])
        for r in read(run/'mpc_output/input_hashes.json')['files']:
            audit.same(digest(r['path']), r['sha256'], 'worker input '+r['path'])
        totals = read(run/'mpc_output/summary.json')
        for k, v in dict(independent_rollouts=8, primary_mpc_solves=240, historical_or_other_mpc_solves=0,
                         matched_state_selector_calls=240, new_VLA_updates=0, GP_or_rigid_optimization_solves=0).items():
            audit.same(totals[k], v, 'worker count '+k)
        audit.same(read(run/'execution_completed.json')['exit_code'], 0, 'worker normal completion')
        pure = official_selector(mpc['mpc_source'])
        env = HospitalEnvironment.load(source['environment_path'])
        return source, config, manifest, mpc, pure, env
    loaded = audit.capture('global source/runtime', global_checks)
    all_rows, all_selection, all_reproduction = [], [], {}
    all_matched_pairs, all_closed_pairs = [], []
    if loaded:
        source, config, manifest, mpc, pure, env = loaded
        weights = mpc['official_settings']['Q_WEIGHTS']
        for case_index, entry in enumerate(manifest):
            start_checks, start_errors = audit.checks, len(audit.errors)
            def case_checks():
                case = run/'cases'/entry['case_directory']; context = read(case/'input_context.json'); route = read(case/'goal_route.json')
                audit.same(sorted(p.name for p in (case/'variants').iterdir() if p.is_dir()), sorted(VARIANTS), 'variant folder coverage')
                fresh = load_frozen_context(source['source_run'], context['episode_id'], context['handoff_id'])
                audit.same(context, fresh, entry['case_id']+'/original raw context', 0.)
                audit.same(route, read(Path(source['reference_source'])/'cases'/entry['case_directory']/'goal_route.json'), 'original goal/route', 0.)
                audit.same(route['gates'], [], 'no gate scope')
                native, common = [np.load(case/p, allow_pickle=False) for p in ('F_native.npy', 'F_common.npy')]
                audit.same(native, context['fresh_world'], 'original native values', 0.)
                factored = build_variants(native, context['B_world'], weights, expected_common=common,
                                         preparation_metadata=read(case/'reference_preparation.json'))
                audit.same(read(case/'reference_factorization.json'), {k:v for k,v in factored.items() if k != 'variants'}, 'factorization metadata')
                matched = read(case/'matched_state_probes.json')
                audit.same(matched, read(run/'mpc_output'/entry['case_directory']/'matched_state_probes.json'), 'matched worker evidence', 0.)
                historical = read(case/'historical/M0_NATIVE/rollout.json')
                audit.same(matched['new_mpc_solves'], 0, 'matched no solve')
                audit.same(matched['state_integration_performed'], False, 'matched no integration')
                audit.same(matched['controller_memory_modified'], False, 'matched no memory edit')
                audit.same(len(matched['states']), 30, 'matched all states')
                variants = {}; reproduced = {}
                for variant_index, name in enumerate(VARIANTS):
                    folder = case/'variants'/name; reference = np.load(folder/'reference_world.npy', allow_pickle=False)
                    lineage, geometry = read(folder/'row_provenance.json'), read(folder/'geometry_audit.json')
                    expected = factored['variants'][name]
                    audit.same(reference, expected['reference_world'], name+'/exact factor input', 0.)
                    audit.same(lineage, expected['row_provenance'], name+'/original resampling lineage')
                    audit_lineage(audit, native, reference, lineage, name)
                    audit.same(geometry, expected['geometry_audit'], name+'/geometry audit')
                    audit.same(read(folder/'definition.json'), {k:v for k,v in expected.items() if k not in ('reference_world','row_provenance','geometry_audit')}, name+'/definition')
                    for p, h in read(folder/'source.json')['derived_sha256'].items():
                        audit.same(digest(folder/p), h, name+'/derived hash '+p)
                    for i, probe in enumerate(matched['states']):
                        audit.same(probe['input_pose_world'], historical['controller_reference_selections'][i]['input_pose_world'], name+f'/same probe state {i}', 0.)
                        audit.same(probe['time_s'], i*6/60, name+f'/same probe time {i}', 0.)
                        audit_selection(audit, pure, reference, probe['input_pose_world'], lineage, probe['variants'][name], weights, name+f'/matched selection {i}')
                    rollout, metrics = read(folder/'rollout.json'), read(folder/'metrics.json')
                    audit.same(rollout, read(run/'mpc_output'/entry['case_directory']/name/'rollout.json'), name+'/new worker rollout', 0.)
                    audit_rollout(audit, pure, rollout, context, reference, lineage, weights, case_index*4+variant_index, name)
                    recomputed = evaluate_variant(name, rollout, context, reference, route, env, config)
                    recomputed['summary']['case_role'] = entry['case_role']
                    for key, value in recomputed.items():
                        if key != 'independent_evaluation_wall_s':
                            audit.same(metrics[key], value, name+'/original outcome '+key)
                    all_rows.append(metrics['summary'])
                    for kind, records in [('matched_state', [{**p, 'selection_diagnostic':p['variants'][name]} for p in matched['states']]),
                                          ('closed_loop', rollout['controller_reference_selections'])]:
                        all_selection.append(dict(case_id=context['case_id'], variant=name, mode=kind, **selection_diagnostics(records)))
                    if name in ('R00_NATIVE', 'R11_CURRENT_ADAPTER'):
                        method = 'M0_NATIVE' if name == 'R00_NATIVE' else 'M0_ADAPTER'
                        reproduced[name] = reproduction_check(rollout, read(case/'historical'/method/'rollout.json'), metrics,
                                                            read(case/'historical'/method/'metrics.json'))
                    variants[name] = dict(reference=reference, provenance=lineage, geometry=geometry, rollout=rollout, metrics=metrics)
                saved_reproduction = read(case/'reproduction_check.json')
                # The runner wraps the two fully recomputed reports with a case ID.
                mapping = saved_reproduction.get('variants', saved_reproduction.get('checks', saved_reproduction))
                for name, report in reproduced.items():
                    audit.same(mapping[name], report, name+'/historical reproduction truth')
                reproduction_expected = dict(case_id=context['case_id'], variants=reproduced,
                    historical_success={name: read(case/'historical'/('M0_NATIVE' if name == 'R00_NATIVE' else 'M0_ADAPTER')/'metrics.json')['primary_success'] for name in reproduced},
                    passed=all(r['passed'] for r in reproduced.values()))
                audit.same(saved_reproduction, reproduction_expected, 'complete reproduction report')
                all_reproduction[context['case_id']] = reproduction_expected
                all_matched_pairs.extend(independent_pairwise(context['case_id'], matched['states'], matched=True))
                all_closed_pairs.extend(independent_pairwise(context['case_id'], {n:v['rollout']['controller_reference_selections'] for n,v in variants.items()}, matched=False))
                if (case/'plots/outcome_summary.json').exists() or require_finalized:
                    audit_plots(audit, run, case, context, route, variants, matched, config)
                else:
                    deferred.append('plots '+entry['case_id'])
            audit.capture('case '+entry['case_id'], case_checks)
            cases_report.append(dict(case_id=entry['case_id'], valid=len(audit.errors)==start_errors,
                                     checks=audit.checks-start_checks, errors=audit.errors[start_errors:]))
        if len(all_rows) == 8:
            audit.capture('outcomes CSV', lambda: audit_csv(audit, run/'aggregate/rollout_outcomes.csv', all_rows, 'outcomes'))
            contrasts = independent_contrasts(all_rows)
            audit.capture('contrasts CSV', lambda: audit_csv(audit, run/'aggregate/factor_contrasts.csv', contrasts, 'contrasts'))
            for name, expected in [('factor_contrasts', contrasts), ('selection_diagnostics', all_selection),
                                   ('matched_state_pairwise', all_matched_pairs), ('closed_loop_pairwise', all_closed_pairs)]:
                audit.capture(name+' JSON', lambda n=name, e=expected: audit.same(read(run/'aggregate'/f'{n}.json'), e, n))
                audit.capture(name+' CSV', lambda n=name, e=expected: audit_csv(audit, run/'aggregate'/f'{n}.csv', e, n))
            summary = read(run/'aggregate/summary.json')
            from run_gp_se2_ref01 import diagnosis
            expected_diagnosis = diagnosis(all_rows, contrasts, all_reproduction, all_matched_pairs, yaml.safe_load((run/'experiment_config.yaml').read_text()))
            audit.same(summary['diagnosis'], expected_diagnosis, 'all descriptive flags')
            audit.same(summary['operational_status'], 'GP_SE2_REF_01_COMPLETED' if all(r['passed'] for r in all_reproduction.values()) else 'GP_SE2_REF_01_COMPLETED_WITH_LIMITATIONS', 'operational reproduction status')
            for k,v in dict(rollouts=8, primary_mpc_solves=240, historical_or_other_mpc_solves=0, matched_state_selector_calls=240, all_cases_and_variants_accounted=True, original_acceptance_unchanged=True, new_VLA_updates=0, GP_or_rigid_optimization_solves=0, optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY').items():
                audit.same(summary[k], v, 'summary '+k)
        else:
            audit.check(False, 'all eight evaluated outcomes required')
        if (run/'review_bundle.zip').exists() or require_finalized:
            audit.capture('strict review bundle', lambda: audit_bundle(audit, run))
        else:
            deferred.append('review bundle strict ZIP allowlist and hashes')
        if (run/'plot_manifest.json').exists() or require_finalized:
            plots = audit.capture('plot manifest', lambda: read(run/'plot_manifest.json'))
        else:
            plots = None
            deferred.append('plot manifest coverage')
        if plots:
            audit.same(plots['image_count'], 22, 'plot coverage count')
            audit.same(len(plots['images']), 22, 'plot coverage records')
            expected = {f"cases/{entry['case_directory']}/plots/{name}.png" for entry in manifest for name in PLOTS}
            audit.same(set(p['path'] for p in plots['images']), expected, 'plot exact coverage')
    report = dict(experiment='GP-SE2-REF-01', valid=not audit.errors, checks=audit.checks, errors=audit.errors,
                  authoritative=require_finalized, deferred_checks=deferred,
                  validation_scope='complete finalized artifacts' if require_finalized else 'non-authoritative preflight; final checks explicitly deferred',
                  case_validations=cases_report, scientific_failures_are_not_validation_failures=True,
                  independent_saved_evidence_only=True, optimizer_or_MPC_solves_performed=0,
                  official_selector_verification='hash-pinned AST-extracted original pure functions; no controller import',
                  validation_wall_s=time.perf_counter()-begin, validator_sha256=digest(__file__),
                  generated_utc=datetime.now(timezone.utc).isoformat())
    if write_output:
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--output', type=Path); parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args(); result = validate(args.run, output=args.output, require_finalized=not args.preflight)
    print(json.dumps({k:v for k,v in result.items() if k != 'case_validations'}, indent=2))
    raise SystemExit(0 if result['valid'] else 1)
