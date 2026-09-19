#!/usr/bin/env python3
"""Freeze and run the fixed-geometry REF-02 selector intervention exactly once."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import clean, read, write, copy_new, digest, utc, revision, table
from reconciliation.gp_se2_ref02_evaluation import METHODS, FIXED_CASES, REPRODUCTION_TOLERANCES

STARTING_SHA = '9a615cee443848e79fff8805e9a038bbbcb2c341'
CASES = list(zip(FIXED_CASES, ('PRIMARY_LARGE_TURN', 'BENIGN_CONTROL')))
SOURCE_VARIANTS = dict(zip(METHODS, ('R00_NATIVE', 'R11_CURRENT_ADAPTER', 'R11_CURRENT_ADAPTER')))
REPORTING_ONLY = {'scripts/validate_gp_se2_ref02.py', 'tests/test_gp_se2_ref02_artifacts.py'}
SELECTOR_DEFINITION = dict(
    nearest='original weighted squared XY and atan2(sin,cos) wrapped-yaw cost; numpy.argmin first minimum',
    source_progress='immutable REF-01 original_fractional_row_coordinate; row order, not physical time or distance',
    A_B_indices='min(j+h,M-1), h=1..H, original official selector callback',
    C_queries='q_h=min(s_j+h,s_last), h=1..H, all offsets from the same unrounded s_j',
    C_indices='searchsorted(s,q_h,side=left), clamp to final existing row; stride exactly 1.0',
    precision='CPU float64; strict comparison, no search epsilon or quantization of s_j',
    target_values='installed C[selected_indices].copy(); no interpolation, raw substitution or path mutation',
    yaw='official scalar sequential unwrap starting from current robot yaw',
    constant_reference='only validated single-source-row metadata plus identical pose rows; repeat final row',
    ordinary_lineage='strictly increasing finite progress; invalid lineage is an error',
    stateful_progress=False, suffix_recomputed=False, rounding_nearest_progress=False,
    MPC_calculation_unchanged=True, reference_selector_changed=True,
    whole_controller_unchanged_claim=False, external_source_modified=False,
    floating_point_boundary_policy_frozen_before_results=True,
)


def validate_config(c):
    if c['experiment'] != 'GP-SE2-REF-02' or c['starting_git_sha'] != STARTING_SHA:
        raise ValueError('fixed REF-02 source revision required')
    if [(x['case_id'], x['role']) for x in c['cases']] != CASES or c['methods'] != list(METHODS):
        raise ValueError('fixed case/method order required')
    if c['source_variants'] != SOURCE_VARIANTS:
        raise ValueError('source arrays may not be replaced')
    expected = dict(source_progress_stride=1., searchsorted_side='left', search_tolerance=0.,
        round_nearest_progress=False, accumulate_overshoot=False, interpolate_targets=False, progress_memory=False)
    if c['selector'] != expected:
        raise ValueError('fixed strict original-progress selector required')
    if c['rollout'] != dict(horizon_s=3., control_hz=10., integration_hz=60.):
        raise ValueError('original integration and control schedule required')
    if c['mpc'] != dict(HORIZON=5, MPC_DT_S=.1, CONTROL_RATE_HZ=10., Q_WEIGHTS=[10., 10., 1.]):
        raise ValueError('original MPC settings required')
    if c['probe_times_s'] != [0., 1., 2.] or c['blas_threads'] != 1 or c['command_divergence_atol'] != 1e-6:
        raise ValueError('frozen probe/display/numerical policies required')
    if any(c[x] for x in ('new_vla_inference', 'gp_or_rigid_optimization', 'new_reference_interpolation', 'retry')):
        raise ValueError('diagnostic scope changed')
    return c


def preservation(primary):
    """Rehash transitive originals plus every REF-01 result and presentation."""
    primary = Path(primary)
    prior = read(primary/'preservation_after.json')
    expected = dict(prior['files'])
    for p, h in read(primary/'artifact_manifest.json')['files'].items():
        if digest(primary/p) != h:
            raise ValueError('authoritative REF-01 artifact changed: '+p)
    core = read(primary/'source.json')['preserved_core_sha256'] | read(primary/'implementation_sources/final/manifest.json')['files']
    expected.update({str(ROOT/p): h for p, h in core.items()})
    folders = [primary, Path(read(primary/'supplemental_presentation.json')['output'])]
    for folder in folders:
        expected.update({str(p.resolve()): digest(p) for p in sorted(folder.rglob('*')) if p.is_file()})
    current = {p: digest(p) if Path(p).is_file() else None for p in expected}
    errors = [p for p, h in expected.items() if current[p] != h]
    provenance = read(primary/'mpc_output/provenance.json')
    checkout = provenance['lightnav_checkout']
    ext_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=checkout, text=True).strip()
    ext_status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=checkout, text=True).strip()
    if ext_sha != provenance['lightnav_sha'] or ext_status:
        errors.append('official external checkout changed')
    users = {p: digest(ROOT/p) for p in prior['user_config_hashes']}
    return dict(valid=not errors, mismatches=errors, files=current, user_config_hashes=users,
                external_git_sha=ext_sha, external_git_status=ext_status, captured_utc=utc())


def prepare(run, config_path):
    from reconciliation.gp_se2_rollout import load_frozen_context
    from reconciliation.gp_se2_ref02_reference import validate_lineage
    begin = time.perf_counter()
    c = validate_config(yaml.safe_load(config_path.read_text()))
    primary, related = [ROOT/c[k] for k in ('primary_source', 'related_source')]
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_ref_02'):
        raise ValueError('new REF-02 run directory required; no overwrite')
    if subprocess.run(['git', 'merge-base', '--is-ancestor', STARTING_SHA, 'HEAD'], cwd=ROOT).returncode:
        raise ValueError('reviewed commit not present')
    prior = read(primary/'source.json'); mpc = read(primary/'mpc_output/provenance.json')
    if Path(prior['primary_source']).resolve() != related.resolve():
        raise ValueError('related GP-SE2-02 source provenance mismatch')
    for p in (primary/'validation.json', related/'validation.json'):
        v = read(p)
        if not v['valid'] or (p.parent == primary and not v['authoritative']):
            raise ValueError('authoritative prior validation must pass')
    if digest(primary/'config_snapshot.yaml') != digest(related/'config_snapshot.yaml'):
        raise ValueError('original acceptance config differs')
    check_start = time.perf_counter(); before = preservation(primary)
    preservation_wall = time.perf_counter()-check_start
    if not before['valid']:
        raise ValueError('original source hash mismatch')
    if any(mpc['official_settings'][k] != v for k, v in c['mpc'].items()):
        raise ValueError('actual pinned MPC settings differ')
    run.mkdir(parents=True)
    copy_new(config_path, run/'experiment_config.yaml')
    copy_new(primary/'config_snapshot.yaml', run/'config_snapshot.yaml')
    write(run/'preservation_before.json', before)
    write(run/'selector_definition.json', SELECTOR_DEFINITION)
    copies, cases, requests, fixed_checks = {}, [], [], []
    lookup = {r['case_id']: r for r in read(primary/'case_manifest.json')['selected']}
    def copied(src, dst):
        copy_new(src, dst); copies[str(src.resolve())] = digest(src)
    for case_id, role in CASES:
        row = lookup[case_id]; src = primary/'cases'/row['case_directory']; dst = run/'cases'/row['case_directory']
        for name in ('input_context.json', 'goal_route.json', 'F_native.npy', 'F_common.npy', 'reference_preparation.json',
                     'reference_factorization.json', 'actual_past_execution.json'):
            copied(src/name, dst/name)
        context = read(dst/'input_context.json')
        if load_frozen_context(prior['source_run'], context['episode_id'], context['handoff_id']) != context:
            raise ValueError('recorded raw handoff context changed')
        if read(dst/'goal_route.json')['gates']:
            raise ValueError('original required gate may not be removed')
        for name in ('old_raw_local.npy', 'old_world.npy', 'fresh_raw_local.npy', 'fresh_world.npy'):
            copied(src/'input_snapshot'/name, dst/'input_snapshot'/name)
        copied(src/'matched_state_probes.json', dst/'historical/matched_state_probes.json')
        copied(src/'historical/M0_NATIVE/rollout.json', dst/'historical/original_native_probe_rollout.json')
        methods = {}
        for method in METHODS:
            old = src/'variants'/SOURCE_VARIANTS[method]; folder = dst/'methods'/method
            for name in ('reference_world.npy', 'row_provenance.json', 'geometry_audit.json', 'definition.json'):
                copied(old/name, folder/name)
            reference = np.load(folder/'reference_world.npy', allow_pickle=False)
            progress = validate_lineage(reference, read(folder/'row_provenance.json'))
            if method == METHODS[0] and not np.array_equal(reference, context['fresh_world']):
                raise ValueError('Native source is not original FRESH')
            methods[method] = dict(reference_path=str((folder/'reference_world.npy').resolve()),
                lineage_path=str((folder/'row_provenance.json').resolve()))
            write(folder/'source.json', dict(case_id=case_id, method=method, original_REF01_variant=SOURCE_VARIANTS[method],
                reference_sha256=digest(folder/'reference_world.npy'), lineage_sha256=digest(folder/'row_provenance.json'),
                reference_shape=list(reference.shape), reference_dtype=str(reference.dtype),
                first_source_progress=float(progress[0]), last_source_progress=float(progress[-1]),
                raw_modified=False, reference_regenerated=False, reanchored=False,
                frame=context['frame'], source_timestamps=context['source_timestamps']))
            if method != METHODS[2]:
                for name in ('rollout.json', 'metrics.json'):
                    copied(old/name, dst/'historical'/method/name)
        b, z = [dst/'methods'/m for m in METHODS[1:]]
        x, y = [np.load(folder/'reference_world.npy', allow_pickle=False) for folder in (b, z)]
        fixed = dict(case_id=case_id, B_reference_sha256=digest(b/'reference_world.npy'), C_reference_sha256=digest(z/'reference_world.npy'),
            B_lineage_sha256=digest(b/'row_provenance.json'), C_lineage_sha256=digest(z/'row_provenance.json'),
            shape=list(x.shape), dtype=str(x.dtype), byte_identical=x.dtype == y.dtype and x.shape == y.shape and x.tobytes() == y.tobytes(),
            file_identical=digest(b/'reference_world.npy') == digest(z/'reference_world.npy'),
            lineage_identical=digest(b/'row_provenance.json') == digest(z/'row_provenance.json'),
            native_row_count=len(context['fresh_world']), dense_row_count=len(x))
        if not all(fixed[k] for k in ('byte_identical', 'file_identical', 'lineage_identical')) or len(x) != 30:
            raise ValueError('B/C fixed original dense geometry/lineage not preserved')
        write(dst/'fixed_dense_reference.json', fixed); fixed_checks.append(fixed)
        cases.append(dict(case_id=case_id, case_role=role, case_directory=row['case_directory'], original_manifest_row=row))
        requests.append(dict(episode_id=context['episode_id'], handoff_id=context['handoff_id'],
            input_context_path=str((dst/'input_context.json').resolve()), goal_route_path=str((dst/'goal_route.json').resolve()),
            historical_native_rollout_path=str((dst/'historical/original_native_probe_rollout.json').resolve()),
            historical_matched_probes_path=str((dst/'historical/matched_state_probes.json').resolve()), methods=methods))
    (run/'aggregate').mkdir()
    write(run/'case_manifest.json', dict(selected=cases, selected_count=2, methods=list(METHODS), result_based_selection=False))
    write(run/'aggregate/fixed_dense_reference.json', fixed_checks)
    source = dict(experiment='GP-SE2-REF-02', created_utc=utc(), starting_git_sha=STARTING_SHA,
        preparation_git_sha=revision(), primary_source=str(primary.resolve()), related_source=str(related.resolve()),
        source_run=prior['source_run'], environment_path=prior['environment_path'],
        environment_export_source=prior['environment_export_source'], environment_file_hashes=prior['environment_file_hashes'],
        original_config_sha256=digest(run/'config_snapshot.yaml'), experiment_config_sha256=digest(run/'experiment_config.yaml'),
        authoritative_validations={str(p.resolve()):digest(p) for p in (primary/'validation.json', related/'validation.json')},
        copied_source_sha256=copies, preserved_core_sha256=prior['preserved_core_sha256'] | read(primary/'implementation_sources/final/manifest.json')['files'],
        original_MPC_environment=prior['original_MPC_environment'], official_mpc=mpc,
        intrinsic_lightnav_waypoint_dt_s=None, frame=prior['frame'],
        preparation_total_wall_s=time.perf_counter()-begin, preservation_hash_audit_wall_s=preservation_wall,
        timing_semantics='preparation total includes preservation hashing; no MPC optimization, plotting or tests',
        reference_selector_changed=True, MPC_calculation_unchanged=True, external_source_modified=False)
    write(run/'source.json', source)
    write(run/'protocol.json', dict(experiment='GP-SE2-REF-02', frozen_before_primary=True, configuration=c,
        execution_order=[dict(case_id=case_id, method=m) for case_id, _ in CASES for m in METHODS],
        matched_state_probe_count=180, primary_rollout_count=6, primary_mpc_solve_count=180, historical_solve_count=0,
        matched_states_source='exact same saved historical Native control input poses used in REF-01',
        probe_policy='selector calls only, before all rollouts; no memory/state update or MPC solve',
        settings=mpc['official_settings'], settings_sha256=mpc['official_settings_sha256'],
        reproduction_tolerances=REPRODUCTION_TOLERANCES, physical_acceptance_config_sha256=digest(run/'config_snapshot.yaml'),
        selector_definition=SELECTOR_DEFINITION, optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY'))
    write(run/'mpc_request.json', dict(source_root=prior['source_run'], expected_official_settings_sha256=mpc['official_settings_sha256'],
        cases=requests, **c['rollout']))
    print(json.dumps(dict(phase='PREPARED', cases=2, methods=6, fixed_dense_reference_preserved=True)), flush=True)


def snapshot(run, phase):
    names = list((ROOT/'src/reconciliation').glob('gp_se2_ref02*.py'))
    names += list((ROOT/'scripts').glob('*gp_se2_ref02*.py')) + list((ROOT/'scripts/lightnav').glob('*gp_se2_ref02*.py'))
    names += list((ROOT/'tests').glob('test_gp_se2_ref02*.py')) + [ROOT/'configs/gp_se2_ref02.yaml']
    folder = run/'implementation_sources'/phase; folder.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for path in sorted(names):
        name = str(path.relative_to(ROOT))
        if phase == 'execution' and name in REPORTING_ONLY:
            continue
        copy_new(path, folder/name); hashes[name] = digest(path)
    write(folder/'manifest.json', dict(files=hashes, reporting_only_excluded=sorted(REPORTING_ONLY)))
    return hashes


def verify(run):
    source = read(run/'source.json'); config = validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()))
    if digest(run/'config_snapshot.yaml') != source['original_config_sha256'] or digest(run/'experiment_config.yaml') != source['experiment_config_sha256']:
        raise ValueError('frozen configuration drift')
    for name, value in source['copied_source_sha256'].items():
        if digest(name) != value: raise ValueError('original input drift: '+name)
    for name, value in source['preserved_core_sha256'].items():
        if digest(ROOT/name) != value: raise ValueError('historical code changed: '+name)
    if (run/'execution_freeze.json').exists():
        frozen = read(run/'execution_freeze.json')
        for name, value in frozen['source_sha256'].items():
            if digest(ROOT/name) != value: raise ValueError('frozen execution code drift: '+name)
        for name, value in frozen['input_sha256'].items():
            if digest(run/name) != value: raise ValueError('frozen input drift: '+name)
    return config, source


def freeze(run):
    verify(run)
    hashes = snapshot(run, 'execution')
    inputs = {str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file()
              and not p.is_relative_to(run/'implementation_sources') and not p.name.endswith('.log')}
    write(run/'execution_freeze.json', dict(frozen_utc=utc(), execution_git_sha=revision(), source_sha256=hashes,
        input_sha256=inputs, no_primary_execution_yet=True))


def execute(run):
    _, source = verify(run)
    if not (run/'execution_freeze.json').exists(): raise ValueError('freeze before selector probes and MPC solves')
    if (run/'execution_started.json').exists() or (run/'mpc_output').exists(): raise FileExistsError('primary already started; no retry')
    command = [str(Path(source['original_MPC_environment'])/'bin/python'), str(ROOT/'scripts/lightnav/gp_se2_ref02_mpc.py'),
        '--request', str(run/'mpc_request.json'), '--output', str(run/'mpc_output'), '--lightnav-checkout', source['official_mpc']['lightnav_checkout']]
    write(run/'execution_started.json', dict(utc=utc(), command=command, request_sha256=digest(run/'mpc_request.json')))
    start = time.perf_counter()
    with (run/'mpc_worker.log').open('x') as stream:
        result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    write(run/'execution_completed.json', dict(utc=utc(), exit_code=result.returncode, worker_total_wall_s=time.perf_counter()-start, command=command))
    if result.returncode: raise RuntimeError('worker failed; preserve failure, fix technically then use a new run ID')
    print(json.dumps(dict(phase='EXECUTED', worker_total_wall_s=time.perf_counter()-start)), flush=True)


def evaluate(run):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_ref02_evaluation import (evaluate_method, reproduction_check, paired_comparisons,
        selection_mechanism, compare_selections, first_command_divergences, result_flags)
    from reconciliation.gp_se2_ref02_reference import enrich_selector_result
    c, source = verify(run); output = run/'mpc_output'
    for record in read(output/'output_hashes.json')['files']:
        path = (output/record['path']).resolve()
        if not path.is_relative_to(output) or digest(path) != record['sha256']: raise ValueError('worker output integrity mismatch')
    if read(output/'provenance.json')['request_sha256'] != digest(run/'mpc_request.json'): raise ValueError('worker request changed')
    start = time.perf_counter(); env = HospitalEnvironment.load(source['environment_path']); environment_s = time.perf_counter()-start
    begin = time.perf_counter(); config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    rows, outcomes, selections, matches, closed, divergent, reproductions, timings = [], [], [], [], [], [], {}, []
    for case in read(run/'case_manifest.json')['selected']:
        folder = run/'cases'/case['case_directory']; worker = output/case['case_directory']
        context, goal = read(folder/'input_context.json'), read(folder/'goal_route.json')
        copy_new(worker/'matched_state_probes.json', folder/'matched_state_probes.json'); probes = read(folder/'matched_state_probes.json')
        reproduction = dict(case_id=case['case_id'], methods={}); actual = {}
        installed = {}
        for method in METHODS:
            target = folder/'methods'/method
            for p in sorted((worker/method).iterdir()):
                if p.is_file(): copy_new(p, target/p.name)
            rollout = read(target/'rollout.json'); ref = np.load(target/'reference_world.npy', allow_pickle=False)
            lineage = read(target/'row_provenance.json'); installed[method] = np.asarray(rollout['installed_reference_world'])
            if rollout['candidate_source']['sha256'] != digest(target/'reference_world.npy'): raise ValueError('actual path file hash differs')
            for solve in rollout['controller_reference_selections']:
                checked = enrich_selector_result(installed[method], solve['input_pose_world'], solve['selection']['reference_world'], lineage,
                    method=method, weights=c['mpc']['Q_WEIGHTS'], horizon=c['mpc']['HORIZON'], original_goal_row_index=len(context['fresh_world'])-1)
                for key in ('nearest_index', 'indices', 'reference_world', 'selected_original_fractional_row_coordinates'):
                    if checked[key] != solve['selection_diagnostic'][key]: raise ValueError('independent selection mismatch: '+key)
            metrics = evaluate_method(method, rollout, context, ref, goal, env, config)
            metrics['summary']['case_role'] = case['case_role']; write(target/'metrics.json', metrics)
            rows.append(metrics['summary']); outcomes.append(metrics); actual[method] = rollout['controller_reference_selections']
            for mode, records in [('matched_state', [dict(time_s=p['time_s'], selection_diagnostic=p['methods'][method]) for p in probes['states']]),
                                  ('closed_loop', actual[method])]:
                selections.append(dict(case_id=case['case_id'], method=method, mode=mode, **selection_mechanism(records)))
            if method != METHODS[2]:
                previous = folder/'historical'/method
                check = reproduction_check(rollout, read(previous/'rollout.json'), metrics, read(previous/'metrics.json'))
                check['comparison'] = 'first prescribed REF-02 primary versus preserved REF-01 actual rollout'
                reproduction['methods'][method] = check
            timings.append(dict(case_id=case['case_id'], method=method, mpc_solves=len(actual[method]),
                rollout_wall_s=rollout['rollout_wall_s'], official_mpc_solve_wall_s=sum(v for v in rollout['official_mpc_solve_wall_s'] if v is not None),
                independent_evaluation_wall_s=metrics['independent_evaluation_wall_s']))
        if not np.array_equal(installed[METHODS[1]], installed[METHODS[2]]): raise ValueError('B/C installed world arrays differ')
        reproduction['passed'] = all(x['passed'] for x in reproduction['methods'].values())
        write(folder/'reproduction_check.json', reproduction); reproductions[case['case_id']] = reproduction
        matches.extend(compare_selections(case['case_id'], probes['states'], matched=True))
        closed.extend(compare_selections(case['case_id'], actual, matched=False))
        divergent.extend(first_command_divergences(case['case_id'], actual, atol=c['command_divergence_atol']))
    comparisons = paired_comparisons(rows)
    fixed = all(x['byte_identical'] and x['file_identical'] and x['lineage_identical'] for x in read(run/'aggregate/fixed_dense_reference.json'))
    flags = result_flags(rows, reproductions, matches, fixed_dense_preserved=fixed, selection_validated=True)
    worker_summary = read(output/'summary.json')
    summary = dict(experiment='GP-SE2-REF-02', operational_status='GP_SE2_REF_02_COMPLETED' if flags['baseline_reproduced'] else 'GP_SE2_REF_02_COMPLETED_WITH_LIMITATIONS',
        result=flags, primary_rollouts=6, primary_mpc_solves=worker_summary['primary_mpc_solves'],
        matched_state_selector_calls=worker_summary['matched_state_selector_calls'], historical_or_other_mpc_solves=0,
        MPC_calculation_unchanged=True, reference_selector_changed=True, external_source_modified=False,
        GP_or_rigid_optimization_solves=0, new_VLA_updates=0, optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY',
        timing=dict(preparation_total_wall_s=source['preparation_total_wall_s'], preservation_hash_audit_wall_s=source['preservation_hash_audit_wall_s'],
            environment_loading_s=environment_s, evaluation_wall_s=time.perf_counter()-begin,
            worker_process_wall_s=read(run/'execution_completed.json')['worker_total_wall_s'], worker_details=worker_summary),
        status_requirement='completion requires authoritative validation.json to pass',
        claim_limit='offline fixed-event selector diagnosis, not a sampling-invariant controller or GP/navigation improvement')
    for name, values in [('outcomes', outcomes), ('paired_comparison', comparisons), ('selection_mechanism', selections),
                         ('matched_state_pairwise', matches), ('closed_loop_pairwise', closed), ('command_divergence', divergent), ('timing', timings)]:
        write(run/'aggregate'/f'{name}.json', values)
        table(run/'aggregate'/f'{name}.csv', rows if name == 'outcomes' else values)
    write(run/'aggregate/summary.json', summary)
    write(run/'evaluation_completed.json', dict(utc=utc(), methods=6, reproduction_pass=flags['baseline_reproduced']))
    print(json.dumps(summary, indent=2), flush=True)


def finalize(run):
    _, source = verify(run); before = read(run/'preservation_before.json'); after = preservation(source['primary_source'])
    for key in ('files', 'user_config_hashes', 'external_git_sha', 'external_git_status'):
        if before[key] != after[key]: raise ValueError('preserved originals changed: '+key)
    if not after['valid']: raise ValueError('original source integrity failed')
    write(run/'preservation_after.json', after)
    write(run/'implementation_final.json', dict(utc=utc(), source_sha256=snapshot(run, 'final')))
    files = {str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file()
        and p.name not in ('validation.json', 'artifact_manifest.json', 'git_completion.json')}
    write(run/'artifact_manifest.json', dict(files=files, created_utc=utc()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['prepare', 'freeze', 'execute', 'evaluate', 'finalize'])
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/gp_se2_ref02.yaml')
    args = parser.parse_args(); run = args.run.resolve()
    try:
        if args.phase == 'prepare': prepare(run, args.config.resolve())
        else: globals()[args.phase](run)
    except Exception as error:
        if run.exists() and not (run/f'technical_failure_{args.phase}.json').exists():
            write(run/f'technical_failure_{args.phase}.json', dict(utc=utc(), error=f'{type(error).__name__}: {error}',
                retry_in_same_run_forbidden=True, original_results_preserved=True))
        raise


if __name__ == '__main__':
    main()
