#!/usr/bin/env python3
"""Frozen two-factor reference diagnostic. No GP/rigid optimizer is invoked."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml

CASES = [('episode_014_repeat_01/handoff_026', 'PRIMARY_LARGE_TURN'),
         ('episode_001_repeat_01/handoff_002', 'BENIGN_CONTROL')]
VARIANTS = ('R00_NATIVE', 'R10_SUFFIX_ONLY', 'R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER')
STARTING_SHA = '47668b868e84173fab4516ab7d5edb65ef75b841'
REPORTING_ONLY = {'scripts/validate_gp_se2_ref01.py', 'tests/test_gp_se2_ref01_artifacts.py'}


def utc():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(clean(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def array(path, value):
    with Path(path).open('xb') as stream:
        np.save(stream, np.asarray(value), allow_pickle=False)


def table(path, rows):
    rows = clean(rows)
    fields = list(dict.fromkeys(k for row in rows for k in row)) or ['status']
    with Path(path).open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in row.items()})


def copy_new(source, target):
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with Path(source).open('rb') as src, target.open('xb') as dst:
        shutil.copyfileobj(src, dst)
    if digest(source) != digest(target):
        raise ValueError('copy hash mismatch')


def revision():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()


def validate_config(config):
    if config['experiment'] != 'GP-SE2-REF-01' or config['starting_git_sha'] != STARTING_SHA:
        raise ValueError('fixed experiment/source revision required')
    if [(r['case_id'], r['role']) for r in config['cases']] != CASES or config['variants'] != list(VARIANTS):
        raise ValueError('fixed cases, variants and order required')
    if config['rollout'] != dict(horizon_s=3., control_hz=10., integration_hz=60.):
        raise ValueError('original rollout schedule required')
    if config['reference'] != dict(first_time_s=.1, final_time_s=3., output_dt_s=.1):
        raise ValueError('original row interpolation convention required')
    if config['mpc'] != dict(HORIZON=5, MPC_DT_S=.1, CONTROL_RATE_HZ=10.):
        raise ValueError('original MPC settings required')
    if any(config[k] for k in ('new_vla_inference', 'new_data_collection', 'gp_or_rigid_optimization', 'controller_modified', 'retry')):
        raise ValueError('diagnostic scope cannot change')
    if config['blas_threads'] != 1 or config['descriptive_effect_floor'] != 1e-6:
        raise ValueError('frozen numerical diagnostics required')
    expected = dict(selected_pose_atol=1e-12, command_atol=1e-6, state_atol=1e-6,
                    endpoint_metric_atol=1e-6, rtol=0., indices='exact', success_flags='exact')
    if any(config['reproduction'][k] != v for k, v in expected.items()):
        raise ValueError('predeclared reproduction diagnostic criteria required')
    return config


def preservation(primary):
    """Rehash prior authoritative inventories, including prior results/images."""
    primary = Path(primary)
    previous = read(primary/'preservation_after.json')
    expected = {}
    for group in ('historical_files', 'previous_diag01_files', 'previous_diag02_files', 'external_files'):
        expected.update({str((ROOT/Path(p)).resolve()): h for p, h in previous[group].items()})
    for path in (primary, Path(read(primary/'final_report.json')['gui']['runtime_validation_path']).parent):
        expected.update({str(p.resolve()): digest(p) for p in sorted(path.rglob('*')) if p.is_file()})
    # Snapshot current prior output inventory; verify each published original file first.
    for p, h in read(primary/'artifact_manifest.json')['files'].items():
        if digest(primary/p) != h:
            raise ValueError('authoritative prior artifact changed: '+p)
    core = read(primary/'implementation_sources/final/manifest.json')['files']
    expected.update({str(ROOT/p): h for p, h in core.items()})
    current = {p: digest(p) if Path(p).is_file() else None for p in expected}
    mismatches = [p for p in expected if current[p] != expected[p]]
    provenance = read(primary/'mpc_output/provenance.json')
    external = Path(provenance['lightnav_checkout'])
    external_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=external, text=True).strip()
    external_status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=external, text=True).strip()
    if external_sha != provenance['lightnav_sha'] or external_status:
        mismatches.append('official external checkout changed')
    users = {p: digest(ROOT/p) for p in previous['user_config_hashes']}
    return dict(valid=not mismatches, mismatches=mismatches, files=current,
                user_config_hashes=users, external_git_sha=external_sha,
                external_git_status=external_status, captured_utc=utc())


def snapshot(run, phase):
    names = list((ROOT/'src/reconciliation').glob('gp_se2_ref01*.py'))
    names += list((ROOT/'scripts').glob('*gp_se2_ref01*.py'))
    names += list((ROOT/'scripts/lightnav').glob('*gp_se2_ref01*.py'))
    names += list((ROOT/'tests').glob('test_gp_se2_ref01*.py'))
    names += [ROOT/'configs/gp_se2_ref01.yaml']
    folder = run/'implementation_sources'/phase
    folder.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for path in sorted(names):
        name = str(path.relative_to(ROOT))
        if phase == 'experiment' and name in REPORTING_ONLY:
            continue
        copy_new(path, folder/name)
        hashes[name] = digest(path)
    write(folder/'manifest.json', dict(files=hashes, reporting_only_excluded=sorted(REPORTING_ONLY)))
    return hashes


def prepare(run, config_path):
    from reconciliation.gp_se2_ref01_reference import build_variants
    from reconciliation.gp_se2_rollout import load_frozen_context
    config = validate_config(yaml.safe_load(config_path.read_text()))
    primary, original = [ROOT/config[k] for k in ('primary_source', 'reference_source')]
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_ref_01'):
        raise ValueError('a NEW run in data/robotless_gp_se2_ref_01 is required')
    if subprocess.run(['git', 'merge-base', '--is-ancestor', STARTING_SHA, 'HEAD'], cwd=ROOT).returncode:
        raise ValueError('reviewed source revision not present')
    for p in (primary/'validation.json', original/'validation.json'):
        if not read(p)['valid']:
            raise ValueError('original authoritative validation must pass')
    prior_source, mpc = read(primary/'source.json'), read(primary/'mpc_output/provenance.json')
    if Path(prior_source['primary_run']).resolve() != original.resolve():
        raise ValueError('reference source differs from provenance')
    if digest(primary/'config_snapshot.yaml') != digest(original/'config_snapshot.yaml'):
        raise ValueError('original configuration mismatch')
    before = preservation(primary)
    if not before['valid']:
        raise ValueError('source preservation mismatch')
    run.mkdir(parents=True)
    copy_new(config_path, run/'experiment_config.yaml')
    copy_new(original/'config_snapshot.yaml', run/'config_snapshot.yaml')
    write(run/'preservation_before.json', before)
    settings = mpc['official_settings']
    if any(settings[k] != v for k, v in config['mpc'].items()):
        raise ValueError('recorded official settings mismatch')
    originals = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    if originals['rollout']['horizon_s'] != 3. or originals['formulation']['goal_position_tolerance'] != .15 or originals['evaluation']['terminal_goal_dwell_s'] != .2:
        raise ValueError('original task acceptance mismatch')
    copied, cases, audits, requests = {}, [], [], []
    lookup = {r['case_id']: r for r in read(primary/'case_manifest.json')['selected']}
    begin = time.perf_counter()
    for case_id, role in CASES:
        row = lookup[case_id]
        src = primary/'cases'/row['case_directory']; dst = run/'cases'/row['case_directory']
        dst.mkdir(parents=True)
        for name in ('input_context.json', 'goal_route.json', 'F_native.npy', 'F_common.npy', 'reference_preparation.json', 'source_metrics.json'):
            if digest(src/name) != digest(original/'cases'/row['case_directory']/name):
                raise ValueError('original case source mismatch: '+name)
            copy_new(src/name, dst/name)
            copied[str((src/name).resolve())] = digest(src/name)
        context = read(dst/'input_context.json')
        fresh_context = load_frozen_context(prior_source['source_run'], context['episode_id'], context['handoff_id'])
        if fresh_context != context:
            raise ValueError('raw context does not reproduce stored case')
        source_root = Path(prior_source['source_run'])
        raw_context = read(source_root/'episodes'/context['episode_id']/'handoffs'/context['handoff_id']/'context.json')
        for kind in ('old', 'fresh'):
            for frame in ('raw_local', 'world'):
                record = raw_context[f'{kind}_{frame}_ref']
                source_path = source_root/'episodes'/context['episode_id']/record['path']
                if digest(source_path) != record['sha256']:
                    raise ValueError('raw array mismatch')
                copy_new(source_path, dst/'input_snapshot'/f'{kind}_{frame}.npy')
                copied[str(source_path)] = digest(source_path)
        historical = {}
        for method in ('M0_NATIVE', 'M0_ADAPTER'):
            for name, path in [('rollout.json', src/'methods'/method/'rollout/rollout.json'),
                               ('metrics.json', src/'methods'/method/'metrics.json')]:
                copy_new(path, dst/'historical'/method/name)
                copied[str(path.resolve())] = digest(path)
            historical[method] = str((dst/'historical'/method/'rollout.json').resolve())
        native, common = [np.load(dst/name, allow_pickle=False) for name in ('F_native.npy', 'F_common.npy')]
        result = build_variants(native, context['B_world'], settings['Q_WEIGHTS'],
            expected_common=common, preparation_metadata=read(dst/'reference_preparation.json'))
        write(dst/'reference_factorization.json', {k: v for k, v in result.items() if k != 'variants'})
        variants = {}
        for name in VARIANTS:
            value = result['variants'][name]; folder = dst/'variants'/name; folder.mkdir(parents=True)
            array(folder/'reference_world.npy', value['reference_world'])
            write(folder/'row_provenance.json', value['row_provenance'])
            write(folder/'geometry_audit.json', value['geometry_audit'])
            write(folder/'definition.json', {k: v for k, v in value.items()
                if k not in ('reference_world', 'row_provenance', 'geometry_audit')})
            hashes = {p.name: digest(p) for p in folder.iterdir() if p.is_file()}
            write(folder/'source.json', dict(case_id=case_id, variant=name, native_sha256=digest(dst/'F_native.npy'),
                preparation_sha256=digest(dst/'reference_preparation.json'), derived_sha256=hashes,
                frame=context['frame'], original_capture_pose=context['original_capture_pose_world'],
                source_timestamps=context['source_timestamps'], raw_modified=False, reanchored=False))
            variants[name] = dict(reference_path=str((folder/'reference_world.npy').resolve()),
                                  lineage_path=str((folder/'row_provenance.json').resolve()))
            audits.append(dict(case_id=case_id, case_role=role, variant=name,
                row_count=len(value['reference_world']), geometry=value['geometry_audit']))
        cases.append(dict(case_id=case_id, case_role=role, case_directory=row['case_directory'],
                          original_manifest_row=row, factorization=result['suffix_selection']))
        requests.append(dict(episode_id=context['episode_id'], handoff_id=context['handoff_id'],
            input_context_path=str((dst/'input_context.json').resolve()), goal_route_path=str((dst/'goal_route.json').resolve()),
            historical_native_rollout_path=historical['M0_NATIVE'], variants=variants))
    write(run/'case_manifest.json', dict(selected=cases, selected_count=2, result_based_selection=False))
    (run/'aggregate').mkdir()
    table(run/'aggregate/input_audit.csv', audits)
    write(run/'source.json', dict(experiment='GP-SE2-REF-01', created_utc=utc(), starting_git_sha=STARTING_SHA,
        preparation_git_sha=revision(), primary_source=str(primary.resolve()), reference_source=str(original.resolve()),
        source_run=prior_source['source_run'], environment_path=prior_source['environment_path'],
        environment_export_source=prior_source['environment_export_source'],
        environment_file_hashes=prior_source['original_environment_file_hashes'],
        original_config_sha256=digest(run/'config_snapshot.yaml'), experiment_config_sha256=digest(run/'experiment_config.yaml'),
        authoritative_validations={str(p.resolve()): digest(p) for p in (primary/'validation.json', original/'validation.json')},
        copied_source_sha256=copied, preserved_core_sha256=read(primary/'implementation_sources/final/manifest.json')['files'],
        original_MPC_environment=prior_source['original_MPC_environment'], official_mpc=mpc,
        intrinsic_lightnav_waypoint_dt_s=None, frame=prior_source['frame'], new_inference=False, gp_or_rigid_optimization=False,
        preparation_wall_s=time.perf_counter()-begin))
    write(run/'protocol.json', dict(experiment='GP-SE2-REF-01', frozen_before_primary=True,
        configuration=config, execution_order=[dict(case_id=cid, variant=v) for cid, _ in CASES for v in VARIANTS],
        matched_state_probe_count=240, primary_rollout_count=8, primary_mpc_solve_count=240, historical_solve_count=0,
        matched_states_source='all 30 saved GP-SE2-02 M0_NATIVE control-solve input poses per case',
        probe_policy='official build_pose_aligned_reference plus independent selection_audit; no state integration or MPC solve',
        raw_source_unchanged=True, no_reference_reanchoring=True, no_new_data=True, no_GP_optimization=True,
        settings=mpc['official_settings'], settings_sha256=mpc['official_settings_sha256'],
        physical_acceptance_config_sha256=digest(run/'config_snapshot.yaml'),
        optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY'))
    write(run/'mpc_request.json', dict(source_root=prior_source['source_run'],
        expected_official_settings_sha256=mpc['official_settings_sha256'], cases=requests,
        reproduction_tolerances=config['reproduction'], **config['rollout']))
    print(json.dumps(dict(phase='PREPARED', cases=2, variants=8)), flush=True)


def verify(run):
    source = read(run/'source.json')
    config = validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()))
    if digest(run/'config_snapshot.yaml') != source['original_config_sha256'] or digest(run/'experiment_config.yaml') != source['experiment_config_sha256']:
        raise ValueError('frozen configuration drift')
    for name, value in source['copied_source_sha256'].items():
        if digest(name) != value:
            raise ValueError('original input drift: '+name)
    for name, value in source['preserved_core_sha256'].items():
        if digest(ROOT/name) != value:
            raise ValueError('original source changed: '+name)
    if (run/'experiment_freeze.json').exists():
        frozen = read(run/'experiment_freeze.json')
        for name, value in frozen['source_sha256'].items():
            if digest(ROOT/name) != value:
                raise ValueError('experiment code changed after freeze: '+name)
        for name, value in frozen['input_sha256'].items():
            if digest(run/name) != value:
                raise ValueError('experiment input changed after freeze: '+name)
    return config, source


def freeze(run):
    verify(run)
    hashes = snapshot(run, 'experiment')
    inputs = {str(p.relative_to(run)): digest(p) for p in sorted(run.rglob('*')) if p.is_file()
              and not p.is_relative_to(run/'implementation_sources') and not p.name.endswith('.log')}
    write(run/'experiment_freeze.json', dict(frozen_utc=utc(), experiment_git_sha=revision(), source_sha256=hashes,
                                           input_sha256=inputs, no_primary_execution_yet=True))


def execute(run):
    _, source = verify(run)
    if not (run/'experiment_freeze.json').exists():
        raise ValueError('freeze code and inputs before official selector probes and rollouts')
    if (run/'execution_started.json').exists() or (run/'mpc_output').exists():
        raise FileExistsError('primary already started; no hidden retry')
    command = [str(Path(source['original_MPC_environment'])/'bin/python'),
        str(ROOT/'scripts/lightnav/gp_se2_ref01_mpc.py'), '--request', str(run/'mpc_request.json'),
        '--output', str(run/'mpc_output'), '--lightnav-checkout', source['official_mpc']['lightnav_checkout']]
    write(run/'execution_started.json', dict(utc=utc(), command=command, request_sha256=digest(run/'mpc_request.json')))
    begin = time.perf_counter()
    with (run/'mpc_worker.log').open('x') as stream:
        result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    write(run/'execution_completed.json', dict(utc=utc(), exit_code=result.returncode,
        worker_total_wall_s=time.perf_counter()-begin, command=command))
    if result.returncode:
        raise RuntimeError('official worker failed; preserve this run and use a new ID after any fix')
    print(json.dumps(dict(phase='EXECUTED', worker_total_wall_s=time.perf_counter()-begin)), flush=True)


FACTOR_PAIRS = [('R00_NATIVE', 'R10_SUFFIX_ONLY'), ('R00_NATIVE', 'R01_RESAMPLE_ONLY'),
                ('R01_RESAMPLE_ONLY', 'R11_CURRENT_ADAPTER'), ('R10_SUFFIX_ONLY', 'R11_CURRENT_ADAPTER')]


def selection_pairwise(case_id, records, *, matched):
    """Measured reference/progress differences; local row numbers are not comparable."""
    from reconciliation.se2 import wrap_angle
    result = []
    for base, other in FACTOR_PAIRS:
        for i in range(30):
            if matched:
                a, b = records[i]['variants'][base], records[i]['variants'][other]
                at = records[i]['time_s']
                command_difference = pose_difference = None
            else:
                first, second = records[base][i], records[other][i]
                a, b = first['selection_diagnostic'], second['selection_diagnostic']
                at = first['time_s']
                command_difference = (np.asarray(second['command'])-first['command']).tolist()
                pose_difference = np.asarray(second['input_pose_world'])-first['input_pose_world']
                pose_difference[2] = wrap_angle(pose_difference[2])
            x, y = np.asarray(a['reference_world']), np.asarray(b['reference_world'])
            delta = y-x; delta[:, 2] = wrap_angle(delta[:, 2])
            progress_delta = np.asarray(b['selected_original_fractional_row_coordinates'])-a['selected_original_fractional_row_coordinates']
            result.append(dict(case_id=case_id, baseline=base, variant=other, time_s=at,
                same_input_pose=matched, max_selected_xy_difference_m=float(np.max(np.linalg.norm(delta[:, :2], axis=1))),
                max_selected_yaw_difference_rad=float(np.max(np.abs(delta[:, 2]))),
                selected_original_progress_difference=progress_delta.tolist(),
                selection_changed=bool(np.max(np.abs(delta)) > 1e-12 or np.max(np.abs(progress_delta)) > 1e-12),
                baseline_goal_in_horizon=a['final_goal_row_in_horizon'], variant_goal_in_horizon=b['final_goal_row_in_horizon'],
                command_difference=command_difference, input_pose_difference=pose_difference,
                interpretation='matched-state input effect' if matched else 'closed-loop difference includes feedback state divergence'))
    return clean(result)


def diagnosis(rows, contrasts, reproductions, matched_pairs, config):
    """Predeclared descriptive flags; successful acceptance is never redefined."""
    result = {}
    for case_id, role in CASES:
        case_rows = [r for r in rows if r['case_id'] == case_id]
        current = {r['variant']: bool(r['primary_success']) for r in case_rows}
        changes = [r for r in contrasts if r['case_id'] == case_id]
        def observed(names):
            return any(r[name] is not None and abs(r[name]) > config['descriptive_effect_floor']
                       for r in changes for name in names)
        same_states = [r for r in matched_pairs if r['case_id'] == case_id]
        reproduction = reproductions[case_id]
        reproduced = all(r['passed'] for r in reproduction['variants'].values())
        result[case_id] = dict(case_role=role, native_adapter_outcomes_reproduced=reproduced,
            success_by_variant=current,
            suffix_effect_observed=observed(['suffix_without_resampling', 'suffix_with_resampling']) or
                current['R10_SUFFIX_ONLY'] != current['R00_NATIVE'] or current['R11_CURRENT_ADAPTER'] != current['R01_RESAMPLE_ONLY'],
            resampling_effect_observed=observed(['resampling_without_suffix', 'resampling_with_suffix']) or
                current['R01_RESAMPLE_ONLY'] != current['R00_NATIVE'] or current['R11_CURRENT_ADAPTER'] != current['R10_SUFFIX_ONLY'],
            interaction_observed=observed(['interaction']),
            matched_state_selection_change_observed=any(r['selection_changed'] for r in same_states),
            matched_state_changed_pair_probes=sum(r['selection_changed'] for r in same_states),
            mechanism_localization_level='INPUT_SELECTION_COMMAND_OUTCOME_MEASURED' if reproduced else 'REPRODUCTION_NOT_CONFIRMED',
            interpretation='fixed-event descriptive observations; no population inference or navigation improvement flag')
    hard = result[CASES[0][0]]
    hard['native_adapter_regression_reproduced'] = bool(hard['native_adapter_outcomes_reproduced'] and
        hard['success_by_variant']['R00_NATIVE'] and not hard['success_by_variant']['R11_CURRENT_ADAPTER'] and
        reproductions[CASES[0][0]]['historical_success']['R00_NATIVE'] and
        not reproductions[CASES[0][0]]['historical_success']['R11_CURRENT_ADAPTER'])
    return dict(cases=result, native_adapter_regression_reproduced=hard['native_adapter_regression_reproduced'],
        suffix_effect_observed=any(r['suffix_effect_observed'] for r in result.values()),
        resampling_effect_observed=any(r['resampling_effect_observed'] for r in result.values()),
        interaction_observed=any(r['interaction_observed'] for r in result.values()),
        matched_state_selection_change_observed=any(r['matched_state_selection_change_observed'] for r in result.values()),
        mechanism_localization_level=hard['mechanism_localization_level'],
        effect_floor=config['descriptive_effect_floor'], effect_floor_meaning=config['effect_floor_meaning'])


def evaluate(run):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_ref01_evaluation import evaluate_variant, reproduction_check, factor_contrasts, selection_diagnostics
    config, source = verify(run)
    output = run/'mpc_output'
    for record in read(output/'output_hashes.json')['files']:
        path = (output/record['path']).resolve()
        if not path.is_relative_to(output) or digest(path) != record['sha256']:
            raise ValueError('official worker output integrity mismatch')
    if read(output/'provenance.json')['request_sha256'] != digest(run/'mpc_request.json'):
        raise ValueError('official worker request changed')
    start = time.perf_counter(); environment = HospitalEnvironment.load(source['environment_path'])
    environment_s = time.perf_counter()-start; begin = time.perf_counter()
    original = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    rows, outcomes, selection_rows, matched_rows, closed_rows, reproductions = [], [], [], [], [], {}
    for case in read(run/'case_manifest.json')['selected']:
        folder = run/'cases'/case['case_directory']; worker = output/case['case_directory']
        context, goal = read(folder/'input_context.json'), read(folder/'goal_route.json')
        copy_new(worker/'matched_state_probes.json', folder/'matched_state_probes.json')
        probes = read(folder/'matched_state_probes.json')
        reproduction = dict(case_id=case['case_id'], variants={}, historical_success={})
        actual = {}
        for variant in VARIANTS:
            target = folder/'variants'/variant
            for path in sorted((worker/variant).iterdir()):
                if path.is_file():
                    copy_new(path, target/path.name)
            rollout = read(target/'rollout.json')
            if rollout['candidate_source']['sha256'] != digest(target/'reference_world.npy'):
                raise ValueError('actual rollout reference hash differs from frozen array')
            reference = np.load(target/'reference_world.npy', allow_pickle=False)
            metrics = evaluate_variant(variant, rollout, context, reference, goal, environment, original)
            metrics['summary']['case_role'] = case['case_role']
            write(target/'metrics.json', metrics)
            rows.append(metrics['summary']); outcomes.append(metrics)
            actual[variant] = rollout['controller_reference_selections']
            for kind, records in [('matched_state', [dict(time_s=p['time_s'], selection_diagnostic=p['variants'][variant]) for p in probes['states']]),
                                  ('closed_loop', actual[variant])]:
                selection_rows.append(dict(case_id=case['case_id'], variant=variant, mode=kind,
                                           **selection_diagnostics(records)))
            if variant in ('R00_NATIVE', 'R11_CURRENT_ADAPTER'):
                name = 'M0_NATIVE' if variant == 'R00_NATIVE' else 'M0_ADAPTER'
                old = read(folder/'historical'/name/'rollout.json')
                old_metrics = read(folder/'historical'/name/'metrics.json')
                reproduction['variants'][variant] = reproduction_check(rollout, old, metrics, old_metrics)
                reproduction['historical_success'][variant] = old_metrics['primary_success']
        reproduction['passed'] = all(r['passed'] for r in reproduction['variants'].values())
        write(folder/'reproduction_check.json', reproduction)
        reproductions[case['case_id']] = reproduction
        matched_rows.extend(selection_pairwise(case['case_id'], probes['states'], matched=True))
        closed_rows.extend(selection_pairwise(case['case_id'], actual, matched=False))
    contrasts = factor_contrasts(rows)
    flags = diagnosis(rows, contrasts, reproductions, matched_rows, config)
    worker_summary = read(output/'summary.json')
    summary = dict(experiment='GP-SE2-REF-01', operational_status='GP_SE2_REF_01_COMPLETED' if all(r['passed'] for r in reproductions.values()) else 'GP_SE2_REF_01_COMPLETED_WITH_LIMITATIONS',
        diagnosis=flags, rollouts=8, primary_mpc_solves=worker_summary['primary_mpc_solves'],
        historical_or_other_mpc_solves=worker_summary['historical_or_other_mpc_solves'],
        matched_state_selector_calls=worker_summary['matched_state_selector_calls'],
        all_cases_and_variants_accounted=True, original_acceptance_unchanged=True,
        new_VLA_updates=0, GP_or_rigid_optimization_solves=0,
        optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY',
        timing=dict(preparation_wall_s=source['preparation_wall_s'],
            environment_loading_s=environment_s, evaluation_wall_s=time.perf_counter()-begin,
            worker_process_wall_s=read(run/'execution_completed.json')['worker_total_wall_s'],
            worker_details=worker_summary),
        status_requirement='completion requires authoritative validation.json to pass',
        scope='offline counterfactual diagnosis on two dependent development-corpus events; not population inference')
    write(run/'aggregate/outcomes.json', outcomes)
    write(run/'aggregate/factor_contrasts.json', contrasts)
    write(run/'aggregate/selection_diagnostics.json', selection_rows)
    write(run/'aggregate/matched_state_pairwise.json', matched_rows)
    write(run/'aggregate/closed_loop_pairwise.json', closed_rows)
    write(run/'aggregate/summary.json', summary)
    for filename, values in [('rollout_outcomes', rows), ('factor_contrasts', contrasts),
                             ('selection_diagnostics', selection_rows), ('matched_state_pairwise', matched_rows),
                             ('closed_loop_pairwise', closed_rows)]:
        table(run/'aggregate'/f'{filename}.csv', values)
    write(run/'evaluation_completed.json', dict(utc=utc(), variants=8, reproduction_pass=all(r['passed'] for r in reproductions.values())))
    print(json.dumps(summary, indent=2), flush=True)


def finalize(run):
    _, source = verify(run)
    before, after = read(run/'preservation_before.json'), preservation(source['primary_source'])
    for key in ('files', 'user_config_hashes', 'external_git_sha', 'external_git_status'):
        if before[key] != after[key]:
            raise ValueError('preserved source inventory changed: '+key)
    if not after['valid']:
        raise ValueError('source integrity failed')
    write(run/'preservation_after.json', after)
    write(run/'implementation_final.json', dict(utc=utc(), source_sha256=snapshot(run, 'final')))
    paths = {str(p.relative_to(run)): digest(p) for p in sorted(run.rglob('*')) if p.is_file()
             and p.name not in ('validation.json', 'artifact_manifest.json', 'git_completion.json')}
    write(run/'artifact_manifest.json', dict(files=paths, created_utc=utc()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['prepare', 'freeze', 'execute', 'evaluate', 'finalize'])
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--config', default=ROOT/'configs/gp_se2_ref01.yaml', type=Path)
    args = parser.parse_args(); run = args.run.resolve()
    try:
        if args.phase == 'prepare':
            prepare(run, args.config.resolve())
        else:
            globals()[args.phase](run)
    except Exception as error:
        if run.exists() and not (run/f'technical_failure_{args.phase}.json').exists():
            write(run/f'technical_failure_{args.phase}.json', dict(utc=utc(), error=f'{type(error).__name__}: {error}',
                retry_in_same_run_forbidden=True, original_results_preserved=True))
        raise


if __name__ == '__main__':
    main()
