#!/usr/bin/env python3
"""Frozen four-case GP transfer and official-MPC counterfactual orchestration.

Every phase writes new files. A technical failure is evidence and requires a new
run ID if implementation changes; it is never an unrecorded scientific retry.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'
os.environ['JAX_PLATFORMS'] = 'cpu'
os.environ['JAX_ENABLE_X64'] = 'true'
os.environ.setdefault('XLA_FLAGS', '--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_01 import digest, read, write, array, table, clean
from reconciliation.gp_se2_02_protocol import (
    CASES, METHODS, GP_METHODS, INITIALIZATIONS, RecordingDerivatives,
    execution_order, select_retained_start, validate_config, classify_derivative_error,
)
from reconciliation.gp_se2_diagnostics import load_frozen_case


def utc():
    return datetime.now(timezone.utc).isoformat()


def repository_revision():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()


def preservation(config):
    from run_gp_se2_diag_01 import preservation_snapshot
    previous = ROOT/config['previous_diag01']
    state = preservation_snapshot(yaml.safe_load((previous/'config_snapshot.yaml').read_text()))
    for key in ('previous_diag01', 'previous_diag02'):
        folder = ROOT/config[key]
        state[key+'_files'] = {str(p.relative_to(ROOT)): digest(p)
                              for p in sorted(folder.rglob('*')) if p.is_file()}
    return state


def preserved_code(config):
    prior = ROOT/config['previous_diag02']
    source = read(prior/'source.json')
    expected = dict(source['preserved_core_sha256'])
    final = read(prior/'implementation_sources/final/manifest.json')['files']
    for name in ('gp_se2_diag02_ad.py', 'gp_se2_diag02_derivatives.py',
                 'gp_se2_diag02_environment.py', 'gp_se2_diag02_solver.py'):
        path = 'src/reconciliation/'+name
        expected[path] = final[path]
    for path in ('src/reconciliation/online_mpc_adapter.py',
                 'src/reconciliation/robotless_online.py',
                 'scripts/lightnav/gp_se2_mpc_rollout.py'):
        expected[path] = digest(ROOT/path)
    for path, value in expected.items():
        if digest(ROOT/path) != value:
            raise ValueError('immutable numerical source mismatch: '+path)
    return expected


def source_snapshot(run, phase):
    paths = set((ROOT/'src/reconciliation').glob('gp_se2*.py'))
    paths.update((ROOT/'scripts').glob('*gp_se2*'))
    paths.update((ROOT/'scripts/isaac').glob('*gp_se2*'))
    paths.update((ROOT/'scripts/lightnav').glob('*gp_se2*'))
    paths.update((ROOT/'tests').glob('test_gp_se2*'))
    paths.update(ROOT/p for p in ('src/reconciliation/se2.py',
        'src/reconciliation/online_mpc_adapter.py', 'src/reconciliation/robotless_online.py',
        'configs/gp_se2_02.yaml', 'pyproject.toml'))
    destination = run/'implementation_sources'/phase
    destination.mkdir(parents=True, exist_ok=False)
    hashes = {}
    reporting_only = {
        'scripts/validate_gp_se2_02.py',
        'src/reconciliation/gp_se2_02_validation.py',
        'src/reconciliation/gp_se2_02_transfer_validation.py',
        'tests/test_gp_se2_02_artifacts.py',
        'tests/test_gp_se2_02_transfer_artifacts.py',
    }
    for path in sorted(paths):
        if not path.is_file():
            continue
        relative = path.relative_to(ROOT)
        if phase == 'experiment' and str(relative) in reporting_only:
            continue
        target = destination/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        hashes[str(relative)] = digest(path)
    write(destination/'manifest.json', dict(created_utc=utc(), files=hashes,
        reporting_only_excluded=sorted(reporting_only) if phase=='experiment' else []))
    return hashes


def prepare(run, config_path):
    config = yaml.safe_load(config_path.read_text())
    validate_config(config)
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_02'):
        raise ValueError('a NEW run below data/robotless_gp_se2_02 is required')
    if subprocess.run(['git', 'merge-base', '--is-ancestor', config['starting_git_sha'], 'HEAD'],
                      cwd=ROOT, check=False).returncode:
        raise ValueError('reviewed revision must be an ancestor')
    primary = (ROOT/config['primary_run']).resolve()
    prior01, prior02 = [ROOT/config[k] for k in ('previous_diag01', 'previous_diag02')]
    authoritative = [primary/'validation.json',
        ROOT/'data/robotless_gp_se2_01/verification_20260918T060500Z/validation.json',
        prior01/'verification/validation.json', prior02/'validation.json',
        prior02/'derivative_checks/authoritative_validation.json']
    for path in authoritative:
        if not read(path)['valid']:
            raise ValueError('authoritative source validation failed: '+str(path))
    core = preserved_code(config)
    before = preservation(config)
    if not before['valid']:
        raise ValueError('retained source data integrity failed')
    original = read(primary/'source.json')
    environment = primary/'environment'
    export = Path(original['environment_export_source'])
    for record in original['environment_file_hashes']:
        if digest(environment/record['path']) != record['sha256']:
            raise ValueError('original environment changed')
        if record['path'] != 'geometry/render_geometry.json' and digest(export/record['path']) != record['sha256']:
            raise ValueError('validated export and original copy differ')
    run.mkdir(parents=True)
    shutil.copy2(primary/'config_snapshot.yaml', run/'config_snapshot.yaml')
    shutil.copy2(config_path, run/'experiment_config.yaml')
    write(run/'preservation_before.json', before)
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    from reconciliation.gp_se2_diag_acceptance import check_full_candidate
    start = time.perf_counter()
    env = HospitalEnvironment.load(environment)
    environment_seconds = time.perf_counter()-start
    rows, inputs = [], {}
    for case_id, role in CASES:
        case = load_frozen_case(primary, case_id, environment=env)
        row = dict(case['manifest_row'], case_role=role,
                   scope_status='UNSUPPORTED_GATE_CASE' if case['goal_route']['gates'] else 'NO_REQUIRED_GATE',
                   derivative_applicability='PENDING_FROZEN_TRANSFER_SMOKE')
        target = run/'cases'/row['case_directory']
        target.mkdir(parents=True)
        for name in ('input_context.json', 'goal_route.json', 'F_native.npy', 'F_common.npy',
                     'reference_preparation.json', 'source_metrics.json'):
            path = case['case_directory']/name
            shutil.copy2(path, target/name)
            inputs[str(path)] = digest(path)
            assert digest(target/name) == digest(path)
        row['input_sha256'] = dict(case['hashes'])
        row['copied_input_sha256'] = {p.name:digest(p) for p in sorted(target.iterdir()) if p.is_file()}
        folder = target/'initializations'
        folder.mkdir()
        p = case['problem']
        seeds, seed_costs = [], {}
        for name, constructor in [('I0_FRESH', lambda:p.initializations()[0]),
                                  ('I1_DECEL', lambda:same_curvature_deceleration_seed(p))]:
            seed_begin = time.perf_counter()
            seeds.append(constructor())
            seed_costs[name] = time.perf_counter()-seed_begin
        vectors = {}
        for name, seed in zip(INITIALIZATIONS, seeds):
            z = p.vector(seed['poses'], seed['twists'])
            array(folder/(name+'.npy'), z)
            pp, vv = p.unpack(z)
            assert np.array_equal(pp[0], p.boundary_pose) and np.array_equal(vv[0], p.initial_twist)
            write(folder/(name+'.json'), dict(name=name, original_name=seed['name'],
                construction=seed.get('metadata'), poses=seed['poses'], twists=seed['twists'],
                construction_wall_s=seed_costs[name],
                chart_reconstructed_poses=pp, chart_reconstructed_twists=vv,
                max_position_roundtrip_error_m=float(np.max(np.abs(pp[:, :2]-seed['poses'][:, :2]))),
                objective=p.evaluate(z)['objective'], vector_sha256=digest(folder/(name+'.npy'))))
            vectors[name] = z
        check_begin = time.perf_counter()
        row['initial_feasibility'] = {}
        for method in GP_METHODS:
            other = load_frozen_case(primary, case_id, method, environment=env)
            expected = [other['problem'].initializations()[0], same_curvature_deceleration_seed(other['problem'])]
            for name, seed in zip(INITIALIZATIONS, expected):
                assert np.array_equal(vectors[name], other['problem'].vector(seed['poses'], seed['twists']))
                full = check_full_candidate(other['problem'], vectors[name], other)
                write(folder/f'initial_full_{method}_{name}.json', full)
                row['initial_feasibility'][method+'/'+name] = bool(full['full_feasible'])
        row['seed_sha256'] = {name: digest(folder/(name+'.npy')) for name in INITIALIZATIONS}
        row['initialization_artifact_sha256'] = {p.name:digest(p) for p in sorted(folder.iterdir()) if p.is_file()}
        row['preparation_timing'] = dict(seed_construction_s=sum(seed_costs.values()),
                                        per_seed_construction_s=seed_costs,
                                        initial_full_checks_s=time.perf_counter()-check_begin)
        rows.append(row)
    write(run/'case_manifest_inputs.json', dict(experiment='GP-SE2-02', selected=rows,
        selected_count=4, requested_case_ids=[c[0] for c in CASES], result_based_selection=False))
    write(run/'source.json', dict(experiment='GP-SE2-02', created_utc=utc(),
        starting_git_sha=config['starting_git_sha'], preparation_git_sha=repository_revision(),
        primary_run=str(primary), previous_diag01=str(prior01), previous_diag02=str(prior02),
        source_run=original['source_run'], environment_path=str(environment),
        environment_export_source=str(export), environment_export_reused=True,
        original_environment_file_hashes=original['environment_file_hashes'],
        authoritative_validations={str(p): dict(sha256=digest(p), check_count=read(p).get('check_count')) for p in authoritative},
        preserved_core_sha256=core, input_sha256=inputs,
        original_config_sha256=digest(primary/'config_snapshot.yaml'),
        experiment_config_sha256=digest(run/'experiment_config.yaml'),
        fixed_manifest_inputs_sha256=digest(run/'case_manifest_inputs.json'),
        intrinsic_lightnav_waypoint_dt_s=None, frame=case['config']['frame'],
        shared_preparation_environment_loading_s=environment_seconds,
        original_MPC_environment='/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv',
        no_new_inference=True, no_synthetic_replacement=True))
    print(json.dumps(dict(phase='PREPARED', run=str(run), cases=4,
                         no_gate_cases=sum(r['scope_status']=='NO_REQUIRED_GATE' for r in rows))), flush=True)


def settings(run):
    config = yaml.safe_load((run/'experiment_config.yaml').read_text())
    validate_config(config)
    source = read(run/'source.json')
    if digest(run/'config_snapshot.yaml') != source['original_config_sha256']:
        raise ValueError('original configuration changed')
    if digest(run/'experiment_config.yaml') != source['experiment_config_sha256']:
        raise ValueError('experiment configuration changed')
    if digest(run/'case_manifest_inputs.json') != source['fixed_manifest_inputs_sha256']:
        raise ValueError('fixed manifest inputs changed')
    for row in read(run/'case_manifest_inputs.json')['selected']:
        folder = run/'cases'/row['case_directory']
        for name, expected in row['copied_input_sha256'].items():
            if digest(folder/name) != expected:
                raise ValueError('copied original input changed: '+name)
        for name, expected in row['initialization_artifact_sha256'].items():
            if digest(folder/'initializations'/name) != expected:
                raise ValueError('frozen seed or seed-check changed: '+name)
    for path, expected in {**source['preserved_core_sha256'], **source['input_sha256']}.items():
        if digest(ROOT/path) != expected:
            raise ValueError('immutable source mismatch: '+path)
    return config, source


def smoke(run):
    from reconciliation.gp_se2_02_transfer import freeze_transfer, run_transfer
    _, source = settings(run)
    folder = run/'derivative_transfer_checks'
    freeze_transfer(folder, source['primary_run'], source['previous_diag02'])
    result = run_transfer(folder, source['primary_run'])
    print(json.dumps(dict(phase='TRANSFER_SMOKE', starts=result['start_lookup'])), flush=True)


def freeze(run):
    config, source = settings(run)
    transfer = read(run/'derivative_transfer_checks/summary.json')
    manifest = read(run/'case_manifest_inputs.json')
    for row in manifest['selected']:
        row['derivative_applicability'] = {method+'/'+name: transfer['start_lookup'][row['case_id']+'/'+method+'/'+name]
            for method in GP_METHODS for name in INITIALIZATIONS}
    write(run/'case_manifest.json', manifest)
    write(run/'protocol.json', dict(experiment='GP-SE2-02', frozen_utc=utc(),
        methods=list(METHODS), fixed_cases=[dict(case_id=c, case_role=r) for c, r in CASES],
        exact_execution_order=execution_order(), gp_planned_starts=16, rigid_planned_starts=8,
        sequential=True, retry=False, replace_cases=False, original_formulation=yaml.safe_load((run/'config_snapshot.yaml').read_text()),
        initializations=list(INITIALIZATIONS), selection=config['candidate_selection'],
        rollout_policy=config['rollout_policy'], no_terminal_stop_constraint_added=True,
        physical_command_and_controller_memory='distinct original values retained',
        GP_body_velocity_feedforward=False, observation_transform='original capture pose, never B reanchor',
        prepared_solve_budget_s=30., max_iterations=200, precision='float64', blas_threads=1,
        new_inference=False, new_environment_export=False, new_MPC_instances_per_candidate=True,
        simulation_schedule='original synchronous 10Hz solves, exact held commands at 60Hz, 3s horizon; no compute delay',
        offline_counterfactual_only=True, gui_label=config['gui_label']))
    import scipy, shapely, jax, jaxlib
    from reconciliation.gp_se2_diag_solver import blas_thread_state
    from scipy.optimize import _slsqp_py
    write(run/'runtime_versions.json', dict(python=sys.version, numpy=np.__version__, scipy=scipy.__version__,
        shapely=shapely.__version__, jax=jax.__version__, jaxlib=jaxlib.__version__,
        jax_backend=jax.default_backend(), jax_x64=bool(jax.config.jax_enable_x64),
        scipy_slsqp_source=str(Path(_slsqp_py.__file__)), scipy_slsqp_sha256=digest(_slsqp_py.__file__),
        blas=blas_thread_state(True)))
    code = source_snapshot(run, 'experiment')
    write(run/'experiment_freeze.json', dict(frozen_utc=utc(), experiment_git_sha=repository_revision(),
        code_sha256=code, protocol_sha256=digest(run/'protocol.json'),
        config_sha256=digest(run/'config_snapshot.yaml'), experiment_config_sha256=digest(run/'experiment_config.yaml'),
        case_manifest_sha256=digest(run/'case_manifest.json'),
        transfer_summary_sha256=digest(run/'derivative_transfer_checks/summary.json'), ready=True))
    print(json.dumps(dict(phase='FROZEN', implementation_sha=repository_revision(), gp_starts=16, rigid_starts=8)), flush=True)


def verify_frozen(run):
    config, source = settings(run)
    frozen = read(run/'experiment_freeze.json')
    for relative, expected in frozen['code_sha256'].items():
        if digest(ROOT/relative) != expected:
            raise ValueError('source changed after primary freeze: '+relative)
    for filename, key in [('protocol.json', 'protocol_sha256'), ('case_manifest.json', 'case_manifest_sha256'),
                          ('derivative_transfer_checks/summary.json', 'transfer_summary_sha256')]:
        if digest(run/filename) != frozen[key]:
            raise ValueError('frozen protocol/input changed: '+filename)
    return config, source


def unavailable_start(initialization, vector, status, detail):
    return dict(initialization=initialization, termination=status, solver_success=False,
        iterations=0, candidate_found=False, candidate_vector=None, candidate_objective=None,
        candidate_world=None, support_poses=None, support_twists=None, selected_iterate=None,
        constraint_report=None, initial_vector=vector, latest_iterate=None,
        callback_snapshots=[], candidate_checks=[], solve_wall_time_s=0.,
        post_solve_validation_time_s=0., setup_wall_time_s=0., solver_executed=False,
        unsupported_detail=detail, fallback_used=False)


def optimize(run):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider, DerivativeError
    from reconciliation.gp_se2_diag02_solver import run_instrumented
    from reconciliation.gp_se2_formulation import solve_rigid
    from reconciliation.gp_se2_02_evaluation import plan_for_method, make_mpc_request
    from run_gp_se2_diag_02 import postprocess
    config, source = verify_frozen(run)
    write(run/'optimization_started.json', dict(utc=utc(), exact_order=execution_order(), retries=False))
    begin = time.perf_counter()
    env = HospitalEnvironment.load(source['environment_path'])
    env_seconds = time.perf_counter()-begin
    transfer = read(run/'derivative_transfer_checks/summary.json')
    starts, request_entries, timings = [], [], []
    entire_start = time.perf_counter()
    for row in read(run/'case_manifest.json')['selected']:
        case_id = row['case_id']
        case = load_frozen_case(source['primary_run'], case_id, environment=env)
        folder = run/'cases'/row['case_directory']
        for method in METHODS:
            (folder/'methods'/method).mkdir(parents=True, exist_ok=False)
        # Unchanged original two rigid initializations/policy in one call.
        p = case['problem']; c = case['config']; target = folder/'methods/M1_RIGID'
        rigid_begin = time.perf_counter(); rigid_utc = utc(); rigid_mono = time.monotonic()
        rigid = solve_rigid(p.boundary_pose, p.initial_twist, p.common_reference, p.goal_pose, p.config,
            obstacle_clearance=p.obstacle_clearance, workspace_margin=p.workspace_margin, gates=p.gates)
        rigid['total_optimization_and_check_wall_s'] = time.perf_counter()-rigid_begin
        write(target/'execution_timing.json', dict(start_utc=rigid_utc, end_utc=utc(),
            start_monotonic_s=rigid_mono, end_monotonic_s=time.monotonic(),
            total_wall_s=rigid['total_optimization_and_check_wall_s'], original_two_start_call=True))
        write(target/'solver_result.json', rigid)
        for i, result in enumerate(rigid['attempts']):
            starts.append(dict(case_id=case_id, case_role=row['case_role'], method='M1_RIGID',
                initialization=result['initialization'], initialization_index=i,
                termination=result['termination'], solver_success=result['solver_success'],
                iterations=result['iterations'], solve_wall_time_s=result['wall_time_s'],
                solver_executed=True, candidate_selection='unchanged solve_rigid internal policy'))
        print(json.dumps(dict(case=case_id, method='M1_RIGID', status=rigid['status'])), flush=True)
        for method in GP_METHODS:
            target = folder/'methods'/method; results, summaries = [], []
            method_begin = time.perf_counter()
            for initialization in INITIALIZATIONS:
                out = target/'starts'/initialization; out.mkdir(parents=True)
                setup_begin = time.perf_counter()
                current = load_frozen_case(source['primary_run'], case_id, method, environment=env)
                problem = current['problem']; z = np.load(folder/'initializations'/f'{initialization}.npy', allow_pickle=False)
                setup_seconds = time.perf_counter()-setup_begin
                applicability = transfer['start_lookup'][case_id+'/'+method+'/'+initialization]
                provider = wrapper = None; graph = warm = 0.; error = None
                status = applicability if isinstance(applicability, str) else applicability['status']
                if status == 'VERIFIED':
                    graph_begin = time.perf_counter()
                    try:
                        provider = DerivativeProvider(problem, EnvironmentDerivatives(env, c['footprint']['radius_m']))
                        graph = time.perf_counter()-graph_begin
                        warm_begin = time.perf_counter(); provider.warmup(z); warm = time.perf_counter()-warm_begin
                        provider.reset_stats(clear_cache=True)
                        wrapper = RecordingDerivatives(provider)
                    except DerivativeError as exc:
                        if provider is None:
                            graph = time.perf_counter()-graph_begin
                        else:
                            warm = time.perf_counter()-warm_begin
                        status = classify_derivative_error(exc)
                        error = dict(reason_code=exc.reason_code, message=str(exc), diagnostics=exc.diagnostics)
                write(out/'setup.json', dict(case_id=case_id, method=method, initialization=initialization,
                    applicability=applicability, effective_applicability=status, error=error,
                    seed_sha256=digest(folder/'initializations'/f'{initialization}.npy'),
                    case_loading_seed_preparation_s=setup_seconds, derivative_graph_construction_s=graph,
                    compilation_first_call_warmup_s=warm, compiled_provider_reused=False,
                    compilation_reuse_scope='fresh provider per start; warmup excluded from prepared 30s budget'))
                solve_utc, solve_mono = utc(), time.monotonic()
                if status == 'VERIFIED':
                    result = run_instrumented(problem, z, initialization_name=initialization, derivative_provider=wrapper)
                    result['solver_executed'] = True
                    result['recorded_derivative_errors'] = wrapper.errors
                    result['effective_termination'] = wrapper.errors[-1]['classification'] if wrapper.errors else result['termination']
                    write(out/'solver_result.json', result)
                    summary = postprocess(current, result, out)
                    summary.update(effective_termination=result['effective_termination'], solver_executed=True)
                else:
                    result = unavailable_start(initialization, z, status, error or applicability)
                    write(out/'solver_result.json', result)
                    write(out/'full_acceptance.json', dict(full_feasible=False, status=status, metrics=None))
                    write(out/'post_full_checks.json', dict(rows=[], unique_full_checks=0,
                        full_validation_wall_time_s=0., selection_rule_unchanged=True, not_executed_reason=status))
                    initial_full = read(folder/'initializations'/f'initial_full_{method}_{initialization}.json')
                    summary = dict(termination=status, effective_termination=status, solver_success=False,
                        solver_executed=False, iterations=0, initial_full_feasible=initial_full['full_feasible'],
                        final_full_feasible=None, selected_full_feasible=False, selected_source=None,
                        initial_objective=float(problem.evaluate(z)['objective']), selected_objective=None,
                        returned_initial_unchanged=False, infeasible_start_recovered=False,
                        feasible_seed_objective_improved=False, solve_wall_time_s=0.,
                        post_solve_validation_time_s=0., independent_full_validation_time_s=0.)
                write(out/'execution_timing.json', dict(start_utc=solve_utc, end_utc=utc(),
                    start_monotonic_s=solve_mono, end_monotonic_s=time.monotonic(),
                    scope='prepared harness and post-dense/full checking; compilation/setup separate'))
                summary.update(case_id=case_id, case_role=row['case_role'], method=method, initialization=initialization,
                    initialization_index=INITIALIZATIONS.index(initialization),
                    case_loading_seed_preparation_s=setup_seconds, derivative_graph_construction_s=graph,
                    compilation_first_call_warmup_s=warm,
                    cold_setup_solve_validation_s=setup_seconds+graph+warm+result['setup_wall_time_s']+
                        result['solve_wall_time_s']+result['post_solve_validation_time_s']+summary['independent_full_validation_time_s'])
                write(out/'result_summary.json', summary)
                results.append(result); summaries.append(summary); starts.append(summary)
                print(json.dumps({k:summary.get(k) for k in ('case_id', 'method', 'initialization', 'effective_termination',
                    'selected_full_feasible', 'selected_objective', 'solve_wall_time_s')}), flush=True)
            selected = select_retained_start(results)
            chosen = None if selected is None else results[selected]
            output = dict(method=method, status='NO_FEASIBLE_CANDIDATE_FOUND' if chosen is None else 'CANDIDATE_FOUND',
                candidate_world=None if chosen is None else chosen['candidate_world'],
                candidate_vector=None if chosen is None else chosen['candidate_vector'],
                candidate_objective=None if chosen is None else chosen['candidate_objective'],
                support_times=problem.times, support_poses=None if chosen is None else chosen['support_poses'],
                support_twists=None if chosen is None else chosen['support_twists'],
                selected_initialization=selected, selected_iterate=None if chosen is None else chosen['selected_iterate'],
                constraint_report=None if chosen is None else chosen['constraint_report'], factor_costs=None,
                starts=[dict(initialization_id=name, relative_path='starts/'+name) for name in INITIALIZATIONS],
                attempts=summaries, fallback_used=False, selection_rule=config['candidate_selection'],
                selected_full_feasible=False if selected is None else summaries[selected]['selected_full_feasible'],
                total_optimization_and_check_wall_s=time.perf_counter()-method_begin,
                sum_all_start_cold_wall_s=sum(s['cold_setup_solve_validation_s'] for s in summaries),
                diagnostic_candidate_world=None if results[-1]['latest_iterate'] is None else
                    problem.unpack(results[-1]['latest_iterate'])[0][1:])
            write(target/'solver_result.json', output)
            full = dict(full_feasible=False, status='NO_CANDIDATE') if selected is None else read(
                target/'starts'/INITIALIZATIONS[selected]/'full_acceptance.json')
            write(target/'full_acceptance.json', full)
        # These explicit controls are always retained; invalid SEED_ONLY is not executed.
        for method, filename in [('M0_NATIVE', 'F_native.npy'), ('M0_ADAPTER', 'F_common.npy')]:
            candidate = np.load(folder/filename, allow_pickle=False)
            write(folder/'methods'/method/'solver_result.json', dict(method=method, status='REFERENCE_AVAILABLE',
                candidate_world=candidate, attempts=[], support_poses=None, support_twists=None,
                total_optimization_and_check_wall_s=0., optimization_performed=False, reference_source_sha256=digest(folder/filename)))
        z = np.load(folder/'initializations/I1_DECEL.npy', allow_pickle=False)
        pp, vv = case['problem'].unpack(z)
        seed_full = read(folder/'initializations/initial_full_M3_GP_CONSTRAINED_I1_DECEL.json')
        write(folder/'methods/SEED_ONLY/solver_result.json', dict(method='SEED_ONLY', status='SEED_GENERATED',
            candidate_world=pp[1:], candidate_vector=z, support_times=p.times, support_poses=pp, support_twists=vv,
            candidate_objective=float(p.evaluate(z)['objective']), attempts=[],
            total_optimization_and_check_wall_s=0., optimization_performed=False, fallback_used=False,
            seed_source_sha256=digest(folder/'initializations/I1_DECEL.npy')))
        write(folder/'methods/SEED_ONLY/full_acceptance.json', seed_full)
        entry = dict(context=case['context'], goal_route=case['goal_route'], methods={})
        for method in METHODS:
            target = folder/'methods'/method; result = read(target/'solver_result.json')
            candidate = result['candidate_world']
            if candidate is not None:
                array(target/'candidate_world.npy', candidate)
            full = read(target/'full_acceptance.json') if (target/'full_acceptance.json').exists() else None
            plan_begin = time.perf_counter()
            plan = plan_for_method(case, method, candidate, solver_result=result, full_acceptance=full)
            write(target/'plan_validation.json', plan)
            timings.append(dict(case_id=case_id, method=method,
                optimizer_and_checks_wall_s=result['total_optimization_and_check_wall_s'],
                final_plan_validation_wall_s=time.perf_counter()-plan_begin))
            entry['methods'][method] = dict(candidate_path=str((target/'candidate_world.npy').resolve()) if candidate is not None else None,
                                             plan=plan, solver_result=result)
        request_entries.append(entry)
    write(run/'mpc_request.json', make_mpc_request(source['source_run'], request_entries, case['config']))
    write(run/'optimization_completed.json', dict(utc=utc(), starts=starts, method_timings=timings,
        planned_GP_starts=16, actual_GP_solver_calls=sum(s['solver_executed'] for s in starts if s['method'] in GP_METHODS),
        rigid_starts=8, exact_order=execution_order(), shared_environment_loading_s=env_seconds,
        offline_optimization_phase_wall_s=time.perf_counter()-entire_start, retries=False))
    (run/'aggregate').mkdir()
    table(run/'aggregate/all_starts.csv', [{k:v for k,v in s.items() if not isinstance(v,(dict,list))} for s in starts])


def mpc(run):
    _, source = verify_frozen(run)
    if not (run/'optimization_completed.json').exists():
        raise ValueError('all planned optimization starts must be accounted before MPC')
    output = run/'mpc_output'
    if output.exists() or (run/'mpc_started.json').exists():
        raise FileExistsError('MPC batch already started; no hidden retry')
    python = Path(source['original_MPC_environment'])/'bin/python'
    command = [str(python), str(ROOT/'scripts/lightnav/gp_se2_mpc_rollout.py'),
               '--request', str(run/'mpc_request.json'), '--output', str(output)]
    write(run/'mpc_started.json', dict(utc=utc(), command=command, request_sha256=digest(run/'mpc_request.json')))
    start = time.perf_counter()
    with (run/'mpc_worker.log').open('x') as stream:
        result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=False)
    write(run/'mpc_batch_timing.json', dict(utc=utc(), wall_seconds=time.perf_counter()-start,
        exit_code=result.returncode, command=command, compute_delay_added_to_simulation=False))
    if result.returncode:
        raise RuntimeError('official isolated MPC worker failed; partial evidence retained')
    print(json.dumps(dict(phase='MPC_COMPLETED', wall_s=time.perf_counter()-start)), flush=True)


def evaluate(run):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_02_evaluation import evaluate_method, comparisons
    _, source = verify_frozen(run)
    output = run/'mpc_output'
    for record in read(output/'output_hashes.json')['files']:
        path = (output/record['path']).resolve()
        if not path.is_relative_to(output) or digest(path) != record['sha256']:
            raise ValueError('isolated MPC output hash mismatch')
    if read(output/'provenance.json')['request_sha256'] != digest(run/'mpc_request.json'):
        raise ValueError('MPC request mismatch')
    begin = time.perf_counter(); env = HospitalEnvironment.load(source['environment_path'])
    environment_seconds = time.perf_counter()-begin; phase_begin = time.perf_counter(); rows = []
    for row in read(run/'case_manifest.json')['selected']:
        case = load_frozen_case(source['primary_run'], row['case_id'], environment=env)
        folder = run/'cases'/row['case_directory']; case_output = output/row['case_directory']
        historical = read(case_output/'historical_solve_audit.json')
        if not historical['passed']:
            raise ValueError('historical solve audit must use original solve state and pass')
        for method in METHODS:
            target = folder/'methods'/method
            solver = read(target/'solver_result.json'); plan = read(target/'plan_validation.json')
            rollout_path = case_output/method/'rollout.json'; rollout = None
            if rollout_path.exists():
                rollout = read(rollout_path)
                if rollout['candidate_source']['sha256'] != digest(target/'candidate_world.npy'):
                    raise ValueError('rollout candidate bytes mismatch')
                shutil.copytree(case_output/method, target/'rollout')
            timing = time.perf_counter()
            metrics = evaluate_method(case, method, solver['candidate_world'], plan, solver, rollout)
            metrics['independent_rollout_evaluation_wall_s'] = time.perf_counter()-timing
            write(target/'metrics.json', metrics)
            rows.append(dict(case_id=row['case_id'], case_role=row['case_role'], method=method,
                candidate_available=metrics['candidate_available'], plan_valid=metrics['plan_valid'],
                rollout_performed=metrics['rollout_performed'], rollout_success=metrics['rollout_success'],
                primary_success=metrics['rollout_success'], termination_reason=metrics.get('termination_reason'),
                failure_reasons=metrics['failure_reasons'], optimizer_wall_s=solver['total_optimization_and_check_wall_s'],
                deformation=plan.get('deformation'), rollout_metrics=metrics))
        write(folder/'validation.json', dict(method_count=6, all_methods_accounted_for=True,
            historical_solve_audit=historical, full_case_validation='pending independent artifact validator'))
    summary = comparisons(rows)
    summary.update(experiment='GP-SE2-02', actual_case_count=4,
        planned_GP_starts=16, actual_GP_solver_calls=read(run/'optimization_completed.json')['actual_GP_solver_calls'],
        actual_rollout_count=sum(r['rollout_performed'] for r in rows),
        operational_status='GP_SE2_02_RUNTIME_NOT_VALIDATED',
        no_new_inference=True, offline_counterfactual=True, online_latency_validated=False,
        shared_evaluation_environment_loading_s=environment_seconds,
        evaluation_phase_wall_s=time.perf_counter()-phase_begin)
    write(run/'aggregate/method_results.json', rows)
    write(run/'aggregate/summary.json', summary)
    table(run/'aggregate/primary_outcomes.csv', [{k:v for k,v in row.items() if k!='rollout_metrics'} for row in rows])
    table(run/'aggregate/candidate_acceptance.csv', [dict(case_id=r['case_id'],case_role=r['case_role'],method=r['method'],
        candidate_available=r['candidate_available'],plan_valid=r['plan_valid'],rollout_performed=r['rollout_performed']) for r in rows])
    table(run/'aggregate/regressions.csv', summary['regressions'])
    table(run/'aggregate/paired_metrics.csv', summary['paired_metrics'])
    timings = []
    for row in rows:
        execution = row['rollout_metrics'].get('execution') or {}
        timings.append(dict(case_id=row['case_id'],method=row['method'],
            total_optimizer_all_starts_wall_s=row['optimizer_wall_s'],
            rollout_evaluation_wall_s=row['rollout_metrics']['independent_rollout_evaluation_wall_s'],
            mpc_solve_count=execution.get('mpc_solve_count_total'),
            mpc_official_solve_total_s=execution.get('mpc_official_solve_total_s'),
            mpc_submit_wait_poll_total_s=execution.get('mpc_submit_wait_poll_total_s')))
    table(run/'aggregate/timing.csv', timings)
    write(run/'evaluation_completed.json', dict(utc=utc(), rows=24, actual_rollouts=summary['actual_rollout_count']))
    print(json.dumps(dict(phase='EVALUATED', actual_rollouts=summary['actual_rollout_count'])), flush=True)


def finalize(run):
    config, _ = settings(run)
    before = read(run/'preservation_before.json'); after = preservation(config)
    for key in ('historical_files', 'external_files', 'user_config_hashes', 'external_git_sha',
                'external_git_status', 'previous_diag01_files', 'previous_diag02_files'):
        if before[key] != after[key]:
            raise ValueError('preserved source changed: '+key)
    if not after['valid']:
        raise ValueError('historical preservation failed')
    write(run/'preservation_after.json', after)
    write(run/'implementation_final.json', dict(utc=utc(), source_sha256=source_snapshot(run, 'final')))
    files = {str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file()
             and p.name not in ('artifact_manifest.json', 'validation.json', 'git_completion.json')}
    write(run/'artifact_manifest.json', dict(utc=utc(), files=files))
    print(json.dumps(dict(phase='ARTIFACTS_FROZEN', files=len(files))), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['prepare','smoke','freeze','optimize','mpc','evaluate','finalize'])
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/gp_se2_02.yaml')
    args = parser.parse_args(); run = args.run.resolve()
    try:
        if args.phase == 'prepare':
            prepare(run, args.config.resolve())
        else:
            globals()[args.phase](run)
    except Exception as error:
        if run.exists() and not (run/f'technical_failure_{args.phase}.json').exists():
            write(run/f'technical_failure_{args.phase}.json', dict(utc=utc(), phase=args.phase,
                error=f'{type(error).__name__}: {error}', retry_in_same_run_forbidden=True,
                result_not_replaced=True, requires_new_run_after_implementation_change=True))
        raise


if __name__ == '__main__':
    main()
