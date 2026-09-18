#!/usr/bin/env python3
"""Read-only, independent saved-artifact validation for GP-SE2-DIAG-01.

No optimizer, inference, export, or MPC rollout is invoked. Original mathematical
and environment checks are reevaluated on saved vectors. A requested report is
created exclusively; historical inputs and existing reports are never rewritten.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import zipfile

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml

from reconciliation.gp_se2_diag_acceptance import check_full_candidate
from reconciliation.gp_se2_diag_fixtures import evaluate_fixture, evaluate_blind_spot, make_fixture_problem
from reconciliation.gp_se2_diagnostics import load_frozen_case, classify_saved_attempt
from reconciliation.gp_se2_formulation import _constraint_report


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def source_path_matches(saved_path, expected_path):
    """Runner provenance may use a repository-relative or absolute file path."""
    source=Path(saved_path)
    if not source.is_absolute():
        source=ROOT/source
    return source.resolve()==Path(expected_path).resolve()


def fixture_initialization_matches(fixture_name, seed_kind, saved_name):
    """Summary seed kind and attempt initialization name are distinct fields."""
    return seed_kind in ('known','perturbed') and saved_name==fixture_name+'_'+seed_kind


def equivalent(a, b, *, atol=1e-11, rtol=1e-9):
    """Numeric comparisons preserve null, booleans, shapes, and dict keys."""
    if isinstance(a, np.ndarray):
        a = a.tolist()
    if isinstance(b, np.ndarray):
        b = b.tolist()
    if a is None or b is None:
        return a is b
    if isinstance(a, (bool, np.bool_)) or isinstance(b, (bool, np.bool_)):
        return type(a) in (bool, np.bool_) and type(b) in (bool, np.bool_) and bool(a) == bool(b)
    if isinstance(a, dict) or isinstance(b, dict):
        return isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys() and all(
            equivalent(a[key], b[key], atol=atol, rtol=rtol) for key in a)
    if isinstance(a, (list, tuple)) or isinstance(b, (list, tuple)):
        if not isinstance(a, (list, tuple)) or not isinstance(b, (list, tuple)) or len(a) != len(b):
            return False
        try:
            aa, bb = np.asarray(a), np.asarray(b)
            if aa.dtype.kind in 'fiu' and bb.dtype.kind in 'fiu' and aa.shape == bb.shape:
                return bool(np.all(np.isfinite(aa)) and np.all(np.isfinite(bb)) and
                            np.allclose(aa, bb, atol=atol, rtol=rtol))
        except (ValueError, TypeError):
            pass
        return all(equivalent(x, y, atol=atol, rtol=rtol) for x, y in zip(a, b))
    if isinstance(a, (float, int, np.number)) and isinstance(b, (float, int, np.number)):
        return bool(np.isfinite(a) and np.isfinite(b) and np.isclose(a, b, atol=atol, rtol=rtol))
    return a == b


class Checks:
    def __init__(self):
        self.count = 0
        self.errors = []

    def check(self, condition, label):
        self.count += 1
        if not bool(condition):
            self.errors.append(label)

    def equal(self, a, b, label, *, exact=False):
        self.check(equivalent(a, b, atol=0. if exact else 1e-11,
                              rtol=0. if exact else 1e-9), label)

    def section(self, name, function):
        try:
            function()
        except Exception as exc:
            self.check(False, f'{name}: {type(exc).__name__}: {exc}')


def validate_attempt(problem, result, checks, label):
    """Recompute every saved callback and candidate; no solver is called."""
    checks.equal(result['config'], problem.config, label+': unchanged formulation', exact=True)
    checks.check(result['variable_count'] == 150 and len(problem.times) == 31 and
                 problem.times[-1] == 3., label+': real 3-second problem size')
    checks.check(result['jacobian_method'].startswith('SciPy numerical forward'), label+': original derivatives')
    checks.check(result['solver_backend'] == 'scipy.optimize.SLSQP', label+': original solver')
    checks.check(not result['fallback_used'] and not result['infeasibility_proven'], label+': no fallback/infeasibility claim')
    threads = result['blas_threads']
    checks.check(threads['single_thread_environment_valid'] and
                 threads['observed_runtime_single_thread_valid'], label+': single BLAS runtime')
    initial = np.asarray(result['initial_vector'], float)
    latest = np.asarray(result['latest_iterate'], float)
    checks.check(initial.shape == latest.shape == (150,), label+': vector dimensions')
    first = problem.evaluate(initial)
    checks.check(result['equality_count'] == len(first['equality']) and
                 result['inequality_count'] == len(first['inequality']), label+': constraint dimensions')
    actual_vectors = {'initial': initial, 'latest_iterate': latest}
    snapshots = result['callback_snapshots']
    checks.check(result['callback_count'] == len(snapshots), label+': callback count')
    checks.check([s['iteration'] for s in snapshots] == list(range(1, len(snapshots)+1)), label+': actual ordered callbacks')
    checks.check(all(b['elapsed_s'] >= a['elapsed_s'] for a, b in zip(snapshots, snapshots[1:])), label+': callback times monotonic')
    for saved in snapshots:
        where = label+f': callback {saved["iteration"]}'
        checks.check(saved['source'] == 'actual scipy SLSQP callback', where+': observed source')
        checks.check(np.asarray(saved['vector']).shape == (150,), where+': chart vector')
        if not saved['evaluation_complete']:
            checks.check(saved['objective'] is None and saved['collocation'] is None, where+': unavailable evaluation remains null')
            continue
        value = problem.evaluate(np.asarray(saved['vector']))
        report = _constraint_report(value['equality'], value['inequality'], problem.config)
        checks.equal(value['objective'], saved['objective'], where+': objective')
        checks.equal(value['equality'], saved['equality_residuals'], where+': equality')
        checks.equal(value['inequality'], saved['inequality_margins'], where+': inequality')
        checks.equal(report, saved['collocation'], where+': collocation report')
        if report['feasible']:
            actual_vectors[f'callback_{saved["iteration"]:04d}'] = np.asarray(saved['vector'])
    checks.check(result['completed_callback_evaluations'] == sum(s['evaluation_complete'] for s in snapshots), label+': completed callback count')
    candidate_checks = result['candidate_checks']
    checks.check([s['iterate'] for s in candidate_checks] == list(actual_vectors), label+': complete original candidate policy')
    choices = []
    for entry in candidate_checks:
        where = label+': '+entry['iterate']
        if entry['iterate'] not in actual_vectors:
            checks.check(False,where+': no actual saved iterate exists')
            continue
        vector = actual_vectors[entry['iterate']]
        checks.equal(entry['vector'], vector, where+': saved candidate vector', exact=True)
        value = problem.evaluate(vector)
        dense = problem.dense_report(vector)
        checks.equal(value['objective'], entry['objective'], where+': candidate objective')
        checks.equal(dense, entry['constraint_report'], where+': original dense report')
        if dense['feasible']:
            choices.append((float(value['objective']), entry['iterate'], vector))
    selected = min(choices, key=lambda x: (x[0], x[1])) if choices else None
    initial_feasible = bool(problem.dense_report(initial)['feasible'])
    checks.check(result['initial_feasible'] == initial_feasible, label+': initial feasibility')
    checks.check(result['candidate_found'] == bool(selected), label+': sampled candidate availability')
    checks.check(result['recovered_from_infeasible'] == (not initial_feasible and selected is not None), label+': recovery semantics')
    checks.check(result['solver_success_matches_sampled_candidate'] == (bool(result['solver_success']) == bool(selected)), label+': termination separate from feasibility')
    if selected is None:
        for field in ('candidate_vector', 'candidate_world', 'candidate_objective', 'support_poses',
                      'support_twists', 'constraint_report', 'selected_iterate', 'objective_delta'):
            checks.check(result[field] is None, label+': rejected '+field+' remains null')
        checks.check(not result['selected_objective_improved'] and not result['selected_objective_improved_beyond_ftol'], label+': no invented improvement')
    else:
        value, selected_label, vector = selected
        checks.check(result['selected_iterate'] == selected_label, label+': selected original min objective')
        checks.equal(result['candidate_vector'], vector, label+': exact selected vector', exact=True)
        p, v = problem.unpack(vector)
        for field, expected in [('support_poses', p), ('support_twists', v), ('candidate_world', p[1:])]:
            checks.equal(result[field], expected, label+': exact '+field, exact=True)
        delta = value-problem.evaluate(initial)['objective']
        checks.equal(result['candidate_objective'], value, label+': selected objective')
        checks.equal(result['constraint_report'],problem.dense_report(vector),label+': selected dense report')
        checks.equal(result['objective_delta'], delta, label+': objective delta')
        checks.check(result['selected_objective_improved'] == (delta < 0), label+': strict improvement semantics')
        checks.check(result['selected_objective_improved_beyond_ftol'] == (delta < -problem.config['ftol']), label+': meaningful improvement semantics')
    same = bool(selected is not None and np.array_equal(selected[2], initial))
    checks.check(result['returned_initial_unchanged'] == same, label+': returned initial semantics')
    checks.check(result['feasible_initial_retained'] == (initial_feasible and same), label+': retained versus recovered')
    profile = result['profiling']
    checks.check(profile['evaluate_requests'] == profile['evaluator_cache_hits']+profile['evaluator_cache_misses'], label+': cache accounting')
    checks.check(0 < profile['unique_vector_evaluations'] <= profile['evaluator_cache_misses'], label+': distinct evaluations')
    for role, field in [('objective','objective_evaluations'),('equality','equality_evaluations'),('inequality','inequality_evaluations')]:
        checks.check(profile['function_calls'].get(role,0) == result[field], label+': '+role+' call count')
    checks.check(sum(profile['function_calls'].values()) == profile['evaluate_requests'], label+': all request roles')
    for field in ('solve_wall_time_s','post_solve_validation_time_s','pre_solve_budget_cost_s','setup_wall_time_s'):
        checks.check(np.isfinite(result[field]) and result[field] >= 0, label+': finite '+field)
    checks.equal(result['solve_and_pre_solve_wall_time_s'], result['solve_wall_time_s']+result['pre_solve_budget_cost_s'], label+': budget cost includes initialization')
    checks.check(result['configured_wall_time_budget_s'] == 30., label+': original wall budget')
    checks.check(result['remaining_solver_budget_s'] == max(0.,30.-result['pre_solve_budget_cost_s']), label+': budget remaining')
    return selected


def validate_saved_audit(run, primary, checks):
    base = run/'existing_failure_audit'
    audit, summary = read(base/'audit.json'), read(base/'summary.json')
    checks.equal(audit['summary'], summary, 'audit: duplicated summary')
    checks.check(len(audit['attempts']) == summary['attempt_count'] == 40, 'audit: all 40 attempts')
    checks.check(summary['saved_iterates_evaluated'] == 80 and summary['unrecorded_iterates_evaluated'] == 0, 'audit: 80 saved, zero invented iterates')
    hashes = read(base/'input_hashes.json')
    checks.equal(hashes['before'], hashes['after'], 'audit: historical inputs unchanged', exact=True)
    for path, saved in hashes['before'].items():
        checks.check(digest(path) == saved, 'audit: input hash '+path)
    records = []
    environment = None
    identities = set()
    for saved in audit['attempts']:
        key = (saved['case_id'], saved['method'], saved['initialization_index'])
        checks.check(key not in identities, 'audit: unique attempt '+str(key)); identities.add(key)
        loaded = load_frozen_case(primary, key[0], key[1], environment=environment)
        environment = loaded['environment']
        historical = read(loaded['case_directory']/'methods'/key[1]/'solver_result.json')['attempts'][key[2]]
        current = classify_saved_attempt(loaded['problem'], historical)
        for field, value in current.items():
            checks.equal(saved[field], value, 'audit: reproduced '+str(key)+' '+field)
        checks.check(saved['iteration_history_status'] == 'UNKNOWN_NOT_SAVED', 'audit: no invented convergence history')
        records.append(current)
    counts = dict(Counter(r['classification'] for r in records))
    checks.equal({k:v for k,v in summary['classification_counts'].items() if v}, counts, 'audit: recomputed classification counts')
    checks.equal(summary['termination_counts'],dict(Counter(r['original_termination'] for r in records)), 'audit: termination counts')
    for field,label in [('initial_classification_counts','initial'),('latest_classification_counts','latest'),
                        ('best_feasible_saved_evidence_counts','best_feasible_intermediate')]:
        checks.equal(summary[field],dict(Counter(r[label]['classification'] for r in records)), 'audit: '+field)
    checks.equal(summary['initial_and_latest_classification_counts'],
                 dict(Counter(r[label]['classification'] for r in records for label in ('initial','latest'))),
                 'audit: all saved initial/latest classes')
    families,sites=Counter(),Counter()
    for record in records:
        failed=[r for r in record['latest']['residuals'] if not r['passed']]
        families.update({r['family'] for r in failed})
        sites.update({(r['family'],r['site'],r['derivative_side'] or 'interior') for r in failed})
    checks.equal(summary['latest_failed_family_counts'],dict(families),'audit: residual family counts')
    expected_sites=[dict(family=k[0],site=k[1],derivative_side=k[2],attempt_count=v) for k,v in sorted(sites.items())]
    checks.equal(summary['latest_failure_site_counts'],expected_sites,'audit: support/midpoint/off-collocation counts')
    checks.check(summary['saved_report_reproduction_count'] == summary['saved_report_reproduction_matches'] == 80, 'audit: historical report reproduction')
    completion = read(base/'completion.json')
    checks.check(completion['optimization_performed'] is False and completion['all_40_attempts_reclassified'], 'audit: no optimization')


def validate_synthetic(run, formulation, checks):
    from reconciliation.gp_se2_diag_plot_sources import verify_math_plot_sources
    base = run/'synthetic_fixtures/math'
    provenance=verify_math_plot_sources(base)
    checks.check(provenance['valid'],'fixtures: all mathematical plot sources/traces/units/limits/hashes: '+str(provenance['errors']))
    checks.check(provenance['plot_count']==13 and provenance['trace_count']==64,
                 'fixtures: complete 13-plot 64-trace mathematical provenance')
    checks.check(provenance['wrote_files'] is False,'fixtures: provenance verification remains read-only')
    summary = read(base/'summary.json')
    checks.check(summary['optimizer_executed'] is False, 'fixtures: optimizer-free mathematics')
    for i, name in enumerate(('S0','S1','S2','S3')):
        saved = read(base/name/'numeric_results.json')
        current = evaluate_fixture(name, formulation)
        checks.equal(saved, current, 'fixtures: independent ODE/FD/GP reevaluation '+name)
        checks.equal(summary['fixtures'][i], saved['summary'], 'fixtures: summary '+name)
        checks.equal(read(base/name/'input.json'), saved['fixture_input'], 'fixtures: frozen input '+name, exact=True)
        checks.check(saved['summary']['support_count'] == 31 and saved['summary']['horizon_s'] == 3., 'fixtures: 3-second size '+name)
    blind = read(base/'collocation_blind_spot/numeric_results.json')
    checks.equal(blind, evaluate_blind_spot(formulation), 'fixtures: independent blind-spot reevaluation')
    checks.equal(summary['blind_spot'], blind['summary'], 'fixtures: blind-spot summary')
    checks.check(blind['summary']['endpoint_midpoint_lateral_pass'] and not blind['summary']['dense_lateral_pass'], 'fixtures: collocation pass is distinct from dense pass')
    solver = run/'synthetic_fixtures/solver'
    solver_summary = read(solver/'summary.json')
    checks.check(len(solver_summary['rows']) == 6 and solver_summary['synthetic_only'], 'fixtures: all six solver attempts')
    for name in ('S0','S1','S2'):
        problem, fixture = make_fixture_problem(name, formulation)
        checks.equal(read(solver/name/'input.json'), fixture, 'fixtures: deterministic saved perturbation '+name)
        for kind in ('known','perturbed'):
            result = read(solver/name/(kind+'.json'))
            checks.equal(result['initial_vector'], fixture[kind+'_vector'], 'fixtures: exact initial '+name+'/'+kind, exact=True)
            validate_attempt(problem,result,checks,'fixtures: '+name+'/'+kind)
            checks.check(result['evidence_kind'] == 'SYNTHETIC_SOLVER_DIAGNOSTIC_NOT_RESEARCH_PERFORMANCE', 'fixtures: synthetic status '+name+'/'+kind)
            rows=[row for row in solver_summary['rows'] if row['fixture']==name and row['initialization']==kind]
            checks.check(len(rows)==1, 'fixtures: unique summary '+name+'/'+kind)
            if rows:
                checks.check(fixture_initialization_matches(name,kind,result['initialization']),
                             'fixtures: seed kind maps to named initialization '+name+'/'+kind)
                for key,value in rows[0].items():
                    if key in result and key!='initialization':
                        checks.equal(value,result[key],'fixtures: summary consistency '+name+'/'+kind+'/'+key)
                checks.check(rows[0]['unique_evaluations']==result['profiling']['unique_vector_evaluations'], 'fixtures: unique evaluation summary '+name+'/'+kind)


def validate_actual(run, primary, config, checks):
    from reconciliation.gp_se2_rollout import load_frozen_context
    snapshot=read(run/'actual_event/input_snapshot.json')
    checks.check(snapshot['case_id']==config['actual_case_id']=='episode_001_repeat_01/handoff_002', 'actual: fixed benign event')
    checks.check(snapshot['gp_initial_twist_source']=='physical u_minus, not controller previous_control', 'actual: physical twist source')
    context=snapshot['context']
    checks.equal(context,load_frozen_context(context['source_root'],context['episode_id'],context['handoff_id']),
                 'actual: independently reloaded original online context',exact=True)
    environment=None
    summaries=[]
    for method in config['methods']:
        case=load_frozen_case(primary,config['actual_case_id'],method,environment=environment)
        environment=case['environment'];problem=case['problem']
        checks.equal(snapshot['source_hashes'],case['hashes'],'actual: exact input hashes '+method,exact=True)
        checks.equal(snapshot['context'],case['context'],'actual: frozen original context '+method,exact=True)
        checks.equal(snapshot['config'],case['config'],'actual: original config '+method,exact=True)
        checks.equal(snapshot['goal_route'],case['goal_route'],'actual: original goal/gates '+method,exact=True)
        checks.equal(snapshot['F_common'],case['common_reference'],'actual: common reference '+method,exact=True)
        checks.equal(snapshot['F_native'],np.load(case['case_directory']/'F_native.npy'), 'actual: raw reference '+method,exact=True)
        checks.equal(snapshot['support_times_s'],problem.times,'actual: support grid '+method,exact=True)
        checks.equal(snapshot['reference_times_s'],problem.times[1:],'actual: reference grid '+method,exact=True)
        starts=problem.initializations()
        original_vectors=[problem.vector(i['poses'],i['twists']) for i in starts]
        for phase in ('baseline','single_change'):
            directory=run/'actual_event'/phase/method
            summary=read(directory/'summary.json');summaries.append(summary)
            attempts=[]
            for index in range(2):
                result=read(directory/f'start_{index:02d}.json');attempts.append(result)
                label=f'actual: {phase}/{method}/{index}'
                checks.equal(result['input_hashes'],case['hashes'],label+': unchanged input hashes',exact=True)
                validate_attempt(problem,result,checks,label)
                if phase=='baseline' or index==0:
                    checks.equal(result['initial_vector'],original_vectors[index],label+': original initialization',exact=True)
                else:
                    from reconciliation.se2 import compose_poses,se2_exp
                    t=problem.times;T=float(t[-1]);q=t-t*t/(2*T)
                    poses=compose_poses(problem.boundary_pose,se2_exp(q[:,None]*problem.initial_twist))
                    twists=(1-t/T)[:,None]*problem.initial_twist
                    checks.equal(result['initial_vector'],problem.vector(poses,twists),label+': only fixed S2 seed changes',exact=True)
                checks.check(result['phase']==phase and result['method']==method and result['case_id']==config['actual_case_id'],label+': identity')
            choices=[(a['candidate_objective'],i,a) for i,a in enumerate(attempts) if a['candidate_found']]
            chosen=min(choices,key=lambda item:(item[0],item[1])) if choices else None
            checks.check(summary['sampled_candidate_found']==bool(chosen),'actual: phase sampled availability '+phase+'/'+method)
            checks.check(not summary['new_execution_performed'] and not summary['fallback_used'],'actual: no execution/fallback '+phase+'/'+method)
            checks.equal(summary['original_formulation_config'],problem.config,'actual: summary hard criteria '+phase+'/'+method,exact=True)
            checks.equal(summary['solver_wall_total_s'],sum(a['solve_wall_time_s'] for a in attempts),'actual: total solve time '+phase+'/'+method)
            checks.equal(summary['post_validation_total_s'],sum(a['post_solve_validation_time_s'] for a in attempts),'actual: postsolve time '+phase+'/'+method)
            checks.check(summary['objective_evaluations']==sum(a['objective_evaluations'] for a in attempts),'actual: objective evaluation total '+phase+'/'+method)
            if chosen:
                selected=chosen[2]
                checks.check(summary['selected_start']==chosen[1] and summary['selected_iterate']==selected['selected_iterate'], 'actual: selected start '+phase+'/'+method)
                current=check_full_candidate(problem,selected['candidate_vector'],case,**config['additional_check_without_claim'])
                saved=read(directory/'independent_acceptance.json')
                checks.equal(saved,current,'actual: full independent acceptance '+phase+'/'+method)
                checks.check(summary['full_candidate_found']==current['full_feasible'],'actual: full candidate status '+phase+'/'+method)
                if current['full_feasible']:
                    for filename,key in [('candidate_world.npy','candidate_world'),('support_poses.npy','support_poses'),('support_twists.npy','support_twists')]:
                        checks.equal(np.load(directory/filename,allow_pickle=False),selected[key], 'actual: exported exact '+phase+'/'+method+'/'+filename,exact=True)
                else:
                    checks.check(summary['candidate_world'] is None and not (directory/'candidate_world.npy').exists(),'actual: rejected full candidate not exported '+phase+'/'+method)
            else:
                checks.check(not summary['full_candidate_found'] and summary['selected_start'] is None and summary['candidate_world'] is None,'actual: absent candidate remains null '+phase+'/'+method)
                checks.check(not (directory/'candidate_world.npy').exists(),'actual: no rejected candidate array '+phase+'/'+method)
            if phase=='single_change':
                witness=check_full_candidate(problem,attempts[1]['initial_vector'],case,**config['additional_check_without_claim'])
                checks.equal(read(directory/'independent_seed_acceptance.json'),witness,'actual: independent initial witness '+method)
                checks.check(summary['seed_full_feasible']==witness['full_feasible'],'actual: seed is separately classified '+method)
                checks.equal(summary['seed_construction_wall_s'],attempts[1]['pre_solve_budget_cost_s'],'actual: seed work fully charged '+method)
                checks.equal(summary['seed_construction_plus_solver_wall_s'],summary['seed_construction_wall_s']+summary['solver_wall_total_s'],'actual: total variant compute '+method)
    checks.check(len(list((run/'actual_event').glob('*/*/start_*.json')))==8,'actual: only fixed two-method four-start-per-phase diagnostic')
    checks.check(not list(run.rglob('rollout.json')) and not list(run.rglob('execution.csv')), 'actual: no new executions')
    return summaries


def validate_preservation(run, source, checks):
    before,after=read(run/'preservation_before.json'),read(run/'preservation_after.json')
    for field in ('historical_files','external_files','external_git_sha','external_git_status','user_config_hashes'):
        checks.equal(before[field],after[field],'preservation: before/after '+field,exact=True)
    checks.check(before['valid'] and after['valid'],'preservation: source validity')
    for group in ('historical_files','external_files','user_config_hashes'):
        for name,value in after[group].items():
            path=Path(name) if Path(name).is_absolute() else ROOT/name
            checks.check(path.is_file() and digest(path)==value,'preservation: current '+name)
    for name,value in source['core_numerical_source_sha256'].items():
        checks.check(digest(ROOT/name)==value,'source: original numerical core '+name)
    differences=[]
    expected_stages={'prepare','baseline','synthetic-solves','single_change'}
    manifests=sorted((run/'implementation_sources').glob('*/manifest.json'))
    checks.check(expected_stages.issubset({path.parent.name for path in manifests}),'source: all experimental stage snapshots exist')
    for manifest in manifests:
        for name,value in read(manifest)['files'].items():
            checks.check(digest(manifest.parent/name)==value,'source: archived stage '+str(manifest.parent.name)+'/'+name)
            current=digest(ROOT/name) if (ROOT/name).is_file() else None
            if current!=value:
                differences.append(dict(stage=manifest.parent.name,path=name,archived_sha256=value,current_sha256=current))
    solver_name='src/reconciliation/gp_se2_diag_solver.py'
    if expected_stages.issubset({path.parent.name for path in manifests}):
        hashes=[read(run/'implementation_sources'/stage/'manifest.json')['files'][solver_name]
                for stage in ('baseline','synthetic-solves','single_change')]
        checks.check(len(set(hashes))==1 and hashes[0]==digest(ROOT/solver_name), 'source: identical instrumented original solver across all experiments')
    checks.check(source['current_full_retained_data_rehash_valid'] and source['source_validation_valid'] and source['source_hash_validation_valid'],'source: original source validations')
    return differences


def validate_bundle(run, checks):
    bundle=run/'review_bundle'
    for name in ('summary.json','existing_rejections.csv','fixture_results.csv','actual_event_comparison.csv','decision.json','README.md'):
        checks.check((bundle/name).is_file(),'bundle: required '+name)
    checks.equal(read(bundle/'decision.json'),read(run/'decision.json'),'bundle: exact decision',exact=True)
    summary=read(bundle/'summary.json')
    checks.equal(summary['existing_failure_audit'],read(run/'existing_failure_audit/summary.json'),'bundle: audit summary',exact=True)
    checks.equal(summary['mathematical_fixtures'],read(run/'synthetic_fixtures/math/summary.json'),'bundle: math summary',exact=True)
    checks.check(summary['no_new_execution'] and summary['synthetic_is_not_performance_evidence'],'bundle: correct evidence scope')
    rows=list(csv.DictReader((bundle/'actual_event_comparison.csv').open()))
    checks.check(len(rows)==len(summary['actual_event_comparison'])==8,'bundle: complete actual comparison table')
    for csv_row,json_row in zip(rows,summary['actual_event_comparison']):
        for name,value in json_row.items():
            checks.check(csv_row[name]==('' if value is None else str(value)),'bundle: JSON/CSV comparison value '+name)
        method=json_row['method'];phase=json_row['phase'];start=json_row['start']
        result=read(run/'actual_event'/phase/method/(start+'.json'))
        for name,value in json_row.items():
            if name in result:
                checks.equal(value,result[name],'bundle: comparison from actual result '+phase+'/'+method+'/'+start+'/'+name)
        checks.check(json_row['source_sha256']==digest(run/'actual_event'/phase/method/(start+'.json')),'bundle: comparison result hash')
    for name,source in [('existing_rejections.csv',run/'existing_failure_audit/existing_rejections.csv'),
                        ('fixture_results.csv',run/'synthetic_fixtures/math/fixture_results.csv'),
                        ('actual_event_comparison.csv',run/'plots/actual_event_comparison.csv')]:
        checks.check((bundle/name).read_bytes()==source.read_bytes(),'bundle: copied table '+name)
    pngs=list(bundle.glob('*.png'))
    checks.check(bool(pngs),'bundle: representative images')
    for png in pngs:
        checks.check(png.read_bytes()[:8]==b'\x89PNG\r\n\x1a\n','bundle: valid PNG signature '+png.name)
    archives=list(run.glob('*.zip'))
    checks.check(len(archives)==1,'bundle: one review ZIP')
    if len(archives)==1:
        with zipfile.ZipFile(archives[0]) as archive:
            members=archive.namelist()
            checks.check(len(members)==len(set(members)),'bundle: no duplicate ZIP entries')
            files={str(path.relative_to(run)):path for path in bundle.rglob('*') if path.is_file()}
            checks.check(set(members)==set(files),'bundle: ZIP contains exactly review files')
            for name in members:
                item=Path(name)
                checks.check(not item.is_absolute() and '..' not in item.parts,'bundle: safe relative member '+name)
                checks.check(item.suffix.lower() in ('.json','.csv','.png','.md'), 'bundle: excludes datasets/weights '+name)
                if name in files:
                    checks.check(archive.read(name)==files[name].read_bytes(),'bundle: exact archived bytes '+name)


def validate_plots(run, primary, config, checks):
    from plot_gp_se2_diag_01 import history_data, trajectory_data, profiling_parts, actual_records
    from reconciliation.gp_se2_diagnostics import CLASSIFICATIONS
    from shapely.geometry import box
    manifest=read(run/'plots/manifest.json')
    checks.check(manifest['valid'] and manifest['no_new_optimization'] and manifest['no_new_execution'], 'plots: declared scope')
    checks.check(len(manifest['plots'])==13,'plots: required 13 new diagnostic graphs')
    loaded=None
    for item in manifest['plots']:
        image,sidecar=Path(item['path']),Path(item['sidecar'])
        checks.check(image.is_relative_to(run/'plots') and sidecar.is_relative_to(run/'plots'),'plots: artifacts inside new run')
        checks.check(digest(image)==item['sha256'] and digest(sidecar)==item['sidecar_sha256'],'plots: manifest hashes '+image.name)
        record=read(sidecar);data=record['numeric_data']
        checks.check(record['image']==image.name and record['image_sha256']==digest(image),'plots: image identity '+image.name)
        checks.check(image.read_bytes()[:8]==b'\x89PNG\r\n\x1a\n' and not record['executed_trajectory'],'plots: PNG/no execution '+image.name)
        for source,saved_hash in record['source_hashes'].items():
            checks.check(digest(source)==saved_hash,'plots: source hash '+source)
        if 'synthetic_fixtures' in image.parts:
            name=image.parent.name
            expected={kind:history_data(read(run/'synthetic_fixtures/solver'/name/(kind+'.json'))) for kind in ('known','perturbed')}
            checks.equal(data['histories'],expected,'plots: true synthetic callback history '+name)
        elif 'actual_event' in image.parts:
            method=image.parent.name
            records=actual_records(run,method)
            if image.name=='constraint_history.png':
                expected={label:history_data(result) for label,_,result,_ in records}
                checks.equal(data['histories'],expected,'plots: true actual callback history '+method)
            elif image.name=='profiling.png':
                expected={label:profiling_parts(result) for label,_,result,_ in records}
                checks.equal(data,expected,'plots: profiling components '+method)
                for label,result in expected.items():
                    checks.equal(result['disjoint_total_seconds'],result['measured_total_seconds'],'plots: disjoint profile total '+label)
            else:
                expected={label:trajectory_data(result,full_accepted=full) for label,_,result,full in records}
                checks.equal(data['traces'],expected,'plots: true saved GP trajectories '+method+'/'+image.name)
                if image.name=='world_xy.png':
                    if loaded is None:
                        loaded=load_frozen_case(primary,config['actual_case_id'])
                    checks.equal(data['boundary_world'],loaded['context']['B_world'],'plots: original B',exact=True)
                    checks.equal(data['goal_world'],loaded['goal_route']['goal_world'],'plots: original goal',exact=True)
                    original=data['original']
                    checks.equal(original['OLD'],loaded['context']['old_world'],'plots: original OLD',exact=True)
                    checks.equal(original['F_native'],np.load(loaded['case_directory']/'F_native.npy'),'plots: original RAW',exact=True)
                    checks.equal(original['F_common'],loaded['common_reference'],'plots: original common',exact=True)
                    lo0,lo1,hi0,hi1=data['axes_bounds_m'];clip=box(lo0,lo1,hi0,hi1)
                    checks.check(data['workspace_wkt']==loaded['environment'].workspace.intersection(clip).wkt,'plots: original workspace')
                    checks.check(data['obstacles_wkt']==loaded['environment'].obstacles.intersection(clip).wkt,'plots: original obstacles')
        elif image.name=='existing_rejection_classification.png':
            audit=read(run/'existing_failure_audit/audit.json')
            checks.equal(data['classifications'],list(CLASSIFICATIONS),'plots: rejection labels',exact=True)
            checks.equal(data['counts'],[audit['summary']['classification_counts'][name] for name in CLASSIFICATIONS],'plots: rejection counts',exact=True)
        elif image.name=='existing_lateral_by_location.png':
            audit=read(run/'existing_failure_audit/audit.json');expected=[]
            for attempt in audit['attempts']:
                row=[]
                for site in ('support','midpoint','off-collocation'):
                    entries=[r for r in attempt['latest']['residuals'] if r['family']=='lateral_velocity' and r['site']==site]
                    row.append(None if not entries else max(max(abs(r['actual_minimum']),abs(r['actual_maximum'])) for r in entries))
                expected.append(row)
            checks.equal(data['maximum_lateral_m_s'],expected,'plots: support/midpoint/off-collocation actual residuals')
    bundle=manifest['review_bundle']
    checks.check(digest(bundle['archive'])==bundle['archive_sha256'],'plots: review archive hash')
    for entry in bundle['files']:
        checks.check(digest(Path(bundle['directory'])/entry['name'])==entry['sha256'],'plots: review member hash '+entry['name'])
    for sidecar in (run/'review_bundle').glob('*.json'):
        payload=read(sidecar)
        if 'image' not in payload:
            continue
        checks.check(digest(sidecar.with_name(payload['image']))==payload['image_sha256'],'bundle: sidecar image hash '+sidecar.name)
        for source,value in payload['source_hashes'].items():
            checks.check(digest(source)==value,'bundle: image source hash '+source)
        original=[item for item in manifest['plots'] if item['sha256']==payload['image_sha256']]
        if original:
            checks.equal(payload['numeric_data'],read(original[0]['sidecar'])['numeric_data'],'bundle: copied plot numbers '+sidecar.name,exact=True)
        else:
            numeric=[Path(path) for path in payload['source_hashes'] if Path(path).name=='numeric_results.json']
            checks.check(len(numeric)==1,'bundle: math numeric source '+sidecar.name)
            if numeric:
                checks.equal(payload['numeric_data'],read(numeric[0]),'bundle: math numbers '+sidecar.name,exact=True)


def validate_decision(run, checks):
    decision=read(run/'decision.json')
    checks.check(decision['selected_change']=='B_FEASIBLE_INITIALIZATION','decision: exactly one initialization change')
    checks.check(not decision['seed_parameters_fitted'] and not decision['raw_rollout_warm_start'] and
                 not decision['restoration_phase'] and decision['no_chained_second_change'], 'decision: no hidden additional remedy')
    for name,value in decision['evidence_sha256'].items():
        checks.check(digest(run/name)==value,'decision: predecision evidence hash '+name)
    decision_time=datetime.fromisoformat(decision['decision_utc'])
    for phase in ('baseline','synthetic_fixtures/solver'):
        path=run/'actual_event/baseline/summary.json' if phase=='baseline' else run/phase/'summary.json'
        checks.check(datetime.fromisoformat(read(path)['completed_utc'])<=decision_time,'decision: completed evidence before decision '+phase)
    variant=read(run/'actual_event/single_change/source.json')
    checks.check(datetime.fromisoformat(variant['created_utc'])>=decision_time,'decision: variant starts after decision')
    budget=decision['budget']
    checks.check(budget['starts_per_method']==2 and budget['max_iterations']==200 and budget['wall_s_per_start']==30. and budget['blas_threads']==1,'decision: unchanged budget')


def validate_final_summary(run,checks):
    summary=read(run/'summary.json')
    methods=[read(run/'actual_event'/phase/method/'summary.json')
             for phase in ('baseline','single_change') for method in ('M2_GP_NO_OBSTACLE','M3_GP_CONSTRAINED')]
    checks.equal(summary['actual_methods'],methods,'final: four method-phase summaries',exact=True)
    candidate_status=('FULL_FEASIBLE_CANDIDATE_FOUND' if any(m['full_candidate_found'] for m in methods)
                      else 'NO_FULL_FEASIBLE_CANDIDATE_FOUND')
    checks.check(summary['actual_event_candidate_status']==candidate_status,'final: candidate status independent of solver termination')
    checks.check(summary['operational_status']=='GP_SE2_DIAG_01_COMPLETED_WITH_LIMITATIONS','final: operational limitations explicit')
    checks.check(summary['diagnosis_status']=='PARTIALLY_LOCALIZED','final: unique root cause not asserted')
    checks.check(summary['actual_case_id']=='episode_001_repeat_01/handoff_002','final: only specified actual event')
    checks.check(not summary['new_inference'] and not summary['new_mpc_execution'] and summary['no_fallback']
                 and not summary['acceptance_relaxed'] and not summary['infeasibility_proven'],'final: no extra scope or overstated failure')
    checks.equal(summary['existing_failure_audit'],read(run/'existing_failure_audit/summary.json'),'final: audit numbers',exact=True)
    checks.equal(summary['mathematical_fixtures'],read(run/'synthetic_fixtures/math/summary.json'),'final: fixture numbers',exact=True)
    checks.equal(summary['synthetic_solver_results'],read(run/'synthetic_fixtures/solver/summary.json')['rows'],'final: synthetic solver numbers',exact=True)
    checks.check(summary['selected_change']==read(run/'decision.json')['selected_change'],'final: one selected change')
    after=read(run/'preservation_after.json')
    checks.check(summary['preservation_valid']==after['valid'] and
                 summary['preserved_historical_file_count']==after['historical_file_count'] and
                 summary['preserved_checkpoint_file_count']==after['checkpoint_file_count'],'final: preservation counts')
    rows=summary['actual_attempts']
    checks.check(len(rows)==8 and len({(r['phase'],r['method'],r['start']) for r in rows})==8,'final: eight unique actual attempts')
    for row in rows:
        path=run/'actual_event'/row['phase']/row['method']/f'start_{row["start"]:02d}.json'
        result=read(path);initial,latest=result['candidate_checks'][:2];profile=result['profiling']
        for key,value in row.items():
            if key in result and key!='seed_construction_wall_s':
                checks.equal(value,result[key],'final: actual direct field '+key)
        expected=dict(initial_objective=initial['objective'],latest_objective=latest['objective'],
            objective_improved_beyond_ftol=result['selected_objective_improved_beyond_ftol'],
            latest_collocation_max_equality_residual=latest['collocation']['maximum_equality_residual'],
            latest_collocation_max_inequality_violation=latest['collocation']['maximum_inequality_violation'],
            latest_dense_max_equality_residual=latest['constraint_report']['maximum_equality_residual'],
            latest_dense_max_inequality_violation=latest['constraint_report']['maximum_inequality_violation'],
            latest_lateral_velocity_m_s=latest['constraint_report']['maximum_absolute_lateral_velocity_m_s'],
            seed_construction_wall_s=result['pre_solve_budget_cost_s'],
            unique_vector_evaluations=profile['unique_vector_evaluations'],evaluator_requests=profile['evaluate_requests'],
            evaluator_cache_misses=profile['evaluator_cache_misses'],numerical_component_seconds=profile['numerical_component_seconds'],
            instrumentation_bookkeeping_s=profile['measured_instrumentation_bookkeeping_seconds'],source_sha256=digest(path))
        for key,value in expected.items():
            checks.equal(row[key],value,'final: recomputed aggregate '+key)
        checks.check(source_path_matches(row['source'],path),'final: source path resolves to exact saved result')
    checks.equal(read(run/'profiling/summary.json')['rows'],rows,'final: profiling duplicates exact numbers',exact=True)
    for method,reduced in summary['full_candidate_checks'].items():
        report=read(run/'actual_event/single_change'/method/'independent_acceptance.json')
        grid=report['additional_grid']
        for key in ('status','full_feasible','original_dense_feasible','original_plan_valid'):
            checks.equal(reduced[key],report[key],'final: acceptance '+method+'/'+key,exact=True)
        expected={k:v for k,v in grid.items() if k not in ('query_times_s','poses_world','body_twists','body_accelerations','environment','route')}
        checks.equal(reduced['additional_grid'],expected,'final: all additional-grid numbers '+method,exact=True)
        checks.equal(reduced['minimum_clearance_m'],grid['environment']['minimum_clearance_m'],'final: clearance '+method,exact=True)
        checks.equal(reduced['goal_position_error_m'],report['original_plan_report']['goal']['position_error_m'],'final: original goal position '+method,exact=True)
        checks.equal(reduced['goal_yaw_error_rad'],report['original_plan_report']['goal']['yaw_error_rad'],'final: original goal yaw '+method,exact=True)
    if (run/'artifact_manifest.json').exists():
        files=read(run/'artifact_manifest.json')['files']
        for name,value in files.items():
            path=(run/name).resolve()
            checks.check(path.is_relative_to(run),'artifacts: confined manifest path '+name)
            checks.check(path.is_file() and digest(path)==value,'artifacts: frozen generated bytes '+name)
        required=[run/'summary.json',run/'decision.json',run/'review_bundle.zip',run/'plots/manifest.json']
        required+=list((run/'actual_event').glob('*/*/start_*.json'))
        required+=list((run/'actual_event').glob('*/*/*.npy'))
        checks.check(all(str(p.relative_to(run)) in files for p in required),'artifacts: all scientific outputs represented in manifest')


def validate(run):
    run=Path(run).resolve();checks=Checks();differences=[];summaries=[]
    source=read(run/'source.json')
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    primary=Path(source['primary_run'])
    config['additional_check_without_claim']={key:config['additional_check'][key] for key in ('maximum_step_s','offset_fraction')}
    original=yaml.safe_load((primary/'config_snapshot.yaml').read_text())
    protocol=read(run/'protocol.json')
    checks.equal(protocol['original_config'],original,'protocol: original full acceptance/configuration',exact=True)
    checks.check(protocol['physical_acceptance_unchanged'] and not protocol['new_mpc_rollouts'] and not protocol['new_inference'],'protocol: authorized unchanged scope')
    checks.check(run!=primary and primary not in run.parents,'run: distinct from historical input')
    checks.check(digest(run/'config_snapshot.yaml')==source['config_sha256'],'source: diagnostic config hash')
    checks.check(digest(primary/'config_snapshot.yaml')==source['original_config_sha256'],'source: unchanged original config')
    checks.check(digest(primary/'case_manifest.json')==source['original_case_manifest_sha256'],'source: unchanged case selection')
    checks.section('saved audit',lambda:validate_saved_audit(run,primary,checks))
    checks.section('synthetic',lambda:validate_synthetic(run,original['formulation'],checks))
    def actual():
        summaries.extend(validate_actual(run,primary,config,checks))
    checks.section('actual',actual)
    checks.section('decision',lambda:validate_decision(run,checks))
    def preservation():
        differences.extend(validate_preservation(run,source,checks))
    checks.section('preservation',preservation)
    checks.section('bundle',lambda:validate_bundle(run,checks))
    checks.section('plots',lambda:validate_plots(run,primary,config,checks))
    checks.section('final summary',lambda:validate_final_summary(run,checks))
    return dict(valid=not checks.errors,check_count=checks.count,errors=checks.errors,
                validated_utc=datetime.now(timezone.utc).isoformat(),run=str(run),
                validation_kind='READ_ONLY_REEVALUATION_OF_SAVED_VECTORS_AND_ARTIFACTS',
                optimizer_invocations=0,new_mpc_rollouts=0,new_inference=0,
                stage_source_differences_from_current=differences,
                source_drift_policy='archived stages verified; later diagnostic/report changes listed, original numerical core unchanged',
                candidate_artifact_sha256={str(path.relative_to(run)):digest(path)
                    for path in sorted((run/'actual_event').glob('*/*/*.npy'))},
                actual_method_phase_summary_count=len(summaries),continuous_time_feasibility_proven=False)


def write_exclusive(path, result):
    with Path(path).open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.output and not args.output.resolve().is_relative_to(args.run.resolve()):
        raise ValueError('validator output must be inside the new diagnostic run')
    if args.output and args.output.exists():
        raise FileExistsError('validator output must be a new file')
    result=validate(args.run)
    if args.output:
        write_exclusive(args.output,result)
    print(json.dumps(result,indent=2))
    return 0 if result['valid'] else 1


if __name__=='__main__':
    raise SystemExit(main())
