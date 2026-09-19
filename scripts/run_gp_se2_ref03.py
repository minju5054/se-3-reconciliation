#!/usr/bin/env python3
"""Bounded REF-03 source-only selection, immutable execution and saved evaluation."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_ref01 import read, write, array, table, copy_new, digest, utc, revision, clean
from run_gp_se2_ref02 import preservation, SELECTOR_DEFINITION
from reconciliation.gp_se2_ref02_reference import METHODS, validate_lineage
from reconciliation.gp_se2_ref01_reference import build_variants
from reconciliation.gp_se2_reference import prepare_reference

STARTING_SHA = '22c655ea421c77c1bc7bd7c70da5fec7c9ae7620'
GROUPS = ('O_OBSTACLE_NEAR', 'R_LARGE_ROTATION', 'P_LARGE_MISMATCH', 'S_BENIGN_STRAIGHT')
REPORTING_ONLY = {'scripts/validate_gp_se2_ref03.py', 'tests/test_gp_se2_ref03_artifacts.py'}


def validate_config(c):
    if c['experiment'] != 'GP-SE2-REF-03' or c['starting_git_sha'] != STARTING_SHA:
        raise ValueError('reviewed source mismatch')
    if c['methods'] != list(METHODS) or c['group_order'] != list(GROUPS) or c['target_per_group'] != 6:
        raise ValueError('fixed method/group order and quotas required')
    if c['selector'] != dict(source_progress_stride=1., searchsorted_side='left', search_tolerance=0.):
        raise ValueError('frozen REF-02 selector required')
    if c['rollout'] != dict(horizon_s=3., control_hz=10., integration_hz=60.):
        raise ValueError('original rollout timing required')
    if c['mpc'] != dict(HORIZON=5, MPC_DT_S=.1, CONTROL_RATE_HZ=10., Q_WEIGHTS=[10.,10.,1.]):
        raise ValueError('original MPC settings required')
    if c['blas_threads'] != 1 or any(c[k] for k in ('new_VLA_updates','gp_or_rigid_optimization','retry','gui_runtime')):
        raise ValueError('bounded offline scope required')
    required={'src/reconciliation/'+name+'.py' for name in ('gp_se2_ref02_reference','gp_se2_ref02_rollout',
        'gp_se2_ref02_evaluation','gp_se2_ref01_reference','gp_se2_reference','gp_se2_rollout','gp_se2_evaluation',
        'online_mpc_adapter','se2','robotless_online')}
    if set(c['frozen_core_sha256'])!=required:raise ValueError('complete fixed core hash coverage required')
    for name,h in c['frozen_core_sha256'].items():
        if digest(ROOT/name) != h:
            raise ValueError('immutable numerical/selector core changed: '+name)
    return c


def reference_inputs(context, settings):
    """Existing preparation and lineage helpers; no new interpolation rule."""
    native = np.asarray(context['fresh_world'], dtype=np.float64)
    prepared = prepare_reference(native, context['B_world'], settings['Q_WEIGHTS'])
    factorized = build_variants(native, context['B_world'], settings['Q_WEIGHTS'],
        expected_common=prepared['common_world'], preparation_metadata=prepared)
    variants = factorized['variants']
    result = {}
    for method,variant in zip(METHODS, ('R00_NATIVE','R11_CURRENT_ADAPTER','R11_CURRENT_ADAPTER')):
        value = variants[variant]
        count = len(native) if method == METHODS[0] else len(prepared['suffix_world'])
        start = 0 if method == METHODS[0] else prepared['first_future_row_index']
        metadata = dict(constant_reference=True, retained_source_row_count=1, original_source_row_index=start) if count == 1 else None
        validate_lineage(value['reference_world'], value['row_provenance'], constant_reference_metadata=metadata)
        result[method] = dict(value=value, metadata=metadata)
    return prepared, factorized, result


def prior_usage(source):
    path = Path(source)/'aggregate/visual_review.json'
    if not path.exists():
        return {}
    result = {}
    content = read(path)
    for key in ('representatives','additional_status_examples'):
        for row in content.get(key,[]):
            result.setdefault(row['episode_id']+'/'+row['event_id'], []).append(dict(
                kind='prior dataset visual representative', record=str(path), record_sha256=digest(path), source_field=key))
    return result


def prepare(run, config_path):
    from reconciliation.gp_se2_ref03_cohort import build_cohort, cohort_policy
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_rollout import load_frozen_context
    from run_gp_se2_01 import route_for_source
    from plot_gp_se2_02 import past_execution
    begin = time.perf_counter()
    c = validate_config(yaml.safe_load(config_path.read_text()))
    if run.exists() or not run.is_relative_to(ROOT/'data/robotless_gp_se2_ref_03'):
        raise FileExistsError('new REF-03 output root required; no overwrite')
    roots = {name:(ROOT/path).resolve() for name,path in c['sources'].items()}
    previous = roots['REF02']; prior = read(previous/'source.json')
    if subprocess.run(['git','merge-base','--is-ancestor',STARTING_SHA,'HEAD'],cwd=ROOT).returncode:
        raise ValueError('reviewed source revision missing')
    validations = {}
    for label,root in roots.items():
        report = read(root/'validation.json')
        if not report['valid'] or (label.startswith('REF') and not report['authoritative']):
            raise ValueError('authoritative source validation failed: '+label)
        validations[str(root/'validation.json')] = digest(root/'validation.json')
    hash_start = time.perf_counter(); before = preservation(previous); hash_s = time.perf_counter()-hash_start
    if not before['valid']:
        raise ValueError('SOURCE_INTEGRITY_BLOCKER: preserved historical hash mismatch')
    config = yaml.safe_load((roots['GP01']/'config_snapshot.yaml').read_text())
    if any(digest(root/'config_snapshot.yaml') != digest(roots['GP01']/'config_snapshot.yaml') for label,root in roots.items() if label != 'online'):
        raise ValueError('historical physical acceptance mismatch')
    mpc = read(previous/'mpc_output/provenance.json')
    if any(mpc['official_settings'][k] != v for k,v in c['mpc'].items()):
        raise ValueError('official MPC configuration mismatch')
    env_start = time.perf_counter(); env = HospitalEnvironment.load(prior['environment_path']); env_s = time.perf_counter()-env_start
    manifests = {k:str(roots[k]/'case_manifest.json') for k in ('GP01','GP02','REF01','REF02')}
    audit_start = time.perf_counter()
    cohort = build_cohort(manifests['GP01'], manifests, source_root=roots['online'], environment=env,
        original_config=config, expected_source_hashes=before['files'], other_known_source_usage=prior_usage(roots['online']))
    cohort_s = time.perf_counter()-audit_start
    run.mkdir(parents=True)
    copy_new(config_path,run/'experiment_config.yaml'); copy_new(roots['GP01']/'config_snapshot.yaml',run/'config_snapshot.yaml')
    write(run/'preservation_before.json',before); write(run/'candidate_catalog.json',cohort)
    # Cohort report is preserved even if source integrity blocks execution.
    if cohort.get('source_integrity_blockers'):
        raise ValueError('SOURCE_INTEGRITY_BLOCKER: see candidate_catalog.json')
    selected = cohort['selected']
    write(run/'cohort_coverage.json',cohort['coverage'])
    copies, rows, requests, input_audit = {}, [], [], []
    def copied(src,dst):
        copy_new(src,dst); copies[str(Path(src).resolve())] = digest(src)
    for entry in selected:
        row = dict(entry); case_id = row['case_id']; episode_id,handoff_id = case_id.split('/')
        row.update(episode_id=episode_id,handoff_id=handoff_id,case_directory=episode_id+'__'+handoff_id)
        row['relative_directory'] = ('controls' if row['cohort']=='KNOWN_CONTROLS' else 'transfer')+'/'+row['case_directory']
        case = run/row['relative_directory']; case.mkdir(parents=True)
        context = load_frozen_context(roots['online'],episode_id,handoff_id)
        prepared,factorized,methods = reference_inputs(context,mpc['official_settings'])
        route = route_for_source(context['B_world'],prepared,env)
        if route['route_status'] != row['route_status'] or route['route_status']=='UNKNOWN':
            raise ValueError('SOURCE_INTEGRITY_BLOCKER: original route eligibility differs '+case_id)
        oldcase = roots['GP01']/'cases'/row['case_directory']
        if oldcase.exists():
            if read(oldcase/'input_context.json') != context or clean(route) != read(oldcase/'goal_route.json'):
                raise ValueError('saved known input/goal/gates do not reconstruct')
            for name in ('input_context.json','reference_preparation.json','goal_route.json','F_native.npy','F_common.npy','source_metrics.json'):
                copied(oldcase/name,case/name)
            if not np.array_equal(np.load(case/'F_common.npy',allow_pickle=False),prepared['common_world']):
                raise ValueError('existing F_common mismatch')
        else:
            write(case/'input_context.json',context);write(case/'reference_preparation.json',prepared);write(case/'goal_route.json',route)
            array(case/'F_native.npy',prepared['native_world']);array(case/'F_common.npy',prepared['common_world'])
            copied(roots['online']/'episodes'/episode_id/'handoffs'/handoff_id/'metrics.json',case/'source_metrics.json')
        raw = read(roots['online']/'episodes'/episode_id/'handoffs'/handoff_id/'context.json')
        for kind in ('old','fresh'):
            for frame in ('raw_local','world'):
                ref=raw[f'{kind}_{frame}_ref']; src=roots['online']/'episodes'/episode_id/ref['path']
                if digest(src)!=ref['sha256']:raise ValueError('raw array hash mismatch')
                copied(src,case/'input_snapshot'/f'{kind}_{frame}.npy')
        write(case/'reference_factorization.json',{k:v for k,v in factorized.items() if k!='variants'})
        request_methods={}
        for method,item in methods.items():
            folder=case/'methods'/method; folder.mkdir(parents=True)
            oldref=previous/'cases'/row['case_directory']/'methods'/method
            value=item['value']
            if oldref.exists():
                if not np.array_equal(np.load(oldref/'reference_world.npy',allow_pickle=False),value['reference_world']) or read(oldref/'row_provenance.json')!=value['row_provenance']:
                    raise ValueError('preserved REF02 array/lineage mismatch')
                for name in ('reference_world.npy','row_provenance.json','geometry_audit.json','definition.json'):
                    copied(oldref/name,folder/name)
            else:
                array(folder/'reference_world.npy',value['reference_world']);write(folder/'row_provenance.json',value['row_provenance'])
                write(folder/'geometry_audit.json',value['geometry_audit']);write(folder/'definition.json',{k:v for k,v in value.items() if k not in ('reference_world','row_provenance','geometry_audit')})
            req=dict(reference_path=str(folder/'reference_world.npy'),lineage_path=str(folder/'row_provenance.json'))
            if item['metadata'] is not None:
                write(folder/'lineage_metadata.json',item['metadata']);req['lineage_metadata_path']=str(folder/'lineage_metadata.json')
            request_methods[method]=req
            write(folder/'source.json',dict(case_id=case_id,method=method,reference_sha256=digest(folder/'reference_world.npy'),
                lineage_sha256=digest(folder/'row_provenance.json'),reference_value_sha256=value['reference_value_sha256'],
                reference_shape=list(value['reference_world'].shape),reference_dtype=str(value['reference_world'].dtype),
                frame=context['frame'],source_timestamps=context['source_timestamps'],raw_modified=False,reanchored=False))
            # Spatial input validation does not exclude any method's diagnostic rollout.
            spatial=env.check_polyline(value['reference_world']);write(folder/'spatial_reference_environment.json',spatial)
            input_audit.append(dict(case_id=case_id,cohort=row['cohort'],stratum=row.get('stratum'),method=method,
                raw_suffix_environment=env.check_polyline(prepared['suffix_world']),reference_environment=spatial,
                geometry=value['geometry_audit'],rollout_excluded_by_spatial_check=False))
            historical=None
            if oldref.exists(): historical=(oldref/'rollout.json',oldref/'metrics.json','REF02')
            elif oldcase.exists() and method in METHODS[:2]:
                oldmethod=oldcase/'methods'/('M0_NATIVE' if method==METHODS[0] else 'M0_ADAPTER')
                historical=(oldmethod/'rollout/rollout.json',oldmethod/'metrics.json','GP01')
            if historical:
                copied(historical[0],case/'historical'/method/'rollout.json');copied(historical[1],case/'historical'/method/'metrics.json')
                write(case/'historical'/method/'source.json',dict(source_experiment=historical[2],new_execution=False))
        b,z=[case/'methods'/m for m in METHODS[1:]]
        for name in ('reference_world.npy','row_provenance.json'):
            if (b/name).read_bytes()!=(z/name).read_bytes():raise ValueError('B/C byte identity failed')
        row['physical_command_controller_memory_differ']=bool(np.any(np.asarray(context['u_minus'])!=context['previous_control']))
        row['required_gate_ids']=[g['gate_id'] for g in route['gates']]
        past_execution(case)
        rows.append(row)
        requests.append(dict(episode_id=episode_id,handoff_id=handoff_id,cohort=row['cohort'],case_role=row['case_role'],stratum=row.get('stratum'),
            input_context_path=str(case/'input_context.json'),goal_route_path=str(case/'goal_route.json'),methods=request_methods))
    (run/'aggregate').mkdir()
    write(run/'case_manifest.json',dict(selected=rows,selected_count=len(rows),selection_used_new_optimization_or_rollouts=False,cohort_policy=cohort_policy()))
    write(run/'aggregate/input_audit.json',input_audit)
    source=dict(experiment='GP-SE2-REF-03',created_utc=utc(),starting_git_sha=STARTING_SHA,preparation_git_sha=revision(),
        sources={k:str(v) for k,v in roots.items()},source_run=str(roots['online']),primary_source=str(previous),
        environment_path=prior['environment_path'],environment_export_source=prior['environment_export_source'],
        environment_file_hashes=prior['environment_file_hashes'],original_MPC_environment=prior['original_MPC_environment'],
        official_mpc=mpc,authoritative_validations=validations,copied_source_sha256=copies,
        original_config_sha256=digest(run/'config_snapshot.yaml'),experiment_config_sha256=digest(run/'experiment_config.yaml'),
        frozen_core_sha256=c['frozen_core_sha256'],preserved_core_sha256=prior['preserved_core_sha256']|read(previous/'implementation_sources/final/manifest.json')['files'],
        preparation_total_wall_s=time.perf_counter()-begin,preservation_hash_audit_wall_s=hash_s,environment_loading_wall_s=env_s,cohort_audit_wall_s=cohort_s,
        timing_semantics='preparation total includes hashing, environment, cohort audit and selected input generation; components not additive to total')
    write(run/'source.json',source)
    write(run/'protocol.json',dict(experiment='GP-SE2-REF-03',configuration=c,cohort_policy=cohort_policy(),selector_definition=SELECTOR_DEFINITION,
        frozen_before_primary=True,expected_events=len(rows),planned_rollouts=len(rows)*3,planned_primary_mpc_solves=len(rows)*90,
        planned_selector_only_calls=len(rows)*60,execution_order=[dict(case_id=r['case_id'],method=m) for r in rows for m in METHODS],
        probe_timing='current A completion -> thirty actual A control input poses times B/C selectors -> B rollout -> C rollout',
        source_progress_units='original row order, not distance or physical time',new_VLA_updates=0,gp_or_rigid_solves=0,
        official_settings=mpc['official_settings'],official_settings_sha256=mpc['official_settings_sha256'],optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC'))
    write(run/'mpc_request.json',dict(source_root=str(roots['online']),expected_official_settings_sha256=mpc['official_settings_sha256'],
        cases=requests,frozen_case_order=[r['case_id'] for r in rows],planned_rollouts=len(rows)*3,
        primary_mpc_solves=len(rows)*90,selector_probes=len(rows)*60,source_progress_stride=1.,**c['rollout']))
    print(clean(dict(phase='PREPARED',events=len(rows),coverage=cohort['coverage'])),flush=True)


def verify(run):
    c=validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()));s=read(run/'source.json')
    for name,key in [('config_snapshot.yaml','original_config_sha256'),('experiment_config.yaml','experiment_config_sha256')]:
        if digest(run/name)!=s[key]:raise ValueError('frozen config changed')
    for p,h in s['copied_source_sha256'].items():
        if digest(p)!=h:raise ValueError('original source changed: '+p)
    for p,h in s['preserved_core_sha256'].items():
        if digest(ROOT/p)!=h:raise ValueError('original code changed: '+p)
    if (run/'execution_freeze.json').exists():
        freeze=read(run/'execution_freeze.json')
        for p,h in freeze['source_sha256'].items():
            if digest(ROOT/p)!=h:raise ValueError('execution code changed: '+p)
        for p,h in freeze['input_sha256'].items():
            if digest(run/p)!=h:raise ValueError('execution input changed: '+p)
    return c,s


def snapshot(run,phase):
    paths=list((ROOT/'src/reconciliation').glob('gp_se2_ref03*.py'))+list((ROOT/'scripts').glob('*gp_se2_ref03*.py'))
    paths+=list((ROOT/'scripts/lightnav').glob('*gp_se2_ref03*.py'))+list((ROOT/'tests').glob('test_gp_se2_ref03*.py'))+[ROOT/'configs/gp_se2_ref03.yaml']
    folder=run/'implementation_sources'/phase;folder.mkdir(parents=True,exist_ok=False);hashes={}
    for p in sorted(paths):
        name=str(p.relative_to(ROOT))
        if phase=='execution' and name in REPORTING_ONLY:continue
        copy_new(p,folder/name);hashes[name]=digest(p)
    write(folder/'manifest.json',dict(files=hashes,reporting_only_excluded=sorted(REPORTING_ONLY)))
    return hashes


def freeze(run):
    verify(run);hashes=snapshot(run,'execution')
    inputs={str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file() and not p.is_relative_to(run/'implementation_sources') and p.suffix!='.log'}
    write(run/'execution_freeze.json',dict(execution_git_sha=revision(),frozen_utc=utc(),no_primary_execution_yet=True,source_sha256=hashes,input_sha256=inputs))


def execute(run):
    _,source=verify(run)
    if not (run/'execution_freeze.json').exists():raise ValueError('freeze before primary execution')
    if (run/'execution_started.json').exists() or (run/'mpc_output').exists():raise FileExistsError('primary already started; no retry')
    command=[str(Path(source['original_MPC_environment'])/'bin/python'),str(ROOT/'scripts/lightnav/gp_se2_ref03_mpc.py'),
        '--request',str(run/'mpc_request.json'),'--output',str(run/'mpc_output'),'--lightnav-checkout',source['official_mpc']['lightnav_checkout']]
    write(run/'execution_started.json',dict(utc=utc(),command=command,request_sha256=digest(run/'mpc_request.json')))
    start=time.perf_counter()
    with (run/'mpc_worker.log').open('x') as stream: result=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    write(run/'execution_completed.json',dict(utc=utc(),exit_code=result.returncode,worker_process_wall_s=time.perf_counter()-start,command=command))
    if result.returncode:raise RuntimeError('worker failed; preserve ledger and logs; no scientific retry')


def scope_exports(aggregate, key):
    return dict(additional_transfer=aggregate['additional_transfer'][key],
        known_controls=aggregate['known_controls'][key],
        transfer_strata={s:v[key] for s,v in aggregate['transfer_strata'].items()},
        known_control_by_role={s:v[key] for s,v in aggregate['known_control_by_role'].items()})


def evaluate(run):
    """Evaluate recorded independent executions once, including missing outcomes."""
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_ref03_evaluation import (evaluate_method, missing_outcome, aggregate_outcomes,
        selection_mechanism, compare_selections, first_command_divergences, control_reproduction)
    c,source=verify(run);output=run/'mpc_output'
    if (run/'evaluation_started.json').exists():raise FileExistsError('evaluation already started; no overwrite')
    for record in read(output/'output_hashes.json')['files']:
        path=(output/record['path']).resolve()
        if not path.is_relative_to(output) or digest(path)!=record['sha256']:raise ValueError('worker output integrity mismatch')
    if read(output/'provenance.json')['request_sha256']!=digest(run/'mpc_request.json'):raise ValueError('worker request changed')
    write(run/'evaluation_started.json',dict(utc=utc(),new_mpc_solves=0))
    start=time.perf_counter();env=HospitalEnvironment.load(source['environment_path']);environment_s=time.perf_counter()-start
    begin=time.perf_counter();config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    manifest=read(run/'case_manifest.json')['selected']
    outcomes,selections,matches,closed,divergent,timings=[],[],[],[],[],[]
    reproductions={};unavailable=[]
    for case in manifest:
        folder=run/case['relative_directory'];worker=output/case['case_directory']
        context,goal=read(folder/'input_context.json'),read(folder/'goal_route.json')
        for name in ('matched_state_probes.json','ledger.json','input_validation.json'):
            copy_new(worker/name,folder/name)
        probes=read(folder/'matched_state_probes.json');actual={};historical={};commands={}
        timings.append(dict(case_id=case['case_id'],method=None,stage='matched_selector_probes',scope='case',
            wall_s=probes['wall_s'],included_in='worker_process',selector_calls=probes['selector_calls_completed'],mpc_solves=0))
        for method in METHODS:
            target=folder/'methods'/method
            for path in sorted((worker/method).iterdir()):
                if path.is_file():copy_new(path,target/path.name)
            status=read(target/'worker_status.json')
            if status['completed']:
                rollout=read(target/'rollout.json');ref=np.load(target/'reference_world.npy',allow_pickle=False)
                if rollout['candidate_source']['sha256']!=digest(target/'reference_world.npy'):raise ValueError('actual path file hash differs')
                metrics=evaluate_method(method,rollout,context,ref,goal,env,config,case_metadata=case)
                actual[method]=dict(rollout=rollout,metrics=metrics)
                commands[method]=rollout['controller_reference_selections']
                selections.append(dict(case_id=case['case_id'],method=method,mode='closed_loop',available=True,
                    **selection_mechanism(commands[method])))
                timings.append(dict(case_id=case['case_id'],method=method,stage='independent_evaluation',scope='method',
                    wall_s=metrics['independent_evaluation_wall_s'],included_in='evaluation_total',mpc_solves=0))
            else:
                reason=status.get('technical_error') or status['status']
                metrics=missing_outcome(case['case_id'],method,attempted=status['attempted'],reason=reason,case_metadata=case)
                actual[method]=None;commands[method]=None
                selections.append(dict(case_id=case['case_id'],method=method,mode='closed_loop',available=False,reason=reason))
                unavailable.append(dict(case_id=case['case_id'],method=method,reason=reason))
            write(target/'metrics.json',metrics);outcomes.append(metrics)
            timings.append(dict(case_id=case['case_id'],method=method,stage='rollout',scope='method',
                wall_s=status.get('rollout_wall_s'),included_in='worker_process',mpc_solves=status['actual_controller_calls']))
            timings.append(dict(case_id=case['case_id'],method=method,stage='official_mpc_solve',scope='method',
                wall_s=status.get('official_mpc_solve_wall_s'),included_in='rollout',mpc_solves=status['actual_controller_calls']))
            old=folder/'historical'/method
            if old.exists():historical[method]=dict(rollout=read(old/'rollout.json'),metrics=read(old/'metrics.json'))
        for method in METHODS[1:]:
            if probes['status']=='COMPLETED':
                records=[dict(time_s=p['time_s'],selection_diagnostic=p['methods'][method]) for p in probes['states']]
                selections.append(dict(case_id=case['case_id'],method=method,mode='matched_state',available=True,**selection_mechanism(records)))
            else:
                selections.append(dict(case_id=case['case_id'],method=method,mode='matched_state',available=False,
                    reason=probes['unavailable_reason']))
        if probes['status']=='COMPLETED':matches.extend(compare_selections(case['case_id'],probes['states'],matched=True))
        if all(commands.values()):closed.extend(compare_selections(case['case_id'],commands,matched=False))
        divergent.extend(first_command_divergences(case['case_id'],commands))
        if case['cohort']=='KNOWN_CONTROLS':
            kind='GP-SE2-01' if case['case_role']=='KNOWN_OBSTACLE_STRESS' else 'GP-SE2-REF-02'
            reproduction=control_reproduction(case['case_id'],actual,historical,source_kind=kind)
            write(folder/'reproduction_check.json',reproduction);reproductions[case['case_id']]=reproduction
    aggregate=aggregate_outcomes(outcomes,manifest);worker_summary=read(output/'summary.json')
    reproduction_pass=all(r['passed'] for r in reproductions.values())
    complete=not unavailable and worker_summary['probe_cases_completed']==len(manifest)
    summary=dict(**aggregate,experiment='GP-SE2-REF-03',
        operational_status='GP_SE2_REF_03_COMPLETED' if complete and reproduction_pass else 'GP_SE2_REF_03_COMPLETED_WITH_LIMITATIONS',
        research_interpretation='PENDING_REGRESSION_FIRST_REVIEW',
        interpretation_record='aggregate/research_interpretation.json is written after reviewing every regression and coverage table; no new solves',
        controls_reproduced=reproduction_pass,control_reproductions=reproductions,
        complete_paired_evidence=complete,unavailable_methods=unavailable,
        fixed_selector_and_mpc_core_preserved=True,MPC_calculation_unchanged=True,reference_selector_changed_between_B_C=True,
        external_source_modified=False,GP_or_rigid_optimization_solves=0,new_VLA_updates=0,
        optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC',
        compute=dict(preparation_total_wall_s=source['preparation_total_wall_s'],
            preparation_preservation_hash_audit_wall_s=source['preservation_hash_audit_wall_s'],
            preparation_environment_loading_wall_s=source['environment_loading_wall_s'],
            preparation_source_cohort_audit_wall_s=source['cohort_audit_wall_s'],
            evaluation_environment_loading_wall_s=environment_s,evaluation_total_wall_s=time.perf_counter()-begin,
            worker_process_wall_s=read(run/'execution_completed.json')['worker_process_wall_s'],worker=worker_summary,
            profiling_definition='nested components are labeled included_in; do not sum inclusive timing rows'),
        status_requirement='completion additionally requires authoritative validation.json; scientific failure is not artifact corruption',
        claim_limit='frozen selector transfer within selected development corpus; not GP, online navigation, population safety or frequency evidence')
    for name,values in [('outcome_matrix',aggregate['ledger']),('success_patterns',aggregate['event_classifications']),
        ('regressions',aggregate['reason_comparisons']),('recoveries',[e for e in aggregate['event_classifications'] if any(
            e['overlapping_flags'][k] is True for k in ('A_success_B_failure_C_success','B_failure_C_success','A_B_failure_C_success'))]),
        ('paired_metrics',aggregate['paired_quality']),('failed_pair_endpoint_diagnostics',aggregate['failed_pair_endpoint_diagnostics']),
        ('selection_mechanism',selections),('matched_state_pairwise',matches),('closed_loop_pairwise',closed),
        ('command_divergence',divergent),('timing',timings)]:
        write(run/'aggregate'/f'{name}.json',values);table(run/'aggregate'/f'{name}.csv',values)
    write(run/'aggregate/duplicate_summary.json',scope_exports(aggregate,'duplicate_summary'))
    write(run/'aggregate/paired_quality_summary.json',scope_exports(aggregate,'paired_quality_equal_weight'))
    write(run/'aggregate/outcomes.json',outcomes);write(run/'aggregate/summary.json',summary)
    write(run/'evaluation_completed.json',dict(utc=utc(),methods=len(outcomes),controls_reproduced=reproduction_pass,new_mpc_solves=0))
    print(clean(dict(phase='EVALUATED',methods=len(outcomes),controls_reproduced=reproduction_pass,
        additional_patterns=aggregate['additional_transfer']['exclusive_patterns'])),flush=True)


def finalize(run):
    _,source=verify(run);before=read(run/'preservation_before.json');after=preservation(source['primary_source'])
    for key in ('files','user_config_hashes','external_git_sha','external_git_status'):
        if before[key]!=after[key]:raise ValueError('original preservation failed: '+key)
    if not after['valid']:raise ValueError('original hash mismatch')
    write(run/'preservation_after.json',after)
    write(run/'implementation_final.json',dict(utc=utc(),source_sha256=snapshot(run,'final')))
    files={str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file() and p.name not in ('validation.json','artifact_manifest.json','git_completion.json')}
    write(run/'artifact_manifest.json',dict(files=files,created_utc=utc()))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('phase',choices=['prepare','freeze','execute','evaluate','finalize'])
    parser.add_argument('--run',required=True,type=Path);parser.add_argument('--config',type=Path,default=ROOT/'configs/gp_se2_ref03.yaml')
    args=parser.parse_args();run=args.run.resolve()
    try:
        if args.phase=='prepare':prepare(run,args.config.resolve())
        else:globals()[args.phase](run)
    except Exception as exc:
        if run.exists() and not (run/f'technical_failure_{args.phase}.json').exists():
            write(run/f'technical_failure_{args.phase}.json',dict(utc=utc(),error=f'{type(exc).__name__}: {exc}',primary_retry_forbidden=True))
        raise


if __name__=='__main__':main()
