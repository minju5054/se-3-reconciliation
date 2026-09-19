#!/usr/bin/env python3
"""Audit saved REF-02 evidence independently; no new MPC or optimizer solves.

Scientific non-recovery is a valid result. Inconsistent inputs, selection,
integration, evaluation or reporting are artifact validation failures.
"""
from __future__ import annotations
import argparse
import ast
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

from validate_gp_se2_ref01 import Audit, audit_csv, audit_lineage, digest, read
from reconciliation.gp_se2_ref02_reference import METHODS, enrich_selector_result
from reconciliation.gp_se2_ref02_evaluation import (
    FIXED_CASES, CONTRAST_METRICS, evaluate_method, paired_comparisons, selection_mechanism,
    compare_selections, first_command_divergences, result_flags, reproduction_check,
)
from reconciliation.gp_se2_02_evaluation import validate_counterfactual
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_rollout import load_frozen_context
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.se2 import wrap_angle

PLOTS = ('fixed_input_world', 'matched_state_targets', 'selected_source_progress_vs_time',
         'selected_target_yaw_vs_time', 'goal_in_horizon_vs_time', 'actual_trajectory_overlay',
         'actual_yaw_and_goal_error', 'linear_command_vs_time', 'angular_command_vs_time',
         'clearance_vs_time', 'outcome_summary')


def official_pure(source_path):
    """Hash-pinned pure selector/frame transforms only: no runtime import."""
    path = Path(source_path)
    if digest(path) != PINNED_MPC_SHA256:
        raise ValueError('official MPC source hash mismatch')
    names = {'wrap_angle', 'build_pose_aligned_reference', 'project_body_to_world', 'project_world_to_local'}
    functions = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name in names]
    if len(functions) != len(names):
        raise ValueError('official pure function coverage mismatch')
    scope = dict(np=np, math=math, Sequence=list)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), scope)
    return scope


def independent_targets(pure, path, pose, lineage, method, weights):
    """Independent C lower-bound scan; A/B call the unchanged pure official code."""
    points, state = np.asarray(path, dtype=float), np.asarray(pose, dtype=float)
    errors = points-state
    errors[:, 2] = [math.atan2(math.sin(float(a)), math.cos(float(a))) for a in errors[:, 2]]
    costs = np.sum(errors*errors*np.asarray(weights), axis=1)
    j = int(np.argmin(costs))
    s = [float(row['original_fractional_row_coordinate']) for row in lineage]
    q = None
    if method == METHODS[2]:
        if len(s) != len(points) or not np.isfinite(s).all() or any(b <= a for a,b in zip(s,s[1:])):
            raise ValueError('fixed actual REF-02 inputs require strictly increasing saved lineage')
        q = [min(s[j]+h, s[-1]) for h in range(1, 6)]
        indices = [next(i for i, value in enumerate(s) if value >= target) for target in q]
        reference = points[indices].copy()
        previous = float(state[2])
        for row in reference:
            difference = float(row[2]-previous)
            previous += math.atan2(math.sin(difference), math.cos(difference))
            row[2] = previous
    elif method in METHODS[:2]:
        indices = [min(j+h, len(points)-1) for h in range(1,6)]
        reference = pure['build_pose_aligned_reference'](points, state, horizon=5, weights=weights)
    else:
        raise ValueError('unknown method: no baseline fallback')
    return dict(reference=reference, indices=indices, nearest=j, costs=costs, q=q, progress=s)


def audit_selection(audit, pure, path, pose, lineage, saved, weights, method, label):
    expected = independent_targets(pure, path, pose, lineage, method, weights)
    enriched = enrich_selector_result(path, pose, expected['reference'], lineage, method=method,
        weights=weights, horizon=5, original_goal_row_index=saved['original_goal_row_index'])
    audit.same(saved, enriched, label+'/complete selector diagnostic')
    audit.same(saved['nearest_index'], expected['nearest'], label+'/independent nearest', 0.)
    audit.same(saved['indices'], expected['indices'], label+'/independent target indices', 0.)
    audit.same(saved['reference_world'], expected['reference'], label+'/actual selected reference', 0.)
    audit.same(saved['weighted_total_pose_distances'], expected['costs'], label+'/pinned nearest costs', 0.)
    audit.same(saved['final_goal_row_in_horizon'], len(path)-1 in expected['indices'], label+'/final goal inclusion')
    if method == METHODS[2]:
        audit.same(saved['q_h'], expected['q'], label+'/independent requested q', 0.)
        audit.same(saved['progress_overshoot'], [expected['progress'][i]-q for i,q in zip(expected['indices'],expected['q'])],
                   label+'/independent quantization overshoot', 0.)
    else:
        audit.same(saved['q_h'], None, label+'/no invented baseline requested progress')
    return expected


def audit_rollout(audit, pure, rollout, context, reference, lineage, weights, method, ordinal, label):
    audit.capture(label+'/original integration/start/schedule checker', lambda: validate_counterfactual(rollout, context, reference))
    for key,value in [('primary_execution_ordinal',ordinal),('independent_tracker_instance',True),
                      ('GP_or_rigid_optimization_performed',False),('method',method),
                      ('actual_selector_calls',30),('actual_controller_calls',30)]:
        audit.same(rollout[key], value, label+'/'+key)
    audit.check(bool(rollout['wrapper_identity']) and all(v is True for v in rollout['wrapper_identity'].values()), label+'/unchanged code identities')
    installed = pure['project_body_to_world'](rollout['candidate_capture_local'], context['original_capture_pose_world'])
    audit.same(rollout['installed_reference_world'], installed, label+'/capture-frame installation', 0.)
    installed_hash = hashlib.sha256(np.asarray(installed,dtype='<f8').tobytes(order='C')).hexdigest()
    audit.same(rollout['installed_path_sha256'], installed_hash, label+'/installed array hash')
    solves = rollout['controller_reference_selections']
    audit.same(len(solves),30,label+'/control solve count')
    for i, solve in enumerate(solves):
        name=label+f'/solve {i}'
        audit.same(solve['input_pose_world'],rollout['states'][i*6]['pose_world'],name+'/input state',0.)
        expected_memory = context['previous_control'] if i==0 else solves[i-1]['previous_control_after']
        audit.same(solve['previous_control'],expected_memory,name+'/recorded memory',0.)
        audit.same(solve['previous_control_after'],solve['command'],name+'/official poll memory',0.)
        audit.same(solve['installed_path_sha256'],installed_hash,name+'/installed hash')
        expected = audit_selection(audit,pure,installed,solve['input_pose_world'],lineage,solve['selection_diagnostic'],weights,method,name)
        for key in ('nearest_index','indices','reference_world','endpoint_repeated','nearest_is_final_row'):
            audit.same(solve['selection'][key],solve['selection_diagnostic'][key],name+'/saved basic audit '+key)
        audit.same(solve['actual_submitted_reference_world'],expected['reference'],name+'/actual submitted reference',0.)
        local = pure['project_world_to_local'](expected['reference'], solve['input_pose_world'])
        audit.same(solve['actual_controller_reference_local'],local,name+'/actual solver local reference',0.)
        audit.same(solve['actual_controller_input_pose_local'],[0.,0.,0.],name+'/original local input pose',0.)
        audit.same(solve['actual_controller_previous_control'],expected_memory,name+'/actual solver memory',0.)
        audit.same(solve['actual_reference_capture_complete'],True,name+'/actual capture')
        audit.same(solve['simulation_time_advanced_during_solve_s'],0.,name+'/offline time')
        if solve['success']:
            audit.same(solve['result_reference_world'],expected['reference'],name+'/actual official result reference',0.)
            prediction=np.asarray(solve['prediction_world'])
            audit.check(prediction.shape==(6,3) and np.isfinite(prediction).all(),name+'/finite MPC prediction')
        else:
            audit.same(solve['command'],[0.,0.],name+'/unchanged failure command policy',0.)
    audit.same(rollout['controller_failure_count'],sum(not r['success'] for r in solves),label+'/failure count')


def independent_paired(rows):
    by={(r['case_id'],r['method']):r for r in rows}
    expected={(c,m) for c in FIXED_CASES for m in METHODS}
    if len(by)!=len(rows) or set(by)!=expected:
        raise ValueError('six unique planned outcomes required')
    result=[]
    for case in FIXED_CASES:
        for base,method in [(METHODS[1],METHODS[2]),(METHODS[0],METHODS[2]),(METHODS[0],METHODS[1])]:
            a,b=by[case,base],by[case,method]
            for metric in CONTRAST_METRICS:
                x,y=a[metric],b[metric]
                result.append(dict(case_id=case,baseline=base,method=method,baseline_success=a['primary_success'],
                    method_success=b['primary_success'],both_success=bool(a['primary_success'] and b['primary_success']),
                    metric=metric,baseline_value=x,method_value=y,difference=None if x is None or y is None else y-x,
                    interpretation='fixed-event descriptive difference; a failure is not a quality improvement'))
    return result


def audit_plots(audit,run,case,context,route,methods,probes,config):
    from plot_gp_se2_ref02 import expected_numeric,input_identity
    import csv
    past=read(case/'actual_past_execution.json')
    audit.same(past['source_sha256'],digest(past['source_path']),'actual past source hash')
    audit.same(past['poses_world'][-1],context['B_world'],'actual past ends at B',0.)
    with Path(past['source_path']).open() as stream:
        history={int(r['state_id']):r for r in csv.DictReader(stream)}
    for i,idx in enumerate(past['state_ids']):
        row=history[idx]
        audit.same(past['poses_world'][i],[float(row[k]) for k in ('x','y','yaw')],f'past original pose {i}',0.)
        audit.same(past['times_relative_to_B_s'][i],float(row['sim_time_s'])-context['switch_sim_time_s'],f'past original time {i}',0.)
    values=expected_numeric(context,route,methods,past,probes,config)
    identity=input_identity(methods)
    for name in PLOTS:
        path=case/'plots'/f'{name}.png'; side=read(path.with_suffix('.json'))
        audit.same(digest(path),side['image_sha256'],name+'/PNG hash')
        audit.same(side['numeric_data'],values[name],name+'/numerical sidecar')
        audit.same(side['input_identity'],identity,name+'/B/C fixed input identity')
        audit.same(side['plotter_sha256'],digest(ROOT/'scripts/plot_gp_se2_ref02.py'),name+'/plot code hash')
        for p,h in side['source_hashes'].items():
            audit.same(digest(p),h,name+'/source '+p)
        for field in ('new_inference','new_optimization','new_execution','gui_runtime_validated'):
            audit.same(side[field],False,name+'/'+field)
        audit.same(side['label'],'OFFLINE LOOKAHEAD-SELECTION DIAGNOSTIC',name+'/label')
    return len(values)


def audit_bundle(audit,run):
    bundle=run/'review_bundle'; manifest=read(bundle/'manifest.json')
    allowed={r['path']:r for r in manifest['allowlisted_files']}
    expected={'source.json','protocol.json','selector_definition.json','config_snapshot.yaml','case_manifest.json',
              'plot_manifest.json','README.md','index.html','manifest.json'}
    expected.update('aggregate/'+p for p in ('outcomes.csv','paired_comparison.csv','selection_mechanism.csv','timing.csv','summary.json'))
    for case in read(run/'case_manifest.json')['selected']:
        base='cases/'+case['case_directory']+'/'
        expected.add(base+'reproduction_check.json')
        expected.update(base+'plots/'+p+ext for p in PLOTS for ext in ('.png','.json'))
    audit.same(sorted(set(allowed)|{'manifest.json'}),sorted(expected),'bundle strict allowlist')
    audit.same(sorted(str(p.relative_to(bundle)) for p in bundle.rglob('*') if p.is_file()),sorted(expected),'bundle files')
    for name,row in allowed.items():
        audit.check(not Path(name).is_absolute() and '..' not in Path(name).parts,'bundle path safe '+name)
        audit.same(digest(bundle/name),row['sha256'],'bundle hash '+name)
        if row['source']!='generated review text':
            audit.same(digest(row['source']),row['sha256'],'bundle source '+name)
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        audit.same(sorted(archive.namelist()),sorted(expected),'ZIP exact allowlist')
        audit.check(len(archive.namelist())==len(set(archive.namelist())),'ZIP unique members')
        for name in archive.namelist():
            audit.same(hashlib.sha256(archive.read(name)).hexdigest(),digest(bundle/name),'ZIP content '+name)
    outer=read(run/'review_bundle_manifest.json')
    audit.same(outer['bundle_sha256'],digest(run/'review_bundle.zip'),'ZIP hash')
    audit.same(outer['image_count'],22,'ZIP coverage')
    audit.same(outer['gui_runtime_validated'],False,'GUI optional not performed')


def validate(run, *, write_output=True, output=None, require_finalized=True):
    run=Path(run).resolve()
    target=Path(output) if output is not None else run/('validation.json' if require_finalized else 'verification/preflight_validation.json')
    if write_output and target.exists():
        raise FileExistsError('refusing authoritative validation overwrite: '+str(target))
    started=time.perf_counter(); audit=Audit(); deferred=[]; case_reports=[]
    def global_checks():
        from run_gp_se2_ref02 import validate_config,SELECTOR_DEFINITION
        source,protocol=read(run/'source.json'),read(run/'protocol.json')
        experimental=validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()))
        config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
        audit.same(read(run/'selector_definition.json'),SELECTOR_DEFINITION,'frozen exact selector definition')
        audit.same(protocol['selector_definition'],SELECTOR_DEFINITION,'protocol selector definition')
        audit.same(digest(run/'config_snapshot.yaml'),source['original_config_sha256'],'authoritative config hash')
        audit.same(digest(run/'config_snapshot.yaml'),digest(Path(source['primary_source'])/'config_snapshot.yaml'),'original acceptance unchanged')
        audit.same(digest(run/'experiment_config.yaml'),source['experiment_config_sha256'],'experiment config hash')
        for path,h in source['authoritative_validations'].items():
            audit.same(digest(path),h,'prior authoritative validation hash '+path)
            audit.same(read(path)['valid'],True,'prior authoritative validation '+path)
        for path,h in source['copied_source_sha256'].items():
            audit.same(digest(path),h,'original copied source '+path)
        for path,h in source['preserved_core_sha256'].items():
            audit.same(digest(ROOT/path),h,'original core '+path)
        freeze=read(run/'execution_freeze.json')
        audit.same(freeze['no_primary_execution_yet'],True,'frozen before execution')
        for path,h in freeze['source_sha256'].items():
            audit.same(digest(ROOT/path),h,'execution source '+path)
        for path,h in freeze['input_sha256'].items():
            audit.same(digest(run/path),h,'execution input '+path)
        before=read(run/'preservation_before.json')
        if (run/'preservation_after.json').exists() or require_finalized:
            after=read(run/'preservation_after.json')
            for key in ('files','user_config_hashes','external_git_sha','external_git_status'):
                audit.same(before[key],after[key],'before/after preservation '+key)
        else:
            after=before; deferred.append('final preservation_after.json')
        for path,h in after['files'].items(): audit.same(digest(path),h,'preserved current '+path)
        for path,h in after['user_config_hashes'].items(): audit.same(digest(ROOT/path),h,'user config current '+path)
        if (run/'artifact_manifest.json').exists() or require_finalized:
            for path,h in read(run/'artifact_manifest.json')['files'].items():
                audit.same(digest(run/path),h,'final artifact '+path)
        else: deferred.append('final artifact manifest')
        cases=read(run/'case_manifest.json')['selected']
        audit.same([r['case_id'] for r in cases],list(FIXED_CASES),'fixed case order')
        audit.same(sorted(p.name for p in (run/'cases').iterdir() if p.is_dir()),sorted(c['case_directory'] for c in cases),'case folder coverage')
        audit.same(protocol['execution_order'],[dict(case_id=c,method=m) for c in FIXED_CASES for m in METHODS],'fixed execution order')
        for key,value in [('matched_state_probe_count',180),('primary_rollout_count',6),('primary_mpc_solve_count',180),('historical_solve_count',0)]:
            audit.same(protocol[key],value,'protocol counts '+key)
        mpc=read(run/'mpc_output/provenance.json')
        audit.same(mpc['official_settings'],source['official_mpc']['official_settings'],'official MPC settings unchanged')
        audit.same(mpc['official_settings_sha256'],hashlib.sha256(json.dumps(mpc['official_settings'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),'actual settings hash')
        audit.same(mpc['request_sha256'],digest(run/'mpc_request.json'),'actual request hash')
        audit.same(read(run/'mpc_output/request.json'),read(run/'mpc_request.json'),'actual request values')
        for row in read(run/'mpc_output/output_hashes.json')['files']:
            audit.same(digest(run/'mpc_output'/row['path']),row['sha256'],'worker output '+row['path'])
        for row in read(run/'mpc_output/input_hashes.json')['files']:
            audit.same(digest(row['path']),row['sha256'],'worker input '+row['path'])
        totals=read(run/'mpc_output/summary.json')
        for key,value in dict(independent_rollouts=6,primary_mpc_solves=180,actual_rollout_selector_calls=180,
            actual_controller_calls=180,historical_or_other_mpc_solves=0,matched_state_selector_calls=180,
            new_VLA_updates=0,GP_or_rigid_optimization_solves=0).items():
            audit.same(totals[key],value,'actual worker counts '+key)
        audit.same(read(run/'mpc_output/probes_completed.json')['mpc_solves'],0,'probes before rollouts no solves')
        audit.same(read(run/'execution_completed.json')['exit_code'],0,'primary normal completion')
        return source,config,experimental,cases,mpc,official_pure(mpc['mpc_source']),HospitalEnvironment.load(source['environment_path'])
    loaded=audit.capture('global source/runtime',global_checks)
    outcomes=[]; rows=[]; mechanisms=[]; matched_pairs=[]; closed_pairs=[]; divergences=[]; timings=[]; reproduced={}; fixed_records=[]
    if loaded:
        source,config,experimental,cases,mpc,pure,env=loaded; weights=mpc['official_settings']['Q_WEIGHTS']
        for case_index,entry in enumerate(cases):
            start_checks,start_errors=audit.checks,len(audit.errors)
            def case_checks():
                from reconciliation.gp_se2_rollout import candidate_in_capture_frame
                case=run/'cases'/entry['case_directory']; context=read(case/'input_context.json'); route=read(case/'goal_route.json')
                source_case=Path(source['primary_source'])/'cases'/entry['case_directory']
                audit.same(context,load_frozen_context(source['source_run'],context['episode_id'],context['handoff_id']),'original raw context',0.)
                audit.same(route,read(source_case/'goal_route.json'),'original goal route',0.)
                audit.same(route['gates'],[],'original no-gate scope')
                audit.same(sorted(p.name for p in (case/'methods').iterdir() if p.is_dir()),sorted(METHODS),'method folder coverage')
                probes=read(case/'matched_state_probes.json')
                audit.same(probes,read(run/'mpc_output'/entry['case_directory']/'matched_state_probes.json'),'actual worker probes',0.)
                historical=read(case/'historical/original_native_probe_rollout.json')
                old_probes=read(case/'historical/matched_state_probes.json')
                for key,value in [('new_mpc_solves',0),('state_integration_performed',False),('controller_memory_modified',False),
                                  ('probe_count',30),('method_selection_count',90),('B_C_fixed_reference_and_nearest_preserved',True)]:
                    audit.same(probes[key],value,'matched probe '+key)
                audit.same(len(probes['states']),30,'all thirty historical poses')
                methods={}; reproduction=dict(case_id=entry['case_id'],methods={}); actual={}
                for method_index,method in enumerate(METHODS):
                    folder=case/'methods'/method; reference=np.load(folder/'reference_world.npy',allow_pickle=False)
                    lineage=read(folder/'row_provenance.json'); original_variant='R00_NATIVE' if method==METHODS[0] else 'R11_CURRENT_ADAPTER'
                    prior=source_case/'variants'/original_variant
                    for name in ('reference_world.npy','row_provenance.json','geometry_audit.json','definition.json'):
                        audit.same(digest(folder/name),digest(prior/name),method+'/byte-identical REF01 '+name)
                    audit.same(reference.dtype.str,np.dtype('float64').str,method+'/source dtype')
                    audit.same(len(reference),len(context['fresh_world']) if method==METHODS[0] else 30,method+'/source rows')
                    audit_lineage(audit,np.asarray(context['fresh_world']),reference,lineage,method)
                    # Probe the exact original installation arithmetic, with zero tracker construction.
                    local,transform=candidate_in_capture_frame(reference,context['original_capture_pose_world'])
                    installed=pure['project_body_to_world'](local,context['original_capture_pose_world'])
                    install=probes['installation'][method]
                    audit.same(install['candidate_capture_local'],local,method+'/probe capture inverse',0.)
                    audit.same(install['candidate_frame_transform'],transform,method+'/probe capture transform')
                    audit.same(install['installed_reference_world'],installed,method+'/probe installed path',0.)
                    audit.same(install['source_reference_array_sha256'],hashlib.sha256(np.asarray(reference,dtype='<f8').tobytes()).hexdigest(),method+'/probe source array hash')
                    audit.same(install['installed_path_sha256'],hashlib.sha256(np.asarray(installed,dtype='<f8').tobytes()).hexdigest(),method+'/probe installed hash')
                    audit.same(install['source_reference_file']['sha256'],digest(folder/'reference_world.npy'),method+'/probe source file hash')
                    for i,probe in enumerate(probes['states']):
                        audit.same(probe['input_pose_world'],historical['controller_reference_selections'][i]['input_pose_world'],method+f'/original probe state {i}',0.)
                        audit.same(probe['input_pose_world'],old_probes['states'][i]['input_pose_world'],method+f'/REF01 probe state {i}',0.)
                        audit.same(probe['time_s'],i*6/60,method+f'/probe time {i}',0.)
                        audit_selection(audit,pure,installed,probe['input_pose_world'],lineage,probe['methods'][method],weights,method,method+f'/matched {i}')
                    rollout,metrics=read(folder/'rollout.json'),read(folder/'metrics.json')
                    audit.same(rollout,read(run/'mpc_output'/entry['case_directory']/method/'rollout.json'),method+'/actual worker rollout',0.)
                    audit.same(rollout['installed_reference_world'],installed,method+'/probe and actual same installation',0.)
                    audit_rollout(audit,pure,rollout,context,reference,lineage,weights,method,case_index*3+method_index,method)
                    for solve in rollout['controller_reference_selections']:
                        audit.same(solve['actual_controller_v_max'],mpc['official_settings']['OBJNAV_V_MAX'],method+'/original solver speed limit')
                    recomputed=evaluate_method(method,rollout,context,reference,route,env,config)
                    recomputed['summary']['case_role']=entry['case_role']
                    for key,value in recomputed.items():
                        if key!='independent_evaluation_wall_s': audit.same(metrics[key],value,method+'/original evaluation '+key)
                    outcomes.append(metrics); rows.append(metrics['summary']); actual[method]=rollout['controller_reference_selections']
                    for mode,records in [('matched_state',[dict(time_s=p['time_s'],selection_diagnostic=p['methods'][method]) for p in probes['states']]),('closed_loop',actual[method])]:
                        mechanisms.append(dict(case_id=entry['case_id'],method=method,mode=mode,**selection_mechanism(records)))
                    if method!=METHODS[2]:
                        report=reproduction_check(rollout,read(case/'historical'/method/'rollout.json'),metrics,read(case/'historical'/method/'metrics.json'))
                        report['comparison']='first prescribed REF-02 primary versus preserved REF-01 actual rollout'
                        reproduction['methods'][method]=report
                    timings.append(dict(case_id=entry['case_id'],method=method,mpc_solves=len(actual[method]),rollout_wall_s=rollout['rollout_wall_s'],
                        official_mpc_solve_wall_s=sum(v for v in rollout['official_mpc_solve_wall_s'] if v is not None),
                        independent_evaluation_wall_s=metrics['independent_evaluation_wall_s']))
                    methods[method]=dict(folder=folder,reference=reference,provenance=lineage,rollout=rollout,metrics=metrics)
                b,c=[methods[m] for m in METHODS[1:]]
                bx,cx=b['folder'],c['folder']; x,y=b['reference'],c['reference']
                fixed=dict(case_id=entry['case_id'],B_reference_sha256=digest(bx/'reference_world.npy'),C_reference_sha256=digest(cx/'reference_world.npy'),
                    B_lineage_sha256=digest(bx/'row_provenance.json'),C_lineage_sha256=digest(cx/'row_provenance.json'),shape=list(x.shape),dtype=str(x.dtype),
                    byte_identical=x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes(),file_identical=(bx/'reference_world.npy').read_bytes()==(cx/'reference_world.npy').read_bytes(),
                    lineage_identical=(bx/'row_provenance.json').read_bytes()==(cx/'row_provenance.json').read_bytes(),native_row_count=len(context['fresh_world']),dense_row_count=len(x))
                audit.same(read(case/'fixed_dense_reference.json'),fixed,'fixed input proof')
                audit.check(all(fixed[k] for k in ('byte_identical','file_identical','lineage_identical')),'B/C exact fixed inputs')
                fixed_records.append(fixed)
                audit.same(b['rollout']['installed_reference_world'],c['rollout']['installed_reference_world'],'B/C exact installed rollout path',0.)
                for probe in probes['states']:
                    for key in ('nearest_index','nearest_original_fractional_row_coordinate','weighted_total_pose_distances','weighted_xy_contributions','weighted_yaw_contributions'):
                        audit.same(probe['methods'][METHODS[1]][key],probe['methods'][METHODS[2]][key],'B/C exact same-state '+key,0.)
                reproduction['passed']=all(r['passed'] for r in reproduction['methods'].values())
                audit.same(read(case/'reproduction_check.json'),reproduction,'complete baseline reproduction report')
                reproduced[entry['case_id']]=reproduction
                matched_pairs.extend(compare_selections(entry['case_id'],probes['states'],matched=True))
                closed_pairs.extend(compare_selections(entry['case_id'],actual,matched=False))
                divergences.extend(first_command_divergences(entry['case_id'],actual,atol=experimental['command_divergence_atol']))
                if (case/'plots/outcome_summary.json').exists() or require_finalized:
                    audit_plots(audit,run,case,context,route,methods,probes,config)
                else: deferred.append('all plots '+entry['case_id'])
            audit.capture('case '+entry['case_id'],case_checks)
            case_reports.append(dict(case_id=entry['case_id'],valid=len(audit.errors)==start_errors,checks=audit.checks-start_checks,errors=audit.errors[start_errors:]))
        if len(rows)==6:
            audit.same(read(run/'aggregate/fixed_dense_reference.json'),fixed_records,'aggregate fixed input proofs')
            comparisons=independent_paired(rows)
            for name,values in [('outcomes',outcomes),('paired_comparison',comparisons),('selection_mechanism',mechanisms),
                                ('matched_state_pairwise',matched_pairs),('closed_loop_pairwise',closed_pairs),('command_divergence',divergences),('timing',timings)]:
                audit.capture(name+' JSON',lambda n=name,v=values:audit.same(read(run/'aggregate'/f'{n}.json'),v,n))
                audit.capture(name+' CSV',lambda n=name,v=values:audit_csv(audit,run/'aggregate'/f'{n}.csv',rows if n=='outcomes' else v,n))
            summary=read(run/'aggregate/summary.json')
            flags=result_flags(rows,reproduced,matched_pairs,fixed_dense_preserved=all(r['byte_identical'] and r['file_identical'] and r['lineage_identical'] for r in fixed_records),selection_validated=True)
            audit.same(summary['result'],flags,'result flags recomputed without success expectation')
            audit.same(summary['operational_status'],'GP_SE2_REF_02_COMPLETED' if flags['baseline_reproduced'] else 'GP_SE2_REF_02_COMPLETED_WITH_LIMITATIONS','operational status')
            for key,value in dict(primary_rollouts=6,primary_mpc_solves=180,matched_state_selector_calls=180,historical_or_other_mpc_solves=0,
                MPC_calculation_unchanged=True,reference_selector_changed=True,external_source_modified=False,GP_or_rigid_optimization_solves=0,
                new_VLA_updates=0,optional_GUI_status='NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY').items():
                audit.same(summary[key],value,'summary '+key)
        else: audit.check(False,'all six planned evaluated outcomes required')
        if (run/'review_bundle.zip').exists() or require_finalized: audit.capture('review bundle',lambda:audit_bundle(audit,run))
        else: deferred.append('review bundle')
        if (run/'plot_manifest.json').exists() or require_finalized:
            def plot_manifest():
                manifest=read(run/'plot_manifest.json')
                audit.same(manifest['image_count'],22,'22 primary PNGs')
                expected={f"cases/{entry['case_directory']}/plots/{p}.png" for entry in cases for p in PLOTS}
                audit.same({r['path'] for r in manifest['images']},expected,'exact required plot coverage')
            audit.capture('plot coverage',plot_manifest)
        else: deferred.append('plot manifest')
    report=dict(experiment='GP-SE2-REF-02',valid=not audit.errors,checks=audit.checks,errors=audit.errors,
        authoritative=require_finalized,deferred_checks=deferred,case_validations=case_reports,
        validation_scope='complete finalized saved artifacts' if require_finalized else 'non-authoritative preflight; final checks deferred',
        scientific_failures_are_not_validation_failures=True,independent_saved_evidence_only=True,
        optimizer_or_MPC_solves_performed=0,official_selector_verification='hash-pinned pure AST functions; independent C linear lower-bound scan; actual solver arguments',
        validation_wall_s=time.perf_counter()-started,validator_sha256=digest(__file__),generated_utc=datetime.now(timezone.utc).isoformat())
    if write_output:
        target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('x') as stream: json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--output',type=Path);parser.add_argument('--preflight',action='store_true')
    args=parser.parse_args(); result=validate(args.run,output=args.output,require_finalized=not args.preflight)
    print(json.dumps({k:v for k,v in result.items() if k!='case_validations'},indent=2))
    raise SystemExit(0 if result['valid'] else 1)
