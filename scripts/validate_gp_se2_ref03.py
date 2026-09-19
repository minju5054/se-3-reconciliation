#!/usr/bin/env python3
"""Independent saved-record REF-03 validator; never invokes MPC or GP solves.

Scientific failures, gates and unavailable calculations are retained as results.
Validation fails for inconsistent source, selection, integration or reporting.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import FunctionType
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml

from validate_gp_se2_ref01 import Audit, audit_csv, audit_lineage, digest, read
from validate_gp_se2_ref02 import official_pure, audit_rollout as _ref02_audit_rollout
from reconciliation.gp_se2_ref02_reference import METHODS, enrich_selector_result
from reconciliation.gp_se2_ref03_evaluation import (
    evaluate_method, missing_outcome, aggregate_outcomes, selection_mechanism,
    compare_selections, first_command_divergences, control_reproduction,
)
from reconciliation.gp_se2_rollout import candidate_in_capture_frame, load_frozen_context
from reconciliation.gp_se2_environment import HospitalEnvironment


def independent_targets(pure, path, pose, lineage, method, weights, metadata=None):
    """Pinned official A/B and independent scalar-cost, linear-scan C oracle."""
    points = np.asarray(path, dtype=np.float64); state = np.asarray(pose, dtype=np.float64)
    s = np.asarray([r['original_fractional_row_coordinate'] for r in lineage], dtype=np.float64)
    if s.shape != (len(points),) or not np.isfinite(s).all() or np.any(s < 0.):
        raise ValueError('one finite nonnegative progress per installed row required')
    constant = metadata is not None
    if constant:
        if (metadata.get('constant_reference') is not True or metadata.get('retained_source_row_count') != 1
                or metadata.get('original_source_row_index') != s[0] or s[0] != int(s[0])
                or not np.all(s == s[0]) or not np.array_equal(points, np.tile(points[0], (len(points), 1)))
                or any(r['original_left_row_index'] != s[0] or r['original_right_row_index'] != s[0] for r in lineage)):
            raise ValueError('invalid single-source constant-reference proof')
    elif len(s) > 1 and not np.all(np.diff(s) > 0.):
        raise ValueError('ordinary saved source progress must be strictly increasing')
    errors = points-state
    errors[:, 2] = [math.atan2(math.sin(float(y)), math.cos(float(y))) for y in errors[:, 2]]
    costs = np.sum(errors*errors*np.asarray(weights), axis=1)
    nearest = next(i for i,cost in enumerate(costs) if cost == min(costs))
    queries = None
    if method == METHODS[2]:
        queries = [min(float(s[nearest])+h, float(s[-1])) for h in range(1,6)]
        ids = [len(points)-1 if constant else next(i for i,value in enumerate(s) if value >= q) for q in queries]
        reference = points[ids].copy(); previous = float(state[2])
        for row in reference:
            delta = float(row[2]-previous)
            previous += math.atan2(math.sin(delta), math.cos(delta)); row[2] = previous
    elif method in METHODS[:2]:
        ids = [min(nearest+h,len(points)-1) for h in range(1,6)]
        reference = pure['build_pose_aligned_reference'](points,state,horizon=5,weights=weights)
    else:
        raise ValueError('unknown method; no fallback oracle')
    return dict(reference=reference,indices=ids,nearest=nearest,costs=costs,q=queries,progress=s)


def audit_selection(audit,pure,path,pose,lineage,saved,weights,method,label,metadata=None):
    expected = independent_targets(pure,path,pose,lineage,method,weights,metadata)
    enriched = enrich_selector_result(path,pose,expected['reference'],lineage,method=method,
        weights=weights,horizon=5,original_goal_row_index=saved['original_goal_row_index'],
        constant_reference_metadata=metadata)
    audit.same(saved,enriched,label+'/complete audit')
    for key,value in [('nearest_index',expected['nearest']),('indices',expected['indices']),
                      ('reference_world',expected['reference']),('weighted_total_pose_distances',expected['costs'])]:
        audit.same(saved[key],value,label+'/independent '+key,0.)
    audit.same(saved['final_goal_row_in_horizon'],any(expected['progress'][i]==saved['original_goal_row_index'] for i in expected['indices']),label+'/goal inclusion')
    audit.same(saved['q_h'],expected['q'],label+'/strict queries',0.)
    if method == METHODS[2]:
        audit.same(saved['progress_overshoot'],[float(expected['progress'][i]-q) for i,q in zip(expected['indices'],expected['q'])],label+'/ceil overshoot',0.)
    return expected


def audit_rollout(audit,pure,rollout,context,reference,lineage,weights,method,ordinal,label,metadata=None):
    """Reuse original recorded integration checks with the extended pure oracle.

    The validator function code is reused with one private binding, without
    changing the historical validator module or any experiment implementation.
    """
    namespace = dict(_ref02_audit_rollout.__globals__)
    namespace['audit_selection'] = lambda *args: audit_selection(*args,metadata=metadata)
    checker = FunctionType(_ref02_audit_rollout.__code__,namespace,_ref02_audit_rollout.__name__,
        _ref02_audit_rollout.__defaults__,_ref02_audit_rollout.__closure__)
    checker(audit,pure,rollout,context,reference,lineage,weights,method,ordinal,label)


def independent_additional_selection(rows):
    """Separate implementation of frozen priorities; no rollout result access."""
    used_ids,episodes,pairs = set(),set(),set(); selected=[]
    for group in ('O','R','P','S'):
        for _ in range(6):
            candidates=[r for r in rows if r['additional_eligible'] and r['group_memberships'][group] and r['case_id'] not in used_ids]
            if not candidates: break
            ordered=sorted(candidates,key=lambda r:(int(r['episode_id'] in episodes),int(r['ordered_raw_pair'] in pairs),
                int(not r['rotation_dominant']) if group=='R' else 0,
                hashlib.sha256(('REF03-v1:'+r['case_id']).encode()).hexdigest(),r['case_id']))
            row=ordered[0];selected.append((row['case_id'],group));used_ids.add(row['case_id'])
            episodes.add(row['episode_id']);pairs.add(row['ordered_raw_pair'])
    return selected


def audit_cohort(audit,run,source,config,env):
    from reconciliation.gp_se2_ref03_cohort import build_cohort,cohort_policy
    from run_gp_se2_ref03 import prior_usage
    roots={k:Path(v) for k,v in source['sources'].items()}
    manifests={k:str(roots[k]/'case_manifest.json') for k in ('GP01','GP02','REF01','REF02')}
    regenerated=build_cohort(manifests['GP01'],manifests,source_root=roots['online'],environment=env,
        original_config=config,expected_source_hashes=read(run/'preservation_before.json')['files'],
        other_known_source_usage=prior_usage(roots['online']))
    saved=read(run/'candidate_catalog.json')
    audit.same(saved,regenerated,'entire 881-source catalog reconstruction')
    audit.same(len(saved['all_candidates']),881,'authoritative source catalog count')
    audit.same(saved['source_integrity_valid'],True,'source-only cohort integrity')
    audit.same(saved['policy'],cohort_policy(),'cohort policy')
    audit.same(read(run/'cohort_coverage.json'),saved['coverage'],'coverage summary')
    audit.same(independent_additional_selection(saved['all_candidates']),[(r['case_id'],r['selected_group']) for r in saved['additional']],'independent deterministic exclusive cohort selection')
    previous=set(saved['exclusion_inventory']['case_ids'])
    for row in saved['all_candidates']:
        label='catalog '+row['case_id']
        audit.same(row['additional_eligible'],bool(row['source_integrity_valid'] and row['original_gp01_eligible'] and row['case_id'] not in previous),label+'/eligibility independent')
        audit.same(row['selection_hash'],hashlib.sha256(('REF03-v1:'+row['case_id']).encode()).hexdigest(),label+'/hash priority')
        yaw,arc=row['suffix_internal_accumulated_abs_wrapped_yaw_deg'],row['suffix_xy_arc_m']
        expected=dict(O=bool(row['original_suffix_environment']['clearance_valid'] and row['suffix_minimum_footprint_edge_clearance_m']<=.15),
            R=bool(yaw>=45.),P=bool(row['original_e_perp_m']>=.10 or (row['original_window_direction_reliable'] and row['original_window_direction_difference_deg'] is not None and row['original_window_direction_difference_deg']>=30.)),S=bool(row['original_A_SMALL_STRAIGHT']))
        audit.same(row['group_memberships'],expected,label+'/frozen threshold memberships')
        audit.same(row['rotation_dominant'],bool(arc<=.20 and yaw>=30.),label+'/rotation priority')
    return saved


def audit_probe_records(audit,pure,probes,native,methods,weights,label):
    for key,value in [('historical_pose_fallback',False),('new_mpc_solves',0),('state_integration_performed',False),('controller_memory_modified',False),('planned_selector_calls',60)]:
        audit.same(probes[key],value,label+'/'+key)
    if native is None:
        audit.same(probes['status'],'UNAVAILABLE',label+'/incomplete A availability')
        audit.same(probes['unavailable_reason'],'CURRENT_A_ROLLOUT_INCOMPLETE_OR_UNAVAILABLE',label+'/no historical fallback reason')
        audit.same(probes['states'],[],label+'/no fabricated probe poses')
        audit.same(probes['selector_calls_attempted'],0,label+'/no current A no calls')
        return
    if probes['status']=='UNAVAILABLE':
        audit.check(bool(probes['unavailable_reason']),label+'/explicit unavailable reason')
        audit.same(probes['states'],[],label+'/unavailable records absent')
        audit.same(probes['selector_calls_attempted'],0,label+'/unavailable calls zero')
        return
    audit.same(len(probes['states']),30,label+'/current A state count')
    audit.same(probes['selector_calls_attempted'],60,label+'/exact B/C probe attempts')
    completed=0
    for method in METHODS[1:]:
        item=methods[method];ref=item['reference'];inst=probes['installation'][method]
        local,transform=candidate_in_capture_frame(ref,native['candidate_frame_transform']['capture_pose_world'])
        installed=pure['project_body_to_world'](local,native['candidate_frame_transform']['capture_pose_world'])
        audit.same(inst['installed_reference_world'],installed,label+'/'+method+' installed geometry',0.)
        audit.same(inst['candidate_capture_local'],local,label+'/'+method+' capture inverse',0.)
        audit.same(inst['candidate_frame_transform'],transform,label+'/'+method+' capture transform')
        audit.same(inst['source_reference_file']['sha256'],digest(item['folder']/'reference_world.npy'),label+'/'+method+' source bytes')
        for i,row in enumerate(probes['states']):
            original=native['controller_reference_selections'][i]
            audit.same(row['input_pose_world'],original['input_pose_world'],label+f'/current A pose {i}',0.)
            audit.same(row['time_s'],original['time_s'],label+f'/current A time {i}',0.)
            audit.same(tuple(row['methods']),METHODS[1:],label+f'/only B/C {i}')
            saved=row['methods'][method]
            if saved.get('status')=='TECHNICAL_FAILURE':
                audit.check(bool(saved.get('error')),label+f'/explicit selector failure {i}');continue
            audit_selection(audit,pure,installed,row['input_pose_world'],item['lineage'],saved,weights,method,label+f'/{method}/{i}',item['metadata'])
            completed+=1
    audit.same(probes['selector_calls_completed'],completed,label+'/completed selector calls')
    audit.same(probes['method_selection_count'],completed,label+'/recorded complete diagnostics')
    for i,row in enumerate(probes['states']):
        b,c=[row['methods'][m] for m in METHODS[1:]]
        if 'nearest_index' in b and 'nearest_index' in c:
            for key in ('nearest_index','nearest_original_fractional_row_coordinate','weighted_total_pose_distances','weighted_xy_contributions','weighted_yaw_contributions'):
                audit.same(b[key],c[key],label+f'/B/C matched {key}/{i}',0.)
    if probes['status']=='COMPLETED':audit.same(completed,60,label+'/completed full probes')


def audit_worker_order(audit,events,rows,cases):
    expected=[]
    for case in cases:
        cid=case['case_id']
        for method in METHODS:
            row=next(r for r in rows if r['case_id']==cid and r['method']==method)
            if row['attempted']:expected.append(('METHOD_STARTED',cid,method))
            expected.append(('METHOD_FINISHED',cid,method))
            if method==METHODS[0]:expected.extend([('PROBES_STARTED',cid,None),('PROBES_FINISHED',cid,None)])
    actual=[(e['type'],e['case_id'],e.get('method')) for e in events]
    audit.same(actual,expected,'sequential A/current-A-probes/B/C execution ledger')
    finished=[e for e in events if e['type']=='METHOD_FINISHED']
    for event,row in zip(finished,rows):
        audit.same({k:v for k,v in event.items() if k!='type'},row,'append-only event/finished ledger')


def independent_event_patterns(rows,manifest):
    """Direct boolean outcome table, separately from the aggregate implementation."""
    by={(r['case_id'],r['method']):r for r in rows}
    if len(by)!=len(rows) or set(by)!={(c['case_id'],m) for c in manifest for m in METHODS}:
        raise ValueError('all planned unique outcomes or explicit missing records required')
    result={}
    for case in manifest:
        values=[]
        for method in METHODS:
            row=by[case['case_id'],method]
            if not row['completed']:
                if row['primary_success'] is not None:raise ValueError('incomplete result cannot become scientific failure')
                values.append(None)
            else:
                if type(row['primary_success']) is not bool:raise ValueError('completed outcome needs boolean predicate')
                values.append(row['primary_success'])
        result[case['case_id']]='UNAVAILABLE' if any(v is None for v in values) else ''.join('1' if v else '0' for v in values)
    return result


def validate(run,*,write_output=True,output=None,require_finalized=True):
    run=Path(run).resolve()
    target=Path(output) if output is not None else run/('validation.json' if require_finalized else 'verification/preflight_validation.json')
    if write_output and target.exists():raise FileExistsError('refusing validation overwrite: '+str(target))
    started=time.perf_counter();audit=Audit();deferred=[];case_reports=[]

    def global_checks():
        from run_gp_se2_ref03 import validate_config
        from run_gp_se2_ref02 import SELECTOR_DEFINITION
        source,protocol=read(run/'source.json'),read(run/'protocol.json')
        experimental=validate_config(yaml.safe_load((run/'experiment_config.yaml').read_text()))
        config=yaml.safe_load((run/'config_snapshot.yaml').read_text())
        audit.same(protocol['selector_definition'],SELECTOR_DEFINITION,'frozen REF02 selector unchanged')
        for name,key in [('config_snapshot.yaml','original_config_sha256'),('experiment_config.yaml','experiment_config_sha256')]:
            audit.same(digest(run/name),source[key],'frozen config hash '+name)
        audit.same(digest(run/'config_snapshot.yaml'),digest(Path(source['sources']['GP01'])/'config_snapshot.yaml'),'original acceptance source unchanged')
        for key in ('frozen_core_sha256','preserved_core_sha256'):
            for path,h in source[key].items():audit.same(digest(ROOT/path),h,key+'/'+path)
        for key in ('authoritative_validations','copied_source_sha256'):
            for path,h in source[key].items():
                # Environment provenance may use paths relative to its export.
                p=Path(path) if Path(path).is_absolute() else Path(source['environment_path'])/path
                audit.same(digest(p),h,key+'/'+path)
                if key=='authoritative_validations':audit.same(read(p)['valid'],True,'authoritative prior valid '+path)
        environment_hashes=source.get('environment_file_hashes') or read(Path(source['primary_source'])/'source.json')['environment_file_hashes']
        for row in environment_hashes:
            audit.same(digest(Path(source['environment_path'])/row['path']),row['sha256'],'original environment '+row['path'])
        freeze=read(run/'execution_freeze.json')
        audit.same(freeze['no_primary_execution_yet'],True,'code/input freeze before execution')
        for path,h in freeze['source_sha256'].items():audit.same(digest(ROOT/path),h,'execution code '+path)
        for path,h in freeze['input_sha256'].items():audit.same(digest(run/path),h,'execution input '+path)
        before=read(run/'preservation_before.json')
        if require_finalized or (run/'preservation_after.json').exists():
            after=read(run/'preservation_after.json')
            for key in ('files','user_config_hashes','external_git_sha','external_git_status'):audit.same(before[key],after[key],'original before/after '+key)
        else:after=before;deferred.append('final preservation_after.json')
        for path,h in after['files'].items():audit.same(digest(path),h,'preserved original '+path)
        for path,h in after['user_config_hashes'].items():audit.same(digest(ROOT/path),h,'unrelated user config '+path)
        if require_finalized or (run/'artifact_manifest.json').exists():
            for path,h in read(run/'artifact_manifest.json')['files'].items():audit.same(digest(run/path),h,'final artifact '+path)
        else:deferred.append('final artifact manifest')
        env=HospitalEnvironment.load(source['environment_path'])
        catalog=audit_cohort(audit,run,source,config,env)
        cases=read(run/'case_manifest.json')['selected']
        audit.same([c['case_id'] for c in cases],[c['case_id'] for c in catalog['selected']],'fixed controls/transfer order')
        for actual,original in zip(cases,catalog['selected']):
            audit.same({k:actual[k] for k in original},original,'selected source metadata '+actual['case_id'])
        count=len(cases)
        for key,value in [('expected_events',count),('planned_rollouts',3*count),('planned_primary_mpc_solves',90*count),('planned_selector_only_calls',60*count),('new_VLA_updates',0),('gp_or_rigid_solves',0)]:
            audit.same(protocol[key],value,'protocol '+key)
        audit.same(protocol['execution_order'],[dict(case_id=c['case_id'],method=m) for c in cases for m in METHODS],'fixed execution order')
        mpc=read(run/'mpc_output/provenance.json')
        audit.same(mpc['official_settings'],source['official_mpc']['official_settings'],'official MPC settings')
        audit.same(mpc['official_settings_sha256'],hashlib.sha256(json.dumps(mpc['official_settings'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),'official settings hash')
        audit.same(mpc['request_sha256'],digest(run/'mpc_request.json'),'actual request hash')
        audit.same(read(run/'mpc_output/request.json'),read(run/'mpc_request.json'),'actual worker request')
        audit.same(read(run/'mpc_request.json')['frozen_case_order'],[c['case_id'] for c in cases],'worker ordered cases')
        for key,value in [('planned_rollouts',3*count),('planned_primary_mpc_solves',90*count),('planned_selector_probes',60*count)]:
            audit.same(read(run/'mpc_request.json')[key],value,'request planned count '+key)
        for path,h in mpc['implementation_sha256'].items():audit.same(digest(path),h,'worker actual implementation '+path)
        for row in read(run/'mpc_output/output_hashes.json')['files']:
            audit.same(digest(run/'mpc_output'/row['path']),row['sha256'],'worker generated '+row['path'])
        for row in read(run/'mpc_output/input_hashes.json')['files']:
            audit.same(digest(row['path']),row['sha256'],'worker consumed '+row['path'])
        ledger=read(run/'mpc_output/ledger.json')['methods'];planned=read(run/'mpc_output/planned_ledger.json')['methods']
        audit.same([(r['case_id'],r['method']) for r in ledger],[(c['case_id'],m) for c in cases for m in METHODS],'all planned actual ledger rows')
        audit.same([(r['case_id'],r['method']) for r in planned],[(c['case_id'],m) for c in cases for m in METHODS],'original planned ledger rows')
        events=[json.loads(line) for line in (run/'mpc_output/events.jsonl').read_text().splitlines()]
        audit_worker_order(audit,events,ledger,cases)
        audit.same(read(run/'execution_completed.json')['exit_code'],0,'primary worker returned normally')
        return source,config,experimental,cases,mpc,official_pure(mpc['mpc_source']),env,ledger

    loaded=audit.capture('global source/cohort/runtime',global_checks)
    outcomes=[];rows=[];mechanisms=[];matched=[];closed=[];divergences=[];reproductions=[];probes_by_case=[];timings=[]
    if loaded:
        source,config,experimental,cases,mpc,pure,env,ledger=loaded
        weights=mpc['official_settings']['Q_WEIGHTS'];statuses={(r['case_id'],r['method']):r for r in ledger}
        for case_index,entry in enumerate(cases):
            nchecks,nerrors=audit.checks,len(audit.errors)
            def case_checks():
                from run_gp_se2_ref03 import reference_inputs
                from run_gp_se2_01 import route_for_source
                case=run/entry['relative_directory'];worker=run/'mpc_output'/entry['case_directory']
                context=read(case/'input_context.json');route=read(case/'goal_route.json')
                audit.same(context,load_frozen_context(source['source_run'],entry['episode_id'],entry['handoff_id']),'original acquired context '+entry['case_id'])
                for key in ('old_raw_local','old_world','fresh_raw_local','fresh_world'):
                    audit.same(np.load(case/'input_snapshot'/f'{key}.npy',allow_pickle=False),context[key],'preserved original snapshot '+key,0.)
                prepared,factorized,expected_methods=reference_inputs(context,mpc['official_settings'])
                audit.same(read(case/'reference_preparation.json'),prepared,'original prepared reference')
                audit.same(route,route_for_source(context['B_world'],prepared,env),'original goal/route/gates reconstructed')
                audit.same(entry['required_gate_ids'],[g['gate_id'] for g in route['gates']],'required gates retained')
                audit.same(entry['physical_command_controller_memory_differ'],bool(np.any(np.asarray(context['u_minus'])!=context['previous_control'])),'physical command/memory distinction')
                methods={};actual={m:None for m in METHODS};actual_for_repro={};historical={}
                for method_index,method in enumerate(METHODS):
                    folder=case/'methods'/method;status=statuses[entry['case_id'],method]
                    ref=np.load(folder/'reference_world.npy',allow_pickle=False);lineage=read(folder/'row_provenance.json')
                    metadata=read(folder/'lineage_metadata.json') if (folder/'lineage_metadata.json').exists() else None
                    item=expected_methods[method]
                    audit.same(ref,item['value']['reference_world'],method+'/original helper values',0.)
                    audit.same(lineage,item['value']['row_provenance'],method+'/original helper lineage',0.)
                    audit.same(metadata,item['metadata'],method+'/single-source proof')
                    audit.same(ref.dtype.str,np.dtype('float64').str,method+'/float64')
                    audit_lineage(audit,np.asarray(context['fresh_world']),ref,lineage,method+'/independent raw affine lineage')
                    audit.same(read(folder/'geometry_audit.json'),item['value']['geometry_audit'],method+'/geometry audit')
                    audit.same(read(folder/'spatial_reference_environment.json'),env.check_polyline(ref),method+'/spatial environment (not exclusion)')
                    audit.same(read(folder/'worker_status.json'),status,method+'/case/global ledger')
                    metrics=read(folder/'metrics.json');rollout=None
                    if status['completed']:
                        audit.same(status['attempted'],True,method+'/completed implies attempted')
                        audit.same(status['status'],'COMPLETED',method+'/computation completion')
                        rollout=read(folder/'rollout.json')
                        audit.same(rollout,read(worker/method/'rollout.json'),method+'/worker output unchanged',0.)
                        audit_rollout(audit,pure,rollout,context,ref,lineage,weights,method,3*case_index+method_index,method,metadata)
                        for solve in rollout['controller_reference_selections']:audit.same(solve['actual_controller_v_max'],mpc['official_settings']['OBJNAV_V_MAX'],method+'/unchanged velocity bound')
                        expected=evaluate_method(method,rollout,context,ref,route,env,config,case_metadata=entry)
                        for key,value in expected.items():
                            if key!='independent_evaluation_wall_s':audit.same(metrics[key],value,method+'/original outcome '+key)
                        for key,value in [('actual_controller_calls',30),('primary_mpc_solve_count',30),('completed_control_cycles',30),('actual_selector_calls_recorded',30),('controller_failure_count',rollout['controller_failure_count'])]:audit.same(status[key],value,method+'/actual counts '+key)
                        actual[method]=rollout['controller_reference_selections'];actual_for_repro[method]=dict(rollout=rollout,metrics=metrics)
                    else:
                        expected=missing_outcome(entry['case_id'],method,attempted=status['attempted'],reason=status['technical_error'],case_metadata=entry)
                        audit.same(metrics,expected,method+'/explicit unavailable outcome')
                        if status['attempted']:
                            capture=read(folder/'partial_actual_calls.json')
                            calls=sum(len(r['controller_calls']) for r in capture['instances'])
                            audit.same(status['actual_controller_calls'],calls,method+'/partial actual controller calls')
                            audit.same(status['primary_mpc_solve_count'],calls,method+'/partial numerical solve attempts')
                            audit.same(capture['states_and_commands_reconstructed'],False,method+'/no fabricated partial trace')
                        else:audit.same(status['actual_controller_calls'],0,method+'/unattempted no calls')
                    methods[method]=dict(folder=folder,reference=ref,lineage=lineage,provenance=lineage,metadata=metadata,rollout=rollout,metrics=metrics)
                    outcomes.append(metrics);rows.append(metrics['summary'])
                    hist=case/'historical'/method
                    if (hist/'rollout.json').exists():historical[method]=dict(rollout=read(hist/'rollout.json'),metrics=read(hist/'metrics.json'))
                    if actual[method] is not None:mechanisms.append(dict(case_id=entry['case_id'],method=method,mode='closed_loop',**selection_mechanism(actual[method])))
                for name in ('reference_world.npy','row_provenance.json'):
                    audit.same((methods[METHODS[1]]['folder']/name).read_bytes(),(methods[METHODS[2]]['folder']/name).read_bytes(),'B/C byte-identical '+name)
                b,c=[methods[m]['rollout'] for m in METHODS[1:]]
                if b is not None and c is not None:audit.same(b['installed_reference_world'],c['installed_reference_world'],'B/C exact installed path',0.)
                probes=read(case/'matched_state_probes.json');audit.same(probes,read(worker/'matched_state_probes.json'),'actual current-A probe record')
                native=methods[METHODS[0]]['rollout']
                audit_probe_records(audit,pure,probes,native,methods,weights,entry['case_id']+'/probes')
                if native is not None:
                    audit.same(probes['source_rollout_sha256'],digest(worker/METHODS[0]/'rollout.json'),'probe source is current A bytes')
                    audit.same(Path(probes['source_rollout_path']).resolve(),(worker/METHODS[0]/'rollout.json').resolve(),'probe source current run path')
                probes_by_case.append(probes)
                if probes['status']=='COMPLETED':
                    matched.extend(compare_selections(entry['case_id'],probes['states'],matched=True))
                    for method in METHODS[1:]:mechanisms.append(dict(case_id=entry['case_id'],method=method,mode='matched_state',**selection_mechanism([dict(time_s=p['time_s'],selection_diagnostic=p['methods'][method]) for p in probes['states']])))
                if all(actual.values()):closed.extend(compare_selections(entry['case_id'],actual,matched=False))
                divergences.extend(first_command_divergences(entry['case_id'],actual))
                if entry['cohort']=='KNOWN_CONTROLS':
                    source_kind='GP-SE2-01' if entry['case_role']=='KNOWN_OBSTACLE_STRESS' else 'GP-SE2-REF-02'
                    reproduction=control_reproduction(entry['case_id'],actual_for_repro,historical,source_kind=source_kind)
                    audit.same(read(case/'reproduction_check.json'),reproduction,'original control reproduction')
                    reproductions.append(reproduction)
                # Plot validation is appended below when the rendering module is available.
                if (case/'plots').exists() or require_finalized:audit.capture('case plots '+entry['case_id'],lambda:audit_case_plots(audit,run,case,context,route,methods,config))
                else:deferred.append('case plots '+entry['case_id'])
            audit.capture('case '+entry['case_id'],case_checks)
            case_reports.append(dict(case_id=entry['case_id'],valid=len(audit.errors)==nerrors,checks=audit.checks-nchecks,errors=audit.errors[nerrors:]))
        if len(rows)==len(cases)*3:
            audit.capture('aggregate results',lambda:audit_aggregates(audit,run,rows,outcomes,cases,ledger,probes_by_case,mechanisms,matched,closed,divergences,reproductions))
        else:audit.check(False,'every planned method needs evaluated or explicitly unavailable result')
        if (run/'plot_manifest.json').exists() or require_finalized:audit.capture('aggregate plots/coverage',lambda:audit_plot_manifest(audit,run,cases))
        else:deferred.append('plot manifest')
        if (run/'review_bundle.zip').exists() or require_finalized:audit.capture('review bundle',lambda:audit_bundle(audit,run))
        else:deferred.append('review bundle')
    report=dict(experiment='GP-SE2-REF-03',valid=not audit.errors,checks=audit.checks,errors=audit.errors,
        authoritative=require_finalized,deferred_checks=deferred,case_validations=case_reports,
        validation_scope='complete finalized saved artifacts' if require_finalized else 'non-authoritative preflight; final checks deferred',
        scientific_failures_are_not_validation_failures=True,unavailable_results_are_not_scientific_failures=True,
        independent_saved_evidence_only=True,optimizer_or_MPC_solves_performed=0,
        official_selector_verification='hash-pinned pure AST; independent strict lower-bound scan; actual solver local/world arguments',
        cohort_verification='entire original 881-source catalog reconstructed plus independent greedy frozen selection',
        validation_wall_s=time.perf_counter()-started,validator_sha256=digest(__file__),generated_utc=datetime.now(timezone.utc).isoformat())
    if write_output:
        target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    return report


def audit_aggregates(audit,run,rows,outcomes,cases,ledger,probes,mechanisms,matched,closed,divergences,reproductions):
    aggregate=aggregate_outcomes(outcomes,cases)
    mapping=dict(outcome_matrix='ledger',success_patterns='event_classifications',regressions='reason_comparisons',
        paired_metrics='paired_quality',failed_pair_endpoint_diagnostics='failed_pair_endpoint_diagnostics')
    for name,key in mapping.items():
        audit.same(read(run/'aggregate'/f'{name}.json'),aggregate[key],name+'/recomputed JSON')
        audit_csv(audit,run/'aggregate'/f'{name}.csv',aggregate[key],name+'/CSV')
    patterns=independent_event_patterns(rows,cases)
    for record in read(run/'aggregate/success_patterns.json'):
        audit.same(record['exclusive_pattern'],patterns[record['case_id']],'independent Boolean success pattern '+record['case_id'])
    by={(r['case_id'],r['method']):r for r in rows}
    for row in read(run/'aggregate/paired_metrics.json'):
        a,b=[by[row['case_id'],m] for m in (row['baseline'],row['method'])]
        eligible=a['completed'] and b['completed'] and a['primary_success'] and b['primary_success']
        audit.same(row['eligible_both_success'],bool(eligible),'independent paired quality eligibility')
        x,y=a.get(row['metric']),b.get(row['metric'])
        audit.same(row['baseline_value'],x if eligible else None,'failed pairs do not supply quality baseline')
        audit.same(row['method_value'],y if eligible else None,'failed pairs do not supply quality method')
        audit.same(row['difference'],y-x if eligible and x is not None and y is not None else None,'independent quality subtraction/N/A')
    summary=read(run/'aggregate/summary.json')
    for key,value in aggregate.items():audit.same(summary[key],value,'recomputed aggregate '+key)
    for file,key in [('duplicate_summary','duplicate_summary'),('paired_quality_summary','paired_quality_equal_weight')]:
        expected={scope:({k:v[key] for k,v in aggregate[scope].items()} if scope in ('transfer_strata','known_control_by_role') else aggregate[scope][key])
            for scope in ('additional_transfer','known_controls','transfer_strata','known_control_by_role')}
        audit.same(read(run/'aggregate'/f'{file}.json'),expected,file+'/all separate scopes')
    # Independent equal weighting: event observations, episode group means, raw-pair group means.
    metadata={c['case_id']:c for c in cases}
    for scope,entries in [('known_controls',[c for c in cases if c['cohort']=='KNOWN_CONTROLS']),('additional_transfer',[c for c in cases if c['cohort']=='ADDITIONAL_TRANSFER'])]:
        ids={c['case_id'] for c in entries}
        for method in METHODS:
            selected=[r for r in rows if r['case_id'] in ids and r['method']==method]
            saved=summary[scope]['method_outcomes'][method]
            for key,value in [('planned',len(selected)),('attempted',sum(r['attempted'] for r in selected)),('completed',sum(r['completed'] for r in selected)),('missing',sum(not r['completed'] for r in selected)),('successes',sum(r['completed'] and r['primary_success'] for r in selected)),('failures',sum(r['completed'] and not r['primary_success'] for r in selected))]:audit.same(saved[key],value,'independent cohort/method '+scope+'/'+method+'/'+key)
            for key,groupkey in [('event_equal','case_id'),('episode_equal','episode_id'),('ordered_raw_pair_equal','ordered_raw_pair')]:
                groups={}
                for row in selected:
                    meta=metadata[row['case_id']];label=meta[groupkey]
                    groups.setdefault(label,[])
                    if row['completed']:groups[label].append(float(row['primary_success']))
                values=[sum(x)/len(x) for x in groups.values() if x]
                audit.same(saved[key]['value'],None if not values else sum(values)/len(values),'independent '+scope+'/'+method+'/'+key)
    # Pairwise logs are diagnostics, not additional episodes.
    audit.same(read(run/'aggregate/matched_state_pairwise.json'),matched,'matched-state comparisons')
    audit.same(read(run/'aggregate/closed_loop_pairwise.json'),closed,'closed-loop comparisons')
    audit.same(read(run/'aggregate/command_divergence.json'),divergences,'command divergence from actual controls')
    saved_mechanisms=read(run/'aggregate/selection_mechanism.json')
    observed={(r['case_id'],r['method'],r['mode']):r for r in saved_mechanisms}
    for expected in mechanisms:
        key=expected['case_id'],expected['method'],expected['mode']
        actual=observed[key]
        audit.same(actual.get('available'),True,'available selection mechanism')
        for name,value in expected.items():audit.same(actual[name],value,'selection mechanism '+str(key)+'/'+name)
    for row in saved_mechanisms:
        if not row['available']:
            audit.check(bool(row.get('reason') or row.get('missing_reason')),'unavailable mechanism reason')
    for name in ('matched_state_pairwise','closed_loop_pairwise','command_divergence','selection_mechanism'):
        audit_csv(audit,run/'aggregate'/f'{name}.csv',read(run/'aggregate'/f'{name}.json'),name+'/CSV')
    recovered=[r for r in aggregate['event_classifications'] if any(r['overlapping_flags'][flag] is True for flag in ('B_failure_C_success','A_success_B_failure_C_success','A_B_failure_C_success'))]
    audit.same(read(run/'aggregate/recoveries.json'),recovered,'all recovery records (overlap preserved)')
    audit_csv(audit,run/'aggregate/recoveries.csv',recovered,'recovery CSV')
    metrics_by={(o['case_id'],o['method']):o for o in outcomes};status_by={(r['case_id'],r['method']):r for r in ledger}
    expected_timing=[]
    for case,probe in zip(cases,probes):
        cid=case['case_id']
        expected_timing.append(dict(case_id=cid,method=None,stage='matched_selector_probes',scope='case',wall_s=probe['wall_s'],included_in='worker_process',selector_calls=probe['selector_calls_completed'],mpc_solves=0))
        for method in METHODS:
            status=status_by[cid,method];metric=metrics_by[cid,method]
            if status['completed']:
                expected_timing.append(dict(case_id=cid,method=method,stage='independent_evaluation',scope='method',wall_s=metric['independent_evaluation_wall_s'],included_in='evaluation_total',mpc_solves=0))
            expected_timing.extend([
                dict(case_id=cid,method=method,stage='rollout',scope='method',wall_s=status['rollout_wall_s'],included_in='worker_process',mpc_solves=status['actual_controller_calls']),
                dict(case_id=cid,method=method,stage='official_mpc_solve',scope='method',wall_s=status['official_mpc_solve_wall_s'],included_in='rollout',mpc_solves=status['actual_controller_calls'])])
    audit.same(read(run/'aggregate/timing.json'),expected_timing,'timing from actual recorded stages (nested inclusions explicit)')
    audit_csv(audit,run/'aggregate/timing.csv',expected_timing,'timing CSV')
    audit.same(read(run/'aggregate/outcomes.json'),outcomes,'every complete original metric record')
    audit.same(summary['control_reproductions'],{r['case_id']:r for r in reproductions},'all historical control reproduction reports')
    audit.same(summary['controls_reproduced'],all(r['passed'] for r in reproductions),'control reproduction conjunction')
    worker=read(run/'mpc_output/summary.json')
    audit.same(summary['compute']['worker'],worker,'all inclusive actual worker cost/call records')
    expected_counts=dict(primary_rollouts_planned=len(cases)*3,rollouts_attempted=sum(r['attempted'] for r in ledger),
        independent_rollouts_completed=sum(r['completed'] for r in ledger),primary_mpc_solves_planned=len(cases)*90,
        primary_mpc_solves=sum(r['actual_controller_calls'] for r in ledger),actual_controller_calls=sum(r['actual_controller_calls'] for r in ledger),
        actual_rollout_selector_calls_recorded=sum(r['actual_selector_calls_recorded'] for r in ledger),
        matched_state_selector_calls_planned=len(cases)*60,matched_state_selector_calls_attempted=sum(p['selector_calls_attempted'] for p in probes),
        matched_state_selector_calls_completed=sum(p['selector_calls_completed'] for p in probes),
        historical_or_other_mpc_solves=0,new_VLA_updates=0,GP_or_rigid_optimization_solves=0,retries=0)
    for key,value in expected_counts.items():audit.same(worker[key],value,'independent planned/actual count '+key)
    audit.same(worker['rows'],ledger,'worker summary/all ledger rows')
    for record in reproductions:audit.same(record['new_mpc_solves'],0,'reproduction no new solve')


def _audit_sidecar(audit,run,path,numeric,identity=None):
    side=read(path.with_suffix('.json'))
    audit.same(side['numeric_data'],numeric,str(path)+'/numeric data')
    audit.same(side['image_sha256'],digest(path),str(path)+'/PNG hash')
    audit.same(side['plotter_sha256'],digest(ROOT/'scripts/plot_gp_se2_ref03.py'),str(path)+'/plotter source')
    audit.same(side['input_identity'],identity,str(path)+'/fixed input identity')
    audit.same(side['primary_time_range_s'],[0.,3.],str(path)+'/entire time range')
    for key in ('new_inference','new_optimization','new_execution','gui_runtime_validated'):audit.same(side[key],False,str(path)+'/'+key)
    for name,h in side['source_hashes'].items():audit.same(digest(name),h,str(path)+'/source '+name)


def audit_case_plots(audit,run,case,context,route,methods,config):
    import csv
    from plot_gp_se2_ref03 import expected_numeric,input_identity,PLOT_NAMES
    past=read(case/'actual_past_execution.json')
    audit.same(past['source_sha256'],digest(past['source_path']),'past acquisition hash')
    audit.same(past['poses_world'][-1],context['B_world'],'actual past stops at B',0.)
    with Path(past['source_path']).open() as stream:history={int(r['state_id']):r for r in csv.DictReader(stream)}
    for i,idx in enumerate(past['state_ids']):
        original=history[idx]
        audit.same(past['poses_world'][i],[float(original[k]) for k in ('x','y','yaw')],'actual past pose',0.)
        audit.same(past['times_relative_to_B_s'][i],float(original['sim_time_s'])-context['switch_sim_time_s'],'actual past time',0.)
    numeric=expected_numeric(context,route,methods,past,config);identity=input_identity(methods)
    for name in PLOT_NAMES:_audit_sidecar(audit,run,case/'plots'/f'{name}.png',numeric[name],identity)


def audit_plot_manifest(audit,run,cases):
    from plot_gp_se2_ref03 import expected_aggregate_numeric,representative_cases,PLOT_NAMES,AGGREGATE_PLOT_NAMES
    manifest=read(run/'plot_manifest.json')
    expected={c['relative_directory']+'/plots/'+name+'.png' for c in cases for name in PLOT_NAMES}
    expected.update('aggregate/plots/'+name+'.png' for name in AGGREGATE_PLOT_NAMES)
    audit.same({r['path'] for r in manifest['images']},expected,'every case and aggregate PNG coverage')
    audit.same(manifest['image_count'],len(cases)*7+4,'all required image count')
    audit.same(manifest['case_count'],len(cases),'all plotted cases')
    for row in manifest['images']:
        path=run/row['path'];audit.same(row['sha256'],digest(path),'manifest PNG hash')
        audit.same(row['sidecar_sha256'],digest(path.with_suffix('.json')),'manifest numeric sidecar hash')
    summary=read(run/'aggregate/summary.json');patterns=read(run/'aggregate/success_patterns.json');regressions=read(run/'aggregate/regressions.json')
    coverage=read(run/'cohort_coverage.json');quality=read(run/'aggregate/paired_metrics.json')
    expected_numeric=expected_aggregate_numeric(summary,cases,coverage,patterns,regressions,quality)
    for name in AGGREGATE_PLOT_NAMES:_audit_sidecar(audit,run,run/'aggregate/plots'/f'{name}.png',expected_numeric[name])
    audit.same(read(run/'representative_selection.json'),representative_cases(cases,patterns,regressions),'predeclared representative policy')
    index=(run/'index.html').read_text()
    for case in cases:audit.check(case['case_id'] in index,'full index includes '+case['case_id'])


def audit_bundle(audit,run):
    from package_gp_se2_ref03_review import TABLES,AGGREGATE,DETAILS,MAJOR
    cases=read(run/'case_manifest.json')['selected'];reps=read(run/'representative_selection.json')
    expected=set(('source.json','protocol.json','config_snapshot.yaml','experiment_config.yaml','case_manifest.json',
        'cohort_coverage.json','representative_selection.json','plot_manifest.json','README.md','index.html','manifest.json'))
    expected.update('aggregate/'+p for p in TABLES)
    expected.update('aggregate/plots/'+name+ext for name in AGGREGATE for ext in ('.png','.json'))
    expected.update(c['relative_directory']+'/plots/'+MAJOR+ext for c in cases for ext in ('.png','.json'))
    expected.update(c['relative_directory']+'/plots/'+name+ext for c in reps['selected'] for name in DETAILS for ext in ('.png','.json'))
    folder=run/'review_bundle';manifest=read(folder/'manifest.json');records={r['path']:r for r in manifest['allowlisted_files']}
    audit.same(set(records)|{'manifest.json'},expected,'strict compact review allowlist')
    audit.same({str(p.relative_to(folder)) for p in folder.rglob('*') if p.is_file()},expected,'actual review files')
    for name,row in records.items():
        audit.check(not Path(name).is_absolute() and '..' not in Path(name).parts,'safe review member '+name)
        audit.same(digest(folder/name),row['sha256'],'review artifact hash '+name)
        if row['source']!='generated review text':audit.same(digest(row['source']),row['sha256'],'review source hash '+name)
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        audit.same(set(archive.namelist()),expected,'ZIP all allowed files')
        audit.same(len(archive.namelist()),len(expected),'ZIP no duplicate member')
        for name in archive.namelist():audit.same(hashlib.sha256(archive.read(name)).hexdigest(),digest(folder/name),'ZIP member bytes '+name)
    outer=read(run/'review_bundle_manifest.json')
    audit.same(outer['bundle_sha256'],digest(run/'review_bundle.zip'),'ZIP SHA256')
    audit.same(outer['all_case_world_overlay_count'],len(cases),'all frozen overlays in review')
    audit.same(outer['image_count'],sum(p.endswith('.png') for p in expected),'review PNG count')
    audit.same(outer['gui_runtime_validated'],False,'no invented GUI evidence')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--output',type=Path);parser.add_argument('--preflight',action='store_true')
    args=parser.parse_args();report=validate(args.run,output=args.output,require_finalized=not args.preflight)
    print(json.dumps({k:v for k,v in report.items() if k!='case_validations'},indent=2))
    raise SystemExit(0 if report['valid'] else 1)
