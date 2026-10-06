#!/usr/bin/env python3
"""Saved-only independent wrapper, controller-state, schedule and report audit."""
import argparse
import json
from pathlib import Path
import numpy as np
from run_canonical_graph_common_b_execution01 import (ROOT,RESULTS,CONFIG,read,save,sha,plain,verify,authenticate,geometry,reference_safety)
from reconciliation.canonical_graph_execution01 import *
from reconciliation.spatial_entry_suffix import execution_metrics
from reconciliation.osa03_native import evaluate
from reconciliation.vector_bridge_execution03 import schedule_check
from reconciliation.direct_transition_hard_eval02 import application_accounting,schedule_prefix_audit
from reconciliation.boundary_row_ablation04 import compare_commands,reference_geometry
from validate_relative_factor_multisource01 import validate_method
from validate_spatial_entry_suffix_execution01 import independent_endpoint_time
from validate_robotless_online_handoffs import equal_record
from validate_canonical_se2_graph_formulation_audit01 import csv_text


def reference_audit(f):
    """Authenticate both planned arrays and actual zero-solve official installation."""
    c=read(f/'common_state.json');refs=read(f/'references.json');fresh=np.load(f/'references/M0_NATIVE_world.npy')
    raw=np.load(f/'references/M0_NATIVE_local.npy');entry=read(f/'entry.json');mapping=read(f/'identity_mapping.json')
    pre=read(f/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    installed={r['identity']:np.asarray(r['installed_world']) for r in pre['rows']};env=geometry(f)
    for n,r in refs.items():
        for frame in ['world','local']:assert sha(r[frame+'_path'])==r[frame+'_sha256']
        world=np.load(r['world_path']);local=np.load(r['local_path'])
        assert world[0].tobytes()==np.array(c['B']).tobytes()
        np.testing.assert_allclose(local[0],relative_pose(c['fresh_capture_pose'],c['B']),atol=0,rtol=0)
        np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],local),world,atol=1e-12,rtol=0)
        assert mapping[n]==identities(r['labels'],entry,fresh)
        equal_record(reference_safety(world,env),r['safety'])
        if n in NEW:
            w,l=(fresh,raw) if n==RAW else (np.load(f/'canonical_optimized.npy'),np.load(f/'canonical_original_A_local.npy'))
            assert world[1:].tobytes()==w.tobytes() and local[1:].tobytes()==l.tobytes()
            actual=installed[n];assert actual.tobytes()==np.load(f/'references'/f'{n}_installed.npy').tobytes()
            np.testing.assert_allclose(actual,world,rtol=0,atol=1e-12)
            safe=reference_safety(actual,env);equal_record(safe,read(f/'installation.json')[n]['safety'])
            assert r['may_execute']==(r['safety']['clearance_valid'] and safe['clearance_valid'])
            if n==RAW:assert actual[1:].tobytes()==np.load(f/'native_installed.npy').tobytes()
    return dict(valid=True,exact_B_raw_canonical=True,original_A_frame=True,complete_B_connector_checked=True,zero_numerical_MPC_preflight=True)


def validate(run):
    authenticate(read(run/'protocol.json'))
    with execution_guard(saved_only=True):
        verify(run);cfg=read(run/'protocol.json');specs=read(run/'selected_sources.json');sources={}
        assert [s['id'] for s in specs]==SOURCE_IDS
        completion=read(run/'completion.json');counts={k:v for k,v in completion.items() if k!='completed'}
        counts.update({n+'_rollouts':0 for n in NEW});counts.update(historical_B_ENTRY_rollouts_reused=0,historical_context_rollouts_reused=0,
            MPC_solves=0,MPC_logical_result_releases=0,MPC_physical_applications=0,withheld_results=0,guard_blocked_applications=0,executed_intervals=0,controller_numerical_failures=0)
        assert all(counts[k]==0 for k in completion if k!='completed')
        for s in specs:
            f=Path(s['folder']);old=Path(s['historical_folder']);canon=Path(s['canonical_folder'])
            for n in ['common_state.json','source_manifest.json','schedule.json','scenario.json','source_audit.json','metric_protocol.json','entry.json','recorded_old_to_B.npy','native_installed.npy']:
                assert sha(f/n)==sha(old/n),n
            for n in ['canonical_optimized.npy','canonical_original_A_local.npy','result.json','context.json']:assert sha(f/n)==sha(canon/n)
            for p,h in read(f/'source_manifest.json')['hashes'].items():assert sha(p)==h,p
            c=read(f/'common_state.json');sched=read(f/'schedule.json');env=geometry(f);scene=read(f/'scenario.json')
            fresh=np.load(f/'references/M0_NATIVE_world.npy');refs=read(f/'references.json');reuse=read(f/'reuse.json')
            mapping=read(f/'identity_mapping.json');rest_pre=read(f/'restoration_preflight.json')
            q=dict(technical_valid=True,integration_dt_s=c['integration_dt_s'],reference_audit=reference_audit(f),
                methods={},selector={},commands={},schedule={},reference_geometry={},common_B_parity={})
            rolls={};restores={}
            for n in [*CENTRAL,*CONTEXT]:
                new=n in NEW;mf=f/'methods'/n if new else Path(reuse[n]['folder'])
                ref=refs[n] if n in CENTRAL else reuse[n]['reference'];method_folder=f if new else old
                if not new:
                    assert sha(mf/'hashes.json')==reuse[n]['hashes_sha256']
                    assert not (f/'methods'/n).exists()
                    counts['historical_B_ENTRY_rollouts_reused' if n==ENTRY else 'historical_context_rollouts_reused']+=1
                q['reference_geometry'][n]=reference_geometry(np.load(ref['world_path']),ref['safety'],ref['labels'])
                if new and not ref['may_execute']:
                    assert read(mf/'skipped.json')['status']=='SKIPPED_REFERENCE_INVALID' and not (mf/'rollout.json').exists()
                    q['methods'][n]=dict(metrics=None,valid=False,unsafe_reference=True,safety_abort=False,termination='SKIPPED_REFERENCE_INVALID')
                    q['selector'][n]=q['commands'][n]=q['schedule'][n]=None
                    continue
                provenance=read(mf/'worker_provenance.json')['provenance'];official=read(f/'source_manifest.json')['mpc']
                assert provenance['mpc_source_sha256']==official['mpc_source_sha256']
                assert provenance['official_settings']==official['official_settings']
                r,m,a=validate_method(method_folder,n,ref,c,fresh,env['on'],scene,official['official_settings'])
                assert a['valid'];rolls[n]=r;restores[n]=read(mf/'restoration.json')
                actual=np.asarray(restores[n]['installed_world']);safe=reference_safety(actual,env);assert safe['clearance_valid']
                if new:
                    assert actual.tobytes()==np.asarray(next(v for v in rest_pre['rows'] if v['identity']==n)['installed_world']).tobytes()
                    assert read(mf/'execution_start.json')['freeze_sha']==read(run/'execution_start.json')['sha']
                    assert read(mf/'execution_start.json')['common_sha256']==sha(f/'common_state.json')
                gate=schedule_check(r,sched,c);gate['prefix_exact']=schedule_prefix_audit(r,sched)
                numerical=any(e.get('status')=='controller_error' for e in r['events']);abort=r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'
                q['technical_valid'] &= bool(gate['prefix_exact'] and (gate['passed'] or abort) and not numerical)
                releases={p['submit_tick']:p['application_tick'] for p in sched['pairs']}
                for wait in r['logical_waits']:
                    assert wait['sim_before']==wait['sim_after'] and wait['pose_before']==wait['pose_after']
                    assert wait['release_tick']==releases[wait['submit_tick']]
                for ev in r['events']:
                    if ev.get('type')=='solve_result' and ev.get('status')=='command':
                        assert ev['result_generation']==ev['official_generation']==c['original_generation']
                        if 'continuation_seen' in ev:assert ev['continuation_seen']['tick']==releases[ev['input_state_id']]
                        else:assert ev['withheld_by_logical_scheduler'] and releases[ev['input_state_id']]>=r['states'][-1]['absolute_tick']
                metric=plain(execution_metrics(r,m,fresh))
                if metric is not None:
                    assert metric[END]==independent_endpoint_time(r,fresh)
                    metric.pop('endpoint_dwell_trace')
                own=evaluate(r,actual,env['on'],scene,read(f/'metric_protocol.json')) if m is not None else None
                ownrow=None if own is None else plain(execution_metrics(r,own,actual))
                if ownrow is not None:
                    ownrow={k:ownrow[k] for k in ['position_auc_03_m_s','position_auc_09_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s','max_position_error_05_m','sustained_attachment_s']}
                legacy_own=None if m is None else plain(evaluate(r,np.load(ref['world_path']),env['on'],scene,read(f/'metric_protocol.json')))
                equal_record(legacy_own,read(mf/'own_reference_metrics.json'))
                q['methods'][n]=dict(metrics=metric,own_installed_reference_secondary=ownrow,
                    valid=bool(not abort and not numerical and gate['passed'] and metric is not None and metric['execution_clearance_lower_bound_m']>=.05),
                    unsafe_reference=False,safety_abort=abort,termination=r['termination'],historical=not new,
                    reference_clearance_m=safe['minimum_clearance_m'])
                maps=mapping[n] if n in CENTRAL else identities(ref['labels'],read(f/'entry.json'),fresh)
                q['selector'][n]=selectors(r,maps);q['commands'][n]=command_metrics(r);q['schedule'][n]=gate
                if new:
                    counts[n+'_rollouts']+=1;accounting=application_accounting(r)
                    assert accounting['MPC_solves']==r['new_MPC_solved']
                    for k,v in accounting.items():counts[k]+=v
                    if r['termination']=='OBSERVATION_CAP':
                        assert accounting['MPC_solves']==len(sched['pairs'])
                        assert accounting['MPC_physical_applications']==len(sched['full']['applications'])
            baseline=rolls[ENTRY]
            for n in NEW:
                if n not in rolls:q['common_B_parity'][n]=dict(available=False,reason='SKIPPED_REFERENCE_INVALID');continue
                r=rolls[n];a=q['selector'][n]['first'];b=q['selector'][ENTRY]['first']
                provenance_keys=['B','B_tick','B_sim_s','u_minus','held_command','previous_control','generation','next_submit_tick','integration_dt_s','capture_pose','reference_version','historical_command_identity']
                provenance=all(restores[n][k]==restores[ENTRY][k] for k in provenance_keys)
                first=a is not None and b is not None and a['submit_tick']==b['submit_tick'] and a['input_pose']==b['input_pose']
                assert provenance and (first or q['methods'][n]['safety_abort'])
                gate=q['schedule'][n];bg=q['schedule'][ENTRY]
                primary=gate['primary']==bg['primary'] and len(r['commands'])>=54
                full=gate['full']==bg['full'] and len(r['commands'])==len(baseline['commands'])==180
                q['common_B_parity'][n]=dict(available=True,initial_state_memory_generation_exact=provenance,
                    first_legal_submit_state_exact=first,primary_09_passed=primary,full_cap_passed=full,
                    explained_safety_censor=q['methods'][n]['safety_abort'],prefix_exact=gate['prefix_exact'],
                    eligible_primary_causal_pair=primary,eligible_full_cap_causal_pair=full)
                if not (primary and full) and not q['methods'][n]['safety_abort']:q['technical_valid']=False
            q['pairwise']={};q['command_comparisons']={}
            for base,name in [(RAW,ENTRY),(RAW,CANONICAL),(ENTRY,CANONICAL)]:
                key=name+'__minus__'+base;xm=q['methods'][name]['metrics'];bm=q['methods'][base]['metrics']
                q['pairwise'][key]={k:delta(xm,bm,k) for k in [AUC,'position_auc_03_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s',ATTACH,END,'remaining_arc_at_attachment_m','linear_command_TV','angular_command_TV']}
                q['command_comparisons'][key]=compare_commands(rolls[base],rolls[name],refs[base]['labels'],refs[name]['labels']) if base in rolls and name in rolls else None
            sources[s['label']]=q
        expected=read(run/'expected_calls.json');assert counts['MPC_solves']<=expected['MPC_solves']
        assert sum(counts[n+'_rollouts'] for n in NEW)==expected['rollouts']
        assert len(list(run.glob('sources/*/methods/*/execution_start.json')))==expected['rollouts']
        assert not list(run.glob('**/optimization_start.json'))
        summary=plain(dict(experiment=cfg['experiment'],starting_head=cfg['starting_head'],scientific_freeze_sha=read(run/'execution_start.json')['sha'],
            selected_sources=specs,sources=sources,counts=counts,classification=classify(sources,cfg['classification_rules']),
            OSA03=cfg['OSA03'],boundary_only_effect=cfg['boundary_only_effect']))
        return summary,dict(valid=all(q['technical_valid'] for q in sources.values()),saved_only=True,new_optimizer_calls=0,new_MPC_calls=0,
            wrappers_frames_identity_safety=True,controller_memory_schedule_guard=True,metrics_selector_commands=True)


def compact(summary):
    value=json.loads(json.dumps(summary))
    for q in value['sources'].values():
        for v in q['selector'].values():
            if v:v.pop('rows',None)
        for v in q['command_comparisons'].values():
            if v:v.pop('rows',None);v.pop('pose_separation_trace',None)
    return value


def tables(s):
    rows={k:[] for k in ['primary','historical_context','command_metrics','selector_progress','reference_geometry','pairwise','common_B_parity']}
    for label,q in s['sources'].items():
        for n,v in q['methods'].items():
            rows['primary' if n in CENTRAL else 'historical_context'].append(dict(source=label,method=n,valid=v['valid'],termination=v['termination'],**(v['metrics'] or {})))
            if n not in CENTRAL:continue
            rows['command_metrics'].append(dict(source=label,method=n,**(q['commands'][n] or {})))
            rows['selector_progress'].append(dict(source=label,method=n,first=None if q['selector'][n] is None else q['selector'][n]['first']))
            rows['reference_geometry'].append(dict(source=label,method=n,**q['reference_geometry'][n]))
        for key,v in q['pairwise'].items():rows['pairwise'].append(dict(source=label,comparison=key,**v))
        for key,v in q['common_B_parity'].items():rows['common_B_parity'].append(dict(source=label,method=key,**v))
    return {k+'.csv':v for k,v in rows.items()}


def export(run,s,v):
    save(run/'summary.json',s);save(RESULTS/'result_summary.json',compact(s));save(RESULTS/'validation.json',v)
    save(RESULTS/'call_accounting.json',s['counts'])
    for n,rows in tables(s).items():(RESULTS/n).write_text(csv_text(rows))
    files={str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file() and p.name!='result_hashes.json'}
    save(run/'result_hashes.json',files);save(RESULTS/'result_hashes.json',dict(files=files,sha256=sha(run/'result_hashes.json')))


def check_exports(run,s):
    assert read(run/'summary.json')==s and read(RESULTS/'result_summary.json')==compact(s)
    for name,rows in tables(s).items():assert (RESULTS/name).read_text()==csv_text(rows)
    assert read(RESULTS/'call_accounting.json')==s['counts']
    ledger=read(RESULTS/'result_hashes.json');assert sha(run/'result_hashes.json')==ledger['sha256']
    assert read(run/'result_hashes.json')==ledger['files']
    for p,h in ledger['files'].items():assert sha(run/p)==h,p
    from report_canonical_graph_common_b_execution01 import figure_numbers
    assert read(RESULTS/'figure_numeric.json')==figure_numbers(run,s)
    manifest=read(RESULTS/'figure_manifest.json');assert sha(RESULTS/'figure_numeric.json')==manifest['numeric_sha256']
    assert sorted(p.name for p in (RESULTS/'figures').glob('*.png'))==sorted(PNGS)
    for p in manifest['figures']:assert sha(RESULTS/'figures'/p['file'])==p['sha256']
    return True


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check-only',action='store_true');a=p.parse_args()
    s,v=validate(a.run.resolve())
    if a.check_only:check_exports(a.run.resolve(),s)
    else:export(a.run.resolve(),s,v)
    print(json.dumps(dict(validation=v,classification=s['classification'],counts=s['counts'])))
