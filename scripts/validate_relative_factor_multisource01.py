#!/usr/bin/env python3
"""Saved-only audit; no optimizer, MPC, RGB or Isaac calls."""
import argparse, json, csv
from pathlib import Path
import numpy as np
from run_relative_factor_multisource01 import ROOT, RESULTS, verify, plain
from reconciliation.join_source03 import read, save, sha
from reconciliation.relative_factor_multisource import (load_source, geometry, reference_safety,
    unique_sources, cross_source, candidate_inventory)
from reconciliation.osa03_relative_ablation import ORDER, comparability, planning, metric_row, gaps
from reconciliation.osa03_relative_replication import selector_diagnostics
from reconciliation.osa03_common_b import references, may_execute, schedule, activation_at_B
from reconciliation.osa03_native import evaluate, command
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_online02 import guard_check, preview_activation
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.se2 import local_trajectory_to_world
from validate_osa03_native_continuation import guard_parity
from validate_robotless_online_handoffs import equal_record
from validate_osa03_relative_factor_replication01 import validate_planning


def validate_method(run,name,ref,common,fresh,env,scene,settings):
    root=run/'methods'/name
    if not may_execute(ref['safety']):
        assert not (root/'rollout.json').exists();s=read(root/'skipped.json');assert s['status']=='REFERENCE_GEOMETRY_UNSAFE' and s['metrics'] is None
        return None,None,dict(valid=True,skipped=True)
    for p,h in read(root/'hashes.json').items():assert sha(root/p)==h
    r=read(root/'rollout.json');equal_record(r['phase'],common);refworld=np.load(ref['world_path'])
    assert r['method']==name and r['reference_world_sha256']==ref['world_sha256']
    assert r['historical_future_results_replayed']==0 and r['new_VLA_calls']==r['new_RGB_calls']==r['new_optimizer_calls']==0
    if not (root/'restoration.json').exists():return r,None,dict(valid=False,reason='no successful controller restoration')
    restored=read(root/'restoration.json')
    for k,v in [('B',common['B']),('held_command',common['u_B_plus']),('previous_control',common['u_mem_B'])]:np.testing.assert_array_equal(restored[k],v)
    assert restored['generation']==common['original_generation'] and restored['next_submit_tick']==common['next_submit_after_B']
    np.testing.assert_allclose(restored['installed_world'],refworld,rtol=0,atol=1e-12)
    assert len(r['commands'])==len(r['states'])-1
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==common['B_tick']+i and s['time_s']==i*common['integration_dt_s'] and s['sim_time_s']==common['B_sim_s']+i*common['integration_dt_s']
        if i:np.testing.assert_array_equal(s['pose_world'],integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),common['integration_dt_s']))
    requests=r['submit_requests'];assert [e['tick'] for e in requests]==list(range(common['next_submit_after_B'],r['states'][-1]['absolute_tick'],6)) or r['termination']!='OBSERVATION_CAP'
    assert all(e['tick']>=common['next_submit_after_B'] and e['tick']%6==0 for e in requests)
    assert len(set(e['solve_id'] for e in requests))==len(requests)
    previous=common['u_mem_B'];solved={}
    for e in r['events']:
        if e.get('status')=='submitted':np.testing.assert_array_equal(e['previous_command'],previous)
        if e.get('type')=='solve_result':
            assert e['solve_id'].startswith(name+'_NEW_') and e['input_state_id']>=common['next_submit_after_B'] and e['input_state_id']%6==0
            assert e['solve_id'] not in solved;solved[e['solve_id']]=e
            np.testing.assert_array_equal(e['input_pose'],r['states'][e['input_state_id']-common['B_tick']]['pose_world'])
            if e['status']=='command':
                np.testing.assert_array_equal(e['previous_command'],previous);previous=e['command']
                assert selection_audit(refworld,e['input_pose'],e['selection']['reference_world'],horizon=settings['HORIZON'],weights=settings['Q_WEIGHTS'])==e['selection']
    a=activation_at_B(common)
    for i,c in enumerate(r['commands']):
        tick=c['application_tick'];s=r['states'][i]
        for event in r['events']:
            if event.get('type')=='solve_result' and event.get('continuation_seen',{}).get('tick')==tick:
                assert a.accept(event,c['sim_time_s'])==event['activation_accepted']
        st=dict(c,state_id=tick,tick=tick,x=s['pose_world'][0],y=s['pose_world'][1],yaw=s['pose_world'][2])
        a,expected,event=preview_activation(a,st,None,None);assert event is None
        for key in expected:assert expected[key]==c[key],(name,tick,key)
        guard=guard_check(env,s['pose_world'],c,common['integration_dt_s']);assert guard['safe'];guard_parity(guard,r['guards'][i])
        if i==0:np.testing.assert_array_equal(command(c),common['u_B_plus'])
        elif c['reason']=='new_solve':
            e=solved[c['solve_id']];assert e['continuation_seen']['tick']==tick;np.testing.assert_array_equal(command(c),e['command'])
    if r['safety_abort']:
        abort=r['safety_abort'];assert not abort['command_applied']
        if r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND':assert not guard_check(env,abort['guard']['start_pose'],abort['proposed_command'],common['integration_dt_s'])['safe']
    metric=None if not r['commands'] else plain(evaluate(r,fresh,env,scene,read(run/'metric_protocol.json')))
    equal_record(metric,read(root/'metrics.json'))
    assert r['new_MPC_submitted']==sum(e.get('status')=='submitted' for e in r['events']) and r['new_MPC_solved']==len(solved)
    return r,metric,dict(valid=True,common_state=True,exact_integration=True,absolute_grid=True,no_historical_future_replay=True,
                         original_FRESH_evaluator=True,command_memory_selection=True,guard=True,metric_recomputation=True)



def source_validate(folder,cfg,spec):
    data=load_source(cfg,spec);common=read(folder/'common_state.json');equal_record(common,data['common'])
    equal_record(read(folder/'schedule.json'),data['schedule'])
    fresh=data['fresh'];raw=data['raw'];env=geometry(folder);refs=read(folder/'references.json')
    assert list(refs)==ORDER
    problem=LocalSE2Problem(common['fresh_capture_pose'],common['B'],fresh)
    expected=references(common['fresh_capture_pose'],common['B'],fresh,fresh)
    rollouts={};metrics={};own={};audits={};costs={};pa={}
    for name,ref in refs.items():
        if ref['status']=='PLANNING_FAILED':
            result=read(folder/'planning'/name/'planning_result.json');assert result['error'] is not None
            assert read(folder/'methods'/name/'skipped.json')['status']=='PLANNING_FAILED'
            assert not (folder/'methods'/name/'rollout.json').exists()
            rollouts[name]=metrics[name]=own[name]=None;pa[name]=result;audits[name]={'valid':True,'planning_failed':True};continue
        for frame in ['world','local']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        x=np.load(ref['world_path']);local=np.load(ref['local_path'])
        np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'],local),x,atol=1e-12,rtol=0)
        if name=='M0_NATIVE':np.testing.assert_array_equal(local,raw)
        if name in ORDER[:2]:np.testing.assert_array_equal(x,expected['M0_NATIVE' if name=='M0_NATIVE' else 'M1_SE2_TAPER'])
        else:
            include=cfg['relative_factor'][name];out=folder/'planning'/name
            pa[name]=validate_planning(out,problem,x,env,include)
            start=read(out/'optimization_start.json')
            assert start['solver_config']==cfg['solver'] and start['include_relative']==include and start['calls']==1
            assert start['initial_world_sha256']==refs['M0_NATIVE']['world_sha256']
            costs[name]=dict(optimized_costs=problem.costs(x,include_relative=include),
                diagnostic_relative_edge_distortion_not_optimized_cost=None if include else problem.costs(x)['R'])
        equal_record(ref['safety'],reference_safety(x,env));equal_record(ref['descriptor'],plain(planning(fresh,x,common['B'])))
        r,m,audit=validate_method(folder,name,ref,common,fresh,env['on'],data['scene'],data['manifest']['mpc']['official_settings'])
        rollouts[name]=r;metrics[name]=m;audits[name]=audit;own[name]=None
        if r is None:continue
        assert r['raw_FRESH_sha256']==data['manifest']['FRESH_sha256']
        release={p['submit_tick']:p['application_tick'] for p in data['schedule']['pairs']}
        for w in r['logical_waits']:
            assert w['sim_before']==w['sim_after'] and w['pose_before']==w['pose_after']
            assert w['release_tick']==release[w['submit_tick']]
        for e in r['events']:
            if e.get('type')=='solve_result' and e['status']=='command':
                assert e['official_generation']==e['result_generation']==common['original_generation']
                if not e.get('after_termination'):assert e['continuation_seen']['tick']==release[e['input_state_id']]
        if r['commands']:
            om=plain(evaluate(r,x,env['on'],data['scene'],read(folder/'metric_protocol.json')))
            equal_record(om,read(folder/'methods'/name/'own_reference_metrics.json'))
            own[name]={k:v for k,v in metric_row(om,r['termination']).items() if k in ['position_auc_03_m_s','position_auc_09_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s','max_position_error_05_m']}
    gate=comparability(rollouts,data['schedule'],common)
    gate['rule']='source-authenticated logical schedule and exact common-state provenance through inclusive B+54'
    gate['full_window_match']={n:r is not None and schedule(r,180)==data['schedule']['full'] for n,r in rollouts.items()}
    primary={n:metric_row(metrics[n],None if rollouts[n] is None else rollouts[n]['termination']) for n in ORDER}
    pairwise={}
    for a,b in [('NO_RELATIVE','FULL_LOCAL_SE2'),('FULL_LOCAL_SE2','M1_TAPER')]:
        if 'world_path' not in refs[a] or 'world_path' not in refs[b]:pairwise[a+'_minus_'+b]=None;continue
        x,y=[np.load(refs[n]['world_path']) for n in [a,b]]
        d=dict(max_reference_XY_separation_m=float(np.linalg.norm(x[:,:2]-y[:,:2],axis=1).max()))
        if rollouts[a] is not None and rollouts[b] is not None:
            length=min(len(rollouts[a]['states']),len(rollouts[b]['states']))
            x,y=[np.array([s['pose_world'] for s in rollouts[n]['states'][:length]]) for n in [a,b]]
            d['max_execution_XY_separation_m']=float(np.linalg.norm(x[:,:2]-y[:,:2],axis=1).max())
        pairwise[a+'_minus_'+b]=d
    summary=dict(source_id=spec['id'],primary_metrics=primary,secondary_own_reference_metrics=own,
        planning={n:r.get('descriptor') for n,r in refs.items()},reference_safety={n:r['safety'] for n,r in refs.items()},
        planning_audits=pa,factor_costs=costs,schedule_gate=gate,selector=selector_diagnostics(rollouts),pairwise=pairwise,
        termination={n:refs[n]['status'] if r is None else r['termination'] for n,r in rollouts.items()},
        endpoints={n:None if m is None else m['endpoint'] for n,m in metrics.items()},
        counts=dict(planning=sum((folder/'planning'/n/'optimization_start.json').exists() for n in ORDER[2:]),
            rollouts=sum(r is not None for r in rollouts.values()),
            MPC=sum(0 if r is None else r['new_MPC_solved'] for r in rollouts.values()),LightNav=0,RGB=0,Isaac=0))
    return summary,dict(valid=all(a['valid'] for a in audits.values()),methods=audits)


def validate(run):
    verify(run);cfg=read(run/'protocol.json');selected=read(run/'selected_sources.json');unique_sources(selected)
    equal_record(candidate_inventory(cfg),read(run/'candidate_inventory.json'))
    summaries={};checks={}
    for s in selected:
        summaries[s['id']],checks[s['id']]=source_validate(Path(s['folder']),cfg,s)
    old_checks={}
    from validate_osa03_relative_factor_ablation01 import validate as v00
    from validate_osa03_relative_factor_replication01 import validate as v01
    for p,validator in zip(cfg['legacy_results'],[v00,v01]):
        old=read(ROOT/p);out=Path(old['run']);s,v=validator(out)
        assert v['valid'];equal_record(s,read(out/'summary.json'))
        assert sha(out/'result_hashes.json')==old['result_hashes_sha256'];old_checks[p]=True
    cross=cross_source(summaries,cfg['sign_tolerance'])
    if not all(c['valid'] for c in checks.values()):cross['classification']='TECHNICAL_BLOCKED'
    summary=dict(experiment=cfg['experiment'],label=cfg['label'],starting_sha=cfg['starting_sha'],
        scientific_freeze_sha=read(run/'execution_start.json')['sha'],selected_sources=selected,sources=summaries,
        cross_source=cross,classification=cross['classification'],
        counts={k:sum(s['counts'][k] for s in summaries.values()) for k in ['planning','rollouts','MPC','LightNav','RGB','Isaac']})
    return summary,dict(valid=all(c['valid'] for c in checks.values()),sources=checks,legacy_saved_validators=old_checks,new_scientific_calls=0)


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    for name,rows in [('primary',[dict(source_id=sid,method=n,**(row or {'termination_reason':q['termination'][n]})) for sid,q in s['sources'].items() for n,row in q['primary_metrics'].items()]),
                      ('signed_gaps',[dict(source_id=sid,**d) for sid,d in s['cross_source']['deltas'].items()]),
                      ('cross_source_signs',s['cross_source']['sign_summary'])]:
        fields=list(dict.fromkeys(k for r in rows for k in r))
        with (RESULTS/(name+'.csv')).open('x',newline='') as f:
            writer=csv.DictWriter(f,fields);writer.writeheader();writer.writerows(rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();write(a.run.resolve())
