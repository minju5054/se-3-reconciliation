#!/usr/bin/env python3
"""Saved-only common-state, schedule, geometry and metric validation; zero solves."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import yaml
from run_osa03_common_b import ROOT,geometry,verify,checked
from reconciliation.join_source03 import read,save,sha
from reconciliation.osa03_common_b import ORDER,M3_HASH,authenticated_m3,references,distortion,common_state,activation_at_B,may_execute,timing_audit,metric_row,signed_gaps,classify
from reconciliation.osa03_native import command,evaluate,replay_prefix
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_online02 import guard_check,preview_activation
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.se2 import relative_pose,local_trajectory_to_world
from run_osa03_native_continuation import source_phase,plain
from run_local_se2_reconciliation_formulation01 import independent_clearance
from validate_robotless_online_handoffs import equal_record
from validate_osa03_native_continuation import guard_parity


def historical_saved_audit(run):
    m=read(run/'source_manifest.json');root=Path(m['historical_native']);source=Path(m['source'])
    phase,fresh,_,provenance=source_phase(source);r=read(root/'rollout.json');env=geometry(source)['on']
    equal_record(phase,r['phase']);np.testing.assert_array_equal(r['states'][0]['pose_world'],phase['B'])
    replay=replay_prefix(phase,env);assert replay['passed']
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==phase['B_tick']+i
        if i:np.testing.assert_array_equal(s['pose_world'],integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),phase['integration_dt_s']))
    for i,c in enumerate(r['commands']):guard_parity(guard_check(env,r['states'][i]['pose_world'],c,phase['integration_dt_s']),r['guards'][i])
    for e in r['events']:
        if e.get('type')=='solve_result' and e.get('status')=='command':
            assert selection_audit(fresh,e['input_pose'],e['selection']['reference_world'],horizon=5,weights=provenance['official_settings']['Q_WEIGHTS'])==e['selection']
    metrics=plain(evaluate(r,fresh,env,read(source/'scenario.json'),yaml.safe_load((root/'protocol.yaml').read_text())))
    equal_record(metrics,read(root/'metrics.json'))
    return dict(valid=True,R00_only=True,source_phase=True,saved_prefix_parity=True,integration_and_guard=True,
                metric_recomputation=True,historical_first_post_B_submit=phase['applications'][1]['submit_tick'],
                historical_first_post_B_application=phase['applications'][1]['application_tick'],new_MPC_calls=0)


def applied_sequence(rollout):
    return [c['application_tick'] for c in rollout['commands'] if c['reason']=='new_solve']


def historical_comparison(run,r,m):
    if r is None or not r['commands'] or m is None:return dict(available=False)
    root=Path(read(run/'source_manifest.json')['historical_native']);old=read(root/'rollout.json');om=read(root/'metrics.json')
    cfg=read(run/'protocol.json');common=read(run/'common_state.json')
    np.testing.assert_array_equal(r['states'][0]['pose_world'],old['states'][0]['pose_world'])
    np.testing.assert_array_equal(command(r['commands'][0]),command(old['commands'][0]))
    np.testing.assert_array_equal(common['u_mem_B'],old['phase']['u_mem_B'])
    current_apps=applied_sequence(r);old_apps=applied_sequence(old);same=current_apps==old_apps and len(r['states'])==len(old['states'])
    n=min(len(r['states']),len(old['states']));pe=float(np.max(abs(np.array([s['pose_world'] for s in r['states'][:n]])-np.array([s['pose_world'] for s in old['states'][:n]]))))
    ce=float(np.max(abs(np.array([command(c) for c in r['commands'][:n-1]])-np.array([command(c) for c in old['commands'][:n-1]]))))
    parity=pe<=cfg['historical_parity_pose_atol'] and ce<=cfg['historical_parity_command_atol']
    first=next((e for e in r['events'] if e.get('type')=='solve_result'),{});hist=old['phase']['applications'][1]['result']
    return dict(available=True,B_identity=True,held_command_identity=True,memory_identity=True,
        next_submit_tick=first.get('input_state_id'),historical_next_submit_tick=hist['input_state_id'],
        first_selected_rows=first.get('selection'),historical_first_selected_rows=hist['selection'],
        first_command=first.get('command'),historical_first_command=hist['command'],
        first_command_error=None if 'command' not in first else float(np.max(abs(np.array(first['command'])-hist['command']))),
        new_application_ticks=current_apps,historical_application_ticks=old_apps,schedules_match=same,
        max_pose_error=pe,max_command_error=ce,strict_parity_required=same,strict_parity_pass=parity if same else None,
        windows_new={k:m['windows'][k] for k in ['18','54']},windows_historical={k:om['windows'][k] for k in ['18','54']},
        attachment_new=m['full']['join_time_s'],attachment_historical=om['full']['join_time_s'],
        clearance_new=m['clearance']['execution']['minimum_clearance_lower_bound_m'],clearance_historical=om['clearance']['execution']['minimum_clearance_lower_bound_m'])


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
    assert restored['generation']==common['original_generation'] and restored['next_submit_tick']==96
    np.testing.assert_allclose(restored['installed_world'],refworld,rtol=0,atol=1e-12)
    assert len(r['commands'])==len(r['states'])-1
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==common['B_tick']+i and s['time_s']==i*common['integration_dt_s'] and s['sim_time_s']==common['B_sim_s']+i*common['integration_dt_s']
        if i:np.testing.assert_array_equal(s['pose_world'],integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),common['integration_dt_s']))
    requests=r['submit_requests'];assert [e['tick'] for e in requests]==list(range(96,r['states'][-1]['absolute_tick'],6)) or r['termination']!='OBSERVATION_CAP'
    assert all(e['tick']>=96 and e['tick']%6==0 for e in requests)
    assert len(set(e['solve_id'] for e in requests))==len(requests)
    previous=common['u_mem_B'];solved={}
    for e in r['events']:
        if e.get('status')=='submitted':np.testing.assert_array_equal(e['previous_command'],previous)
        if e.get('type')=='solve_result':
            assert e['solve_id'].startswith(name+'_NEW_') and e['input_state_id']>=96 and e['input_state_id']%6==0
            assert e['solve_id'] not in solved;solved[e['solve_id']]=e
            np.testing.assert_array_equal(e['input_pose'],r['states'][e['input_state_id']-92]['pose_world'])
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


def validate(run):
    verify(run);cfg=read(run/'protocol.json');checked(cfg);manifest=read(run/'source_manifest.json');source=Path(manifest['source'])
    phase,fresh,_,provenance=source_phase(source);common=common_state(phase);equal_record(common,read(run/'common_state.json'))
    m3=authenticated_m3(Path(manifest['M3_run'])/'derived/optimized_world.npy');expected=references(common['fresh_capture_pose'],common['B'],fresh,m3)
    refs=read(run/'references.json');assert list(refs)==ORDER;env=geometry(source);rollouts={};metrics={};checks={}
    assert sorted(p.name for p in (run/'methods').iterdir())==sorted(ORDER)
    for name,ref in refs.items():
        assert sha(ref['world_path'])==ref['world_sha256'] and sha(ref['local_path'])==ref['local_sha256']
        world=np.load(ref['world_path']);local=np.load(ref['local_path']);np.testing.assert_array_equal(world,expected[name])
        np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'],local),world,rtol=0,atol=1e-12)
        equal_record(independent_clearance(world,env),ref['safety']);equal_record(distortion(fresh,world),ref['descriptor'])
        r,m,c=validate_method(run,name,ref,common,fresh,env['on'],read(source/'scenario.json'),provenance['official_settings'])
        rollouts[name]=r;metrics[name]=m;checks[name]=c
    timing=timing_audit(rollouts);rows={n:metric_row(m) for n,m in metrics.items()};gaps=signed_gaps(rows)
    history=historical_comparison(run,rollouts['M0_NATIVE'],metrics['M0_NATIVE']) if metrics['M0_NATIVE'] else dict(available=False)
    valid=all(c['valid'] for c in checks.values()) and not (history.get('strict_parity_required') and not history['strict_parity_pass'])
    overall=classify({m:may_execute(r['safety']) for m,r in refs.items()},rollouts,timing,valid)
    counts={m:0 if r is None else r['new_MPC_solved'] for m,r in rollouts.items()}
    computation={m:None if r is None else dict(rollout_wall_s=r['wall_s'],solve_wall_s=sum((e.get('official_solve_ms') or 0) for e in r['events'] if e.get('type')=='solve_result')/1000,
        per_solve_ms=[e.get('official_solve_ms') for e in r['events'] if e.get('type')=='solve_result'],
        event_counts={key:sum(e.get('status')==key for e in r['events']) for key in ['busy','stale_rejected','controller_error','error']},
        hold_timeout=r['termination']=='HOLD_TIMEOUT') for m,r in rollouts.items()}
    summary=dict(classification=overall,primary_metrics=rows,signed_method_minus_native=gaps,timing=timing,historical_M0_audit=history,
        computation=computation,counts=dict(MPC=counts,scientific_rollouts=sum(r is not None for r in rollouts.values()),LightNav=0,RGB=0,graph_optimization=0,GP=0,source_acquisition=0),
        method_termination={m:'REFERENCE_GEOMETRY_UNSAFE' if r is None else r['termination'] for m,r in rollouts.items()},
        metric_origin='common B at source tick92; all attachment targets original raw FRESH',
        limitation='all methods share already-applied first FRESH command; no effect on the initial switch command is measured')
    validation=dict(valid=valid,methods=checks,source_hashes_preserved=True,M3_frozen_hash=True,common_state=True,
        original_FRESH_evaluation=True,classification=overall,new_validation_model_MPC_optimization_calls=0)
    return summary,validation


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();r=a.run.resolve()
    summary,validation=validate(r);save(r/'summary.json',plain(summary));save(r/'validation.json',validation)
    print(json.dumps(dict(validation=validation,primary=summary['primary_metrics'],counts=summary['counts'],classification=summary['classification'])))
