"""Synthetic implementation tests and saved-source checks; zero numerical MPC solves."""
from copy import deepcopy
from pathlib import Path
import sys
import ast
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.osa03_trackability import first_legal_tick,spatial_descriptors,slice_for_evaluation,origin_metrics,paired_metrics,audit_observation,restored_activation
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_source03 import read,sha,save
from run_osa03_trackability import load_source,measure
from analyze_join_online02 import csvread,jsonlines
from test_online_mpc_adapter import configured,submit,reference


@pytest.mark.parametrize('tick,submitted,expected',[(75,False,78),(78,False,78),(78,True,84),(79,False,84)])
def test_first_legal_absolute_tick(tick,submitted,expected):assert first_legal_tick(tick,submitted)==expected


def test_pending_old_result_cannot_advance_nonzero_restored_memory(configured):
    a,t,path=configured;t.previous_command=t.command=(.6,-.03);submit(a);ref=reference(a)
    a.install(episode_id='ep0',chunk_id='fresh',reference_version=1,raw_local_path=path,capture_pose=[1,2,1])
    t.complete(ref,command=(.8,.5));r=a.poll()
    assert r['status']=='stale_rejected' and not r['command_available_for_application']
    assert t.previous_command==t.command==(.6,-.03)


@pytest.mark.parametrize('n',[1,2,10,17])
def test_raw_N_spatial_only_observation_anchor(n):
    raw=np.c_[np.arange(n)*.2,np.arange(n)*.03,np.linspace(-3.13,3.13,n)];before=raw.copy();obs=[2,3,1]
    f=local_trajectory_to_world(obs,raw);d=spatial_descriptors(raw,obs,f)
    np.testing.assert_array_equal(raw,before);assert d['N']==n and d['waypoint_intrinsic_dt_s'] is None
    assert np.all(abs(d['yaw_increments_rad'])<=np.pi)
    with pytest.raises(AssertionError):spatial_descriptors(raw,[2.2,3,1],f)


class Free:
    def check_polyline(self,p):return dict(minimum_clearance_m=1.,clearance_valid=True)
    def check_trajectory(self,*args,**kwargs):return dict(clearance_valid=True,minimum_clearance_lower_bound_m=1.)


def synthetic():
    dt=float(np.float32(1/60));start=75;sim=1.25;commands=[];states=[];x=np.zeros(3)
    for i in range(181):
        states.append(dict(absolute_tick=start+i,time_s=i*dt,sim_time_s=sim+i*dt,pose_world=x.tolist()))
        if i==180:break
        c=dict(v_mps=.2,omega_radps=0.,reason='new_solve' if i>=4 and (i-4)%6==0 else 'hold',chunk_id='fresh' if i>=4 else 'old',application_tick=start+i,sim_time_s=sim+i*dt)
        commands.append(c);x=integrate_unicycle(x,[.2,0],dt)
    r=dict(states=states,commands=commands,phase=dict(integration_dt_s=dt,u_obs=[.2,0],incoming_application={'sim_time_s':sim-2*dt},fresh_chunk_id='fresh'),termination='OBSERVATION_CAP')
    f=np.array([[0,0,0],[1.,0.,0.]])
    cfg=yaml.safe_load((ROOT/'configs/osa03_native_continuation_01.yaml').read_text())
    return r,f,cfg


def test_two_origins_equal_windows_unavailable_full_no_padding():
    r,f,cfg=synthetic();m=origin_metrics(r,f,Free(),dict(center_xy=[2,2],forward_xy=[1,0]),cfg)
    assert m['first_FRESH_tick']==79 and m['first_FRESH_index']==4
    assert m['availability']['windows']['180']['available']
    assert not m['execution']['windows']['180']['available'] and not m['full_execution_3s_available']
    assert m['execution']['windows']['18']['available'] and m['execution']['windows']['54']['available']
    assert m['execution']['full']['join_time_s']==0.
    assert len(m['execution']['trace']['times_s'])==177
    np.testing.assert_allclose(m['availability_to_application_travel_m'],.2*4*r['phase']['integration_dt_s'])
    assert slice_for_evaluation(r,4)['phase']['u_minus']==[.2,0]
    rows=paired_metrics(m,m['availability']);assert all(z['B_minus_observation']==0 for z in rows if z['metric']=='first09_position_AUC_m_s')
    base=deepcopy(m['availability']);base['windows']['54']['position_auc_m_s']=-.1
    assert next(z for z in paired_metrics(m,base) if z['metric']=='first09_position_AUC_m_s')['B_minus_observation']<0


def source_inputs():
    source=ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    if not source.exists():pytest.skip('immutable local source unavailable; never synthesize empirical evidence')
    ep=source/'episodes/REPEAT_00';context=read(source/'source_bundle/REPEAT_00/state_and_timing.json')['context']
    return source,[context,csvread(ep/'execution.csv'),csvread(ep/'commands.csv'),jsonlines(ep/'controller/events.jsonl'),read(ep/'obstacle_reveal.json'),read(ep/'metadata.json')['resolved_integration_dt_s']]


def test_exact_saved_observation_state_memory_and_no_B_substitution():
    source,args=source_inputs();p,f,raw,_=load_source(source)
    assert sha(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy')=='8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521'
    assert p['obs_tick']==75 and p['next_legal_submit_tick']==78 and p['pending_OLD']==[]
    assert .599<p['memory_obs'][0]<.601 and .599<p['u_obs'][0]<.601
    assert p['obs_pose']!=p['original_delayed_B'] and p['old_install']['official_generation']==2
    a=restored_activation(p);assert a.active['chunk_id']==p['old_install']['chunk_id'] and a.installed['chunk_id']==p['fresh_chunk_id']
    np.testing.assert_array_equal(p['memory_obs'],p['last_accepted_OLD']['command'])


@pytest.mark.parametrize('mode',['missing','pending','B_substitution','zero'])
def test_unresolved_observation_blocks(mode):
    _,args=source_inputs();args=deepcopy(args);events=args[3]
    last=[e for e in events if e.get('solve_id')=='REPEAT_00_solve_000002' and e.get('type')=='solve_result'][0]
    if mode=='missing':events.remove(last)
    elif mode=='pending':last['seen_in_isaac']['host_monotonic_s']=args[0]['t_obs']['host_monotonic_s']+1
    elif mode=='B_substitution':last['command']=[.8,.5]
    else:last['command']=[0,0]
    with pytest.raises((AssertionError,KeyError,ValueError)):audit_observation(*args)


def test_saved_measure_pipeline_without_new_solve():
    source,args=source_inputs();p,f,raw,_=load_source(source)
    # Saved-only wiring check: actual recorded obs-to-end is NOT a counterfactual result.
    states=[s for s in args[1] if int(s['state_id'])>=p['obs_tick']]
    commands=[c for c in args[2] if int(c['application_tick'])>=p['obs_tick']]
    for c in commands:
        for key in ['v_mps','omega_radps','sim_time_s','episode_time_s']:c[key]=float(c[key])
        c['application_tick']=int(c['application_tick'])
    r=dict(phase=p,states=[dict(absolute_tick=int(s['state_id']),time_s=i*p['integration_dt_s'],sim_time_s=float(s['sim_time_s']),pose_world=[float(s[k]) for k in ['x','y','yaw']]) for i,s in enumerate(states)],commands=commands,termination='OBSERVATION_CAP')
    from run_join_online02 import environments
    _,env,_,_=environments(source)
    cfg=yaml.safe_load((ROOT/'configs/osa03_native_continuation_01.yaml').read_text())
    m=origin_metrics(r,f,env,read(source/'scenario.json'),cfg)
    assert m['first_FRESH_tick']==92 and m['first_FRESH_index']==17
    assert not m['execution']['windows']['54']['available']


def test_no_forbidden_execution_and_exclusive_output(tmp_path):
    for name in ['scripts/run_osa03_trackability.py','scripts/osa03_trackability_mpc_worker.py','src/reconciliation/osa03_trackability.py']:
        tree=ast.parse((ROOT/name).read_text());imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert all(not any(s in x for s in ['gp_se2_ref02','formulation','lightnav_client','lightnav_worker']) for x in imports)
    p=tmp_path/'result.json';save(p,dict(x=1))
    with pytest.raises(FileExistsError):save(p,dict(x=2))


def test_measure_common_execution_exposure_no_baseline_resolve(tmp_path,monkeypatch):
    import run_osa03_trackability as runner
    r,f,cfg=synthetic();r['phase']['obs_pose']=[0,0,0]
    scene=dict(center_xy=[2,2],forward_xy=[1,0]);metrics=origin_metrics(r,f,Free(),scene,cfg)
    baseline=slice_for_evaluation(r,0)
    (tmp_path/'metric_protocol.yaml').write_text(yaml.safe_dump(cfg))
    records={'source_manifest.json':dict(source='/saved/source',comparator='/saved/baseline'),
             'scenario.json':scene,'metrics.json':metrics['availability'],'rollout.json':baseline}
    monkeypatch.setattr(runner,'read',lambda path:deepcopy(records[Path(path).name]))
    m=runner.measure(tmp_path,r,f,Free())
    assert m['common_execution_exposure']['integration_intervals']==176
    assert m['common_execution_exposure']['B_metrics']['full']['observation_horizon_s']==m['origins']['execution']['full']['observation_horizon_s']


def test_new_saved_validator_when_scientific_record_exists():
    run=ROOT/'data/osa03_fresh_trackability_control_01/primary_20260924T012000Z'
    if not (run/'rollout.json').exists():pytest.skip('pre-execution: no scientific record yet')
    from run_osa03_trackability import validate
    assert validate(run)['valid']
    if (run/'review/manifest.json').exists():
        from report_osa03_trackability import validate_figures
        assert validate_figures(run)['valid']
