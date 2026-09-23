"""Synthetic implementation fixtures; no official solver/model is called."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import numpy as np
import pytest
from reconciliation.osa03_native import phase_audit,replay_prefix,next_control_tick,command,classification,evaluate
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.handoff_execution_loss import attachment_loss
from reconciliation.se2 import local_trajectory_to_world


def fixture(n=10):
    dt=float(np.float32(1/60));fresh=np.c_[np.arange(n)*.2,np.zeros(n),np.zeros(n)]
    x=np.array([0.,0.,0.]);states=[];commands=[]
    settings=dict(HORIZON=5,Q_WEIGHTS=[10.,10.,1.]);events=[]
    for tick in range(9):
        states.append(dict(state_id=tick,tick=tick,sim_time_s=tick*dt,episode_time_s=tick*dt,
            host_monotonic_s=tick*dt,host_monotonic_ns=tick,host_utc='synthetic',x=x[0],y=x[1],yaw=x[2]))
        if tick==8:break
        u=[.2,0.] if tick<2 else [.4,.5] if tick<7 else [.6,1.]
        sid='old' if tick<2 else 'first' if tick<7 else 'second'
        commands.append(dict(v_mps=u[0],omega_radps=u[1],solve_id=sid,reason='new_solve' if tick in [0,2,7] else 'hold',held=tick not in [0,2,7],application_state_id=tick,application_tick=tick,chunk_id='fresh' if tick>=2 else 'old',reference_version=1 if tick>=2 else 0,sim_time_s=tick*dt))
        x=integrate_unicycle(x,u,dt)
    for sid,inp,app,prev,u in [('first',0,2,[.2,0],[.4,.5]),('second',6,7,[.4,.5],[.6,1.])]:
        p=[states[inp][k] for k in ['x','y','yaw']];diff=fresh-np.array(p);near=int(np.argmin(np.sum(diff**2*np.array(settings['Q_WEIGHTS']),axis=1)));idx=[min(near+j,n-1) for j in range(1,6)]
        result=dict(type='solve_result',solve_id=sid,input_state_id=inp,input_pose=p,status='command',command=u,previous_command=prev,seen_in_isaac={'sim_time_s':app*dt},official_generation=3,chunk_id='fresh',reference_version=1,selection=selection_audit(fresh,p,fresh[idx],horizon=5,weights=settings['Q_WEIGHTS']))
        events.append(dict(type='reply',status='submitted',input_state_id=inp,solve_id=sid));events.append(result)
    first=deepcopy(events[1]);first['seen_sim_time_s']=2*dt
    B=[states[2][k] for k in ['x','y','yaw']]
    context=dict(switch_state_id=2,B=B,R_obs=[0.,0.,0.],first_fresh_solve=first,fresh_chunk_id='fresh',fresh_reference_version=1)
    boundary=dict(pose_world=B,u_minus=[.2,0.],memory={'available':True,'previous_control':[.4,.5]})
    return context,states,commands,events,boundary,fresh,fresh.copy(),settings,dt


@pytest.mark.parametrize('n',[1,4,10,17])
def test_arbitrary_raw_N_and_phase(n):
    a=phase_audit(*fixture(n));r=replay_prefix(a)
    assert r['passed'] and r['steps']==6 and r['maximum_pose_error']==0
    assert a['u_minus']!=a['u_B_plus'] and a['u_mem_B']==a['u_B_plus']
    assert a['next_submit_after_B']==6 and a['first_new_submit_tick']==12
    assert a['first_new_submit_tick']!=a['B_tick']


@pytest.mark.parametrize('bad',['memory','submit','application','anchor','prefix'])
def test_reconstruction_rejects_missing_or_corrupt_phase(bad):
    args=list(fixture());c,st,u,ev,b,*_=args
    if bad=='memory':b['memory']['available']=False
    if bad=='submit':ev[2]['input_state_id']=7
    if bad=='application':u[2]['solve_id']='missing'
    if bad=='anchor':c['R_obs']=[1.,0.,0.]
    if bad=='prefix':st[4]['x']+=.001
    with pytest.raises((AssertionError,KeyError)):
        a=phase_audit(*args);replay_prefix(a)


def test_no_B_reanchor_observation_transform():
    raw=np.array([[.1,0,0],[.3,.1,.2]])
    obs=np.array([2.,3.,np.pi/2]);B=np.array([2.,2.,np.pi/2])
    assert not np.allclose(local_trajectory_to_world(obs,raw),local_trajectory_to_world(B,raw))


def test_historical_command_identity_not_just_equal_values():
    a=phase_audit(*fixture());a['prefix_commands'][0]['chunk_id']='other'
    with pytest.raises(AssertionError):replay_prefix(a)


def test_original_attachment_wrap_dwell_progress_censor():
    f=np.array([[0,0,np.pi-.01],[1,0,-np.pi+.01]])
    t=np.arange(61)/60;p=np.c_[np.linspace(0,1,61),np.zeros(61),np.full(61,-np.pi)]
    r=attachment_loss(t,p,f);assert r['join_success'] and r['join_time_s']==0 and r['progress_monotonic']
    p[:,1]=.2;p[20,1]=0
    assert attachment_loss(t,p,f)['observation_status']=='TRANSIENT_ENTRY_THEN_EXIT'
    p[-4:,1]=0
    assert attachment_loss(t,p,f)['observation_status']=='TUBE_ENTERED_DWELL_RIGHT_CENSORED'


def test_saved_windows_not_padded_and_full_metrics():
    from shapely.geometry import box
    from reconciliation.gp_se2_environment import HospitalEnvironment
    # Existing environment constructor is covered below through a minimal direct mock.
    class Free:
        def check_polyline(self,p):return dict(minimum_clearance_m=1.,clearance_valid=True)
        def check_trajectory(self,*args,**kw):return dict(clearance_valid=True,minimum_clearance_lower_bound_m=1.)
    import yaml
    cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/osa03_native_continuation_01.yaml').read_text())
    dt=float(np.float32(1/60));p=[np.zeros(3)];cmd=[]
    for i in range(180):
        cmd.append(dict(v_mps=.2,omega_radps=0,reason='new_solve' if i%6==0 else 'hold'))
        p.append(integrate_unicycle(p[-1],[.2,0],dt))
    r=dict(states=[dict(time_s=i*dt,pose_world=x.tolist()) for i,x in enumerate(p)],commands=cmd,phase=dict(u_minus=[.2,0],previous_command_application_sim_s=-.1,B_sim_s=0),termination='OBSERVATION_CAP')
    f=np.array([[0,0,0],[.8,0,0]]);s=evaluate(r,f,Free(),{'center_xy':[1,1],'forward_xy':[1,0]},cfg)
    assert all(w['available'] for w in s['windows'].values())
    assert s['full']['position_auc_m_s']<1e-12
    assert s['classification']=='NATIVE_CONTINUATION_SAFE_SUSTAINED_ATTACHMENT'
    assert s['endpoint']['remaining_arc_m']>0
    r['commands']=r['commands'][:10];r['states']=r['states'][:11]
    s=evaluate(r,f,Free(),{'center_xy':[1,1],'forward_xy':[1,0]},cfg)
    assert not s['windows']['18']['available']


@pytest.mark.parametrize('term,join,expected',[
 ('OBSERVATION_CAP',True,'NATIVE_CONTINUATION_SAFE_SUSTAINED_ATTACHMENT'),
 ('OBSERVATION_CAP',False,'NATIVE_CONTINUATION_SAFE_NO_SUSTAINED_ATTACHMENT'),
 ('SAFETY_ABORT_BEFORE_UNSAFE_COMMAND',True,'NATIVE_CONTINUATION_SAFETY_ABORT'),
 ('CONTROLLER_ERROR',True,'NATIVE_CONTINUATION_CONTROLLER_FAILURE')])
def test_factual_classification(term,join,expected):assert classification(term,join)==expected


def test_no_lookahead_model_or_optimization_import():
    root=Path(__file__).parents[1]
    import ast
    for name in ['src/reconciliation/osa03_native.py','scripts/run_osa03_native_continuation.py','scripts/osa03_native_mpc_worker.py']:
        tree=ast.parse((root/name).read_text())
        imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert all(not any(b in s for b in ['gp_se2_ref02','formulation','lightnav_client','lightnav_worker']) for s in imports)
    assert next_control_tick(92)==96 and next_control_tick(98)==102


def test_guard_aborts_without_command_or_reference_repair():
    from reconciliation.join_online02 import guard_check
    class Unsafe:
        def check_trajectory(self,*args,**kwargs):return dict(clearance_valid=False)
    c={'v_mps':.8,'omega_radps':.5,'reason':'new_solve'}
    saved=deepcopy(c);g=guard_check(Unsafe(),np.zeros(3),c,1/60)
    assert not g['safe'] and c==saved and not g['command_modified'] and not g['raw_reference_consulted']


def test_exclusive_artifacts_and_static_selection_keys(tmp_path):
    from reconciliation.join_source03 import save
    f=tmp_path/'raw.json';save(f,{'raw':[1,2,3]})
    with pytest.raises(FileExistsError):save(f,{'raw':[4,5,6]})
    root=Path(__file__).parents[1]
    source=(root/'scripts/report_osa03_native_continuation.py').read_text()
    assert "['indices']" in source and "['selected_indices']" not in source


def test_source_phase_is_verified_from_saved_artifacts_when_available():
    root=Path(__file__).parents[1]
    source=root/'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    if not source.exists():pytest.skip('Local immutable saved-source contract; no inference/solve')
    import sys
    sys.path[:0]=[str(root/'scripts'),str(root/'scripts/isaac')]
    from run_osa03_native_continuation import source_phase
    p,f,raw,_=source_phase(source)
    assert p['B_tick']==92 and p['next_submit_after_B']==96 and p['first_new_submit_tick']==102
    assert replay_prefix(p)['maximum_pose_error']==0
    np.testing.assert_allclose(local_trajectory_to_world(p['fresh_capture_pose'],raw),f,rtol=0,atol=1e-12)
    assert p['u_minus']!=p['u_mem_B']
