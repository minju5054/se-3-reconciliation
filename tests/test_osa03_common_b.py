"""Synthetic contract tests and saved R00 integrity checks. No scientific solves."""
import ast,hashlib,sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
ROOT=Path(__file__).parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.osa03_common_b import *
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.join_source03 import save,sha
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.handoff_execution_loss import attachment_loss


def inputs(n=10):
    A=np.array([3.,-2.,3.12]);B=np.array([3.2,-1.8,-3.1]);raw=np.c_[np.linspace(.1,1,n)**2,np.linspace(0,.5,n),np.linspace(.1,.5,n)]
    F=local_trajectory_to_world(A,raw);M=F.copy();M[0,0]+=.03
    return A,B,raw,F,M


@pytest.mark.parametrize('n',[2,3,10,17])
def test_frozen_methods_generic_N_original_frame_no_mutation(n):
    A,B,raw,F,M=inputs(n);before=[x.copy() for x in [A,B,raw,F,M]];r=references(A,B,F,M)
    assert list(r)==ORDER
    np.testing.assert_array_equal(r[ORDER[0]],F);np.testing.assert_array_equal(r[ORDER[3]],M)
    np.testing.assert_allclose(r[ORDER[1]][0],r[ORDER[2]][0],atol=1e-12)
    np.testing.assert_allclose(r[ORDER[1]][-1],F[-1],atol=1e-12)
    np.testing.assert_allclose(relative_pose(B,r[ORDER[2]]),raw,atol=1e-12)
    for x in r.values():np.testing.assert_allclose(local_trajectory_to_world(A,relative_pose(A,x)),x,atol=1e-12)
    for x,y in zip([A,B,raw,F,M],before):np.testing.assert_array_equal(x,y)
    r[ORDER[0]][0,0]+=1;np.testing.assert_array_equal(F,before[3])


def test_taper_arc_not_row_and_se2_not_global_addition():
    A,B,_,F,M=inputs(4);r=references(A,B,F,M);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(F[:,:2],axis=0),axis=1))];s=arc/arc[-1]
    assert not np.allclose(s,np.linspace(0,1,4))
    g=compose_poses(B,inverse_pose(A));expected=compose_poses(se2_exp((1-s)[:,None]*se2_log(g)),F)
    np.testing.assert_array_equal(r[ORDER[1]],expected)
    assert not np.allclose(expected,F+(1-s)[:,None]*(B-A))
    assert distortion(F,r[ORDER[2]])['relative_translation_RMS_m']<1e-12


def test_invalid_reference_and_M3_hash_fail_closed(tmp_path):
    A,B,_,F,M=inputs(3)
    for f,m in [(F[:1],M[:1]),(F,M[:2]),(np.zeros((3,3)),M)]:
        with pytest.raises(ValueError):references(A,B,f,m)
    p=tmp_path/'m.npy';np.save(p,M);np.testing.assert_array_equal(authenticated_m3(p,sha(p)),M)
    with pytest.raises(ValueError):authenticated_m3(p,'bad')
    with pytest.raises(FileExistsError):
        save(tmp_path/'raw.json',{'x':1});save(tmp_path/'raw.json',{'x':2})


def fixture_common():
    first=dict(type='solve_result',status='command',solve_id='saved005',chunk_id='fresh',reference_version=1,command=[.8,.5],previous_command=[.8,0],official_solve_ms=4.,prediction_world=[[0,0,0]],selection={'reference_world':[[0,0,0]]})
    p=dict(B=[1,2,.1],B_tick=92,B_sim_s=1.5,u_minus=[.8,0],u_B_plus=[.8,.5],u_mem_B=[.8,.5],delta_u_B=[0,.5],previous_command_application_sim_s=1.4,fresh_capture_pose=[.8,2,.1],fresh_chunk_id='fresh',fresh_version=1,original_generation=3,integration_dt_s=float(np.float32(1/60)),next_submit_after_B=96,first_FRESH_solve=first,
        applications=[dict(application_tick=92,result=first),dict(application_tick=97,result={'solve_id':'forbidden006'})])
    return common_state(p)


def test_distinct_physical_memory_no_future_replay_shared_activation():
    c=fixture_common();assert 'applications' not in c and c['u_minus']!=c['u_mem_B']
    assert c['B_tick']!=c['next_submit_after_B']==96
    aa=[activation_at_B(c) for _ in ORDER]
    assert len({id(x) for x in aa})==4
    st=dict(state_id=92,tick=92,sim_time_s=1.5,episode_time_s=0,host_monotonic_s=0,host_monotonic_ns=0,host_utc='fixture',x=1,y=2,yaw=.1)
    for a in aa:
        _,command,event=preview_activation(a,st,None,None)
        assert event is None and command['solve_id']=='saved005' and command['reason']=='new_solve'
        assert [command['v_mps'],command['omega_radps']]==c['u_B_plus']


def test_preclock_restore_mock_no_numerical_solve(tmp_path):
    from osa03_common_b_mpc_worker import initialize
    c=fixture_common();A,B,raw,F,M=inputs();c['fresh_capture_pose']=A.tolist()
    local=tmp_path/'local.npy';world=tmp_path/'world.npy';np.save(local,raw);np.save(world,F)
    class Adapter:
        def __init__(self):self.episode_id=None;self.used_solve_ids=set();self.pending=None;self.tracker=SimpleNamespace(_future=None)
        def reset(self,identity):self.episode_id=identity
        def install(self,**kw):
            assert kw['setup_only_before_counterfactual_clock'];self.world=local_trajectory_to_world(kw['capture_pose'],np.load(kw['raw_local_path']));self.installed={'official_generation':1}
            return kw
    adapters=[Adapter() for _ in ORDER]
    for a,n in zip(adapters,ORDER):
        r=initialize(a,c,dict(local_path=str(local),world_path=str(world),local_sha256=sha(local),world_sha256=sha(world)),n)
        assert r['new_solve_calls']==r['install_events_during_rollout']==r['restored_future_results']==0
        assert r['held_command']==c['u_B_plus'] and r['previous_control']==c['u_mem_B'] and r['generation']==3
    adapters[0].tracker.previous_command=(0,0)
    assert adapters[1].tracker.previous_command==tuple(c['u_mem_B'])


def rollout_schedule(app=97,term='OBSERVATION_CAP'):
    return dict(phase=fixture_common(),events=[dict(type='reply',status='submitted',solve_id='new',input_state_id=96),dict(type='solve_result',status='command',solve_id='new',input_state_id=96)],submit_requests=[dict(tick=96)],commands=[dict(application_tick=92,solve_id='saved005',reason='new_solve'),dict(application_tick=app,solve_id='new',reason='new_solve')]+[dict(application_tick=i,reason='hold') for i in range(98,152)],termination=term)


def test_timing_audit_exact_app_ticks_not_only_nominal_frequency():
    a=rollout_schedule();b=rollout_schedule(98)
    assert timing_audit({'a':a,'b':deepcopy(a)})['comparable']
    assert not timing_audit({'a':a,'b':b})['comparable']
    assert timing_audit({'a':a,'skip':None})['comparable']
    b['commands']=b['commands'][:10];assert not timing_audit({'a':a,'b':b})['complete_primary_exposure']


def test_skipped_unsafe_NA_timing_and_signed_gaps():
    assert not may_execute({'clearance_valid':False}) and metric_row(None) is None
    r={ORDER[0]:rollout_schedule(),ORDER[1]:None};safe={ORDER[0]:True,ORDER[1]:False}
    assert classify(safe,r,timing_audit(r))['primary']=='REFERENCE_LIMITED_COMMON_B_COMPARISON'
    gaps=signed_gaps({'M0_NATIVE':{'auc':1.,'join':None},'M1_SE2_TAPER':{'auc':.3,'join':.1},'M2_RIGID_TRANSPORT':None})
    assert gaps['M1_SE2_TAPER']=={'auc':-.7,'join':None} and gaps['M2_RIGID_TRANSPORT'] is None
    r[ORDER[0]]['termination']='HOLD_TIMEOUT';assert classify(safe,r,timing_audit(r))['primary']=='TECHNICAL_EXECUTION_BLOCKED'


def test_attachment_original_FRESH_wrap_dwell_not_method_reference():
    f=np.array([[0,0,np.pi-.01],[1,0,-np.pi+.01]]);t=np.arange(61)/60;p=np.c_[np.linspace(0,1,61),np.zeros(61),np.full(61,-np.pi)]
    assert attachment_loss(t,p,f)['join_time_s']==0
    shifted=f.copy();shifted[:,1]=.3;assert not attachment_loss(t,p,shifted)['join_success']
    p[:,1]=.2;p[20,1]=0;assert not attachment_loss(t,p,f)['join_success']
    p[-4:,1]=0;assert attachment_loss(t,p,f)['observation_status']=='TUBE_ENTERED_DWELL_RIGHT_CENSORED'
    p[:,0]=np.linspace(1,0,len(p));assert attachment_loss(t,p,f)['progress_monotonic']


def test_native_selector_and_exact_integration():
    f=np.c_[np.linspace(0,1,7),np.zeros(7),np.zeros(7)];p=[.21,0,0];idx=[2,3,4,5,6]
    assert selection_audit(f,p,f[idx],horizon=5,weights=[10,10,1])['indices']==idx
    x=np.array([1.,2.,.3]);dt=float(np.float32(1/60));u=[.8,.5]
    y=x.copy()
    for _ in range(6):y=integrate_unicycle(y,u,dt)
    np.testing.assert_allclose(y,integrate_unicycle(x,u,dt*6),atol=1e-12)


def test_guard_aborts_only_without_modification():
    class Unsafe:
        def check_trajectory(self,*args,**kwargs):return dict(clearance_valid=False)
    cmd=dict(v_mps=.8,omega_radps=.5,reason='new_solve');before=deepcopy(cmd)
    g=guard_check(Unsafe(),[0,0,0],cmd,1/60)
    assert not g['safe'] and not g['command_modified'] and cmd==before


def test_no_new_optimization_VLA_lookahead_and_fixed_execution_structure():
    for filename in ['src/reconciliation/osa03_common_b.py','scripts/run_osa03_common_b.py','scripts/osa03_common_b_mpc_worker.py']:
        tree=ast.parse((ROOT/filename).read_text());calls=[n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id if isinstance(n.func,ast.Name) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)]
        assert not set(calls)&{'solve_graph','solve_rigid','solve_least_squares','least_squares','minimize','generate','capture_rgb'}
        imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert not any('ref02' in n or 'lightnav_worker' in n for n in imports)
    text=(ROOT/'scripts/run_osa03_common_b.py').read_text();assert 'exist_ok=False' in text and "assert tick>=common['next_submit_after_B'] and tick!=bt" in text


def test_saved_R00_and_frozen_M3_contract_no_solve():
    from run_osa03_native_continuation import source_phase
    source=ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    if not source.exists():pytest.skip('Local sealed artifacts unavailable')
    p,F,raw,_=source_phase(source);c=common_state(p)
    assert c['B_tick']==92 and c['next_submit_after_B']==96
    assert sha(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy')=='8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521'
    M=authenticated_m3(ROOT/'data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z/derived/optimized_world.npy')
    refs=references(c['fresh_capture_pose'],c['B'],F,M)
    np.testing.assert_array_equal(refs[ORDER[3]],M);np.testing.assert_array_equal(refs[ORDER[0]],F)


def test_runner_validator_end_to_end_mock_controller_no_real_solve(tmp_path,monkeypatch):
    import run_osa03_common_b as runner
    from validate_osa03_common_b import validate_method
    bank=ROOT/'data/osa03_common_b_method_comparison_01/primary_20260925T162125Z'
    if not bank.exists():pytest.skip('Local preflight bank unavailable; synthetic worker only')
    from reconciliation.join_source03 import read
    for filename in ['protocol.json','metric_protocol.json','common_state.json','source_manifest.json','references.json']:
        save(tmp_path/filename,read(bank/filename))
    c=read(tmp_path/'common_state.json');ref=read(bank/'references.json')[ORDER[0]];world=np.load(ref['world_path']);manifest=read(bank/'source_manifest.json')
    (tmp_path/'references').mkdir();np.save(tmp_path/'references/M0_NATIVE_world.npy',world)
    class FakeWorker:
        def __init__(self,*a,**kw):self.queue=[];self.prev=c['u_mem_B'];self.initialized=False
        def await_type(self,kind,**kw):return {'provenance':manifest['mpc']}
        def drain(self):out=self.queue;self.queue=[];return out
        def send(self,op,**kw):
            assert self.initialized and op=='submit' and kw['input_state_id']>=96
            p=kw['pose'];near=np.argmin(np.sum((world-p)**2*np.array([10,10,1]),axis=1));idx=np.minimum(near+np.arange(1,6),len(world)-1)
            self.queue.append(dict(type='reply',status='submitted',previous_command=self.prev,**kw))
            ev=deepcopy(c['first_FRESH_solve']);ev.update(kw,input_pose=p,previous_command=self.prev,command=[0.,0.],official_solve_ms=0,selection=selection_audit(world,p,world[idx],horizon=5,weights=[10,10,1]))
            ev.pop('pose',None);self.prev=ev['command'];self.queue.append(ev)
        def shutdown(self):pass
    def fake_ask(worker,op,**kw):
        assert op=='initialize_common_B';worker.initialized=True
        return dict(B=c['B'],held_command=c['u_B_plus'],previous_control=c['u_mem_B'],generation=3,next_submit_tick=96,B_tick=92,B_sim_s=c['B_sim_s'],integration_dt_s=c['integration_dt_s'],installed_world=world.tolist())
    monkeypatch.setattr(runner,'Worker',FakeWorker);monkeypatch.setattr(runner,'ask',fake_ask);monkeypatch.setattr(runner.time,'sleep',lambda _:None)
    runner.run_method(tmp_path,ORDER[0]);r=read(tmp_path/'methods'/ORDER[0]/'rollout.json')
    assert r['error'] is None and len(r['commands'])==180 and r['new_MPC_solved']==30
    source=Path(manifest['source']);env=runner.geometry(source)['on']
    checked=validate_method(tmp_path,ORDER[0],ref,c,world,env,read(source/'scenario.json'),manifest['mpc']['official_settings'])
    assert checked[-1]['valid']

    # Saved-only report smoke test, including unavailable traces (no model/controller).
    import validate_osa03_common_b as validator
    import report_osa03_common_b as reporter
    for n in ORDER[1:]:
        (tmp_path/'methods'/n).mkdir()
    metric=read(tmp_path/'methods'/ORDER[0]/'metrics.json');rows={n:metric_row(metric) if n==ORDER[0] else None for n in ORDER}
    summary=dict(classification={'primary':'SYNTHETIC_IMPLEMENTATION_TEST'},primary_metrics=rows,signed_method_minus_native=signed_gaps(rows))
    save(tmp_path/'summary.json',summary);save(tmp_path/'validation.json',{'valid':True});save(tmp_path/'freeze.json',{'synthetic':True})
    monkeypatch.setattr(validator,'validate',lambda _: (summary,{'valid':True}))
    reporter.report(tmp_path);assert reporter.validate_figures(tmp_path)['valid']
