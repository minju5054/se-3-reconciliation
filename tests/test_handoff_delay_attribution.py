"""Synthetic implementation checks only; no MPC/VLA scientific evidence."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
import pytest
from reconciliation.handoff_delay_attribution import (memory_at,boundary_state,fixed_references,DT,STEPS,IDS,CONDITIONS,category_change,paired,value_hash)
from reconciliation.gp_se2_ref02_reference import select_source_progress,audit_source_progress
from reconciliation.handoff_execution_loss import attachment_loss
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import local_trajectory_to_world
ROOT=Path(__file__).resolve().parents[1]

def result(lo=2.,hi=3.,u=(.6,.2),episode='e',status='command'):
    return dict(type='solve_result',episode_id=episode,status=status,command=list(u),result_seen_worker_host_monotonic_s=lo,seen_in_isaac={'host_monotonic_s':hi})
def submit(t,u):return dict(type='reply',status='submitted',episode_id='e',submit_host_monotonic_s=t,previous_command=list(u))

def test_observation_memory_separate_from_later_B_memory():
    events=[submit(1,[.4,0]),result()]
    assert memory_at(events,'e',1.5)['previous_control']==[.4,0]
    assert memory_at(events,'e',3.1)['previous_control']==[.6,.2]
    assert not memory_at(events,'e',2.5)['available']

def test_submit_tightens_poll_bound():
    events=[submit(1,[.4,0]),result(2,4),submit(2.1,[.6,.2])]
    assert memory_at(events,'e',2.2)['previous_control']==[.6,.2]

def test_missing_memory_not_zero_and_stale_does_not_update():
    assert not memory_at([],'e',2)['available']
    assert memory_at([submit(1,[.4,.1]),result(status='stale_rejected')],'e',5)['previous_control']==[.4,.1]
    assert not memory_at([submit(1,[.4,.1]),result(status='controller_error')],'e',5)['available']

def test_previous_episode_result_cannot_seed_memory():
    assert not memory_at([result(1,2,episode='old')],'e',3)['available']

def test_same_value_poll_overlap_is_not_ambiguous():
    assert memory_at([submit(1,[.6,.2]),result()],'e',2.5)['previous_control']==[.6,.2]

def test_actual_physical_incoming_boundary_reconstruction():
    a=[1.,2.,.5];u=[.4,.2];b=integrate_unicycle(a,u,DT).tolist()
    states=[dict(state_id=0,sim_time_s=0,x=a[0],y=a[1],yaw=a[2],incoming_command_id=-1),
            dict(state_id=1,sim_time_s=DT,x=b[0],y=b[1],yaw=b[2],incoming_command_id=0)]
    commands=[dict(command_id=0,application_state_id=0,host_monotonic_s=1.,v_mps=.4,omega_radps=.2)]
    context=dict(episode_id='e',obs_state_id=1,switch_state_id=1,R_obs=b,B=b,t_obs={'host_monotonic_s':2.},t_switch={'host_monotonic_s':2.})
    q=boundary_state(context,states,commands,[submit(1.5,[.6,.7])],'LATENCY_FREE')
    assert q['u_minus']==u and q['memory']['previous_control']==[.6,.7] and q['pose_world']==b
    q=boundary_state(context,states,commands,[],'LATENCY_FREE');assert q['reason']=='ORACLE_CONTROLLER_MEMORY_UNAVAILABLE'

@pytest.mark.parametrize('n',[2,7,10,19])
def test_reference_not_reanchored_and_shared_for_both_cuts(n):
    raw=np.column_stack([np.linspace(.1,1.5,n),np.linspace(.0,.2,n),np.linspace(3.1,3.3,n)])
    before=raw.copy();obs=[5.,7.,.6];fresh=local_trajectory_to_world(obs,raw);B=[5.4,7.1,.9]
    r=fixed_references(fresh,B)
    assert np.array_equal(raw,before) and np.array_equal(r['native'],fresh)
    assert r['common_hash']==value_hash(r['common']) and r['prepared_at']=='B_ONCE'
    assert len(r['common'])==30 and r['semantics']=='PREPARED_REFERENCE_PLUS_SOURCE_PROGRESS_SELECTOR'
    assert r==fixed_references(fresh,B)

@pytest.mark.parametrize('n',[1,2,10,23])
def test_integer_raw_progress_native_equivalence(n):
    path=np.column_stack([np.arange(n)*.1,np.zeros(n),np.linspace(3.0,3.5,n)]);pose=[0,0,-3.1]
    actual,info=select_source_progress(path,pose,np.arange(n,dtype=float),horizon=5,weights=[10,10,1],constant_reference=n==1)
    j=info['nearest_index'];assert info['indices']==[min(j+h,n-1) for h in range(1,6)]
    audit_source_progress(path,pose,np.arange(n,dtype=float),actual,horizon=5,weights=[10,10,1],constant_reference=n==1)

def test_endpoint_constant_reference_preserves_lineage():
    raw=np.array([[0.,0.,0.],[1.,0.,.1]])
    r=fixed_references(raw,[1.,0.,.1]);assert r['constant_metadata']['original_source_row_index']==1
    assert all(row['original_fractional_row_coordinate']==1 for row in r['common_lineage'])

def test_equal_exposure_schedule():
    assert STEPS==54 and STEPS*DT==pytest.approx(.9000000469386578,abs=1e-15)
    assert list(range(0,STEPS,6))==[0,6,12,18,24,30,36,42,48]

def test_dwell_transient_and_censoring_not_failure_time():
    t=np.arange(55)*DT;fresh=np.array([[0.,0.,0.],[2.,0.,0.]])
    poses=np.zeros((55,3));poses[:,0]=.5;poses[:,1]=.3;poses[10,1]=0
    a=attachment_loss(t,poses,fresh);assert a['join_time_s'] is None and a['observation_status']=='TRANSIENT_ENTRY_THEN_EXIT'
    poses[50:,1]=0;a=attachment_loss(t,poses,fresh);assert a['observation_status']=='TUBE_ENTERED_DWELL_RIGHT_CENSORED'
    poses[:]=[.5,0,2*np.pi];a=attachment_loss(t,poses,fresh);assert a['join_success'] and a['join_time_s']==0

def test_category_changes_do_not_order_censoring():
    assert category_change('NO_TUBE_ENTRY_OBSERVED','OBSERVED_SAMPLED_JOIN')=='IMPROVED_TO_OBSERVED_JOIN'
    assert category_change('TRANSIENT_ENTRY_THEN_EXIT','TUBE_ENTERED_DWELL_RIGHT_CENSORED')=='CHANGED_CENSORING_NOT_ORDERED'

def test_frozen_cohort_and_no_extra_solver_or_model_paths():
    assert len(IDS)==13 and len(set(IDS))==13 and len(CONDITIONS)==4
    import ast
    for p in [ROOT/'scripts/run_handoff_delay_attribution.py',ROOT/'scripts/lightnav/handoff_delay_mpc_worker.py',ROOT/'src/reconciliation/handoff_delay_attribution.py']:
        tree=ast.parse(p.read_text())
        banned={'solve_gp','run_instrumented','minimize','LightNavClient','IsaacApp','SimulationApp','request_prediction'}
        called={n.func.id if isinstance(n.func,ast.Name) else n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))}
        assert not called & banned

def test_output_refuses_overwrite(tmp_path):
    sys.path[:0]=[str(ROOT/'scripts')]
    from run_handoff_delay_attribution import save
    p=tmp_path/'raw.json';save(p,{'a':1})
    with pytest.raises(FileExistsError):save(p,{'a':2})
    assert json.loads(p.read_text())=={'a':1}

@pytest.mark.parametrize('abort',[False,True])
def test_bounded_execution_loop_with_fixture_worker(tmp_path,monkeypatch,abort):
    """Entire runner with synthetic commands; no official controller instantiated."""
    sys.path[:0]=[str(ROOT/'scripts')]
    import run_handoff_delay_attribution as run
    cfg={};event=dict(case_id='test/e',episode_id='test',cohort='GENUINE_13',available=True,reason=None,
        initials={k:dict(available=True,reason=None,pose_world=[0.,0.,0.],u_minus=[.1,0.],memory={'previous_control':[.1,0.]}) for k in ['DELAYED','LATENCY_FREE']},
        references=fixed_references(np.array([[.1,0,0],[1.,0,0]]),[0.,0.,0.]),fresh_world=[[.1,0,0],[1.,0,0]],
        frozen=dict(original_capture_pose_world=[0.,0.,0.],fresh_raw_local=[[.1,0,0],[1.,0,0]]))
    cfg['checkout']='fixture';run.save(tmp_path/'protocol.json',cfg);run.save(tmp_path/'events.json',[event])
    monkeypatch.setattr(run,'verify',lambda p:{'execution_sha':'fixture'})
    monkeypatch.setattr(run,'git',lambda *a:'fixture');monkeypatch.setattr(run,'envs',lambda c:{'GENUINE_13':None})
    instances=[]
    class Worker:
        def __init__(self,*a):pass
        def call(self,q):
            if q['op']=='init':instances.append(q);return {}
            if q['op']=='solve':return dict(command=[.1,0.])
            return {}
        def close(self):pass
    monkeypatch.setattr(run,'Worker',Worker)
    def guard(env,pose,command,dt):return dict(safe=not abort or pose[0]<.003,command=command,start_pose=np.asarray(pose).tolist())
    monkeypatch.setattr(run,'guard_check',guard);run.execute(tmp_path)
    assert len(instances)==4
    assert instances[0]['reference']==instances[2]['reference'] and instances[1]['reference']==instances[3]['reference']
    for c in CONDITIONS:
        r=run.read(tmp_path/'rollouts/test__e'/f'{c}.json')
        assert r['status']==('SAFETY_ABORT' if abort else 'COMPLETED')
        assert len(r['poses_world'])==len(r['commands'])+1
        if not abort:assert len(r['solves'])==9 and len(r['commands'])==54
        else:assert not r['abort']['applied'] and len(r['commands'])<54
    with pytest.raises(FileExistsError):run.execute(tmp_path)

def test_paired_arithmetic_and_missing_equal_exposure():
    from reconciliation.handoff_delay_attribution import AUC
    rows=[]
    for c,x in zip(CONDITIONS,[4.,3.,2.,1.]):
        rows.append(dict(case_id='e/h',episode_id='e',cohort='GENUINE_13',condition=c,
           primary={**{k:x for k in AUC},'observation_status':'NO_TUBE_ENTRY_OBSERVED'},
           geometry={'clearance_valid':True},command={'nominal_command_grid_valid':True},controller_failures=0))
    gaps,transitions=paired(rows)
    assert [g['position_auc_m_s'] for g in gaps]==[2.,2.,1.,1.,0.]
    rows[2]['primary']=None;gaps,_=paired(rows)
    assert gaps[0]['position_auc_m_s'] is None and not gaps[0]['available'] and gaps[4]['position_auc_m_s'] is None

def test_native_stdout_cannot_corrupt_protocol_pipe():
    """Reproduce native-print framing issue without importing/solving CasADi."""
    import subprocess
    code="""
import sys,os,json
sys.path.insert(0,'scripts/lightnav')
from handoff_delay_mpc_worker import protocol_stream
out=protocol_stream()
os.write(1,b'NATIVE LIBRARY BANNER\\n')
print(json.dumps({'ok':True}),file=out,flush=True)
"""
    q=subprocess.run([sys.executable,'-c',code],cwd=ROOT,capture_output=True,text=True,check=True)
    assert json.loads(q.stdout)=={'ok':True} and q.stderr=='NATIVE LIBRARY BANNER\n'
