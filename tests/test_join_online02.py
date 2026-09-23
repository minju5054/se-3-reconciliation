"""Synthetic implementation fixtures only; these tests never call model/MPC."""
from pathlib import Path
import copy,sys
import numpy as np
import pytest,yaml
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_online02 import (ORDER, INSTRUCTION, DISTANCES, ABORT,
    select_start, preview_activation, guard_check, hallway_geometry, classify_chunk, episode_outcome, candidate_source)
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.robotless_online import CommandActivation,dump_new
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
CFG=yaml.safe_load((ROOT/'configs/join_online_02_far_approach.yaml').read_text())

def env(obstacle=None):
    return HospitalEnvironment(box(9,9,10,10) if obstacle is None else obstacle,box(-10,-10,12,12))

def test_frozen_schedule_native_contract():
    assert CFG['order']==ORDER==['OFF_REPEAT_00','ON_REPEAT_00','OFF_REPEAT_01','ON_REPEAT_01']
    assert CFG['instruction']==INSTRUCTION=='Avoid the supply cart and continue to the end of the hallway.'
    assert CFG['candidate_center_distances_m']==DISTANCES
    assert CFG['maximum_activated_after_C0']==20 and CFG['maximum_sim_s_after_C0']==25
    assert not CFG['guard']['raw_future_unsafe_aborts']
    assert CFG['generation_environment']==dict(VLN_EVAL_TEMPERATURE='0',VLN_EVAL_TOP_P='1',VLN_EVAL_TOP_K='0',VLN_EVAL_TRAJ_TOP1='0')

def test_farthest_geometry_only_no_new_distance():
    cart=box(-.2,-.2,.2,.2);on=env(cart)
    r=select_start(env(),on,[0,0],[1,0],cart)
    assert [x['distance_m'] for x in r['candidates']]==DISTANCES
    assert r['selected']['pose_world']==[-5.,0.,0.]
    limited=HospitalEnvironment(cart,box(-4.8,-2,2,2))
    assert select_start(env(),limited,[0,0],[1,0],cart)['selected']['distance_m']==4.5
    none=HospitalEnvironment(cart,box(-3,-2,2,2))
    assert select_start(env(),none,[0,0],[1,0],cart)['status']=='GEOMETRY_START_BLOCKER'

def state(i):return dict(state_id=i,tick=i,sim_time_s=i*.1,episode_time_s=i*.1,host_monotonic_s=1+i*.1,host_monotonic_ns=1000000000+i*100000000,host_utc='fixture',x=i*.01,y=0.,yaw=0.)
def ready(i):return dict(status='command',chunk_id=f'c{i}',reference_version=i,solve_id=f's{i}',command=[.2,.1])

def test_preview_identical_to_native_but_unapplied_has_no_B():
    m=CommandActivation();m.install('c0',0,{});m.accept(ready(0),0.)
    untouched=copy.deepcopy(m.__dict__)
    proposal,cmd,event=preview_activation(m,state(0),None,None)
    assert m.__dict__==untouched and m.active is None and not m.activations
    native=copy.deepcopy(m);nc,ne=native.apply(state(0),None,None)
    assert (cmd,event)==(nc,ne) and proposal.__dict__==native.__dict__
    assert event['B']==[0,0,0] and event['bootstrap']
    m=proposal;m.install('c1',1,{})
    _,held,event=preview_activation(m,state(1),state(0),cmd)
    assert event is None and held['chunk_id']=='c0'
    m.accept(ready(1),.2)
    _,fresh,event=preview_activation(m,state(2),state(1),held)
    assert fresh['chunk_id']=='c1' and event['B']==[.02,0,0]
    assert event['old']['chunk_id']=='c0' and m.active['chunk_id']=='c0'

def test_guard_is_unchanged_command_only_and_arc_bound():
    cmd=dict(v_mps=.8,omega_radps=2.,reason='new_solve')
    r=guard_check(env(),[0,0,0],cmd,1/60)
    assert r['safe'] and r['command']==cmd and r['duration_s']==.1
    assert not r['raw_reference_consulted'] and not r['command_modified']
    assert len(r['poses'])==7
    np.testing.assert_allclose(r['poses'][-1],integrate_unicycle([0,0,0],[.8,2],.1))
    near=env(box(.32,-.2,.7,.2))
    cmd=dict(v_mps=.8,omega_radps=0.,reason='new_solve')
    assert not guard_check(near,[0,0,0],cmd,1/60)['safe']
    cmd['reason']='hold';assert guard_check(near,[0,0,0],cmd,1/60)['safe']
    assert cmd['v_mps']==.8

def geo(p):return hallway_geometry(p,[-2,0,0],[0,0],[1,0],[-.3,.3],CFG['classification'])
def classify(p,safe=True,stop=False):return classify_chunk(geo(p),safe,stop,[-1,1],[-.3,.3],CFG['classification'])

@pytest.mark.parametrize('n',[1,3,10,17])
def test_generic_N_and_no_observation_connector(n):
    p=np.column_stack([np.linspace(-1.8,-1,n),np.zeros(n),np.zeros(n)])
    assert geo(p)['row_count']==n
    assert geo(p)['raw_arc_m']==pytest.approx(.8 if n>1 else 0.)

def test_whole_path_not_endpoint_or_yaw_only():
    assert classify([[-1,0,0],[-.6,0,0]])=='SAFE_SHORTEN'
    assert classify([[-4,0,0],[-2,0,0]])=='STRAIGHT_OR_BASELINE'
    assert classify([[-1,0,0],[-.6,0,1.]])=='SAFE_SHORTEN'
    assert classify([[-1,0,0],[-.6,.3,.7]])=='SAFE_BYPASS_ONSET'
    assert classify([[-1,0,0],[.7,.6,.5]])=='SAFE_BYPASS'
    assert classify([[-1,0,0],[.7,.6,.5]],False)=='UNSAFE_INTERSECTING'
    assert classify([[-1,0,0]],stop=True)=='SAFE_STOP'

def test_first_unsafe_row_segment_uses_entire_path():
    from reconciliation.join_source03 import path_geometry
    r=path_geometry(np.array([[-1,0,0],[0,0,0],[1,0,0]]),env(box(-.1,-.1,.1,.1)))
    assert r['first_unsafe_segment_zero_based']==0 and r['first_unsafe_waypoint_zero_based']==1
    assert not r['whole']['clearance_valid']

def test_late_response_does_not_rescue_abort_and_stop_is_distinct():
    late=[dict(classification='SAFE_BYPASS',received_before_end=False)]
    assert episode_outcome(late,ABORT)=='UNSAFE_NO_BYPASS_BEFORE_GUARD_ABORT'
    assert episode_outcome([], 'MODEL_STOP')=='NATURAL_MODEL_STOP_BEFORE_BYPASS'
    assert candidate_source([]) is None
    rows=[dict(episode='ON_REPEAT_00',classification='SAFE_BYPASS_ONSET',t_obs=1,received_before_end=True),dict(episode='ON_REPEAT_00',classification='SAFE_BYPASS',t_obs=2,received_before_end=True),dict(episode='ON_REPEAT_01',classification='SAFE_BYPASS',t_obs=.5,received_before_end=True)]
    assert candidate_source(rows) is rows[1]

def test_guard_abort_precedes_apply_and_no_request_after_break():
    s=(ROOT/'scripts/isaac/robotless_online_handoffs.py').read_text()
    block=s[s.index('if command_guard is None:'):s.index('nextpose=integrate_unicycle')]
    assert block.index("if not decision['safe']:")<block.index('activation=proposal')<block.index("commands.append(command)")
    assert 'break' in block[block.index("if not decision['safe']:"):block.index('activation=proposal')]
    assert "command_applied=False" in block

def test_exclusive_output(tmp_path):
    p=tmp_path/'record.json';dump_new(p,dict(valid=True))
    with pytest.raises(FileExistsError):dump_new(p,dict(valid=False))

def test_replay_uses_only_saved_samples_and_never_writes_model_rgb():
    from robotless_online_replay import SavedEpisode
    s=SavedEpisode.__new__(SavedEpisode);s.times=np.array([0.,.1,.2]);s.reset();s.playing=True;s.advance(.15)
    assert s.index==1
    source=(ROOT/'scripts/isaac/join_online02_replay.py').read_text()
    for forbidden in ['integrate_unicycle(','.send(','.submit(','.solve(','.predict(']:assert forbidden not in source
    assert 'SavedEpisode(' in source and 'source_url' in source

def test_saved_off_match_serializes_as_boolean(monkeypatch,tmp_path):
    import analyze_join_online02 as audit
    def episode(run,e):
        return dict(summary=dict(episode=e,outcome='NATURAL_MODEL_STOP_BEFORE_BYPASS'),rows=[dict(
            episode=e,chunk_id='chunk_000',classification='STRAIGHT_OR_BASELINE',t_obs=1.,received_before_end=True,
            observation_longitudinal_m=-4.,observation=[0.,0.,0.])])
    monkeypatch.setattr(audit,'analyze_episode',episode)
    result=audit.analyze(tmp_path)
    assert all(type(r['matched']) is bool for r in result['OFF_matches'])
