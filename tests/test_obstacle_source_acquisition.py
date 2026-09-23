"""Synthetic implementation fixtures only; never model/source evidence."""
from copy import deepcopy
import numpy as np
import pytest
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.obstacle_source_acquisition import select_side,paired_frames,future_at_pose,geometry,qualify,timing_gate,moving_boundary
from reconciliation.join_source03 import save
from reconciliation.se2 import local_trajectory_to_world

GATES=dict(lateral_m=.1,angle_deg=20.,reliable_chord_m=.02,interaction_before_front_m=.5,remaining_rows=4,remaining_arc_m=.6)
SCENARIO=dict(center_xy=[1.,0.],forward_xy=[1.,0.],cart_extents=[-.1,.1])


def envs():
    base=HospitalEnvironment(box(4.,4.,5.,5.),box(-10,-10,10,10),metadata={})
    on=HospitalEnvironment(box(.9,-.2,1.1,.2),box(-10,-10,10,10),metadata={})
    return base,on


def paths():
    base,on=envs();off=np.c_[np.linspace(.1,2.,20),np.zeros(20),np.zeros(20)]
    turned=np.array([[.1,0,0],[.3,-.3,-.5],[.6,-.6,-.2],[.9,-.6,0],[1.2,-.6,0],[1.5,-.6,0],[1.8,-.4,.2]])
    a=geometry(off,[0,0,0],base,on,SCENARIO);b=geometry(turned,[0,0,0],base,on,SCENARIO)
    a['stop']=b['stop']=False
    return a,b


def test_side_coordinate_tie_and_unavailable():
    p=[dict(side=s,check=dict(clearance_valid=True,minimum_clearance_m=c)) for s,c in [(-1,.07),(1,.06)]]
    assert select_side(p)=='RIGHT'
    p[1]['check']['minimum_clearance_m']=.08;assert select_side(p)=='LEFT'
    p[0]['check']['minimum_clearance_m']=.08;assert select_side(p)=='RIGHT'
    for x in p:x['check']['clearance_valid']=False
    assert select_side(p) is None


def test_whole_polyline_and_safe_turning_gates():
    a,b=paths();q=qualify(a,b,SCENARIO,GATES)
    assert q['qualified'] and a['geometry_on']['clearance_valid'] is False
    assert a['geometry_off']['clearance_valid'] and b['geometry_on']['clearance_valid']
    assert b['geometry_on']['checked_row_count']==len(b['world'])
    assert b['geometry_on']['connector_included'] is False


@pytest.mark.parametrize('field,value,gate', [('stop',True,'E_TASK_PROGRESS'),('arc_m',.1,'E_TASK_PROGRESS'),('forward_progress_m',0.,'E_TASK_PROGRESS')])
def test_stop_shortening_and_no_progress_rejected(field,value,gate):
    a,b=paths();b[field]=value
    assert qualify(a,b,SCENARIO,GATES)['gates'][gate] is False


def test_text_only_turn_and_unsafe_prefix_rejected():
    a,b=paths();a['max_lateral_m']=.2
    assert not qualify(a,b,SCENARIO,GATES)['gates']['A_OFF']
    base,on=envs();bad=np.array([[1,0,0],[1,-.6,0],[2,-.6,0]])
    r=geometry(bad,[0,0,0],base,on,SCENARIO)
    assert not r['geometry_on']['clearance_valid']
    assert r['geometry_on']['checked_row_count']==3


@pytest.mark.parametrize('n',[4,10,17])
def test_generic_N_observation_anchor_and_future(n):
    base,on=envs();a=np.c_[np.linspace(.1,1.,n),np.zeros(n),np.zeros(n)];saved=a.copy()
    pose=[-2,-2,np.pi-1e-8];r=geometry(a,pose,base,on,SCENARIO)
    np.testing.assert_array_equal(a,saved)
    np.testing.assert_allclose(r['world'],local_trajectory_to_world(pose,a))
    assert r['N']==n and r['intrinsic_waypoint_dt'] is None
    assert future_at_pose(a,a[-1])['remaining_arc_m']==0


def test_actual_B_not_memory_reset_and_timing():
    r=moving_boundary([.8,.1],[.8,.5],.04,.1)
    assert r['valid'] and not r['memory_equals_physical']
    assert not moving_boundary([.2,0],[.8,0],.04,.1)['valid']
    assert timing_gate([1.,.8,1.2],.25)['qualified']
    assert not timing_gate([1.,1.21,1.],.24)['qualified']
    assert not timing_gate([1.,1.,1.],.251)['qualified']
    assert not timing_gate([None,1.,1.],.1)['qualified']


def bank():
    out=[]
    for i in range(14):
        row={}
        for branch in ('OFF','ON'):
            row[branch]=dict(frame=dict(frame_id=str(i),path=f'/{i}/{branch}.jpg',pose_world=[i,0,0],camera={},
                capture_sim_time_s=i*.25,capture_monotonic_ns=i+1,sha256=branch+str(i)))
        out.append(row)
    return out


def test_pair_identity_history_and_no_padding():
    b=bank();r=paired_frames(b,10)
    assert r['OFF'][:-1]==r['ON'][:-1] and len(r['ON'])==8
    assert r['ON'][-1]['path']=='/10/ON.jpg'
    assert [f['frame_id'] for f in r['OFF']]==[str(i) for i in range(3,11)]
    b[10]['ON']['frame']['pose_world']=[1,2,3]
    with pytest.raises(ValueError):paired_frames(b,10)
    with pytest.raises(ValueError):paired_frames(b,4)


def test_immutable_writer(tmp_path):
    p=tmp_path/'record.json';save(p,dict(a=1))
    with pytest.raises(FileExistsError):save(p,dict(a=2))


def test_frozen_order_no_search_and_no_optimizer_imports():
    import ast
    from pathlib import Path
    import yaml
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'configs/obstacle_source_acquisition_01.yaml').read_text())
    assert cfg['candidate_bank_indices']==[10,11,12,13]
    assert cfg['execute_all_candidates'] and cfg['phaseB']['repetitions']==2
    for p in [root/'src/reconciliation/obstacle_source_acquisition.py',root/'scripts/lightnav/obstacle_source_predict.py']:
        names={n.id for n in ast.walk(ast.parse(p.read_text())) if isinstance(n,ast.Name)}
        assert names.isdisjoint({'solve_gp','solve_rigid','GPProblem','minimize','MpcTracker'})


def test_H8_replay_parity_and_clock_domains():
    from reconciliation.obstacle_source_history import FrozenHistory
    from reconciliation.online_history import SessionHistory
    from test_online_history import frame,contract
    frames=[frame(i,capture_monotonic_ns=10**15+i*250_000_000) for i in range(8)]
    c=contract(64,'ring');h=FrozenHistory('fixed',c,None,frames);original=SessionHistory('fixed',c,None)
    for x in (h,original):x.login();x.reset()
    for i,f in enumerate(frames):
        a=h.begin(f,predict=i==7,send_monotonic_ns=100+i)
        b=original.begin(f,predict=i==7,send_monotonic_ns=f['capture_monotonic_ns']+100)
        assert a['frame']==b['frame']
        aa=h.complete(i,server_actions_step=8 if i==7 else None)
        bb=original.complete(i,server_actions_step=8 if i==7 else None)
        assert aa.pop('clock_semantics')['capture_to_request_age_s'] is None
        assert aa==bb
    assert h.prediction_count==1
    with pytest.raises(ValueError):h.begin(frames[-1],predict=True,send_monotonic_ns=999)


def test_declaration_strings_and_exact_instruction():
    import yaml
    from pathlib import Path
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/obstacle_source_acquisition_01.yaml').read_text())
    assert cfg['branches']==['OFF','ON']
    assert cfg['instructions']['RIGHT']=='Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.'
