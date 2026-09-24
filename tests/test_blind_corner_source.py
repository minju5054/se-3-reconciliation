"""Synthetic implementation tests only; never model evidence."""
import ast
from pathlib import Path
import numpy as np
import pytest
import yaml
from reconciliation.blind_corner_source import (FirstCrossing,ray_first_hit,project_camera,left_corner,old_turning,motion_gate,select_representative,INSTRUCTION)
from reconciliation.join_source02 import observation_anchored_world,whole_raw_polyline_check
from reconciliation.obstacle_source_acquisition import future_at_pose,timing_gate
from reconciliation.gp_se2_environment import HospitalEnvironment
from shapely.geometry import box
ROOT=Path(__file__).resolve().parents[1]


def test_first_crossing_never_later_substituted():
    l=FirstCrossing()
    for i,n in enumerate([0,19,20,0,200]):l.observe(str(i),n,15*i)
    assert l.crossing['current']['frame_id']=='2' and l.allows('2') and not l.allows('4')
    with pytest.raises(ValueError):l.observe('4',200,90)


def test_initial_visible_is_not_a_crossing():
    l=FirstCrossing();l.observe('0',20,0);assert l.crossing is None
    l.observe('1',21,15);assert l.crossing is None


def test_real_wall_segment_occlusion_vs_outside_frustum():
    tri=np.array([[[1,-1,-1],[1,1,-1],[1,0,1]]])
    h=ray_first_hit([0,0,0],[2,0,0],tri);assert h['alpha']==.5
    assert ray_first_hit([0,0,0],[.5,0,0],tri) is None
    camera=dict(T_world_camera=np.eye(4).tolist(),actual_intrinsics=dict(K=[[100,0,50],[0,100,50],[0,0,1]],resolution_width_height=[100,100]))
    assert project_camera([0,0,-2],camera)['in_frustum']
    assert not project_camera([5,0,-2],camera)['in_frustum']


def test_left_corner_not_reverse_or_straight():
    assert left_corner([0,-1],[1,0]);assert left_corner([-1,0],[0,-1])
    assert not left_corner([0,-1],[-1,0]);assert not left_corner([1,0],[1,0])

@pytest.mark.parametrize('N',[5,10,21])
def test_old_turning_and_arbitrary_N(N):
    t=np.linspace(0,np.pi/2,N);p=np.c_[np.sin(t),1-np.cos(t),t]
    assert old_turning(p)['qualified']
    assert not old_turning(np.c_[np.linspace(0,1,N),np.zeros(N),np.zeros(N)])['qualified']
    assert not old_turning(np.c_[p[:,:2]*.1,t])['qualified']


def test_motion_requires_physical_rotation_not_memory():
    assert motion_gate([.8,.1],.02,.05)
    assert not motion_gate([.8,0],.2,.5)
    assert not motion_gate([.2,.5],.2,.5)
    assert not motion_gate([.8,.5],.019,.5)


def test_whole_raw_safety_no_prefix_trim_or_connector():
    env=HospitalEnvironment(box(.45,-.1,.55,.1),box(-10,-10,10,10))
    bad=np.array([[0,0,0],[1,0,0],[1,1,0]])
    assert not whole_raw_polyline_check(bad,env)['clearance_valid']
    assert whole_raw_polyline_check(bad[1:],env)['clearance_valid']
    # Checker evaluates full supplied raw rows; no observation connector invented.
    assert whole_raw_polyline_check(np.array([[1,1,0],[2,1,0]]),env)['clearance_valid']


def test_anchor_is_observation_and_future_is_eligibility_only():
    raw=np.array([[0,0,0],[.3,0,.2],[.6,0,.4],[.9,0,.6],[1.2,0,.8]])
    w=observation_anchored_world(raw,[10,20,np.pi/2]);np.testing.assert_allclose(w[:,:2],np.c_[np.full(5,10),20+raw[:,0]],atol=1e-12)
    before=w.copy();q=future_at_pose(w,[10,20,np.pi/2]);assert q['remaining_rows']==5 and q['remaining_arc_m']>=.6
    np.testing.assert_array_equal(w,before)


def test_timing_and_frozen_order():
    assert timing_gate([1.0],.2,1)['qualified'];assert not timing_gate([.7],.2,1)['qualified']
    assert not timing_gate([1.0],.26,1)['qualified']
    rows=[dict(candidate_id='C02',qualified=False)];assert select_representative(rows,['C02']) is None
    with pytest.raises(ValueError):select_representative(rows,['C03'])


def test_frozen_protocol_and_static_runtime_scope():
    c=yaml.safe_load((ROOT/'configs/blind_corner_source_acquisition_01.yaml').read_text())
    assert c['instruction']==INSTRUCTION and c['candidate_order']==['C01','C02']
    assert len(c['candidates'])<=3 and c['maximum_terminal_predictions_per_candidate']==2
    tree=ast.parse((ROOT/'scripts/isaac/blind_corner_online.py').read_text())
    hook=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='StaticOcclusion')
    names=[n.func.id for n in ast.walk(hook) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    assert 'set_present' not in names and 'create_prop' not in names
    main=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='main')
    calls=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    activate=[n for n in calls if n.func.id=='set_present'];execute=[n for n in calls if n.func.id=='collect_episode']
    assert len(activate)==1 and activate[0].lineno<execute[0].lineno and activate[0].args[1].value is True
    assert not any(n.func.id in ('minimize','least_squares','solve_gp','rollout') for n in calls)
