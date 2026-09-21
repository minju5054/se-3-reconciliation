"""Synthetic implementation fixtures only; no primary experimental evidence."""
import os
os.environ['JAX_PLATFORMS']='cpu';os.environ['JAX_ENABLE_X64']='true'
import json
from pathlib import Path
import numpy as np
import pytest
from shapely.geometry import box
from reconciliation.gp_se2_join01 import *
from reconciliation.gp_se2_join01_environment import RevealEnvironment,RevealDerivatives
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.gp_se2_join01_formulation import JoinView,JoinDerivatives,verify_derivatives
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
from reconciliation.gp_se2_diag02_validation import ConstantEnvironmentDerivatives
from reconciliation.se2 import local_trajectory_to_world,se2_log,relative_pose


def path(n=10):return np.column_stack([np.linspace(0,1.5,n),np.zeros((n,2))])


@pytest.mark.parametrize('n',[1,2,3,10,37])
def test_forward_grid_arbitrary_N(n):
    f=path(n);cs=join_candidates(f,f[-1]);assert len(cs)==6
    assert all(c['join_index']==n-1 for c in cs)
    assert len({(c['join_time_s'],c['join_index']) for c in cs})==len(cs)
    fcopy=f.copy();cs=join_candidates(f,f[n//2]);assert np.array_equal(f,fcopy)
    assert all(c['join_index']>=n//2 for c in cs)


def test_post_join_preserves_original_endpoint_and_join():
    f=path(7);t=np.arange(31)*.1;c=join_candidates(f,[0,0,0])[2];r=post_join_reference(f,c,t)
    assert np.array_equal(r[0],f[c['join_index']])
    assert np.array_equal(r[-1],f[-1])
    assert len(r)==31-c['support_index']
    assert not np.shares_memory(r,f)


def test_single_row_suffix_constant():
    c=join_candidates([[5,6,1]],[0,0,0])[0]
    assert np.allclose(post_join_reference([[5,6,1]],c,np.arange(31)*.1),[5,6,1])


def test_wrap_tube():
    a=np.array([[0,0,-np.pi+.01]]);r=np.array([[0,0,np.pi-.01]])
    m=tube_margins(a,r);assert np.all(m>0);assert np.isclose(m[1],YAW_RAD-.02)
    assert tube_margins([[.11,0,0]],[[0,0,0]])[0]<0


def test_observation_transform_not_boundary_anchor():
    raw=path();copy=raw.copy();obs=[5,6,np.pi/2];b=[6,8,0]
    world=local_trajectory_to_world(obs,raw)
    assert np.allclose(world[0],[5,6,np.pi/2])
    assert not np.allclose(world,local_trajectory_to_world(b,raw))
    join_candidates(world,b);assert np.array_equal(raw,copy)


def test_one_frame_crossing_fails_and_NA_preserved():
    t=np.arange(31)*.1;x=path(31);x[:,1]=.2;x[10,1]=0
    r=sustained_join(t,x,path());assert not r['join_success']
    assert r['sustained_join_time_s'] is None and r['pre_join_position_auc_m_s'] is None
    assert r['full_window_position_auc_m_s']>0


def test_sustained_duration_and_final_window():
    t=np.arange(31)*.1;x=path(31);x[:,1]=.2;x[10:14,1]=0
    r=sustained_join(t,x,path());assert r['sustained_join_time_s']==1.
    x[:,1]=.2;x[-3:,1]=0;assert not sustained_join(t,x,path())['join_success']


def test_yaw_can_prevent_join_even_on_line():
    t=np.arange(31)*.1;x=path(31);x[:,2]=.4
    assert not sustained_join(t,x,path())['join_success']


def test_progress_no_backtracking_and_shortest_yaw():
    f=path(3);x=np.array([[1.2,0,0],[.2,0,0],[1.4,0,0]])
    r=forward_projection(f,x);assert np.all(np.diff([a['progress'] for a in r])>=0)
    assert np.isclose(r[1]['distance_m'],1.)
    f=np.array([[0,0,np.pi-.1],[1,0,-np.pi+.1]])
    r=forward_projection(f,[[.5,0,np.pi]])[0];assert r['yaw_error_rad']<1e-12


def test_degenerate_rows_preserved():
    f=np.array([[0,0,0],[0,0,1],[1,0,1]])
    assert len(forward_projection(f,[[0,0,0]]))==1
    assert np.array_equal(f[1],[0,0,1])


def test_lexicographic_selection_no_fallback():
    assert select_candidate([]) is None
    rows=[dict(full_valid=True,join_time_s=t,objective=o,join_index=j,initialization=i,source_label=s)
          for t,o,j,i,s in [(.6,1,0,'I0','latest'),(.4,2,2,'I1','latest'),(.4,2,1,'I1','latest'),(.4,2,1,'I0','latest')]]
    assert select_candidate(rows)==rows[-1]
    rows[-1]['full_valid']=False;assert select_candidate(rows)==rows[-2]


def environment():
    base=HospitalEnvironment(box(8,8,9,9),box(-5,-5,10,10),grid=dict(origin=np.array([-5.,-5.]),resolution=np.array(1.),distances=np.full((16,16),7.)),metadata={'grid_distance_absolute_error_bound_m':.70710678})
    obstacle=dict(pose_world=[1.,0.,.3],dimensions_m=[.3,.6,.9])
    return RevealEnvironment(base,obstacle)


def test_box_in_optimizer_and_independent_geometry():
    e=environment();assert e.clearance([1,0])==-.2
    assert e.optimizer_clearance([1,0])==-.2
    assert not e.check_polyline([[0,0],[2,0]])['clearance_valid']
    assert e.check_polyline([[0,1],[2,1]])['clearance_valid']


def test_box_rotation_analytic_gradient():
    e=environment();ed=RevealDerivatives(e);p=np.array([[.5,.6],[1.4,.6]])
    v,g,m=ed.obstacle(p)
    assert np.allclose(v,e.optimizer_clearance(p))
    for j in range(2):
        d=np.eye(2)[j]*1e-6
        fd=(e.optimizer_clearance(p+d)-e.optimizer_clearance(p-d))/(2e-6)
        assert np.allclose(fd,g[:,j],atol=1e-8,rtol=1e-6)


def test_box_placement_no_extrapolation():
    r=obstacle_on_old([0,0,0],path(),1.3,[.3,.6,.9]);assert np.allclose(r['pose_world'],[1.3,0,0])
    with pytest.raises(ValueError,match='OUTSIDE'):obstacle_on_old([0,0,0],path(),2.,[.3,.6,.9])


def problem():
    common=np.column_stack([np.arange(1,31)*.04,np.zeros((30,2))])
    return GPProblem([0,0,0],[.4,0,0],common,common[-1],{},lambda xy:np.full(len(xy),5.),lambda xy:np.full(len(xy),5.))


def test_original_constraints_literal_and_masked_cost():
    b=problem();f=path();spec=join_candidates(f,b.boundary_pose)[0];view=JoinView(b,f,spec)
    z=b.vector(b.initializations()[0]['poses'],b.initializations()[0]['twists'])
    old=b.evaluate(z);new=view.evaluate(z)
    assert np.array_equal(new['equality'],old['equality'])
    assert np.array_equal(new['inequality'][:-12],old['inequality'])
    assert len(new['equality'])==30 and len(new['inequality'])==915
    assert np.array_equal(new['gp_factor_costs'],old['gp_factor_costs'])
    p=new['poses'][view.k:];r=se2_log(relative_pose(view.reference,p))/b.config['fresh_std']
    assert new['objective']==float(np.sum(old['gp_factor_costs'])+np.mean(np.sum(r*r,axis=1)))


def test_join_derivative_gate_nonzero_chart():
    b=problem();f=path();spec=join_candidates(f,b.boundary_pose)[1];v=JoinView(b,f,spec)
    z=b.vector(b.initializations()[0]['poses'],b.initializations()[0]['twists'])
    z=z+np.random.default_rng(4).normal(size=150)*1e-4
    class Constant:
        def workspace(self,xy):return np.full(len(xy),5.),np.zeros_like(xy),{'points':[]}
        obstacle=workspace
    d=DerivativeProvider(b,Constant());jd=JoinDerivatives(v,d);jd.warmup(z)
    r=verify_derivatives(v,jd,z);assert r['passed'],r
    assert np.array_equal(jd.equality_jacobian(z),d.equality_jacobian(z))
    assert np.array_equal(jd.inequality_jacobian(z)[:-12],d.inequality_jacobian(z))


def test_new_config_no_witness_or_reserve_and_fixed_scope():
    import yaml
    root=Path(__file__).resolve().parents[1];c=yaml.safe_load((root/'configs/gp_se2_join_01.yaml').read_text())
    assert len(c['placements'])==3 and c['join']['times_s']==list(JOIN_TIMES)
    assert c['gp']['max_iterations']==200 and c['gp']['prepared_budget_s_per_start']==30
    assert c['join']['position_m']==POSITION_M and c['join']['sustained_s']==DWELL_S


def test_exclusive_output(tmp_path):
    from reconciliation.gp_se2_diag02_validation import write_new
    p=tmp_path/'a.json';write_new(p,{'a':1})
    with pytest.raises(FileExistsError):write_new(p,{'a':2})
    assert json.loads(p.read_text())=={'a':1}


def test_qualification_uses_only_source_geometry():
    from reconciliation.gp_se2_join01_qualification import qualify
    import yaml
    root=Path(__file__).resolve().parents[1];c=yaml.safe_load((root/'configs/gp_se2_join_01.yaml').read_text())
    e=environment();f=np.array([[0,0,0],[.3,.6,0],[.8,.7,0],[1.4,.7,0],[1.8,.3,0],[2.,0,0]])
    ctx=dict(old_world=path(15)*[1.5,1,1],fresh_world=f,B_world=[.1,0,0])
    timing=dict(old_observation_before_reveal=True,fresh_observation_after_reveal=True,inflight_execution_valid=True)
    q=qualify(ctx,e.obstacle,e.base,c,visibility={'obstacle_pixels':100},timing=timing)
    assert q['qualified'],q['failure_reasons']
    assert not q['optimizer_outcomes_used']
    ctx['fresh_world']=ctx['old_world'];q=qualify(ctx,e.obstacle,e.base,c,visibility={'obstacle_pixels':100},timing=timing)
    assert not q['qualified'] and 'fresh_suffix_valid' in q['failure_reasons']
