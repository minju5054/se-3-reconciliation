"""Synthetic implementation tests only, never handoff performance evidence."""
import os
os.environ['JAX_ENABLE_X64']='true';os.environ['JAX_PLATFORMS']='cpu'
import copy
import hashlib
from pathlib import Path
import numpy as np
import pytest
import yaml
from shapely.geometry import box
from reconciliation.gp_se2_attach01 import (T_SET,FixedAttachView,AttachDerivatives,support_index,
    select_full_valid,method_schedule,sustained_join,forward_projection,verify_derivatives)
from reconciliation.gp_se2_attach01_source import source_order,geometry_eligibility
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
from reconciliation.se2 import local_trajectory_to_world,relative_pose,se2_log

ROOT=Path(__file__).resolve().parents[1]


def problem():
    common=np.column_stack([np.arange(1,31)*.04,np.zeros((30,2))])
    return GPProblem([0,0,0],[.4,0,0],common,common[-1],{},
        lambda xy:np.full(len(xy),5.),lambda xy:np.full(len(xy),5.))


def vector(base):
    seed=base.initializations()[0];return base.vector(seed['poses'],seed['twists'])


@pytest.mark.parametrize('duration',T_SET)
def test_fixed_common_mapping_and_original_constraints(duration):
    b=problem();before=b.common_reference.tobytes();z=vector(b);v=FixedAttachView(b,duration)
    k=round(duration*10)
    assert v.k==k and np.array_equal(v.reference,b.common_reference[k-1:])
    assert v.reference[0,0]==pytest.approx(.4*duration)
    old=b.evaluate(z);new=v.evaluate(z)
    assert np.array_equal(old['equality'],new['equality'])
    assert np.array_equal(old['inequality'],new['inequality'][:903])
    assert len(z)==150 and len(new['equality'])==30 and len(new['inequality'])==915
    assert b.common_reference.tobytes()==before
    check=v.independent_tube_check(z)
    assert check['reference_row_indices']==list(range(k-1,k+3))
    assert np.allclose(check['times_s'],duration+np.arange(4)*.1)
    assert check['valid']


def test_mask_begins_exactly_at_T_not_before():
    b=problem();v=FixedAttachView(b,.8);z=vector(b)
    z0=z.copy();z0[(v.k-2)*5+1]=.04
    assert v.evaluate(z0)['fresh_cost']==0.
    z1=z.copy();z1[(v.k-1)*5+1]=.04
    assert v.evaluate(z1)['fresh_cost']>0.
    new=v.evaluate(z1);old=b.evaluate(z1)
    residual=se2_log(relative_pose(b.common_reference[v.k-1:],new['poses'][v.k:]))/b.config['fresh_std']
    assert new['objective']==np.sum(old['gp_factor_costs'])+b.config['lambda_fresh']*np.mean(np.sum(residual**2,axis=1))


def test_nominal_tube_not_relaxed_by_solver_tolerance():
    b=problem();v=FixedAttachView(b,.4);z=vector(b)
    z[(v.k-1)*5+1]=.100001
    assert not v.independent_tube_check(z)['valid']
    assert v.evaluate(z)['attachment_margins'][0]<0


def test_tube_yaw_wrap():
    b=problem();b.common_reference[:,2]=np.pi-.01;b._anchors[1:,2]=b.common_reference[:,2]
    v=FixedAttachView(b,.4);p=b._anchors.copy();p[4:8,2]=-np.pi+.01
    twists=np.zeros_like(p);twists[0]=b.initial_twist;z=b.vector(p,twists)
    assert np.allclose(v.independent_tube_check(z)['yaw_error_rad'],.02)


def test_duration_is_external_and_no_search_or_retiming():
    assert len(method_schedule())==9
    assert [t for _,t in method_schedule()[3:]]==list(T_SET)
    b=problem()
    with pytest.raises(ValueError):FixedAttachView(b,.5)
    text=(ROOT/'src/reconciliation/gp_se2_attach01.py').read_text()
    assert 'post_join_reference(' not in text and 'join_candidates(' not in text
    assert all(len(vector(b))==150 for t in T_SET)


@pytest.mark.parametrize('n',[2,3,10,37])
def test_original_observation_frame_and_variable_N(n):
    raw=np.column_stack([np.linspace(.1,1.4,n),np.zeros((n,2))]);saved=raw.tobytes()
    obs=[4,5,.6];boundary=[4.2,5.3,.9];world=local_trajectory_to_world(obs,raw);worldsaved=world.tobytes()
    prep=prepare_reference(world,boundary,[10,10,1]);fixed=prep['common_world'].tobytes()
    b=GPProblem(boundary,[.2,0,0],prep['common_world'],world[-1],{},lambda x:np.ones(len(x)),lambda x:np.ones(len(x)))
    for t in T_SET:
        view=FixedAttachView(b,t)
        assert hashlib.sha256(fixed).hexdigest()==view.common_value_sha256
    assert raw.tobytes()==saved and world.tobytes()==worldsaved
    assert not np.allclose(world,local_trajectory_to_world(boundary,raw))


def test_no_full_valid_candidate_means_NA():
    row=dict(full_valid=False,objective=-999,initialization='I0',source_label='latest')
    assert select_full_valid([row]) is None
    rows=[dict(row,full_valid=True,objective=1,initialization=i)for i in ['I1','I0']]
    assert select_full_valid(rows)['initialization']=='I0'


def test_sustained_metric_rejects_crossing_and_backward_projection():
    fresh=np.array([[0,0,0],[2,0,0.]])
    t=np.arange(31)*.1;poses=np.column_stack([t/2,np.full(31,.2),np.zeros(31)])
    poses[4,1]=0;metric=sustained_join(t,poses,fresh)
    assert not metric['join_success'] and metric['sustained_join_time_s'] is None
    poses[10:14,1]=0
    assert sustained_join(t,poses,fresh)['sustained_join_time_s']==1.
    progress=forward_projection(fresh,[[1,0,0],[.2,0,0],[1.5,0,0]])
    assert np.all(np.diff([r['progress']for r in progress])>=0)


def test_source_only_deterministic_order_and_gate_exclusion():
    assert source_order('a')[0]==hashlib.sha256(b'ATTACH01-v1:a').hexdigest()
    env=HospitalEnvironment(box(-2,.3,3,.5),box(-5,-5,5,5))
    f=np.column_stack([np.linspace(.1,1,10),np.zeros((10,2))]);p=prepare_reference(f,[0,0,0],[10,10,1])
    policy=yaml.safe_load((ROOT/'configs/gp_se2_attach_01.yaml').read_text())['source_selection']
    original=dict(eligible=True,groups=dict(reliable_direction=True),route_status='NOT_REQUIRED_CLEAR_SHORTCUT',B_environment=env.query([0,0]))
    metrics=dict(e_perp_m=.11,abs_e_dir_window_deg=0.,abs_e_yaw_deg=0.)
    r=geometry_eligibility(f,p,original,metrics,env,policy)
    assert all(r['predicates'].values())
    changed=copy.deepcopy(original);changed['route_status']='REQUIRED_ORDERED_GATES'
    assert not geometry_eligibility(f,p,changed,metrics,env,policy)['predicates']['no_required_gate_and_known_route']
    # Arbitrary downstream outcomes must not influence eligibility.
    original.update(new_GP_success=False,new_MPC_success=True)
    assert geometry_eligibility(f,p,original,metrics,env,policy)==r


def test_derivative_gate_and_literal_original_jacobians():
    b=problem();view=FixedAttachView(b,.8);z=vector(b)+np.random.default_rng(4).normal(size=150)*1e-4
    class Constant:
        def workspace(self,xy):return np.full(len(xy),5.),np.zeros_like(xy),{'points':[]}
        obstacle=workspace
    base=DerivativeProvider(b,Constant());dp=AttachDerivatives(view,base);dp.warmup(z)
    assert verify_derivatives(view,dp,z)['passed']
    assert np.array_equal(dp.equality_jacobian(z),base.equality_jacobian(z))
    assert np.array_equal(dp.inequality_jacobian(z)[:903],base.inequality_jacobian(z))
    assert dp.inequality_jacobian(z).shape==(915,150)


def test_source_projection_rejects_self_intersection_and_short_future():
    env=HospitalEnvironment(box(-2,2,3,3),box(-5,-5,5,5))
    policy=yaml.safe_load((ROOT/'configs/gp_se2_attach_01.yaml').read_text())['source_selection']
    original=dict(eligible=True,groups=dict(reliable_direction=True),route_status='NOT_REQUIRED_CLEAR_SHORTCUT',B_environment=env.query([0,0]))
    metrics=dict(e_perp_m=.11,abs_e_dir_window_deg=0.,abs_e_yaw_deg=0.)
    f=np.array([[0,0,0],[1,1,0],[0,1,0],[1,0,0.]])
    r=geometry_eligibility(f,prepare_reference(f,[0,0,0],[10,10,1]),original,metrics,env,policy)
    assert not r['predicates']['unambiguous_projection']
    f=np.array([[0,0,0],[1,0,0.]])
    r=geometry_eligibility(f,prepare_reference(f,f[-1],[10,10,1]),original,metrics,env,policy)
    assert not r['predicates']['enough_future']


def test_original_environment_callbacks_remain_in_M4():
    b=problem();view=FixedAttachView(b,1.2);z=vector(b)
    # Change only a synthetic fixture callback, not a primary environment.
    b.obstacle_clearance=lambda xy:np.full(len(xy),-.2)
    view.clear_cache();old=b.evaluate(z);new=view.evaluate(z)
    assert np.array_equal(new['inequality'][:903],old['inequality'])
    assert np.all(new['inequality'][813:903]<0.)


def test_existing_output_refused_before_source_access(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location('attach_runner_test',ROOT/'scripts/run_gp_se2_attach01.py')
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    with pytest.raises(FileExistsError):runner.prepare(tmp_path)
