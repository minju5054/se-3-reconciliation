"""Synthetic constrained numerical fixtures; not experimental evidence."""
import numpy as np
import pytest

from reconciliation.gp_se2_formulation import GPProblem, resolve_config, solve_gp, solve_rigid, _gate_rows
from reconciliation.se2 import compose_poses, se2_exp, relative_pose


def free(points): return np.ones(np.asarray(points).shape[:-1])*2.
def closed(points): return np.ones(np.asarray(points).shape[:-1])*(-.2)


def fixture():
    b=np.array([.3,-.4,.2]);v=np.array([.2,0,0]);f=compose_poses(b,se2_exp(np.arange(1,31)[:,None]*.1*v))
    return b,v,f,f[-1].copy()


def test_fixed_boundary_and_twist_in_every_vector_no_source_mutation():
    b,v,f,g=fixture();copies=[x.copy() for x in [b,v,f,g]]
    problem=GPProblem(b,v,f,g,{},free,free,[],True)
    start=problem.initializations()[0];x=problem.vector(start['poses'],start['twists'])
    x=x+np.random.default_rng(0).normal(size=x.shape)*.01
    poses,twists=problem.unpack(x)
    np.testing.assert_array_equal(poses[0],b);np.testing.assert_array_equal(twists[0],v)
    np.testing.assert_array_equal(twists[:,1],0.)
    for value,original in zip([b,v,f,g],copies):np.testing.assert_array_equal(value,original)


def test_m2_m3_identical_except_obstacle_rows_including_initializations():
    b,v,f,g=fixture();m2=GPProblem(b,v,f,g,{},free,free,[],False);m3=GPProblem(b,v,f,g,{},free,free,[],True)
    for i2,i3 in zip(m2.initializations(),m3.initializations()):
        np.testing.assert_array_equal(i2['poses'],i3['poses']);np.testing.assert_array_equal(i2['twists'],i3['twists'])
    initial=m2.initializations()[0];x=m2.vector(initial['poses'],initial['twists'])
    a,c=m2.evaluate(x),m3.evaluate(x)
    assert a['objective']==c['objective']
    np.testing.assert_array_equal(a['equality'],c['equality'])
    np.testing.assert_array_equal(a['inequality'],c['inequality'][:len(a['inequality'])])
    np.testing.assert_allclose(c['inequality'][len(a['inequality']):],1.95)


def test_midpoint_lateral_is_hard_equality_and_dense_detects_slip():
    b,v,f,g=fixture();p=GPProblem(b,v,f,g,{},free,free,[],False)
    initial=p.initializations()[0];initial['poses'][5,1]+=.02
    x=p.vector(initial['poses'],initial['twists']);ev=p.evaluate(x)
    assert np.max(np.abs(ev['equality']))>.01
    assert not p.dense_report(x)['feasible']


def test_real_numerical_constant_twist_solve_and_exact_initial_conditions():
    b,v,f,g=fixture();result=solve_gp(b,v,f,g,{'initializations':1,'max_iterations':5,'wall_time_s':5.},free,workspace_margin=free)
    assert result['status']=='CANDIDATE_FOUND'
    assert result['attempts'][0]['objective_evaluations']>0
    np.testing.assert_array_equal(result['support_poses'][0],b)
    np.testing.assert_array_equal(result['support_twists'][0],v)
    np.testing.assert_allclose(result['candidate_world'],f,atol=1e-8)
    assert result['constraint_report']['body_derivative_identity_valid']
    assert len(result['candidate_world'])==30


def test_rotation_only_reference_preserved_by_gp_and_rigid():
    b=np.array([0.,0.,3.1]);v=np.array([0.,0.,.2]);f=compose_poses(b,se2_exp(np.arange(1,31)[:,None]*.1*v))
    result=solve_gp(b,v,f,f[-1],{'initializations':1,'max_iterations':3,'wall_time_s':5.},free)
    assert result['status']=='CANDIDATE_FOUND'
    np.testing.assert_allclose(relative_pose(result['candidate_world'],f),0,atol=1e-8)
    rigid=solve_rigid(b,v,f,f[-1],{'initializations':1,'max_iterations':3},free)
    assert rigid['status']=='CANDIDATE_FOUND'
    np.testing.assert_allclose(relative_pose(rigid['candidate_world'],f),0,atol=1e-8)


def test_goal_and_ordered_directed_gate_constraints():
    b,v,f,g=fixture();gate={'center_xy':f[14,:2], 'normal_xy':[np.cos(.2),np.sin(.2)],
        'half_width_m':.1,'time_s':1.5}
    p=GPProblem(b,v,f,g,{},free,free,[gate],True)
    i=p.initializations()[0];x=p.vector(i['poses'],i['twists'])
    assert p.dense_report(x)['feasible']
    reverse=dict(gate,normal_xy=[-np.cos(.2),-np.sin(.2)])
    r=GPProblem(b,v,f,g,{},free,free,[reverse],True)
    assert not r.dense_report(x)['feasible']
    stationary=np.tile(b,(31,1));zero=np.zeros((31,3))
    assert not p.dense_report(p.vector(stationary,zero))['feasible']
    with pytest.raises(ValueError):_gate_rows(lambda ts:np.zeros((len(ts),3)),[gate,gate],0,3.)


def test_solver_timeout_or_failure_never_claims_infeasibility_or_raw_fallback():
    b,v,f,g=fixture()
    result=solve_gp(b,v,f,g,{'initializations':1,'wall_time_s':1e-9,'max_iterations':1},closed)
    assert result['status']=='NO_FEASIBLE_CANDIDATE_FOUND'
    assert result['candidate_world'] is None
    assert not result['infeasibility_proven']
    assert result['attempts'][0]['termination']=='TIMEOUT'
    assert np.isfinite(result['attempts'][0]['latest_iterate']).all()


def test_unknown_environment_and_workspace_never_become_feasible():
    b,v,f,g=fixture();unknown=lambda p:np.full(len(p),np.nan)
    result=solve_rigid(b,v,f,g,{'initializations':1,'max_iterations':1},unknown)
    assert result['status']=='NO_FEASIBLE_CANDIDATE_FOUND'
    p=GPProblem(b,v,f,g,{},free,unknown,[],False)
    i=p.initializations()[0]
    assert not p.dense_report(p.vector(i['poses'],i['twists']))['feasible']


def test_rigid_single_transform_has_no_impossible_boundary_equality():
    b,v,f,g=fixture();f=f.copy();f[:,1]+=.01
    result=solve_rigid(b,v,f,g,{'initializations':1,'max_iterations':30},free)
    assert result['status']=='CANDIDATE_FOUND'
    np.testing.assert_allclose(result['candidate_world'],compose_poses(result['transform_pose'],f),atol=1e-12)
    assert result['constraint_report']['initial_connector']=='UNDEFINED_EVALUATED_ONLY_BY_MPC'
    assert result['constraint_report']['times'][0]==pytest.approx(.1)
    assert result['attempts'][0]['candidate_checks'][0]['constraint_report']['equality_count']==0


def test_no_unsupported_configuration_or_silent_lateral_boundary():
    with pytest.raises(ValueError):resolve_config({'unknown':1})
    with pytest.raises(ValueError):resolve_config({'dense_dt_s':.1})
    b,v,f,g=fixture();v[1]=.1
    with pytest.raises(ValueError):GPProblem(b,v,f,g,{})


def test_nonzero_curved_mismatch_is_actually_optimized_with_hard_constraints():
    b=np.array([1.,-2.,.4]); target=np.array([.3,0,.4]);initial=np.array([.25,0,.35])
    f=compose_poses(b,se2_exp(np.arange(1,6)[:,None]*.1*target))
    result=solve_gp(b,initial,f,f[-1],{'horizon_s':.5,'initializations':1,
        'wall_time_s':5.,'goal_position_tolerance':.01},free)
    assert result['status']=='CANDIDATE_FOUND'
    assert result['attempts'][0]['solver_success']
    assert result['attempts'][0]['iterations']>1
    assert result['factor_costs']['total']<result['attempts'][0]['candidate_checks'][0]['objective']*.5
    assert result['constraint_report']['feasible']
    assert result['constraint_report']['maximum_absolute_lateral_velocity_m_s']<1e-5
    assert result['attempts'][0]['best_feasible_iterate'] is not None
    assert not result['attempts'][0]['random_generator_used']
