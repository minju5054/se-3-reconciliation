"""Fixed seed family is tested independently; synthetic tests are not evidence."""
import numpy as np
import pytest
from scipy.integrate import solve_ivp

from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.gp_se2 import sample_gp
from reconciliation.se2 import compose_poses,se2_exp,wrap_angle


def problem(eta=(.8,0,-.00011474391164847372)):
    b=np.array([19.2,19.68,-1.57]);eta=np.asarray(eta)
    ts=np.linspace(0,3,31);common=compose_poses(b,se2_exp(ts[1:,None]*eta*.49))
    free=lambda xy:np.full(np.asarray(xy).shape[:-1],10.)
    return GPProblem(b,eta,common,common[-1],{},free,free,[],True)


@pytest.mark.parametrize('eta',[(.8,0,-.00011474391164847372),(.3,0,.4),(.3,0,0.)])
def test_seed_is_reproduced_by_gp_and_independent_ode(eta):
    p=problem(eta);seed=same_curvature_deceleration_seed(p);eta=np.asarray(eta)
    ode=solve_ivp(lambda t,x: [(1-t/3)*eta[0]*np.cos(x[2]),(1-t/3)*eta[0]*np.sin(x[2]),
                             (1-t/3)*eta[2]],(0,3),p.boundary_pose,method='DOP853',rtol=1e-12,atol=1e-12,dense_output=True)
    ts=np.linspace(0,3,3001)
    poses,vel,acc=sample_gp(p.times,seed['poses'],seed['twists'],ts)
    np.testing.assert_allclose(poses[:,:2],ode.sol(ts).T[:,:2],rtol=0,atol=1e-10)
    np.testing.assert_allclose(wrap_angle(poses[:,2]-ode.sol(ts)[2]),0,atol=1e-11)
    np.testing.assert_allclose(vel,(1-ts[:,None]/3)*eta,rtol=0,atol=1e-10)
    np.testing.assert_allclose(acc,np.tile(-eta/3,(len(ts),1)),rtol=0,atol=1e-8)
    np.testing.assert_array_equal(seed['poses'][0],p.boundary_pose)
    np.testing.assert_array_equal(seed['twists'][0],p.initial_twist)
    np.testing.assert_array_equal(seed['twists'][-1],np.zeros(3))


def test_seed_does_not_fit_goal_or_modify_original_initializers_or_inputs():
    p=problem();before=p.initializations();config=p.config.copy();common=p.common_reference.copy()
    a=same_curvature_deceleration_seed(p)
    p.goal_pose+=np.array([20.,10.,.4]);b=same_curvature_deceleration_seed(p)
    np.testing.assert_array_equal(a['poses'],b['poses'])
    np.testing.assert_array_equal(a['twists'],b['twists'])
    np.testing.assert_array_equal(p.common_reference,common)
    assert p.config==config
    for x,y in zip(before,p.initializations()):
        np.testing.assert_array_equal(x['poses'],y['poses'])
        np.testing.assert_array_equal(x['twists'],y['twists'])
    assert not p.dense_report(p.vector(b['poses'],b['twists']))['feasible']
    assert b['metadata']['feasible_status'].startswith('NOT_EVALUATED')


def test_same_horizon_and_constraints_seed_is_not_raw_return():
    p=problem();s=same_curvature_deceleration_seed(p)
    assert not np.array_equal(s['poses'][1:],p.common_reference)
    assert p.dense_report(p.vector(s['poses'],s['twists']))['feasible']
    assert not s['metadata']['parameters_fitted']
    assert not s['metadata']['raw_rollout_used']
