"""Synthetic mathematics checks, not dataset performance evidence."""
import numpy as np
import pytest

from reconciliation.gp_se2 import (adjoint_algebra, gp_factor_jacobian, gp_residual,
    interpolate_interval, prior_dependency_pattern, process_covariance, right_jacobian,
    right_jacobian_directional, sample_gp, whiten_residual)
from reconciliation.se2 import compose_poses, relative_pose, retract_pose, se2_exp, se2_log


@pytest.mark.parametrize('xi', [[.3,-.2,0], [.3,-.2,1e-8], [.7,.4,.8], [-1,.5,-2.9], [.1,.2,3.13]])
def test_exp_log_right_retraction_and_jacobian_finite_difference(xi):
    xi = np.array(xi); pose = np.array([2.,-3.,.6]); d = np.array([.2,-.3,.4]); eps=1e-6
    np.testing.assert_allclose(se2_log(se2_exp(xi)), xi, atol=1e-12)
    np.testing.assert_allclose(relative_pose(pose, retract_pose(pose,xi)), se2_exp(xi), atol=1e-12)
    numerical = (se2_log(relative_pose(se2_exp(xi),se2_exp(xi+eps*d)))
                 -se2_log(relative_pose(se2_exp(xi),se2_exp(xi-eps*d))))/(2*eps)
    np.testing.assert_allclose(numerical, right_jacobian(xi)@d, atol=2e-9)
    numerical_dj = (right_jacobian(xi+eps*d)-right_jacobian(xi-eps*d))/(2*eps)
    np.testing.assert_allclose(numerical_dj,right_jacobian_directional(xi,d),atol=1e-8)


def test_adjoint_bracket_and_zero_jacobian():
    np.testing.assert_array_equal(adjoint_algebra([1,2,3])@np.array([1,2,3]), np.zeros(3))
    np.testing.assert_array_equal(right_jacobian([0,0,0]), np.eye(3))


@pytest.mark.parametrize('h',[.01,.1,1.,3.])
def test_q_symmetry_positive_definite_and_whitening(h):
    q = process_covariance(h,[1.,2.,3.]); r=np.arange(6)*.03
    np.testing.assert_allclose(q,q.T)
    assert np.min(np.linalg.eigvalsh(q))>0
    white=whiten_residual(r,h,[1.,2.,3.])
    assert white@white == pytest.approx(r@np.linalg.solve(q,r))


@pytest.mark.parametrize('nu',[[.4,0,0],[.4,0,.8],[0,0,-1.2],[.2,.3,.6]])
def test_constant_twist_zero_residual_and_interpolant(nu):
    p=np.array([1.,2.,3.10]); nu=np.array(nu); h=.1
    end=compose_poses(p,se2_exp(h*nu))
    np.testing.assert_allclose(gp_residual(p,nu,end,nu,h),0,atol=2e-14)
    for u in [0,.23,.5,.89,1]:
        po,ve,ac=interpolate_interval(p,nu,end,nu,h,u)
        np.testing.assert_allclose(relative_pose(compose_poses(p,se2_exp(h*u*nu)),po),0,atol=1e-13)
        np.testing.assert_allclose(ve,nu,atol=1e-12)
        np.testing.assert_allclose(ac,0,atol=1e-10)


def test_endpoint_reproduction_pose_derivative_identity_and_acceleration():
    p0=np.array([2.,-1.,3.10]); p1=np.array([2.02,-1.003,-3.13])
    v0=np.array([.2,0,.5]); v1=np.array([.3,.04,.1]); h=.1
    for u,p,v in [(0,p0,v0),(1,p1,v1)]:
        actual,velocity,_=interpolate_interval(p0,v0,p1,v1,h,u)
        np.testing.assert_allclose(relative_pose(p,actual),0,atol=1e-12)
        np.testing.assert_allclose(velocity,v,atol=1e-12)
    for u in [.17,.42,.79]:
        eps=1e-5
        p,v,a=interpolate_interval(p0,v0,p1,v1,h,u)
        pm,vm,_=interpolate_interval(p0,v0,p1,v1,h,u-eps/h)
        pp,vp,_=interpolate_interval(p0,v0,p1,v1,h,u+eps/h)
        finite=(se2_log(relative_pose(p,pp))-se2_log(relative_pose(p,pm)))/(2*eps)
        np.testing.assert_allclose(finite,v,atol=1e-7)
        np.testing.assert_allclose((vp-vm)/(2*eps),a,atol=1e-7)


def test_joint_connection_and_yaw_chart_boundary():
    times=np.array([0,.1,.2]); poses=np.array([[0,0,3.12],[-.02,0,-3.13],[-.04,-.003,-3.08]])
    twists=np.array([[.2,0,.3],[.3,0,.5],[.2,0,.4]])
    for i in range(2):
        p,v,_=interpolate_interval(poses[i],twists[i],poses[i+1],twists[i+1],.1,1.)
        np.testing.assert_allclose(relative_pose(p,poses[i+1]),0,atol=1e-12)
        np.testing.assert_allclose(v,twists[i+1],atol=1e-12)
    p,v,_=sample_gp(times,poses,twists,times)
    np.testing.assert_allclose(p,poses,atol=1e-12); np.testing.assert_allclose(v,twists,atol=1e-12)
    with pytest.raises(ValueError): sample_gp(times,poses,twists,[-.01])


def test_gp_factor_jacobian_predicts_independent_random_right_local_perturbations():
    p0=np.array([1.,2.,.4]); p1=np.array([1.03,2.02,.46]); v0=np.array([.3,0,.5]); v1=np.array([.2,.01,.4])
    j=gp_factor_jacobian(p0,v0,p1,v1,.1)
    d=np.random.default_rng(23).normal(size=12); d*=1e-5
    shifted=gp_residual(retract_pose(p0,d[:3]),v0+d[3:6],retract_pose(p1,d[6:9]),v1+d[9:],.1)
    base=gp_residual(p0,v0,p1,v1,.1)
    np.testing.assert_allclose(shifted-base,j@d,atol=3e-9)
    np.testing.assert_allclose(j,gp_factor_jacobian(p0,v0,p1,v1,.1,3e-6),atol=2e-8)


def test_sparse_dependency_pattern_matches_actual_perturbation():
    n=5; pattern=prior_dependency_pattern(n); times=np.arange(n)*.1
    nu=np.array([.3,0,.2]); poses=se2_exp(times[:,None]*nu); twists=np.tile(nu,(n,1))
    base=gp_residual(poses[:-1],twists[:-1],poses[1:],twists[1:],.1).reshape(-1)
    for index in range(n):
        for dimension in range(6):
            p,v=poses.copy(),twists.copy(); delta=np.zeros(3); delta[dimension%3]=1e-5
            if dimension<3: p[index]=retract_pose(p[index],delta)
            else: v[index]+=delta
            change=gp_residual(p[:-1],v[:-1],p[1:],v[1:],.1).reshape(-1)-base
            assert np.all(np.abs(change[~pattern[:,6*index+dimension]])<1e-14)
    assert pattern.sum()==(n-1)*72


def test_hermite_mean_matches_integrated_wiener_conditional_covariance():
    from reconciliation.gp_se2 import right_jacobian_inverse_apply
    h=.3;t=.117
    p0=np.array([1.,-2.,.7]);p1=np.array([1.1,-1.95,.8])
    v0=np.array([.2,0,.1]);v1=np.array([.4,.02,.2])
    xi=se2_log(relative_pose(p0,p1));g0=np.r_[np.zeros(3),v0]
    g1=np.r_[xi,right_jacobian_inverse_apply(xi,v1)]
    def phi(dt):return np.block([[np.eye(3),dt*np.eye(3)],[np.zeros((3,3)),np.eye(3)]])
    cov=process_covariance(t,[1.,2.,3.])@phi(h-t).T
    psi=np.linalg.solve(process_covariance(h,[1.,2.,3.]),cov.T).T
    gamma=phi(t)@g0+psi@(g1-phi(h)@g0)
    pose,velocity,_=interpolate_interval(p0,v0,p1,v1,h,t/h)
    np.testing.assert_allclose(se2_log(relative_pose(p0,pose)),gamma[:3],atol=1e-12)
    np.testing.assert_allclose(velocity,right_jacobian(gamma[:3])@gamma[3:],atol=1e-12)
