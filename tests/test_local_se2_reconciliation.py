"""Synthetic implementation tests only; no scientific source solve or MPC/VLA."""
import json
from pathlib import Path
import numpy as np
import pytest
from reconciliation.local_se2_reconciliation import LocalSE2Problem, planning_diagnostics, rigid_fit_diagnostic
from reconciliation.graph_optimizer import SolverConfig, OptimizationError, solve_least_squares
from reconciliation.se2 import compose_poses, relative_pose, se2_log


def fixture(n=7):
    a = np.array([1., -2., 3.1]); b = compose_poses(a, [.24, .04, .12])
    t = np.linspace(0, 1, n)**1.5
    f = compose_poses(a, np.c_[t, .3*t*t, .25+.4*t])
    return LocalSE2Problem(a, b, f)


def test_zero_latency_zero_cost_optimum():
    p = fixture(); same = LocalSE2Problem(p.A, p.A, p.fresh)
    np.testing.assert_allclose(same.target, p.fresh, atol=2e-15)
    assert same.costs(p.fresh)['total'] < 1e-25
    result = solve_least_squares(p.fresh, same.residual_vector, SolverConfig())
    np.testing.assert_array_equal(result.optimized, p.fresh)


def test_initial_and_transported_identities_and_anchor_breaks_degeneracy():
    p = fixture()
    c = p.costs(p.fresh); t = p.costs(p.target)
    assert c['L'] > 0 and c['R'] < 1e-25 and c['A'] < 1e-25
    assert t['A'] > 0 and t['L'] < 1e-25 and t['R'] < 1e-25
    np.testing.assert_allclose(relative_pose(p.B, p.target), relative_pose(p.A, p.fresh), atol=3e-15)
    h = [.6, -.4, .9]
    assert p.costs(compose_poses(h, p.fresh))['R'] < 1e-25
    assert p.costs(compose_poses(h, p.fresh))['A'] > 0


def test_global_left_frame_invariance_and_cost_mean_normalization():
    p = fixture(); x = compose_poses([.03, -.06, -.1], p.target); h = [10., -7., -2.8]
    q = LocalSE2Problem(compose_poses(h, p.A), compose_poses(h, p.B), compose_poses(h, p.fresh))
    np.testing.assert_allclose(p.residual_vector(x), q.residual_vector(compose_poses(h, x)), atol=2e-13)
    assert p.residual_vector(x) @ p.residual_vector(x) == pytest.approx(p.costs(x)['total'])
    raw=p.raw_residuals(x); s=p.progress
    assert p.costs(x)['L'] == pytest.approx(np.sum((1-s)**2*np.sum((raw['L']/p.scales)**2,axis=1))/sum((1-s)**2))
    assert p.costs(x)['R'] == pytest.approx(np.mean(np.sum((raw['R']/p.scales)**2,axis=1)))


def test_yaw_wrap():
    f = [[0,0,np.pi-.001],[1,0,-np.pi+.001]]
    p = LocalSE2Problem([0,0,0],[.1,0,0],f)
    x=np.array(f); x[:,2]+=2*np.pi
    assert p.costs(x)['A'] < 1e-25 and p.costs(x)['R'] < 1e-25


@pytest.mark.parametrize('n',[2,3,10,17])
def test_generic_rows_no_timing_and_no_mutation(n):
    p=fixture(n); before=p.fresh.copy(); x=p.fresh.copy()
    p.residual_vector(x); p.costs(x); planning_diagnostics(p,x)
    np.testing.assert_array_equal(x,before); np.testing.assert_array_equal(p.fresh,before)
    assert p.progress[0] == 0 and p.progress[-1] == 1
    assert not p.fresh.flags.writeable
    assert p.residual_vector(x).size == 6*n+3*(n-1)
    np.testing.assert_allclose(compose_poses(p.A,p.original_observation_local(p.target)),p.target,atol=2e-15)
    assert np.linalg.norm(p.original_observation_local(p.fresh)-relative_pose(p.B,p.fresh)) > .1


def test_nonuniform_arc_and_reject_degenerate():
    p=LocalSE2Problem([0,0,0],[.1,0,0],[[0,0,0],[.1,0,.2],[1,0,.4]])
    np.testing.assert_allclose(p.progress,[0,.1,1])
    for f in ([[0,0,0]],[[0,0,0],[0,0,.3]]):
        with pytest.raises(ValueError): LocalSE2Problem([0,0,0],[0,0,0],f)
    with pytest.raises(ValueError): p.residual_vector(p.fresh[:-1])


def test_unsafe_improving_rejection_same_damping_and_initial_fail_closed():
    x=np.array([[0.,0.,0.],[.1,0,0]])
    def residual(x): return (x-np.array([[1,0,0],[1.1,0,0]])).ravel()
    events=[]
    r=solve_least_squares(x,residual,SolverConfig(max_iterations=4),
                          candidate_feasibility_fn=lambda q: bool(q[0,0] <= .01), iteration_callback=events.append)
    rejected=[e for e in events if e['decision']=='rejected_unsafe']
    assert len(rejected)==4
    assert all(e['candidate_cost'] < e['cost_before'] and e['damping']==10*e['damping_before'] for e in rejected)
    np.testing.assert_array_equal(r.optimized,x)
    with pytest.raises(OptimizationError,match='initial state is infeasible'):
        solve_least_squares(x,residual,SolverConfig(),candidate_feasibility_fn=lambda q:False)


def test_callback_cannot_mutate_state_and_legacy_golden_parity():
    data=json.loads((Path(__file__).parent/'fixtures/local_se2_legacy_solver.json').read_text())
    for c in data['cases']:
        fn=lambda x:se2_log(relative_pose(np.array(c['target']),x)).ravel()
        r=solve_least_squares(c['initial'],fn,SolverConfig())
        np.testing.assert_array_equal(r.optimized,c['optimized']); assert r.to_dict()==c['result']
        def observer(e):e['state'][:]=999
        q=solve_least_squares(c['initial'],fn,SolverConfig(),candidate_feasibility_fn=lambda x:True,iteration_callback=observer)
        np.testing.assert_array_equal(q.optimized,r.optimized); assert q.to_dict()==r.to_dict()


def test_closed_form_rigid_diagnostic():
    p=fixture(); h=[1,-2,.5]; result=rigid_fit_diagnostic(p.fresh,compose_poses(h,p.fresh))
    np.testing.assert_allclose(result['transform'],h,atol=1e-14)
    assert result['translation_RMS_m'] < 1e-14 and result['yaw_RMS_rad'] < 1e-14


def test_damping_can_admit_smaller_feasible_step_without_projection():
    x=np.zeros((2,3)); events=[]
    result=solve_least_squares(x,lambda q:(q-np.array([1.,0,0])).ravel(),SolverConfig(max_iterations=8),
                              candidate_feasibility_fn=lambda q:bool(np.max(q[:,0])<=.01),iteration_callback=events.append)
    assert any(e['decision']=='rejected_unsafe' for e in events)
    assert any(e['decision']=='accepted' for e in events)
    assert 0 < result.optimized[0,0] <= .01
