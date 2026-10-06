"""Synthetic algebra/solver tests; saved parity uses no new scientific solve."""
from dataclasses import asdict
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).parents[1];sys.path[:0]=[str(ROOT/'scripts')]
from reconciliation.canonical_se2_graph import CanonicalSE2Problem
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.se2 import compose_poses,relative_pose,se2_log,se2_exp,wrap_angle
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares,OptimizationError,retract_trajectory
from validate_local_se2_reconciliation_formulation01 import independent_costs,parity


def toy(n=5):
    u=np.linspace(0,1,n)**2
    return CanonicalSE2Problem([1,2,.3],[1.2,2.1,.5],np.c_[1+u,2+.2*u*u,3.1+.2*u])


@pytest.mark.parametrize('n',[2,3,5,11])
def test_generic_progress_and_full_default_parity(n):
    p=toy(n);old=LocalSE2Problem(p.A,p.B,p.fresh)
    assert p.fresh.shape==(n,3)
    np.testing.assert_array_equal(p.target,old.target);np.testing.assert_array_equal(p.progress,old.progress)
    np.testing.assert_array_equal(p.progress,p.arc/p.arc[-1]);assert np.all(np.diff(p.progress)>0)
    for state in [p.fresh,p.target,compose_poses(p.fresh,se2_exp(np.full((n,3),.05)))]:
        assert list(p.residual_blocks(state))==['L','R','A']
        for key in ['L','R','A']:
            np.testing.assert_array_equal(p.raw_residuals(state)[key],old.raw_residuals(state)[key])
            np.testing.assert_array_equal(p.residual_blocks(state)[key],old.residual_blocks(state)[key])
        np.testing.assert_array_equal(p.residual_vector(state),old.residual_vector(state))
        assert p.costs(state)==old.costs(state);parity(p.costs(state),independent_costs(p.fresh,p.target,state))
        assert len(p.residual_vector(state))==3*(3*n-1)
    with pytest.raises(TypeError):p.residual_vector(p.fresh,include_relative=False)


@pytest.mark.parametrize('n',[2,7])
def test_exact_residual_equations_zero_and_common_left_invariance(n):
    p=toy(n);f=p.fresh
    for key in ['R','A']:np.testing.assert_allclose(p.raw_residuals(f)[key],0,atol=1e-14)
    for key in ['L','R']:np.testing.assert_allclose(p.raw_residuals(p.target)[key],0,atol=1e-14)
    x=compose_poses([-.2,.6,-.7],f)
    np.testing.assert_allclose(p.raw_residuals(x)['R'],0,atol=1e-14)
    np.testing.assert_array_equal(p.raw_residuals(x)['L'],se2_log(relative_pose(p.target,x)))
    np.testing.assert_array_equal(p.raw_residuals(x)['A'],se2_log(relative_pose(f,x)))
    g=[2.,-1.,1.];q=CanonicalSE2Problem(compose_poses(g,p.A),compose_poses(g,p.B),compose_poses(g,f))
    np.testing.assert_allclose(q.residual_vector(compose_poses(g,x)),p.residual_vector(x),atol=5e-14)


def test_frames_immutable_no_B_reanchor_shortest_wrap():
    A=np.array([4.,2.,3.1]);B=np.array([4.4,2.1,-3.1]);local=np.array([[.1,0,0],[.4,.1,.1]])
    fresh=compose_poses(A,local);bits=fresh.tobytes();p=CanonicalSE2Problem(A,B,fresh)
    A[0]=9;fresh[0,0]=999
    assert p.fresh.tobytes()==bits and p.A[0]==4
    for value in [p.fresh,p.A,p.B]:assert not value.flags.writeable
    np.testing.assert_allclose(p.original_observation_local(p.fresh),local,atol=1e-14)
    np.testing.assert_allclose(relative_pose(p.B,p.target),relative_pose(p.A,p.fresh),atol=1e-14)
    assert not np.allclose(compose_poses(B,local),p.fresh)
    x=p.fresh.copy();x[:,2]+=2*np.pi;np.testing.assert_allclose(p.raw_residuals(x)['A'],0,atol=1e-14)


@pytest.mark.parametrize('fresh',[[[0,0,0]],[[0,0,0],[0,0,.5]],[[0,0,0],[np.nan,0,0]],[[0,0,0],[1,0,np.inf]]])
def test_invalid_finite_N_and_zero_arc(fresh):
    with pytest.raises(ValueError):CanonicalSE2Problem([0,0,0],[.1,0,0],fresh)


def test_unchanged_solver_defaults_raw_init_right_local_and_last_feasible():
    p=toy(2);before=p.fresh.copy();trace=[];seen=[]
    def feasible(x):seen.append(x.copy());return np.array_equal(x,before)
    result=solve_least_squares(p.fresh,p.residual_vector,SolverConfig(),candidate_feasibility_fn=feasible,iteration_callback=trace.append)
    np.testing.assert_array_equal(result.optimized,before)
    assert result.termination_reason=='step_tolerance'
    np.testing.assert_array_equal(seen[0],before)
    assert any(e['decision']=='rejected_unsafe' for e in trace)
    for e in trace:
        np.testing.assert_array_equal(e['state'],before)
        if 'candidate' in e:np.testing.assert_array_equal(e['candidate'],retract_trajectory(before,e['delta']))
    np.testing.assert_array_equal(p.fresh,before)
    import yaml
    cfg=yaml.safe_load((ROOT/'configs/canonical_se2_graph_formulation_audit_01.yaml').read_text())
    assert cfg['solver']==asdict(SolverConfig())


def test_synthetic_safe_solve_retains_shape_and_cost_decreases():
    p=toy(3);result=solve_least_squares(p.fresh,p.residual_vector,SolverConfig(),candidate_feasibility_fn=lambda x:True)
    assert result.converged and result.optimized.shape==p.fresh.shape and result.final_cost<result.initial_cost


def test_OSA03_historical_algebra_parity_without_resolve():
    from reconciliation.join_source03 import read
    path=ROOT/'data/local_se2_reconciliation_formulation_01/primary_20260925T013000Z'
    if not path.exists():pytest.skip('sealed source not installed')
    saved=read(path/'result.json');f=np.load(path/'derived/original_world.npy');x=np.load(path/'derived/optimized_world.npy')
    p=CanonicalSE2Problem(saved['A'],saved['B'],f);old=LocalSE2Problem(saved['A'],saved['B'],f)
    np.testing.assert_array_equal(p.target,np.load(path/'derived/transported_target.npy'))
    for state in [f,x,p.target]:
        np.testing.assert_array_equal(p.residual_vector(state),old.residual_vector(state))
        parity(p.costs(state),independent_costs(f,p.target,state))
    parity(p.costs(x),saved['final'])
