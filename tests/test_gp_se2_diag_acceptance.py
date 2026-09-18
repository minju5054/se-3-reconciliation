"""Additional sampled checking keeps the original physics and frame conventions."""
import numpy as np
import pytest

from reconciliation.gp_se2_diag_acceptance import check_full_candidate, offset_grid
from reconciliation.gp_se2_formulation import GPProblem
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.se2 import compose_poses,se2_exp


def make_case():
    from shapely.geometry import box
    environment=HospitalEnvironment(box(10,10,11,11),box(-5,-5,5,5))
    boundary=np.array([1.,-1.,.4]);twist=np.array([.3,0.,.4]);times=np.linspace(0,3,31)
    poses=compose_poses(boundary,se2_exp(times[:,None]*twist))
    config=dict(footprint=dict(radius_m=.2,required_clearance_m=.05),
        evaluation=dict(gp_initial_query_step_s=.005,gp_refinement_minimum_step_s=.00125,
                        gp_refinement_clearance_trigger_m=.1,gate_crossing_tolerance_m=1e-6,
                        motion_numeric_tolerance=1e-5))
    problem=GPProblem(boundary,twist,poses[1:],poses[-1],{},
        lambda xy:environment.optimizer_clearance(xy,.2),lambda xy:environment.workspace_margin(xy,.2),[],True)
    config['formulation']=problem.config
    case=dict(config=config,environment=environment,
              goal_route=dict(goal_world=poses[-1],gates=[],route_status='NOT_REQUIRED_CLEAR_SHORTCUT'))
    x=problem.vector(poses,np.tile(twist,(31,1)))
    return problem,x,case


def test_offset_grid_includes_supports_and_queries_outside_collocation():
    knots=np.linspace(0,3,31);qt=offset_grid(knots)
    assert all(t in qt for t in knots)
    assert np.max(np.diff(qt))<=.001+1e-12
    assert .000371 in qt
    assert qt[0]==0 and qt[-1]==3
    with pytest.raises(ValueError):offset_grid(knots,offset_fraction=1)


def test_known_turning_curve_passes_original_and_independent_grid():
    p,x,case=make_case();before=p.config.copy()
    result=check_full_candidate(p,x,case)
    assert result['full_feasible']
    assert all(result['additional_grid']['flags'].values())
    assert result['additional_grid']['maximum_absolute_lateral_velocity_m_s']<1e-10
    assert result['additional_grid']['derivative_probe_count']>3000
    assert p.config==before
    assert not result['new_execution_performed']


def test_original_obstacle_constraints_apply_even_to_m2_full_acceptance():
    from shapely.geometry import box
    p,x,case=make_case();p.include_obstacles=False
    case['environment']=HospitalEnvironment(box(.5,-1.5,2,1),box(-5,-5,5,5))
    result=check_full_candidate(p,x,case)
    assert result['original_dense_feasible']
    assert not result['full_feasible']
    assert not result['additional_grid']['flags']['original_obstacle_clearance']


def test_slipping_curve_is_rejected_without_zero_metric_or_fallback():
    p,x,case=make_case();x=x.copy();x[6]+=.01
    result=check_full_candidate(p,x,case)
    assert not result['full_feasible']
    assert result['additional_grid']['maximum_absolute_lateral_velocity_m_s']>1e-5
