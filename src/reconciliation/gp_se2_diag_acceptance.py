"""Independent sampled verification of diagnostic candidates, never a fallback.

The historical formulation and physical thresholds are reused without overrides.
An additional offset grid checks interpolation between the historical query times.
No finite set of samples is described as a continuous-time feasibility proof.
"""
from __future__ import annotations

import numpy as np

from .gp_se2 import interpolate_interval, sample_gp
from .gp_se2_evaluation import evaluate_plan, environment_trace, route_check
from .se2 import relative_pose, se2_log, wrap_angle


def offset_grid(times, maximum_step_s=.001, offset_fraction=.371):
    if maximum_step_s<=0 or not 0<offset_fraction<1:
        raise ValueError('positive step and interior offset fraction required')
    knots=np.asarray(times,float)
    n=int(np.ceil((knots[-1]-knots[0])/maximum_step_s))
    base=np.linspace(knots[0],knots[-1],n+1)
    offset=base[:-1]+offset_fraction*np.diff(base)
    return np.unique(np.r_[base,offset,knots,(knots[:-1]+knots[1:])/2])


def check_full_candidate(problem, vector, case, *, maximum_step_s=.001, offset_fraction=.371):
    """Check original dense solver rules, original independent plan and offset grid."""
    poses,twists=problem.unpack(vector)
    c=problem.config
    method='M3_GP_CONSTRAINED' if problem.include_obstacles else 'M2_GP_NO_OBSTACLE'
    solver=dict(support_poses=poses,support_twists=twists,support_times=problem.times)
    dense=problem.dense_report(vector)
    plan=evaluate_plan(method,poses[1:],solver,problem.common_reference,
                       problem.boundary_pose,problem.initial_twist,case['goal_route'],
                       case['environment'],case['config'])
    qt=offset_grid(problem.times,maximum_step_s,offset_fraction)
    pp,vv,aa=sample_gp(problem.times,poses,twists,qt)
    _,side_v,side_a=interpolate_interval(poses[:-1,None],twists[:-1,None],poses[1:,None],
                                       twists[1:,None],c['support_dt_s'],np.array([0.,1.]))
    av=np.vstack([aa,side_a.reshape(-1,3)])
    vel=np.vstack([vv,side_v.reshape(-1,3)])
    epsilon=1e-5
    probes=qt[(qt>epsilon)&(qt<problem.times[-1]-epsilon)&
              (np.min(np.abs(qt[:,None]-problem.times[None,:]),axis=1)>2*epsilon)]
    center,body,_=sample_gp(problem.times,poses,twists,probes)
    left=sample_gp(problem.times,poses,twists,probes-epsilon)[0]
    right=sample_gp(problem.times,poses,twists,probes+epsilon)[0]
    dxy=(right[:,:2]-left[:,:2])/(2*epsilon)
    theta=center[:,2]
    measured=np.column_stack([np.cos(theta)*dxy[:,0]+np.sin(theta)*dxy[:,1],
        -np.sin(theta)*dxy[:,0]+np.cos(theta)*dxy[:,1],wrap_angle(right[:,2]-left[:,2])/(2*epsilon)])
    derivative_errors=np.max(np.abs(measured-body),axis=0)
    environment=environment_trace(case['environment'],pp,qt,case['config'])
    route=route_check(pp,qt,case['goal_route'],case['config']['evaluation']['gate_crossing_tolerance_m'])
    etol=c['equality_tolerance'];itol=c['inequality_tolerance']
    flags=dict(
        boundary_pose=bool(np.array_equal(poses[0],problem.boundary_pose)),
        initial_twist=bool(np.array_equal(twists[0],problem.initial_twist)),
        pose_body_derivative_identity=bool(np.max(derivative_errors)<=etol),
        lateral_velocity=bool(np.max(np.abs(vel[:,1]))<=etol),
        linear_speed=bool(np.min(vel[:,0])>=-itol and np.max(vel[:,0])<=c['v_max']+itol),
        angular_speed=bool(np.max(np.abs(vel[:,2]))<=c['w_max']+itol),
        linear_acceleration=bool(np.max(np.abs(av[:,0]))<=c['a_v_max']+itol),
        angular_acceleration=bool(np.max(np.abs(av[:,2]))<=c['a_w_max']+itol),
        original_goal=bool(plan['goal']['endpoint_in_tolerance']),
        original_route=bool(route['valid']),original_workspace=bool(environment['workspace_known']),
        original_obstacle_clearance=bool(environment['clearance_valid']))
    result=dict(original_dense_feasible=bool(dense['feasible']),original_plan_valid=bool(plan['plan_valid']),
        original_dense_report=dense,original_plan_report=plan,
        additional_grid=dict(flags=flags,valid=all(flags.values()),query_count=len(qt),
            maximum_step_s=float(np.max(np.diff(qt))),offset_fraction=offset_fraction,
            derivative_probe_count=len(probes),finite_difference_epsilon_s=epsilon,
            pose_body_derivative_error_by_component=derivative_errors,
            derivative_error_units=['m/s','m/s','rad/s'],
            maximum_absolute_lateral_velocity_m_s=float(np.max(np.abs(vel[:,1]))),
            minimum_linear_speed_m_s=float(np.min(vel[:,0])),maximum_linear_speed_m_s=float(np.max(vel[:,0])),
            maximum_absolute_angular_speed_rad_s=float(np.max(np.abs(vel[:,2]))),
            maximum_absolute_linear_acceleration_m_s2=float(np.max(np.abs(av[:,0]))),
            maximum_absolute_angular_acceleration_rad_s2=float(np.max(np.abs(av[:,2]))),
            environment=environment,route=route,query_times_s=qt,
            poses_world=pp,body_twists=vv,body_accelerations=aa),
        physical_acceptance_unchanged=True,original_config=case['config'],
        continuous_time_proof=False,new_execution_performed=False)
    result['full_feasible']=bool(dense['feasible'] and plan['plan_valid'] and all(flags.values()))
    result['status']='FULL_FEASIBLE_CANDIDATE_FOUND' if result['full_feasible'] else 'NO_FULL_FEASIBLE_CANDIDATE_FOUND'
    return result
