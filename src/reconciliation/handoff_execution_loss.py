"""Saved native execution costs, not a new objective, controller, or rollout."""
from __future__ import annotations

import numpy as np

from .gp_se2_join01 import POSITION_M, YAW_RAD, DWELL_S, sustained_join
from .robotless_online import integrate_unicycle
from .se2 import wrap_angle


def attachment_loss(times, poses, fresh):
    """Use the frozen sampled attachment definition; preserve right censoring."""
    t=np.asarray(times,float)
    if len(t)<2 or t[0]!=0:
        raise ValueError('at least two saved samples starting exactly at B time zero required')
    result=sustained_join(t,poses,fresh)
    d=result['distance_m'];yaw=result['yaw_error_rad'];good=(d<=POSITION_M)&(yaw<=YAW_RAD)
    joined=result['sustained_join_time_s']
    j=None if joined is None else int(np.searchsorted(t,joined))
    begins=np.flatnonzero(good & ~np.r_[False,good[:-1]])
    ends=np.flatnonzero(good & ~np.r_[good[1:],False])
    runs=[dict(first_s=float(t[a]),last_s=float(t[b]),observed_span_s=float(t[b]-t[a]),
               reaches_window_end=bool(b==len(t)-1)) for a,b in zip(begins,ends)]
    if joined is not None:status='OBSERVED_SAMPLED_JOIN'
    elif not runs:status='NO_TUBE_ENTRY_OBSERVED'
    elif runs[-1]['reaches_window_end']:status='TUBE_ENTERED_DWELL_RIGHT_CENSORED'
    else:status='TRANSIENT_ENTRY_THEN_EXIT'
    return dict(join_success=result['join_success'],join_time_s=joined,
        observation_status=status,inside_tube_runs=runs,
        first_tube_entry_time_s=None if not runs else runs[0]['first_s'],
        longest_observed_inside_span_s=max((r['observed_span_s'] for r in runs),default=0.),
        observation_horizon_s=float(t[-1]),latest_testable_join_start_s=max(0.,float(t[-1]-DWELL_S)),
        B_distance_m=float(d[0]),B_yaw_error_deg=float(np.degrees(yaw[0])),
        end_distance_m=float(d[-1]),end_yaw_error_deg=float(np.degrees(yaw[-1])),
        mean_distance_time_weighted_m=float(np.trapezoid(d,t)/t[-1]),
        max_distance_m=float(d.max()),maximum_distance_time_s=float(t[int(np.argmax(d))]),
        position_auc_m_s=float(np.trapezoid(d,t)),
        position_excess_auc_m_s=float(np.trapezoid(np.maximum(d-POSITION_M,0),t)),
        yaw_auc_rad_s=float(np.trapezoid(yaw,t)),
        yaw_excess_auc_rad_s=float(np.trapezoid(np.maximum(yaw-YAW_RAD,0),t)),
        sampled_bad_fraction=float(np.mean(~good)),
        outside_tube_left_sample_quadrature_s=float(np.sum(np.diff(t)*(~good[:-1]))),
        pre_join_position_auc_m_s=result['pre_join_position_auc_m_s'],
        exits_after_first_join=None if j is None else bool(np.any(~good[j:])),
        post_join_max_distance_m=result['post_join_max_distance_m'],
        post_join_mean_distance_m=result['post_join_mean_distance_m'],
        progress_monotonic=bool(np.all(np.diff([r['progress'] for r in result['projection']])>=0)),
        trace=dict(times_s=t,poses_world=np.asarray(poses),distance_m=d,yaw_error_rad=yaw,
                   good=good,projection=result['projection']),
        sampled_only=True,continuous_time_attachment_proof=False)


def command_loss(times, poses, commands, u_minus, config):
    t=np.asarray(times,float);p=np.asarray(poses,float);u=np.asarray(commands,float)
    if u.shape!=(len(t)-1,2) or p.shape!=(len(t),3) or np.any(np.diff(t)<=0):
        raise ValueError('one held command per original recorded integration interval required')
    delta=np.diff(np.vstack([u_minus,u]),axis=0)
    hz=float(config['command_grid_hz']);a=np.abs(delta)*hz
    limits=config['motion_limits'];tol=limits['numeric_tolerance']
    flags=dict(linear_speed=bool(np.any(u[:,0]<-1e-6) or np.any(u[:,0]>limits['v_max_mps']+1e-6)),
        angular_speed=bool(np.any(abs(u[:,1])>limits['omega_max_radps']+1e-6)),
        nominal_grid_linear_acceleration=bool(np.any(a[:,0]>limits['nominal_grid_a_v_max_mps2']+tol)),
        nominal_grid_angular_acceleration=bool(np.any(a[:,1]>limits['nominal_grid_a_omega_max_radps2']+tol)))
    reconstructed=np.array([integrate_unicycle(x,c,float(dt)) for x,c,dt in zip(p[:-1],u,np.diff(t))])
    error=reconstructed-p[1:];error[:,2]=wrap_angle(error[:,2])
    reconstruction=float(np.max(np.abs(error)))
    if reconstruction>1e-10:raise ValueError('saved commands do not reconstruct original execution')
    return dict(linear_TV_mps=float(np.abs(delta[:,0]).sum()),angular_TV_radps=float(np.abs(delta[:,1]).sum()),
        first_delta_v_mps=float(delta[0,0]),first_delta_omega_radps=float(delta[0,1]),
        nominal_10Hz_max_delta_v_over_dt_mps2=float(a[:,0].max()),
        nominal_10Hz_max_delta_omega_over_dt_radps2=float(a[:,1].max()),
        nominal_command_grid_violations=flags,nominal_command_grid_valid=not any(flags.values()),
        max_forward_speed_mps=float(u[:,0].max()),min_forward_speed_mps=float(u[:,0].min()),
        max_abs_omega_radps=float(np.abs(u[:,1]).max()),
        actual_path_length_m=float(np.sum(abs(u[:,0])*np.diff(t))),
        accumulated_yaw_variation_rad=float(np.sum(abs(u[:,1])*np.diff(t))),
        reconstruction_max_error=reconstruction,
        curve_bound_m=float(np.max(abs(u[:,0]*u[:,1])*np.diff(t)**2/8)),
        commands=u,command_start_times_s=t[:-1],
        acceleration_semantics='10Hz command-difference diagnostic, including physical u_minus; not continuous acceleration or 60Hz difference',
        reconstruction_only_no_new_rollout=True)


def audit_case(case, environment, config):
    post=case['post'];row=case['row']
    if post is None:raise ValueError('missing actual post-switch execution')
    t=post['times']-post['times'][0];p=post['poses'];fresh=case['fresh']
    u=np.array([[float(r['v_mps']),float(r['omega_radps'])] for r in post['commands']])
    u_minus=np.array([row['motion']['v_minus_mps'],row['motion']['omega_minus_radps']])
    n=config['common_window_integration_steps']
    if len(u)<n:raise ValueError('insufficient recorded common window; no padding')
    full=attachment_loss(t,p,fresh);common=attachment_loss(t[:n+1],p[:n+1],fresh)
    full['window_end_reason']='NEXT_REFERENCE_APPLICATION'
    common['window_end_reason']='FIXED_FIRST_54_RECORDED_INTERVALS'
    command=command_loss(t,p,u,u_minus,config)
    common_command=command_loss(t[:n+1],p[:n+1],u[:n],u_minus,config)
    geometry=environment.check_trajectory(t,p,radius=config['footprint_radius_m'],
        required_clearance=config['required_clearance_m'],curved_path_error_bound_m=command['curve_bound_m'])
    is_interior=row['mismatch']['projection']['projection_location']=='interior'
    return dict(case_id=row['case_id'],episode_id=row['case_id'].split('/')[0],
        ordered_raw_pair=row['ordered_raw_pair'],source_paths=row['source_paths'],
        projection_group='INTERIOR' if is_interior else 'ENDPOINT_CAVEAT',
        preexisting_B_mismatch=row['mismatch'],full=full,common=common,
        command=command,common_command=common_command,environment=geometry,
        physical_u_minus=u_minus,previous_control=row['motion']['previous_control'],
        memory_discrepancy=not np.allclose(u_minus,row['motion']['previous_control'],atol=1e-12,rtol=0),
        route_status=row['route_status'],original_goal_success='NOT_EVALUATED_OVER_VARIABLE_REFERENCE_LIFETIME',
        new_solver_model_rollout_calls=0)
