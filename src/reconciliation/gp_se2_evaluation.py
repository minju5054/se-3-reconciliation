"""Independent dense plan/rollout checks and failure-first pilot comparisons."""
from __future__ import annotations

import numpy as np

from reconciliation.gp_se2_reference import directed_gate_crossings, interpolate_rows
from reconciliation.gp_se2_rollout import dense_rollout_samples, execution_metrics
from reconciliation.se2 import relative_pose, se2_log, wrap_angle

METHODS = ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID', 'M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')
PAIRS = [('M0_NATIVE','M0_ADAPTER'),('M0_ADAPTER','M3_GP_CONSTRAINED'),
         ('M0_NATIVE','M3_GP_CONSTRAINED'),('M1_RIGID','M3_GP_CONSTRAINED'),
         ('M2_GP_NO_OBSTACLE','M3_GP_CONSTRAINED')]


def goal_trace(poses, goal, position_tolerance=.15, yaw_tolerance=np.pi/12):
    p, g = np.asarray(poses, float), np.asarray(goal, float)
    distance = np.linalg.norm(p[:, :2]-g[:2], axis=1)
    yaw = np.abs(wrap_angle(p[:, 2]-g[2]))
    return distance, yaw, (distance <= position_tolerance) & (yaw <= yaw_tolerance)


def route_check(poses, times, goal_route, tolerance=1e-6):
    if goal_route['route_status'] == 'UNKNOWN':
        return dict(valid=False,status='UNKNOWN',gates=[],reason=goal_route.get('reason'))
    return directed_gate_crossings(np.asarray(poses)[:, :2],times,goal_route['gates'],tolerance=tolerance)


def environment_trace(env, poses, times, config, *, curve_error_bound=0.):
    """Use direct geometry, not optimizer's interpolated distance grid."""
    radius=config['footprint']['radius_m']; margin=config['footprint']['required_clearance_m']
    report=env.check_trajectory(times,poses,radius=radius,required_clearance=margin,
                                curved_path_error_bound_m=curve_error_bound)
    clearance=np.asarray(env.clearance(np.asarray(poses)[:,:2],radius),float)
    known=np.asarray(env.workspace_status(np.asarray(poses)[:,:2],radius),bool)
    overlap=np.flatnonzero(clearance < -env.obstacle_uncertainty_m)
    report.update(clearance_samples_m=clearance.tolist(),workspace_known_samples=known.tolist(),
                  first_sampled_overlap_time_s=float(times[overlap[0]]) if len(overlap) else None,
                  original_required_clearance_m=margin,geometry_uncertainty_m=env.obstacle_uncertainty_m,
                  continuous_time_collision_proof=False)
    segment=report.get('first_collision_segment_index')
    report['first_overlap_time_bracket_s']=None if segment is None else [float(times[segment]),float(times[segment+1])]
    return report


def evaluate_rollout(rollout, goal_route, env, config):
    c=config['formulation']; step=min(.005,config['evaluation']['maximum_time_step_s'])
    times,poses=dense_rollout_samples(rollout,max_dt_s=step)
    # A constant unicycle arc deviates from its chord by at most v*|w|*dt^2/8.
    # Sampling can straddle command changes, so use the more conservative
    # bounded world-acceleration deviation v*|w|*dt^2/2 plus speed-jump dt/2.
    # Include every integration/command boundary, avoiding such straddling.
    times=np.unique(np.r_[times,[r['time_s'] for r in rollout['states']]])
    commands=rollout['commands']; starts=np.array([r['time_s'] for r in commands])
    from reconciliation.robotless_online import integrate_unicycle
    exact=[]
    for t in times:
        j=min(int(np.searchsorted(starts,t,side='right')-1),len(commands)-1)
        if t >= rollout['horizon_s']:
            exact.append(rollout['states'][-1]['pose_world'])
        else:
            elapsed=float(t-starts[j])
            exact.append(rollout['states'][j]['pose_world'] if elapsed<=1e-14 else
                         integrate_unicycle(rollout['states'][j]['pose_world'],commands[j]['command'],elapsed))
    poses=np.asarray(exact)
    max_vw=max(abs(r['command'][0]*r['command'][1]) for r in commands)
    bound=float(max_vw*np.max(np.diff(times))**2/8)
    environment=environment_trace(env,poses,times,config,curve_error_bound=bound)
    route=route_check(poses,times,goal_route,config['evaluation']['gate_crossing_tolerance_m'])
    metrics=execution_metrics(rollout,goal_route['goal_world'],
        position_tolerance_m=c['goal_position_tolerance'],yaw_tolerance_rad=c['goal_yaw_tolerance'],
        dwell_s=config['evaluation']['terminal_goal_dwell_s'],
        limits=dict(v_max=c['v_max'],omega_max=c['w_max'],a_v_max=c['a_v_max'],a_omega_max=c['a_w_max']))
    reasons=[]
    if environment['physical_overlap']:reasons.append('new_collision')
    elif not environment['clearance_valid']:reasons.append('clearance_regression')
    if not environment['workspace_known']:reasons.append('unknown_workspace')
    if not route['valid']:reasons.append('route_violation' if route['status']!='UNKNOWN' else 'unknown_route')
    if not metrics['goal_reached'] or not metrics['terminal_goal_dwell_pass']:reasons.append('goal_failure')
    if not metrics['motion_limits_pass']:reasons.append('motion_violation')
    if metrics['controller_failure_count']:reasons.append('controller_numerical_failure')
    return dict(primary_success=not reasons,failure_reasons=reasons,execution=metrics,environment=environment,
                route=route,minimum_clearance_m=environment['minimum_clearance_m'],
                dense_times_s=times.tolist(),dense_poses_world=poses.tolist(),
                evaluation_kind='OFFLINE_COUNTERFACTUAL_KINEMATIC_LOCAL_TRANSITION',
                goal=goal_route['goal_world'],validation_level='SAMPLED_CLEARANCE_VALID_WITH_SWEPT_POLYLINE_CHECK')


def evaluate_plan(method, candidate, solver_result, common, boundary, initial_twist, goal_route, env, config):
    if candidate is None:
        return dict(plan_valid=False,status='NO_CANDIDATE',motion_applicable=method.startswith('M2') or method.startswith('M3'),
                    environment=None,route=None,goal=None,deformation=None)
    c=config['formulation']; candidate=np.asarray(candidate,float); motion=None
    if method in ('M2_GP_NO_OBSTACLE','M3_GP_CONSTRAINED'):
        from reconciliation.gp_se2 import sample_gp, interpolate_interval
        support=np.asarray(solver_result['support_poses']); twists=np.asarray(solver_result['support_twists'])
        knots=np.asarray(solver_result['support_times'])
        dt=config['evaluation'].get('gp_initial_query_step_s',.005)
        times=np.unique(np.r_[np.linspace(0,c['horizon_s'],round(c['horizon_s']/dt)+1),knots,(knots[1:]+knots[:-1])/2])
        poses,velocities,acceleration=sample_gp(knots,support,twists,times)
        refinement_trigger=config['evaluation'].get('gp_refinement_clearance_trigger_m',.10)
        refined=bool(np.min(env.clearance(poses[:,:2],config['footprint']['radius_m'])) <= refinement_trigger)
        if refined:
            dt=config['evaluation'].get('gp_refinement_minimum_step_s',.00125)
            times=np.unique(np.r_[np.linspace(0,c['horizon_s'],round(c['horizon_s']/dt)+1),knots])
            poses,velocities,acceleration=sample_gp(knots,support,twists,times)
        # Acceleration is generally discontinuous at support knots. Include
        # both limits in bounds, but do not straddle a knot in derivative probes.
        side_acceleration=[]
        for k in range(len(knots)-1):
            side_acceleration.extend(interpolate_interval(support[k],twists[k],support[k+1],twists[k+1],
                                     float(knots[k+1]-knots[k]),np.array([0.,1.]))[2])
        all_acceleration=np.vstack([acceleration,side_acceleration])
        endpoint_error=float(np.max(np.abs(sample_gp(knots,support,twists,knots)[1]-twists)))
        e=1e-5; interior=(times>e)&(times<c['horizon_s']-e)&(np.min(np.abs(times[:,None]-knots[None,:]),axis=1)>2*e)
        q=times[interior]
        left=sample_gp(knots,support,twists,q-e)[0]; right=sample_gp(knots,support,twists,q+e)[0]
        center,body,_=sample_gp(knots,support,twists,q)
        world_dot=(right[:,:2]-left[:,:2])/(2*e); yaw_dot=wrap_angle(right[:,2]-left[:,2])/(2*e)
        ct,st=np.cos(center[:,2]),np.sin(center[:,2])
        measured=np.column_stack((ct*world_dot[:,0]+st*world_dot[:,1],-st*world_dot[:,0]+ct*world_dot[:,1],yaw_dot))
        derivative_error=float(np.max(np.abs(measured-body)))
        tolerance=config['evaluation']['motion_numeric_tolerance']
        violation={
            'boundary_pose':float(np.max(np.abs(se2_log(relative_pose(boundary,support[0]))))) > tolerance,
            'boundary_twist':float(np.max(np.abs(twists[0]-initial_twist))) > tolerance,
            'lateral_velocity':float(np.max(np.abs(velocities[:,1]))) > tolerance,
            'linear_speed':float(np.min(velocities[:,0])) < -tolerance or float(np.max(velocities[:,0])) > c['v_max']+tolerance,
            'angular_speed':float(np.max(np.abs(velocities[:,2]))) > c['w_max']+tolerance,
            'linear_acceleration':float(np.max(np.abs(all_acceleration[:,0]))) > c['a_v_max']+tolerance,
            'angular_acceleration':float(np.max(np.abs(all_acceleration[:,2]))) > c['a_w_max']+tolerance,
            'pose_derivative_identity':derivative_error > tolerance,
            'support_velocity_reproduction':endpoint_error > tolerance,
        }
        motion=dict(valid=not any(violation.values()),violations=violation,pose_derivative_max_error=derivative_error,
                    body_twist=velocities.tolist(),body_acceleration=acceleration.tolist(),
                    one_sided_knot_acceleration=side_acceleration,
                    support_velocity_max_error=endpoint_error,
                    proximity_refined=refined,refinement_termination_step_s=dt,
                    independent_finite_difference_epsilon_s=e,sampled_only=True)
    else:
        first=config['reference']['first_time_s']; end=c['horizon_s']
        knots=np.linspace(first,end,len(candidate)) if len(candidate)>1 else np.array([first])
        times=np.linspace(first,end,581)
        poses=interpolate_rows(candidate,knots,times)
    environment=environment_trace(env,poses,times,config)
    route=route_check(poses,times,goal_route,config['evaluation']['gate_crossing_tolerance_m'])
    dist,yaw,within=goal_trace(poses,goal_route['goal_world'],c['goal_position_tolerance'],c['goal_yaw_tolerance'])
    deformation=None
    if method!='M0_NATIVE':
        residual=se2_log(relative_pose(common,candidate)); scaled=residual/np.asarray(c['fresh_std'])
        deformation=dict(uniform_mahalanobis_mean=float(np.mean(np.sum(scaled**2,axis=1))),
                         mean_translation_m=float(np.mean(np.linalg.norm(residual[:,:2],axis=1))),
                         max_translation_m=float(np.max(np.linalg.norm(residual[:,:2],axis=1))),
                         mean_abs_yaw_rad=float(np.mean(np.abs(residual[:,2]))),
                         reference='common prepared reference at identical 30 future samples')
    valid=bool(environment['clearance_valid'] and route['valid'] and within[-1] and (motion is None or motion['valid']))
    return dict(plan_valid=valid,status='SAMPLED_PLAN_VALID' if valid else 'PLAN_INVALID',
                environment=environment,route=route,motion=motion,motion_applicable=motion is not None,
                goal=dict(endpoint_in_tolerance=bool(within[-1]),position_error_m=float(dist[-1]),yaw_error_rad=float(yaw[-1])),
                deformation=deformation,dense_times_s=times.tolist(),dense_poses_world=poses.tolist(),
                initial_connector='GP optimized boundary state' if motion is not None else 'UNDEFINED; actual initial motion assessed by MPC rollout')


def comparisons(rows):
    """All attempts remain visible; secondary comparisons require paired success."""
    by={(r['case_id'],r['method']):r for r in rows}
    cases=sorted({r['case_id'] for r in rows}); regressions=[]; paired=[]
    metrics=['time_to_goal_s','terminal_position_error_m','terminal_yaw_error_rad','path_length_m',
             'first_command_delta_v_mps','first_command_delta_omega_radps',
             'command_total_variation_v_mps','command_total_variation_omega_radps',
             'control_grid_max_abs_acceleration_v_mps2','control_grid_max_abs_acceleration_omega_radps2',
             'goal_distance_improvement_m','net_displacement_m','goal_reentries_after_exit',
             'mpc_official_solve_total_s','mpc_official_solve_median_s','mpc_official_solve_max_s']
    for case in cases:
        for base,other in PAIRS:
            a,b=by[case,base],by[case,other]
            if a['primary_success'] and not b['primary_success'] and other=='M3_GP_CONSTRAINED':
                regressions.append(dict(case_id=case,baseline=base,method=other,reasons=b['failure_reasons']))
            if a['primary_success'] and b['primary_success']:
                ar,br=a['rollout_metrics'],b['rollout_metrics']
                item=dict(case_id=case,baseline=base,method=other,both_primary_success=True)
                for key in metrics:
                    av,bv=ar['execution'].get(key),br['execution'].get(key)
                    item[key+'_baseline']=av;item[key+'_method']=bv
                    item[key+'_difference']=None if av is None or bv is None else bv-av
                for label,r in [('baseline',ar),('method',br)]:item['minimum_clearance_m_'+label]=r['minimum_clearance_m']
                for label,r in [('baseline',a),('method',b)]:
                    item['optimization_and_check_wall_s_'+label]=r.get('optimizer_wall_s')
                    item['fresh_deformation_'+label]=r.get('deformation')
                paired.append(item)
    method_summary={m:dict(attempts=sum(r['method']==m for r in rows),
        candidates=sum(r['method']==m and r['candidate_found'] for r in rows),
        plan_valid=sum(r['method']==m and r['plan_valid'] for r in rows),
        primary_success=sum(r['method']==m and r['primary_success'] for r in rows),
        failure_cases=[dict(case_id=r['case_id'],reasons=r['failure_reasons']) for r in rows if r['method']==m and not r['primary_success']]) for m in METHODS}
    return dict(methods=method_summary,regressions=regressions,paired_metrics=paired,
                native_success_to_gp_failure=sum(r['baseline']=='M0_NATIVE' for r in regressions),
                adapter_success_to_gp_failure=sum(r['baseline']=='M0_ADAPTER' for r in regressions))
