"""Saved-record REF-04 audit. No controller, optimizer or simulation is invoked.

Reconstruction evaluates an already applied held command, never a new policy.
Prediction polylines and selected-target connectors are diagnostic geometries;
the original evaluator remains the authority for full execution acceptance.
"""
from __future__ import annotations
import numpy as np
from .gp_se2_evaluation import evaluate_rollout
from .robotless_online import integrate_unicycle
from .se2 import wrap_angle

METHODS = ('B_DENSE_ROW_STEP', 'C_DENSE_SOURCE_PROGRESS')
CASE_ID = 'episode_017_repeat_00/handoff_007'
DT = .1
HORIZON = 5
PROTOCOL = dict(
    experiment='GP-SE2-REF-04', case_id=CASE_ID, methods=list(METHODS),
    new_VLA_inferences=0, new_GP_or_rigid_solves=0, new_MPC_solves=0, new_rollouts=0,
    scope='read-only saved-record audit; A_NATIVE historical context only',
    prediction_geometry='SAVED_DISCRETE_PREDICTION_POLYLINE',
    selected_connections='diagnostic straight connectors, not predicted or executed motion',
    prediction_dt_s=DT, prediction_horizon_steps=HORIZON,
    first_interval='[issue,issue+0.1]', unexecuted_tail='(issue+0.1,issue+0.5]',
    refinement_max_bracket_width_s=.001,
    clearance='original direct swept geometry, radius/clearance/uncertainty unchanged; original full-run curve bound retained in executed subintervals',
    refinement='leftmost failing swept half, exact saved-command position for execution, linear interpolation for prediction; no endpoint-only safety assertion',
    minimum_time='linear parameter of closest point on original checked segment; not an exact continuous arc extremum',
    gate='original directed center-interval geometry/tolerance/order, no second footprint subtraction',
    reconstruction_atol=1e-10, prediction_input_atol=1e-8, reference_atol=1e-8,
    command_divergence_atol=1e-6, original_outcome_comparison='literal equality of original evaluator fields',
    representative_rule=['t=0','C first prospective prediction violation solve','C first actual violation control interval',
                         'C minimum clearance control interval','C first invalid gate crossing control interval'],
    missing='N/A with reason; no replacement predictions or solves',
    model_limitations=['future optimized controls and pre-clipping first control not saved',
        'saved prediction versus applied Euler can include clipping and solver numeric error',
        'after first interval prediction/actual differences include replanning',
        'prediction nodes beyond recorded 3 s have no actual counterpart'],
)


def execution_at(rollout, t):
    """Interpolate only a stored held command from its saved integration state."""
    t = float(t)
    if not np.isfinite(t) or t < 0 or t > float(rollout['horizon_s'])+1e-12:
        raise ValueError('time outside saved execution; no extrapolation')
    if t >= rollout['horizon_s']:
        return np.asarray(rollout['states'][-1]['pose_world'], float)
    starts = np.asarray([x['time_s'] for x in rollout['commands']])
    i = max(0, int(np.searchsorted(starts, t, side='right')-1))
    pose = np.asarray(rollout['states'][i]['pose_world'], float)
    elapsed = t-starts[i]
    return pose.copy() if elapsed <= 1e-14 else integrate_unicycle(pose, rollout['commands'][i]['command'], elapsed)


def euler_endpoint(pose, command, dt=DT):
    p = np.asarray(pose, float).copy(); v,w = np.asarray(command, float)
    return p + dt*np.array([v*np.cos(p[2]), v*np.sin(p[2]), w])


def prediction_data(record):
    value = record.get('prediction_world')
    reason = None
    try:
        p = np.asarray(value, float)
        if value is None: reason = 'MISSING_SAVED_PREDICTION'
        elif p.shape != (HORIZON+1,3): reason = 'PREDICTION_SHAPE_MUST_BE_6_BY_3'
        elif not np.all(np.isfinite(p)): reason = 'NONFINITE_SAVED_PREDICTION'
    except (TypeError, ValueError):
        reason = 'MALFORMED_SAVED_PREDICTION'
    if reason:
        return dict(available=False, reason=reason, poses_world=None, node_times_s=None,
                    current_node_matches_input=None)
    same = bool(np.allclose(p[0], record['input_pose_world'], atol=PROTOCOL['prediction_input_atol'], rtol=0))
    return dict(available=True, reason=None, poses_world=p.tolist(),
                node_times_s=(float(record['time_s'])+np.arange(HORIZON+1)*DT).tolist(),
                current_node_matches_input=same,
                current_node_bitwise_equal=bool(np.array_equal(p[0],record['input_pose_world'])))


def _pose_error(a,b):
    d=np.asarray(a)-np.asarray(b)
    return dict(xy_m=float(np.linalg.norm(d[:2])),yaw_rad=float(abs(wrap_angle(d[2]))))


def _prefix_times(outcome, begin, end):
    t=np.asarray(outcome['dense_times_s'])
    return np.unique(np.r_[begin,t[(t>begin+1e-12)&(t<end-1e-12)],end])


def _world_to_local(rows, pose):
    rows=np.asarray(rows);pose=np.asarray(pose); d=rows[:,:2]-pose[:2]
    c,s=np.cos(pose[2]),np.sin(pose[2])
    return np.column_stack([c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1],rows[:,2]-pose[2]])


def audit_method(rollout, stored_metrics, route, env, config):
    from .gp_se2_ref04_geometry import polyline_audit, gate_audit, spatial_gate_audit
    outcome=evaluate_rollout(rollout,route,env,config)
    matches={k:outcome[k]==stored_metrics.get(k) for k in outcome}
    bound=outcome['environment']['curved_path_error_bound_m']
    at=lambda t:execution_at(rollout,t)
    geom=lambda p,t=None,b=0.,f=None:polyline_audit(p,env,config,times=t,curve_bound=b,pose_at=f)
    gates=lambda p,t,f=None,scope='PREDICTION':gate_audit(p,t,route['gates'],env,config,pose_at=f,scope=scope)
    full=geom(outcome['dense_poses_world'],outcome['dense_times_s'],bound,at)
    crossings=gates(outcome['dense_poses_world'],outcome['dense_times_s'],at,'FULL_EXECUTION')
    reconstruction=[]
    for i,cmd in enumerate(rollout['commands']):
        reconstructed=integrate_unicycle(rollout['states'][i]['pose_world'],cmd['command'],cmd['end_time_s']-cmd['time_s'])
        reconstruction.append(_pose_error(reconstructed,rollout['states'][i+1]['pose_world']))
    records=[]; selections=rollout['controller_reference_selections']
    for k,r in enumerate(selections):
        t=float(r['time_s']); end=min(t+DT,rollout['horizon_s']); pose=np.asarray(r['input_pose_world'],float)
        targets=np.asarray(r['actual_submitted_reference_world'],float)
        if targets.shape != (5,3) or not np.all(np.isfinite(targets)):
            raise ValueError('actual saved reference must be 5 finite target poses')
        diagnostic=r['selection_diagnostic']; indices=diagnostic['indices']
        installed_path=np.asarray(rollout['installed_reference_world'])
        if len(indices)!=5 or any(not isinstance(i,(int,np.integer)) or i<0 or i>=len(installed_path) for i in indices):
            raise ValueError('saved target indices must name five installed rows')
        installed=installed_path[indices]
        local_expected=_world_to_local(targets,pose)
        local=np.asarray(r['actual_controller_reference_local'])
        local_error=float(np.max(np.abs(local_expected-local)))
        target_error=float(np.max(np.abs(installed[:,:2]-targets[:,:2])))
        yaw_error=float(np.max(np.abs(wrap_angle(installed[:,2]-targets[:,2]))))
        selected=dict(poses_world=targets.tolist(),indices=indices,
            source_progress=diagnostic['selected_original_fractional_row_coordinates'],
            actual_controller_reference_local=local.tolist(), sequential_unwrapped_yaw=targets[:,2].tolist(),
            current_input_pose_world=pose.tolist(), nearest_index=diagnostic['nearest_index'],
            source_key=f'controller_reference_selections[{k}].actual_submitted_reference_world',
            points=[geom([p]) for p in targets],target_polyline=geom(targets),
            entry_connector=geom(np.vstack([pose,targets[0]])),
            target_polyline_gate_crossings=spatial_gate_audit(targets,route['gates'],env,config),
            entry_connector_gate_crossings=spatial_gate_audit(np.vstack([pose,targets[0]]),route['gates'],env,config),
            connections_are_diagnostic_not_execution=True,
            actual_local_reference_max_error=local_error, installed_target_xy_max_error_m=target_error,
            installed_target_periodic_yaw_max_error_rad=yaw_error)
        times=_prefix_times(outcome,t,end); prefixposes=np.asarray([at(x) for x in times])
        exact=integrate_unicycle(pose,r['command'],end-t)
        euler=euler_endpoint(pose,r['command'],end-t); actual=at(end)
        prefix_geometry=geom(prefixposes,times,bound,at)
        prefix=dict(times_s=times.tolist(),poses_world=prefixposes.tolist(),geometry=prefix_geometry,
            applied_command=r['command'],previous_control=r['previous_control'],
            euler_endpoint=euler.tolist(),exact_endpoint=exact.tolist(),actual_endpoint=actual.tolist(),
            saved_prediction_endpoint=None,gate_crossings=gates(prefixposes,times,at,'APPLIED_PREFIX'),
            source_key=f'controller_reference_selections[{k}].command; states/commands of same stored rollout')
        prefix['applied_euler_geometry']=geom([pose,euler],[t,end])
        prefix['applied_euler_exact_clearance_classification_differs']=bool(prefix['applied_euler_geometry']['clearance_valid']!=prefix_geometry['clearance_valid'])
        pred=prediction_data(r)
        pred.update(geometry_kind='SAVED_DISCRETE_PREDICTION_POLYLINE',
                    source_key=f'controller_reference_selections[{k}].prediction_world',
                    future_solved_controls_available=False,preclipping_control_available=False)
        if pred['available']:
            pp=np.asarray(pred['poses_world']); pt=np.asarray(pred['node_times_s'])
            pred['geometry']=geom(pp,pt)
            pred['first_interval']=geom(pp[:2],pt[:2])
            pred['tail']=geom(pp[1:],pt[1:])
            pred['tail_boundary_note']='offset .1 is retained only as the segment boundary; future tail begins after .1'
            start_clear=float(env.clearance(pose[:2][None,:],config['footprint']['radius_m'])[0])
            start_known=bool(env.workspace_status(pose[:2][None,:],config['footprint']['radius_m'])[0])
            start_valid=start_known and start_clear>=pred['geometry']['effective_threshold_m']
            pred['start_status']='VALID_START' if start_valid else 'INHERITED_INVALID_START'
            pred['prospective_warning']=bool(start_valid and not pred['geometry']['clearance_valid'])
            pred['gate_crossings']=gates(pp,pt)
            pred['actual_gate_history_before_issue']=[x for x in crossings if x.get('crossing_time_s') is not None and x['crossing_time_s']<t]
            prefix['saved_prediction_endpoint']=pp[1].tolist()
            prefix['model_discrepancy']=dict(prediction_vs_applied_euler=_pose_error(pp[1],euler),
                prediction_vs_exact=_pose_error(pp[1],exact),exact_vs_saved_actual=_pose_error(exact,actual),
                euler_vs_exact=_pose_error(euler,exact),
                clipping_and_solver_residual_not_separately_identified=True)
            prefix['prediction_exact_clearance_classification_differs']=bool(pred['first_interval']['clearance_valid']!=prefix_geometry['clearance_valid'])
            prefix['applied_euler_exact_clearance_classification_differs']=bool(prefix['applied_euler_geometry']['clearance_valid']!=prefix_geometry['clearance_valid'])
            future=[]
            for h in range(1,6):
                tt=float(pt[h]); comparable=tt<=rollout['horizon_s']+1e-12
                future.append(dict(offset_index=h,absolute_time_s=tt,actual_available=comparable,
                    actual_pose_world=at(tt).tolist() if comparable else None,
                    discrepancy=_pose_error(pp[h],at(tt)) if comparable else None,
                    interpretation='first applied interval' if h==1 else 'includes replanning; not pure integration/tracking error',
                    missing_reason=None if comparable else 'BEYOND_RECORDED_EXECUTION_HORIZON'))
            pred['future_actual_comparison']=future
            pred['tail_actual_comparison']=[]
            for h in range(1,5):
                aa,zz=float(pt[h]),float(pt[h+1]); available=zz<=rollout['horizon_s']+1e-12
                ag=None
                if available:
                    qt=_prefix_times(outcome,aa,min(zz,rollout['horizon_s'])); ag=geom([at(x) for x in qt],qt,bound,at)
                pred['tail_actual_comparison'].append(dict(prediction_segment_index=h,
                    prediction_clearance_valid=pred['geometry']['segments'][h]['clearance_valid'],
                    actual_available=available, actual_clearance_valid=None if ag is None else ag['clearance_valid'],
                    actual_minimum_clearance_m=None if ag is None else ag['minimum_clearance_m'],
                    interpretation='subsequent recorded replanning, not execution of this predicted tail'))
        else:
            pred.update(geometry=None,first_interval=None,tail=None,prospective_warning=None,start_status=None,
                        gate_crossings=[],future_actual_comparison=[],tail_actual_comparison=[])
            prefix.update(model_discrepancy=None,prediction_exact_clearance_classification_differs=None)
        ticks=[x for x in rollout['commands'] if x['solve_index']==k]
        timing_audit=dict(issue_time_matches_schedule=abs(t-k*DT)<=1e-12,
            input_equals_saved_control_tick=bool(np.allclose(pose,rollout['states'][r['tick']]['pose_world'],atol=1e-10,rtol=0)),
            six_held_ticks=len(ticks)==6,
            applied_command_matches_all_held_ticks=all(x['command']==r['command'] for x in ticks),
            controller_memory_matches_previous_record=r['previous_control']==(rollout['initial_previous_control'] if k==0 else selections[k-1]['previous_control_after']),
            exact_prefix_matches_next_tick=_pose_error(exact,actual)['xy_m']<=1e-10 and _pose_error(exact,actual)['yaw_rad']<=1e-10)
        records.append(dict(solve_index=k,issue_time_s=t,input_pose_world=pose.tolist(),timing_audit=timing_audit,
            actual_command=r['command'],selected_reference=selected,prediction=pred,applied_prefix=prefix,
            evaluation_completeness='COMPLETE_SAVED_WORLD_PREDICTION' if pred['available'] else 'MISSING_PREDICTION'))
    return dict(original_outcome=outcome,original_outcome_field_matches=matches,
        original_outcome_reproduced=all(matches.values()),full_execution=full,gate_crossings=crossings,per_solve=records,
        stored_counts=dict(states=len(rollout['states']),integration_commands=len(rollout['commands']),control_solves=len(selections)),
        reconstruction=dict(max_xy_error_m=max(x['xy_m'] for x in reconstruction),
            max_periodic_yaw_error_rad=max(x['yaw_rad'] for x in reconstruction)),
        stored_initial_physical_command=rollout['initial_physical_command'],
        stored_initial_previous_control=rollout['initial_previous_control'])


def assemble_audit(methods):
    """Derive timelines and representative choices from all saved records."""
    timeline=[]; summary={}; b,c=[methods[m] for m in METHODS]
    for method,result in methods.items():
        def event(name,issue=None,when=None,bracket=None,layer='',interpretation=''):
            timeline.append(dict(method=method,event=name,issue_time_s=issue,event_time_s=when,
                                 event_time_bracket_s=bracket,layer=layer,interpretation=interpretation))
        predictions=[r for r in result['per_solve'] if r['prediction']['available']]
        warnings=[r for r in predictions if r['prediction']['prospective_warning']]
        first_intervals=[r for r in predictions if not r['prediction']['first_interval']['clearance_valid']]
        first=warnings[0] if warnings else None
        if first:
            g=first['prediction']['geometry']; br=g['first_refined_violation_bracket_s'] or g['first_swept_violation_bracket_s']
            event('FIRST_PROSPECTIVE_PREDICTION_CLEARANCE_WARNING',first['issue_time_s'],br[1],br,'L2',
                  'issue time differs from forecast; retrospective audit, no online safety monitor')
        if first_intervals:
            r=first_intervals[0];g=r['prediction']['first_interval'];br=g['first_refined_violation_bracket_s'] or g['first_swept_violation_bracket_s']
            event('FIRST_PREDICTED_FIRST_INTERVAL_VIOLATION',r['issue_time_s'],br[1],br,'L2',r['prediction']['start_status'])
        full=result['full_execution']; br=full['first_refined_violation_bracket_s'] or full['first_swept_violation_bracket_s']
        event('FIRST_ACTUAL_CLEARANCE_VIOLATION',when=None if br is None else br[1],bracket=br,layer='L3')
        event('ACTUAL_MINIMUM_CLEARANCE',when=full['minimum_time_s'],layer='L3',interpretation='closest location on original checked polyline')
        invalid=[r for r in result['gate_crossings'] if r['classification']=='INVALID_INTERVAL_CROSSING']
        if invalid:
            x=invalid[0];event('FIRST_INVALID_GATE_CROSSING',when=x['crossing_time_s'],bracket=x.get('refined_bracket_s') or x['bracket_s'],layer='L3')
        summary[method]=dict(original_success=result['original_outcome']['primary_success'],
            original_failure_reasons=result['original_outcome']['failure_reasons'],
            original_outcome_reproduced=result['original_outcome_reproduced'],
            selected_point_invalid_count=sum(not point['clearance_valid'] for r in result['per_solve'] for point in r['selected_reference']['points']),
            selected_target_polyline_invalid_count=sum(not r['selected_reference']['target_polyline']['clearance_valid'] for r in result['per_solve']),
            entry_connector_invalid_count=sum(not r['selected_reference']['entry_connector']['clearance_valid'] for r in result['per_solve']),
            prediction_records_available=len(predictions),prediction_clearance_invalid_count=sum(not r['prediction']['geometry']['clearance_valid'] for r in predictions),
            prospective_warning_count=len(warnings),inherited_invalid_start_count=sum(r['prediction']['start_status']=='INHERITED_INVALID_START' for r in predictions),
            first_prospective_warning_issue_s=None if first is None else first['issue_time_s'],
            first_prospective_warning_solve_index=None if first is None else first['solve_index'],
            first_prospective_prediction_event_bracket_s=None if first is None else first['prediction']['geometry']['first_refined_violation_bracket_s'],
            first_predicted_first_interval_violation_issue_s=None if not first_intervals else first_intervals[0]['issue_time_s'],
            first_actual_violation_bracket_s=br,minimum_clearance_m=full['minimum_clearance_m'],minimum_time_s=full['minimum_time_s'],
            physical_overlap=full['physical_overlap'],first_invalid_gate_crossing_s=None if not invalid else invalid[0]['crossing_time_s'],
            first_interval_classification_disagreements=sum(bool(r['applied_prefix']['prediction_exact_clearance_classification_differs']) for r in predictions),
            max_prediction_vs_euler_xy_m=max((r['applied_prefix']['model_discrepancy']['prediction_vs_applied_euler']['xy_m'] for r in predictions),default=None),
            max_prediction_vs_exact_xy_m=max((r['applied_prefix']['model_discrepancy']['prediction_vs_exact']['xy_m'] for r in predictions),default=None),
            future_nodes_without_actual_counterpart=sum(not x['actual_available'] for r in predictions for x in r['prediction']['future_actual_comparison']),
            warning_tail_segments_actual_safe=sum(not x['prediction_clearance_valid'] and x['actual_clearance_valid'] is True for r in predictions for x in r['prediction']['tail_actual_comparison']))
    target_diff=command_diff=None
    for rb,rc in zip(b['per_solve'],c['per_solve']):
        if target_diff is None and not np.allclose(rb['selected_reference']['poses_world'],rc['selected_reference']['poses_world'],atol=1e-8,rtol=0):target_diff=rb['issue_time_s']
        if command_diff is None and not np.allclose(rb['actual_command'],rc['actual_command'],atol=1e-6,rtol=0):command_diff=rb['issue_time_s']
    for name,t,layer in [('FIRST_SELECTED_TARGET_DIFFERENCE',target_diff,'L1'),('FIRST_ACTUAL_COMMAND_DIFFERENCE',command_diff,'L3')]:
        timeline.append(dict(method='B_vs_C',event=name,issue_time_s=t,event_time_s=t,event_time_bracket_s=None,layer=layer,
            interpretation='actual records: same input only at t=0; later closed-loop states differ'))
    cs=summary[METHODS[1]]; choices=[('t=0',0),('C first prospective warning',cs['first_prospective_warning_solve_index'])]
    for label,t in [('C first actual violation',None if cs['first_actual_violation_bracket_s'] is None else cs['first_actual_violation_bracket_s'][0]),
                    ('C minimum clearance',cs['minimum_time_s']),('C invalid gate crossing',cs['first_invalid_gate_crossing_s'])]:
        choices.append((label,None if t is None else min(29,int(np.floor((t+1e-12)/DT)))))
    reps=[];missing=[]
    for label,i in choices:
        if i is None:missing.append(dict(reason=label,status='N/A'));continue
        row=next((x for x in reps if x['solve_index']==i),None)
        if row is None:row=dict(solve_index=i,reasons=[]);reps.append(row)
        row['reasons'].append(label)
    complete=all(x['prediction_records_available']==30 and x['original_outcome_reproduced'] for x in summary.values())
    return dict(task='GP-SE2-REF-04',case_id=CASE_ID,methods=methods,timeline=timeline,
        representative_intervals=reps,representative_unavailable=missing,
        summary=dict(methods=summary,new_VLA_inferences=0,new_GP_or_rigid_solves=0,new_MPC_solves=0,new_rollouts=0,
            first_target_difference_time_s=target_diff,first_command_difference_time_s=command_diff,
            diagnosis_status='FAILURE_LAYER_LOCALIZED' if complete else 'PARTIALLY_LOCALIZED',
            operational_status='GP_SE2_REF_04_COMPLETED_WITH_LIMITATIONS',
            limits=PROTOCOL['model_limitations']))
