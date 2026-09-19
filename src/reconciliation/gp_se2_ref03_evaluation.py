"""REF-03 frozen-selector transfer outcomes; unchanged physical/route predicates.

Unavailable computation is not a failed rollout. Controls and transfer strata
remain separate; duplicate-aware summaries are descriptive, not population tests.
"""
from __future__ import annotations
from copy import deepcopy
import numpy as np

from .gp_se2_ref02_evaluation import (
    METHODS, evaluate_method as _evaluate_method, selection_mechanism as _selection_mechanism,
    compare_selections as _compare_selections,
    reproduction_check, REPRODUCTION_TOLERANCES, CONTRAST_METRICS,
)
from .se2 import wrap_angle

CONTROL_CASES = {
    'episode_014_repeat_01/handoff_026': 'REPRO_LARGE_TURN',
    'episode_001_repeat_01/handoff_002': 'REPRO_BENIGN',
    'episode_017_repeat_00/handoff_007': 'KNOWN_OBSTACLE_STRESS',
}
STRATA = ('O_OBSTACLE_NEAR', 'R_LARGE_ROTATION', 'P_LARGE_MISMATCH', 'S_BENIGN_STRAIGHT')
COHORTS = ('KNOWN_CONTROLS', 'ADDITIONAL_TRANSFER')
PAIRS = ((METHODS[1], METHODS[2]), (METHODS[0], METHODS[2]), (METHODS[0], METHODS[1]))
PATTERNS = tuple(f'{i:03b}' for i in range(8))
OVERLAPPING_FLAGS = ('B_success_C_failure', 'A_success_C_failure', 'A_success_B_failure_C_success',
                     'B_failure_C_success', 'A_B_failure_C_success', 'all_failure')
SAFETY_REASONS = {'COLLISION', 'CLEARANCE_VIOLATION', 'UNKNOWN_WORKSPACE', 'CONTROLLER_FAILURE'}
MOTION_REASONS = {'MOTION_VIOLATION'}
ROUTE_REASONS = {'ROUTE_VIOLATION', 'UNKNOWN_ROUTE'}
GOAL_REASONS = {'GOAL_POSITION_FAILURE', 'GOAL_YAW_FAILURE', 'GOAL_DWELL_FAILURE'}
ENDPOINT_DIAGNOSTIC_METRICS = ('terminal_position_error_m', 'terminal_yaw_error_rad',
                             'terminal_signed_yaw_error_rad', 'minimum_clearance_m',
                             'terminal_achieved_goal_dwell_sampled_s')


def canonical_raw_pair(value):
    """Preserve OLD/FRESH order; list/tuple and the original colon form agree."""
    if isinstance(value, (list, tuple)):
        if len(value) != 2 or not all(isinstance(x, str) and x for x in value):
            raise ValueError('ordered raw pair must contain exactly two source identities')
        return ':'.join(value)
    if isinstance(value, str) and value:
        return value
    if isinstance(value, dict) and set(value) == {'old', 'fresh'}:
        return canonical_raw_pair([value['old'], value['fresh']])
    raise ValueError('explicit original ordered raw-pair identity required')


def case_metadata(row):
    """Normalize source metadata before inspecting numerical outcomes."""
    cohort = row.get('cohort')
    if cohort is None:
        cohort = {'KNOWN_CONTROL': 'KNOWN_CONTROLS', 'ADDITIONAL_TRANSFER': 'ADDITIONAL_TRANSFER'}.get(row.get('cohort_role'))
    role = row.get('case_role', row.get('control_role'))
    stratum = row.get('stratum')
    if stratum is None and cohort == 'ADDITIONAL_TRANSFER':
        stratum = dict(zip(('O','R','P','S'), STRATA)).get(row.get('selected_group'))
    if cohort not in COHORTS or (cohort == 'ADDITIONAL_TRANSFER' and stratum not in STRATA):
        raise ValueError('explicit frozen cohort and stratum metadata required')
    if cohort == 'KNOWN_CONTROLS' and (row['case_id'] not in CONTROL_CASES or stratum is not None):
        raise ValueError('known controls must be the frozen three IDs, without transfer strata')
    if cohort == 'KNOWN_CONTROLS' and role != CONTROL_CASES[row['case_id']]:
        raise ValueError('control role differs from fixed source ID')
    episode = row.get('episode_id', row['case_id'].split('/')[0])
    if episode != row['case_id'].split('/')[0]:
        raise ValueError('episode grouping differs from source case ID')
    return dict(case_id=row['case_id'], episode_id=episode, cohort=cohort, case_role=role,
                stratum=stratum, ordered_raw_pair=canonical_raw_pair(row['ordered_raw_pair']))


def _violation_magnitudes(rollout, outcome, config):
    c = config['formulation']; execution = outcome['execution']
    commands = np.asarray([r['command'] for r in rollout['controller_reference_selections']])
    accelerations = np.diff(np.vstack([rollout['initial_physical_command'], commands]), axis=0)*rollout['control_hz']
    minimum = outcome['minimum_clearance_m']
    return dict(linear_speed_lower_excess_mps=float(max(0., -np.min(commands[:,0]))),
        linear_speed_upper_excess_mps=float(max(0., np.max(commands[:,0])-c['v_max'])),
        angular_speed_excess_radps=float(max(0., np.max(np.abs(commands[:,1]))-c['w_max'])),
        linear_acceleration_excess_mps2=float(max(0., np.max(np.abs(accelerations[:,0]))-c['a_v_max'])),
        angular_acceleration_excess_radps2=float(max(0., np.max(np.abs(accelerations[:,1]))-c['a_w_max'])),
        footprint_overlap_depth_diagnostic_m=None if minimum is None else float(max(0., -minimum)),
        nominal_required_clearance_deficit_diagnostic_m=None if minimum is None else float(max(0., config['footprint']['required_clearance_m']-minimum)),
        effective_clearance_deficit_diagnostic_m=None if minimum is None else float(max(0., outcome['environment']['required_clearance_m']+outcome['environment']['geometry_uncertainty_m']-minimum)),
        terminal_position_excess_m=float(max(0., execution['terminal_position_error_m']-execution['goal_position_tolerance_m'])),
        terminal_yaw_excess_rad=float(max(0., execution['terminal_yaw_error_rad']-execution['goal_yaw_tolerance_rad'])),
        terminal_sampled_dwell_deficit_s=float(max(0., execution['terminal_goal_dwell_s']-outcome['terminal_achieved_goal_dwell_sampled_s'])))


def _memory_diagnostic(rollout, config):
    physical, memory = [np.asarray(rollout[k],float) for k in ('initial_physical_command','initial_previous_control')]
    first = np.asarray(rollout['controller_reference_selections'][0]['command'])
    acceleration_physical = (first-physical)*rollout['control_hz']
    acceleration_memory = (first-memory)*rollout['control_hz']
    limits = np.array([config['formulation']['a_v_max'], config['formulation']['a_w_max']])
    failed = np.abs(acceleration_physical)>limits+1e-5  # exact original first-step motion threshold
    return dict(physical_u_minus=physical.tolist(), controller_previous_control=memory.tolist(),
        previous_control_minus_physical_u_minus=(memory-physical).tolist(),
        discrepancy_present=bool(np.any(np.abs(memory-physical)>1e-12)),
        first_applied_command=first.tolist(), first_delta_from_physical=(first-physical).tolist(),
        first_delta_from_controller_memory=(first-memory).tolist(),
        first_acceleration_from_physical=acceleration_physical.tolist(),
        first_acceleration_from_controller_memory=acceleration_memory.tolist(),
        first_original_motion_violation=bool(failed.any()), first_original_motion_violation_components=failed.tolist(),
        first_memory_relative_bound_excess=(np.abs(acceleration_memory)>limits+1e-5).tolist(),
        source_discrepancy_associated_with_first_motion_violation=bool(np.any(np.abs(memory-physical)>1e-12) and failed.any()),
        interpretation='source-associated discrepancy diagnostic; not proof of selector causality; original physical-motion predicate unchanged')


def evaluate_method(method, rollout, context, reference, goal_route, env, config, *, case_metadata=None):
    outcome = _evaluate_method(method, rollout, context, reference, goal_route, env, config)
    summary = outcome['summary']
    summary.update(attempted=True, completed=True, available=True, computation_status='COMPLETED', missing_reason=None,
        route_valid=outcome['route']['valid'], route_status=outcome['route'].get('status'),
        required_gate_count=len(goal_route['gates']), original_gate_definitions=deepcopy(goal_route['gates']),
        collision=outcome['environment']['physical_overlap'], clearance_valid=outcome['environment']['clearance_valid'],
        workspace_known=outcome['environment']['workspace_known'],
        goal_position_pass=outcome['execution']['terminal_position_error_m']<=outcome['execution']['goal_position_tolerance_m'],
        goal_yaw_pass=outcome['execution']['terminal_yaw_error_rad']<=outcome['execution']['goal_yaw_tolerance_rad'],
        controller_validity_pass=outcome['execution']['controller_failure_count']==0,
        violation_magnitudes=_violation_magnitudes(rollout,outcome,config),
        memory_diagnostic=_memory_diagnostic(rollout,config))
    if case_metadata is not None:
        meta = globals()['case_metadata'](case_metadata)
        if meta['case_id'] != context['case_id']:
            raise ValueError('case metadata and actual handoff differ')
        summary.update(meta)
    outcome.update(attempted=True, completed=True, computation_status='COMPLETED',
        original_goal_route_preserved=True, violation_magnitudes=summary['violation_magnitudes'],
        memory_diagnostic=summary['memory_diagnostic'],
        violation_diagnostic_definitions=dict(original_predicates_unchanged=True,
            nominal_required_clearance_m=config['footprint']['required_clearance_m'],
            curve_error_bound_m=outcome['environment']['curved_path_error_bound_m'],
            geometry_uncertainty_m=outcome['environment']['geometry_uncertainty_m'],
            effective_clearance_threshold_m=outcome['environment']['required_clearance_m']+outcome['environment']['geometry_uncertainty_m'],
            physical_overlap_threshold_m=-outcome['environment']['geometry_uncertainty_m'],
            note='nonnegative geometric margins are diagnostics; workspace and original swept checks remain independently necessary'))
    return outcome


def missing_outcome(case_id, method, *, attempted, reason, case_metadata=None):
    if method not in METHODS or not isinstance(attempted,bool) or not isinstance(reason,str) or not reason:
        raise ValueError('explicit method, attempt status and missing reason required')
    summary = {metric:None for metric in CONTRAST_METRICS}
    summary.update(case_id=case_id,method=method,attempted=attempted,completed=False,available=False,
        computation_status='ATTEMPTED_INCOMPLETE' if attempted else 'NOT_ATTEMPTED',missing_reason=reason,
        primary_success=None,rollout_success=None,terminal_goal_dwell_pass=None,minimum_clearance_m=None,
        motion_limits_pass=None,controller_failure_count=None,failure_reasons=None,route_valid=None,
        route_status=None,required_gate_count=None,collision=None,clearance_valid=None,workspace_known=None,
        goal_position_pass=None,goal_yaw_pass=None,controller_validity_pass=None,
        violation_magnitudes=None,memory_diagnostic=None)
    if case_metadata is not None:
        meta=globals()['case_metadata'](case_metadata)
        if meta['case_id']!=case_id:raise ValueError('missing case metadata mismatch')
        summary.update(meta)
    return dict(case_id=case_id,method=method,attempted=attempted,completed=False,
        computation_status=summary['computation_status'],primary_success=None,rollout_success=None,
        failure_reasons=None,execution=None,environment=None,route=None,summary=summary)


def _row(outcome):
    value=deepcopy(outcome.get('summary',outcome))
    if value.get('method') not in METHODS:
        raise ValueError('unknown method')
    for key in ('attempted','completed'):
        if not isinstance(value.get(key),bool):raise ValueError('explicit computation ledger required: '+key)
    if value['completed'] and (not value['attempted'] or not isinstance(value.get('primary_success'),bool)):
        raise ValueError('completed rollout needs attempt and boolean original outcome')
    if not value['completed'] and value.get('primary_success') is not None:
        raise ValueError('missing result must remain null, not a scientific failure')
    return value


def event_classification(by):
    """Overlapping conditional statements plus exactly one eight-pattern class."""
    success=[by[m]['primary_success'] if by[m]['completed'] else None for m in METHODS]
    a,b,c=success
    def logical(indices,predicate):
        return None if any(success[i] is None for i in indices) else bool(predicate())
    flags=dict(B_success_C_failure=logical([1,2],lambda:b and not c),
        A_success_C_failure=logical([0,2],lambda:a and not c),
        A_success_B_failure_C_success=logical([0,1,2],lambda:a and not b and c),
        B_failure_C_success=logical([1,2],lambda:not b and c),
        A_B_failure_C_success=logical([0,1,2],lambda:not a and not b and c),
        all_failure=logical([0,1,2],lambda:not a and not b and not c))
    return dict(success_by_method=dict(zip(METHODS,success)),
                exclusive_pattern='UNAVAILABLE' if None in success else ''.join('1' if x else '0' for x in success),
                overlapping_flags=flags,overlap_note='flags overlap intentionally; do not sum as distinct events')


def weighted_summary(rows, value_key, group_key):
    """Average observed event rates within groups, then weight groups equally."""
    groups={}
    for row in rows:groups.setdefault(row[group_key],[]).append(row.get(value_key))
    rates={key:None if all(v is None for v in values) else float(np.mean([v for v in values if v is not None])) for key,values in groups.items()}
    observed=[x for x in rates.values() if x is not None]
    return dict(value=None if not observed else float(np.mean(observed)),
        planned_groups=len(groups),observed_groups=len(observed),groups_without_observed_outcome=sum(v is None for v in rates.values()),
        group_values=rates,definition='equal weight per '+group_key+' after within-group mean of observed event outcomes; missing values excluded and counted')


def duplicate_summary(metadata):
    """Source-defined groups, including singleton membership and maximum degree."""
    result={}
    for key in ('episode_id','ordered_raw_pair'):
        groups={}
        for row in metadata.values():groups.setdefault(row[key],[]).append(row['case_id'])
        frequencies={identity:len(members) for identity,members in groups.items()}
        result[key]=dict(memberships=groups,frequencies=frequencies,group_count=len(groups),
            maximum_degree=max(frequencies.values(),default=0),
            duplicate_group_count=sum(n>1 for n in frequencies.values()),
            events_in_duplicate_groups=sum(n for n in frequencies.values() if n>1))
    return result


def _scope_summary(rows, events, metadata, quality):
    patterns={p:0 for p in (*PATTERNS,'UNAVAILABLE')}
    for event in events:patterns[event['exclusive_pattern']]+=1
    flags={key:dict(count=sum(e['overlapping_flags'][key] is True for e in events),
                    comparable_events=sum(e['overlapping_flags'][key] is not None for e in events),
                    unavailable_events=sum(e['overlapping_flags'][key] is None for e in events)) for key in OVERLAPPING_FLAGS}
    for key in OVERLAPPING_FLAGS:
        values=[dict(case_id=e['case_id'],episode_id=e['episode_id'],ordered_raw_pair=e['ordered_raw_pair'],
                     value=e['overlapping_flags'][key]) for e in events]
        flags[key]['equal_weight_summaries']={label:weighted_summary(values,'value',group) for label,group in
            (('event_equal','case_id'),('episode_equal','episode_id'),('ordered_raw_pair_equal','ordered_raw_pair'))}
    methods={}
    for name in METHODS:
        selected=[r for r in rows if r['method']==name]
        values=[dict(case_id=r['case_id'],episode_id=metadata[r['case_id']]['episode_id'],
                     ordered_raw_pair=metadata[r['case_id']]['ordered_raw_pair'],success=float(r['primary_success']) if r['completed'] else None) for r in selected]
        methods[name]=dict(planned=len(selected),attempted=sum(r['attempted'] for r in selected),completed=sum(r['completed'] for r in selected),
            missing=sum(not r['completed'] for r in selected),successes=sum(r['completed'] and r['primary_success'] for r in selected),
            failures=sum(r['completed'] and not r['primary_success'] for r in selected),
            event_equal=weighted_summary(values,'success','case_id'),episode_equal=weighted_summary(values,'success','episode_id'),
            ordered_raw_pair_equal=weighted_summary(values,'success','ordered_raw_pair'))
    paired=[]
    for baseline,method in PAIRS:
        for metric in CONTRAST_METRICS:
            selected=[r for r in quality if r['baseline']==baseline and r['method']==method and r['metric']==metric]
            paired.append(dict(baseline=baseline,method=method,metric=metric,
                eligible_both_success_events=sum(r['eligible_both_success'] for r in selected),
                event_equal=weighted_summary(selected,'difference','case_id'),
                episode_equal=weighted_summary(selected,'difference','episode_id'),
                ordered_raw_pair_equal=weighted_summary(selected,'difference','ordered_raw_pair')))
    return dict(event_count=len(events),method_outcomes=methods,exclusive_patterns=patterns,
        exclusive_pattern_order='A/B/C, 1=original success;0=completed failure;UNAVAILABLE=any incomplete method',
        overlapping_flags=flags,overlapping_counts_are_not_disjoint=True,
        duplicate_summary=duplicate_summary(metadata),paired_quality_equal_weight=paired,
        episode_count=len({m['episode_id'] for m in metadata.values()}),
        ordered_raw_pair_count=len({m['ordered_raw_pair'] for m in metadata.values()}),
        interpretation='fixed selected development-corpus events; no population frequency or significance claim')


def aggregate_outcomes(outcomes, manifest_rows):
    metadata={r['case_id']:case_metadata(r) for r in manifest_rows}
    if len(metadata)!=len(manifest_rows):raise ValueError('duplicate manifest event')
    rows=[_row(r) for r in outcomes];by={(r['case_id'],r['method']):r for r in rows}
    expected={(c,m) for c in metadata for m in METHODS}
    if len(by)!=len(rows) or set(by)!=expected:raise ValueError('every planned case/method requires exactly one outcome or explicit missing record')
    ledger=[];events=[];reason_pairs=[];quality=[];endpoint_diagnostics=[]
    for case_id,meta in metadata.items():
        methods={m:by[case_id,m] for m in METHODS}
        for row in methods.values():
            row.update(meta);ledger.append(row)
        events.append({**meta,**event_classification(methods)})
        for baseline,method in PAIRS:
            a,b=methods[baseline],methods[method];complete=a['completed'] and b['completed']
            new=[] if not complete else [reason for reason in b.get('failure_reasons',[]) if reason not in a.get('failure_reasons',[])]
            magnitudes={}
            if complete:
                for key,value in (b.get('violation_magnitudes') or {}).items():
                    old=(a.get('violation_magnitudes') or {}).get(key)
                    magnitudes[key]=None if value is None or old is None else float(value-old)
            reason_pairs.append(dict(**meta,baseline=baseline,method=method,comparable=complete,
                baseline_success=a['primary_success'],method_success=b['primary_success'],
                new_failure_reasons=new if complete else None,
                new_safety_reasons=[r for r in new if r in SAFETY_REASONS] if complete else None,
                new_motion_reasons=[r for r in new if r in MOTION_REASONS] if complete else None,
                new_route_reasons=[r for r in new if r in ROUTE_REASONS] if complete else None,
                new_goal_reasons=[r for r in new if r in GOAL_REASONS] if complete else None,
                violation_magnitude_differences=magnitudes if complete else None,
                safety_checked_even_if_baseline_failed=True))
            both_success=bool(complete and a['primary_success'] and b['primary_success'])
            for metric in CONTRAST_METRICS:
                av,bv=a.get(metric),b.get(metric)
                quality.append(dict(**meta,baseline=baseline,method=method,metric=metric,eligible_both_success=both_success,
                    baseline_value=av if both_success else None,method_value=bv if both_success else None,
                    difference=float(bv-av) if both_success and av is not None and bv is not None else None,
                    ineligible_reason=None if both_success else 'INCOMPLETE_PAIR' if not complete else 'PAIR_NOT_BOTH_SUCCESS'))
            if complete and not both_success:
                endpoint_diagnostics.append(dict(**meta,baseline=baseline,method=method,baseline_success=a['primary_success'],method_success=b['primary_success'],
                    metrics={k:dict(baseline=a.get(k),method=b.get(k),difference=None if a.get(k) is None or b.get(k) is None else float(b[k]-a[k])) for k in ENDPOINT_DIAGNOSTIC_METRICS},
                    interpretation='failed-pair endpoint diagnostic only; not execution-quality improvement'))
    def scope(predicate):
        selected={k:v for k,v in metadata.items() if predicate(v)}
        return _scope_summary([r for r in ledger if r['case_id'] in selected],[e for e in events if e['case_id'] in selected],selected,
                              [r for r in quality if r['case_id'] in selected])
    return dict(ledger=ledger,event_classifications=events,reason_comparisons=reason_pairs,paired_quality=quality,
        failed_pair_endpoint_diagnostics=endpoint_diagnostics,
        known_controls=scope(lambda x:x['cohort']=='KNOWN_CONTROLS'),
        known_control_by_role={role:scope(lambda x,r=role:x['cohort']=='KNOWN_CONTROLS' and x['case_role']==r) for role in CONTROL_CASES.values()},
        transfer_strata={stratum:scope(lambda x,s=stratum:x['cohort']=='ADDITIONAL_TRANSFER' and x['stratum']==s) for stratum in STRATA},
        additional_transfer=scope(lambda x:x['cohort']=='ADDITIONAL_TRANSFER'),
        method_count=len(rows),event_count=len(metadata),controls_not_pooled_with_additional=True,
        regression_reporting_order=['B_success_C_failure','A_success_C_failure','new safety/motion/route/goal reasons','recoveries'],
        regression_recovery_events={key:[e for e in events if e['overlapping_flags'][key] is True] for key in OVERLAPPING_FLAGS},
        missing_values_are_not_failures=True)


def selection_mechanism(records):
    result=_selection_mechanism(records)
    rows=[r.get('selection_diagnostic',r) for r in records]
    overshoot=[float(x) for r in rows for x in (r.get('progress_overshoot') or [])]
    margins=[r.get('nearest_cost_margin') for r in rows]
    result.update(near_tie_probe_count=sum(len(r.get('near_tied_nearest_indices',[]))>1 for r in rows),
        nearest_cost_margins=margins,minimum_nearest_cost_margin=min((v for v in margins if v is not None),default=None),
        ceil_overshoot_min=None if not overshoot else min(overshoot),ceil_overshoot_max=None if not overshoot else max(overshoot),
        ceil_overshoot_mean=None if not overshoot else float(np.mean(overshoot)),
        requested_q_h=[r.get('q_h') for r in rows],progress_overshoot=[r.get('progress_overshoot') for r in rows],
        endpoint_clamp_count=None if not overshoot else sum(sum(r.get('endpoint_clamped') or []) for r in rows),
        probe_or_control_count=len(rows),final_target_persistence_scope='observed selection calls only; not unobserved between-call poses')
    return result


def compare_selections(case_id, records, *, matched):
    if not matched:
        return _compare_selections(case_id,records,matched=False)
    result=[]
    for row in records:
        a,b=[row['methods'][name] for name in METHODS[1:]]
        if a['input_pose_world']!=b['input_pose_world'] or a['input_pose_world']!=row['input_pose_world']:
            raise ValueError('B/C matched selector probes must use exactly the same state')
        nearest=a['nearest_index']==b['nearest_index'] and a['nearest_original_fractional_row_coordinate']==b['nearest_original_fractional_row_coordinate']
        costs=a['weighted_total_pose_distances']==b['weighted_total_pose_distances']
        if not(nearest and costs):raise ValueError('matched B/C nearest or costs differ')
        delta=np.asarray(b['reference_world'])-a['reference_world'];delta[:,2]=wrap_angle(delta[:,2])
        progress=np.asarray(b['selected_original_fractional_row_coordinates'])-a['selected_original_fractional_row_coordinates']
        result.append(dict(case_id=case_id,baseline=METHODS[1],method=METHODS[2],time_s=row['time_s'],same_input_pose=True,
            same_nearest_index_and_source_progress=nearest,same_all_nearest_costs=costs,
            max_selected_xy_difference_m=float(np.max(np.linalg.norm(delta[:,:2],axis=1))),
            max_selected_yaw_difference_rad=float(np.max(np.abs(delta[:,2]))),selected_original_progress_difference=progress.tolist(),
            baseline_goal_in_horizon=a['final_goal_row_in_horizon'],method_goal_in_horizon=b['final_goal_row_in_horizon'],
            selected_indices_baseline=a['indices'],selected_indices_method=b['indices'],
            selection_changed=bool(np.max(np.abs(delta))>1e-12 or np.max(np.abs(progress))>1e-12),
            command_difference=None,input_pose_difference=None,interpretation='same-state selector intervention'))
    return result


def first_command_divergences(case_id, actual, *, atol=1e-6):
    # The fixed 30-solve interface remains original; no extrapolation of missing logs.
    result=[]
    for baseline,method in PAIRS[:2]:
        a,b=actual.get(baseline),actual.get(method)
        if a is None or b is None:
            result.append(dict(case_id=case_id,baseline=baseline,method=method,available=False,
                first_divergent_command_time_s=None,first_divergent_linear_command_time_s=None,
                first_divergent_angular_command_time_s=None,differences=None,atol=atol,reason='INCOMPLETE_PAIR'))
            continue
        if len(a)!=30 or len(b)!=30 or [r['time_s'] for r in a]!=[r['time_s'] for r in b]:
            raise ValueError('completed command comparison requires the same 30 original solve times')
        delta=np.asarray([r['command'] for r in b])-np.asarray([r['command'] for r in a])
        def first(mask):return next((float(r['time_s']) for r,flag in zip(a,mask) if flag),None)
        result.append(dict(case_id=case_id,baseline=baseline,method=method,available=True,atol=atol,
            first_divergent_command_time_s=first(np.any(np.abs(delta)>atol,axis=1)),
            first_divergent_linear_command_time_s=first(np.abs(delta[:,0])>atol),
            first_divergent_angular_command_time_s=first(np.abs(delta[:,1])>atol),differences=delta.tolist()))
    return result


def control_reproduction(case_id, actual, historical, *, source_kind):
    """actual/historical map method to {rollout, metrics}; no extra MPC solves."""
    if case_id not in CONTROL_CASES:raise ValueError('only frozen known controls have prescribed reproduction')
    expected=METHODS[:2] if case_id=='episode_017_repeat_00/handoff_007' else METHODS
    required_source='GP-SE2-01' if len(expected)==2 else 'GP-SE2-REF-02'
    if source_kind!=required_source:raise ValueError('wrong authoritative historical source for control')
    reports={}
    for method in expected:
        if actual.get(method) is None or historical.get(method) is None:
            reports[method]=dict(available=False,passed=None,reason='MISSING_PRESCRIBED_REPRODUCTION_INPUT')
        else:
            reports[method]=dict(available=True,**reproduction_check(actual[method]['rollout'],historical[method]['rollout'],
                actual[method]['metrics'],historical[method]['metrics']))
    return dict(case_id=case_id,source_kind=source_kind,methods=reports,
        passed=all(r['passed'] is True for r in reports.values()),
        comparison_not_applicable=[m for m in METHODS if m not in expected],
        tolerances=deepcopy(REPRODUCTION_TOLERANCES),new_mpc_solves=0)
