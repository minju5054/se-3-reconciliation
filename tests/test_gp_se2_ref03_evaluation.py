"""REF-03 aggregation fixtures are tests, not actual transfer evidence."""
from copy import deepcopy
import itertools

import numpy as np
import pytest

from reconciliation import gp_se2_ref03_evaluation as evaluation
from reconciliation.gp_se2_evaluation import evaluate_rollout
from test_gp_se2_ref01_rollout import fixture, make_rollout
from test_gp_se2_evaluation import environment, route

METHODS=evaluation.METHODS


def metadata(index=0, *, episode=None, pair=None, stratum=None):
    episode=episode or f'episode_{index+100:03d}_repeat_00'
    return dict(case_id=f'{episode}/handoff_{index:03d}',episode_id=episode,
        ordered_raw_pair=pair or f'old{index}:fresh{index}',cohort='ADDITIONAL_TRANSFER',
        case_role='ADDITIONAL_TRANSFER',stratum=stratum or evaluation.STRATA[0])


def control(index=0):
    case_id,role=list(evaluation.CONTROL_CASES.items())[index]
    return dict(case_id=case_id,episode_id=case_id.split('/')[0],ordered_raw_pair='control:pair',
        cohort='KNOWN_CONTROLS',case_role=role,stratum=None)


def outcome(meta,method,success,**extra):
    row=dict(case_id=meta['case_id'],method=method,attempted=True,completed=True,
        primary_success=success,failure_reasons=[] if success else ['GOAL_DWELL_FAILURE'])
    row.update({metric:1. for metric in evaluation.CONTRAST_METRICS})
    row.update(extra)
    return row


def trio(meta,flags=(True,True,True)):
    return [outcome(meta,m,s) for m,s in zip(METHODS,flags)]


def test_original_predicates_gate_route_and_source_memory_preserved(fixture):
    context,native,_,_,config=fixture
    rollout,_=make_rollout(fixture)
    frozen=deepcopy(rollout)
    gate=dict(gate_id='synthetic_gate',center_xy=[.3,0.],normal_xy=[1.,0.],half_width_m=.4)
    goal_route=route(native[-1]);goal_route.update(route_status='REQUIRED_ORDERED_GATES',gates=[gate])
    original=evaluate_rollout(rollout,goal_route,environment(),config)
    result=evaluation.evaluate_method(METHODS[0],rollout,context,native,goal_route,environment(),config)
    assert rollout==frozen
    for key in ('primary_success','execution','environment','route','minimum_clearance_m'):
        assert result[key]==original[key]
    assert result['summary']['original_gate_definitions']==[gate]
    assert result['summary']['required_gate_count']==1
    assert result['summary']['completed'] and result['original_goal_route_preserved']
    assert result['summary']['goal_position_pass']==(original['execution']['terminal_position_error_m']<=original['execution']['goal_position_tolerance_m'])
    assert result['summary']['goal_yaw_pass']==(original['execution']['terminal_yaw_error_rad']<=original['execution']['goal_yaw_tolerance_rad'])
    assert result['summary']['controller_validity_pass']==(original['execution']['controller_failure_count']==0)
    memory=result['memory_diagnostic']
    assert memory['physical_u_minus']==context['u_minus']
    assert memory['controller_previous_control']==context['previous_control']
    assert memory['discrepancy_present']
    np.testing.assert_allclose(memory['first_acceleration_from_physical'],[1.,.3])
    np.testing.assert_allclose(memory['first_acceleration_from_controller_memory'],[-1.,1.5])
    definitions=result['violation_diagnostic_definitions']
    assert definitions['effective_clearance_threshold_m']==pytest.approx(
        .05+original['environment']['curved_path_error_bound_m']+original['environment']['geometry_uncertainty_m'])
    goal_route['gates'][0]['center_xy']=[4.,0.]
    failed=evaluation.evaluate_method(METHODS[0],rollout,context,native,goal_route,environment(),config)
    assert not failed['route']['valid']
    assert 'ROUTE_VIOLATION' in failed['summary']['failure_reasons']


def test_memory_discrepancy_diagnostic_retains_physical_motion_predicate(fixture):
    context,native,_,_,config=fixture
    rollout,_=make_rollout(fixture)
    # Direct diagnostic fixture isolates a memory jump; no rollout is claimed.
    rollout['initial_physical_command']=[0.,0.]
    rollout['initial_previous_control']=[.5,0.]
    rollout['controller_reference_selections'][0]['command']=[.5,0.]
    result=evaluation._memory_diagnostic(rollout,config)
    assert result['first_original_motion_violation']
    assert not any(result['first_memory_relative_bound_excess'])
    assert result['source_discrepancy_associated_with_first_motion_violation']
    assert 'not proof of selector causality' in result['interpretation']


@pytest.mark.parametrize('attempted',[False,True])
def test_missing_computation_not_scientific_failure_or_zero(attempted):
    meta=metadata();rows=trio(meta)
    rows[2]=evaluation.missing_outcome(meta['case_id'],METHODS[2],attempted=attempted,
        reason='SYNTHETIC_TECHNICAL_ERROR',case_metadata=meta)
    result=evaluation.aggregate_outcomes(rows,[meta]);summary=result['additional_transfer']
    stats=summary['method_outcomes'][METHODS[2]]
    assert stats['completed']==stats['failures']==stats['successes']==0
    assert stats['attempted']==int(attempted) and stats['missing']==1
    assert stats['event_equal']['value'] is None
    assert summary['exclusive_patterns']['UNAVAILABLE']==1
    assert result['event_classifications'][0]['overlapping_flags']['B_success_C_failure'] is None
    assert all(row['difference'] is None for row in result['paired_quality'] if row['method']==METHODS[2])
    assert result['ledger'][2]['terminal_position_error_m'] is None
    assert result['ledger'][2]['goal_position_pass'] is result['ledger'][2]['goal_yaw_pass'] is result['ledger'][2]['controller_validity_pass'] is None


def test_all_eight_exclusive_patterns_and_overlapping_recovery_counts():
    manifests=[];rows=[]
    for index,flags in enumerate(itertools.product((False,True),repeat=3)):
        meta=metadata(index);manifests.append(meta);rows.extend(trio(meta,flags))
    result=evaluation.aggregate_outcomes(rows,manifests)['additional_transfer']
    assert result['exclusive_patterns']=={**{p:1 for p in evaluation.PATTERNS},'UNAVAILABLE':0}
    expected={'B_success_C_failure':2,'A_success_C_failure':2,'A_success_B_failure_C_success':1,
        'B_failure_C_success':2,'A_B_failure_C_success':1,'all_failure':1}
    assert {k:v['count'] for k,v in result['overlapping_flags'].items()}==expected
    assert sum(expected.values())>result['event_count']
    assert result['overlapping_counts_are_not_disjoint']


def test_new_safety_route_motion_reasons_are_found_even_when_baseline_already_failed():
    meta=metadata();rows=trio(meta,(False,False,False))
    rows[1].update(failure_reasons=['GOAL_DWELL_FAILURE'],violation_magnitudes={'nominal_required_clearance_deficit_diagnostic_m':0.})
    rows[2].update(failure_reasons=['GOAL_DWELL_FAILURE','COLLISION','ROUTE_VIOLATION','MOTION_VIOLATION','GOAL_YAW_FAILURE'],
        violation_magnitudes={'nominal_required_clearance_deficit_diagnostic_m':.1})
    pair=evaluation.aggregate_outcomes(rows,[meta])['reason_comparisons'][0]
    assert not pair['baseline_success'] and not pair['method_success']
    assert pair['new_safety_reasons']==['COLLISION']
    assert pair['new_route_reasons']==['ROUTE_VIOLATION']
    assert pair['new_motion_reasons']==['MOTION_VIOLATION']
    assert pair['new_goal_reasons']==['GOAL_YAW_FAILURE']
    assert pair['violation_magnitude_differences']['nominal_required_clearance_deficit_diagnostic_m']==.1


def test_paired_quality_only_both_success_and_null_goal_time_preserved():
    meta=metadata();rows=trio(meta,(True,False,True))
    rows[0].update(time_to_goal_s=None,terminal_position_error_m=.02)
    rows[1].update(terminal_position_error_m=.01)
    rows[2].update(time_to_goal_s=2.,terminal_position_error_m=.015)
    result=evaluation.aggregate_outcomes(rows,[meta])
    bc=[r for r in result['paired_quality'] if r['baseline']==METHODS[1]]
    assert all(r['difference'] is None and r['baseline_value'] is None for r in bc)
    ac={r['metric']:r for r in result['paired_quality'] if r['baseline']==METHODS[0] and r['method']==METHODS[2]}
    assert ac['time_to_goal_s']['difference'] is None
    assert ac['terminal_position_error_m']['difference']==pytest.approx(-.005)
    assert len(result['failed_pair_endpoint_diagnostics'])==2
    assert 'not execution-quality improvement' in result['failed_pair_endpoint_diagnostics'][0]['interpretation']


def test_controls_strata_duplicates_and_weighted_quality_are_separate():
    metas=[metadata(0,episode='episode_100_repeat_00',pair=['x','y']),
           metadata(1,episode='episode_100_repeat_00',pair='x:y'),
           metadata(2,episode='episode_101_repeat_00',pair='p:q'),control()]
    rows=[]
    for index,meta in enumerate(metas):
        current=trio(meta,(True,True,index!=2))
        current[1]['terminal_position_error_m']=1.
        current[2]['terminal_position_error_m']=1.+index
        rows.extend(current)
    result=evaluation.aggregate_outcomes(rows,metas)
    transfer=result['additional_transfer'];stats=transfer['method_outcomes'][METHODS[2]]
    assert result['known_controls']['event_count']==1 and transfer['event_count']==3
    assert stats['event_equal']['value']==pytest.approx(2/3)
    assert stats['episode_equal']['value']==.5 and stats['ordered_raw_pair_equal']['value']==.5
    duplicates=transfer['duplicate_summary']
    assert duplicates['ordered_raw_pair']['memberships']['x:y']==[metas[0]['case_id'],metas[1]['case_id']]
    assert duplicates['episode_id']['maximum_degree']==2
    assert result['transfer_strata'][evaluation.STRATA[1]]['method_outcomes'][METHODS[0]]['event_equal']['value'] is None
    quality=next(r for r in transfer['paired_quality_equal_weight'] if r['baseline']==METHODS[1] and r['metric']=='terminal_position_error_m')
    assert quality['eligible_both_success_events']==2
    assert quality['event_equal']['value']==.5
    assert quality['episode_equal']['value']==.5
    assert quality['episode_equal']['groups_without_observed_outcome']==1
    assert evaluation.canonical_raw_pair(['x','y'])!=evaluation.canonical_raw_pair(['y','x'])


def test_equal_weight_quality_gives_each_episode_one_weight():
    metas=[metadata(0,episode='episode_100_repeat_00'),metadata(1,episode='episode_100_repeat_00'),metadata(2)]
    rows=[]
    for meta,difference in zip(metas,[0.,2.,10.]):
        group=trio(meta);group[2]['terminal_position_error_m']=1.+difference;rows.extend(group)
    quality=evaluation.aggregate_outcomes(rows,metas)['additional_transfer']['paired_quality_equal_weight']
    metric=next(r for r in quality if r['baseline']==METHODS[1] and r['metric']=='terminal_position_error_m')
    assert metric['event_equal']['value']==4.
    assert metric['episode_equal']['value']==5.5


def test_coverage_and_source_grouping_rejects_omitted_or_inconsistent_records():
    meta=metadata();rows=trio(meta)
    with pytest.raises(ValueError,match='every planned'):evaluation.aggregate_outcomes(rows[:-1],[meta])
    with pytest.raises(ValueError,match='every planned'):evaluation.aggregate_outcomes(rows+[rows[0]],[meta])
    with pytest.raises(ValueError,match='duplicate manifest'):evaluation.aggregate_outcomes(rows,[meta,meta])
    bad=deepcopy(rows);bad[2].update(completed=False,primary_success=False)
    with pytest.raises(ValueError,match='must remain null'):evaluation.aggregate_outcomes(bad,[meta])
    badmeta={**meta,'episode_id':'wrong'}
    with pytest.raises(ValueError,match='episode grouping'):evaluation.case_metadata(badmeta)
    with pytest.raises(ValueError,match='raw-pair'):evaluation.canonical_raw_pair(None)


def diagnostic(progress, *, goal=False):
    return dict(input_pose_world=[0.,0.,0.],nearest_index=0,nearest_original_fractional_row_coordinate=0.,
        weighted_total_pose_distances=[0.,1.,2.],reference_world=[[x,0.,0.] for x in progress],
        selected_original_fractional_row_coordinates=progress,indices=list(range(len(progress))),
        final_goal_row_in_horizon=goal,horizon_xy_arc_length_m=progress[-1]-progress[0],
        horizon_yaw_span_rad=0.,selected_last_target_distance_m=progress[-1],endpoint_repetition_count=0,
        nearest_cost_margin=.01,near_tied_nearest_indices=[0],q_h=progress,
        progress_overshoot=[.1]*len(progress),endpoint_clamped=[False]*len(progress))


def test_matched_state_exact_nearest_cost_identity_not_assumed_in_closed_loop():
    b=diagnostic([.1,.2,.3,.4,.5]);c=diagnostic([1.,2.,3.,4.,5.],goal=True)
    probe=dict(time_s=0.,input_pose_world=[0.,0.,0.],methods={METHODS[1]:b,METHODS[2]:c})
    result=evaluation.compare_selections('synthetic',[probe],matched=True)[0]
    assert result['same_all_nearest_costs'] and result['selection_changed']
    assert result['command_difference'] is None
    changed=deepcopy(probe);changed['methods'][METHODS[2]]['weighted_total_pose_distances'][1]+=.1
    with pytest.raises(ValueError,match='costs differ'):evaluation.compare_selections('synthetic',[changed],matched=True)
    changed=deepcopy(probe);changed['methods'][METHODS[2]]['input_pose_world'][0]=.1
    with pytest.raises(ValueError,match='same state'):evaluation.compare_selections('synthetic',[changed],matched=True)
    records={method:[dict(time_s=i/10,command=[0.,0.],input_pose_world=[0.,0.,0.],selection_diagnostic=deepcopy(b)) for i in range(30)] for method in METHODS}
    records[METHODS[2]][0]['selection_diagnostic']['weighted_total_pose_distances'][1]=100.
    assert evaluation.compare_selections('synthetic',records,matched=False)[0]['same_all_nearest_costs'] is False


def test_mechanism_retains_goal_loss_backtracking_ceil_and_near_ties():
    records=[dict(time_s=t,selection_diagnostic=diagnostic(p,goal=g)) for t,p,g in
        [(0.,[1.,2.,3.,4.,5.],False),(.1,[2.,3.,4.,5.,6.],True),(.2,[1.,2.,3.,4.,5.],False)]]
    records[1]['selection_diagnostic']['near_tied_nearest_indices']=[0,1]
    result=evaluation.selection_mechanism(records)
    assert result['final_target_first_in_horizon_time_s']==.1
    assert result['final_target_stays_after_first'] is False
    assert result['final_target_absent_after_first_count']==1
    assert result['backward_progress_count']==1
    assert result['ceil_overshoot_max']==.1 and result['near_tie_probe_count']==1
    assert result['minimum_nearest_cost_margin']==.01


def test_missing_A_keeps_available_BC_first_command_comparison():
    records=[dict(time_s=i/10,command=[0.,0.]) for i in range(30)]
    changed=deepcopy(records);changed[12]['command'][1]=.1
    results=evaluation.first_command_divergences('synthetic',{METHODS[0]:None,METHODS[1]:records,METHODS[2]:changed})
    assert results[0]['available'] and results[0]['first_divergent_command_time_s']==1.2
    assert results[0]['first_divergent_linear_command_time_s'] is None
    assert not results[1]['available'] and results[1]['differences'] is None


def test_control_reproduction_preserves_old_tolerances_and_stress_C_is_not_applicable(fixture):
    rollout,metrics=make_rollout(fixture)
    inputs={m:dict(rollout=rollout,metrics=metrics) for m in METHODS}
    report=evaluation.control_reproduction(control(0)['case_id'],inputs,inputs,source_kind='GP-SE2-REF-02')
    assert report['passed'] and len(report['methods'])==3
    assert report['tolerances']==evaluation.REPRODUCTION_TOLERANCES
    stress=evaluation.control_reproduction(control(2)['case_id'],inputs,inputs,source_kind='GP-SE2-01')
    assert stress['passed'] and stress['comparison_not_applicable']==[METHODS[2]]
    missing=evaluation.control_reproduction(control(2)['case_id'],{METHODS[0]:None},inputs,source_kind='GP-SE2-01')
    assert missing['methods'][METHODS[0]]['passed'] is None and not missing['passed']
    with pytest.raises(ValueError,match='wrong authoritative'):
        evaluation.control_reproduction(control(2)['case_id'],inputs,inputs,source_kind='GP-SE2-REF-02')
    changed=deepcopy(inputs);changed[METHODS[0]]['rollout']['states'][-1]['pose_world'][0]+=.001
    assert not evaluation.control_reproduction(control(0)['case_id'],changed,inputs,source_kind='GP-SE2-REF-02')['passed']
