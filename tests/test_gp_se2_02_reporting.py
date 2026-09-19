"""Synthetic reporting-only checks; no optimization or controller execution."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('gp02_reporting',ROOT/'scripts/report_gp_se2_02_results.py')
reporting=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(reporting)


def row(performed=True):
    execution=dict(motion_limits_pass=True,controller_validity_pass=True,terminal_goal_dwell_pass=False,
        terminal_position_error_m=.1,terminal_yaw_error_rad=.01,goal_position_tolerance_m=.15,goal_yaw_tolerance_rad=.26,
        mpc_official_solve_total_s=.03,mpc_solve_count_total=30)
    environment=dict(physical_overlap=False,clearance_valid=True,workspace_known=True)
    return dict(case_id='synthetic/not_evidence',case_role='SYNTHETIC',method='M0_ADAPTER',candidate_available=performed,
        plan_valid=False,rollout_performed=performed,rollout_success=False,
        rollout_metrics=dict(execution=execution if performed else None,environment=environment if performed else None,
            minimum_clearance_m=1. if performed else None,diagnostic_rollout_plan_invalid=performed,
            status_reasons=['PLAN_INVALID','GOAL_DWELL_FAILURE'] if performed else ['NO_CANDIDATE'],
            termination_reasons=['PLAN_INVALID','GOAL_DWELL_FAILURE'] if performed else ['NO_CANDIDATE']))


def test_expanded_matrix_preserves_unavailable_as_null_not_zero():
    result=reporting.outcome_matrix([row(False)])[0]
    for key in ('collision','clearance_valid','known_workspace','motion_valid','controller_valid','goal_position_valid',
                'goal_yaw_valid','goal_dwell_valid','terminal_position_error_m','terminal_yaw_error_rad','minimum_clearance_m'):
        assert result[key] is None
    assert not result['rollout_performed'] and not result['candidate_available']


def test_expanded_matrix_keeps_endpoint_dwell_and_plan_execution_separate():
    result=reporting.outcome_matrix([row()])[0]
    assert result['goal_position_valid'] and result['goal_yaw_valid']
    assert not result['goal_dwell_valid'] and not result['rollout_success']
    assert result['diagnostic_rollout_plan_invalid'] and result['rollout_performed'] and not result['plan_valid']


def test_matrix_matches_independent_numeric_goal_threshold_mapping():
    import validate_gp_se2_02 as validator
    values=[row(),row(False)]
    assert reporting.outcome_matrix(values)==validator.expanded_outcomes(values)
    changed=deepcopy(values);changed[0]['rollout_metrics']['execution']['terminal_position_error_m']=.2
    assert reporting.outcome_matrix(changed)!=validator.expanded_outcomes(changed)
    changed[0]['rollout_metrics']['status_reasons'].append('GOAL_POSITION_FAILURE')
    assert reporting.outcome_matrix(changed)==validator.expanded_outcomes(changed)


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))


def test_compute_cost_includes_every_start_and_does_not_add_nested_mpc(tmp_path):
    fields=('case_loading_seed_preparation_s','derivative_graph_construction_s','compilation_first_call_warmup_s',
        'solve_wall_time_s','post_solve_validation_time_s','independent_full_validation_time_s','cold_setup_solve_validation_s')
    starts=[]
    for i in range(16):
        method='M2_GP_NO_OBSTACLE' if i%2==0 else 'M3_GP_CONSTRAINED'
        starts.append(dict(method=method,**{k:i+1. for k in fields}))
        result=dict(setup_wall_time_s=.01,iterations=2,objective_evaluations=3,equality_evaluations=4,
            inequality_evaluations=5,callback_count=2,derivative_calls=dict(objective_gradient=2,equality_jacobian=2,inequality_jacobian=2),
            profiling=dict(evaluator_cache_misses=3,numerical_component_calls=dict(environment_query=3,collocation_gp_interpolation=3)),
            derivative_provider_stats=dict(phases=dict(runtime=dict(environment_calls=2,core_ad_evaluations=2))))
        write(tmp_path/'cases'/str(i)/'methods'/method/'starts/I0_FRESH/solver_result.json',result)
    timings=[dict(optimizer_and_checks_wall_s=20.,final_plan_validation_wall_s=.1)]
    write(tmp_path/'optimization_completed.json',dict(starts=starts,shared_environment_loading_s=.1,method_timings=timings,offline_optimization_phase_wall_s=100.))
    write(tmp_path/'case_manifest.json',dict(selected=[dict(preparation_timing=dict(initial_full_checks_s=2.,seed_construction_s=.2)) for _ in range(4)]))
    write(tmp_path/'source.json',dict(shared_preparation_environment_loading_s=.1))
    write(tmp_path/'aggregate/summary.json',dict(shared_evaluation_environment_loading_s=.1,evaluation_phase_wall_s=3.))
    write(tmp_path/'mpc_batch_timing.json',dict(wall_seconds=2.))
    rows=[row()];write(tmp_path/'aggregate/method_results.json',rows)
    result=reporting.compute_summary(tmp_path,rows)
    assert result['gp_all_16_starts']['solve_wall_time_s']==136.
    assert result['gp_all_16_starts']['harness_setup_s']==pytest.approx(.16)
    assert result['gp_evaluation_counts']['objective_evaluations']==48
    assert result['gp_evaluation_counts']['objective_gradient']==32
    assert result['initial_full_checks_s']==8. and result['seed_construction_s']==.8
    assert result['official_mpc_rollout_solve_s']==.03
    assert result['optimization_mpc_evaluation_phase_wall_sum_s']==105.  # .03 already nested in batch
    assert result['method_costs']==timings
