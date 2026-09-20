"""SciPy/JAX-free G4 execution bridge for the pinned official MPC environment.

Additive technical import separation. Numerical G4 source and saved solve
results stay frozen. Labels/gates are checked for parity in research tests.
"""
from __future__ import annotations
from .gp_se2_diag07_lateral_execution import (
    REQUIRED_FLAGS,diagnostic_rollout,validate_rollout_records)


def execution_admission(full,reserve,*,hard):
    flags=full.get('additional_grid',{}).get('flags',{})
    if set(flags)!=REQUIRED_FLAGS:raise ValueError('missing original full checker coverage')
    failed=sorted(k for k,v in flags.items() if not v)
    motion_failed=sorted(k for k,v in full['original_plan_report']['motion']['violations'].items() if v)
    valid=bool(full['full_feasible'])
    lateral=(failed==['lateral_velocity'] and motion_failed==['lateral_velocity'] and not valid
        and full['original_dense_report']['maximum_inequality_violation']<=full['original_config']['formulation']['inequality_tolerance'])
    ok=reserve['planning_endpoint_reserve_pass'];eligible=bool(ok and (valid or hard and lateral))
    reasons=([] if ok else ['planning_endpoint_reserve'])+failed
    if not valid and not failed:reasons+=['original_dense_or_plan_failure']
    return dict(plan_valid=valid,planning_endpoint_reserve_pass=ok,G4_plan_valid=bool(valid and ok),
        deployment_candidate=bool(valid and ok),eligible_for_execution=eligible,
        plan_failure_reason=None if valid else 'lateral_velocity' if lateral else reasons,
        reference_kind='PLAN_VALID_CANDIDATE' if valid else 'LATERAL_ONLY_PLAN_INVALID_REFERENCE' if lateral else 'OTHER_PLAN_INVALID_REFERENCE',
        diagnostic_execution_authorized=eligible,diagnostic_execution_only=bool(eligible and not valid),
        ineligible_reasons=[] if eligible else reasons,
        label='PLAN INVALID: LATERAL ONLY / DIAGNOSTIC EXECUTION ONLY' if lateral else
            'PLAN VALID CANDIDATE / OFFLINE COUNTERFACTUAL' if valid else 'PLAN INVALID / NOT EXECUTED')


def execute_reference(module,frozen,reference,admission):
    if not admission['eligible_for_execution']:raise ValueError('ineligible G4 reference')
    result=diagnostic_rollout(module,frozen,reference,
        dict(admission,deployment_candidate=False,diagnostic_execution_only=True))
    result.update(admission);result['actual_deployment_performed']=False
    return result


def validate_execution_records(rollout,context,reference,admission):
    errors=[f'admission field {k}' for k,v in admission.items() if rollout.get(k)!=v]
    if not admission['eligible_for_execution']:errors.append('ineligible execution')
    if not admission['plan_valid'] and (admission['deployment_candidate'] or not admission['diagnostic_execution_only']):errors.append('invalid plan promoted')
    return errors+validate_rollout_records(dict(rollout,deployment_candidate=False,diagnostic_execution_only=True),context,reference)
