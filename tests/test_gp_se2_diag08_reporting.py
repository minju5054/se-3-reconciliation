"""No-runtime regression tests for the explicit report-only list-alias fix."""
import copy
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from finalize_gp_se2_diag08 import reporting_error_gate,nonmutating_admission
from test_gp_se2_diag08 import full


def test_only_exact_known_validator_error_is_admitted():
    report=dict(valid=False,errors=['no second-failure admission.plan_failure_reason']*4,
        original_full_checker_recomputed=True,GP_solves_during_validation=0,MPC_solves_during_validation=0,rollouts_during_validation=0)
    reporting_error_gate(report)
    for bad in [dict(report,errors=[]),dict(report,errors=report['errors']+['full mismatch']),dict(report,MPC_solves_during_validation=1)]:
        with pytest.raises(ValueError):reporting_error_gate(bad)


def test_supplemental_reason_does_not_mutate_primary_failure_list():
    f=full();f['additional_grid']['flags']['linear_speed']=False;before=copy.deepcopy(f)
    a=nonmutating_admission(f,{'planning_endpoint_reserve_pass':False},[{'family':'linear_speed_lower'}],True)
    assert a['plan_failure_reason']==['planning_endpoint_reserve','lateral_velocity','linear_speed']
    assert a['ineligible_reasons']==a['plan_failure_reason']+['supplemental_motion_violation']
    assert not a['eligible_for_execution'] and not a['deployment_candidate'] and f==before
