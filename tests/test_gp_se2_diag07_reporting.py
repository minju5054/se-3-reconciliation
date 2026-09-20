"""Reporting-only completion must never waive scientific or source failures."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from finalize_gp_se2_diag07_review import reporting_error_gate,finalize
from present_gp_se2_diag07 import scope_text

def initial():
    return dict(valid=False,errors=['artifact hash plot_console.log'],no_new_MPC=True,no_new_GP=True,no_new_VLA=True,no_new_rollout=True)

def test_only_exact_terminal_stdout_issue_allowed():
    reporting_error_gate(initial(),['plot_console.log'])
    d=initial();d['errors'].append('independent execution predicate ref_00')
    with pytest.raises(ValueError):reporting_error_gate(d,['plot_console.log'])
    with pytest.raises(ValueError):reporting_error_gate(initial(),['plot_console.log','rollouts/ref_00/rollout.json'])

def test_presentation_separates_status_and_never_waives_plan_failure():
    assert 'PLAN INVALID: LATERAL ONLY' in scope_text(dict(hard=True))
    assert 'NOT A DEPLOYMENT CANDIDATE' in scope_text(dict(hard=True))
    assert 'BENIGN OPTIMIZED CONTROL' in scope_text(dict(hard=False))

def test_completion_refuses_existing_verification(tmp_path):
    (tmp_path/'verification').mkdir()
    with pytest.raises(FileExistsError):finalize(tmp_path)
