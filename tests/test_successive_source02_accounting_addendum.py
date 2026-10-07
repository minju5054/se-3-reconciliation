"""Only recorded reply accounting; no controller calls or simulated motion."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from validate_successive_source02_accounting_addendum import complete_accounting

def messages(s,status='command'):
    return [dict(status='submitted',solve_id=s),dict(type='solve_result',status=status,solve_id=s)]

def test_late_submit_and_result_counted_but_never_applied():
    e=messages('a')+messages('b','stale_rejected')
    tail=dict(messages=messages('c'),physically_applied=False,simulation_advanced=False)
    r=complete_accounting(e,tail,[dict(reason='new_solve',solve_id='a')])
    assert (r['accepted_in_loop'],r['accepted_in_shutdown_drain'],r['accepted_total'],r['solved_total'])==(2,1,3,3)
    assert r['successful_total']==2 and r['stale_rejected_total']==1 and r['physical_new_applications']==1

def test_historical_no_tail_counts_unchanged():
    r=complete_accounting(messages('a'),dict(messages=[],physically_applied=False,simulation_advanced=False),[])
    assert r['accepted_total']==r['accepted_in_loop']==1 and r['accepted_in_shutdown_drain']==0

@pytest.mark.parametrize('defect',['duplicate','missing','late_applied','advanced'])
def test_count_correction_cannot_hide_execution_or_missing_result(defect):
    e=messages('a');tail=dict(messages=messages('b'),physically_applied=False,simulation_advanced=False);commands=[]
    if defect=='duplicate':tail['messages']+=messages('a')
    if defect=='missing':tail['messages'].pop()
    if defect=='late_applied':commands=[dict(reason='new_solve',solve_id='b')]
    if defect=='advanced':tail['simulation_advanced']=True
    with pytest.raises(AssertionError):complete_accounting(e,tail,commands)
