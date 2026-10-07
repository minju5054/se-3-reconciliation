from copy import deepcopy
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from reconciliation.successive_scheduler02 import capture_phase_audit
from audit_successive_scheduler02 import historical_revalidation


def row(tick=0, **kw):
    return dict(start_state_id=tick,simulation_start_s=tick/60,sim_steps=1,
        model_received=[],inference_in_flight_at_start=False,capture_happened=True,
        render_readback_s=.01,mpc_submitted=False,wire_request_kind=None,**kw)

def frame(tick=0):
    return dict(rendered_state_id=tick,capture_sim_time_s=tick/60,
        render_sim_time_before_s=tick/60,render_sim_time_after_s=tick/60)

def test_A_scheduled_capture_required():
    assert capture_phase_audit([row()],[frame()])['valid']
    assert not capture_phase_audit([row()],[])['valid']

@pytest.mark.parametrize('status',['SCENE_INVALID','RAW_UNSAFE','MODEL_STOP'])
def test_B_C_terminal_zero_step_before_capture(status):
    r=row();r.update(sim_steps=0,inference_in_flight_at_start=True,capture_happened=False,
        render_readback_s=None,model_received=[dict(type='result',kind='prediction',status=status,chunk_id='chunk_002')])
    assert capture_phase_audit([r],[])['valid']
    for field,value in [('sim_steps',1),('inference_in_flight_at_start',False),('mpc_submitted',True)]:
        bad=deepcopy(r);bad[field]=value
        assert not capture_phase_audit([bad],[])['valid']

def test_D_nonterminal_cadence():
    rows=[row(i) for i in range(31)]
    for r in rows:r['capture_happened']=r['start_state_id']%15==0
    assert capture_phase_audit(rows,[frame(i) for i in (0,15,30)])['valid']
    assert not capture_phase_audit(rows,[frame(i) for i in (0,30)])['valid']

@pytest.mark.parametrize('field',['rendered_state_id','capture_sim_time_s','render_sim_time_before_s','render_sim_time_after_s'])
def test_E_exact_state_and_time(field):
    f=frame();f[field]+=1
    assert not capture_phase_audit([row()],[f])['valid']

def test_saved_LONG01_old_result_unchanged():
    v=historical_revalidation()
    assert v['corrected_scheduler']['valid']
    assert v['historical_classification_unchanged']=='TECHNICAL_EXECUTION_BLOCKED'
    assert v['corrected_scheduler']['capture_phase_audit']['before_capture_terminal_exemptions'][0]['state_id']==120
