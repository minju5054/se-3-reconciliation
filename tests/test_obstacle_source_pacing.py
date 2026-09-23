"""Synthetic timing fixtures, not scientific evidence. No model/MPC calls."""
from pathlib import Path
import ast
import numpy as np
import pytest
import yaml
from reconciliation.online_pacing import NoCatchupPacer,make_pacer,POLICY
from reconciliation.robotless_online import AbsolutePacer,integrate_unicycle,CommandActivation
from reconciliation.obstacle_source_acquisition import timing_gate
ROOT=Path(__file__).resolve().parents[1]


def simulate(instrument=False,blocking=True):
    dt=float(np.float32(1/60));p=NoCatchupPacer(100.,0.);host=100.;sim=0.;pose=np.zeros(3);states=[];diag=[]
    for tick in range(180):
        before=host
        host+=.09 if blocking and tick%15==0 else .002
        next_sim=sim+dt;wait=max(0.,p.remaining(next_sim,host));host+=wait
        pose=integrate_unicycle(pose,[.8,.1],dt);sim=next_sim
        states.append((sim,host,pose.copy()));p.observe(sim,host)
        if instrument:diag.append(dict(tick=tick,wall_step=host-before,wait=wait))
    return states,diag


def test_no_catchup_after_blocking():
    rows,diag=simulate(True)
    dt=float(np.float32(1/60))
    assert all(x['wall_step']>=dt-1e-10 for x in diag)
    assert diag[0]['wait']==0
    assert diag[1]['wait']>0.014
    # The historical absolute policy would repay this blocking debt.
    old=AbsolutePacer(100.,0.)
    assert old.remaining(2*dt,100.092)<0


def test_wall_deadline_rebases_and_clocks_stay_distinct():
    p=NoCatchupPacer(10.,3.)
    assert p.deadline(3.1)==pytest.approx(10.1)
    p.observe(3.1,10.25)
    assert p.deadline(3.2)==pytest.approx(10.35)
    assert p.origin_host_s==10 and p.origin_sim_s==3
    with pytest.raises(ValueError):p.deadline(3.)
    with pytest.raises(ValueError):make_pacer('new_unknown',0.,0.,1.,.01)
    assert type(make_pacer('absolute',0.,0.,1.,.01)) is AbsolutePacer


def test_nominal_ticks_and_exact_held_execution_unchanged():
    rows,_=simulate()
    dt=float(np.float32(1/60))
    assert len(range(0,len(rows),15))==12  # 4 Hz on 3 s simulation grid
    assert len(range(0,len(rows),6))==30  # 10 Hz
    assert np.allclose(np.diff([0]+[r[0] for r in rows]),dt,rtol=0,atol=1e-14)
    np.testing.assert_allclose(rows[-1][2],integrate_unicycle(np.zeros(3),[.8,.1],180*dt),rtol=0,atol=1e-12)
    # OLD continues throughout an arbitrary 0.3s in-flight host interval.
    window=[r for r in rows if 101.1<=r[1]<=101.4]
    assert len(window)>2 and np.linalg.norm(window[-1][2][:2]-window[0][2][:2])>.02


def test_instrumentation_is_observational_only():
    a,_=simulate(False);b,d=simulate(True)
    assert len(d)==len(a)
    for x,y in zip(a,b):np.testing.assert_array_equal(x[2],y[2]);assert x[:2]==y[:2]


def test_unchanged_request_and_stall_gate():
    assert timing_gate([.8,1.,1.2],.25)['qualified']
    for rtf,stall in [(1.201,.2),(.799,.2),(1.,.25001)]:
        assert not timing_gate([rtf]*3,stall)['qualified']
    assert not timing_gate([1.,1.],.2)['qualified']
    assert not timing_gate([None]*3,.1)['qualified']


def test_frozen_source_no_fallback_and_phaseA():
    cfg=yaml.safe_load((ROOT/'configs/obstacle_source_acquisition_02.yaml').read_text())
    assert cfg['selected_candidate']=='POSE11'
    assert cfg['stage2']['repetitions']==['REPEAT_00','REPEAT_01']
    assert cfg['stage2']['initial_backwards_m']==.4
    assert cfg['stage1']['episodes']==1 and not cfg['stage1']['retry']
    for name in ('online_pacing.py',):
        tree=ast.parse((ROOT/'src/reconciliation'/name).read_text())
        names={x.id for x in ast.walk(tree) if isinstance(x,ast.Name)}
        assert not names.intersection({'solve_gp','solve_rigid','MpcTracker','minimize'})
    runner=(ROOT/'scripts/run_obstacle_source_acquisition02.py').read_text()
    assert 'POSE12' not in runner and 'obstacle_source_predict' not in runner


def test_collector_cadence_and_separation_retained():
    s=(ROOT/'scripts/isaac/robotless_online_handoffs.py').read_text()
    assert 'tick % capture_stride==0' in s and 'tick%control_stride==0' in s
    assert "pacing_policy=config['execution'].get('pacing_policy','absolute')" in s
    assert "context['t_install']=installed" in s and "event['t_switch']=applied_stamp" in s
    assert 'time.sleep(wait)' in s
    assert s.index("with (ep/'scheduler.jsonl').open('x')")>s.index('journal.close()')


def test_physical_command_and_memory_remain_distinct():
    from reconciliation.obstacle_source_acquisition import moving_boundary
    result=moving_boundary([.8,.1],[.8,.7],.04,.10)
    assert result['valid'] and not result['memory_equals_physical']
    assert result['u_minus']==[.8,.1] and result['controller_previous_control']==[.8,.7]
