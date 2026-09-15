"""Synthetic unit fixtures only; no experimental evidence."""
import json
import math
import numpy as np
import pytest
from reconciliation.robotless_online import (AbsolutePacer,CommandActivation,dump_new,integrate_unicycle,raw_manifest,verify_resume)

@pytest.mark.parametrize('angle',[-math.pi,-1,0,1,math.pi])
def test_straight_unicycle(angle):
    p=integrate_unicycle([3,4,angle],[.8,0],.1)
    np.testing.assert_allclose(p[:2],[3+.08*math.cos(angle),4+.08*math.sin(angle)])

@pytest.mark.parametrize('omega',[-3,-.000000001,0,.000000001,3])
def test_finite_rotation_only(omega):
    p=integrate_unicycle([3,4,0],[0,omega],.1)
    np.testing.assert_allclose(p,[3,4,.1*omega],atol=1e-15)

def test_exact_arc_split_time():
    p=[1,2,3]
    result=integrate_unicycle(p,[.8,2.1],1)
    for _ in range(60): p=integrate_unicycle(p,[.8,2.1],1/60)
    np.testing.assert_allclose(p,result,atol=1e-13)

@pytest.mark.parametrize('dt',[0,-1,float('nan'),float('inf')])
def test_invalid_dt(dt):
    with pytest.raises(ValueError):integrate_unicycle([0,0,0],[0,0],dt)

@pytest.mark.parametrize('p,u',[([0,0],[0,0]),([0,0,0],[1]),([0,0,float('inf')],[0,0])])
def test_invalid_values(p,u):
    with pytest.raises(ValueError):integrate_unicycle(p,u,.1)

def state(i):
    return dict(state_id=i,tick=i,sim_time_s=i*.1,episode_time_s=i*.1,host_monotonic_ns=1000000000+i*100000000,host_monotonic_s=1+i*.1,host_utc='fixture',x=3+i*.01,y=4,yaw=.2)

def result(version,u=(.1,.2)):
    return dict(status='command',chunk_id=f'c{version}',reference_version=version,solve_id=f's{version}',command=list(u))

def test_install_does_not_activate_or_snap():
    machine=CommandActivation();p=state(0)
    machine.install('c0',0,{})
    cmd,event=machine.apply(p,None,None)
    assert event is None and machine.active is None and cmd['v_mps']==0 and p==state(0)

def test_actual_same_numeric_command_identity_switch_p_b_chain():
    m=CommandActivation();m.install('c0',0,{});m.accept(result(0),0)
    cmd,boot=m.apply(state(0),None,None)
    assert boot['bootstrap']
    m.install('c1',1,{});held,event=m.apply(state(1),state(0),cmd)
    assert event is None and held['chunk_id']=='c0'
    m.accept(result(1),.2);fresh,event=m.apply(state(2),state(1),held)
    assert not event['bootstrap'] and event['old']['chunk_id']=='c0'
    assert event['B']==[3.02,4,.2] and event['P']==[3.01,4,.2]
    assert fresh['v_mps']==held['v_mps'] and fresh['chunk_id']=='c1'
    m.install('c2',2,{});m.accept(result(2),.3)
    _,next_event=m.apply(state(3),state(2),fresh)
    assert next_event['old']['chunk_id']==event['fresh']['chunk_id']

def test_stale_result_cannot_reactivate_old_generation():
    m=CommandActivation();m.install('c0',0,{});m.install('c1',1,{})
    assert not m.accept(result(0),0)
    cmd,event=m.apply(state(0),None,None)
    assert event is None and cmd['reference_version'] is None and len(m.rejected)==1

def test_hold_timeout_zero_preserves_reference_identity():
    m=CommandActivation(.5);m.install('c0',0,{});m.accept(result(0),0)
    old,_=m.apply(state(0),None,None)
    cmd,event=m.apply(state(6),state(5),old)
    assert event is None and cmd['v_mps']==cmd['omega_radps']==0
    assert cmd['reason']=='controller_timeout' and cmd['chunk_id']=='c0'

def test_nonmonotonic_install_refused():
    m=CommandActivation();m.install('c0',0,{})
    with pytest.raises(ValueError):m.install('c1',0,{})

def test_absolute_deadlines_do_not_accumulate_sleep_drift():
    p=AbsolutePacer(100,2)
    assert p.remaining(3,100.2)==pytest.approx(.8)
    assert p.remaining(4,101.8)==pytest.approx(.2)
    assert p.observe(4,102)['rtf']==1
    assert p.observe(4,103)['deadline_missed'] and p.missed_deadlines==1

def test_resume_immutable_and_no_overwrite(tmp_path):
    ep=tmp_path/'ep';assert not verify_resume(ep)
    ep.mkdir()
    with pytest.raises(FileExistsError):verify_resume(ep)
    (ep/'rgb').mkdir();(ep/'rgb/f.jpg').write_bytes(b'synthetic')
    dump_new(ep/'completion.json',{'raw_manifest':raw_manifest(ep)})
    assert verify_resume(ep)
    with pytest.raises(FileExistsError):dump_new(ep/'completion.json',{})
    (ep/'rgb/f.jpg').write_bytes(b'changed')
    with pytest.raises(ValueError):verify_resume(ep)


@pytest.mark.parametrize('raises',[False,True])
def test_render_guard_restores_automatic_physics_and_does_not_step(raises):
    from reconciliation.robotless_online import without_automatic_physics
    class Settings:
        value=True
        def get(self,key):return self.value
        def set_bool(self,key,value):self.value=value
    settings=Settings();calls=[]
    def render():
        assert not settings.value
        calls.append('render')
        if raises:raise RuntimeError('fixture')
        return 7
    if raises:
        with pytest.raises(RuntimeError):without_automatic_physics(settings,render)
    else:assert without_automatic_physics(settings,render)==7
    assert settings.value and calls==['render']
