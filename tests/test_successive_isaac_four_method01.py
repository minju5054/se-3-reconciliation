"""Synthetic/runtime-contract tests and zero-solve source preflight; no live Isaac."""
import ast
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from reconciliation.se2 import local_trajectory_to_world
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_successive_isaac_four_method01 as runner
from reconciliation.successive_isaac_four_method01 import *
from reconciliation.join_source03 import read,save,sha
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares
from reconciliation.osa03_relative_ablation import LogicalRelease
from reconciliation.spatial_correspondence_selector import SpatialCurve
PREP=ROOT/'data/successive_isaac_four_method_comparison_01/primary_20261009'


@pytest.fixture(scope='module')
def source():
    return runner.inputs(read(PREP/'protocol.json'))


def test_authenticated_corrected_window_excludes_next_install(source):
    _,_,auth,b,arr,c,s,h=source
    assert auth['valid'] and auth['sealed_file_count']==401
    assert s['start_tick']==92 and s['stop_tick']==120 and s['integration_steps']==28
    assert s['attempted_submit_ticks']==[96,102,108,114]
    assert s['application_ticks']==[97,103,109,115]
    assert s['excluded_next_install']['chunk_id']=='chunk_002'
    assert len(h['states'])==29 and len(h['controls'])==28 and len(h['solves'])==4
    assert all(e['chunk_id']=='chunk_001' for e in h['solves'])
    assert c['u_minus']!=c['u_mem_B']==c['u_B_plus']


def test_saved_C3_shared_exact_entry_no_tuning(source):
    _,_,_,b,arr,_,_,_=source
    refs,S,e,hermite,p=fixed_references(arr['fresh_world'],arr['fresh_raw_local'],b['A'],b['B'],b['P'])
    assert e==read(PREP/'entry.json')
    assert e['correspondence']==SpatialCurve(arr['fresh_world']).correspondences(b['B'])['C3_FORWARD_SE2']
    E=np.asarray(e['correspondence']['target_world'])
    assert E.tobytes()==S[0].tobytes()==p.S[0].tobytes()
    assert E.tobytes()==refs['B_ENTRY'][0][1].tobytes()
    assert E.tobytes()==refs['HERMITE'][0][hermite['M']].tobytes()
    assert e['raw_prefix_rows_removed']==0 and e['correspondence']['arc_m']==0
    assert hermite['M']==2 and len(p.S)==10


def test_raw_and_downstream_bit_identity_no_reanchor(source):
    _,_,_,b,a,_,_,_=source
    before={k:v.tobytes() for k,v in a.items()}
    refs,S,e,h,p=fixed_references(a['fresh_world'],a['fresh_raw_local'],b['A'],b['B'],b['P'])
    for n,(w,l,labels) in refs.items():
        np.testing.assert_allclose(local_trajectory_to_world(b['A'],l),w,rtol=0,atol=1e-12)
        assert not np.allclose(local_trajectory_to_world(b['B'],l),w)
        for j,lab in enumerate(labels):
            if lab.startswith('F_'):
                i=int(lab[2:]);assert w[j].tobytes()==a['fresh_world'][i].tobytes()
                assert l[j].tobytes()==a['fresh_raw_local'][i].tobytes()
    assert before=={k:v.tobytes() for k,v in a.items()}


def problem():
    S=np.array([[.1,.05,3.12],[.5,.13,-3.10],[1.,.1,-3.0],[1.7,.2,-2.9]])
    return ProgressGraph([-.1,0,0],[0,0,3.1],S)


def test_graph_initialization_exact_fixed_endpoints_only_internal():
    p=problem();x=p.initial();assert x.tobytes()==p.initial().tobytes()
    delta=compose_poses(p.B,inverse_pose(p.S[0]))
    expected=compose_poses(se2_exp((1-p.s)[:,None]*se2_log(delta)),p.S)
    np.testing.assert_array_equal(x[1:-1],expected[1:-1])
    z=p.pack(x);z[0]+=.01;y=p.unpack(z)
    assert y[0].tobytes()==p.B.tobytes() and y[-1].tobytes()==p.S[-1].tobytes()
    assert z.shape==(len(p.S)-2,3)
    with pytest.raises(ValueError):p.unpack(x)
    with pytest.raises(ValueError):ProgressGraph(p.P,p.B,p.S[:2])


def test_graph_relative_anchor_and_total_algebra():
    p=problem();x=p.initial();x[1,1]+=.013
    factors=p.factors(p.pack(x))
    dS=relative_pose(p.S[:-1],p.S[1:]);dX=relative_pose(x[:-1],x[1:])
    expected=se2_log(relative_pose(dS,dX))/[.10,.10,np.deg2rad(10)]
    np.testing.assert_array_equal(factors['R'],expected.ravel())
    expected=p.s[1:-1,None]*se2_log(relative_pose(p.S[1:-1],x[1:-1]))/SCALES
    np.testing.assert_array_equal(factors['A'],expected.ravel())
    assert len(factors['A'])==3*(len(p.S)-2)
    assert np.isclose(p.costs(p.pack(x))['total'],p.residual(p.pack(x))@p.residual(p.pack(x)))


def test_direction_wrap_and_no_incoming_distance_factor():
    phi=np.deg2rad(179);P=-np.r_[np.cos(phi),np.sin(phi),0.]
    S=np.array([[0,0,0],[-1,-.01,0],[-2,-.01,0]])
    p=ProgressGraph(P,[0,0,0],S);other=ProgressGraph(P*9,[0,0,0],S)
    interior=np.array([[np.cos(-phi),np.sin(-phi),0.]])
    np.testing.assert_allclose(p.factors(interior)['T'],[2/15],atol=1e-12)
    np.testing.assert_array_equal(p.residual(interior),other.residual(interior))


def test_hermite_frozen_tangents_and_spatial_density(source):
    from reconciliation.b_to_entry_bridge import hermite_curve,hermite_bridge
    _,_,_,b,a,_,_,_=source
    S=np.load(PREP/'suffix.npy');h,m=hermite_bridge(b['P'],b['B'],S)
    xy,deriv,arc,m0,m1=hermite_curve(b['P'],b['B'],S[0],S[1])
    np.testing.assert_allclose(deriv(0),m0);np.testing.assert_allclose(deriv(1),m1)
    assert m['M']==max(2,int(np.ceil(arc(1)/np.median(np.linalg.norm(np.diff(S[:,:2],axis=0),axis=1)))))
    assert h[0].tobytes()==np.asarray(b['B']).tobytes() and h[-1].tobytes()==S[0].tobytes()
    assert not any('time' in k or k=='dt' for k in m)


def test_graph_synthetic_solve_only_not_evidence():
    S=np.array([[0.,0,0],[.4,0,0],[.8,0,0],[1.2,0,0]])
    p=ProgressGraph([-.3,0,0],[-.1,0,0],S)
    r=solve_least_squares(p.pack(p.initial()),p.residual,SolverConfig(),candidate_feasibility_fn=p.noncollapsed)
    assert r.converged and p.unpack(r.optimized)[-1].tobytes()==S[-1].tobytes()


def test_preflight_real_official_objects_zero_numerical_solves():
    p=read(PREP/'restoration_preflight.json')
    assert p['passed'] and p['numerical_MPC_calls']==0
    assert p['provenance']['official_settings']['OBJNAV_V_MAX']==.4
    assert p['provenance']['official_settings']['HORIZON']==5
    c=read(PREP/'common_state.json')
    for row in p['rows']:
        assert row['previous_control']==c['u_mem_B'] and row['held_command']==c['u_B_plus']
        assert row['generation']==3 and row['restored_future_results']==row['new_solve_calls']==0


def test_clock_priming_and_wait_do_not_fake_time():
    class World:
        current_time=0.;calls=0
        def step(self,render):self.current_time+=float(np.float32(1/60));self.calls+=1
    w=World();dt=float(np.float32(1/60));r=prime_clock(w,94*dt,dt)
    assert w.calls==94 and w.current_time==94*dt and r['zero_motion_initialization_steps']==94
    with pytest.raises(ValueError):prime_clock(w,w.current_time-.2,dt)
    before=w.current_time
    for _ in range(100):_ = w.current_time
    assert w.current_time==before


def synthetic_run(tmp_path,monkeypatch,source,unsafe=False):
    cfg=read(PREP/'protocol.json');_,_,_,b,arr,c,s,h=source
    for n in ['protocol.json','common_state.json','schedule.json','mpc_provenance.json','historical_prefix.json']:
        save(tmp_path/n,read(PREP/n))
    save(tmp_path/'scientific_start.json',{'freeze_sha':'SYNTHETIC_TEST'})
    refs=read(PREP/'preflight_references.json');refs['GRAPH']=refs.pop('GRAPH_INITIAL_DIAGNOSTIC')
    save(tmp_path/'references.json',refs)
    class Isaac:
        def __init__(self):self.reset_names=[];self.steps=0
    isaac=Isaac();created=[]
    class MPC:
        def __init__(self,*a):
            self.queue=[];self.prev=None;self.process=SimpleNamespace(returncode=0);created.append(self)
        def await_type(self,*a,**kw):return dict(provenance=read(PREP/'mpc_provenance.json'))
        def send(self,op,**kw):
            assert op=='submit'
            e=deepcopy(next(e for e in h['solves'] if e['input_state_id']==kw['input_state_id']))
            e.update(input_pose=kw['pose'],previous_command=self.prev,solve_id=kw['solve_id'])
            self.prev=e['command']
            self.queue=[dict(status='submitted',**kw),e]
        def drain(self):q=self.queue;self.queue=[];return q
        def shutdown(self):pass
    def ask(worker,op,**kw):
        if worker is isaac:
            if op=='reset_method':
                worker.reset_names.append(kw['method']);worker.x=np.array(c['B']);worker.tick=92
            elif op=='step':
                assert kw['guard']['safe'];worker.x=integrate_unicycle(worker.x,[kw['command']['v_mps'],kw['command']['omega_radps']],c['integration_dt_s']);worker.tick+=1;worker.steps+=1
            return dict(pose_world=worker.x.tolist(),sim_time_s=c['B_sim_s']+(worker.tick-92)*c['integration_dt_s'],tick=worker.tick,
                cart={'present':True},USD_readback_error=0.)
        worker.prev=c['u_mem_B']
        return dict(B=c['B'],held_command=c['u_B_plus'],previous_control=c['u_mem_B'],generation=3,
            restored_future_results=0,new_solve_calls=0,installed_world=np.load(kw['reference']['world_path']).tolist())
    monkeypatch.setattr(runner,'Worker',MPC);monkeypatch.setattr(runner,'ask',ask)
    if unsafe:
        original=runner.guard_check
        def guard(*a,**kw):g=original(*a,**kw);g['safe']=False;return g
        monkeypatch.setattr(runner,'guard_check',guard)
    results=[runner.run_method(tmp_path,n,isaac) for n in ORDER]
    return results,isaac,created


def test_synthetic_all_methods_same_schedule_reset_no_leakage_and_RAW_parity(tmp_path,monkeypatch,source):
    results,isaac,workers=synthetic_run(tmp_path,monkeypatch,source)
    assert isaac.reset_names==ORDER and isaac.steps==4*28 and len(workers)==4
    c=source[5];s=source[6];h=source[7]
    for r in results:
        assert r['error'] is None and len(r['commands'])==28
        assert r['states'][0]['pose_world']==c['B']
        assert [q['tick'] for q in r['submit_requests']]==[96,102,108,114]
        assert [q['application_tick'] for q in r['commands'] if q['reason']=='new_solve']==[92,97,103,109,115]
        assert all(w['Isaac_clock_and_pose_unchanged'] for w in r['waits'])
    gate=parity(results[0],h,s,read(PREP/'protocol.json')['parity'])
    assert gate['passed'] and gate['pose_max_error']<=1e-10
    bad=deepcopy(results[0]);bad['commands'][5]['omega_radps']+=1e-6
    assert not parity(bad,h,s,read(PREP/'protocol.json')['parity'])['passed']


def test_synthetic_guard_abort_never_steps_unsafe_command(tmp_path,monkeypatch,source):
    results,isaac,workers=synthetic_run(tmp_path,monkeypatch,source,unsafe=True)
    assert isaac.steps==0
    assert all(r['abort']['command_applied'] is False and len(r['states'])==1 and not r['commands'] for r in results)


def test_incomplete_parity_and_AUC_remain_null(source):
    s,h=source[6:8]
    q=parity(dict(states=[],commands=[]),h,s,read(PREP/'protocol.json')['parity'])
    assert not q['passed'] and q['pose_max_error'] is None
    assert auc(np.array([0.,.1]),np.ones(2),.3) is None
    assert np.isclose(auc(np.array([0.,.2,.4]),np.ones(3),.3),.3)


def test_no_model_C2_or_optimizer_in_runtime_and_validators():
    for n in ['scripts/isaac/successive_four_method01.py','scripts/successive_four_method_mpc_worker01.py',
              'scripts/validate_successive_isaac_four_method01.py','scripts/report_successive_isaac_four_method01.py']:
        tree=ast.parse((ROOT/n).read_text())
        calls={x.func.attr if isinstance(x.func,ast.Attribute) else x.func.id for x in ast.walk(tree)
            if isinstance(x,ast.Call) and isinstance(x.func,(ast.Name,ast.Attribute))}
        assert not calls & {'solve_least_squares','graph_solve','capture_observation','capture_rgb','get_data','generate'}
        assert not any(isinstance(x,ast.Constant) and x.value=='chunk_002' for x in ast.walk(tree))
    text=(ROOT/'scripts/isaac/successive_four_method01.py').read_text()
    assert "world.step(render=False)" in text and 'set_precise_agent_pose' in text
    assert 'C2' not in text and 'set_current_time(before)' in text


def test_two_PNGs_with_NA_no_fabricated_execution(tmp_path):
    from report_successive_isaac_four_method01 import draw
    import shutil
    fixture=tmp_path/'input';fixture.mkdir()
    for n in ['protocol.json','common_state.json','entry.json','prepared_references.json','old.npy']:
        shutil.copyfile(PREP/n,fixture/n)
    (fixture/'references').mkdir()
    shutil.copyfile(PREP/'references/RAW_world.npy',fixture/'references/RAW_world.npy')
    summary=dict(classification='SYNTHETIC_PLOT_TEST_NOT_EVIDENCE',entry=read(PREP/'entry.json'),primary_metrics={n:None for n in ORDER})
    m=draw(fixture,summary,tmp_path/'figures')
    assert m['final_png_count']==2 and sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert all(v is None for values in m['numeric_sidecar'].values() for v in values.values())


@pytest.mark.parametrize('planning_valid',[False,True])
def test_scientific_driver_stops_before_other_methods_on_failed_gate(tmp_path,monkeypatch,planning_valid):
    for n in ['protocol.json','launch_config.json','launch_environment.json','historical_prefix.json','schedule.json']:
        save(tmp_path/n,read(PREP/n))
    (tmp_path/'logs').mkdir()
    monkeypatch.setattr(runner,'verify',lambda *a:None)
    monkeypatch.setattr(runner,'git',lambda *a:'SYNTHETIC_TEST')
    monkeypatch.setattr(runner,'launch_argv',lambda *a:read(tmp_path/'launch_environment.json')['argv'])
    monkeypatch.setattr(runner,'graph_solve',lambda *a:dict(valid=planning_valid))
    monkeypatch.setattr(runner,'observe_process',lambda *a:{'synthetic':True})
    executed=[]
    def method(*args):executed.append(args[1]);return {'error':None}
    monkeypatch.setattr(runner,'run_method',method)
    monkeypatch.setattr(runner,'parity',lambda *a:dict(passed=False,reason='synthetic mismatch'))
    class Fake:
        def __init__(self,*a):assert planning_valid;self.process=SimpleNamespace(returncode=0)
        def await_type(self,*a,**kw):return dict(pid=0)
        def shutdown(self):pass
        def drain(self):return []
    monkeypatch.setattr(runner,'Worker',Fake)
    runner.execute(tmp_path)
    assert executed==(['RAW'] if planning_valid else [])
    result=read(tmp_path/'execution_summary.json')
    assert result['classification']==('RAW_ISAAC_REPLAY_PARITY_FAIL' if planning_valid else 'GRAPH_PLANNING_FAILED')
    with pytest.raises(FileExistsError):runner.execute(tmp_path)
