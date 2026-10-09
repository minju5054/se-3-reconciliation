"""Authenticated source inspection and synthetic contracts; no S3 scientific calls."""
import ast
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
import shutil
import sys
import numpy as np
import pytest
import yaml
from reconciliation.se2 import local_trajectory_to_world,relative_pose,wrap_angle
from reconciliation.s3_isaac_ablation01 import *
from reconciliation.successive_isaac_four_method01 import ProgressGraph
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import integrate_unicycle
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_s3_isaac_relative_factor_ablation01 as runner
import validate_s3_isaac_relative_factor_ablation01 as validator
PREP=ROOT/'data/s3_isaac_relative_factor_ablation_01/primary_20261009'


@pytest.fixture(scope='module')
def source():return runner.inputs(yaml.safe_load(runner.CONFIG.read_text()))


def test_authenticated_historical_chain_and_assertions(source):
    assert source['auth']['valid'] and len(source['auth']['historical_chain'])==4
    e=source['entry']['correspondence']
    assert e['arc_m']==.46193137914572885 and e['normalized_progress']==.3662231723212672
    assert e['segment']==2 and e['alpha']==.9725845948085713
    assert e==read(PREP/'entry.json')['correspondence']
    assert source['schedule']['start_tick']==1804 and source['schedule']['attempted_submit_ticks'][0]==1806
    assert source['historical']['termination']=='OBSERVATION_CAP'


def test_authentication_rejects_changed_pin():
    cfg=yaml.safe_load(runner.CONFIG.read_text());cfg['historical_result_sha256']='0'*64
    with pytest.raises(AssertionError):runner.inputs(cfg)


def test_exact_C3_and_suffix_native_identity(source):
    from reconciliation.spatial_correspondence_selector import SpatialCurve
    c=source['common'];F=source['native_installed'];S=source['suffix'];e=source['entry']
    assert SpatialCurve(F).correspondences(c['B'])['C3_FORWARD_SE2']==e['correspondence']
    assert S[0].tobytes()==np.asarray(e['correspondence']['target_world']).tobytes()
    assert S[1:].tobytes()==F[e['row_original_identities'][1:]].tobytes()
    assert len(S)==8 and e['raw_prefix_rows_removed']==3


def test_raw_A_anchor_B_entry_immutable_no_B_reanchor(source):
    before={k:source[k].tobytes() for k in ['raw','original','native_installed','suffix']}
    c=source['common'];w,l,labels=source['references']['B_ENTRY']
    assert source['original'].tobytes()==local_trajectory_to_world(c['fresh_capture_pose'],source['raw']).tobytes()
    assert not np.allclose(local_trajectory_to_world(c['B'],source['raw']),source['original'])
    assert w[0].tobytes()==np.asarray(c['B']).tobytes()
    for i,lab in enumerate(labels):
        if lab.startswith('F_'):
            j=int(lab[2:]);assert w[i].tobytes()==source['native_installed'][j].tobytes()
            assert l[i].tobytes()==source['raw'][j].tobytes()
    for rel in [True,False]:
        p=AblationGraph(source['P'],c['B'],source['suffix'],include_relative=rel);p.residual(p.pack(p.initial()))
    assert before=={k:source[k].tobytes() for k in before}


def test_exact_common_clock_memory_and_schedule(source):
    c=source['common'];s=source['schedule'];h=source['historical']
    assert c==read(PREP/'common_state.json') and s==read(PREP/'schedule.json')
    assert (PREP/'historical_schedule.json').read_bytes()==(source['folder']/'schedule.json').read_bytes()
    assert source['P'].tobytes()==np.load(PREP/'recorded_old_to_B.npy')[-2].tobytes()
    assert c['u_mem_B']==c['u_B_plus']!=c['u_minus']
    assert c['original_generation']==326 and c['fresh_version']==24
    assert s['integration_steps']==180 and len(s['pairs'])==30
    assert [e['input_state_id'] for e in h['events'] if e.get('type')=='solve_result']==s['attempted_submit_ticks']


def test_source_specific_official_speed_zero_solve_preflight(source):
    p=read(PREP/'restoration_preflight.json');c=source['common']
    assert p['passed'] and p['numerical_MPC_calls']==0
    assert p['provenance']['official_settings']==source['mpc']['official_settings']
    assert p['provenance']['effective_linear_velocity_limit_m_s']==.8
    assert p['provenance']['official_settings']['HORIZON']==5
    assert len(p['rows'])==4
    for r in p['rows']:
        assert r['previous_control']==c['u_mem_B'] and r['held_command']==c['u_B_plus']
        assert r['generation']==326 and r['restored_future_results']==r['new_solve_calls']==0


def test_historical_safety_reference_gate(source):
    env=runner.geometry(PREP);p=read(PREP/'prepared_references.json')
    for name,ref in p.items():
        safe=runner.reference_safety(np.load(ref['world_path']),env)
        assert safe==ref['safety'] and safe['clearance_valid']
        assert safe['footprint_radius_m']==.2 and safe['required_edge_clearance_m']==.05
        assert safe['numerical_tolerance_m']==1e-7 and not safe['cart_present']
    outside=np.array([[1e6,1e6,0],[1e6+1,1e6,0]])
    assert not runner.reference_safety(outside,env)['clearance_valid']


def synthetic_problem(n=5,relative=True):
    S=np.c_[np.linspace(.1,1.3,n),np.linspace(.04,.2,n),np.linspace(3.10,3.4,n)]
    return AblationGraph([-.1,0,3.], [0,0,3.1],S,include_relative=relative)


@pytest.mark.parametrize('n',[3,5,9])
def test_default_FULL_unchanged_NO_R_absent_and_identical_initial(n):
    p=synthetic_problem(n);q=synthetic_problem(n,False);old=ProgressGraph(p.P,p.B,p.S)
    z=p.pack(p.initial())
    assert p.initial().tobytes()==q.initial().tobytes()==old.initial().tobytes()
    np.testing.assert_array_equal(p.residual(z),old.residual(z));assert p.costs(z)==old.costs(z)
    f=old.factors(z);np.testing.assert_array_equal(q.residual(z),np.r_[f['T'],f['A']])
    assert len(p.residual(z))-len(q.residual(z))==3*(n-1)
    assert set(q.costs(z))=={'T','A','total'}
    assert q.relative_diagnostic(z)['normalized_squared_residual']==p.costs(z)['R']
    assert not q.relative_diagnostic(z)['optimized']
    z+=.02;x=q.unpack(z)
    assert x[0].tobytes()==q.B.tobytes() and x[-1].tobytes()==q.S[-1].tobytes()
    assert len(z)==n-2


def test_frozen_source_initial_hashes_no_new_solve(source):
    h=read(PREP/'initialization_hashes.json');assert len(set(h.values()))==1
    assert read(PREP/'protocol.json')['solver']==asdict(SolverConfig())
    for n in GRAPHS:
        p=AblationGraph(source['P'],source['common']['B'],source['suffix'],include_relative=n=='FULL_GRAPH')
        assert sha(PREP/(n+'_initial.npy'))==h[n]
        assert np.load(PREP/(n+'_initial.npy')).tobytes()==p.initial().tobytes()


def test_synthetic_NO_R_solve_diagnostic_and_safety_rejection():
    S=np.array([[0.,0,0],[.4,0,0],[.8,0,0],[1.2,0,0]])
    p=AblationGraph([-.3,0,0],[-.1,0,0],S,include_relative=False)
    z=p.pack(p.initial());checks=[]
    def deny(x):checks.append(x.copy());return np.array_equal(x,z)
    r=solve_least_squares(z,p.residual,SolverConfig(max_iterations=3),candidate_feasibility_fn=deny)
    assert checks and not r.converged;np.testing.assert_array_equal(r.optimized,z)
    r=solve_least_squares(z,p.residual,SolverConfig(),candidate_feasibility_fn=p.noncollapsed)
    assert r.converged and p.relative_diagnostic(r.optimized)['normalized_squared_residual']>0
    d=deformation(p,p.unpack(r.optimized));assert d['endpoint_displacement_m']==0
    assert 'R' not in d['factor_costs']


def test_clock_batches_exact_saved_time_and_independent_reset(source):
    class World:
        def __init__(self):self.current_time=0.;self.calls=0
        def step(self,render):assert render is False;self.current_time+=dt;self.calls+=1
    c=source['common'];dt=c['integration_dt_s'];budget=read(PREP/'clock_budget.json')['maximum_steps']
    results=[]
    for _ in ORDER:
        w=World();r=prime_s3_clock(w,c['B_sim_s'],dt,budget);results.append(r)
        assert w.calls==1806 and w.current_time==c['B_sim_s'] and len(r['batches'])==2
        before=w.current_time
        for _ in range(40):_ = w.current_time
        assert w.current_time==before
    assert all(r==results[0] for r in results)
    with pytest.raises(ValueError):prime_s3_clock(World(),c['B_sim_s'],dt,1000)
    with pytest.raises(ValueError):prime_s3_clock(World(),c['B_sim_s']+.001,dt,budget+1)


def synthetic_run(tmp_path,monkeypatch,source,unsafe=False):
    for name in ['protocol.json','common_state.json','schedule.json','mpc_provenance.json','historical_native.json','source_manifest.json','scenario.json','entry.json','source_authentication.json','original.npy','suffix.npy','native_installed.npy','FULL_GRAPH_initial.npy','GRAPH_NO_R_initial.npy']:
        shutil.copyfile(PREP/name,tmp_path/name)
    save(tmp_path/'scientific_start.json',dict(freeze_sha='SYNTHETIC_TEST'))
    refs=read(PREP/'preflight_references.json')
    for n in GRAPHS:refs[n]=refs.pop(n+'_INITIAL')
    save(tmp_path/'references.json',refs)
    c=source['common'];h=source['historical'];s=source['schedule'];workers=[]
    class Isaac:
        def __init__(self):self.names=[];self.steps=0
    isaac=Isaac()
    class MPC:
        def __init__(self,*args):self.queue=[];self.previous=None;self.process=SimpleNamespace(returncode=0);workers.append(self)
        def await_type(self,*args,**kwargs):return dict(provenance=read(PREP/'mpc_provenance.json'))
        def send(self,op,**kw):
            assert op=='submit'
            e=deepcopy(next(e for e in h['events'] if e.get('type')=='solve_result' and e['input_state_id']==kw['input_state_id']))
            e.update(input_pose=kw['pose'],previous_command=self.previous,solve_id=kw['solve_id'])
            self.previous=e['command']
            self.queue=[dict(status='submitted',**kw),e]
        def drain(self):q=self.queue;self.queue=[];return q
        def shutdown(self):pass
    def ask(worker,op,**kw):
        if worker is isaac:
            if op=='reset_method':worker.names.append(kw['method']);worker.x=np.array(c['B']);worker.tick=c['B_tick']
            elif op=='step':
                assert kw['guard']['safe'];worker.x=integrate_unicycle(worker.x,[kw['command']['v_mps'],kw['command']['omega_radps']],c['integration_dt_s']);worker.tick+=1;worker.steps+=1
            return dict(tick=worker.tick,pose_world=worker.x.tolist(),sim_time_s=c['B_sim_s']+(worker.tick-c['B_tick'])*c['integration_dt_s'],USD_readback_error=0.,scene={'cart_present':False})
        worker.previous=c['u_mem_B']
        return dict(B=c['B'],held_command=c['u_B_plus'],previous_control=c['u_mem_B'],generation=c['original_generation'],reference_version=c['fresh_version'],restored_future_results=0,new_solve_calls=0,
            installed_world=next(row['installed_world'] for row in read(PREP/'restoration_preflight.json')['rows'] if row['identity']==kw['identity']+('_INITIAL' if kw['identity'] in GRAPHS else '')))
    monkeypatch.setattr(runner,'Worker',MPC);monkeypatch.setattr(runner,'ask',ask)
    if unsafe:
        original=runner.guard_check
        def guard(*args,**kw):g=original(*args,**kw);g['safe']=False;return g
        monkeypatch.setattr(runner,'guard_check',guard)
    results=[runner.run_method(tmp_path,n,isaac) for n in ORDER]
    return results,isaac,workers


def test_synthetic_runtime_schedule_reset_memory_wall_wait_RAW_gate(tmp_path,monkeypatch,source):
    results,isaac,workers=synthetic_run(tmp_path,monkeypatch,source)
    assert isaac.names==ORDER and isaac.steps==720 and len(workers)==4
    c=source['common'];s=source['schedule'];tol=read(PREP/'protocol.json')['parity']
    for r in results:
        assert r['error'] is None,r['error']
        assert len(r['commands'])==180 and len(r['collected_results'])==30 and len(r['states'])==181
        assert [q['tick'] for q in r['submit_requests']]==s['attempted_submit_ticks']
        assert [q['application_tick'] for q in r['commands'] if q['reason']=='new_solve' and q['application_tick']>c['B_tick']]==s['application_ticks']
        assert all(w['Isaac_clock_and_pose_unchanged'] for w in r['waits'])
        assert r['collected_results'][0]['previous_command']==c['u_mem_B']
        assert all(a['command']==b['previous_command'] for a,b in zip(r['collected_results'],r['collected_results'][1:]))
    gate=raw_parity(results[0],source['historical'],s,tol);assert gate['passed'],gate
    for key in ['pose','command','clock','memory','selection','generation']:
        bad=deepcopy(results[0]);e=next(e for e in bad['events'] if e.get('type')=='solve_result')
        if key=='pose':bad['states'][5]['pose_world'][0]+=.001
        if key=='command':bad['commands'][5]['omega_radps']+=.001
        if key=='clock':bad['states'][5]['sim_time_s']+=.001
        if key=='memory':e['previous_command'][0]+=.001
        if key=='selection':e['selection']['indices'][0]+=1
        if key=='generation':e['result_generation']+=1
        assert not raw_parity(bad,source['historical'],s,tol)['passed'],key
    with pytest.raises(FileExistsError):runner.run_method(tmp_path,'RAW',isaac)
    # Saved-only metric contract runs on a synthetic replay, never scientific evidence.
    m=evaluate(results[0],source['original'],runner.geometry(PREP)['on'],source['entry'],180*c['integration_dt_s'])
    old=metrics(results[0],source['original'],runner.geometry(PREP)['on'],180*c['integration_dt_s'])
    assert m['position_auc_03_m_s']==old['position_auc_03_m_s'] and m['yaw_auc_03_rad_s']==old['yaw_auc_03_rad_s']


def test_synthetic_abort_leaves_unsafe_command_unapplied(tmp_path,monkeypatch,source):
    results,isaac,workers=synthetic_run(tmp_path,monkeypatch,source,True)
    assert isaac.steps==0
    assert all(r['abort']['command_applied'] is False and len(r['states'])==1 and not r['commands'] for r in results)
    m=evaluate(results[0],source['original'],runner.geometry(PREP)['on'],source['entry'],180*source['common']['integration_dt_s'])
    assert m['position_auc_03_m_s'] is None and m['turn50_03']['status']=='INCOMPLETE_WINDOW'


@pytest.mark.parametrize('sign',[1,-1])
def test_turn_response_signed_persistence_and_censor(sign):
    t=[0,.1,.2,.3];yaw=sign*np.array([0,.6,.4,.6])
    assert turn50(t,yaw,0,sign,.3)['time_s'] is None
    yaw=sign*np.array([0,.6,.6,.6]);r=turn50(t,yaw,0,sign,.3)
    assert r['time_s']==.1 and r['confirmation_time_s']==.2 and r['status']=='OBSERVED'
    assert turn50(t,np.zeros(4),0,sign,.3)['status']=='CENSORED_NOT_REACHED'
    assert turn50(t,yaw,0,sign,.9)['status']=='INCOMPLETE_WINDOW'


def test_turn_wrap_near_zero_and_missing_order():
    r=turn50([0,.1,.2,.3],[3.1,-3.05,-3.,-2.9],3.1,-3.,.3)
    assert r['time_s']==.1
    n=turn50([0,.1],[0,0],0,0,.1);assert n['status']=='N/A_NEAR_ZERO_TURN'
    assert latency_difference(n,n,.01)['delta_s'] is None
    short=turn50([0,.1],[0,.6],0,1,.3)
    full=turn50([0,.1,.2,.3],[0,.6,.6,.6],0,1,.3)
    assert latency_difference(full,short,.01)['ordering']=='INCOMPLETE_WINDOW_NO_ORDER'


def test_saved_guard_blocks_new_scientific_calls():
    p=synthetic_problem()
    with pytest.raises(RuntimeError,match='saved-only'):
        with validator.saved_guard():solve_least_squares(p.pack(p.initial()),p.residual,SolverConfig())


def test_scientific_budget_guard_no_retry_before_solve(tmp_path):
    with pytest.raises(AssertionError):runner.graph_solve(tmp_path,'FULL_GRAPH')
    save(tmp_path/'scientific_start.json',dict(freeze_sha='SYNTHETIC'))
    for name in ['protocol.json','common_state.json','source_authentication.json','suffix.npy','FULL_GRAPH_initial.npy','source_manifest.json']:
        shutil.copyfile(PREP/name,tmp_path/name)
    (tmp_path/'planning/FULL_GRAPH').mkdir(parents=True)
    with pytest.raises(FileExistsError):runner.graph_solve(tmp_path,'FULL_GRAPH')


def test_runtime_and_saved_tools_no_model_Hermite_or_new_controller():
    for name in ['scripts/isaac/s3_relative_ablation01.py','scripts/validate_s3_isaac_relative_factor_ablation01.py','scripts/report_s3_isaac_relative_factor_ablation01.py']:
        tree=ast.parse((ROOT/name).read_text())
        calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))}
        assert not calls & {'solve_least_squares','capture_rgb','capture_observation','get_data','generate','hermite_bridge','graph_solve'}
    text=(ROOT/'scripts/isaac/s3_relative_ablation01.py').read_text()
    assert 'world.reset()' in text and 'world.step(render=False)' in text and 'scene_check()' in text
    assert 'set_precise_agent_pose' in text and 'prime_s3_clock' in text
    assert 'successive_four_method_mpc_worker' not in (ROOT/'scripts/run_s3_isaac_relative_factor_ablation01.py').read_text()


def test_exact_two_saved_only_PNGs_missing_execution(tmp_path,monkeypatch,source):
    import report_s3_isaac_relative_factor_ablation01 as report
    fixture=tmp_path/'fixture';fixture.mkdir();out=tmp_path/'out';out.mkdir()
    for name in ['original.npy','recorded_old_to_B.npy','source_authentication.json','source_manifest.json']:
        shutil.copyfile(PREP/name,fixture/name)
    save(fixture/'references.json',read(PREP/'prepared_references.json'))
    monkeypatch.setattr(report,'OUT',out)
    s=dict(entry=source['entry']['correspondence'],common=source['common'],classification='SYNTHETIC_TEST_NOT_EVIDENCE',primary_metrics={n:None for n in ORDER},planning={n:dict(valid=False,deformation=None) for n in GRAPHS})
    m=report.render(fixture,s,{})
    assert m['figure_count']==2 and sorted(p.name for p in (out/'figures').iterdir())==sorted(PNGS)
    assert m['missing_execution']==ORDER and all(v is None for v in m['max_matched_XY_separation_m'].values())


def test_saved_validator_computes_synthetic_RAW_and_missing_methods(tmp_path,monkeypatch,source):
    results,_,_=synthetic_run(tmp_path,monkeypatch,source)
    for n in ORDER[1:]:(tmp_path/'methods'/n/'rollout.json').unlink()
    refs=read(tmp_path/'references.json')
    for n in GRAPHS:
        p=AblationGraph(source['P'],source['common']['B'],source['suffix'],include_relative=n=='FULL_GRAPH');x=p.initial()
        folder=tmp_path/'planning'/n;folder.mkdir(parents=True)
        save(folder/'result.json',dict(valid=True,status='REFERENCE_SAFE',optimizer_calls=1,solver=dict(converged=True),candidate_world=x.tolist(),deformation=deformation(p,x),initial_costs=p.costs(p.pack(x)),final_costs=p.costs(p.pack(x)),reference=refs[n]))
    gate=raw_parity(results[0],source['historical'],source['schedule'],read(PREP/'protocol.json')['parity'])
    save(tmp_path/'RAW_parity.json',gate)
    save(tmp_path/'execution_summary.json',dict(classification='TECHNICAL_BLOCKED'))
    with validator.saved_guard():summary,validation,traces=validator.compute(tmp_path)
    assert validation['valid'] and summary['calls']['official_MPC_solves']==30
    assert summary['classification']=='TECHNICAL_BLOCKED' and summary['RAW_parity']['passed']
    assert summary['primary_metrics']['RAW']['integration_steps']==180
    assert all(summary['primary_metrics'][n] is None for n in ORDER[1:])
    assert set(traces)=={'RAW'}
    # Test the report's non-empty error/q(t) paths, including invalid planning labels.
    import report_s3_isaac_relative_factor_ablation01 as report
    shutil.copyfile(PREP/'recorded_old_to_B.npy',tmp_path/'recorded_old_to_B.npy')
    out=tmp_path/'output';out.mkdir();monkeypatch.setattr(report,'OUT',out)
    report.render(tmp_path,summary,traces)
    assert len(list((out/'figures').glob('*.png')))==2


@pytest.mark.parametrize('planning_valid',[False,True])
def test_RAW_gate_stops_scientific_driver_and_repeat_forbidden(tmp_path,monkeypatch,planning_valid):
    for n in ['protocol.json','launch_config.json','launch_environment.json','historical_native.json','schedule.json','prepared_references.json']:
        shutil.copyfile(PREP/n,tmp_path/n)
    (tmp_path/'logs').mkdir();out=tmp_path/'out';out.mkdir();monkeypatch.setattr(runner,'OUT',out)
    monkeypatch.setattr(runner,'verify',lambda *a:None);monkeypatch.setattr(runner,'git',lambda *a:'SYNTHETIC')
    monkeypatch.setattr(runner,'launch_argv',lambda *a:read(tmp_path/'launch_environment.json')['argv'])
    monkeypatch.setattr(runner,'graph_solve',lambda run,n:dict(valid=planning_valid,reference=read(tmp_path/'prepared_references.json')['RAW']))
    monkeypatch.setattr(runner,'observe_process',lambda *a:{'synthetic':True})
    monkeypatch.setattr(runner,'reserve_launch',lambda *a:None)
    called=[]
    def method(run,n,isaac):called.append(n);return {'error':None}
    monkeypatch.setattr(runner,'run_method',method);monkeypatch.setattr(runner,'raw_parity',lambda *a:dict(passed=False))
    class Fake:
        def __init__(self,*a):self.process=SimpleNamespace(returncode=0)
        def await_type(self,*a,**kw):return dict(pid=0)
        def shutdown(self):pass
        def drain(self):return []
    monkeypatch.setattr(runner,'Worker',Fake)
    runner.execute(tmp_path)
    assert called==['RAW']
    assert read(tmp_path/'execution_summary.json')['classification']=='RAW_ISAAC_PARITY_FAIL'
    with pytest.raises(FileExistsError):runner.execute(tmp_path)
