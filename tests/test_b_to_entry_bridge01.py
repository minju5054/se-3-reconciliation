"""Synthetic geometry/solver/executor tests plus authenticated saved-only reads."""
import ast
from copy import deepcopy
from dataclasses import asdict
import inspect
import json
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_b_to_entry_bridge01 as runner
import validate_b_to_entry_bridge01 as validator
import test_relative_factor_multisource01 as old
import test_spatial_entry_suffix_execution01 as entry_tests
from reconciliation.b_to_entry_bridge import *
from reconciliation.se2 import local_trajectory_to_world,compose_poses
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares,OptimizationError
from reconciliation.spatial_entry_suffix import no_reconciliation_optimizer
from reconciliation.join_source03 import read,save,sha
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
PREP=ROOT/'data/b_to_entry_bridge_01/primary_20261002T020000Z'
SELECTED=read(PREP/'selected_sources.json')
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())


@pytest.mark.parametrize('spec',SELECTED,ids=SOURCE_IDS)
def test_C3_P_B_E_suffix_raw_frames_density_and_official_preflight_exact(spec):
    f=Path(spec['folder']);p=runner.load_problem(f);c=read(f/'common_state.json');r=read(f/'reuse.json')
    raw=np.load(r['methods'][NATIVE]['reference']['local_path']);copy=raw.copy()
    fresh=np.load(r['methods'][NATIVE]['reference']['world_path']);entry=read(f/'entry.json');b=read(f/'bridge_input.json')
    runner.suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['historical_C3'])
    np.testing.assert_array_equal(p.B,c['B']);np.testing.assert_array_equal(p.E,entry['correspondence']['target_world'])
    np.testing.assert_array_equal(p.P,read(f/'source_audit.json')['context']['P'])
    assert b['P_state_id']==c['B_tick']-1
    native=np.asarray(read(Path(r['methods'][NATIVE]['folder'])/'restoration.json')['installed_world'])
    h,hd=hermite_bridge(p.P,p.B,p.suffix);np.testing.assert_array_equal(h,np.load(f/'hermite_bridge.npy'))
    assert hd['M']==max(2,math.ceil(hd['Hermite_continuous_arc_m']/hd['d_F_m']))
    w,l,labels=reference_arrays(p,h,c['fresh_capture_pose'],raw,b['original_ids'])
    np.testing.assert_array_equal(w[0],p.B);np.testing.assert_array_equal(w[p.M],p.E)
    np.testing.assert_array_equal(w[p.M+1:],native[b['original_ids']])
    np.testing.assert_array_equal(l[p.M+1:],raw[b['original_ids']]);np.testing.assert_array_equal(raw,copy)
    np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],l),w,rtol=0,atol=1e-12)
    assert not np.allclose(local_trajectory_to_world(c['B'],l),w)
    pre=read(f/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    np.testing.assert_array_equal(np.asarray(pre['rows'][0]['installed_world'])[p.M+1:],native[b['original_ids']])
    assert pre['rows'][0]['previous_control']==c['u_mem_B'] and pre['rows'][0]['generation']==c['original_generation']
    for path,ref in r['copies'].items():assert sha(f/path)==sha(ref['path'])==ref['sha256']
    for n in [NATIVE,ENTRY]:
        out=Path(r['methods'][n]['folder']);assert sha(out/'hashes.json')==r['methods'][n]['hashes_sha256']
        for path,digest in read(out/'hashes.json').items():assert sha(out/path)==digest
    assert [label for label in labels if label.startswith('F_')]==[f'F_{j}' for j in b['original_ids']]


def test_authenticate_chain_and_tamper():
    runner.authenticate(CFG,deep=True)
    bad=deepcopy(CFG);bad['historical_result_sha256']='0'*64
    with pytest.raises(AssertionError):runner.authenticate(bad)
    assert CFG['solver']==asdict(SolverConfig())


def synthetic():
    P=np.array([-.03,0,.2]);B=np.array([0.,0,3.1]);suffix=np.array([[.7,.2,-3.1],[1.,.3,-3.05],[1.3,.4,-3.]])
    return P,B,suffix


def test_Hermite_analytic_endpoints_tangents_equal_arc_shortest_yaw():
    P,B,s=synthetic();x,d=hermite_bridge(P,B,s);xy,derivative,arc,m0,m1=hermite_curve(P,B,s[0],s[1])
    np.testing.assert_allclose(xy(0),B[:2],atol=0);np.testing.assert_allclose(xy(1),s[0,:2],atol=0)
    np.testing.assert_array_equal(derivative(0),m0);np.testing.assert_array_equal(derivative(1),m1)
    np.testing.assert_allclose(np.linalg.norm(m0),np.linalg.norm(s[0,:2]-B[:2]),rtol=1e-14)
    np.testing.assert_allclose([arc(t) for t in d['Hermite_parameters']],np.linspace(0,arc(1),len(x)),atol=1e-12,rtol=0)
    np.testing.assert_allclose(wrap_angle(x[:,2]-B[2]),np.linspace(0,wrap_angle(s[0,2]-B[2]),len(x)),atol=1e-14)
    np.testing.assert_array_equal(x[0],B);np.testing.assert_array_equal(x[-1],s[0])


def test_density_is_spatial_and_incoming_distance_absent():
    P,B,s=synthetic();x,d=hermite_bridge(P,B,s)
    P2=B+100*(P-B);x2,d2=hermite_bridge(P2,B,s)
    np.testing.assert_array_equal(x,x2)
    p=BridgeProblem(P,B,s,d['d_F_m'],d['M']);p2=BridgeProblem(P2,B,s,d['d_F_m'],d['M'])
    np.testing.assert_array_equal(p.residual(x[1:-1]),p2.residual(x[1:-1]))
    B3=B.copy();B3[:2]*=3;P3=P.copy();P3[:2]*=3;s3=s.copy();s3[:,:2]*=3
    _,d3=hermite_bridge(P3,B3,s3);assert d3['M']==d['M']
    assert all('time' not in key and 'dt' not in key for key in inspect.signature(hermite_bridge).parameters)


@pytest.mark.parametrize('M',[2,3,5])
def test_graph_fixed_endpoints_smoothness_spacing_and_real_LM_synthetic(M):
    P=np.array([-.01,0,0]);B=np.array([0.,0,0]);suffix=np.array([[1.,.2,.1],[1.5,.3,.1]])
    p=BridgeProblem(P,B,suffix,.3,M)
    x=np.linspace(B,suffix[0],M+1);interior=x[1:-1].copy();f=p.factors(interior)
    D=np.array([relative_pose(a,b) for a,b in zip(x,x[1:])])
    expected=np.array([se2_log(relative_pose(a,b)) for a,b in zip(D,D[1:])])/[.3,.3,np.deg2rad(10)]
    np.testing.assert_array_equal(f['E_smooth'],expected.ravel())
    np.testing.assert_array_equal(f['E_space'],np.diff(np.linalg.norm(np.diff(x[:,:2],axis=0),axis=1))/.3)
    assert p.residual(interior).shape==(2+4*(M-1),)
    assert np.isclose(sum(p.costs(interior)[k] for k in f),p.residual(interior)@p.residual(interior))
    trace=[]
    result=solve_least_squares(interior,p.residual,SolverConfig(),candidate_feasibility_fn=p.nondegenerate,iteration_callback=trace.append)
    assert result.final_cost<=result.initial_cost and result.converged
    for e in trace:
        full=p.reference(e['state']);np.testing.assert_array_equal(full[0],B);np.testing.assert_array_equal(full[M:],suffix)
    np.testing.assert_array_equal(interior,x[1:-1])


def test_straight_uniform_graph_all_residuals_zero_and_no_P_distance_cost():
    p=BridgeProblem([-.003,0,0],[0,0,0],[[1,0,0],[1.5,0,0]],.5,2)
    np.testing.assert_array_equal(p.residual([[.5,0,0]]),np.zeros(6))
    assert not p.nondegenerate([[0,0,0]])
    assert np.isfinite(p.residual([[0,0,0]])).all()


def test_graph_safety_acceptance_rejects_improving_proposals_without_repair():
    p=BridgeProblem([-.1,0,0],[0,0,0],[[1,0,0],[2,0,0]],1.,2)
    initial=np.array([[.5,.2,.1]]);trace=[]
    def feasible(x):return bool(np.array_equal(x,initial))
    result=solve_least_squares(initial,p.residual,SolverConfig(),candidate_feasibility_fn=feasible,iteration_callback=trace.append)
    np.testing.assert_array_equal(result.optimized,initial)
    assert result.termination_reason=='step_tolerance'
    assert any(e['decision']=='rejected_unsafe' for e in trace)
    for e in trace:np.testing.assert_array_equal(e['state'],initial)


@pytest.mark.parametrize('which',['incoming','chord','suffix'])
def test_undefined_geometry_fails_closed(which):
    P,B,s=synthetic()
    if which=='incoming':P=B.copy()
    elif which=='chord':s[0]=B
    else:s[1]=s[0]
    with pytest.raises(ValueError):hermite_bridge(P,B,s)


def setup(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene=old.setup(tmp_path,monkeypatch,bt)
    fresh=np.load(refs[NATIVE]['world_path'])
    for n in [ENTRY,*NEW]:
        refs[n]=deepcopy(refs[NATIVE])
        w=np.vstack([c['B'],fresh]) if n in NEW else fresh.copy()
        for frame,array in [('world',w),('local',relative_pose(c['fresh_capture_pose'],w))]:
            path=tmp_path/'references'/f'{n}_{frame}.npy';np.save(path,array)
            refs[n][frame+'_path']=str(path);refs[n][frame+'_sha256']=sha(path)
        refs[n]['safety']=runner.reference_safety(w,env)
    (tmp_path/'references.json').write_text(json.dumps(refs))
    return c,refs,manifest,env,scene


@pytest.mark.parametrize('bt',[92,997,1804,1524])
def test_arbitrary_N_official_executor_identical_schedule_memory_and_metrics(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene=setup(tmp_path,monkeypatch,bt)
    assert runner.legacy.run_method is old.runner.run_method
    for name in ORDER:
        runner.legacy.run_method(tmp_path,name);out=tmp_path/'methods'/name;r=read(out/'rollout.json');m=read(out/'metrics.json')
        assert r['error'] is None and r['new_MPC_solved']==30 and len(r['commands'])==180
        _,_,audit=validator.validate_method(tmp_path,name,refs[name],c,np.load(refs[NATIVE]['world_path']),env['on'],scene,manifest['mpc']['official_settings'])
        assert audit['valid'];assert validator.schedule(r,54)==read(tmp_path/'schedule.json')['primary']
        assert validator.schedule(r,180)==read(tmp_path/'schedule.json')['full']
        row=execution_metrics(r,m,np.load(refs[NATIVE]['world_path']))
        for k,v in validator.metric_row(m,r['termination']).items():assert row[k]==v
        assert row['original_FRESH_endpoint_dwell_s']==validator.independent_endpoint_time(r,np.load(refs[NATIVE]['world_path']))
        for wait in r['logical_waits']:assert wait['pose_before']==wait['pose_after'] and wait['sim_before']==wait['sim_after']
    old.test_release_and_generation_existing_path_unchanged()


@pytest.mark.parametrize('name',NEW)
def test_bridge_reference_gate_and_abort_command_unapplied(tmp_path,monkeypatch,name):
    _,refs,_,_,_=setup(tmp_path,monkeypatch,997);calls=[];guard=old.runner.guard_check
    def reject(*a):
        g=guard(*a);calls.append(1)
        if len(calls)==6:g['safe']=False
        return g
    monkeypatch.setattr(old.runner,'guard_check',reject);runner.legacy.run_method(tmp_path,name)
    r=read(tmp_path/'methods'/name/'rollout.json');assert len(r['commands'])==5 and not r['safety_abort']['command_applied']
    (tmp_path/'methods'/name).rename(tmp_path/'methods'/'ABORT_SYNTHETIC')
    refs[name]['safety']['clearance_valid']=False;(tmp_path/'references.json').write_text(json.dumps(refs))
    monkeypatch.setattr(old.runner,'Worker',lambda *a:pytest.fail('unsafe bridge reached controller'))
    with pytest.raises(AssertionError):runner.legacy.run_method(tmp_path,name)


@pytest.mark.parametrize('duration,expected',[(.29,None),(.3,0.),(.6,0.)])
def test_endpoint_complete_execution_dwell_and_null(duration,expected):
    t=np.linspace(0,duration,31);p=np.zeros((31,3));u=np.zeros((30,2))
    result=endpoint_dwell(t,p,[0,0,0],u,attachment_s=None)
    assert result['original_FRESH_endpoint_dwell_s']==expected and result['T_post_attach_s'] is None
    assert execution_metrics is entry_tests.execution_metrics


def sources():
    s={}
    for sid in SOURCE_IDS:
        metrics={n:dict(position_auc_09_m_s=.2,sustained_attachment_s=1.,original_FRESH_endpoint_dwell_s=2.) for n in ORDER}
        if sid==SOURCE_IDS[2]:
            for r in metrics.values():r.update(sustained_attachment_s=None,original_FRESH_endpoint_dwell_s=None)
        s[sid]=dict(primary_metrics=metrics,technical_valid=True,reference_failure=False)
    return s


def test_classification_prespecified_categories_and_loss():
    s=sources();assert classify(s)['classification']=='BRIDGE_INSUFFICIENT'
    hard=s[SOURCE_IDS[2]]['primary_metrics'];hard[GRAPH]['position_auc_09_m_s']=.18
    assert classify(s)['classification']=='BRIDGE_PARTIAL_ONLY'
    hard[GRAPH]['sustained_attachment_s']=1.
    assert classify(s)['classification']=='GRAPH_BRIDGE_SUPPORTED'
    hard[HERMITE]['sustained_attachment_s']=1.
    assert classify(s)['classification']=='SIMPLE_BRIDGE_SUFFICIENT'
    s[SOURCE_IDS[0]]['primary_metrics'][GRAPH]['original_FRESH_endpoint_dwell_s']=None
    assert classify(s)['classification']=='BRIDGE_TRADEOFF'
    s[SOURCE_IDS[0]]['reference_failure']=True
    assert classify(s)['classification']=='BRIDGE_REFERENCE_FAILURE'
    s[SOURCE_IDS[0]]['technical_valid']=False
    assert classify(s)['classification']=='TECHNICAL_BLOCKED'


def test_saved_validation_and_report_no_solve_and_unchanged_controller():
    for role in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{role}_b_to_entry_bridge01.py').read_text())
        names={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not names&{'solve_least_squares','solve_bridge','run_method','execute','Worker','submit','load_official'}
    def solve_least_squares():pytest.fail('entered forbidden solve')
    with pytest.raises(RuntimeError,match='forbidden'):
        with no_reconciliation_optimizer():solve_least_squares()
    source=read(Path(SELECTED[0]['folder'])/'source_manifest.json')['mpc']['mpc_source']
    assert sha(source)==PINNED_MPC_SHA256


def test_exactly_four_PNGs_and_nulls_separate(tmp_path,monkeypatch):
    import report_b_to_entry_bridge01 as report
    selected=[];summary={};original=Path(SELECTED[0]['folder']);reuse=read(original/'reuse.json')
    native=Path(reuse['methods'][NATIVE]['folder']);h=read(original/'prepared_references.json')[HERMITE]
    problem=runner.load_problem(original);b=read(original/'bridge_input.json')
    for i,sid in enumerate(SOURCE_IDS):
        folder=tmp_path/f'S{i}';folder.mkdir();(folder/'recorded_old_to_B.npy').write_bytes((original/'recorded_old_to_B.npy').read_bytes())
        refs={NATIVE:reuse['methods'][NATIVE]['reference'],ENTRY:reuse['methods'][ENTRY]['reference'],HERMITE:h,GRAPH:h}
        save(folder/'final_references.json',refs)
        q=dict(bridge_input=b,method_folders={n:str(native) for n in ORDER},primary_metrics={},bridge_geometry={})
        for n in ORDER:
            m=read(native/'metrics.json');r=read(native/'rollout.json');row=execution_metrics(r,m,np.load(refs[NATIVE]['world_path']))
            if i==2:row.update(sustained_attachment_s=None,original_FRESH_endpoint_dwell_s=None,remaining_arc_at_attachment_m=None)
            q['primary_metrics'][n]=row
        for n in NEW:q['bridge_geometry'][n]=bridge_geometry(problem,np.load(original/'hermite_bridge.npy')[1:-1],h['safety'])
        selected.append(dict(id=sid,folder=str(folder)));summary[sid]=q
    monkeypatch.setattr(report,'geometry',lambda _:runner.geometry(original))
    rows=report.compact_figures(dict(selected_sources=selected,sources=summary),tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert len(rows[3]['numeric_sidecar']['censored'])==4
    assert all(r['T_endpoint'] is None for r in rows[3]['numeric_sidecar']['censored'])
    assert all((tmp_path/'figures'/r['file']).read_bytes().startswith(b'\x89PNG') for r in rows)


def test_actual_planner_saved_trace_validation_is_solve_free(tmp_path,monkeypatch):
    f=tmp_path/'run/sources/toy';f.mkdir(parents=True)
    P,B,s=synthetic();bridge,d=hermite_bridge(P,B,s)
    save(f/'bridge_input.json',d);np.save(f/'suffix_native_installed.npy',s);np.save(f/'hermite_bridge.npy',bridge)
    save(f/'protocol.json',CFG);save(f.parents[1]/'execution_start.json',dict(sha='synthetic'))
    monkeypatch.setattr(runner,'geometry',lambda _:None)
    checker=lambda x,env:dict(clearance_valid=True,minimum_clearance_m=1.)
    monkeypatch.setattr(runner,'reference_safety',checker);monkeypatch.setattr(validator,'reference_safety',checker)
    x=runner.solve_bridge(f,'synthetic');assert x is not None
    monkeypatch.setattr(runner,'solve_least_squares',lambda *a,**k:pytest.fail('saved audit solved again'))
    with no_reconciliation_optimizer():audit=validator.validate_planning(f,None)
    assert audit['valid'] and audit['solver_success'] and audit['new_solves']==0
    with pytest.raises(FileExistsError):runner.solve_bridge(f,'synthetic')


def test_first_H5_bridge_and_original_identity_mapping():
    event=dict(type='solve_result',status='command',input_state_id=10,input_pose=[0,0,0],
        selection=dict(indices=[1,2,3,4,4],nearest_index=0),continuation_seen=dict(tick=11))
    rows=validator.selection_rows(dict(events=[event]),['B','bridge_1','E*','F_3','F_4'])
    assert rows[0]['selected_identities']==['bridge_1','E*','F_3','F_4','F_4']
    assert rows[0]['nearest_identity']=='B' and rows[0]['application_tick']==11
