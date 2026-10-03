"""Saved authentication plus synthetic fixtures; no scientific planner or MPC calls."""
import ast
from copy import deepcopy
from pathlib import Path
import json
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_b_to_entry_vector_bridge_execution03 as runner
import validate_b_to_entry_vector_bridge_execution03 as validator
import report_b_to_entry_vector_bridge_execution03 as report
import test_relative_factor_multisource01 as old
from reconciliation.vector_bridge_execution03 import *
from reconciliation.join_source03 import read,save,sha
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.se2 import local_trajectory_to_world
PREP=ROOT/'data/b_to_entry_vector_bridge_execution_03/primary_20261003T100000Z'
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
SELECTED=read(PREP/'selected_sources.json')


@pytest.mark.parametrize('s',SELECTED,ids=['S1','S2','S3','S4'])
def test_exact_saved_V2_B_X1_E_suffix_frames_raw_schedule_installation(s):
    f=Path(s['folder']);refs=read(f/'references.json');c=read(f/'common_state.json');b=read(f/'bridge_input.json')
    reuse=read(f/'reuse.json');p=read(f/'planning_reuse.json');raw=np.load(refs[NATIVE]['local_path']);before=raw.copy()
    assert sha(f/'v2_bridge.npy')==p['diag_bridge_sha256']==sha(p['diag_bridge_path'])
    assert sha(refs[VECTOR]['world_path'])==p['diag_reference_sha256']==sha(p['diag_reference_path'])
    bridge=np.load(f/'v2_bridge.npy');world=np.load(refs[VECTOR]['world_path']);suffix=np.load(f/'suffix_native_installed.npy')
    w,l,labels=installed_reference(bridge,world,suffix,c['fresh_capture_pose'],raw,b['original_ids'],c['B'],b['E'])
    assert w[:3].tobytes()==bridge.tobytes() and l[3:].tobytes()==raw[b['original_ids']].tobytes()
    assert bridge[0].tobytes()==np.asarray(c['B']).tobytes() and bridge[2].tobytes()==np.asarray(b['E']).tobytes()
    np.testing.assert_array_equal(l,np.load(refs[VECTOR]['local_path']));np.testing.assert_array_equal(raw,before)
    np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],l),w,atol=1e-12,rtol=0)
    assert not np.allclose(local_trajectory_to_world(c['B'],l),w)
    assert labels==['B','bridge_1','E*',*[f'F_{i}' for i in b['original_ids']]]
    native=np.asarray(read(Path(reuse['methods'][NATIVE]['folder'])/'restoration.json')['installed_world'])
    assert w[3:].tobytes()==native[b['original_ids']].tobytes()
    pre=read(f/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    assert pre['rows'][0]['previous_control']==c['u_mem_B'] and pre['rows'][0]['generation']==c['original_generation']
    assert np.asarray(pre['rows'][0]['installed_world'])[3:].tobytes()==native[b['original_ids']].tobytes()
    assert refs[VECTOR]['safety']['clearance_valid'] and read(f/'installation.json')['installed_safety']['clearance_valid']
    for name,rec in reuse['copies'].items():assert sha(f/name)==sha(rec['path'])==rec['sha256']
    assert sha(read(f/'source_manifest.json')['mpc']['mpc_source'])==PINNED_MPC_SHA256
    broken=world.copy();broken[1,0]+=.001
    with pytest.raises(AssertionError):installed_reference(bridge,broken,suffix,c['fresh_capture_pose'],raw,b['original_ids'],c['B'],b['E'])


def test_full_authentication_prior_validators_stable4_and_tamper():
    with no_reconciliation_optimizer():d,h=runner.authenticate(CFG,deep=True)
    assert all(q['V2']['stable'] for q in d['summary']['sources'].values())
    for key in ['diag_result_sha256','bridge_result_sha256','entry_result_sha256']:
        bad=deepcopy(CFG);bad[key]='0'*64
        with pytest.raises(AssertionError):runner.authenticate(bad)
    hard=h['summary']['sources'][SOURCE_IDS[2]]['primary_metrics']
    assert all(hard[n]['original_FRESH_endpoint_dwell_s'] is None for n in [NATIVE,ENTRY])
    assert hard[HERMITE]['original_FRESH_endpoint_dwell_s'] is not None


def setup(tmp_path,monkeypatch,bt):
    c,refs,m,env,scene=old.setup(tmp_path,monkeypatch,bt)
    refs[VECTOR]=deepcopy(refs[NATIVE]);refs[HERMITE]=deepcopy(refs[NATIVE])
    (tmp_path/'references.json').write_text(json.dumps(refs))
    return c,refs,m,env,scene


@pytest.mark.parametrize('bt',[92,997,1804,1524])
def test_unchanged_executor_memory_schedule_release_original_metric_endpoint(tmp_path,monkeypatch,bt):
    c,refs,m,env,scene=setup(tmp_path,monkeypatch,bt)
    assert runner.runtime.run_method is old.runner.run_method
    with no_reconciliation_optimizer():runner.runtime.run_method(tmp_path,VECTOR)
    r=read(tmp_path/'methods'/VECTOR/'rollout.json');metric=read(tmp_path/'methods'/VECTOR/'metrics.json')
    assert r['error'] is None and r['new_MPC_solved']==30 and r['new_optimizer_calls']==0
    _,_,audit=validator.validate_method(tmp_path,VECTOR,refs[VECTOR],c,np.load(refs[NATIVE]['world_path']),env['on'],scene,m['mpc']['official_settings'])
    assert audit['valid'] and schedule_check(r,read(tmp_path/'schedule.json'),c)['passed']
    for w in r['logical_waits']:assert w['pose_before']==w['pose_after'] and w['sim_before']==w['sim_after']
    row=execution_metrics(r,metric,np.load(refs[NATIVE]['world_path']))
    assert row['original_FRESH_endpoint_dwell_s']==validator.independent_endpoint_time(r,np.load(refs[NATIVE]['world_path']))
    for k,v in validator.metric_row(metric,r['termination']).items():assert row[k]==v
    old.test_release_and_generation_existing_path_unchanged()


def test_unsafe_reference_and_abort_unapplied(tmp_path,monkeypatch):
    c,refs,m,env,scene=setup(tmp_path,monkeypatch,997);calls=[];guard=runner.runtime.guard_check
    def reject(*args):
        g=guard(*args);calls.append(1)
        if len(calls)==6:g['safe']=False
        return g
    monkeypatch.setattr(runner.runtime,'guard_check',reject)
    runner.runtime.run_method(tmp_path,VECTOR);r=read(tmp_path/'methods'/VECTOR/'rollout.json')
    assert len(r['commands'])==5 and not r['safety_abort']['command_applied']
    assert not schedule_check(r,read(tmp_path/'schedule.json'),c)['passed']
    (tmp_path/'methods'/VECTOR).rename(tmp_path/'methods'/'SYNTHETIC_ABORT')
    refs[VECTOR]['safety']['clearance_valid']=False;(tmp_path/'references.json').write_text(json.dumps(refs))
    monkeypatch.setattr(runner.runtime,'Worker',lambda *a:pytest.fail('unsafe reference reached worker'))
    with pytest.raises(AssertionError):runner.runtime.run_method(tmp_path,VECTOR)


@pytest.mark.parametrize('duration,attachment,expected,post',[(.29,None,None,None),(.3,None,0.,None),(.6,0.,0.,0.)])
def test_endpoint_execution_time_null_postattach_parity(duration,attachment,expected,post):
    r=endpoint_dwell(np.linspace(0,duration,31),np.zeros((31,3)),[0,0,0],np.zeros((30,2)),attachment)
    assert r['original_FRESH_endpoint_dwell_s']==expected and r['T_post_attach_s']==post


def synthetic_sources():
    result={}
    for sid in SOURCE_IDS:
        metrics={n:dict(sustained_attachment_s=1.,original_FRESH_endpoint_dwell_s=2.,position_auc_09_m_s=.2,linear_command_TV=1.,angular_command_TV=2.) for n in ORDER}
        result[sid]=dict(primary_metrics=metrics,integration_dt_s=1/60,technical_valid=True,reference_or_execution_failure=False)
    return result


@pytest.mark.parametrize('kind,expected',[
    ('technical','TECHNICAL_BLOCKED'),('unsafe','VECTOR_REFERENCE_OR_EXECUTION_FAILURE'),
    ('lost','HARD_CASE_RECOVERY_LOST'),('partial','HARD_CASE_RECOVERY_LOST'),
    ('easy_lost','VECTOR_GRAPH_TRADEOFF_WORSE'),('later','VECTOR_GRAPH_TRADEOFF_WORSE'),
    ('reduced','VECTOR_GRAPH_TRADEOFF_REDUCED'),('same','VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN')])
def test_classification_fixed_precedence(kind,expected):
    s=synthetic_sources();easy=[sid for sid in SOURCE_IDS if sid!=SOURCE_IDS[2]]
    if kind=='technical':s[easy[0]]['technical_valid']=False
    if kind=='unsafe':s[easy[0]]['reference_or_execution_failure']=True
    if kind in ['lost','partial']:
        s[SOURCE_IDS[2]]['primary_metrics'][VECTOR]['original_FRESH_endpoint_dwell_s']=None
        if kind=='lost':s[SOURCE_IDS[2]]['primary_metrics'][VECTOR]['sustained_attachment_s']=None
    if kind=='easy_lost':s[easy[0]]['primary_metrics'][VECTOR]['original_FRESH_endpoint_dwell_s']=None
    if kind=='later':s[easy[0]]['primary_metrics'][VECTOR]['original_FRESH_endpoint_dwell_s']+=1/60
    if kind=='reduced':
        for sid in easy[:2]:s[sid]['primary_metrics'][VECTOR]['original_FRESH_endpoint_dwell_s']-=1/60
    assert classify(s)['classification']==expected


def test_one_tick_threshold_and_null_never_cap():
    s=synthetic_sources();v=s[SOURCE_IDS[0]]['primary_metrics'][VECTOR]
    v['original_FRESH_endpoint_dwell_s']+=1/60-2e-9
    assert classify(s)['classification']=='VECTOR_GRAPH_RECOVERY_NO_CLEAR_TRADEOFF_GAIN'
    v['original_FRESH_endpoint_dwell_s']+=2e-9
    assert classify(s)['classification']=='VECTOR_GRAPH_TRADEOFF_WORSE'


def test_selector_identity_exposure_and_tick_aligned_commands():
    f=Path(SELECTED[0]['historical_folder']);r=read(f/'methods/HERMITE_BRIDGE/rollout.json');labels=read(f/'final_references.json')[HERMITE]['labels']
    ex=exposure(r,labels);assert ex['first']['H5_identities'][0]=='bridge_1'
    assert ex['first']['nearest_identity']=='B' and ex['bridge_1_submit_count']>0
    assert all(x in labels for row in ex['rows'] for x in row['H5_identities'])
    v=deepcopy(r);event=next(e for e in v['events'] if e.get('type')=='solve_result' and e.get('status')=='command');event['command'][0]+=.001
    comp=command_comparison(r,v,labels,labels)
    assert comp['first_different_submit']['tick']==event['input_state_id'] and comp['first_different_application'] is None
    assert all(x['actual_XY_separation_m']==0 for x in comp['rows'])
    assert len([x for x in comp['rows'] if x['kind']=='applied_interval'])==180


def test_no_new_optimizer_or_historical_execution_call_path():
    for role in ['run','validate','report']:
        tree=ast.parse((ROOT/f'scripts/{role}_b_to_entry_vector_bridge_execution03.py').read_text())
        calls={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not calls&{'solve_least_squares','solve_graph','solve_bridge','solve_condition','solve_mpc','capture_rgb'}
        if role!='run':assert not calls&{'run_method','Worker','preflight'}
        else:
            invocations=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='run_method']
            assert len(invocations)==1 and invocations[0].args[1].id=='VECTOR'
    def solve_least_squares():pytest.fail('guard should reject')
    with pytest.raises(RuntimeError):
        with no_reconciliation_optimizer():solve_least_squares()
    assert CFG['LightNav']==CFG['RGB']==CFG['Isaac']==CFG['optimizer_calls']==CFG['historical_reruns']==CFG['V3_rollouts']==0


def test_exactly_three_PNGs_nulls_and_saved_only_report(tmp_path):
    historical=read(ROOT/CFG['bridge_result'])['summary'];sources={};selected=[]
    for spec in SELECTED:
        sid=spec['id'];q=historical['sources'][sid];f=Path(spec['folder']);refs=read(f/'references.json')
        metrics={n:deepcopy(q['primary_metrics'][n]) for n in ORDER[:-1]};metrics[VECTOR]=deepcopy(metrics[HERMITE])
        if sid==SOURCE_IDS[2]:metrics[VECTOR]['sustained_attachment_s']=metrics[VECTOR]['original_FRESH_endpoint_dwell_s']=None
        paths={n:q['method_folders'][n] for n in ORDER[:-1]};paths[VECTOR]=paths[HERMITE]
        selector={n:exposure(read(Path(paths[n])/'rollout.json'),refs[HERMITE]['labels'] if n==VECTOR else
            refs[HERMITE]['labels'] if n==HERMITE else ['E*',*[f'F_{i}' for i in q['bridge_input']['original_ids']]]) for n in [ENTRY,HERMITE,VECTOR]}
        sources[sid]=dict(primary_metrics=metrics,method_folders=paths,bridge_input=q['bridge_input'],selector_exposure=selector,
            bridge_geometry=geometry_pair(np.load(refs[HERMITE]['world_path']),np.load(refs[VECTOR]['world_path']),{n:refs[n]['safety'] for n in [HERMITE,VECTOR]}),
            integration_dt_s=read(f/'common_state.json')['integration_dt_s'],technical_valid=True,reference_or_execution_failure=False)
        selected.append(spec)
    summary=dict(sources=sources,selected_sources=selected,cross_source=classify(sources))
    figures=report.compact_figures(summary,tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert len(figures)==3 and all((tmp_path/'figures'/f['file']).read_bytes().startswith(b'\x89PNG') for f in figures)
    assert figures[1]['numeric_sidecar']['original_FRESH_endpoint_dwell_s'][VECTOR][2] is None
    assert not list(tmp_path.rglob('*.html'))
