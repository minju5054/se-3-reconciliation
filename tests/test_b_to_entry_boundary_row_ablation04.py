"""Saved authentication and synthetic fixtures; no new scientific rollout/solve."""
import ast
from copy import deepcopy
from pathlib import Path
import json
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_b_to_entry_boundary_row_ablation04 as runner
import validate_b_to_entry_boundary_row_ablation04 as validator
import report_b_to_entry_boundary_row_ablation04 as report
import test_relative_factor_multisource01 as old
from reconciliation.boundary_row_ablation04 import *
from reconciliation.spatial_entry_suffix import endpoint_dwell,suffix_reference
from reconciliation.join_source03 import read,sha
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
PREP=ROOT/'data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z'
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
SELECTED=read(PREP/'selected_sources.json')


@pytest.mark.parametrize('spec',SELECTED,ids=['S1','S2','S3','S4'])
def test_frozen_B_E_suffix_raw_identity_world_local_safety_zero_solve(spec):
    f=Path(spec['folder']);refs=read(f/'references.json');c=read(f/'common_state.json');entry=read(f/'entry.json');reuse=read(f/'reuse.json')
    native=np.asarray(read(Path(reuse['methods'][NATIVE]['folder'])/'restoration.json')['installed_world']);raw=np.load(refs[NATIVE]['local_path']);before=raw.tobytes()
    w,l,labels=staged_reference(native,raw,c['fresh_capture_pose'],c['B'],entry,np.load(f/'suffix_native_installed.npy'))
    assert w.tobytes()==np.load(refs[STAGE]['world_path']).tobytes() and l.tobytes()==np.load(refs[STAGE]['local_path']).tobytes()
    assert w[0].tobytes()==np.asarray(c['B']).tobytes() and w[1].tobytes()==np.asarray(entry['correspondence']['target_world']).tobytes()
    ids=entry['row_original_identities'][1:];assert w[2:].tobytes()==native[ids].tobytes() and l[2:].tobytes()==raw[ids].tobytes()
    assert raw.tobytes()==before and labels==['B','E*',*[f'F_{i}' for i in ids]] and 'bridge_1' not in labels
    assert len(w)==len(native[ids])+2 and not np.allclose(local_trajectory_to_world(c['B'],l),w)
    mapping=read(f/'identity_mapping.json')[STAGE];assert mapping==identity_mapping(labels,entry,native)
    assert mapping[0]['original_raw_id'] is None and mapping[0]['fractional_original_row'] is None and mapping[0]['original_arc_m'] is None
    assert mapping[1]['fractional_original_row']==entry['correspondence']['segment']+entry['correspondence']['alpha']
    # Historical correspondence parity is evaluated against the original planning FRESH.
    fresh=np.load(refs[NATIVE]['world_path']);_,_,parity=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['historical_C3'])
    assert parity['correspondence']==entry['correspondence']
    pre=read(f/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    assert pre['rows'][0]['previous_control']==c['u_mem_B'] and pre['rows'][0]['generation']==c['original_generation']
    installed=np.asarray(pre['rows'][0]['installed_world']);assert installed[2:].tobytes()==native[ids].tobytes()
    np.testing.assert_allclose(installed,w,atol=1e-12,rtol=0)
    env=runner.geometry(f);assert runner.reference_safety(w,env)['clearance_valid']
    assert len(runner.reference_safety(w,env)['per_segment_clearance_m'])==len(w)-1
    for name,rec in reuse['copies'].items():assert sha(f/name)==sha(rec['path'])==rec['sha256']
    assert sha(read(f/'source_manifest.json')['mpc']['mpc_source'])==PINNED_MPC_SHA256


def test_historical_authentication_and_all_validators_tamper():
    with no_reconciliation_optimizer():h=runner.authenticate(CFG,deep=True)
    bad=deepcopy(CFG);bad['vector_result_sha256']='0'*64
    with pytest.raises(AssertionError):runner.authenticate(bad)
    for sid,q in h['summary']['sources'].items():
        if sid==SOURCE_IDS[2]:
            assert all(q['primary_metrics'][n]['original_FRESH_endpoint_dwell_s'] is None for n in [NATIVE,ENTRY])
            assert all(q['primary_metrics'][n]['original_FRESH_endpoint_dwell_s'] is not None for n in [HERMITE,VECTOR])


@pytest.mark.parametrize('x',[1.,1.-5e-13,.5])
def test_duplicate_vertex_convention_preserved(x):
    native=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
    # A saved suffix already applies the frozen vertex deduplication rule.
    snap=abs(x-1)<1e-12;suffix=native[1:] if snap else np.vstack([[x,0,0],native[1:]])
    entry=dict(correspondence=dict(target_world=[x,0.,0.],remaining_arc_m=2-x,segment=0,alpha=x),row_original_identities=[1,2] if snap else [None,1,2])
    w,l,labels=staged_reference(native,native,[0,0,0],[-1,0,0],entry,suffix)
    assert w[1,0]==x and sum(abs(w[:,0]-1)<1e-12)==1
    assert labels==(['B','E*','F_2'] if snap else ['B','E*','F_1','F_2'])


@pytest.mark.parametrize('n',[2,3,7])
def test_short_arbitrary_references(n):
    native=np.column_stack([np.arange(n),np.zeros(n),np.zeros(n)])
    entry=dict(correspondence=dict(target_world=[.5,0.,0.],remaining_arc_m=n-1.5,segment=0,alpha=.5),row_original_identities=[None,*range(1,n)])
    w,l,labels=staged_reference(native,native,[0,0,0],[-.2,0,0],entry,np.vstack([[.5,0,0],native[1:]]))
    assert len(w)==n+1 and l.shape==w.shape


def setup(tmp_path,monkeypatch,bt):
    c,refs,m,env,scene=old.setup(tmp_path,monkeypatch,bt);refs[STAGE]=deepcopy(refs[NATIVE])
    (tmp_path/'references.json').write_text(json.dumps(refs));return c,refs,m,env,scene


@pytest.mark.parametrize('bt',[92,997,1804,1524])
def test_generic_executor_schedule_memory_release_primary_endpoint(tmp_path,monkeypatch,bt):
    c,refs,m,env,scene=setup(tmp_path,monkeypatch,bt)
    assert runner.runtime.run_method is old.runner.run_method
    with no_reconciliation_optimizer():runner.runtime.run_method(tmp_path,STAGE)
    r=read(tmp_path/'methods'/STAGE/'rollout.json');metric=read(tmp_path/'methods'/STAGE/'metrics.json');fresh=np.load(refs[NATIVE]['world_path'])
    _,_,audit=validator.validate_method(tmp_path,STAGE,refs[STAGE],c,fresh,env['on'],scene,m['mpc']['official_settings'])
    assert audit['valid'] and r['new_optimizer_calls']==0 and r['new_MPC_solved']==30 and r['error'] is None
    assert schedule_check(r,read(tmp_path/'schedule.json'),c)['passed']
    assert all(w['pose_before']==w['pose_after'] and w['sim_before']==w['sim_after'] for w in r['logical_waits'])
    row=execution_metrics(r,metric,fresh);assert row['original_FRESH_endpoint_dwell_s']==validator.independent_endpoint_time(r,fresh)
    for k,v in validator.metric_row(metric,r['termination']).items():assert row[k]==v
    old.test_release_and_generation_existing_path_unchanged()


def test_reference_gate_and_abort_unapplied(tmp_path,monkeypatch):
    c,refs,m,env,scene=setup(tmp_path,monkeypatch,997);calls=[];guard=runner.runtime.guard_check
    def reject(*args):
        g=guard(*args);calls.append(1)
        if len(calls)==6:g['safe']=False
        return g
    monkeypatch.setattr(runner.runtime,'guard_check',reject);runner.runtime.run_method(tmp_path,STAGE)
    r=read(tmp_path/'methods'/STAGE/'rollout.json');assert len(r['commands'])==5 and not r['safety_abort']['command_applied']
    assert not schedule_check(r,read(tmp_path/'schedule.json'),c)['passed']
    (tmp_path/'methods'/STAGE).rename(tmp_path/'methods'/'SYNTHETIC_ABORT')
    refs[STAGE]['safety']['clearance_valid']=False;(tmp_path/'references.json').write_text(json.dumps(refs))
    monkeypatch.setattr(runner.runtime,'Worker',lambda *a:pytest.fail('unsafe reference reached worker'))
    with pytest.raises(AssertionError):runner.runtime.run_method(tmp_path,STAGE)


@pytest.mark.parametrize('duration,attachment,expected,post',[(.29,None,None,None),(.3,None,0.,None),(.6,0.,0.,0.)])
def test_endpoint_execution_time_null_handling(duration,attachment,expected,post):
    r=endpoint_dwell(np.linspace(0,duration,31),np.zeros((31,3)),[0,0,0],np.zeros((30,2)),attachment)
    assert r['original_FRESH_endpoint_dwell_s']==expected and r['T_post_attach_s']==post


def synthetic_sources():
    return {sid:dict(primary_metrics={n:dict(sustained_attachment_s=1.,original_FRESH_endpoint_dwell_s=2.,position_auc_09_m_s=.2) for n in ORDER},
        integration_dt_s=1/60,technical_valid=True,reference_or_execution_failure=False) for sid in SOURCE_IDS}


@pytest.mark.parametrize('kind,expected',[
    ('technical','TECHNICAL_BLOCKED'),('unsafe','BOUNDARY_STAGE_REFERENCE_OR_EXECUTION_FAILURE'),
    ('lost','INTERMEDIATE_BRIDGE_NEEDED_SUPPORTED'),('partial','INTERMEDIATE_BRIDGE_NEEDED_SUPPORTED'),
    ('reduced','STAGING_SUFFICIENT_AND_PENALTY_REDUCED'),('same','STAGING_SUFFICIENT_NO_PENALTY_GAIN'),
    ('easy_lost','STAGING_RECOVERY_WITH_NEW_TRADEOFF'),('later','STAGING_RECOVERY_WITH_NEW_TRADEOFF')])
def test_precedence_and_direction(kind,expected):
    s=synthetic_sources();hard=s[SOURCE_IDS[2]]['primary_metrics'][STAGE];easy=[sid for sid in SOURCE_IDS if sid!=SOURCE_IDS[2]]
    if kind=='technical':s[easy[0]]['technical_valid']=False
    if kind=='unsafe':s[easy[0]]['reference_or_execution_failure']=True
    if kind=='lost':hard['sustained_attachment_s']=hard['original_FRESH_endpoint_dwell_s']=None
    if kind=='partial':hard['original_FRESH_endpoint_dwell_s']=None
    if kind=='easy_lost':s[easy[0]]['primary_metrics'][STAGE]['original_FRESH_endpoint_dwell_s']=None
    if kind=='later':s[easy[0]]['primary_metrics'][STAGE]['original_FRESH_endpoint_dwell_s']+=1/60
    if kind=='reduced':
        for sid in easy[:2]:s[sid]['primary_metrics'][STAGE]['original_FRESH_endpoint_dwell_s']-=1/60
    result=classify(s);assert result['classification']==expected and not result['graph_necessity_established']
    assert result['direction']==(None if kind in ['technical','unsafe'] else 'B' if kind in ['lost','partial'] else 'A' if kind=='reduced' else 'C')


def test_one_tick_threshold():
    s=synthetic_sources();m=s[SOURCE_IDS[0]]['primary_metrics'][STAGE]
    m['original_FRESH_endpoint_dwell_s']+=1/60-2e-9
    assert classify(s)['classification']=='STAGING_SUFFICIENT_NO_PENALTY_GAIN'
    m['original_FRESH_endpoint_dwell_s']+=2e-9
    assert classify(s)['classification']=='STAGING_RECOVERY_WITH_NEW_TRADEOFF'


def test_selector_physical_pose_metadata_and_command_alignment():
    f=Path(SELECTED[0]['folder']);reuse=read(f/'reuse.json');r=read(Path(reuse['methods'][HERMITE]['folder'])/'rollout.json')
    mapping=read(f/'identity_mapping.json')[HERMITE];labels=[x['label'] for x in mapping];ex=selector_exposure(r,mapping)
    e=next(e for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command')
    assert ex['first']['H5_poses']==e['selection']['reference_world'] and ex['first']['input_pose']==e['input_pose']
    assert ex['first']['nearest_identity']=='B' and ex['first']['first_H5_identity']=='bridge_1'
    assert ex['first']['H5_original_metadata'][0]['original_arc_m'] is None
    assert ex['first']['H5_original_metadata'][1]['original_arc_m'] is not None
    changed=deepcopy(r);next(e for e in changed['events'] if e.get('type')=='solve_result' and e.get('status')=='command')['command'][0]+=.001
    comp=compare_commands(r,changed,labels,labels)
    assert comp['first_different_submit']['tick']==e['input_state_id'] and comp['first_different_application'] is None and comp['first_different_H5'] is None
    assert len(comp['pose_separation_trace'])==181 and comp['max_matched_XY_separation_m']==0
    assert any(x['shared_preexisting_B_command'] for x in comp['rows'])


def test_no_planning_or_historical_execution_paths():
    paths=[ROOT/'src/reconciliation/boundary_row_ablation04.py',*[ROOT/f'scripts/{s}_b_to_entry_boundary_row_ablation04.py' for s in ['run','validate','report']]]
    for path in paths:
        tree=ast.parse(path.read_text());calls={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not calls&{'solve_least_squares','solve_graph','solve_bridge','hermite_bridge','sample_hermite','hermite_curve','solve_condition','solve_mpc','capture_rgb','suffix_reference','correspondences'}
        if not path.name.startswith('run_'):assert not calls&{'run_method','Worker','preflight'}
        else:
            invocations=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='run_method']
            assert len(invocations)==1 and invocations[0].args[1].id=='STAGE'
    def solve_least_squares():pytest.fail('optimizer must be blocked')
    with pytest.raises(RuntimeError):
        with no_reconciliation_optimizer():solve_least_squares()
    assert CFG['LightNav']==CFG['RGB']==CFG['Isaac']==CFG['optimizer_calls']==CFG['historical_reruns']==CFG['V3_rollouts']==0


def plot_fixture():
    hist=read(ROOT/CFG['vector_result'])['summary'];sources={}
    for spec in SELECTED:
        sid=spec['id'];q=deepcopy(hist['sources'][sid]);f=Path(spec['folder']);mapping=read(f/'identity_mapping.json')
        q['primary_metrics'][STAGE]=deepcopy(q['primary_metrics'][ENTRY]);q['method_folders'][STAGE]=q['method_folders'][ENTRY]
        q['boundary_B']=q['bridge_input']['B'];q['entry']=read(f/'entry.json')
        q['selector_exposure']={n:selector_exposure(read(Path(q['method_folders'][n])/'rollout.json'),mapping[ENTRY if n==STAGE else n]) for n in ORDER}
        sources[sid]=q
    return dict(sources=sources,selected_sources=SELECTED,cross_source=classify(sources))


def test_three_PNGs_explicit_nulls_no_html(tmp_path):
    s=plot_fixture();figures=report.compact_figures(s,tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS) and len(figures)==3
    assert figures[1]['numeric_sidecar']['original_FRESH_endpoint_dwell_s'][STAGE][2] is None
    assert all((tmp_path/'figures'/f['file']).read_bytes().startswith(b'\x89PNG') for f in figures)
    assert not list(tmp_path.rglob('*.html'))


def test_actual_B_to_entry_edge_is_safety_checked():
    from shapely.geometry import box
    from reconciliation.gp_se2_environment import HospitalEnvironment
    env=HospitalEnvironment(box(.9,-.1,1.1,.1),box(-3,-3,5,3))
    geometry=dict(base=env,on=env,cart=None)
    suffix=np.array([[2.,0.,0.],[3.,0.,0.]])
    assert runner.reference_safety(suffix,geometry)['clearance_valid']
    full=np.vstack([[0.,0.,0.],suffix])
    assert not runner.reference_safety(full,geometry)['clearance_valid']
    assert runner.reference_safety(full,geometry)['first_minimum_segment']==0


def test_exact_start_vertex_single_copy():
    native=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
    entry=dict(correspondence=dict(target_world=[0.,0.,0.],remaining_arc_m=2.,segment=0,alpha=0.),row_original_identities=[0,1,2])
    w,l,labels=staged_reference(native,native,[0,0,0],[-1,0,0],entry,native)
    assert labels==['B','E*','F_1','F_2'] and w[1:].tobytes()==native.tobytes()


def test_tampered_original_suffix_rejected():
    f=Path(SELECTED[0]['folder']);refs=read(f/'references.json');c=read(f/'common_state.json');entry=read(f/'entry.json')
    native=np.asarray(read(Path(read(f/'reuse.json')['methods'][NATIVE]['folder'])/'restoration.json')['installed_world'])
    suffix=np.load(f/'suffix_native_installed.npy');suffix[-1,0]+=.001
    with pytest.raises(AssertionError):staged_reference(native,np.load(refs[NATIVE]['local_path']),c['fresh_capture_pose'],c['B'],entry,suffix)


def test_historical_first_legal_state_equal_not_assumed_B():
    for spec in SELECTED:
        f=Path(spec['folder']);reuse=read(f/'reuse.json');c=read(f/'common_state.json');rows=[]
        for n in HISTORICAL:
            r=read(Path(reuse['methods'][n]['folder'])/'rollout.json')
            rows.append(next(e for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command'))
        assert all(e['input_pose']==rows[0]['input_pose'] and e['input_state_id']==rows[0]['input_state_id'] for e in rows)
        assert (rows[0]['input_pose']==c['B'])==(spec['label']=='S4')
