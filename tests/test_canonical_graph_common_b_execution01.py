"""Synthetic fixtures and saved authentication only; no scientific solves."""
import ast
import json
from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_canonical_graph_common_b_execution01 as runner
import validate_canonical_graph_common_b_execution01 as validator
import report_canonical_graph_common_b_execution01 as reporter
import test_relative_factor_multisource01 as old
from reconciliation.canonical_graph_execution01 import *
from reconciliation.join_source03 import read,save,sha
from reconciliation.spatial_entry_suffix import endpoint_dwell
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from shapely.geometry import box
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
RUN=ROOT/'data/canonical_graph_common_b_execution_01/primary_20261006'


@pytest.mark.parametrize('n',[2,3,10,19])
@pytest.mark.parametrize('prefix',['F','X'])
def test_exact_arrays_frames_generic_N_no_mutation(n,prefix):
    A=np.array([3.,4.,.4]);B=np.array([3.5,4.2,.9]);raw=np.column_stack([np.arange(n)*.2,np.arange(n)*.01,np.arange(n)*.02])
    world=local_trajectory_to_world(A,raw);before=(raw.tobytes(),world.tobytes(),B.tobytes())
    w,l,labels=boundary_wrapper(A,B,world,raw,prefix)
    assert w[0].tobytes()==B.tobytes() and w[1:].tobytes()==world.tobytes() and l[1:].tobytes()==raw.tobytes()
    assert before==(raw.tobytes(),world.tobytes(),B.tobytes())
    np.testing.assert_array_equal(l[0],relative_pose(A,B))
    np.testing.assert_allclose(local_trajectory_to_world(A,l),w,atol=1e-12,rtol=0)
    assert not np.allclose(local_trajectory_to_world(B,l),w)
    entry=dict(correspondence=dict(segment=0,alpha=.5,arc_m=.1))
    ids=identities(labels,entry,world);assert ids[0]['original_raw_id'] is None and ids[0]['original_arc_m'] is None
    assert [r['original_raw_id'] for r in ids[1:]]==list(range(n))
    np.testing.assert_allclose([r['original_arc_m'] for r in ids[1:]],np.r_[0,np.cumsum(np.linalg.norm(np.diff(world[:,:2],axis=0),axis=1))])


@pytest.mark.parametrize('bad',[np.zeros((1,3)),np.zeros((2,2)),np.full((2,3),np.nan)])
def test_bad_reference_fails(bad):
    with pytest.raises(ValueError):boundary_wrapper([0,0,0],[0,0,0],bad,bad,'F')


@pytest.mark.parametrize('prefix',['F','X'])
def test_connector_safety_checked(prefix):
    # The raw/canonical polyline itself is safe, but the explicit B connector crosses an obstacle.
    env=HospitalEnvironment(box(-.1,-.1,.1,.1),box(-5,-5,5,5))
    fresh=np.array([[1.,0.,0.],[2.,0.,0.]])
    w,_,_=boundary_wrapper([0,0,0],[-1,0,0],fresh,fresh,prefix)
    scene=dict(on=env,cart=None)
    assert runner.reference_safety(fresh,scene)['clearance_valid']
    assert not runner.reference_safety(w,scene)['clearance_valid']
    assert len(runner.reference_safety(w,scene)['per_segment_clearance_m'])==2


@pytest.mark.parametrize('name',['solve_least_squares','solve_graph','solve_variant','solve_half','solve_condition','infer','capture_rgb','scan'])
@pytest.mark.parametrize('saved_only',[False,True])
def test_optimizer_and_acquisition_guard(name,saved_only,monkeypatch):
    namespace={};exec('def '+name+'():\n raise AssertionError("body entered")',namespace)
    monkeypatch.setattr(runner,name,namespace[name],raising=False);before=sys.getprofile()
    with pytest.raises(RuntimeError,match='execution guard'):
        with execution_guard(saved_only):getattr(runner,name)()
    assert sys.getprofile() is before


@pytest.mark.parametrize('name',['run_method','preflight','solve_mpc','Popen'])
def test_saved_only_no_controller_calls(name):
    ns={};exec('def '+name+'(): pass',ns)
    with pytest.raises(RuntimeError):
        with execution_guard(True):ns[name]()


@pytest.mark.parametrize('label',['E1','E2','E3','E4'])
def test_prepared_authentication_B_entry_mapping_schedule_and_official_unchanged(label):
    f=RUN/'sources'/label
    assert validator.reference_audit(f)['valid']
    c=read(f/'common_state.json');oldf=ROOT/CFG['historical']['run']/'sources'/label
    assert sha(f/'schedule.json')==sha(oldf/'schedule.json') and sha(f/'common_state.json')==sha(oldf/'common_state.json')
    assert read(f/'identity_mapping.json')[ENTRY]==read(oldf/'final_identity_mapping.json')[ENTRY]
    for row in read(f/'restoration_preflight.json')['rows']:
        historical=read(oldf/'methods'/ENTRY/'restoration.json')
        for k in ['B','B_tick','B_sim_s','held_command','previous_control','generation','next_submit_tick','capture_pose','reference_version']:
            assert row[k]==historical[k]
    assert sha(read(f/'source_manifest.json')['mpc']['mpc_source'])==PINNED_MPC_SHA256
    for n,r in read(f/'reuse.json').items():
        assert sha(Path(r['folder'])/'hashes.json')==r['hashes_sha256']
        assert not (f/'methods'/n).exists()


def test_authority_hash_tamper_fails():
    cfg=deepcopy(CFG);cfg['canonical']['summary_sha256']='0'*64
    with pytest.raises(AssertionError):runner.authenticate(cfg)


def source():
    def method():return dict(metrics={ATTACH:1.,END:2.,AUC:.1,'linear_command_TV':1.,'angular_command_TV':2.},valid=True,unsafe_reference=False,safety_abort=False)
    return dict(integration_dt_s=1/60,technical_valid=True,methods={n:method() for n in CENTRAL})


@pytest.mark.parametrize('case,expected',[
 ('technical','TECHNICAL_BLOCKED'),('unsafe','CANONICAL_INTERFACE_NOT_EXECUTABLE'),
 ('timing','CANONICAL_EXECUTION_GAIN_SUPPORTED'),('recovery','CANONICAL_EXECUTION_GAIN_SUPPORTED'),
 ('early','CANONICAL_EXECUTION_GAIN_SUPPORTED'),('same','STAGING_REMAINS_SUFFICIENT'),
 ('tradeoff','CANONICAL_EXECUTION_TRADEOFF'),('regression','CANONICAL_EXECUTION_REGRESSION'),('mixed','MIXED_EXECUTION_EVIDENCE')])
def test_all_categories(case,expected):
    s={str(i):source() for i in range(4)}
    for q in list(s.values())[:2]:
        x=q['methods'][CANONICAL];b=q['methods'][ENTRY];m=x['metrics']
        if case=='technical':q['technical_valid']=False
        if case=='unsafe':x.update(unsafe_reference=True,valid=False,metrics=None)
        if case in ['timing','tradeoff']:m[ATTACH]-=1/60
        if case=='tradeoff':m[END]+=.1
        if case=='regression':m[END]=None
        if case=='early':b['metrics'][ATTACH]=m[ATTACH]=0.;m[AUC]=.09
        if case=='recovery':b['metrics'][ATTACH]=None
        if case=='mixed':b['metrics'][END]=m[END]=None
    assert classify(s,CFG['classification_rules'])['classification']==expected


def test_thresholds_zero_attachment_endpoint_gate_and_command_tradeoff():
    q=source();b=q['methods'][ENTRY]['metrics'];x=q['methods'][CANONICAL]['metrics'];r=CFG['classification_rules']
    b[ATTACH]=x[ATTACH]=0.;x[AUC]=b[AUC]-1e-8
    assert source_outcome(q,r)['positive'] is None
    x[AUC]=.09;assert source_outcome(q,r)['positive']=='CANONICAL_EARLY_TRACKING_GAIN'
    x[END]+=q['integration_dt_s'];assert source_outcome(q,r)['positive'] is not None
    x[END]+=2e-9;assert source_outcome(q,r)['positive'] is None
    assert source_outcome(q,r)['repeated_tradeoff_candidate']
    x[END]=b[END];x['angular_command_TV']=2.3
    assert source_outcome(q,r)['command_TV_worse']==['angular_command_TV']
    b[AUC]=.005;x[AUC]=.0045
    assert source_outcome(q,r)['positive'] is None  # 10% but insufficient absolute reduction


@pytest.mark.parametrize('bt',[188,673,189,649])
def test_new_wrappers_use_unchanged_synthetic_runtime_schedule_guard_and_metrics(tmp_path,monkeypatch,bt):
    c,refs,m,env,scene=old.setup(tmp_path,monkeypatch,bt)
    refs[RAW]=deepcopy(refs['M0_NATIVE']);refs[CANONICAL]=deepcopy(refs['M0_NATIVE']);(tmp_path/'references.json').write_text(json.dumps(refs))
    assert runner.runtime.run_method is old.runner.run_method
    for n in NEW:
        with execution_guard():runner.runtime.run_method(tmp_path,n)
        r=read(tmp_path/'methods'/n/'rollout.json');metric=read(tmp_path/'methods'/n/'metrics.json')
        _,_,a=validator.validate_method(tmp_path,n,refs[n],c,np.load(refs[n]['world_path']),env['on'],scene,m['mpc']['official_settings'])
        assert a['valid'] and validator.schedule_check(r,read(tmp_path/'schedule.json'),c)['passed']
        assert all(w['pose_before']==w['pose_after'] and w['sim_before']==w['sim_after'] for w in r['logical_waits'])
        d=command_metrics(r);assert d['first_new_application_tick']==c['next_submit_after_B']+1
        value=np.asarray(d['first_new_applied_command'])
        np.testing.assert_array_equal(d['delta_from_physical_u_minus'],value-c['u_minus'])
        np.testing.assert_array_equal(d['delta_from_previous_control'],value-c['u_mem_B'])
        row=validator.execution_metrics(r,metric,np.load(refs[n]['world_path']))
        assert row[END]==validator.independent_endpoint_time(r,np.load(refs[n]['world_path']))
    old.test_release_and_generation_existing_path_unchanged()


def test_abort_command_unapplied_early_null(tmp_path,monkeypatch):
    c,refs,m,env,scene=old.setup(tmp_path,monkeypatch,188);refs[RAW]=deepcopy(refs['M0_NATIVE']);(tmp_path/'references.json').write_text(json.dumps(refs))
    checks=[];guard=runner.runtime.guard_check
    def reject(*args):
        v=guard(*args);checks.append(1)
        if len(checks)==3:v['safe']=False
        return v
    monkeypatch.setattr(runner.runtime,'guard_check',reject)
    runner.runtime.run_method(tmp_path,RAW);r=read(tmp_path/'methods'/RAW/'rollout.json')
    assert len(r['commands'])==2 and not r['safety_abort']['command_applied']
    d=command_metrics(r);assert d['first_new_applied_command'] is None and d['early_03_linear_TV'] is None
    assert validator.schedule_prefix_audit(r,read(tmp_path/'schedule.json'))


@pytest.mark.parametrize('duration,expected',[(.29,None),(.3,0.),(.6,0.)])
def test_endpoint_null_execution_time_parity(duration,expected):
    r=endpoint_dwell(np.linspace(0,duration,31),np.zeros((31,3)),[0,0,0],np.zeros((30,2)),None)
    assert r[END]==expected and r['T_post_attach_s'] is None


def test_static_no_solve_historical_rerun_and_frozen_code():
    for p in [Path(runner.__file__),Path(validator.__file__),Path(reporter.__file__)]:
        tree=ast.parse(p.read_text());calls=[n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))]
        assert not set(calls)&{'solve_least_squares','solve_graph','solve_variant','correspondences','scan','capture_rgb'}
        if p!=Path(runner.__file__):assert not set(calls)&{'run_method','preflight','Worker'}
    text=Path(runner.__file__).read_text();assert 'for n in NEW:' in text and "open('x')" in text
    assert runner.NEW==[RAW,CANONICAL] and len(CFG['sources'])==4


def test_exactly_three_png_fixture_nulls_and_secondary_independence(tmp_path):
    geometry={};metrics={}
    for label in ['E1','E2','E3','E4']:
        raw=[[0.,0.,0.],[1.,.1,.1],[2.,.2,.2]]
        geometry[label]=dict(raw=raw,canonical=raw,B=[-.1,0,0],entry=[.5,.05,.05],references={n:[[-.1,0.,0.],*raw] for n in CENTRAL})
        q=source();methods=q['methods']
        for n in CENTRAL:methods[n]['metrics']['remaining_arc_at_attachment_m']=.8
        methods[CANONICAL]['metrics'][END]=None
        metrics[label]=dict(methods=methods,commands={n:dict(delta_from_physical_u_minus=[.1,.2]) for n in CENTRAL},pairwise={})
    data=dict(geometry=geometry,metrics=metrics);original=deepcopy(data)
    reporter.render(data,tmp_path);assert data==original
    assert sorted(p.name for p in tmp_path.iterdir())==sorted(PNGS)


def test_selector_X_original_identity_and_primary_own_targets_separate():
    f=ROOT/CFG['historical']['run']/'sources/E1';r=read(f/'methods'/ENTRY/'rollout.json')
    labels=read(f/'references.json')[ENTRY]['labels'];labels=[s.replace('F_','X_') for s in labels]
    fresh=np.load(f/'references/M0_NATIVE_world.npy');mapping=identities(labels,read(f/'entry.json'),fresh)
    result=selectors(r,mapping)
    for row in result['rows']:
        for identity in row['H5_original_metadata']:
            if identity['label'].startswith('X_'):assert identity['original_raw_id']==int(identity['label'][2:])
    old.historical.test_primary_attachment_unchanged_own_reference_separate()


def test_saved_only_validator_runs_without_MPC_or_optimizer(tmp_path,monkeypatch):
    # Empty synthetic source set exercises validator fail-closed accounting without scientific execution.
    save(tmp_path/'protocol.json',CFG);save(tmp_path/'selected_sources.json',[])
    monkeypatch.setattr(validator,'authenticate',lambda *a:None);monkeypatch.setattr(validator,'verify',lambda *a:None)
    monkeypatch.setattr(runner.runtime,'run_method',lambda *a:pytest.fail('scientific rollout'))
    with pytest.raises(AssertionError):validator.validate(tmp_path)
