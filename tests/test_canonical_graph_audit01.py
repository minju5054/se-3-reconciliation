"""Saved geometry, accounting, no-execution guards and compact report fixtures."""
import ast
from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).parents[1];sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_canonical_se2_graph_formulation_audit01 as runner
import validate_canonical_se2_graph_formulation_audit01 as validator
import report_canonical_se2_graph_formulation_audit01 as reporter
from reconciliation.canonical_graph_audit01 import *
from reconciliation.canonical_se2_graph import CanonicalSE2Problem
from reconciliation.join_source03 import read,sha
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
RUN=ROOT/'data/canonical_se2_graph_formulation_audit_01/primary_20261006'


@pytest.mark.parametrize('name',['run_method','run_rollout','solve_mpc','_solve','infer','capture_rgb','get_rgb','collect_source','integrate_unicycle','scan','preflight'])
def test_no_execution_guard_monkeypatched_entrypoints(name,monkeypatch):
    namespace={};exec('def '+name+'():\n    raise AssertionError("body executed")',namespace)
    monkeypatch.setattr(runner,name,namespace[name],raising=False)
    previous=sys.getprofile()
    with pytest.raises(RuntimeError,match='planning-only guard'):
        with planning_guard(allow_optimizer=True):getattr(runner,name)()
    assert sys.getprofile() is previous


@pytest.mark.parametrize('module,name',[('official_mpc_worker','solve'),('lightnav.inference','forward'),('isaacsim','step'),('torch','forward')])
def test_module_guards(module,name):
    ns={'__name__':module};exec('def '+name+'(): pass',ns)
    with pytest.raises(RuntimeError):
        with planning_guard(True):ns[name]()


def test_no_optimizer_in_validator_guard_and_no_subprocess():
    ns={};exec('def solve_least_squares(): pass',ns)
    with pytest.raises(RuntimeError):
        with planning_guard():ns['solve_least_squares']()
    with planning_guard(True):ns['solve_least_squares']()
    import subprocess
    with pytest.raises(RuntimeError):
        with planning_guard(True):subprocess.run(['true'])


def test_projection_not_same_index_ties_wrap_and_variable_N():
    reference=np.array([[0,0,3.1],[1,0,-3.1],[2,0,-3.]])
    samples=np.array([[.5,.2,np.pi],[1.5,.3,-3.05]])
    d=directed_projection(samples,reference)
    assert d['XY_max_m']==pytest.approx(.3);assert d['XY_RMS_m']==pytest.approx(np.sqrt(.065))
    assert d['yaw_max_rad']<1e-14
    with pytest.raises(ValueError):pose_difference(samples,reference)
    duplicated=np.array([[0,0,0],[0,0,1],[1,0,1]])
    assert directed_projection([[0,0,0]],duplicated)['yaw_max_rad']==0


def test_v2_geometry_differences():
    h=np.array([[0,0,0],[.5,.1,.2],[1,0,.4],[2,0,.4]])
    v=h.copy();v[1]+=[.001,.002,.003]
    r=v2_hermite_similarity(h,v,h,v)
    assert r['X1_XY_difference_m']==pytest.approx(np.sqrt(5)*.001)
    assert r['X1_yaw_difference_rad']==pytest.approx(.003)
    assert r['full_installed']['XY_max_m']==pytest.approx(np.sqrt(5)*.001)


@pytest.mark.parametrize('spec',CFG['selected_sources'],ids=lambda s:s['label'])
def test_saved_inputs_C3_frames_baselines_authentication_and_similarity(spec):
    f=RUN/'sources'/spec['label']
    if not f.exists():pytest.skip('prepared sealed inputs unavailable')
    c=read(f/'context.json');old=Path(c['historical_folder']);p=runner.load_problem(f)
    assert sha(c['raw_local_path'])==c['raw_local_sha256'] and sha(c['raw_world_path'])==c['raw_world_sha256']
    assert sha(old/'schedule.json')==c['schedule_sha256'] and sha(old/'common_state.json')==c['common_state_sha256']
    from reconciliation.spatial_entry_suffix import suffix_reference
    _,_,e=suffix_reference(p.fresh,np.load(f/'raw_observation_local.npy'),p.A,p.B,read(old/'entry.json')['correspondence'])
    assert e['correspondence']==read(old/'entry.json')['correspondence']
    refs=read(old/'references.json')
    for n,original in BASELINES.items():
        assert sha(refs[original]['world_path'])==refs[original]['world_sha256']
        assert np.load(f/(n+'.npy')).tobytes()==np.load(refs[original]['world_path']).tobytes()
    r=v2_hermite_similarity(*[np.load(f/(n+'.npy')) for n in ['HERMITE','V2_SINGLE_NODE','HERMITE_installed','V2_SINGLE_NODE_installed']])
    assert r==read(RUN/'v2_hermite_similarity.json')[spec['label']]


def test_runner_one_call_no_retry_and_validator_never_solves():
    text=Path(runner.__file__).read_text();tree=ast.parse(text)
    names=[n.func.id if isinstance(n.func,ast.Name) else n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))]
    assert names.count('solve_least_squares')==1
    assert not set(names)&{'run_method','solve_variant','infer','capture_rgb','scan','preflight'}
    assert 'head==remote' in text and "optimization_start.json" in text
    tree=ast.parse(Path(validator.__file__).read_text())
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='solve_least_squares' for n in ast.walk(tree))


def fixture_source():
    f=np.array([[0,0,0],[.5,.1,.1],[1,.2,.2]])
    p=CanonicalSE2Problem([0,0,0],[.3,0,.1],f)
    x=f.copy();x[:,0]+=[.2,.1,.01]
    b=np.vstack([[.3,0,.1],f])
    d=diagnostics(p,x,{k:b for k in BASELINES},dict(minimum_clearance_m=.5,clearance_valid=True))
    return dict(diagnostics=d,solver=dict(converged=True),technical_failure=None)


def test_classification_precedence_thresholds_and_no_execution_claim():
    sources={k:fixture_source() for k in ['a','b']}
    r=classify(sources,CFG);assert r['classification']=='CANONICAL_DISTINCT_AND_STRUCTURALLY_PLAUSIBLE' and r['direction']=='A'
    assert not r['execution_benefit_demonstrated']
    sources['a']['technical_failure']='parity';assert classify(sources,CFG)['classification']=='TECHNICAL_BLOCKED'
    sources={k:fixture_source() for k in ['a','b']}
    for q in sources.values():q['solver']['converged']=False
    assert classify(sources,CFG)['classification']=='CANONICAL_FORMULATION_INVALID'
    for q in sources.values():q['solver']['converged']=True;q['diagnostics']['rigid_fit']['translation_RMS_m']=0.;q['diagnostics']['rigid_fit']['yaw_RMS_rad']=0.
    assert classify(sources,CFG)['classification']=='CANONICAL_COLLAPSES_TO_RIGID_OR_EXISTING'


def test_exactly_four_PNGs_with_synthetic_saved_figures(tmp_path,monkeypatch):
    sources={};summary=dict(sources={},V2_Hermite_similarity={})
    for label in ['OSA03_R00','E1','E2','E3','E4']:
        r=fixture_source();f=np.array([[0,0,0],[.5,.1,.1],[1,.2,.2]])
        h=np.vstack([[.2,0,.1],[.21,.01,.1],f]);v=h.copy();v[1,0]+=.001
        summary['sources'][label]=r;summary['V2_Hermite_similarity'][label]=v2_hermite_similarity(h,v,h,v)
        sources[label]=dict(A=[0,0,0],B=[.2,0,.1],raw=f.tolist(),transported=(f+[.2,0,0]).tolist(),canonical=(f+[.1,0,0]).tolist(),
            baselines=dict(B_ENTRY=h.tolist(),HERMITE=h.tolist(),V2_SINGLE_NODE=v.tolist()),nodes=r['diagnostics']['nodes'])
    data=dict(sources=sources,summary=summary)
    validator.write_json(tmp_path/'comparison_summary.json',summary)
    monkeypatch.setattr(reporter,'figure_data',lambda *args:data)
    with planning_guard():r=reporter.report(tmp_path,tmp_path/'report')
    assert r['count']==4 and sorted(p.name for p in (tmp_path/'report/figures').iterdir())==sorted(PNGS)
    from PIL import Image
    for p in r['paths']:
        with Image.open(p) as im:assert im.width>=1500 and im.height>=1000


def test_formulation_audit_preexists_core_and_protocol_contract():
    runner.contract(CFG);audit=read(runner.RESULTS/'formulation_audit.json')
    assert audit['written_before_new_implementation']
    methods={x['method']:x for x in audit['methods']}
    assert methods['V2_SINGLE_NODE_GRAPH']['editable_poses']=='1'
    assert methods['CANONICAL_GRAPH_PROPOSED']['editable_poses']=='N'
    assert methods['LOCAL_SE2']['factor_definitions']==methods['CANONICAL_GRAPH_PROPOSED']['factor_definitions']


def test_budget_guard_no_retry():
    ns={};exec('def solve_least_squares(): pass',ns)
    with planning_guard(True,1) as count:
        ns['solve_least_squares']()
        with pytest.raises(RuntimeError,match='budget exceeded'):ns['solve_least_squares']()
    assert count['canonical_optimizer_calls']==2


def test_synthetic_end_to_end_runner_and_saved_validator(tmp_path,monkeypatch):
    """Synthetic fixtures only; never touches or solves sealed scientific inputs."""
    from shapely.geometry import box
    from reconciliation.gp_se2_environment import HospitalEnvironment
    env=dict(base=HospitalEnvironment(box(5,5,6,6),box(-10,-10,10,10)),cart=None)
    env['on']=env['base']
    cfg=deepcopy(CFG);selected=[]
    for i in range(5):
        label=f'toy{i}';f=tmp_path/'sources'/label;f.mkdir(parents=True)
        A=[0,0,0];B=[.2,0,.1];raw=np.array([[0,0,0],[.5,.1,.1],[1,.2,.2]])
        p=CanonicalSE2Problem(A,B,raw)
        for name,array in [('raw_fresh',raw),('raw_observation_local',raw),('transported_target',p.target)]:runner.np_save(f/(name+'.npy'),array)
        h=np.vstack([B,[.3,.03,.05],raw]);v=h.copy();v[1,0]+=.001
        for name,array in [('B_ENTRY',h),('HERMITE',h),('V2_SINGLE_NODE',v),('HERMITE_installed',h),('V2_SINGLE_NODE_installed',v)]:runner.np_save(f/(name+'.npy'),array)
        c=dict(A=A,B=B,historical_folder=str(f),raw_local_path=str(f/'raw_observation_local.npy'),raw_world_path=str(f/'raw_fresh.npy'),
               raw_local_sha256=sha(f/'raw_observation_local.npy'),raw_world_sha256=sha(f/'raw_fresh.npy'))
        runner.save(f/'context.json',c);selected.append(dict(label=label,id=label,folder=str(f)))
    cfg['selected_sources']=selected;runner.save(tmp_path/'protocol.json',cfg);runner.save(tmp_path/'selected_sources.json',selected)
    runner.save(tmp_path/'v2_hermite_similarity.json',{s['label']:v2_hermite_similarity(h,v,h,v) for s in selected})
    monkeypatch.setattr(runner,'verify',lambda *a,**kw:None);monkeypatch.setattr(validator,'verify',lambda *a,**kw:None)
    monkeypatch.setattr(runner,'git',lambda *a:'synthetic-not-science');monkeypatch.setattr(runner,'geometry',lambda *a:env);monkeypatch.setattr(validator,'geometry',lambda *a:env)
    runner.execute(tmp_path)
    with pytest.raises(FileExistsError):runner.execute(tmp_path)
    summary,val=validator.validate(tmp_path)
    assert val['valid'] and summary['call_accounting']['canonical_scientific_solves']==5
    assert summary['call_accounting']['MPC']==0
    monkeypatch.setattr(validator,'RESULTS',tmp_path/'results')
    validator.export(tmp_path,summary,val)
    reporter.report(tmp_path,tmp_path/'results')
    assert validator.check_exports(tmp_path,summary)


def test_saved_hash_tamper_fails_closed(tmp_path):
    cfg=deepcopy(CFG);key=next(iter(cfg['historical_results']));cfg['historical_results'][key]='0'*64
    with pytest.raises(AssertionError):runner.authenticate(cfg)


def test_distinct_no_gain_and_mixed_classifications():
    sources={k:fixture_source() for k in ['a','b']}
    for q in sources.values():q['diagnostics']['endpoint_over_first']=.9
    assert classify(sources,CFG)['classification']=='CANONICAL_DISTINCT_BUT_NO_CLEAR_STRUCTURAL_GAIN'
    assert classify(sources,CFG)['direction']=='B'
    sources['a']['solver']['converged']=False
    assert classify(sources,CFG)['classification']=='MIXED_FORMULATION_EVIDENCE'
