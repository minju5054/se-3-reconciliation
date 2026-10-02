"""Synthetic tests and authenticated saved reads; no new scientific solves."""
import ast
from copy import deepcopy
import inspect
import json
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_spatial_entry_suffix_execution01 as runner
import validate_spatial_entry_suffix_execution01 as validator
import test_relative_factor_multisource01 as old
from reconciliation.spatial_entry_suffix import *
from reconciliation.spatial_correspondence_selector import official_selector
from reconciliation.join_source03 import read,save,sha
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256

def world_trajectory_to_local(a, f):
    return np.asarray([relative_pose(a, p) for p in f])

PREP=ROOT/'data/spatial_entry_suffix_execution_01/primary_20261002T000000Z'
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
SELECTED=read(PREP/'selected_sources.json')
DIAGNOSTIC=read(ROOT/next(iter(CFG['historical_results'])))
MPC=Path('/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/vln_mujoco/mpc.py')


@pytest.mark.parametrize('spec',SELECTED,ids=SOURCE_IDS)
def test_exact_frozen_C3_source_schedule_reuse_and_official_restoration(spec):
    f=Path(spec['folder']);c=read(f/'common_state.json');refs=read(f/'references.json')
    fresh=np.load(refs[NATIVE]['world_path']);raw=np.load(refs[NATIVE]['local_path'])
    before=fresh.copy(),raw.copy()
    frozen=DIAGNOSTIC['summary']['sources'][spec['id']]['correspondences']['C3_FORWARD_SE2']
    world,local,d=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],frozen)
    assert d==refs[ENTRY]['descriptor'] and len(world) in [8,9]
    np.testing.assert_array_equal(world,np.load(refs[ENTRY]['world_path']))
    np.testing.assert_allclose(world[0],SpatialCurve(fresh).pose(frozen['arc_m']),atol=0,rtol=0)
    np.testing.assert_array_equal(fresh,before[0]);np.testing.assert_array_equal(raw,before[1])
    assert not np.allclose(local_trajectory_to_world(c['B'],local),world)
    pre=read(f/'restoration_preflight.json')
    assert pre['passed'] and pre['numerical_MPC_calls']==0
    assert pre['rows'][0]['generation']==c['original_generation']
    assert pre['rows'][0]['previous_control']==c['u_mem_B']
    reuse=read(f/'reuse.json')
    for p,r in reuse['copies'].items():assert sha(f/p)==r['sha256']==sha(r['path'])
    for name in [NATIVE,FULL]:
        out=Path(reuse['methods'][name]['folder'])
        assert sha(out/'hashes.json')==reuse['methods'][name]['hashes_sha256']
        for p,h in read(out/'hashes.json').items():assert sha(out/p)==h
        assert refs[name]==reuse['methods'][name]['reference']
    native=np.asarray(read(Path(reuse['methods'][NATIVE]['folder'])/'restoration.json')['installed_world'])
    for j,identity in enumerate(d['row_original_identities']):
        if identity is not None:
            assert identity>=frozen['segment']+1
            np.testing.assert_array_equal(world[j],fresh[identity])
            np.testing.assert_array_equal(local[j],raw[identity])
            np.testing.assert_array_equal(pre['rows'][0]['installed_world'][j],native[identity])
    assert d['row_original_identities'][0] is None
    assert refs[ENTRY]['safety']['clearance_valid']
    assert read(f/'protocol.json')==CFG==read(PREP/'protocol.json')


def test_historical_authentication_and_all_five_saved_validators():
    runner.authenticate(CFG,deep=True)
    bad=deepcopy(CFG);key=next(iter(bad['historical_results']));bad['historical_results'][key]='0'*64
    with pytest.raises(AssertionError):runner.authenticate(bad)


@pytest.mark.parametrize('n',[2,3,5,10,25])
def test_generic_suffix_official_H5_and_no_raw_mutation(n):
    f=np.c_[np.linspace(0,2,n),np.zeros(n),np.zeros(n)];a=np.array([-.3,.2,.1])
    raw=world_trajectory_to_local(a,f);before=raw.copy()
    w,l,d=suffix_reference(f,raw,a,[.1,.05,0])
    assert len(w)>=2 and d['row_original_arc_m'][0]>=.1-1e-12
    np.testing.assert_array_equal(raw,before)
    fn,meta=official_selector(MPC)
    selected=fn(w,np.array([.1,.05,0]),horizon=5,weights=(10.,10.,1.))
    assert np.asarray(selected).shape==(5,3) and meta['source_sha256']==PINNED_MPC_SHA256


@pytest.mark.parametrize('x,count,first',[(0.,3,0),(1.,2,1),(1.-5e-13,2,1)])
def test_duplicate_vertex_one_original_copy(x,count,first):
    f=np.array([[0.,0,0],[1.,0,0],[2.,0,0]])
    w,l,d=suffix_reference(f,f,[0,0,0],[x,0,0])
    assert len(w)==count and d['row_original_identities'][0]==first
    np.testing.assert_array_equal(w,f[first:]);np.testing.assert_array_equal(l,f[first:])


def test_final_vertex_ineligible_and_C3_mismatch_rejected():
    f=np.array([[0.,0,0],[1.,0,0]])
    with pytest.raises(ValueError,match='positive'):suffix_reference(f,f,[0,0,0],[1,0,0])
    with pytest.raises(AssertionError,match='frozen'):suffix_reference(f,f,[0,0,0],[.2,0,0],{})


def test_shortest_angle_entry_and_A_frame():
    f=np.array([[0.,0,np.deg2rad(170)],[1.,0,np.deg2rad(-170)],[2.,0,np.deg2rad(-160)]])
    a=[2.,3.,.4];raw=world_trajectory_to_local(a,f)
    w,l,d=suffix_reference(f,raw,a,[.5,0,np.pi])
    assert w[0,0]==pytest.approx(.5) and abs(w[0,2])==pytest.approx(np.pi)
    restored=local_trajectory_to_world(a,l)
    np.testing.assert_allclose(restored[:,:2],w[:,:2],rtol=0,atol=1e-12)
    np.testing.assert_allclose(wrap_angle(restored[:,2]-w[:,2]),0,rtol=0,atol=1e-12)
    assert 'time' not in inspect.signature(suffix_reference).parameters


def endpoint_fixture(good,t=None,yaw=0.):
    t=np.arange(len(good))*.1 if t is None else np.asarray(t)
    p=np.zeros((len(t),3));p[:,0]=np.where(good,.05,.2);p[:,2]=yaw
    return t,p,np.zeros(3),np.tile([.4,.1],(len(t)-1,1))


@pytest.mark.parametrize('good,expect,status',[
    ([True]*4,0.,'OBSERVED_ORIGINAL_FRESH_ENDPOINT_DWELL'),
    ([False,True,True,True,True],.1,'OBSERVED_ORIGINAL_FRESH_ENDPOINT_DWELL'),
    ([False,True,True,True],None,'ENDPOINT_TUBE_DWELL_RIGHT_CENSORED'),
    ([False,True,True,False,False],None,'TRANSIENT_ENDPOINT_TUBE_ENTRY_THEN_EXIT'),
    ([False]*5,None,'NO_ENDPOINT_TUBE_ENTRY_OBSERVED')])
def test_endpoint_execution_dwell_null_and_censor(good,expect,status):
    args=endpoint_fixture(good);r=endpoint_dwell(*args,attachment_s=.05)
    assert r['original_FRESH_endpoint_dwell_s']==expect and r['endpoint_observation_status']==status
    assert r['T_post_attach_s']==(None if expect is None else expect-.05)
    if expect is not None:
        assert r['executed_path_length_to_endpoint_dwell_m']==pytest.approx(.4*expect)
        assert r['mean_abs_commanded_v_before_endpoint_dwell_mps']==(None if expect==0 else pytest.approx(.4))
    assert endpoint_dwell(*args)['T_post_attach_s'] is None


def test_endpoint_dwell_threshold_wrap_and_offgrid_execution_time():
    t,p,e,u=endpoint_fixture([True]*5,[0,.07,.16,.29,.31],yaw=2*np.pi)
    assert endpoint_dwell(t,p,e,u)['original_FRESH_endpoint_dwell_s']==0.
    p[-1,0]=.10000001
    assert endpoint_dwell(t,p,e,u)['original_FRESH_endpoint_dwell_s'] is None
    p[:,:2]=[.1,0];p[:,2]=np.pi/12-1e-12
    assert endpoint_dwell(t,p,e,u)['original_FRESH_endpoint_dwell_s']==0.
    p[:,2]+=1e-7
    assert endpoint_dwell(t,p,e,u)['original_FRESH_endpoint_dwell_s'] is None
    assert 'waypoint' not in inspect.signature(endpoint_dwell).parameters


@pytest.mark.parametrize('t',[[0,0,.3],[0,.2,.1],[0,float('nan'),.3]])
def test_invalid_execution_time_rejected(t):
    with pytest.raises(ValueError):first_dwell_index(t,[True]*len(t))


def entry_setup(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene=old.setup(tmp_path,monkeypatch,bt)
    refs[ENTRY]=deepcopy(refs[NATIVE]);refs[ENTRY]['world_path']=str(tmp_path/'references/entry.npy')
    fresh=np.load(refs[NATIVE]['world_path'])
    w,l,d=suffix_reference(fresh,np.load(refs[NATIVE]['local_path']),c['fresh_capture_pose'],c['B'])
    np.save(refs[ENTRY]['world_path'],w);refs[ENTRY]['world_sha256']=sha(refs[ENTRY]['world_path'])
    refs[ENTRY]['local_path']=str(tmp_path/'references/entry_local.npy');np.save(refs[ENTRY]['local_path'],l)
    refs[ENTRY]['local_sha256']=sha(refs[ENTRY]['local_path'])
    refs[ENTRY]['safety']=runner.reference_safety(w,env)
    (tmp_path/'references.json').write_text(json.dumps(refs))
    return c,refs,manifest,env,scene,d


@pytest.mark.parametrize('bt',[92,997,1804,1524])
def test_same_executor_schedule_memory_generation_and_primary_parity(tmp_path,monkeypatch,bt):
    c,refs,manifest,env,scene,d=entry_setup(tmp_path,monkeypatch,bt)
    assert runner.legacy.run_method is old.runner.run_method
    rollouts={}
    for name in ORDER:
        runner.legacy.run_method(tmp_path,name)
        out=tmp_path/'methods'/name;r=read(out/'rollout.json');m=read(out/'metrics.json');rollouts[name]=r
        assert len(r['commands'])==180 and r['new_MPC_solved']==30 and r['error'] is None
        _,_,v=validator.validate_method(tmp_path,name,refs[name],c,np.load(refs[NATIVE]['world_path']),env['on'],scene,manifest['mpc']['official_settings'])
        assert v['valid']
        rows=execution_metrics(r,m,np.load(refs[NATIVE]['world_path']))
        for key,val in metric_row(m,r['termination']).items():assert rows[key]==val
        assert rows['original_FRESH_endpoint_dwell_s']==validator.independent_endpoint_time(r,np.load(refs[NATIVE]['world_path']))
        previous=c['u_mem_B']
        for event in r['events']:
            if event.get('status')=='submitted':assert event['previous_command']==previous
            if event.get('type')=='solve_result':previous=event['command']
        assert all(w['pose_before']==w['pose_after'] and w['sim_before']==w['sim_after'] for w in r['logical_waits'])
    gate=schedule_gate(rollouts,read(tmp_path/'schedule.json'),c);assert gate['passed']
    sel=selector_progress(rollouts[ENTRY],d);assert not sel['any_identity_before_entry']
    assert min(a for row in sel['rows'] for a in row['selected_original_arcs_m'])>=d['first_presented_original_arc_m']
    own=read(tmp_path/'methods'/ENTRY/'own_reference_metrics.json')
    assert own!=read(tmp_path/'methods'/ENTRY/'metrics.json')
    rollouts[ENTRY]['submit_requests'][0]['tick']+=1
    assert not schedule_gate(rollouts,read(tmp_path/'schedule.json'),c)['passed']


def test_logical_release_wait_stale_generation_and_no_early_application():
    old.test_release_and_generation_existing_path_unchanged()


def test_entry_unsafe_reference_and_abort_guard_preserve_prefix(tmp_path,monkeypatch):
    _,refs,_,_,_,_=entry_setup(tmp_path,monkeypatch,997)
    original=old.runner.guard_check;calls=[]
    def guard(*a):
        result=original(*a);calls.append(1)
        if len(calls)==6:result['safe']=False
        return result
    monkeypatch.setattr(old.runner,'guard_check',guard);old.runner.run_method(tmp_path,ENTRY)
    r=read(tmp_path/'methods'/ENTRY/'rollout.json')
    assert len(r['commands'])==5 and not r['safety_abort']['command_applied']
    assert r['events'][-1]['withheld_by_logical_scheduler']
    (tmp_path/'methods'/ENTRY).rename(tmp_path/'methods'/'ABORT_FIXTURE')
    refs[ENTRY]['safety']['clearance_valid']=False
    (tmp_path/'references.json').write_text(json.dumps(refs))
    monkeypatch.setattr(old.runner,'Worker',lambda *a:pytest.fail('unsafe reference started controller'))
    with pytest.raises(AssertionError):old.runner.run_method(tmp_path,ENTRY)


def synthetic_sources():
    sources={}
    for i in range(4):
        metrics={name:dict(position_auc_09_m_s=.2 if name!=ENTRY else .1,sustained_attachment_s=1.,
            original_FRESH_endpoint_dwell_s=2.,remaining_arc_at_attachment_m=.3,execution_clearance_lower_bound_m=.2) for name in ORDER}
        sources[f'S{i}']=dict(primary_metrics=metrics,reference_safety={ENTRY:dict(clearance_valid=True)},
            termination={ENTRY:'OBSERVATION_CAP'},selector={ENTRY:dict(any_identity_before_entry=False)},schedule_gate=dict(passed=True))
    return sources


def test_predeclared_categories_and_no_cap_imputation():
    s=synthetic_sources();assert comparison(s)['classification']=='ENTRY_TRANSITION_AND_COMPLETION_SUPPORTED'
    e=s['S0']['primary_metrics'][ENTRY];e['sustained_attachment_s']=.5;e['original_FRESH_endpoint_dwell_s']=2.5
    assert comparison(s)['classification']=='FAST_ATTACHMENT_SLOW_COMPLETION_TRADEOFF'
    e['original_FRESH_endpoint_dwell_s']=None
    assert comparison(s)['pairwise']['S0'][NATIVE]['endpoint_lost']
    s['S0']['reference_safety'][ENTRY]['clearance_valid']=False
    assert comparison(s)['classification']=='ENTRY_REFERENCE_FAILURE'
    s['S0']['selector'][ENTRY]['any_identity_before_entry']=True
    assert comparison(s)['classification']=='TECHNICAL_BLOCKED'
    s=synthetic_sources()
    for q in s.values():
        for row in q['primary_metrics'].values():row['original_FRESH_endpoint_dwell_s']=None
    assert comparison(s)['classification']=='ENTRY_PROGRESS_INSUFFICIENT'
    s=synthetic_sources()
    for q in s.values():q['primary_metrics'][ENTRY]['original_FRESH_endpoint_dwell_s']=2.2
    assert comparison(s)['classification']=='ENTRY_PROGRESS_INSUFFICIENT'


def test_larger_remaining_arc_tradeoff_and_observed_denominator():
    s=synthetic_sources();e=s['S0']['primary_metrics'][ENTRY]
    e['sustained_attachment_s']=.8;e['remaining_arc_at_attachment_m']=.5
    assert comparison(s)['classification']=='FAST_ATTACHMENT_SLOW_COMPLETION_TRADEOFF'
    s=synthetic_sources()
    for sid in ['S1','S2','S3']:
        for row in s[sid]['primary_metrics'].values():row['original_FRESH_endpoint_dwell_s']=None
    s['S0']['primary_metrics'][ENTRY]['original_FRESH_endpoint_dwell_s']=None
    assert comparison(s)['systematically_later_endpoint']


def test_saved_validator_report_no_scientific_calls_and_optimizer_guard():
    for role in ['validate','report']:
        tree=ast.parse((ROOT/f'scripts/{role}_spatial_entry_suffix_execution01.py').read_text())
        names={n.func.id if isinstance(n.func,ast.Name) else n.func.attr if isinstance(n.func,ast.Attribute) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
        assert not names&{'solve_least_squares','solve_condition','run_method','execute','Worker','submit','load_official'}
    def solve_condition():pytest.fail('optimizer entered')
    with pytest.raises(RuntimeError,match='forbidden'):
        with no_reconciliation_optimizer():solve_condition()
    assert sha(MPC)==PINNED_MPC_SHA256


def test_exactly_four_PNGs_nulls_and_numeric_sidecars(tmp_path,monkeypatch):
    import report_spatial_entry_suffix_execution01 as report
    # Repeated saved R00 curves are only a renderer fixture, never new evidence.
    selected=[];sources={}
    original=Path(SELECTED[0]['folder']);reuse=read(original/'reuse.json')
    for i in range(4):
        folder=tmp_path/f'S{i}';folder.mkdir()
        for file in ['references.json','common_state.json','recorded_old_to_B.npy','reuse.json']:
            (folder/file).write_bytes((original/file).read_bytes())
        (folder/'methods').mkdir();(folder/'methods'/ENTRY).symlink_to(reuse['methods'][NATIVE]['folder'],target_is_directory=True)
        q=dict(entry=read(original/'entry.json'),primary_metrics={},selector={},termination={},max_execution_XY_gap_m={NATIVE:0.,FULL:0.})
        for name in ORDER:
            # Only frozen primary rows are used; endpoint outcomes here are explicitly synthetic.
            baseline=NATIVE if name==ENTRY else name
            row=deepcopy(read(Path(reuse['methods'][baseline]['folder'])/'metrics.json'))
            q['primary_metrics'][name]=metric_row(row,'OBSERVATION_CAP')
            q['primary_metrics'][name].update(original_FRESH_endpoint_dwell_s=None if i==3 else 2.,attachment_original_arc_m=.4,
                endpoint_dwell_projected_original_arc_m=None if i==3 else 1.3)
            q['selector'][name]=dict(first=dict(first_selected_original_arc_m=.3))
            q['termination'][name]='OBSERVATION_CAP'
        selected.append(dict(id=f'S{i}',folder=str(folder)));sources[f'S{i}']=q
    monkeypatch.setattr(report,'geometry',lambda _:runner.geometry(original))
    records=report.compact_figures(dict(selected_sources=selected,sources=sources),tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert all((tmp_path/'figures'/r['file']).read_bytes().startswith(b'\x89PNG') for r in records)
    assert len(records[3]['numeric_sidecar']['censored'])==3
    assert all(r['T_endpoint'] is None for r in records[3]['numeric_sidecar']['censored'])
