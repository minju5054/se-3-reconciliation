"""Synthetic protocol tests and read-only prepared-source checks; no new science."""
import ast
from copy import deepcopy
from dataclasses import fields
import inspect
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_staging_graph_necessity_eval01 as runner
from reconciliation.staging_graph_necessity_eval01 import *
from reconciliation.staging_eval_sources import exclusion_registry
from reconciliation.boundary_row_ablation04 import staged_reference,identity_mapping
from reconciliation.spatial_entry_suffix import suffix_reference,endpoint_dwell,no_reconciliation_optimizer
from reconciliation.b_to_entry_bridge import hermite_bridge
from reconciliation.b_to_entry_graph_diag02 import sample_hermite
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.se2 import local_trajectory_to_world
from validate_b_to_entry_graph_formulation_diag02 import independent_factors
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
RUN=ROOT/'data/staging_graph_necessity_eval_01/primary_20261006'


def fixture_records(n=8):
    return [SelectionRecord(f'ep{i}/h',f'ep{i}',True,float(n-i),float(i)) for i in range(n)]


def test_deterministic_two_independent_axes():
    rows=fixture_records();x=select(rows)
    assert x['selected_ids']==['ep0/h','ep1/h','ep2/h','ep7/h','ep6/h','ep5/h']
    assert select(list(reversed(rows)))==x and not x['outcome_fields_used']


@pytest.mark.parametrize('n',[0,1,3,4,5,6])
def test_target_minimum_and_insufficient(n):
    r=select(fixture_records(n))
    assert r['selected_count']==min(n,6)
    assert (r['status']=='INSUFFICIENT_SOURCE_DIVERSITY')==(n<4)


def test_one_per_episode_and_lexical_ties():
    rows=[SelectionRecord(c,c.split('/')[0],True,1.,1.) for c in ['b/z','a/z','a/a','c/x','d/x']]
    r=select(rows);assert r['selected_ids']==['a/a','b/z','c/x','d/x']


def test_outcomes_cannot_enter_rank_type_or_file_reads(monkeypatch):
    assert [f.name for f in fields(SelectionRecord)]==['case_id','episode_id','eligible','delta_phi_rad','spacing_severity_abs_log_rho']
    with pytest.raises(TypeError):SelectionRecord('e/h','e',True,1.,1.,AUC=0.)
    with pytest.raises(TypeError):select([dict(case_id='e/h',attachment=0.)])
    monkeypatch.setattr(Path,'read_text',lambda *a,**k:pytest.fail('selector must not read files'))
    assert select(fixture_records())['selected_count']==6


@pytest.mark.parametrize('changed,failed',[
    ({'v_minus':.2},'physically_moving'),({'travel':.0199},'physically_moving'),
    ({'remaining':.4999},'sufficient_future'),({'suffix_rows':4},'sufficient_future'),
    ({'stage_safe':False},'B_ENTRY_safe'),({'fresh_safe':False},'entire_FRESH_safe'),
    ({'history_safe':False},'recorded_history_safe'),({'excluded':True},'development_disjoint'),
    ({'geometry_defined':False},'severity_defined')])
def test_gates(changed,failed):
    args=dict(valid=True,fresh_safe=True,B_safe=True,history_safe=True,v_minus=.8,travel=.02,remaining=.5,
        suffix_rows=5,stage_safe=True,geometry_defined=True,excluded=False,config=CFG['selection'])
    assert eligibility(**args)[1]==[];args.update(changed);assert failed in eligibility(**args)[1]


def test_delta_wrap_rho_and_no_temporal_input():
    B=np.zeros(3);P=[1.,-.001,0];suffix=np.array([[-.2,-.001,0],[-.4,-.001,0],[-.6,-.001,0]])
    s=severity(P,B,suffix);assert s['delta_phi_rad']<.01
    assert s['rho_d']==pytest.approx(np.hypot(.2,.001)/.2)
    assert s['spacing_severity_abs_log_rho']==abs(np.log(s['rho_d']))
    assert not {'dt','time','speed'}&set(inspect.signature(severity).parameters)


@pytest.mark.parametrize('P,suffix',[([0,0,0],[[.2,0,0],[.3,0,0]]),([-1,0,0],[[0,0,0],[.3,0,0]]),([-1,0,0],[[.2,0,0],[.2,0,0]])])
def test_degenerate_fail_closed(P,suffix):
    with pytest.raises(ValueError):severity(P,[0,0,0],suffix)


def test_micrometre_gap_is_not_removed_by_postsolve_collapse_rule():
    s=severity([-.01,0,0],[0,0,0],[[0,2e-6,0],[.15,2e-6,0]])
    assert s['B_to_entry_chord_m']==2e-6 and s['spacing_severity_abs_log_rho']>10


def test_registry_includes_hard_GP_and_prior_episodes():
    reg=exclusion_registry(ROOT,CFG)
    for ep in ['episode_013_repeat_01','episode_001_repeat_01','episode_008_repeat_01','episode_013_repeat_00','episode_021_repeat_01','episode_016_repeat_01']:
        assert ep in reg['excluded_episodes']


@pytest.mark.parametrize('label',[f'E{i}' for i in range(1,7)])
def test_prepared_exact_geometry_controller_state_and_schedule(label):
    if not (RUN/'selected_sources.json').exists():pytest.skip('local ignored prepared corpus absent')
    spec=next(s for s in runner.read(RUN/'selected_sources.json') if s['label']==label);f=Path(spec['folder'])
    c=runner.read(f/'common_state.json');e=runner.read(f/'entry.json');ref=runner.read(f/'native_reference.json')['M0_NATIVE']
    raw=np.load(ref['local_path']);fresh=np.load(ref['world_path']);bits=raw.tobytes()
    suffix,_,computed=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],e['correspondence']);assert computed==e
    installed=np.load(f/'native_installed.npy');saved=np.load(f/'suffix.npy');ids=e['row_original_identities'][1:]
    assert saved[1:].tobytes()==installed[ids].tobytes()
    w,l,labels=staged_reference(installed,raw,c['fresh_capture_pose'],c['B'],e,saved)
    assert w[0].tobytes()==np.asarray(c['B']).tobytes() and w[1].tobytes()==np.asarray(e['correspondence']['target_world']).tobytes()
    assert l[2:].tobytes()==raw[ids].tobytes() and raw.tobytes()==bits
    mapping=identity_mapping(labels,e,installed);assert mapping[0]['original_arc_m'] is None
    assert mapping[1]['fractional_original_row']==e['correspondence']['segment']+e['correspondence']['alpha']
    h,hm=hermite_bridge(np.load(f/'recorded_old_to_B.npy')[-2],c['B'],saved)
    assert h.tobytes()==np.load(f/'hermite_bridge.npy').tobytes()
    v,_=sample_hermite(hm['P'],c['B'],saved,2);assert v.tobytes()==np.load(f/'initial_M2.npy').tobytes()
    p=runner.load_problem(f,'V2');assert p.M==2 and p.boundary=='vector'
    for k,a in independent_factors(p,v[1:-1]).items():np.testing.assert_allclose(a,p.factors(v[1:-1])[k],atol=1e-11)
    assert runner.read(f/'restoration_preflight.json')['numerical_MPC_calls']==0
    m=runner.read(f/'source_manifest.json');assert runner.sha(m['mpc']['mpc_source'])==PINNED_MPC_SHA256
    data=runner.load_source(runner.loader_config(RUN),spec)
    assert data['common']==c and data['schedule']==runner.read(f/'schedule.json')
    assert runner.read(f/'restoration_preflight.json')['provenance']['official_settings']==m['mpc']['official_settings']


@pytest.mark.parametrize('N',[2,3,8])
def test_arbitrary_N_raw_immutable_no_reanchor(N):
    raw=np.array([[j*.2,0,0] for j in range(N)]);A=np.array([2.,3.,.2]);B=local_trajectory_to_world(A,[[.03,.01,0]])[0]
    fresh=local_trajectory_to_world(A,raw);bits=raw.tobytes();s,_,e=suffix_reference(fresh,raw,A,B)
    w,l,labels=staged_reference(fresh,raw,A,B,e,s)
    assert raw.tobytes()==bits and w[2:].tobytes()==s[1:].tobytes() and len(w)>=2
    np.testing.assert_allclose(local_trajectory_to_world(A,l),w,atol=1e-12)


def method(a=1.,e=2.,valid=True,planning=True,unsafe=False,nonconverged=False):
    return dict(valid=valid,planning_valid=planning,technical_valid=True,unsafe_reference=unsafe,safety_abort=False,
        nonconvergence=nonconverged,metrics={ATTACH:a,ENDPOINT:e})


def source(b=None,h=None,v=None):
    return dict(technical_valid=True,integration_dt_s=.02,methods={METHODS[0]:method(),METHODS[1]:b or method(),METHODS[2]:h or method(),METHODS[3]:v or method()})


@pytest.mark.parametrize('category',['TECHNICAL_BLOCKED','INSUFFICIENT_SOURCE_DIVERSITY','GRAPH_SPECIFIC_GAIN_SUPPORTED','INTERMEDIATE_GEOMETRY_NEEDED','STAGING_EVALUATION_SUPPORTED','GRAPH_FORMULATION_NOT_ROBUST','MIXED_EVIDENCE'])
def test_every_classification(category):
    s={str(i):source() for i in range(4)};technical=True;n=4
    if category=='TECHNICAL_BLOCKED':technical=False
    elif category=='INSUFFICIENT_SOURCE_DIVERSITY':s={};n=3
    elif category=='GRAPH_SPECIFIC_GAIN_SUPPORTED':s={str(i):source(v=method(.9,2.)) for i in range(4)}
    elif category=='INTERMEDIATE_GEOMETRY_NEEDED':s={str(i):source(b=method(None,None)) for i in range(4)}
    elif category=='GRAPH_FORMULATION_NOT_ROBUST':s={str(i):source(b=method(None,None),v=method(None,None,valid=False,planning=False,nonconverged=True)) for i in range(4)}
    elif category=='MIXED_EVIDENCE':s['0']=source(b=method(None,None))
    assert classify(s,n,technical)['classification']==category


def test_graph_positive_needs_both_baselines_and_one_tick_and_safety():
    assert source_decision(source(v=method(.98,2.)))['GRAPH_SPECIFIC_POSITIVE']
    assert not source_decision(source(v=method(.99,2.)))['GRAPH_SPECIFIC_POSITIVE']
    assert not source_decision(source(h=method(.9,2.),v=method(.95,2.)))['GRAPH_SPECIFIC_POSITIVE']
    assert not source_decision(source(v=method(.9,2.1)))['GRAPH_SPECIFIC_POSITIVE']
    assert not source_decision(source(v=method(.9,2.,valid=False)))['GRAPH_SPECIFIC_POSITIVE']
    assert source_decision(source(b=method(None,None),h=method(None,None)))['strong_recovery']


def test_numerical_negatives_remain_visible_even_when_staging_precedes_robustness():
    q=source(v=method(None,None,valid=False,planning=False,nonconverged=True))
    r=classify({str(i):q for i in range(4)},4)
    assert r['classification']=='STAGING_EVALUATION_SUPPORTED' and r['V2_nonconvergence_count']==4


def test_prepare_and_validation_forbid_optimizer_and_no_V3_or_acquisition_path():
    def solve_least_squares():pytest.fail('body must never run')
    with no_reconciliation_optimizer(),pytest.raises(RuntimeError):solve_least_squares()
    assert 'with no_reconciliation_optimizer()' in inspect.getsource(runner.prepare)
    tree=ast.parse(inspect.getsource(runner.execute))
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='solve_variant']
    assert len(calls)==1 and calls[0].args[1].value=='V2'
    assert 'exist_ok=False' in inspect.getsource(runner.solve_variant)
    assert 'save(run/\'execution_start.json\'' in inspect.getsource(runner.execute)
    assert not {'capture_rgb','run_source_episode','Session'}&{n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}


def test_endpoint_execution_time_and_null_parity():
    t=np.arange(31)*.02;p=np.tile([0.,0.,0.],(31,1));u=np.zeros((30,2))
    r=endpoint_dwell(t,p,[0,0,0],u,None);assert r[ENDPOINT]==0 and r['T_post_attach_s'] is None
    p[:20,0]=1.;r=endpoint_dwell(t,p,[0,0,0],u,.2)
    assert r[ENDPOINT] is None and r['T_post_attach_s'] is None


def test_expected_call_budget_and_png_contract():
    assert len(PNGS)==3 and len(set(PNGS))==3
    if not (RUN/'expected_calls.json').exists():pytest.skip('prepared local sources absent')
    expected=runner.read(RUN/'expected_calls.json');assert expected['optimizer']==6 and expected['MPC_solves']==720
    assert expected['MPC_applications']==712


def test_saved_only_renderer_three_pngs_numeric_hash_parity(tmp_path,monkeypatch):
    import report_staging_graph_necessity_eval01 as report
    from shapely.geometry import GeometryCollection
    from types import SimpleNamespace
    numbers={};specs=[]
    for i in range(6):
        sid=f'synthetic_{i}';specs.append(dict(id=sid,label=f'T{i}',folder='unused'))
        q=source();decision=source_decision(q)
        path=[[0.,0.,0.],[.2,.1,.2],[.5,.1,.2]]
        numbers[sid]=dict(label=f'T{i}',fresh=path,OLD_to_B=[[-.1,0.,0.],[0.,0.,0.]],B=path[0],E=path[1],
            references={n:path for n in METHODS},V2_stable=i<3,
            severity=dict(B_to_entry_chord_m=.22,delta_phi_rad=i*.1,spacing_severity_abs_log_rho=i*.2),
            source_category=decision,metrics={n:None if n==METHODS[3] and i>=3 else dict(
                position_auc_09_m_s=.04,sustained_attachment_s=1.,original_FRESH_endpoint_dwell_s=2.,
                remaining_arc_at_attachment_m=.5) for n in METHODS})
    monkeypatch.setattr(report,'figure_numbers',lambda _:numbers)
    monkeypatch.setattr(report,'geometry',lambda _:dict(on=SimpleNamespace(obstacles=GeometryCollection())))
    with no_reconciliation_optimizer():manifest=report.render(dict(selected_sources=specs),tmp_path)
    assert sorted(p.name for p in tmp_path.glob('*.png'))==sorted(PNGS)
    for item in manifest:
        assert item['numeric_sidecar']==numbers and item['sha256']==runner.sha(tmp_path/item['file'])


@pytest.mark.parametrize('n',[0,3])
def test_insufficient_source_stops_before_any_scientific_call(tmp_path,monkeypatch,n):
    runner.save(tmp_path/'selected_sources.json',[dict(id=f'ep{i}/h') for i in range(n)])
    monkeypatch.setattr(runner,'verify',lambda *a,**k:None)
    monkeypatch.setattr(runner,'git',lambda *a:'synthetic-freeze')
    monkeypatch.setattr(runner,'solve_variant',lambda *a:pytest.fail('no planning with fewer than four'))
    monkeypatch.setattr(runner.runtime,'run_method',lambda *a:pytest.fail('no rollout with fewer than four'))
    runner.execute(tmp_path)
    assert runner.read(tmp_path/'completion.json')==dict(status='INSUFFICIENT_SOURCE_DIVERSITY',optimizer=0,MPC=0)
