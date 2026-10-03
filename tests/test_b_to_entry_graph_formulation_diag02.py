"""Synthetic formulation tests and saved historical authentication; no new science."""
import ast
from copy import deepcopy
from dataclasses import asdict
import inspect
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import run_b_to_entry_graph_formulation_diag02 as runner
import validate_b_to_entry_graph_formulation_diag02 as validator
import report_b_to_entry_graph_formulation_diag02 as report
from reconciliation.b_to_entry_graph_diag02 import *
from reconciliation.b_to_entry_bridge import BridgeProblem, hermite_curve
from reconciliation.graph_optimizer import solve_least_squares, retract_trajectory
from reconciliation.se2 import compose_poses, relative_pose, se2_log, wrap_angle
from reconciliation.spatial_entry_suffix import no_reconciliation_optimizer, suffix_reference
from reconciliation.join_source03 import read,save,sha
CFG=runner.yaml.safe_load(runner.CONFIG.read_text())
HIST=read(ROOT/CFG['historical_result'])
SOURCES=HIST['summary']['selected_sources']


def toy(variant):
    P=np.array([-.03,0,.2]);B=np.array([0.,0,3.1]);suffix=np.array([[.7,.2,-3.1],[1.,.3,-3.05],[1.3,.4,-3.]])
    initial,_=sample_hermite(P,B,suffix,VARIANTS[variant][1])
    return DiagnosticProblem(P,B,suffix,.3,initial,variant)


@pytest.mark.parametrize('s',SOURCES,ids=['S1','S2','S3','S4'])
def test_historical_A2_residual_trace_C3_and_raw_authentication(s):
    f=Path(s['folder']);spec=read(f/'bridge_input.json');suffix=np.load(f/'suffix_native_installed.npy')
    initial=np.load(f/'hermite_bridge.npy');copied=initial.copy()
    p=DiagnosticProblem(spec['P'],spec['B'],suffix,spec['d_F_m'],initial,'A2')
    old=BridgeProblem(spec['P'],spec['B'],suffix,spec['d_F_m'],2)
    out=f/'planning/SE2_GRAPH_BRIDGE';trace=read(out/'trace.json')
    for e in trace:
        np.testing.assert_array_equal(p.residual(e['state']),old.residual(e['state']))
        assert p.costs(e['state'])==e['factor_costs']
    with no_reconciliation_optimizer():
        validator.audit_trace(p,trace,read(out/'feasibility_checks.json'),read(out/'result.json'),runner.geometry(f))
    assert not read(out/'result.json')['solver']['converged']
    rawref=read(f/'reuse.json')['methods']['M0_NATIVE']['reference']
    raw=np.load(rawref['local_path']);fresh=np.load(rawref['world_path']);rawbits=raw.tobytes();freshbits=fresh.tobytes()
    c=read(f/'common_state.json');e=read(f/'entry.json')
    suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],e['historical_C3'])
    np.testing.assert_array_equal(p.E,e['correspondence']['target_world'])
    assert raw.tobytes()==rawbits and fresh.tobytes()==freshbits
    np.testing.assert_array_equal(initial,copied)
    sampled,_=sample_hermite(p.P,p.B,p.suffix,2);np.testing.assert_array_equal(sampled,initial)
    M3,meta=sample_hermite(p.P,p.B,p.suffix,3)
    assert meta['continuous_arc_m']==spec['Hermite_continuous_arc_m']
    _,_,arc,_,_=hermite_curve(p.P,p.B,p.E,p.suffix[1])
    np.testing.assert_allclose([arc(t) for t in meta['parameters']],np.linspace(0,arc(1),4),atol=1e-12,rtol=0)
    assert M3[0].tobytes()==p.B.tobytes() and M3[-1].tobytes()==p.E.tobytes()


def test_full_historical_validator_chain_and_tamper():
    runner.authenticate(CFG,deep=True)
    bad=deepcopy(CFG);bad['historical_result_sha256']='0'*64
    with pytest.raises(AssertionError):runner.authenticate(bad)


@pytest.mark.parametrize('variant',VARIANTS)
def test_M_fixed_endpoints_suffix_and_only_interior_right_local_variables(variant):
    p=toy(variant);initial=p.initial.copy();suffix=p.suffix.copy()
    state=retract_trajectory(initial[1:-1],np.full((p.M-1,3),.01))
    b=p.bridge(state);r=p.reference(state)
    assert len(b)==p.M+1 and state.shape==(p.M-1,3)
    assert b[0].tobytes()==p.B.tobytes() and b[-1].tobytes()==p.E.tobytes()
    assert r[p.M+1:].tobytes()==suffix[1:].tobytes()
    np.testing.assert_array_equal(initial,p.initial)
    assert all(x.flags.writeable is False for x in [p.B,p.E,p.initial,p.suffix])
    assert p.residual(state).size==(2 if variant.startswith('A') else 4)+4*(p.M-1)


@pytest.mark.parametrize('variant',VARIANTS)
def test_independent_smoothness_spacing_and_angle_semantics(variant):
    p=toy(variant);state=p.initial[1:-1].copy();state[:,0]+=.02
    expected=validator.independent_factors(p,state)
    for key,value in p.factors(state).items():np.testing.assert_allclose(value,expected[key],atol=1e-14)
    old=BridgeProblem(p.P,p.B,p.suffix,p.d_F,p.M)
    for key in ['E_smooth','E_space']:np.testing.assert_array_equal(p.factors(state)[key],old.factors(state)[key])
    if variant.startswith('A'):np.testing.assert_array_equal(p.residual(state),old.residual(state))


@pytest.mark.parametrize('variant',['V2','V3'])
@pytest.mark.parametrize('edge',['in','out'])
def test_vector_formula_and_collapsed_boundary_cannot_zero(variant,edge):
    p=toy(variant);state=p.initial[1:-1].copy()
    if edge=='in':state[0,:2]=p.B[:2];target=p.in_target
    else:state[-1,:2]=p.E[:2];target=p.out_target
    residual=p.factors(state)['E_'+edge]
    np.testing.assert_array_equal(residual,-target/p.d_F)
    assert np.linalg.norm(residual)>0 and not p.nondegenerate(state)


@pytest.mark.parametrize('variant',['V2','V3'])
def test_vector_targets_spatial_Hermite_not_P_travel_and_no_waypoint_time(variant):
    p=toy(variant);other=DiagnosticProblem(p.B+1000*(p.P-p.B),p.B,p.suffix,p.d_F,p.initial,variant)
    np.testing.assert_array_equal(p.in_target,other.in_target)
    np.testing.assert_array_equal(p.residual(p.initial[1:-1]),other.residual(p.initial[1:-1]))
    assert np.isclose(np.linalg.norm(p.in_target),np.linalg.norm(p.initial[1,:2]-p.B[:2]))
    assert np.isclose(np.linalg.norm(p.out_target),np.linalg.norm(p.E[:2]-p.initial[-2,:2]))
    for f in [DiagnosticProblem.__init__,sample_hermite]:
        assert not any('time' in n or n=='dt' for n in inspect.signature(f).parameters)


@pytest.mark.parametrize('variant',['V2','V3'])
def test_global_SE2_transform_preserves_vector_squared_cost(variant):
    p=toy(variant);G=np.array([1.3,-2.1,2.9])
    q=DiagnosticProblem(compose_poses(G,p.P),compose_poses(G,p.B),compose_poses(G,p.suffix),
        p.d_F,compose_poses(G,p.initial),variant)
    for key in ['E_in','E_out']:
        np.testing.assert_allclose(p.costs(p.initial[1:-1])[key],q.costs(q.initial[1:-1])[key],atol=1e-13,rtol=1e-12)


@pytest.mark.parametrize('M',[2,3])
def test_shortest_yaw_pi_branch_and_equal_arc(M):
    p=toy('A2' if M==2 else 'A3');x,meta=sample_hermite(p.P,p.B,p.suffix,M)
    np.testing.assert_allclose(wrap_angle(x[:,2]-p.B[2]),np.linspace(0,wrap_angle(p.E[2]-p.B[2]),M+1),atol=1e-14)
    assert np.max(abs(wrap_angle(np.diff(x[:,2]))))<.05


def test_solver_and_safety_source_unchanged_and_config_exact():
    assert CFG['solver']==asdict(SolverConfig())
    assert sha(ROOT/'src/reconciliation/graph_optimizer.py')==CFG['solver_source_sha256']
    assert sha(ROOT/'src/reconciliation/b_to_entry_bridge.py')==CFG['historical_bridge_source_sha256']
    from reconciliation.relative_factor_multisource import reference_safety
    assert runner.reference_safety is reference_safety
    assert DiagnosticProblem.nondegenerate is BridgeProblem.nondegenerate
    assert COLLAPSE_M==CFG['numerical_collapse_threshold_m']==1e-5
    assert COLLAPSE_M==pytest.approx(10*SolverConfig().finite_difference_epsilon,rel=1e-15)


def test_postsolve_collapse_gate_does_not_tighten_feasibility_and_rho():
    p=toy('V2');state=p.initial[1:-1].copy();state[0,:2]=p.B[:2]+[5e-6,0]
    assert p.nondegenerate(state)
    r=describe(p,state,dict(converged=True),[],dict(clearance_valid=True,minimum_clearance_m=1.),0.)
    assert r['numerical_collapse'] and not r['stable']
    np.testing.assert_array_equal(r['rho'],np.array(r['final_edge_lengths_m'])/p.initial_lengths)
    assert r['rho_min']==min(r['rho']) and r['min_edge_over_fd']==pytest.approx(5,rel=1e-15)


def test_unsafe_improving_synthetic_proposals_rejected():
    p=toy('V3');initial=p.initial[1:-1].copy();trace=[]
    result=solve_least_squares(initial,p.residual,SolverConfig(),
        candidate_feasibility_fn=lambda x:np.array_equal(x,initial),iteration_callback=trace.append)
    np.testing.assert_array_equal(result.optimized,initial)
    assert any(e['decision']=='rejected_unsafe' for e in trace)
    for e in trace:np.testing.assert_array_equal(e['state'],initial)


def test_planning_guard_and_no_execution_call_path():
    def integrate_unicycle():pytest.fail('forbidden body executed')
    with pytest.raises(RuntimeError),runner.planning_only():integrate_unicycle()
    forbidden={'run_method','run_rollout','Worker','OfficialMPCAdapter','integrate_unicycle','capture_rgb'}
    for module in [runner,validator,report]:
        tree=ast.parse(inspect.getsource(module))
        for node in ast.walk(tree):
            if isinstance(node,ast.Call):
                name=node.func.id if isinstance(node.func,ast.Name) else getattr(node.func,'attr','')
                assert name not in forbidden
    assert runner.ZERO_COUNTS==dict(MPC_solves=0,rollouts=0,state_integrations=0,controller_memory_updates=0,
        command_applications=0,LightNav=0,RGB=0,Isaac=0,retries=0)


@pytest.mark.parametrize('counts,label',[
    ([0,4,4,0],'MULTIPLE_STABLE_FORMULATIONS'),([0,4,0,0],'RESOLUTION_LIMIT_SUPPORTED'),
    ([0,0,4,0],'BOUNDARY_DEGENERACY_SUPPORTED'),([0,0,0,4],'BOTH_RESOLUTION_AND_BOUNDARY_NEEDED'),
    ([0,3,2,1],'MIXED_FORMULATION_EFFECT'),([0,2,2,2],'FORMULATION_STILL_BLOCKED'),
    ([0,0,0,0],'FORMULATION_STILL_BLOCKED')])
def test_frozen_classification_precedence(counts,label):
    sources={f'S{i+1}':{v:dict(stable=i<counts[j]) for j,v in enumerate(VARIANTS)} for i in range(4)}
    assert classify(sources)['classification']==label
    assert classify(sources,False)['classification']=='TECHNICAL_BLOCKED'


def test_synthetic_runner_one_call_and_saved_validator_no_resolve(tmp_path,monkeypatch):
    p=toy('V3');folder=tmp_path/'Sx';folder.mkdir()
    save(folder/'input.json',dict(P=p.P,B=p.B,d_F_m=p.d_F,historical_folder='synthetic'))
    np.save(folder/'suffix.npy',p.suffix);np.save(folder/'initial_M3.npy',p.initial)
    monkeypatch.setattr(runner,'geometry',lambda f:None)
    checker=lambda x,env:dict(clearance_valid=True,minimum_clearance_m=1.)
    monkeypatch.setattr(runner,'reference_safety',checker);monkeypatch.setattr(validator,'reference_safety',checker)
    runner.solve_variant(folder,'V3','synthetic')
    out=folder/'planning/V3'
    monkeypatch.setattr(runner,'solve_least_squares',lambda *a,**kw:pytest.fail('validator called solve'))
    with no_reconciliation_optimizer():
        x=validator.audit_trace(p,read(out/'trace.json'),read(out/'feasibility_checks.json'),read(out/'result.json'),None)
    np.testing.assert_array_equal(p.bridge(x),np.load(out/'last_accepted_bridge.npy'))
    with pytest.raises(FileExistsError):runner.solve_variant(folder,'V3','synthetic')
    with pytest.raises(ValueError):runner.solve_variant(folder,'A2','synthetic')


def test_exactly_three_PNGs_no_HTML_and_numeric_sidecars(tmp_path):
    sources={}
    for i in range(4):
        rows={}
        for v in VARIANTS:
            p=toy(v);r=describe(p,p.initial[1:-1],dict(converged=v.startswith('V'),iterations=12),[],
                dict(clearance_valid=True,minimum_clearance_m=1.),0.)
            r.update(initial_bridge=p.initial.tolist(),last_accepted_bridge=p.initial.tolist(),fixed_suffix=p.suffix.tolist(),B=p.B.tolist(),E=p.E.tolist())
            rows[v]=r
        sources[f'S{i+1}']=rows
    records=report.compact_figures(dict(sources=sources),tmp_path/'figures')
    assert sorted(p.name for p in (tmp_path/'figures').iterdir())==sorted(PNGS)
    assert len(records)==3 and all(r['numeric_sidecar'] for r in records)
    assert not list(tmp_path.rglob('*.html'))
    for r in records:assert sha(tmp_path/'figures'/r['file'])==r['sha256']
