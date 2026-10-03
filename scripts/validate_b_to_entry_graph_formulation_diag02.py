#!/usr/bin/env python3
"""Saved-only authentication, independent residuals and trace/stability validation."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from run_b_to_entry_graph_formulation_diag02 import (ROOT, RESULTS, read, save, sha, plain,
    verify, authenticate, load_problem, ZERO_COUNTS)
from reconciliation.b_to_entry_graph_diag02 import VARIANTS, NEW, PNGS, describe, classify, sample_hermite
from reconciliation.graph_optimizer import SolverConfig, retract_trajectory
from reconciliation.se2 import relative_pose, se2_log, wrap_angle
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.spatial_entry_suffix import no_reconciliation_optimizer
from validate_robotless_online_handoffs import equal_record


def independent_factors(p, interior):
    x=np.vstack([p.B,interior,p.E]);edges=np.diff(x[:,:2],axis=0);d=np.linalg.norm(edges,axis=1)
    if p.variant.startswith('A'):
        phi=np.arctan2(edges[:,1],edges[:,0])
        boundary=[np.atleast_1d(wrap_angle(phi[0]-p.phi_in)/np.deg2rad(15)),
                  np.atleast_1d(wrap_angle(phi[-1]-p.phi_out)/np.deg2rad(15))]
    else:
        h=np.linalg.norm(np.diff(p.initial[:,:2],axis=0),axis=1)
        vi=p.B[:2]-p.P[:2];vo=p.suffix[1,:2]-p.E[:2]
        boundary=[(edges[0]-h[0]*vi/np.linalg.norm(vi))/p.d_F,
                  (edges[-1]-h[-1]*vo/np.linalg.norm(vo))/p.d_F]
    edge_poses=[relative_pose(x[j],x[j+1]) for j in range(p.M)]
    smooth=np.array([se2_log(relative_pose(edge_poses[j-1],edge_poses[j]))/
                     [p.d_F,p.d_F,np.deg2rad(10)] for j in range(1,p.M)]).ravel()
    return dict(E_in=boundary[0],E_out=boundary[1],E_smooth=smooth,E_space=np.diff(d)/p.d_F)


def audit_trace(p, trace, checks, result, env):
    state=p.initial[1:-1].copy();r=p.residual(state);cost=float(r@r);accepted_costs=[cost]
    assert trace and trace[0]['decision']=='initial'
    improving_states=[state.copy()]
    for e in trace:
        if 'candidate' in e:
            candidate=np.asarray(e['candidate'])
            np.testing.assert_array_equal(candidate,retract_trajectory(state,e['delta']))
            residual=p.residual(candidate);cc=float(residual@residual)
            assert np.isclose(cost,e['cost_before'],rtol=1e-13,atol=1e-13)
            assert np.isclose(cc,e['candidate_cost'],rtol=1e-13,atol=1e-13)
            feasible=(p.nondegenerate(candidate) and reference_safety(p.reference(candidate),env)['clearance_valid']) if cc<cost else None
            assert feasible==e['candidate_feasible']
            decision='accepted' if cc<cost and feasible else ('rejected_unsafe' if cc<cost else 'rejected_non_improving')
            assert decision==e['decision']
            damping=e['damping_before']*(.3 if decision=='accepted' else 10.)
            assert e['damping']==(max(np.finfo(float).eps,damping) if decision=='accepted' else damping)
            if cc<cost: improving_states.append(candidate.copy())
            if decision=='accepted':state=candidate;cost=cc;accepted_costs.append(cost)
        np.testing.assert_array_equal(e['state'],state)
        for key,value in independent_factors(p,state).items():
            np.testing.assert_allclose(value,p.factors(state)[key],rtol=1e-11,atol=1e-11)
        equal_record(e['factor_costs'],p.costs(state))
        assert np.isclose(e['cost'],cost,rtol=1e-13,atol=1e-13)
        assert p.bridge(state)[0].tobytes()==p.B.tobytes()
        assert p.bridge(state)[-1].tobytes()==p.E.tobytes()
        assert p.reference(state)[p.M+1:].tobytes()==p.suffix[1:].tobytes()
    assert len(checks)==len(improving_states)
    for c,x in zip(checks,improving_states):
        np.testing.assert_array_equal(c['interior'],x)
        equal_record(c['safety'],reference_safety(p.reference(x),env))
        assert c['nondegenerate']==p.nondegenerate(x)
        assert c['accepted']==(p.nondegenerate(x) and c['safety']['clearance_valid'])
    solver=result['solver']
    if 'cost_history' in solver:
        np.testing.assert_array_equal(solver['cost_history'],accepted_costs)
        assert solver['initial_cost']==accepted_costs[0] and solver['final_cost']==accepted_costs[-1]
        assert solver['iterations']==trace[-1]['iteration']
        if solver['converged']:
            assert solver['termination_reason'] in ['gradient_tolerance','step_tolerance','cost_tolerance']
            if solver['termination_reason']=='cost_tolerance':assert accepted_costs[-2]-accepted_costs[-1]<=1e-12
            if solver['termination_reason']=='gradient_tolerance':assert trace[-2]['gradient_inf']<=1e-9
            if solver['termination_reason']=='step_tolerance':assert trace[-2]['step_norm']<=1e-9
        else:assert solver['termination_reason']=='maximum_iterations' and solver['iterations']==80
    return state


def validate_source(spec, freeze_sha):
    folder=Path(spec['folder']);old=Path(spec['historical_folder']);env=geometry(old)
    assert (folder/'initial_M2.npy').read_bytes()==(old/'hermite_bridge.npy').read_bytes()
    assert (folder/'suffix.npy').read_bytes()==(old/'suffix_native_installed.npy').read_bytes()
    inp=read(folder/'input.json');hist=read(old/'bridge_input.json')
    for k,v in hist.items():assert inp[k]==v,k
    generated,meta=sample_hermite(inp['P'],inp['B'],np.load(folder/'suffix.npy'),3)
    np.testing.assert_array_equal(generated,np.load(folder/'initial_M3.npy'))
    assert meta==read(folder/'M3_sampling.json')
    rows={}
    for variant in VARIANTS:
        p=load_problem(folder,variant)
        out=old/'planning/SE2_GRAPH_BRIDGE' if variant=='A2' else folder/'planning'/variant
        result=read(out/'result.json');trace=read(out/'trace.json');checks=read(out/'feasibility_checks.json')
        if variant!='A2':
            start=read(out/'optimization_start.json')
            assert start['calls']==1 and start['freeze_sha']==freeze_sha
            assert start['initial_sha256']==sha(folder/f'initial_M{p.M}.npy')
            from dataclasses import asdict
            assert start['solver']==asdict(SolverConfig())
        else:
            assert not result['solver']['converged'] and hist['M']==2
        state=audit_trace(p,trace,checks,result,env)
        safety=reference_safety(p.reference(state),env)
        row=describe(p,state,result['solver'],trace,safety,result['wall_s'])
        row.update(initial_bridge=p.initial.tolist(),last_accepted_bridge=p.bridge(state).tolist(),
            fixed_suffix=p.suffix.tolist(),B=p.B.tolist(),E=p.E.tolist(),
            trace_path=str(out/'trace.json'),trace_sha256=sha(out/'trace.json'),
            solver_result_sha256=sha(out/'result.json'),safety=safety)
        if variant!='A2':
            np.testing.assert_array_equal(p.bridge(state),np.load(out/'last_accepted_bridge.npy'))
            assert (out/'converged_reference.npy').exists()==result['solver']['converged']
            if result['solver']['converged']:
                reference=np.load(out/'converged_reference.npy')
                np.testing.assert_array_equal(reference,p.reference(state))
                assert reference[0].tobytes()==p.B.tobytes() and reference[p.M].tobytes()==p.E.tobytes()
                assert reference[p.M+1:].tobytes()==p.suffix[1:].tobytes()
        rows[variant]=plain(row)
    return rows


def validate(run, deep=True):
    with no_reconciliation_optimizer():
        verify(run);authenticate(read(run/'protocol.json'),deep=deep)
        start=read(run/'execution_start.json');complete=read(run/'completion.json')
        assert complete['new_graph_solves']==12 and complete['A2_new_solves']==0
        for k,v in ZERO_COUNTS.items():assert start[k]==complete[k]==v
        selected=read(run/'selected_sources.json')
        assert [s['label'] for s in selected]==['S1','S2','S3','S4']
        starts=list((run/'sources').glob('*/planning/*/optimization_start.json'));assert len(starts)==12
        ordered=[read(Path(s['folder'])/'planning'/v/'optimization_start.json')['started'] for s in selected for v in NEW]
        assert ordered==sorted(ordered) and len(set(ordered))==12
        sources={s['label']:validate_source(s,start['sha']) for s in selected}
        summary=plain(dict(experiment='B_TO_ENTRY_GRAPH_FORMULATION_DIAG_02',
            starting_sha=read(run/'protocol.json')['starting_sha'],scientific_freeze_sha=start['sha'],
            selected_sources=selected,sources=sources,**classify(sources),
            counts=dict(new_graph_solves=12,historical_A2_reused=4,A2_new_solves=0,**ZERO_COUNTS)))
        validation=dict(valid=True,historical_A2_authenticated=True,all_historical_validators=deep,
            independent_residuals=True,trace_decisions=True,exact_boundaries_suffix=True,new_scientific_solves=0)
        if (run/'summary.json').exists():equal_record(summary,read(run/'summary.json'))
        if (run/'result_hashes.json').exists():
            for path,h in read(run/'result_hashes.json').items():assert sha(run/path)==h,path
            tracked=read(RESULTS/'result_summary.json');assert tracked['summary']==summary
            assert tracked['result_hashes_sha256']==sha(run/'result_hashes.json')
            for path,h in tracked['table_hashes'].items():assert sha(RESULTS/path)==h
            assert sorted(p.name for p in (RESULTS/'figures').iterdir())==sorted(PNGS)
            manifest=read(RESULTS/'figure_manifest.json');assert manifest['summary_sha256']==sha(run/'summary.json')
            for f in manifest['figures']:assert sha(RESULTS/'figures'/f['file'])==f['sha256']
        return summary,validation


def csv_write(path, rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rows:w.writerow({k:json.dumps(v) if isinstance(v,(list,dict)) else v for k,v in r.items()})


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    rows=[];costs=[];edges=[];convergence=[]
    for label,variants in s['sources'].items():
        for variant,r in variants.items():
            small={k:v for k,v in r.items() if k not in ['initial_bridge','last_accepted_bridge','fixed_suffix','safety']}
            rows.append(dict(source=label,**small))
            for factor in r['initial_costs']:
                costs.append(dict(source=label,variant=variant,factor=factor,initial=r['initial_costs'][factor],
                    final=r['final_costs'][factor],final_state_label=r['final_state_label']))
            for j,(a,b,rho) in enumerate(zip(r['initial_edge_lengths_m'],r['final_edge_lengths_m'],r['rho'])):
                edges.append(dict(source=label,variant=variant,edge=j,initial_length_m=a,final_length_m=b,rho=rho,
                    length_over_fd=b/1e-6,below_or_at_numerical_threshold=b<=1e-5,stable=r['stable']))
            convergence.append(dict(source=label,variant=variant,**{k:r[k] for k in ['historical','converged','stable',
                'termination_reason','iterations','accepted_steps','rejected_steps','final_damping','final_gradient_inf',
                'final_step_norm','unsafe_improving_proposals','wall_s','numerical_collapse','rho_min']}))
    for name,values in [('variant_summary',rows),('factor_costs',costs),('edge_scale',edges),('convergence_summary',convergence)]:
        csv_write(RESULTS/(name+'.csv'),values)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--check-only',action='store_true');args=parser.parse_args()
    if args.check_only:
        s,v=validate(args.run.resolve());print(json.dumps(dict(validation=v,classification=s['classification'],counts=s['counts'])))
    else:write(args.run.resolve())
