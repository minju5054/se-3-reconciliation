#!/usr/bin/env python3
"""Saved-only independent cost, retraction, acceptance, source and figure audit."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from run_canonical_se2_graph_formulation_audit01 import (ROOT,RESULTS,read,save,sha,plain,verify,load_problem,
    geometry,reference_safety,BASELINES,PNGS,planning_guard,diagnostics,v2_hermite_similarity,independent_costs,parity)
from reconciliation.canonical_graph_audit01 import classify
from reconciliation.se2 import compose_poses,se2_exp,relative_pose


def write_json(path,value): Path(path).write_text(json.dumps(plain(value),indent=2,allow_nan=False)+'\n')


def csv_text(rows):
    import io
    stream=io.StringIO(newline='');fields=list(dict.fromkeys(k for r in rows for k in r))
    w=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');w.writeheader()
    w.writerows({k:json.dumps(plain(v),separators=(',',':'),allow_nan=False) if isinstance(v,(dict,list,np.ndarray)) else v for k,v in r.items()} for r in rows)
    return stream.getvalue()


def audit_trace(p,x,trace,checks,solver,env):
    """No Jacobian or optimizer invocation; rebuild all costs and decisions."""
    state=p.fresh.copy();costs=[independent_costs(p.fresh,p.target,state)['total']];expected_checks=[state.copy()]
    damping=.001
    assert trace[0]['decision']=='initial'
    for e in trace:
        parity(e['factor_costs'],independent_costs(p.fresh,p.target,np.asarray(e['state'])))
        if 'candidate' in e:
            q=np.array(e['candidate']);delta=np.array(e['delta']).reshape(state.shape)
            np.testing.assert_allclose(q,compose_poses(state,se2_exp(delta)),atol=1e-12,rtol=0)
            before=independent_costs(p.fresh,p.target,state)['total'];after=independent_costs(p.fresh,p.target,q)['total']
            parity(e['cost_before'],before);parity(e['candidate_cost'],after);parity(e['damping_before'],damping)
            # Use the saved residual dot arithmetic for strict machine-level ordering.
            rb=p.residual_vector(state);ra=p.residual_vector(q);improving=float(ra@ra)<float(rb@rb)
            if improving:expected_checks.append(q);feasible=reference_safety(q,env)['clearance_valid']
            else:feasible=None
            assert e['candidate_feasible']==feasible
            decision='accepted' if improving and feasible else 'rejected_unsafe' if improving else 'rejected_non_improving'
            assert e['decision']==decision
            if decision=='accepted':state=q;costs.append(after);damping=max(np.finfo(float).eps,damping*.3)
            else:damping*=10.
        parity(e['damping'],damping);np.testing.assert_array_equal(e['state'],state)
        parity(e['cost'],independent_costs(p.fresh,p.target,state)['total'])
    np.testing.assert_array_equal(state,x)
    if 'cost_history' in solver:parity(solver['cost_history'],costs)
    assert len(checks)==len(expected_checks)
    for check,q in zip(checks,expected_checks):
        np.testing.assert_array_equal(check['state'],q);parity(check['check'],reference_safety(q,env))
    assert all(reference_safety(np.asarray(e['state']),env)['clearance_valid'] for e in trace)
    return dict(accepted=sum(e['decision']=='accepted' for e in trace),unsafe_rejected=sum(e['decision']=='rejected_unsafe' for e in trace),checked_feasibility=len(checks))


def source_row(label,r):
    d=r['diagnostics'];c=r['factor_costs'];rigid=d['rigid_fit']
    return dict(source=label,N=len(d['nodes']),converged=r['solver']['converged'],first_node_m=d['first_node_correction_m'],
        endpoint_m=d['endpoint_correction_m'],max_correction_m=d['raw_difference']['XY_max_m'],RMS_correction_m=d['raw_difference']['XY_RMS_m'],
        max_yaw_correction_rad=d['raw_difference']['yaw_max_rad'],RMS_yaw_correction_rad=d['raw_difference']['yaw_RMS_rad'],
        relative_translation_RMS_m=d['relative_translation_RMS_m'],relative_translation_max_m=d['relative_translation_max_m'],
        relative_yaw_RMS_rad=d['relative_yaw_RMS_rad'],relative_yaw_max_rad=d['relative_yaw_max_rad'],
        rigid_fit_translation_RMS_m=rigid['translation_RMS_m'],rigid_fit_yaw_RMS_rad=rigid['yaw_RMS_rad'],
        reference_clearance_m=d['reference_clearance_m'],raw_cost=c['RAW']['total'],canonical_cost=c['CANONICAL']['total'],
        canonical_vs_V2_vertex_XY_max_m=d['comparisons']['V2_SINGLE_NODE']['symmetric_vertex_XY_max_m'])


def figure_data(run,summary):
    data={}
    for s in read(run/'selected_sources.json'):
        f=Path(s['folder']);c=read(f/'context.json')
        data[s['label']]=dict(A=c['A'],B=c['B'],raw=np.load(f/'raw_fresh.npy').tolist(),
            transported=np.load(f/'transported_target.npy').tolist(),canonical=np.load(f/'canonical_optimized.npy').tolist(),
            baselines={n:np.load(f/(n+'.npy')).tolist() for n in BASELINES},
            nodes=summary['sources'][s['label']]['diagnostics']['nodes'])
    return dict(scope='planning only; no executed paths',sources=data,summary=summary)


def validate(run):
    with planning_guard():
        verify(run);cfg=read(run/'protocol.json');selected=read(run/'selected_sources.json');start=read(run/'execution_start.json')
        assert len(selected)==5 and len(list(run.glob('sources/*/optimization_start.json')))==5
        assert not list(run.glob('sources/*/methods'))
        sources={};audits={};similarity={}
        for s in selected:
            f=Path(s['folder']);p=load_problem(f);c=read(f/'context.json');env=geometry(Path(c['historical_folder']))
            assert read(f/'optimization_start.json')['scientific_freeze_sha']==start['scientific_freeze_sha']
            for key,file in [('local','raw_observation_local.npy'),('world','raw_fresh.npy')]:
                historical=Path(c['raw_'+key+'_path']);assert sha(historical)==c['raw_'+key+'_sha256']
                np.testing.assert_array_equal(np.load(f/file),np.load(historical))
            np.testing.assert_allclose(compose_poses(p.A,np.load(f/'raw_observation_local.npy')),p.fresh,atol=1e-12,rtol=0)
            np.testing.assert_array_equal(np.load(f/'transported_target.npy'),p.target)
            np.testing.assert_allclose(relative_pose(p.B,p.target),relative_pose(p.A,p.fresh),atol=1e-12,rtol=0)
            x=np.load(f/'canonical_optimized.npy');r=read(f/'result.json')
            np.testing.assert_allclose(compose_poses(p.A,np.load(f/'canonical_original_A_local.npy')),x,atol=1e-12,rtol=0)
            assert np.isfinite(x).all();assert r['canonical_scientific_calls']==1
            assert r['scientific_freeze_sha']==start['scientific_freeze_sha']
            for key,arr in [('RAW',p.fresh),('FULL_TRANSPORT',p.target),('CANONICAL',x)]:parity(r['factor_costs'][key],independent_costs(p.fresh,p.target,arr))
            if (f/'local_se2_historical.npy').exists():
                h=np.load(f/'local_se2_historical.npy');np.testing.assert_allclose(x,h,atol=cfg['historical_solution_atol'],rtol=0)
                parity(r['factor_costs']['LOCAL_SE2_HISTORICAL'],independent_costs(p.fresh,p.target,h))
                assert r['historical_solution_max_abs_difference']==float(np.max(abs(x-h)))
            safe=reference_safety(x,env);parity(read(f/'reference_geometry.json'),safe)
            baselines={n:np.load(f/(n+'.npy')) for n in BASELINES}
            d=diagnostics(p,x,baselines,safe);parity(r['diagnostics'],d);parity(read(f/'rigid_fit.json'),d['rigid_fit'])
            parity(read(f/'factor_costs.json'),r['factor_costs'])
            audits[s['label']]=audit_trace(p,x,read(f/'solver_trace.json'),read(f/'feasibility_checks.json'),r['solver'],env)
            sources[s['label']]=r
            similarity[s['label']]=v2_hermite_similarity(baselines['HERMITE'],baselines['V2_SINGLE_NODE'],np.load(f/'HERMITE_installed.npy'),np.load(f/'V2_SINGLE_NODE_installed.npy'))
            assert similarity[s['label']]==read(run/'v2_hermite_similarity.json')[s['label']]
        counts=read(run/'completion.json');assert counts=={k:cfg[k] for k in counts}
        assert counts['canonical_scientific_solves']==len(sources)
        summary=dict(experiment=cfg['experiment'],starting_sha=cfg['starting_sha'],scientific_freeze_sha=start['scientific_freeze_sha'],
            sources=sources,classification=classify(sources,cfg),call_accounting=counts,V2_Hermite_similarity=similarity)
        validation=dict(valid=True,saved_only=True,new_scientific_solves=0,source_hashes=True,independent_costs=True,
            right_local_and_acceptance=True,frame_parity=True,OSA03_historical_solution_parity=True,sources=audits)
        return summary,validation


def tables(summary):
    return {'source_metrics.csv':[source_row(k,r) for k,r in summary['sources'].items()],
        'v2_hermite_similarity.csv':[dict(source=k,**v) for k,v in summary['V2_Hermite_similarity'].items()],
        'comparison.csv':[dict(source=k,baseline=n,**c) for k,r in summary['sources'].items() for n,c in r['diagnostics']['comparisons'].items()]}


def export(run,summary,validation):
    RESULTS.mkdir(parents=True,exist_ok=True)
    for path in [run/'comparison_summary.json',RESULTS/'comparison_summary.json']:write_json(path,summary)
    for name,value in [('classification',summary['classification']),('call_accounting',summary['call_accounting']),('validation',validation)]:write_json(RESULTS/(name+'.json'),value)
    for name,rows in tables(summary).items():(RESULTS/name).write_text(csv_text(rows))
    for s in read(run/'selected_sources.json'):
        f=Path(s['folder']);d=summary['sources'][s['label']]['diagnostics']
        (f/'correction_profile.csv').write_text(csv_text(d['nodes']))
        (f/'relative_edges.csv').write_text(csv_text([dict(j=j,log_x_m=r[0],log_y_m=r[1],yaw_rad=r[2]) for j,r in enumerate(d['relative_edges'])]))
    hashes={str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file() and p.name!='result_hashes.json'}
    write_json(run/'result_hashes.json',hashes);write_json(RESULTS/'result_hashes.json',dict(run=str(run),sha256=sha(run/'result_hashes.json'),files=hashes))


def check_exports(run,summary):
    assert read(RESULTS/'comparison_summary.json')==plain(summary)
    assert read(run/'comparison_summary.json')==plain(summary)
    for name,rows in tables(summary).items():assert (RESULTS/name).read_text()==csv_text(rows),name
    for s in read(run/'selected_sources.json'):
        f=Path(s['folder']);d=summary['sources'][s['label']]['diagnostics']
        assert (f/'correction_profile.csv').read_text()==csv_text(d['nodes'])
        assert (f/'relative_edges.csv').read_text()==csv_text([dict(j=j,log_x_m=r[0],log_y_m=r[1],yaw_rad=r[2]) for j,r in enumerate(d['relative_edges'])])
    ledger=read(RESULTS/'result_hashes.json');assert sha(run/'result_hashes.json')==ledger['sha256']
    assert read(run/'result_hashes.json')==ledger['files']
    for p,h in ledger['files'].items():assert sha(run/p)==h,p
    assert sorted(p.name for p in (RESULTS/'figures').iterdir())==sorted(PNGS)
    manifest=read(RESULTS/'figure_manifest.json')
    assert read(RESULTS/'figure_numeric.json')==plain(figure_data(run,summary))
    assert manifest['numeric_sha256']==sha(RESULTS/'figure_numeric.json')
    for item in manifest['figures']:assert sha(RESULTS/'figures'/item['file'])==item['sha256']
    return True


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args();summary,validation=validate(args.run.resolve())
    if args.check_only:check_exports(args.run.resolve(),summary)
    else:export(args.run.resolve(),summary,validation)
    print(dict(validation=validation,classification=summary['classification']['classification']))
