#!/usr/bin/env python3
"""Saved-only bridge factor, immutable-boundary, schedule and execution audit."""
import argparse
import json
from pathlib import Path
import numpy as np
from run_b_to_entry_bridge01 import ROOT, RESULTS, read, save, sha, plain, verify, authenticate, load_problem
from reconciliation.b_to_entry_bridge import *
from reconciliation.spatial_entry_suffix import no_reconciliation_optimizer, suffix_reference
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.graph_optimizer import retract_trajectory
from reconciliation.osa03_common_b import schedule
from reconciliation.osa03_native import evaluate
from reconciliation.osa03_relative_ablation import metric_row
from validate_spatial_entry_suffix_execution01 import independent_endpoint_time
from validate_relative_factor_multisource01 import validate_method
from validate_robotless_online_handoffs import equal_record
from validate_spatial_correspondence_selector_diag01 import csv_write


def method_folder(folder,name):
    return folder/'methods'/name if name in NEW else Path(read(folder/'reuse.json')['methods'][name]['folder'])


def validate_planning(folder,env):
    p=load_problem(folder);out=folder/'planning'/GRAPH;initial=np.load(folder/'hermite_bridge.npy')[1:-1]
    start=read(out/'optimization_start.json');result=read(out/'result.json');trace=read(out/'trace.json')
    assert start['calls']==1 and start['initial_sha256']==sha(folder/'hermite_bridge.npy')
    assert start['freeze_sha']==read(folder.parents[1]/'execution_start.json')['sha']
    assert start['solver']==read(folder/'protocol.json')['solver']
    state=initial.copy();residual=p.residual(state);cost=float(residual@residual);accepted=0
    for e in trace:
        if 'candidate' in e:
            candidate=np.asarray(e['candidate']);np.testing.assert_array_equal(candidate,retract_trajectory(state,e['delta']))
            residual=p.residual(candidate);cc=float(residual@residual)
            assert np.isclose(cc,e['candidate_cost'],rtol=1e-13,atol=1e-13)
            feasible=(p.nondegenerate(candidate) and reference_safety(p.reference(candidate),env)['clearance_valid']) if cc<cost else None
            assert feasible==e['candidate_feasible']
            decision='accepted' if cc<cost and feasible else ('rejected_unsafe' if cc<cost else 'rejected_non_improving')
            assert decision==e['decision']
            expected=e['damping_before']*(.3 if decision=='accepted' else 10.)
            assert np.isclose(e['damping'],max(np.finfo(float).eps,expected) if decision=='accepted' else expected,rtol=1e-14)
            if decision=='accepted':state=candidate;cost=cc;accepted+=1
        np.testing.assert_array_equal(state,e['state']);equal_record(e['factor_costs'],p.costs(state))
        np.testing.assert_array_equal(p.bridge(state)[0],p.B);np.testing.assert_array_equal(p.bridge(state)[-1],p.E)
    checks=read(out/'feasibility_checks.json')
    assert len(checks)==1+sum(e.get('candidate_cost',float('inf'))<e.get('cost_before',-float('inf')) for e in trace)
    for c in checks:
        equal_record(c['safety'],reference_safety(p.reference(c['interior']),env))
        assert c['nondegenerate']==p.nondegenerate(c['interior'])
        assert c['accepted']==(c['nondegenerate'] and c['safety']['clearance_valid'])
    if result['error'] is None:
        assert result['solver']['converged'];assert result['solver']['final_cost']==trace[-1]['cost']
        np.testing.assert_array_equal(p.bridge(state),np.load(out/'bridge.npy'))
    return dict(valid=True,solver_success=result['error'] is None,solver=result['solver'],error=result['error'],
        accepted_steps=accepted,unsafe_rejections=sum(e['decision']=='rejected_unsafe' for e in trace),new_solves=0)


def selection_rows(rollout,labels):
    rows=[]
    for e in rollout['events']:
        if e.get('type')=='solve_result' and e.get('status')=='command':
            ix=e['selection']['indices']
            rows.append(dict(submit_tick=e['input_state_id'],input_pose=e['input_pose'],selected_reference_rows=ix,
                selected_identities=[labels[i] for i in ix],nearest_identity=labels[e['selection']['nearest_index']],
                application_tick=e.get('continuation_seen',{}).get('tick'),
                held_after_cap=bool(e.get('withheld_by_logical_scheduler'))))
    return rows


def validate_source(folder,historical):
    reuse=read(folder/'reuse.json');c=read(folder/'common_state.json');spec=read(folder/'bridge_input.json')
    refs=read(folder/'final_references.json');assert refs==read(folder/'references.json')
    for path,record in reuse['copies'].items():assert sha(folder/path)==sha(record['path'])==record['sha256']
    fresh=np.load(refs[NATIVE]['world_path']);raw=np.load(refs[NATIVE]['local_path'])
    entry=read(folder/'entry.json')
    suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['historical_C3'])
    p=load_problem(folder);past=np.load(folder/'recorded_old_to_B.npy')
    np.testing.assert_array_equal(p.P,past[-2]);np.testing.assert_array_equal(p.B,c['B'])
    np.testing.assert_array_equal(p.E,entry['correspondence']['target_world'])
    native=np.asarray(read(method_folder(folder,NATIVE)/'restoration.json')['installed_world'])
    np.testing.assert_array_equal(p.suffix[1:],native[spec['original_ids']])
    h,hd=hermite_bridge(p.P,p.B,p.suffix);np.testing.assert_array_equal(h,np.load(folder/'hermite_bridge.npy'))
    for k,v in hd.items():assert spec[k]==v
    env=geometry(folder);pa=validate_planning(folder,env)
    scene=read(folder/'scenario.json');settings=read(folder/'source_manifest.json')['mpc']['official_settings']
    frozen=read(folder/'schedule.json');release={p['submit_tick']:p['application_tick'] for p in frozen['pairs']}
    pre=read(folder/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
    assert pre['provenance']['official_settings']==settings
    primary={};secondary={};audits={};rollouts={};selectors={};geom={};gates={};roundoff={};failure=False
    for name in ORDER:
        out=method_folder(folder,name)
        if name not in refs:
            assert name==GRAPH and read(out/'skipped.json')['status']=='PLANNING_FAILED'
            assert not (out/'rollout.json').exists()
            primary[name]=None;rollouts[name]=None;selectors[name]=[];audits[name]=dict(valid=True,planning_failed=True)
            continue
        ref=refs[name]
        for frame in ['local','world']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        x=np.load(ref['world_path']);equal_record(ref['safety'],reference_safety(x,env))
        if name in NEW:
            b=x[:p.M+1]
            w,l,labels=reference_arrays(p,b,c['fresh_capture_pose'],raw,spec['original_ids'])
            np.testing.assert_array_equal(x,w);np.testing.assert_array_equal(np.load(ref['local_path']),l)
            assert labels==ref['labels']
            np.testing.assert_array_equal(x[p.M+1:],native[spec['original_ids']])
            geom[name]=bridge_geometry(p,b[1:-1],ref['safety'])
            if name==GRAPH:np.testing.assert_array_equal(b,np.load(folder/'planning'/GRAPH/'bridge.npy'))
            failure |= not ref['safety']['clearance_valid']
        else:
            old=reuse['methods'][name];assert ref==old['reference']
            assert sha(out/'hashes.json')==old['hashes_sha256']
            labels=[f'F_{j}' for j in range(len(x))] if name==NATIVE else [
                'E*' if j is None else f'F_{j}' for j in entry['row_original_identities']]
        r,m,a=validate_method(folder if name in NEW else out.parent.parent,name,ref,c,fresh,env['on'],scene,settings)
        rollouts[name]=r;audits[name]=a;primary[name]=None if r is None else plain(execution_metrics(r,m,fresh))
        selectors[name]=[] if r is None else selection_rows(r,labels)
        if r is None:continue
        if name not in NEW:equal_record(primary[name],historical['primary_metrics'][name])
        restored=read(out/'restoration.json')
        if name in NEW:
            assert read(out/'execution_start.json')['freeze_sha']==read(folder.parents[1]/'execution_start.json')['sha']
            np.testing.assert_array_equal(np.asarray(restored['installed_world'])[p.M+1:],native[spec['original_ids']])
            roundoff[name]=float(np.max(abs(np.asarray(restored['installed_world'])[:p.M+1]-x[:p.M+1])))
            own=plain(evaluate(r,x,env['on'],scene,read(folder/'metric_protocol.json'))) if m is not None else None
            equal_record(own,read(out/'own_reference_metrics.json'))
            secondary[name]=None if own is None else metric_row(own,r['termination'])
            failure |= (r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND' or
                any(e.get('type')=='solve_result' and e.get('status')=='controller_error' for e in r['events']) or
                (primary[name] is not None and primary[name]['execution_clearance_lower_bound_m']<.05))
        for wait in r['logical_waits']:
            assert wait['pose_before']==wait['pose_after'] and wait['sim_before']==wait['sim_after']
            assert wait['release_tick']==release[wait['submit_tick']]
        for e in r['events']:
            if e.get('type')=='solve_result' and e.get('status')=='command':
                assert e['official_generation']==e['result_generation']==c['original_generation']
                if e.get('after_termination'):
                    assert e['withheld_by_logical_scheduler'] and release[e['input_state_id']]>=r['states'][-1]['absolute_tick']
                else:assert e['continuation_seen']['tick']==release[e['input_state_id']]
        if m is not None:assert primary[name]['original_FRESH_endpoint_dwell_s']==independent_endpoint_time(r,fresh)
        gate=dict(initial=r['phase']==c and r['states'][0]['pose_world']==c['B'],steps=len(r['commands']),
            primary=schedule(r,54),full=schedule(r,180))
        gate['passed']=gate['initial'] and gate['steps']==180 and gate['primary']==frozen['primary'] and gate['full']==frozen['full']
        gates[name]=gate
    technical=(pa['solver_success'] or not refs[HERMITE]['safety']['clearance_valid']) and all(a['valid'] for a in audits.values())
    for n,r in rollouts.items():
        if r is not None:
            technical &= gates[n]['passed'] or r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND' or (
                r['termination']=='CONTROLLER_ERROR' and any(e.get('status')=='controller_error' for e in r['events']))
    first=[v[0]['input_pose'] for v in selectors.values() if v]
    assert not first or all(x==first[0] for x in first)
    return plain(dict(primary_metrics=primary,secondary_own_reference_metrics=secondary,bridge_geometry=geom,
        planning=pa,selector=selectors,schedule_gate=dict(passed=all(g.get('passed') for g in gates.values()) and len(gates)==4,methods=gates),
        technical_valid=bool(technical),reference_failure=bool(failure),entry=entry,bridge_input=spec,
        installation_roundoff_max_abs=roundoff,method_folders={n:str(method_folder(folder,n)) for n in ORDER},
        termination={n:refs[n]['status'] if r is None and n in refs else ('PLANNING_FAILED' if r is None else r['termination']) for n,r in rollouts.items()},
        reference_safety={n:r['safety'] for n,r in refs.items()},
        counts=dict(graph_solves=1,new_rollouts=sum(rollouts[n] is not None for n in NEW),
            MPC_solves=sum(0 if rollouts[n] is None else rollouts[n]['new_MPC_solved'] for n in NEW),
            MPC_applied=sum(sum('continuation_seen' in e for e in rollouts[n]['events']) for n in NEW if rollouts[n] is not None),
            historical_rollouts_reused=2,LightNav=0,RGB=0,Isaac=0,retries=0))),dict(valid=all(a['valid'] for a in audits.values()),methods=audits)


def validate(run,deep=True):
    with no_reconciliation_optimizer():
        verify(run);h=authenticate(read(run/'protocol.json'),deep=deep)
        sources={};audits={};selected=read(run/'selected_sources.json')
        assert [s['id'] for s in selected]==SOURCE_IDS
        for s in selected:sources[s['id']],audits[s['id']]=validate_source(Path(s['folder']),h['summary']['sources'][s['id']])
        cross=classify(sources)
        summary=plain(dict(experiment='B_TO_ENTRY_BRIDGE_01',label=read(run/'protocol.json')['label'],
            starting_sha=read(run/'protocol.json')['starting_sha'],scientific_freeze_sha=read(run/'execution_start.json')['sha'],
            selected_sources=selected,sources=sources,cross_source=cross,classification=cross['classification'],
            counts={k:sum(s['counts'][k] for s in sources.values()) for k in next(iter(sources.values()))['counts']}))
        validation=dict(valid=all(a['valid'] for a in audits.values()),sources=audits,
            all_historical_validators=deep,new_scientific_solves=0,exact_fixed_boundaries_suffix=True,
            original_evaluator_and_endpoint_dwell_parity=True)
        if (run/'summary.json').exists():equal_record(summary,read(run/'summary.json'))
        if (run/'result_hashes.json').exists():
            for p,h in read(run/'result_hashes.json').items():assert sha(run/p)==h,p
            tracked=read(RESULTS/'result_summary.json');assert tracked['summary']==summary
            assert tracked['result_hashes_sha256']==sha(run/'result_hashes.json')
            for p,h in tracked['table_hashes'].items():assert sha(RESULTS/p)==h
            assert sorted(p.name for p in (RESULTS/'figures').iterdir())==sorted(PNGS)
            manifest=read(RESULTS/'figure_manifest.json');assert manifest['summary_sha256']==sha(run/'summary.json')
            for f in manifest['figures']:assert sha(RESULTS/'figures'/f['file'])==f['sha256']
        return summary,validation


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    primary=[];geometries=[];comparison=[];selectors=[]
    for sid,q in s['sources'].items():
        for name,r in q['primary_metrics'].items():
            primary.append(dict(source_id=sid,method=name,**({} if r is None else {k:v for k,v in r.items() if k!='endpoint_dwell_trace'})))
        geometries.extend(dict(source_id=sid,method=n,**r) for n,r in q['bridge_geometry'].items())
        comparison.extend(dict(source_id=sid,method=n,**(r or {})) for n,r in s['cross_source']['pairwise'][sid].items())
        selectors.extend(dict(source_id=sid,method=n,**r) for n,rows in q['selector'].items() for r in rows)
    for name,rows in [('primary',primary),('bridge_geometry',geometries),('comparison',comparison),('selector_progress',selectors)]:
        csv_write(RESULTS/(name+'.csv'),rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    if args.check_only:
        s,v=validate(args.run.resolve());print(json.dumps(dict(valid=v['valid'],historical=v['all_historical_validators'],counts=s['counts'],classification=s['classification'])))
    else:write(args.run.resolve())
