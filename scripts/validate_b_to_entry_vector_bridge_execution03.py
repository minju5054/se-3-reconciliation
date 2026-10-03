#!/usr/bin/env python3
"""Saved-only V2 execution audit: no planner or numerical controller calls."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from run_b_to_entry_vector_bridge_execution03 import ROOT,RESULTS,read,save,sha,plain,verify,authenticate
from reconciliation.vector_bridge_execution03 import *
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.osa03_native import evaluate
from reconciliation.osa03_relative_ablation import metric_row
from validate_relative_factor_multisource01 import validate_method
from validate_spatial_entry_suffix_execution01 import independent_endpoint_time
from validate_robotless_online_handoffs import equal_record


def method_folder(folder, name):
    return folder/'methods'/name if name == VECTOR else Path(read(folder/'reuse.json')['methods'][name]['folder'])


def validate_source(folder, historical, freeze_sha):
    reuse=read(folder/'reuse.json'); c=read(folder/'common_state.json'); b=read(folder/'bridge_input.json')
    refs=read(folder/'references.json'); assert list(refs)==ORDER
    for path, rec in reuse['copies'].items(): assert sha(folder/path)==sha(rec['path'])==rec['sha256']
    planning=read(folder/'planning_reuse.json')
    assert planning['optimizer_calls']==0 and planning['V2_stable']
    for kind, path in [('bridge',folder/'v2_bridge.npy'),('reference',Path(refs[VECTOR]['world_path']))]:
        assert sha(path)==sha(planning['diag_'+kind+'_path'])==planning['diag_'+kind+'_sha256']
    fresh=np.load(refs[NATIVE]['world_path']); raw=np.load(refs[NATIVE]['local_path'])
    world=np.load(refs[VECTOR]['world_path']); bridge=np.load(folder/'v2_bridge.npy')
    w,l,labels=installed_reference(bridge,world,np.load(folder/'suffix_native_installed.npy'),
        c['fresh_capture_pose'],raw,b['original_ids'],c['B'],read(folder/'entry.json')['correspondence']['target_world'])
    assert labels==refs[VECTOR]['labels'];np.testing.assert_array_equal(l,np.load(refs[VECTOR]['local_path']))
    native=np.asarray(read(method_folder(folder,NATIVE)/'restoration.json')['installed_world'])
    assert w[3:].tobytes()==native[b['original_ids']].tobytes()
    env=geometry(folder); frozen=read(folder/'schedule.json'); release={p['submit_tick']:p['application_tick'] for p in frozen['pairs']}
    scene=read(folder/'scenario.json'); manifest=read(folder/'source_manifest.json'); settings=manifest['mpc']['official_settings']
    for path,digest in manifest['hashes'].items(): assert sha(path)==digest
    install=read(folder/'installation.json')
    if refs[VECTOR]['safety']['clearance_valid']:
        pre=read(folder/'restoration_preflight.json'); assert pre['passed'] and pre['numerical_MPC_calls']==0
        assert pre['provenance']['official_settings']==settings and install['performed']
        assert sha(install['installed_path'])==install['installed_sha256']
        equal_record(reference_safety(np.load(install['installed_path']),env),install['installed_safety'])
    metrics={}; selectors={}; audits={}; gates={}; rollouts={}; secondary=None; failure=not refs[VECTOR]['safety']['clearance_valid'];technical=True
    all_labels={NATIVE:[f'F_{j}' for j in range(len(fresh))], ENTRY:[
        'E*' if i is None else f'F_{i}' for i in read(folder/'entry.json')['row_original_identities']],
        HERMITE:refs[HERMITE]['labels'],VECTOR:labels}
    for name in ORDER:
        out=method_folder(folder,name);ref=refs[name]
        for frame in ['local','world']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        equal_record(ref['safety'],reference_safety(np.load(ref['world_path']),env))
        if name!=VECTOR:
            old=reuse['methods'][name];assert ref==old['reference'] and sha(out/'hashes.json')==old['hashes_sha256']
        r,m,a=validate_method(folder if name==VECTOR else out.parent.parent,name,ref,c,fresh,env['on'],scene,settings)
        audits[name]=a;technical &= a['valid'];rollouts[name]=r
        metrics[name]=None if m is None else plain(execution_metrics(r,m,fresh))
        if name!=VECTOR:equal_record(metrics[name],historical['primary_metrics'][name])
        gates[name]=schedule_check(r,frozen,c);selectors[name]=None if r is None else exposure(r,all_labels[name])
        if r is None:continue
        assert r['raw_FRESH_sha256']==manifest['FRESH_sha256']
        numerical_failure=any(e.get('type')=='solve_result' and e.get('status')=='controller_error' for e in r['events'])
        explained=r['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND' or numerical_failure
        technical &= gates[name]['passed'] or explained
        for wait in r['logical_waits']:
            assert wait['sim_before']==wait['sim_after'] and wait['pose_before']==wait['pose_after']
            assert wait['release_tick']==release[wait['submit_tick']]
        for e in r['events']:
            if e.get('type')=='solve_result' and e.get('status')=='command':
                assert e['official_generation']==e['result_generation']==c['original_generation']
                if e.get('after_termination'):
                    assert e['withheld_by_logical_scheduler'] and release[e['input_state_id']]>=r['states'][-1]['absolute_tick']
                else:assert e['continuation_seen']['tick']==release[e['input_state_id']]
        if metrics[name] is not None:assert metrics[name]['original_FRESH_endpoint_dwell_s']==independent_endpoint_time(r,fresh)
        if name==VECTOR:
            assert read(out/'execution_start.json')['freeze_sha']==freeze_sha
            restored=read(out/'restoration.json') if a['valid'] else None
            if restored is not None:
                actual=np.asarray(restored['installed_world']);assert actual[3:].tobytes()==native[b['original_ids']].tobytes()
                assert actual.tobytes()==np.load(install['installed_path']).tobytes()
                assert reference_safety(actual,env)['clearance_valid']
            own=None if m is None else plain(evaluate(r,world,env['on'],scene,read(folder/'metric_protocol.json')))
            equal_record(own,read(out/'own_reference_metrics.json'));secondary=None if own is None else metric_row(own,r['termination'])
            failure |= explained or (metrics[name] is not None and metrics[name]['execution_clearance_lower_bound_m']<.05)
    # Historical schedule and provenance must always pass; censoring never fabricates parity.
    assert all(gates[n]['passed'] for n in ORDER[:-1])
    new=rollouts[VECTOR]
    compared=None if new is None else command_comparison(rollouts[HERMITE],new,all_labels[HERMITE],labels)
    geometries=geometry_pair(np.load(refs[HERMITE]['world_path']),world,{n:refs[n]['safety'] for n in [HERMITE,VECTOR]})
    return plain(dict(primary_metrics=metrics,secondary_own_reference_metrics=secondary,
        selector_exposure=selectors,command_comparison=compared,bridge_geometry=geometries,bridge_input=b,
        integration_dt_s=c['integration_dt_s'],technical_valid=bool(technical),reference_or_execution_failure=bool(failure),
        schedule_gate=dict(passed=all(g['passed'] for g in gates.values()),methods=gates),installation=install,
        method_folders={n:str(method_folder(folder,n)) for n in ORDER},
        counts=dict(optimizer_calls=0,new_V2_rollouts=int(new is not None),MPC_solves=0 if new is None else new['new_MPC_solved'],
            MPC_applications=0 if new is None else sum('continuation_seen' in e for e in new['events']),
            historical_rollouts_reused=3,historical_reruns=0,V3_rollouts=0,LightNav=0,RGB=0,Isaac=0,retries=0))),dict(valid=all(a['valid'] for a in audits.values()),methods=audits)


def validate(run,deep=True):
    with no_reconciliation_optimizer():
        verify(run);d,h=authenticate(read(run/'protocol.json'),deep=deep)
        freeze_sha=read(run/'execution_start.json')['sha'];selected=read(run/'selected_sources.json')
        assert [s['id'] for s in selected]==SOURCE_IDS
        assert read(run/'completion.json')['optimizer_calls']==0
        sources={};audits={}
        for spec in selected:sources[spec['id']],audits[spec['id']]=validate_source(Path(spec['folder']),h['summary']['sources'][spec['id']],freeze_sha)
        cross=classify(sources)
        summary=plain(dict(experiment='B_TO_ENTRY_VECTOR_BRIDGE_EXECUTION_03',starting_sha=read(run/'protocol.json')['starting_sha'],
            scientific_freeze_sha=freeze_sha,diag_freeze_sha=d['summary']['scientific_freeze_sha'],
            diag_result_commit=read(run/'protocol.json')['diag_result_commit'],selected_sources=selected,sources=sources,
            classification=cross['classification'],cross_source=cross,
            counts={k:sum(q['counts'][k] for q in sources.values()) for k in next(iter(sources.values()))['counts']}))
        starts=list((run/'sources').glob('*/methods/*/execution_start.json'))
        assert len(starts)==summary['counts']['new_V2_rollouts']<=4
        assert all(p.parent.name==VECTOR for p in starts) and not list(run.rglob('optimization_start.json'))
        ordered=[read(Path(s['folder'])/'methods'/VECTOR/'execution_start.json')['started'] for s in selected if (Path(s['folder'])/'methods'/VECTOR/'execution_start.json').exists()]
        assert [e['host_monotonic_ns'] for e in ordered]==sorted(e['host_monotonic_ns'] for e in ordered)
        v=dict(valid=all(a['valid'] for a in audits.values()),all_historical_validators=deep,
            exact_saved_V2_reuse=True,optimizer_calls=0,new_scientific_solves=0,source_audits=audits)
        if (run/'summary.json').exists():equal_record(summary,read(run/'summary.json'))
        if (run/'result_hashes.json').exists():
            for p,digest in read(run/'result_hashes.json').items():assert sha(run/p)==digest,p
            tracked=read(RESULTS/'result_summary.json');assert tracked['summary']==summary
            assert tracked['result_hashes_sha256']==sha(run/'result_hashes.json')
            for p,digest in tracked['table_hashes'].items():assert sha(RESULTS/p)==digest
            assert sorted(p.name for p in (RESULTS/'figures').iterdir())==sorted(PNGS)
            manifest=read(RESULTS/'figure_manifest.json');assert manifest['summary_sha256']==sha(run/'summary.json')
            for f in manifest['figures']:assert sha(RESULTS/'figures'/f['file'])==f['sha256']
        return summary,v


def csv_write(path,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys,lineterminator='\n');writer.writeheader()
        for r in rows:writer.writerow({k:json.dumps(v,allow_nan=False) if isinstance(v,(list,dict)) else v for k,v in r.items()})


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    tables={k:[] for k in ['primary','paired_comparison','bridge_geometry','selector_exposure','command_comparison']}
    for sid,q in s['sources'].items():
        for n,m in q['primary_metrics'].items():tables['primary'].append(dict(source_id=sid,method=n,**({} if m is None else {k:v for k,v in m.items() if k!='endpoint_dwell_trace'})))
        tables['paired_comparison'].append(dict(source_id=sid,**s['cross_source']['paired'][sid]))
        for n,g in q['bridge_geometry'].items():tables['bridge_geometry'].append(dict(source_id=sid,method=n,**g))
        for n,e in q['selector_exposure'].items():
            if e is not None:tables['selector_exposure'].append(dict(source_id=sid,method=n,**e))
        if q['command_comparison'] is not None:tables['command_comparison'].extend(dict(source_id=sid,**row) for row in q['command_comparison']['rows'])
    for name,rows in tables.items():csv_write(RESULTS/(name+'.csv'),rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check-only',action='store_true');a=p.parse_args()
    if a.check_only:
        s,v=validate(a.run.resolve());print(json.dumps(dict(valid=v['valid'],historical=v['all_historical_validators'],classification=s['classification'],counts=s['counts'])))
    else:write(a.run.resolve())
