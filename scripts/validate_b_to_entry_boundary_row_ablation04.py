#!/usr/bin/env python3
"""Saved-only authentication, state/schedule/safety audit and original-FRESH metrics."""
import argparse
import json
from pathlib import Path
import numpy as np
from run_b_to_entry_boundary_row_ablation04 import ROOT,RESULTS,read,save,sha,plain,verify,authenticate
from reconciliation.boundary_row_ablation04 import *
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.osa03_native import evaluate
from reconciliation.osa03_relative_ablation import metric_row
from validate_relative_factor_multisource01 import validate_method
from validate_spatial_entry_suffix_execution01 import independent_endpoint_time
from validate_robotless_online_handoffs import equal_record
from validate_b_to_entry_vector_bridge_execution03 import csv_write


def method_folder(folder,name):
    return folder/'methods'/name if name==STAGE else Path(read(folder/'reuse.json')['methods'][name]['folder'])


def validate_source(folder,historical,freeze_sha):
    reuse=read(folder/'reuse.json');c=read(folder/'common_state.json');entry=read(folder/'entry.json')
    refs=read(folder/'references.json');assert list(refs)==ORDER
    for path,rec in reuse['copies'].items():assert sha(folder/path)==sha(rec['path'])==rec['sha256']
    fresh=np.load(refs[NATIVE]['world_path']);raw=np.load(refs[NATIVE]['local_path'])
    native=np.asarray(read(method_folder(folder,NATIVE)/'restoration.json')['installed_world'])
    w,l,labels=staged_reference(native,raw,c['fresh_capture_pose'],c['B'],entry,np.load(folder/'suffix_native_installed.npy'))
    assert w.tobytes()==np.load(refs[STAGE]['world_path']).tobytes() and l.tobytes()==np.load(refs[STAGE]['local_path']).tobytes()
    assert labels==refs[STAGE]['labels'];ids=entry['row_original_identities'][1:]
    mapping=read(folder/'identity_mapping.json');all_labels={n:[r['label'] for r in mapping[n]] for n in ORDER}
    for n in ORDER:assert mapping[n]==identity_mapping(all_labels[n],entry,native)
    env=geometry(folder);frozen=read(folder/'schedule.json');release={p['submit_tick']:p['application_tick'] for p in frozen['pairs']}
    scene=read(folder/'scenario.json');manifest=read(folder/'source_manifest.json');settings=manifest['mpc']['official_settings']
    for path,digest in manifest['hashes'].items():assert sha(path)==digest
    install=read(folder/'installation.json')
    if refs[STAGE]['safety']['clearance_valid']:
        pre=read(folder/'restoration_preflight.json');assert pre['passed'] and pre['numerical_MPC_calls']==0
        assert pre['provenance']['official_settings']==settings and install['performed']
        assert sha(install['installed_path'])==install['installed_sha256']
        equal_record(reference_safety(np.load(install['installed_path']),env),install['installed_safety'])
    metrics={};selectors={};audits={};gates={};rollouts={};secondary=None
    failure=not refs[STAGE]['safety']['clearance_valid'];technical=True
    for name in ORDER:
        out=method_folder(folder,name);ref=refs[name]
        for frame in ['local','world']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        equal_record(ref['safety'],reference_safety(np.load(ref['world_path']),env))
        if name!=STAGE:
            old=reuse['methods'][name];assert ref==old['reference'] and sha(out/'hashes.json')==old['hashes_sha256']
        r,m,a=validate_method(folder if name==STAGE else out.parent.parent,name,ref,c,fresh,env['on'],scene,settings)
        audits[name]=a;technical &= a['valid'];rollouts[name]=r
        metrics[name]=None if m is None else plain(execution_metrics(r,m,fresh))
        if name!=STAGE:equal_record(metrics[name],historical['primary_metrics'][name])
        gates[name]=schedule_check(r,frozen,c);selectors[name]=None if r is None else selector_exposure(r,mapping[name])
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
                if e.get('after_termination'):assert e['withheld_by_logical_scheduler'] and release[e['input_state_id']]>=r['states'][-1]['absolute_tick']
                else:assert e['continuation_seen']['tick']==release[e['input_state_id']]
        if metrics[name] is not None:assert metrics[name]['original_FRESH_endpoint_dwell_s']==independent_endpoint_time(r,fresh)
        if name==STAGE:
            assert read(out/'execution_start.json')['freeze_sha']==freeze_sha
            if a['valid']:
                actual=np.asarray(read(out/'restoration.json')['installed_world'])
                assert actual[2:].tobytes()==native[ids].tobytes() and actual.tobytes()==np.load(install['installed_path']).tobytes()
                assert reference_safety(actual,env)['clearance_valid']
            own=None if m is None else plain(evaluate(r,w,env['on'],scene,read(folder/'metric_protocol.json')))
            equal_record(own,read(out/'own_reference_metrics.json'));secondary=None if own is None else metric_row(own,r['termination'])
            failure |= explained or (metrics[name] is not None and metrics[name]['execution_clearance_lower_bound_m']<.05)
    assert all(gates[n]['passed'] for n in HISTORICAL)
    first=[selectors[n]['first'] for n in PRIMARY if selectors[n] is not None and selectors[n]['first'] is not None]
    same_first=all(e['submit_tick']==first[0]['submit_tick'] and e['input_pose']==first[0]['input_pose'] for e in first)
    assert same_first,'first legal submit must use identical actual state across references'
    pairs={}
    for left,right in [(ENTRY,STAGE),(STAGE,HERMITE),(STAGE,VECTOR)]:
        pairs[left+'__'+right]=None if rollouts[left] is None or rollouts[right] is None else compare_commands(rollouts[left],rollouts[right],all_labels[left],all_labels[right])
    geometries={n:reference_geometry(np.load(refs[n]['world_path']),refs[n]['safety'],all_labels[n]) for n in ORDER}
    new=rollouts[STAGE]
    return plain(dict(primary_metrics=metrics,secondary_own_reference_metrics=secondary,selector_exposure=selectors,
        command_comparison=pairs,reference_geometry=geometries,entry=entry,boundary_B=c['B'],
        first_legal_submit_state_identical=same_first,integration_dt_s=c['integration_dt_s'],technical_valid=bool(technical),reference_or_execution_failure=bool(failure),
        schedule_gate=dict(passed=all(g['passed'] for g in gates.values()),methods=gates),installation=install,
        method_folders={n:str(method_folder(folder,n)) for n in ORDER},
        counts=dict(optimizer_calls=0,new_B_ENTRY_rollouts=int(new is not None),MPC_solves=0 if new is None else new['new_MPC_solved'],
            MPC_applications=0 if new is None else sum('continuation_seen' in e for e in new['events']),historical_rollouts_reused=4,
            historical_reruns=0,V3_rollouts=0,LightNav=0,RGB=0,Isaac=0,retries=0))),dict(valid=all(a['valid'] for a in audits.values()),methods=audits)


def validate(run,deep=True):
    with no_reconciliation_optimizer():
        verify(run);h=authenticate(read(run/'protocol.json'),deep=deep)
        freeze_sha=read(run/'execution_start.json')['sha'];selected=read(run/'selected_sources.json');assert [s['id'] for s in selected]==SOURCE_IDS
        assert read(run/'completion.json')['optimizer_calls']==0
        sources={};audits={}
        for s in selected:sources[s['id']],audits[s['id']]=validate_source(Path(s['folder']),h['summary']['sources'][s['id']],freeze_sha)
        cross=classify(sources)
        summary=plain(dict(experiment='B_TO_ENTRY_BOUNDARY_ROW_ABLATION_04',starting_sha=read(run/'protocol.json')['starting_sha'],
            scientific_freeze_sha=freeze_sha,historical_result_commit=read(run/'protocol.json')['vector_result_commit'],selected_sources=selected,
            sources=sources,classification=cross['classification'],cross_source=cross,
            counts={k:sum(q['counts'][k] for q in sources.values()) for k in next(iter(sources.values()))['counts']}))
        starts=list((run/'sources').glob('*/methods/*/execution_start.json'))
        assert len(starts)==summary['counts']['new_B_ENTRY_rollouts']<=4
        assert all(p.parent.name==STAGE for p in starts) and not list(run.rglob('optimization_start.json'))
        ordered=[read(Path(s['folder'])/'methods'/STAGE/'execution_start.json')['started']['host_monotonic_ns'] for s in selected if (Path(s['folder'])/'methods'/STAGE/'execution_start.json').exists()]
        assert ordered==sorted(ordered)
        v=dict(valid=all(a['valid'] for a in audits.values()),all_historical_validators=deep,optimizer_calls=0,new_scientific_solves=0,source_audits=audits)
        if (run/'summary.json').exists():equal_record(summary,read(run/'summary.json'))
        if (run/'result_hashes.json').exists():
            for p,digest in read(run/'result_hashes.json').items():assert sha(run/p)==digest,p
            tracked=read(RESULTS/'result_summary.json');assert tracked['summary']==summary and tracked['result_hashes_sha256']==sha(run/'result_hashes.json')
            for p,digest in tracked['table_hashes'].items():assert sha(RESULTS/p)==digest
            assert sorted(p.name for p in (RESULTS/'figures').iterdir())==sorted(PNGS)
            manifest=read(RESULTS/'figure_manifest.json');assert manifest['summary_sha256']==sha(run/'summary.json')
            for f in manifest['figures']:assert sha(RESULTS/'figures'/f['file'])==f['sha256']
        return summary,v


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    tables={k:[] for k in ['primary','structural_comparison','paired_timing','selector_exposure','command_comparison','reference_geometry']}
    for sid,q in s['sources'].items():
        for n,m in q['primary_metrics'].items():
            row=dict(source_id=sid,method=n,**({} if m is None else {k:v for k,v in m.items() if k!='endpoint_dwell_trace'}));tables['primary'].append(row)
            if n in PRIMARY:tables['structural_comparison'].append(dict(row,row_sequence=q['reference_geometry'][n]['labels']))
        tables['paired_timing'].append(dict(source_id=sid,**s['cross_source']['paired'][sid]))
        for n,g in q['reference_geometry'].items():tables['reference_geometry'].append(dict(source_id=sid,method=n,**g))
        for n,e in q['selector_exposure'].items():
            if e is not None:tables['selector_exposure'].append(dict(source_id=sid,method=n,**e))
        for pair,record in q['command_comparison'].items():
            if record is not None:tables['command_comparison'].extend(dict(source_id=sid,pair=pair,**r) for r in record['rows'])
    for name,rows in tables.items():csv_write(RESULTS/(name+'.csv'),rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check-only',action='store_true');a=p.parse_args()
    if a.check_only:
        s,v=validate(a.run.resolve());print(json.dumps(dict(valid=v['valid'],historical=v['all_historical_validators'],classification=s['classification'],counts=s['counts'])))
    else:write(a.run.resolve())
