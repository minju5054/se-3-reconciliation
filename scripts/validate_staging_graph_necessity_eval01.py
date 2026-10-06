#!/usr/bin/env python3
"""Saved-only source, formulation, schedule, safety and metric validation."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from run_staging_graph_necessity_eval01 import (ROOT,RESULTS,read,save,sha,plain,verify,loader_config,load_problem,METHODS,
    geometry,reference_safety,identity_mapping,PNGS)
from reconciliation.staging_eval_sources import scan,exclusion_registry
from reconciliation.staging_graph_necessity_eval01 import classify
from reconciliation.spatial_entry_suffix import execution_metrics,suffix_reference,no_reconciliation_optimizer
from reconciliation.boundary_row_ablation04 import staged_reference,selector_exposure,reference_geometry
from reconciliation.vector_bridge_execution03 import schedule_check
from reconciliation.relative_factor_multisource import load_source
from reconciliation.b_to_entry_bridge import hermite_bridge
from reconciliation.b_to_entry_graph_diag02 import describe,sample_hermite
from validate_b_to_entry_graph_formulation_diag02 import audit_trace
from validate_relative_factor_multisource01 import validate_method
from validate_spatial_entry_suffix_execution01 import independent_endpoint_time
from validate_robotless_online_handoffs import equal_record


def csv_write(path,rows,fields=None):
    fields=fields or list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');w.writeheader()
        w.writerows({k:json.dumps(v,separators=(',',':'),allow_nan=False) if isinstance(v,(dict,list)) else v for k,v in r.items()} for r in rows)


def selection_audit(run):
    cfg=read(run/'protocol.json');registry=exclusion_registry(ROOT,cfg)
    assert registry==read(run/'development_exclusion_registry.json')
    r=scan(ROOT,cfg,registry)
    assert r['rows']==read(run/'candidate_ledger.json') and r['selection']==read(run/'selection.json')
    assert r['source_hashes']==read(run/'source.json')['source_hashes']
    return r


def validate(run,deep=True):
    with no_reconciliation_optimizer():
        verify(run)
        if deep:selection_audit(run)
        cfg=read(run/'protocol.json');selected=read(run/'selected_sources.json');sources={};counts=dict(
            saved_source_records_inspected=len(read(run/'candidate_ledger.json')),
            eligible_candidates=read(run/'selection.json')['eligible_candidates'],selected_source_count=len(selected),
            new_V2_optimizer_solves=0,V2_retries=0,MPC_solves=0,MPC_applications=0,LightNav=0,RGB=0,Isaac=0,
            new_source_acquisitions=0,historical_scientific_reruns=0,**{n+'_rollouts':0 for n in METHODS})
        assert [s['id'] for s in selected]==read(run/'selection.json')['selected_ids']
        if len(selected)<4:
            assert read(run/'completion.json')['status']=='INSUFFICIENT_SOURCE_DIVERSITY'
            assert not list(run.glob('sources/*/planning/*/optimization_start.json'))
            assert not list(run.glob('sources/*/methods/*/execution_start.json'))
            return dict(scientific_freeze_sha=read(run/'execution_start.json')['sha'],selected_sources=selected,
                sources={},counts=counts,classification=classify({},len(selected))),dict(valid=True,saved_only=True,new_scientific_solves=0)
        full_cap_expected_solves=full_cap_actual_solves=0
        full_cap_expected_apps=full_cap_actual_apps=0
        for spec in selected:
            f=Path(spec['folder']);data=load_source(loader_config(run),spec)
            for key,name in [('common','common_state'),('schedule','schedule'),('manifest','source_manifest')]:
                equal_record(data[key],read(f/(name+'.json')))
            c=data['common'];fresh=data['fresh'];raw=data['raw'];env=geometry(f);entry=read(f/'entry.json')
            w,l,e=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['correspondence']);assert e==entry
            installed=np.load(f/'native_installed.npy');ids=e['row_original_identities'][1:];suffix=np.load(f/'suffix.npy')
            assert suffix[1:].tobytes()==installed[ids].tobytes()
            assert suffix[0].tobytes()==np.asarray(e['correspondence']['target_world']).tobytes()
            refs=read(f/'references.json');mappings=read(f/'final_identity_mapping.json')
            sw,sl,_=staged_reference(installed,raw,c['fresh_capture_pose'],c['B'],entry,suffix)
            assert sw.tobytes()==np.load(refs[METHODS[1]]['world_path']).tobytes()
            assert sl.tobytes()==np.load(refs[METHODS[1]]['local_path']).tobytes()
            h,hm=hermite_bridge(data['past'][-2],c['B'],suffix)
            assert h.tobytes()==np.load(f/'hermite_bridge.npy').tobytes() and hm==read(f/'hermite_input.json')
            initial,_=sample_hermite(data['past'][-2],c['B'],suffix,2)
            assert initial.tobytes()==np.load(f/'initial_M2.npy').tobytes()
            p=load_problem(f,'V2');out=f/'planning/V2';result=read(out/'result.json');trace=read(out/'trace.json')
            start=read(out/'optimization_start.json');assert start['calls']==1 and start['variant']=='V2'
            assert start['freeze_sha']==read(run/'execution_start.json')['sha']
            counts['new_V2_optimizer_solves']+=1
            if trace:last=audit_trace(p,trace,read(out/'feasibility_checks.json'),result,env)
            else:
                assert not result['solver']['converged'] and result['error'] is not None
                last=p.initial[1:-1]
            bridge=p.bridge(last);assert bridge.tobytes()==np.load(out/'last_accepted_bridge.npy').tobytes()
            status=describe(p,last,result['solver'],trace,reference_safety(p.reference(last),env),result['wall_s'])
            equal_record(status,read(out/'status.json'))
            q=dict(technical_valid=True,integration_dt_s=c['integration_dt_s'],methods={},severity=spec['severity'],
                remaining_arc_m=spec['remaining_arc_m'],entry=entry,planning=status,selector={},reference_geometry={},schedule={})
            for n in METHODS:
                ref=refs[n];world=np.load(ref['world_path']);local=np.load(ref['local_path'])
                for frame in ['world','local']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
                equal_record(reference_safety(world,env),ref['safety'])
                assert local[-len(ids):].tobytes()==raw[ids].tobytes()
                assert world[-len(ids):].tobytes()==installed[ids].tobytes()
                assert mappings[n]==identity_mapping(ref['labels'],entry,installed)
                q['reference_geometry'][n]=reference_geometry(world,ref['safety'],ref['labels'])
                mf=f/'methods'/n;valid_ref=ref['safety']['clearance_valid'] and (n!=METHODS[3] or status['stable'])
                metric=None;abort=False;technical=True;r=None
                if not valid_ref:
                    assert not (mf/'rollout.json').exists() and read(mf/'skipped.json')['status']=='SKIPPED_REFERENCE_INVALID'
                    termination='SKIPPED_REFERENCE_INVALID';q['schedule'][n]=None;q['selector'][n]=None
                else:
                    r,m,a=validate_method(f,n,ref,c,fresh,env['on'],data['scene'],data['manifest']['mpc']['official_settings'])
                    technical=a['valid'];termination=r['termination'];abort=termination=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'
                    numerical=any(x.get('status')=='controller_error' for x in r['events'])
                    gate=schedule_check(r,data['schedule'],c);q['schedule'][n]=gate
                    technical &= gate['passed'] or abort or numerical
                    release={v['submit_tick']:v['application_tick'] for v in data['schedule']['pairs']}
                    for wait in r['logical_waits']:
                        assert wait['sim_before']==wait['sim_after'] and wait['pose_before']==wait['pose_after']
                        assert wait['release_tick']==release[wait['submit_tick']]
                    for ev in r['events']:
                        if ev.get('type')=='solve_result' and ev.get('status')=='command':
                            assert ev['result_generation']==ev['official_generation']==c['original_generation']
                            if 'continuation_seen' in ev:assert ev['continuation_seen']['tick']==release[ev['input_state_id']]
                            else:assert ev['withheld_by_logical_scheduler'] and release[ev['input_state_id']]>=r['states'][-1]['absolute_tick']
                    actual=np.asarray(read(mf/'restoration.json')['installed_world'])
                    assert actual[-len(ids):].tobytes()==installed[ids].tobytes()
                    assert reference_safety(actual,env)['clearance_valid']
                    metric=None if m is None else plain(execution_metrics(r,m,fresh))
                    if metric is not None:assert metric['original_FRESH_endpoint_dwell_s']==independent_endpoint_time(r,fresh)
                    q['selector'][n]=selector_exposure(r,mappings[n]);counts[n+'_rollouts']+=1
                    counts['MPC_solves']+=r['new_MPC_solved'];counts['MPC_applications']+=sum('continuation_seen' in v for v in r['events'])
                    if termination=='OBSERVATION_CAP':
                        full_cap_expected_solves+=len(data['schedule']['pairs'])
                        full_cap_actual_solves+=r['new_MPC_solved']
                        full_cap_expected_apps+=len(data['schedule']['full']['applications'])
                        full_cap_actual_apps+=sum('continuation_seen' in v for v in r['events'])
                    assert read(mf/'execution_start.json')['freeze_sha']==read(run/'execution_start.json')['sha']
                    assert read(mf/'execution_start.json')['started']['host_monotonic_ns']>=read(run/'planning_complete.json')['completed']['host_monotonic_ns']
                q['technical_valid'] &= technical
                safe_execution=metric is not None and metric['execution_clearance_lower_bound_m']>=.05 and termination=='OBSERVATION_CAP'
                q['methods'][n]=dict(metrics=metric,technical_valid=technical,valid=bool(valid_ref and safe_execution),
                    planning_valid=valid_ref,unsafe_reference=not ref['safety']['clearance_valid'],safety_abort=abort,
                    nonconvergence=n==METHODS[3] and not status['converged'],termination=termination)
            first=[v['first'] for v in q['selector'].values() if v is not None and v['first'] is not None]
            assert all(v['submit_tick']==first[0]['submit_tick'] and v['input_pose']==first[0]['input_pose'] for v in first)
            sources[spec['id']]=q
        classification=classify(sources,len(selected));summary=plain(dict(scientific_freeze_sha=read(run/'execution_start.json')['sha'],
            selected_sources=selected,sources=sources,counts=counts,classification=classification))
        assert counts['new_V2_optimizer_solves']==len(selected)
        expected=read(run/'expected_calls.json');assert counts['MPC_solves']<=expected['MPC_solves']
        assert full_cap_actual_solves==full_cap_expected_solves
        assert full_cap_actual_apps==full_cap_expected_apps
        return summary,dict(valid=True,source_selection_reproduces=True,saved_only=True,new_scientific_solves=0,
            source_state_schedule_geometry_metrics=True)


def export(run,summary,validation):
    RESULTS.mkdir(parents=True,exist_ok=True)
    primary=[];severity_rows=[];pairwise=[];geometry_rows=[];selectors=[]
    for spec in summary['selected_sources']:
        sid=spec['id'];q=summary['sources'][sid];label=spec['label']
        severity_rows.append(dict(source_id=sid,label=label,remaining_arc_m=q['remaining_arc_m'],**q['severity'],
            **summary['classification']['sources'][sid]))
        for n,r in q['methods'].items():
            m={} if r['metrics'] is None else {k:v for k,v in r['metrics'].items() if k!='endpoint_dwell_trace'}
            primary.append(dict(source_id=sid,label=label,method=n,termination=r['termination'],valid=r['valid'],**m))
            geometry_rows.append(dict(source_id=sid,method=n,**q['reference_geometry'][n]))
            exposure=q['selector'][n]
            selectors.append(dict(source_id=sid,method=n,**({} if exposure is None else {k:v for k,v in exposure.items() if k!='rows'})))
        v=q['methods'][METHODS[3]]['metrics']
        for n in METHODS[1:3]:
            b=q['methods'][n]['metrics'];p=dict(source_id=sid,baseline=n)
            for k in ['position_auc_09_m_s','sustained_attachment_s','original_FRESH_endpoint_dwell_s']:
                p[k]=None if v is None or b is None or v[k] is None or b[k] is None else v[k]-b[k]
            pairwise.append(p)
    csv_write(RESULTS/'primary.csv',primary,['source_id','method'] if not primary else None)
    for name,rows in [('pairwise',pairwise),('severity',severity_rows),('reference_geometry',geometry_rows),('selector_exposure',selectors)]:
        csv_write(RESULTS/(name+'.csv'),rows,['source_id'] if not rows else None)
    ledger=read(run/'candidate_ledger.json')
    compact=[dict(case_id=r['case_id'],episode_id=r['episode_id'],eligible=r['eligible'],gates=r['gates'],rejection_reasons=r['rejection_reasons'],
        geometry_error=r['geometry_error'],severity=r['severity'],remaining_arc_m=r['remaining_arc_m'],suffix_rows=r['suffix_rows'],
        v_minus_mps=r['v_minus_mps'],observation_to_B_travel_m=r['observation_to_B_travel_m']) for r in ledger]
    save(RESULTS/'candidate_ledger.json',compact);csv_write(RESULTS/'candidate_ledger.csv',compact)
    save(RESULTS/'call_accounting.json',summary['counts']);save(RESULTS/'validation.json',validation)
    save(run/'summary.json',summary)
    hashes={str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file() and p.name!='result_hashes.json'}
    save(run/'result_hashes.json',hashes);save(RESULTS/'result_hashes.json',hashes)
    # Keep selector event arrays / execution traces under ignored data; compact tracked metrics only.
    compact_summary=json.loads(json.dumps(summary))
    for q in compact_summary['sources'].values():
        for r in q['methods'].values():
            if r['metrics'] is not None:r['metrics'].pop('endpoint_dwell_trace',None)
        for exposure in q['selector'].values():
            if exposure is not None:exposure.pop('rows',None)
    table_hashes={p.name:sha(p) for p in RESULTS.glob('*.csv')}
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=compact_summary,validation=validation,
        result_hashes_sha256=sha(run/'result_hashes.json'),table_hashes=table_hashes))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check-only',action='store_true')
    a=p.parse_args();run=a.run.resolve();s,v=validate(run)
    if a.check_only:
        equal_record(s,read(run/'summary.json'))
        for rel,h in read(run/'result_hashes.json').items():assert sha(run/rel)==h
        for rel,h in read(RESULTS/'result_summary.json')['table_hashes'].items():assert sha(RESULTS/rel)==h
        if (RESULTS/'figure_manifest.json').exists():
            from report_staging_graph_necessity_eval01 import figure_numbers
            manifest=read(RESULTS/'figure_manifest.json')
            assert manifest['summary_sha256']==sha(run/'summary.json')
            assert sorted(p.name for p in (RESULTS/'figures').glob('*.png'))==sorted(PNGS)
            for fig in manifest['figures']:
                assert sha(RESULTS/'figures'/fig['file'])==fig['sha256']
                equal_record(fig['numeric_sidecar'],figure_numbers(s))
    else:export(run,s,v)
    print(json.dumps(dict(validation=v,classification=s['classification']['classification'],counts=s['counts'])))
