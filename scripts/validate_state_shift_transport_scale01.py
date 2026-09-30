#!/usr/bin/env python3
"""Saved-only scientific validation. Never starts optimization or controller work."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from run_state_shift_transport_scale01 import ROOT,RESULTS,verify,authenticate,plain
from reconciliation.join_source03 import read,save,sha
from reconciliation.state_shift_transport_scale import *
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.osa03_relative_ablation import planning,metric_row
from reconciliation.local_se2_reconciliation import planning_diagnostics
from reconciliation.osa03_native import evaluate
from reconciliation.se2 import local_trajectory_to_world
from validate_relative_factor_multisource01 import validate_method
from validate_osa03_relative_factor_replication01 import validate_planning
from validate_robotless_online_handoffs import equal_record


def method_folder(folder,name):
    return folder/'methods'/name if name==HALF else Path(read(folder/'reuse.json')['methods'][name]['folder'])


def validate_source(folder):
    c=read(folder/'common_state.json');cfg=read(folder/'protocol.json');reuse=read(folder/'reuse.json')
    old=Path(reuse['source_folder']);env=geometry(folder);scene=read(folder/'scenario.json')
    manifest=read(folder/'source_manifest.json');settings=manifest['mpc']['official_settings']
    for f,a in reuse['copies'].items():assert sha(folder/f)==sha(a['path'])==a['sha256']
    fresh=np.load(folder/'references/M0_NATIVE_world.npy');refs=read(folder/'references.json')
    assert list(refs)==ORDER
    p=TransportScaleProblem(c['fresh_capture_pose'],c['B'],fresh,alpha=.5)
    rollouts={};metrics={};own={};audits={};plans={};costs={};plan_audit={};detail={}
    frozen=read(folder/'schedule.json');release={r['submit_tick']:r['application_tick'] for r in frozen['pairs']}
    for name in ORDER:
        ref=refs[name];where=folder if name==HALF else old;out=method_folder(folder,name)
        if name!=HALF:
            assert ref==read(old/'references.json')[name]==reuse['methods'][name]['reference']
            assert sha(out/'hashes.json')==reuse['methods'][name]['hashes_sha256']
        if ref['status']=='PLANNING_FAILED':
            assert name==HALF and read(out/'skipped.json')['status']=='PLANNING_FAILED'
            assert not (out/'rollout.json').exists()
            assert read(folder/'planning'/HALF/'planning_result.json')['error'] is not None
            rollouts[name]=metrics[name]=own[name]=plans[name]=None;audits[name]={'valid':True};continue
        for frame in ['world','local']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        x=np.load(ref['world_path']);local=np.load(ref['local_path'])
        np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],local),x,rtol=0,atol=1e-12)
        equal_record(ref['safety'],reference_safety(x,env));equal_record(ref['descriptor'],plain(planning(fresh,x,c['B'])))
        plans[name]=ref['descriptor']
        problem=TransportScaleProblem(c['fresh_capture_pose'],c['B'],fresh,alpha=ALPHAS[ORDER.index(name)])
        costs[name]=problem.costs(x);detail[name]=plain(planning_diagnostics(problem,x))
        if name==NATIVE:
            np.testing.assert_array_equal(x,fresh)
            assert costs[name]['total']<1e-24
        elif name==HALF:
            plan_audit[name]=validate_planning(folder/'planning'/HALF,p,x,env,True)
            start=read(folder/'planning'/HALF/'optimization_start.json')
            assert start['alpha']==.5 and start['calls']==1 and start['include_relative']
            assert start['initial_world_sha256']==refs[NATIVE]['world_sha256'] and start['solver_config']==cfg['solver']
        r,m,audit=validate_method(where,name,ref,c,fresh,env['on'],scene,settings)
        rollouts[name]=r;metrics[name]=m;audits[name]=audit;own[name]=None
        if r is None:continue
        assert r['raw_FRESH_sha256']==manifest['FRESH_sha256']
        for w in r['logical_waits']:
            assert w['sim_before']==w['sim_after'] and w['pose_before']==w['pose_after']
            assert w['release_tick']==release[w['submit_tick']]
        for e in r['events']:
            if e.get('type')=='solve_result' and e.get('status')=='command':
                assert e['official_generation']==e['result_generation']==c['original_generation']
                if not e.get('after_termination'):assert e['continuation_seen']['tick']==release[e['input_state_id']]
                else:assert e.get('withheld_by_logical_scheduler') and release[e['input_state_id']]>=r['states'][-1]['absolute_tick']
        if m is not None:
            om=plain(evaluate(r,x,env['on'],scene,read(folder/'metric_protocol.json')))
            equal_record(om,read(out/'own_reference_metrics.json'))
            own[name]={k:v for k,v in metric_row(om,r['termination']).items() if k in
                       ['position_auc_03_m_s','position_auc_09_m_s','yaw_auc_03_rad_s','yaw_auc_09_rad_s','max_position_error_05_m']}
    primary={n:metric_row(metrics[n],None if rollouts[n] is None else rollouts[n]['termination']) for n in ORDER}
    # Exact saved historical metrics and summaries, including nulls, remain unchanged.
    historical=read(old.parents[1]/'summary.json')['sources']
    sid=next(s for s in historical if s.replace('/','__')==folder.name)
    for n in [NATIVE,FULL]:equal_record(primary[n],historical[sid]['primary_metrics'][n])
    pair={}
    for name in [NATIVE,FULL]:
        if 'world_path' not in refs[HALF]:pair[name]=None;continue
        x,y=[np.load(refs[n]['world_path']) for n in [HALF,name]]
        z=dict(max_reference_XY_gap_m=float(np.linalg.norm(x[:,:2]-y[:,:2],axis=1).max()))
        if rollouts[HALF] and rollouts[name]:
            k=min(len(rollouts[n]['states']) for n in [HALF,name])
            a,b=[np.array([s['pose_world'] for s in rollouts[n]['states'][:k]]) for n in [HALF,name]]
            z['max_execution_XY_gap_m']=float(np.linalg.norm(a[:,:2]-b[:,:2],axis=1).max())
        pair[name]=z
    q=dict(source_id=sid,transport=plain(transport_diagnostics(p)),primary_metrics=primary,
        secondary_own_reference_metrics=own,planning=plans,planning_node_edge_diagnostics=detail,
        factor_costs=costs,planning_audits=plan_audit,reference_safety={n:refs[n]['safety'] for n in ORDER},
        schedule_gate=schedule_gate(rollouts,frozen,c),selector=selectors(rollouts,primary),pairwise=pair,
        termination={n:refs[n]['status'] if r is None else r['termination'] for n,r in rollouts.items()},
        endpoints={n:None if m is None else m['endpoint'] for n,m in metrics.items()},
        counts=dict(new_planning=1,new_rollouts=int(rollouts[HALF] is not None),
            new_MPC=0 if rollouts[HALF] is None else rollouts[HALF]['new_MPC_solved'],
            reused_references=2,reused_rollouts=2,reused_metrics=2,reused_planning_solutions=1,
            LightNav=0,RGB=0,Isaac=0,retries=0))
    return q,dict(valid=all(v['valid'] for v in audits.values()),methods=audits)


def validate(run):
    verify(run);cfg=read(run/'protocol.json');authenticate(cfg,deep=True)
    selected=read(run/'selected_sources.json');assert [s['id'] for s in selected]==SOURCE_IDS
    sources={};audits={}
    for spec in selected:sources[spec['id']],audits[spec['id']]=validate_source(Path(spec['folder']))
    cross=cross_source(sources,cfg['sign_tolerance'])
    if not all(v['valid'] for v in audits.values()):cross['classification']='TECHNICAL_BLOCKED'
    s=dict(experiment=cfg['experiment'],label=cfg['label'],starting_sha=cfg['starting_sha'],
        scientific_freeze_sha=read(run/'execution_start.json')['sha'],selected_sources=selected,sources=sources,
        cross_source=cross,classification=cross['classification'],
        counts={k:sum(q['counts'][k] for q in sources.values()) for k in next(iter(sources.values()))['counts']})
    return s,dict(valid=all(v['valid'] for v in audits.values()),sources=audits,
        historical_multisource_R00_R01_validators=True,new_scientific_calls=0)


def csv_write(path,rows):
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fields,lineterminator='\n');w.writeheader();w.writerows(rows)


def write(run):
    s,v=validate(run);save(run/'summary.json',s);save(run/'validation.json',v)
    csv_write(RESULTS/'primary.csv',[dict(source_id=sid,method=n,alpha=ALPHAS[ORDER.index(n)],
        **(row or {'termination_reason':q['termination'][n]})) for sid,q in s['sources'].items() for n,row in q['primary_metrics'].items()])
    comparisons=[];selector_rows=[]
    for sid,q in s['sources'].items():
        row=dict(source_id=sid,body_log_translation_m=q['transport']['body_log_translation_m'])
        for n,r in q['primary_metrics'].items():
            for k in PRIMARY_KEYS:row[n+'_'+k]=None if r is None else r[k]
        row.update({'Half_minus_Full_'+k:v for k,v in s['cross_source']['deltas'][sid].items()})
        row['schedule_gate']=q['schedule_gate']['passed'];comparisons.append(row)
        for n,rs in q['selector']['methods'].items():
            selector_rows.extend(dict(source_id=sid,method=n,**{k:json.dumps(v) if isinstance(v,list) else v for k,v in r.items()}) for r in rs)
    csv_write(RESULTS/'transport_scale_comparison.csv',comparisons)
    csv_write(RESULTS/'selector_diagnostics.csv',selector_rows)
    print(json.dumps(dict(valid=v['valid'],classification=s['classification'],counts=s['counts'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();write(a.run.resolve())
