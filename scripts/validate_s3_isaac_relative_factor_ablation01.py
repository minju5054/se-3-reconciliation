#!/usr/bin/env python3
"""Saved-only S3 validation: no planning solve, MPC, SimulationApp or rollout."""
import argparse,csv,json
from pathlib import Path
from contextlib import contextmanager
import sys
import numpy as np
from run_s3_isaac_relative_factor_ablation01 import ROOT,OUT,read,save,sha,plain,verify,namespace
from reconciliation.s3_isaac_ablation01 import ORDER,GRAPHS,AblationGraph,deformation,evaluate,raw_parity,comparisons,classify
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_online02 import guard_check
from reconciliation.online_mpc_adapter import selection_audit
from reconciliation.se2 import local_trajectory_to_world


@contextmanager
def saved_guard():
    old=sys.getprofile()
    def profile(frame,event,arg):
        if event!='call':return
        name=frame.f_code.co_name;file=frame.f_code.co_filename
        if name in {'solve_least_squares','solve_graph','graph_solve','run_method','load_official','_solve'} or '/casadi/' in file or ('simulation_app' in file and name=='__init__'):
            raise RuntimeError('saved-only validator forbids scientific calls: '+name)
    sys.setprofile(profile)
    try:yield
    finally:sys.setprofile(old)


def validate(run):
    verify(run)
    seal=read(OUT/'raw_result_integrity.json');assert seal['run']==str(run) and sha(run/'result_hashes.json')==seal['hash_manifest_sha256']
    for rel,h in read(run/'result_hashes.json').items():assert sha(run/rel)==h,rel
    with saved_guard():return compute(run)


def compute(run):
    cfg=read(run/'protocol.json');c=read(run/'common_state.json');s=read(run/'schedule.json');a=read(run/'source_authentication.json')
    F=np.load(run/'original.npy');S=np.load(run/'suffix.npy');entry=read(run/'entry.json');env=geometry(run)
    refs=read(run/'references.json');execinfo=read(run/'execution_summary.json');planning={};metrics={};traces={};safety={};audits={};rollouts={}
    assert sha(run/'FULL_GRAPH_initial.npy')==sha(run/'GRAPH_NO_R_initial.npy')
    for name in GRAPHS:
        r=read(run/'planning'/name/'result.json');p=AblationGraph(a['P'],c['B'],S,include_relative=cfg['relative_factor'][name])
        initial=np.load(run/f'{name}_initial.npy');assert initial.tobytes()==p.initial().tobytes()
        assert r['initial_costs']==p.costs(p.pack(initial)) and r['optimizer_calls']==1
        if r['candidate_world'] is not None:
            x=np.asarray(r['candidate_world']);assert r['deformation']==deformation(p,x)
            assert r['final_costs']==p.costs(p.pack(x))
            if r['valid']:
                assert r['solver']['converged'] and p.noncollapsed(p.pack(x))
                assert x.tobytes()==np.load(refs[name]['world_path']).tobytes()
        planning[name]=r
    schedules=[];solves=0;gate=None
    for name in ORDER:
        ref=refs.get(name);path=run/'methods'/name/'rollout.json';r=read(path) if path.exists() else None
        rollouts[name]=r;metrics[name]=None
        if ref is not None:
            for frame in ['world','local']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
            world=np.load(ref['world_path']);local=np.load(ref['local_path'])
            assert ref['safety']==reference_safety(world,env)
            np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],local),world,rtol=0,atol=1e-12)
            assert ref['valid']
        safety[name]=dict(reference_status=planning[name]['status'] if name in GRAPHS else 'REFERENCE_SAFE',
            reference_clearance_m=None if ref is None else ref['safety']['minimum_clearance_m'],
            executed=r is not None,execution_clearance_m=None,guard_abort=None,physical_overlap=None)
        if r is None:continue
        assert r['phase']==c
        events=[e for e in r['events'] if e.get('type')=='solve_result'];solves+=len(r['collected_results'])
        assert len(events)==len(r['collected_results']) or r['abort'] is not None or r['error'] is not None
        if r['error'] is not None and not r['states']:
            audits[name]=dict(technical_error=r['error']);continue
        restored=r['controller_restoration']
        if restored is not None:
            assert restored['previous_control']==c['u_mem_B'] and restored['held_command']==c['u_B_plus']
            assert restored['generation']==c['original_generation'] and restored['reference_version']==c['fresh_version']
            assert restored['restored_future_results']==restored['new_solve_calls']==0
            installed=np.asarray(restored['installed_world'])
            assert reference_safety(installed,env)['clearance_valid']
            if name=='RAW':assert installed.tobytes()==np.load(run/'native_installed.npy').tobytes()
        requests=[q['tick'] for q in r['submit_requests']]
        accepted=[e['input_state_id'] for e in r['events'] if e.get('status')=='submitted']
        apps=[q['application_tick'] for q in r['commands'] if q['reason']=='new_solve' and q['application_tick']>c['B_tick']]
        full=r['termination']=='OBSERVATION_CAP'
        if full:
            assert requests==accepted==s['attempted_submit_ticks'] and apps==s['application_ticks']
            assert len(r['commands'])==180 and len(r['states'])==181
        assert accepted==requests[:len(accepted)]
        assert requests==s['attempted_submit_ticks'][:len(requests)]
        assert apps==s['application_ticks'][:len(apps)]
        previous=c['u_mem_B'];states={x['absolute_tick']:x for x in r['states']};release={p['submit_tick']:p['application_tick'] for p in s['pairs']}
        for e in events:
            assert e['status']=='command' and e['official_generation']==e['result_generation']==c['original_generation']
            assert e['previous_command']==previous;previous=e['command']
            assert e['input_pose']==states[e['input_state_id']]['pose_world']
            assert e['continuation_seen']['tick']==release[e['input_state_id']]
            assert selection_audit(installed,e['input_pose'],e['selection']['reference_world'],horizon=5,weights=read(run/'mpc_provenance.json')['official_settings']['Q_WEIGHTS'])==e['selection']
        assert len(r['states'])==len(r['commands'])+1
        for i,cmd in enumerate(r['commands']):
            x,y=r['states'][i:i+2];tick=c['B_tick']+i
            assert x['absolute_tick']==cmd['application_tick']==tick and y['absolute_tick']==tick+1
            assert abs(x['sim_time_s']-(c['B_sim_s']+i*c['integration_dt_s']))<=1e-10
            expected=integrate_unicycle(x['pose_world'],[cmd['v_mps'],cmd['omega_radps']],c['integration_dt_s'])
            np.testing.assert_allclose(y['pose_world'],expected,rtol=0,atol=1e-10)
            g=guard_check(env['on'],np.array(x['pose_world']),cmd,c['integration_dt_s'])
            assert plain(g)==r['guards'][i] and g['safe']
            assert y['Isaac']['USD_readback_error']<=1e-10
        if r['abort'] is not None:
            assert r['abort']['command_applied'] is False and not r['guards'][-1]['safe']
            assert r['abort']['tick']==r['states'][-1]['absolute_tick']
        for wait in r['waits']:assert wait['Isaac_clock_and_pose_unchanged']
        audits[name]=dict(complete=full,attempted_submit_ticks=requests,accepted_submit_ticks=accepted,
            application_ticks=apps,intervals=len(r['commands']),restoration=r['scene_restoration'],error=r['error'])
        schedules.append((requests,accepted,apps,len(r['commands'])))
        if name=='RAW' and r['error'] is None:
            gate=raw_parity(r,read(run/'historical_native.json'),s,cfg['parity']);assert gate==read(run/'RAW_parity.json')
        if r['states']:
            result=plain(evaluate(r,F,env['on'],entry,180*c['integration_dt_s']))
            traces[name]=result.pop('trace');metrics[name]=result
            safety[name].update(execution_clearance_m=result['swept_clearance_lower_bound_m'],guard_abort=result['guard_abort'],physical_overlap=result['physical_overlap'])
    if gate is not None and not gate['passed']:assert set(n for n,r in rollouts.items() if r is not None)=={'RAW'}
    technical=execinfo['classification']=='TECHNICAL_BLOCKED' or any(r is not None and r['error'] is not None for r in rollouts.values())
    valid_metrics={n:m for n,m in metrics.items() if m is not None}
    category=classify(valid_metrics,planning,gate,c['integration_dt_s'],technical)
    calls=dict(Graph_FULL=planning['FULL_GRAPH']['optimizer_calls'],Graph_NO_R=planning['GRAPH_NO_R']['optimizer_calls'],
        SimulationApp=int((run/'isaac_start.json').exists()),launch_attempts=int((run/'launch_attempt.json').exists()),
        rollouts={n:int((run/'methods'/n/'execution_start.json').exists()) for n in ORDER},official_MPC_solves=solves,
        LightNav=0,RGB_model_requests=0,new_source_acquisition=0,Hermite_execution=0,retries=0)
    assert calls['Graph_FULL']==calls['Graph_NO_R']==1 and calls['SimulationApp']<=1 and solves<=120
    summary=dict(experiment=cfg['experiment'],starting_sha=cfg['starting_sha'],scientific_freeze_sha=read(run/'scientific_start.json')['freeze_sha'],
        source_id=cfg['source_id'],classification=category,entry=entry['correspondence'],common=c,schedule=s,
        speed_mps=read(run/'mpc_provenance.json')['effective_linear_velocity_limit_m_s'],planning=planning,RAW_parity=gate,
        primary_metrics=metrics,paired=comparisons(metrics,c['integration_dt_s']),safety=safety,calls=calls,
        schedule_comparable=len(schedules)==4 and all(q==schedules[0] for q in schedules),execution_audits=audits,
        protocol_deviations=[],scope='one historical DEVELOPMENT S3; actual Isaac logical Xform, controlled timing; no robot dynamics')
    return summary,dict(valid=True,saved_only=True,new_scientific_calls=0,raw_parity_pass=None if gate is None else gate['passed'],classification=category),traces


def write_csv(path,rows):
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader()
        for r in rows:w.writerow({k:json.dumps(v,allow_nan=False) if isinstance(v,(dict,list)) else v for k,v in r.items()})


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--write',action='store_true');a=p.parse_args();run=namespace(a.run)
    s,v,traces=validate(run)
    if a.write:
        save(OUT/'result_summary.json',s);save(OUT/'validation_summary.json',v);save(run/'metric_traces.json',traces)
        for name,data in [('planning',s['planning']),('RAW_parity',s['RAW_parity']),('safety',s['safety']),('paired_comparison',s['paired']),('call_accounting',s['calls'])]:save(OUT/(name+'.json'),data)
        write_csv(OUT/'primary.csv',[dict(method=n,**(m or {'status':'NOT_EXECUTED'})) for n,m in s['primary_metrics'].items()])
        write_csv(OUT/'deformation.csv',[dict(method=n,**(r['deformation'] or {'status':r['status']})) for n,r in s['planning'].items()])
    else:
        assert s==read(OUT/'result_summary.json') and v==read(OUT/'validation_summary.json') and traces==read(run/'metric_traces.json')
    print(json.dumps(dict(validation=v,calls=s['calls'])))


if __name__=='__main__':main()
