#!/usr/bin/env python3
"""Prepare, freeze, execute once and audit saved-state 2x2 counterfactuals."""
from __future__ import annotations
import argparse,csv,hashlib,json,os,subprocess,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.handoff_delay_attribution import (IDS,CONDITIONS,DT,STEPS,AUC,boundary_state,fixed_references,metrics,paired,value_hash)
from reconciliation.gp_se2_rollout import load_frozen_context
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_online02 import guard_check
from reconciliation.robotless_online import integrate_unicycle
from genuine_source_scan_gui import SavedSources,recorded_fresh_lifetime
from run_join_online02 import environments
CONFIG=ROOT/'configs/handoff_delay_attribution_01.yaml'

def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def plain(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if isinstance(v,dict):return {k:plain(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [plain(x) for x in v]
    return v

def save(p,v):
    Path(p).parent.mkdir(parents=True,exist_ok=True)
    with Path(p).open('x') as f:json.dump(plain(v),f,indent=2,allow_nan=False);f.write('\n')
def csvread(p):
    with Path(p).open() as f:return list(csv.DictReader(f))
def csvsave(p,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(p).open('x',newline='') as f:
        w=csv.DictWriter(f,keys);w.writeheader();w.writerows(rows)
def git(*a):return subprocess.check_output(['git','-C',str(ROOT),*a],text=True).strip()
def config():return yaml.safe_load(CONFIG.read_text())
def envs(cfg):
    scan=read(ROOT/cfg['scan_run']/'source.json')
    return {'GENUINE_13':HospitalEnvironment.load(scan['environment_path']),
            'ONLINE03_ONSET':environments(ROOT/cfg['online_run'])[1]}

def code_files():
    names=['src/reconciliation/handoff_delay_attribution.py','scripts/run_handoff_delay_attribution.py',
      'scripts/lightnav/handoff_delay_mpc_worker.py','scripts/plot_handoff_delay_attribution.py',
      'tests/test_handoff_delay_attribution.py','configs/handoff_delay_attribution_01.yaml',
      'src/reconciliation/gp_se2_ref02_reference.py','src/reconciliation/gp_se2_ref02_rollout.py',
      'src/reconciliation/gp_se2_ref02_evaluation.py','src/reconciliation/gp_se2_ref01_reference.py',
      'src/reconciliation/gp_se2_reference.py','src/reconciliation/gp_se2_rollout.py',
      'src/reconciliation/gp_se2_environment.py','src/reconciliation/gp_se2_join01.py',
      'src/reconciliation/handoff_execution_loss.py','src/reconciliation/robotless_online.py',
      'src/reconciliation/se2.py','src/reconciliation/online_mpc_adapter.py','src/reconciliation/join_online02.py',
      'src/reconciliation/join_source02_geometry.py','scripts/run_join_online02.py',
      'scripts/isaac/genuine_source_scan_gui.py','src/reconciliation/online_handoff_analysis.py']
    return [ROOT/x for x in names]

def construct(cfg):
    saved=SavedSources(ROOT/cfg['scan_run'],True)
    assert tuple(saved.ids)==IDS and tuple(cfg['conditions'])==CONDITIONS
    hashes=dict(saved.hashes); hashes.update(saved.source['source_hashes']); hashes.update(saved.source['code_hashes'])
    environments_=envs(cfg);events=[]
    specs=[('GENUINE_13',ROOT/cfg['corpus_run'],cid,*cid.split('/'),c) for cid,c in zip(IDS,saved.cases)]
    online=ROOT/cfg['online_run']
    for ep in ['ON_REPEAT_00','ON_REPEAT_01']:
        contexts=[(p,read(p)) for p in sorted((online/'episodes'/ep/'handoffs').glob('*/context.json'))]
        for i in range(1,5):
            match=[(p,c) for p,c in contexts if c['fresh_chunk_id']==f'chunk_{i:03d}']
            if len(match)!=1:
                specs.append(('ONLINE03_ONSET',online,f'{ep}/C{i}',ep,None,None));continue
            specs.append(('ONLINE03_ONSET',online,f'{ep}/C{i}',ep,match[0][0].parent.name,None))
    for cohort,root,cid,ep,hid,prior in specs:
        if hid is None:
            events.append(dict(case_id=cid,episode_id=ep,cohort=cohort,available=False,reason='MISSING_APPLIED_HANDOFF'));continue
        f=load_frozen_context(root,ep,hid);folder=root/'episodes'/ep
        c=read(folder/'handoffs'/hid/'context.json');states=csvread(folder/'execution.csv');commands=csvread(folder/'commands.csv')
        logs=[json.loads(l) for l in (folder/'controller/events.jsonl').read_text().splitlines()]
        for r in f['source_files']:hashes[str(root/r['path'])]=r['sha256']
        meta_path=folder/'chunks'/c['fresh_chunk_id']/'metadata.json';meta=read(meta_path)
        hashes[str(meta_path)]=sha(meta_path)
        obs=meta['observation'];assert obs['pose_world']==c['R_obs'] and obs['capture_monotonic_ns']/1e9==c['t_obs']['host_monotonic_s']
        hashes[str(folder/obs['path'])]=sha(folder/obs['path']);assert hashes[str(folder/obs['path'])]==obs['sha256']
        life=recorded_fresh_lifetime(c,states,commands,np.asarray(f['old_world']),np.asarray(f['fresh_world']))
        starts={x:boundary_state(c,states,commands,logs,x) for x in ['DELAYED','LATENCY_FREE']}
        refs=fixed_references(f['fresh_world'],c['B'])
        if prior:assert refs['common_hash']==prior['row']['common_value_sha256']
        rawcheck=environments_[cohort].check_polyline(f['fresh_world'])
        dt=np.diff(life['times']); enough=len(dt)>=STEPS
        if enough and not np.allclose(dt[:STEPS],DT,atol=1e-12,rtol=0):raise ValueError('source exposure is not frozen integration dt')
        reasons=[]
        if not enough:reasons.append('INSUFFICIENT_SOURCE_EXPOSURE')
        if cohort=='ONLINE03_ONSET':
            if not rawcheck['clearance_valid']:reasons.append('FRESH_REQUIRED_CLEARANCE_FAILED')
            if starts['DELAYED']['u_minus'][0]<=0:reasons.append('NO_MOVING_B')
        obs_i=c['obs_state_id'];b_i=c['switch_state_id']
        past=[[float(r[k]) for k in ('x','y','yaw')] for r in states if obs_i<=int(r['state_id'])<=b_i]
        events.append(dict(case_id=cid,episode_id=ep,cohort=cohort,available=not reasons,reason=';'.join(reasons) or None,
            source_root=str(root),handoff_id=hid,context=c,frozen=f,initials=starts,references=refs,
            fresh_world=f['fresh_world'],old_world=f['old_world'],past= past,raw_fresh_check=rawcheck,
            source_lifetime_steps=len(dt),source_lifetime_s=float(life['times'][-1]-life['times'][0]),
            primary_duration_s=STEPS*DT,original_timing_flags=c['timing_flags'],
            source_role='ENDPOINT_CAVEAT' if prior and prior['row']['mismatch']['projection']['projection_location']!='interior' else ('SAFE_PARTIAL_BYPASS_ONSET' if not prior else 'INTERIOR'),
            legacy_pre_FRESH_memory=f['previous_control'],actual_B_memory_policy=cfg['initial_state_policy']))
    # Preserve environment, authoritative source validation and external core.
    extra=[ROOT/cfg['scan_run']/'validation.json',online/'validation.json',online/'scenario.json',online/'source.json',online/'config_snapshot.yaml',
           ROOT/cfg['saved_loss_run']/'validation.json',Path(cfg['checkout'])/'mujoco_demo/vln_mujoco/mpc.py']
    _,_,_,on_source=environments(online)
    extra += [Path(read(online/'scenario.json')['source03_input'])]
    for envpath in [Path(saved.source['environment_path']),Path(on_source['environment_export'])]:
        extra.extend(p for p in envpath.rglob('*') if p.is_file())
    for p in extra:hashes[str(p)]=sha(p)
    assert sha(Path(cfg['checkout'])/'mujoco_demo/vln_mujoco/mpc.py')==cfg['mpc_sha256']
    assert read(online/'validation.json')['valid']
    return plain(events),hashes

def availability(events):
    rows=[]
    for e in events:
        for condition in CONDITIONS:
            key='LATENCY_FREE' if condition.startswith('LATENCY_FREE') else 'DELAYED'
            initial=e.get('initials',{}).get(key,{})
            ok=e['available'] and initial.get('available',False)
            rows.append(dict(case_id=e['case_id'],cohort=e['cohort'],condition=condition,available=ok,
                             reason=None if ok else (e['reason'] or initial.get('reason','MISSING_STATE'))))
    return rows

def prepare(run):
    run.mkdir(parents=True,exist_ok=False);cfg=config();begin=time.perf_counter()
    events,hashes=construct(cfg)
    save(run/'events.json',events);save(run/'protocol.json',cfg)
    save(run/'source_manifest.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),hashes=hashes,
                                       scientific_VLA_calls=0,source_prepare_wall_s=time.perf_counter()-begin,unrelated_edit_hashes={str(ROOT/p):sha(ROOT/p) for p in ['configs/stage0_jackal_controller_validation.yaml','configs/stage0_lightnav_single_chunk.yaml']}))
    save(run/'controller_state_reconstruction.json',[dict(case_id=e['case_id'],states=e.get('initials'),legacy_pre_FRESH_memory=e.get('legacy_pre_FRESH_memory')) for e in events])
    csvsave(run/'cohort.csv',[{k:e.get(k) for k in ['case_id','episode_id','cohort','available','reason','source_role','source_lifetime_steps','source_lifetime_s']} for e in events])
    av=availability(events);csvsave(run/'availability.csv',av)
    print(json.dumps(dict(events=len(events),conditions=len(av),available=sum(r['available'] for r in av),reasons=[r for r in av if not r['available']])))

def freeze(run):
    assert git('rev-parse','HEAD')==git('rev-parse','origin/main'),'push freeze commit before binding run'
    paths=code_files()
    dirty=set(git('diff','--name-only').splitlines())|set(git('diff','--cached','--name-only').splitlines())
    assert not dirty.intersection(str(p.relative_to(ROOT)) for p in paths)
    save(run/'freeze.json',dict(execution_sha=git('rev-parse','HEAD'),code_hashes={str(p):sha(p) for p in paths},
        input_hashes={str(p):sha(p) for p in run.rglob('*') if p.is_file()},planned_rollouts=sum(r['available']=='True' for r in csvread(run/'availability.csv')),
        planned_maximum_solves=9*sum(r['available']=='True' for r in csvread(run/'availability.csv'))))

def verify(run):
    f=read(run/'freeze.json')
    for p,h in {**f['code_hashes'],**f['input_hashes'],**read(run/'source_manifest.json')['hashes'],**read(run/'source_manifest.json')['unrelated_edit_hashes']}.items():
        if sha(p)!=h:raise ValueError('preservation mismatch: '+p)
    return f

class Worker:
    def __init__(self,cfg,log):
        self.transcript=log.with_suffix('.jsonl').open('x');self.log=log.open('x');self.p=subprocess.Popen([cfg['mpc_python'],str(ROOT/'scripts/lightnav/handoff_delay_mpc_worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
    def call(self,q):
        self.transcript.write(json.dumps({'direction':'request','record':q})+'\n');self.transcript.flush()
        self.p.stdin.write(json.dumps(q)+'\n');self.p.stdin.flush();line=self.p.stdout.readline()
        self.transcript.write(json.dumps({'direction':'response','line':line})+'\n');self.transcript.flush()
        if not line:raise RuntimeError('worker ended: '+str(self.p.poll()))
        r=json.loads(line)
        if not r['ok']:raise RuntimeError(r['error'])
        return r['result']
    def close(self):
        self.p.stdin.close();self.p.wait(timeout=30);self.log.close();self.transcript.close()

def execute(run):
    f=verify(run);assert git('rev-parse','HEAD')==f['execution_sha'];assert git('rev-parse','origin/main')==f['execution_sha']
    cfg=read(run/'protocol.json');events=read(run/'events.json');env=envs(cfg)
    save(run/'execution_start.json',dict(sha=f['execution_sha'],wall_start=time.time(),scientific_VLA_calls=0))
    worker=Worker(cfg,run/'mpc_worker.log');begin=time.perf_counter()
    try:
        for e in events:
            for av in availability([e]):
                condition=av['condition'];folder=run/'rollouts'/e['case_id'].replace('/','__');path=folder/(condition+'.json')
                if not av['available']:
                    save(path,dict(condition=condition,status='UNAVAILABLE',reason=av['reason'],times_s=[],poses_world=[],commands=[],solves=[]));continue
                initial=e['initials']['LATENCY_FREE' if condition.startswith('LATENCY_FREE') else 'DELAYED']
                refs=e['references'];look=condition.endswith('LOOKAHEAD');ref=refs['common' if look else 'native']
                save(folder/(condition+'_attempt.json'),dict(condition=condition,started=time.time(),planned_solves=9,no_retry=True))
                result=dict(condition=condition,status='RUNNING',initial=initial,times_s=[0.],poses_world=[initial['pose_world']],commands=[],solves=[],solve_attempts=[],guards=[],reason=None)
                start=time.perf_counter();initialized=False
                try:
                    result['installation']=worker.call(dict(op='init',checkout=cfg['checkout'],method='C_DENSE_SOURCE_PROGRESS' if look else 'A_NATIVE',
                        reference=ref,lineage=refs['common_lineage' if look else 'native_lineage'],goal_index=len(e['fresh_world'])-1,
                        constant=refs['constant_metadata'] if look else None,capture=e['frozen']['original_capture_pose_world'],
                        raw_local=e['frozen']['fresh_raw_local'],memory=initial['memory']['previous_control'],physical=initial['u_minus']))
                    initialized=True;pose=np.array(initial['pose_world']);u=initial['u_minus']
                    for tick in range(STEPS):
                        if tick%6==0:
                            result['solve_attempts'].append(dict(tick=tick,time_s=tick*DT,pose_world=pose.tolist()))
                            solve=worker.call(dict(op='solve',pose=pose.tolist()));solve.update(tick=tick,time_s=tick*DT);result['solves'].append(solve);u=solve['command']
                        command=dict(v_mps=u[0],omega_radps=u[1],reason='new_solve' if tick%6==0 else 'hold')
                        guard=guard_check(env[e['cohort']],pose,command,DT);guard.update(tick=tick,time_s=tick*DT);result['guards'].append(guard)
                        if not guard['safe']:
                            result.update(status='SAFETY_ABORT',reason='UNSAFE_PROPOSED_HELD_COMMAND',abort=dict(tick=tick,time_s=tick*DT,pose=pose.tolist(),proposed_command=u,applied=False));break
                        pose=integrate_unicycle(pose,u,DT);result['commands'].append(u);result['poses_world'].append(pose.tolist());result['times_s'].append((tick+1)*DT)
                    else:result['status']='COMPLETED'
                except Exception:result.update(status='TECHNICAL_FAILURE',reason=traceback.format_exc())
                finally:
                    if initialized:
                        try:worker.call(dict(op='close'))
                        except Exception:result['close_error']=traceback.format_exc()
                result['wall_s']=time.perf_counter()-start;save(path,result)
                print(e['case_id'],condition,result['status'],len(result['solves']),flush=True)
    finally:worker.close()
    save(run/'execution_end.json',dict(wall_s=time.perf_counter()-begin,scientific_VLA_calls=0))

def collect(run):
    cfg=read(run/'protocol.json');environments_=envs(cfg);rows=[]
    for e in read(run/'events.json'):
        for c in CONDITIONS:
            r=read(run/'rollouts'/e['case_id'].replace('/','__')/(c+'.json'))
            rows.append(metrics(r,e,environments_[e['cohort']],cfg))
    return plain(rows)

def flat(r):
    p=r['primary'] or {};c=r['command'] or {};g=r['geometry'] or {}
    return {**{k:r[k] for k in ['case_id','episode_id','cohort','condition','status','complete','reason','integration_steps','solve_count','official_mpc_wall_s','rollout_wall_s','controller_failures']},
            **{k:p.get(k) for k in [*AUC,'observation_status','join_time_s','first_tube_entry_time_s']},
            **{k:c.get(k) for k in ['linear_TV_mps','angular_TV_radps','nominal_command_grid_valid','nominal_10Hz_max_delta_v_over_dt_mps2','nominal_10Hz_max_delta_omega_over_dt_radps2']},
            **{k:g.get(k) for k in ['minimum_clearance_m','minimum_clearance_lower_bound_m','clearance_valid','physical_overlap','workspace_known']}}

def summarize(rows,gaps,transitions):
    output={}
    for cohort in ['GENUINE_13','ONLINE03_ONSET']:
        rr=[r for r in rows if r['cohort']==cohort];gg=[g for g in gaps if g['cohort']==cohort];stats={}
        for comparison in dict.fromkeys(g['comparison'] for g in gg):
            pairs=[g for g in gg if g['comparison']==comparison and g['available']];episodes=sorted(set(g['episode_id'] for g in pairs))
            stats[comparison]=dict(pairs=len(pairs),episodes=len(episodes),event_mean={k:float(np.mean([g[k] for g in pairs])) if pairs else None for k in AUC},
                episode_equal_mean={k:float(np.mean([np.mean([g[k] for g in pairs if g['episode_id']==ep]) for ep in episodes])) if episodes else None for k in AUC},
                position_positive=sum(g[AUC[0]]>0 for g in pairs),position_negative=sum(g[AUC[0]]<0 for g in pairs),
                both_safe_motion_valid=sum(g['both_safe_motion_valid'] for g in pairs))
        output[cohort]=dict(events=len(set(r['case_id'] for r in rr)),complete_rollouts=sum(r['complete'] for r in rr),
            condition_summary={c:dict(complete=sum(r['complete'] for r in rr if r['condition']==c),
              observed_join=sum(bool(r['primary'] and r['primary']['join_success']) for r in rr if r['condition']==c),
              motion_invalid=sum(bool(r['command'] and not r['command']['nominal_command_grid_valid']) for r in rr if r['condition']==c)) for c in CONDITIONS},
            paired=stats,attachment_transitions=[t for t in transitions if t['cohort']==cohort])
    return dict(cohorts=output,scientific_VLA_calls=0,new_GP_rigid_graph_calls=0,actual_MPC_solves=sum(r['solve_count'] for r in rows),
                official_MPC_wall_s=sum(r['official_mpc_wall_s'] for r in rows),
                aborts=sum(r['status']=='SAFETY_ABORT' for r in rows),technical_failures=sum(r['status']=='TECHNICAL_FAILURE' for r in rows),
                interpretation_policy='Human interpretation in report and interpretation.json; no automatic score',not_recoverable_maximum=True)

def analyze(run):
    verify(run);rows=collect(run);gaps,transitions=paired(rows)
    save(run/'event_records.json',rows);csvsave(run/'event_metrics.csv',[flat(r) for r in rows]);csvsave(run/'paired_delay_gaps.csv',gaps);csvsave(run/'attachment_transitions.csv',transitions)
    save(run/'summary.json',summarize(rows,gaps,transitions))
    from plot_handoff_delay_attribution import render
    render(run)

def validate(run):
    begin=time.perf_counter();verify(run);cfg=read(run/'protocol.json');events,hashes=construct(cfg)
    assert events==read(run/'events.json'),'input/state/reference reconstruction differs'
    assert hashes==read(run/'source_manifest.json')['hashes']
    environments_=envs(cfg);unique=[];instance_ordinals=[];solve_ordinals=[]
    for e in events:
        installs={}
        for av in availability([e]):
            c=av['condition'];r=read(run/'rollouts'/e['case_id'].replace('/','__')/(c+'.json'))
            if not av['available']:assert r['status']=='UNAVAILABLE';continue
            assert r['initial']==e['initials']['LATENCY_FREE' if c.startswith('LATENCY_FREE') else 'DELAYED']
            assert all(r['installation']['identity'].values())
            unique.append((e['case_id'],c));instance_ordinals.append(r['installation']['instance_ordinal']);solve_ordinals.extend(s['worker_solve_ordinal'] for s in r['solves'])
            assert r['installation']['provenance']['mpc_source_sha256']==cfg['mpc_sha256']
            expected=np.array(e['references']['common' if c.endswith('LOOKAHEAD') else 'native'])
            np.testing.assert_allclose(r['installation']['installed'],expected,atol=1e-11,rtol=0)
            interface=c.rsplit('_',1)[1];h=r['installation']['installed_hash']
            if interface in installs:assert installs[interface]==h
            installs[interface]=h
            assert r['times_s']==(np.arange(len(r['commands'])+1)*DT).tolist()
            assert len(r['solve_attempts'])==len(r['solves'])
            assert [s['tick'] for s in r['solves']]==list(range(0,len(r['commands'])+(r['status']=='SAFETY_ABORT'),6))
            memory=r['initial']['memory']['previous_control']
            from reconciliation.gp_se2_ref02_reference import enrich_selector_result
            from reconciliation.se2 import relative_pose
            # No official module/solver import: independent NumPy selector audit
            # enrich_selector_result validates actual selected values/index semantics.
            for s in r['solves']:
                assert s['previous_control']==memory;memory=s['previous_control_after']
                assert s['input_pose_world']==r['poses_world'][s['tick']]
                look=c.endswith('LOOKAHEAD');refs=e['references']
                audited=enrich_selector_result(np.array(r['installation']['installed']),s['input_pose_world'],s['actual_submitted_reference_world'],refs['common_lineage' if look else 'native_lineage'],
                    method='C_DENSE_SOURCE_PROGRESS' if look else 'A_NATIVE',weights=[10.,10.,1.],horizon=5,original_goal_row_index=len(e['fresh_world'])-1,
                    selection_details=s['selection_diagnostic'],constant_reference_metadata=refs['constant_metadata'] if look else None)
                assert audited['indices']==s['selection']['indices']
                np.testing.assert_allclose(relative_pose(s['input_pose_world'],s['actual_submitted_reference_world']),s['actual_controller_reference_local'],atol=1e-11,rtol=0)
            for guard in r['guards']:
                again=guard_check(environments_[e['cohort']],guard['start_pose'],guard['command'],DT)
                assert all(again[k]==guard[k] for k in again)
                tick=guard['tick'];assert guard['start_pose']==r['poses_world'][tick]
                s=r['solves'][tick//6]
                assert [guard['command']['v_mps'],guard['command']['omega_radps']]==s['command']
                if guard['safe']:assert r['commands'][tick]==s['command']
                else:assert tick==len(r['commands']) and not r['abort']['applied']
    assert len(unique)==len(set(unique))
    assert instance_ordinals==list(range(1,len(instance_ordinals)+1))
    assert solve_ordinals==list(range(1,len(solve_ordinals)+1))
    rows=collect(run);assert rows==read(run/'event_records.json');gaps,transitions=paired(rows)
    assert summarize(rows,gaps,transitions)==read(run/'summary.json')
    for name,expected in [('event_metrics.csv',[flat(r) for r in rows]),('paired_delay_gaps.csv',gaps),('attachment_transitions.csv',transitions),('availability.csv',availability(events))]:
        actual=csvread(run/name)
        assert actual==[{k:'' if v is None else str(v) for k,v in row.items()} for row in expected],name
    from plot_handoff_delay_attribution import validate_plots
    validate_plots(run)
    result=dict(valid=True,events=len(events),conditions=len(rows),actual_MPC_solves=sum(r['solve_count'] for r in rows),new_validation_MPC_VLA_calls=0,
                source_preserved=True,metric_recomputed=True,controller_memory_reconstructed=True,plot_numeric_parity=True,wall_s=time.perf_counter()-begin)
    save(run/'validation.json',result);print(json.dumps(result))

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','freeze','execute','analyze','validate']);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();globals()[a.action](a.run.resolve())
if __name__=='__main__':main()
