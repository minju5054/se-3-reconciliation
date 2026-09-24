#!/usr/bin/env python3
"""One frozen observation-start counterfactual; B comparator is saved-only."""
import argparse,json,sys,time
from pathlib import Path
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.osa03_trackability import audit_observation,restored_activation,spatial_descriptors,origin_metrics,paired_metrics
from reconciliation.osa03_native import command,evaluate
from reconciliation.robotless_online import integrate_unicycle,stamp
from reconciliation.online_pacing import NoCatchupPacer
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256,selection_audit
from run_osa03_native_continuation import git,plain,source_phase,ask
from robotless_online_handoffs import Worker
from run_join_online02 import environments
from analyze_join_online02 import csvread,jsonlines
from report_obstacle_source_acquisition03 import validate_report
from validate_osa03_native_continuation import validate as validate_baseline,guard_parity
from validate_robotless_online_handoffs import equal_record
CONFIG=ROOT/'configs/osa03_fresh_trackability_control_01.yaml'


def load_source(source):
    _,fresh,raw,provenance=source_phase(source)
    ep=source/'episodes/REPEAT_00';c=read(source/'source_bundle/REPEAT_00/state_and_timing.json')['context']
    meta=read(ep/'metadata.json')
    phase=audit_observation(c,csvread(ep/'execution.csv'),csvread(ep/'commands.csv'),jsonlines(ep/'controller/events.jsonl'),read(ep/'obstacle_reveal.json'),meta['resolved_integration_dt_s'])
    np.testing.assert_allclose(phase['obs_pose'],phase['fresh_capture_pose'],rtol=0,atol=0)
    return phase,fresh,raw,provenance


def prepare(run):
    run.mkdir(parents=True,exist_ok=False);cfg=yaml.safe_load(CONFIG.read_text());source=(ROOT/cfg['source_run']).resolve();base=(ROOT/cfg['comparator_run']).resolve()
    save(run/'source_revalidation.json',validate_report(source));save(run/'B_saved_revalidation.json',validate_baseline(base))
    phase,fresh,raw,prov=load_source(source);_,_,_,envsource=environments(source)
    save(run/'observation_phase.json',phase);save(run/'spatial_descriptors.json',plain(spatial_descriptors(raw,phase['obs_pose'],fresh,cfg['descriptor_min_segment_m'],cfg['reliable_tangent_chord_m'])))
    with (run/'protocol.yaml').open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
    (run/'metric_protocol.yaml').write_bytes((base/'protocol.yaml').read_bytes())
    paths=[q for d in [source,base,Path(envsource['environment_export'])] for q in d.rglob('*') if q.is_file()]
    paths += [Path(read(source/'scenario.json')['source03_input']),Path(prov['mpc_source']),CONFIG]
    paths += [ROOT/'configs/stage0_jackal_controller_validation.yaml',ROOT/'configs/stage0_lightnav_single_chunk.yaml']
    save(run/'source_manifest.json',dict(source=str(source),comparator=str(base),representative='REPEAT_00',starting_sha=git('rev-parse','HEAD'),
        raw_FRESH_sha256=sha(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy'),raw_OLD_sha256=sha(source/'source_bundle/REPEAT_00/raw/old_lightnav.npy'),
        hashes={str(p):sha(p) for p in paths},mpc=prov))
    print(json.dumps(dict(observation_tick=phase['obs_tick'],next_submit=phase['next_legal_submit_tick'],memory=phase['memory_obs'])))


def setup_worker(run,log):
    m=read(run/'source_manifest.json');p=read(run/'observation_phase.json');source=Path(m['source']);checkout=Path(m['mpc']['lightnav_checkout'])
    w=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/osa03_trackability_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/log)
    records=[w.await_type('ready')];assert records[0]['provenance']['mpc_source_sha256']==PINNED_MPC_SHA256
    records.append(ask(w,'reset',episode_id='OSA03_FRESH_TRACKABILITY_CONTROL_01'))
    old=p['old_install']
    records.append(ask(w,'install',episode_id='OSA03_FRESH_TRACKABILITY_CONTROL_01',chunk_id=old['chunk_id'],reference_version=old['reference_version'],
        raw_local_path=str(source/'source_bundle/REPEAT_00/raw/old_lightnav.npy'),capture_pose=old['capture_pose'],raw_sha256=m['raw_OLD_sha256']))
    records.append(ask(w,'restore_observation',phase_path=str(run/'observation_phase.json'),phase_sha256=sha(run/'observation_phase.json')))
    records.append(ask(w,'install',episode_id='OSA03_FRESH_TRACKABILITY_CONTROL_01',chunk_id=p['fresh_chunk_id'],reference_version=p['fresh_version'],
        raw_local_path=str(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy'),capture_pose=p['fresh_capture_pose'],raw_sha256=m['raw_FRESH_sha256']))
    a=ask(w,'state_audit');records.append(a)
    np.testing.assert_array_equal(a['memory'],p['memory_obs']);assert a['pending'] is None
    np.testing.assert_allclose(a['world'],np.load(source/'source_bundle/REPEAT_00/derived/fresh_world.npy'),rtol=0,atol=1e-12)
    assert a['installed']['official_generation']==old['official_generation']+1
    return w,records


def preflight(run):
    # Construction/installation only: there is no submit request or numerical solve.
    w,records=setup_worker(run,'restoration_preflight.log');w.shutdown()
    save(run/'restoration_preflight.json',dict(passed=True,records=records,model_calls=0,MPC_solve_calls=0))


def freeze(run):
    paths=[CONFIG,Path(__file__),ROOT/'scripts/osa03_trackability_mpc_worker.py',ROOT/'scripts/report_osa03_trackability.py',ROOT/'tests/test_osa03_trackability.py']
    paths += [ROOT/'src/reconciliation'/n for n in ['osa03_trackability.py','osa03_native.py','handoff_delay_attribution.py','handoff_execution_loss.py','online_mpc_adapter.py','robotless_online.py','join_online02.py','se2.py','online_pacing.py','gp_se2_environment.py','gp_se2_join01.py','join_source02_geometry.py','join_source02_acquisition.py']]
    paths += [ROOT/'scripts'/n for n in ['online_mpc_worker.py','isaac/robotless_online_handoffs.py','run_osa03_native_continuation.py','validate_osa03_native_continuation.py','run_join_online02.py']]
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in paths},inputs={str(run/n):sha(run/n) for n in ['protocol.yaml','metric_protocol.yaml','source_manifest.json','observation_phase.json','spatial_descriptors.json','restoration_preflight.json','B_saved_revalidation.json','source_revalidation.json']}))


def verify(run,pushed=False):
    for group in ['files','inputs']:
        for p,h in read(run/'freeze.json')[group].items():assert sha(p)==h,p
    for p,h in read(run/'source_manifest.json')['hashes'].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('rev-parse','origin/main')
        assert not git('diff','HEAD','--',*read(run/'freeze.json')['files'])


def measure(run,r,fresh,env):
    cfg=yaml.safe_load((run/'metric_protocol.yaml').read_text());m=read(run/'source_manifest.json');scene=read(Path(m['source'])/'scenario.json')
    obs=plain(origin_metrics(r,fresh,env,scene,cfg));base=read(Path(m['comparator'])/'metrics.json')
    paired=paired_metrics(obs,base)
    common=None
    if obs['execution'] is not None:
        n=len(obs['execution']['trace']['times_s'])-1;br=read(Path(m['comparator'])/'rollout.json')
        br['states']=br['states'][:n+1];br['commands']=br['commands'][:n]
        bm=plain(evaluate(br,fresh,env,scene,cfg));common=dict(integration_intervals=n,B_metrics=bm,paired=paired_metrics(obs,bm))
    return dict(origins=obs,paired=paired,common_execution_exposure=common)


def execute(run):
    verify(run,True);save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),maximum_rollouts=1))
    cfg=yaml.safe_load((run/'protocol.yaml').read_text());manifest=read(run/'source_manifest.json');source=Path(manifest['source'])
    p,fresh,raw,prov=load_source(source);equal_record(p,read(run/'observation_phase.json'));_,env,_,_=environments(source)
    dt=p['integration_dt_s'];start=p['obs_tick'];sim0=p['obs_sim_s'];x=np.asarray(p['obs_pose']);a=restored_activation(p)
    states=[dict(absolute_tick=start,time_s=0.,sim_time_s=sim0,pose_world=x.tolist())];commands=[];events=[];guards=[];pacing=[]
    worker=None;abort=None;error=None;termination='OBSERVATION_CAP';requests=0;started=time.monotonic();tick=start
    try:
        worker,restoration=setup_worker(run,'mpc_worker.log');save(run/'restoration.json',restoration)
        pacer=NoCatchupPacer(time.monotonic(),sim0);last_result=float(p['incoming_application']['sim_time_s'])
        while tick<start+cfg['cap_integration_steps']:
            sim=sim0+(tick-start)*dt;t=(tick-start)*dt
            st=dict(state_id=tick,tick=tick,**stamp(sim,t),x=float(x[0]),y=float(x[1]),yaw=float(x[2]))
            for e in worker.drain():
                e['continuation_seen']=dict(tick=tick,time_s=t,**stamp(sim,t));events.append(e)
                if e.get('type') in ('fatal','error','worker_exit') or e.get('status') in ('controller_error','error'):raise RuntimeError(e)
                if e.get('type')=='solve_result':
                    assert a.accept(e,sim),e
                    last_result=sim
            if tick%cfg['control_stride']==0:
                assert tick>=p['next_legal_submit_tick']
                worker.send('submit',solve_id=f'OSA03_OBS_solve_{requests:06d}',pose=x.tolist(),input_state_id=tick,input_sim_time_s=sim,input_host_monotonic_s=time.monotonic());requests+=1
            proposal,c,_=preview_activation(a,st,None,commands[-1] if commands else p['incoming_application'])
            c.update(command_id=tick,relative_time_s=t,origin='OBSERVATION_START_NATIVE')
            g=guard_check(env,x,c,dt);guards.append(g)
            if not g['safe']:
                abort=dict(state=st,proposed_command=c,guard=g,command_applied=False);termination='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND';break
            if sim-last_result>cfg['controller_stall_sim_s']:raise RuntimeError('controller result stall')
            a=proposal;commands.append(c);x=integrate_unicycle(x,command(c),dt);tick+=1
            newsim=sim0+(tick-start)*dt;wait=max(0.,pacer.remaining(newsim,time.monotonic()))
            if wait:time.sleep(wait)
            pacing.append(pacer.observe(newsim,time.monotonic()))
            states.append(dict(absolute_tick=tick,time_s=(tick-start)*dt,sim_time_s=newsim,pose_world=x.tolist()))
    except Exception:
        import traceback
        error=traceback.format_exc();termination='CONTROLLER_ERROR'
    finally:
        if worker:
            worker.shutdown()
            for e in worker.drain():e['after_termination']=True;events.append(e)
    r=dict(kind='OFFLINE observation-start same-FRESH Native counterfactual; B comparator saved-only',phase=p,states=states,commands=commands,events=events,guards=guards,pacing=pacing,
        termination=termination,error=error,safety_abort=abort,raw_FRESH_sha256=manifest['raw_FRESH_sha256'],fresh_world=fresh.tolist(),wall_s=time.monotonic()-started,
        new_MPC_requests=requests,new_MPC_busy=sum(e.get('status')=='busy' for e in events),new_MPC_submitted=sum(e.get('status')=='submitted' for e in events),
        new_MPC_solved=sum(e.get('type')=='solve_result' for e in events),new_VLA_calls=0,new_optimizer_calls=0,new_RGB_calls=0,new_B_start_solves=0)
    save(run/'rollout.json',r);save(run/'metrics.json',measure(run,r,fresh,env));verify(run)
    print(json.dumps(dict(termination=termination,duration_s=states[-1]['time_s'],solves=r['new_MPC_solved'],error=error)))


def validate(run):
    verify(run);m=read(run/'source_manifest.json');source=Path(m['source']);p,fresh,raw,prov=load_source(source)
    equal_record(p,read(run/'observation_phase.json'));assert validate_baseline(Path(m['comparator']))['valid']
    _,env,_,_=environments(source);r=read(run/'rollout.json');assert r['phase']==p and r['raw_FRESH_sha256']==m['raw_FRESH_sha256']
    dt=p['integration_dt_s'];assert len(r['states'])==len(r['commands'])+1;assert len(r['commands'])<=180
    np.testing.assert_array_equal(r['states'][0]['pose_world'],p['obs_pose'])
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==p['obs_tick']+i and s['time_s']==i*dt and s['sim_time_s']==p['obs_sim_s']+i*dt
        if i:np.testing.assert_array_equal(integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),dt),s['pose_world'])
    a=restored_activation(p);previous=p['memory_obs'];solved={};submitted=[]
    for e in r['events']:
        if e.get('status')=='submitted':
            np.testing.assert_array_equal(e['previous_command'],previous);submitted.append(e)
        if e.get('type')=='solve_result':
            solved[e['solve_id']]=e
            np.testing.assert_array_equal(e['previous_command'],previous)
            if e['status']=='command':previous=e['command']
    for e in submitted:
        tick=e['input_state_id'];assert tick%6==0 and tick>=p['next_legal_submit_tick']
        np.testing.assert_array_equal(e['input_pose'],r['states'][tick-p['obs_tick']]['pose_world'])
    if submitted:assert submitted[0]['input_state_id']==p['next_legal_submit_tick']
    for e in solved.values():
        if e['status']=='command':
            assert selection_audit(fresh,e['input_pose'],e['selection']['reference_world'],horizon=prov['official_settings']['HORIZON'],weights=prov['official_settings']['Q_WEIGHTS'])==e['selection']
    for i,c in enumerate(r['commands']):
        tick=p['obs_tick']+i;pose=r['states'][i]['pose_world']
        for e in solved.values():
            if e.get('continuation_seen',{}).get('tick')==tick:assert a.accept(e,c['sim_time_s'])
        st=dict(c,state_id=tick,tick=tick,x=pose[0],y=pose[1],yaw=pose[2])
        a,expected,_=preview_activation(a,st,None,None)
        for k in expected:assert expected[k]==c[k],(tick,k)
        g=guard_check(env,pose,c,dt);assert g['safe'];guard_parity(g,r['guards'][i])
        if c['reason']=='new_solve':assert c['application_tick']==solved[c['solve_id']]['continuation_seen']['tick']
    if r['safety_abort']:
        q=r['safety_abort'];assert not q['command_applied'];assert not guard_check(env,q['guard']['start_pose'],q['proposed_command'],dt)['safe']
    equal_record(measure(run,r,fresh,env),read(run/'metrics.json'))
    assert r['new_MPC_submitted']==len(submitted) and r['new_MPC_solved']==len(solved)
    assert r['new_VLA_calls']==r['new_optimizer_calls']==r['new_RGB_calls']==r['new_B_start_solves']==0
    restore=read(run/'restoration.json');np.testing.assert_array_equal(restore[-1]['memory'],p['memory_obs']);np.testing.assert_allclose(restore[-1]['world'],fresh,rtol=0,atol=1e-12)
    return dict(valid=True,source_and_B_unchanged=True,observation_phase=True,controller_memory_chain=True,absolute_grid=True,integration=True,safety=True,native_selection=True,
                two_origins=True,signed_metrics=True,saved_only=True,validation_model_MPC_optimizer_calls=0)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--mode',choices=['prepare','preflight','freeze','execute','validate'],required=True);args=ap.parse_args();run=args.run.resolve()
    if args.mode=='validate':v=validate(run);save(run/'validation.json',v);print(json.dumps(v))
    else:globals()[args.mode](run)
