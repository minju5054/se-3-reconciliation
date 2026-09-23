#!/usr/bin/env python3
"""Prepare, freeze, execute ONCE, and saved-validate OSA03 Native continuation."""
import argparse,json,subprocess,sys,time
from pathlib import Path
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.osa03_native import phase_audit,replay_prefix,pose,command,evaluate
from reconciliation.robotless_online import integrate_unicycle,stamp
from reconciliation.online_pacing import NoCatchupPacer
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256,selection_audit
from robotless_online_handoffs import Worker
from run_join_online02 import environments
from report_obstacle_source_acquisition03 import validate_report
from analyze_join_online02 import csvread,jsonlines
from validate_robotless_online_handoffs import equal_record

CONFIG=ROOT/'configs/osa03_native_continuation_01.yaml'

def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()

def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [plain(v) for v in x]
    return x


def source_phase(source):
    seal=source/'source_bundle';manifest=read(seal/'manifest.json')
    assert manifest['representative']=='REPEAT_00' and manifest['classification']=='QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE'
    bundle=seal/'REPEAT_00'
    for name,h in read(bundle/'hashes.json').items():assert sha(bundle/name)==h,name
    ep=source/'episodes/REPEAT_00';state=read(bundle/'state_and_timing.json')
    c=state['context'];fresh=np.load(bundle/'derived/fresh_world.npy');raw=np.load(bundle/'raw/fresh_lightnav.npy')
    assert sha(bundle/'raw/fresh_lightnav.npy')==c['fresh_raw_local_ref']['sha256']
    for kind in ['fresh','old']:
        assert (bundle/f'raw/{kind}_lightnav.npy').read_bytes()==(ep/c[f'{kind}_raw_local_ref']['path']).read_bytes()
    meta=read(ep/'metadata.json');settings=meta['mpc_worker']['provenance']['official_settings']
    assert meta['mpc_worker']['provenance']['mpc_source_sha256']==PINNED_MPC_SHA256
    phase=phase_audit(c,csvread(ep/'execution.csv'),csvread(ep/'commands.csv'),jsonlines(ep/'controller/events.jsonl'),state['B_state'],fresh,raw,settings,meta['resolved_integration_dt_s'])
    return phase,fresh,raw,meta['mpc_worker']['provenance']


def prepare(run):
    run.mkdir(parents=True,exist_ok=False);cfg=yaml.safe_load(CONFIG.read_text());source=(ROOT/cfg['source_run']).resolve()
    save(run/'source_revalidation.json',validate_report(source))
    phase,fresh,raw,provenance=source_phase(source)
    _,env,cart,envsource=environments(source)
    p=replay_prefix(phase,env);p.pop('activation');save(run/'stage0_prefix_reconstruction.json',plain(p))
    save(run/'controller_phase.json',phase)
    with (run/'protocol.yaml').open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
    # Authenticate entire authoritative run and external geometry; no copies of RGB.
    paths=[q for q in source.rglob('*') if q.is_file()]
    paths += [Path(read(source/'scenario.json')['source03_input']),Path(provenance['mpc_source']),CONFIG]
    paths += [q for q in Path(envsource['environment_export']).rglob('*') if q.is_file()]
    paths += [ROOT/'configs/stage0_jackal_controller_validation.yaml',ROOT/'configs/stage0_lightnav_single_chunk.yaml']
    save(run/'source_manifest.json',dict(source=str(source),representative='REPEAT_00',starting_sha=git('rev-parse','HEAD'),
        raw_FRESH_sha256=sha(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy'),raw_OLD_sha256=sha(source/'source_bundle/REPEAT_00/raw/old_lightnav.npy'),
        hashes={str(p):sha(p) for p in paths},mpc=provenance))
    print(json.dumps(dict(phase_resolved=True,prefix=p['passed'],B_tick=phase['B_tick'],prefix_steps=phase['prefix_steps'],first_new_submit_tick=phase['first_new_submit_tick'])))


def freeze(run):
    paths=[CONFIG,Path(__file__),ROOT/'src/reconciliation/osa03_native.py',ROOT/'scripts/osa03_native_mpc_worker.py',ROOT/'tests/test_osa03_native.py',ROOT/'scripts/report_osa03_native_continuation.py']
    paths += [ROOT/'src/reconciliation'/n for n in ['robotless_online.py','online_pacing.py','online_mpc_adapter.py','se2.py','gp_se2_environment.py','gp_se2_join01.py','handoff_execution_loss.py','join_online02.py','join_source02_acquisition.py','join_source02_geometry.py']]
    paths += [ROOT/'scripts/online_mpc_worker.py',ROOT/'scripts/isaac/robotless_online_handoffs.py',ROOT/'scripts/run_join_online02.py']
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in paths},inputs={str(run/n):sha(run/n) for n in ['source_manifest.json','controller_phase.json','protocol.yaml','stage0_prefix_reconstruction.json','restoration_preflight.json']},maximum_rollouts=1))


def verify(run,pushed=False):
    for k in ['files','inputs']:
        for p,h in read(run/'freeze.json')[k].items():assert sha(p)==h,p
    for p,h in read(run/'source_manifest.json')['hashes'].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('rev-parse','origin/main')
        assert not git('diff','HEAD','--',*[str(p) for p in read(run/'freeze.json')['files']])


def ask(worker,op,**kwargs):
    ident=worker.send(op,**kwargs)
    reply=worker.await_type('reply',timeout=30)
    assert reply['request_id']==ident and reply.get('status') not in ['error','controller_error'],reply
    return reply


def execute(run):
    verify(run,True);save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),maximum_rollouts=1))
    cfg=yaml.safe_load((run/'protocol.yaml').read_text());manifest=read(run/'source_manifest.json');source=Path(manifest['source'])
    phase,fresh,raw,provenance=source_phase(source);equal_record(phase,read(run/'controller_phase.json'))
    _,env,_,_=environments(source);dt=phase['integration_dt_s'];Btick=phase['B_tick'];Bsim=phase['B_sim_s']
    replay=replay_prefix(phase,env);a=replay.pop('activation');save(run/'prefix_parity.json',plain(replay))
    if not replay['passed']:save(run/'result.json',dict(classification='NATIVE_PREFIX_REPLAY_INVALID'));return
    worker=None;started=time.monotonic();events=[];guards=replay['guards'];commands=replay['commands'];states=[];pacing=[];restores=[];abort=None
    for i,x in enumerate(replay['states']):states.append(dict(absolute_tick=Btick+i,time_s=i*dt,sim_time_s=Bsim+i*dt,pose_world=x.tolist(),kind='HISTORICAL_PREFIX_RECONSTRUCTED'))
    x=np.array(states[-1]['pose_world']);tick=phase['prefix_end_tick'];termination='OBSERVATION_CAP';error=None;nextid=7
    try:
        checkout=Path(provenance['lightnav_checkout'])
        worker=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/osa03_native_mpc_worker.py'),'--lightnav-checkout',str(checkout)],run/'mpc_worker.log')
        ready=worker.await_type('ready');save(run/'worker_provenance.json',ready)
        assert ready['provenance']['mpc_source_sha256']==PINNED_MPC_SHA256
        ask(worker,'reset',episode_id='OSA03_NATIVE_CONTINUATION_01')
        ask(worker,'install',episode_id='OSA03_NATIVE_CONTINUATION_01',chunk_id=phase['fresh_chunk_id'],reference_version=phase['fresh_version'],raw_local_path=str(source/'source_bundle/REPEAT_00/raw/fresh_lightnav.npy'),capture_pose=phase['fresh_capture_pose'],raw_sha256=manifest['raw_FRESH_sha256'])
        # Restore both accepted results in their saved order before live time starts.
        for i in range(len(phase['applications'])):
            restored=ask(worker,'restore_saved',phase_path=str(run/'controller_phase.json'),phase_sha256=sha(run/'controller_phase.json'),historical_index=i)
            np.testing.assert_allclose(restored['installed_world'],fresh,rtol=0,atol=cfg['selection_atol']);restores.append(restored)
        save(run/'restoration.json',restores)
        pacer=NoCatchupPacer(time.monotonic(),Bsim+(tick-Btick)*dt)
        last_result=Bsim+(phase['applications'][-1]['application_tick']-Btick)*dt
        while tick<Btick+cfg['integration_steps']:
            sim=Bsim+(tick-Btick)*dt;t=(tick-Btick)*dt
            st=dict(state_id=tick,tick=tick,**stamp(sim,t),x=float(x[0]),y=float(x[1]),yaw=float(x[2]))
            for event in worker.drain():
                event['continuation_seen']=dict(tick=tick,time_s=t,**stamp(sim,t));events.append(event)
                if event.get('type') in ('fatal','error','worker_exit') or event.get('status') in ('controller_error','error'):
                    raise RuntimeError(event)
                if event.get('type')=='solve_result':
                    assert a.accept(event,sim),event
                    last_result=sim
            if tick%cfg['control_stride']==0:
                assert tick>=phase['first_new_submit_tick'] and tick!=Btick
                worker.send('submit',solve_id=f'OSA03_NATIVE_solve_{nextid:06d}',pose=x.tolist(),input_state_id=tick,input_sim_time_s=sim,input_host_monotonic_s=time.monotonic());nextid+=1
            proposal,c,_=preview_activation(a,st,None,commands[-1]);g=guard_check(env,x,c,dt);guards.append(g)
            if not g['safe']:
                abort=dict(state=st,proposed_command=c,guard=g,command_applied=False);termination='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND';break
            if sim-last_result>cfg['controller_stall_sim_s']:raise RuntimeError('controller result stall')
            a=proposal;c.update(command_id=tick,relative_time_s=t,origin='NEW_NATIVE_CONTINUATION');commands.append(c)
            x=integrate_unicycle(x,command(c),dt);tick+=1
            newsim=Bsim+(tick-Btick)*dt;wait=max(0.,pacer.remaining(newsim,time.monotonic()))
            if wait:time.sleep(wait)
            timing=pacer.observe(newsim,time.monotonic());pacing.append(timing)
            states.append(dict(absolute_tick=tick,time_s=(tick-Btick)*dt,sim_time_s=newsim,pose_world=x.tolist(),kind='NEW_NATIVE_CONTINUATION'))
    except Exception as exc:
        import traceback
        error=traceback.format_exc();termination='CONTROLLER_ERROR'
    finally:
        if worker:
            worker.shutdown()
            for event in worker.drain():event['after_termination']=True;events.append(event)
    result=dict(kind='OFFLINE NATIVE-ONLY PHASE-PRESERVING CONTINUATION; saved prefix then live asynchronous MPC',phase=phase,
        states=states,commands=commands,events=events,guards=guards,pacing=pacing,termination=termination,error=error,safety_abort=abort,
        raw_FRESH_sha256=manifest['raw_FRESH_sha256'],fresh_world=fresh.tolist(),prefix_parity=True,wall_s=time.monotonic()-started,
        new_MPC_requests=nextid-7,new_MPC_busy=sum(e.get('status')=='busy' for e in events),
        new_MPC_submitted=sum(e.get('status')=='submitted' for e in events),new_MPC_solved=sum(e.get('type')=='solve_result' for e in events),
        historical_results_replayed=len(phase['applications']),new_VLA_calls=0,new_optimizer_calls=0)
    save(run/'rollout.json',result)
    measured=evaluate(result,fresh,env,read(source/'scenario.json'),cfg);save(run/'metrics.json',plain(measured));verify(run)
    print(json.dumps(dict(classification=measured['classification'],duration_s=states[-1]['time_s'],new_MPC_solved=result['new_MPC_solved'],error=error)))


def validate(run):
    verify(run);m=read(run/'source_manifest.json');source=Path(m['source']);phase,fresh,raw,provenance=source_phase(source)
    equal_record(phase,read(run/'controller_phase.json'));_,env,_,_=environments(source)
    r=read(run/'rollout.json');cfg=yaml.safe_load((run/'protocol.yaml').read_text())
    assert r['phase']==phase and r['raw_FRESH_sha256']==m['raw_FRESH_sha256']
    replay=replay_prefix(phase,env);activation=replay.pop('activation');equal_record(plain(replay),read(run/'prefix_parity.json'))
    for i,s in enumerate(r['states']):
        assert s['absolute_tick']==phase['B_tick']+i and s['time_s']==i*phase['integration_dt_s']
        if i:
            pred=integrate_unicycle(r['states'][i-1]['pose_world'],command(r['commands'][i-1]),phase['integration_dt_s'])
            np.testing.assert_array_equal(pred,s['pose_world'])
    np.testing.assert_array_equal([s['pose_world'] for s in r['states'][:phase['prefix_steps']+1]],replay['states'])
    assert len(r['commands'])==len(r['states'])-1
    for i,c in enumerate(r['commands']):
        g=guard_check(env,r['states'][int(c['application_tick'])-phase['B_tick']]['pose_world'],c,phase['integration_dt_s']);assert g['safe'];equal_record(g,r['guards'][i])
    previous=phase['applications'][-1]['result']['command']
    for e in r['events']:
        if e.get('status')=='submitted':np.testing.assert_array_equal(e['previous_command'],previous)
        if e.get('type')=='solve_result' and e.get('status')=='command':
            np.testing.assert_array_equal(e['previous_command'],previous);previous=e['command']
    seen={e['solve_id']:e for e in r['events'] if e.get('type')=='solve_result'}
    for e in seen.values():
        assert e['input_state_id']>=phase['first_new_submit_tick'] and e['input_state_id']%6==0
        if e.get('status')=='command':
            np.testing.assert_array_equal(e['input_pose'],r['states'][e['input_state_id']-phase['B_tick']]['pose_world'])
            assert selection_audit(fresh,e['input_pose'],e['selection']['reference_world'],horizon=provenance['official_settings']['HORIZON'],weights=provenance['official_settings']['Q_WEIGHTS'])==e['selection']
    for c in r['commands'][phase['prefix_steps']:]:
        tick=c['application_tick']
        for e in r['events']:
            if e.get('type')=='solve_result' and e.get('continuation_seen',{}).get('tick')==tick:
                assert activation.accept(e,c['sim_time_s'])
        pp=r['states'][tick-phase['B_tick']]['pose_world']
        st=dict(c,state_id=tick,tick=tick,x=pp[0],y=pp[1],yaw=pp[2])
        activation,expected,_=preview_activation(activation,st,None,None)
        for key in expected:assert expected[key]==c[key],(tick,key)
        if c['reason']=='new_solve':
            e=seen[c['solve_id']];np.testing.assert_array_equal(command(c),e['command']);assert c['application_tick']==e['continuation_seen']['tick']
    if r['safety_abort']:
        a=r['safety_abort'];assert not a['command_applied'];assert not guard_check(env,a['guard']['start_pose'],a['proposed_command'],phase['integration_dt_s'])['safe']
    measured=plain(evaluate(r,fresh,env,read(source/'scenario.json'),cfg));equal_record(measured,read(run/'metrics.json'))
    assert r['new_MPC_submitted']==sum(e.get('status')=='submitted' for e in r['events'])
    assert r['new_MPC_solved']==len(seen) and r['new_VLA_calls']==0 and r['new_optimizer_calls']==0
    return dict(valid=True,source_hashes_preserved=True,prefix_parity=True,command_integration=True,native_selection=True,metric_recomputation=True,classification=measured['classification'],new_validation_model_MPC_optimizer_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','execute','validate'],required=True);a=p.parse_args();run=a.run.resolve()
    if a.mode=='validate':
        v=validate(run);save(run/'validation.json',v);print(json.dumps(v))
    else:globals()[a.mode](run)
