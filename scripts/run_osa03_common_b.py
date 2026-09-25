#!/usr/bin/env python3
"""Freeze four references, then one independent official-MPC rollout per safe method."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.osa03_common_b import ORDER,M3_HASH,authenticated_m3,references,distortion,common_state,activation_at_B,may_execute
from reconciliation.osa03_native import command,evaluate
from reconciliation.se2 import relative_pose,local_trajectory_to_world
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.robotless_online import integrate_unicycle,stamp
from reconciliation.online_pacing import NoCatchupPacer
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.join_source03 import read,save,sha
from run_osa03_native_continuation import source_phase,plain,ask,git
from run_join_online02 import environments
from run_local_se2_reconciliation_formulation01 import independent_clearance
from robotless_online_handoffs import Worker

CONFIG=ROOT/'configs/osa03_common_b_method_comparison_01.yaml'
FILES=[CONFIG,Path(__file__),ROOT/'scripts/osa03_common_b_mpc_worker.py',ROOT/'scripts/validate_osa03_common_b.py',ROOT/'scripts/report_osa03_common_b.py',ROOT/'tests/test_osa03_common_b.py']
FILES += [ROOT/'src/reconciliation'/n for n in ['osa03_common_b.py','osa03_native.py','local_se2_reconciliation.py','robotless_online.py','online_pacing.py','online_mpc_adapter.py','join_online02.py','se2.py','trajectory.py','gp_se2_environment.py','gp_se2_join01.py','handoff_execution_loss.py','join_source02_geometry.py','join_source02_acquisition.py']]
FILES += [ROOT/'scripts'/n for n in ['online_mpc_worker.py','run_osa03_native_continuation.py','run_join_online02.py','run_local_se2_reconciliation_formulation01.py','isaac/robotless_online_handoffs.py']]


def checked(cfg):
    assert cfg['order']==ORDER and cfg['representative']=='REPEAT_00'
    assert cfg['B_tick']==92 and cfg['first_submit_tick']==96 and cfg['integration_steps']==180 and cfg['control_stride']==6
    assert cfg['primary_intervals']==54 and cfg['retries']==0 and cfg['maximum_rollouts']==4
    assert cfg['M3_world_sha256']==M3_HASH and cfg['footprint_radius_m']==.2 and cfg['required_clearance_m']==.05


def geometry(source):
    base,on,cart,envsource=environments(source)
    return dict(base=base,on=on,cart=cart,env_source=envsource)


def prepare(run):
    run.mkdir(parents=True,exist_ok=False);cfg=yaml.safe_load(CONFIG.read_text());checked(cfg)
    source=ROOT/cfg['source_run'];m3run=ROOT/cfg['M3_run'];historical=ROOT/cfg['historical_native_run']
    phase,fresh,raw,provenance=source_phase(source);common=common_state(phase)
    assert common['B_tick']==92 and common['next_submit_after_B']==96
    assert common['integration_dt_s']==float(np.float32(1/60))
    bundle=source/'source_bundle/REPEAT_00'
    assert sha(bundle/'raw/old_lightnav.npy')==cfg['OLD_sha256'] and sha(bundle/'raw/fresh_lightnav.npy')==cfg['FRESH_sha256']
    for p,h in read(m3run/'result_hashes.json').items():assert sha(m3run/p)==h
    assert read(m3run/'validation_final.json')['valid']
    proposed=authenticated_m3(m3run/'derived/optimized_world.npy')
    np.testing.assert_array_equal(np.load(m3run/'derived/original_world.npy'),fresh)
    np.testing.assert_array_equal(read(m3run/'result.json')['A'],common['fresh_capture_pose'])
    np.testing.assert_array_equal(read(m3run/'result.json')['B'],common['B'])
    env=geometry(source);refs=references(common['fresh_capture_pose'],common['B'],fresh,proposed);records={}
    (run/'references').mkdir()
    for name,world in refs.items():
        local=raw.copy() if name=='M0_NATIVE' else relative_pose(common['fresh_capture_pose'],world)
        np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'],local),world,rtol=0,atol=1e-12)
        paths={}
        for frame,arr in [('world',world),('local',local)]:
            p=run/'references'/f'{name}_{frame}.npy'
            with p.open('xb') as f:np.save(f,arr,allow_pickle=False)
            paths.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
        safety=independent_clearance(world,env)
        records[name]=dict(**paths,safety=safety,descriptor=distortion(fresh,world),
                           status='REFERENCE_GEOMETRY_SAFE' if may_execute(safety) else 'REFERENCE_GEOMETRY_UNSAFE')
    assert records['M0_NATIVE']['local_sha256']==cfg['FRESH_sha256']
    assert records['M3_LOCAL_SE2']['world_sha256']==cfg['M3_world_sha256']
    save(run/'references.json',records);save(run/'common_state.json',common);save(run/'protocol.json',cfg)
    metric=yaml.safe_load((ROOT/cfg['metric_protocol']).read_text());save(run/'metric_protocol.json',metric)
    from validate_obstacle_source_acquisition03 import episode
    audit=episode(source,'REPEAT_00');assert audit['qualified'] and audit['original_validation']['valid'];save(run/'source_revalidation.json',audit)
    # Scope-preserving source manifest: R00 and shared geometry only, no other episode data.
    paths=[q for folder in [bundle,source/'episodes/REPEAT_00',m3run,historical,Path(env['env_source']['environment_export'])] for q in folder.rglob('*') if q.is_file()]
    paths += [source/n for n in ['scenario.json','acquisition_scene.json','protocol.json','config_snapshot.yaml','execution_start.json','workers.json','source.json']]
    paths += [Path(read(source/'scenario.json')['source03_input']),Path(read(source/'scenario.json')['triangles_path']),Path(provenance['mpc_source'])]
    paths += [ROOT/'configs/stage0_jackal_controller_validation.yaml',ROOT/'configs/stage0_lightnav_single_chunk.yaml',ROOT/cfg['metric_protocol']]
    save(run/'source_manifest.json',dict(starting_sha=git('rev-parse','HEAD'),source=str(source),M3_run=str(m3run),historical_native=str(historical),
        mpc=provenance,hashes={str(p):sha(p) for p in paths},OLD_sha256=cfg['OLD_sha256'],FRESH_sha256=cfg['FRESH_sha256'],
        M3_world_sha256=M3_HASH,scope='R00 and shared scene only; no other-source evaluation',
        coordinates='world metres yaw CCW; body forward/left; all local representations use original observation A',
        waypoint_intrinsic_timestamps=None))
    print(json.dumps({m:dict(status=r['status'],clearance=r['safety']['minimum_clearance_m']) for m,r in records.items()}))


def preflight(run):
    from validate_osa03_common_b import historical_saved_audit
    save(run/'historical_saved_revalidation.json',historical_saved_audit(run))
    checkout=Path(read(run/'source_manifest.json')['mpc']['lightnav_checkout'])
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    commandline=[str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/osa03_common_b_mpc_worker.py'),'--preflight','--run',str(run),'--checkout',str(checkout)]
    result=subprocess.run(commandline,capture_output=True,text=True,env=env,check=True)
    with (run/'restoration_preflight.stderr').open('x') as f:f.write(result.stderr)
    record=json.loads(result.stdout);assert record['passed'] and record['numerical_MPC_calls']==0
    save(run/'restoration_preflight.json',record);print('historical audit and zero-solve four-reference initialization PASS')


def freeze(run):
    inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in FILES},inputs=inputs,
                               references=read(run/'references.json'),maximum_rollouts=4,retries=0))


def verify(run,pushed=False):
    frozen=read(run/'freeze.json')
    for group in ['files','inputs']:
        for p,h in frozen[group].items():assert sha(p)==h,p
    for p,h in read(run/'source_manifest.json')['hashes'].items():assert sha(p)==h,p
    if pushed:
        remote=git('ls-remote','origin','refs/heads/main').split()[0];assert git('rev-parse','HEAD')==remote
        assert not git('diff','HEAD','--',*[str(p) for p in FILES])


def run_method(run,name):
    out=run/'methods'/name;out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');common=read(run/'common_state.json');ref=read(run/'references.json')[name]
    assert may_execute(ref['safety'])
    manifest=read(run/'source_manifest.json');source=Path(manifest['source']);env=geometry(source)['on']
    fresh=np.load(run/'references/M0_NATIVE_world.npy');phase=deepcopy(common)
    a=activation_at_B(common);x=np.array(common['B']);dt=common['integration_dt_s'];bt=common['B_tick'];bs=common['B_sim_s']
    tick=bt;events=[];states=[dict(absolute_tick=bt,time_s=0.,sim_time_s=bs,pose_world=x.tolist())]
    commands=[];guards=[];pacing=[];requests=[];worker=None;abort=None;error=None;termination='OBSERVATION_CAP';t0=time.monotonic()
    save(out/'execution_start.json',dict(method=name,freeze_sha=git('rev-parse','HEAD'),started=stamp(),common_sha256=sha(run/'common_state.json'),reference=ref))
    try:
        checkout=Path(manifest['mpc']['lightnav_checkout'])
        worker=Worker([str(checkout/'mujoco_demo/.venv/bin/python'),str(ROOT/'scripts/osa03_common_b_mpc_worker.py'),'--lightnav-checkout',str(checkout)],out/'worker.log')
        ready=worker.await_type('ready',timeout=30);save(out/'worker_provenance.json',ready)
        assert ready['provenance']['mpc_source_sha256']==PINNED_MPC_SHA256
        assert ready['provenance']['official_settings']==manifest['mpc']['official_settings']
        restored=ask(worker,'initialize_common_B',common_path=str(run/'common_state.json'),common_sha256=sha(run/'common_state.json'),reference=ref,identity=name)
        for key,target in [('B',common['B']),('held_command',common['u_B_plus']),('previous_control',common['u_mem_B'])]:np.testing.assert_array_equal(restored[key],target)
        assert restored['generation']==common['original_generation'] and restored['next_submit_tick']==96
        assert restored['B_tick']==bt and restored['B_sim_s']==bs and restored['integration_dt_s']==dt
        save(out/'restoration.json',restored)
        pacer=NoCatchupPacer(time.monotonic(),bs);last_result=bs
        while tick<bt+cfg['integration_steps']:
            t=(tick-bt)*dt;sim=bs+t;st=dict(state_id=tick,tick=tick,**stamp(sim,t),x=float(x[0]),y=float(x[1]),yaw=float(x[2]))
            for event in worker.drain():
                event['continuation_seen']=dict(tick=tick,time_s=t,**stamp(sim,t));events.append(event)
                if event.get('type') in ['fatal','error','worker_exit'] or event.get('status') in ['controller_error','error']:raise RuntimeError(event)
                if event.get('type')=='solve_result':
                    accepted=a.accept(event,sim);event['activation_accepted']=accepted
                    if accepted:last_result=sim
            if tick%cfg['control_stride']==0:
                assert tick>=common['next_submit_after_B'] and tick!=bt
                sid=f'{name}_NEW_{len(requests):06d}';requests.append(dict(tick=tick,solve_id=sid,pose_world=x.tolist(),sim_time_s=sim))
                worker.send('submit',solve_id=sid,pose=x.tolist(),input_state_id=tick,input_sim_time_s=sim,input_host_monotonic_s=time.monotonic())
            proposal,c,_=preview_activation(a,st,None,commands[-1] if commands else None)
            if c['reason']=='controller_timeout':
                abort=dict(state=st,proposed_command=c,command_applied=False,kind='hold_timeout');termination='HOLD_TIMEOUT';break
            g=guard_check(env,x,c,dt);guards.append(g)
            if not g['safe']:
                abort=dict(state=st,proposed_command=c,guard=g,command_applied=False);termination='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND';break
            if sim-last_result>cfg['controller_stall_sim_s']:raise RuntimeError('controller result stall')
            a=proposal;c.update(command_id=tick,relative_time_s=t,origin='COMMON_FIRST_FRESH' if tick==bt else 'NEW_COMMON_B_MPC');commands.append(c)
            x=integrate_unicycle(x,command(c),dt);tick+=1
            newsim=bs+(tick-bt)*dt;wait=max(0.,pacer.remaining(newsim,time.monotonic()))
            if wait:time.sleep(wait)
            pacing.append(pacer.observe(newsim,time.monotonic()))
            states.append(dict(absolute_tick=tick,time_s=(tick-bt)*dt,sim_time_s=newsim,pose_world=x.tolist()))
    except Exception:
        import traceback
        error=traceback.format_exc();termination='CONTROLLER_ERROR'
    finally:
        if worker:
            worker.shutdown()
            for event in worker.drain():event['after_termination']=True;events.append(event)
    result=dict(method=name,phase=phase,states=states,commands=commands,events=events,guards=guards,pacing=pacing,submit_requests=requests,
        termination=termination,error=error,safety_abort=abort,wall_s=time.monotonic()-t0,
        new_MPC_submitted=sum(e.get('status')=='submitted' for e in events),new_MPC_solved=sum(e.get('type')=='solve_result' for e in events),
        new_MPC_requests=len(requests),historical_future_results_replayed=0,common_first_command_restored=True,
        new_VLA_calls=0,new_RGB_calls=0,new_optimizer_calls=0,reference_world_sha256=ref['world_sha256'],raw_FRESH_sha256=manifest['FRESH_sha256'])
    save(out/'rollout.json',result)
    metric=None if not commands else plain(evaluate(result,fresh,env,read(source/'scenario.json'),read(run/'metric_protocol.json')))
    save(out/'metrics.json',metric)
    save(out/'hashes.json',{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()})
    print(json.dumps(dict(method=name,termination=termination,steps=len(commands),solves=result['new_MPC_solved'],error=error)),flush=True)


def execute(run):
    verify(run,True);cfg=read(run/'protocol.json');checked(cfg)
    save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),order=ORDER,maximum_rollouts=4,retries=0))
    (run/'methods').mkdir()
    for name,ref in read(run/'references.json').items():
        assert list(read(run/'references.json'))==ORDER
        if not may_execute(ref['safety']):
            out=run/'methods'/name;out.mkdir();save(out/'skipped.json',dict(status='REFERENCE_GEOMETRY_UNSAFE',reference=ref,rollouts=0,metrics=None));continue
        run_method(run,name)
    verify(run);save(run/'completion.json',dict(completed=stamp(),source_preserved=True))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','preflight','freeze','execute'],required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())
