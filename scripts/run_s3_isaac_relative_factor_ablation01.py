#!/usr/bin/env python3
"""Authenticate/freeze S3, then two planning solves and at most four Isaac rollouts."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json,os,subprocess,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.s3_isaac_ablation01 import ORDER,GRAPHS,AblationGraph,deformation,raw_parity
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.osa03_common_b import activation_at_B
from reconciliation.osa03_relative_ablation import LogicalRelease
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.se2 import relative_pose,local_trajectory_to_world
from reconciliation.robotless_online import stamp
from run_osa03_native_continuation import ask,git,plain
from run_continuous_obstacle_reveal_episode01b import historical_launch,launch_argv,loader_checks,reserve_launch,observe_process,sanitized_environment
from robotless_online_handoffs import Worker
from s3_isaac_source01 import inputs
CONFIG=ROOT/'configs/s3_isaac_relative_factor_ablation_01.yaml'
OUT=ROOT/'results/s3_isaac_relative_factor_ablation_01'
DOC=ROOT/'docs/S3_ISAAC_RELATIVE_FACTOR_ABLATION_01.md'


def namespace(run):
    run=Path(run).resolve();assert run.parent==ROOT/'data/s3_isaac_relative_factor_ablation_01'
    return run


def record_reference(run,name,world,local,labels):
    c=read(run/'common_state.json');installed=local_trajectory_to_world(c['fresh_capture_pose'],local)
    np.testing.assert_allclose(installed,world,rtol=0,atol=1e-12)
    info={}
    for frame,arr in [('world',world),('local',local)]:
        p=run/'references'/f'{name}_{frame}.npy'
        with p.open('xb') as f:np.save(f,arr,allow_pickle=False)
        info.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
    env=geometry(run);safe=reference_safety(world,env);actual=reference_safety(installed,env)
    return dict(**info,labels=labels,safety=safe,installed_roundtrip_safety=actual,
        installed_roundtrip_max_error=float(abs(installed-world).max()),
        valid=bool(safe['clearance_valid'] and actual['clearance_valid']),
        actual_checked_polyline='all supplied rows and edges; includes B when supplied; no implicit connector')


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());d=inputs(cfg)
    assert cfg['solver']==asdict(SolverConfig())
    run.mkdir(parents=True,exist_ok=False)
    for name in ['references','logs','technical_preflight']: (run/name).mkdir()
    save(run/'protocol.json',cfg);save(run/'source_authentication.json',d['auth'])
    for name in ['common_state.json','source_manifest.json','scenario.json','metric_protocol.json','entry.json','recorded_old_to_B.npy']:
        (run/name).write_bytes((d['folder']/name).read_bytes())
    (run/'historical_schedule.json').write_bytes((d['folder']/'schedule.json').read_bytes())
    save(run/'schedule.json',d['schedule']);save(run/'historical_native.json',d['historical'])
    (run/'config_snapshot.yaml').write_bytes(d['config_path'].read_bytes());save(run/'mpc_provenance.json',d['mpc'])
    save(run/'clock_budget.json',dict(maximum_steps=d['common']['B_tick']+2,unchanged_prime_clock_batch_limit=1000))
    for name,arr in [('suffix',d['suffix']),('original',d['original']),('native_installed',d['native_installed'])]:
        with (run/f'{name}.npy').open('xb') as f:np.save(f,arr,allow_pickle=False)
    refs={n:record_reference(run,n,*r) for n,r in d['references'].items()}
    assert all(r['valid'] for r in refs.values()),'historical RAW/B_ENTRY reference unsafe'
    save(run/'prepared_references.json',refs)
    pre=dict(refs);initial_hashes={}
    for n in GRAPHS:
        p=AblationGraph(d['P'],d['common']['B'],d['suffix'],include_relative=cfg['relative_factor'][n]);x=p.initial()
        path=run/f'{n}_initial.npy'
        with path.open('xb') as f:np.save(f,x,allow_pickle=False)
        initial_hashes[n]=sha(path)
        local=relative_pose(d['common']['fresh_capture_pose'],x);local[-1]=d['raw'][-1]
        pre[n+'_INITIAL']=record_reference(run,n+'_INITIAL',x,local,['B',*[f'X_{i}' for i in range(1,len(x)-1)],'F_end'])
    assert len(set(initial_hashes.values()))==1
    save(run/'initialization_hashes.json',initial_hashes);save(run/'preflight_references.json',pre)
    lc=yaml.safe_load((ROOT/'configs/successive_native_source_acquisition_02.yaml').read_text())
    provenance=historical_launch(lc);lc['collector']='scripts/isaac/s3_relative_ablation01.py'
    save(run/'launch_config.json',lc);save(run/'launch_environment.json',dict(argv=launch_argv(run,lc),loader_provenance=provenance))
    assert loader_checks(run,lc)['valid']
    restoration_preflight(run,d)


def restoration_preflight(run,d):
    lc=read(run/'launch_config.json');pre=read(run/'preflight_references.json')
    result=subprocess.run([d['mpc']['python_executable'],str(ROOT/'scripts/s3_isaac_mpc_preflight01.py'),'--run',str(run)],
        env=sanitized_environment(os.environ,lc),capture_output=True,text=True,check=True)
    preflight=json.loads(result.stdout);assert preflight['passed'] and preflight['numerical_MPC_calls']==0
    for row in preflight['rows']:
        actual=np.asarray(row['installed_world']);expected=np.load(pre[row['identity']]['world_path'])
        np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)
        if row['identity']=='RAW':assert actual.tobytes()==d['native_installed'].tobytes()
        if row['identity']=='B_ENTRY':assert actual[2:].tobytes()==d['native_installed'][d['entry']['row_original_identities'][1:]].tobytes()
        assert reference_safety(actual,geometry(run))['clearance_valid'],row['identity']
    save(run/'restoration_preflight.json',preflight)
    OUT.mkdir(parents=True,exist_ok=True)
    for name in ['source_authentication.json','common_state.json','schedule.json','entry.json','initialization_hashes.json']:
        (OUT/name).write_bytes((run/name).read_bytes())
    print(json.dumps(dict(prepared=str(run),entry=d['entry']['correspondence'],MPC_solve_calls=0,graph_solve_calls=0,Isaac_launches=0)))


def freeze(run):
    cfg=read(run/'protocol.json');inputs(cfg);assert cfg==yaml.safe_load(CONFIG.read_text())
    previous=read(ROOT/'results/successive_isaac_four_method_comparison_01/freeze_manifest.json')['manifest']
    files=set(previous['files'])|set(read(Path(read(run/'source_authentication.json')['historical_folder']).parents[1]/'freeze.json')['files'])
    files.update(str(ROOT/p) for p in ['configs/s3_isaac_relative_factor_ablation_01.yaml',
        'src/reconciliation/s3_isaac_ablation01.py','src/reconciliation/successive_handoff_severity01.py',
        'scripts/s3_isaac_source01.py','scripts/s3_isaac_mpc_preflight01.py','scripts/isaac/s3_relative_ablation01.py',
        'scripts/run_s3_isaac_relative_factor_ablation01.py','scripts/validate_s3_isaac_relative_factor_ablation01.py',
        'scripts/report_s3_isaac_relative_factor_ablation01.py','tests/test_s3_isaac_relative_factor_ablation01.py'])
    (run/'protocol_document.md').write_bytes(DOC.read_bytes())
    f=dict(files={p:sha(p) for p in sorted(files)},external_files={p:sha(p) for p in previous['external_files']},
        inputs={str(p):sha(p) for p in sorted(run.rglob('*')) if p.is_file()},budget=cfg['budget'])
    save(run/'freeze.json',f);save(OUT/'freeze_manifest.json',dict(run=str(run),manifest=f,source=read(run/'source_authentication.json'),protocol=cfg))


def verify(run,pushed=False):
    f=read(run/'freeze.json')
    for p,h in {**f['files'],**f['inputs'],**f['external_files']}.items():assert sha(p)==h,p
    inputs(read(run/'protocol.json'))
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in f['files']:assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(OUT/'freeze_manifest.json'))


def graph_solve(run,name):
    assert name in GRAPHS and (run/'scientific_start.json').exists()
    cfg=read(run/'protocol.json');a=read(run/'source_authentication.json');c=read(run/'common_state.json')
    p=AblationGraph(a['P'],c['B'],np.load(run/'suffix.npy'),include_relative=cfg['relative_factor'][name])
    initial=np.load(run/f'{name}_initial.npy');assert initial.tobytes()==p.initial().tobytes()
    env=geometry(run);trace=[];checks=[];folder=run/'planning'/name;folder.mkdir(parents=True,exist_ok=False)
    def feasible(x):
        q=reference_safety(p.unpack(x),env);valid=p.noncollapsed(x) and q['clearance_valid']
        checks.append(dict(valid=bool(valid),safety=q));return valid
    def observe(e):trace.append(dict(**plain(e),factors=p.costs(e['state'])))
    save(folder/'solve_start.json',dict(freeze_sha=read(run/'scientific_start.json')['freeze_sha'],at=stamp(),calls=1,initial_sha256=sha(run/f'{name}_initial.npy')))
    solved=None;error=None;ref=None;start=time.monotonic()
    try:
        solved=solve_least_squares(p.pack(initial),p.residual,SolverConfig(**cfg['solver']),candidate_feasibility_fn=feasible,iteration_callback=observe)
        if solved.converged and p.noncollapsed(solved.optimized):
            w=p.unpack(solved.optimized);local=relative_pose(c['fresh_capture_pose'],w);local[-1]=np.load(read(run/'prepared_references.json')['RAW']['local_path'])[-1]
            ref=record_reference(run,name,w,local,['B',*[f'X_{i}' for i in range(1,len(w)-1)],'F_end'])
    except Exception:error=traceback.format_exc()
    save(folder/'trace.json',trace);save(folder/'feasibility.json',checks)
    report=dict(method=name,solver=None if solved is None else solved.to_dict(),error=error,wall_s=time.monotonic()-start,
        valid=ref is not None and ref['valid'],status='GRAPH_PLANNING_FAILED' if ref is None else 'REFERENCE_SAFE' if ref['valid'] else 'METHOD_REFERENCE_UNSAFE',
        initial_costs=p.costs(p.pack(initial)),final_costs=None if solved is None else p.costs(solved.optimized),
        candidate_world=None if solved is None else p.unpack(solved.optimized).tolist(),
        deformation=None if solved is None else deformation(p,p.unpack(solved.optimized)),reference=ref,optimizer_calls=1)
    save(folder/'result.json',report);return report


def run_method(run,name,isaac):
    out=run/'methods'/name;out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');c=read(run/'common_state.json');schedule=read(run/'schedule.json');ref=read(run/'references.json')[name]
    env=geometry(run)['on'];bt=c['B_tick'];bs=c['B_sim_s'];dt=c['integration_dt_s']
    worker=None;events=[];collected=[];states=[];commands=[];guards=[];requests=[];waits=[];abort=None;error=None
    termination='OBSERVATION_CAP';restored=None;scene_restore=None
    save(out/'execution_start.json',dict(method=name,at=stamp(),freeze_sha=read(run/'scientific_start.json')['freeze_sha']))
    try:
        scene_restore=ask(isaac,'reset_method',method=name);save(out/'scene_restoration.json',scene_restore)
        np.testing.assert_array_equal(scene_restore['pose_world'],c['B']);assert scene_restore['sim_time_s']==bs
        x=np.array(scene_restore['pose_world']);states=[dict(absolute_tick=bt,time_s=0.,sim_time_s=bs,pose_world=x.tolist())]
        mpc=read(run/'mpc_provenance.json')
        worker=Worker([mpc['python_executable'],str(ROOT/'scripts/osa03_common_b_mpc_worker.py'),'--lightnav-checkout',mpc['lightnav_checkout']],out/'worker.log')
        ready=worker.await_type('ready',timeout=30);save(out/'worker_provenance.json',ready)
        assert ready['provenance']['official_settings']==mpc['official_settings']
        assert ready['provenance']['mpc_source_sha256']==mpc['mpc_source_sha256']
        assert ready['provenance']['effective_linear_velocity_limit_m_s']==mpc['effective_linear_velocity_limit_m_s']
        restored=ask(worker,'initialize_common_B',common_path=str(run/'common_state.json'),common_sha256=sha(run/'common_state.json'),reference=ref,identity=name)
        save(out/'restoration.json',restored)
        for k,v in [('held_command',c['u_B_plus']),('previous_control',c['u_mem_B']),('B',c['B'])]:np.testing.assert_array_equal(restored[k],v)
        assert restored['generation']==c['original_generation'] and restored['restored_future_results']==restored['new_solve_calls']==0
        assert restored['reference_version']==c['fresh_version']
        installed=np.asarray(restored['installed_world']);assert reference_safety(installed,geometry(run))['clearance_valid']
        a=activation_at_B(c);logical=LogicalRelease(schedule['pairs'],c['original_generation'])
        ids={r['input_state_id']:r['solve_id'] for r in read(run/'historical_native.json')['events'] if r.get('type')=='solve_result'}
        for tick in range(bt,schedule['stop_tick']):
            t=(tick-bt)*dt;sim=bs+t;event=logical.release(tick)
            if event is not None:
                event['continuation_seen']=dict(tick=tick,time_s=t,sim_time_s=sim)
                accepted=a.accept(event,sim);event['activation_accepted']=accepted;events.append(event);assert accepted
            if tick in logical.by_submit:
                requests.append(dict(tick=tick,pose_world=x.tolist(),sim_time_s=sim,solve_id=ids[tick]))
                wait=logical.collect(worker,tick,ids[tick],x,sim,events,cfg['runtime']['wall_wait_timeout_s'])
                collected.append(deepcopy(logical.pending[1]))
                snap=ask(isaac,'snapshot',method=name,tick=tick)
                assert snap['sim_time_s']==sim;np.testing.assert_array_equal(snap['pose_world'],x)
                wait['Isaac_clock_and_pose_unchanged']=True;waits.append(wait)
            st=dict(state_id=tick,tick=tick,**stamp(sim,t),x=float(x[0]),y=float(x[1]),yaw=float(x[2]))
            proposal,cmd,_=preview_activation(a,st,None,commands[-1] if commands else None)
            guard=guard_check(env,x,cmd,dt);guards.append(plain(guard))
            if not guard['safe']:
                abort=dict(tick=tick,proposed_command=cmd,command_applied=False);termination='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND';break
            if cmd['reason']=='controller_timeout':raise RuntimeError('held-command timeout')
            step=ask(isaac,'step',method=name,tick=tick,pose_world=x.tolist(),command=cmd,guard=plain(guard))
            a=proposal;commands.append(cmd);x=np.array(step['pose_world'])
            assert step['tick']==tick+1 and abs(step['sim_time_s']-(sim+dt))<=1e-10
            states.append(dict(absolute_tick=tick+1,time_s=(tick+1-bt)*dt,sim_time_s=step['sim_time_s'],pose_world=x.tolist(),Isaac=step))
        assert logical.pending is None or abort is not None
    except Exception:error=traceback.format_exc();termination='TECHNICAL_BLOCKED'
    finally:
        if worker is not None:
            worker.shutdown();save(out/'worker_shutdown.json',dict(messages=worker.drain(),returncode=worker.process.returncode))
    r=dict(method=name,phase=deepcopy(c),reference=ref,events=events,states=states,commands=commands,guards=guards,
        submit_requests=requests,collected_results=collected,waits=waits,abort=abort,error=error,termination=termination,
        controller_restoration=restored,scene_restoration=scene_restore)
    save(out/'rollout.json',plain(r));return r


def execute(run):
    verify(run,True);cfg=read(run/'protocol.json')
    save(run/'scientific_start.json',dict(at=stamp(),freeze_sha=git('rev-parse','HEAD'),budget=cfg['budget']))
    planning={n:graph_solve(run,n) for n in GRAPHS}
    refs=read(run/'prepared_references.json')
    refs.update({n:p['reference'] for n,p in planning.items() if p['valid']})
    save(run/'references.json',refs);order=[n for n in ORDER if n in refs];save(run/'execution_order.json',order)
    lc=read(run/'launch_config.json');argv=launch_argv(run,lc);assert argv==read(run/'launch_environment.json')['argv']
    reserve_launch(run,argv);isaac=None;done=[];classification='PENDING_SAVED_ONLY_INTERPRETATION';error=None
    try:
        isaac=Worker(argv,run/'logs/isaac.log')
        ready=isaac.await_type('ready',timeout=cfg['runtime']['startup_timeout_s']);save(run/'isaac_ready.json',ready)
        try:save(run/'isaac_process.json',observe_process(ready['pid']))
        except OSError as exc:save(run/'isaac_process_observer_error.json',dict(error=repr(exc)))
        for name in order:
            r=run_method(run,name,isaac);done.append(name)
            if r['error'] is not None:classification='TECHNICAL_BLOCKED';break
            if name=='RAW':
                gate=raw_parity(r,read(run/'historical_native.json'),read(run/'schedule.json'),cfg['parity'])
                save(run/'RAW_parity.json',gate)
                if not gate['passed']:classification='RAW_ISAAC_PARITY_FAIL';break
    except Exception:error=traceback.format_exc();classification='TECHNICAL_BLOCKED'
    finally:
        if isaac is not None:
            isaac.shutdown();save(run/'isaac_exit.json',dict(returncode=isaac.process.returncode,messages=isaac.drain()))
    save(run/'execution_summary.json',dict(classification=classification,executed_methods=done,error=error,
        graph_solves=2,Isaac_launches=1,LightNav=0,RGB=0,Hermite_execution=0,retries=0))
    verify(run)
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in sorted(run.rglob('*')) if p.is_file()})
    save(OUT/'raw_result_integrity.json',dict(run=str(run),hash_manifest_sha256=sha(run/'result_hashes.json')))
    print(json.dumps(read(run/'execution_summary.json')))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','execute','verify'],required=True)
    a=p.parse_args();globals()[a.mode](namespace(a.run))
