#!/usr/bin/env python3
"""One E_R ablation with authenticated logical release; scientific run requires push."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from copy import deepcopy
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from reconciliation.osa03_relative_ablation import ORDER,LABEL,LogicalRelease,load_schedule,planning
from reconciliation.osa03_common_b import common_state,activation_at_B,may_execute,authenticated_m3,references
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares,OptimizationError
from reconciliation.osa03_native import command,evaluate
from reconciliation.se2 import relative_pose,local_trajectory_to_world
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.robotless_online import integrate_unicycle,stamp
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.join_source03 import read,save,sha
from run_osa03_native_continuation import source_phase,plain,ask,git
from run_osa03_common_b import geometry,FILES as COMMON_FILES
from run_local_se2_reconciliation_formulation01 import independent_clearance
from robotless_online_handoffs import Worker
CONFIG=ROOT/'configs/osa03_relative_factor_ablation_01.yaml'
RESULTS=ROOT/'results/osa03_relative_factor_ablation_01'
FILES=list(dict.fromkeys(COMMON_FILES+[CONFIG,Path(__file__),ROOT/'src/reconciliation/osa03_relative_ablation.py',
    ROOT/'src/reconciliation/graph_optimizer.py',ROOT/'scripts/validate_osa03_relative_factor_ablation01.py',
    ROOT/'scripts/report_osa03_relative_factor_ablation01.py',ROOT/'tests/test_osa03_relative_factor_ablation01.py',
    ROOT/'tests/fixtures/relative_ablation_full_golden.json']))


def authenticate(cfg):
    old=ROOT/cfg['common_B_run']
    assert sha(old/'result_hashes.json')==cfg['common_B_result_hashes_sha256']
    for p,h in read(old/'result_hashes.json').items():assert sha(old/p)==h,p
    manifest=read(old/'source_manifest.json')
    for p,h in manifest['hashes'].items():assert sha(p)==h,p
    assert read(old/'validation.json')['valid']
    schedule=load_schedule(old/'methods/M0_NATIVE/rollout.json',cfg['M0_rollout_sha256'],
        ROOT/cfg['tracked_common_result'],cfg['tracked_common_result_sha256'])
    assert cfg['order']==ORDER and cfg['integration_steps']==schedule['integration_steps']==180
    assert cfg['primary_intervals']==54 and cfg['solver']==asdict(SolverConfig())
    assert cfg['relative_factor']==dict(FULL_LOCAL_SE2=True,NO_RELATIVE=False)
    assert cfg['normalization']==dict(translation_m=.1,yaw_degrees=10.)
    assert cfg['footprint_radius_m']==.2 and cfg['required_edge_clearance_m']==.05
    assert cfg['scientific_optimizer_calls']==1 and cfg['maximum_rollouts']==4 and cfg['retries']==0
    return old,manifest,schedule


def reference_record(run,name,world,raw,common,fresh,env):
    local=raw.copy() if name=='M0_NATIVE' else relative_pose(common['fresh_capture_pose'],world)
    np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'],local),world,rtol=0,atol=1e-12)
    record={}
    for frame,arr in [('world',world),('local',local)]:
        path=run/'references'/f'{name}_{frame}.npy'
        with path.open('xb') as stream:np.save(stream,arr,allow_pickle=False)
        record.update({frame+'_path':str(path),frame+'_sha256':sha(path)})
    safety=independent_clearance(world,env)
    record.update(safety=safety,descriptor=plain(planning(fresh,world,common['B'])),
                  status='REFERENCE_GEOMETRY_SAFE' if may_execute(safety) else 'REFERENCE_GEOMETRY_UNSAFE')
    return record


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());old,manifest,schedule=authenticate(cfg)
    phase,fresh,raw,provenance=source_phase(Path(manifest['source']));common=common_state(phase)
    assert common==read(old/'common_state.json') and schedule['integration_dt_s']==common['integration_dt_s']
    full=authenticated_m3(Path(manifest['M3_run'])/'derived/optimized_world.npy')
    frozen=references(common['fresh_capture_pose'],common['B'],fresh,full)
    run.mkdir(parents=True,exist_ok=False);(run/'references').mkdir()
    save(run/'protocol.json',cfg);save(run/'schedule.json',schedule);save(run/'common_state.json',common)
    save(run/'source_manifest.json',manifest);save(run/'metric_protocol.json',read(old/'metric_protocol.json'))
    env=geometry(Path(manifest['source']));records={}
    for name,previous in zip(ORDER[:3],['M0_NATIVE','M1_SE2_TAPER','M3_LOCAL_SE2']):
        records[name]=reference_record(run,name,frozen[previous],raw,common,fresh,env)
        assert records[name]['world_sha256']==read(old/'references.json')[previous]['world_sha256']
    save(run/'prepared_references.json',records)
    from validate_obstacle_source_acquisition03 import episode
    audit=episode(Path(manifest['source']),'REPEAT_00');assert audit['qualified'] and audit['original_validation']['valid']
    save(run/'source_revalidation.json',audit)
    print(json.dumps(dict(prepared=str(run),schedule=schedule['primary'],planning_calls=0,MPC_calls=0)))


def preflight(run):
    checkout=Path(read(run/'source_manifest.json')['mpc']['lightnav_checkout'])
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    code="import sys,json;from pathlib import Path;sys.path[:0]=[sys.argv[1],sys.argv[2]];from osa03_common_b_mpc_worker import preflight;from reconciliation.join_source03 import read;r=Path(sys.argv[3]);print(json.dumps(preflight(sys.argv[4],read(r/'common_state.json'),read(r/'prepared_references.json'))))"
    result=subprocess.run([str(checkout/'mujoco_demo/.venv/bin/python'),'-c',code,str(ROOT/'scripts'),str(ROOT/'src'),str(run),str(checkout)],capture_output=True,text=True,env=env,check=True)
    record=json.loads(result.stdout);assert record['passed'] and record['numerical_MPC_calls']==0
    save(run/'restoration_preflight.json',record)
    print('Official restoration/installation preflight PASS; zero numerical solves')


def freeze(run):
    protocol=ROOT/'docs/OSA03_RELATIVE_FACTOR_ABLATION_01.md'
    with (run/'protocol_document.md').open('xb') as stream:stream.write(protocol.read_bytes())
    files={str(p):sha(p) for p in FILES}
    inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}
    save(run/'freeze.json',dict(files=files,inputs=inputs,maximum_rollouts=4,optimizer_calls=1,retries=0))
    RESULTS.mkdir(parents=True,exist_ok=True)
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),schedule=read(run/'schedule.json'),
        common_state=read(run/'common_state.json'),source_manifest_sha256=sha(run/'source_manifest.json')))


def verify(run,pushed=False):
    frozen=read(run/'freeze.json')
    for group in ['files','inputs']:
        for p,h in frozen[group].items():assert sha(p)==h,p
    authenticate(read(run/'protocol.json'))
    if pushed:
        head=git('rev-parse','HEAD');assert head==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in frozen['files']:
            relative=str(Path(p).relative_to(ROOT))
            assert git('rev-parse',f'HEAD:{relative}')==git('hash-object',p),p
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))
        doc='docs/OSA03_RELATIVE_FACTOR_ABLATION_01.md'
        assert git('rev-parse',f'HEAD:{doc}')==git('hash-object',str(run/'protocol_document.md'))


def solve_no_relative(run):
    c=read(run/'common_state.json');fresh=np.load(run/'references/M0_NATIVE_world.npy')
    p=LocalSE2Problem(c['fresh_capture_pose'],c['B'],fresh);env=geometry(Path(read(run/'source_manifest.json')['source']))
    trace=[];checks=[]
    def feasible(x):
        check=env['on'].check_polyline(x);checks.append(dict(state=x.copy(),check=check));return check['clearance_valid']
    def observe(event):trace.append({**event,'factor_costs':p.costs(event['state'],include_relative=False)})
    save(run/'optimization_start.json',dict(freeze_sha=git('rev-parse','HEAD'),started=stamp(),calls=1))
    start=time.monotonic();error=None
    try:
        solved=solve_least_squares(p.fresh,lambda x:p.residual_vector(x,include_relative=False),
            SolverConfig(**read(run/'protocol.json')['solver']),candidate_feasibility_fn=feasible,iteration_callback=observe)
        x=solved.optimized;status=solved.to_dict()
    except OptimizationError as exc:
        error=str(exc);x=None;status=dict(termination_reason='OptimizationError',error=error)
    save(run/'solver_trace.json',plain(trace));save(run/'feasibility_checks.json',plain(checks))
    record=dict(solver=status,error=error,wall_s=time.monotonic()-start,initial=p.costs(fresh,include_relative=False),
        final=None if x is None else p.costs(x,include_relative=False),
        diagnostic_relative_edge_distortion_not_optimized_cost=None if x is None else p.costs(x)['R'])
    save(run/'planning_result.json',record)
    return x

def run_method(run,name):
    out=run/'methods'/name;out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');common=read(run/'common_state.json');ref=read(run/'references.json')[name]
    assert may_execute(ref['safety'])
    manifest=read(run/'source_manifest.json');source=Path(manifest['source']);env=geometry(source)['on']
    fresh=np.load(run/'references/M0_NATIVE_world.npy');phase=deepcopy(common)
    a=activation_at_B(common);x=np.array(common['B']);dt=common['integration_dt_s'];bt=common['B_tick'];bs=common['B_sim_s']
    tick=bt;events=[];states=[dict(absolute_tick=bt,time_s=0.,sim_time_s=bs,pose_world=x.tolist())]
    commands=[];guards=[];pacing=[];requests=[];waits=[];logical=None;worker=None;abort=None;error=None;termination='OBSERVATION_CAP';t0=time.monotonic()
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
        logical=LogicalRelease(read(run/'schedule.json')['pairs'],common['original_generation']);last_result=bs
        while tick<bt+cfg['integration_steps']:
            t=(tick-bt)*dt;sim=bs+t;st=dict(state_id=tick,tick=tick,**stamp(sim,t),x=float(x[0]),y=float(x[1]),yaw=float(x[2]))
            event=logical.release(tick)
            if event is not None:
                event['continuation_seen']=dict(tick=tick,time_s=t,**stamp(sim,t))
                accepted=a.accept(event,sim);event['activation_accepted']=accepted;events.append(event)
                if not accepted:raise RuntimeError('released official result rejected by unchanged activation')
                last_result=sim
            if tick in logical.by_submit:
                sid=f'{name}_NEW_{len(requests):06d}'
                requests.append(dict(tick=tick,solve_id=sid,pose_world=x.tolist(),sim_time_s=sim))
                waits.append(logical.collect(worker,tick,sid,x,sim,events,cfg['wall_wait_timeout_s']))
            proposal,c,_=preview_activation(a,st,None,commands[-1] if commands else None)
            if c['reason']=='controller_timeout':
                abort=dict(state=st,proposed_command=c,command_applied=False,kind='hold_timeout');termination='HOLD_TIMEOUT';break
            g=guard_check(env,x,c,dt);guards.append(g)
            if not g['safe']:
                abort=dict(state=st,proposed_command=c,guard=g,command_applied=False);termination='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND';break
            if sim-last_result>2.:raise RuntimeError('controller result stall')
            a=proposal;c.update(command_id=tick,relative_time_s=t,origin='COMMON_FIRST_FRESH' if tick==bt else 'NEW_COMMON_B_MPC');commands.append(c)
            x=integrate_unicycle(x,command(c),dt);tick+=1
            newsim=bs+(tick-bt)*dt
            states.append(dict(absolute_tick=tick,time_s=(tick-bt)*dt,sim_time_s=newsim,pose_world=x.tolist()))
    except Exception:
        import traceback
        error=traceback.format_exc();termination='CONTROLLER_ERROR'
    finally:
        if logical is not None and logical.pending is not None:
            _,unreleased=logical.pending
            unreleased['after_termination']=True
            unreleased['withheld_by_logical_scheduler']=True
            events.append(unreleased)
        if worker:
            worker.shutdown()
            for event in worker.drain():event['after_termination']=True;events.append(event)
    result=dict(method=name,phase=phase,states=states,commands=commands,events=events,guards=guards,pacing=pacing,logical_waits=waits,submit_requests=requests,label=LABEL,
        termination=termination,error=error,safety_abort=abort,wall_s=time.monotonic()-t0,
        new_MPC_submitted=sum(e.get('status')=='submitted' for e in events),new_MPC_solved=sum(e.get('type')=='solve_result' for e in events),
        new_MPC_requests=len(requests),historical_future_results_replayed=0,common_first_command_restored=True,
        new_VLA_calls=0,new_RGB_calls=0,new_optimizer_calls=0,reference_world_sha256=ref['world_sha256'],raw_FRESH_sha256=manifest['FRESH_sha256'])
    save(out/'rollout.json',result)
    metric=None if not commands else plain(evaluate(result,fresh,env,read(source/'scenario.json'),read(run/'metric_protocol.json')))
    save(out/'metrics.json',metric)
    own=None if not commands else plain(evaluate(result,np.load(ref['world_path']),env,read(source/'scenario.json'),read(run/'metric_protocol.json')))
    save(out/'own_reference_metrics.json',own)
    save(out/'hashes.json',{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()})
    print(json.dumps(dict(method=name,termination=termination,steps=len(commands),solves=result['new_MPC_solved'],error=error)),flush=True)


def execute(run):
    verify(run,True)
    save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),order=ORDER,label=LABEL,retries=0))
    x=solve_no_relative(run)
    if x is None:
        save(run/'completion.json',dict(classification='PLANNING_SOLVER_BLOCKED',rollouts=0));return
    refs=read(run/'prepared_references.json');c=read(run/'common_state.json');manifest=read(run/'source_manifest.json')
    _,fresh,raw,_=source_phase(Path(manifest['source']))
    refs['NO_RELATIVE']=reference_record(run,'NO_RELATIVE',x,raw,c,fresh,geometry(Path(manifest['source'])))
    save(run/'references.json',refs);(run/'methods').mkdir()
    for name in ORDER:
        if not may_execute(refs[name]['safety']):
            out=run/'methods'/name;out.mkdir();save(out/'skipped.json',dict(status='REFERENCE_GEOMETRY_UNSAFE',metrics=None,rollouts=0));continue
        run_method(run,name)
    verify(run);save(run/'completion.json',dict(completed=stamp(),source_preserved=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--mode',choices=['prepare','preflight','freeze','execute'],required=True)
    args=parser.parse_args();globals()[args.mode](args.run.resolve())
