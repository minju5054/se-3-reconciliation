#!/usr/bin/env python3
"""Single frozen multisource comparison. Historical experiment files are immutable."""
import argparse, json, sys, time
from pathlib import Path
from copy import deepcopy
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_osa03_relative_factor_ablation01 as prior
from run_osa03_native_continuation import plain, ask, git
from reconciliation.join_source03 import read, save, sha
from reconciliation.osa03_relative_ablation import ORDER, LABEL, LogicalRelease
from reconciliation.osa03_common_b import activation_at_B, may_execute, references
from reconciliation.local_se2_reconciliation import LocalSE2Problem
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares, OptimizationError
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256
from reconciliation.osa03_native import command, evaluate
from reconciliation.robotless_online import integrate_unicycle, stamp
from reconciliation.join_online02 import preview_activation, guard_check
from reconciliation.relative_factor_multisource import (load_source, geometry, reference_record,
    unique_sources, candidate_inventory)
from robotless_online_handoffs import Worker
CONFIG=ROOT/'configs/relative_factor_multisource_01.yaml'
RESULTS=ROOT/'results/relative_factor_multisource_01'
DOC=ROOT/'docs/RELATIVE_FACTOR_MULTISOURCE_01.md'
FILES=list(dict.fromkeys(prior.FILES+[ROOT/p for p in [
    'scripts/run_osa03_relative_factor_replication01.py',
    'scripts/validate_osa03_relative_factor_replication01.py',
    'src/reconciliation/osa03_relative_replication.py',
    'src/reconciliation/handoff_delay_attribution.py',
    'src/reconciliation/relative_factor_multisource.py',
    'scripts/run_relative_factor_multisource01.py',
    'scripts/validate_relative_factor_multisource01.py',
    'scripts/report_relative_factor_multisource01.py',
    'tests/test_relative_factor_multisource01.py',
    'configs/relative_factor_multisource_01.yaml']]))


def checked(cfg):
    old=yaml.safe_load(prior.CONFIG.read_text())
    for k in ['solver','order','relative_factor','normalization','initialization','weights',
              'footprint_radius_m','required_edge_clearance_m','integration_steps',
              'primary_intervals','retries','wall_wait_timeout_s','metric_target']:
        assert cfg[k]==old[k],k
    assert cfg['solver']==asdict(SolverConfig()) and 3<=len(cfg['selected'])<=6
    for p,h in {**cfg['inventory_hashes'],**cfg['legacy_results']}.items(): assert sha(ROOT/p)==h,p


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());checked(cfg)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg)
    save(run/'candidate_inventory.json',candidate_inventory(cfg))
    selected=[]
    for spec in cfg['selected']:
        data=load_source(cfg,spec);folder=run/'sources'/spec['id'].replace('/','__')
        folder.mkdir(parents=True);(folder/'references').mkdir()
        for filename,key in [('common_state','common'),('source_manifest','manifest'),
                             ('schedule','schedule'),('scenario','scene'),('source_audit','audit')]:
            save(folder/(filename+'.json'),data[key])
        save(folder/'protocol.json',cfg)
        save(folder/'metric_protocol.json',read(ROOT/cfg['osa_run']/'metric_protocol.json'))
        np.save(folder/'recorded_old_to_B.npy',data['past'],allow_pickle=False)
        f=references(data['common']['fresh_capture_pose'],data['common']['B'],data['fresh'],data['fresh'])
        refs={n:reference_record(folder,n,f[k],data['raw'],data['common'],data['fresh'],geometry(folder))
              for n,k in [('M0_NATIVE','M0_NATIVE'),('M1_TAPER','M1_SE2_TAPER')]}
        save(folder/'prepared_references.json',refs)
        assert refs['M0_NATIVE']['safety']['clearance_valid']
        prior.preflight(folder)
        assert read(folder/'restoration_preflight.json')['provenance']['official_settings']==data['manifest']['mpc']['official_settings']
        selected.append(dict(**spec,folder=str(folder),raw_sha256=data['manifest']['FRESH_sha256'],
                             raw_value_sha256=data['audit']['raw_value_sha256']))
    unique_sources(selected);save(run/'selected_sources.json',selected)
    print(json.dumps(dict(prepared=str(run),sources=selected,scientific_calls=0)))


def freeze(run):
    with (run/'protocol_document.md').open('xb') as f:f.write(DOC.read_bytes())
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in FILES},
        inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},optimizer_calls=8,maximum_rollouts=16,retries=0))
    RESULTS.mkdir(parents=True,exist_ok=True)
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),
        selected_sources=read(run/'selected_sources.json'),candidates=read(run/'candidate_inventory.json'),
        sources={s['id']:{k:read(Path(s['folder'])/(k+'.json')) for k in ['common_state','schedule','source_audit']}
                 for s in read(run/'selected_sources.json')}))


def verify(run,pushed=False):
    f=read(run/'freeze.json');cfg=read(run/'protocol.json');checked(cfg)
    for group in ['files','inputs']:
        for p,h in f[group].items():assert sha(p)==h,p
    for s in read(run/'selected_sources.json'):
        folder=Path(s['folder']);m=read(folder/'source_manifest.json')
        assert read(folder/'restoration_preflight.json')['provenance']['official_settings']==m['mpc']['official_settings']
        for p,h in m['hashes'].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in f['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def solve_condition(run, name):
    """Same problem/init/solver/gate for both; only R inclusion differs."""
    include = read(run/'protocol.json')['relative_factor'][name]
    out = run/'planning'/name; out.mkdir(parents=True, exist_ok=False)
    common = read(run/'common_state.json'); fresh = np.load(run/'references/M0_NATIVE_world.npy')
    p = LocalSE2Problem(common['fresh_capture_pose'], common['B'], fresh)
    env = geometry(run); trace = []; checks = []
    def feasible(x):
        check = env['on'].check_polyline(x); checks.append(dict(state=x.copy(), check=check))
        return check['clearance_valid']
    def observe(e):
        trace.append({**e, 'factor_costs': p.costs(e['state'], include_relative=include)})
    save(out/'optimization_start.json', dict(freeze_sha=git('rev-parse', 'HEAD'), started=stamp(), calls=1,
        initialization='original observation-anchored FRESH', initial_world_sha256=sha(run/'references/M0_NATIVE_world.npy'),
        include_relative=include, solver_config=read(run/'protocol.json')['solver']))
    start = time.monotonic(); error = None; x = None
    try:
        solved = solve_least_squares(p.fresh, lambda state: p.residual_vector(state, include_relative=include),
            SolverConfig(**read(run/'protocol.json')['solver']), candidate_feasibility_fn=feasible, iteration_callback=observe)
        status = solved.to_dict()
        if solved.converged:
            x = solved.optimized
        else:
            error = 'planning did not converge; returned state recorded in trace, no rollout'
    except (OptimizationError, FloatingPointError, ValueError) as exc:
        error = str(exc); status = dict(termination_reason=type(exc).__name__, error=error)
    save(out/'solver_trace.json', plain(trace)); save(out/'feasibility_checks.json', plain(checks))
    save(out/'planning_result.json', dict(solver=status, error=error, wall_s=time.monotonic()-start,
        initial=p.costs(fresh, include_relative=include), final=None if x is None else p.costs(x, include_relative=include),
        diagnostic_relative_edge_distortion_not_optimized_cost=None if x is None or include else p.costs(x)['R']))
    return x



def run_method(run,name):
    out=run/'methods'/name;out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');common=read(run/'common_state.json');ref=read(run/'references.json')[name]
    assert may_execute(ref['safety'])
    manifest=read(run/'source_manifest.json');source=Path(manifest['source']);env=geometry(run)['on']
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
        assert restored['generation']==common['original_generation'] and restored['next_submit_tick']==common['next_submit_after_B']
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
    metric=None if not commands else plain(evaluate(result,fresh,env,read(run/'scenario.json'),read(run/'metric_protocol.json')))
    save(out/'metrics.json',metric)
    own=None if not commands else plain(evaluate(result,np.load(ref['world_path']),env,read(run/'scenario.json'),read(run/'metric_protocol.json')))
    save(out/'own_reference_metrics.json',own)
    save(out/'hashes.json',{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()})
    print(json.dumps(dict(method=name,termination=termination,steps=len(commands),solves=result['new_MPC_solved'],error=error)),flush=True)



def execute(run):
    verify(run,True)
    save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),label=LABEL,retries=0))
    for s in read(run/'selected_sources.json'):
        folder=Path(s['folder']);refs=read(folder/'prepared_references.json')
        c=read(folder/'common_state.json');fresh=np.load(refs['M0_NATIVE']['world_path'])
        raw=np.load(refs['M0_NATIVE']['local_path']);env=geometry(folder)
        for name in ORDER[2:]:
            x=solve_condition(folder,name)
            refs[name]=(dict(status='PLANNING_FAILED',safety={'clearance_valid':False}) if x is None else
                        reference_record(folder,name,x,raw,c,fresh,env))
        save(folder/'references.json',refs)
        for name in ORDER:
            if not may_execute(refs[name]['safety']):
                out=folder/'methods'/name;out.mkdir(parents=True)
                save(out/'skipped.json',dict(status=refs[name]['status'],metrics=None,rollouts=0));continue
            run_method(folder,name)
    verify(run);save(run/'completion.json',dict(completed=stamp(),sources_preserved=True))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())
