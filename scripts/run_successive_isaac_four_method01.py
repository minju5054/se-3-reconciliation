#!/usr/bin/env python3
"""Frozen single-handoff planning and real Isaac execution. No automatic retry."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_isaac_replay01 import authenticate,load_boundary,extract_schedule,jsonlines
from reconciliation.successive_isaac_four_method01 import ORDER,ProgressGraph,fixed_references,isolated_schedule,parity
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares
from reconciliation.osa03_common_b import activation_at_B
from reconciliation.osa03_relative_ablation import LogicalRelease
from reconciliation.join_online02 import preview_activation,guard_check
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.se2 import relative_pose,local_trajectory_to_world
from reconciliation.robotless_online import stamp
from continuous_obstacle_reveal_exploratory_geometry02 import environments
from run_osa03_native_continuation import ask,git,plain
from robotless_online_handoffs import Worker
from run_continuous_obstacle_reveal_episode01b import historical_launch,launch_argv,loader_checks,reserve_launch,observe_process,sanitized_environment
CONFIG=ROOT/'configs/successive_isaac_four_method_comparison_01.yaml'
OUT=ROOT/'results/successive_isaac_four_method_comparison_01'
DOC=ROOT/'docs/SUCCESSIVE_ISAAC_FOUR_METHOD_COMPARISON_01.md'


def namespace(run):
    run=Path(run).resolve()
    assert run.parent==ROOT/'data/successive_isaac_four_method_comparison_01'
    return run


def inputs(cfg):
    prior=yaml.safe_load((ROOT/cfg['source_config']).read_text())
    source,ep,auth=authenticate(ROOT,prior)
    b,arr,states,commands,events=load_boundary(source,ep,prior)
    full=extract_schedule(states,commands,jsonlines(ep/'scheduler.jsonl'),events,'chunk_001')
    s=isolated_schedule(full,'chunk_001')
    assert (s['start_tick'],s['stop_tick'],s['integration_steps'])==(cfg['start_tick'],cfg['stop_tick'],cfg['integration_steps'])
    assert s['attempted_submit_ticks']==cfg['submit_ticks'] and s['application_ticks']==cfg['application_ticks']
    assert cfg['solver']==asdict(SolverConfig()) and cfg['order']==ORDER
    g=cfg['GRAPH']
    assert [g[k] for k in ['lambda_T','lambda_R','lambda_A']]==[1.,1.,1.]
    assert [g[k] for k in ['sigma_translation_m','sigma_yaw_deg','sigma_direction_deg','noncollapsed_edge_min_m']]==[.1,10.,15.,1e-12]
    ready=read(source/'source_bundle/handoffs/C0_to_C1/ready_record.json')
    prior_apps=[c for c in commands if int(c['application_tick'])<b['B_tick'] and c['reason']=='new_solve']
    c=dict(B=b['B'],B_tick=b['B_tick'],B_sim_s=b['B_sim_s'],u_minus=b['u_minus'],u_B_plus=b['u_B_plus'],
        u_mem_B=b['controller_memory']['previous_control'],delta_u_B=(np.asarray(b['u_B_plus'])-b['u_minus']).tolist(),
        previous_command_application_sim_s=float(prior_apps[-1]['sim_time_s']),fresh_capture_pose=b['A'],fresh_chunk_id='chunk_001',
        fresh_version=b['reference_version'],original_generation=b['generation'],integration_dt_s=b['integration_dt_s'],
        next_submit_after_B=s['attempted_submit_ticks'][0],first_FRESH_solve=ready['first_solve'])
    selected_states=[x for x in states if s['start_tick']<=int(x['tick'])<=s['stop_tick']]
    selected_commands=[x for x in commands if s['start_tick']<=int(x['application_tick'])<s['stop_tick']]
    hist=dict(states=[dict(absolute_tick=int(x['tick']),time_s=(int(x['tick'])-s['start_tick'])*b['integration_dt_s'],
        sim_time_s=float(x['sim_time_s']),pose_world=[float(x[k]) for k in ['x','y','yaw']]) for x in selected_states],
        controls=[[float(x[k]) for k in ['v_mps','omega_radps']] for x in selected_commands],commands=selected_commands,
        solves=[e for e in events if e.get('type')=='solve_result' and e['input_state_id'] in s['accepted_submit_ticks']],
        guards=[g for g in jsonlines(ep/'guard.jsonl') if s['start_tick']<=g['command']['application_tick']<s['stop_tick']])
    return source,ep,auth,b,arr,c,s,hist


def record_reference(run,name,world,local,labels,env):
    c=read(run/'common_state.json')
    installed=local_trajectory_to_world(c['fresh_capture_pose'],local)
    np.testing.assert_allclose(installed,world,rtol=0,atol=1e-12)
    info={}
    for kind,arr in [('world',world),('local',local)]:
        p=run/'references'/f'{name}_{kind}.npy'
        with p.open('xb') as f:np.save(f,arr,allow_pickle=False)
        info.update({kind+'_path':str(p),kind+'_sha256':sha(p)})
    safe=whole_raw_polyline_check(world,env)
    actual=whole_raw_polyline_check(installed,env)
    return dict(**info,labels=labels,safety=safe,installed_roundtrip_safety=actual,
        installed_roundtrip_max_error=float(abs(installed-world).max()),
        valid=bool(safe['clearance_valid'] and actual['clearance_valid']))


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());source,ep,auth,b,arr,c,s,hist=inputs(cfg)
    run.mkdir(parents=True,exist_ok=False)
    for name in ['references','logs','technical_preflight']: (run/name).mkdir()
    save(run/'protocol.json',cfg);save(run/'common_state.json',c);save(run/'schedule.json',s)
    save(run/'source_authentication.json',dict(**auth,boundary=b,source=str(source),episode=str(ep)))
    save(run/'historical_prefix.json',hist)
    for name in ['scenario.json','config_snapshot.yaml','speed_intervention.json']:
        (run/name).write_bytes((source/name).read_bytes())
    save(run/'mpc_provenance.json',read(source/'workers.json')['mpc']['provenance'])
    ref,suffix,entry,hermite,graph=fixed_references(arr['fresh_world'],arr['fresh_raw_local'],b['A'],b['B'],b['P'])
    for name,arr0 in [('suffix',suffix),('graph_initial',graph.initial()),('old',arr['old_world'])]:
        with (run/f'{name}.npy').open('xb') as f:np.save(f,arr0)
    save(run/'entry.json',entry);save(run/'hermite.json',hermite)
    env=environments(source)[1]
    refs={n:record_reference(run,n,*r,env) for n,r in ref.items()}
    save(run/'prepared_references.json',refs)
    initial=graph.initial();local=relative_pose(b['A'],initial);local[-1]=np.load(b['reference_files']['fresh_raw_local']['path'])[-1]
    preview=record_reference(run,'GRAPH_INITIAL_DIAGNOSTIC',initial,local,['B',*[f'X_{i}' for i in range(1,len(initial)-1)],'F_end'],env)
    save(run/'preflight_references.json',{**refs,'GRAPH_INITIAL_DIAGNOSTIC':preview})
    lc=yaml.safe_load((ROOT/'configs/successive_native_source_acquisition_02.yaml').read_text())
    provenance=historical_launch(lc)
    lc['collector']='scripts/isaac/successive_four_method01.py'
    save(run/'launch_config.json',lc);save(run/'launch_environment.json',dict(argv=launch_argv(run,lc),historical_sanitation=provenance))
    assert loader_checks(run,lc)['valid']
    assert all(r['valid'] for r in refs.values()),'METHOD_REFERENCE_UNSAFE before science'
    # Native solver objects may be constructed, but every numerical solve is forbidden.
    mpc=read(run/'mpc_provenance.json')
    result=subprocess.run([mpc['python_executable'],str(ROOT/'scripts/successive_four_method_mpc_worker01.py'),
        '--run',str(run),'--lightnav-checkout',mpc['lightnav_checkout'],'--preflight'],
        env=sanitized_environment(os.environ,lc),capture_output=True,text=True,check=True)
    pre=json.loads(result.stdout);assert pre['passed'] and pre['numerical_MPC_calls']==0
    assert pre['provenance']['official_settings']==mpc['official_settings']
    for row in pre['rows']:
        name=row['identity'];actual=np.array(row['installed_world']);expected=np.load(read(run/'preflight_references.json')[name]['world_path'])
        np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-12)
        if name=='RAW': assert actual.tobytes()==expected.tobytes()
        if name in ['B_ENTRY','HERMITE']:
            count=len(entry['row_original_identities'])-1
            assert actual[-count:].tobytes()==expected[-count:].tobytes()
    save(run/'restoration_preflight.json',pre)
    print(json.dumps(dict(prepared=str(run),entry=entry['correspondence'],Hermite_M=hermite['M'],schedule=s,scientific_calls=0)))


def freeze(run):
    cfg=read(run/'protocol.json');inputs(cfg)
    files=set(read(ROOT/'results/successive_isaac_reconciliation_replay_01/freeze_manifest.json')['code_hashes'])
    files.update(['src/reconciliation/spatial_entry.py','src/reconciliation/spatial_entry_suffix.py',
        'src/reconciliation/spatial_correspondence_selector.py','src/reconciliation/boundary_row_ablation04.py',
        'src/reconciliation/b_to_entry_bridge.py','src/reconciliation/graph_optimizer.py','src/reconciliation/se2.py',
        'src/reconciliation/gp_se2_join01.py','scripts/isaac/robotless_runtime.py',
        'scripts/isaac/robotless_old_consistent_observation.py','scripts/isaac/join_source02_preflight.py',
        'scripts/isaac/join_source03_render.py',str(CONFIG.relative_to(ROOT)),
        'src/reconciliation/successive_isaac_four_method01.py','src/reconciliation/isaac_clock_restore01.py',
        'scripts/successive_four_method_mpc_worker01.py','scripts/isaac/successive_four_method01.py',
        'scripts/run_successive_isaac_four_method01.py','scripts/validate_successive_isaac_four_method01.py',
        'scripts/report_successive_isaac_four_method01.py','tests/test_successive_isaac_four_method01.py'])
    (run/'protocol_document.md').write_bytes(DOC.read_bytes())
    external=[Path('/home/gpuadmin/isaacsim')/p for p in [
        'python.sh','setup_python_env.sh',
        'extsDeprecated/isaacsim.core.api/isaacsim/core/api/world/world.py',
        'extsDeprecated/isaacsim.core.api/isaacsim/core/api/simulation_context/simulation_context.py']]
    manifest=dict(files={str(ROOT/p):sha(ROOT/p) for p in sorted(files)},
        external_files={str(p):sha(p) for p in external},
        inputs={str(p):sha(p) for p in sorted(run.rglob('*')) if p.is_file()},budget=cfg['budget'])
    save(run/'freeze.json',manifest)
    save(OUT/'freeze_manifest.json',dict(run=str(run),manifest=manifest,source=read(run/'source_authentication.json'),
        entry=read(run/'entry.json'),schedule=read(run/'schedule.json'),reference_definitions=cfg,
        historical_replay_audit_unchanged=True,scientific_calls_before_freeze=0))


def verify(run,pushed=False):
    f=read(run/'freeze.json')
    for path,h in {**f['files'],**f['inputs'],**f['external_files']}.items():assert sha(path)==h,path
    inputs(read(run/'protocol.json'))
    if pushed:
        head=git('rev-parse','HEAD');assert head==git('ls-remote','origin','refs/heads/main').split()[0]
        for path in f['files']:
            rel=str(Path(path).relative_to(ROOT));assert git('rev-parse',f'HEAD:{rel}')==git('hash-object',path),rel
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(OUT/'freeze_manifest.json'))


def graph_solve(run):
    b=read(run/'source_authentication.json')['boundary'];p=ProgressGraph(b['P'],b['B'],np.load(run/'suffix.npy'))
    initial=np.load(run/'graph_initial.npy');np.testing.assert_array_equal(initial,p.initial())
    env=environments(ROOT/read(run/'protocol.json')['source'])[1];trace=[];checks=[]
    def feasible(x):
        q=whole_raw_polyline_check(p.unpack(x),env);valid=p.noncollapsed(x) and q['clearance_valid']
        checks.append(dict(valid=bool(valid),safety=q));return valid
    def observe(e):trace.append(dict(**plain(e),factors=p.costs(e['state'])))
    save(run/'graph_solve_start.json',dict(at=stamp(),freeze_sha=read(run/'scientific_start.json')['freeze_sha'],optimizer_calls=1))
    start=time.monotonic();result=None;error=None;ref=None;solved=None
    try:
        solved=solve_least_squares(p.pack(initial),p.residual,SolverConfig(**read(run/'protocol.json')['solver']),
            candidate_feasibility_fn=feasible,iteration_callback=observe)
        result=solved.to_dict()
        if solved.converged and p.noncollapsed(solved.optimized):
            w=p.unpack(solved.optimized);local=relative_pose(b['A'],w)
            local[-1]=np.load(read(run/'prepared_references.json')['RAW']['local_path'])[-1]
            ref=record_reference(run,'GRAPH',w,local,['B',*[f'X_{i}' for i in range(1,len(w)-1)],'F_end'],env)
    except Exception: error=traceback.format_exc()
    save(run/'graph_trace.json',trace);save(run/'graph_feasibility.json',checks)
    report=dict(solver=result,error=error,wall_s=time.monotonic()-start,initial_costs=p.costs(p.pack(initial)),
        final_costs=None if solved is None else p.costs(solved.optimized),
        valid=ref is not None and ref['valid'],optimizer_calls=1)
    save(run/'graph_planning.json',report)
    if ref is not None: save(run/'references.json',{**read(run/'prepared_references.json'),'GRAPH':ref})
    return report


def run_method(run,name,isaac):
    out=run/'methods'/name;out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');c=read(run/'common_state.json');schedule=read(run/'schedule.json');ref=read(run/'references.json')[name]
    env=environments(ROOT/cfg['source'])[1];phase=deepcopy(c);bt=c['B_tick'];bs=c['B_sim_s'];dt=c['integration_dt_s']
    worker=None;events=[];collected=[];states=[];commands=[];guards=[];requests=[];waits=[];abort=None;error=None
    termination='PRE_NEXT_INSTALL_CAP';restored=None;scene_restore=None
    save(out/'execution_start.json',dict(method=name,at=stamp(),freeze_sha=read(run/'scientific_start.json')['freeze_sha']))
    try:
        scene_restore=ask(isaac,'reset_method',method=name);save(out/'scene_restoration.json',scene_restore)
        np.testing.assert_array_equal(scene_restore['pose_world'],c['B']);assert scene_restore['sim_time_s']==bs
        x=np.array(scene_restore['pose_world']);states=[dict(absolute_tick=bt,time_s=0.,sim_time_s=bs,pose_world=x.tolist())]
        mpc=read(run/'mpc_provenance.json')
        worker=Worker([mpc['python_executable'],str(ROOT/'scripts/successive_four_method_mpc_worker01.py'),
            '--run',str(run),'--lightnav-checkout',mpc['lightnav_checkout']],out/'worker.log')
        ready=worker.await_type('ready',timeout=30);save(out/'worker_provenance.json',ready)
        assert ready['provenance']['official_settings']==mpc['official_settings']
        assert ready['provenance']['mpc_source_sha256']==mpc['mpc_source_sha256']
        restored=ask(worker,'initialize_common_B',common_path=str(run/'common_state.json'),common_sha256=sha(run/'common_state.json'),reference=ref,identity=name)
        save(out/'restoration.json',restored)
        for k,v in [('held_command',c['u_B_plus']),('previous_control',c['u_mem_B']),('B',c['B'])]:np.testing.assert_array_equal(restored[k],v)
        assert restored['generation']==c['original_generation'] and restored['restored_future_results']==restored['new_solve_calls']==0
        installed=np.asarray(restored['installed_world']);assert whole_raw_polyline_check(installed,env)['clearance_valid']
        a=activation_at_B(c);logical=LogicalRelease(schedule['pairs'],c['original_generation'])
        ids={r['input_state_id']:r['solve_id'] for r in read(run/'historical_prefix.json')['solves']}
        for tick in range(bt,schedule['stop_tick']):
            t=(tick-bt)*dt;sim=bs+t
            event=logical.release(tick)
            if event is not None:
                event['continuation_seen']=dict(tick=tick,time_s=t,sim_time_s=sim)
                assert a.accept(event,sim);events.append(event)
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
    except Exception:
        error=traceback.format_exc();termination='TECHNICAL_BLOCKED'
    finally:
        if worker is not None:
            worker.shutdown();extra=worker.drain()
            save(out/'worker_shutdown.json',dict(messages=extra,returncode=worker.process.returncode))
    r=dict(method=name,phase=phase,reference=ref,events=events,states=states,commands=commands,guards=guards,
        submit_requests=requests,collected_results=collected,waits=waits,abort=abort,error=error,termination=termination,
        controller_restoration=restored,scene_restoration=scene_restore)
    save(out/'rollout.json',plain(r));return r


def execute(run):
    verify(run,True);cfg=read(run/'protocol.json')
    save(run/'scientific_start.json',dict(at=stamp(),freeze_sha=git('rev-parse','HEAD'),budget=cfg['budget']))
    g=graph_solve(run)
    if not g['valid']:
        save(run/'execution_summary.json',dict(classification='GRAPH_PLANNING_FAILED',executed_methods=[],Isaac_launches=0));return
    lc=read(run/'launch_config.json');argv=launch_argv(run,lc);assert argv==read(run/'launch_environment.json')['argv']
    reserve_launch(run,argv);isaac=None;done=[];classification='FOUR_METHOD_MIXED_EVIDENCE';error=None
    try:
        isaac=Worker(argv,run/'logs/isaac.log')
        ready=isaac.await_type('ready',timeout=cfg['runtime']['startup_timeout_s']);save(run/'isaac_ready.json',ready)
        try:save(run/'isaac_process.json',observe_process(ready['pid']))
        except OSError as exc:save(run/'isaac_process_observer_error.json',dict(error=repr(exc)))
        for name in ORDER:
            r=run_method(run,name,isaac);done.append(name)
            if r['error'] is not None:classification='TECHNICAL_BLOCKED';break
            if name=='RAW':
                gate=parity(r,read(run/'historical_prefix.json'),read(run/'schedule.json'),cfg['parity'])
                save(run/'RAW_parity.json',gate)
                if not gate['passed']:classification='RAW_ISAAC_REPLAY_PARITY_FAIL';break
    except Exception:
        error=traceback.format_exc();classification='TECHNICAL_BLOCKED'
    finally:
        if isaac is not None:
            isaac.shutdown();save(run/'isaac_exit.json',dict(returncode=isaac.process.returncode,messages=isaac.drain()))
    save(run/'execution_summary.json',dict(classification=classification,error=error,executed_methods=done,Isaac_launches=1))


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','verify','execute'],required=True);a=p.parse_args()
    globals()[a.mode](namespace(a.run))


if __name__=='__main__':main()
