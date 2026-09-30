#!/usr/bin/env python3
"""Four new half-transport solves/rollouts; Native and Full are authenticated reuse."""
import argparse,json,shutil,time
from pathlib import Path
from dataclasses import asdict
import run_relative_factor_multisource01 as legacy
from run_relative_factor_multisource01 import ROOT,git,plain,stamp,np,yaml
from reconciliation.join_source03 import read,save,sha
from reconciliation.state_shift_transport_scale import *
from reconciliation.graph_optimizer import SolverConfig,solve_least_squares,OptimizationError
from reconciliation.relative_factor_multisource import geometry,reference_record
CONFIG=ROOT/'configs/state_shift_transport_scale_01.yaml'
RESULTS=ROOT/'results/state_shift_transport_scale_01'
DOC=ROOT/'docs/STATE_SHIFT_TRANSPORT_SCALE_01.md'
NEW_FILES=['src/reconciliation/state_shift_transport_scale.py','scripts/run_state_shift_transport_scale01.py',
    'scripts/validate_state_shift_transport_scale01.py','scripts/report_state_shift_transport_scale01.py',
    'tests/test_state_shift_transport_scale01.py','configs/state_shift_transport_scale_01.yaml']
FILES=list(dict.fromkeys(legacy.FILES+[ROOT/p for p in NEW_FILES]))


def authenticate(cfg,deep=False):
    for p,h in cfg['historical_hashes'].items():assert sha(ROOT/p)==h,p
    tracked=read(ROOT/cfg['historical_result']);old=Path(tracked['run'])
    assert sha(old/'result_hashes.json')==tracked['result_hashes_sha256']
    for p,h in read(old/'result_hashes.json').items():assert sha(old/p)==h,p
    legacy.verify(old)
    assert tracked['summary']==read(old/'summary.json') and tracked['validation']['valid']
    specs=read(old/'selected_sources.json');assert [s['id'] for s in specs]==SOURCE_IDS
    if deep:
        from validate_relative_factor_multisource01 import validate
        s,v=validate(old);assert v['valid'] and s==tracked['summary']
    for k in ['solver','normalization','initialization','weights','footprint_radius_m',
              'required_edge_clearance_m','integration_steps','primary_intervals','retries','wall_wait_timeout_s']:
        assert cfg[k]==read(old/'protocol.json')[k],k
    assert cfg['solver']==asdict(SolverConfig()) and cfg['alphas']==ALPHAS and cfg['order']==ORDER
    return old,specs


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());old,specs=authenticate(cfg,deep=True)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg);selected=[]
    for spec in specs:
        before=Path(spec['folder']);folder=run/'sources'/spec['id'].replace('/','__')
        folder.mkdir(parents=True);(folder/'references').mkdir()
        copies={}
        for f in ['common_state.json','source_manifest.json','schedule.json','scenario.json',
                  'source_audit.json','metric_protocol.json','recorded_old_to_B.npy','restoration_preflight.json']:
            shutil.copyfile(before/f,folder/f);copies[f]=dict(path=str(before/f),sha256=sha(before/f))
        save(folder/'protocol.json',cfg)
        refs=read(before/'references.json');reused={n:refs[n] for n in [NATIVE,FULL]}
        save(folder/'prepared_references.json',reused)
        # Generic official runner reads this named file; copy bytes without recomputing coordinates.
        shutil.copyfile(refs[NATIVE]['world_path'],folder/'references/M0_NATIVE_world.npy')
        reused_methods={n:dict(folder=str(before/'methods'/n),
            hashes_sha256=sha(before/'methods'/n/'hashes.json'),reference=refs[n]) for n in [NATIVE,FULL]}
        save(folder/'reuse.json',dict(source_folder=str(before),copies=copies,methods=reused_methods))
        selected.append(dict(spec,folder=str(folder),historical_folder=str(before)))
    save(run/'selected_sources.json',selected)
    save(run/'authentication.json',dict(historical_run=str(old),historical_result_sha256=sha(ROOT/cfg['historical_result']),
        historical_saved_validators_passed=True,new_scientific_calls=0,reused_references=8,reused_rollouts=8))
    print(json.dumps(dict(prepared=str(run),sources=SOURCE_IDS,scientific_calls=0)))


def freeze(run):
    with (run/'protocol_document.md').open('xb') as f:f.write(DOC.read_bytes())
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in FILES},
        inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},new_planning_calls=4,new_rollouts=4,retries=0))
    RESULTS.mkdir(parents=True,exist_ok=True)
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),
        selected_sources=read(run/'selected_sources.json'),authentication=read(run/'authentication.json'),
        source_states_schedules={s['id']:{k:read(Path(s['folder'])/(k+'.json')) for k in ['common_state','schedule','reuse']}
                               for s in read(run/'selected_sources.json')}))


def verify(run,pushed=False):
    cfg=read(run/'protocol.json');authenticate(cfg)
    for group in ['files','inputs']:
        for p,h in read(run/'freeze.json')[group].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def solve_half(folder):
    out=folder/'planning'/HALF;out.mkdir(parents=True,exist_ok=False)
    c=read(folder/'common_state.json');cfg=read(folder/'protocol.json')
    fresh=np.load(folder/'references/M0_NATIVE_world.npy')
    p=TransportScaleProblem(c['fresh_capture_pose'],c['B'],fresh,alpha=.5)
    env=geometry(folder);trace=[];checks=[]
    def feasible(x):
        check=env['on'].check_polyline(x);checks.append(dict(state=x.copy(),check=check))
        return check['clearance_valid']
    def observe(e):trace.append({**e,'factor_costs':p.costs(e['state'])})
    save(out/'optimization_start.json',dict(freeze_sha=git('rev-parse','HEAD'),started=stamp(),calls=1,
        alpha=.5,include_relative=True,initialization=cfg['initialization'],
        initial_world_sha256=sha(folder/'references/M0_NATIVE_world.npy'),solver_config=cfg['solver']))
    start=time.monotonic();error=None;x=None
    try:
        solved=solve_least_squares(p.fresh,p.residual_vector,SolverConfig(**cfg['solver']),
            candidate_feasibility_fn=feasible,iteration_callback=observe)
        status=solved.to_dict()
        if solved.converged:x=solved.optimized
        else:error='planning did not converge; no rollout or retry'
    except (OptimizationError,FloatingPointError,ValueError) as exc:
        error=str(exc);status=dict(termination_reason=type(exc).__name__,error=error)
    save(out/'solver_trace.json',plain(trace));save(out/'feasibility_checks.json',plain(checks))
    save(out/'planning_result.json',dict(solver=status,error=error,wall_s=time.monotonic()-start,
        initial=p.costs(fresh),final=None if x is None else p.costs(x),
        diagnostic_relative_edge_distortion_not_optimized_cost=None))
    return x


def execute(run):
    verify(run,True)
    save(run/'execution_start.json',dict(sha=git('rev-parse','HEAD'),started=stamp(),retries=0))
    for spec in read(run/'selected_sources.json'):
        folder=Path(spec['folder']);refs=read(folder/'prepared_references.json')
        c=read(folder/'common_state.json');fresh=np.load(refs[NATIVE]['world_path']);raw=np.load(refs[NATIVE]['local_path'])
        x=solve_half(folder)
        refs[HALF]=(dict(status='PLANNING_FAILED',safety={'clearance_valid':False}) if x is None else
                    reference_record(folder,HALF,x,raw,c,fresh,geometry(folder)))
        save(folder/'references.json',{n:refs[n] for n in ORDER})
        if refs[HALF]['safety']['clearance_valid']:legacy.run_method(folder,HALF)
        else:
            out=folder/'methods'/HALF;out.mkdir(parents=True)
            save(out/'skipped.json',dict(status=refs[HALF]['status'],metrics=None,rollouts=0))
    verify(run);save(run/'completion.json',dict(completed=stamp(),sources_preserved=True))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())
