#!/usr/bin/env python3
"""Freeze once, then one Hermite rollout and one graph solve/rollout per source."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import scipy
import yaml
import run_relative_factor_multisource01 as legacy
import run_spatial_entry_suffix_execution01 as prior
from reconciliation.join_source03 import read, save, sha
from reconciliation.robotless_online import stamp
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares, OptimizationError
from reconciliation.b_to_entry_bridge import *
from reconciliation.spatial_entry_suffix import suffix_reference
from run_osa03_native_continuation import git, plain
CONFIG = ROOT/'configs/b_to_entry_bridge_01.yaml'
RESULTS = ROOT/'results/b_to_entry_bridge_01'
DOC = ROOT/'docs/B_TO_ENTRY_BRIDGE_01.md'
NEW_FILES = [CONFIG, ROOT/'src/reconciliation/b_to_entry_bridge.py',
    *[ROOT/'scripts'/f'{s}_b_to_entry_bridge01.py' for s in ['run','validate','report']],
    ROOT/'tests/test_b_to_entry_bridge01.py']


def authenticate(cfg, deep=False):
    p = ROOT/cfg['historical_result']
    assert sha(p) == cfg['historical_result_sha256']
    result = read(p); old = Path(result['run'])
    assert sha(old/'result_hashes.json') == result['result_hashes_sha256']
    for path,h in read(old/'result_hashes.json').items():
        assert sha(old/path) == h, path
    assert result['summary'] == read(old/'summary.json') and result['validation']['valid']
    prior.verify(old)
    if deep:
        from validate_spatial_entry_suffix_execution01 import validate
        s,v = validate(old)
        assert s == result['summary'] and v['valid']
    assert cfg['sources'] == SOURCE_IDS and cfg['order'] == ORDER
    assert cfg['solver'] == asdict(SolverConfig())
    assert cfg['integration_steps'] == 180 and cfg['primary_intervals'] == 54
    return result


def load_problem(folder):
    p = read(folder/'bridge_input.json')
    return BridgeProblem(p['P'],p['B'],np.load(folder/'suffix_native_installed.npy'),p['d_F_m'],p['M'])


def write_array(path, value):
    with Path(path).open('xb') as f:
        np.save(f,value,allow_pickle=False)


def make_reference(folder, name, bridge):
    problem = load_problem(folder)
    spec = read(folder/'bridge_input.json'); common = read(folder/'common_state.json')
    raw = np.load(read(folder/'reuse.json')['methods'][NATIVE]['reference']['local_path'])
    w,l,labels = reference_arrays(problem,bridge,common['fresh_capture_pose'],raw,spec['original_ids'])
    result = {}
    for frame,array in [('world',w),('local',l)]:
        p = folder/'references'/f'{name}_{frame}.npy';write_array(p,array)
        result.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
    safety = reference_safety(w,geometry(folder))
    result.update(safety=safety,labels=labels,
        status='REFERENCE_GEOMETRY_SAFE' if safety['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')
    return result


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text()); historical = authenticate(cfg,deep=True)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg)
    selected = []
    for s in historical['summary']['selected_sources']:
        old = Path(s['folder']);folder = run/'sources'/s['id'].replace('/','__')
        (folder/'references').mkdir(parents=True)
        copies = {}
        for name in ['common_state.json','source_manifest.json','schedule.json','scenario.json',
                     'metric_protocol.json','recorded_old_to_B.npy','source_audit.json','entry.json']:
            shutil.copyfile(old/name,folder/name)
            copies[name] = dict(path=str(old/name),sha256=sha(old/name))
        save(folder/'protocol.json',cfg)
        refs = read(old/'references.json');c=read(folder/'common_state.json');entry=read(folder/'entry.json')
        fresh=np.load(refs[NATIVE]['world_path']);raw=np.load(refs[NATIVE]['local_path'])
        _,_,descriptor=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['historical_C3'])
        assert descriptor == refs[ENTRY]['descriptor']
        from validate_spatial_entry_suffix_execution01 import method_folder
        reuse = dict(source_folder=str(old),copies=copies,methods={n:dict(folder=str(method_folder(old,n)),
            reference=refs[n],hashes_sha256=sha(method_folder(old,n)/'hashes.json')) for n in [NATIVE,ENTRY]})
        save(folder/'reuse.json',reuse)
        native = np.asarray(read(method_folder(old,NATIVE)/'restoration.json')['installed_world'])
        # Current four entries are strictly interior, so the original IDs follow E*.
        ids = [i for i in descriptor['row_original_identities'] if i is not None]
        E = np.array(entry['correspondence']['target_world'])
        ids = [i for i in ids if descriptor['correspondence']['arc_m'] <
               float(np.r_[0,np.cumsum(np.linalg.norm(np.diff(fresh[:,:2],axis=0),axis=1))][i])-EPS]
        suffix = np.vstack([E,native[ids]])
        past=np.load(folder/'recorded_old_to_B.npy');ctx=read(folder/'source_audit.json')['context']
        np.testing.assert_array_equal(past[-1],c['B']);np.testing.assert_array_equal(past[-2],ctx['P'])
        assert ctx['previous_state_id'] == c['B_tick']-1 and ctx['switch_state_id'] == c['B_tick']
        bridge,description=hermite_bridge(past[-2],c['B'],suffix)
        save(folder/'bridge_input.json',dict(**description,original_ids=ids,
            P_state_id=ctx['previous_state_id'],B_state_id=c['B_tick'],
            P_sim_time_s=c['B_sim_s']-c['integration_dt_s'],B_sim_time_s=c['B_sim_s'],
            entry_C3_exact=True,native_installation_path=str(method_folder(old,NATIVE)/'restoration.json'),
            native_installation_sha256=sha(method_folder(old,NATIVE)/'restoration.json')))
        write_array(folder/'suffix_native_installed.npy',suffix)
        write_array(folder/'hermite_bridge.npy',bridge)
        shutil.copyfile(refs[NATIVE]['world_path'],folder/'references/M0_NATIVE_world.npy')
        newref=make_reference(folder,HERMITE,bridge)
        save(folder/'prepared_references.json',{HERMITE:newref})
        legacy.prior.preflight(folder)
        pre=read(folder/'restoration_preflight.json')
        installed=np.asarray(pre['rows'][0]['installed_world'])
        np.testing.assert_array_equal(installed[len(bridge):],native[ids])
        np.testing.assert_array_equal(np.load(newref['world_path'])[len(bridge):],native[ids])
        selected.append(dict(id=s['id'],folder=str(folder),historical_folder=str(old),raw_sha256=s['raw_sha256']))
    save(run/'selected_sources.json',selected)
    save(run/'authentication.json',dict(historical_entry_and_all_five_prior_validators=True,exact_C3_matches=4,
        exact_P_previous_tick=True,official_restore_preflight_numerical_solves=0,
        native_installed_downstream_exact=True,reused_rollouts=8,new_scientific_calls=0,
        versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__)))
    print(json.dumps(dict(prepared=str(run),M=[read(Path(s['folder'])/'bridge_input.json')['M'] for s in selected])))


def freeze(run):
    cfg=yaml.safe_load(CONFIG.read_text());historical=authenticate(cfg)
    assert cfg==read(run/'protocol.json')
    files=list(dict.fromkeys([Path(p) for p in read(Path(historical['run'])/'freeze.json')['files']]+NEW_FILES))
    with (run/'protocol_document.md').open('xb') as f:f.write(DOC.read_bytes())
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},
        inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},graph_solves=4,new_rollouts=8,retries=0))
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),
        authentication=read(run/'authentication.json'),selected_sources=read(run/'selected_sources.json'),
        sources={s['id']:{k:read(Path(s['folder'])/(k+'.json')) for k in ['bridge_input','entry','schedule','common_state','reuse']}
                 for s in read(run/'selected_sources.json')}))


def verify(run,pushed=False):
    authenticate(read(run/'protocol.json'))
    for group in ['files','inputs']:
        for p,h in read(run/'freeze.json')[group].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p)
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def solve_bridge(folder,freeze_sha):
    p=load_problem(folder);initial=np.load(folder/'hermite_bridge.npy')[1:-1];env=geometry(folder)
    out=folder/'planning'/GRAPH;out.mkdir(parents=True,exist_ok=False)
    trace=[];checks=[]
    def feasible(x):
        check=reference_safety(p.reference(x),env)
        ok=p.nondegenerate(x) and check['clearance_valid']
        checks.append(dict(interior=x.copy(),safety=check,nondegenerate=p.nondegenerate(x),accepted=ok))
        return ok
    def observe(e):trace.append({**e,'factor_costs':p.costs(e['state'])})
    save(out/'optimization_start.json',dict(freeze_sha=freeze_sha,started=stamp(),calls=1,
        initial_sha256=sha(folder/'hermite_bridge.npy'),solver=asdict(SolverConfig())))
    start=time.monotonic();error=None;bridge=None
    try:
        result=solve_least_squares(initial,p.residual,SolverConfig(),candidate_feasibility_fn=feasible,iteration_callback=observe)
        status=result.to_dict()
        if result.converged:bridge=p.bridge(result.optimized)
        else:error='solver did not converge; no graph rollout'
    except (OptimizationError,ValueError,FloatingPointError) as e:
        error=str(e);status=dict(termination_reason=type(e).__name__)
    save(out/'trace.json',plain(trace));save(out/'feasibility_checks.json',plain(checks))
    save(out/'result.json',dict(solver=status,error=error,wall_s=time.monotonic()-start))
    if bridge is not None:write_array(out/'bridge.npy',bridge)
    return bridge


def execute(run):
    verify(run,pushed=True);frozen=git('rev-parse','HEAD')
    save(run/'execution_start.json',dict(sha=frozen,started=stamp(),retries=0))
    # Source order S1..S4, Hermite rollout then graph planning/rollout. No result feeds a later method.
    for s in read(run/'selected_sources.json'):
        folder=Path(s['folder']);refs={n:r['reference'] for n,r in read(folder/'reuse.json')['methods'].items()}
        refs.update(read(folder/'prepared_references.json'))
        # Generic executor reads this file. Initial namespace excludes the uncomputed graph.
        save(folder/'references.json',refs)
        h=refs[HERMITE]
        if h['safety']['clearance_valid']:legacy.run_method(folder,HERMITE)
        else:save(folder/'methods'/HERMITE/'skipped.json',dict(status=h['status'],metrics=None))
        bridge=solve_bridge(folder,frozen)
        if bridge is None:
            save(folder/'methods'/GRAPH/'skipped.json',dict(status='PLANNING_FAILED',metrics=None))
        else:
            ref=make_reference(folder,GRAPH,bridge)
            # Keep frozen/precomputed records immutable; publish final refs as a new file,
            # then replace only this explicitly derived execution index.
            refs[GRAPH]=ref
        save(folder/'final_references.json',refs)
        (folder/'references.json').write_text(json.dumps(plain(refs),indent=2)+'\n')
        if bridge is not None:
            if ref['safety']['clearance_valid']:legacy.run_method(folder,GRAPH)
            else:save(folder/'methods'/GRAPH/'skipped.json',dict(status=ref['status'],metrics=None))
    verify(run)
    save(run/'completion.json',dict(completed=stamp(),no_historical_reruns=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    args=parser.parse_args();globals()[args.mode](args.run.resolve())
