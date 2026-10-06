#!/usr/bin/env python3
"""Authenticate/prepare, freeze, then exactly five planning-only canonical solves."""
import argparse
from dataclasses import asdict
from pathlib import Path
import shutil
import sys
import time
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
from reconciliation.canonical_se2_graph import CanonicalSE2Problem
from reconciliation.canonical_graph_audit01 import BASELINES, PNGS, planning_guard, diagnostics, v2_hermite_similarity
from reconciliation.graph_optimizer import SolverConfig, OptimizationError, solve_least_squares
from reconciliation.join_source03 import read,save,sha
from reconciliation.relative_factor_multisource import geometry,reference_safety
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.robotless_online import stamp
from run_local_se2_reconciliation_formulation01 import git,plain
from validate_local_se2_reconciliation_formulation01 import independent_costs,parity
CONFIG=ROOT/'configs/canonical_se2_graph_formulation_audit_01.yaml'
RESULTS=ROOT/'results/canonical_se2_graph_formulation_audit_01'
DOC=ROOT/'docs/CANONICAL_SE2_GRAPH_FORMULATION_AUDIT_01.md'


def np_save(path,array):
    with Path(path).open('xb') as stream: np.save(stream,array,allow_pickle=False)


def contract(cfg):
    assert cfg['solver']==asdict(SolverConfig())
    assert cfg['normalization']==dict(translation_m=.1,yaw_degrees=10.)
    assert cfg['lambdas']==dict(L=1.,R=1.,A=1.) and cfg['residual_order']==['L','R','A']
    assert cfg['canonical_scientific_solves']==5 and len(cfg['selected_sources'])==5
    assert all(cfg[k]==0 for k in ['MPC','controller_rollouts','LightNav','RGB','Isaac','GP','retries','historical_optimization_calls'])
    assert cfg['figures']==PNGS and sha(RESULTS/'formulation_audit.json')==cfg['formulation_audit_sha256']


def authenticate(cfg,validate_saved=False):
    contract(cfg); out={}
    for path,digest in cfg['historical_results'].items():
        p=ROOT/path; assert sha(p)==digest,path
        assert git('show',cfg['starting_sha']+':'+path)==p.read_text().strip()
        result=read(p);run=Path(result['run'])
        assert sha(run/'result_hashes.json')==result['result_hashes_sha256']
        for q,h in read(run/'result_hashes.json').items(): assert sha(run/q)==h,q
        for q,h in result['table_hashes'].items(): assert sha(p.parent/q)==h,q
        out[path]=dict(run=str(run),result_sha256=digest,result_hashes_sha256=sha(run/'result_hashes.json'))
    from run_direct_transition_hard_eval02 import verify as direct_verify
    from run_b_to_entry_boundary_row_ablation04 import verify as boundary_verify
    direct=Path(out['results/direct_transition_hard_eval_02/result_summary.json']['run'])
    boundary=Path(out['results/b_to_entry_boundary_row_ablation_04/result_summary.json']['run'])
    direct_verify(direct);boundary_verify(boundary)
    from validate_local_se2_reconciliation_formulation01 import validate as local_validate
    local=local_validate(ROOT/cfg['local_history'],authoritative=validate_saved)
    if validate_saved:
        from validate_direct_transition_hard_eval02 import validate as direct_validate
        from validate_b_to_entry_boundary_row_ablation04 import validate as boundary_validate
        # No corpus re-selection/search: validate only the already selected records.
        ds,dv=direct_validate(direct,deep=False);bs,bv=boundary_validate(boundary,deep=True)
        assert dv['valid'] and bv['valid'] and bv['all_historical_validators']
        out['saved_validation']=dict(direct=True,boundary_and_all_ancestors=True,local=local,new_scientific_solves=0)
    return out


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());history=authenticate(cfg,validate_saved=True)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg);save(run/'authentication.json',history)
    selected=[];hashes={};similarities={}
    with planning_guard():
        for s in cfg['selected_sources']:
            old=ROOT/s['folder'];f=run/'sources'/s['label'];f.mkdir(parents=True)
            c=read(old/'common_state.json');manifest=read(old/'source_manifest.json');refs=read(old/'references.json')
            native=refs.get('M0_NATIVE') or read(old/'native_reference.json')['M0_NATIVE']
            raw=np.load(native['local_path']);fresh=np.load(native['world_path'])
            for frame in ['world','local']: assert sha(native[frame+'_path'])==native[frame+'_sha256']
            for path,h in manifest['hashes'].items(): assert sha(path)==h;hashes[path]=h
            np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],raw),fresh,rtol=0,atol=cfg['algebra_atol'])
            assert sha(native['local_path'])==manifest['FRESH_sha256']
            entry=read(old/'entry.json')
            from reconciliation.spatial_entry_suffix import suffix_reference
            _,_,e=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],entry['correspondence'])
            assert e['correspondence']==entry['correspondence']
            p=CanonicalSE2Problem(c['fresh_capture_pose'],c['B'],fresh)
            states=[p.fresh,p.target]
            if s['label']=='OSA03_R00':
                hist=ROOT/cfg['local_history'];h=np.load(hist/'derived/optimized_world.npy')
                assert np.array_equal(fresh,np.load(hist/'derived/original_world.npy'))
                np_save(f/'local_se2_historical.npy',h);states.append(h)
                for path in hist.rglob('*'):
                    if path.is_file():hashes[str(path)]=sha(path)
            for state in states:
                for key in ['L','R','A']:np.testing.assert_array_equal(p.raw_residuals(state)[key],p.algebra.raw_residuals(state)[key])
                np.testing.assert_array_equal(p.residual_vector(state),p.algebra.residual_vector(state))
                parity(p.costs(state),independent_costs(fresh,p.target,state))
            np_save(f/'raw_fresh.npy',fresh);np_save(f/'raw_observation_local.npy',raw);np_save(f/'transported_target.npy',p.target)
            installed={};baseline={}
            for name,original in BASELINES.items():
                rec=refs[original]
                for frame in ['world','local']:
                    path=rec[frame+'_path'];assert sha(path)==rec[frame+'_sha256'];hashes[path]=sha(path)
                w=np.load(rec['world_path']);np_save(f/(name+'.npy'),w);baseline[name]=w
                method=old/'methods'/original
                if not method.exists(): method=Path(read(old/'reuse.json')['methods'][original]['folder'])
                rest=method/'restoration.json';installed[name]=np.array(read(rest)['installed_world'])
                np_save(f/(name+'_installed.npy'),installed[name]);hashes[str(rest)]=sha(rest)
                ids=e['row_original_identities'][1:];offset=2 if name=='B_ENTRY' else 3
                # Fixed original suffix matches the historical Native world installation.
                native_inst=(np.load(old/'native_installed.npy') if (old/'native_installed.npy').exists() else
                    np.array(read(Path(read(old/'reuse.json')['methods']['M0_NATIVE']['folder'])/'restoration.json')['installed_world']))
                assert w[offset:].tobytes()==native_inst[ids].tobytes()
            similarities[s['label']]=v2_hermite_similarity(baseline['HERMITE'],baseline['V2_SINGLE_NODE'],installed['HERMITE'],installed['V2_SINGLE_NODE'])
            for name in ['common_state.json','source_manifest.json','source_audit.json','scenario.json','schedule.json','entry.json']:
                hashes[str(old/name)]=sha(old/name)
            audit=read(old/'source_audit.json');ctx=audit['context'];env=geometry(old)
            safety=reference_safety(fresh,env);assert safety['clearance_valid']
            context=dict(label=s['label'],id=s['id'],historical_folder=str(old),A=p.A.tolist(),B=p.B.tolist(),
                N=len(fresh),raw_local_path=native['local_path'],raw_world_path=native['world_path'],
                raw_local_sha256=sha(native['local_path']),raw_world_sha256=sha(native['world_path']),
                times={k:ctx.get(k) for k in ['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch','obs_state_id','switch_state_id']},
                common_state_sha256=sha(old/'common_state.json'),schedule_sha256=sha(old/'schedule.json'),
                original_frame='observation-local +x forward,+y left,yaw CCW, metres/radians',world_frame='world XY metres,+Z up,yaw CCW radians',
                waypoint_times=None,algebra_parity=True,raw_safety=safety)
            save(f/'context.json',context);selected.append(dict(label=s['label'],id=s['id'],folder=str(f)))
    save(run/'selected_sources.json',selected);save(run/'source_manifest.json',dict(files=hashes,source_reuse=True,new_source_search=False))
    save(run/'v2_hermite_similarity.json',plain(similarities))
    print(dict(prepared=str(run),sources=len(selected),canonical_solves=0,MPC=0))


def freeze(run):
    cfg=read(run/'protocol.json');assert cfg==yaml.safe_load(CONFIG.read_text());contract(cfg)
    for name in ['protocol.json','selected_sources.json','source_manifest.json']:
        shutil.copyfile(run/name,RESULTS/name)
    files=[ROOT/p for p in git('ls-files','src','scripts','tests').splitlines() if (ROOT/p).is_file()]
    files += [CONFIG,*[ROOT/p for p in git('ls-files','--others','--exclude-standard','src','scripts','tests').splitlines() if 'canonical' in p],RESULTS/'formulation_audit.json',RESULTS/'method_dof_table.csv',*[RESULTS/n for n in ['protocol.json','selected_sources.json','source_manifest.json']]]
    shutil.copyfile(DOC,run/'protocol_document.md')
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}))
    save(RESULTS/'freeze.json',dict(run=str(run),freeze=read(run/'freeze.json'),sources=read(run/'selected_sources.json'),
        source_manifest_sha256=sha(run/'source_manifest.json'),protocol=cfg))


def verify(run,pushed=False):
    cfg=read(run/'protocol.json');contract(cfg)
    assert read(RESULTS/'freeze.json')['freeze']==read(run/'freeze.json')
    for group in read(run/'freeze.json').values():
        for path,h in group.items(): assert sha(path)==h,path
    for path,h in read(run/'source_manifest.json')['files'].items(): assert sha(path)==h,path
    for p,h in cfg['historical_results'].items(): assert sha(ROOT/p)==h,p
    if pushed:
        head=git('rev-parse','HEAD');remote=git('ls-remote','origin','refs/heads/main').split()[0];assert head==remote
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze.json'))


def load_problem(folder):
    c=read(folder/'context.json');return CanonicalSE2Problem(c['A'],c['B'],np.load(folder/'raw_fresh.npy'))


def execute(run):
    verify(run,pushed=True);freeze_sha=git('rev-parse','HEAD');save(run/'execution_start.json',dict(scientific_freeze_sha=freeze_sha,started=stamp()))
    cfg=read(run/'protocol.json')
    with planning_guard(allow_optimizer=True,optimizer_budget=5) as calls:
        for spec in read(run/'selected_sources.json'):
            f=Path(spec['folder']);p=load_problem(f);env=geometry(Path(read(f/'context.json')['historical_folder']))
            save(f/'optimization_start.json',dict(scientific_freeze_sha=freeze_sha,started=stamp(),call=1,retry=0))
            trace=[];checks=[]
            def feasible(x):
                safe=reference_safety(x,env);checks.append(dict(state=x.tolist(),check=safe));return safe['clearance_valid']
            def callback(e): trace.append(plain(dict(e,factor_costs=p.costs(e['state']))))
            start=time.monotonic();technical=None
            try:
                solved=solve_least_squares(p.fresh,p.residual_vector,SolverConfig(),candidate_feasibility_fn=feasible,iteration_callback=callback)
                x=solved.optimized;solver=solved.to_dict()
            except (OptimizationError,FloatingPointError,np.linalg.LinAlgError) as error:
                x=np.asarray(trace[-1]['state']) if trace else p.fresh.copy()
                solver=dict(converged=False,termination_reason=str(error));technical=str(error)
            wall=time.monotonic()-start
            np_save(f/'canonical_optimized.npy',x);np_save(f/'canonical_original_A_local.npy',p.original_observation_local(x))
            safe=reference_safety(x,env);baselines={n:np.load(f/(n+'.npy')) for n in BASELINES}
            d=diagnostics(p,x,baselines,safe)
            costs={n:p.costs(a) for n,a in [('RAW',p.fresh),('FULL_TRANSPORT',p.target),('CANONICAL',x)]}
            costs['B_ENTRY']=dict(status='N/A: different row identities, no canonical-state embedding imposed')
            historic_delta=None
            if (f/'local_se2_historical.npy').exists():
                h=np.load(f/'local_se2_historical.npy');costs['LOCAL_SE2_HISTORICAL']=p.costs(h)
                historic_delta=float(np.max(abs(x-h)))
                if historic_delta>cfg['historical_solution_atol']:technical='historical solution parity failure'
            result=dict(label=spec['label'],solver=solver,technical_failure=technical,wall_s=wall,diagnostics=d,
                historical_solution_max_abs_difference=historic_delta,factor_costs=costs,canonical_scientific_calls=1,
                actual_transition_safety='NOT_EVALUATED_NO_EXECUTION',scientific_freeze_sha=freeze_sha)
            for name,value in [('result',result),('solver_trace',trace),('feasibility_checks',checks),('factor_costs',costs),('rigid_fit',d['rigid_fit']),('reference_geometry',safe)]:save(f/(name+'.json'),plain(value))
    assert calls['canonical_optimizer_calls']==5
    save(run/'completion.json',dict(canonical_scientific_solves=calls['canonical_optimizer_calls'],retries=0,MPC=0,controller_rollouts=0,LightNav=0,RGB=0,Isaac=0,GP=0,historical_optimization_calls=0))
    verify(run);print(read(run/'completion.json'))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    args=parser.parse_args();globals()[args.mode](args.run.resolve())
