#!/usr/bin/env python3
"""Authenticate, freeze and perform exactly twelve planning solves; no execution."""
import argparse
from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import scipy
import yaml
from reconciliation.join_source03 import read, save, sha
from reconciliation.graph_optimizer import SolverConfig, solve_least_squares, OptimizationError
from reconciliation.b_to_entry_graph_diag02 import DiagnosticProblem, sample_hermite, NEW, VARIANTS, PNGS, COLLAPSE_M
from reconciliation.spatial_correspondence_selector import SOURCE_IDS
from reconciliation.relative_factor_multisource import geometry, reference_safety

CONFIG = ROOT/'configs/b_to_entry_graph_formulation_diag02.yaml'
RESULTS = ROOT/'results/b_to_entry_graph_formulation_diag02'
DOC = ROOT/'docs/B_TO_ENTRY_GRAPH_FORMULATION_DIAG_02.md'
FILES = [CONFIG, ROOT/'src/reconciliation/b_to_entry_graph_diag02.py',
         *[ROOT/'scripts'/f'{s}_b_to_entry_graph_formulation_diag02.py' for s in ['run','validate','report']],
         ROOT/'tests/test_b_to_entry_graph_formulation_diag02.py']
ZERO_COUNTS = dict(MPC_solves=0, rollouts=0, state_integrations=0, controller_memory_updates=0,
                   command_applications=0, LightNav=0, RGB=0, Isaac=0, retries=0)


def plain(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    if isinstance(x, dict): return {k:plain(v) for k,v in x.items()}
    if isinstance(x, (tuple,list)): return [plain(v) for v in x]
    return x


def git(*args):
    return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()


def stamp():
    return datetime.now(timezone.utc).isoformat()


def write_array(path, array):
    with Path(path).open('xb') as f: np.save(f,array,allow_pickle=False)


@contextmanager
def planning_only():
    """Fail before an accidental controller, integration or acquisition call."""
    old = sys.getprofile()
    forbidden = {'integrate_unicycle', 'run_method', 'run_rollout', 'run_source_episode',
                 'solve_mpc', 'compute_control', 'capture_rgb'}
    def profile(frame,event,arg):
        if event != 'call': return
        filename = frame.f_code.co_filename.replace('\\','/')
        if frame.f_code.co_name in forbidden or filename.endswith('/vln_mujoco/mpc.py'):
            raise RuntimeError('controller/integration/acquisition forbidden in planning diagnostic')
    sys.setprofile(profile)
    try: yield
    finally: sys.setprofile(old)


def authenticate(cfg, deep=False):
    assert cfg['sources']==SOURCE_IDS and cfg['new_variant_order']==NEW
    assert cfg['solver']==asdict(SolverConfig())
    assert cfg['numerical_collapse_threshold_m']==COLLAPSE_M
    assert cfg['feasibility_min_edge_m']==1e-12
    assert sha(ROOT/'src/reconciliation/graph_optimizer.py')==cfg['solver_source_sha256']
    assert sha(ROOT/'src/reconciliation/b_to_entry_bridge.py')==cfg['historical_bridge_source_sha256']
    for key in ['historical_result','historical_failure_diagnostics']:
        assert sha(ROOT/cfg[key])==cfg[key+'_sha256']
    h=read(ROOT/cfg['historical_result']);old=Path(h['run'])
    assert sha(old/'result_hashes.json')==h['result_hashes_sha256']
    for p,digest in read(old/'result_hashes.json').items(): assert sha(old/p)==digest,p
    assert h['summary']==read(old/'summary.json') and h['validation']['valid']
    from run_b_to_entry_bridge01 import verify as historical_verify
    historical_verify(old)
    if deep:
        from validate_b_to_entry_bridge01 import validate as historical_validate
        summary,validation=historical_validate(old)
        assert summary==h['summary'] and validation['valid'] and validation['all_historical_validators']
    failure=read(ROOT/cfg['historical_failure_diagnostics'])
    assert failure['source_result_ledger_sha256']==sha(old/'result_hashes.json')
    for row in failure['rows']:
        assert sha(row['trace_path'])==row['trace_sha256']
        assert not row['returned_reference'] and not row['installed_in_controller'] and not row['executed']
    return h


def load_problem(folder, variant):
    spec=read(folder/'input.json');M=VARIANTS[variant][1]
    return DiagnosticProblem(spec['P'],spec['B'],np.load(folder/'suffix.npy'),spec['d_F_m'],
                             np.load(folder/f'initial_M{M}.npy'),variant)


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());h=authenticate(cfg,deep=True)
    run.mkdir(parents=True,exist_ok=False);save(run/'protocol.json',cfg)
    selected=[]
    for i,s in enumerate(h['summary']['selected_sources'],1):
        old=Path(s['folder']);folder=run/'sources'/f'S{i}';folder.mkdir(parents=True)
        spec=read(old/'bridge_input.json');assert spec['M']==2
        for a,b in [('hermite_bridge.npy','initial_M2.npy'),('suffix_native_installed.npy','suffix.npy')]:
            shutil.copyfile(old/a,folder/b)
        save(folder/'input.json',dict(**spec,source_id=s['id'],role=cfg['source_roles'][i-1],historical_folder=str(old)))
        initial,metadata=sample_hermite(spec['P'],spec['B'],np.load(folder/'suffix.npy'),3)
        write_array(folder/'initial_M3.npy',initial);save(folder/'M3_sampling.json',metadata)
        m2,_=sample_hermite(spec['P'],spec['B'],np.load(folder/'suffix.npy'),2)
        np.testing.assert_array_equal(m2,np.load(folder/'initial_M2.npy'))
        env=geometry(old);checks={}
        for v in VARIANTS:
            p=load_problem(folder,v);checks[v]=reference_safety(p.reference(p.initial[1:-1]),env)
            assert checks[v]['clearance_valid'] and p.nondegenerate(p.initial[1:-1])
        save(folder/'initial_safety.json',checks)
        selected.append(dict(label=f'S{i}',id=s['id'],folder=str(folder),historical_folder=str(old),raw_sha256=s['raw_sha256']))
    save(run/'selected_sources.json',selected)
    save(run/'authentication.json',dict(historical_bridge_and_six_earlier_validators=True,
        A2_authentication=True,exact_C3=True,exact_P_B_E_suffix=True,
        M2_initialization_byte_identical=True,M3_same_continuous_curve=True,new_scientific_solves=0,
        versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__),**ZERO_COUNTS))
    print(json.dumps(dict(prepared=str(run),sources=4,new_solves=0)))


def freeze(run):
    cfg=yaml.safe_load(CONFIG.read_text());h=authenticate(cfg)
    assert cfg==read(run/'protocol.json')
    files=list(dict.fromkeys([Path(p) for p in read(Path(h['run'])/'freeze.json')['files']]+FILES+
                            [ROOT/cfg['historical_result'],ROOT/cfg['historical_failure_diagnostics']]))
    with (run/'protocol_document.md').open('xb') as f:f.write(DOC.read_bytes())
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},
        inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},
        historical_result_ledger_sha256=sha(Path(h['run'])/'result_hashes.json'),new_graph_solves=12,**ZERO_COUNTS))
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),
        authentication=read(run/'authentication.json'),selected_sources=read(run/'selected_sources.json'),
        sources={s['label']:dict(input=read(Path(s['folder'])/'input.json'),M3_sampling=read(Path(s['folder'])/'M3_sampling.json'),
            M2=np.load(Path(s['folder'])/'initial_M2.npy'),M3=np.load(Path(s['folder'])/'initial_M3.npy'))
            for s in read(run/'selected_sources.json')}))


def verify(run,pushed=False):
    authenticate(read(run/'protocol.json'))
    f=read(run/'freeze.json')
    for group in ['files','inputs']:
        for p,digest in f[group].items():assert sha(p)==digest,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in f['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p)
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT)))==git('hash-object',str(run/'protocol_document.md'))
        assert git('rev-parse','HEAD:'+str((RESULTS/'freeze_summary.json').relative_to(ROOT)))==git('hash-object',str(RESULTS/'freeze_summary.json'))


def solve_variant(folder, variant, freeze_sha):
    if variant not in NEW:raise ValueError('A2 is historical reuse only')
    p=load_problem(folder,variant);env=geometry(Path(read(folder/'input.json')['historical_folder']))
    out=folder/'planning'/variant;out.mkdir(parents=True,exist_ok=False)
    save(out/'optimization_start.json',dict(freeze_sha=freeze_sha,started=stamp(),calls=1,variant=variant,
        initial_sha256=sha(folder/f'initial_M{p.M}.npy'),solver=asdict(SolverConfig())))
    trace=[];checks=[]
    def feasible(x):
        safety=reference_safety(p.reference(x),env);nondegenerate=p.nondegenerate(x)
        ok=nondegenerate and safety['clearance_valid']
        checks.append(dict(interior=x.copy(),safety=safety,nondegenerate=nondegenerate,accepted=ok))
        return ok
    def observe(e):trace.append({**e,'factor_costs':p.costs(e['state'])})
    initial=p.initial[1:-1].copy();start=time.monotonic();error=None
    try:
        result=solve_least_squares(initial,p.residual,SolverConfig(),
            candidate_feasibility_fn=feasible,iteration_callback=observe)
        solver=result.to_dict();last=result.optimized
    except (OptimizationError, FloatingPointError, ValueError) as e:
        error=dict(type=type(e).__name__,message=str(e))
        last=np.asarray(trace[-1]['state']) if trace else initial
        solver=dict(converged=False,termination_reason=type(e).__name__,
            iterations=trace[-1]['iteration'] if trace else 0)
    elapsed=time.monotonic()-start
    save(out/'trace.json',plain(trace));save(out/'feasibility_checks.json',plain(checks))
    save(out/'result.json',plain(dict(solver=solver,error=error,wall_s=elapsed)))
    write_array(out/'last_accepted_bridge.npy',p.bridge(last))
    # A converged state is saved separately; the STABLE gate is evaluated by the validator.
    if solver['converged']:write_array(out/'converged_reference.npy',p.reference(last))
    print(json.dumps(dict(source=folder.name,variant=variant,converged=solver['converged'],
                         termination=solver['termination_reason'])),flush=True)


def execute(run):
    verify(run,pushed=True);frozen=git('rev-parse','HEAD')
    save(run/'execution_start.json',dict(sha=frozen,started=stamp(),expected_graph_solves=12,**ZERO_COUNTS))
    with planning_only():
        for s in read(run/'selected_sources.json'):
            for variant in NEW:solve_variant(Path(s['folder']),variant,frozen)
    verify(run)
    save(run/'completion.json',dict(completed=stamp(),new_graph_solves=12,A2_new_solves=0,**ZERO_COUNTS))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--mode',choices=['prepare','freeze','execute'],required=True)
    args=parser.parse_args();globals()[args.mode](args.run.resolve())
