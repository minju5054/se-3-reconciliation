#!/usr/bin/env python3
"""One sealed R00 spatial solve. Prepare is saved-only; execute requires pushed freeze.

No model, controller, simulator, correspondence or GP calls are made here.
"""
import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.local_se2_reconciliation import LocalSE2Problem, planning_diagnostics
from reconciliation.graph_optimizer import SolverConfig, OptimizationError, solve_least_squares
from reconciliation.se2 import relative_pose, se2_log, local_trajectory_to_world
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_geometry import projection_from_metadata, compose_environment

CONFIG = ROOT/'configs/local_se2_reconciliation_formulation_01.yaml'
CODE = [
    'src/reconciliation/local_se2_reconciliation.py', 'src/reconciliation/graph_optimizer.py',
    'src/reconciliation/se2.py', 'src/reconciliation/trajectory.py',
    'src/reconciliation/gp_se2_environment.py', 'src/reconciliation/join_source02_geometry.py',
    'scripts/run_local_se2_reconciliation_formulation01.py',
    'scripts/validate_local_se2_reconciliation_formulation01.py',
    'scripts/report_local_se2_reconciliation_formulation01.py',
    'scripts/validate_obstacle_source_acquisition03.py',
    'tests/test_local_se2_reconciliation.py', 'tests/test_local_se2_saved.py',
    'tests/fixtures/local_se2_legacy_solver.json']


def read(path):
    return json.loads(Path(path).read_text())


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: plain(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)): return [plain(v) for v in value]
    return value


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(plain(value), f, indent=2, allow_nan=False)
        f.write('\n')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git','-C',str(ROOT),*args], text=True).strip()


def verify_hashes(mapping):
    for path, expected in mapping.items():
        if sha(path) != expected: raise ValueError(f'input hash mismatch: {path}')


def config_contract(cfg):
    assert cfg['representative'] == 'REPEAT_00'
    assert cfg['fresh_sha256'] == '8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521'
    assert cfg['source_run'] == 'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    assert cfg['normalization'] == dict(translation_m=.1,yaw_degrees=10.)
    assert cfg['lambdas'] == dict(L=1.,R=1.,A=1.)
    assert cfg['weights'] == dict(L='(1-s)^2',A='s^2')
    assert cfg['solver'] == asdict(SolverConfig())
    assert cfg['safety']['footprint_radius_m'] == .20 and cfg['safety']['required_edge_clearance_m'] == .05
    assert cfg['scientific_planning_solve_budget'] == 1
    assert all(cfg[k] == 0 for k in ['LightNav_calls','MPC_calls','GP_calls','actual_execution_calls'])


def load_source(cfg):
    """Read/authenticate only R00 and shared scene records; never inspect R01 arrays."""
    config_contract(cfg)
    source=ROOT/cfg['source_run']; bundle=source/'source_bundle/REPEAT_00'
    root_manifest=read(source/'source_bundle/manifest.json')
    assert root_manifest['representative']=='REPEAT_00' and root_manifest['classification']==cfg['source_classification']
    expected={str(bundle/p):h for p,h in read(bundle/'hashes.json').items()}
    verify_hashes(expected)
    assert sha(bundle/'raw/fresh_lightnav.npy') == cfg['fresh_sha256']
    assert read(bundle/'qualification.json')['qualified']
    sealed=read(bundle/'state_and_timing.json'); ctx=sealed['context']
    raw=np.load(bundle/'raw/fresh_lightnav.npy',allow_pickle=False)
    world=np.load(bundle/'derived/fresh_world.npy',allow_pickle=False)
    A=np.array(ctx['R_obs'],float); B=np.array(ctx['B'],float)
    ep=source/'episodes/REPEAT_00'
    with (ep/'execution.csv').open() as f: rows=list(csv.DictReader(f))
    poses=np.array([[float(r[k]) for k in ['x','y','yaw']] for r in rows])
    np.testing.assert_array_equal(A,poses[ctx['obs_state_id']])
    np.testing.assert_array_equal(B,poses[ctx['switch_state_id']])
    assert int(rows[ctx['switch_state_id']]['incoming_command_id']) == ctx['switch_state_id']-1
    np.testing.assert_allclose(world,local_trajectory_to_world(A,raw),rtol=0,atol=1e-12)
    for label in ('fresh','old'):
        for key,name in [('raw_local_ref','raw_local.npy'),('world_ref','world.npy')]:
            ref=ctx[f'{label}_{key}']; assert sha(ep/ref['path'])==ref['sha256']
    assert np.array_equal(raw,np.array(read(bundle/'raw/fresh_response.json')['data']['actions']['actions'],float))
    np.testing.assert_array_equal(B,sealed['B_state']['pose_world'])
    scene=read(source/'scenario.json'); env_source=read(scene['source03_input'])
    base=HospitalEnvironment.load(env_source['environment_export'])
    projection=projection_from_metadata(env_source['projection'])
    on=compose_environment(base,projection,True); cart=projection['obstacle_geometry']
    problem=LocalSE2Problem(A,B,world)
    assert on.check_polyline(world)['clearance_valid']
    return dict(source=source,bundle=bundle,episode=ep,raw=raw,problem=problem,
                old_prefix=poses[:ctx['switch_state_id']+1],context=ctx,sealed=sealed,
                scene=scene,env_source=env_source,base=base,on=on,cart=cart)


def independent_clearance(state, loaded):
    """Full GEOS union query separate from optimizer's nearest-part checker."""
    from shapely.geometry import LineString
    from shapely.ops import nearest_points
    xy=np.asarray(state)[:,:2]; path=LineString(xy)
    base,on,cart=loaded['base'],loaded['on'],loaded['cart']
    distances=[path.distance(base.obstacles)-.2,path.distance(cart)-.2]
    limiting='Hospital' if distances[0] <= distances[1] else 'cart'
    obstacle=base.obstacles if limiting=='Hospital' else cart
    parts=[LineString(xy[j:j+2]) for j in range(len(xy)-1)]
    clearances=[p.distance(on.obstacles)-.2 for p in parts]
    j=int(np.argmin(clearances)); point, nearest=nearest_points(parts[j],obstacle)
    alpha=float(parts[j].project(point,normalized=True)) if parts[j].length else 0.
    c=float(path.distance(on.obstacles)-.2); eps=on.numerical_tolerance_m
    known=bool(on.workspace.covers(path) and path.distance(on.workspace.boundary)>=.2+eps)
    native=on.check_polyline(state)
    np.testing.assert_allclose(c,native['minimum_clearance_m'],rtol=0,atol=1e-12)
    mesh=None
    if limiting=='Hospital':
        from shapely.geometry import Point
        idx=int(base.tree.nearest(Point(nearest.x,nearest.y)))
        root=Path(loaded['env_source']['environment_export'])
        records=read(root/'geometry/projected/obstacle_parts.json')
        mesh_id=records[idx]['mesh_id']; meshes=read(root/'geometry/meshes.json')
        mesh=dict(mesh_id=mesh_id,prim_path=meshes[mesh_id]['prim_path'])
    return dict(minimum_clearance_m=c,Hospital_only_m=float(distances[0]),cart_only_m=float(distances[1]),
                limiting_geometry=limiting,limiting_Hospital_mesh=mesh,first_minimum_segment=j,
                minimum_segment_fraction=alpha,minimum_path_xy=[point.x,point.y],nearest_obstacle_xy=[nearest.x,nearest.y],
                per_segment_clearance_m=clearances,workspace_known=known,
                clearance_valid=known and c>=.05+eps,required_edge_clearance_m=.05,footprint_radius_m=.2,
                numerical_tolerance_m=eps,query='full union direct GEOS, no B connector, no time/execution claim')


def prepare(run):
    run.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load(CONFIG.read_text()); config_contract(cfg)
    save(run/'protocol.json',cfg)
    loaded=load_source(cfg)
    from validate_obstacle_source_acquisition03 import episode
    audit=episode(loaded['source'],'REPEAT_00')
    assert audit['qualified'] and audit['original_validation']['valid']
    save(run/'source_validation.json',audit)
    source=loaded['source']; files=set()
    for directory in [loaded['bundle'],loaded['episode'],Path(loaded['env_source']['environment_export'])]:
        files.update(p for p in directory.rglob('*') if p.is_file())
    files.update(source/name for name in ['scenario.json','acquisition_scene.json','config_snapshot.yaml','protocol.json','execution_start.json','workers.json','server_launch.json','source.json','freeze.json'])
    files.add(Path(loaded['scene']['source03_input'])); files.add(Path(loaded['scene']['triangles_path']))
    # Preserve the pinned input contracts already recorded in the historical source.
    preserved=read(source/'source.json')['preserved']; verify_hashes(preserved)
    hashes={str(p.resolve()):sha(p) for p in sorted(files)}
    save(run/'source_manifest.json',dict(files=hashes,historical_preservation=preserved,
         source_run=str(source),representative='REPEAT_00',bundle_hashes=read(loaded['bundle']/'hashes.json'),
         source_execution=read(source/'execution_start.json'),state_and_timing=loaded['sealed'],
         coordinate='Isaac world XY metres yaw CCW; body forward/left; local outputs use original observation A',
         waypoint_intrinsic_timing=None,source_raw_validation='REPEAT_00 only; no REPEAT_01 evaluation'))
    save(run/'freeze_inputs.json',dict(config_sha256=sha(CONFIG),code_sha256={p:sha(ROOT/p) for p in CODE},
         source_manifest_sha256=sha(run/'source_manifest.json'),protocol_sha256=sha(run/'protocol.json'),
         source_validation_sha256=sha(run/'source_validation.json'),starting_sha=git('rev-parse','HEAD'),
         prepared_utc=datetime.now(timezone.utc).isoformat(),calls=dict(LightNav=0,MPC=0,GP=0,execution=0,planning=0)))
    print(json.dumps(dict(prepared=str(run),N=len(loaded['raw']),A=loaded['problem'].A.tolist(),B=loaded['problem'].B.tolist(),source_validation=True)))


def execute(run):
    cfg=read(run/'protocol.json'); frozen=read(run/'freeze_inputs.json')
    assert sha(CONFIG)==frozen['config_sha256']
    assert sha(run/'protocol.json')==frozen['protocol_sha256']
    assert sha(run/'source_manifest.json')==frozen['source_manifest_sha256']
    verify_hashes({str(ROOT/p):h for p,h in frozen['code_sha256'].items()})
    head=git('rev-parse','HEAD')
    remote=git('ls-remote','origin','refs/heads/main').split()[0]
    assert head==remote, 'scientific solve requires pushed HEAD on origin/main'
    for p in [str(CONFIG.relative_to(ROOT)),*CODE]:
        assert git('rev-parse',f'HEAD:{p}')==git('hash-object',p),'owned implementation must be committed'
    manifest=read(run/'source_manifest.json'); verify_hashes(manifest['files']); verify_hashes(manifest['historical_preservation'])
    save(run/'execution_start.json',dict(scientific_freeze_sha=head,start_utc=datetime.now(timezone.utc).isoformat(),
         scientific_planning_calls=1,LightNav=0,MPC=0,GP=0,actual_execution=0,
         scope='exactly one R00 saved-only planning call; marker prevents retries'))
    loaded=load_source(cfg); p=loaded['problem']; solver=SolverConfig(**cfg['solver'])
    trace=[]; feasibility=[]
    def feasible(x):
        check=loaded['on'].check_polyline(x)
        feasibility.append(dict(state=x.copy(),check=check))
        return check['clearance_valid']
    def observe(event):
        trace.append({**event,'factor_costs':p.costs(event['state'])})
    t=time.perf_counter(); error=None
    try:
        result=solve_least_squares(p.fresh,p.residual_vector,solver,candidate_feasibility_fn=feasible,iteration_callback=observe)
        x=result.optimized; status=result.to_dict()
    except OptimizationError as exc:
        error=str(exc); x=p.fresh.copy() if not trace else trace[-1]['state'].copy()
        status=dict(converged=False,termination_reason='OptimizationError',error=error)
    wall=time.perf_counter()-t
    (run/'derived').mkdir()
    for name,value in [('original_world',p.fresh),('transported_target',p.target),('optimized_world',x),
                       ('optimized_original_A_local',p.original_observation_local(x)),('OLD_executed_prefix',loaded['old_prefix'])]:
        with (run/'derived'/f'{name}.npy').open('xb') as f:np.save(f,value,allow_pickle=False)
    save(run/'solver_trace.json',trace); save(run/'feasibility_checks.json',feasibility)
    save(run/'result.json',dict(scientific_freeze_sha=head,status='SAFE_PLANNING_RESULT' if error is None else 'SOLVER_LIMITATION_LAST_FEASIBLE_STATE',
         A=p.A,B=p.B,log_A_inverse_B=se2_log(relative_pose(p.A,p.B)),
         observation_B_translation_m=float(np.linalg.norm(p.B[:2]-p.A[:2])),
         observation_B_travel_m=loaded['sealed']['observation_B_travel_m'],
         N=len(p.fresh),raw_arc_m=float(p.arc[-1]),initial=p.costs(p.fresh),final=p.costs(x),solver=status,
         accepted_steps=sum(e['decision']=='accepted' for e in trace),
         rejected_unsafe_steps=sum(e['decision']=='rejected_unsafe' for e in trace),
         rejected_non_improving_steps=sum(e['decision']=='rejected_non_improving' for e in trace),
         solve_wall_s=wall,diagnostics=planning_diagnostics(p,x),
         raw_safety=independent_clearance(p.fresh,loaded),optimized_safety=independent_clearance(x,loaded),
         calls=dict(planning=1,LightNav=0,MPC=0,GP=0,actual_execution=0,rigid_baseline=0,splice=0),
         GP_support_timing=None,waypoint_intrinsic_timing=None,
         scope='PLANNING ONLY / NO MPC EXECUTION / NO RECONCILIATION PERFORMANCE CLAIM'))
    verify_hashes(manifest['files']); verify_hashes(manifest['historical_preservation'])
    save(run/'result_hashes.json',{str(q.relative_to(run)):sha(q) for q in sorted(run.rglob('*')) if q.is_file()})
    print(json.dumps(dict(run=str(run),solver=status,initial=p.costs(p.fresh),final=p.costs(x))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('mode',choices=['prepare','execute']); parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args(); {'prepare':prepare,'execute':execute}[args.mode](args.run.resolve())
