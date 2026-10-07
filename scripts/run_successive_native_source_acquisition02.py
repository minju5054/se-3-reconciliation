#!/usr/bin/env python3
"""Frozen three-pose development acquisition; one attempt each, stop first qualified."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import stamp
from reconciliation.successive_native_source_acquisition02 import next_candidate
from run_continuous_obstacle_reveal_episode01 import verify as verify01,start as start01
from run_continuous_obstacle_reveal_episode01b import historical_launch,launch_argv,tree_hashes,loader_checks,reserve_launch,observe_process
from run_join_source05 import git
CONFIG=ROOT/'configs/successive_native_source_acquisition_02.yaml'
OUT=ROOT/'results/successive_native_source_acquisition_02'
COPIED=('selected_pose11.json','scenario.json','side_passages.json','config_snapshot.yaml','episode_schedule.json','external_authority.json','speed_intervention.json')

def declaration(): return yaml.safe_load(CONFIG.read_text())
def namespace(run):
    run=Path(run).resolve()
    if run.parent.parent!=ROOT/'data/successive_native_source_acquisition_02' or run.name not in ('CANDIDATE_01','CANDIDATE_02','CANDIDATE_03'): raise ValueError('candidate namespace')
    return run

def candidates(old):
    from continuous_obstacle_reveal_exploratory_geometry02 import environments
    base,on,cart,_=environments(old)
    p=read(old/'protocol.json');T0=np.asarray(p['initial_pose_world'])
    camera=read(old/'acquisition_scene.json')['camera']
    agent=np.asarray(camera['T_world_agent']);relative=np.asarray(camera['T_agent_camera'])
    out=[]
    for i,d in enumerate(declaration()['backward_offsets_m'],1):
        pose=T0.copy();pose[:2]-=d*np.array([np.cos(T0[2]),np.sin(T0[2])])
        T=agent.copy();T[:2,3]=pose[:2];cam=T@relative
        initial=base.check_polyline([pose]);corridor=on.check_polyline([pose,T0])
        camera_check=base.check_polyline([cam[:2,3]])
        valid=bool(initial['clearance_valid'] and corridor['clearance_valid'] and camera_check['clearance_valid']
            and np.isfinite(cam).all() and np.isclose(np.linalg.det(cam[:3,:3]),1))
        out.append(dict(candidate_id=f'CANDIDATE_{i:02}',execution_order=i,backward_offset_m=d,
            pose_world=pose.tolist(),original_POSE11=T0.tolist(),initial_clearance=initial,
            corridor_to_POSE11=corridor,camera_XY_footprint_check=camera_check,
            camera_placement_scope='Inherited calibrated camera height/basis; finite proper transform and conservative XY footprint/workspace check. Actual USD camera/agent matrices authenticated before model requests.',
            T_world_agent=T.tolist(),T_world_camera=cam.tolist(),T_agent_camera=relative.tolist(),
            cart_transform=read(old/'scenario.json')['prop']['wrapper_matrix_column'],geometry_valid=valid,
            transform='T_candidate=T0*Trans(-d,0,0); world XY metres +Z up CCW radians; local +x forward +y left',
            additional_scene_launches=0,model_calls=0))
    return out

def authenticate():
    cfg=declaration();old=ROOT/cfg['historical_run'];out=ROOT/cfg['historical_results']
    from validate_long_continuous_obstacle_reveal_source01 import validate,validate_bundle
    from report_long_continuous_obstacle_reveal_source01 import validate_report
    from validate_robotless_online_handoffs import equal_record
    equal_record(validate(old),read(old/'attempt_validation.json'))
    assert validate_bundle(old,read(old/'validation.json'))['valid'] and validate_report(old,out)['valid']
    from audit_successive_scheduler02 import historical_revalidation
    assert historical_revalidation()==read(OUT/'historical_long01_corrected_audit.json')
    for p in out.rglob('*'):
        if p.is_file(): assert p.read_bytes()==subprocess.check_output(['git','show',cfg['historical_commit']+':'+str(p.relative_to(ROOT))],cwd=ROOT)
    return {**tree_hashes(old),**tree_hashes(out)}

def equivalence(run):
    cfg=declaration();old=ROOT/cfg['historical_run'];c=read(run/'candidate.json')
    assert c==next(x for x in read(OUT/'candidate_manifest.json') if x['candidate_id']==run.name)
    a=read(old/'protocol.json');b=read(run/'protocol.json')
    assert b['initial_pose_world']==c['pose_world'] and b['candidate']==c['candidate_id']
    for k in ('initial_pose_world','candidate','experiment'): b[k]=a[k]
    assert b['declaration']=={**a['declaration'],**{k:cfg[k] for k in ('maximum_terminal_predictions','maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s','classification_priority')},'experiment':cfg['experiment']}
    b['declaration']=a['declaration'];assert a==b
    x=yaml.safe_load((old/'config_snapshot.yaml').read_text());y=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    for k in ('maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s'):
        assert y['online'][k]==cfg[k];y['online'][k]=x['online'][k]
    assert x==y
    a=read(old/'episode_schedule.json');b=read(run/'episode_schedule.json')
    assert b['episodes'][0]['R0']==c['pose_world'];b['episodes'][0]['R0']=a['episodes'][0]['R0'];assert a==b
    for n in COPIED:
        if n not in ('config_snapshot.yaml','episode_schedule.json'): assert (old/n).read_bytes()==(run/n).read_bytes()
    return dict(valid=True,candidate=c['candidate_id'],pose=c['pose_world'],
        only_runtime_deltas=['initial world pose','maximum terminal requests 12','active cap 10 s'],
        previous_half_speed_and_external_MPC_unchanged=True,immediate_reaction_and_direction_not_qualification_gates=True)

def prepare(batch):
    cfg=declaration();old=ROOT/cfg['historical_run'];preserved=authenticate();historical=historical_launch(cfg)
    manifest=candidates(old);save(OUT/'candidate_manifest.json',manifest)
    batch.mkdir(parents=True,exist_ok=False)
    for c in manifest:
        run=batch/c['candidate_id'];run.mkdir();(run/'logs').mkdir();(run/'technical_preflight').mkdir()
        for n in COPIED: (run/n).write_bytes((old/n).read_bytes())
        runtime=yaml.safe_load((run/'config_snapshot.yaml').read_text())
        for k in ('maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s'):runtime['online'][k]=cfg[k]
        (run/'config_snapshot.yaml').write_text(yaml.safe_dump(runtime,sort_keys=False))
        p=read(old/'protocol.json');p.update(experiment=cfg['experiment'],candidate=c['candidate_id'],initial_pose_world=c['pose_world'])
        p['declaration'].update({k:cfg[k] for k in ('experiment','maximum_terminal_predictions','maximum_handoff_attempts','maximum_active_sim_s','postroll_sim_s','classification_priority')})
        save(run/'protocol.json',p);save(run/'candidate.json',c)
        schedule=read(old/'episode_schedule.json');schedule['episodes'][0]['R0']=c['pose_world']
        (run/'episode_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
        prior=read(old/'source.json');save(run/'source.json',{**prior,'starting_sha':cfg['historical_commit'],
            'audit_fix_sha':git('rev-parse','HEAD'),'origin_main':git('rev-parse','origin/main'),'preserved':{**prior['preserved'],**preserved}})
        save(run/'protocol_equivalence.json',equivalence(run))
        save(run/'launch_environment.json',dict(**historical,argv=launch_argv(run,cfg),authorized_launches=1,retries=0,full_preflight_startups=0,
            external_sha256={p:sha(p) for p in (cfg['isaac_python'],str(Path(cfg['isaac_python']).parent/'setup_python_env.sh'))}))
        assert loader_checks(run,cfg)['valid']
    save(OUT/'protocol_inputs.json',dict(batch=str(batch),starting_sha=cfg['historical_commit'],audit_fix_sha=git('rev-parse','HEAD'),
        candidate_order=[c['candidate_id'] for c in manifest],maximum_live_episodes=3,maximum_Isaac_launches=3,
        preserved_historical_files=len(preserved),scientific_calls=0))

def freeze(batch):
    inherited=read(ROOT/declaration()['historical_run']/'freeze.json')['source_sha256']
    for p,h in inherited.items():assert sha(ROOT/p)==h,p
    files=set(inherited)
    for pat in ('scripts/*successive*02.py','scripts/isaac/*successive*02.py','src/reconciliation/*successive*02.py','tests/test_successive*02.py'):
        files.update(str(p.relative_to(ROOT)) for p in ROOT.glob(pat))
    files.update(['configs/successive_native_source_acquisition_02.yaml','results/successive_native_source_acquisition_02/candidate_manifest.json','docs/SUCCESSIVE_NATIVE_SOURCE_ACQUISITION_02_PROTOCOL.md'])
    manifests={}
    for run in sorted(batch.glob('CANDIDATE_*')):
        save(run/'freeze.json',dict(source_sha256={p:sha(ROOT/p) for p in sorted(files)},input_sha256=tree_hashes(run),
            maximum_terminal_predictions=12,episodes=['EPISODE_00'],retries=0,maximum_SimulationApp_starts=1))
        verify(run);manifests[run.name]=sha(run/'freeze.json')
    save(OUT/'freeze_manifest.json',dict(candidates=manifests,source_files=len(files),scientific_calls_before_freeze=0))

def verify(run,pushed=False):
    result=verify01(run,pushed)
    assert equivalence(run)==read(run/'protocol_equivalence.json')
    for p,h in read(run/'launch_environment.json')['external_sha256'].items():assert sha(p)==h
    loader=read(run/'loader_verification.json');assert loader['valid'] and sha(declaration()['libcusparse'])==loader['libcusparse_sha256']
    for c in loader['cases']:assert sha(c['selected_path'])==c['library_sha256'] and sha(c['output'])==c['output_sha256']
    return result

def search_gate(run):
    completed={}
    for p in sorted(run.parent.glob('CANDIDATE_*/attempt_validation.json')):
        seal=read(p.parent/'candidate_seal.json')
        for path,h in seal['files'].items():assert sha(path)==h,'failed/finished candidate changed: '+path
        completed[p.parent.name]=read(p)
    assert next_candidate(read(OUT/'candidate_manifest.json'),completed)==run.name,'order/budget/first qualified source gate'
    assert read(run/'candidate.json')['geometry_valid']
    assert not (run/'launch_attempt.json').exists(),'no retry'

def launch(run):
    verify(run, True); search_gate(run); cfg=declaration()
    assert read(run/'generation_actual.json')['valid']
    argv = launch_argv(run,cfg); assert argv == read(run/'launch_environment.json')['argv']
    reserve_launch(run,argv)
    samples=[]; observed=set(); observer_errors=[]
    with (run/'logs/isaac.log').open('x') as log:
        child = subprocess.Popen(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        while child.poll() is None:
            if (run/'execution_start.json').exists():
                try:
                    pid=read(run/'execution_start.json')['pid']
                    sample=observe_process(pid)
                    # Persist initial environment and each new selected-library set.
                    key=tuple(sample['libraries'])
                    if key not in observed:
                        observed.add(key); samples.append(sample)
                        save(run/'process_observations'/f'{len(samples):03}.json',sample)
                except (OSError, json.JSONDecodeError) as exc:
                    record = dict(error=repr(exc), at=stamp())
                    if not observer_errors or observer_errors[-1]['error'] != record['error']:
                        observer_errors.append(record)
                        save(run/'observer_errors'/f'{len(observer_errors):03}.json',record)
            time.sleep(.1)
    save(run/'launch_result.json', dict(returncode=child.returncode, at=stamp(), observer_errors=observer_errors,
        full_launch_attempts=1, episode_directory_exists=(run/'episodes').exists(),
        scene_initialized=(run/'acquisition_scene.json').exists(),
        schedule_completed=(run/'schedule_completion.json').exists(),
        actual_library_sha256={p:sha(p) for s in samples for p in s['libraries']},
        no_retry=True, result_is_not_inferred_from_exit_code=True))
    print(read(run/'launch_result.json'),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','verify','start','launch','stop'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.mode in ('prepare','freeze'):
        assert run.parent==ROOT/'data/successive_native_source_acquisition_02'
        globals()[a.mode](run);return
    run=namespace(run)
    if a.mode=='verify':verify(run,True)
    elif a.mode=='start':verify(run,True);search_gate(run);start01(run)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else:launch(run)
if __name__=='__main__':main()
