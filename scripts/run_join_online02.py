#!/usr/bin/env python3
"""Prepare and freeze a fixed four-episode native LightNav approach experiment."""
import argparse,json,os,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha,mpc_audit
from reconciliation.join_source02_geometry import projection_from_metadata,compose_environment
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_online02 import ORDER,INSTRUCTION,DISTANCES,select_start

def git(*args):return subprocess.check_output(['git','-C',str(ROOT),*args],text=True).strip()

def environments(run):
    s=read(run/'scenario.json');source=read(s['source03_input'])
    base=HospitalEnvironment.load(source['environment_export'])
    projection=projection_from_metadata(source['projection'])
    return base,compose_environment(base,projection,True),projection['obstacle_geometry'],source

def initialize(run):
    run.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((ROOT/'configs/join_online_02_far_approach.yaml').read_text())
    assert cfg['order']==ORDER and cfg['instruction']==INSTRUCTION and cfg['candidate_center_distances_m']==DISTANCES
    previous=ROOT/cfg['source04_run']
    assert read(previous/'validation_final.json')['valid']
    oldcfg=yaml.safe_load((previous/'config_snapshot.yaml').read_text())
    resolved=yaml.safe_load((ROOT/'configs/robotless_online_handoffs.yaml').read_text())
    assert resolved['camera']==oldcfg['camera']
    resolved['lighting']=oldcfg['lighting'];resolved['join_online02']=cfg
    resolved['online'].update(maximum_handoff_attempts=20,maximum_active_sim_s=25.,postroll_sim_s=0.)
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(resolved,f,sort_keys=False)
    source=read(previous/'source03_input.json');triangles=np.load(previous/'frame_bank/triangles.npz')['triangles']
    f=np.asarray(source['influence']['forward_xy']);center=np.asarray(source['center_xy'])
    along=(triangles.reshape(-1,3)[:,:2]-center)@f
    left=np.array([-f[1],f[0]]);lateral=(triangles.reshape(-1,3)[:,:2]-center)@left
    save(run/'scenario.json',dict(source03_input=str(previous/'source03_input.json'),source04_run=str(previous),
        center_xy=center.tolist(),forward_xy=f.tolist(),cart_extents=[float(along.min()),float(along.max())],
        cart_lateral_extents=[float(lateral.min()),float(lateral.max())],triangles_path=str(previous/'frame_bank/triangles.npz'),
        prop=source['prop'],fixed_since_before_episode=True))
    base,on,cart,_=environments(run)
    start=select_start(base,on,center,f,cart);save(run/'start_selection.json',start)
    passages=[]
    for sign in [-1,1]:
        side=float(lateral.min()-.26 if sign<0 else lateral.max()+.26)
        points=[center+f*(along.min()-.3)+left*side,center+f*(along.max()+.3)+left*side]
        passages.append(dict(side=sign,poses=np.asarray(points).tolist(),check=on.check_polyline(points)))
    save(run/'side_passages.json',passages)
    preservation={str(p):sha(p) for p in [previous/'validation_final.json',previous/'source03_input.json',
        previous/'config_snapshot.yaml',previous/'frame_bank/triangles.npz',ROOT/'configs/stage0_jackal_controller_validation.yaml',
        ROOT/'configs/stage0_lightnav_single_chunk.yaml']}
    preservation.update(read(previous/'source.json')['official_contract_sha256'])
    preservation.update({str(p):sha(p) for p in Path(source['environment_export']).rglob('*') if p.is_file()})
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),
        preserved=preservation,source04_run=str(previous),new_source_experiment=True))
    audit=mpc_audit(ROOT);save(run/'mpc_audit.json',audit)
    assert audit['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    save(run/'protocol.json',dict(declaration=cfg,coordinate='world metres Z-up CCW; local forward/left; observation anchored',
        waypoint_intrinsic_dt=None,initial_motion=[0.,0.],initial_MPC_memory='native new-episode reset; never reset at chunk installation',
        guard='new solve: next nominal .1s held interval; every subsequent held 60Hz step checked again; abort only',
        no_model_before_start_selection=True,no_reconciliation=True,
        safe_shorten='safe/no meaningful side movement/end before front and (reaches front-.5m influence or raw arc<=.5m); far normal straight chunks separate',
        side_response='delta lateral>=.2m OR reliable .1m-chord tangent>=20deg with actual lateral change>=.02m; validated side passage',
        bypass='whole raw polyline safe and onset plus endpoint beyond rear+.25m; not a traversal claim',
        natural_STOP='no further prediction/command application; response retained, B null',
        episode_outcome_priority=['full bypass','onset','natural STOP','guard abort','technical','repeated shorten at limit','limit'],
        technical_smoke='render/geometry/collector hook unit checks only; no model predictions or MPC solves'))
    if start['selected']:
        save(run/'episode_schedule.json',dict(episodes=[dict(episode_id=e,R0=start['selected']['pose_world'],
            instruction=INSTRUCTION,cart_present=e.startswith('ON_')) for e in ORDER]))
    else:save(run/'blocked.json',dict(status='GEOMETRY_START_BLOCKER',new_model_calls=0))
    (run/'logs').mkdir()
    print(json.dumps(start))

def freeze(run):
    assert read(run/'technical_preflight/result.json')['valid']
    files=[*ROOT.glob('scripts/*join_online02*.py'),*ROOT.glob('scripts/isaac/*join_online02*.py'),
        ROOT/'src/reconciliation/join_online02.py',ROOT/'tests/test_join_online02.py',ROOT/'configs/join_online_02_far_approach.yaml',
        ROOT/'scripts/isaac/robotless_online_handoffs.py',ROOT/'scripts/online_lightnav_worker.py',ROOT/'scripts/online_mpc_worker.py',
        ROOT/'src/reconciliation/online_mpc_adapter.py',ROOT/'src/reconciliation/online_history.py',
        ROOT/'src/reconciliation/robotless_online.py',ROOT/'src/reconciliation/se2.py',
        ROOT/'src/reconciliation/join_source02_geometry.py',ROOT/'src/reconciliation/gp_se2_environment.py',
        ROOT/'scripts/isaac/join_source02_preflight.py',ROOT/'scripts/isaac/join_source03_render.py']
    save(run/'freeze.json',dict(preparation_sha=git('rev-parse','HEAD'),source_sha256={str(p.relative_to(ROOT)):sha(p) for p in files},
        input_sha256={str(p):sha(p) for p in run.rglob('*') if p.is_file()},scientific_episodes=ORDER))

def verify(run,pushed=False):
    f=read(run/'freeze.json')
    for group in [f['source_sha256'],f['input_sha256'],read(run/'source.json')['preserved']]:
        for p,h in group.items():
            if sha(ROOT/p)!=h:raise ValueError('frozen source/input changed: '+p)
    if pushed:
        assert git('rev-parse','HEAD')==git('rev-parse','@{upstream}'),'push pre-primary freeze first'
        assert not git('status','--porcelain','--',*f['source_sha256']),'dirty scientific code'
    return True

def start(run):
    verify(run,True)
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());settings=cfg['join_online02']['generation_environment']
    env=os.environ.copy()
    for k in ['ACTION_TOKENIZER_BUNDLE','MAX_BATCH_SIZE','MAX_WAIT_MS','VLN_VIT_CACHE_ENTRIES']:env.pop(k,None)
    env.update(settings,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'start',str(run)],env=env,check=True)
    pid=read(run/'server_process.json')['process_id']
    actual=dict(x.split('=',1) for x in Path(f'/proc/{pid}/environ').read_bytes().decode().split('\0') if '=' in x)
    argv=[x.decode() for x in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if x]
    old=read(Path(read(run/'source.json')['source04_run'])/'server_launch.json');new=read(run/'server_launch.json')
    same=[(x['path'],x['sha256']) for x in old['checkpoint_files']]==[(x['path'],x['sha256']) for x in new['checkpoint_files']]
    def normalized(argv):
        argv=list(argv);argv[argv.index('--ready_file')+1]='RUN_READY_FILE';return argv
    same_argv=normalized(old['argv'])==normalized(new['argv'])
    valid=same and same_argv and new['lightnav_git_sha']=='c6f40e3220edbf7011e4f17eaf2c865416737d4d' and all(actual.get(k)==v for k,v in settings.items())
    save(run/'generation_actual.json',dict(valid=valid,argv=argv,
        environment={k:v for k,v in actual.items() if k.startswith(('VLN_','VLLM_')) or k in ['CUDA_VISIBLE_DEVICES','ASPECT_MODE']},
        checkpoint_identity=bool(same),argv_identity_except_ready_path=same_argv,official_source=new.get('lightnav_git_sha'),pid=pid))
    assert valid,'actual generation mismatch'

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['init','freeze','verify','start','stop'],required=True);a=p.parse_args();run=a.run.resolve()
    if a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    elif a.mode=='verify':verify(run,True)
    else:{'init':initialize,'freeze':freeze,'start':start}[a.mode](run)
if __name__=='__main__':main()
