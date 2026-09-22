#!/usr/bin/env python3
"""Initialize/freeze/execute exactly the declared persistence sessions, no motion."""
import argparse
from copy import deepcopy
import hashlib
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml
from shapely.geometry import Point

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha,mpc_audit,path_geometry
from reconciliation.join_source04 import ORDER,KS,LABEL,construct_frames,premodel,affordance,motion_geometry,agreement
from reconciliation.join_source02 import observation_anchored_world,response_geometry
from reconciliation.join_source02_geometry import project_prop,compose_environment
from reconciliation.gp_se2_environment import HospitalEnvironment
from join_source02_paired import wire_parity


def git(*a):return subprocess.check_output(['git','-C',str(ROOT),*a],text=True).strip()


def initialize(run):
    run.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((ROOT/'configs/join_source_04_persistence_affordance.yaml').read_text())
    source=ROOT/cfg['source03_run'];m=read(source/'input_manifests'/f"{cfg['source03_condition']}.json")
    for name in ('validation_v2.json','validation_review.json'):
        if not read(source/name)['valid']:raise ValueError('historical authoritative validation is not valid')
    preserved=read(source/'initial_source.json')['preserved']
    for p,h in preserved.items():
        if sha(p)!=h:raise ValueError('historical input/core changed: '+p)
    preserved.update({str(p.resolve()):sha(p) for p in source.rglob('*') if p.is_file()})
    audit=mpc_audit(ROOT);save(run/'mpc_audit.json',audit)
    if audit['status']!='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL':raise ValueError('official MPC provenance mismatch')
    config=yaml.safe_load((source/'config_snapshot.yaml').read_text());config['source04']=cfg;config['experiment']=cfg['experiment']
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(config,f,sort_keys=False)
    save(run/'source03_input.json',m)
    save(run/'pose_lineage.json',[*m['history'],m['final_frames']['A']])
    external=ROOT.parent/'external/LightNav-0-official-demo'
    paths=['README.md','docs/PROTOCOL.md','src/lightnav/slowfast.py','src/lightnav/serving/protocol.py',
        'src/lightnav/serving/ws_server.py','src/lightnav/vln_utils.py','src/lightnav/traj_vocab.py']
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),
        status=git('status','--porcelain'),source03_input_sha256=sha(source/'input_manifests'/f"{cfg['source03_condition']}.json"),
        preserved=preserved,official_contract_sha256={str(external/p):sha(external/p) for p in paths},
        source03_checkpoint_manifest=str(source/'server_launch.json'),scope=LABEL))
    save(run/'protocol.json',dict(experiment=cfg['experiment'],declaration=cfg,scope=LABEL,
        model_time='same session indices 0..15 and official video_fps4; not new capture wall-time or waypoint timestamps',
        no_online_execution=True,no_new_MPC_GP_reconciliation=True))


def prepare(run):
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());bank=read(run/'frame_bank/bank.json')
    m=read(run/'source03_input.json');projection=project_prop(np.load(run/'frame_bank/triangles.npz')['triangles'])
    assert projection['metadata']==m['projection'],'source cart geometry drift'
    env=HospitalEnvironment.load(m['environment_export']);on=compose_environment(env,projection,True)
    cart=projection['obstacles'] if 'obstacles' in projection else None
    if cart is None:
        import shapely
        cart=shapely.from_wkb(bytes.fromhex(projection['metadata']['obstacle_wkb_hex']))
    for row in bank:
        for name in ('OFF','ON'):
            r=row[name];pose=np.asarray(r['frame']['pose_world'])
            r['cart_edge_clearance_m']=float(cart.distance(Point(pose[:2]))-.20)
            r['original_environment_query']=on.query(pose[:2]) if name=='ON' else env.query(pose[:2])
    save(run/'bank_manifest.json',bank)
    gates={}
    for cid in ORDER:
        k=KS[cid];frames=construct_frames(bank,k);gate=premodel(bank,k)
        save(run/'conditions'/f'{cid}.json',dict(condition=cid,K=k,frames=frames,instruction=cfg['source04']['instruction'],
            premodel=gate,scope=LABEL,H=16,terminal_bank_state='OFF' if k==0 else 'ON'))
        gates[cid]=gate
    patterns=['src/reconciliation/join_source0*.py','src/reconciliation/online_history.py',
        'src/reconciliation/online_mpc_adapter.py','src/reconciliation/gp_se2_environment.py',
        'scripts/*join_source04*.py','scripts/isaac/*join_source04*.py','scripts/lightnav/*join_source04*.py',
        'scripts/isaac/join_source02_preflight.py','scripts/isaac/join_source03_render.py','scripts/lightnav/join_source02_paired.py',
        'scripts/online_lightnav_worker.py','scripts/lightnav/robotless_online_server.py',
        'configs/join_source_04_persistence_affordance.yaml','tests/test_join_source04*.py']
    sources={str(p.relative_to(ROOT)):sha(p) for pat in patterns for p in ROOT.glob(pat)}
    inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file() and 'logs' not in p.parts}
    save(run/'freeze.json',dict(preparation_sha=git('rev-parse','HEAD'),source_sha256=sources,input_sha256=inputs,
        premodel_gates=gates,planned_maximum_predictions=7,available_predictions=sum(g['valid'] for g in gates.values())))
    print({k:v['status'] for k,v in gates.items()})


def verify(run,pushed=False):
    f=read(run/'freeze.json')
    for group in ('source_sha256','input_sha256'):
        for p,h in f[group].items():
            if sha(p)!=h:raise ValueError('frozen file changed: '+p)
    if pushed:
        if git('rev-parse','HEAD')!=git('rev-parse','@{upstream}'):raise ValueError('push before predictions')
        if git('status','--porcelain','--',*f['source_sha256']):raise ValueError('uncommitted frozen source')
    return f


def evaluate(run,cid):
    m=read(run/'conditions'/f'{cid}.json');out=run/'predictions'/cid;r=read(out/'result.json')
    if r['status'] not in ('PREDICTION_READY','MODEL_STOP'):
        return dict(condition=cid,K=m['K'],status=r['status'],reason=r.get('error'),scope=LABEL)
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());source=read(run/'source03_input.json')
    base=HospitalEnvironment.load(source['environment_export']);projection=project_prop(np.load(run/'frame_bank/triangles.npz')['triangles'])
    on=compose_environment(base,projection,True)
    import shapely
    cart=shapely.from_wkb(bytes.fromhex(projection['metadata']['obstacle_wkb_hex']))
    path=out/'chunks/terminal';raw=np.load(path/'raw_local.npy');world=np.load(path/'world.npy')
    response=read(path/'response.json')['data'];frames=read(out/'replayed_inputs.json')['frames']
    if not wire_parity(out,frames)['valid']:raise ValueError('wire/input mismatch')
    assert np.array_equal(raw,np.asarray(response['actions']['actions'],float)),'raw array mismatch'
    pose=m['frames'][-1]['pose_world'];expected=observation_anchored_world(raw,pose)
    assert np.allclose(world,expected,atol=1e-12,rtol=0),'world anchoring mismatch'
    terminal=read(run/'bank_manifest.json')[-1][m['terminal_bank_state']]
    mask=np.load(terminal['mask_path'])['mask'];depth=np.load(terminal['depth_path'])['distance_m']
    apos=affordance(response,terminal,mask,depth,on if m['K'] else base,cart,np.asarray(source['center_xy']),cfg['source']['target_prim'])
    forward=np.array([np.cos(pose[2]),np.sin(pose[2])]);vertices=np.load(run/'frame_bank/triangles.npz')['triangles'].reshape(-1,3)
    extent=(vertices[:,:2]-source['center_xy'])@forward
    motion=motion_geometry(raw,world,pose,np.asarray(source['target_xy']),np.asarray(source['center_xy']),forward,[extent.min(),extent.max()])
    g=path_geometry(world,on);off=path_geometry(world,base)
    reference=run/'predictions/K0_OFF/chunks/terminal/world.npy'
    change=None if not reference.exists() else response_geometry(np.load(reference),world,source['influence'])
    return dict(condition=cid,K=m['K'],status=r['status'],scope=LABEL,raw_local=raw.tolist(),world=world.tolist(),
        raw_sha256=sha(path/'raw_local.npy'),world_sha256=sha(path/'world.npy'),response_sha256=sha(path/'response.json'),
        raw_text=response.get('raw_text'),stop=response.get('stop'),target_visible=response.get('visible'),
        geometry_on=g,geometry_off=off,motion=motion,apos=apos,versus_K0=change,
        affordance_action_category=agreement(apos,motion,g['whole']['clearance_valid']),
        client_rtt_s=r['terminal_prediction']['client_rtt_s'],worker_wall_s=r['wall_s'],session_id=r['session_close']['connection_id'],
        input_sha256=sha(run/'conditions'/f'{cid}.json'))


def execute(run):
    verify(run,pushed=True);save(run/'execution.json',dict(sha=git('rev-parse','HEAD'),host_monotonic_ns=time.monotonic_ns(),scope=LABEL))
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());python=(ROOT/cfg['paths']['lightnav_checkout']).resolve()/'.venv/bin/python'
    ledger=[]
    for cid in ORDER:
        m=read(run/'conditions'/f'{cid}.json')
        if not m['premodel']['valid']:row=dict(condition=cid,K=m['K'],status='PREMODEL_K_UNAVAILABLE',calls=0)
        else:
            log=run/'logs'/f'{cid}.log';log.parent.mkdir(exist_ok=True)
            with log.open('x') as f:
                p=subprocess.run([str(python),str(ROOT/'scripts/lightnav/join_source04_predict.py'),'--run',str(run),'--condition',cid],stdout=f,stderr=subprocess.STDOUT,cwd=ROOT)
            if p.returncode:row=dict(condition=cid,K=m['K'],status='TECHNICAL_ERROR',returncode=p.returncode,calls=None)
            else:
                result=evaluate(run,cid);save(run/'predictions'/cid/'evaluation.json',result)
                row=dict(condition=cid,K=m['K'],status='COMPLETED',calls=1)
        ledger.append(row);save(run/'aggregate/ledger_steps'/f'{len(ledger):02}.json',row);print(row,flush=True)
    save(run/'aggregate/ledger.json',ledger)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['init','prepare','verify','execute'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.mode=='init':initialize(run)
    elif a.mode=='prepare':prepare(run)
    elif a.mode=='verify':verify(run,pushed=True)
    else:execute(run)


if __name__=='__main__':main()
