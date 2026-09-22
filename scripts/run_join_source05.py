#!/usr/bin/env python3
"""Freeze SOURCE04 bytes, then run twelve independent wording-only sessions once."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import shapely
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read, save, sha, path_geometry, mpc_audit
from reconciliation.join_source04 import affordance, motion_geometry, agreement
from reconciliation.join_source05 import (ORDER, KS, INSTRUCTIONS, GENERATION, LABEL,
    condition, verify_bank, historical_id, requests, request_parity)
from reconciliation.join_source02 import observation_anchored_world
from reconciliation.join_source02_geometry import project_prop, compose_environment
from reconciliation.gp_se2_environment import HospitalEnvironment
from join_source02_paired import wire_parity


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def source_run(run):
    return Path(read(run/'source.json')['source04_run'])


def initialize(run):
    run.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((ROOT/'configs/join_source_05_instruction_avoidance.yaml').read_text())
    if cfg['instructions'] != INSTRUCTIONS or cfg['order'] != ORDER or cfg['generation_environment'] != GENERATION:
        raise ValueError('frozen declaration mismatch')
    source = ROOT/cfg['source04_run']
    from validate_join_source04 import validate
    validation = validate(source)
    save(run/'technical_preflight/source04_revalidation.json', validation)
    if not validation['valid'] or not read(source/'validation_final.json')['valid']:
        raise ValueError('SOURCE04 authoritative/source validation failed')
    original = read(source/'freeze.json')['input_sha256']
    original.update(read(source/'technical_correction.json')['previous_artifact_sha256'])
    bank = read(source/'bank_manifest.json')
    bank_hashes = verify_bank(bank, original)
    save(run/'technical_preflight/frame_identity.json', dict(valid=True, original_sha256=bank_hashes,
        original_bank_sha256=sha(source/'bank_manifest.json'), new_render_count=0))
    baseline = yaml.safe_load((source/'config_snapshot.yaml').read_text())
    baseline.update(experiment=cfg['experiment'], source05=cfg)
    with (run/'config_snapshot.yaml').open('x') as f:
        yaml.safe_dump(baseline, f, sort_keys=False)
    preserved = {str(p.resolve()): sha(p) for p in source.rglob('*') if p.is_file()}
    preserved.update(bank_hashes)
    preserved.update(read(source/'source.json')['official_contract_sha256'])
    source03 = read(source/'source03_input.json')
    for p in Path(source03['environment_export']).rglob('*'):
        if p.is_file(): preserved[str(p.resolve())] = sha(p)
    unrelated = ['configs/stage0_jackal_controller_validation.yaml', 'configs/stage0_lightnav_single_chunk.yaml']
    preserved.update({str(ROOT/p): sha(ROOT/p) for p in unrelated})
    save(run/'source.json', dict(starting_sha=git('rev-parse', 'HEAD'), origin_main=git('rev-parse', 'origin/main'),
        source04_run=str(source.resolve()), preserved=preserved, scope=LABEL,
        historical_generation_evidence='SOURCE04 frozen environment/config and documented launch; inherited process environment was not fully archived'))
    save(run/'mpc_audit/result.json', mpc_audit(ROOT))
    if read(run/'mpc_audit/result.json')['status'] != 'MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL':
        raise ValueError('official provenance mismatch')
    save(run/'protocol.json', dict(scope=LABEL, declaration=cfg, no_online_execution=True,
        no_new_MPC_GP_reconciliation=True, I0_historical_only=True, renderer_calls=0,
        coordinates='world XY metres, Z-up CCW yaw; ego forward/left; world=T_world_observation*T_local; no B',
        time='SOURCE04 poses/frame times and delivered indices preserved; waypoint intrinsic_dt=null'))
    for cid in ORDER:
        m = condition(bank, cid)
        old = read(source/'conditions'/f'{historical_id(m["K"])}.json')
        if m['frames'] != old['frames']: raise ValueError('historical condition input mismatch')
        save(run/'conditions'/f'{cid}.json', m)
    save(run/'historical_baselines/manifest.json', {cid: dict(path=str(source/'predictions'/cid),
        evaluation_sha256=sha(source/'predictions'/cid/'evaluation.json')) for cid in ['K0_OFF', 'K0_OFF_SHAM', 'K1', 'K1_SHAM', 'K2', 'K4', 'K8']})


def freeze(run):
    paths = ['src/reconciliation/join_source05.py', 'scripts/run_join_source05.py',
             'scripts/report_join_source05.py', 'scripts/validate_join_source05.py',
             'configs/join_source_05_instruction_avoidance.yaml', 'tests/test_join_source05.py',
             'src/reconciliation/join_source05_history.py', 'scripts/lightnav/join_source05_predict.py',
             'scripts/lightnav/join_source04_predict.py', 'scripts/lightnav/join_source02_paired.py',
             'scripts/online_lightnav_worker.py', 'scripts/lightnav/robotless_online_server.py',
             'scripts/lightnav/robotless_successive_server.py', 'src/reconciliation/online_history.py',
             'src/reconciliation/join_source02.py', 'src/reconciliation/join_source02_geometry.py',
             'src/reconciliation/join_source03.py', 'src/reconciliation/join_source04.py',
             'src/reconciliation/gp_se2_environment.py', 'src/reconciliation/se2.py']
    save(run/'freeze.json', dict(preparation_sha=git('rev-parse', 'HEAD'),
        source_sha256={p: sha(ROOT/p) for p in paths},
        input_sha256={str(p): sha(p) for p in run.rglob('*') if p.is_file()},
        planned_scientific_predictions=12, planned_buffer_requests=180))


def verify(run, pushed=False):
    f = read(run/'freeze.json')
    for group in (f['source_sha256'], f['input_sha256'], read(run/'source.json')['preserved']):
        for p,h in group.items():
            if sha(ROOT/p) != h: raise ValueError('frozen file changed: '+p)
    correction=run/'technical_correction.json'
    if correction.exists():
        for p,h in read(correction)['previous_artifact_sha256'].items():
            if sha(p)!=h:raise ValueError('failed pre-request evidence changed: '+p)
    if pushed:
        if git('rev-parse', 'HEAD') != git('rev-parse', '@{upstream}'): raise ValueError('push freeze first')
        if git('status', '--porcelain', '--', *f['source_sha256']): raise ValueError('uncommitted frozen implementation')
    return f


def start(run):
    verify(run, pushed=True)
    env = os.environ.copy()
    unset = ['ACTION_TOKENIZER_BUNDLE','MAX_BATCH_SIZE','MAX_WAIT_MS','VLN_VIT_CACHE_ENTRIES']
    for k in unset: env.pop(k, None)
    env.update(GENERATION, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    save(run/'server_environment_requested.json', dict(explicit=GENERATION, unset=unset,
        source='SOURCE04 documented exact launch environment; no new decoding policy'))
    subprocess.run([sys.executable, str(ROOT/'scripts/lightnav/robotless_online_server.py'), 'start', str(run)], env=env, check=True)
    pid = read(run/'server_process.json')['process_id']
    actual = dict(x.split('=',1) for x in Path(f'/proc/{pid}/environ').read_bytes().decode().split('\0') if '=' in x)
    observed = {k:v for k,v in actual.items() if k.startswith(('VLN_', 'VLLM_')) or k in ['ASPECT_MODE', 'ACTION_TOKENIZER_BUNDLE', 'MAX_BATCH_SIZE', 'MAX_WAIT_MS', 'CUDA_VISIBLE_DEVICES']}
    argv = [x.decode() for x in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if x]
    valid = all(actual.get(k) == v for k,v in GENERATION.items())
    old = read(source_run(run)/'server_launch.json'); current = read(run/'server_launch.json')
    valid &= [(x['path'],x['sha256']) for x in current['checkpoint_files']] == [(x['path'],x['sha256']) for x in old['checkpoint_files']]
    def scientific_argv(v):
        v=list(v);i=v.index('--ready_file');v[i+1]='RUN_SPECIFIC_READY_FILE';return v
    valid &= scientific_argv(current['argv']) == scientific_argv(old['argv'])
    save(run/'generation_actual.json', dict(valid=bool(valid), process_id=pid, actual_argv=argv,
        actual_relevant_inherited_environment=observed, expected=GENERATION,
        checkpoint_and_scientific_argv_match_historical=True if valid else False,
        evidence='live /proc of recorded official server PID before any scientific session'))
    if not valid: raise ValueError('actual generation/source mismatch; scientific calls blocked')


def evaluate(run, cid):
    source = source_run(run); m = read(run/'conditions'/f'{cid}.json'); out=run/'predictions'/cid
    result=read(out/'result.json')
    if result['status'] not in ('PREDICTION_READY','MODEL_STOP'):
        return dict(condition=cid, K=m['K'], status=result['status'], reason=result.get('error'))
    context=read(source/'source03_input.json'); cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())
    base=HospitalEnvironment.load(context['environment_export'])
    triangles=np.load(source/'frame_bank/triangles.npz')['triangles']; projection=project_prop(triangles)
    on=compose_environment(base,projection,True);cart=shapely.from_wkb(bytes.fromhex(projection['metadata']['obstacle_wkb_hex']))
    path=out/'chunks/terminal';raw=np.load(path/'raw_local.npy');world=np.load(path/'world.npy');response=read(path/'response.json')['data']
    if not np.array_equal(raw,np.asarray(response['actions']['actions'],float)): raise ValueError('raw response mismatch')
    pose=m['frames'][-1]['pose_world']
    if not np.allclose(world,observation_anchored_world(raw,pose),atol=1e-12,rtol=0): raise ValueError('observation anchor mismatch')
    frames=read(out/'replayed_inputs.json')['frames']
    if not wire_parity(out,frames)['valid']:raise ValueError('wire JPEG mismatch')
    parity=request_parity(requests(out),requests(source/'predictions'/historical_id(m['K'])),m['frames'],m['instruction'])
    if not parity['valid']:raise ValueError('instruction-only payload parity failed')
    terminal=read(source/'bank_manifest.json')[-1][m['terminal_bank_state']]
    apos=affordance(response,terminal,np.load(terminal['mask_path'])['mask'],np.load(terminal['depth_path'])['distance_m'],
        on if m['K'] else base,cart,np.asarray(context['center_xy']),cfg['source']['target_prim'])
    apos['target_semantics']='instruction differs from shelf goal; visible/OPOS uninterpreted, geometry class target denotes historical shelf only'
    forward=np.array([np.cos(pose[2]),np.sin(pose[2])]);ext=(triangles.reshape(-1,3)[:,:2]-context['center_xy'])@forward
    motion=motion_geometry(raw,world,pose,np.asarray(context['target_xy']),np.asarray(context['center_xy']),forward,[ext.min(),ext.max()])
    motion['historical_shelf_progress_m']=motion.pop('target_directed_progress_m')
    g=path_geometry(world,on);off=path_geometry(world,base)
    return dict(condition=cid,instruction_id=m['instruction_id'],K=m['K'],status=result['status'],scope=LABEL,
        instruction=m['instruction'],raw_local=raw.tolist(),world=world.tolist(),raw_sha256=sha(path/'raw_local.npy'),
        world_sha256=sha(path/'world.npy'),response_sha256=sha(path/'response.json'),raw_text=response.get('raw_text'),
        stop=response.get('stop'),visible=response.get('visible'),geometry_on=g,geometry_off=off,motion=motion,apos=apos,
        hallway_endpoint_forward_m=float((world[-1,:2]-pose[:2])@forward),
        hallway_returned_path_progress_m=float((world[-1,:2]-world[0,:2])@forward),
        affordance_action_category=agreement(apos,motion,g['whole']['clearance_valid']),
        client_rtt_s=result['terminal_prediction']['client_rtt_s'],worker_wall_s=result['wall_s'],
        session_id=result['session_close']['connection_id'],request_parity=parity)


def execute(run):
    verify(run,pushed=True)
    if not read(run/'generation_actual.json')['valid']:raise ValueError('generation gate failed')
    save(run/'execution.json',dict(sha=git('rev-parse','HEAD'),scope=LABEL,start_monotonic_ns=time.monotonic_ns(),retry_count=0))
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());python=(ROOT/cfg['paths']['lightnav_checkout']).resolve()/'.venv/bin/python'
    ledger=[]
    for cid in ORDER:
        log=run/'logs'/f'{cid}.log'
        with log.open('x') as f:
            p=subprocess.run([str(python),str(ROOT/'scripts/lightnav/join_source05_predict.py'),'--run',str(run),'--condition',cid],stdout=f,stderr=subprocess.STDOUT,cwd=ROOT)
        row=dict(condition=cid,status='TECHNICAL_ERROR' if p.returncode else 'COMPLETED',returncode=p.returncode)
        if not p.returncode:save(run/'predictions'/cid/'evaluation.json',evaluate(run,cid))
        ledger.append(row);save(run/'aggregate/ledger_steps'/f'{len(ledger):02}.json',row);print(row,flush=True)
    save(run/'aggregate/ledger.json',ledger)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['init','freeze','verify','start','execute','stop'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.mode=='verify':verify(run,pushed=True)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else: {'init':initialize,'freeze':freeze,'start':start,'execute':execute}[a.mode](run)


if __name__=='__main__':main()
