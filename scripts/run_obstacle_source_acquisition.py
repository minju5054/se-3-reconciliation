#!/usr/bin/env python3
"""Freeze and run one bounded source bank. Never constructs an optimizer."""
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
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha,mpc_audit
from reconciliation.join_source05 import verify_bank
from reconciliation.obstacle_source_acquisition import select_side,paired_frames,geometry,qualify,timing_gate
from run_join_online02 import git,environments
from run_join_source05 import verify,start
from join_source02_paired import wire_parity
CONFIG=ROOT/'configs/obstacle_source_acquisition_01.yaml'


def initialize(run):
    cfg=yaml.safe_load(CONFIG.read_text());source=ROOT/cfg['source04_run'];online=ROOT/cfg['online03_run']
    run.mkdir(parents=True,exist_ok=False)
    from validate_join_source04 import validate
    valid=validate(source);save(run/'technical_preflight/source04_validation.json',valid)
    assert valid['valid'] and read(source/'validation_final.json')['valid'] and read(online/'validation.json')['valid']
    bank=read(source/'bank_manifest.json');hashes=read(source/'freeze.json')['input_sha256']
    hashes.update(read(source/'technical_correction.json')['previous_artifact_sha256'])
    reused=verify_bank(bank,hashes)
    original=yaml.safe_load((online/'config_snapshot.yaml').read_text())
    sides=read(online/'side_passages.json');side=select_side(sides)
    if side is None:raise ValueError('GEOMETRY_INVALID: no valid passage')
    instruction=cfg['instructions'][side]
    original['acquisition01']=cfg
    original['join_online02']['instruction']=instruction
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(original,f,sort_keys=False)
    for name in ('scenario.json','side_passages.json'):
        with (run/name).open('xb') as f:f.write((online/name).read_bytes())
    scenario=read(run/'scenario.json');base,on,cart,_=environments(run)
    candidates=[];order=[]
    for cid,index in zip(cfg['candidate_order'],cfg['candidate_bank_indices'],strict=True):
        pair=paired_frames(bank,index,cfg['history_frames']);row=bank[index];pose=row['source_pose_world']
        straight_end=np.r_[scenario['center_xy'],pose[2]]
        pre=dict(pair_signature=row['OFF']['scene_signature']==row['ON']['scene_signature'],
            stable=row['OFF']['stable'] and row['ON']['stable'],
            same_product=row['OFF']['same_product'] and row['ON']['same_product'],
            camera=row['OFF']['camera']==row['ON']['camera'],
            cart_pixels=row['ON']['cart_pixels']>=cfg['gates']['visible_pixels'] and row['OFF']['cart_pixels']==0,
            observation_clearance=on.check_polyline([pose])['clearance_valid'])
        record=dict(candidate=cid,bank_index=index,pose_world=pose,
            cart_transform=row['ON']['cart_transform'],cart_center_xy=scenario['center_xy'],
            cart_relative_forward_m=float((np.asarray(scenario['center_xy'])-pose[:2])@np.asarray(scenario['forward_xy'])),
            cart_relative_lateral_m=float((np.asarray(scenario['center_xy'])-pose[:2])@np.array([-np.sin(pose[2]),np.cos(pose[2])])),
            straight_design_probe_off=base.check_polyline([pose,straight_end]),
            straight_design_probe_on=on.check_polyline([pose,straight_end]),
            design_probe_is_not_LightNav=True,passages=sides,premodel=dict(valid=all(pre.values()),checks=pre),
            cart_pixels=row['ON']['cart_pixels'],terminal_source=row)
        candidates.append(record)
        for branch in cfg['branches']:
            condition=cid+'_'+branch;order.append(condition)
            save(run/'conditions'/f'{condition}.json',dict(condition=condition,candidate=cid,branch=branch,
                frames=pair[branch],instruction=instruction,premodel=record['premodel'],H=cfg['history_frames']))
    save(run/'candidate_manifest.json',candidates)
    save(run/'protocol.json',dict(declaration=cfg,order=order,pass_side=side,instruction=instruction,
        scope='PHASE A: COUNTERFACTUAL MOVING-POSE HISTORY; source-only OFF/ON bank, no B or execution',
        same_history='seven byte-identical OFF buffers; eighth terminal OFF vs ON at same saved pose',
        no_rerender=True,coordinate='world Z-up m/rad CCW; local forward/left; observation anchor only',
        intrinsic_waypoint_dt=None,source_selection='all four once, then first qualifying in declared order',
        phaseB_requires='separate selected-candidate commit/push AND Phase0 pass',
        future_proxy='official weighted nearest at observation; not actual B; must recheck in PhaseB'))
    preserved={str(p.resolve()):sha(p) for p in source.rglob('*') if p.is_file()}
    preserved.update(reused);preserved.update(read(source/'source.json')['official_contract_sha256'])
    preserved.update(read(online/'source.json')['preserved'])
    for name in ('source.json','validation.json','config_snapshot.yaml','scenario.json','side_passages.json','start_selection.json'):
        p=online/name;preserved[str(p.resolve())]=sha(p)
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),
        source04_run=str(source.resolve()),online03_run=str(online.resolve()),preserved=preserved))
    audit=mpc_audit(ROOT);save(run/'mpc_audit/result.json',audit)
    assert audit['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    # Independent technical OFF episode: no cart/source qualification and no candidate selection.
    p0=run/'phase0';(p0/'logs').mkdir(parents=True)
    technical=deepcopy(original);technical['online'].update(maximum_handoff_attempts=3,maximum_active_sim_s=5.,postroll_sim_s=.10)
    with (p0/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(technical,f,sort_keys=False)
    for name in ('scenario.json','side_passages.json','start_selection.json'):
        with (p0/name).open('xb') as f:f.write((online/name).read_bytes())
    save(p0/'episode_schedule.json',dict(episodes=[dict(episode_id='PACING_OFF_00',
        R0=read(p0/'start_selection.json')['selected']['pose_world'],instruction=instruction,cart_present=False)]))
    (run/'logs').mkdir();print(json.dumps(dict(run=str(run),side=side,candidates=[{k:r[k] for k in ('candidate','pose_world','cart_relative_forward_m','premodel')} for r in candidates])))


def freeze(run):
    files=[CONFIG,Path(__file__).resolve(),ROOT/'scripts/lightnav/obstacle_source_predict.py',
        ROOT/'scripts/isaac/obstacle_source_pacing.py',ROOT/'scripts/report_obstacle_source_acquisition.py',
        ROOT/'src/reconciliation/obstacle_source_acquisition.py',ROOT/'src/reconciliation/obstacle_source_history.py',
        ROOT/'tests/test_obstacle_source_acquisition.py']
    names=['scripts/isaac/robotless_online_handoffs.py','scripts/isaac/join_online02_collect.py',
        'scripts/join_online02_geometry_worker.py','scripts/online_mpc_worker.py','scripts/online_lightnav_worker.py',
        'scripts/run_join_online02.py','scripts/run_join_source05.py','scripts/lightnav/join_source02_paired.py',
        'src/reconciliation/online_mpc_adapter.py','src/reconciliation/online_history.py','src/reconciliation/robotless_online.py',
        'src/reconciliation/join_source02.py','src/reconciliation/join_source02_acquisition.py','src/reconciliation/join_online02.py',
        'src/reconciliation/join_source02_geometry.py','src/reconciliation/gp_se2_environment.py','src/reconciliation/se2.py',
        'scripts/lightnav/robotless_online_server.py','scripts/lightnav/robotless_successive_server.py']
    files.extend(ROOT/n for n in names)
    save(run/'freeze.json',dict(preparation_sha=git('rev-parse','HEAD'),source_sha256={str(p.relative_to(ROOT)):sha(p) for p in files},
        input_sha256={str(p):sha(p) for p in run.rglob('*') if p.is_file()},planned_scientific_predictions=8,
        planned_phase0_technical_predictions=4,phaseB_maximum_episodes=2))


def execute(run):
    verify(run,True);assert read(run/'generation_actual.json')['valid']
    save(run/'phaseA_execution_start.json',dict(sha=git('rev-parse','HEAD'),at_monotonic_ns=time.monotonic_ns()))
    ledger=[];env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());checkout=(ROOT/cfg['paths']['lightnav_checkout']).resolve()
    for cid in read(run/'protocol.json')['order']:
        m=read(run/'conditions'/f'{cid}.json')
        if not m['premodel']['valid']:
            row=dict(condition=cid,status='PREMODEL_UNAVAILABLE',attempted=False)
        else:
            with (run/'logs'/f'{cid}.log').open('x') as log:
                r=subprocess.run([str(checkout/'.venv/bin/python'),str(ROOT/'scripts/lightnav/obstacle_source_predict.py'),
                    '--run',str(run),'--condition',cid],env=env,stdout=log,stderr=subprocess.STDOUT)
            row=dict(condition=cid,returncode=r.returncode,attempted=True)
        ledger.append(row);save(run/'ledger'/f'{cid}.json',row);print(row,flush=True)
    save(run/'phaseA_completion.json',dict(ledger=ledger,no_retry=True,end_monotonic_ns=time.monotonic_ns()))


def evaluate(run):
    base,on,_,_=environments(run);scenario=read(run/'scenario.json');protocol=read(run/'protocol.json')
    results={};pairs=[]
    for cid in protocol['order']:
        out=run/'phaseA'/cid;m=read(run/'conditions'/f'{cid}.json')
        if not (out/'result.json').exists():results[cid]=dict(status='UNAVAILABLE');continue
        rec=read(out/'result.json')
        if rec['status'] not in ('PREDICTION_READY','MODEL_STOP'):results[cid]=dict(status=rec['status']);continue
        path=out/'chunks/terminal';raw=np.load(path/'raw_local.npy');world=np.load(path/'world.npy');resp=read(path/'response.json')['data']
        assert np.array_equal(raw,np.asarray(resp['actions']['actions'],float))
        g=geometry(raw,m['frames'][-1]['pose_world'],base,on,scenario)
        np.testing.assert_allclose(world,g['world'],rtol=0,atol=1e-12)
        assert wire_parity(out,read(out/'replayed_inputs.json')['frames'])['valid']
        g.update(status=rec['status'],stop=bool(resp.get('stop')),raw_text=resp.get('raw_text'),pointing=resp.get('pointing'),
            raw_sha256=sha(path/'raw_local.npy'),world_sha256=sha(path/'world.npy'),response_sha256=sha(path/'response.json'),
            input_hashes=[f['sha256'] for f in m['frames']],wall_s=rec['wall_s'],client_rtt_s=rec['terminal_prediction'].get('client_rtt_s'))
        results[cid]=g
    for cid in protocol['declaration']['candidate_order']:
        a,b=results[cid+'_OFF'],results[cid+'_ON']
        if 'world' not in a or 'world' not in b:q=dict(qualified=False,failure_reasons=['TECHNICAL_UNAVAILABLE'])
        else:q=qualify(a,b,scenario,protocol['declaration']['gates'])
        pairs.append(dict(candidate=cid,**q))
    selected=next((p['candidate'] for p in pairs if p['qualified']),None)
    return dict(results=results,pairs=pairs,selected=selected,qualified_count=sum(p['qualified'] for p in pairs),
        primary_classification='LIGHTNAV_OBSTACLE_SOURCE_QUALIFICATION_FAILED' if selected is None else 'PHASE_A_QUALIFIED_PENDING_TIMING_ONLINE',
        GP_rigid_optimizer_calls=0,phaseA_MPC_calls=0,new_phaseA_RGB=0)


def pacing_result(run):
    from analyze_join_online02 import analyze_episode
    r=analyze_episode(run/'phase0','PACING_OFF_00')
    gate=timing_gate([x.get('inflight_rtf') for x in r['rows'][1:]],r['summary']['max_loop_stall_s'])
    gate['all_requests']=[{k:x.get(k) for k in ('chunk_id','client_rtt_s','inflight_rtf','inflight_max_state_gap_s','t_obs','t_apply')} for x in r['rows']]
    gate['technical_scope']='real OFF collector/history/model/MPC; no representative source or paired condition'
    save(run/'phase0/analysis.json',r);save(run/'phase0/qualification.json',gate);print(json.dumps(gate))


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['init','freeze','verify','start','stop','execute','evaluate','pacing-result'],required=True)
    a=p.parse_args();run=a.run.resolve()
    if a.mode=='verify':verify(run,True)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    elif a.mode=='evaluate':
        result=evaluate(run);save(run/'aggregate/analysis.json',result);print(json.dumps({k:v for k,v in result.items() if k!='results' and k!='pairs'}))
    elif a.mode=='pacing-result':pacing_result(run)
    else:{'init':initialize,'freeze':freeze,'start':start,'execute':execute}[a.mode](run)


if __name__=='__main__':main()
