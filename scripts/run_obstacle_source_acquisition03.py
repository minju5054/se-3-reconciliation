#!/usr/bin/env python3
"""Prepare and freeze exact POSE11 OLD; reuse OSA02 source and pacing verbatim."""
import argparse
from pathlib import Path
import subprocess
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.obstacle_source03 import target_pose,REPETITIONS
from run_join_source05 import verify,start,git
from validate_obstacle_source02_report import validate
CONFIG=ROOT/'configs/obstacle_source_acquisition_03.yaml'


def prepare(run):
    declaration=yaml.safe_load(CONFIG.read_text());source=ROOT/declaration['source_run']
    validated=validate(source)
    assert validated['valid'] and read(source/'phase0/qualification.json')['qualified']
    run.mkdir(parents=True,exist_ok=False);(run/'logs').mkdir()
    save(run/'technical_preflight/OSA02_saved_revalidation.json',validated)
    selected=read(source/'selected_pose11.json');pose=target_pose(selected)
    save(run/'selected_pose11.json',selected)
    for name in ('scenario.json','side_passages.json'):
        with (run/name).open('xb') as f:f.write((source/name).read_bytes())
    cfg=yaml.safe_load((source/'phaseB/config_snapshot.yaml').read_text())
    cfg['online']['minimum_active_before_prediction_sim_s']=declaration['minimum_active_before_prediction_sim_s']
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
    old=read(source/'phaseB/protocol.json')
    protocol=dict(candidate='POSE11',initial_pose_world=pose,target_OLD_observation_pose=pose,
        initial_physical_command=[0.,0.],instruction=old['instruction'],side=old['side'],prior_side_caveat=old['prior_side_caveat'],
        gates=old['gates'],schedule=list(REPETITIONS),declaration=declaration,
        target_OLD='chunk_000 is genuine prediction at exact stationary initial pose; legacy bootstrap.json records actual OLD first application',
        reveal_rule=declaration['reveal'],first_post_reveal_frame_only=True,
        pre_reveal_queue_buffer_only=True,physical_command_and_memory_separate=True,
        no_PhaseA=True,no_optimizer=True,maximum_post_reveal_predictions_per_episode=1,
        initial_pose_source=dict(path=str(source/'selected_pose11.json'),sha256=sha(source/'selected_pose11.json')),
        request_eligibility_change=dict(original_s=.5,current_s=0.,reason=declaration['request_eligibility_change']),
        previous_pacing_qualification_sha256=sha(source/'phase0/validation.json'),
        representative_rule='first qualifying REPEAT_00 then REPEAT_01; run both regardless')
    save(run/'protocol.json',protocol)
    save(run/'episode_schedule.json',dict(episodes=[dict(episode_id=e,R0=pose,instruction=old['instruction'],cart_present_initial=False,cart_present_dynamic=True) for e in REPETITIONS]))
    preserved={**read(source/'source.json')['preserved'],**{str(q.resolve()):sha(q) for q in source.rglob('*') if q.is_file()}}
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),OSA02=str(source.resolve()),
        source04_run=read(source/'source.json')['source04_run'],preserved=preserved,no_historical_changes=True))
    print(run)


def freeze(run):
    source=Path(read(run/'source.json')['OSA02'])
    inherited=read(source/'phaseB/freeze.json')['source_sha256']
    for p,h in inherited.items():assert sha(ROOT/p)==h,p
    files=['configs/obstacle_source_acquisition_03.yaml','src/reconciliation/obstacle_source03.py',
           'scripts/run_obstacle_source_acquisition03.py','scripts/isaac/obstacle_source03_online.py',
           'scripts/validate_obstacle_source_acquisition03.py','tests/test_obstacle_source03.py']
    save(run/'freeze.json',dict(source_sha256={**inherited,**{p:sha(ROOT/p) for p in files}},
        input_sha256={str(p.resolve()):sha(p) for p in run.rglob('*') if p.is_file()},
        repetitions=list(REPETITIONS),maximum_new_terminal_predictions=4,phaseA_calls=0,technical_qualification_calls=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','verify','start','stop'],required=True);a=p.parse_args();run=a.run.resolve()
    if a.mode=='verify':verify(run,True)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else:dict(prepare=prepare,freeze=freeze,start=start)[a.mode](run)
