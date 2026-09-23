#!/usr/bin/env python3
"""Separate pushed PhaseB freeze after successful technical qualification."""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.obstacle_source_online import initial_pose,REPETITIONS
from run_join_source05 import verify,git
from validate_obstacle_source_acquisition02 import validate
from validate_robotless_online_handoffs import equal_record


def prepare(parent):
    v=validate(parent);equal_record(v,read(parent/'phase0/validation.json'),'technical revalidation')
    assert v['qualification']['qualified'],'PhaseB blocked'
    source=Path(read(parent/'source.json')['OSA01']);selected=read(parent/'selected_pose11.json');assert selected['candidate']=='POSE11'
    cfg=yaml.safe_load((parent/'config_snapshot.yaml').read_text());cfg['online'].update(maximum_handoff_attempts=1,maximum_active_sim_s=4.,postroll_sim_s=.10)
    run=parent/'phaseB';run.mkdir(exist_ok=False);(run/'logs').mkdir()
    for name in ('scenario.json','side_passages.json'):
        with (run/name).open('xb') as f:f.write((parent/name).read_bytes())
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
    scene=read(run/'scenario.json');pose=initial_pose(selected['pose_world'],scene['forward_xy'])
    p=read(parent/'protocol.json')
    save(run/'protocol.json',dict(candidate='POSE11',source_only_identity=selected,
        reveal_plane_pose_world=selected['pose_world'],hallway_forward=scene['forward_xy'],initial_pose_world=pose,
        initial_backward_m=.40,instruction=p['instruction'],side=p['pass_side'],prior_side_caveat=p['prior_side_caveat'],
        gates=p['gates'],schedule=list(REPETITIONS),maximum_post_reveal_predictions_per_episode=1,
        reveal_rule='first 4Hz capture at/past plane with OLD active; simultaneous render/oracle cart activation',
        first_post_reveal_frame_only=True,pre_reveal_queue_buffer_only=True,physical_command_and_memory_separate=True,
        representative_rule='first qualifying REPEAT_00 then REPEAT_01; run both regardless',
        no_PhaseA=True,no_optimizer=True,technical_qualification_sha256=sha(parent/'phase0/validation.json')))
    save(run/'episode_schedule.json',dict(episodes=[dict(episode_id=e,R0=pose,instruction=p['instruction'],cart_present_initial=False,cart_present_dynamic=True) for e in REPETITIONS]))
    # Environment loader uses parent/scenario only; each episode starts OFF.
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),parent=str(parent),OSA01=str(source),
        preserved={**read(parent/'source.json')['preserved'],**{str(q):sha(q) for q in (parent/'phase0').rglob('*') if q.is_file()}}))
    print(run)


def freeze(parent):
    run=parent/'phaseB';paths=['scripts/run_obstacle_source02_online.py','scripts/isaac/obstacle_source02_online.py',
        'src/reconciliation/obstacle_source_online.py','scripts/validate_obstacle_source02_online.py','tests/test_obstacle_source_online.py']
    save(run/'freeze.json',dict(source_sha256={**read(parent/'freeze.json')['source_sha256'],**{p:sha(ROOT/p) for p in paths}},
        input_sha256={str(p):sha(p) for p in run.rglob('*') if p.is_file()},repetitions=list(REPETITIONS),maximum_new_terminal_predictions=4))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','verify'],required=True);a=p.parse_args();r=a.run.resolve()
    if a.mode=='verify':verify(r,True);verify(r/'phaseB',True)
    else:dict(prepare=prepare,freeze=freeze)[a.mode](r)
