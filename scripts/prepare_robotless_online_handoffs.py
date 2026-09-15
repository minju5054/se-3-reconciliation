#!/usr/bin/env python3
"""Freeze a complete deterministic 30 x 2 schedule before genuine online capture."""
import argparse
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from reconciliation.robotless_online import dump_new, file_record, stamp

SOURCES = [
 'configs/robotless_online_handoffs.yaml', 'scripts/prepare_robotless_online_handoffs.py',
 'scripts/isaac/robotless_online_handoffs.py', 'scripts/isaac/robotless_runtime.py',
 'scripts/isaac/robotless_old_consistent_observation.py', 'scripts/online_lightnav_worker.py',
 'scripts/online_mpc_worker.py', 'scripts/lightnav/robotless_online_server.py',
 'src/reconciliation/robotless_online.py', 'src/reconciliation/online_history.py',
 'src/reconciliation/online_mpc_adapter.py', 'src/reconciliation/robotless_single_chunk.py',
 'src/reconciliation/se2.py', 'scripts/launch_robotless_online_handoffs.sh',
]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--smoke', action='store_true')
    args=parser.parse_args()
    run=args.run.resolve(); run.mkdir(parents=True, exist_ok=False)
    config=yaml.safe_load((ROOT/'configs/robotless_online_handoffs.yaml').read_text())
    bankpath=ROOT/config['schedule']['source_config']
    bank=yaml.safe_load(bankpath.read_text())
    # Only starting pose, instruction and scenario labels cross from prior bank.
    episodes=[{'episode_id': f'{ep["episode_id"]}_repeat_{rep:02d}',
               'condition_id': ep['episode_id'], 'repeat':rep, 'R0':ep['R0'],
               'instruction':ep['instruction'],'category':ep['category'],
               'geometry_context':ep['geometry_context']}
              for ep in bank['episodes'] for rep in range(2)]
    assert len(episodes)==60 and len({e['episode_id'] for e in episodes})==60
    if args.smoke: episodes=episodes[:1]
    config['run_kind']='technical_smoke' if args.smoke else 'primary'
    if args.smoke:
        config['online']['maximum_handoff_attempts']=3
        config['online']['maximum_active_sim_s']=25.0
    with (run/'config_snapshot.yaml').open('x') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    dump_new(run/'episode_schedule.json', {'schema_version':1,'run_kind':config['run_kind'],
        'source':file_record(bankpath, ROOT),'reused_fields':['R0','instruction','category','geometry_context'],
        'order':'condition index ascending then repetition 00,01; no shuffle', 'episodes':episodes})
    dump_new(run/'protocol.json', {'schema_version':1,'label':'GENUINE ONLINE ROBOTLESS KINEMATIC COLLECTION',
        'bootstrap':'4 live captures at 4Hz; first three buffer-only, fourth predicts C0',
        'request_policy':config['online'], 'execution':config['execution'],
        'frames':{'world':'metres; Z up; yaw radians CCW', 'agent':'x forward,y left,z up',
                  'trajectory':'raw cumulative local poses; own capture-time agent anchor',
                  'transformation':'T_W_waypoint=T_W_agent_at_capture*T_agent_waypoint; no correction',
                  'camera':'USD camera local x right,y up,z backward; explicit agent-from-camera extrinsic'},
        'clocks':{'host_utc':'provenance only', 'host_monotonic':'RTT and wall durations',
                  'sim_time':'Isaac World.current_time after explicit PhysX step; USD timeline synchronized to that clock',
                  'episode_time':'sim_time minus episode origin; no host-domain subtraction',
                  'switch':'first FRESH-command PRE-integration state; P previous state sample'},
        'corpus':'formulation development; prior screening starting-condition bank; not held-out',
        'collision_validity':'unknown; no physical collision response or qualified geometry gate',
        'waypoint_dt_s':None, 'added_inference_delay_s':0, 'added_activation_delay_s':0})
    for name in SOURCES:
        target=run/'processing_source_snapshots'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('xb') as stream:stream.write((ROOT/name).read_bytes())
    dump_new(run/'provenance.json', {'schema_version':1,'frozen_at':stamp(),
        'starting_research_sha':'e56c0fe320309c2c4ffd8488fb3abdac42da8bfe',
        'collector_git_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'run_kind':config['run_kind'], 'source_files':[file_record(ROOT/p, ROOT) for p in SOURCES],
        'schedule':file_record(run/'episode_schedule.json',run),
        'config':file_record(run/'config_snapshot.yaml',run),
        'protocol':file_record(run/'protocol.json',run),
        'lightnav':config['lightnav']})
    print(run)

if __name__=='__main__': main()
