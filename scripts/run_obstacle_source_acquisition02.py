#!/usr/bin/env python3
"""Exclusive OSA02 preparation / technical qualification; no Phase-A search."""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
import subprocess
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from run_join_source05 import verify,start,git
from diagnose_obstacle_source_pacing import audit
CONFIG=ROOT/'configs/obstacle_source_acquisition_02.yaml'


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());source=ROOT/cfg['source_run']
    # Run historical authoritative validator BEFORE the permitted collector change.
    from report_obstacle_source_acquisition import validate
    validation=validate(source)
    run.mkdir(parents=True,exist_ok=False)
    save(run/'diagnosis/OSA01_revalidation_before_correction.json',validation)
    diag=audit(source/'phase0/episodes/PACING_OFF_00');save(run/'diagnosis/diagnosis.json',diag)
    old=read(source/'aggregate/completion.json');assert old['selected_phaseA']==cfg['selected_candidate']=='POSE11'
    save(run/'selected_pose11.json',next(x for x in read(source/'candidate_manifest.json') if x['candidate']=='POSE11'))
    protocol=read(source/'protocol.json')
    save(run/'protocol.json',dict(declaration=cfg,instruction=protocol['instruction'],pass_side=protocol['pass_side'],gates=protocol['declaration']['gates'],
        scope='one technical OFF pacing episode; then only if qualified two genuine sudden reveals after second pushed freeze',
        no_PhaseA=True,prior_side_caveat=old['limitation_actual_side'],cadences='60/10/4Hz simulation ticks unchanged; wall throughput may decrease after blocking; no timestamp normalization'))
    preserved=dict(read(source/'source.json')['preserved'])
    preserved.update({str(p.resolve()):sha(p) for p in source.rglob('*') if p.is_file()})
    collector=ROOT/'scripts/isaac/robotless_online_handoffs.py'
    with (run/'diagnosis/collector_before.py').open('xb') as f:f.write(collector.read_bytes())
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),
        source04_run=read(source/'source.json')['source04_run'],OSA01=str(source.resolve()),preserved=preserved,
        allowed_code_change=dict(path=str(collector),before_sha256=sha(collector),historical_copy='diagnosis/collector_before.py')))
    for name in ('scenario.json','side_passages.json'):
        with (run/name).open('xb') as f:f.write((source/name).read_bytes())
    config=yaml.safe_load((source/'phase0/config_snapshot.yaml').read_text())
    config['execution']['pacing_policy']=cfg['pacing_policy'];config['online']['scheduler_diagnostics']=True
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(config,f,sort_keys=False)
    p0=run/'phase0';(p0/'logs').mkdir(parents=True)
    for name in ('config_snapshot.yaml','scenario.json','side_passages.json'):
        with (p0/name).open('xb') as f:f.write((run/name).read_bytes())
    for name in ('start_selection.json','episode_schedule.json'):
        with (p0/name).open('xb') as f:f.write((source/'phase0'/name).read_bytes())
    (run/'logs').mkdir()
    print(run)


def freeze(run):
    source=Path(read(run/'source.json')['OSA01'])
    files=list(read(source/'freeze.json')['source_sha256'])
    files += ['configs/obstacle_source_acquisition_02.yaml','src/reconciliation/online_pacing.py',
        'scripts/diagnose_obstacle_source_pacing.py','scripts/run_obstacle_source_acquisition02.py',
        'scripts/isaac/obstacle_source02_collect.py','scripts/validate_obstacle_source_acquisition02.py',
        'tests/test_obstacle_source_pacing.py']
    save(run/'freeze.json',dict(source_sha256={p:sha(ROOT/p) for p in files},
        input_sha256={str(p):sha(p) for p in run.rglob('*') if p.is_file()},
        technical_episodes=1,phaseA_calls=0,phaseB_conditional_repetitions=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','verify','start','stop'],required=True);a=p.parse_args();run=a.run.resolve()
    if a.mode=='verify':verify(run,True)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else:dict(prepare=prepare,freeze=freeze,start=start)[a.mode](run)
