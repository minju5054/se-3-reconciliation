#!/usr/bin/env python3
"""Finish saved DIAG06 reporting after its missing aggregate-directory error.

This additive repair never reruns analysis, optimization, candidate selection or
acceptance. The frozen numerical runner and failed console log are preserved.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import time
from run_gp_se2_diag06 import verify_frozen,assemble,read,write,table,digest,utc
from plot_gp_se2_diag06 import generate


def finish(run):
    verify_frozen(run)
    if not (run/'optimization_completed.json').exists():raise ValueError('saved five-start execution required')
    if any((run/p).exists() for p in ('aggregate','analysis_completed.json','plots','reporting_correction.json')):
        raise FileExistsError('exclusive report repair only; no overwrite')
    for row in read(run/'experiment_manifest.json')['starts']:
        if not (run/'solves'/row['solve_id']/'analysis.json').exists():raise ValueError('all saved analyses required')
    log=run/'analysis_console.log'
    if "aggregate/all_methods.csv" not in log.read_text() or 'FileNotFoundError' not in log.read_text():
        raise ValueError('this repair is restricted to the recorded missing-directory error')
    begin=time.perf_counter();write(run/'reporting_correction.json',dict(utc=utc(),
        issue='Frozen runner omitted aggregate.mkdir before exclusive CSV writes',
        failed_log_sha256=digest(log),repair_script_sha256=digest(__file__),
        scope='create missing directory; assemble existing saved analyses; render/package',
        original_execution_source_unchanged=True,new_GP_solves=0,new_MPC=0,new_rollout=0,
        numerical_analysis_rerun=False,all_original_outputs_preserved=True))
    data,tables=assemble(run);(run/'aggregate').mkdir()
    for name,rows in tables.items():table(run/'aggregate'/(name+'.csv'),rows)
    write(run/'aggregate/summary.json',data['summary']);write(run/'plot_input.json',data)
    write(run/'analysis_completed.json',dict(utc=utc(),report_repair_wall_s=time.perf_counter()-begin,
        primary_analysis_wall_s=sum(read(run/'solves'/row['solve_id']/'analysis.json')['wall_s'] for row in read(run/'experiment_manifest.json')['starts']),
        note='primary analysis ran once before CSV mkdir failure; source per-start timings retained'))
    t=time.perf_counter();generate(run);write(run/'rendering_completed.json',dict(utc=utc(),wall_s=time.perf_counter()-t))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);finish(p.parse_args().run.resolve())
