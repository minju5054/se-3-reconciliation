#!/usr/bin/env python3
"""Saved-only terminal-loop evidence; never changes frozen validation/classification."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from analyze_join_online02 import jsonlines


def diagnose(run):
    v=read(run/'validation.json');ep=Path(v['episode'])
    rows=jsonlines(ep/'scheduler.jsonl');captures=jsonlines(ep/'capture.jsonl')
    complete=[r for r in rows if r['sim_steps']==1]
    aborted=[r for r in rows if r['sim_steps']==0]
    actual=[r['rendered_state_id'] for r in captures]
    frozen_expected=list(range(0,len(rows),15))
    completed_capture_ticks=[r['start_state_id'] for r in complete if r['start_state_id']%15==0]
    assert len(aborted)==1 and aborted[0] is rows[-1]
    end=aborted[0]
    assert not end['capture_happened'] and not end['mpc_submitted']
    assert end['model_received']==[dict(type='result',kind='prediction',status='SCENE_INVALID',chunk_id='chunk_002')]
    assert actual==completed_capture_ticks and set(frozen_expected)-set(actual)=={end['start_state_id']}
    assert v['classification']=='TECHNICAL_EXECUTION_BLOCKED' and not v['scheduler']['capture_ticks_valid']
    return dict(classification_unchanged=v['classification'],raw_termination=v['termination_reason'],
        frozen_capture_gate=False,actual_capture_ticks=actual,frozen_expected_capture_ticks=frozen_expected,
        completed_integration_capture_ticks=completed_capture_ticks,
        terminal_loop=end,terminal_loop_executed_steps=0,
        interpretation='Frozen scheduler_audit includes the abort-only final loop in expected captures. The unchanged collector handles model rejection before capture. No capture or integration follows rejection.',
        no_reclassification=True,no_runtime_or_frozen_validator_change=True,new_scientific_calls=0,
        input_sha256={str(ep/n):sha(ep/n) for n in ('scheduler.jsonl','capture.jsonl','metadata.json','raw_unsafe_abort.json')},
        validation_sha256=sha(run/'validation.json'),
        implementation_sha256={n:sha(ROOT/n) for n in ('scripts/isaac/robotless_online_handoffs.py','scripts/validate_obstacle_source_acquisition02.py')},
        diagnostic_script_sha256=sha(__file__))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate',action='store_true')
    a=p.parse_args();run=a.run.resolve();result=diagnose(run)
    output=ROOT/'results/long_continuous_obstacle_reveal_source_01/termination_diagnostic.json'
    if a.validate:assert result==read(output)
    else:save(output,result)
    print(result['classification_unchanged'], 'terminal-loop diagnosis matches saved evidence; zero new calls')
