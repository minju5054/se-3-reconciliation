#!/usr/bin/env python3
"""Corrected saved-only audit, retaining every other historical timing check."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.successive_scheduler02 import capture_phase_audit
from reconciliation.join_source03 import read, save, sha
from analyze_join_online02 import jsonlines
from validate_obstacle_source_acquisition02 import scheduler_audit as historical_audit


def scheduler_audit(ep):
    result=historical_audit(ep)
    capture=capture_phase_audit(jsonlines(ep/'scheduler.jsonl'),jsonlines(ep/'capture.jsonl'))
    result.update(capture_phase_audit=capture, capture_ticks_valid=capture['valid'],
        valid=not result['burst_state_ids'] and result['control_ticks_valid'] and capture['valid'])
    return result


def historical_revalidation():
    run=ROOT/'data/long_continuous_obstacle_reveal_source_01/primary_20261007'
    ep=run/'episodes/EPISODE_00'; v=read(run/'validation.json')
    result=scheduler_audit(ep)
    assert result['valid'] and not v['scheduler']['valid']
    return dict(corrected_scheduler=result,historical_classification_unchanged=v['classification'],
        historical_validation_sha256=sha(run/'validation.json'),
        inputs={str(ep/n):sha(ep/n) for n in ('scheduler.jsonl','capture.jsonl','metadata.json','execution.csv')},
        explanation='The previous technical classification included an audit-definition limitation. Original result and frozen validator are unchanged. This saved-only audit does not increase its two-chunk applied prefix.',
        new_LightNav_MPC_Isaac_optimizer_calls=0)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args()
    result=historical_revalidation()
    out=ROOT/'results/successive_native_source_acquisition_02/historical_long01_corrected_audit.json'
    if a.check: assert result==read(out)
    else: save(out,result)
    print('Saved LONG_SOURCE_01 corrected scheduler PASS; frozen classification unchanged; new calls 0')
