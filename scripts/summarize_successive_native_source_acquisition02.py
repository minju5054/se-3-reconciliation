#!/usr/bin/env python3
"""Saved-only bounded search audit and report selection. Makes no scientific calls."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_native_source_acquisition02 import QUALIFIED
from run_successive_native_source_acquisition02 import OUT,declaration,verify
from validate_successive_native_source_acquisition02 import validate,validate_bundle
from validate_robotless_online_handoffs import equal_record


def summarize(batch):
    manifest=read(OUT/'candidate_manifest.json');attempts=[];selected=None;report_candidate=None
    for c in manifest:
        run=batch/c['candidate_id'];verify(run)
        if not c['geometry_valid']:
            assert not (run/'launch_attempt.json').exists()
            attempts.append(dict(candidate_id=run.name,geometry_rejected=True));continue
        if not (run/'attempt_validation.json').exists():
            assert not (run/'launch_attempt.json').exists(),'attempt not sealed'
            if selected is None: raise ValueError('bounded search incomplete')
            continue
        assert selected is None,'ran a candidate after first qualified source'
        seal=read(run/'candidate_seal.json')
        for p,h in seal['files'].items():assert sha(p)==h,p
        v=validate(run);equal_record(v,read(run/'attempt_validation.json'))
        if v['scientific']: assert validate_bundle(run,v['scientific'])['valid']
        v=v['scientific'] or v
        report_candidate=run.name
        attempts.append(dict(candidate_id=run.name,classification=v['classification'],
            termination=v.get('termination_reason'),prefix=v.get('longest_applied_prefix'),
            counts=read(run/'attempt_validation.json')['calls'],
            attempt_validation_sha256=sha(run/'attempt_validation.json'),seal_sha256=sha(run/'candidate_seal.json')))
        if v['classification'] in QUALIFIED: selected=run.name
    counts={}
    for a in attempts:
        for k,v in a.get('counts',{}).items():
            if type(v) in (int,float):counts[k]=counts.get(k,0)+v
    assert counts.get('full_SimulationApp_launch_attempts',0)<=3
    return dict(experiment=declaration()['experiment'],source_type='DEVELOPMENT SOURCE',
        selected_candidate=selected,primary_figure_candidate=selected or report_candidate,
        classification=next(a['classification'] for a in attempts if a['candidate_id']==selected) if selected else 'NO_USABLE_SUCCESSIVE_SOURCE',
        candidates=attempts,aggregate_call_accounting=counts,qualification_ignores_lateral_sign=True,
        maximum_three_attempts=True,no_retry=True,new_validation_model_MPC_Isaac_optimizer_calls=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--batch',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    result=summarize(a.batch.resolve())
    if a.check:equal_record(result,read(OUT/'search_summary.json'))
    else:save(OUT/'search_summary.json',result)
    print(result['classification'],result['selected_candidate'],result['aggregate_call_accounting'])
