#!/usr/bin/env python3
"""Recompute SOURCE05 evidence from saved records; no inference or rendering."""
import argparse
import csv
import json
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.join_source05 import ORDER,KS,GENERATION,condition,summarize,requests,request_parity,historical_id
from run_join_source05 import verify,evaluate,source_run
from report_join_source05 import load_results,rows,call_counts
from join_source02_paired import wire_parity


def norm(x):return json.loads(json.dumps(x,allow_nan=False))


def validate(run):
    checks={}
    def check(k,v):checks[k]=bool(v)
    try:verify(run);check('freeze_and_all_historical_hashes',True)
    except (ValueError,FileNotFoundError) as e:check('freeze:'+str(e),False)
    source=source_run(run);bank=read(source/'bank_manifest.json');ledger=read(run/'aggregate/ledger.json')
    check('order',[x['condition'] for x in ledger]==ORDER)
    actual=read(run/'generation_actual.json')
    check('actual_generation',actual['valid'] and all(actual['actual_relevant_inherited_environment'].get(k)==v for k,v in GENERATION.items()))
    results,historical,context=load_results(run);sessions=[];calls=0;buffer=0
    for cid in ORDER:
        m=read(run/'conditions'/f'{cid}.json');check('condition:'+cid,m==condition(bank,cid))
        out=run/'predictions'/cid
        if not (out/'result.json').exists():
            check('missing_not_fabricated:'+cid,cid not in results);continue
        result=read(out/'result.json')
        if (out/'replayed_inputs.json').exists():
            frames=read(out/'replayed_inputs.json')['frames'];w=wire_parity(out,frames)
            calls+=w.get('actual_terminal_predictions_sent',0);buffer+=w.get('actual_next_requests_sent',0)-w.get('actual_terminal_predictions_sent',0)
        if cid not in results:continue
        r=evaluate(run,cid);check('recomputed:'+cid,norm(r)==results[cid]);check('wire:'+cid,w['valid'])
        parity=request_parity(requests(out),requests(source/'predictions'/historical_id(m['K'])),m['frames'],m['instruction'])
        check('only_terminal_instruction:'+cid,parity['valid'])
        close=read(out/'session_close.json');sessions.append(close['connection_id'])
        check('no_retry:'+cid,close['retry_count']==close['reconnect_count']==0 and close['connection_count']==close['wire_reset_count']==close['wire_login_count']==1)
        check('input16:'+cid,len(frames)==16 and [f['sha256'] for f in frames]==[f['sha256'] for f in m['frames']])
        check('no_overwritten_source:'+cid,result['source_unchanged'] and result['checkpoint_stat_unchanged'])
    check('independent_sessions',len(sessions)==len(set(sessions)))
    check('bounded_calls',calls<=12 and buffer<=180)
    check('all_completed_coverage',len(results)!=12 or (calls==12 and buffer==180))
    for i in ('I1','I2'):
        a=read(run/'conditions'/f'{i}_K1.json');b=read(run/'conditions'/f'{i}_K1_SHAM.json')
        check('sham_input:'+i,a['frames']==b['frames'] and a['instruction']==b['instruction'])
    summary=summarize(results,historical,context['influence']);check('summary',norm(summary)==read(run/'aggregate/summary.json'))
    check('calls_and_timing',call_counts(run,results)==read(run/'aggregate/actual_call_counts.json'))
    check('zero_execution',summary['new_MPC_GP_rigid_reconciliation_rollout_render_calls']==0)
    numbers=rows(results,historical)
    with (run/'aggregate/outcomes.csv').open() as f:actual_rows=list(csv.DictReader(f))
    expected=[{k:'' if v is None else str(v) for k,v in row.items()} for row in numbers]
    check('table',actual_rows==expected)
    for fig in read(run/'review/figure_manifest.json'):
        check('figure:'+fig['png'],sha(run/'review'/fig['png'])==fig['sha256'])
        side=read(run/'review'/fig['sidecar'])
        check('numbers:'+fig['png'],side['numbers']==norm(numbers) and side['summary']==norm(summary))
        check('figure_config:'+fig['png'],side['config_sha256']==sha(run/'config_snapshot.yaml'))
        check('figure_bank:'+fig['png'],side['frame_bank_sha256']==sha(source/'bank_manifest.json'))
        for p,h in side['input_sha256'].items():check('figure_input:'+p,sha(p)==h)
    with zipfile.ZipFile(run/'review_bundle.zip') as z:
        for name in z.namelist():check('zip:'+name,z.read(name)==(run/name).read_bytes())
        check('no_raw_corpus_in_zip',not any('/rgb/' in n or n.endswith('.npy') for n in z.namelist()))
    return dict(valid=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],
        actual_scientific_predictions=calls,actual_buffer_only_requests=buffer,
        actual_successful_predictions=len(results),new_validation_model_MPC_GP_calls=0,
        scientific_failure_is_not_corruption=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',default='validation.json');a=p.parse_args()
    r=validate(a.run.resolve());save(a.run/a.output,r);print({k:v for k,v in r.items() if k!='checks'});raise SystemExit(0 if r['valid'] else 2)
