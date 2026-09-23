#!/usr/bin/env python3
"""Additive saved-only completion; never modifies frozen inputs or evaluations."""
import argparse
import csv
import html
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.obstacle_source_acquisition import timing_gate
from run_obstacle_source_acquisition import verify
from report_obstacle_source_acquisition import validate,calls
from analyze_join_online02 import csvread,jsonlines
from validate_robotless_online_handoffs import equal_record


def complete(run):
    validation=validate(run);a=read(run/'aggregate/analysis.json');protocol=read(run/'protocol.json')
    p0=read(run/'phase0/analysis.json');q=read(run/'phase0/qualification.json')
    expected=timing_gate([r.get('inflight_rtf') for r in p0['rows'][1:]],p0['summary']['max_loop_stall_s'])
    equal_record({k:q[k] for k in expected},expected,'pacing_gate')
    assert not q['qualified'],'this completion is only for frozen pacing-blocked outcome'
    assert not (run/'phaseB').exists(),'no unexpected online acquisition'
    assert read(run/'validation.json')['valid']
    with (run/'aggregate/phaseA.csv').open() as f:table=list(csv.DictReader(f))
    for row,p in zip(table,a['pairs'],strict=True):
        assert row['candidate']==p['candidate'] and row['qualified']==str(p['qualified'])
        assert row['failure_reasons']==';'.join(p['failure_reasons'])
        for branch in ('OFF','ON'):
            r=a['results'][p['candidate']+'_'+branch]
            for k in ('N','arc_m','max_lateral_m','max_yaw_deg','forward_progress_m','endpoint_from_front_m'):
                assert float(row[branch+'_'+k])==r[k]
            assert row[branch+'_stop']==str(r['stop'])
            for scene in ('off','on'):
                assert float(row[branch+'_clearance_'+scene+'_m'])==r['geometry_'+scene]['minimum_clearance_m']
    for f in read(run/'review/plot_manifest.json'):
        assert sha(run/'review'/(f['name']+'.png'))==f['png_sha256']
        assert sha(run/'review'/(f['name']+'.json'))==f['json_sha256']
        s=read(run/'review'/(f['name']+'.json'))
        for branch in ('OFF','ON'):equal_record(s['data'][branch],a['results'][f['name']+'_'+branch],'plot')
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        for name in archive.namelist():assert archive.read(name)==(run/name).read_bytes()
    count=calls(run);equal_record(count,read(run/'aggregate/call_counts.json'),'counts')
    ev=jsonlines(run/'phase0/episodes/PACING_OFF_00/controller/events.jsonl')
    submitted={r['solve_id'] for r in ev if r.get('op')=='submit' and r.get('status')=='submitted'}
    finished={r['solve_id'] for r in ev if r.get('type')=='solve_result'}
    unavailable=sorted(submitted-finished)
    metrics=[]
    for p in a['pairs']:
        row=dict(candidate=p['candidate'],qualified=p['qualified'],gates=p['gates'],failure_reasons=p['failure_reasons'],conditions={})
        for branch in ('OFF','ON'):
            r=a['results'][p['candidate']+'_'+branch]
            row['conditions'][branch]={k:r[k] for k in ('stop','N','arc_m','max_lateral_m','max_yaw_deg','forward_progress_m','future_proxy','complete_bypass_descriptor','raw_sha256','input_hashes','client_rtt_s')}
            row['conditions'][branch].update(clearance_actual_m=r['geometry_off' if branch=='OFF' else 'geometry_on']['minimum_clearance_m'],
                clearance_cart_on_m=r['geometry_on']['minimum_clearance_m'],final_lateral_signed_m=r['raw_local'][-1][1],
                final_yaw_rad=r['raw_local'][-1][2])
        row['mismatch']={k:p['difference'][k] for k in ('maximum_interior_separation_m','maximum_reliable_tangent_difference_deg','maximum_reliable_yaw_difference_deg')}
        metrics.append(row)
    bledger=[dict(repetition=i,status='NOT_RUN_PACING_GATE',OLD=None,FRESH=None,B=None,u_minus=None,
        controller_memory=None,observation_B_travel_m=None,RTF=None,stall=None,qualified=None) for i in range(2)]
    bundle=dict(selected=a['selected'],scope='QUALIFIED PHASE A PAIRED INPUT ONLY; NOT A GENUINE MOVING HANDOFF',
        online_optimization_ready=False,B=None,u_minus=None,controller_memory=None,
        instruction=protocol['instruction'],pass_side=protocol['pass_side'],
        source_paths={b:str(run/'phaseA'/(a['selected']+'_'+b)) for b in ('OFF','ON')},
        source_sha256={str(p):sha(p) for b in ('OFF','ON') for p in (run/'phaseA'/(a['selected']+'_'+b)).rglob('*') if p.is_file()},
        scene=read(run/'scenario.json'),preserved_inputs_manifest=str(run/'source.json'))
    summary=dict(classification='PACING_BLOCKED_ONLINE_ACQUISITION',execution_sha=read(run/'execution_revision.json')['sha'],
        qualified_phaseA_count=a['qualified_count'],selected_phaseA=a['selected'],phaseA=metrics,phase0=q,phaseB=bledger,
        representative_genuine_source=None,calls=count,missing_MPC_completion_ids=unavailable,
        server_synthetic_warmup_calls=1,phaseA_wall_s=(read(run/'phaseA_completion.json')['end_monotonic_ns']-read(run/'phaseA_execution_start.json')['at_monotonic_ns'])/1e9,
        phase0_collection_wall_s=read(run/'phase0/completion.json')['wall_s'],
        phase0_terminal_RTT_sum_s=p0['summary']['model_terminal_RTT_sum_s'],
        limitation_actual_side='ON POSE10/11/12 turn local +y (LEFT). Exact instruction names cart on your RIGHT. No claim of RIGHT-side execution or new rejection gate.',
        remaining_uncertainty='Whether qualified paired raw response survives genuine sudden reveal with timing-qualified moving B.',
        tests_real_model_MPC_optimizer_calls=0)
    save(run/'aggregate/completion.json',summary);save(run/'paired_candidate_bundle/manifest.json',bundle)
    save(run/'phaseB_availability.json',bledger)
    validation.update(csv_numeric_parity=True,PNG_JSON_hash_parity=True,zip_members_exact=True,pacing_recomputed=True,
        no_phaseB=True,completion_script_sha256=sha(__file__),completed_source_count=0)
    save(run/'validation_final.json',validation)
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;margin:30px}img{max-width:100%}td,th{padding:6px;border:1px solid #aaa}</style><h1>OBSTACLE-SOURCE-ACQUISITION-01</h1>'
    body+='<h2>PACING_BLOCKED_ONLINE_ACQUISITION</h2><p>2/4 paired source candidates qualify. POSE11 is first in frozen order. No Phase B / moving-B representative source.</p>'
    body+='<p>RTF: '+str(q['request_rtfs'])+'; maximum loop stall '+str(q['max_loop_stall_s'])+' s. Request-local upper limit remains1.2.</p>'
    body+='<blockquote>'+html.escape(protocol['instruction'])+'</blockquote><p>Returned turns are LEFT in local +y. The exact text says pass the cart on your RIGHT; side compliance is not established. No output correction.</p>'
    body+='<table><tr><th>Candidate</th><th>OFF hypothetical ON edge m</th><th>ON edge m</th><th>ON lateral m</th><th>ON arc m</th><th>Qualified</th><th>Reasons</th></tr>'
    for row in metrics:
        o=row['conditions']['OFF'];n=row['conditions']['ON']
        values=[row['candidate'],f"{o['clearance_cart_on_m']:.6f}",f"{n['clearance_actual_m']:.6f}",f"{n['final_lateral_signed_m']:.6f}",f"{n['arc_m']:.6f}",row['qualified'],','.join(row['failure_reasons'])]
        body+='<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in values)+'</tr>'
    body+='</table><p>Raw predictions only; no observation connector, no suffix trimming, no complete bypass or controller execution claim.</p>'
    for row in metrics:
        cid=row['candidate'];body+=f'<h2>{cid}</h2><a href="review/{cid}.json">numeric/hash sidecar</a><br><img src="review/{cid}.png">'
    with (run/'index_final.html').open('x') as f:f.write(body)
    with zipfile.ZipFile(run/'review_bundle_complete.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [run/'index_final.html',run/'protocol.json',run/'source.json',run/'validation_final.json',run/'phaseB_availability.json',run/'aggregate/completion.json',run/'aggregate/phaseA.csv',*sorted((run/'review').glob('*'))]:z.write(p,p.relative_to(run))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    r=complete(a.run.resolve());print(json.dumps({k:r[k] for k in ('classification','qualified_phaseA_count','selected_phaseA','missing_MPC_completion_ids')}))
