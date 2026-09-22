#!/usr/bin/env python3
"""Saved-only review completion: full APOS proxies, call ledger and ZIP integrity.

Adds presentation artifacts without changing frozen analysis or raw records.
"""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import re
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import shapely

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.join_source04 import ORDER,LABEL
from join_source03_presentation import collection_plot
from join_source02_paired import wire_parity


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();r=a.run.resolve();checks={};results={};calls=0;buffers=0;sessions=[]
    for cid in ORDER:
        out=r/'predictions'/cid;record=read(out/'evaluation.json');results[cid]=record
        wire=wire_parity(out,read(out/'replayed_inputs.json')['frames'])
        checks['wire:'+cid]=wire['valid'];calls+=wire['actual_terminal_predictions_sent'];buffers+=wire['actual_next_requests_sent']-1
        close=read(out/'session_close.json');sessions.append(close['connection_id'])
        checks['one_session:'+cid]=close['prediction_count']==1 and close['next_count']==16 and close['reconnect_count']==close['retry_count']==0
    checks['exact_calls']=calls==7 and buffers==105 and len(set(sessions))==7
    correction=read(r/'technical_correction.json');old=Path(correction['previous_run'])
    for f,h in correction['previous_artifact_sha256'].items():checks['failed_run_preserved:'+f]=sha(f)==h
    checks['failed_run_zero_wire']=not list(old.glob('predictions/*/requests/wire.jsonl'))
    timing=[]
    for path in [old,r]:
        start=read(path/'server_process.json');stop=read(path/'server_shutdown.json');log=(path/'logs/server.log').read_text()
        warmups=re.findall(r'warmup done in (\d+) ms',log)
        timing.append(dict(run=str(path),startup_s=(start['ready_event']['monotonic_ns']-start['start_event']['monotonic_ns'])/1e9,
            lifetime_s=(stop['exit_observed_event']['monotonic_ns']-start['start_event']['monotonic_ns'])/1e9,
            startup_synthetic_warmups=len(warmups),warmup_ms=[int(x) for x in warmups]))
    count=dict(scientific_terminal_predictions=calls,buffer_only_requests=buffers,independent_sessions=len(sessions),
        scientific_MPC_GP_rigid_reconciliation_rollout=0,failed_pre_request_worker_processes=7,failed_worker_prediction_requests=0,
        technical_server_synthetic_warmups=sum(x['startup_synthetic_warmups'] for x in timing),server_timing=timing,
        actual_bank_render_s=read(r/'frame_bank/completed.json')['wall_s'],
        terminal_RTT_sum_s=sum(x['client_rtt_s'] for x in results.values()),worker_wall_sum_s=sum(x['worker_wall_s'] for x in results.values()),
        timing_caveat='RTT/warmup nested in worker/server lifetime, not additive; full-suite historical MPC test counted separately')
    save(r/'aggregate/actual_call_counts.json',count)
    source=read(r/'source03_input.json');pose=np.asarray(source['observation_pose_world']);cart=shapely.from_wkb(bytes.fromhex(source['projection']['obstacle_wkb_hex']))
    fig,ax=plt.subplots(figsize=(8,8));collection_plot(cart,ax=ax,color='gray',alpha=.7,add_points=False)
    collection_plot(cart.buffer(.25),ax=ax,facecolor='none',edgecolor='gray',linestyle=':',add_points=False)
    for cid,color in [('K0_OFF','tab:blue'),('K1','tab:orange'),('K8','tab:purple')]:
        d=results[cid];ap=d['apos'];world=np.asarray(d['world']);ax.plot(world[:,0],world[:,1],color=color,label=cid+' raw')
        origin=np.asarray(ap['ray_world_origin']);ground=ap['ground_xy']
        if ground is not None:ax.plot([origin[0],ground[0]],[origin[1],ground[1]],'--',c=color);ax.scatter(*ground,marker='x',c=color,s=70)
        if ap['first_hit_valid']:ax.scatter(*ap['first_hit_world_xyz'][:2],marker='+',c=color,s=90)
    ax.scatter(*pose[:2],marker='o',c='black',label='Observation pose');ax.set_aspect('equal');ax.legend(loc='upper right')
    ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Full ground proxies (x) and raster first hits (+)\nDashed rays are evaluator proxies; ON points are censored')
    fig.tight_layout();figure=r/'review/apos_ground_world_proxies.png';fig.savefig(figure,dpi=160);plt.close(fig)
    numeric={cid:d['apos'] for cid,d in results.items()};save(figure.with_suffix('.json'),dict(numbers=numeric,
        source_sha256={str(r/'predictions'/cid/'evaluation.json'):sha(r/'predictions'/cid/'evaluation.json') for cid in results},
        script_sha256=sha(__file__),scope=LABEL,limitation='A floor raster hit through mesh gaps does not imply footprint-free ground; clamped intended point unknown'))
    rows=list(csv.DictReader((r/'aggregate/outcomes.csv').open()))
    for row in rows:
        d=results[row['condition']]
        for key,value in [('clearance_on_m',d['geometry_on']['whole']['minimum_clearance_m']),('arc_m',d['geometry_on']['total_arc_m']),('max_lateral_m',d['motion']['max_abs_lateral_m']),('RTT_s',d['client_rtt_s'])]:
            checks['csv:'+row['condition']+':'+key]=float(row[key])==value
    checks['csv_all_conditions']=[x['condition'] for x in rows]==ORDER
    summary=read(r/'aggregate/summary.json');provenance=dict(execution=read(r/'execution.json'),config_sha256=sha(r/'config_snapshot.yaml'),
        source_json_sha256=sha(r/'source.json'),bank_sha256=sha(r/'bank_manifest.json'),freeze_sha256=sha(r/'freeze.json'),
        historical_preserved_hash_count=len(read(r/'source.json')['preserved']),MPC_audit=read(r/'mpc_audit.json')['status'],
        raw_and_external_not_in_zip=True,scope=LABEL)
    save(r/'review/provenance_summary.json',provenance)
    esc=html.escape;page='<html><meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1200px;margin:auto}td,th{padding:6px;border:1px solid #ccc}img{max-width:100%}</style><h1>JOIN-SOURCE-04</h1><p>'+LABEL+'</p>'
    page+='<p>All ON outputs remain unsafe. Persistence improves minimum clearance but does not produce a bypass; all ON APOS are clamped. No online source was obtained.</p><table><tr><th>Condition</th><th>ON clearance m</th><th>First unsafe row/segment (0-based)</th><th>Arc m</th><th>APOS</th></tr>'
    for row in rows:page+='<tr>'+''.join('<td>'+esc(str(v))+'</td>' for v in [row['condition'],row['clearance_on_m'],row['first_unsafe_row']+'/'+row['first_unsafe_segment'],row['arc_m'],row['apos_px']+' clamped='+row['apos_clamped']])+'</tr>'
    page+='</table><p>K0 was safe in its OFF scene (0.935417m); its ON value is a hypothetical obstruction check. Required edge clearance 0.05m. Negative value indicates footprint overlap, not an executed collision.</p><p>K4/K8 paths coincide exactly; no graphical offset. The original local direction panel clips the long K0 ground ray; the following full world proxy plot shows its endpoint.</p>'
    manifest=read(r/'review/figure_manifest.json')+[dict(png=figure.name,sidecar=figure.with_suffix('.json').name,sha256=sha(figure))]
    for f in manifest:
        checks['figure_hash:'+f['png']]=sha(r/'review'/f['png'])==f['sha256']
        page+=f'<h2>{esc(f["png"])}</h2><img src="{f["png"]}"><p><a href="{f["sidecar"]}">Numeric/source sidecar</a></p>'
    page+='<h2>Calls/timing</h2><pre>'+esc(json.dumps(count,indent=2))+'</pre><p>Technical import failure and prior run retained. No scientific-output retry. See repository report for source, thresholds, ray limits and tests.</p></html>'
    with (r/'review/final.html').open('x') as f:f.write(page)
    with (r/'index_final.html').open('x') as f:f.write('<a href="review/final.html">Complete SOURCE04 static evidence review</a>')
    save(r/'review/final_figure_manifest.json',manifest)
    checks['original_validation']=read(r/'validation.json')['valid']
    files=list((r/'review').iterdir())+[r/p for p in ['protocol.json','config_snapshot.yaml','aggregate/summary.json','aggregate/outcomes.csv','aggregate/actual_call_counts.json','validation.json']]
    with zipfile.ZipFile(r/'review_bundle_final.zip','x',zipfile.ZIP_DEFLATED) as z:
        for f in files:z.write(f,str(f.relative_to(r)))
    with zipfile.ZipFile(r/'review_bundle_final.zip') as z:
        for f in files:checks['zip:'+str(f.relative_to(r))]=z.read(str(f.relative_to(r)))==f.read_bytes()
    validation=dict(valid=all(checks.values()),checks=checks,failed=[k for k,v in checks.items() if not v],new_model_MPC_GP_calls=0,
        final_archive_sha256=sha(r/'review_bundle_final.zip'),final_archive_bytes=(r/'review_bundle_final.zip').stat().st_size)
    save(r/'validation_review.json',validation);print(json.dumps(dict(valid=validation['valid'],checks=len(checks),failed=validation['failed'])))
    return 0 if validation['valid'] else 2


if __name__=='__main__':raise SystemExit(main())
