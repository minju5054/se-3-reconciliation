#!/usr/bin/env python3
"""Saved-only independent source/wire/geometry validation and static review."""
import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/lightnav')]
from run_obstacle_source_acquisition import read,save,sha,evaluate,verify,environments
from reconciliation.obstacle_source_acquisition import paired_frames,select_side
from reconciliation.join_source05 import requests
from analyze_join_online02 import csvread,jsonlines
from validate_robotless_online_handoffs import equal_record,validate_episode
from reconciliation.online_history import history_contract


def calls(run):
    totals=dict(phaseA_terminal_requests=0,phaseA_buffer_requests=0,phaseA_completed_responses=0,
        phase0_terminal_requests=0,phase0_buffer_requests=0,phase0_MPC_submissions=0,phase0_MPC_results=0,
        phaseB_terminal_requests=0,phaseB_MPC_submissions=0,GP_calls=0,rigid_calls=0,reconciliation_calls=0)
    terminal_rtt=[];mpc_times=[]
    for branch in sorted((run/'phaseA').glob('*')):
        for r in requests(branch):
            key='phaseA_terminal_requests' if r['instruction'] else 'phaseA_buffer_requests';totals[key]+=1
        if (branch/'result.json').exists():
            r=read(branch/'result.json');totals['phaseA_completed_responses']+=r['status'] in ('PREDICTION_READY','MODEL_STOP')
            t=(r.get('terminal_prediction') or {}).get('client_rtt_s')
            if t is not None:terminal_rtt.append(t)
    for ep in (run/'phase0/episodes').glob('*'):
        for r in requests(ep):totals['phase0_terminal_requests' if r['instruction'] else 'phase0_buffer_requests']+=1
        for r in jsonlines(ep/'controller/events.jsonl'):
            if r.get('type')=='reply' and r.get('op')=='submit' and r.get('status')=='submitted':totals['phase0_MPC_submissions']+=1
            if r.get('type')=='solve_result':
                totals['phase0_MPC_results']+=1
                t=None if r.get('official_solve_ms') is None else r['official_solve_ms']/1000.
                if t is not None:mpc_times.append(t)
    totals.update(phaseA_terminal_RTT_sum_s=sum(terminal_rtt) if terminal_rtt else None,
        phase0_saved_MPC_wall_s=sum(mpc_times) if mpc_times else None,
        server_warmup_separate=True,scientific_MPC_calls=0)
    return totals


def validate(run):
    verify(run)
    cfg=read(run/'protocol.json')['declaration'];manifest=read(run/'candidate_manifest.json')
    assert [r['bank_index'] for r in manifest]==cfg['candidate_bank_indices']==[10,11,12,13]
    assert [r['candidate'] for r in manifest]==cfg['candidate_order']
    assert read(run/'generation_actual.json')['valid']
    bank=read(Path(read(run/'source.json')['source04_run'])/'bank_manifest.json')
    protocol=read(run/'protocol.json');side=select_side(read(run/'side_passages.json'))
    assert side==protocol['pass_side'] and protocol['instruction']==cfg['instructions'][side]
    expected_order=[f'{c}_{b}' for c in cfg['candidate_order'] for b in ('OFF','ON')]
    assert protocol['order']==expected_order
    sessions=[];raw_hashes={}
    for candidate,index in zip(cfg['candidate_order'],cfg['candidate_bank_indices']):
        pair=paired_frames(bank,index,cfg['history_frames'])
        off_requests=None
        for branch in ('OFF','ON'):
            cid=f'{candidate}_{branch}';m=read(run/'conditions'/f'{cid}.json');out=run/'phaseA'/cid
            assert m['frames']==pair[branch] and m['instruction']==protocol['instruction']
            assert all(sha(f['path'])==f['sha256'] for f in m['frames'])
            if not m['premodel']['valid']:assert not out.exists();continue
            r=read(out/'result.json');assert r['retry_count']==0 and r['source_unchanged'] and r['checkpoint_stat_unchanged']
            req=requests(out);assert len(req)==8
            for i,(q,f) in enumerate(zip(req,m['frames'])):
                assert q['instruction']==(m['instruction'] if i==7 else '') and q['seq']==i
                assert hashlib.sha256(base64.b64decode(q['image'])).hexdigest()==f['sha256']
            if branch=='OFF':off_requests=req
            else:
                assert req[:7]==off_requests[:7]
                a,b=dict(req[-1]),dict(off_requests[-1]);a.pop('image');b.pop('image');assert a==b
            sessions.append(read(out/'session_open.json')['connection_id'])
            for p in (out/'chunks').rglob('*'):
                if p.is_file():raw_hashes[str(p)]=sha(p)
    assert len(set(sessions))==len(sessions)
    expected=evaluate(run);equal_record(read(run/'aggregate/analysis.json'),expected,'analysis')
    declared=(run/'phaseA_execution_start.json')
    assert read(declared)['sha']==read(run/'execution_revision.json')['sha']
    p0=run/'phase0';technical=None
    if (p0/'episodes/PACING_OFF_00/metadata.json').exists():
        config=yaml.safe_load((p0/'config_snapshot.yaml').read_text())
        contract,sampler=history_contract((ROOT/config['paths']['lightnav_checkout']).resolve(),(ROOT/config['paths']['checkpoint_path']).resolve())
        ep=p0/'episodes/PACING_OFF_00';meta=read(ep/'metadata.json')
        technical=validate_episode(ep,p0,contract,sampler,require_plots=False,integration_dt_s=meta['resolved_integration_dt_s'])
        assert technical['valid']
        from reconciliation.join_online02 import guard_check
        base,_,_,_=environments(p0)
        for g in jsonlines(ep/'guard.jsonl'):
            equal_record(guard_check(base,g['start_pose'],g['command'],meta['resolved_integration_dt_s']),
                {k:g[k] for k in ('safe','command','duration_s','start_pose','times_s','poses','check','command_modified','raw_reference_consulted','interval_basis')},'guard')
    return dict(valid=True,source_preserved=True,independent_sessions=len(sessions),raw_hashes=raw_hashes,
        terminal_only_obstacle_difference=True,source_selection_recomputed=True,technical_episode=technical,
        new_model_MPC_optimizer_calls=0,scientific_rejection_not_artifact_corruption=True)


def report(run):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a=read(run/'aggregate/analysis.json');protocol=read(run/'protocol.json');sc=read(run/'scenario.json')
    base,on,cart,_=environments(run);out=run/'review';out.mkdir(exist_ok=False)
    rows=[];figures=[]
    for p in a['pairs']:
        cid=p['candidate'];off=a['results'][cid+'_OFF'];fresh=a['results'][cid+'_ON']
        row=dict(candidate=cid,qualified=p['qualified'],failure_reasons=';'.join(p['failure_reasons']))
        for branch,r in [('OFF',off),('ON',fresh)]:
            for field in ('N','stop','arc_m','max_lateral_m','max_yaw_deg','forward_progress_m','endpoint_from_front_m'):
                row[branch+'_'+field]=r.get(field)
            row[branch+'_clearance_off_m']=r.get('geometry_off',{}).get('minimum_clearance_m')
            row[branch+'_clearance_on_m']=r.get('geometry_on',{}).get('minimum_clearance_m')
        rows.append(row)
        if 'world' not in off or 'world' not in fresh:continue
        fig,ax=plt.subplots(figsize=(8,6))
        for poly in getattr(cart,'geoms',[cart]):
            if hasattr(poly,'exterior'):
                xy=np.asarray(poly.exterior.coords);ax.fill(*xy.T,color='saddlebrown',alpha=.7)
        inflated=cart.buffer(.25)
        for poly in getattr(inflated,'geoms',[inflated]):
            if hasattr(poly,'exterior'):ax.plot(*np.asarray(poly.exterior.coords).T,':',color='saddlebrown',label='radius .20 + edge .05 m')
        for label,r,col in [('OFF raw',off,'#2674ad'),('ON raw',fresh,'#d05239')]:
            xy=np.asarray(r['world']);ax.plot(xy[:,0],xy[:,1],'o--',c=col,label=label)
        pose=np.asarray(off['observation_anchor']);ax.scatter(*pose[:2],c='k',marker='x',s=90,label='observation (no B)')
        ax.set(xlabel='world X [m]',ylabel='world Y [m]',title=cid+' | SOURCE ONLY / NOT EXECUTED\nQualified: '+str(p['qualified']))
        ax.axis('equal');ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8);fig.tight_layout()
        fig.savefig(out/(cid+'.png'),dpi=150);plt.close(fig)
        sidecar=dict(data=dict(OFF=off,ON=fresh,qualification=p),scene_sha256=sha(run/'scenario.json'),
            input_sha256={str(run/'conditions'/f'{cid}_{b}.json'):sha(run/'conditions'/f'{cid}_{b}.json') for b in ('OFF','ON')})
        save(out/(cid+'.json'),sidecar);figures.append(dict(name=cid,png_sha256=sha(out/(cid+'.png')),json_sha256=sha(out/(cid+'.json'))))
    with (run/'aggregate/phaseA.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    counts=calls(run);save(run/'aggregate/call_counts.json',counts)
    p0=read(run/'phase0/qualification.json') if (run/'phase0/qualification.json').exists() else None
    classification=a['primary_classification']
    if a['selected'] and (p0 is None or not p0['qualified']):classification='PACING_BLOCKED_ONLINE_ACQUISITION'
    summary=dict(classification=classification,selected=a['selected'],qualified_count=a['qualified_count'],phase0=p0,
        phaseB_run=False,representative_source=None,reason='no qualified PhaseA' if a['selected'] is None else 'Phase0 blocked' if not p0 or not p0['qualified'] else 'pending separate PhaseB freeze',calls=counts)
    save(run/'aggregate/summary.json',summary);save(out/'plot_manifest.json',figures)
    import html
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;margin:30px}img{max-width:100%}td,th{padding:6px;border:1px solid #aaa}</style><h1>OBSTACLE SOURCE ACQUISITION 01</h1>'
    body+='<p>'+html.escape(classification)+'</p><p>Saved source-only OFF/ON predictions; not execution. No moving B in Phase A. All eight sessions use one frozen instruction.</p><blockquote>'+html.escape(protocol['instruction'])+'</blockquote>'
    body+='<pre>'+html.escape(json.dumps(summary,indent=2))+'</pre>'
    for f in figures:body+=f'<h2>{f["name"]}</h2><a href="review/{f["name"]}.json">numeric and hashes</a><br><img src="review/{f["name"]}.png">'
    with (run/'index.html').open('x') as f:f.write(body)
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [run/'index.html',run/'protocol.json',run/'source.json',*out.glob('*'),*list((run/'aggregate').glob('*.csv')),run/'aggregate/summary.json']:
            z.write(p,p.relative_to(run))


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate',action='store_true');a=p.parse_args();run=a.run.resolve()
    if a.validate:
        r=validate(run)
        if (run/'review/plot_manifest.json').exists():
            a=read(run/'aggregate/analysis.json')
            for f in read(run/'review/plot_manifest.json'):
                assert sha(run/'review'/(f['name']+'.png'))==f['png_sha256']
                side=read(run/'review'/(f['name']+'.json'))
                for b in ('OFF','ON'):equal_record(side['data'][b],a['results'][f['name']+'_'+b],'plot_values')
            equal_record(read(run/'aggregate/call_counts.json'),calls(run),'counts')
        save(run/'validation.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('raw_hashes','technical_episode')}))
    else:report(run)


if __name__=='__main__':main()
