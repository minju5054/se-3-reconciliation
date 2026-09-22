#!/usr/bin/env python3
"""Source-only corpus scan and saved-only recomputation; never run a controller."""
import argparse
import csv
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import numpy as np
import yaml
from reconciliation.genuine_source_scan import run_scan, order_key
from reconciliation.gp_se2_attach01_source import digest
from reconciliation.gp_se2_environment import HospitalEnvironment


def save(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


def flat(row):
    m=row['mismatch'];v=row['motion']
    return dict(case_id=row['case_id'],**row['flags'],v_minus_mps=v['v_minus_mps'],omega_minus_radps=v['omega_minus_radps'],
        observation_to_B_arc_m=v['observation_to_B_arc_m'],observation_to_B_sim_s=v['observation_to_B_sim_s'],
        e_perp_m=m['projection']['e_perp_m'],projection_location=m['projection']['projection_location'],
        signed_lateral_gap_m=m['signed_lateral_gap_m'],along_tangent_gap_m=m['along_tangent_gap_m'],
        reliable_direction_deg=m['reliable_window_direction_deg'],pose_yaw_deg=m['pose_yaw_difference_deg'],
        raw_full_clearance_m=row['entire_raw_environment']['minimum_clearance_m'],
        raw_suffix_clearance_m=row['raw_suffix_clearance_m'],common_clearance_m=row['common_clearance_m'],
        past_clearance_lower_bound_m=row['past_environment']['minimum_clearance_lower_bound_m'],
        B_clearance_m=row['boundary_environment']['clearance_m'],route_status=row['route_status'],raw_hash_changed=row['raw_local_hash_changed'],
        suffix_rows=row['suffix_rows'],suffix_arc_m=row['suffix_arc_m'],ordered_raw_pair=row['ordered_raw_pair'],
        rejection_reasons=';'.join(row['rejection_reasons']))


def figures(out,result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely.geometry import box
    rows=result['rows'];environment=HospitalEnvironment.load(result['environment_path'])
    plotdir=out/'plots';plotdir.mkdir()
    candidates=sorted([r for r in rows if r['flags']['candidate']],key=order_key)
    nearby=[r for r in candidates if r['flags']['candidate_obstacle_sensitive_no_gate']]
    representatives=(nearby+[r for r in candidates if r not in nearby])[:6]
    def write_figure(name,fig,numeric):
        fig.savefig(plotdir/(name+'.png'),dpi=160,bbox_inches='tight');plt.close(fig)
        save(plotdir/(name+'.json'),dict(numbers=numeric,ledger_sha256=digest(out/'ledger.json'),
            source_manifest_sha256=digest(out/'source.json'),png_sha256=digest(plotdir/(name+'.png'))))
    f,ax=plt.subplots(figsize=(8,5))
    for flag,color,label in [('candidate','tab:orange','All source criteria pass'),('safe_entire_raw_and_common','tab:blue','Safe FRESH/common, other criteria vary')]:
        selected=[r for r in rows if r['flags'][flag] and r['mismatch']['signed_lateral_gap_m'] is not None]
        if flag=='safe_entire_raw_and_common':selected=[r for r in selected if not r['flags']['candidate']]
        ax.scatter([r['motion']['v_minus_mps'] for r in selected],[abs(r['mismatch']['signed_lateral_gap_m']) for r in selected],s=15,c=color,label=label,alpha=.7)
    ax.axvline(.2,color='k',ls='--');ax.axhline(.1,color='k',ls=':');ax.set(xlabel='Physical v immediately before B [m/s]',ylabel='Absolute lateral component to FRESH [m]',title='Saved source geometry only; orientation also qualifies')
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1));write_figure('corpus_moving_lateral',f,[flat(r) for r in rows])
    links=[]
    for r in representatives:
        old=np.load(r['source_paths']['old_world']);fresh=np.load(r['source_paths']['fresh_world']);context=json.load(open(r['source_paths']['context']))
        ep=Path(r['source_paths']['context']).parents[2]
        with (ep/'execution.csv').open() as stream:
            states=[s for s in csv.DictReader(stream) if context['obs_state_id']<=int(s['state_id'])<=context['switch_state_id']]
        past=np.array([[float(s[k]) for k in ('x','y','yaw')] for s in states]);b=np.array(r['B_world']);q=np.array(r['mismatch']['projection']['Q_xy_world_m'])
        xy=np.vstack([old[:,:2],fresh[:,:2],past[:,:2]]);lo=xy.min(axis=0)-.6;hi=xy.max(axis=0)+.6
        f,ax=plt.subplots(figsize=(7,7));geom=environment.obstacles.intersection(box(*lo,*hi))
        polygons=list(geom.geoms) if hasattr(geom,'geoms') else [geom]
        for polygon in polygons:
            if hasattr(polygon,'exterior'):
                x,y=polygon.exterior.xy;ax.fill(x,y,color='.25',alpha=.7)
        ax.plot(old[:,0],old[:,1],'--',c='tab:blue',label='Original OLD (prediction)')
        ax.plot(fresh[:,0],fresh[:,1],'o-',c='tab:red',label='Original FRESH (prediction)',ms=4)
        ax.plot(past[:,0],past[:,1],c='.5',lw=3,label='Recorded execution: observation to B')
        ax.plot(b[0],b[1],'*',ms=15,c='k',label='Actual B');ax.plot(*r['observation_pose_world'][:2],'s',c='tab:green',label='FRESH observation')
        ax.plot([b[0],q[0]],[b[1],q[1]],':',c='purple',label='Diagnostic projection, not a plan')
        ax.add_patch(plt.Circle(b[:2],.2,fill=False,color='k'));ax.add_patch(plt.Circle(b[:2],.25,fill=False,color='k',ls=':'))
        ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]',title=r['case_id']+'\nSOURCE ONLY: no new execution')
        ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);name=r['case_id'].replace('/','__');write_figure(name,f,r);links.append((r['case_id'],'plots/'+name+'.png'))
    columns=['case_id','v_minus_mps','observation_to_B_arc_m','signed_lateral_gap_m','reliable_direction_deg','pose_yaw_deg','raw_full_clearance_m','raw_suffix_clearance_m','candidate_obstacle_sensitive_no_gate']
    table='<table border="1"><tr>'+''.join('<th>'+k+'</th>' for k in columns)+'</tr>'
    for r in candidates:
        row=flat(r);table+='<tr>'+''.join('<td>'+html.escape(str(row[k]) if row[k] is not None else 'N/A')+'</td>' for k in columns)+'</tr>'
    table+='</table>'
    content='<h1>Genuine source-only moving/mismatch scan</h1><p>No new inference, optimization, MPC or rollout. FRESH geometry validity is not execution feasibility.</p><pre>'+html.escape(json.dumps(result['summary'],indent=2))+'</pre>'+table
    content+='<p><a href="ledger.csv">All 881 records</a></p><img width="800" src="plots/corpus_moving_lateral.png">'
    for name,p in links:content+='<h2>'+name+'</h2><img width="700" src="'+p+'">'
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8">'+content)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--validate',action='store_true');a=p.parse_args();out=a.output.resolve()
    if a.validate:
        source=json.load(open(out/'source.json'))
        for path,h in {**source['source_hashes'],**source['code_hashes']}.items():
            if digest(path)!=h:raise ValueError('hash mismatch: '+path)
        protocol=yaml.safe_load((out/'protocol.yaml').read_text());r=run_scan(ROOT,protocol)
        if r['rows']!=json.load(open(out/'ledger.json')) or r['summary']!=json.load(open(out/'summary.json')):raise ValueError('saved scan does not reproduce')
        for path in (out/'plots').glob('*.json'):
            side=json.load(open(path))
            if side['png_sha256']!=digest(path.with_suffix('.png')) or side['ledger_sha256']!=digest(out/'ledger.json'):raise ValueError('plot hash differs')
            expected=[flat(x) for x in r['rows']] if path.stem=='corpus_moving_lateral' else next(x for x in r['rows'] if x['case_id'].replace('/','__')==path.stem)
            if side['numbers']!=expected:raise ValueError('plot numbers differ')
        with (out/'ledger.csv').open() as stream:
            saved=list(csv.DictReader(stream))
        expected=[{k:'' if v is None else str(v) for k,v in flat(x).items()} for x in r['rows']]
        if saved!=expected:raise ValueError('CSV differs')
        save(out/'validation.json',dict(valid=True,record_count=len(r['rows']),saved_only=True,new_calls=0,source_and_code_hashes_unchanged=True))
        print('VALID saved source-only scan',flush=True);return
    out.mkdir(parents=True,exist_ok=False)
    config=ROOT/'configs/genuine_source_moving_mismatch_scan.yaml';text=config.read_text();protocol=yaml.safe_load(text);(out/'protocol.yaml').write_text(text)
    files=[config,ROOT/'src/reconciliation/genuine_source_scan.py',Path(__file__).resolve(),ROOT/protocol['reuse_source_policy']]
    files += [ROOT/'src/reconciliation'/name for name in ['gp_se2_attach01_source.py','gp_se2_environment.py','gp_se2_reference.py','gp_se2_rollout.py','online_handoff_analysis.py','robotless_online.py','se2.py']]
    hashes={str(path):digest(path) for path in files}
    start=time.perf_counter();result=run_scan(ROOT,protocol)
    save(out/'ledger.json',result['rows']);save(out/'summary.json',result['summary'])
    save(out/'source.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),git_SHA=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_hashes=result['source_hashes'],code_hashes=hashes,environment_path=result['environment_path'],scan_wall_s=time.perf_counter()-start,code_uncommitted_at_scan=True))
    with (out/'ledger.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat(result['rows'][0])));writer.writeheader();writer.writerows(flat(r) for r in result['rows'])
    figures(out,result);print(json.dumps(result['summary'],indent=2),flush=True)


if __name__=='__main__':main()
