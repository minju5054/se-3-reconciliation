#!/usr/bin/env python3
"""Static saved-record SOURCE03 presentation. Never invokes inference or motion."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import shapely
from shapely.plotting import plot_polygon

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha, trajectory_equal
from reconciliation.gp_se2_environment import HospitalEnvironment


def summarize(run):
    ledger=read(run/'aggregate/ledger.json')
    data={r['condition_id']:read(run/'paired_diagnostics'/r['condition_id']/'evaluation.json')
          for r in ledger if r['status']=='COMPLETED'}
    lc=read(run/'aggregate/lighting_decision.json')['classification']
    def safe(cid):
        r=data.get(cid)
        return bool(r and r.get('whole_raw_on_safe') and r.get('off_on',{}).get('meaningful') and not r.get('stops',{}).get('B_ON'))
    target = ('TARGET_VISIBLE_SAFE_RESPONSE' if safe('target_bright_H16') else
              'TARGET_VISIBLE_STILL_UNSAFE' if 'target_bright_H16' in data else 'TARGET_VISIBILITY_DIAGNOSTIC_INCONCLUSIVE')
    if 'target_bright_H32' in data:
        h16,h32=data['target_bright_H16'],data['target_bright_H32']
        change=h32['checks']['B_ON']['on']['minimum_clearance_m']-h16['checks']['B_ON']['on']['minimum_clearance_m']
        history=('LONGER_HISTORY_RECOVERS_SAFE_RESPONSE' if safe('target_bright_H32') and not safe('target_bright_H16') else
                 'LONGER_HISTORY_IMPROVES_BUT_UNSAFE' if change>=.02 and not safe('target_bright_H32') else 'HISTORY_NOT_EXPLANATORY_UNDER_TEST')
    else: history='HISTORY_UNAVAILABLE' if 'target_bright_H16' not in data else 'NOT_RUN_CONDITIONAL'
    distances=[c for c in data if c.startswith('distance')]
    bypass=[c for c in distances if data[c]['qualified']]
    if bypass:
        distance='FARTHER_OBSTACLE_RECOVERS_SAFE_DETOUR'
    elif len(distances)==3:
        raws=[data[c]['details']['B']['raw_local'] for c in distances]
        distance='LOCAL_HORIZON_NOT_SUFFICIENT_EXPLANATION' if all(trajectory_equal(raws[0],x) for x in raws[1:]) else 'DISTANCE_CHANGES_REACTION_BUT_STILL_UNSAFE'
    else: distance='HORIZON_DIAGNOSTIC_INCONCLUSIVE'
    sham_equal=all(trajectory_equal(r['details']['A']['raw_local'],r['details']['SHAM']['raw_local']) for r in data.values())
    lighting_status={'BRIGHT_RECOVERS_SAFE_RESPONSE':'MAJOR_FACTOR_SUPPORTED_IN_THIS_SCENARIO',
        'BRIGHT_IMPROVES_CLEARANCE_BUT_UNSAFE':'CONTRIBUTING_FACTOR_SUPPORTED',
        'BRIGHT_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN':'DISFAVORED',
        'DARK_AND_BRIGHT_BEHAVIOR_EQUIVALENT':'DISFAVORED'}.get(lc,'INCONCLUSIVE')
    matrix=[
        dict(cause='Accidental MPC modification',evidence_for='None observed',evidence_against='six provenance lineages and actual current import match; terminal paired calls use no MPC',status='STRONGLY_DISFAVORED'),
        dict(cause='Low illumination',evidence_for=lc,evidence_against='single terminal-image lighting intervention; history remains DARK',status=lighting_status),
        dict(cause='Target grounding / visibility',evidence_for=target,evidence_against='renderer pixels are not neural recognition; new-pose diagnostic also changes geometry',status='NOT_ISOLATED'),
        dict(cause='Insufficient temporal history',evidence_for=history,evidence_against='H64 not available; session/SlowFast changes accompany H32',status='CONTRIBUTING_FACTOR_SUPPORTED' if history.startswith('LONGER_HISTORY_RECOVERS') else 'DISFAVORED' if history=='HISTORY_NOT_EXPLANATORY_UNDER_TEST' else 'INCONCLUSIVE'),
        dict(cause='Obstacle distance / finite local horizon',evidence_for=distance,evidence_against='model horizon unchanged; distance range limited by historical local path; later chunks not executed',status='CONTRIBUTING_FACTOR_SUPPORTED' if bypass else 'DISFAVORED' if len(distances)==3 else 'INCONCLUSIVE'),
        dict(cause='Same-input stochastic variability',evidence_for='off/sham difference observed' if not sham_equal else 'none observed in completed off/sham pairs',evidence_against='one sham per condition is not a distribution',status='NOT_ISOLATED' if not sham_equal else 'DISFAVORED'),
        dict(cause='Spatial/free-space/action-generation limitation',evidence_for='unsafe outputs persist' if any(not r['whole_raw_on_safe'] for r in data.values()) else 'no residual unsafe on output in tested inputs',evidence_against='internal perception/geometry/action stages not separately observed',status='NOT_ISOLATED')]
    conclusion={'BRIGHT_RECOVERS_SAFE_RESPONSE':'LIGHTING_MAJOR_FACTOR_SUPPORTED',
        'BRIGHT_IMPROVES_CLEARANCE_BUT_UNSAFE':'LIGHTING_CONTRIBUTES_BUT_NOT_SUFFICIENT',
        'BRIGHT_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN':'LIGHTING_DISFAVORED_AS_PRIMARY_CAUSE',
        'DARK_AND_BRIGHT_BEHAVIOR_EQUIVALENT':'LIGHTING_DISFAVORED_AS_PRIMARY_CAUSE'}.get(lc,'TECHNICAL_BLOCKER')
    if history=='LONGER_HISTORY_RECOVERS_SAFE_RESPONSE':conclusion='HISTORY_FACTOR_SUPPORTED'
    if bypass:conclusion='LOCAL_HORIZON_FACTOR_SUPPORTED'
    return dict(lighting=lc,target=target,history=history,distance=distance,
        receding_horizon='NOT_RUN_OPTIONAL; later-chunk recovery untested',
        root_cause='UNRESOLVED_AFTER_BOUNDED_DIAGNOSIS' if lc!='BRIGHT_RECOVERS_SAFE_RESPONSE' else 'LIGHTING_MAJOR_FACTOR_SUPPORTED_IN_THIS_SCENARIO',
        conclusion=conclusion,evidence_matrix=matrix,off_sham_all_equivalent=sham_equal,
        terminal_predictions=sum(r['primary_calls'] for r in data.values()),
        buffer_only=sum(r['buffer_calls'] for r in data.values()),
        diagnostic_MPC_solves=0,GP_rigid_reconciliation_solves=0,closed_loop_rollouts=0,
        diagnostic_worker_wall_s=sum(r['worker_wall_s'] for r in data.values()),
        request_RTT_sum_s=sum(d['client_rtt_s'] for r in data.values() for d in r['details'].values()),
        qualified_development_conditions=[c for c,r in data.items() if r['qualified']],
        optimization_ready_online_source=False,online_moving_B=None), data, ledger


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve();out=run/'review';out.mkdir(exist_ok=False)
    summary,data,ledger=summarize(run)
    save(run/'aggregate/summary.json',summary)
    figure_records=[]
    inputs={c:read(run/'input_manifests'/f'{c}.json') for c in data}
    hashes=dict(config_sha256=sha(run/'config_snapshot.yaml'),protocol_sha256=sha(run/'protocol.json'),
                execution_sha=read(run/'execution.json')['sha'])
    def finish(fig,name,numbers,source_paths):
        fig.savefig(out/(name+'.png'),dpi=170,bbox_inches='tight');plt.close(fig)
        side=dict(**hashes,numbers=numbers,source_sha256={str(x):sha(x) for x in source_paths},
                  label='DEVELOPMENT INPUT DIAGNOSTIC — predicted rows, not execution')
        save(out/(name+'.json'),side)
        figure_records.append(dict(png=name+'.png',png_sha256=sha(out/(name+'.png')),
                                   sidecar=name+'.json',sidecar_sha256=sha(out/(name+'.json'))))
    tech=run/'technical_preflight/render';luma=read(tech/'luminance.json')
    fig,axs=plt.subplots(1,2,figsize=(12,4))
    for ax,label in zip(axs,['DARK','BRIGHT']):
        ax.imshow(Image.open(tech/f'{label}_ON.jpg'));ax.axis('off')
        ax.set_title(f'{label} — mean {luma[label]["whole"]["mean"]:.2f}; unchanged pose/mesh')
    finish(fig,'dark_bright_same_pose_rgb',luma,[tech/'DARK_ON.jpg',tech/'BRIGHT_ON.jpg',tech/'luminance.json'])
    fig,axs=plt.subplots(1,2,figsize=(10,3.5))
    for ax,region in zip(axs,['whole','cart']):
        for label in ['DARK','BRIGHT']:
            hist=np.array(luma[label][region]['histogram']);ax.plot(np.arange(256),hist/hist.sum(),label=label)
        ax.set(xlabel='sRGB-code luma (0–255)',ylabel='Pixel fraction',title=region);ax.legend()
    finish(fig,'luminance_distribution',luma,[tech/'luminance.json'])
    rows=[]
    for cid,r in data.items():
        m=inputs[cid];folder=run/'paired_diagnostics'/cid
        fig,axs=plt.subplots(1,2,figsize=(11,4.5));ax=axs[0]
        allxy=np.concatenate([np.asarray(d['world'])[:,:2] for d in r['details'].values()]+[np.array([m['observation_pose_world'][:2]])])
        lo=allxy.min(0)-.7;hi=allxy.max(0)+.7
        base=HospitalEnvironment.load(m['environment_export'])
        clip=shapely.geometry.box(lo[0],lo[1],hi[0],hi[1])
        plot_polygon(base.obstacles.intersection(clip),ax=ax,add_points=False,color='.75')
        cart=shapely.from_wkb(bytes.fromhex(m['projection']['obstacle_wkb_hex']))
        plot_polygon(cart,ax=ax,add_points=False,color='black',alpha=.8)
        plot_polygon(cart.buffer(.25),ax=ax,add_points=False,facecolor='none',edgecolor='gray',linestyle=':')
        for alias,color,style in [('A','tab:blue','--'),('B','tab:red','-'),('SHAM','tab:green',':')]:
            d=r['details'][alias];w=np.array(d['world']);ax.plot(w[:,0],w[:,1],style,color=color,label=alias,marker='.',markersize=4)
            axs[1].plot(np.arange(len(w)),d['geometry_on']['node_clearance_m'],style,color=color,label=alias,marker='.')
            g=d['geometry_on']
            rows.append(dict(condition=cid,branch=alias,N=len(w),clearance_on_m=g['whole']['minimum_clearance_m'],
                clearance_off_m=d['geometry_off']['whole']['minimum_clearance_m'],target_visible=d['target_visible'],stop=d['stop'],
                first_unsafe_row=g['first_unsafe_waypoint_zero_based'],first_unsafe_segment=g['first_unsafe_segment_zero_based'],
                arc_m=g['total_arc_m'],RTT_s=d['client_rtt_s']))
        ax.scatter(*m['observation_pose_world'][:2],marker='o',color='black',label='Observation pose')
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X (m)',ylabel='World Y (m)');ax.set_aspect('equal')
        ax.legend(loc='upper left',bbox_to_anchor=(0,-.16),ncol=4,fontsize=8)
        axs[1].axhline(.05,color='black',ls='--',label='required 5cm');axs[1].axhline(0,color='gray')
        axs[1].set(xlabel='Original row index (not time)',ylabel='Footprint-edge clearance (m)',title='Point clearance; swept segments also checked')
        axs[1].legend(fontsize=8)
        fig.suptitle(cid+' | '+r['label']+' | predicted, not executed',fontsize=11)
        fig.tight_layout()
        finish(fig,cid+'_trajectory_clearance',r,[folder/'evaluation.json',run/'input_manifests'/f'{cid}.json'])
        fig,axs=plt.subplots(1,3,figsize=(12,3.4))
        for ax,alias in zip(axs,['A','B','SHAM']):
            ax.imshow(Image.open(m['final_frames'][alias]['path']))
            pt=r['details'][alias]['pointing'] or {}
            for key,color in [('apos_px','cyan'),('opos_px','yellow')]:
                if pt.get(key) is not None:ax.scatter(*pt[key],marker='x',c=color,s=65,label=key)
            ax.set_title(f'{alias}: target visible={r["details"][alias]["target_visible"]}',fontsize=10)
            ax.axis('off')
            if ax.get_legend_handles_labels()[0]:ax.legend(loc='upper left',fontsize=7)
        fig.suptitle(cid+' — pointing overlay, never sent to model',fontsize=11)
        finish(fig,cid+'_pointing',{a:r['details'][a]['pointing'] for a in ['A','B','SHAM']},
            [folder/'evaluation.json']+[Path(m['final_frames'][a]['path']) for a in ['A','B']])
    if len(data)>=2:
        fig,ax=plt.subplots(figsize=(9,4))
        for cid,r in data.items():
            d=r['details']['B'];ax.plot(np.arange(len(d['world'])),d['geometry_on']['node_clearance_m'],marker='.',label=cid)
        ax.axhline(.05,c='black',ls='--');ax.axhline(0,c='gray');ax.set(xlabel='Original row index (not time)',ylabel='ON footprint-edge clearance (m)')
        ax.legend(loc='upper left',bbox_to_anchor=(1,1),fontsize=8)
        finish(fig,'all_on_row_clearance',{c:r['details']['B']['geometry_on'] for c,r in data.items()},[run/'paired_diagnostics'/c/'evaluation.json' for c in data])
    fig,ax=plt.subplots(figsize=(13,4));ax.axis('off')
    table=ax.table(cellText=[[r['cause'],r['status']] for r in summary['evidence_matrix']],colLabels=['Candidate cause','Evidence status'],loc='center',cellLoc='left',colWidths=[.56,.44])
    table.auto_set_font_size(False);table.set_fontsize(9);table.scale(1,2)
    ax.set_title(summary['conclusion']+'\nOne bounded scenario; internal root cause may remain unresolved',fontsize=11)
    finish(fig,'root_cause_evidence_matrix',summary['evidence_matrix'],[run/'aggregate/summary.json'])
    with (run/'aggregate/outcomes.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    save(out/'figure_manifest.json',figure_records)
    html='<html><meta charset="utf-8"><title>JOIN-SOURCE-03</title><body><h1>JOIN-SOURCE-03 — upstream diagnosis</h1><p>'+summary['conclusion']+'</p><p>Actual Isaac input renders; saved LightNav predictions. No new MPC execution, GP or online source confirmation.</p><pre>'+json.dumps(ledger,indent=2)+'</pre>'
    for f in figure_records:html+=f'<h2>{f["png"]}</h2><img style="max-width:100%" src="{f["png"]}"><p><a href="{f["sidecar"]}">numbers and source hashes</a></p>'
    (out/'index.html').write_text(html+'</body></html>')
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(out.iterdir()):z.write(p,'review/'+p.name)
        for p in [run/'protocol.json',run/'config_snapshot.yaml',run/'mpc_audit/result.json',run/'aggregate/summary.json',run/'aggregate/outcomes.csv']:
            z.write(p,str(p.relative_to(run)))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
