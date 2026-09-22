#!/usr/bin/env python3
"""Static scientific plots and tables from saved persistence predictions only."""
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

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.join_source04 import ORDER,KS,LABEL,persistence
from join_source03_presentation import collection_plot


def summarize(results,ledger):
    decision=persistence(results)
    ons=[r for r in results.values() if r['K']>0 and r['condition']!='K1_SHAM']
    if ons and all(r['apos']['pointing'].get('apos_clamped') for r in ons):
        apos='AFFORDANCE_REMAINS_AMBIGUOUS_DUE_TO_CLAMPING'
    elif any(r['affordance_action_category']=='APOS_TRAJECTORY_DIRECTION_DISAGREEMENT' for r in ons):
        apos='AFFORDANCE_ACTION_DIRECTION_DISAGREEMENT'
    elif any(r['affordance_action_category']=='APOS_CLEAR_SIDE_TRAJ_UNSAFE' for r in ons):
        apos='AFFORDANCE_FREE_SIDE_BUT_ACTION_UNSAFE'
    elif any(r['affordance_action_category']=='APOS_CLEAR_SIDE_TRAJ_SAFE' for r in ons):
        apos='AFFORDANCE_FREE_SIDE_AND_ACTION_SAFE'
    elif ons and all(r['apos']['first_hit_class'] in ('cart','wall','target','other') for r in ons):
        apos='AFFORDANCE_POINTS_TO_OBSTACLE_OR_NONFREE_REGION'
    else:apos='AFFORDANCE_DIAGNOSTIC_INCONCLUSIVE'
    promising=[r['condition'] for r in ons if r['geometry_on']['whole']['clearance_valid'] and
               not r['stop'] and r['motion']['target_directed_progress_m']>0]
    comparisons=decision.get('comparisons',{})
    decision['materially_improving_conditions']=[k for k,v in comparisons.items() if v['material'] or v['unsafe_onset_later']]
    ordered=[results[k]['geometry_on']['whole']['minimum_clearance_m'] for k in ('K1','K2','K4','K8') if k in results]
    decision['minimum_clearance_nondecreasing_over_available_K']=all(b>=a for a,b in zip(ordered,ordered[1:])) if len(ordered)>1 else None
    label=decision['classification'];sudden=('MAJOR_FACTOR_SUPPORTED_IN_THIS_SCENARIO' if label=='PERSISTENCE_RECOVERS_SAFE_FRESH'
        else 'CONTRIBUTING_FACTOR_SUPPORTED' if label=='PERSISTENCE_IMPROVES_BUT_REMAINS_UNSAFE'
        else 'DISFAVORED' if label in ('PERSISTENCE_HAS_NO_MATERIAL_EFFECT','PERSISTENCE_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN')
        else 'INCONCLUSIVE')
    matrix=[
        dict(cause='Accidental MPC modification',status='STRONGLY_DISFAVORED',evidence='SOURCE03/source04 pinned hash match; no diagnostic MPC calls',limit='historical motion affected original pose lineage'),
        dict(cause='Low illumination',status='CONTRIBUTING_FACTOR_SUPPORTED',evidence='Historical SOURCE03 terminal relighting changed clearance; all current inputs BRIGHT',limit='SOURCE03 and SOURCE04 are different histories, not repeats'),
        dict(cause='Insufficient history count',status='DISFAVORED',evidence='Historical H16/H32 did not recover; current H fixed16',limit='no new history-count intervention'),
        dict(cause='Sudden single-frame reveal',status=sudden,evidence=label,limit='one fixed moving-pose counterfactual, no online episode'),
        dict(cause='Obstacle distance/local horizon',status='DISFAVORED',evidence='Historical near/medium/far unsafe; current cart fixed',limit='later chunks not tested'),
        dict(cause='Target grounding',status='NOT_ISOLATED',evidence='target visible/OPOS recorded per condition',limit='renderer visibility is not model recognition'),
        dict(cause='Affordance spatial reasoning',status='NOT_ISOLATED',evidence=apos,limit='coarse/clamped pixel is not a metric plan'),
        dict(cause='Action geometric realization',status='NOT_ISOLATED',evidence='raw trajectory clearance and broad-side agreement measured',limit='observable mismatch cannot identify RVQ failure'),
        dict(cause='Same-input variability',status='DISFAVORED' if all(decision.get('sham',{}).values()) and decision.get('sham') else 'INCONCLUSIVE',evidence=str(decision.get('sham')),limit='one K0 and one K1 sham, not a distribution')]
    return dict(scope=LABEL,persistence=decision,apos_classification=apos,evidence_matrix=matrix,
        promising_counterfactual_conditions=promising,online_source_obtained=False,
        model_predictions=sum(x.get('calls') or 0 for x in ledger),new_MPC_GP_reconciliation_rollout=0,
        RTT_sum_s=sum(r['client_rtt_s'] for r in results.values()),worker_wall_sum_s=sum(r['worker_wall_s'] for r in results.values()))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve();out=run/'review';out.mkdir(exist_ok=False)
    ledger=read(run/'aggregate/ledger.json');results={x['condition']:read(run/'predictions'/x['condition']/'evaluation.json') for x in ledger if x['status']=='COMPLETED'}
    bank=read(run/'bank_manifest.json');summary=summarize(results,ledger);save(run/'aggregate/summary.json',summary)
    inputs={str(run/'predictions'/k/'evaluation.json'):sha(run/'predictions'/k/'evaluation.json') for k in results}
    sources=dict(input_sha256=inputs,bank_manifest_sha256=sha(run/'bank_manifest.json'),config_sha256=sha(run/'config_snapshot.yaml'),
                 renderer_sha256=sha(__file__),scope=LABEL)
    figures=[]
    def emit(fig,name,numbers):
        fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig)
        save(out/(name+'.json'),dict(numbers=numbers,**sources))
        figures.append(dict(png=name+'.png',sidecar=name+'.json',sha256=sha(out/(name+'.png'))))
    for cid in ('K0_OFF','K1','K4','K8'):
        m=read(run/'conditions'/f'{cid}.json');fig,axs=plt.subplots(4,4,figsize=(13,9))
        for i,(ax,frame) in enumerate(zip(axs.ravel(),m['frames'])):
            ax.imshow(Image.open(frame['path']));ax.set_title(f"{i}: {frame['frame_id']} | {'ON' if frame['presence'] else 'OFF'}",fontsize=8);ax.axis('off')
        fig.suptitle(cid+' | '+LABEL+'\nAll BRIGHT; titles outside unannotated model RGB',fontsize=11)
        fig.tight_layout(rect=(0,0,1,.94));emit(fig,'history_'+cid,m)
    keys=[k for k in ORDER if k in results];fig,axs=plt.subplots(2,4,figsize=(16,8))
    for ax,cid in zip(axs.ravel(),keys):
        r=results[cid];state='ON' if r['K'] else 'OFF';b=bank[-1][state]
        ax.imshow(Image.open(b['frame']['path']));mask=np.load(b['mask_path'])['mask'];cart=np.isin(mask,b['instance']['matched_instance_ids'])
        if cart.any():ax.contour(cart,levels=[.5],colors=['red'],linewidths=.7)
        apos=r['apos'];pixel=apos['pointing'].get('apos_px')
        if pixel is not None:ax.scatter(*pixel,marker='x',color='cyan',s=70)
        ax.set_title(f"{cid}: {apos['interpretation']}\nraster first hit: {apos['first_hit_class']}",fontsize=8);ax.axis('off')
    for ax in axs.ravel()[len(keys):]:ax.axis('off')
    fig.suptitle('Reported APOS cell / cart outline — diagnostic overlay, never sent to model',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.95));emit(fig,'terminal_affordance',{k:results[k]['apos'] for k in keys})
    mainkeys=[k for k in ('K0_OFF','K1','K2','K4','K8') if k in results]
    kvals=[results[k]['K'] for k in mainkeys]
    fig,axs=plt.subplots(1,3,figsize=(13,4))
    for component,label,ax in [(0,'APOS u (px)',axs[0]),(1,'APOS v (px)',axs[1])]:
        values=[None if results[k]['apos']['pointing'].get('apos_px') is None else results[k]['apos']['pointing']['apos_px'][component] for k in mainkeys]
        ax.plot(kvals,[np.nan if x is None else x for x in values],'o-');ax.set(xlabel='Visible-frame count K',ylabel=label)
    bearings=[results[k]['apos']['ground_bearing_rad'] for k in mainkeys]
    axs[2].plot(kvals,[np.nan if x is None else np.degrees(x) for x in bearings],'x--');axs[2].set(xlabel='K',ylabel='Ground bearing proxy (deg)')
    fig.suptitle('Clamped APOS: censored coordinate; dashed bearing is BOUNDARY_RAY_PROXY, not intended waypoint',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.92));emit(fig,'apos_vs_K',{k:results[k]['apos'] for k in mainkeys})
    source=read(run/'source03_input.json');cart=shapely.from_wkb(bytes.fromhex(source['projection']['obstacle_wkb_hex']))
    fig,ax=plt.subplots(figsize=(7,7));collection_plot(cart,ax=ax,color='black',alpha=.6,add_points=False)
    collection_plot(cart.buffer(.25),ax=ax,facecolor='none',edgecolor='gray',linestyle=':',add_points=False)
    for k in mainkeys:
        w=np.asarray(results[k]['world']);ax.plot(w[:,0],w[:,1],'-o',ms=3,label=k)
    pose=source['observation_pose_world'];ax.scatter(*pose[:2],marker='x',c='black',label='Observation pose')
    ax.arrow(pose[0],pose[1],.2*np.cos(pose[2]),.2*np.sin(pose[2]),width=.005,color='black')
    ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Raw returned paths — never executed\nCart actual mesh; dotted radius+.05m exclusion');ax.set_aspect('equal');ax.legend(fontsize=8)
    fig.tight_layout();emit(fig,'world_trajectory_overlay',{k:results[k] for k in mainkeys})
    fig,axs=plt.subplots(1,2,figsize=(12,4.5))
    for k in mainkeys:
        g=results[k]['geometry_on'];arcs=np.r_[0,np.cumsum(g['segment_lengths_m'])]
        axs[0].plot(range(g['row_count']),g['node_clearance_m'],'o-',label=k)
        axs[1].plot(arcs,g['node_clearance_m'],'o-',label=k)
    for ax in axs:ax.axhline(.05,c='black',ls='--');ax.axhline(0,c='gray');ax.set_ylabel('Footprint-edge clearance (m)');ax.legend(fontsize=8)
    axs[0].set_xlabel('Row index, not time');axs[1].set_xlabel('Arc from first returned row (m)')
    fig.suptitle('All paths evaluated with cart present; K0 is hypothetical ON geometry. Swept segments also checked',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.92));emit(fig,'row_arc_clearance',{k:results[k]['geometry_on'] for k in mainkeys})
    fig,axs=plt.subplots(1,3,figsize=(12,4))
    for ax,key,label in zip(axs,['max_abs_lateral_m','accumulated_abs_yaw_rad','arc'],['Max |lateral| (m)','Accumulated |yaw increment| (rad)','Returned arc (m)']):
        vals=[results[k]['geometry_on']['total_arc_m'] if key=='arc' else results[k]['motion'][key] for k in mainkeys]
        ax.plot(kvals,vals,'o-');ax.set(xlabel='K',ylabel=label)
    fig.suptitle('Untimed raw trajectory geometry');fig.tight_layout(rect=(0,0,1,.93));emit(fig,'geometry_vs_K',{k:results[k]['motion'] for k in mainkeys})
    allraw=np.concatenate([np.asarray(results[k]['raw_local']) for k in mainkeys])
    lateral_range=max(1.0,float(np.max(np.abs(allraw[:,1])))+.2)
    forward_range=max(1.6,float(allraw[:,0].max())+.2)
    fig,axs=plt.subplots(1,len(mainkeys),figsize=(3.2*len(mainkeys),5),squeeze=False)
    for ax,k in zip(axs[0],mainkeys):
        r=results[k];raw=np.asarray(r['raw_local']);ax.plot(raw[:,1],raw[:,0],'-o',ms=3,label='Raw trajectory')
        ground=r['apos']['ground_local_forward_left_m']
        if ground is not None:ax.plot([0,ground[1]],[0,ground[0]],'--',c='gray',label='Ground proxy')
        ax.scatter(0,0,c='black',marker='x');ax.set(title=k,xlabel='Local left (m)',ylabel='Forward (m)',xlim=(-lateral_range,lateral_range),ylim=(min(0,float(allraw[:,0].min())-.05),forward_range));ax.set_aspect('equal')
    axs[0,0].legend(fontsize=7);fig.suptitle('Affordance/action directions — clamped ground ray cannot establish intended free side',fontsize=11)
    fig.tight_layout(rect=(0,0,1,.93));emit(fig,'affordance_action_geometry',{k:results[k] for k in mainkeys})
    fig,ax=plt.subplots(figsize=(14,6));ax.axis('off');tab=ax.table(cellText=[[r['cause'],r['status']] for r in summary['evidence_matrix']],colLabels=['Candidate cause','Evidence status'],loc='center',cellLoc='left');tab.auto_set_font_size(False);tab.set_fontsize(9);tab.scale(1,1.8)
    fig.suptitle(summary['persistence']['classification']+'\n'+summary['apos_classification'],fontsize=11);fig.tight_layout();emit(fig,'evidence_matrix',summary)
    rows=[]
    for x in ledger:
        cid=x['condition'];row=dict(condition=cid,K=KS[cid],status=x['status'])
        if cid in results:
            r=results[cid];row.update(clearance_on_m=r['geometry_on']['whole']['minimum_clearance_m'],clearance_off_m=r['geometry_off']['whole']['minimum_clearance_m'],
                safe_on=r['geometry_on']['whole']['clearance_valid'],N=len(r['raw_local']),first_unsafe_row=r['geometry_on']['first_unsafe_waypoint_zero_based'],
                first_unsafe_segment=r['geometry_on']['first_unsafe_segment_zero_based'],arc_m=r['geometry_on']['total_arc_m'],
                max_lateral_m=r['motion']['max_abs_lateral_m'],final_yaw_rad=r['motion']['final_yaw_rad'],
                apos_px=r['apos']['pointing'].get('apos_px'),apos_clamped=r['apos']['pointing'].get('apos_clamped'),
                hit_class=r['apos']['first_hit_class'],ground_bearing_rad=r['apos']['ground_bearing_rad'],agreement=r['affordance_action_category'],
                visible=r['target_visible'],stop=r['stop'],RTT_s=r['client_rtt_s'])
        rows.append(row)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (run/'aggregate/outcomes.csv').open('x',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    save(out/'figure_manifest.json',figures)
    html='<html><meta charset="utf-8"><h1>JOIN-SOURCE-04</h1><p>'+LABEL+'</p><pre>'+json.dumps(summary,indent=2)+'</pre>'
    for f in figures:html+=f'<h2>{f["png"]}</h2><img style="max-width:100%" src="{f["png"]}"><p><a href="{f["sidecar"]}">numbers/source</a></p>'
    (out/'index.html').write_text(html+'</html>')
    with (run/'index.html').open('x') as f:f.write('<a href="review/index.html">Saved-only persistence and affordance diagnostic review</a>')
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for f in sorted(out.iterdir()):z.write(f,str(f.relative_to(run)))
        for name in ['protocol.json','aggregate/summary.json','aggregate/outcomes.csv','config_snapshot.yaml','mpc_audit.json']:
            z.write(run/name,name)


if __name__=='__main__':main()
