#!/usr/bin/env python3
"""Saved-only SOURCE05 tables, numeric-linked figures and compact review."""
import argparse
import csv
import html
from pathlib import Path
import re
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
from reconciliation.join_source05 import KS,ORDER,INSTRUCTIONS,summarize,historical_id
from run_join_source05 import source_run
from join_source02_paired import wire_parity
from join_source03_presentation import collection_plot


def load_results(run):
    source=source_run(run)
    historical={historical_id(k):read(source/'predictions'/historical_id(k)/'evaluation.json') for k in KS}
    results={c:read(run/'predictions'/c/'evaluation.json') for c in ORDER if (run/'predictions'/c/'evaluation.json').exists()}
    return results,historical,read(source/'source03_input.json')


def rows(results,historical):
    out=[]
    for cid,r in [(f'I0_K{k}',historical[historical_id(k)]) for k in KS]+list(results.items()):
        g=r['geometry_on'];m=r['motion'];p=r['apos']['pointing'];i=cid[:2];k=r['K']
        out.append(dict(condition=cid,instruction=i,K=k,provenance='historical SOURCE04' if i=='I0' else 'new SOURCE05',
            status=r['status'],safe_actual_scene=(g if k else r['geometry_off'])['whole']['clearance_valid'],
            safe_cart_on=g['whole']['clearance_valid'],clearance_cart_on_m=g['whole']['minimum_clearance_m'],
            clearance_off_m=r['geometry_off']['whole']['minimum_clearance_m'],N=len(r['raw_local']),
            first_unsafe_row=g['first_unsafe_waypoint_zero_based'],first_unsafe_segment=g['first_unsafe_segment_zero_based'],
            safe_prefix_arc_upper_m=None if g['safe_prefix_boundary'] is None else g['safe_prefix_boundary']['distance_from_first_row_m'][1],
            arc_m=g['total_arc_m'],max_lateral_m=m['max_abs_lateral_m'],final_lateral_m=m['final_lateral_m'],
            final_yaw_rad=m['final_yaw_rad'],initial_tangent_rad=m['initial_tangent_rad'],
            hallway_endpoint_forward_m=r['raw_local'][-1][0],endpoint_beyond_cart_rear_m=m['endpoint_beyond_rear_m'],
            apos_pixel=str(p.get('apos_px')),apos_clamped=p.get('apos_clamped'),opos_state=p.get('opos_state'),
            visible=r.get('visible',r.get('target_visible')),stop=r['stop'],RTT_s=r['client_rtt_s'],
            action_tokens='/'.join(re.findall(r'<act_l\d+_(\d+)>',r['raw_text'])),raw_sha256=r['raw_sha256']))
    return out


def write_csv(path,records):
    with Path(path).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)


def call_counts(run,results):
    predictions=buffer=0
    for cid in ORDER:
        p=run/'predictions'/cid
        if (p/'replayed_inputs.json').exists():
            w=wire_parity(p,read(p/'replayed_inputs.json')['frames'])
            predictions+=w.get('actual_terminal_predictions_sent',0)
            buffer+=w.get('actual_next_requests_sent',0)-w.get('actual_terminal_predictions_sent',0)
    server=read(run/'server_process.json');stop=read(run/'server_shutdown.json')
    warmups=re.findall(r'\[lightnav-ws\] warmup done in (\d+) ms',(run/'logs/server.log').read_text())
    return dict(scientific_terminal_predictions=predictions,buffer_only_requests=buffer,
        independent_completed_sessions=len(results),official_synthetic_startup_warmups=len(warmups),
        warmup_ms=list(map(int,warmups)),test_real_model_MPC_calls=0,new_MPC_GP_rigid_reconciliation_rollout_render_calls=0,
        server_startup_s=(server['ready_event']['monotonic_ns']-server['start_event']['monotonic_ns'])/1e9,
        server_lifetime_s=(stop['exit_observed_event']['monotonic_ns']-server['start_event']['monotonic_ns'])/1e9,
        terminal_RTT_sum_s=sum(r['client_rtt_s'] for r in results.values()),worker_wall_sum_s=sum(r['worker_wall_s'] for r in results.values()),
        timing_caveat='nested startup/RTT/worker/server times are not additive; warmup is synthetic official infrastructure, not evidence')


def render(run):
    results,historical,context=load_results(run);out=run/'review';out.mkdir(exist_ok=False)
    summary=summarize(results,historical,context['influence']);save(run/'aggregate/summary.json',summary)
    save(run/'aggregate/actual_call_counts.json',call_counts(run,results))
    table=rows(results,historical);write_csv(run/'aggregate/outcomes.csv',table)
    allr={f'I0_K{k}':historical[historical_id(k)] for k in KS};allr.update(results)
    source=source_run(run);bank=read(source/'bank_manifest.json');cart=shapely.from_wkb(bytes.fromhex(context['projection']['obstacle_wkb_hex']))
    inputs={str(run/'predictions'/c/'evaluation.json'):sha(run/'predictions'/c/'evaluation.json') for c in results}
    inputs.update({str(source/'predictions'/historical_id(k)/'evaluation.json'):sha(source/'predictions'/historical_id(k)/'evaluation.json') for k in KS})
    figures=[]
    def emit(fig,name):
        fig.savefig(out/f'{name}.png',dpi=150);plt.close(fig)
        save(out/f'{name}.json',dict(numbers=table,summary=summary,input_sha256=inputs,
            frame_bank_sha256=sha(source/'bank_manifest.json'),config_sha256=sha(run/'config_snapshot.yaml'),
            source_sha256=sha(__file__),scope='RAW PREDICTIONS ONLY / NOT EXECUTED'))
        figures.append(dict(png=f'{name}.png',sidecar=f'{name}.json',sha256=sha(out/f'{name}.png')))
    colors={'I0':'#555555','I1':'#006bb3','I2':'#d35400'}
    fig,axs=plt.subplots(1,3,figsize=(15,5))
    import textwrap
    for ax,i in zip(axs,INSTRUCTIONS):
        ax.imshow(Image.open(bank[-1]['ON']['frame']['path']));ax.axis('off')
        ax.set_title(i+(' — historical' if i=='I0' else ' — new')+'\n'+textwrap.fill(INSTRUCTIONS[i],36),fontsize=10)
    fig.suptitle('Identical SOURCE04 terminal JPEG; instruction text outside image\nHistory also byte-identical for matched K',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.88));emit(fig,'same_RGB_instructions')
    allxy=np.concatenate([np.asarray(r['world'])[:,:2] for r in allr.values()]+[np.asarray(cart.convex_hull.exterior.coords)])
    low=allxy.min(axis=0)-.3;high=allxy.max(axis=0)+.3
    for k in KS:
        fig,ax=plt.subplots(figsize=(8,7));collection_plot(cart,ax=ax,color='black',alpha=.5,add_points=False)
        collection_plot(cart.buffer(.25),ax=ax,facecolor='none',edgecolor='gray',linestyle=':',add_points=False)
        for i in INSTRUCTIONS:
            r=allr.get(f'{i}_K{k}')
            if r is None:continue
            w=np.asarray(r['world']);ax.plot(w[:,0],w[:,1],'-o',ms=3,color=colors[i],label=i+(' historical' if i=='I0' else ' new'))
        pose=context['observation_pose_world'];ax.scatter(*pose[:2],c='black',marker='x',label='Observation (no B)')
        ax.arrow(pose[0],pose[1],.2*np.cos(pose[2]),.2*np.sin(pose[2]),width=.004,color='black')
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',xlim=(low[0],high[0]),ylim=(low[1],high[1]),
               title=f'K{k}: raw returned paths, NOT EXECUTED\n'+('Cart outline hypothetical; cart OFF input' if k==0 else 'Actual cart footprint + dotted radius .20m / margin .05m'))
        ax.set_aspect('equal');ax.legend(loc='best');fig.tight_layout();emit(fig,f'trajectory_K{k}')
    def series(ax,key,label):
        for i in INSTRUCTIONS:
            rr=[next((r for r in table if r['condition']==f'{i}_K{k}'),None) for k in KS]
            ax.plot(KS,[np.nan if r is None or r[key] is None else r[key] for r in rr],'o-',color=colors[i],label=i+(' historical' if i=='I0' else ' new'))
        ax.set(xlabel='Cart-present frames K (H=16)',ylabel=label);ax.set_xticks(KS);ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig,ax=plt.subplots(figsize=(8,5));series(ax,'clearance_cart_on_m','Minimum footprint-edge clearance (m)')
    ax.axhline(.05,c='black',ls='--',label='Required .05m');ax.axhline(0,c='gray',ls=':');ax.legend(fontsize=8)
    ax.set_title('Full swept raw path; K0 evaluated with hypothetical cart ON');fig.tight_layout();emit(fig,'clearance_vs_K')
    fig,ax=plt.subplots(figsize=(8,5));series(ax,'max_lateral_m','Maximum absolute local lateral (m)');ax.axhline(.20,c='gray',ls='--')
    ax.set_title('Lateral magnitude alone is not safe avoidance');fig.tight_layout();emit(fig,'lateral_vs_K')
    fig,axs=plt.subplots(1,2,figsize=(12,4.5));series(axs[0],'final_yaw_rad','Final local yaw (rad)');series(axs[1],'arc_m','Raw XY arc (m)')
    fig.suptitle('Untimed raw geometry — no waypoint clock');fig.tight_layout();emit(fig,'yaw_arc_vs_K')
    fig,ax=plt.subplots(figsize=(13,7));ax.axis('off')
    cells=[[r['condition'],r['action_tokens'],str(r['safe_cart_on']),f"{r['clearance_cart_on_m']:.6f}",r['apos_pixel'],str(r['apos_clamped'])] for r in table]
    t=ax.table(cellText=cells,colLabels=['Condition','RVQ l0/l1/l2','Safe cart ON','Edge m','APOS px','Clamped'],loc='center');t.auto_set_font_size(False);t.set_fontsize(9);t.scale(1,1.55)
    ax.set_title('Literal action / pointing output; I0 historical, I1/I2 new');fig.tight_layout();emit(fig,'tokens_and_APOS')
    fig,axs=plt.subplots(1,2,figsize=(12,5))
    for i in INSTRUCTIONS:
        for component,ax in enumerate(axs):
            vals=[allr.get(f'{i}_K{k}',{}).get('apos',{}).get('pointing',{}).get('apos_px') for k in KS]
            ax.plot(KS,[np.nan if v is None else v[component] for v in vals],'o-',color=colors[i],label=i)
            ax.set(xlabel='K',ylabel=('APOS u (px)' if component==0 else 'APOS v (px)'));ax.legend();ax.set_xticks(KS)
    fig.suptitle('Clamped coordinates are CENSORED_BY_CLAMPING, not exact metric targets');fig.tight_layout();emit(fig,'APOS_vs_K')
    fig,axs=plt.subplots(1,2,figsize=(13,7))
    for ax,i in zip(axs,('I1','I2')):
        ax.axis('off');decision=summary['instructions'][i]
        cells=[]
        for k,c in decision.get('comparisons',{}).items():
            cells.append([k,c['safe_on'],c['versus_same_instruction_K0']['meaningful'],c['visual_conditioned_safe_detour'],c['full_bypass']])
        if cells:
            t=ax.table(cellText=cells,colLabels=['K','Safe','Visual change','Safe detour','Bypass'],loc='center');t.auto_set_font_size(False);t.set_fontsize(9);t.scale(1,2)
        ax.set_title(i+'\n'+textwrap.fill(decision['classification'],42),fontsize=10)
    fig.suptitle(textwrap.fill(summary['classification'],80)+'\nSame-input shams and historical comparison saved separately',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.90));emit(fig,'instruction_effect_matrix')
    save(out/'figure_manifest.json',figures)
    header='<meta charset="utf-8"><title>JOIN-SOURCE-05</title><style>body{font-family:sans-serif;max-width:1200px;margin:auto}img{max-width:100%}td,th{padding:5px;border:1px solid #ccc}table{border-collapse:collapse}</style>'
    body='<h1>JOIN-SOURCE-05 — '+html.escape(summary['classification'])+'</h1><p>Counterfactual input history. No online B, traversal, MPC or reconciliation. I0 historical; I1/I2 new. No model input annotations.</p>'
    body+='<p>New instructions also change shelf-goal semantics to hallway progress. APOS/OPOS are observable tokens, not proof of obstacle recognition.</p>'
    body+='<table><tr>'+''.join('<th>'+x+'</th>' for x in ['condition','safe actual','safe cart ON','clearance m','lateral m','yaw rad'])+'</tr>'
    for r in table:
        body+='<tr>'+''.join('<td>'+html.escape(str(r[x]))+'</td>' for x in ['condition','safe_actual_scene','safe_cart_on','clearance_cart_on_m','max_lateral_m','final_yaw_rad'])+'</tr>'
    body+='</table>'
    body+=''.join(f'<h2>{x["png"]}</h2><img src="review/{x["png"]}"><p><a href="review/{x["sidecar"]}">Numbers and source hashes</a></p>' for x in figures)
    with (run/'index.html').open('x') as f:f.write(header+body)
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [run/'index.html',run/'protocol.json',run/'config_snapshot.yaml',run/'source.json',run/'generation_actual.json',run/'aggregate/outcomes.csv',run/'aggregate/summary.json',run/'aggregate/actual_call_counts.json',*out.iterdir()]:
            if p.is_file():z.write(p,str(p.relative_to(run)))
    print(summary['classification'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();render(a.run.resolve())
