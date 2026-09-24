#!/usr/bin/env python3
"""Static source review; no new model/controller or synthetic scientific curves."""
import argparse
from pathlib import Path
import sys
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from analyze_join_online02 import jsonlines,csvread,pose_rows
from run_join_online02 import environments

def geometries(ax,obj,**kw):
    if obj.is_empty:return
    if obj.geom_type=='Polygon':
        x,y=obj.exterior.xy;ax.fill(x,y,**kw)
    elif hasattr(obj,'geoms'):
        for sub in obj.geoms:geometries(ax,sub,**kw)
    elif obj.geom_type in ('LineString','LinearRing'):
        x,y=obj.xy;ax.plot(x,y,color=kw.get('color','gray'),alpha=kw.get('alpha',1))


def payload(run,row):
    cid=row['candidate_id'];ep=run/'candidates'/cid/'episodes'/cid
    captures=jsonlines(ep/'capture.jsonl');vis=jsonlines(ep/'visibility.jsonl');poses=pose_rows(csvread(ep/'execution.csv'))
    return dict(candidate_id=cid,poses_world=poses.tolist(),visibility=vis,captures=captures,
        OLD=row['OLD'],FRESH=row['FRESH'],B=row['B'],context=row['context'],crossing=row['crossing'],bootstrap=row['bootstrap'],
        source_hashes=row['raw_source_hashes'],qualification_sha256=sha(run/'validation.json'),scope='GENUINE SOURCE ACQUISITION ONLY; no complete bypass demonstrated')


def report(run):
    v=read(run/'validation.json');out=run/'review';out.mkdir(exist_ok=False);panels=[]
    for row in v['rows']:
        cid=row['candidate_id'];p=payload(run,row);ep=run/'candidates'/cid/'episodes'/cid
        base,on,cart,_=environments(run/'candidates'/cid);c=next(r['candidate'] for r in v['geometry']['rows'] if r['candidate_id']==cid)
        fig,ax=plt.subplots(figsize=(8,7));geometries(ax,base.obstacles,color='gray',alpha=.5);geometries(ax,cart,color='orange',alpha=.8);geometries(ax,cart.buffer(.25),color='orange',alpha=.15)
        poses=np.array(p['poses_world']);ax.plot(poses[:,0],poses[:,1],color='black',lw=2,label='Actual OLD execution / source postroll')
        for label,color in [('OLD','royalblue'),('FRESH','crimson')]:
            r=p[label]
            if r and 'world' in r:
                a=np.array(r['world']);ax.plot(a[:,0],a[:,1],'--o',ms=3,color=color,label=f'Original {label} prediction (not executed path)')
                obs=np.array(r['observation']['pose_world']);ax.scatter(*obs[:2],c=color,marker='x');ax.arrow(*obs[:2],.35*np.cos(obs[2]),.35*np.sin(obs[2]),color=color,width=.01)
        if p['B']:ax.scatter(*p['B']['pose_world'][:2],marker='*',s=150,c='green',label='Actual B')
        else:ax.text(.02,.98,'No applied FRESH / no B' if not p['B'] else '',transform=ax.transAxes,va='top')
        xy=np.vstack([poses[:,:2],c['cart_center_xy'],c['approach_pose'][:2]]);lo=xy.min(axis=0)-.8;hi=xy.max(axis=0)+.8
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title=f'{cid}: static cart + real wall occlusion\n{row["termination"]}; qualified={row["qualified"]}')
        ax.set_aspect('equal');ax.legend(fontsize=7);fig.tight_layout();name=cid+'_scene_handoff';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig);save(out/(name+'.json'),p);panels.append(name)
        cross=row['crossing']
        ids=([cross['previous']['frame_id'],cross['current']['frame_id']] if cross else [captures['frame_id'] for captures in [p['captures'][0],p['captures'][-1]]])
        fig,axes=plt.subplots(1,2,figsize=(12,4))
        for ax,fid in zip(axes,ids):
            cap=next(r for r in p['captures'] if r['frame_id']==fid);vr=next(r for r in p['visibility'] if r['frame_id']==fid)
            ax.imshow(Image.open(ep/cap['path']));ax.axis('off');ax.set_title(f'{fid}: {vr["instance"]["visible_pixels"]} cart pixels\n'+('Hidden / first visible pair' if cross else 'No crossing: first / final captures'))
        fig.tight_layout();name=cid+'_visibility_RGB';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig);save(out/(name+'.json'),dict(frame_ids=ids,**p));panels.append(name)
        fig,axes=plt.subplots(2,1,figsize=(10,6));t=np.array([r['sim_time_s'] for r in p['visibility']]);px=[r['instance']['visible_pixels'] for r in p['visibility']]
        axes[0].plot(t,px,'o-');axes[0].axhline(20,color='red',ls='--');axes[0].set(ylabel='Cart instance pixels',xlabel='Simulation time [s]')
        baseline=p['captures'][0]['capture_monotonic_ns']/1e9
        stamps=[]
        for label in ['OLD','FRESH']:
            r=p[label]
            if r:
                stamps.append((label+' obs',r['observation']['capture_monotonic_ns']/1e9))
                for key in ['request','receipt']:
                    if r.get(key):stamps.append((label+' '+key,r[key]['monotonic_ns']/1e9))
        if p['context']:
            for key in ['t_ready_seen_sim','t_install','t_switch']:
                if p['context'].get(key):stamps.append((key,p['context'][key]['host_monotonic_s']))
        for i,(label,stamp) in enumerate(stamps):axes[1].scatter(stamp-baseline,i);axes[1].text(stamp-baseline,i+.1,label,fontsize=8)
        axes[1].set(xlabel='Host monotonic seconds since first capture (same clock)',yticks=[],title='Observation / receipt / install / application remain separate')
        fig.tight_layout();name=cid+'_timing';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig);save(out/(name+'.json'),dict(host_events=stamps,host_origin_s=baseline,**p));panels.append(name)
    rows=[dict(candidate_id=r['candidate_id'],geometry_qualified=r['qualified'],pixels=r['pixels'],failures=r['failure_reasons']) for r in v['geometry']['rows']]
    save(out/'summary.json',dict(classification=v['classification'],geometry=rows,scientific=[{k:r[k] for k in ['candidate_id','qualified','failure_reasons','calls','timing','termination']} for r in v['rows']]))
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Blind corner source</title><h1>'+v['classification']+'</h1><p>Static cart; genuine online acquisition. Prediction paths are not executed paths. No complete bypass or optimization claim.</p>'+''.join(f'<h2>{n}</h2><a href="{n}.json">Numeric/hash sidecar</a><br><img style="max-width:100%" src="{n}.png">' for n in panels))
    for row in v['rows']:
        p=payload(run,row)
        for suffix in ['scene_handoff','visibility_RGB','timing']:
            side=read(out/(row['candidate_id']+'_'+suffix+'.json'))
            assert all(side[k]==value for k,value in p.items()),'plot numeric parity'
    save(run/'artifact_validation.json',dict(valid=True,numeric_sidecar_parity=True,figures={p.name:sha(p) for p in out.glob('*.png')},model_MPC_calls=0))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
