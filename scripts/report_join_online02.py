#!/usr/bin/env python3
"""Static saved-record presentation; no acquisition or trajectory generation."""
import argparse,html,json,math,sys,zipfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from run_join_online02 import read,save,sha,environments
from analyze_join_online02 import csvread,pose_rows
COLORS=['#2781b5','#d12995','#33966c','#de8128']

def presentation(run):
    result=read(run/'aggregate/analysis.json');out=run/'review';out.mkdir(exist_ok=False)
    base,on,cart,_=environments(run);scenario=read(run/'scenario.json')
    source={str(p):sha(p) for p in [run/'aggregate/analysis.json',run/'source.json',run/'scenario.json',run/'config_snapshot.yaml',run/'protocol.json',*sorted((run/'episodes').glob('*/completion.json'))]}
    entries=[]
    def finish(fig,name,data):
        fig.tight_layout();p=out/(name+'.png');fig.savefig(p,dpi=160);plt.close(fig)
        save(p.with_suffix('.json'),dict(source_hashes=source,data=data,image_sha256=sha(p),label='ACTUAL ONLINE EXECUTION; original predicted chunks drawn separately'))
        entries.append(name)
    def background(ax):
        for geom in getattr(cart,'geoms',[cart]):
            if hasattr(geom,'exterior'):
                xy=np.array(geom.exterior.coords);ax.fill(xy[:,0],xy[:,1],color='saddlebrown',alpha=.75)
        inflated=cart.buffer(.25)
        for geom in getattr(inflated,'geoms',[inflated]):
            if hasattr(geom,'exterior'):
                xy=np.array(geom.exterior.coords);ax.plot(xy[:,0],xy[:,1],':',color='brown',lw=1)
        ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.2)
    episodes=result['episodes']
    for ep in episodes:
        eid=ep['summary']['episode'];rows=ep['rows'];poses=pose_rows(csvread(run/'episodes'/eid/'execution.csv'))
        fig,ax=plt.subplots(figsize=(8,7));background(ax)
        for i,r in enumerate(rows):
            if 'world' not in r:continue
            p=np.asarray(r['world']);ax.plot(p[:,0],p[:,1],'--',lw=1,alpha=.6,label='raw predicted futures' if i==0 else None)
            ob=r['observation'];ax.scatter(*ob[:2],s=13,c='purple');ax.annotate(r['display_id'],ob[:2],fontsize=7)
        ax.plot(poses[:,0],poses[:,1],color='#424242',lw=2.5,label='actually executed')
        ax.scatter(*poses[0,:2],marker='s',c='green',label='initial state')
        ax.scatter(*poses[-1,:2],marker='x',c='red',label='last applied state')
        ax.set_title(eid+'\n'+ep['summary']['outcome'],fontsize=10)
        ax.set_xlim(scenario['center_xy'][0]-1.6,scenario['center_xy'][0]+1.6)
        ax.legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1))
        if eid.startswith('OFF'):ax.text(.02,.02,'Cart outline: hypothetical only; absent during OFF',transform=ax.transAxes,fontsize=8)
        finish(fig,eid+'_world',dict(rows=rows,execution=poses.tolist(),summary=ep['summary']))
        # Every original trigger plus actual raw world polyline, paginated, no cherry picking.
        for page,start in enumerate(range(0,len(rows),6)):
            subset=rows[start:start+6];fig,axes=plt.subplots(len(subset),2,figsize=(12,3.3*len(subset)),squeeze=False)
            for r,(a,b) in zip(subset,axes):
                a.imshow(Image.open(r['RGB_path']));a.axis('off');a.set_title(f"{r['display_id']} saved RGB; t_obs={r['t_obs']:.3f}s",fontsize=10)
                background(b)
                if 'world' in r:
                    p=np.asarray(r['world']);b.plot(p[:,0],p[:,1],'m.--',label='raw FRESH')
                    ob=r['observation'];b.scatter(*ob[:2],c='orange',label='observation')
                    if r['B'] is not None:b.scatter(*r['B'][:2],c='black',marker='x',label='B')
                    b.set_title(r['classification']+f"; edge={r['raw_geometry']['whole']['minimum_clearance_m']:.3f}m",fontsize=9)
                else:b.set_title('N/A: technical unavailable')
                b.set_xlim(scenario['center_xy'][0]-1.6,scenario['center_xy'][0]+1.6)
                b.legend(fontsize=7,loc='upper left',bbox_to_anchor=(1.02,1))
            finish(fig,f'{eid}_chunks_{page:02}',subset)
        for kind,key in [('first_onset','first_onset'),('first_bypass','first_bypass')]:
            event=ep['summary'][key]
            if not event:continue
            r=next(r for r in rows if r['chunk_id']==event['chunk_id']);fig,ax=plt.subplots(figsize=(7,6));background(ax)
            old=next((x for x in rows if x['chunk_id']==r['old_chunk_id']),None)
            for x,c,label in [(old,'blue','OLD'),(r,'magenta','FRESH')]:
                if x is not None:
                    p=np.array(x['world']);ax.plot(p[:,0],p[:,1],'--',color=c,label=label)
            if r['B'] is not None:ax.scatter(*r['B'][:2],c='black',label='B')
            ax.plot(poses[:,0],poses[:,1],color='gray',label='actual execution')
            ax.set_title(eid+' '+kind);ax.legend();finish(fig,eid+'_'+kind,r)
    for name,field,unit in [('clearance','minimum_clearance_m','Raw minimum edge clearance (m)'),('lateral','max_lateral_m','Max |lateral| from hallway centre (m)'),('arc','raw_arc_m','Raw returned XY arc (m)')]:
        fig,ax=plt.subplots(figsize=(9,5));data=[]
        for ep,color in zip(episodes,COLORS):
            rows=[r for r in ep['rows'] if 'world' in r];x=[r['observation_cart_center_distance_m'] for r in rows]
            y=[r['raw_geometry']['whole'][field] if name=='clearance' else r[field] for r in rows]
            ax.scatter(x,y,label=ep['summary']['episode'],c=color)
            for xx,yy,r in zip(x,y,rows):ax.annotate(r['display_id'],(xx,yy),fontsize=7)
            data.extend(dict(episode=r['episode'],chunk=r['chunk_id'],x=xx,y=yy) for xx,yy,r in zip(x,y,rows))
        if name=='clearance':ax.axhline(.05,color='red',ls=':',label='required .05m');ax.axhline(0,color='black',lw=.5)
        ax.invert_xaxis();ax.set_xlabel('Observation to cart centre distance (m), approaching →');ax.set_ylabel(unit)
        ax.set_title('OFF raw paths checked against hypothetical cart ON');ax.legend(fontsize=8);ax.grid(alpha=.2)
        finish(fig,name+'_vs_distance',data)
    labels=['STRAIGHT_OR_BASELINE','UNSAFE_INTERSECTING','SAFE_SHORTEN','SAFE_STOP','SAFE_BYPASS_ONSET','SAFE_BYPASS','PAST_OBSTACLE','TECHNICAL_UNAVAILABLE']
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=False)
    for ax,ep in zip(axes,episodes):
        for r in ep['rows']:
            ax.scatter(r['t_obs'],labels.index(r['classification']),c='gray' if not r['received_before_end'] else 'magenta')
            ax.annotate(r['display_id'],(r['t_obs'],labels.index(r['classification'])),fontsize=8)
        ax.set_yticks(range(len(labels)),[x.replace('SAFE_','S_') for x in labels],fontsize=7);ax.set_title(ep['summary']['episode']);ax.set_xlabel('Saved observation simulation time (s)');ax.grid(alpha=.2)
    finish(fig,'classification_timeline',episodes)
    fig,axes=plt.subplots(1,3,figsize=(15,4))
    for ax,key,title in zip(axes,['max_lateral_m','raw_arc_m','final_yaw_relative_hallway_rad'],['Lateral (m)','Arc (m)','Final yaw wrt hallway (rad)']):
        for ep,color in zip(episodes,COLORS):
            rows=[r for r in ep['rows'] if key in r]
            ax.scatter([r['observation_longitudinal_m'] for r in rows],[r[key] for r in rows],label=ep['summary']['episode'],color=color)
        ax.set_xlabel('Observation longitudinal position relative to cart (m)');ax.set_ylabel(title);ax.grid(alpha=.2)
    axes[0].legend(fontsize=7);finish(fig,'OFF_ON_longitudinal',dict(episodes=episodes,matches=result['OFF_matches']))
    table='<table><tr><th>Episode</th><th>Chunk</th><th>Class</th><th>obs / apply s</th><th>clearance m</th><th>arc m</th><th>lateral m</th></tr>'
    for ep in episodes:
        for r in ep['rows']:
            fmt=lambda v:'N/A' if v is None else f'{v:.4f}'
            values=[r['episode'],r['display_id'],r['classification'],fmt(r['t_obs'])+' / '+fmt(r['t_apply']),fmt((r.get('raw_geometry') or {}).get('whole',{}).get('minimum_clearance_m')),fmt(r.get('raw_arc_m')),fmt(r.get('max_lateral_m'))]
            table+='<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+'</tr>'
    table+='</table>'
    timing=[dict(episode=ep['summary']['episode'],whole_RTF=ep['summary']['episode_rtf'],maximum_loop_stall_s=ep['summary']['max_loop_stall_s'],prediction_RTF=[r.get('inflight_rtf') for r in ep['rows']]) for ep in episodes]
    save(out/'timing_and_calls.json',dict(timing=timing,calls=read(run/'aggregate/call_counts.json'),source_hashes=source))
    text='<html><meta charset="utf-8"><style>body{font-family:sans-serif;margin:30px}img{max-width:100%}td,th{border:1px solid #aaa;padding:5px}table{border-collapse:collapse}</style><h1>JOIN-ONLINE-02</h1><p>ACTUAL ONLINE EXECUTION / saved diagnostic presentation. Cart OFF outlines are hypothetical. No GP or reconciliation.</p><h2>'+result['overall']+'</h2>'
    text+='<p>Timing limitation: whole-episode RTF is distinct from request-local RTF. See <a href="timing_and_calls.json">all measured timing and call counts</a>. No timing-qualified source is claimed. A model STOP ends acquisition; no post-STOP traversal is invented.</p>'
    text+='<p>The frozen SAFE_SHORTEN category additionally requires a spatial influence/arc gate. Measured shortening can occur while that label remains STRAIGHT_OR_BASELINE; thresholds were not changed after results. First onset/full-bypass N/A remain N/A.</p>'+table
    for ep in episodes:
        if not ep['summary']['first_onset']:text+='<p>'+ep['summary']['episode']+': first bypass-onset N/A.</p>'
        if not ep['summary']['first_bypass']:text+='<p>'+ep['summary']['episode']+': first full bypass N/A.</p>'
    text+=''.join(f'<h2>{n}</h2><a href="{n}.json">numeric/source sidecar</a><br><img src="{n}.png">' for n in entries)+'</html>'
    (out/'index.html').write_text(text)
    save(out/'plot_manifest.json',dict(figures=[dict(name=n,png_sha256=sha(out/(n+'.png')),sidecar_sha256=sha(out/(n+'.json'))) for n in entries],source_hashes=source))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [*sorted(out.glob('*')),run/'aggregate/chunks.csv',run/'aggregate/call_counts.json',run/'protocol.json',run/'source.json',run/'scenario.json',run/'config_snapshot.yaml']:
            z.write(p,p.relative_to(run))
    print(out/'index.html')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();presentation(a.run.resolve())
