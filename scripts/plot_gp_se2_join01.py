#!/usr/bin/env python3
"""Attachment-focused static evidence; never draws an unavailable execution."""
import argparse,csv,html,json,os,sys,zipfile
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/join01-mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from run_gp_se2_join01 import read,write,digest,load_case,verify
from reconciliation.gp_se2_join01 import METHODS,box_polygon


def draw_geometry(ax,geom,**kwargs):
    if geom.is_empty:return
    if hasattr(geom,'geoms'):
        for g in geom.geoms:draw_geometry(ax,g,**kwargs)
    elif geom.geom_type=='Polygon':
        xy=np.asarray(geom.exterior.coords);ax.fill(xy[:,0],xy[:,1],**kwargs)
    elif hasattr(geom,'coords'):
        xy=np.asarray(geom.coords);ax.plot(xy[:,0],xy[:,1],color=kwargs.get('edgecolor','#777'))


def past(source,episode,Bstate):
    rows=list(csv.DictReader((source/'episodes'/episode/'execution.csv').open()))
    return [[float(r[k]) for k in ('x','y','yaw')] for r in rows if int(r['state_id'])<=Bstate]


def render(run):
    source=verify(run);out=run/'plots';out.mkdir(exist_ok=False)
    figs=[];records=[]
    def save(name,fig,data):
        fig.tight_layout();path=out/(name+'.png');fig.savefig(path,dpi=160);plt.close(fig)
        write(path.with_suffix('.json'),dict(image_sha256=digest(path),source_sha256=digest(run/'source.json'),
            protocol_sha256=digest(run/'protocol.json'),numeric=data))
        figs.append(path)
    collection=read(run/'collection_result.json')
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from shapely.geometry import box
    base=HospitalEnvironment.load(source['environment'])
    for attempt in collection['attempts']:
        ep=attempt['placement'];q=read(run/'qualification'/(ep+'.json'));fig,ax=plt.subplots(figsize=(8,7))
        if 'context' in q:
            ctx=q['context'];old=np.asarray(ctx['old_world']);fresh=np.asarray(ctx['fresh_world']);b=np.asarray(ctx['B_world'])
            prior=np.asarray(past(run/'source_event',ep,ctx['switch_state_id']))
            points=np.vstack([old[:,:2],fresh[:,:2],prior[:,:2]])
            lo=points.min(0)-.6;hi=points.max(0)+.6;clip=box(*lo,*hi)
            draw_geometry(ax,base.workspace.intersection(clip),facecolor='#f4f8ed',edgecolor='#ccc')
            draw_geometry(ax,base.obstacles.intersection(clip),facecolor='#777',alpha=.7)
            ax.plot(old[:,0],old[:,1],':',color='#3476ac',label='original OLD')
            ax.plot(fresh[:,0],fresh[:,1],'--',color='#a44397',label='original FRESH')
            ax.plot(prior[:,0],prior[:,1],color='#777',label='actually executed OLD prefix')
            ax.scatter(*b[:2],color='black',label='B at FRESH command application')
            ax.scatter(*fresh[-1,:2],marker='*',color='green',s=80,label='original goal')
            obstacle=box_polygon(q['obstacle']);draw_geometry(ax,obstacle,facecolor='#d55e00',alpha=.6)
            xx,yy=obstacle.buffer(.25).exterior.xy;ax.plot(xx,yy,':',color='#c34311',label='footprint + required clearance outline')
            for kind in ['old','fresh']:
                record=read(run/'source_event'/'episodes'/ep/'handoffs/handoff_000/context.json')
                captures=[json.loads(line) for line in (run/'source_event'/'episodes'/ep/'capture.jsonl').read_text().splitlines()]
                fid=record['fresh_observation_pose_time']['frame_id'] if kind=='fresh' else next((r['frame_id'] for r in captures if r['capture_sim_time_s']==record['old_observation_pose_time']['time']),None)
                if fid is not None:
                    records.append(dict(episode=ep,kind=kind,rgb=str(Path('../source_event/episodes')/ep/'rgb'/(fid+'.jpg'))))
            ax.legend(fontsize=8);ax.set_aspect('equal');ax.set(xlabel='world x [m]',ylabel='world y [m]',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),title=ep+' — genuine source qualification')
            save(ep+'_source_handoff',fig,q)
        else:
            ax.axis('off');ax.text(.05,.8,'No qualified source record\n'+q.get('error',''),wrap=True,transform=ax.transAxes)
            save(ep+'_source_unavailable',fig,q)
    if (run/'outcomes.json').exists():
        case=load_case(run);ctx=case['context'];f=case['native'];b=np.asarray(ctx['B_world']);outcomes=read(run/'outcomes.json')
        prior=np.asarray(past(run/'source_event',ctx['episode_id'],ctx['switch_state_id']))
        allxy=[prior[:,:2],f[:,:2]]
        for m in METHODS:
            p=run/'rollouts'/m/'rollout.json'
            if p.exists():allxy.append(np.array([r['pose_world'][:2] for r in read(p)['states']]))
        points=np.vstack(allxy);lo=points.min(0)-.4;hi=points.max(0)+.4
        from shapely.geometry import box
        clip=box(*lo,*hi);fig,axes=plt.subplots(1,5,figsize=(20,6),squeeze=False)
        world_data={}
        for ax,m in zip(axes[0],METHODS):
            draw_geometry(ax,case['environment'].workspace.intersection(clip),facecolor='#f4f8ed',edgecolor='#ccc')
            draw_geometry(ax,case['environment'].obstacles.intersection(clip),facecolor='#777',alpha=.7)
            ax.plot(prior[:,0],prior[:,1],color='#999',label='executed OLD prefix')
            ax.plot(f[:,0],f[:,1],'--',color='#b14799',label='original FRESH')
            old=np.asarray(ctx['old_world']);ax.plot(old[:,0],old[:,1],':',color='#4284ac',label='original OLD')
            ax.scatter(*b[:2],color='black',label='B');ax.scatter(*f[-1,:2],color='green',marker='*',label='goal')
            bx=box_polygon(read(run/'inputs/obstacle.json'));xx,yy=bx.buffer(.25).exterior.xy;ax.plot(xx,yy,':',color='#b43',label='required centre clearance')
            data=dict(context=ctx,outcome=outcomes[m],old_prefix=prior)
            ref=run/'methods'/m/'reference.npy'
            if ref.exists():
                p=np.load(ref);ax.plot(p[:,0],p[:,1],color='#007e87',lw=1,label='method pose reference');data['reference']=p
            plan=run/'methods'/m/'planned_join.json'
            if plan.exists():
                metric=read(plan);p=metric['joined_pose_world'];data['planned_join']=metric
                if p is not None:ax.scatter(*p[:2],marker='s',s=65,facecolors='none',edgecolors='#007e87',label='measured planned join')
            if m=='M4_JOIN_GP' and (run/'methods'/m/'selected.json').exists():
                selection=read(run/'methods'/m/'selected.json');support=read(run/'methods'/m/'support.json')
                p=support['support_poses'][selection['support_index']]
                ax.scatter(*p[:2],marker='x',color='#067',label=f"planned tube start {selection['join_time_s']:.1f}s")
                data['selected_outer_candidate']=selection
            execution=run/'evaluation'/m/'execution.json'
            if execution.exists():
                e=read(execution);p=np.asarray(e['dense_poses_world']);j=read(run/'evaluation'/m/'join.json');data.update(execution=e,join=j)
                ax.plot(p[:,0],p[:,1],color='#e47a20',lw=2,label='actual MPC execution')
                if j['joined_pose_world'] is not None:ax.scatter(*j['joined_pose_world'][:2],marker='o',s=65,facecolors='none',edgecolors='#e47a20',label='executed sustained join')
            else:ax.text(.03,.04,'NO EXECUTION',transform=ax.transAxes,color='#a33')
            ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]',title=m);ax.set_aspect('equal');ax.legend(fontsize=6)
            world_data[m]=data
        fig.suptitle('OFFLINE SINGLE-HANDOFF COUNTERFACTUAL — original FRESH attachment')
        save('attachment_world',fig,world_data)
        for field,threshold,ylabel,name in [('distance_m',.1,'execution → original FRESH distance [m]','execution_to_fresh_distance'),('yaw_error_rad',np.pi/12,'absolute yaw error to projected original FRESH [rad]','execution_to_fresh_yaw')]:
            fig,ax=plt.subplots(figsize=(10,5));numeric={}
            for m in METHODS:
                path=run/'evaluation'/m/'join.json'
                if not path.exists():continue
                d=read(path);line,=ax.plot(d['times_s'],d[field],label=m);numeric[m]=d
                if d['sustained_join_time_s'] is not None:ax.axvline(d['sustained_join_time_s'],color=line.get_color(),ls=':',alpha=.6)
            ax.axhline(threshold,color='black',ls='--',label='frozen join threshold');ax.set(xlim=(0,3),xlabel='time after B [s]',ylabel=ylabel);ax.legend();ax.grid(alpha=.2)
            save(name,fig,numeric)
    text=['<!doctype html><meta charset="utf-8"><title>JOIN-01 obstacle reveal</title><style>body{font:16px system-ui;max-width:1400px;margin:30px auto}img{max-width:100%}</style>',
          '<h1>GP-SE2-JOIN-01</h1>','<p>'+html.escape(collection['status'])+'</p>',
          '<p>One controlled handoff; genuine LightNav; no synthetic evidence. Qualification precedes optimization. Unavailable methods have no execution trace.</p>',
          '<pre>'+html.escape(json.dumps(read(run/'outcomes.json') if (run/'outcomes.json').exists() else collection,indent=2))+'</pre>']
    for p in figs:text+=['<h2>'+p.stem+'</h2>',f'<a href="plots/{p.stem}.json">Numbers/source hashes</a>',f'<img src="plots/{p.name}">']
    for r in records:text += [f'<p>{r["episode"]} {r["kind"]} actual RGB</p>',f'<img src="{r["rgb"].replace("../source_event","source_event")}">']
    with (run/'index.html').open('x') as f:f.write('\n'.join(text))
    write(run/'plot_manifest.json',dict(figures=[str(p.relative_to(run)) for p in figs],gui_runtime=False,renderer_rgb=True))


def package(run):
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        paths=[run/p for p in ['source.json','protocol.json','planning_config.yaml','collection_result.json','validation.json','plot_manifest.json','index.html']]
        paths += list((run/'plots').glob('*.png'))+list((run/'plots').glob('*.json'))+list((run/'qualification').glob('*.json'))
        for name in ['outcomes.json','method_manifest.json','all_starts.json','summary.json']:
            if (run/name).exists():paths.append(run/name)
        for p in paths:z.write(p,p.relative_to(run))
        z.writestr('README.txt','Static single-handoff review. Raw RGB is excluded; HTML RGB links resolve only in the full local run. Numeric PNG sidecars and provenance are included. No online performance claim.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--package',action='store_true');a=p.parse_args()
    (package if a.package else render)(a.run.resolve())
