#!/usr/bin/env python3
"""Static ATTACH-01 presentation: every external duration, no missing traces."""
import argparse
import csv
import html
import json
import os
import sys
import zipfile
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/attach01-mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely.geometry import box
from run_gp_se2_attach01 import read,write,digest,load_case
from reconciliation.gp_se2_attach01 import method_schedule
from plot_gp_se2_join01 import draw_geometry


def old_prefix(context):
    episode=Path(context['source_root'])/'episodes'/context['episode_id']
    raw=read(episode/'handoffs'/context['handoff_id']/'context.json')
    with (episode/'commands.csv').open() as stream:commands=list(csv.DictReader(stream))
    ids=[int(r['application_state_id']) for r in commands if r['chunk_id']==raw['old_chunk_id'] and int(r['application_state_id'])<context['switch_state_id']]
    if not ids:raise ValueError('recorded OLD prefix missing')
    with (episode/'execution.csv').open() as stream:states=list(csv.DictReader(stream))
    return np.array([[float(r[k])for k in ['x','y','yaw']]for r in states if min(ids)<=int(r['state_id'])<=context['switch_state_id']])


def csv_file(path,rows):
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader()
        writer.writerows({k:json.dumps(v)if isinstance(v,(dict,list))else v for k,v in r.items()}for r in rows)


def render(run):
    case=load_case(run);context=case['context'];outcomes=read(run/'outcomes.json');manifest=read(run/'method_manifest.json')
    plots=run/'plots';plots.mkdir(exist_ok=False);aggregate=run/'aggregate';aggregate.mkdir(exist_ok=False)
    csv_file(aggregate/'outcomes.csv',list(outcomes.values()))
    csv_file(aggregate/'all_starts.csv',read(run/'all_starts.json'))
    csv_file(aggregate/'duration_sweep.csv',[r for r in outcomes.values()if r['duration_s'] is not None])
    figures=[]
    def save(name,fig,numeric):
        fig.tight_layout(rect=(0,.08,1,.96) if name=='attachment_world' else (0,0,1,1))
        path=plots/(name+'.png');fig.savefig(path,dpi=155);plt.close(fig)
        write(path.with_suffix('.json'),dict(image_sha256=digest(path),source_sha256=digest(run/'source.json'),
            config_sha256=digest(run/'config_snapshot.yaml'),common_file_sha256=digest(run/'inputs/common.npy'),
            outcomes_sha256=digest(run/'outcomes.json'),numeric=numeric))
        figures.append(str(path.relative_to(run)))
    f=case['native'];common=case['common'];old=np.array(context['old_world']);b=np.array(context['B_world']);past=old_prefix(context)
    executions={};attachments={};references={};plans={}
    for method in outcomes:
        ep=run/'evaluation'/method/'execution.json';rp=run/'methods'/method/'reference.npy';pp=run/'methods'/method/'planned_attachment.json'
        if ep.exists():executions[method]=read(ep);attachments[method]=read(ep.with_name('attachment.json'))
        if rp.exists():references[method]=np.load(rp)
        if pp.exists():plans[method]=read(pp)
    points=np.vstack([f[:,:2],old[:,:2],past[:,:2],b[None,:2]]+[np.array(e['dense_poses_world'])[:,:2]for e in executions.values()]+[r[:,:2]for r in references.values()])
    lo=points.min(0)-.35;hi=points.max(0)+.35;clip=box(*lo,*hi);env=case['environment']
    def background(ax,include_common=True):
        draw_geometry(ax,env.workspace.intersection(clip),facecolor='#f1f5ed',edgecolor='#b7c3aa')
        draw_geometry(ax,env.obstacles.intersection(clip),facecolor='#777',alpha=.75)
        draw_geometry(ax,env.obstacles.buffer(.25).boundary.intersection(clip),edgecolor='#bb7755')
        ax.plot(old[:,0],old[:,1],':',color='#4785ac',label='original OLD')
        ax.plot(past[:,0],past[:,1],color='#777',lw=2,label='executed OLD prefix')
        ax.plot(f[:,0],f[:,1],'--',color='#9d4391',label='original FRESH')
        if include_common:ax.plot(common[:,0],common[:,1],'.',color='#187c82',ms=3,label='fixed common (all T)')
        ax.scatter(*b[:2],color='black',s=24,label='B');ax.scatter(*f[-1,:2],marker='*',color='#267947',s=90,label='original goal')
        ax.add_patch(Circle(b[:2],.20,fill=False,color='black',alpha=.3))
        ax.arrow(*b[:2],.14*np.cos(b[2]),.14*np.sin(b[2]),head_width=.025,color='black',length_includes_head=True)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal')
    fig,ax=plt.subplots(figsize=(9,7));background(ax);ax.set_title(context['case_id']+' — source geometry')
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8)
    save('selected_source_geometry',fig,dict(old=old,fresh=f,common=common,executed_OLD_prefix=past,B=b,
        footprint_m=.20,required_edge_clearance_m=.05,obstacle_center_exclusion_radius_m=.25))
    fig,axes=plt.subplots(3,3,figsize=(16,12));world={}
    for ax,(method,t) in zip(axes.flat,method_schedule()):
        background(ax,False);numeric=dict(outcome=outcomes[method])
        if method in references:
            r=references[method];ax.plot(r[:,0],r[:,1],color='#008a92',lw=1,label='method reference');numeric['reference']=r
        if method in executions:
            p=np.array(executions[method]['dense_poses_world']);ax.plot(p[:,0],p[:,1],color='#e47722',lw=2,label='actual MPC execution');numeric['execution']=p
            j=attachments[method]['joined_pose_world']
            if j is not None:ax.scatter(*j[:2],s=60,facecolors='none',edgecolors='#e47722',marker='o',label='executed sustained attachment')
        else:ax.text(.03,.05,'NO FULL-VALID PLAN\nNO EXECUTION',transform=ax.transAxes,color='#a33',fontsize=9)
        if method in plans:
            j=plans[method]['joined_pose_world']
            if j is not None:ax.scatter(*j[:2],s=65,facecolors='none',edgecolors='#008a92',marker='s',label='planned sustained attachment')
            numeric['planned_attachment']=plans[method]
        ax.set_title(method.replace('M4_FIXED_ATTACH_','M4 '),fontsize=11);world[method]=numeric
    handles,labels=axes.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=4,fontsize=9)
    fig.suptitle('OFFLINE SINGLE-HANDOFF COUNTERFACTUAL — fixed correspondence; external T')
    fig.subplots_adjust(bottom=.13);save('attachment_world',fig,world)
    for field,threshold,ylabel,name in [('distance_m',.1,'execution → original FRESH position error [m]','execution_to_fresh_distance'),
        ('yaw_error_rad',np.pi/12,'absolute yaw error to original FRESH [rad]','execution_to_fresh_yaw')]:
        fig,ax=plt.subplots(figsize=(11,5));numeric={}
        for method,j in attachments.items():
            line,=ax.plot(j['times_s'],j[field],label=method);numeric[method]=j
            if j['sustained_join_time_s'] is not None:ax.axvline(j['sustained_join_time_s'],color=line.get_color(),ls=':',alpha=.6)
        ax.axhline(threshold,color='black',ls='--',label='attachment threshold')
        ax.set(xlim=(0,3),xlabel='time after B [s]',ylabel=ylabel);ax.grid(alpha=.2);ax.legend(fontsize=8)
        save(name,fig,numeric)
    sweep=[outcomes[m]for m,t in method_schedule()if t is not None]
    fig,axes=plt.subplots(2,2,figsize=(12,8));ts=[r['duration_s']for r in sweep]
    axes[0,0].scatter(ts,[int(r['plan_valid'])for r in sweep],marker='s',label='full-valid plan')
    executed=[r for r in sweep if r['rollout_performed']]
    if executed:axes[0,0].scatter([r['duration_s']for r in executed],[int(r['execution_join_success'])for r in executed],marker='o',facecolors='none',edgecolors='#e47722',label='execution attachment')
    for r in sweep:
        if not r['rollout_performed']:axes[0,0].annotate('exec N/A',(r['duration_s'],.10),rotation=60,fontsize=8)
    axes[0,0].set(yticks=[0,1],yticklabels=['fail','pass'],ylim=(-.2,1.25));axes[0,0].legend(fontsize=8)
    for ax,key,label in [(axes[0,1],'executed_attachment_s','executed attachment time [s]'),(axes[1,0],'minimum_clearance_m','executed minimum edge clearance [m]')]:
        available=[r for r in sweep if r[key] is not None]
        if available:ax.plot([r['duration_s']for r in available],[r[key]for r in available],'o-')
        else:ax.text(.5,.5,'N/A — no executed full-valid M4 plan',ha='center',transform=ax.transAxes,fontsize=9)
        ax.set_ylabel(label)
    axes[1,0].axhline(.05,color='black',ls='--')
    solve_s=[sum(s.get('solve_s',0.)for s in manifest[r['method']]['condition_starts'])for r in sweep]
    axes[1,1].plot(ts,solve_s,'o-');axes[1,1].set_ylabel('prepared solve time, BOTH starts [s]')
    for ax in axes.flat:ax.set(xlabel='externally imposed T [s]',xticks=ts,xlim=(.3,1.5));ax.grid(alpha=.2)
    save('duration_sweep_summary',fig,dict(outcomes=sweep,total_prepared_solve_s=solve_s))
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for ax,key,unit in [(axes[0],'command_total_variation_v_mps','linear command TV [m/s]'),(axes[1],'command_total_variation_omega_radps','angular command TV [rad/s]')]:
        if executed:ax.plot([r['duration_s']for r in executed],[r['execution_metrics'][key]for r in executed],'o-')
        else:ax.text(.5,.5,'N/A — no M4 execution',ha='center',transform=ax.transAxes)
        ax.set(xlabel='externally imposed T [s]',ylabel=unit,xlim=(.3,1.5),xticks=ts)
    save('duration_control_variation',fig,sweep)
    lines=['<!doctype html><meta charset="utf-8"><title>ATTACH-01</title><style>body{font:15px system-ui;max-width:1500px;margin:30px auto}img{max-width:100%}td,th{padding:7px;border:1px solid #ccc}table{border-collapse:collapse}</style>',
        '<h1>GP-SE2-ATTACH-01 — fixed correspondence, externally imposed T</h1>',
        '<p>OFFLINE SINGLE-HANDOFF COUNTERFACTUAL. No correspondence/time optimization, new inference or GUI. All six conditions retained. Unavailable execution is N/A, never a fabricated trace.</p>',
        '<p>Source: '+html.escape(context['case_id'])+'. Recorded moving handoff; boundary physical speed nearly zero. OLD/FRESH local raw arrays are identical but observation transforms differ. No new-obstacle reaction claim.</p>',
        '<table><tr><th>Method / T</th><th>Plan valid</th><th>Planned attach s</th><th>Executed attach s</th><th>Execution success</th><th>Failure</th></tr>']
    for method,r in outcomes.items():
        lines.append('<tr>'+''.join('<td>'+html.escape(str(v)if v is not None else 'N/A')+'</td>'for v in [method,r['plan_valid'],r['planned_attachment_s'],r['executed_attachment_s'],r['execution_success'],r['failure_reason']])+'</tr>')
    lines+=['</table>','<p><a href="aggregate/all_starts.csv">All starts</a> · <a href="aggregate/duration_sweep.csv">Duration table</a> · <a href="validation.json">Artifact validation</a></p>']
    for p in figures:lines+=['<h2>'+Path(p).stem+'</h2>',f'<a href="{Path(p).with_suffix(".json")}">Numeric sidecar</a>',f'<img src="{p}">']
    with (run/'index.html').open('x')as stream:stream.write('\n'.join(lines))
    write(run/'plot_manifest.json',dict(figures=figures,GUI_runtime=False,synthetic_primary_evidence=False))


def package(run):
    paths=[run/name for name in ['protocol.json','config_snapshot.yaml','source.json','outcomes.json','summary.json','validation.json','plot_manifest.json','index.html']]
    paths+=list((run/'plots').glob('*'))+list((run/'aggregate').glob('*.csv'))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED)as archive:
        for path in paths:archive.write(path,path.relative_to(run))
        archive.writestr('README.txt','One recorded obstacle-sensitive handoff; fixed correspondence and external duration sweep. Static evidence only. No new LightNav, Isaac, optimal-time or correspondence claim. Generated arrays, raw RGB, full environment and external source excluded. Original source hashes retained.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--package',action='store_true')
    args=parser.parse_args();(package if args.package else render)(args.run.resolve())
