#!/usr/bin/env python3
"""Plot every frozen GP-SE2-01 case/method, including failed candidates."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as PolygonPatch
import numpy as np
from shapely.geometry import box
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import METHODS, goal_trace
from reconciliation.gp_se2 import sample_gp

REQUIRED=('environment_context.png','candidate_overlay.png','rollout_overlay.png','boundary_zoom.png',
 'clearance_vs_time.png','linear_command_vs_time.png','angular_command_vs_time.png','goal_error_vs_time.png')
GP_EXTRA=('interpolated_lateral_velocity.png','constraint_violations.png','factor_costs.png')
COLORS=dict(old='#2774b6',fresh='#aa3377',candidate='#008c95',rollout='#e16b13',diagnostic='#8058a8')

def read(p):return json.loads(Path(p).read_text())
def digest(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,value):
    with Path(p).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def geometry(ax,g,*,facecolor,edgecolor,alpha=1.,linewidth=.6):
    if g.is_empty:return
    if hasattr(g,'geoms'):
        for part in g.geoms:geometry(ax,part,facecolor=facecolor,edgecolor=edgecolor,alpha=alpha,linewidth=linewidth)
    elif g.geom_type=='Polygon':
        ax.add_patch(PolygonPatch(np.asarray(g.exterior.coords),facecolor=facecolor,edgecolor=edgecolor,alpha=alpha,linewidth=linewidth))
        for ring in g.interiors:
            ax.add_patch(PolygonPatch(np.asarray(ring.coords),facecolor=ax.get_facecolor(),edgecolor=edgecolor,linewidth=linewidth))
    elif hasattr(g,'coords'):
        xy=np.asarray(g.coords);ax.plot(xy[:,0],xy[:,1],color=edgecolor,alpha=alpha,linewidth=linewidth)

def limits_for(case,methods):
    context=read(case/'input_context.json');parts=[np.asarray(context[k])[:,:2] for k in ('old_world','fresh_world')]
    parts.append(np.asarray(context['B_world'])[None,:2])
    for m in methods.values():
        for k in ('candidate','diagnostic'):
            if m.get(k) is not None:parts.append(m[k][:,:2])
        if m['metrics'].get('dense_poses_world'):parts.append(np.asarray(m['metrics']['dense_poses_world'])[:,:2])
    xy=np.vstack(parts);lo=xy.min(axis=0)-.65;hi=xy.max(axis=0)+.65
    center=(lo+hi)/2;span=max(float(np.max(hi-lo)),2.)
    return [center[0]-span/2,center[0]+span/2,center[1]-span/2,center[1]+span/2]

def load_methods(case):
    out={}
    for method in METHODS:
        p=case/'methods'/method;s=read(p/'solver_result.json')
        out[method]=dict(path=p,solver=s,plan=read(p/'plan_validation.json'),metrics=read(p/'metrics.json'),
          candidate=np.load(p/'candidate_world.npy') if (p/'candidate_world.npy').is_file() else None,
          diagnostic=np.asarray(s['diagnostic_candidate_world']) if s.get('candidate_world') is None and s.get('diagnostic_candidate_world') is not None else None,
          rollout=read(p/'rollout/rollout.json') if (p/'rollout/rollout.json').is_file() else None)
    return out

def status_text(m):
    return f"candidate: {m['solver']['status']} | plan: {m['plan']['status']} | rollout success: {m['metrics']['primary_success']}\n"+(', '.join(m['metrics']['failure_reasons']) or 'all primary local-transition conditions passed')

def draw_xy(ax,context,route,m,env,limits,cfg,*,highlight='all'):
    x0,x1,y0,y1=limits;clip=box(x0,y0,x1,y1);ax.set_facecolor('#eee9e2')
    geometry(ax,env.workspace.intersection(clip),facecolor='#f4faf0',edgecolor='#92aa80',alpha=1.)
    geometry(ax,env.obstacles.intersection(clip),facecolor='#939393',edgecolor='#4f4f4f')
    for key,label in [('old_world','original OLD'),('fresh_world','original FRESH')]:
        xy=np.asarray(context[key]);color=COLORS['old' if key=='old_world' else 'fresh']
        ax.plot(xy[:,0],xy[:,1],'.--',c=color,lw=1.1,ms=3,label=label,zorder=4)
    if m.get('candidate') is not None:
        xy=m['candidate'];ax.plot(xy[:,0],xy[:,1],'.-',c=COLORS['candidate'],lw=2.2 if highlight=='candidate' else 1.5,ms=3,label='candidate reference',zorder=5)
    if m.get('plan',{}).get('motion') is not None and m['plan'].get('dense_poses_world'):
        continuous=np.asarray(m['plan']['dense_poses_world'])
        ax.plot(continuous[:,0],continuous[:,1],color='#5651b8',ls='--',lw=1.4,label='interpolated GP plan',zorder=5)
    if m.get('diagnostic') is not None:
        xy=m['diagnostic'];ax.plot(xy[:,0],xy[:,1],':',c=COLORS['diagnostic'],lw=1.5,label='REJECTED last iterate (no candidate)',zorder=5)
    if m['metrics'].get('dense_poses_world'):
        xy=np.asarray(m['metrics']['dense_poses_world']);ax.plot(xy[:,0],xy[:,1],c=COLORS['rollout'],lw=2.3 if highlight=='rollout' else 1.7,label='actual MPC rollout',zorder=6)
    b=np.asarray(context['B_world']);goal=np.asarray(route['goal_world']);radius=cfg['footprint']['radius_m'];margin=cfg['footprint']['required_clearance_m']
    ax.add_patch(Circle(b[:2],radius,fill=False,color='black',lw=1.,label='footprint at B: r=0.20 m',zorder=7))
    ax.add_patch(Circle(b[:2],radius+margin,fill=False,color='black',ls=':',lw=1.,label='required margin: +0.05 m',zorder=7))
    ax.scatter(*b[:2],marker='o',c='black',s=28,zorder=8,label='actual boundary B')
    ax.arrow(*b[:2],.30*np.cos(b[2]),.30*np.sin(b[2]),width=.009,head_width=.065,color='black',length_includes_head=True,zorder=8)
    ax.add_patch(Circle(goal[:2],cfg['formulation']['goal_position_tolerance'],facecolor='#36a65c',alpha=.12,edgecolor='#238647'))
    ax.scatter(*goal[:2],marker='*',s=90,c='#238647',zorder=8,label='fixed original FRESH goal')
    ax.arrow(*goal[:2],.25*np.cos(goal[2]),.25*np.sin(goal[2]),width=.006,head_width=.05,color='#238647',length_includes_head=True,zorder=8)
    for i,g in enumerate(route['gates']):
        center=np.asarray(g['center_xy']);normal=np.asarray(g['normal_xy']);tangent=np.array([-normal[1],normal[0]])
        ends=center+np.array([[-1],[1]])*g['half_width_m']*tangent
        ax.plot(ends[:,0],ends[:,1],color='#86611a',lw=2,ls='-.',label=f'route gate {i+1} (safe centre interval)')
        if x0<center[0]<x1 and y0<center[1]<y1:ax.arrow(*center,* (.25*normal),color='#86611a',head_width=.07,length_includes_head=True)
    ax.set(xlim=(x0,x1),ylim=(y0,y1),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal');ax.grid(alpha=.18)
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=7.5,frameon=False)

def save(fig,path,title,dpi):
    fig.suptitle(title,fontsize=10)
    fig.text(.015,.012,'Static Hospital oracle | 0.20 m footprint | offline counterfactual | hashes: plot_provenance.json',fontsize=7,color='#555555')
    fig.tight_layout(rect=(0,.04,1,.94));fig.savefig(path,dpi=dpi);plt.close(fig)

def empty(ax,label):ax.text(.5,.5,label,ha='center',va='center',transform=ax.transAxes,fontsize=12);ax.set(xlim=(0,3),xlabel='counterfactual time [s]');ax.grid(alpha=.2)
def gp_trace(m):
    s=m['solver'];p=s.get('support_poses');v=s.get('support_twists');label='accepted GP'
    if p is None:p=s.get('diagnostic_support_poses');v=s.get('diagnostic_support_twists');label='REJECTED last iterate; no candidate'
    if p is None or v is None:return None
    knots=np.asarray(s['support_times']);q=np.linspace(0,3,601)
    try:poses,vel,acc=sample_gp(knots,np.asarray(p),np.asarray(v),q)
    except (ValueError,FloatingPointError):return None
    return q,poses,vel,acc,label

def plot_method(run,case,row,method,m,env,limits,cfg):
    out=m['path']/'plots';out.mkdir(exist_ok=False);context=read(case/'input_context.json');route=read(case/'goal_route.json');dpi=cfg['plots']['dpi']
    heading=f"{row['case_id']} | {method}\n{status_text(m)}"; b=np.asarray(context['B_world'])
    for filename in REQUIRED[:4]:
        fig,ax=plt.subplots(figsize=(11,7))
        bounds=[b[0]-1,b[0]+1,b[1]-1,b[1]+1] if filename=='boundary_zoom.png' else limits
        draw_xy(ax,context,route,m,env,bounds,cfg,highlight='candidate' if filename=='candidate_overlay.png' else 'rollout' if filename=='rollout_overlay.png' else 'all')
        save(fig,out/filename,heading,dpi)
    fig,ax=plt.subplots(figsize=(9,5));drawn=False
    for label,payload in [('plan / reference',m['plan']),('actual MPC rollout',m['metrics'])]:
        if payload.get('environment') is not None:
            ax.plot(payload['dense_times_s'],payload['environment']['clearance_samples_m'],label=label);drawn=True
    ax.axhline(.05,c='red',ls='--',label='required clearance');ax.axhline(0,c='black',ls=':',label='physical overlap boundary')
    if not drawn:empty(ax,'NO CANDIDATE / NO ROLLOUT\nNo clearance curve is fabricated')
    ax.set(xlabel='evaluation time [s]; reference timing is a convention',ylabel='footprint-edge clearance [m]');ax.legend();ax.grid(alpha=.2);save(fig,out/'clearance_vs_time.png',heading,dpi)
    for component,filename,label,lo,hi in [(0,'linear_command_vs_time.png','linear command [m/s]',0,.8),(1,'angular_command_vs_time.png','angular command [rad/s]',-3,3)]:
        fig,ax=plt.subplots(figsize=(9,5));roll=m['rollout']
        if roll and roll.get('controller_reference_selections'):
            solves=roll['controller_reference_selections'];ts=[0]+[s['time_s'] for s in solves]+[3.];ys=[context['u_minus'][component]]+[s['command'][component] for s in solves]+[solves[-1]['command'][component]]
            ax.step(ts,ys,where='post',c=COLORS['rollout'],label='applied command');ax.scatter(0,context['u_minus'][component],c='black',label='physical u_minus',zorder=4)
        else:empty(ax,'NO CANDIDATE / NO ROLLOUT\nNo zero-filled command trace')
        ax.axhline(lo,c='red',ls='--');ax.axhline(hi,c='red',ls='--',label='controller speed bounds');ax.set(xlabel='counterfactual time [s]',ylabel=label);ax.legend();ax.grid(alpha=.2);save(fig,out/filename,heading,dpi)
    fig,axes=plt.subplots(2,1,figsize=(9,7),sharex=True)
    if m['metrics'].get('dense_poses_world'):
        t=m['metrics']['dense_times_s'];dist,yaw,_=goal_trace(m['metrics']['dense_poses_world'],route['goal_world'])
        axes[0].plot(t,dist,c=COLORS['rollout']);axes[1].plot(t,np.degrees(yaw),c=COLORS['rollout'])
    else:
        for ax in axes:empty(ax,'NO CANDIDATE / NO ROLLOUT')
    for ax,tol,label in zip(axes,[.15,15],['goal position error [m]','goal yaw error [deg]']):
        ax.axhline(tol,c='red',ls='--');ax.axvspan(2.8,3,color='#36a65c',alpha=.12);ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[-1].set_xlabel('counterfactual time [s]; shaded: terminal 0.20 s dwell');save(fig,out/'goal_error_vs_time.png',heading,dpi)
    if method.startswith(('M2','M3')):
        trace=gp_trace(m);fig,ax=plt.subplots(figsize=(9,5))
        if trace:
            t,p,v,a,label=trace;ax.plot(t,v[:,1],label=label);ax.axhline(1e-5,c='red',ls='--');ax.axhline(-1e-5,c='red',ls='--');ax.legend()
        else:empty(ax,'GP trace unavailable; no accepted candidate')
        ax.set(xlabel='GP time [s]',ylabel='interpolated body lateral velocity [m/s]');ax.grid(alpha=.2);save(fig,out/GP_EXTRA[0],heading,dpi)
        fig,axes=plt.subplots(5,1,figsize=(9,11),sharex=True)
        if trace:
            t,p,v,a,label=trace
            traces=[np.abs(v[:,1])-1e-5,np.maximum(-v[:,0],v[:,0]-.8),np.abs(v[:,2])-3,np.abs(a[:,0])-2,np.abs(a[:,2])-5]
            labels=['lateral excess\n[m/s]','speed excess\n[m/s]','angular speed\nexcess [rad/s]','linear accel.\nexcess [m/s²]','angular accel.\nexcess [rad/s²]']
            for ax,y,l in zip(axes,traces,labels):ax.plot(t,np.maximum(y,0),label=label);ax.set_ylabel(l,fontsize=8)
        else:
            for ax in axes:empty(ax,'No GP iterate available')
        for ax in axes:ax.grid(alpha=.2)
        axes[-1].set_xlabel('GP time [s]; sampled diagnostic; authoritative checks: plan_validation.json');save(fig,out/GP_EXTRA[1],heading,dpi)
        fig,axes=plt.subplots(2,1,figsize=(9,7));cost=m['solver'].get('factor_costs')
        if cost:
            axes[0].bar(np.arange(len(cost['gp'])),cost['gp']);axes[0].set(xlabel='adjacent support factor',ylabel='GP whitened cost')
            axes[1].bar(['GP sum','uniform FRESH'],[sum(cost['gp']),cost['fresh_uniform']]);axes[1].set_ylabel('own objective contribution')
        else:
            for ax in axes:empty(ax,'NO ACCEPTED CANDIDATE\nAccepted factor costs: N/A')
        save(fig,out/GP_EXTRA[2],heading,dpi)
    images=[dict(path=p.name,sha256=digest(p),bytes=p.stat().st_size) for p in sorted(out.glob('*.png'))]
    write(out/'plot_provenance.json',dict(config_sha256=digest(run/'config_snapshot.yaml'),source_sha256=digest(run/'source.json'),metrics_sha256=digest(m['path']/'metrics.json'),
      context_sha256=digest(case/'input_context.json'),plan_sha256=digest(m['path']/'plan_validation.json'),solver_sha256=digest(m['path']/'solver_result.json'),images=images,dpi=dpi,
      axes_common_all_methods=limits,boundary_zoom_half_width_m=1.,coordinates='unscaled Hospital world XY metres',missing_data='explicit no candidate/no rollout; never zero-filled'))
    return images

def main(run):
    run=Path(run).resolve()
    if (run/'index.html').exists():raise FileExistsError('refusing plot overwrite')
    manifest=read(run/'case_manifest.json');cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());env=HospitalEnvironment.load(run/'environment');summary=read(run/'aggregate/summary.json')
    if cfg['plots']['dpi']<160:raise ValueError('PNG dpi must be at least 160')
    sections=['<!doctype html><html><head><meta charset="utf-8"><title>GP-SE2-01</title><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto;padding:20px}img{width:48%;vertical-align:top}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #bbb}a{color:#1360a0}pre{white-space:pre-wrap}</style></head><body><h1>OFFLINE COUNTERFACTUAL HANDOFF COMPARISON</h1>',
      '<p>Static Hospital oracle; unchanged official MPC; 3.0 s independent kinematic rollouts. Failed candidates remain in every method view. Images use equal metric axes.</p>',
      f"<p><strong>Native success → M3 failure: {summary['native_success_to_gp_failure']}; adapter success → M3 failure: {summary['adapter_success_to_gp_failure']}.</strong></p>",
      '<p><a href="aggregate/summary.json">All outcomes and paired metrics</a> · <a href="case_manifest.json">Frozen cases and all exclusion reasons</a> · <a href="protocol.json">Protocol</a></p>']
    count=0
    for row in manifest['selected']:
        case=run/'cases'/row['case_directory'];methods=load_methods(case);limits=limits_for(case,methods)
        sections.append(f"<h2>{html.escape(row['case_id'])} — {html.escape(row['selected_group'])}</h2>")
        # Common overlay uses precisely the same world limits as each individual method.
        caseplots=case/'plots';caseplots.mkdir(exist_ok=False);fig,ax=plt.subplots(figsize=(11,7));context=read(case/'input_context.json');route=read(case/'goal_route.json')
        blank=dict(candidate=None,diagnostic=None,metrics={});draw_xy(ax,context,route,blank,env,limits,cfg)
        for i,(name,m) in enumerate(methods.items()):
            if m['metrics'].get('dense_poses_world'):
                p=np.asarray(m['metrics']['dense_poses_world']);ax.plot(p[:,0],p[:,1],label=name,lw=1.6,c=plt.get_cmap('tab10')(i))
        ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=7,frameon=False);save(fig,caseplots/'all_methods_rollout.png',row['case_id']+' | independent actual MPC rollouts',cfg['plots']['dpi'])
        rel=str((caseplots/'all_methods_rollout.png').relative_to(run));sections.append(f'<a href="{rel}"><img src="{rel}" alt="all method rollouts"></a>')
        write(caseplots/'plot_provenance.json',dict(config_sha256=digest(run/'config_snapshot.yaml'),source_sha256=digest(run/'source.json'),method_metrics_sha256={k:digest(m['path']/'metrics.json') for k,m in methods.items()},images=[dict(path='all_methods_rollout.png',sha256=digest(caseplots/'all_methods_rollout.png'))]))
        for method,m in methods.items():
            images=plot_method(run,case,row,method,m,env,limits,cfg);count+=len(images)
            sections.append(f'<h3>{method}</h3><p>{html.escape(status_text(m))}</p>')
            for record in images:
                rel=str((m['path']/'plots'/record['path']).relative_to(run));sections.append(f'<a href="{rel}"><img loading="lazy" src="{rel}" alt="{record["path"]}"></a>')
        print(json.dumps(dict(case=row['case_id'],method_images_written=count)),flush=True)
    sections.append('</body></html>')
    with (run/'index.html').open('x') as f:f.write('\n'.join(sections))
    write(run/'plot_completion.json',dict(case_count=len(manifest['selected']),method_image_count=count,common_overlay_count=len(manifest['selected']),index_sha256=digest(run/'index.html'),dpi=cfg['plots']['dpi']))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);main(p.parse_args().run)
