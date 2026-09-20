#!/usr/bin/env python3
"""Readable per-reference saved DIAG-07 presentation; no new runtime or edits."""
from __future__ import annotations
import argparse
import html
from pathlib import Path
import sys
import zipfile
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely.geometry import box
import numpy as np
from plot_gp_se2_diag07 import plot_data,PLOTS,numeric
from plot_gp_se2_01 import geometry
from run_gp_se2_ref01 import read,write,digest,clean
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diag07_lateral_execution import EXECUTION_LABEL


def scope_text(record):
    return ('PLAN INVALID: LATERAL ONLY\nDIAGNOSTIC EXECUTION / NOT A DEPLOYMENT CANDIDATE'
            if record['hard'] else 'PLAN VALID: BENIGN OPTIMIZED CONTROL\nSAVED FIXED-REFERENCE EXECUTION')


def render(run):
    target=run/'presentation';target.mkdir(exist_ok=False)
    data=plot_data(run);env=HospitalEnvironment.load(read(run/'source.json')['environment_path']);images=[];limits={}
    for case in {r['case_id'] for r in data['records']}:
        a=np.vstack([p for r in data['records'] if r['case_id']==case for p in [r['world_reference'],r['actual_poses'],r['context']['context']['old_world'],r['context']['context']['fresh_world']]])[:,:2]
        lo=a.min(0)-.25;hi=a.max(0)+.25;mid=(hi+lo)/2;span=max(hi-lo);limits[case]=(mid-span/2,mid+span/2)
    for record in data['records']:
        folder=target/record['id'];folder.mkdir();r=record;d=r['diagnostic'];ctx=r['context']['context'];goal=r['context']['goal_route']['goal_world']
        outcome=r['outcome'];t=d['command_times_s'];cmd=np.asarray(d['command_values'])
        for name in PLOTS:
            two=name in (PLOTS[2],PLOTS[5]);fig,axes=plt.subplots(1,2 if two else 1,figsize=(12,7),squeeze=False);ax=axes[0,0]
            def xy(points,*args,**kwargs):
                a=np.asarray(points);ax.plot(a[:,0],a[:,1],*args,**kwargs)
            if name in (PLOTS[0],PLOTS[7]):
                lo,hi=limits[r['case_id']];clip=box(*lo,*hi)
                geometry(ax,env.workspace.intersection(clip),facecolor='#f2f6eb',edgecolor='#81936f')
                geometry(ax,env.obstacles.intersection(clip),facecolor='#aaa',edgecolor='#555')
                xy(ctx['old_world'],':',color='#537caf',label='original OLD')
                xy(ctx['fresh_world'],'--',color='#b968a3',label='original FRESH')
                xy(r['world_reference'],'o--',ms=3,color='#087b88',label='unchanged GP reference rows')
                xy(r['actual_poses'],'-',color='#df771c',lw=2,label='new official-MPC execution')
                if name==PLOTS[0]:xy(r['plan_poses'],'-',lw=.8,color='#009e73',label='GP interpolated plan')
                else:
                    for i,color in zip((0,10,20),('#b7445a','#6857b6','#327fc3')):
                        p=r['predictions'][i];xy(p['reference'],'x:',color=color,label=f'{i/10:g}s actual targets')
                        if p['prediction'] is not None:xy(p['prediction'],'s--',ms=3,color=color,label=f'{i/10:g}s saved prediction')
                b=ctx['B_world'];ax.scatter(*b[:2],c='k',s=25,label='original B');ax.scatter(*goal[:2],marker='*',c='green',s=80,label='original goal')
                ax.add_patch(Circle(b[:2],.2,fill=False,color='#888'));ax.add_patch(Circle(b[:2],.25,fill=False,ls=':',color='#888'))
                ax.add_patch(Circle(goal[:2],.15,fill=False,ls=':',color='green',label='goal position tolerance'))
                ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal')
            elif name==PLOTS[1]:
                ax.plot(r['plan_times'],r['plan_vy'],color='#087b88',label='GP planned body vy (not actual command)')
                tol=data['config']['formulation']['equality_tolerance']
                for value in (-tol,tol):ax.axhline(value,color='k',ls=':',label='original ±1e-5 tolerance' if value==tol else None)
                ax.set_ylabel('planned lateral velocity [m/s]')
                ax.text(.02,.95,'Actual command is [v, omega]; no measured lateral channel.\nExecution '+('PASS' if outcome['primary_success'] else 'FAIL: goal position and dwell'),transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='#ccc'))
            elif name==PLOTS[2]:
                for j,(key,label,bound) in enumerate([('position_error_m','position error [m]',.15),('signed_goal_yaw_error_rad','wrapped goal yaw error [rad]',np.pi/12)]):
                    a=axes[0,j];a.plot(d['error_times_s'],d[key],color='#df771c',label='executed original-goal error');a.axhline(bound,color='k',ls=':',label='original tolerance')
                    if j:a.axhline(-bound,color='k',ls=':')
                    a.set_ylabel(label)
            elif name in (PLOTS[3],PLOTS[4]):
                j=0 if name==PLOTS[3] else 1;ax.step(t+[3.],np.r_[cmd[:,j],cmd[-1,j]],where='post',color='#df771c',label='actual held command')
                ax.axhline(.8 if j==0 else 3.,color='k',ls=':',label='original bounds');ax.axhline(0 if j==0 else -3.,color='k',ls=':');ax.set_ylabel('v [m/s]' if j==0 else 'omega [rad/s]')
            elif name==PLOTS[5]:
                a=np.asarray(d['control_grid_acceleration'])
                for j in (0,1):
                    b=axes[0,j];b.plot(t,a[:,j],'.-',color='#df771c',label='control-grid command difference');limit=2 if j==0 else 5
                    b.axhline(limit,color='k',ls=':',label='original nominal bounds');b.axhline(-limit,color='k',ls=':');b.set_ylabel('dv/dt [m/s²]' if j==0 else 'domega/dt [rad/s²]')
            elif name==PLOTS[6]:
                s=d['selection'];idx=np.asarray(s['selected_indices']);ax.step(t,s['nearest_indices'],where='post',color='k',label='nearest row')
                for j in range(5):ax.step(t,idx[:,j],where='post',label=f'selected target {j+1}')
                ax.axhline(29,color='#888',ls=':',label='final GP row')
                ax.set_ylabel('installed row index (not time or distance)')
                ax.text(.02,.97,f"First final GP row: {d['first_final_row_in_horizon_s']} s\nFirst target inside original goal region: {d['first_goal_region_in_horizon_s']} s",transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',edgecolor='#ccc',alpha=.9))
            elif name==PLOTS[8]:
                ax.plot(r['actual_times'],r['clearance'],color='#df771c',label='executed footprint-edge clearance');ax.axhline(.05,color='k',ls=':',label='nominal required 0.05 m');ax.set_ylabel('clearance [m]')
            else:
                f=r['plan_flags'];e=r['execution_flags'];keys=['Plan\nlateral','Other\nplan','Full\nplan','Exec\nclearance','Exec\nroute','Exec\ngoal','Exec\ndwell','Exec\nmotion','Full\nexecution']
                values=[f['lateral_velocity'],all(v for k,v in f.items() if k!='lateral_velocity'),outcome['plan_valid'],e['clearance'],e['route'],e['goal'],e['dwell'],e['motion'],outcome['primary_success']]
                ax.imshow([values],vmin=0,vmax=1,cmap=matplotlib.colors.ListedColormap(['#e5998e','#a2d2ab']),aspect='auto')
                for j,v in enumerate(values):ax.text(j,0,'PASS' if v else 'FAIL',ha='center',va='center',fontsize=11)
                ax.set_xticks(range(len(keys)),keys);ax.set_yticks([])
                ax.text(.01,-.18,'Execution success never changes plan validity.\nExecution failure does not identify lateral velocity as the cause.',transform=ax.transAxes,fontsize=10,va='top')
            for a in axes[0]:
                if name not in (PLOTS[0],PLOTS[7],PLOTS[9]):a.set(xlim=(0,3),xlabel='time after original B [s]')
                a.grid(alpha=.2)
                h,_=a.get_legend_handles_labels()
                if h:a.legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1) if not two else (0,1.02),frameon=False)
            fig.suptitle(name.replace('_',' ')+'\n'+r['id']+' — '+', '.join(r['labels'])+' — Execution '+('PASS' if outcome['primary_success'] else 'FAIL'),fontsize=13,y=.97)
            fig.text(.06,.845,scope_text(r),fontsize=10,color='#983820' if r['hard'] else '#236c44',va='top')
            fig.text(.06,.025,EXECUTION_LABEL+' | adjacent JSON: exact plotted data and source hashes',fontsize=9)
            fig.subplots_adjust(left=.08,right=.70 if not two and name!=PLOTS[9] else .96,bottom=.22 if name==PLOTS[9] else .14,top=.72,wspace=.35)
            p=folder/(name+'.png');fig.savefig(p,dpi=140);plt.close(fig)
            payload=numeric(name,{**data,'records':[r]})
            write(p.with_suffix('.json'),dict(numeric=payload,image_sha256=digest(p),parent_numeric_source=str(run/'plot_data.json'),parent_sha256=digest(run/'plot_data.json'),presentation_only=True))
            images.append(str(p.relative_to(target)))
    content=['<!doctype html><meta charset="utf-8"><title>DIAG-07 readable review</title><style>body{font:16px system-ui;max-width:1200px;margin:30px auto}img{width:100%}td,th{border:1px solid #ccc;padding:8px}table{border-collapse:collapse}</style>',
      '<h1>DIAG-07: fixed lateral-invalid plans and MPC execution</h1><p>'+EXECUTION_LABEL+'</p>',
      '<p><strong>'+data['summary']['interpretation']+'</strong></p><p>Four distinct hard references, one hard event. Hard plans remain invalid. All hard executions fail original goal position/dwell; safety and command limits pass. Benign plan and execution pass. No failure is causally attributed to lateral velocity.</p>',
      '<table><tr><th>Provenance</th><th>Plan</th><th>Execution</th><th>Position m</th><th>Yaw deg</th></tr>']
    for r in data['records']:
        o=r['outcome'];content.append(f"<tr><td>{html.escape(', '.join(r['labels']))}</td><td>{o['plan_valid']}</td><td>{o['primary_success']}</td><td>{o['terminal_position_error_m']:.9f}</td><td>{np.degrees(o['terminal_yaw_error_rad']):.6f}</td></tr>")
    content+=['</table><p>Original acceptance and all scientific records are unchanged. This presentation fixes overlapping labels in the initial multi-panel images; original figures remain available.</p>']
    for r in data['records']:
        content.append('<h2>'+html.escape(', '.join(r['labels']))+'</h2>')
        for name in PLOTS:
            rel=r['id']+'/'+name
            content.append(f'<details {"open" if name in (PLOTS[0],PLOTS[2],PLOTS[9]) else ""}><summary>{name}</summary><a href="{rel}.json">Numbers and hashes</a><img loading="lazy" src="{rel}.png"></details>')
    with (target/'index.html').open('x') as f:f.write('\n'.join(content))
    write(target/'source.json',dict(primary=str(run),source_sha256=digest(run/'source.json'),plot_data_sha256=digest(run/'plot_data.json'),
         presentation_script_sha256=digest(__file__),new_MPC=0,new_GP=0,new_rollout=0,original_images_preserved=True))
    # A self-contained bundle: all 50 readable plots and their numeric sidecars.
    with zipfile.ZipFile(run/'review_bundle_readable.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(target.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(target))
        for parent in ['aggregate']:
            for p in (run/parent).glob('*'):z.write(p,p.relative_to(run))
        for name in ['review_source.json','protocol.json','reference_manifest.json']:z.write(run/name,name)
    write(target/'artifact_hashes.json',{str(p.relative_to(target)):digest(p) for p in sorted(target.rglob('*')) if p.is_file()})
    print(dict(readable_plots=len(images),zip_bytes=(run/'review_bundle_readable.zip').stat().st_size),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);render(p.parse_args().run.resolve())
