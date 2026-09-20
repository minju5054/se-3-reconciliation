#!/usr/bin/env python3
"""Static DIAG-07 figures: plan-invalid references stay invalid in every label."""
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
import numpy as np
import yaml
from shapely.geometry import box
from run_gp_se2_ref01 import read,write,digest,clean
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diag07_lateral_execution import HARD,HARD_LABEL,EXECUTION_LABEL
from plot_gp_se2_01 import geometry
PLOTS=('hard_plan_reference_and_execution_xy','hard_plan_vy_vs_execution_outcome','terminal_goal_error',
       'command_v_over_time','command_omega_over_time','command_acceleration','mpc_selected_reference_progress',
       'mpc_prediction_vs_execution','clearance_over_time','plan_vs_execution_acceptance_matrix')


def plot_data(run):
    source=read(run/'source.json');manifest=read(run/'reference_manifest.json');summary=read(run/'aggregate/summary.json');data=[]
    for r,o in zip(manifest['unique_references'],summary['outcomes']):
        uid=r['unique_reference_id'];folder=run/'rollouts'/uid;roll=read(folder/'rollout.json')
        full=read(Path(r['plan_recheck_path']))['original_full_check'];e=read(folder/'full_execution_evaluation.json');d=read(folder/'diagnostics.json')
        ctx=read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'))
        grid=full['additional_grid']
        data.append(dict(id=uid,labels=r['provenance_labels'],case_id=r['case_id'],hard=r['case_id']==HARD,
            admission=r['label'],outcome=o,world_reference=roll['reference_world'],context=ctx,
            plan_times=grid['query_times_s'],plan_poses=grid['poses_world'],plan_vy=np.asarray(grid['body_twists'])[:,1].tolist(),
            actual_times=e['dense_times_s'],actual_poses=e['dense_poses_world'],clearance=e['environment']['clearance_samples_m'],
            effective_clearance_threshold=e['environment'].get('effective_required_clearance_m'),
            diagnostic=d,plan_flags=grid['flags'],predictions=[dict(time_s=s['time_s'],pose=s['input_pose_world'],
              reference=s['selection']['reference_world'],prediction=s['prediction_world']) for s in roll['controller_reference_selections']],
            execution_flags=dict(clearance=e['environment']['clearance_valid'],workspace=e['environment']['workspace_known'],
             route=e['route']['valid'],goal=e['execution']['goal_reached'],dwell=e['execution']['terminal_goal_dwell_pass'],
             motion=e['execution']['motion_limits_pass'],controller=e['execution']['controller_validity_pass'])))
    return clean(dict(records=data,summary=summary,source_sha256=digest(run/'source.json'),config_sha256=digest(run/'config_snapshot.yaml'),
        manifest_sha256=digest(run/'reference_manifest.json'),config=yaml.safe_load((run/'config_snapshot.yaml').read_text())))


def numeric(name,data):
    base=['id','labels','case_id','hard','admission','outcome','plan_flags','execution_flags']
    fields={PLOTS[0]:['world_reference','context','plan_poses','actual_poses'],
        PLOTS[1]:['plan_times','plan_vy'],PLOTS[2]:['diagnostic'],PLOTS[3]:['diagnostic'],PLOTS[4]:['diagnostic'],
        PLOTS[5]:['diagnostic'],PLOTS[6]:['diagnostic'],PLOTS[7]:['context','world_reference','actual_poses','predictions'],
        PLOTS[8]:['actual_times','clearance','effective_clearance_threshold'],PLOTS[9]:[]}[name]
    return {**{k:data[k] for k in ('source_sha256','config_sha256','manifest_sha256','config')},
            'records':[{k:r[k] for k in base+fields} for r in data['records']]}


def title(r):
    return r['id']+' | '+', '.join(r['labels'])+' | Execution '+('PASS' if r['outcome']['primary_success'] else 'FAIL')

def labels(ax,r,*,time=True):
    ax.set_title(title(r),fontsize=9,loc='left');ax.grid(alpha=.2)
    if time:ax.set(xlim=(0,3),xlabel='time after original B [s]')
    ax.text(0,1.03,'PLAN INVALID: LATERAL ONLY / DIAGNOSTIC EXECUTION / NOT A DEPLOYMENT CANDIDATE' if r['hard'] else 'PLAN VALID: BENIGN OPTIMIZED CONTROL',
            transform=ax.transAxes,fontsize=6.5,color='#973c14' if r['hard'] else '#246a46')

def line(ax,points,*args,**kwargs):
    p=np.asarray(points);ax.plot(p[:,0],p[:,1],*args,**kwargs)

def world(ax,r,env,bounds):
    lo,hi=bounds;clip=box(*lo,*hi)
    geometry(ax,env.workspace.intersection(clip),facecolor='#f4f8ef',edgecolor='#889578')
    geometry(ax,env.obstacles.intersection(clip),facecolor='#b0b0b0',edgecolor='#555')
    c=r['context']['context'];b=c['B_world'];g=r['context']['goal_route']['goal_world']
    line(ax,c['old_world'],':',color='#547cb6',label='OLD');line(ax,c['fresh_world'],'--',color='#b568aa',label='original FRESH')
    line(ax,r['world_reference'],'o--',ms=2,color='#007d8c',label='same 30 GP pose rows')
    line(ax,r['actual_poses'],'-',lw=2,color='#de7b1e',label='new MPC execution')
    ax.scatter(*b[:2],s=25,c='k',label='original B');ax.scatter(*g[:2],marker='*',s=65,c='green',label='original goal')
    ax.add_patch(Circle(b[:2],.2,fill=False,color='#777',lw=.7));ax.add_patch(Circle(b[:2],.25,fill=False,color='#777',ls=':',lw=.7))
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal')
    labels(ax,r,time=False)

def render(run):
    data=plot_data(run);env=HospitalEnvironment.load(read(run/'source.json')['environment_path']);rows=data['records'];n=len(rows)
    folder=run/'plots';folder.mkdir(exist_ok=False);bounds={}
    for case in {r['case_id'] for r in rows}:
        a=np.vstack([p for r in rows if r['case_id']==case for p in [r['world_reference'],r['actual_poses'],r['context']['context']['old_world'],r['context']['context']['fresh_world']]])[:,:2]
        lo=a.min(0)-.3;hi=a.max(0)+.3;center=(hi+lo)/2;span=max(hi-lo);bounds[case]=(center-span/2,center+span/2)
    for name in PLOTS:
        cols=2 if name in (PLOTS[2],PLOTS[5]) else 1
        fig,axes=plt.subplots(n,cols,figsize=(14,3.1*n),squeeze=False)
        for k,r in enumerate(rows):
            ax=axes[k,0];d=r['diagnostic'];t=d['command_times_s'];cmd=np.asarray(d['command_values'])
            if name in (PLOTS[0],PLOTS[7]):
                world(ax,r,env,bounds[r['case_id']])
                if name==PLOTS[0]:line(ax,r['plan_poses'],'-',color='#009e73',lw=.8,label='continuous GP plan (invalid if hard)')
                else:
                    for i,col in zip((0,10,20),('#ab4a42','#773bb5','#3a78b3')):
                        pred=r['predictions'][i]
                        line(ax,pred['reference'],'x:',color=col,label=f't={i/10:g} targets')
                        if pred['prediction'] is not None:line(ax,pred['prediction'],'s--',ms=2,color=col,label=f't={i/10:g} prediction (Euler)')
            elif name==PLOTS[1]:
                ax.plot(r['plan_times'],r['plan_vy'],label='GP body vy; NOT an executed command',color='#007d8c')
                tol=data['config']['formulation']['equality_tolerance']
                ax.axhline(tol,color='k',ls=':',label='unchanged plan tolerance');ax.axhline(-tol,color='k',ls=':');ax.set_ylabel('planned lateral velocity [m/s]');labels(ax,r)
            elif name==PLOTS[2]:
                for j,(key,ylabel,bound) in enumerate([('position_error_m','position error [m]',.15),('signed_goal_yaw_error_rad','wrapped yaw error [rad]',np.pi/12)]):
                    b=axes[k,j];b.plot(d['error_times_s'],d[key],color='#de7b1e',label='executed original-goal error');b.axhline(bound,color='k',ls=':');
                    if j:b.axhline(-bound,color='k',ls=':')
                    b.set_ylabel(ylabel);labels(b,r)
            elif name in (PLOTS[3],PLOTS[4]):
                j=0 if name==PLOTS[3] else 1;ax.step(t+[3.],np.r_[cmd[:,j],cmd[-1,j]],where='post',color='#de7b1e',label='actual held command')
                limit=.8 if j==0 else 3.;ax.axhline(limit,color='k',ls=':');ax.axhline(0 if j==0 else -limit,color='k',ls=':')
                ax.set_ylabel('v [m/s]' if j==0 else 'omega [rad/s]');labels(ax,r)
            elif name==PLOTS[5]:
                a=np.asarray(d['control_grid_acceleration'])
                for j in range(2):
                    b=axes[k,j];b.plot(t,a[:,j],'.-',color='#de7b1e',label='10 Hz differences incl. physical u_minus');limit=2 if j==0 else 5
                    b.axhline(limit,color='k',ls=':');b.axhline(-limit,color='k',ls=':');b.set_ylabel('dv/dt [m/s²]' if j==0 else 'domega/dt [rad/s²]');labels(b,r)
            elif name==PLOTS[6]:
                selection=d['selection'];idx=np.asarray(selection['selected_indices'])
                ax.step(t,selection['nearest_indices'],where='post',label='nearest row',color='k')
                for j in range(5):ax.step(t,idx[:,j],where='post',alpha=.7,label=f'target {j+1}')
                ax.axhline(29,color='#777',ls=':',label='final GP reference row');ax.set_ylabel('installed row index (not time/distance)');labels(ax,r)
            elif name==PLOTS[8]:
                ax.plot(r['actual_times'],r['clearance'],color='#de7b1e',label='executed footprint-edge clearance');ax.axhline(.05,color='k',ls=':',label='required 0.05 m')
                ax.set_ylabel('clearance [m]');labels(ax,r)
            else:
                keys=['plan lateral','other plan','full plan','exec clearance','exec route','exec goal','exec dwell','exec motion','exec full']
                flags=r['plan_flags'];e=r['execution_flags'];values=[flags['lateral_velocity'],all(v for k,v in flags.items() if k!='lateral_velocity'),r['outcome']['plan_valid'],e['clearance'],e['route'],e['goal'],e['dwell'],e['motion'],r['outcome']['primary_success']]
                ax.imshow([values],cmap=matplotlib.colors.ListedColormap(['#e28c7e','#a8d5af']),vmin=0,vmax=1,aspect='auto')
                for j,v in enumerate(values):ax.text(j,0,'PASS' if v else 'FAIL',ha='center',va='center',fontsize=9)
                ax.set_xticks(range(len(keys)),keys,rotation=20,ha='right');ax.set_yticks([]);labels(ax,r,time=False)
            for b in axes[k]:
                handles,_=b.get_legend_handles_labels()
                if handles:b.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=6.5,frameon=False)
        fig.suptitle(name.replace('_',' ')+'\n'+EXECUTION_LABEL,fontsize=13)
        fig.tight_layout(rect=(0,.015,1,.95));path=folder/(name+'.png');fig.savefig(path,dpi=140);plt.close(fig)
        write(path.with_suffix('.json'),dict(image_sha256=digest(path),numeric=numeric(name,data),plotter_sha256=digest(__file__),
            source_record_hashes={str(run/'rollouts'/r['id']/'rollout.json'):digest(run/'rollouts'/r['id']/'rollout.json') for r in rows}))
    write(run/'plot_data.json',data)
    text=['<!doctype html><meta charset="utf-8"><title>DIAG-07 plan vs execution</title><style>body{font:16px system-ui;max-width:1300px;margin:30px auto}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style>',
        '<h1>DIAG-07: lateral-only plan rejection vs official MPC execution</h1>', '<p>'+EXECUTION_LABEL+'</p>',
        '<p>Hard references remain '+HARD_LABEL+'. GP body velocities are not fed forward. The actual command has only [v, omega].</p>',
        '<p>Interpretation: '+data['summary']['interpretation']+'</p><table><tr><th>Reference / provenance</th><th>Plan</th><th>Execution</th><th>Reasons</th></tr>']
    for r in rows:text.append('<tr><td>'+html.escape(title(r))+'</td><td>'+str(r['outcome']['plan_valid'])+'</td><td>'+str(r['outcome']['primary_success'])+'</td><td>'+html.escape(', '.join(r['outcome']['failure_taxonomy']))+'</td></tr>')
    text+=['</table><p>One hard event, four provenance starts; no population success-rate claim. Execution never changes plan acceptance. GUI was not run.</p>']
    for name in PLOTS:text.append(f'<h2>{name}</h2><a href="plots/{name}.json">Numeric sidecar and hashes</a><img src="plots/{name}.png">')
    with (run/'index.html').open('x') as f:f.write('\n'.join(text))
    source=read(run/'source.json')
    write(run/'review_source.json',dict(starting_sha=source['starting_sha'],execution_sha=read(run/'execution_freeze.json')['execution_sha'],
        source_json_sha256=digest(run/'source.json'),source_manifest_path=str(run/'source.json'),
        config_sha256=digest(run/'config_snapshot.yaml'),official=source['official'],limitations=data['summary']['limitations']))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        paths=[run/p for p in ['index.html','review_source.json','protocol.json','reference_manifest.json','provenance_aliases.json','config_snapshot.yaml']]
        paths+=list((run/'aggregate').glob('*'))+list(folder.glob('*'))
        for p in paths:z.write(p,p.relative_to(run))
    write(run/'artifact_hashes.json',{str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file() and p.name not in ['artifact_hashes.json','validation.json','completion.json']})
    print(dict(plots=len(PLOTS),zip_bytes=(run/'review_bundle.zip').stat().st_size),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);render(p.parse_args().run.resolve())
