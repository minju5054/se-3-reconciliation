#!/usr/bin/env python3
"""Static G3 historical / G4 new comparison; no solver or simulator calls."""
from __future__ import annotations
import argparse
import html
import os
from pathlib import Path
import sys
import zipfile
os.environ.setdefault('MPLCONFIGDIR','/tmp/gp_diag08_mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import ListedColormap
import numpy as np
from shapely.geometry import box
from run_gp_se2_diag08 import read,write,digest,clean,D7
from plot_gp_se2_01 import geometry
from reconciliation.gp_se2_environment import HospitalEnvironment
PLOTS=['world_xy_g3_vs_g4','goal_region_terminal_geometry','endpoint_error_and_reserve',
       'endpoint_tracking_error','execution_goal_error_over_time','plan_goal_track_vector_decomposition',
       'selected_reference_progress','command_v_over_time','command_omega_over_time','plan_vs_execution_acceptance_matrix']
COLORS={'G3':'#3578b5','G4':'#d66e20'}


def plot_data(run):
    records=read(run/'comparison_records.json');manifest=read(run/'reference_manifest.json');out=[]
    starts=read(run/'experiment_manifest.json')['starts']
    for r,start,m in zip(records,starts,manifest['records']):
        ctx=read(run/'contexts'/(r['case_id'].replace('/','__')+'.json'))
        item=dict(id=r['reference_id'],case_id=r['case_id'],context=ctx,classification=r['classification'],
            admission=r['admission'],variants={})
        for grid in ['G3','G4']:
            base=r[grid];folder=D7/'rollouts'/start['historical_uid'] if grid=='G3' else (None if r['unique_reference_id'] is None else run/'rollouts'/r['unique_reference_id'])
            rollout=None if folder is None else read(folder/'rollout.json')
            ref=np.load(start['historical_reference']['world_reference_path'] if grid=='G3' else m['world_reference_path'],allow_pickle=False)
            v=dict(provenance=base['provenance'],reference=ref,plan=base['plan'],geometry=base['geometry'],
                execution=None if base['evaluation'] is None else base['evaluation']['execution'],diagnostics=base['diagnostics'],
                actual=None if rollout is None else [s['pose_world'] for s in rollout['states']],
                actual_times=None if rollout is None else [s['time_s'] for s in rollout['states']],
                predictions=None if rollout is None else [dict(issue_time_s=s['time_s'],endpoint=None if s['prediction_world'] is None else s['prediction_world'][-1]) for s in rollout['controller_reference_selections']],
                execution_flags=None if base['evaluation'] is None else dict(success=base['evaluation']['primary_success'],
                    clearance=base['evaluation']['environment']['clearance_valid'],workspace=base['evaluation']['environment']['workspace_known'],
                    route=base['evaluation']['route']['valid'],goal=base['evaluation']['execution']['goal_reached'],
                    dwell=base['evaluation']['execution']['terminal_goal_dwell_pass'],motion=base['evaluation']['execution']['motion_limits_pass'],
                    controller=base['evaluation']['execution']['controller_failure_count']==0))
            item['variants'][grid]=v
        out.append(item)
    return clean(dict(records=out,summary=read(run/'aggregate/summary.json'),source_sha256=digest(run/'source.json'),
        config_sha256=digest(run/'config_snapshot.yaml'),freeze_sha256=digest(run/'execution_freeze.json')))


def numeric(name,r):
    fields={PLOTS[0]:['reference','actual','plan'],PLOTS[1]:['reference','actual','geometry'],
        PLOTS[2]:['plan','geometry'],PLOTS[3]:['geometry'],PLOTS[4]:['diagnostics'],
        PLOTS[5]:['geometry'],PLOTS[6]:['diagnostics','predictions'],PLOTS[7]:['diagnostics'],
        PLOTS[8]:['diagnostics'],PLOTS[9]:['plan','execution_flags']}
    return dict(id=r['id'],case_id=r['case_id'],classification=r['classification'],admission=r['admission'],
        context=r['context'] if name in [PLOTS[0],PLOTS[1],PLOTS[5]] else None,
        variants={g:{k:v[k] for k in ['provenance']+fields[name]} for g,v in r['variants'].items()})


def render(run):
    import time
    start=time.perf_counter();target=run/'plots';target.mkdir(exist_ok=False)
    data=plot_data(run);write(run/'presentation/data.json',data)
    env=HospitalEnvironment.load(read(run/'source.json')['environment_path']);limits={};files=[]
    for caseid in {r['case_id'] for r in data['records']}:
        arrays=[]
        for r in data['records']:
            if r['case_id']!=caseid:continue
            arrays.extend([r['context']['context']['old_world'],r['context']['context']['fresh_world']])
            arrays.extend(v['reference'] for v in r['variants'].values())
            arrays.extend(v['actual'] for v in r['variants'].values() if v['actual'] is not None)
        a=np.vstack(arrays)[:,:2];lo=a.min(0)-.28;hi=a.max(0)+.28;mid=(hi+lo)/2;span=max(hi-lo)
        limits[caseid]=(mid-span/2,mid+span/2)
    for r in data['records']:
        folder=target/r['id'];folder.mkdir();ctx=r['context']['context'];goal=np.asarray(r['context']['goal_route']['goal_world'])
        for name in PLOTS:
            two=name in [PLOTS[2],PLOTS[4],PLOTS[6]]
            fig,axes=plt.subplots(1,2 if two else 1,figsize=(12,7),squeeze=False);ax=axes[0,0]
            def xy(a,**kw):
                a=np.asarray(a);ax.plot(a[:,0],a[:,1],**kw)
            if name in [PLOTS[0],PLOTS[1]]:
                lo,hi=limits[r['case_id']] if name==PLOTS[0] else (goal[:2]-.24,goal[:2]+.24)
                clip=box(*lo,*hi);geometry(ax,env.workspace.intersection(clip),facecolor='#f4f6ef',edgecolor='#84946c')
                geometry(ax,env.obstacles.intersection(clip),facecolor='#aaa',edgecolor='#555')
                if name==PLOTS[0]:
                    xy(ctx['old_world'],color='#8b8b8b',ls=':',label='original OLD')
                    xy(ctx['fresh_world'],color='#ae70a0',ls='--',label='original FRESH')
                    ax.scatter(*ctx['B_world'][:2],c='k',s=30,label='original B')
                for g,v in r['variants'].items():
                    xy(v['reference'],color=COLORS[g],ls='--',lw=1,label=g+' plan rows')
                    ax.scatter(*np.asarray(v['reference'])[-1,:2],marker='s',s=40,color=COLORS[g],label=g+' plan endpoint')
                    if v['actual'] is not None:
                        xy(v['actual'],color=COLORS[g],lw=2,label=g+' executed')
                        ax.scatter(*np.asarray(v['actual'])[-1,:2],marker='X',s=60,color=COLORS[g],label=g+' executed final')
                ax.add_patch(Circle(goal[:2],.15,fill=False,color='green',ls=':',label='execution goal radius 0.15 m'))
                ax.add_patch(Circle(goal[:2],.11,fill=False,color='green',ls='--',label='G4 plan goal radius 0.11 m'))
                ax.scatter(*goal[:2],marker='*',color='green',s=100,label='original goal')
                ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal')
                if name==PLOTS[1]:ax.text(.01,.02,'Circles are goal tolerances, not footprint or safety radii.',transform=ax.transAxes,fontsize=8)
            elif name==PLOTS[2]:
                for j,(which,label) in enumerate([('error','position error [m]'),('reserve','reserve inside original 0.15 m region [m]')]):
                    a=axes[0,j]
                    for n,(g,v) in enumerate(r['variants'].items()):
                        vals=[v['plan']['endpoint_position_error_m'],None if v['geometry'] is None else v['geometry']['execution_error_m']]
                        vals=[None if x is None else x if which=='error' else .15-x for x in vals]
                        for k,value in enumerate(vals):
                            x=k+(n-.5)*.28
                            if value is None:a.text(x,0,'N/A',ha='center')
                            else:
                                a.bar(x,value,width=.25,color=COLORS[g],label=g if k==0 else None)
                                a.annotate(f'{value:.6f}',(x,value),xytext=(0,6 if value>=0 else -16),textcoords='offset points',ha='center',fontsize=9)
                    a.axhline(.15 if which=='error' else 0,color='k',ls=':',label='execution threshold')
                    a.axhline(.11 if which=='error' else .04,color='green',ls='--',label='G4 plan threshold')
                    a.set(xticks=[0,1],xticklabels=['Plan','Execution'],ylabel=label);a.margins(y=.25)
            elif name==PLOTS[3]:
                for i,(g,v) in enumerate(r['variants'].items()):
                    value=None if v['geometry'] is None else v['geometry']['tracking_displacement_m']
                    if value is None:ax.text(i,.005,'N/A — not executed',ha='center')
                    else:
                        ax.bar(i,value,color=COLORS[g]);ax.text(i,value+.001,f'{value:.8f}',ha='center')
                ax.axhline(.04,color='green',ls=':',label='observed-scale rationale, not tracking constraint')
                ax.set(xticks=[0,1],xticklabels=['G3 historical','G4 new'],ylabel='final execution-to-plan endpoint distance [m]');ax.set_ylim(bottom=0);ax.margins(y=.2)
            elif name==PLOTS[4]:
                for j,key in enumerate(['position_error_m','signed_goal_yaw_error_rad']):
                    a=axes[0,j]
                    for g,v in r['variants'].items():
                        d=v['diagnostics']
                        if d is not None:a.plot(d['error_times_s'],d[key],color=COLORS[g],label=g)
                    limit=.15 if j==0 else np.pi/12;a.axhline(limit,color='k',ls=':',label='original tolerance')
                    if j:a.axhline(-limit,color='k',ls=':')
                    a.axvspan(2.8,3.,alpha=.12,color='green',label='required final dwell window')
                    a.set(xlim=(0,3),xlabel='time from B [s]',ylabel='goal position error [m]' if j==0 else 'wrapped goal yaw error [rad]')
            elif name==PLOTS[5]:
                for g,v in r['variants'].items():
                    geom=v['geometry']
                    if geom is None:continue
                    ep,et,ee=[np.asarray(geom[k]) for k in ['e_plan_m','e_track_m','e_exec_m']]
                    for origin,delta,style,label in [(np.zeros(2),ep,'--','e_plan'),(ep,et,':','e_track'),(np.zeros(2),ee,'-','e_exec')]:
                        ax.annotate('',xy=origin+delta,xytext=origin,arrowprops=dict(arrowstyle='->',color=COLORS[g],linestyle=style,lw=2))
                        ax.plot([origin[0],(origin+delta)[0]],[origin[1],(origin+delta)[1]],ls=style,color=COLORS[g],label=g+' '+label)
                ax.scatter(0,0,marker='*',color='green',s=80,label='goal origin');ax.set(xlabel='goal-relative x [m]',ylabel='goal-relative y [m]');ax.set_aspect('equal');ax.margins(.3)
            elif name==PLOTS[6]:
                for g,v in r['variants'].items():
                    d=v['diagnostics']
                    if d is None:continue
                    s=d['selection'];idx=np.asarray(s['selected_indices'])
                    for j,(key,label) in enumerate([('nearest_indices','nearest row'),('last','last selected row')]):
                        ax.step(s['times_s'],s[key] if key!='last' else idx[:,-1],where='post',color=COLORS[g],ls='--' if j==0 else '-',label=g+' '+label)
                    axes[0,1].step(s['times_s'],np.asarray(s['final_reference_row_in_horizon'],int),where='post',color=COLORS[g],label=g+' final row included')
                ax.set(xlim=(0,3),xlabel='solve issue time [s]',ylabel='installed GP row index (row order only)')
                axes[0,1].set(xlim=(0,3),ylim=(-.05,1.15),xlabel='solve issue time [s]',ylabel='final reference row in 5-target horizon')
            elif name in [PLOTS[7],PLOTS[8]]:
                j=0 if name==PLOTS[7] else 1
                for g,v in r['variants'].items():
                    d=v['diagnostics']
                    if d is None:continue
                    cmd=np.asarray(d['command_values']);ax.step(d['command_times_s']+[3.],np.r_[cmd[:,j],cmd[-1,j]],where='post',color=COLORS[g],label=g+' applied command')
                ax.set(xlim=(0,3),xlabel='time from B [s]',ylabel='v [m/s]' if j==0 else 'omega [rad/s]')
            else:
                labels=['Plan\nlateral','Other\nplan','Original\nfull plan','G4\nreserve','Exec\nsafety/route','Exec\nmotion','Exec\ngoal','Exec\ndwell','Full\nexecution']
                matrix=[]
                for g,v in r['variants'].items():
                    f=v['plan']['flags'];e=v['execution_flags']
                    matrix.append([f['lateral_velocity'],all(x for k,x in f.items() if k!='lateral_velocity'),v['plan']['original_full_feasible'],
                        None if g=='G3' else r['admission']['planning_endpoint_reserve_pass'],
                        *([None]*5 if e is None else [e['clearance'] and e['workspace'] and e['route'],e['motion'],e['goal'],e['dwell'],e['success']])])
                values=np.array([[-1 if v is None else int(v) for v in row] for row in matrix]);ax.imshow(values,vmin=-1,vmax=1,cmap=ListedColormap(['#ddd','#e99b8d','#a3d2ac']),aspect='auto')
                for i,row in enumerate(matrix):
                    for j,v in enumerate(row):ax.text(j,i,'N/A' if v is None else 'PASS' if v else 'FAIL',ha='center',va='center',fontsize=9)
                ax.set_xticks(range(len(labels)),labels);ax.set_yticks([0,1],['G3 historical','G4 new'])
            for a in axes[0]:
                a.grid(alpha=.15)
                handles,_=a.get_legend_handles_labels()
                if handles:a.legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1) if not two else (0,1.20),frameon=False)
            title=name.replace('_',' ')+'\n'+r['id']
            fig.suptitle(title,fontsize=14,y=.97)
            status=r['admission']['label']+(' | G4 NOT EXECUTED' if r['variants']['G4']['actual'] is None else '')
            fig.text(.07,.85,status,fontsize=10,color='#923f28',va='top')
            fig.text(.07,.80,r['classification'],fontsize=10,va='top')
            fig.text(.07,.025,'G3 historical / G4 new | OFFLINE COUNTERFACTUAL, NOT ONLINE | planned velocity is not applied command',fontsize=9)
            fig.subplots_adjust(left=.09,right=.72 if not two and name!=PLOTS[9] else .96,top=.68,bottom=.20 if name==PLOTS[9] else .15,wspace=.36)
            path=folder/(name+'.png');fig.savefig(path,dpi=140);plt.close(fig)
            write(path.with_suffix('.json'),dict(numeric=numeric(name,r),image_sha256=digest(path),source_sha256=data['source_sha256'],
                config_sha256=data['config_sha256'],freeze_sha256=data['freeze_sha256']))
            files.extend([path,path.with_suffix('.json')])
    content=['<!doctype html><meta charset="utf-8"><title>DIAG-08 endpoint reserve</title><style>body{font:16px system-ui;max-width:1200px;margin:30px auto}img{width:100%}table{border-collapse:collapse}td,th{padding:9px;border:1px solid #ccc}</style>',
        '<h1>DIAG-08: fixed 4 cm planning endpoint reserve</h1><p>Execution: original 15 cm, 15°, final 0.20 s dwell; 3 s horizon. Planning: one appended 11 cm position row.</p>',
        '<p><b>'+html.escape(data['summary']['interpretation'])+'</b></p><p>One hard and one benign development event. G3 results are historical; G4 is new. Full plan validity and execution success remain separate.</p>',
        '<table><tr><th>Start</th><th>G4 plan valid</th><th>11 cm pass</th><th>G3 → G4 execution error [m]</th><th>Outcome</th></tr>']
    for r in data['records']:
        v=r['variants'];old=v['G3']['geometry']['execution_error_m'];new=None if v['G4']['geometry'] is None else v['G4']['geometry']['execution_error_m']
        content.append(f"<tr><td>{r['id']}</td><td>{r['admission']['plan_valid']}</td><td>{r['admission']['planning_endpoint_reserve_pass']}</td><td>{old:.8f} → {('N/A' if new is None else f'{new:.8f}')}</td><td>{r['classification']}</td></tr>")
    content.append('</table>')
    for r in data['records']:
        content.append('<h2>'+r['id']+'</h2>')
        for name in PLOTS:
            rel='plots/'+r['id']+'/'+name
            content.append(f'<details {"open" if name in [PLOTS[1],PLOTS[4],PLOTS[9]] else ""}><summary>{name}</summary><a href="{rel}.json">Numbers and source hashes</a><img loading="lazy" src="{rel}.png"></details>')
    with (run/'index.html').open('x') as f:f.write('\n'.join(content))
    write(run/'plot_artifacts.json',{str(p.relative_to(run)):digest(p) for p in files+[run/'index.html',run/'presentation/data.json']})
    write(run/'rendering_completed.json',dict(wall_s=time.perf_counter()-start,PNG_count=len(files)//2,new_runtime=0))
    print(dict(PNG_count=len(files)//2,index=str(run/'index.html')),flush=True)


def package(run):
    if not read(run/'validation.json')['valid']:raise ValueError('valid saved-record audit required before packaging')
    source=read(run/'source.json')
    write(run/'review_source.json',dict(source_sha256=digest(run/'source.json'),starting_sha=source['starting_sha'],
        execution_sha=read(run/'execution_freeze.json')['execution_sha'],original_config_sha256=digest(run/'config_snapshot.yaml'),
        witness_sha256=digest(run/'margin/frozen_g3_witness_union.json'),validation_sha256=digest(run/'validation.json'),
        preserved_source_file_count=len(source['preserved_hashes']),full_source_inventory=str(run/'source.json'),
        no_raw_RGB_environment_checkpoint=True))
    files=[run/n for n in ['index.html','protocol.json','config_snapshot.yaml','validation.json','review_source.json','plot_artifacts.json']]
    files += [p for folder in ['plots','aggregate','margin'] for p in (run/folder).rglob('*') if p.is_file() and
        (folder!='margin' or p.name in ['definition.json','derivative_validation.json'])]
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(files):archive.write(p,p.relative_to(run))
    with zipfile.ZipFile(run/'review_bundle.zip') as archive:
        errors=[str(p) for p in files if archive.read(str(p.relative_to(run)))!=p.read_bytes()]
    write(run/'bundle_validation.json',dict(valid=not errors,errors=errors,zip_sha256=digest(run/'review_bundle.zip'),
        bytes=(run/'review_bundle.zip').stat().st_size,member_count=len(files)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);p.add_argument('--package',action='store_true');a=p.parse_args()
    (package if a.package else render)(a.run.resolve())
