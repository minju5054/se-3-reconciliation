#!/usr/bin/env python3
"""Static saved G0/G1/G2/G3 evidence. No solver or simulator calls."""
from __future__ import annotations
import argparse
import copy
import html
import json
from pathlib import Path
import shutil
import sys
import zipfile
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment
from plot_gp_se2_01 import geometry
from plot_gp_se2_diag04 import pairs,pair_label,visible_results,full_history
from plot_gp_se2_diag03 import _line
from run_gp_se2_ref01 import read,write,digest
PLOTS=('world_xy_g0_g2_g3','lateral_velocity_g0_g2_g3','forward_speed_g0_g2_g3',
    'linear_acceleration_g0_g2_g3','witness_locations','witness_margin_before_after',
    'full_grid_remaining_violations','objective_history_g2_g3','solver_outcome_summary','full_acceptance_matrix')
COLORS={'G0':'#0072b2','G1':'#999999','G2':'#e69f00','G3':'#009e73'}
LABEL='PLAN ONLY / NOT EXECUTED'


def expected_numeric(name,data):
    value=copy.deepcopy(data)
    for solve in value['solves']:
        for field in ('conditioning','margins'):solve.pop(field,None)
        for phase in ('latest','selected'):
            if solve.get(phase) is None:continue
            sample=solve[phase].pop('samples')
            if name=='world_xy_g0_g2_g3':keys=['poses_world']
            elif name in PLOTS[1:5] or name=='full_grid_remaining_violations':keys=['times_s','interval_index','local_fraction','quantities']
            else:keys=[]
            if keys:solve[phase]['samples']={k:sample[k] for k in keys}
        if name!='objective_history_g2_g3':solve.pop('history',None)
        if name!='witness_margin_before_after':
            solve.pop('witness_margins',None);solve.pop('historical_G2_witness_margins',None)
        if name!='full_grid_remaining_violations':solve.pop('remaining_motion_runs',None)
    if name!='world_xy_g0_g2_g3':value.pop('contexts',None)
    return value


def label(s):return s['grid'][:2]+(' NEW' if s['grid'].startswith('G3') else ' historical')

def layout(columns=1,height=2.7):return plt.subplots(5,columns,figsize=(15 if columns==2 else 12,5*height),squeeze=False)


def world(data,env):
    fig,axes=layout(height=3.2)
    for ax,(pair,group) in zip(axes[:,0],pairs(data['solves'])):
        group=[s for s in group if not s['grid'].startswith('G1')];c=data['contexts'][pair[0]]
        xy=np.vstack([c['old_world'],c['fresh_world']]+[r['samples']['poses_world'] for s in group for _,r,_ in visible_results(s)])[:,:2]
        lo=xy.min(0)-.3;hi=xy.max(0)+.3;clip=box(*lo,*hi)
        if env is not None:
            geometry(ax,env.workspace.intersection(clip),facecolor='#f2f7eb',edgecolor='#718768')
            geometry(ax,env.obstacles.intersection(clip),facecolor='#aaa',edgecolor='#555')
        for key,color,lab in [('old_world','#777','OLD'),('fresh_world','#ac66ac','FRESH')]:
            a=np.asarray(c[key]);ax.plot(a[:,0],a[:,1],'.:',color=color,label=lab,lw=.8)
        for s in group:
            for role,r,style in visible_results(s):
                p=np.asarray(r['samples']['poses_world']);color=COLORS[s['grid'][:2]]
                ax.plot(p[:,0],p[:,1],style,color=color,label=label(s)+' '+role,lw=1.2)
                q=p[np.linspace(0,len(p)-1,7,dtype=int)];ax.quiver(q[:,0],q[:,1],np.cos(q[:,2]),np.sin(q[:,2]),color=color,angles='xy',scale_units='xy',scale=14,width=.002)
        b=c['B_world'];g=c['goal_world'];ax.scatter(*b[:2],c='k',s=25,label='B');ax.scatter(*g[:2],c='k',marker='*',s=70,label='original goal')
        ax.add_patch(Circle(b[:2],.2,fill=False,color='k',lw=.5));ax.add_patch(Circle(b[:2],.25,fill=False,color='k',ls=':',lw=.5))
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]',title=pair_label(pair));ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7,frameon=False)
    return fig


def motion(data,q):
    fig,axes=layout(2);cfg=data['formulation_config'];tol=cfg['equality_tolerance' if q=='vy' else 'inequality_tolerance']
    unit='m/s²' if q=='ax' else 'm/s'
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        for s in group:
            if s['grid'].startswith('G1'):continue
            for role,r,style in visible_results(s):
                a=r['samples'];t=a['times_s'];y=np.asarray(a['quantities'][q]);color=COLORS[s['grid'][:2]]
                for col,ax in enumerate(axes[row]):
                    values=np.abs(y)-cfg['a_v_max'] if q=='ax' and col else y
                    _line(ax,t,values,color=color,linestyle=style,label=label(s)+' '+role,lw=1,interval_index=a['interval_index'])
                if s['grid'].startswith('G3'):
                    u=np.asarray(a['local_fraction']);mask=np.any(np.isclose(u[:,None],[0,.25,.5,.75,1],atol=1e-10,rtol=0),axis=1)
                    axes[row,0].scatter(np.asarray(t)[mask],y[mask],s=6,color=color,alpha=.5)
        for col,ax in enumerate(axes[row]):
            ax.set(xlim=(0,3),xlabel='trajectory time [s]',ylabel=(('|ax| − 2' if col and q=='ax' else q)+' ['+unit+']'));ax.grid(alpha=.2)
            ax.set_title(pair_label(pair)+(' · full range' if col==0 else ' · threshold detail'),fontsize=9)
            if q=='vy':
                for v in [-tol,tol]:ax.axhline(v,color='k',ls=':',lw=.7)
            elif q=='vx':
                for v in [-tol,0,cfg['v_max']+tol]:ax.axhline(v,color='k',ls=':',lw=.7)
            elif col:ax.axhline(tol,color='k',ls=':',lw=.7);ax.axhline(0,color='#777',lw=.5)
            else:
                for v in [-cfg['a_v_max']-tol,cfg['a_v_max']+tol]:ax.axhline(v,color='k',ls=':',lw=.7)
        if q=='vx':axes[row,1].set_ylim(-.0035,.001)
        elif q=='ax':axes[row,1].set_yscale('symlog',linthresh=tol)
        else:axes[row,1].set_ylim(-1.5e-4,1.5e-4)
    axes[0,0].legend(loc='best',fontsize=7);return fig


def witness_locations(data):
    families=list(dict.fromkeys(w['family'] for w in data['frozen_witnesses']))
    fig,axes=plt.subplots(max(1,len(families)),1,figsize=(12,3.4*max(1,len(families))),squeeze=False)
    for ax,family in zip(axes[:,0],families):
        rows=[w for w in data['frozen_witnesses'] if w['family']==family]
        for row in rows:
            ax.scatter(row['time_s'],row['interval_index'],s=45,c=COLORS['G3'])
            ax.annotate(f"w{row['witness_row_index']} · u={row['local_fraction']:.6g}",
                (row['time_s'],row['interval_index']),xytext=(8,8),textcoords='offset points',fontsize=9)
        ax.set(xlim=(0,3),xlabel='trajectory time [s]',ylabel='support interval index',title=family+' · one frozen common union');ax.grid(alpha=.3)
    if not families:axes[0,0].text(.5,.5,'No violating motion witnesses',ha='center')
    return fig


def witness_margins(data):
    fig,axes=layout(height=2.5);tol=data['formulation_config']['inequality_tolerance']
    for ax,(pair,group) in zip(axes[:,0],pairs(data['solves'])):
        new=group[-1];rows=new['witness_margins']['latest_iterate'];old=group[-2]
        before=[w['new_margin'] for w in new['historical_G2_witness_margins']]
        x=np.arange(len(rows));ax.plot(x,before,'o--',color=COLORS['G2'],label='G2 historical at same fixed locations')
        ax.plot(x,[w['new_margin'] for w in rows],'s-',color=COLORS['G3'],label='G3 latest at same fixed locations')
        ax.axhline(-tol,color='k',ls=':',label='unchanged numerical allowance');ax.set_yscale('symlog',linthresh=tol)
        ax.set_xticks(x,[str(w['witness_row_index'])+' '+w['family'].replace('linear_','') for w in rows],rotation=12,fontsize=8)
        ax.set(title=pair_label(pair),ylabel='signed margin [family unit]');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=7);return fig


def remaining(data):
    fig,axes=layout(2)
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        s=group[-1];a=s['latest']['samples'];cfg=data['formulation_config'];t=np.asarray(a['times_s']);tol=cfg['inequality_tolerance']
        for ax,q,bound in zip(axes[row],['vx','ax'],[0,cfg['a_v_max']]):
            y=-np.asarray(a['quantities'][q])-tol if q=='vx' else np.abs(a['quantities'][q])-bound-tol
            _line(ax,t,y,label='G3 latest',color=COLORS['G3'],lw=1,interval_index=a['interval_index']);ax.axhline(0,color='k',ls=':')
            positive=y>0;ax.scatter(t[positive],y[positive],s=3,c='#d55e00')
            ax.set_yscale('symlog',linthresh=tol);ax.set(xlim=(0,3),xlabel='trajectory time [s]',ylabel='tolerance residual ['+('m/s' if q=='vx' else 'm/s²')+']',title=pair_label(pair)+' · '+q);ax.grid(alpha=.2)
            if not positive.any():ax.text(.98,.9,'No observed violation',ha='right',transform=ax.transAxes,fontsize=8)
    return fig


def objective(data):
    fig,axes=layout(2)
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        for s in group[-2:]:
            h=s['history'];color=COLORS[s['grid'][:2]]
            axes[row,0].plot([v['elapsed_s'] for v in h],[v['objective'] for v in h],color=color,label=label(s))
            best=full_history(s);axes[row,1].step(best['elapsed_s'],[np.nan if v is None else v for v in best['best_full_feasible_objective']],where='post',color=color,label=label(s))
        axes[row,0].set_yscale('log');axes[row,0].set_title(pair_label(pair)+' · callbacks',fontsize=9)
        axes[row,1].set_title('Best full-feasible retained objective · checked post-solve',fontsize=9)
        if not any(any(v is not None for v in full_history(s)['best_full_feasible_objective']) for s in group[-2:]):axes[row,1].text(.5,.5,'N/A: no retained full-feasible iterate',ha='center',transform=axes[row,1].transAxes)
        if pair[0]=='BENIGN_CONTROL':
            for ax in axes[row]:ax.axhline(group[-1]['outcome']['initial_objective'],color='#777',ls=':',label='known-valid seed')
        for ax in axes[row]:ax.set(xlabel='prepared solve wall time [s]',ylabel='original objective');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);return fig


def termination(data):
    fig,ax=plt.subplots(figsize=(16,10));ax.axis('off');rows=[]
    for s in data['solves']:
        o=s['outcome'];rows.append([pair_label((s['case_role'],s['method'],s['initialization'])),label(s),o['solver_status'],o['iterations'],o['latest_grid_feasible'],o['latest_dense_feasible'],o['latest_original_full_feasible'],o['selected_full_feasible'],f"{o['latest_objective']:.9g}",f"{o['solve_wall_time_s']:.2f}"])
    table=ax.table(cellText=rows,colLabels=['case / method / seed','grid / provenance','status','iter','grid','dense','latest full','selected full','J latest','solve s'],loc='center',cellLoc='center',colWidths=[.20,.11,.04,.04,.05,.05,.075,.08,.12,.06]);table.auto_set_font_size(False);table.set_fontsize(8);table.scale(1,1.85)
    ax.set_title('SLSQP status and plan feasibility are distinct. G0/G1/G2 timings are historical.');return fig


def acceptance(data):
    from plot_gp_se2_diag05 import acceptance as plot_acceptance
    # This helper only consumes saved outcome rows; no fixed grid assumptions or mutation.
    fig=plot_acceptance(data);fig.set_size_inches(15,11);return fig


def generate(run,*,environment=None):
    run=Path(run)
    if any((run/p).exists() for p in ('plots','index.html','review_bundle.zip')):raise FileExistsError('no plot overwrite')
    data=read(run/'plot_input.json');groups=pairs(data['solves'])
    if len(groups)!=5 or len(data['solves'])!=20:raise ValueError('all twenty historical/new outcomes required')
    if any([s['grid'][:2] for s in group]!=['G0','G1','G2','G3'] for _,group in groups):raise ValueError('grid/provenance order')
    source=read(run/'source.json')
    if environment is None:environment=HospitalEnvironment.load(source['environment_path'])
    (run/'plots').mkdir();hashes={str(run/p):digest(run/p) for p in ('source.json','protocol.json','config_snapshot.yaml','plot_input.json','witness_selection/frozen_union.json','execution_freeze.json')}
    callbacks=[lambda d:world(d,environment),lambda d:motion(d,'vy'),lambda d:motion(d,'vx'),lambda d:motion(d,'ax'),witness_locations,witness_margins,remaining,objective,termination,acceptance]
    for name,fn in zip(PLOTS,callbacks):
        numeric=expected_numeric(name,data);fig=fn(numeric);fig.suptitle(name.replace('_',' ')+'\n'+LABEL,fontsize=13)
        fig.tight_layout(rect=(0,.01,1,.95));path=run/'plots'/(name+'.png');fig.savefig(path,dpi=140,facecolor='white');plt.close(fig)
        write(path.with_suffix('.json'),dict(numeric_data=numeric,source_hashes=hashes,image_sha256=digest(path),plotter_sha256=digest(__file__),label=LABEL))
    text=['<!doctype html><html><meta charset="utf-8"><title>GP-SE2-DIAG-06</title><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}</style>',
        '<h1>Frozen extremum-witness motion-inequality refinement</h1><p>'+LABEL+'</p>',
        '<p>G0/G1/G2 historical; only five new G3 solves. One hard event and one benign control. Witnesses are observed sampled extrema derived from the development event, not a general algorithm or continuous certificate. Original thresholds unchanged; no execution.</p>',
        '<pre>'+html.escape(json.dumps(data['summary'],indent=2))+'</pre>']
    for name in PLOTS:text.append(f'<h2>{name}</h2><a href="plots/{name}.json">Numeric records and hashes</a><img src="plots/{name}.png">')
    (run/'index.html').write_text('\n'.join(text)+'</html>');bundle=run/'review_bundle';bundle.mkdir()
    names=['index.html','source.json','protocol.json','config_snapshot.yaml','experiment_manifest.json','execution_freeze.json','historical_baselines/manifest.json','witness_selection/frozen_union.json','witness_selection/validation.json']
    names += [str(p.relative_to(run)) for parent in ('plots','aggregate') for p in (run/parent).glob('*') if p.is_file()]
    for name in names:
        target=bundle/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(run/name,target)
    (bundle/'README.txt').write_text('PLAN ONLY / NOT EXECUTED\nOne frozen union; five new G3 starts. Historical baselines not rerun. Original acceptance unchanged. Finite sampled extrema are not continuous guarantees. No RGB, full historical arrays, environment export or external source included.\n')
    with zipfile.ZipFile(run/'review_bundle.zip','x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(bundle.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(bundle))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);generate(p.parse_args().run)
