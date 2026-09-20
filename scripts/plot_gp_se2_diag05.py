#!/usr/bin/env python3
"""Static DIAG05 comparison from saved numbers only; no optimizer or rollout."""
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
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diag04_constraints import MOTION_ROWS
from plot_gp_se2_01 import geometry
from plot_gp_se2_diag03 import _quantity,_line,_bands
from plot_gp_se2_diag04 import pairs,pair_label,visible_results,full_history
from run_gp_se2_ref01 import read,write,digest
GRIDS=('G0_ORIGINAL','G1_QUARTER','G2_QUARTER_INEQUALITY_ONLY')
COLORS=dict(zip(GRIDS,('#0072b2','#d55e00','#009e73')))
PLOTS=('world_xy_comparison','lateral_velocity_comparison','forward_speed_comparison',
    'linear_acceleration_comparison','angular_acceleration_comparison','constraint_margin_summary',
    'objective_history','solver_termination_summary','equality_singular_values','full_acceptance_matrix')
LABEL='PLAN ONLY / NOT EXECUTED'


def verify_payload(data):
    groups=pairs(data['solves'])
    if len(data['solves'])!=15 or len({s['solve_id'] for s in data['solves']})!=15 or len(groups)!=5:
        raise ValueError('five G0/G1 historical + G2 new triples required')
    if any(tuple(s['grid'] for s in group)!=GRIDS for _,group in groups):raise ValueError('grid order')
    for _,group in groups:
        if [s['provenance'] for s in group]!=['HISTORICAL_DIAG04_NOT_RERUN']*2+['NEW_DIAG05_G2']:
            raise ValueError('historical/new provenance')
    return groups


def expected_numeric(name,data):
    """Each image carries the exact numbers it consumes, with shared input hash."""
    value=copy.deepcopy(data)
    fields={'world_xy_comparison':['poses_world'],
        'lateral_velocity_comparison':['times_s','interval_index','local_u','body_twists'],
        'forward_speed_comparison':['times_s','interval_index','local_u','body_twists'],
        'linear_acceleration_comparison':['times_s','interval_index','local_u','body_accelerations'],
        'angular_acceleration_comparison':['times_s','interval_index','local_u','body_accelerations']}
    for solve in value['solves']:
        for phase in ('latest','selected'):
            if solve.get(phase) is not None:
                samples=solve[phase].pop('samples',None)
                if name in fields and samples is not None:
                    solve[phase]['samples']={k:samples[k] for k in fields[name]}
        if name!='objective_history':solve.pop('history',None)
        if name!='equality_singular_values':solve.pop('conditioning',None)
        if name!='constraint_margin_summary':solve.pop('margins',None)
    if name!='world_xy_comparison':value.pop('contexts',None)
    return value


def layout(columns=1,height=2.6):
    return plt.subplots(5,columns,figsize=(14 if columns==2 else 12,5*height),squeeze=False)


def key(fig):
    handles=[Line2D([],[],color=COLORS[g],label=g[:2]+(' historical DIAG04' if i<2 else ' new inequality-only')) for i,g in enumerate(GRIDS)]
    handles += [Line2D([],[],color='black',ls='-',label='latest (or latest = selected)'),Line2D([],[],color='black',ls='--',label='distinct selected')]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.014),ncol=3,fontsize=8,frameon=False)


def world(data,env):
    fig,axes=layout(height=3.2)
    for ax,(pair,group) in zip(axes[:,0],pairs(data['solves'])):
        context=data['contexts'][pair[0]]
        paths=[np.asarray(context[k]) for k in ('old_world','fresh_world')]
        paths += [np.asarray(r['samples']['poses_world']) for s in group for _,r,_ in visible_results(s)]
        xy=np.vstack(paths)[:,:2];lo=xy.min(0)-.35;hi=xy.max(0)+.35;clip=box(*lo,*hi)
        if env is not None:
            geometry(ax,env.workspace.intersection(clip),facecolor='#f2f7eb',edgecolor='#718768')
            geometry(ax,env.obstacles.intersection(clip),facecolor='#aaa',edgecolor='#555')
        for field,color,label in [('old_world','#888','OLD'),('fresh_world','#ac66ac','FRESH')]:
            a=np.asarray(context[field]);ax.plot(a[:,0],a[:,1],'.:',color=color,label=label,lw=.8)
        for s in group:
            for role,r,style in visible_results(s):
                p=np.asarray(r['samples']['poses_world']);color=COLORS[s['grid']]
                ax.plot(p[:,0],p[:,1],ls=style,color=color,label=s['grid'][:2]+' '+role,lw=1.2)
                v=p[np.linspace(0,len(p)-1,7,dtype=int)]
                ax.quiver(v[:,0],v[:,1],np.cos(v[:,2]),np.sin(v[:,2]),color=color,angles='xy',scale_units='xy',scale=12,width=.0025)
        b=context['B_world'];goal=context['goal_world'];ax.scatter(*b[:2],c='k',s=24,label='B');ax.scatter(*goal[:2],c='k',marker='*',s=75,label='original goal')
        ax.add_patch(Circle(b[:2],.20,fill=False,color='k',lw=.6));ax.add_patch(Circle(b[:2],.25,fill=False,color='k',ls=':',lw=.6))
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]',title=pair_label(pair));ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7,frameon=False)
    key(fig);return fig


def motion(data,quantity):
    # Full range and a fixed physical-bound detail use the SAME saved numbers.
    units={'v_y':'m/s','v_x':'m/s','a_x':'m/s²','a_omega':'rad/s²'}
    fig,axes=layout(2);cfg=data['formulation_config']
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        small=[_quantity(r['samples'],quantity) for s in group if s['grid']!='G1_QUARTER' for _,r,_ in visible_results(s)]
        for col,ax in enumerate(axes[row]):
            for s in group:
                for role,r,style in visible_results(s):
                    samples=r['samples'];y=_quantity(samples,quantity);t=np.asarray(samples['times_s']);u=np.asarray(samples['local_u']);color=COLORS[s['grid']]
                    _line(ax,t,y,label=s['grid'][:2]+' '+role,color=color,linestyle=style,lw=1.,interval_index=samples['interval_index'])
                    for fractions,marker in [([0.,.5,1.],'o'),([.25,.75],'x')]:
                        mask=np.any(np.isclose(u[:,None],fractions,rtol=0,atol=1e-10),axis=1)
                        ax.scatter(t[mask],y[mask],marker=marker,s=7,color=color,linewidths=.4,**({'facecolors':'none'} if marker=='o' else {}))
            _bands(ax,quantity,cfg,label=False);ax.set(xlim=(0,3),xlabel='trajectory time [s]',ylabel=quantity+' ['+units[quantity]+']');ax.grid(alpha=.2)
            ax.set_title(pair_label(pair)+(' · full range' if col==0 else ' · G0/G2 detail (G1 may extend outside)'),fontsize=9)
            ax.ticklabel_format(axis='y',style='sci',scilimits=(-3,3),useOffset=False)
        tol=cfg['equality_tolerance'] if quantity=='v_y' else cfg['inequality_tolerance']
        if quantity=='v_y':lo,hi=-max(max(np.max(np.abs(v)) for v in small),tol)*1.2,max(max(np.max(np.abs(v)) for v in small),tol)*1.2
        elif quantity=='v_x':lo,hi=-max(-min(np.min(v) for v in small),tol*5)*1.2,max(-min(np.min(v) for v in small),tol*5)*2.5
        else:
            bound=cfg['a_v_max' if quantity=='a_x' else 'a_w_max'];excess=max(max(np.max(np.abs(v))-bound for v in small),tol*5)
            # Signed residual against the nearest absolute bound, displayed separately below.
            lo,hi=-bound-excess*2,bound+excess*2
        axes[row,1].set_ylim(lo,hi)
        if quantity in ('a_x','a_omega'):
            ax=axes[row,1];ax.clear()
            bound=cfg['a_v_max' if quantity=='a_x' else 'a_w_max']
            for s in group:
                for _,r,style in visible_results(s):
                    sm=r['samples'];y=np.abs(_quantity(sm,quantity))-bound
                    _line(ax,sm['times_s'],y,label=s['grid'][:2],color=COLORS[s['grid']],linestyle=style,lw=1,interval_index=sm['interval_index'])
            ax.axhline(tol,color='k',ls=':',lw=.7);ax.axhline(0,color='#777',lw=.5)
            ax.set_yscale('symlog',linthresh=tol);ax.set(xlim=(0,3),xlabel='trajectory time [s]',ylabel='|'+quantity+'| − bound ['+units[quantity]+']',title='Signed bound excess · symlog; dotted = original tolerance');ax.grid(alpha=.2)
    fig.text(.5,.057,'○ support / midpoint; × quarter queries. G2 quarter lateral values are diagnostics, NOT equality constraints.',ha='center',fontsize=8)
    key(fig);return fig


def margins(data):
    fig,axes=plt.subplots(4,2,figsize=(13,13))
    new=[s for s in data['solves'] if s['grid']==GRIDS[2]]
    for k,(ax,row) in enumerate(zip(axes.flat,MOTION_ROWS)):
        for phase,marker in [('initial','o'),('latest_iterate','x'),('selected','s')]:
            ys=[s['margins'][phase][k]['minimum_margin'] if phase in s['margins'] else np.nan for s in new]
            ax.plot(range(5),ys,marker=marker,label=phase,lw=.9)
        tol=data['formulation_config']['inequality_tolerance'];ax.axhline(-tol,color='k',ls=':',label='−feasibility tolerance')
        ax.set_yscale('symlog',linthresh=tol);ax.set_xticks(range(5),['H M2/I0','H M2/I1','H M3/I0','H M3/I1','Benign'],rotation=12)
        ax.set(title=row[0],ylabel='minimum signed margin ['+row[2]+']');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);return fig


def objective(data):
    fig,axes=layout(2)
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        for s in group:
            h=s['history'];color=COLORS[s['grid']]
            axes[row,0].plot([v['elapsed_s'] for v in h],[v['objective'] for v in h],color=color,lw=1)
            b=full_history(s);axes[row,1].step(b['elapsed_s'],[np.nan if v is None else v for v in b['best_full_feasible_objective']],where='post',color=color)
        for ax in axes[row]:ax.set(xlabel='prepared solve wall time [s]',ylabel='original objective');ax.grid(alpha=.2)
        axes[row,0].set(title=pair_label(pair)+' · all callbacks',yscale='log')
        axes[row,1].set_title('Best retained full-feasible objective · certified post-solve',fontsize=9)
        if not any(any(v is not None for v in full_history(s)['best_full_feasible_objective']) for s in group):axes[row,1].text(.5,.5,'N/A: no full-feasible retained iterate',ha='center',transform=axes[row,1].transAxes)
        if pair[0]=='BENIGN_CONTROL':
            for ax in axes[row]:ax.axhline(group[2]['outcome']['initial_objective'],color='#555',ls=':',label='known-valid seed');ax.legend(fontsize=7)
    key(fig);return fig


def termination(data):
    fig,ax=plt.subplots(figsize=(15,8));ax.axis('off')
    labels=['case / method / seed','grid / provenance','status','iterations','grid','dense','latest full','selected full','objective (latest)','solve s']
    rows=[]
    for s in data['solves']:
        o=s['outcome'];rows.append([pair_label((s['case_role'],s['method'],s['initialization'])),s['grid'][:2]+(' historical' if s['grid']!=GRIDS[2] else ' NEW'),o['solver_status'],o['iterations'],o['latest_grid_feasible'],o['latest_dense_feasible'],o['latest_original_full_feasible'],o['selected_full_feasible'],f"{o['latest_objective']:.9g}",f"{o['solve_wall_time_s']:.2f}"])
    tbl=ax.table(cellText=rows,colLabels=labels,loc='center',cellLoc='center',colWidths=[.21,.10,.045,.06,.05,.05,.075,.08,.12,.06]);tbl.auto_set_font_size(False);tbl.set_fontsize(8);tbl.scale(1,1.9)
    for i,s in enumerate(data['solves'],1):tbl[i,1].set_facecolor(COLORS[s['grid']]+'33')
    ax.set_title('SLSQP status is not feasibility proof. G0/G1 historical; G2 new.');return fig


def singular(data):
    fig,axes=layout(2)
    for row,(pair,group) in enumerate(pairs(data['solves'])):
        for s in group:
            cond=s['conditioning']
            for phase,style in [('initial',':'),('latest','-'),('selected','--')]:
                phase_data=cond.get(phase)
                if not phase_data:continue
                value=phase_data.get(s['grid'])
                if value is None:continue
                for ax,field in zip(axes[row],['raw_singular_values','scaled_singular_values']):
                    y=np.asarray(value[field]);ax.semilogy(np.arange(1,len(y)+1),np.maximum(y,1e-18),color=COLORS[s['grid']],ls=style,lw=1)
        for ax in axes[row]:ax.set(xlabel='singular value index',ylabel='σ (zeros shown at 1e−18)');ax.grid(alpha=.2)
        axes[row,0].set_title(pair_label(pair)+' · raw',fontsize=9);axes[row,1].set_title('column-scaled DIAGNOSTIC only',fontsize=9)
    fig.text(.5,.057,'At each G2 vector: J_eq(G2) = J_eq(G0) literally (30 × 150). G1: 90 rows. Different final vectors need not have equal spectra.\nDotted: initial; solid: latest; dashed: selected. No scaling is supplied to SLSQP.',ha='center',fontsize=8)
    key(fig);return fig


def acceptance(data):
    columns=['grid','dense','lateral','speed','angular speed','linear accel','angular accel','goal','workspace','obstacle','latest full','selected full']
    mapping=['lateral_velocity','linear_speed','angular_speed','linear_acceleration','angular_acceleration','original_goal','original_workspace','original_obstacle_clearance']
    cells=[];labels=[]
    for s in data['solves']:
        o=s['outcome'];bad=o['failure_flags'];values=[o['latest_grid_feasible'],o['latest_dense_feasible']]+[None if bad is None else k not in bad for k in mapping]+[o['latest_original_full_feasible'],o['selected_full_feasible']]
        cells.append([.5 if v is None else int(v) for v in values]);labels.append(pair_label((s['case_role'],s['method'],s['initialization']))+' / '+s['grid'][:2])
    fig,ax=plt.subplots(figsize=(14,9));ax.imshow(cells,cmap='RdYlGn',vmin=0,vmax=1,aspect='auto')
    for i,row in enumerate(cells):
        for j,v in enumerate(row):ax.text(j,i,'N/A' if v==.5 else 'PASS' if v else 'FAIL',ha='center',va='center',fontsize=7)
    ax.set_xticks(range(len(columns)),columns,rotation=35,ha='right');ax.set_yticks(range(len(labels)),labels,fontsize=8)
    ax.set_title('Original acceptance unchanged; no selected candidate = selected full FAIL.\nPer-family values: original independent full offset grid.');return fig


def generate(run, *, environment=None):
    run=Path(run)
    if (run/'plots').exists() or (run/'index.html').exists() or (run/'review_bundle.zip').exists():raise FileExistsError('no plot overwrite')
    data=read(run/'plot_input.json');verify_payload(data);source=read(run/'source.json')
    if environment is None and source.get('environment_path'):environment=HospitalEnvironment.load(source['environment_path'])
    folder=run/'plots';folder.mkdir()
    functions=[lambda:world(data,environment),lambda:motion(data,'v_y'),lambda:motion(data,'v_x'),lambda:motion(data,'a_x'),lambda:motion(data,'a_omega'),lambda:margins(data),lambda:objective(data),lambda:termination(data),lambda:singular(data),lambda:acceptance(data)]
    hashes={str(run/f):digest(run/f) for f in ('source.json','config_snapshot.yaml','protocol.json','plot_input.json')}
    for name,fn in zip(PLOTS,functions):
        fig=fn();fig.suptitle(name.replace('_',' ')+'\n'+LABEL,fontsize=13);fig.tight_layout(rect=(0,.075,1,.95))
        path=folder/(name+'.png');fig.savefig(path,dpi=150,facecolor='white');plt.close(fig)
        write(folder/(name+'.json'),dict(numeric_data=expected_numeric(name,data),source_hashes=hashes,image_sha256=digest(path),plotter_sha256=digest(__file__),label=LABEL))
    text=['<!doctype html><html><meta charset="utf-8"><title>GP-SE2-DIAG-05</title><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}</style>',
        '<h1>GP-SE2-DIAG-05 · quarter motion inequality-only ablation</h1><p>'+LABEL+'</p>',
        '<p>G0/G1: saved DIAG04 baselines, not rerun. G2: five new solves. One hard event and one benign control. Original acceptance unchanged. No execution or navigation evidence.</p>',
        '<pre>'+html.escape(json.dumps(data['summary'],indent=2))+'</pre>']
    for name in PLOTS:text.append(f'<h2>{name}</h2><p><a href="plots/{name}.json">Numeric records and hashes</a></p><img src="plots/{name}.png">')
    text.append('</html>');(run/'index.html').write_text('\n'.join(text))
    bundle=run/'review_bundle';bundle.mkdir()
    names=['index.html','source.json','protocol.json','config_snapshot.yaml','experiment_manifest.json','historical_baselines/diag04_g0_g1_manifest.json']
    names += [str(p.relative_to(run)) for parent in ('plots','aggregate') for p in (run/parent).glob('*') if p.is_file()]
    for name in names:
        path=run/name
        if path.exists():target=bundle/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target)
    (bundle/'README.txt').write_text('PLAN ONLY / NOT EXECUTED\nHistorical G0/G1 and new G2 have explicit provenance. No RGB, environment export, external source or full historical arrays are bundled.\nNumeric sidecars retain saved plotted values; finite sampling is not continuous-time proof. No physical acceptance was relaxed.\n')
    with zipfile.ZipFile(run/'review_bundle.zip','x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(bundle.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(bundle))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);generate(p.parse_args().run)
