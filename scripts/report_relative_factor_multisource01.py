#!/usr/bin/env python3
"""Compact saved-only PNG report. Exactly four required figures, optional selector."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from run_relative_factor_multisource01 import RESULTS
from reconciliation.join_source03 import read,save,sha
from reconciliation.relative_factor_multisource import REQUIRED_PNG,SELECTOR_PNG,geometry
from reconciliation.osa03_relative_ablation import ORDER
from reconciliation.se2 import relative_pose
from validate_relative_factor_multisource01 import validate
from validate_robotless_online_handoffs import equal_record
COLORS=['#3268b2','#dc8624','#1d9873','#b2479c']
STYLES=['-','--','-.',':']
LABELS=['Native','Taper','Full','No-relative']
MARKERS=['o','s','D','v']


def polygons(ax,g,**kw):
    if g.is_empty:return
    if g.geom_type=='Polygon':
        x,y=g.exterior.xy;ax.fill(x,y,**kw)
    elif hasattr(g,'geoms'):
        for p in g.geoms:polygons(ax,p,**kw)


def expected_figures(s):
    extra=any(q['selector']['differing_submit_ticks'] for q in s['sources'].values())
    return REQUIRED_PNG+([SELECTOR_PNG] if extra else [])


def compact_figures(run,s,out):
    out.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.spines.top':False,'axes.spines.right':False})
    selected=s['selected_sources'];n=len(selected);cols=2;rows=int(np.ceil(n/cols));records=[]
    def finish(fig,name,numbers):
        fig.savefig(out/name,dpi=180,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=numbers))
    def axes_grid():
        f,a=plt.subplots(rows,cols,figsize=(15,6*rows),squeeze=False)
        for extra in a.flat[n:]:extra.set_visible(False)
        return f,list(a.flat)
    def title(spec):
        return spec['id'].replace('episode_','E').replace('_repeat_',' / repeat ').replace('/handoff_',' / H')
    f,axes=axes_grid();world_data={}
    for ax,spec in zip(axes,selected):
        sid=spec['id'];folder=Path(spec['folder']);q=s['sources'][sid];refs=read(folder/'references.json')
        original=np.load(refs['M0_NATIVE']['world_path']);past=np.load(folder/'recorded_old_to_B.npy')
        common=read(folder/'common_state.json');B=np.array(common['B']);env=geometry(folder)
        clouds=[original[:,:2],past[:,:2]];world_data[sid]={}
        for j,name in enumerate(ORDER):
            p=folder/'methods'/name/'rollout.json'
            if not p.exists():world_data[sid][name]={'unavailable':q['termination'][name]};continue
            r=read(p);m=read(p.parent/'metrics.json');xy=np.array([z['pose_world'][:2] for z in r['states']]);clouds.append(xy)
            ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],lw=2.2,marker=MARKERS[j],ms=3.5,markevery=(j*3,18),alpha=.85,zorder=4+j*.1)
            ax.scatter(*xy[-1],c=COLORS[j],marker='^',s=68,zorder=7,edgecolors='white',linewidths=.5)
            entry=dict(execution_xy=xy.tolist(),termination=r['termination'],attachment_xy=None,minimum_clearance_xy=None)
            if m is not None:
                minxy=m['clearance']['minimum_pose'][:2];ax.scatter(*minxy,c=COLORS[j],marker='x',s=95,linewidths=2,zorder=9)
                jt=q['primary_metrics'][name]['sustained_attachment_s']
                if jt is not None:
                    k=int(np.searchsorted(m['trace']['times_s'],jt));ax.scatter(*xy[k],c=COLORS[j],marker='*',s=150,zorder=10,edgecolors='white',linewidths=.5);entry['attachment_xy']=xy[k].tolist()
                entry['minimum_clearance_xy']=minxy
            world_data[sid][name]=entry
        xy=np.vstack(clouds);lo=xy.min(0)-.25;hi=xy.max(0)+.25
        from shapely.geometry import box
        polygons(ax,env['base'].obstacles.intersection(box(*lo,*hi)),color='#d4d9df',zorder=0)
        if env['cart'] is not None:
            polygons(ax,env['cart'].buffer(.25),color='#eac9c9',alpha=.45,zorder=1)
            polygons(ax,env['cart'],color='#737981',zorder=2)
        ax.plot(*past[:,:2].T,color='#777777',lw=3,ls=(0,(2,2)),zorder=3)
        ax.plot(*original[:,:2].T,color='black',marker='.',ls='--',lw=1,ms=6,zorder=3)
        ax.scatter(*B[:2],marker='P',c='black',s=120,zorder=12)
        ax.annotate('B',B[:2],xytext=(7,7),textcoords='offset points',weight='bold')
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title=title(spec)+'\n'+spec['role']);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        pair=q['pairwise'].get('NO_RELATIVE_minus_FULL_LOCAL_SE2') or {}
        mx=pair.get('max_execution_XY_separation_m')
        ax.text(.02,.02,('Full / NoR max XY gap: '+f'{mx*1000:.2f} mm') if mx is not None else 'Full / NoR comparison unavailable',transform=ax.transAxes,fontsize=9,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
    handles=[Line2D([],[],color=c,ls=ls,marker=mk,label=l) for c,ls,mk,l in zip(COLORS,STYLES,MARKERS,LABELS)]
    handles += [Line2D([],[],color='black',ls='--',marker='.',label='Original FRESH rows'),Line2D([],[],color='#777',ls=':',label='Recorded OLD to B'),Line2D([],[],color='black',ls='',marker='P',label='B'),Line2D([],[],color='black',ls='',marker='*',label='Sustained attachment'),Line2D([],[],color='black',ls='',marker='x',label='Min clearance'),Line2D([],[],color='black',ls='',marker='^',label='Endpoint'),Patch(facecolor='#eac9c9',label='Cart + 0.25 m centre exclusion')]
    f.legend(handles=handles,loc='lower center',ncol=4,fontsize=10,bbox_to_anchor=(.5,.01))
    f.suptitle('Executed trajectories | four distinct saved FRESH paths',fontsize=18,y=.99)
    f.text(.5,.952,'Offline timing-controlled comparison. Overlapping curves use different styles and markers.',ha='center')
    f.subplots_adjust(top=.89,bottom=.14,hspace=.35,wspace=.25)
    finish(f,REQUIRED_PNG[0],world_data)
    # Primary metrics: full numeric values and nulls remain in sidecars/CSV.
    metrics=[('position_auc_09_m_s','Original-FRESH position AUC, first 0.9 s [m s]'),('sustained_attachment_s','Sustained attachment [s]; NA = no complete dwell'),('execution_clearance_lower_bound_m','Swept footprint-edge clearance lower bound [m]'),('endpoint_error_m','Original-FRESH endpoint error [m]')]
    f,a=plt.subplots(2,2,figsize=(15,9));x=np.arange(n);nums={}
    short=[f'S{i+1}' for i in range(n)]
    for ax,(key,label) in zip(a.flat,metrics):
        nums[key]={}
        for j,name in enumerate(ORDER):
            vals=[None if s['sources'][z['id']]['primary_metrics'][name] is None else s['sources'][z['id']]['primary_metrics'][name][key] for z in selected];nums[key][name]=vals
            v=np.array([np.nan if v is None else v for v in vals]);ax.bar(x+(j-1.5)*.19,v,width=.18,color=COLORS[j],label=LABELS[j])
            for k,val in enumerate(vals):
                if val is None:ax.text(x[k]+(j-1.5)*.19,0,'NA',rotation=90,fontsize=8,ha='center')
        if key.startswith('execution_clearance'):ax.axhline(.05,color='#b22',ls='--',lw=1)
        ax.set(title=label,xticks=x,xticklabels=short);ax.grid(axis='y',alpha=.2)
    f.legend(handles=handles[:4],loc='upper center',ncol=4,bbox_to_anchor=(.5,.955));f.suptitle('Primary metrics against ORIGINAL FRESH',fontsize=17)
    f.text(.5,.01,' | '.join(f'S{i+1}: {title(z)}' for i,z in enumerate(selected)),ha='center',fontsize=9)
    f.tight_layout(rect=(0,.04,1,.91));finish(f,REQUIRED_PNG[1],nums)
    metrics=[('sustained_attachment_s','Attachment [s]'),('position_auc_09_m_s','Position AUC 0.9 s [m s]'),('execution_clearance_lower_bound_m','Clearance [m]'),('endpoint_error_m','Endpoint error [m]'),('relative_translation_RMS_m','Relative-edge translation RMS [m]'),('linear_command_TV','Linear command TV [m/s]')]
    f,a=plt.subplots(2,3,figsize=(16,8));nums={}
    for ax,(key,label) in zip(a.flat,metrics):
        vals=[s['cross_source']['deltas'][z['id']][key] for z in selected];nums[key]=vals
        for i,v in enumerate(vals):
            if v is None:ax.text(0,i,'NA',ha='left')
            else:ax.barh(i,v,color='#b2479c' if v>=0 else '#367ab7');ax.annotate(f'{v:+.5g}',(v,i),xytext=(4 if v>=0 else -4,0),textcoords='offset points',ha='left' if v>=0 else 'right',va='center',fontsize=9)
        ax.axvline(0,color='black',lw=1);ax.set(yticks=x,yticklabels=short,title=label);ax.margins(x=.35);ax.grid(axis='x',alpha=.2)
    f.suptitle('No-relative minus Full | signed differences; no combined score',fontsize=17)
    f.tight_layout(rect=(0,0,1,.94));finish(f,REQUIRED_PNG[2],nums)
    f,axes=axes_grid();nums={}
    for ax,spec in zip(axes,selected):
        folder=Path(spec['folder']);c=read(folder/'common_state.json');refs=read(folder/'references.json');nums[spec['id']]={}
        for j,name in enumerate(ORDER):
            if 'world_path' not in refs[name]:continue
            raw=relative_pose(c['fresh_capture_pose'],np.load(refs[name]['world_path']));nums[spec['id']][name]=dict(observation_local_reference=raw.tolist(),planning=refs[name]['descriptor'])
            ax.plot(*raw[:,:2].T,color='black' if j==0 else COLORS[j],ls=STYLES[j],marker=MARKERS[j],ms=4,lw=1.7,label='Original FRESH' if j==0 else LABELS[j])
        bp=relative_pose(c['fresh_capture_pose'],c['B']);ax.scatter(*bp[:2],marker='P',s=90,c='black');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        ax.set(title=title(spec)+'\nOriginal observation frame A',xlabel='Observation-local forward X [m]',ylabel='Observation-local left Y [m]')
        q=s['sources'][spec['id']];p=q['planning'];text=[]
        for name,label in [('M1_TAPER','Taper'),('FULL_LOCAL_SE2','Full'),('NO_RELATIVE','NoR')]:
            if p[name] is not None:text.append(f"{label} edge RMS {p[name]['relative_translation_RMS_m']:.5f} m; arc {p[name]['total_XY_arc_m']:.3f} m")
        ax.text(.02,.98,'\n'.join(text),transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    axes[0].legend(loc='lower right');f.suptitle('Reference deformation | raw arrays remain anchored at A',fontsize=18);f.tight_layout(rect=(0,0,1,.95));finish(f,REQUIRED_PNG[3],nums)
    if SELECTOR_PNG in expected_figures(s):
        f,axes=axes_grid();nums={}
        for ax,spec in zip(axes,selected):
            q=s['sources'][spec['id']]['selector'];c=read(Path(spec['folder'])/'common_state.json');nums[spec['id']]=q
            for j,name in [(2,'FULL_LOCAL_SE2'),(3,'NO_RELATIVE')]:
                z=q['methods'][name];times=[(r['submit_tick']-c['B_tick'])*c['integration_dt_s'] for r in z]
                ax.step(times,[r['nearest_row'] for r in z],where='post',color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],ms=4,label=LABELS[j])
            for tick in q['differing_submit_ticks']:ax.axvline((tick-c['B_tick'])*c['integration_dt_s'],color='#999',lw=.7,alpha=.5)
            ax.set(title=title(spec)+f"\n{len(q['differing_submit_ticks'])} differing H5 selections",xlabel='Logical time after B [s]',ylabel='Nearest original row index');ax.legend();ax.grid(alpha=.2)
        f.suptitle('Observed selector differences | unchanged official nearest + 1 selector',fontsize=16);f.tight_layout(rect=(0,0,1,.94));finish(f,SELECTOR_PNG,nums)
    assert sorted(p.name for p in out.iterdir())==sorted(expected_figures(s))
    return records


def report(run):
    s,v=validate(run);assert v['valid'];equal_record(s,read(run/'summary.json'))
    records=compact_figures(run,s,RESULTS/'figures')
    save(run/'figure_manifest.json',dict(summary_sha256=sha(run/'summary.json'),figures=records,
        omitted_selector_reason=None if SELECTOR_PNG in expected_figures(s) else 'No differing H5 selection observed; conditional PNG omitted',extra_pngs=0))
    save(RESULTS/'figure_manifest.json',read(run/'figure_manifest.json'))
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=s,validation=v,
        result_hashes_sha256=sha(run/'result_hashes.json'),figures=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in records]))
    print(json.dumps(dict(classification=s['classification'],figures=[r['file'] for r in records])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
