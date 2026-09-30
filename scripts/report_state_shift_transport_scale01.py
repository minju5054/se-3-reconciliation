#!/usr/bin/env python3
"""Exactly four saved-data PNGs; no new scientific computation."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from run_state_shift_transport_scale01 import RESULTS
from validate_state_shift_transport_scale01 import validate,method_folder
from reconciliation.state_shift_transport_scale import *
from reconciliation.join_source03 import read,save,sha
from reconciliation.relative_factor_multisource import geometry
from reconciliation.se2 import relative_pose
from report_relative_factor_multisource01 import polygons
COLORS=['#3268b2','#b2479c','#1d9873']
STYLES=['-','--','-.'];MARKERS=['o','s','D'];LABELS=['Native (alpha=0)','Half (alpha=0.5)','Full (alpha=1)']
METRICS=[('position_auc_09_m_s','Original-FRESH position AUC 0.9 s [m s]'),
         ('sustained_attachment_s','Sustained attachment [s]; N/A = no complete dwell'),
         ('execution_clearance_lower_bound_m','Swept footprint-edge clearance lower bound [m]'),
         ('endpoint_error_m','Original-FRESH endpoint error [m]')]


def compact_figures(s,out):
    out.mkdir(parents=True,exist_ok=False);records=[];selected=s['selected_sources']
    plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.spines.top':False,'axes.spines.right':False})
    handles=[Line2D([],[],color=c,ls=l,marker=m,label=t) for c,l,m,t in zip(COLORS,STYLES,MARKERS,LABELS)]
    def finish(fig,name,nums):
        fig.savefig(out/name,dpi=180,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=nums))
    def title(i,spec):return f'S{i+1}: '+spec['id'].replace('episode_','E').replace('_repeat_',' / R').replace('/handoff_',' / H')
    fig,axes=plt.subplots(2,2,figsize=(15,12));nums={}
    for i,(ax,spec) in enumerate(zip(axes.flat,selected)):
        folder=Path(spec['folder']);sid=spec['id'];q=s['sources'][sid];refs=read(folder/'references.json')
        original=np.load(refs[NATIVE]['world_path']);past=np.load(folder/'recorded_old_to_B.npy')
        B=np.array(read(folder/'common_state.json')['B']);env=geometry(folder)
        clouds=[original[:,:2],past[:,:2]];nums[sid]=dict(original=original.tolist(),past=past.tolist(),B=B.tolist(),methods={})
        for j,n in enumerate(ORDER):
            base=method_folder(folder,n)
            if not (base/'rollout.json').exists():nums[sid]['methods'][n]={'unavailable':q['termination'][n]};continue
            r=read(base/'rollout.json');m=read(base/'metrics.json');xy=np.array([v['pose_world'][:2] for v in r['states']]);clouds.append(xy)
            ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],lw=2.2,marker=MARKERS[j],ms=3.5,markevery=(j*4,18),alpha=.9,zorder=4+j*.1)
            ax.scatter(*xy[-1],c=COLORS[j],marker='^',s=70,zorder=9,edgecolors='white',linewidths=.5)
            entry=dict(execution_xy=xy.tolist(),attachment_xy=None,minimum_clearance_xy=None,termination=r['termination'])
            if m is not None:
                minxy=m['clearance']['minimum_pose'][:2];entry['minimum_clearance_xy']=minxy
                ax.scatter(*minxy,c=COLORS[j],marker='x',s=90,linewidths=2,zorder=10)
                jt=q['primary_metrics'][n]['sustained_attachment_s']
                if jt is not None:
                    k=int(np.searchsorted(m['trace']['times_s'],jt));entry['attachment_xy']=xy[k].tolist()
                    ax.scatter(*xy[k],c=COLORS[j],marker='*',s=155,zorder=11,edgecolors='white',linewidths=.5)
            nums[sid]['methods'][n]=entry
        xy=np.vstack(clouds);lo=xy.min(0)-.25;hi=xy.max(0)+.25
        if env['cart'] is not None:
            b=np.array(env['cart'].bounds);lo=np.minimum(lo,b[:2]-.3);hi=np.maximum(hi,b[2:]+.3)
        centre=(lo+hi)/2;half=max(hi-lo)/2;lo=centre-half;hi=centre+half
        from shapely.geometry import box
        polygons(ax,env['base'].obstacles.intersection(box(*lo,*hi)),color='#d4d9df',zorder=0)
        if env['cart'] is not None:
            polygons(ax,env['cart'].buffer(.25),color='#eac9c9',alpha=.5,zorder=1)
            polygons(ax,env['cart'],color='#737981',zorder=2)
        ax.plot(*past[:,:2].T,color='#777',ls=':',lw=3,zorder=3)
        ax.plot(*original[:,:2].T,color='black',ls='--',marker='.',ms=6,lw=1,zorder=3)
        ax.scatter(*B[:2],c='black',marker='P',s=110,zorder=12);ax.annotate('B',B[:2],xytext=(6,7),textcoords='offset points',weight='bold')
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title=title(i,spec)+'\n'+spec['role'])
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        d=q['transport']['body_log_translation_m'];pair=q['pairwise'][FULL] or {};gap=pair.get('max_execution_XY_gap_m')
        text=f'|Log(A^-1 B)| translation: {d:.3f} m\nHalf / Full max XY gap: '+('N/A' if gap is None else f'{gap*1000:.2f} mm')
        ax.text(.02,.02,text,transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
        nums[sid].update(transport=q['transport'],max_execution_gap_m=gap)
    extra=[Line2D([],[],color='black',ls='--',marker='.',label='Original FRESH rows'),Line2D([],[],color='#777',ls=':',label='Recorded OLD to B'),Line2D([],[],color='black',ls='',marker='P',label='B'),Line2D([],[],color='black',ls='',marker='*',label='Sustained attachment'),Line2D([],[],color='black',ls='',marker='x',label='Min clearance'),Line2D([],[],color='black',ls='',marker='^',label='Endpoint'),Patch(facecolor='#eac9c9',label='Cart + 0.25 m centre exclusion')]
    fig.legend(handles=handles+extra,loc='lower center',ncol=4,fontsize=10,bbox_to_anchor=(.5,.005))
    fig.suptitle('Transport magnitude | executed trajectories',fontsize=18,y=.99)
    fig.text(.5,.95,'Offline timing-controlled comparison. Native / Full reused; only Half is newly executed.',ha='center')
    fig.subplots_adjust(top=.89,bottom=.14,hspace=.35,wspace=.25);finish(fig,PNGS[0],nums)
    fig,axes=plt.subplots(2,2,figsize=(15,9));nums={};x=np.arange(4)
    for ax,(key,label) in zip(axes.flat,METRICS):
        nums[key]={}
        for j,n in enumerate(ORDER):
            vals=[None if q['primary_metrics'][n] is None else q['primary_metrics'][n][key] for q in s['sources'].values()];nums[key][n]=vals
            ax.bar(x+(j-1)*.24,[np.nan if v is None else v for v in vals],width=.23,color=COLORS[j])
            for k,v in enumerate(vals):
                if v is None:ax.text(k+(j-1)*.24,.01,'N/A',rotation=90,ha='center',fontsize=8,transform=ax.get_xaxis_transform())
        if key.startswith('execution_clearance'):ax.axhline(.05,color='#b22',ls='--',lw=1)
        ax.set(xlim=(-.5,3.5),xticks=x,xticklabels=['S1','S2','S3','S4'],title=label);ax.grid(axis='y',alpha=.2)
    fig.legend(handles=handles,loc='upper center',ncol=3,bbox_to_anchor=(.5,.955));fig.suptitle('Primary metrics against ORIGINAL FRESH',fontsize=17)
    fig.text(.5,.012,'S1: OSA03 R00 | S2: E001 / R01 / H013 | S3: E008 / R01 / H023 | S4: E013 / R00 / H020',ha='center')
    fig.tight_layout(rect=(0,.045,1,.91));finish(fig,PNGS[1],nums)
    fig,axes=plt.subplots(2,2,figsize=(14,9));nums={}
    for ax,(key,label) in zip(axes.flat,METRICS):
        vals=[s['cross_source']['deltas'][z['id']][key] for z in selected];nums[key]=vals
        nonnull=[abs(v) for v in vals if v is not None];span=max(nonnull+[1e-8]);ax.set_xlim(-1.4*span,1.4*span)
        for i,v in enumerate(vals):
            if v is None:ax.text(.03*span,i,'N/A (one or both attachment times missing)',va='center',fontsize=8)
            else:
                ax.barh(i,v,color='#b2479c' if v>=0 else '#3268b2')
                ax.annotate(f'{v:+.6g}',(v,i),xytext=(4 if v>=0 else -4,0),textcoords='offset points',ha='left' if v>=0 else 'right',va='center',fontsize=9)
        ax.axvline(0,color='black',lw=1);ax.set(ylim=(-.5,3.5),yticks=x,yticklabels=['S1','S2','S3','S4'],title=label.replace('Original-FRESH ','').split(';')[0]);ax.grid(axis='x',alpha=.2)
    fig.suptitle('Half minus Full | signed differences, no combined score',fontsize=17)
    fig.tight_layout(rect=(0,0,1,.94));finish(fig,PNGS[2],nums)
    fig,axes=plt.subplots(2,2,figsize=(15,12));nums={}
    for i,(ax,spec) in enumerate(zip(axes.flat,selected)):
        folder=Path(spec['folder']);c=read(folder/'common_state.json');refs=read(folder/'references.json');q=s['sources'][spec['id']];nums[spec['id']]={}
        for j,n in enumerate(ORDER):
            if 'world_path' not in refs[n]:continue
            local=relative_pose(c['fresh_capture_pose'],np.load(refs[n]['world_path']))
            nums[spec['id']][n]=dict(observation_local_reference=local.tolist(),planning=refs[n]['descriptor'])
            ax.plot(*local[:,:2].T,color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],ms=4,lw=1.8)
        b=relative_pose(c['fresh_capture_pose'],c['B']);ax.scatter(*b[:2],c='black',marker='P',s=100);ax.annotate('B',b[:2],xytext=(6,6),textcoords='offset points')
        ax.set(title=title(i,spec)+'\nOriginal observation frame A',xlabel='Observation-local forward X [m]',ylabel='Observation-local left Y [m]')
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        lo,hi=ax.get_ylim();ax.set_ylim(lo,hi+max(.18,.3*(hi-lo)))
        notes=[]
        for n,l in [(HALF,'Half'),(FULL,'Full')]:
            p=q['planning'].get(n)
            if p is not None:notes.append(f"{l}: first correction {p['first_node_displacement_m']:.3f} m; edge RMS {p['relative_translation_RMS_m']:.4f} m")
        ax.text(.02,.98,'\n'.join(notes),transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
    fig.legend(handles=handles,loc='lower center',ncol=3);fig.suptitle('Reference transport | original, half and full',fontsize=18)
    fig.subplots_adjust(top=.90,bottom=.08,hspace=.50,wspace=.28);finish(fig,PNGS[3],nums)
    assert sorted(p.name for p in out.iterdir())==sorted(PNGS)
    return records


def report(run):
    s,v=validate(run);assert v['valid'] and s==read(run/'summary.json')
    records=compact_figures(s,RESULTS/'figures')
    manifest=dict(summary_sha256=sha(run/'summary.json'),figures=records,final_png_count=4,extra_pngs=0,new_scientific_calls=0)
    save(run/'figure_manifest.json',manifest);save(RESULTS/'figure_manifest.json',manifest)
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=s,validation=v,result_hashes_sha256=sha(run/'result_hashes.json'),
        figures=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in records]))
    print(json.dumps(dict(classification=s['classification'],figures=[r['file'] for r in records])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
