#!/usr/bin/env python3
"""Three compact saved-only PNGs, with numerical sidecars and explicit nulls."""
import argparse
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from run_b_to_entry_boundary_row_ablation04 import ROOT,RESULTS,read,save,sha
from reconciliation.boundary_row_ablation04 import *
from reconciliation.relative_factor_multisource import geometry
from report_spatial_correspondence_selector_diag01 import polygons
from validate_b_to_entry_boundary_row_ablation04 import validate
METHODS=PRIMARY
COLORS=['#a04f9f','#276dc2','#cd7910','#007f73']
STYLES=['--','-',':','-.']
MARKERS=['s','x','^','o']
LABELS=['C3 (saved)','B entry (new)','Hermite (saved)','V2 (saved)']


def compact_figures(summary,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.titlesize':11,'figure.dpi':150})
    records=[];loaded={}
    handles=[Line2D([],[],color=c,ls=ls,marker=m,mfc='none',label=n) for c,ls,m,n in zip(COLORS,STYLES,MARKERS,LABELS)]
    def finish(fig,name,values):
        fig.savefig(out/name,dpi=160,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=values))
    for spec in summary['selected_sources']:
        sid=spec['id'];f=Path(spec['folder']);q=summary['sources'][sid]
        refs=read(f/'references.json');methods={}
        for n,p in q['method_folders'].items():
            p=Path(p);methods[n]=dict(rollout=read(p/'rollout.json') if (p/'rollout.json').exists() else None,
                metrics=read(p/'metrics.json') if (p/'metrics.json').exists() else None)
        loaded[sid]=dict(folder=f,refs=refs,methods=methods,past=np.load(f/'recorded_old_to_B.npy'),env=geometry(f))
    fig,axes=plt.subplots(2,2,figsize=(15,12));values={}
    for i,(ax,(sid,q)) in enumerate(zip(axes.flat,summary['sources'].items())):
        d=loaded[sid];fresh=np.load(d['refs'][NATIVE]['world_path']);B=np.asarray(q['boundary_B']);E=np.asarray(q['entry']['correspondence']['target_world'])
        clouds=[fresh[:,:2],d['past'][:,:2]];numbers={}
        for j,n in enumerate(METHODS):
            r=d['methods'][n]['rollout'];m=d['methods'][n]['metrics'];metric=q['primary_metrics'][n]
            if r is None:continue
            xy=np.array([s['pose_world'][:2] for s in r['states']]);clouds.append(xy)
            ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],lw=2,marker=MARKERS[j],markevery=(j*4,18),ms=4,mfc='none',zorder=6+j)
            numbers[n]=dict(execution_xy=xy.tolist(),attachment_xy=None,endpoint_dwell_xy=None)
            if m is None:continue
            for key,marker,field in [('sustained_attachment_s','*','attachment_xy'),('original_FRESH_endpoint_dwell_s','o','endpoint_dwell_xy')]:
                t=metric[key]
                if t is not None:
                    pt=xy[np.searchsorted(m['trace']['times_s'],t)]
                    ax.scatter(*pt,marker=marker,s=145,facecolors='none',edgecolors=COLORS[j],lw=1.5,zorder=15+j)
                    numbers[n][field]=pt.tolist()
        cloud=np.vstack(clouds);lo=cloud.min(0)-.18;hi=cloud.max(0)+.18;env=d['env']
        if env['cart'] is not None:
            bounds=np.asarray(env['cart'].bounds);lo=np.minimum(lo,bounds[:2]-.26);hi=np.maximum(hi,bounds[2:]+.26)
        center=(lo+hi)/2;half=max(hi-lo)/2;lo=center-half;hi=center+half
        from shapely.geometry import box
        clip=box(*lo,*hi)
        polygons(ax,env['base'].obstacles.buffer(.25).intersection(clip),color='#eddddd',alpha=.7,zorder=0)
        polygons(ax,env['base'].obstacles.intersection(clip),color='#bbc1c6',zorder=1)
        if env['cart'] is not None:
            polygons(ax,env['cart'].buffer(.25),color='#e7bcbc',alpha=.65,zorder=1);polygons(ax,env['cart'],color='#777d85',zorder=2)
        ax.plot(*d['past'][:,:2].T,color='#888',ls=':',lw=2.5,zorder=3)
        ax.plot(*fresh[:,:2].T,color='#333',ls='--',marker='.',ms=4,lw=1,zorder=4)
        ax.scatter(*B[:2],marker='P',color='black',s=75,zorder=22);ax.annotate('B',B[:2],xytext=(-14,-12),textcoords='offset points')
        ax.scatter(*E[:2],marker='D',edgecolors='#682e89',facecolors='none',s=110,lw=1.6,zorder=22)
        ax.scatter(*fresh[-1,:2],marker='X',color='black',s=75,zorder=22)
        ax.set(title=f'S{i+1} | {sid}',xlabel='World X [m]',ylabel='World Y [m]',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.16)
        values[sid]=dict(original=fresh.tolist(),OLD_to_B=d['past'].tolist(),B=B.tolist(),E=E.tolist(),methods=numbers)
    extra=[Line2D([],[],color='#333',ls='--',marker='.',label='Original FRESH'),Line2D([],[],color='#888',ls=':',label='OLD to B'),
        Line2D([],[],color='#682e89',ls='',marker='D',mfc='none',label='C3 entry E*'),Line2D([],[],color='black',ls='',marker='X',label='Original endpoint'),
        Line2D([],[],color='black',ls='',marker='*',mfc='none',label='Attachment dwell start'),Line2D([],[],color='black',ls='',marker='o',mfc='none',label='Endpoint dwell start')]
    fig.suptitle('Boundary staging | actual execution on four development sources',fontsize=17,y=.99)
    fig.text(.5,.958,'Offline controlled timing; original-FRESH evaluation. Distinct markers expose overlapping executions.',ha='center')
    fig.legend(handles=handles+extra,loc='lower center',ncol=3,bbox_to_anchor=(.5,.012),fontsize=9)
    fig.subplots_adjust(top=.91,bottom=.15,hspace=.28,wspace=.24);finish(fig,PNGS[0],values)

    fig,axes=plt.subplots(2,3,figsize=(18,10));values={};x=np.arange(4)
    panels=[('position_auc_09_m_s','Original-FRESH position AUC .9 s [m s]'),
        ('sustained_attachment_s','Sustained attachment [s after B]'),
        ('original_FRESH_endpoint_dwell_s','Original-FRESH endpoint dwell [s after B]'),
        ('attachment_fractional_arc','Original arc fraction at attachment'),
        ('remaining_arc_at_attachment_m','Remaining original arc at attachment [m]'),
        ('yaw_auc_09_rad_s','Original-FRESH yaw AUC .9 s [rad s]')]
    def bars(ax, values_by_method, names, ticks):
        for j,n in enumerate(names):
            v=values_by_method[n];offset=(j-(len(names)-1)/2)*.19
            ax.bar(np.arange(len(ticks))+offset,[np.nan if a is None else a for a in v],width=.18,color=COLORS[METHODS.index(n)])
            for k,a in enumerate(v):
                if a is None:ax.text(k+offset,.025,'N/A',rotation=90,ha='center',fontsize=9,transform=ax.get_xaxis_transform())
        ax.set(xticks=np.arange(len(ticks)),xticklabels=ticks);ax.grid(axis='y',alpha=.17)
    for ax,(key,title) in zip(axes.flat,panels):
        values[key]={n:[None if q['primary_metrics'][n] is None else q['primary_metrics'][n][key] for q in summary['sources'].values()] for n in METHODS}
        bars(ax,values[key],METHODS,['S1','S2','S3','S4']);ax.set_title(title)
    fig.suptitle('Transition, attachment location and endpoint dwell',fontsize=17)
    fig.legend(handles=handles,loc='lower center',ncol=4,bbox_to_anchor=(.5,.015))
    fig.text(.5,.065,'Full original FRESH is the primary target. N/A is never replaced by the cap.',ha='center')
    fig.tight_layout(rect=(0,.11,1,.94));finish(fig,PNGS[1],values)

    fig,axes=plt.subplots(3,2,figsize=(18,16));values={}
    ax=axes[0,0];ax.axis('off')
    rows=[['C3','E* -> F...'],['B entry','B -> E* -> F...'],['Hermite','B -> X1_H -> E* -> F...'],['V2','B -> X1_V2 -> E* -> F...']]
    table=ax.table(cellText=rows,colLabels=['Method','Reference structure'],colWidths=[.24,.76],cellLoc='left',bbox=[0,.36,1,.58]);table.auto_set_font_size(False);table.set_fontsize(12)
    ax.set_title('Only new condition: fixed boundary row staging')
    ax.text(.01,.1,'B has no original progress ID. E* keeps its frozen fractional ID.\nAll downstream original rows are unchanged. No optimizer.\nH5 identities below come from actual successful submits.',fontsize=11,transform=ax.transAxes)
    ax=axes[0,1];ax.axis('off');rows=[]
    for i,(sid,q) in enumerate(summary['sources'].items()):
        values[sid]=dict(selector=q['selector_exposure'],paired=summary['cross_source']['paired'][sid])
        for n,label in zip(METHODS,['C3','B entry','Hermite','V2']):
            e=q['selector_exposure'][n];f=None if e is None else e['first']
            rows.append([f'S{i+1} {label}','N/A' if f is None else f['nearest_identity'],'N/A' if f is None else ', '.join(f['H5_identities'])])
    table=ax.table(cellText=rows,colLabels=['Source/method','Nearest','Actual first H5'],colWidths=[.2,.14,.66],cellLoc='left',bbox=[0,0,1,.97]);table.auto_set_font_size(False);table.set_fontsize(8.7)
    ax.set_title('First legal submit: identical actual state within each source')
    for ax,key,title in [(axes[1,0],'entry_submit_count','Successful submits containing E* in H5'),(axes[1,1],'entry_sample_span_s','E* first-to-last exposure sample span [s]')]:
        vals={n:[None if q['selector_exposure'][n] is None else q['selector_exposure'][n][key] for q in summary['sources'].values()] for n in METHODS}
        bars(ax,vals,METHODS,['S1','S2','S3','S4']);ax.set_title(title)
    ax=axes[2,0];easy=[s for s in SOURCE_IDS if s!=SOURCE_IDS[2]]
    vals={n:[summary['cross_source']['paired'][s][k] for s in easy] for n,k in [(STAGE,'delay_B_vs_C3_s'),(HERMITE,'delay_Hermite_vs_C3_s'),(VECTOR,'delay_V2_vs_C3_s')]}
    bars(ax,vals,[STAGE,HERMITE,VECTOR],['S1','S2','S4']);ax.axhline(0,color='#777',lw=.7);ax.set_title('Easy-source endpoint dwell delay versus C3 [s]')
    ax=axes[2,1];hard=summary['sources'][SOURCE_IDS[2]]['primary_metrics']
    vals={n:[None if hard[n] is None else hard[n][k] for k in ['sustained_attachment_s','original_FRESH_endpoint_dwell_s']] for n in METHODS}
    bars(ax,vals,METHODS,['Attachment','Endpoint dwell']);ax.set_title('S3 recovery requires BOTH dwells [s after B]')
    fig.suptitle('Reference structure and natural official H5 selection',fontsize=18,y=.99)
    fig.legend(handles=handles,loc='lower center',ncol=4,bbox_to_anchor=(.5,.012))
    fig.text(.5,.055,'Exposure is observed at submit samples only, including successful results withheld beyond the cap; no continuous exposure or causal selector claim.',ha='center',fontsize=10)
    fig.subplots_adjust(top=.94,bottom=.11,wspace=.25,hspace=.34);finish(fig,PNGS[2],values)
    assert sorted(p.name for p in out.iterdir())==sorted(PNGS)
    return records


def report(run):
    s,v=validate(run,deep=False);assert v['valid'] and read(run/'validation.json')['all_historical_validators']
    figures=compact_figures(s,RESULTS/'figures')
    manifest=dict(summary_sha256=sha(run/'summary.json'),figures=figures,final_png_count=3,new_scientific_solves=0)
    save(run/'figure_manifest.json',manifest);save(RESULTS/'figure_manifest.json',manifest)
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=s,validation=read(run/'validation.json'),
        result_hashes_sha256=sha(run/'result_hashes.json'),table_hashes={p.name:sha(p) for p in RESULTS.glob('*.csv')},
        figures=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in figures]))
    print(json.dumps(dict(figures=[str(RESULTS/'figures'/p) for p in PNGS])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
