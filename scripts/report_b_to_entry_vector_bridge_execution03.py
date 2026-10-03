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
from run_b_to_entry_vector_bridge_execution03 import ROOT,RESULTS,read,save,sha
from reconciliation.vector_bridge_execution03 import *
from reconciliation.relative_factor_multisource import geometry
from report_spatial_correspondence_selector_diag01 import polygons
from validate_b_to_entry_vector_bridge_execution03 import validate
METHODS=[ENTRY,HERMITE,VECTOR]
COLORS=['#a04f9f','#cd7910','#007f73']
STYLES=['--','-.','-']
MARKERS=['s','^','o']
LABELS=['C3 (saved)','Hermite (saved)','V2 graph (new)']


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
        d=loaded[sid];fresh=np.load(d['refs'][NATIVE]['world_path']);B=np.asarray(q['bridge_input']['B']);E=np.asarray(q['bridge_input']['E'])
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
    fig.suptitle('Saved V2 bridge | actual execution on four development sources',fontsize=17,y=.99)
    fig.text(.5,.958,'Offline controlled timing; original-FRESH evaluation. Distinct markers expose overlapping executions.',ha='center')
    fig.legend(handles=handles+extra,loc='lower center',ncol=3,bbox_to_anchor=(.5,.012),fontsize=9)
    fig.subplots_adjust(top=.91,bottom=.15,hspace=.28,wspace=.24);finish(fig,PNGS[0],values)

    fig,axes=plt.subplots(2,2,figsize=(14,9));values={};x=np.arange(4)
    for ax,(key,title) in zip(axes.flat,[('position_auc_09_m_s','Original-FRESH position AUC .9 s [m s]'),
        ('sustained_attachment_s','Sustained attachment [s after B]'),('original_FRESH_endpoint_dwell_s','Original-FRESH endpoint dwell [s after B]'),
        ('remaining_arc_at_attachment_m','Remaining original arc at attachment [m]')]):
        values[key]={}
        for j,n in enumerate(METHODS):
            v=[None if q['primary_metrics'][n] is None else q['primary_metrics'][n][key] for q in summary['sources'].values()];values[key][n]=v
            ax.bar(x+(j-1)*.24,[np.nan if a is None else a for a in v],width=.22,color=COLORS[j])
            for k,a in enumerate(v):
                if a is None:ax.text(k+(j-1)*.24,.025,'N/A',rotation=90,ha='center',fontsize=9,transform=ax.get_xaxis_transform())
        ax.set(xticks=x,xticklabels=['S1','S2','S3','S4'],title=title);ax.grid(axis='y',alpha=.17)
    fig.suptitle('Transition and endpoint dwell are distinct execution outcomes',fontsize=17)
    fig.legend(handles=handles,loc='lower center',ncol=3,bbox_to_anchor=(.5,.02))
    fig.text(.5,.072,'N/A stays unobserved/censored; no time is replaced by the cap.',ha='center')
    fig.tight_layout(rect=(0,.12,1,.94));finish(fig,PNGS[1],values)

    fig,axes=plt.subplots(2,4,figsize=(19,10));values={}
    for i,(sid,q) in enumerate(summary['sources'].items()):
        ax=axes[0,i];g=q['bridge_geometry'];values[sid]=dict(geometry=g,selector=q['selector_exposure'],paired=summary['cross_source']['paired'][sid])
        for j,n in enumerate([HERMITE,VECTOR],1):
            b=np.asarray(g[n]['nodes']);ax.plot(*b[:,:2].T,color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],mfc='none',ms=7,lw=1.7)
        b=np.asarray(g[VECTOR]['nodes']);ax.scatter(*b[0,:2],marker='P',color='black',zorder=8);ax.scatter(*b[-1,:2],marker='D',color='#682e89',zorder=8)
        ax.set(title=f'S{i+1} | max node delta {1000*g["difference"]["max_bridge_node_XY_m"]:.3f} mm',xlabel='World X [m]',ylabel='World Y [m]')
        cloud=np.vstack([np.asarray(g[n]['nodes'])[:,:2] for n in [HERMITE,VECTOR]])
        lo,hi=cloud.min(0),cloud.max(0);center=(lo+hi)/2;half=max(hi-lo)*.65
        ax.set(xlim=(center[0]-half,center[0]+half),ylim=(center[1]-half,center[1]+half))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.17);ax.ticklabel_format(useOffset=False);ax.locator_params(nbins=3)
    ax=axes[1,0];easy=[s for s in SOURCE_IDS if s!=SOURCE_IDS[2]]
    for j,(key,n) in enumerate([('delay_Hermite_vs_C3_s',HERMITE),('delay_V2_vs_C3_s',VECTOR)]):
        v=[summary['cross_source']['paired'][s][key] for s in easy]
        ax.bar(np.arange(3)+(j-.5)*.32,[np.nan if t is None else t for t in v],width=.3,color=COLORS[j+1])
        for k,t in enumerate(v):
            if t is None:ax.text(k+(j-.5)*.32,.03,'N/A',rotation=90,transform=ax.get_xaxis_transform())
    ax.axhline(0,color='#777',lw=.7);ax.set(title='Easy-source endpoint delay vs C3',ylabel='Delay [s]',xticks=np.arange(3),xticklabels=['S1','S2','S4']);ax.grid(axis='y',alpha=.15)
    ax=axes[1,1];hard=summary['sources'][SOURCE_IDS[2]]['primary_metrics']
    for j,n in enumerate(METHODS):
        v=[None if hard[n] is None else hard[n][k] for k in ['sustained_attachment_s','original_FRESH_endpoint_dwell_s']]
        ax.bar(np.arange(2)+(j-1)*.24,[np.nan if t is None else t for t in v],width=.22,color=COLORS[j])
        for k,t in enumerate(v):
            if t is None:ax.text(k+(j-1)*.24,.03,'N/A',rotation=90,transform=ax.get_xaxis_transform(),ha='center')
    ax.set(title='S3 observed dwells',ylabel='Time after B [s]',xticks=[0,1],xticklabels=['Attachment','Endpoint']);ax.grid(axis='y',alpha=.15)
    ax=axes[1,2]
    for j,n in enumerate(METHODS):
        v=[None if q['selector_exposure'][n] is None else q['selector_exposure'][n]['any_bridge_submit_count'] for q in summary['sources'].values()]
        ax.bar(x+(j-1)*.24,[np.nan if a is None else a for a in v],width=.22,color=COLORS[j])
    ax.set(title='H5 exposure to B / bridge_1 / E*',ylabel='Submit events containing derived rows',xticks=x,xticklabels=['S1','S2','S3','S4']);ax.grid(axis='y',alpha=.15)
    ax=axes[1,3];twin=ax.twinx();pairs=list(summary['cross_source']['paired'].values())
    for j,(target,key,color) in enumerate([(ax,'V2_minus_Hermite_linear_TV','#345e9d'),(twin,'V2_minus_Hermite_angular_TV','#a74242')]):
        target.bar(x+(j-.5)*.32,[np.nan if r[key] is None else r[key] for r in pairs],width=.3,color=color,alpha=.8)
    ax.set(title='Command TV: V2 minus Hermite',ylabel='Linear TV difference [m/s]',xticks=x,xticklabels=['S1','S2','S3','S4']);twin.set_ylabel('Angular TV difference [rad/s]')
    ax.axhline(0,color='#555',lw=.7);ax.yaxis.label.set_color('#345e9d');twin.yaxis.label.set_color('#a74242')
    fig.suptitle('Bridge geometry, completion trade-off and natural selector exposure',fontsize=18,y=.99)
    fig.legend(handles=handles,loc='lower center',ncol=3,bbox_to_anchor=(.5,.008))
    fig.text(.5,.068,'B and E* are fixed. Exposure counts include the final submitted solve if its result is withheld beyond the cap. Diagnostics are descriptive.',ha='center',fontsize=10)
    fig.subplots_adjust(top=.92,bottom=.16,wspace=.43,hspace=.43);finish(fig,PNGS[2],values)
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
