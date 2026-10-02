#!/usr/bin/env python3
"""Four compact PNGs: execution, transition, original progress, endpoint dwell."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from run_spatial_entry_suffix_execution01 import RESULTS, read, save, sha
from reconciliation.spatial_entry_suffix import ORDER, ENTRY, NATIVE, FULL, PNGS
from reconciliation.relative_factor_multisource import geometry
from validate_spatial_entry_suffix_execution01 import validate, method_folder
from report_spatial_correspondence_selector_diag01 import polygons

COLORS=['#2768b0','#b67524','#168977']
STYLES=['-','--','-.']
MARKERS=['o','s','D']
LABELS=['Native (saved)','Full Local-SE2 (saved)','C3 Entry-Suffix (new)']


def compact_figures(summary,out):
    out.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.spines.top':False,'axes.spines.right':False})
    records=[]
    handles=[Line2D([],[],color=c,ls=l,marker=m,label=n,mfc='none') for c,l,m,n in zip(COLORS,STYLES,MARKERS,LABELS)]
    def title(i,sid):
        return f'S{i+1} | '+sid.replace('episode_','E').replace('_repeat_',' / R').replace('/handoff_',' / H')
    def finish(fig,name,values):
        fig.savefig(out/name,dpi=180,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=values))
    fig,axes=plt.subplots(2,2,figsize=(15,12));values={}
    for i,(ax,spec) in enumerate(zip(axes.flat,summary['selected_sources'])):
        sid=spec['id'];folder=Path(spec['folder']);q=summary['sources'][sid]
        refs=read(folder/'references.json');fresh=np.load(refs[NATIVE]['world_path'])
        past=np.load(folder/'recorded_old_to_B.npy');common=read(folder/'common_state.json')
        B=np.asarray(common['B']);entry=np.asarray(q['entry']['correspondence']['target_world']);env=geometry(folder)
        clouds=[fresh[:,:2],past[:,:2],B[None,:2]];numbers={}
        for j,name in enumerate(ORDER):
            where=method_folder(folder,name)
            if not (where/'rollout.json').exists():
                numbers[name]={'unavailable':q['termination'][name]};continue
            r=read(where/'rollout.json');m=read(where/'metrics.json');row=q['primary_metrics'][name]
            xy=np.asarray([s['pose_world'][:2] for s in r['states']]);clouds.append(xy)
            ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],mfc='none',
                    ms=[5,4,3][j],markevery=(j*4,17),lw=[2.8,2.1,1.6][j],zorder=5+j*.1)
            numbers[name]=dict(execution_xy=xy.tolist(),attachment_xy=None,endpoint_dwell_xy=None,minimum_clearance_xy=None)
            if m is None:continue
            for key,marker,size,field in [('sustained_attachment_s','*',160,'attachment_xy'),
                                         ('original_FRESH_endpoint_dwell_s','o',190,'endpoint_dwell_xy')]:
                time=row[key]
                if time is not None:
                    k=int(np.searchsorted(m['trace']['times_s'],time));point=xy[k]
                    ax.scatter(*point,marker=marker,s=size,edgecolors=COLORS[j],facecolors='none',lw=1.7,zorder=10+j)
                    numbers[name][field]=point.tolist()
            point=m['clearance']['minimum_pose'][:2]
            ax.scatter(*point,color=COLORS[j],marker='x',s=70,lw=1.8,zorder=14)
            numbers[name]['minimum_clearance_xy']=point
        xy=np.vstack(clouds);lo=xy.min(0)-.20;hi=xy.max(0)+.20
        if env['cart'] is not None:
            bounds=np.asarray(env['cart'].bounds);lo=np.minimum(lo,bounds[:2]-.28);hi=np.maximum(hi,bounds[2:]+.28)
        centre=(lo+hi)/2;half=max(hi-lo)/2;lo=centre-half;hi=centre+half
        from shapely.geometry import box
        clip=box(*lo,*hi)
        polygons(ax,env['base'].obstacles.buffer(.25).intersection(clip),color='#e8dddd',alpha=.65,zorder=0)
        polygons(ax,env['base'].obstacles.intersection(clip),color='#bbc0c7',zorder=1)
        if env['cart'] is not None:
            polygons(ax,env['cart'].buffer(.25),color='#e8bfbf',alpha=.65,zorder=1)
            polygons(ax,env['cart'],color='#737982',zorder=2)
        ax.plot(*past[:,:2].T,color='#7d8590',ls=':',lw=2.5,zorder=3)
        ax.plot(*fresh[:,:2].T,color='#333',ls=(0,(2,3)),marker='.',ms=4,lw=1,zorder=4)
        ax.scatter(*entry[:2],edgecolors='#8b4ac2',facecolors='none',marker='D',s=160,lw=2,zorder=15)
        ax.scatter(*B[:2],color='black',marker='P',s=80,zorder=16)
        ax.annotate('B',B[:2],xytext=(-13,-14),textcoords='offset points')
        ax.scatter(*fresh[-1,:2],color='black',marker='X',s=80,zorder=16)
        gaps=q['max_execution_XY_gap_m']
        text='C3 max XY gap vs Native / Full: '+('N/A' if not gaps else f"{1000*gaps[NATIVE]:.2f} / {1000*gaps[FULL]:.2f} mm")
        ax.set(title=title(i,sid)+'\n'+text,xlabel='World X [m]',ylabel='World Y [m]',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.18)
        values[sid]=dict(original=fresh.tolist(),past=past.tolist(),B=B.tolist(),entry=entry.tolist(),methods=numbers,max_gaps_m=gaps)
    extra=[Line2D([],[],color='#333',ls=':',marker='.',label='Full original FRESH'),
           Line2D([],[],color='#7d8590',ls=':',label='Recorded OLD to B'),
           Line2D([],[],color='#8b4ac2',ls='',marker='D',mfc='none',label='C3 continuous entry'),
           Line2D([],[],color='black',ls='',marker='X',label='Original endpoint'),
           Line2D([],[],color='black',ls='',marker='*',mfc='none',label='Attachment dwell start'),
           Line2D([],[],color='black',ls='',marker='o',mfc='none',label='Endpoint dwell start'),
           Line2D([],[],color='black',ls='',marker='x',label='Minimum clearance'),
           Patch(facecolor='#e8dddd',label='Geometry + 0.25 m centre exclusion')]
    fig.legend(handles=handles+extra,loc='lower center',ncol=3,fontsize=9,bbox_to_anchor=(.5,.005))
    fig.suptitle('C3 entry suffix | actual execution from the same B',fontsize=19,y=.995)
    fig.text(.5,.96,'Timing-controlled offline comparison. Original downstream rows stay unchanged. No B-to-entry bridge.',ha='center')
    fig.subplots_adjust(top=.89,bottom=.17,hspace=.38,wspace=.26);finish(fig,PNGS[0],values)

    fig,axes=plt.subplots(2,2,figsize=(15,9));values={};x=np.arange(4)
    metrics=[('position_auc_09_m_s','Original-FRESH position AUC, first .9 s [m s]'),
             ('sustained_attachment_s','Attachment dwell start [s]'),
             ('original_FRESH_endpoint_dwell_s','Original-FRESH endpoint dwell time [s]'),
             ('remaining_arc_at_attachment_m','Remaining original arc at attachment [m]')]
    for ax,(key,label) in zip(axes.flat,metrics):
        values[key]={}
        for j,name in enumerate(ORDER):
            v=[None if q['primary_metrics'][name] is None else q['primary_metrics'][name][key] for q in summary['sources'].values()]
            values[key][name]=v;ax.bar(x+(j-1)*.24,[np.nan if a is None else a for a in v],width=.23,color=COLORS[j])
            for k,a in enumerate(v):
                if a is None:ax.text(k+(j-1)*.24,.025,'N/A',ha='center',rotation=90,fontsize=9,transform=ax.get_xaxis_transform())
        ax.set(xticks=x,xticklabels=['S1','S2','S3','S4'],title=label);ax.grid(axis='y',alpha=.18)
    fig.suptitle('Transition and downstream endpoint dwell | separate outcomes',fontsize=18)
    fig.legend(handles=handles,loc='lower center',ncol=3,bbox_to_anchor=(.5,.015))
    fig.text(.5,.063,'N/A = no complete observed dwell (or censored metric); never replaced by the 3 s cap.',ha='center')
    fig.tight_layout(rect=(0,.105,1,.94));finish(fig,PNGS[1],values)

    fig,axes=plt.subplots(2,2,figsize=(15,9));values={}
    for i,(ax,(sid,q)) in enumerate(zip(axes.flat,summary['sources'].items())):
        entry=q['entry']['correspondence']['arc_m'];total=q['entry']['correspondence']['total_arc_m']
        values[sid]=dict(entry_arc_m=entry,total_original_arc_m=total,methods={})
        ax.axhline(entry,color='#8b4ac2',ls=':',lw=1.5)
        ax.axhline(total,color='#888',ls=':',lw=1)
        for j,name in enumerate(ORDER):
            row=q['primary_metrics'][name];first=q['selector'][name]['first']
            v=[None if first is None else first['first_selected_original_arc_m'],
               None if row is None else row['attachment_original_arc_m'],
               None if row is None else row['endpoint_dwell_projected_original_arc_m']]
            values[sid]['methods'][name]=v
            ax.plot(np.arange(3)+(j-1)*.055,[np.nan if a is None else a for a in v],color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],mfc='none',ms=8,lw=1.8)
            for k,a in enumerate(v):
                if a is None:ax.text(k+(j-1)*.18,.02,'N/A',color=COLORS[j],fontsize=8,ha='center',rotation=90,transform=ax.get_xaxis_transform())
        ax.set(title=title(i,sid),xticks=[0,1,2],xticklabels=['First H5 target','At attachment','At endpoint dwell'],ylabel='Original-FRESH arc [m]',ylim=(-.03,total*1.12),xlim=(-.3,2.3))
        ax.grid(alpha=.18)
    fig.suptitle('Original progress | entry, controller selection and executed projection',fontsize=17)
    fig.legend(handles=handles+[Line2D([],[],color='#8b4ac2',ls=':',label='C3 entry arc'),Line2D([],[],color='#888',ls=':',label='Original endpoint arc')],loc='lower center',ncol=3,bbox_to_anchor=(.5,.005))
    fig.text(.5,.09,'First H5 is a reference target; attachment/endpoint-dwell points are projections of actual robot states.',ha='center')
    fig.tight_layout(rect=(0,.125,1,.94));finish(fig,PNGS[2],values)

    fig=plt.figure(figsize=(15,9));grid=fig.add_gridspec(1,2,width_ratios=[1.25,1]);ax=fig.add_subplot(grid[0,0]);side=fig.add_subplot(grid[0,1]);side.axis('off')
    values=dict(observed=[],censored=[]);texts=[];times=[];labels=[]
    for i,(sid,q) in enumerate(summary['sources'].items()):
        for j,name in enumerate(ORDER):
            r=q['primary_metrics'][name]
            ta=None if r is None else r['sustained_attachment_s'];te=None if r is None else r['original_FRESH_endpoint_dwell_s']
            item=dict(source_id=sid,method=name,T_attach=ta,T_endpoint=te,remaining_arc_at_attach=None if r is None else r['remaining_arc_at_attachment_m'])
            tag=f"S{i+1} {['Native','Full','C3'][j]}"
            if ta is None or te is None:
                values['censored'].append(item)
                texts.append([tag,'N/A' if ta is None else f'{ta:.4f}','N/A' if te is None else f'{te:.4f}'])
                continue
            values['observed'].append(item);times.extend([ta,te])
            ax.scatter(ta,te,color=COLORS[j],marker=MARKERS[j],s=100,facecolors='none',linewidths=1.8)
            labels.append((ta,te,tag,r['remaining_arc_at_attachment_m'],COLORS[j]))
    if times:
        low=max(0,min(times)-.3);high=max(times)+.4
        ax.plot([low,high],[low,high],color='#888',ls=':',lw=1)
        xlow=max(0,min(p[0] for p in labels)-.12)
        xspan=max(.35,max(p[0] for p in labels)-xlow)
        ax.set(xlim=(xlow,xlow+2*xspan),ylim=(low,high))
        # Reserved label column with leader lines, without moving any observation.
        ranked=sorted(labels,key=lambda p:(-p[1],p[0],p[2]))
        for y,(ta,te,tag,left,color) in zip(np.linspace(.92,.12,len(ranked)),ranked):
            ax.annotate(f'{tag} | left {left:.3f} m',(ta,te),xytext=(.58,y),
                textcoords='axes fraction',fontsize=8.5,color=color,
                arrowprops=dict(arrowstyle='-',color=color,alpha=.45,lw=.7),
                bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=1))
    ax.set(xlabel='Attachment dwell start [s after B]',ylabel='Original-FRESH endpoint dwell time [s after B]')
    ax.grid(alpha=.2)
    side.set_title('Missing complete dwell — kept separate',pad=20)
    table=side.table(cellText=texts or [['None','—','—']],colLabels=['Source / method','Attach [s]','Endpoint [s]'],cellLoc='center',loc='upper center',colWidths=[.46,.27,.27])
    table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.8)
    for (rr,cc),cell in table.get_celld().items():
        cell.set_edgecolor('#d3d9e0')
        if rr==0:cell.set_facecolor('#e9edf2')
    side.text(0,.15,'No censored observation is plotted at a numeric cap.\n\nEndpoint means this original FRESH chunk’s final pose.\nIt is not a navigation task completion claim.',transform=side.transAxes,fontsize=10,linespacing=1.5)
    fig.suptitle('Earlier attachment does not establish earlier endpoint dwell',fontsize=18)
    fig.legend(handles=handles,loc='lower center',ncol=3,bbox_to_anchor=(.5,.03))
    fig.subplots_adjust(top=.86,bottom=.16,wspace=.30);finish(fig,PNGS[3],values)
    assert sorted(p.name for p in out.iterdir()) == sorted(PNGS)
    return records


def report(run):
    s,v=validate(run,deep=False)
    assert v['valid'] and read(run/'validation.json')['old_correspondence_transport_multisource_R00_R01_validators']
    figures=compact_figures(s,RESULTS/'figures')
    manifest=dict(summary_sha256=sha(run/'summary.json'),figures=figures,final_png_count=4,extra_pngs=0,new_scientific_solves=0)
    save(run/'figure_manifest.json',manifest);save(RESULTS/'figure_manifest.json',manifest)
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=s,validation=read(run/'validation.json'),
        result_hashes_sha256=sha(run/'result_hashes.json'),table_hashes={p.name:sha(p) for p in RESULTS.glob('*.csv')},
        figures=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in figures]))
    print(json.dumps(dict(figures=[str(RESULTS/'figures'/p) for p in PNGS])))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();report(args.run.resolve())
