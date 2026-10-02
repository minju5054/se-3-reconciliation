#!/usr/bin/env python3
"""Exactly four static scientific PNGs, with numeric sidecars and explicit nulls."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from run_b_to_entry_bridge01 import ROOT,RESULTS,read,save,sha
from reconciliation.b_to_entry_bridge import *
from reconciliation.relative_factor_multisource import geometry
from validate_b_to_entry_bridge01 import validate
from report_spatial_correspondence_selector_diag01 import polygons
COLORS=['#176bb2','#b25ab5','#d47a09','#00856b']
STYLES=['-','--','-.',':']
MARKERS=['o','s','^','D']
LABELS=['Native (saved)','C3 suffix (saved)','Hermite bridge','SE(2) graph bridge']


def compact_figures(summary,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.titlesize':12,'figure.dpi':150})
    handles=[Line2D([],[],color=c,ls=s,marker=m,mfc='none',label=l) for c,s,m,l in zip(COLORS,STYLES,MARKERS,LABELS)]
    records=[]
    def finish(fig,name,values):
        fig.savefig(out/name,dpi=160,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=values))
    loaded={}
    for spec in summary['selected_sources']:
        sid=spec['id'];f=Path(spec['folder']);q=summary['sources'][sid]
        refs=read(f/'final_references.json')
        methods={}
        for n,path in q['method_folders'].items():
            p=Path(path)
            methods[n]=dict(rollout=read(p/'rollout.json') if (p/'rollout.json').exists() else None,
                metrics=read(p/'metrics.json') if (p/'metrics.json').exists() else None,
                reference=np.load(refs[n]['world_path']) if n in refs else None)
        loaded[sid]=dict(folder=f,methods=methods,past=np.load(f/'recorded_old_to_B.npy'),env=geometry(f))
    fig,axes=plt.subplots(2,2,figsize=(15,12));values={}
    for i,(ax,(sid,q)) in enumerate(zip(axes.flat,summary['sources'].items())):
        d=loaded[sid];fresh=d['methods'][NATIVE]['reference'];B=np.array(q['bridge_input']['B']);E=np.array(q['bridge_input']['E'])
        clouds=[fresh[:,:2],d['past'][:,:2]];numbers={}
        for j,n in enumerate(ORDER):
            z=d['methods'][n];r=z['rollout'];m=z['metrics'];metric=q['primary_metrics'][n]
            ref=z['reference']
            if n in NEW and ref is not None:
                bridge=ref[:q['bridge_input']['M']+1];clouds.append(bridge[:,:2])
                ax.plot(*bridge[:,:2].T,color=COLORS[j],ls='--',lw=1.3,marker=MARKERS[j],ms=5,mfc='white',zorder=7)
            if r is None:continue
            xy=np.array([s['pose_world'][:2] for s in r['states']]);clouds.append(xy)
            ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],lw=2,marker=MARKERS[j],markevery=(j*3,18),ms=4.5,mfc='none',zorder=8+j)
            numbers[n]=dict(execution_xy=xy.tolist(),attachment_xy=None,endpoint_dwell_xy=None,minimum_clearance_xy=None)
            if m is not None:
                for key,marker,field in [('sustained_attachment_s','*','attachment_xy'),('original_FRESH_endpoint_dwell_s','o','endpoint_dwell_xy')]:
                    t=metric[key]
                    if t is not None:
                        point=xy[np.searchsorted(m['trace']['times_s'],t)]
                        ax.scatter(*point,marker=marker,s=155,facecolors='none',edgecolors=COLORS[j],lw=1.6,zorder=18+j)
                        numbers[n][field]=point.tolist()
                point=m['clearance']['minimum_pose'][:2]
                ax.scatter(*point,color=COLORS[j],marker='x',s=60,lw=1.5,zorder=22)
                numbers[n]['minimum_clearance_xy']=point
        cloud=np.vstack(clouds);lo=cloud.min(0)-.18;hi=cloud.max(0)+.18;env=d['env']
        if env['cart'] is not None:
            bound=np.asarray(env['cart'].bounds);lo=np.minimum(lo,bound[:2]-.26);hi=np.maximum(hi,bound[2:]+.26)
        centre=(lo+hi)/2;half=max(hi-lo)/2;lo=centre-half;hi=centre+half
        from shapely.geometry import box
        clip=box(*lo,*hi)
        polygons(ax,env['base'].obstacles.buffer(.25).intersection(clip),color='#eddddd',alpha=.7,zorder=0)
        polygons(ax,env['base'].obstacles.intersection(clip),color='#bac0c5',zorder=1)
        if env['cart'] is not None:
            polygons(ax,env['cart'].buffer(.25),color='#e7bcbc',alpha=.65,zorder=1)
            polygons(ax,env['cart'],color='#777d85',zorder=2)
        ax.plot(*d['past'][:,:2].T,color='#777',ls=':',lw=2.5,zorder=3)
        ax.plot(*fresh[:,:2].T,color='#333',ls='--',marker='.',ms=4,lw=1,zorder=4)
        ax.scatter(*B[:2],color='black',marker='P',s=80,zorder=23);ax.annotate('B',B[:2],xytext=(-14,-12),textcoords='offset points')
        ax.scatter(*E[:2],marker='D',s=120,edgecolors='#682e89',facecolors='none',lw=2,zorder=23)
        ax.scatter(*fresh[-1,:2],marker='X',color='black',s=85,zorder=23)
        ax.set(title=f'S{i+1} | {sid}',xlabel='World X [m]',ylabel='World Y [m]',xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.15)
        values[sid]=dict(original=fresh.tolist(),OLD_to_B=d['past'].tolist(),B=B.tolist(),E=E.tolist(),methods=numbers)
    extra=[Line2D([],[],color='#555',ls='--',marker='.',label='Original FRESH'),Line2D([],[],color='#777',ls=':',label='Recorded OLD to B'),
        Line2D([],[],color='#682e89',ls='',marker='D',mfc='none',label='Frozen C3 E*'),Line2D([],[],color='black',ls='',marker='X',label='Original endpoint'),
        Line2D([],[],color='black',ls='',marker='*',mfc='none',label='Attachment dwell start'),Line2D([],[],color='black',ls='',marker='o',mfc='none',label='Endpoint dwell start'),
        Line2D([],[],color='black',ls='',marker='x',label='Min swept clearance'),Patch(facecolor='#eddddd',label='Geometry + .25 m exclusion')]
    fig.suptitle('B-to-entry bridges | reference geometry and actual execution',fontsize=18,y=.995)
    fig.text(.5,.966,'Offline controlled timing. Thin dashed node paths = bridges; thick lines = execution. Overlaps retain distinct markers.',ha='center',fontsize=10)
    fig.legend(handles=handles+extra,loc='lower center',ncol=4,fontsize=9,bbox_to_anchor=(.5,.008))
    fig.subplots_adjust(top=.91,bottom=.15,hspace=.28,wspace=.24);finish(fig,PNGS[0],values)

    fig=plt.figure(figsize=(16,12));grid=fig.add_gridspec(2,2);values={}
    for i,(sid,q) in enumerate(summary['sources'].items()):
        cell=grid[i//2,i%2].subgridspec(2,1,height_ratios=[4,1.6],hspace=.42)
        ax=fig.add_subplot(cell[0]);caption=fig.add_subplot(cell[1]);caption.axis('off')
        b=q['bridge_input'];B=np.array(b['B']);E=np.array(b['E']);P=np.array(b['P']);Fnext=np.array(b['next_original_pose'])
        unit_in=unit(B[:2]-P[:2]);unit_out=unit(Fnext[:2]-E[:2]);scale=np.linalg.norm(E[:2]-B[:2])*.25
        ax.quiver(*B[:2],*(scale*unit_in),angles='xy',scale_units='xy',scale=1,color='#333',width=.006,zorder=5)
        ax.quiver(*E[:2],*(scale*unit_out),angles='xy',scale_units='xy',scale=1,color='#333',width=.006,zorder=5)
        ax.plot([B[0],E[0]],[B[1],E[1]],color='#999',ls=':',lw=1)
        lines=[];values[sid]=dict(input=b,methods=q['bridge_geometry'])
        for j,n in enumerate(NEW,2):
            ref=loaded[sid]['methods'][n]['reference']
            if ref is None:lines.append(f'{LABELS[j]}: N/A');continue
            xy=ref[:b['M']+1,:2];ax.plot(*xy.T,color=COLORS[j],ls=STYLES[j],marker=MARKERS[j],ms=7,mfc='none',lw=2)
            g=q['bridge_geometry'][n];c=g['factor_costs']
            lines.append(f"{['Hermite','Graph'][j-2]} L/chord={g['length_chord_ratio']:.3f}; turn max/RMS={np.rad2deg(g['maximum_turning_angle_rad']):.1f}/{np.rad2deg(g['RMS_turning_angle_rad']):.1f} deg\n"
                f"   edge min/max={g['segment_length_min_m']:.3f}/{g['segment_length_max_m']:.3f} m; costs in/out/smooth/space="
                f"{c['E_in']:.2f}/{c['E_out']:.2f}/{c['E_smooth']:.2f}/{c['E_space']:.2f}")
        ax.scatter(*B[:2],marker='P',color='black',s=70);ax.annotate('B',B[:2],xytext=(5,-13),textcoords='offset points')
        ax.scatter(*E[:2],marker='D',facecolors='none',edgecolors='#682e89',s=85);ax.annotate('E*',E[:2],xytext=(5,6),textcoords='offset points')
        ax.set(title=f'S{i+1} | M={b["M"]}, d_F={b["d_F_m"]:.5f} m',xlabel='World X [m]',ylabel='World Y [m]')
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.17);ax.margins(.3)
        caption.text(0,.97,'\n'.join(lines),transform=caption.transAxes,fontsize=8.8,linespacing=1.5,va='top')
    fig.suptitle('Fixed B and E* | spatial tangent and regularity diagnostics',fontsize=18)
    fig.legend(handles=handles[2:]+[Line2D([],[],color='#333',marker='>',label='Incoming / outgoing unit direction')],loc='lower center',ncol=3)
    fig.subplots_adjust(top=.90,bottom=.08,hspace=.28,wspace=.28);finish(fig,PNGS[1],values)

    fig,axes=plt.subplots(2,2,figsize=(15,9));values={};x=np.arange(4)
    for ax,(key,label) in zip(axes.flat,[('position_auc_09_m_s','Original-FRESH position AUC .9 s [m s]'),
        ('sustained_attachment_s','Sustained attachment time [s]'),('original_FRESH_endpoint_dwell_s','Original-FRESH endpoint dwell time [s]'),
        ('remaining_arc_at_attachment_m','Remaining original arc at attachment [m]')]):
        values[key]={}
        for j,n in enumerate(ORDER):
            v=[None if q['primary_metrics'][n] is None else q['primary_metrics'][n][key] for q in summary['sources'].values()];values[key][n]=v
            ax.bar(x+(j-1.5)*.19,[np.nan if a is None else a for a in v],width=.18,color=COLORS[j])
            for k,a in enumerate(v):
                if a is None:ax.text(k+(j-1.5)*.19,.025,'N/A',rotation=90,fontsize=8,ha='center',transform=ax.get_xaxis_transform())
        ax.set(xticks=x,xticklabels=['S1','S2','S3','S4'],title=label);ax.grid(axis='y',alpha=.17)
    fig.suptitle('Transition and original-FRESH endpoint dwell | distinct outcomes',fontsize=18)
    fig.legend(handles=handles,loc='lower center',ncol=4,bbox_to_anchor=(.5,.015))
    fig.text(.5,.065,'N/A is an unobserved/censored metric; it is never replaced by the execution cap.',ha='center')
    fig.tight_layout(rect=(0,.11,1,.94));finish(fig,PNGS[2],values)

    fig=plt.figure(figsize=(15,9));grid=fig.add_gridspec(1,2,width_ratios=[1.5,1]);ax=fig.add_subplot(grid[0,0]);side=fig.add_subplot(grid[0,1]);side.axis('off')
    values=dict(observed=[],censored=[]);missing=[];labels=[]
    for i,(sid,q) in enumerate(summary['sources'].items()):
        for j,n in enumerate(ORDER):
            r=q['primary_metrics'][n];a=None if r is None else r['sustained_attachment_s'];e=None if r is None else r['original_FRESH_endpoint_dwell_s']
            left=None if r is None else r['remaining_arc_at_attachment_m'];tag=f"S{i+1} {['Native','C3','Hermite','Graph'][j]}"
            v=dict(source=sid,method=n,T_attach=a,T_endpoint=e,remaining_arc_m=left)
            if a is None or e is None:
                values['censored'].append(v);missing.append([tag,'N/A' if a is None else f'{a:.4f}','N/A' if e is None else f'{e:.4f}']);continue
            values['observed'].append(v);ax.scatter(a,e,color=COLORS[j],marker=MARKERS[j],s=90,facecolors='none',lw=1.6)
            labels.append((a,e,tag,left,COLORS[j]))
    if labels:
        low=max(0,min(min(a,e) for a,e,*_ in labels)-.15);high=max(max(a,e) for a,e,*_ in labels)+.15
        ax.plot([low,high],[low,high],color='#888',ls=':',lw=1)
        xlo=min(a for a,*_ in labels)-.08;span=max(.35,max(a for a,*_ in labels)-xlo)
        ax.set(xlim=(xlo,xlo+2.4*span),ylim=(low,high))
        for yy,(a,e,tag,left,c) in zip(np.linspace(.95,.07,len(labels)),sorted(labels,key=lambda p:(-p[1],p[0],p[2]))):
            ax.annotate(f'{tag} | left {left:.3f} m',(a,e),xytext=(.55,yy),textcoords='axes fraction',fontsize=8,color=c,
                arrowprops=dict(arrowstyle='-',color=c,lw=.5,alpha=.5),bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=1))
    ax.set(xlabel='Sustained attachment [s after B]',ylabel='Original-FRESH endpoint dwell time [s after B]');ax.grid(alpha=.2)
    side.set_title('Missing complete dwell | kept separate',pad=20)
    table=side.table(cellText=missing or [['None','—','—']],colLabels=['Source / method','Attach [s]','Endpoint [s]'],loc='upper center',cellLoc='center',colWidths=[.5,.25,.25])
    table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.7)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('#d3d9e0')
        if r==0:cell.set_facecolor('#e9edf2')
    side.text(0,.16,'No censored point is plotted at a numeric cap.\n\nEndpoint refers to the original FRESH chunk.\nIt does not establish navigation task completion.',fontsize=10,linespacing=1.5)
    fig.suptitle('Attachment and downstream endpoint dwell must be assessed together',fontsize=18)
    fig.legend(handles=handles,loc='lower center',ncol=4,bbox_to_anchor=(.5,.02))
    fig.subplots_adjust(top=.88,bottom=.15,wspace=.25);finish(fig,PNGS[3],values)
    assert sorted(p.name for p in out.iterdir())==sorted(PNGS)
    return records


def report(run):
    s,v=validate(run,deep=False);assert v['valid'] and read(run/'validation.json')['all_historical_validators']
    figures=compact_figures(s,RESULTS/'figures')
    manifest=dict(summary_sha256=sha(run/'summary.json'),figures=figures,final_png_count=4,new_scientific_solves=0)
    save(run/'figure_manifest.json',manifest);save(RESULTS/'figure_manifest.json',manifest)
    save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
    save(RESULTS/'result_summary.json',dict(run=str(run),summary=s,validation=read(run/'validation.json'),
        result_hashes_sha256=sha(run/'result_hashes.json'),table_hashes={p.name:sha(p) for p in RESULTS.glob('*.csv')},
        figures=[dict(path=str(RESULTS/'figures'/r['file']),sha256=r['sha256']) for r in figures]))
    print(json.dumps(dict(figures=[str(RESULTS/'figures'/p) for p in PNGS])))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();report(args.run.resolve())
