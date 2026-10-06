#!/usr/bin/env python3
"""Exactly three static scientific PNGs, generated exclusively from saved records."""
import argparse
import math
import textwrap
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from run_direct_transition_hard_eval02 import ROOT,RESULTS,read,save,sha,METHODS,PNGS,geometry
from report_spatial_correspondence_selector_diag01 import polygons
COLORS=['#98509b','#2468b4','#d0821c','#148274']
STYLES=['--','-',':','-.']
NAMES=['C3','B entry','Hermite','V2']


def figure_numbers(summary):
    numbers={}
    for spec in summary['selected_sources']:
        sid=spec['id']
        if sid not in summary['sources']:continue
        q=summary['sources'][sid];f=Path(spec['folder']);refs=read(f/'references.json')
        numbers[sid]=dict(label=spec['label'],fresh=np.load(f/'references/M0_NATIVE_world.npy').tolist(),
            OLD_to_B=np.load(f/'recorded_old_to_B.npy').tolist(),B=read(f/'common_state.json')['B'],
            E=read(f/'entry.json')['correspondence']['target_world'],
            references={n:np.load(refs[n]['world_path']).tolist() for n in METHODS},
            V2_stable=q['planning']['stable'],severity=q['severity'],
            source_category=summary['classification']['sources'][sid],
            metrics={n:None if r['metrics'] is None else {k:r['metrics'][k] for k in
                ['position_auc_09_m_s','sustained_attachment_s','original_FRESH_endpoint_dwell_s','remaining_arc_at_attachment_m']} for n,r in q['methods'].items()})
    return numbers


def render(summary,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);numbers=figure_numbers(summary);manifest=[]
    plt.rcParams.update({'font.size':10,'axes.titlesize':11})
    handles=[Line2D([],[],color=c,ls=l,label=n) for c,l,n in zip(COLORS,STYLES,NAMES)]
    def finish(fig,name):
        if not numbers:
            fig.text(.5,.5,'INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES\nNo scientific solves or rollouts; no execution evidence',ha='center',va='center',fontsize=17,bbox=dict(facecolor='white',edgecolor='gray'))
        if any(s.startswith('synthetic_') for s in numbers):
            fig.text(.5,.98,'SYNTHETIC TEST FIXTURE — NOT EXPERIMENTAL EVIDENCE',ha='center',color='#a22',fontsize=10)
        fig.savefig(out/name,dpi=160,facecolor='white');plt.close(fig)
        manifest.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=numbers))
    cols=2 if len(numbers)<=4 else 3
    fig,axes=plt.subplots(max(1,math.ceil(len(numbers)/cols)),cols,figsize=(15,9),squeeze=False)
    from shapely.geometry import box
    for ax,spec in zip(axes.flat,[s for s in summary['selected_sources'] if s['id'] in numbers]):
        sid=spec['id'];r=numbers[sid];fresh=np.array(r['fresh']);past=np.array(r['OLD_to_B'])
        cloud=np.vstack([fresh[:,:2],past[:,:2]]);lo=cloud.min(0)-.12;hi=cloud.max(0)+.12
        center=(lo+hi)/2;half=max(hi-lo)/2;lo=center-half;hi=center+half
        env=geometry(Path(spec['folder']))['on'];clip=box(*lo,*hi)
        polygons(ax,env.obstacles.buffer(.25).intersection(clip),color='#eddddd',alpha=.6)
        polygons(ax,env.obstacles.intersection(clip),color='#aab0b5')
        ax.plot(*fresh[:,:2].T,color='#333',lw=1,ls='--',marker='.',label='Original FRESH')
        ax.plot(*past[:,:2].T,color='#888',lw=2,ls=':',label='OLD to B')
        for j,n in enumerate(METHODS[1:],1):
            a=np.array(r['references'][n]);ax.plot(*a[:,:2].T,color=COLORS[j],ls=STYLES[j],lw=1.5,marker=['','x','^','o'][j],ms=4,mfc='none')
        ax.scatter(*r['B'][:2],marker='P',color='black',s=60,zorder=8)
        ax.scatter(*r['E'][:2],marker='D',facecolors='none',edgecolors='purple',s=90,zorder=9)
        gap=r['severity']['B_to_entry_chord_m'];status='stable' if r['V2_stable'] else 'INVALID DIAGNOSTIC ONLY'
        ax.set(title=f"{spec['label']} | {sid.replace('/',' / ')}\nB-E {gap:.3g} m; delta phi {np.degrees(r['severity']['delta_phi_rad']):.1f} deg\nV2 {status}",
               xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]')
        ax.set_aspect('equal');ax.grid(alpha=.15)
    for ax in list(axes.flat)[len(numbers):]:ax.axis('off')
    fig.suptitle('Source-disjoint saved-corpus geometry | fixed B / C3 entry / original suffix',fontsize=16)
    fig.legend(handles=handles[1:]+[Line2D([],[],color='#333',ls='--',label='Original FRESH'),
        Line2D([],[],color='#888',ls=':',label='OLD to B'),Line2D([],[],color='black',marker='P',ls='',label='B'),
        Line2D([],[],color='purple',marker='D',mfc='none',ls='',label='E*')],loc='lower center',ncol=7)
    fig.text(.5,.065,'Original suffix curves coincide exactly; markers distinguish rows. Invalid V2 curves are saved last iterates, never executed.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.1,1,.94));finish(fig,PNGS[0])

    fig,axes=plt.subplots(2,2,figsize=(14,9));labels=[r['label']+('\nSHARED SAFETY' if r['source_category']['shared_execution_failure'] else '') for r in numbers.values()]
    panels=[('position_auc_09_m_s','Original-FRESH position AUC .9 [m s]'),
        ('sustained_attachment_s','Sustained attachment [s after B]'),
        ('original_FRESH_endpoint_dwell_s','Original-FRESH endpoint dwell [s after B]'),
        ('remaining_arc_at_attachment_m','Original arc remaining at attachment [m]')]
    for ax,(key,title) in zip(axes.flat,panels):
        for j,n in enumerate(METHODS):
            vals=[None if r['metrics'][n] is None else r['metrics'][n][key] for r in numbers.values()]
            x=np.arange(len(vals))+(j-1.5)*.19
            ax.bar(x,[np.nan if v is None else v for v in vals],width=.18,color=COLORS[j])
            for k,v in enumerate(vals):
                if v is None:ax.text(x[k],.025,'N/A',rotation=90,ha='center',fontsize=8,transform=ax.get_xaxis_transform())
        ax.set(title=title,xticks=np.arange(len(labels)),xticklabels=labels);ax.grid(axis='y',alpha=.15)
    fig.suptitle('Actual offline execution | full ORIGINAL FRESH is the evaluation target',fontsize=16)
    fig.legend(handles=handles,loc='lower center',ncol=4)
    fig.text(.5,.065,'N/A: no complete dwell / reference skipped; SHARED SAFETY: all four guard-aborted. Endpoint is the original FRESH final pose, not a navigation goal.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.1,1,.94));finish(fig,PNGS[1])

    fig,axes=plt.subplots(2,2,figsize=(14,10));missing={'attachment':[],'endpoint dwell':[]}
    for col,axis in enumerate(['delta_phi_rad','rho_jump']):
        for row,key in enumerate(['sustained_attachment_s','original_FRESH_endpoint_dwell_s']):
            ax=axes[row,col]
            for sid,r in numbers.items():
                for j,n in enumerate([METHODS[3],METHODS[2]]):
                    v,b=r['metrics'][n],r['metrics'][METHODS[1]]
                    delta=None if v is None or b is None or v[key] is None or b[key] is None else v[key]-b[key]
                    if delta is None:
                        if col==0:missing[['attachment','endpoint dwell'][row]].append(f"{r['label']}/{['V2','Hermite'][j]}")
                        continue
                    ax.scatter(r['severity'][axis],delta,marker=['o','x'][j],color=COLORS[[3,2][j]],s=55)
                    ax.annotate(r['label'],(r['severity'][axis],delta),xytext=(5,5+j*9),textcoords='offset points',fontsize=9)
            ax.axhline(0,color='#777',lw=1,ls=':');ax.grid(alpha=.15)
            ax.set(xlabel='Direction mismatch [rad]' if col==0 else 'Relative direct jump rho = chord / d_F',
                   ylabel='Method minus B entry [s]',title='Attachment difference' if row==0 else 'Endpoint dwell difference')
    categories=[]
    for r in numbers.values():
        d=r['source_category'];cat=('graph-specific' if d['GRAPH_SPECIFIC_POSITIVE'] else 'intermediate' if d['INTERMEDIATE_GEOMETRY_POSITIVE']
            else 'SHARED SAFETY FAILURE' if d['shared_execution_failure'] else 'staging both dwells' if d['BOTH_DWELLS'][METHODS[1]] else 'mixed')
        categories.append(r['label']+': '+cat+(' / V2 invalid' if not r['V2_stable'] else ''))
    fig.suptitle('Frozen pre-outcome severity versus paired execution differences',fontsize=16)
    fig.text(.02,.073,'\n'.join(textwrap.wrap('; '.join(categories),150)),fontsize=9)
    missing_text=' | '.join(k+' N/A: '+(', '.join(v) or 'none') for k,v in missing.items())
    fig.text(.02,.033,'\n'.join(textwrap.wrap(missing_text,170)),fontsize=8)
    fig.text(.02,.012,'Descriptive only. Negative = method earlier than B entry. Circles: V2; crosses: Hermite. No population regression claim.',fontsize=9)
    fig.tight_layout(rect=(0,.115,1,.94));finish(fig,PNGS[2])
    assert sorted(p.name for p in out.glob('*.png'))==sorted(PNGS)
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    s=read(run/'summary.json');assert read(RESULTS/'validation.json')['valid']
    manifest=render(s,RESULTS/'figures')
    save(RESULTS/'figure_manifest.json',dict(summary_sha256=sha(run/'summary.json'),figures=manifest))
    print('saved exactly three PNGs; no new scientific calls')
