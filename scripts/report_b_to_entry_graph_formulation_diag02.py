#!/usr/bin/env python3
"""Three static planning diagnostic figures. No controller or optimizer calls."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from run_b_to_entry_graph_formulation_diag02 import RESULTS,read,save,sha
from reconciliation.b_to_entry_graph_diag02 import VARIANTS,FACTORS,PNGS
from validate_b_to_entry_graph_formulation_diag02 import validate

COLORS={'A2':'#777777','A3':'#be5570','V2':'#216bb2','V3':'#00856b'}
MARKERS={'A2':'x','A3':'s','V2':'o','V3':'D'}


def compact_figures(summary,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':9,'axes.titlesize':11})
    records=[]
    def finish(fig,name,sidecar):
        fig.savefig(out/name,dpi=160,facecolor='white');plt.close(fig)
        records.append(dict(file=name,sha256=sha(out/name),numeric_sidecar=sidecar))
    handles=[Line2D([],[],color=COLORS[v],marker=MARKERS[v],mfc='none',label=v+(' historical' if v=='A2' else ' new')) for v in VARIANTS]
    fig,axes=plt.subplots(2,2,figsize=(15,12));values={}
    for ax,(label,variants) in zip(axes.flat,summary['sources'].items()):
        suffix=np.array(variants['A2']['fixed_suffix']);B=np.array(variants['A2']['B']);E=np.array(variants['A2']['E'])
        ax.plot(*suffix[:,:2].T,'k.--',alpha=.5,lw=1,label='Fixed original suffix')
        inset=ax.inset_axes([.04,.54,.48,.40]);cloud=[];statuses=[]
        for v,r in variants.items():
            x=np.array(r['last_accepted_bridge']);cloud.append(x[:,:2])
            ls='-' if r['stable'] else ('--' if r['converged'] else ':')
            for a in (ax,inset):
                a.plot(*x[:,:2].T,color=COLORS[v],marker=MARKERS[v],ls=ls,lw=1.8,
                    ms=5,mfc='none',alpha=1 if r['stable'] else .65,zorder=5)
            statuses.append(v+(': STABLE' if r['stable'] else (': UNCONVERGED' if not r['converged'] else ': UNSTABLE')))
        for v,ls in [('A2','--'),('A3','-.')]:
            h=np.array(variants[v]['initial_bridge']);cloud.append(h[:,:2])
            for a in (ax,inset):a.plot(*h[:,:2].T,color='#b58b39',ls=ls,lw=1.3,marker='.',ms=4,zorder=3)
        for a in (ax,inset):
            a.scatter(*B[:2],marker='P',color='black',s=65,zorder=8)
            a.scatter(*E[:2],marker='*',color='#702570',s=100,zorder=8)
            a.set_aspect('equal',adjustable='box');a.grid(alpha=.15)
            a.ticklabel_format(useOffset=False,style='plain')
        xy=np.vstack(cloud);lo=xy.min(0);hi=xy.max(0);centre=(lo+hi)/2;half=max(hi-lo)*.62
        inset.set(xlim=(centre[0]-half,centre[0]+half),ylim=(centre[1]-half,centre[1]+half))
        inset.tick_params(labelsize=6);inset.set_title('Bridge detail',fontsize=8)
        ax.annotate('B',B[:2],xytext=(4,-13),textcoords='offset points')
        ax.annotate('E*',E[:2],xytext=(4,7),textcoords='offset points')
        world=np.vstack([xy,suffix[:,:2]])
        centre_world=(world.min(0)+world.max(0))/2
        half_world=max(world.max(0)-world.min(0))*.65
        ax.set(title=label+' | '+', '.join(statuses[:2])+'\n'+', '.join(statuses[2:]),
            xlabel='World X [m]',ylabel='World Y [m]',
            xlim=(centre_world[0]-half_world,centre_world[0]+half_world),
            ylim=(centre_world[1]-half_world,centre_world[1]+half_world))
        values[label]={v:{k:r[k] for k in ['initial_bridge','last_accepted_bridge','fixed_suffix','stable','converged','final_state_label']} for v,r in variants.items()}
    fig.suptitle('Bridge formulation matrix: angle / vector × M=2 / M=3',fontsize=17,y=.99)
    fig.text(.5,.953,'Dotted failed traces: UNCONVERGED DIAGNOSTIC — NOT A RETURNED REFERENCE',ha='center',fontsize=10,color='#9a2940')
    fig.text(.5,.93,'Solid colored bridges are STABLE planning references. No execution was performed.',ha='center',fontsize=10)
    extra=[Line2D([],[],color='#b58b39',ls='--',label='Hermite M2'),Line2D([],[],color='#b58b39',ls='-.',label='Hermite M3'),
        Line2D([],[],color='black',ls='--',marker='.',label='Fixed downstream suffix'),Line2D([],[],color='black',marker='P',ls='',label='Fixed B'),
        Line2D([],[],color='#702570',marker='*',ls='',label='Fixed E*')]
    fig.legend(handles=handles+extra,ncol=5,loc='lower center',bbox_to_anchor=(.5,.008))
    fig.subplots_adjust(top=.87,bottom=.12,hspace=.30,wspace=.25);finish(fig,PNGS[0],values)

    fig,axes=plt.subplots(2,2,figsize=(14,9));values={}
    for ax,(key,title) in zip(axes.flat,[('stable','Converged / STABLE status'),('iterations','LM iterations'),
                                       ('min_edge_over_fd','Minimum edge / FD epsilon (log10 color)'),('rho_min','Minimum optimized / initial edge ratio')]):
        data=np.array([[float(q[v][key]) for v in VARIANTS] for q in summary['sources'].values()])
        colors=np.log10(np.maximum(data,1e-30)) if key=='min_edge_over_fd' else data
        ax.imshow(colors,cmap='YlGnBu',aspect='auto')
        for i,q in enumerate(summary['sources'].values()):
            for j,v in enumerate(VARIANTS):
                r=q[v]
                label=('C / STABLE' if r['stable'] else ('C / unstable' if r['converged'] else 'NO / unstable')) if key=='stable' else (
                    f'{data[i,j]:.0f}' if key=='iterations' else f'{data[i,j]:.3g}')
                ax.text(j,i,label,ha='center',va='center',color='black',fontsize=10,
                        bbox=dict(facecolor='white',alpha=.8,edgecolor='none',pad=2))
        ax.set(title=title,xticks=range(4),xticklabels=['A2\nhistorical','A3\nnew','V2\nnew','V3\nnew'],yticks=range(4),yticklabels=['S1','S2','S3','S4'])
        values[key]=data.tolist()
    fig.suptitle('Angle: A2(M2), A3(M3) | Vector: V2(M2), V3(M3)',fontsize=17)
    fig.text(.5,.025,'Post-solve numerical gate: min edge / epsilon > 10. It never changes feasibility during LM.',ha='center')
    fig.tight_layout(rect=(0,.07,1,.93));finish(fig,PNGS[1],values)

    fig,axes=plt.subplots(4,5,figsize=(17,12));values={}
    for i,(source,q) in enumerate(summary['sources'].items()):
        values[source]={}
        for j,factor in enumerate(FACTORS):
            ax=axes[i,j];values[source][factor]={}
            for k,v in enumerate(VARIANTS):
                r=q[v];a,b=r['initial_costs'][factor],r['final_costs'][factor]
                ax.plot([k-.14,k+.14],[a,b],color=COLORS[v],lw=1.4)
                ax.scatter(k-.14,a,marker='o',s=23,facecolors='none',edgecolors=COLORS[v])
                ax.scatter(k+.14,b,marker='o' if r['stable'] else 'x',s=25,color=COLORS[v])
                values[source][factor][v]=dict(initial=a,final=b,final_state_label=r['final_state_label'])
            ax.set_yscale('symlog',linthresh=1e-5);ax.grid(axis='y',alpha=.15)
            ax.set(xticks=range(4),xticklabels=['A2†','A3','V2','V3'])
            if i==0:ax.set_title(factor.replace('E_',''))
            if j==0:ax.set_ylabel(source+' | normalized cost')
    fig.suptitle('Factor costs: Hermite initialization → last saved state',fontsize=17,y=.99)
    fig.text(.5,.958,'Open circle = initial; filled circle = STABLE final; × = diagnostic final. A2† is historical.',ha='center')
    fig.text(.5,.022,'Unconverged ×: LAST ACCEPTED ITERATE — UNCONVERGED. Boundary objectives differ; lower cost does not rank formulations or execution.',ha='center',fontsize=10,color='#9a2940')
    fig.tight_layout(rect=(0,.06,1,.925));finish(fig,PNGS[2],values)
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
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();report(args.run.resolve())
