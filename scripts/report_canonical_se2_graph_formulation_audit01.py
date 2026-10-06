#!/usr/bin/env python3
"""Exactly four compact planning-only PNGs from saved arrays; no scientific solve."""
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from validate_canonical_se2_graph_formulation_audit01 import RESULTS,PNGS,read,sha,write_json,figure_data,planning_guard
COLORS=dict(raw='#252525',transported='#929292',B_ENTRY='#d5a000',HERMITE='#00a082',V2_SINGLE_NODE='#a142a4',canonical='#1874c8')


def report(run,output=RESULTS):
    with planning_guard():
        summary=read(run/'comparison_summary.json');data=figure_data(run,summary);sources=data['sources'];labels=list(sources)
        figdir=output/'figures';figdir.mkdir(parents=True,exist_ok=True)
        plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
        def finish(fig,name):
            fig.savefig(figdir/name,dpi=160,facecolor='white');plt.close(fig)
        fig,axes=plt.subplots(2,3,figsize=(14,9),layout='constrained')
        handles=[]
        for ax,label in zip(axes.flat,labels):
            s=sources[label]
            for key,style,marker in [('raw','-',None),('transported',':',None),('B_ENTRY','--','.'),('HERMITE','-.','s'),('V2_SINGLE_NODE',':','x'),('canonical','-','o')]:
                a=np.array(s.get(key,s['baselines'].get(key)));line,=ax.plot(a[:,0],a[:,1],style,color=COLORS[key],marker=marker,markersize=3,linewidth=1.6,label=key)
                if label==labels[0]:handles.append(line)
            for key,m in [('A','^'),('B','D')]:ax.scatter(*s[key][:2],marker=m,c='black',s=35);ax.annotate(key,s[key][:2],xytext=(5,5),textcoords='offset points')
            ax.set(title=label,xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal',adjustable='datalim');ax.margins(.14);ax.grid(alpha=.2)
        axes.flat[-1].axis('off');axes.flat[-1].legend(handles=handles,loc='center',fontsize=11)
        axes.flat[-1].text(.05,.12,'Reference geometry only.\nA/B fixed; no execution.\nBridge suffix curves overlap exactly.\nCanonical: all original rows editable.',transform=axes.flat[-1].transAxes)
        fig.suptitle('Canonical full-FRESH formulation versus fixed-suffix bridges',fontsize=14)
        finish(fig,PNGS[0])
        fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
        for ax,label in zip(axes.flat,labels):
            n=sources[label]['nodes'];s=[r['s'] for r in n]
            ax.plot(s,[r['world_XY_displacement_m'] for r in n],'-o',ms=3,label='distance to raw',color=COLORS['canonical'])
            ax.plot(s,[r['target_world_XY_displacement_m'] for r in n],'--s',ms=3,label='distance to target',color='#d55e00')
            ax.set(title=label,xlabel='original XY arc fraction',ylabel='XY displacement [m]');ax.set_ylim(bottom=0);ax.grid(alpha=.2)
            twin=ax.twinx();twin.plot(s,[r['w_L'] for r in n],':',c='gray',label='w_L');twin.plot(s,[r['w_A'] for r in n],'-.',c='gray',label='w_A');twin.set_ylim(0,1.05);twin.set_ylabel('factor weight',color='gray')
        axes.flat[-1].axis('off');axes.flat[-1].text(.08,.65,'Blue: distance to original FRESH\nOrange: distance to transported target\nGray dotted: w_L = (1-s)^2\nGray dash-dot: w_A = s^2\n\nSpatial progress; no waypoint time.',transform=axes.flat[-1].transAxes,fontsize=11,va='top')
        fig.suptitle('Canonical correction and frozen spatial weights',fontsize=14);finish(fig,PNGS[1])
        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained');xx=np.arange(len(labels))
        ds=[summary['sources'][k]['diagnostics'] for k in labels]
        for ax,key,title,scale in [(axes[0,0],'relative_translation_RMS_m','Relative-edge translation RMS [m]',1),(axes[0,1],'relative_yaw_RMS_rad','Relative-edge yaw RMS [deg]',180/np.pi)]:
            ax.bar(xx,[d[key]*scale for d in ds],color=COLORS['canonical']);ax.set(title=title,xticks=xx,xticklabels=labels);ax.grid(axis='y',alpha=.2)
        ax=axes[1,0];ax.bar(xx,[d['rigid_fit']['translation_RMS_m'] for d in ds],color='#a142a4');ax.set(title='Best XY rigid-fit residual [m]',xticks=xx,xticklabels=labels);ax.grid(axis='y',alpha=.2)
        ax=axes[1,1];ax.bar(xx-.17,[d['first_node_correction_m'] for d in ds],.34,label='first node');ax.bar(xx+.17,[d['endpoint_correction_m'] for d in ds],.34,label='endpoint');ax.set(title='Correction to original FRESH [m]',xticks=xx,xticklabels=labels);ax.legend();ax.grid(axis='y',alpha=.2)
        fig.suptitle('Relative distortion, non-rigidity and downstream recovery',fontsize=14);finish(fig,PNGS[2])
        fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
        for ax,label in zip(axes.flat,[k for k in labels if k!='OSA03_R00']):
            s=sources[label];h=np.array(s['baselines']['HERMITE'])[:3];v=np.array(s['baselines']['V2_SINGLE_NODE'])[:3];base=h[0,:2]
            # Offset and millimetres make the small bridge difference readable.
            for a,key,style,mark in [(h,'HERMITE','-','s'),(v,'V2_SINGLE_NODE','--','x')]:
                xy=1000*(a[:,:2]-base);ax.plot(*xy.T,style,color=COLORS[key],marker=mark,label=key)
            q=summary['V2_Hermite_similarity'][label];ax.set(title=f"{label}: X1 difference {q['X1_XY_difference_m']*1000:.3f} mm",xlabel='x - B.x [mm]',ylabel='y - B.y [mm]')
            for i,text in [(0,'B'),(2,'E*')]:ax.annotate(text,1000*(h[i,:2]-base),xytext=(6,6),textcoords='offset points')
            ax.set_aspect('equal',adjustable='datalim');ax.margins(.20);ax.grid(alpha=.2);ax.legend(loc='best')
        fig.suptitle('Saved V2 versus Hermite: fixed endpoints, one editable interior pose',fontsize=14);finish(fig,PNGS[3])
        write_json(output/'figure_numeric.json',data)
        write_json(output/'figure_manifest.json',dict(numeric_sha256=sha(output/'figure_numeric.json'),figures=[dict(file=p,sha256=sha(figdir/p)) for p in PNGS],scope='planning only',count=4))
        assert sorted(p.name for p in figdir.iterdir())==sorted(PNGS)
        return dict(count=4,paths=[str(figdir/p) for p in PNGS])


if __name__=='__main__':
    from pathlib import Path
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();print(report(args.run.resolve()))
