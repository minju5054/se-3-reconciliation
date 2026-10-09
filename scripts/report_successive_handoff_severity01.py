#!/usr/bin/env python3
"""Two geometry-only PNGs. Native metrics never appear as selection evidence."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_handoff_severity01 import PNGS
from audit_successive_handoff_severity01 import OUT,namespace,validate
from continuous_obstacle_reveal_exploratory_geometry02 import environments


def polygons(ax,geom,**kw):
    if geom.geom_type=='Polygon':ax.fill(*geom.exterior.xy,**kw)
    elif hasattr(geom,'geoms'):
        for p in geom.geoms:polygons(ax,p,**kw)


def short(name):return name.replace('_to_','→').replace('C','')


def draw(summary,details,cart,out):
    out.mkdir(parents=True,exist_ok=False)
    rows=summary['rows'];selected=summary['selection']['recommended']
    plt.rcParams.update({'font.size':10,'axes.spines.right':False,'axes.spines.top':False})
    colors={'TURN':'#b94934','PROGRESS':'#7b52ad','REVISION':'#25835c','STABLE_CONTROL':'#757575',None:'#2d69a5'}
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    for i,r in enumerate(rows):
        label=short(r['handoff']);color=colors[r['candidate_role']];marker='D' if r['evolution']=='STABLE' else 'o'
        xy=[(r['B_to_E_gap_m'],np.degrees(r['abs_transition_turn_demand_rad'])),
            (r['removed_prefix_fraction'],r['local_polyline_separation_m'])]
        for ax,(x,y) in zip(axes,xy):
            ax.scatter(x,y,c=color,marker=marker,s=55,edgecolors='white',zorder=4)
            ax.annotate(label,(x,y),xytext=(6,4+(i%3)*7),textcoords='offset points',fontsize=8)
        axes[2].bar(i,np.degrees(r['fresh_suffix_abs_net_yaw_rad']),color=color,
                    hatch='//' if r['evolution']=='STABLE' else None)
    axes[0].set(xlabel='B→E* gap [m]',ylabel='Absolute OLD→FRESH tangent mismatch [deg]',title='Gap and immediate turn')
    axes[1].set(xlabel='Removed original arc fraction',ylabel='Existing local separation [m]',title='Progress and revision')
    axes[2].set(xlabel='OLD→FRESH handoff',ylabel='Absolute suffix net yaw [deg]',title='Original FRESH future turn')
    axes[2].set_xticks(range(10),[short(r['handoff']) for r in rows],rotation=45)
    for ax in axes:ax.grid(alpha=.2);ax.margins(.16)
    handles=[Line2D([],[],color=colors[k],marker='o' if k!='STABLE_CONTROL' else 'D',ls='',label=k or 'Other EVOLVING') for k in ['TURN','PROGRESS','REVISION','STABLE_CONTROL',None]]
    fig.legend(handles=handles,loc='lower center',ncol=5)
    fig.suptitle('10 post-reveal handoffs | geometry-only screening | no combined difficulty score')
    fig.tight_layout(rect=(0,.09,1,.94));fig.savefig(out/PNGS[0],dpi=170);plt.close(fig)
    names=[*selected.values(),*summary['stable_controls']]
    fig,axes=plt.subplots(2,3,figsize=(13,10))
    for ax,name in zip(axes.flat,names):
        d=details[name];r=next(r for r in rows if r['handoff']==name)
        F=np.array(d['FRESH_world']);B=np.array(d['B']);P=np.array(d['P']);E=np.array(r['E_star_world'])
        polygons(ax,cart.buffer(.20),color='#eddbc3',alpha=.65)
        polygons(ax,cart,color='#888888',alpha=.8)
        ax.plot(F[:,0],F[:,1],color='#222222',lw=1.6,marker='.',ms=4)
        ax.plot([P[0],B[0]],[P[1],B[1]],color='#2778a9',lw=3,marker='o',ms=4)
        ax.scatter(*B[:2],marker='s',s=45,c='#2778a9',zorder=5)
        ax.scatter(*E[:2],marker='*',s=105,c='#dfab36',edgecolors='black',zorder=6)
        ax.annotate('B',B[:2],xytext=(6,8),textcoords='offset points')
        ax.annotate('E*',E[:2],xytext=(7,-14),textcoords='offset points')
        ax.set_title(f"{short(name)} | {r['candidate_role']}\ngap {r['B_to_E_gap_m']:.4f} m; turn {np.degrees(r['abs_transition_turn_demand_rad']):.2f}°",fontsize=10)
        ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2);ax.margins(.12)
    ax=axes.flat[-1];ax.axis('off')
    ax.legend(handles=[Line2D([],[],color='#222',marker='.',label='Original observation-anchored FRESH'),
        Line2D([],[],color='#2778a9',marker='o',label='Actual incoming P→B'),
        Line2D([],[],color='#2778a9',marker='s',ls='',label='B switch boundary'),
        Line2D([],[],color='#dfab36',marker='*',ls='',label='Frozen C3 entry E*'),
        Line2D([],[],color='#888',lw=8,label='Cart'),Line2D([],[],color='#eddbc3',lw=8,label='Cart + 0.20 m footprint')],loc='center',fontsize=10)
    ax.text(.05,.15,'Geometry only.\nNo Graph, Hermite or B_ENTRY execution.\nSTABLE refers to raw local output equality.\nA different observation anchor can still\nproduce a different world path.',transform=ax.transAxes,fontsize=10)
    fig.suptitle('Recommended complementary candidates and stable controls | one DEVELOPMENT SOURCE')
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/PNGS[1],dpi=170);plt.close(fig)
    return dict(files=[dict(path=str(out/n),sha256=sha(out/n)) for n in PNGS],figure_count=2,
        geometry_only=True,recommended=selected,stable_controls=summary['stable_controls'],
        numeric_sidecar=[{k:r[k] for k in ['handoff','evolution','candidate_role','B_to_E_gap_m',
            'abs_transition_turn_demand_rad','removed_prefix_fraction','local_polyline_separation_m','fresh_suffix_abs_net_yaw_rad']} for r in rows])


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args()
    run=namespace(a.run);validate(run)
    if a.check:
        m=read(OUT/'figure_manifest.json');assert sorted(p.name for p in (OUT/'figures').glob('*.png'))==sorted(PNGS)
        for row in m['files']:assert sha(row['path'])==row['sha256']
    else:
        s=read(OUT/'result_summary.json');cart=environments(Path(s['source']))[2]
        save(OUT/'figure_manifest.json',draw(s,read(run/'geometry_details.json'),cart,OUT/'figures'))
    print('Two saved-only geometry PNGs checked')


if __name__=='__main__':main()
