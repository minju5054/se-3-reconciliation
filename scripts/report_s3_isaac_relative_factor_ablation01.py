#!/usr/bin/env python3
"""Exactly two saved-only S3 scientific PNGs; no new solve or execution."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import box
from validate_s3_isaac_relative_factor_ablation01 import validate,read,save,sha,OUT,namespace,geometry
from reconciliation.s3_isaac_ablation01 import ORDER,GRAPHS,PNGS
from reconciliation.spatial_correspondence_selector import wrap
COLORS={'RAW':'#2e70ad','B_ENTRY':'#178267','FULL_GRAPH':'#874bb6','GRAPH_NO_R':'#df7525'}
MARKERS={'RAW':'o','B_ENTRY':'s','FULL_GRAPH':'^','GRAPH_NO_R':'x'}


def polygons(ax,g,**kwargs):
    if g.geom_type=='Polygon':ax.fill(*g.exterior.xy,**kwargs)
    elif hasattr(g,'geoms'):
        for child in g.geoms:polygons(ax,child,**kwargs)


def render(run,s,traces):
    out=OUT/'figures';out.mkdir(exist_ok=False)
    F=np.load(run/'original.npy');E=np.array(s['entry']['target_world']);B=np.array(s['common']['B']);P=np.array(read(run/'source_authentication.json')['P'])
    past=np.load(run/'recorded_old_to_B.npy');refs=read(run/'references.json');env=geometry(run)['on']
    rolls={n:read(run/'methods'/n/'rollout.json') for n in ORDER if (run/'methods'/n/'rollout.json').exists()}
    paths={n:np.array([v['pose_world'] for v in r['states']]).reshape(-1,3) for n,r in rolls.items()}
    plt.rcParams.update({'font.size':10,'axes.spines.right':False,'axes.spines.top':False})
    fig,axes=plt.subplots(1,2,figsize=(13,6))
    allxy=np.vstack([F[:,:2],past[:,:2],*[np.load(r['world_path'])[:,:2] for r in refs.values()],*[p[:,:2] for p in paths.values() if len(p)]])
    for panel,ax in enumerate(axes):
        pts=allxy if panel==0 else np.vstack([B[:2],E[:2],F[2:6,:2],*[p[:min(55,len(p)),:2] for p in paths.values() if len(p)]])
        lo=pts.min(axis=0)-.18;hi=pts.max(axis=0)+.18;clip=box(*lo,*hi)
        polygons(ax,env.obstacles.buffer(.25).intersection(clip),color='#eddab6',alpha=.6)
        polygons(ax,env.obstacles.intersection(clip),color='#888888',alpha=.8)
        ax.plot(past[:,0],past[:,1],color='#757575',lw=2,label='Saved OLD-to-B')
        ax.plot(F[:,0],F[:,1],color='black',ls=':',lw=2,marker='.',ms=4,label='Original FRESH')
        for j,n in enumerate(ORDER):
            if n in refs and n!='RAW':
                w=np.load(refs[n]['world_path']);ax.plot(w[:,0],w[:,1],color=COLORS[n],ls='--',lw=1,alpha=.8,marker=MARKERS[n],ms=3,label=n+' reference')
            p=paths.get(n)
            if p is not None and len(p):
                ax.plot(p[:,0],p[:,1],color=COLORS[n],lw=1.6,marker=MARKERS[n],markevery=(j*3,15),ms=4,label=n+' Isaac')
        ax.scatter(*B[:2],c='black',marker='s',s=50,zorder=8,label='B')
        ax.scatter(*E[:2],c='#f6cf48',edgecolors='black',marker='*',s=130,zorder=8,label='Frozen E*')
        ax.annotate('B',B[:2],xytext=(-15,-15),textcoords='offset points');ax.annotate('E*',E[:2],xytext=(8,10),textcoords='offset points')
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title='Full saved geometry and 3 s execution' if panel==0 else 'Transition detail; same world frame')
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=5,fontsize=8)
    missing=[n for n in ORDER if n not in paths or not len(paths[n])]
    fig.suptitle('S3 actual Isaac | '+s['classification'],fontsize=12)
    fig.text(.5,.125,'Hospital obstacles + 0.25 m centre exclusion; no added cart. Solid=actual execution, dashed=derived reference.'+('\nExecution N/A: '+', '.join(missing) if missing else ''),ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.17,1,.94));fig.savefig(out/PNGS[0],dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    for n,trace in traces.items():
        t=np.array(trace['time_s']);p=paths[n]
        axes[0,0].plot(t,trace['position_error_m'],color=COLORS[n],label=n,marker=MARKERS[n],markevery=20,ms=3)
        axes[0,1].plot(t,np.degrees(trace['yaw_error_rad']),color=COLORS[n],label=n,marker=MARKERS[n],markevery=20,ms=3)
        delta=s['primary_metrics'][n]['turn50_03']['delta_psi_F_rad']
        if abs(delta)>1e-12:
            q=np.sign(delta)*wrap(p[:,2]-B[2])/abs(delta);axes[1,0].plot(t,q,color=COLORS[n],label=n,marker=MARKERS[n],markevery=20,ms=3)
    axes[0,0].set(ylabel='Original-FRESH position error [m]',title='Position tracking')
    axes[0,1].set(ylabel='Original-FRESH yaw error [deg]',title='Yaw tracking')
    axes[1,0].set(ylabel='Signed heading progress q(t)',title='Turn response: two saved samples required')
    axes[1,0].axhline(.5,color='black',ls=':',label='q=0.5')
    for ax in [axes[0,0],axes[0,1],axes[1,0]]:
        ax.axvspan(0,.3,color='#dddddd',alpha=.3);ax.axvline(.9,color='#999999',lw=.8,ls=':');ax.set_xlabel('Actual execution time after B [s]');ax.grid(alpha=.2)
        if not traces:ax.text(.5,.5,'N/A — no execution',transform=ax.transAxes,ha='center')
    ax=axes[1,1];twin=ax.twinx();twin.spines['right'].set_visible(True)
    for n in GRAPHS:
        p=s['planning'][n];d=p['deformation']
        if d is None:continue
        label=n+(' (invalid planning candidate)' if not p['valid'] else '')
        ax.plot(d['per_node_translation_m'],color=COLORS[n],marker=MARKERS[n],label=label+' XY')
        twin.plot(np.degrees(d['per_node_yaw_correction_rad']),color=COLORS[n],ls='--',alpha=.7,label=label+' yaw')
    ax.set(xlabel='Original suffix node index (0 replaced by fixed B)',ylabel='Translation correction [m]',title='FULL vs NO_R deformation');twin.set_ylabel('Yaw correction [deg]');ax.grid(alpha=.2)
    h,l=axes[0,0].get_legend_handles_labels()
    dh,dl=ax.get_legend_handles_labels()
    fig.legend(h+dh,l+dl,loc='lower center',ncol=3,fontsize=8)
    diagnostic=['Solid: XY; dashed: yaw. Relative-edge RMS (XY / yaw):']
    for n in GRAPHS:
        d=s['planning'][n]['deformation']
        if d is not None:
            r=d['relative_distortion'];diagnostic.append(f"{n}: {r['translation_RMS_m']:.4f} m / {np.degrees(r['yaw_RMS_rad']):.2f} deg")
    diagnostic.append('NO_R R diagnostic is not optimized.')
    ax.text(.02,.04,'\n'.join(diagnostic),transform=ax.transAxes,va='bottom',fontsize=7,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
    fig.suptitle('S3 response and deformation | shaded interval = primary 0.30 s',fontsize=12)
    fig.tight_layout(rect=(0,.10,1,.95));fig.savefig(out/PNGS[1],dpi=170);plt.close(fig)
    separation={}
    for i,a in enumerate(ORDER):
        for b in ORDER[i+1:]:
            pa,pb=paths.get(a),paths.get(b)
            separation[a+' vs '+b]=None if pa is None or pb is None or len(pa)!=len(pb) else float(np.linalg.norm(pa[:,:2]-pb[:,:2],axis=1).max())
    return dict(files=[dict(path=str(out/n),sha256=sha(out/n)) for n in PNGS],figure_count=2,
        classification=s['classification'],metrics=s['primary_metrics'],planning_deformation={n:p['deformation'] for n,p in s['planning'].items()},
        max_matched_XY_separation_m=separation,missing_execution=missing,no_new_scientific_calls=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();run=namespace(a.run)
    s,v,traces=validate(run);assert s==read(OUT/'result_summary.json')
    if a.check:
        m=read(OUT/'figure_manifest.json');assert sorted(p.name for p in (OUT/'figures').glob('*.png'))==sorted(PNGS)
        for row in m['files']:assert sha(row['path'])==row['sha256']
        assert m['metrics']==s['primary_metrics'] and m['planning_deformation']=={n:p['deformation'] for n,p in s['planning'].items()}
    else:save(OUT/'figure_manifest.json',render(run,s,traces))
    print('Exactly two saved-only scientific PNGs validated')


if __name__=='__main__':main()
