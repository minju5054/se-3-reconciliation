#!/usr/bin/env python3
"""Two compact PNGs from saved-only validated data; unavailable executions stay N/A."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_isaac_four_method01 import ORDER,PNGS
from continuous_obstacle_reveal_exploratory_geometry02 import environments
from validate_successive_isaac_four_method01 import validate
from run_successive_isaac_four_method01 import OUT,namespace
COLORS=['#2459ac','#c45b13','#008473','#9647a4']
MARKERS=['o','s','^','x']
STYLES=['-',(0,(7,2)),(0,(2,1)),(0,(5,1,1,1))]


def polygons(ax,geom,**kw):
    if geom.is_empty:return
    if geom.geom_type=='Polygon':ax.fill(*geom.exterior.xy,**kw)
    elif hasattr(geom,'geoms'):
        for g in geom.geoms:polygons(ax,g,**kw)


def draw(run,s,out):
    out.mkdir(parents=True,exist_ok=False)
    cfg=read(run/'protocol.json');c=read(run/'common_state.json');B=np.array(c['B'])
    F=np.load(run/'references/RAW_world.npy');old=np.load(run/'old.npy');E=np.array(s['entry']['correspondence']['target_world'])
    refs=read(run/('references.json' if (run/'references.json').exists() else 'prepared_references.json'))
    paths={}
    for n in ORDER:
        p=run/'methods'/n/'rollout.json'
        if p.exists():paths[n]=np.array([r['pose_world'] for r in read(p)['states']])
    base,on,cart,_=environments(ROOT/cfg['source'])
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,2,figsize=(12,7),gridspec_kw={'width_ratios':[1,1.2]})
    for panel,ax in enumerate(axes):
        polygons(ax,cart.buffer(.20),color='#ecd8bd',alpha=.8,label='Cart + 0.20 m footprint')
        polygons(ax,cart,color='#73777d',alpha=.9,label='Cart')
        ax.plot(old[:,0],old[:,1],color='#777777',ls=':',lw=2,label='Original OLD C0')
        ax.plot(F[:,0],F[:,1],color='#151515',lw=2,alpha=.55,label='Original FRESH C1')
        for j,n in enumerate(ORDER):
            if n in refs:
                w=np.load(refs[n]['world_path']);ax.plot(w[:,0],w[:,1],color=COLORS[j],ls='--',lw=1,alpha=.55)
            p=paths.get(n)
            if p is not None and len(p):
                ax.plot(p[:,0],p[:,1],color=COLORS[j],ls=STYLES[j],lw=1.8,marker=MARKERS[j],
                    markevery=(j,5),ms=5,mfc='none',zorder=5+j,label=n+' executed')
        ax.scatter(*B[:2],marker='D',c='black',s=45,zorder=15)
        ax.scatter(*E[:2],marker='*',c='#e3a827',edgecolors='black',s=95,zorder=16)
        ax.annotate('B1',B[:2],xytext=(8,7),textcoords='offset points')
        ax.annotate('E*=F0',E[:2],xytext=(8,-14),textcoords='offset points')
        ax.add_patch(Circle(B[:2],.20,fill=False,color='#666666',lw=.9,ls=':'))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2);ax.set_xlabel('World X [m]');ax.set_ylabel('World Y [m]')
        if panel==0:
            ax.set_title('Source geometry and complete references')
            bounds=np.asarray(cart.bounds)
            xy=np.vstack([F[:,:2],old[:,:2],B[None,:2],bounds[:2],bounds[2:]])
            ax.set_xlim(xy[:,0].min()-.28,xy[:,0].max()+.28);ax.set_ylim(xy[:,1].min()-.28,xy[:,1].max()+.28)
        else:
            ax.set_title('Actual Isaac transition: ticks 92–119')
            ax.set_xlim(B[0]-.13,B[0]+.13);ax.set_ylim(B[1]-.235,B[1]+.045)
    handles=[Line2D([],[],color='#777',ls=':',label='OLD C0'),Line2D([],[],color='#111',label='Original C1'),
        Line2D([],[],color='#777',ls='--',label='Method reference')]
    handles += [Line2D([],[],color=COLORS[j],marker=MARKERS[j],ls=STYLES[j],label=n+' execution' if n in paths else n+' execution N/A') for j,n in enumerate(ORDER)]
    fig.legend(handles=handles,loc='lower center',ncol=4,bbox_to_anchor=(.5,.01))
    missing=[n for n in ORDER if n not in paths]
    fig.suptitle('Frozen C0→C1 | one development handoff | '+s['classification'],fontsize=13)
    fig.text(.5,.108,'Footprint radius 0.20 m. References dashed; executed paths use distinct markers. '+('Not executed: '+', '.join(missing) if missing else 'All methods start with the same held command.'),ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.13,1,.94));fig.savefig(out/PNGS[0],dpi=180);plt.close(fig)
    keys=[('position_auc_full_m_s','Position AUC, full window [m s]'),('yaw_auc_full_rad_s','Yaw AUC, full window [rad s]'),
        ('position_auc_03_m_s','Position AUC, 0.3 s [m s]'),('yaw_auc_03_rad_s','Yaw AUC, 0.3 s [rad s]'),
        ('abs_delta_v_from_u_B_plus','First NEW |Δv| [m/s]'),('abs_delta_omega_from_u_B_plus','First NEW |Δω| [rad/s]'),
        ('linear_TV','Post-B linear command TV [m/s]'),('angular_TV','Post-B angular command TV [rad/s]'),
        ('swept_clearance_lower_bound_m','Executed swept clearance bound [m]')]
    fig,axes=plt.subplots(3,3,figsize=(13,10));numbers={}
    for ax,(key,title) in zip(axes.flat,keys):
        vals=[None if s['primary_metrics'][n] is None else s['primary_metrics'][n][key] for n in ORDER];numbers[key]=dict(zip(ORDER,vals))
        for i,value in enumerate(vals):
            if value is None:ax.text(i,.05,'N/A',ha='center',transform=ax.get_xaxis_transform())
            else:
                ax.bar(i,value,color=COLORS[i],width=.65)
                ax.annotate(f'{value:.5g}',(i,value),xytext=(0,4),textcoords='offset points',ha='center',fontsize=8)
        ax.set_xticks(range(4),['RAW','B_ENTRY','Hermite','Graph'],rotation=15);ax.set_title(title,fontsize=10)
        ax.margins(y=.23);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Original immutable FRESH target | actual execution time | no scalar winner',fontsize=14)
    fig.text(.5,.015,'Full window = 28 saved integration intervals. N/A is censored/unexecuted. Shared B command is excluded from first NEW command differences.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(out/PNGS[1],dpi=170);plt.close(fig)
    separation={}
    for i,a in enumerate(ORDER):
        for z in ORDER[i+1:]:
            if a in paths and z in paths and len(paths[a])==len(paths[z]):
                separation[a+'_vs_'+z]=float(np.linalg.norm(paths[a][:,:2]-paths[z][:,:2],axis=1).max())
    return dict(files=[dict(path=str(out/n),sha256=sha(out/n)) for n in PNGS],numeric_sidecar=numbers,
        max_matched_execution_XY_separation_m=separation,final_png_count=2,new_scientific_calls=0)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();run=namespace(a.run)
    s,v,_=validate(run);assert read(OUT/'result_summary.json')==s and v['valid']
    if a.check:
        m=read(OUT/'figure_manifest.json');assert sorted(p.name for p in (OUT/'figures').glob('*.png'))==sorted(PNGS)
        for r in m['files']:assert sha(r['path'])==r['sha256']
    else:
        save(OUT/'figure_manifest.json',draw(run,s,OUT/'figures'))
        save(run/'result_hashes.json',{str(p.relative_to(run)):sha(p) for p in run.rglob('*') if p.is_file()})
        save(OUT/'raw_result_integrity.json',dict(run=str(run),hash_manifest_sha256=sha(run/'result_hashes.json')))
    print('Two saved-only PNGs validated; no new solves')


if __name__=='__main__':main()
