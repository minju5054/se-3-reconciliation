#!/usr/bin/env python3
"""Saved-only descriptive influence-region panel and tables; frozen gates unchanged."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely.geometry import box
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from report_blind_corner_source import geometries
from run_join_online02 import environments
from analyze_join_online02 import csvread,pose_rows


def supplement(run):
    v=read(run/'validation.json');out=run/'review';data=[]
    for r in v['rows']:
        cid=r['candidate_id'];base,on,cart,_=environments(run/'candidates'/cid)
        if not r['mismatch']:continue
        f=r['mismatch']['influence'];center=np.array(f['origin_xy']);forward=np.array(f['forward_xy']);left=np.array([-forward[1],forward[0]])
        progress=[f['progress_min_m'],f['progress_max_m']]
        polygon=np.array([center+p*forward+l*left for p,l in [(progress[0],-2),(progress[1],-2),(progress[1],2),(progress[0],2)]])
        ep=run/'candidates'/cid/'episodes'/cid;poses=pose_rows(csvread(ep/'execution.csv'))
        fig,ax=plt.subplots(figsize=(9,8));window=box(3.2,11.3,5.7,13.7)
        geometries(ax,base.obstacles.intersection(window),color='gray',alpha=.6)
        geometries(ax,cart,color='orange',alpha=.9);geometries(ax,cart.buffer(.05),color='orange',alpha=.2)
        ax.fill(*polygon.T,color='purple',alpha=.07,label='Frozen cart-influence slab (evaluation only)')
        ax.plot(*np.array([center+progress[0]*forward-2*left,center+progress[0]*forward+2*left]).T,ls=':',c='purple')
        for label,color in [('OLD','royalblue'),('FRESH','crimson')]:
            rec=r[label]
            if rec and 'world' in rec:
                w=np.asarray(rec['world']);ax.plot(*w[:,:2].T,'--o',c=color,ms=4,label=f'Full original {label} prediction')
                ax.add_patch(Circle(w[-1,:2],.20,fill=False,color=color,lw=1,label=f'{label} endpoint footprint (prediction)'))
        ax.plot(*poses[:,:2].T,c='black',lw=3,label='Actual execution / short source postroll')
        if r['B']:ax.scatter(*r['B']['pose_world'][:2],marker='*',c='green',s=130,label='Actual B')
        ax.set(xlim=(3.2,5.7),ylim=(11.3,13.7),xlabel='World X [m]',ylabel='World Y [m]',title='C03: safe raw paths, but OLD does not enter cart interaction\nNo eligible within-region OLD/FRESH correspondence')
        ax.set_aspect('equal');ax.legend(fontsize=7,loc='lower right');fig.tight_layout()
        name=cid+'_influence_region';assert not (out/(name+'.png')).exists();fig.savefig(out/(name+'.png'),dpi=170);plt.close(fig)
        payload=dict(influence=f,polygon_xy=polygon.tolist(),OLD=r['OLD'],FRESH=r['FRESH'],execution_world=poses.tolist(),B=r['B'],
            mismatch=r['mismatch'],radius_m=.20,required_margin_m=.05,source_hashes=r['raw_source_hashes'],
            validation_sha256=sha(run/'validation.json'),changed_qualification=False)
        save(out/(name+'.json'),payload)
        assert read(out/(name+'.json'))==payload
        data.append(name)
    ledger=[]
    for g in v['geometry']['rows']:
        r=next((r for r in v['rows'] if r['candidate_id']==g['candidate_id']),None)
        a=r['clearance_attribution'] if r else {}
        ledger.append(dict(candidate_id=g['candidate_id'],geometry_qualified=g['qualified'],scientific_attempted=r is not None,
            geometry_failures=';'.join(g['failure_reasons']),scientific_failures=None if not r else ';'.join(r['failure_reasons']),
            bypass_Hospital_m=g['probes']['bypass']['Hospital_only_m'],bypass_cart_m=g['probes']['bypass']['cart_only_m'],
            OLD_Hospital_m=None if not a.get('OLD') else a['OLD']['Hospital_only_m'],OLD_cart_m=None if not a.get('OLD') else a['OLD']['cart_only_m'],
            FRESH_Hospital_m=None if not a.get('FRESH') else a['FRESH']['Hospital_only_m'],FRESH_cart_m=None if not a.get('FRESH') else a['FRESH']['cart_only_m'],
            v_minus=None if not r or not r['B'] else r['B']['u_minus'][0],omega_minus=None if not r or not r['B'] else r['B']['u_minus'][1],
            qualified=False if not r else r['qualified']))
    save(run/'aggregate/ledger.json',ledger)
    with (run/'aggregate/ledger.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(ledger[0]));w.writeheader();w.writerows(ledger)
    with (run/'aggregate/ledger.csv').open() as f:
        actual=list(csv.DictReader(f))
    for a,b in zip(actual,ledger):assert a=={k:'' if val is None else str(val) for k,val in b.items()}
    save(run/'supplemental_review_validation.json',dict(valid=True,CSV_JSON_parity=True,numeric_sidecar_parity=True,
        figures={name:sha(out/(name+'.png')) for name in data},script_sha256=sha(__file__),changed_qualification=False,model_MPC_calls=0))
    # Separate supplement index avoids overwriting the frozen reporter's products.
    with (out/'influence_index.html').open('x') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>Blind corner 02 influence audit</title><h1>'+v['classification']+'</h1><a href="index.html">Full frozen source report</a><p>Saved-only descriptive panel; no new experiment or changed gate. Colored circles are predicted endpoint footprints, not executed robots.</p>'+''.join(f'<h2>{name}</h2><a href="{name}.json">Numeric/hash sidecar</a><br><img style="max-width:100%" src="{name}.png">' for name in data))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();supplement(a.run.resolve())
