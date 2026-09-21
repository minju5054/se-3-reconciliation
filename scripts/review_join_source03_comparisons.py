#!/usr/bin/env python3
"""Paired scientific figures from immutable SOURCE03 outputs only."""
import argparse
import csv
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import shapely

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from join_source03_presentation import collection_plot


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve();out=run/'review_comparisons';out.mkdir(exist_ok=False)
    groups=[('lighting',['core_dark_H16','core_bright_H16']),
            ('history',['target_bright_H16','target_bright_H32']),
            ('distance',['distance_near_H16','distance_medium_H16','distance_far_H16'])]
    figures=[]
    for name,cids in groups:
        if not all((run/'paired_diagnostics'/c/'evaluation.json').exists() for c in cids):continue
        records=[read(run/'paired_diagnostics'/c/'evaluation.json') for c in cids]
        inputs=[read(run/'input_manifests'/f'{c}.json') for c in cids]
        fig,axs=plt.subplots(2,len(cids),figsize=(5*len(cids),7.5),squeeze=False)
        allxy=np.concatenate([np.asarray(r['details'][b]['world'])[:,:2] for r in records for b in ['A','B']]+[np.array([m['observation_pose_world'][:2]]) for m in inputs])
        lo=allxy.min(0)-.55;hi=allxy.max(0)+.55
        for j,(cid,r,m) in enumerate(zip(cids,records,inputs)):
            ax=axs[0,j];d=r['details']['B'];w=np.asarray(d['world']);off=np.asarray(r['details']['A']['world'])
            cart=shapely.from_wkb(bytes.fromhex(m['projection']['obstacle_wkb_hex']))
            collection_plot(cart,ax=ax,color='black',alpha=.75,add_points=False)
            collection_plot(cart.buffer(.25),ax=ax,facecolor='none',edgecolor='gray',linestyle=':',add_points=False)
            ax.plot(off[:,0],off[:,1],'--',c='tab:blue',label='OFF prediction')
            ax.plot(w[:,0],w[:,1],'-o',c='tab:red',ms=3,label='ON prediction')
            ax.scatter(*m['observation_pose_world'][:2],c='black',marker='x',label='Observation pose')
            unsafe=d['geometry_on']['first_unsafe_waypoint_zero_based']
            if unsafe is not None:
                ax.add_patch(Circle(w[unsafe,:2],.2,fill=False,edgecolor='tab:red',lw=1.5,label='20cm footprint at first unsafe row'))
            ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X (m)',ylabel='World Y (m)',title=cid)
            ax.set_aspect('equal')
            ax=axs[1,j];ax.plot(np.arange(len(w)),d['geometry_on']['node_clearance_m'],'o-',c='tab:red')
            ax.axhline(.05,c='black',ls='--',label='required 5cm');ax.axhline(0,c='gray')
            ax.set(xlabel='Original row index — not time',ylabel='Footprint-edge clearance (m)',ylim=(-.22,.65),
                   title=f'Min swept margin {d["geometry_on"]["whole"]["minimum_clearance_m"]:.6f}m')
        handles,labels=axs[0,0].get_legend_handles_labels()
        fig.legend(handles,labels,loc='lower center',ncol=2,fontsize=9)
        fig.suptitle(name.upper()+' | saved model predictions, never executed\nDotted cart outline = robot-center exclusion (radius .20m + clearance .05m)',fontsize=12)
        fig.tight_layout(rect=(0,.07,1,.93))
        png=out/(name+'_comparison.png');fig.savefig(png,dpi=170);plt.close(fig)
        side=dict(conditions=cids,records=records,inputs=inputs,config_sha256=sha(run/'config_snapshot.yaml'),
                  source_sha256={str(run/'paired_diagnostics'/c/'evaluation.json'):sha(run/'paired_diagnostics'/c/'evaluation.json') for c in cids},
                  renderer_sha256=sha(__file__),equal_world_axes=True,not_executed=True)
        save(out/(name+'_comparison.json'),side)
        figures.append(dict(png=png.name,png_sha256=sha(png),sidecar=name+'_comparison.json'))
    # Explicit row table; no reinterpretation as physical waypoint time.
    rows=[]
    for row in read(run/'aggregate/ledger.json'):
        if row['status']!='COMPLETED':continue
        cid=row['condition_id'];r=read(run/'paired_diagnostics'/cid/'evaluation.json')
        for alias,d in r['details'].items():
            for i,(local,world,clearance) in enumerate(zip(d['raw_local'],d['world'],d['geometry_on']['node_clearance_m'])):
                rows.append(dict(condition=cid,branch=alias,row_index=i,local_forward_m=local[0],local_left_m=local[1],
                    local_yaw_rad=local[2],world_x_m=world[0],world_y_m=world[1],world_yaw_rad=world[2],
                    edge_clearance_m=clearance,source_world_sha256=d['world_sha256']))
    with (run/'aggregate/row_geometry.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    save(out/'figure_manifest.json',figures)
    html='<html><meta charset="utf-8"><h1>JOIN-SOURCE-03 paired comparisons</h1><p>Predictions, not execution. No safe ON output obtained.</p>'
    for f in figures:html+=f'<h2>{f["png"]}</h2><img style="max-width:100%" src="{f["png"]}"><p><a href="{f["sidecar"]}">numeric/source sidecar</a></p>'
    (out/'index.html').write_text(html+'</html>')
    (run/'index.html').write_text('<html><meta charset="utf-8"><h1>JOIN-SOURCE-03</h1><p>LIGHTING_CONTRIBUTES_BUT_NOT_SUFFICIENT — no safe FRESH recovered.</p><ul><li><a href="review_v2/index.html">Actual DARK/BRIGHT RGB and all seven conditions</a></li><li><a href="review_comparisons/index.html">Lighting / H16–H32 / distance comparisons</a></li><li><a href="validation_v2.json">Authoritative saved-record validation</a></li></ul></html>')
    with zipfile.ZipFile(run/'review_bundle_final.zip','x',zipfile.ZIP_DEFLATED) as z:
        for folder in ['review_v2','review_comparisons']:
            for f in sorted((run/folder).iterdir()):z.write(f,str(f.relative_to(run)))
        for name in ['index.html','protocol.json','config_snapshot.yaml','mpc_audit/result.json','aggregate/summary.json',
                     'aggregate/outcomes.csv','aggregate/row_geometry.csv','validation_v2.json']:
            z.write(run/name,name)


if __name__=='__main__':main()
