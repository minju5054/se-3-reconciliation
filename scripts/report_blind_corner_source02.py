#!/usr/bin/env python3
"""Static source/geometry review with numeric/hash sidecars, no new inference."""
import argparse
from pathlib import Path
import sys
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import LineString,box
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_geometry import projection_from_metadata
from report_blind_corner_source import report as scientific_report,geometries


def report(run):
    v=read(run/'validation.json');scientific_report(run);out=run/'review';panels=[]
    cfg=read(run/'declaration.json')['config'];base=HospitalEnvironment.load(ROOT/cfg['environment'])
    fig,axes=plt.subplots(1,3,figsize=(17,6))
    side=dict(rows=v['geometry']['rows'],validation_sha256=sha(run/'validation.json'),declaration_sha256=sha(run/'declaration.json'),
              source_hashes=read(run/'freeze.json')['input_sha256'],scope='MODEL-FREE GEOMETRY, NOT EXECUTION')
    for ax,r in zip(axes,v['geometry']['rows']):
        c=r['candidate'];cart=projection_from_metadata(r['projection'])['obstacle_geometry']
        center=np.asarray(c['corner_xy']);window=box(center[0]-2,center[1]-1.5,center[0]+2.5,center[1]+2.7)
        geometries(ax,base.obstacles.intersection(window),color='gray',alpha=.6)
        geometries(ax,cart,color='orange',alpha=.8)
        geometries(ax,cart.buffer(.25),color='orange',alpha=.15)
        for p in c['nominal_paths']:
            a=np.array(p['poses']);ax.plot(a[:,0],a[:,1],'--',lw=1,label=f'Nominal r={p["radius_m"]:.2f} m')
            geometries(ax,LineString(a[:,:2]).buffer(.25),color='royalblue',alpha=.05)
        a=np.array(c['bypass_probe']);ax.plot(a[:,0],a[:,1],color='green',lw=2,label='Free-space probe (not executed)')
        a=np.array(c['probe_poses']);i=r['expected_first_visible_probe']
        ax.scatter(*a[0,:2],c='black',marker='x',label='Initial viewpoint')
        if i is not None:ax.scatter(*a[i,:2],c='purple',marker='*',s=80,label='First visible diagnostic probe')
        ax.set(xlim=(center[0]-2,center[0]+2.5),ylim=(center[1]-1.5,center[1]+2.7),xlabel='World X [m]',ylabel='World Y [m]',
            title=f'{c["id"]}: geometry {"PASS" if r["qualified"] else "FAIL"}\ncombined bypass {r["probes"]["bypass"]["combined_m"]:.4f} m')
        ax.set_aspect('equal');ax.legend(fontsize=6,loc='upper left')
    fig.tight_layout();name='geometry_bank';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig);save(out/(name+'.json'),side);panels.append(name)
    for r in v['geometry']['rows']:
        cid=r['candidate_id'];folder=run/'technical_preflight'/cid;i=r['expected_first_visible_probe']
        ids=[0,i if i is not None else len(r['pixels'])-1];fig,axes=plt.subplots(1,2,figsize=(11,4));inputs=[]
        for ax,j in zip(axes,ids):
            rec=read(folder/f'probe_{j:02}.json');ax.imshow(Image.open(folder/rec['rgb_file']));ax.axis('off')
            ax.set_title(f'{cid} diagnostic pose {j}: {r["pixels"][j]} cart pixels\nSpatial probe, not online execution');inputs.append(rec)
        fig.tight_layout();name=cid+'_preflight_RGB';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig)
        save(out/(name+'.json'),dict(records=inputs,geometry_row=r,source_hashes={str(folder/x['rgb_file']):x['rgb_jpeg_sha256'] for x in inputs}));panels.append(name)
    for r in v['rows']:
        fig,ax=plt.subplots(figsize=(8,5));data=[]
        for label in ['OLD','FRESH']:
            c=r['clearance_attribution'][label]
            if c:
                for kind in ['Hospital_only_m','cart_only_m','combined_m']:data.append((label+' '+kind,c[kind]))
        if data:ax.bar(np.arange(len(data)),[x[1] for x in data]);ax.set_xticks(np.arange(len(data)),[x[0] for x in data],rotation=30,ha='right')
        else:ax.text(.5,.5,'No returned path available',ha='center')
        ax.axhline(.05,color='red',ls='--',label='Required edge margin .05 m');ax.set(ylabel='Whole raw path footprint-edge clearance [m]',title=r['candidate_id']+' actual raw-path attribution');ax.legend()
        fig.tight_layout();name=r['candidate_id']+'_clearance_attribution';fig.savefig(out/(name+'.png'),dpi=160);plt.close(fig)
        save(out/(name+'.json'),dict(values=data,attribution=r['clearance_attribution'],source_hashes=r['raw_source_hashes'],validation_sha256=sha(run/'validation.json')));panels.append(name)
    html=(out/'index.html').read_text();html+=''.join(f'<h2>{n}</h2><a href="{n}.json">Numeric/hash sidecar</a><br><img style="max-width:100%" src="{n}.png">' for n in panels)
    (out/'index.html').write_text(html)
    assert read(out/'geometry_bank.json')==side
    for r in v['rows']:
        assert read(out/(r['candidate_id']+'_clearance_attribution.json'))['attribution']==r['clearance_attribution']
    save(run/'artifact_validation02.json',dict(valid=True,numeric_sidecar_parity=True,figures={p.name:sha(p) for p in out.glob('*.png')},model_MPC_calls=0))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
