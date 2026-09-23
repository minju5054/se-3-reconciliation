#!/usr/bin/env python3
"""Static overlays of ORIGINAL saved RGB; no new scientific render/input."""
import argparse
import html
import math
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image

from audit_join_online03_turnback import read, write_new, sha, table_rows, EPISODES, environments


def presentation(out):
    result = read(out/'records.json'); summary = read(out/'summary.json')
    folder = out/'plots'; folder.mkdir(exist_ok=False)
    sources = {str(out/p): sha(out/p) for p in ['records.json', 'summary.json', 'source_manifest.json', 'protocol.json']}
    entries = []
    def finish(fig, name, numbers):
        png = folder/(name+'.png'); fig.savefig(png, dpi=160); plt.close(fig)
        write_new(folder/(name+'.json'), dict(data=numbers, source_hashes=sources,
            source_manifest_sha256=sha(out/'source_manifest.json'), image_sha256=sha(png),
            scope='SAVED OBSERVATIONAL AUDIT; overlays never used as model input'))
        entries.append(dict(name=name, png_sha256=sha(png), sidecar_sha256=sha(folder/(name+'.json'))))
    def rgb(ax, frame, pointing=None):
        ax.imshow(Image.open(frame['RGB_path']))
        m = np.load(frame['mask_path'])['mask']; selected = np.isin(m, frame['cart_ids'])
        if selected.any():
            ax.contour(selected.astype(float), [.5], colors=['yellow'], linewidths=.7)
            x0,y0,x1,y1 = frame['bbox_xyxy_inclusive']
            ax.add_patch(Rectangle((x0,y0),x1-x0+1,y1-y0+1,fill=False,edgecolor='yellow',ls=':',lw=1))
        if pointing:
            for key,c,marker in [('apos','cyan','o'),('opos','#ff3377','x')]:
                p = pointing[key]
                if p['pixel'] is not None:
                    ax.scatter(*p['pixel'],c=c,marker=marker,s=50,linewidths=1.8)
        ax.set_xticks([]); ax.set_yticks([])
    rows = result['rows']; frames = {(f['episode'],f['frame_id']): f for f in result['frames']}
    chosen = [r for r in rows if int(r['chunk'][-3:]) >= 3]
    fig,axes = plt.subplots(2,4,figsize=(18,8),layout='constrained')
    for ax,r in zip(axes.flat,chosen):
        rgb(ax,r['current'],r['pointing'])
        ax.set_title(f"{r['episode']} C{int(r['chunk'][-3:])}\ncart={r['current']['visible_pixels']:,} px; edge={r['original_geometry']['minimum_clearance_m']:.4f} m",fontsize=10)
        ax.set_xlabel(f"APOS: {r['pointing']['apos']['raster_class']} | OPOS: {r['pointing']['opos']['raster_class']}\nclamped A/O: {r['pointing']['apos']['clamped']}/{r['pointing']['opos']['clamped']}",fontsize=9)
    fig.suptitle('Original current RGB | yellow=visible cart / bbox, cyan=APOS, pink=OPOS\nRaster labels are observable evidence, not neural recognition',fontsize=13)
    finish(fig,'current_RGB_sequence',chosen)
    for r in rows:
        if r['chunk'] not in ('chunk_004','chunk_005'): continue
        hs = r['sampled_history']; cols=4
        fig,axes = plt.subplots(math.ceil(len(hs)/cols),cols,figsize=(16,2.85*math.ceil(len(hs)/cols)),layout='constrained')
        for ax,s in zip(axes.flat,hs):
            frame = frames[(r['episode'],s['frame_id'])];rgb(ax,frame)
            ax.set_title(f"slot {s['slot']} / {s['frame_id']}\nage {s['age_sim_s']:.2f} s; cart {s['visible_pixels']:,} px; pool{s['pool_spatial']}",fontsize=9)
        for ax in axes.flat[len(hs):]: ax.axis('off')
        fig.suptitle(f"{r['episode']} / {r['chunk']}: wire-derived history + pinned sampler reconstruction\nSaved original RGB, chronological slots; internal selection telemetry unavailable",fontsize=12)
        finish(fig,r['episode']+'_'+r['chunk']+'_history',hs)
    run=Path(read(out/'source_manifest.json')['primary_run']);_,_,cart,_=environments(run)
    fig,axes=plt.subplots(2,2,figsize=(14,11),layout='constrained',gridspec_kw={'width_ratios':[1,1.5]})
    for ei,eid in enumerate(EPISODES):
        ax,txt=axes[ei]; rs=[r for r in chosen if r['episode']==eid]
        for shape in getattr(cart,'geoms',[cart]):
            if hasattr(shape,'exterior'):
                xy=np.asarray(shape.exterior.coords);ax.fill(*xy.T,color='saddlebrown',alpha=.7)
        for shape in getattr(cart.buffer(.25),'geoms',[cart.buffer(.25)]):
            if hasattr(shape,'exterior'):
                xy=np.asarray(shape.exterior.coords);ax.plot(*xy.T,':',c='brown')
        for r,c in zip(rs,['#16864d','#1c78b2','#e3870a','#cc3377']):
            p=np.asarray(r['world']);ax.plot(*p[:,:2].T,'.-',c=c,label=f"C{int(r['chunk'][-3:])}")
            ax.scatter(*r['observation_pose'][:2],marker='x',c=c)
        ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.2);ax.legend()
        ax.set_title(eid+' | RAW FRESH, not execution')
        txt.axis('off')
        notes=[]
        for r in rs:
            h=r['history']['unique'];g=r['trajectory']
            notes.append(f"C{int(r['chunk'][-3:])}: current {r['current']['visible_pixels']:,} px\n"
                f"history {h['cart_visible_count']}/{h['count']} visible, {h['total_cart_pixels']:,} px sum\n"
                f"yaw={np.degrees(g['final_yaw_hallway_rad']):+.2f}°, edge={r['original_geometry']['minimum_clearance_m']:.4f} m\n"
                f"inward interior segments={g['inward_interior_segments']}")
        txt.text(0,.97,'\n\n'.join(notes),va='top',fontsize=11)
    # Common limits across the repeat views.
    xlim=[min(ax.get_xlim()[0] for ax in axes[:,0]),max(ax.get_xlim()[1] for ax in axes[:,0])]
    ylim=[min(ax.get_ylim()[0] for ax in axes[:,0]),max(ax.get_ylim()[1] for ax in axes[:,0])]
    for ax in axes[:,0]:ax.set_xlim(xlim);ax.set_ylim(ylim)
    fig.suptitle('C3 → C6: world geometry and available visual evidence\nDotted cart contour includes .20 m radius + .05 m required clearance',fontsize=13)
    finish(fig,'world_and_visual_evidence',chosen)
    fig,axes=plt.subplots(5,2,figsize=(13,16),layout='constrained')
    for col,eid in enumerate(EPISODES):
        rs=[r for r in rows if r['episode']==eid];x=np.arange(7)
        series=[([r['current']['visible_pixels'] for r in rs],'Current cart pixels'),
                ([r['history']['unique']['visible_fraction'] for r in rs],'Sampled visible frame fraction'),
                ([r['prior_history']['unique']['most_recent_visible_age_s'] for r in rs],'Latest preceding visible age (s)'),
                ([r['trajectory']['endpoint_lateral_m'] for r in rs],'Signed hallway lateral (m)'),
                ([r['original_geometry']['minimum_clearance_m'] for r in rs],'Minimum footprint-edge clearance (m)')]
        for row,(y,label) in enumerate(series):
            ax=axes[row,col];ax.plot(x,y,'o-',label='endpoint' if row==3 else None)
            if row==1:ax.set_ylim(0,1.1)
            if row==3:
                ax.plot(x,[r['trajectory']['min_signed_lateral_m'] for r in rs],'s--',label='minimum across full path');ax.legend(fontsize=8)
            if row==4:ax.axhline(.05,c='red',ls=':',label='required .05 m');ax.axhline(0,c='gray',lw=.6);ax.legend(fontsize=8)
            ax.axvspan(4,5,alpha=.09,color='orange');ax.set_xticks(x,[f'C{i}' for i in x]);ax.set_ylabel(label);ax.grid(alpha=.2)
            if row==0:ax.set_title(eid)
    fig.suptitle('All C0–C6 records; shaded C4→C5 primary transition\nHistory visibility includes current; preceding age excludes it. Pixel counts do not measure attention.',fontsize=13)
    finish(fig,'sequence_metrics',rows)
    write_new(folder/'manifest.json',dict(figures=entries,source_hashes=sources))
    table=table_rows(result)['chunk_visibility']
    keys=['episode','chunk','current_pixels','sampled_frames','visible_sampled_frames','history_pixels','preceding_latest_visible_age_s','min_edge_m','safe']
    text='<meta charset="utf-8"><style>body{font:16px sans-serif;margin:28px}img{max-width:100%}td,th{border:1px solid #aaa;padding:5px}table{border-collapse:collapse}</style>'
    text+='<h1>JOIN-ONLINE-03 turn-back visibility audit</h1><p>SAVED OBSERVATIONAL AUDIT. New model/MPC/GP/rollout calls: 0.</p>'
    text+='<h2>'+summary['interpretation']+'</h2><p>Available cart pixels increase during turn-back. This does not prove recognition, attention, memory loss or a causal mechanism. Internal sampler IDs are source-reconstructed, not server telemetry. Source image pixels precede model resize/spatial pooling.</p>'
    text+='<table><tr>'+''.join('<th>'+k+'</th>' for k in keys)+'</tr>'
    for row in table:text+='<tr>'+''.join('<td>'+html.escape(str(row[k]))+'</td>' for k in keys)+'</tr>'
    text+='</table>'
    for e in entries:text+=f'<h2>{e["name"]}</h2><a href="plots/{e["name"]}.json">numeric/source sidecar</a><br><img src="plots/{e["name"]}.png">'
    with (out/'index.html').open('x') as f:f.write(text)
    with zipfile.ZipFile(out/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [out/'index.html',out/'protocol.json',out/'summary.json',out/'source_manifest.json',
                  out/'turnback_metrics.json',*out.glob('*.csv'),*sorted(folder.iterdir())]:z.write(p,p.relative_to(out))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    presentation(p.parse_args().output.resolve())
