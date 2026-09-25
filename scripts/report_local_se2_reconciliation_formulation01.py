#!/usr/bin/env python3
"""Static planning review from immutable saved arrays; no optimizer call."""
import argparse
import csv
import html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from run_local_se2_reconciliation_formulation01 import read,save,sha,load_source

LABEL='PLANNING ONLY | NO MPC EXECUTION\nNO RECONCILIATION PERFORMANCE CLAIM'


def draw_shape(ax, geom, **kwargs):
    if hasattr(geom,'geoms'):
        for part in geom.geoms:draw_shape(ax,part,**kwargs)
    elif geom.geom_type=='Polygon':
        xy=np.asarray(geom.exterior.coords); ax.fill(xy[:,0],xy[:,1],**kwargs)
    elif hasattr(geom,'coords'):
        xy=np.asarray(geom.coords)
        if len(xy):ax.plot(xy[:,0],xy[:,1],color=kwargs.get('color','grey'),lw=.8)


def report(run):
    result=read(run/'result.json'); trace=read(run/'solver_trace.json'); loaded=load_source(read(run/'protocol.json'))
    out=run/'review'; out.mkdir(exist_ok=False)
    arrays={name:np.load(run/'derived'/f'{name}.npy') for name in ['original_world','transported_target','optimized_world','OLD_executed_prefix']}
    f,t,x=arrays['original_world'],arrays['transported_target'],arrays['optimized_world']
    nodes=result['diagnostics']['nodes']; s=np.array([n['s'] for n in nodes])
    from shapely.geometry import box,mapping
    extent=np.vstack([f,t,x,loaded['old_prefix']]); lo=extent[:,:2].min(0)-.45; hi=extent[:,:2].max(0)+.45
    cart=loaded['cart']; cb=cart.bounds; lo=np.minimum(lo,np.array(cb[:2])-.35); hi=np.maximum(hi,np.array(cb[2:])+.35)
    cropped=loaded['base'].obstacles.intersection(box(*lo,*hi))
    source_hashes={str(q):sha(q) for q in [run/'source_manifest.json',run/'result.json',run/'solver_trace.json',run/'protocol.json',*sorted((run/'derived').glob('*.npy'))]}
    figures=[]
    def finish(fig,name,caption):
        fig.suptitle(LABEL,fontsize=10)
        fig.tight_layout(rect=(0,0,1,.93));fig.savefig(out/f'{name}.png',dpi=170);plt.close(fig)
        save(out/f'{name}.json',dict(scope=result['scope'],caption=caption,source_hashes=source_hashes,
             result=result,trace=trace,arrays={k:v.tolist() for k,v in arrays.items()},
             cart_geometry=mapping(cart),Hospital_plot_geometry=mapping(cropped)))
        figures.append(dict(name=name,caption=caption))
    fig,ax=plt.subplots(figsize=(8,8))
    draw_shape(ax,cropped,color='#b8bec5',alpha=.7)
    draw_shape(ax,cart.buffer(.25),color='#eeaaaa',alpha=.3)
    draw_shape(ax,cart.buffer(.05),color='#cc6666',alpha=.25)
    draw_shape(ax,cart,color='#704f40',alpha=.85)
    ax.plot(*arrays['OLD_executed_prefix'][:,:2].T,color='#888888',lw=2,label='Saved OLD execution (historical)')
    for v,color,style,label in [(f,'#158d9d','--','Original FRESH'),(t,'#df9730',':','Fully transported target'),(x,'#743da8','-','Optimized X (planned)')]:
        ax.plot(*v[:,:2].T,style,color=color,marker='o',ms=3,lw=1.8,label=label)
        ax.quiver(v[:,0],v[:,1],.06*np.cos(v[:,2]),.06*np.sin(v[:,2]),color=color,angles='xy',scale_units='xy',scale=1,width=.003)
    for pose,label,marker in [(result['A'],'A: observation','s'),(result['B'],'B: application','*')]:
        ax.scatter(*pose[:2],s=70,marker=marker,color='black');ax.annotate(label,pose[:2],xytext=(-120,8),textcoords='offset points',fontsize=8)
    # Depicted disks distinguish footprint from obstacle edge-margin buffer.
    from matplotlib.patches import Circle,Patch
    ax.add_patch(Circle(x[-1,:2],.2,fill=False,color='#743da8',lw=1,ls='--'))
    handles,labels=ax.get_legend_handles_labels()
    handles += [Patch(color='#704f40',label='Cart mesh footprint'),Patch(color='#eeaaaa',alpha=.4,label='Cart + radius .20 + margin .05 (center exclusion)')]
    ax.legend(handles=handles,fontsize=7,loc='upper left')
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
    finish(fig,'figure_1_world','Original FRESH, transported target and optimized spatial reference. Arrows are pose headings. Endpoint circle is the .20 m footprint; there is no B-to-X0 connector.')
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True)
    for prefix,label,col in [('correction','Original FRESH → X','#743da8'),('target','Transported target → X','#df9730')]:
        axes[0].plot(s,[n[prefix+'_translation_m'] for n in nodes],'-o',label=label,color=col)
        axes[1].plot(s,np.rad2deg([n[prefix+'_yaw_rad'] for n in nodes]),'-o',label=label,color=col)
    axes[0].set_ylabel('SE(2) Log translation norm [m]');axes[1].set_ylabel('|Log yaw| [deg]');axes[1].set_xlabel('Original XY arc progress s')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    finish(fig,'figure_2_corrections','Node corrections are norms of exact SE(2) Log residuals, not row timestamps.')
    fig,ax=plt.subplots(figsize=(8,4));ax.plot(s,(1-s)**2,'-o',label='w_L=(1-s)²');ax.plot(s,s*s,'-o',label='w_A=s²')
    ax.set(xlabel='Original XY arc progress s',ylabel='Frozen factor weight');ax.legend();ax.grid(alpha=.2)
    finish(fig,'figure_3_weights','Fixed symmetric schedules, each factor block normalized by its own weight sum.')
    accepted=[e for e in trace if e['decision'] in ['initial','accepted']]
    fig,ax=plt.subplots(figsize=(8,4))
    for k in ['L','R','A','total']:ax.plot([e['iteration'] for e in accepted],[e['factor_costs'][k] for e in accepted],'-o',ms=3,label=k)
    ax.set(xlabel='LM iteration (initial and accepted states)',ylabel='Normalized squared residual cost');ax.grid(alpha=.2);ax.legend()
    finish(fig,'figure_4_costs','Factor costs at initial and accepted LM states; rejected steps retained in solver_trace.json.')
    keys=[k for k,v in nodes[0].items() if not isinstance(v,list)]
    with (out/'nodes.csv').open('x') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows({k:n[k] for k in keys} for n in nodes)
    with (out/'factor_costs.csv').open('x') as stream:
        writer=csv.DictWriter(stream,fieldnames=['iteration','L','R','A','total']);writer.writeheader()
        writer.writerows(dict(iteration=e['iteration'],**e['factor_costs']) for e in accepted)
    content='<!doctype html><meta charset="utf-8"><title>Local SE2 formulation 01</title><style>body{max-width:1100px;margin:30px auto;font:16px sans-serif;color:#203040}img{max-width:100%}pre{white-space:pre-wrap}</style>'
    content+='<h1>LOCAL SE(2) — OSA03 REPEAT_00</h1><p><b>'+LABEL.replace('\n','<br>')+'</b></p>'
    content+='<p>Spatial deformation of one sealed FRESH. No new robot execution, no baseline comparison. Cart circle inflation is for centerline exclusion; it is not applied twice to the robot footprint.</p>'
    for item in figures:
        content+=f'<h2>{html.escape(item["caption"])}</h2><img src="{item["name"]}.png"><p><a href="{item["name"]}.json">Numeric and source-hash sidecar</a></p>'
    content+='<p><a href="nodes.csv">Per-node CSV</a> · <a href="factor_costs.csv">Factor costs CSV</a></p>'
    with (out/'index.html').open('x') as stream:stream.write(content)
    save(out/'manifest.json',dict(files={str(q.relative_to(out)):sha(q) for q in sorted(out.iterdir()) if q.is_file()},source_hashes=source_hashes))
    print(out/'index.html')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();report(args.run.resolve())
