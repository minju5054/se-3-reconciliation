#!/usr/bin/env python3
"""Static saved-only paired presentation with numeric/hash sidecars."""
import argparse,csv,html,json,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from reconciliation.join_source03 import read,save,sha
from run_osa03_trackability import validate
from run_osa03_native_continuation import plain
from run_join_online02 import environments
from analyze_join_online02 import csvread,pose_rows
from validate_robotless_online_handoffs import equal_record


def data(run):
    manifest=read(run/'source_manifest.json');r=read(run/'rollout.json');m=read(run/'metrics.json');base=Path(manifest['comparator']);b=read(base/'rollout.json');bm=read(base/'metrics.json')
    _,_,cart,_=environments(Path(manifest['source']))
    curves={}
    for key,rr,mm,start in [('observation_start',r,m['origins']['execution'],m['origins'].get('first_FRESH_index')),('saved_B_start',b,bm,0)]:
        if mm is None:continue
        curves[key]=dict(pose=[s['pose_world'] for s in rr['states'][start:]],metrics=mm,
            selections=[e for e in rr['events'] if e.get('type')=='solve_result' and e.get('status')=='command'])
    prefix=pose_rows(csvread(Path(manifest['source'])/'episodes/REPEAT_00/execution.csv'))[:b['phase']['B_tick']+1]
    return plain(dict(curves=curves,FRESH=r['fresh_world'],observation_pose=r['phase']['obs_pose'],B=b['phase']['B'],cart_wkb_hex=cart.wkb_hex,
        OLD_execution=prefix,availability_states=r['states'],availability_delay_s=m['origins'].get('availability_to_application_s'),paired=m['paired'],
        common=m['common_execution_exposure'],spatial=read(run/'spatial_descriptors.json')))


def write_csv(path,rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def report(run):
    v=validate(run);d=data(run);out=run/'review';out.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from shapely import wkb
    sources=[run/n for n in ['rollout.json','metrics.json','source_manifest.json','protocol.yaml','spatial_descriptors.json']]
    sources += [Path(read(run/'source_manifest.json')['comparator'])/n for n in ['rollout.json','metrics.json']]
    inputs={str(p):sha(p) for p in sources};manifest=[]
    def finish(fig,name):
        fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=160,bbox_inches='tight');plt.close(fig)
        save(out/(name+'.json'),dict(data=d,inputs=inputs,processing_sha256=sha(__file__)))
        manifest.append(dict(name=name,png_sha256=sha(out/(name+'.png')),sidecar_sha256=sha(out/(name+'.json'))))
    colors={'observation_start':'#078066','saved_B_start':'#d37121'}
    labels={'observation_start':'Observation-start: new Native execution','saved_B_start':'B-start: frozen saved Native execution'}
    fig,ax=plt.subplots(figsize=(10,7));f=np.asarray(d['FRESH']);old=np.asarray(d['OLD_execution']);cart=wkb.loads(d['cart_wkb_hex'],hex=True)
    for g in ([cart] if not hasattr(cart,'geoms') else cart.geoms):ax.fill(*g.exterior.xy,color='#80633f',alpha=.6)
    buffer=cart.buffer(.05)
    for i,g in enumerate([buffer] if not hasattr(buffer,'geoms') else buffer.geoms):ax.plot(*g.exterior.xy,'--',color='#80633f',label='Cart + .05 m edge margin' if i==0 else None)
    ax.plot(old[:,0],old[:,1],color='gray',label='Recorded OLD to delayed B')
    ax.plot(f[:,0],f[:,1],'o--',color='#ae3882',ms=3,label='Same original LightNav FRESH (raw rows)')
    for key,c in d['curves'].items():
        p=np.asarray(c['pose']);mm=c['metrics'];color=colors[key]
        ax.plot(p[:,0],p[:,1],color=color,lw=2,label=labels[key])
        q=mm['clearance']['minimum_pose'];ax.add_patch(Circle(q[:2],.2,fill=False,color=color,alpha=.6));ax.scatter(*q[:2],marker='x',color=color)
        if mm['full']['join_time_s'] is not None:
            i=int(np.searchsorted(mm['trace']['times_s'],mm['full']['join_time_s']));ax.scatter(*p[i,:2],marker='D',color=color,s=55)
    availability=np.asarray([s['pose_world'] for s in d['availability_states']]);ax.plot(availability[:,0],availability[:,1],':',color=colors['observation_start'],alpha=.5)
    ax.scatter(*d['observation_pose'][:2],marker='o',color='black',label='Original observation state')
    ax.scatter(*d['B'][:2],marker='*',s=100,color='black',label='Actual delayed B')
    ax.scatter(*f[-1,:2],marker='s',color='#ae3882',label='FRESH endpoint, not completed bypass')
    ax.set(aspect='equal',xlabel='World X [m]',ylabel='World Y [m]',title='Same FRESH, unchanged Native MPC | two starting states\nCircles: .20 m physical footprint at minimum clearance; diamonds: sustained attachment')
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);ax.grid(alpha=.2);finish(fig,'world')
    fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for key,c in d['curves'].items():
        mm=c['metrics'];t=mm['trace']['times_s']
        for ax,field,scale in zip(axs,['distance_m','yaw_error_rad'],[1,180/np.pi]):
            ax.plot(t,np.asarray(mm['trace'][field])*scale,color=colors[key],label=labels[key])
            if mm['full']['join_time_s'] is not None:ax.axvline(mm['full']['join_time_s'],ls=':',color=colors[key])
    axs[0].axhline(.1,color='gray',ls='--');axs[1].axhline(15,color='gray',ls='--');axs[0].set_ylabel('Position distance [m]');axs[1].set_ylabel('Wrapped yaw error [deg]');axs[1].set_xlabel('Seconds after FIRST FRESH command application')
    axs[0].legend(fontsize=8);fig.suptitle(f"Execution-origin comparison | observation availability wait {d['availability_delay_s']:.6f} s excluded")
    for ax in axs:ax.grid(alpha=.2)
    finish(fig,'errors')
    fig,axs=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for key,c in d['curves'].items():
        mm=c['metrics'];u=np.asarray(mm['command']['commands']);t=mm['command']['command_start_times_s']
        for j,ax in enumerate(axs):ax.step(t,u[:,j],where='post',color=colors[key],label=labels[key])
    axs[0].set_ylabel('v [m/s]');axs[1].set_ylabel('omega [rad/s]');axs[1].set_xlabel('Seconds after first FRESH command');axs[0].legend(fontsize=8)
    fig.suptitle('Actual held commands | unchanged official MPC, no new VLA')
    for ax in axs:ax.grid(alpha=.2)
    finish(fig,'commands')
    fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for key,c in d['curves'].items():
        mm=c['metrics'];tr=mm['trace'];axs[0].plot(tr['times_s'],[p['progress'] for p in tr['projection']],color=colors[key],label=labels[key])
        axs[1].plot(tr['dense_times_s'],tr['dense_clearance_m'],color=colors[key])
    axs[0].set_ylabel('Original fractional row progress');axs[0].legend(fontsize=8);axs[1].axhline(.05,ls='--',color='red',label='Required footprint-edge margin .05 m')
    axs[1].set_ylabel('Footprint-edge clearance [m]');axs[1].set_xlabel('Seconds after first FRESH command');axs[1].legend()
    fig.suptitle('Forward-only evaluation progress and direct geometry clearance')
    for ax in axs:ax.grid(alpha=.2)
    finish(fig,'progress_clearance')
    fig,axs=plt.subplots(1,3,figsize=(12,4));chosen=['first09_position_AUC_m_s','attachment_time_s','minimum_execution_clearance_lower_bound_m']
    for ax,k in zip(axs,chosen):
        row=next(r for r in d['paired'] if r['metric']==k);values=[row['observation_start'],row['B_start']]
        for i,x in enumerate(values):
            if x is not None:ax.bar(i,x,color=list(colors.values())[i]);ax.text(i,x,f'{x:.6f}',ha='center',va='bottom',fontsize=9)
        ax.set_xticks([0,1],['Observation','Saved B']);ax.set_title(k.replace('_',' '),fontsize=9);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Signed descriptive contrasts; no additive latency decomposition');finish(fig,'paired')
    write_csv(out/'paired.csv',d['paired'])
    rows=[]
    for key,c in d['curves'].items():
        for n,w in c['metrics']['windows'].items():rows.append(dict(condition=key,window_steps=n,**{k:w.get(k) for k in ['available','observation_horizon_s','mean_distance_time_weighted_m','max_distance_m','position_auc_m_s','position_excess_auc_m_s','yaw_auc_rad_s','yaw_excess_auc_rad_s']}))
    write_csv(out/'windows.csv',rows)
    save(out/'manifest.json',dict(figures=manifest,inputs=inputs,validation=v))
    body='<h1>OSA03 same-FRESH trackability control</h1><p>OFFLINE counterfactual. New observation-start versus saved delayed B-start. No new VLA or optimization. Three seconds is an availability observation cap, not FRESH row timing. Circles show physical radius .20 m; dashed cart outline is only the .05 m edge margin.</p>'
    for item in manifest:body+=f'<h2>{item["name"]}</h2><img style="max-width:100%" src="{item["name"]}.png"><p><a href="{item["name"]}.json">Numeric/hash sidecar</a></p>'
    body+='<p><a href="paired.csv">Signed paired table</a> | <a href="windows.csv">Window metrics</a></p>'
    with (out/'index.html').open('x') as fp:fp.write('<!doctype html><meta charset="utf-8"><title>OSA03 Trackability</title>'+body)
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in out.iterdir():z.write(p,'review/'+p.name)
    validate_figures(run)


def validate_figures(run):
    out=run/'review';m=read(out/'manifest.json');d=data(run)
    for p,h in m['inputs'].items():assert sha(p)==h
    for item in m['figures']:
        name=item['name'];assert sha(out/(name+'.png'))==item['png_sha256'];assert sha(out/(name+'.json'))==item['sidecar_sha256']
        s=read(out/(name+'.json'));equal_record(s['data'],d);assert s['inputs']==m['inputs'] and s['processing_sha256']==sha(__file__)
    with (out/'paired.csv').open() as fp:rows=list(csv.DictReader(fp))
    assert len(rows)==len(d['paired'])
    for a,b in zip(rows,d['paired']):
        for k,v in b.items():assert a[k]==('' if v is None else str(v))
    return dict(valid=True,numeric_plot_table_parity=True,source_hashes=True,validation_model_MPC_calls=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
