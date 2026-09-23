#!/usr/bin/env python3
"""Saved-only Native continuation figures/tables, each with exact numeric hashes."""
import argparse,csv,html,json,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from reconciliation.join_source03 import read,save,sha
from run_osa03_native_continuation import plain
from validate_osa03_native_continuation import validate
from run_join_online02 import environments
from analyze_join_online02 import csvread,pose_rows
from validate_robotless_online_handoffs import equal_record


def plot_data(run):
    r=read(run/'rollout.json');m=read(run/'metrics.json');source=Path(read(run/'source_manifest.json')['source'])
    _,_,cart,_=environments(source);p=np.array([s['pose_world'] for s in r['states']]);t=np.array([s['time_s'] for s in r['states']]);f=np.array(r['fresh_world']);tr=m['trace']
    oldprefix=pose_rows(csvread(source/'episodes/REPEAT_00/execution.csv'))[:r['phase']['B_tick']+1]
    selections=[dict(origin='SAVED_ACCEPTED_RESULT',time_s=(a['submit_tick']-r['phase']['B_tick'])*r['phase']['integration_dt_s'],result=a['result']) for a in r['phase']['applications']]
    selections += [dict(origin='NEW_NATIVE_SOLVE',time_s=(e['input_state_id']-r['phase']['B_tick'])*r['phase']['integration_dt_s'],result=e) for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command']
    return plain(dict(world=dict(OLD_execution_prefix=oldprefix,FRESH=f,Native=p,times_s=t,B=r['phase']['B'],cart_wkb_hex=cart.wkb_hex,
            join_time_s=m['full']['join_time_s'],minimum_clearance_pose=m['clearance']['minimum_pose'],endpoint=f[-1],prefix_steps=r['phase']['prefix_steps']),
        attachment=dict(times_s=t,distance_m=tr['distance_m'],yaw_error_rad=tr['yaw_error_rad'],yaw_world_rad=p[:,2],lateral_m=tr['lateral_m'],projection=tr['projection'],join_time_s=m['full']['join_time_s']),
        commands=dict(times_s=m['command']['command_start_times_s'],commands=m['command']['commands'],u_minus=r['phase']['u_minus'],u_B_plus=r['phase']['u_B_plus'],u_mem_B=r['phase']['u_mem_B'],application_rates=m['command']['application_intervals']),
        progress=dict(times_s=t,arc_progress_m=tr['arc_progress_m'],remaining_arc_m=tr['remaining_arc_m'],unrestricted_progress=tr['unrestricted_progress'],fractional_row_progress=[x['progress'] for x in tr['projection']]),
        clearance=dict(times_s=tr['dense_times_s'],clearance_m=tr['dense_clearance_m'],raw_FRESH_minimum_m=m['clearance']['raw_FRESH']['minimum_clearance_m'],required_m=.05,minimum_time_s=m['clearance']['minimum_time_s']),
        selections=dict(rows=selections)))


def write_csv(path,rows):
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def report(run):
    v=validate(run);m=read(run/'metrics.json');r=read(run/'rollout.json');data=plot_data(run)
    out=run/'review';out.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from shapely import wkb
    inputs=[run/'rollout.json',run/'metrics.json',run/'source_manifest.json',run/'protocol.yaml']
    manifest=[]
    def finish(fig,key):
        fig.tight_layout();fig.savefig(out/(key+'.png'),dpi=160,bbox_inches='tight');plt.close(fig)
        save(out/(key+'.json'),dict(data=data[key],inputs={str(p):sha(p) for p in inputs},processing_sha256=sha(__file__)))
        manifest.append(dict(name=key,png_sha256=sha(out/(key+'.png')),sidecar_sha256=sha(out/(key+'.json'))))
    d=data['world'];f=np.array(d['FRESH']);p=np.array(d['Native']);old=np.array(d['OLD_execution_prefix']);cart=wkb.loads(d['cart_wkb_hex'],hex=True)
    fig,ax=plt.subplots(figsize=(10,7))
    def polygons(g):return list(g.geoms) if hasattr(g,'geoms') else [g]
    for i,q in enumerate(polygons(cart)):
        ax.fill(*q.exterior.xy,color='#88633d',alpha=.6,label='Cart mesh footprint' if i==0 else None)
    for i,q in enumerate(polygons(cart.buffer(.25))):
        ax.plot(*q.exterior.xy,':',color='#88633d',label='Nominal radius .20 + margin .05 m' if i==0 else None)
    ax.plot(old[:,0],old[:,1],color='gray',lw=2,label='Recorded OLD execution to B')
    ax.plot(f[:,0],f[:,1],'o--',ms=4,color='#bd3091',label='Original LightNav FRESH (raw rows)')
    ax.plot(p[:,0],p[:,1],color='#007d92',lw=2.8,label='Native continuation (frozen FRESH)')
    n=d['prefix_steps'];ax.plot(p[:n+1,0],p[:n+1,1],color='#e08d12',lw=4,label='Replayed saved .10 s prefix')
    ax.scatter(*d['B'][:2],marker='*',s=130,color='black',label='Genuine switch B',zorder=6)
    ax.scatter(*f[-1,:2],marker='s',color='#bd3091',s=65,label='FRESH endpoint',zorder=6)
    q=d['minimum_clearance_pose'];ax.scatter(*q[:2],marker='x',s=75,color='red',label='Minimum execution clearance',zorder=6)
    ax.add_patch(Circle(q[:2],.2,fill=False,color='red',alpha=.5))
    if d['join_time_s'] is not None:
        i=int(np.searchsorted(d['times_s'],d['join_time_s']));ax.scatter(*p[i,:2],marker='D',color='green',s=70,label='First sampled sustained attachment',zorder=7)
    for x in p[::30]:ax.arrow(*x[:2],.09*np.cos(x[2]),.09*np.sin(x[2]),width=.002,color='#007d92',zorder=4)
    ax.set(aspect='equal',xlabel='World X [m]',ylabel='World Y [m]',title='OSA03 REPEAT_00 | Native execution tracking original FRESH\nOffline continuation; no new VLA; endpoint is not completed bypass')
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);ax.grid(alpha=.2);finish(fig,'world')
    d=data['attachment'];t=np.array(d['times_s'])
    fig,axes=plt.subplots(4,1,figsize=(10,9),sharex=True)
    for ax,vals,label,limit in zip(axes,[d['distance_m'],np.degrees(d['yaw_error_rad']),np.degrees(np.unwrap(d['yaw_world_rad'])),d['lateral_m']],['FRESH distance [m]','Wrapped yaw error [deg]','Execution yaw [deg]','Hallway-left displacement [m]'],[.1,15,None,None]):
        ax.plot(t,vals,color='#007d92');ax.set_ylabel(label);ax.grid(alpha=.2)
        if limit is not None:ax.axhline(limit,ls='--',color='#bd3091',label='Frozen tube threshold')
        if d['join_time_s'] is not None:ax.axvline(d['join_time_s'],ls=':',color='green',label='Sustained attachment starts')
    axes[0].legend(loc='best');axes[-1].set_xlabel('Seconds after B');fig.suptitle('Attachment to ORIGINAL FRESH | .10 m / 15 deg for following .30 s');finish(fig,'attachment')
    d=data['commands'];u=np.array(d['commands']);tc=np.array(d['times_s'])
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for j,(ax,label,limits) in enumerate(zip(axes,['v [m/s]','omega [rad/s]'],[[0,.8],[-3,3]])):
        ax.step(np.r_[tc,t[-1]],np.r_[u[:,j],u[-1,j]],where='post',color='#007d92',label='Applied held command')
        ax.scatter([0],[d['u_minus'][j]],marker='x',color='gray',label='Physical pre-switch u-minus')
        ax.scatter([0],[d['u_mem_B'][j]],marker='o',facecolors='none',edgecolors='red',label='B memory = saved first result')
        for lim in limits:ax.axhline(lim,color='gray',ls=':',lw=.7)
        ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[0].legend(loc='best',fontsize=8);axes[-1].set_xlabel('Seconds after B');fig.suptitle('Official Native commands | no extra solve at B');finish(fig,'commands')
    d=data['progress'];fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    axes[0].plot(t,d['fractional_row_progress'],label='Forward-only evaluator');axes[0].plot(t,d['unrestricted_progress'],'--',label='Unrestricted diagnostic');axes[0].set_ylabel('Original fractional row');axes[0].legend()
    axes[1].plot(t,d['arc_progress_m'],label='Arc progress');axes[1].plot(t,d['remaining_arc_m'],label='Remaining arc');axes[1].set_ylabel('Original-FRESH arc [m]');axes[1].legend();axes[1].set_xlabel('Seconds after B')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Projection is evaluation only; no selector change');finish(fig,'progress')
    d=data['clearance'];fig,ax=plt.subplots(figsize=(10,5));ax.plot(d['times_s'],d['clearance_m'],label='Execution footprint-edge clearance')
    ax.axhline(.05,color='red',ls='--',label='Required .05 m');ax.axhline(d['raw_FRESH_minimum_m'],color='#bd3091',ls=':',label='Raw FRESH minimum')
    ax.axvline(d['minimum_time_s'],color='gray',ls=':');ax.set(xlabel='Seconds after B',ylabel='Edge clearance [m]',title='Unchanged direct Hospital + cart checker | finite diagnostic samples');ax.grid(alpha=.2);ax.legend();finish(fig,'clearance')
    d=data['selections'];rr=d['rows'];ts=[z['time_s'] for z in rr];nearest=[z['result']['selection']['nearest_index'] for z in rr]
    selected=np.array([z['result']['selection']['indices'] for z in rr])
    fig,ax=plt.subplots(figsize=(10,5));ax.plot(ts,nearest,'ko-',ms=3,label='Native nearest row')
    for j in range(selected.shape[1]):ax.plot(ts,selected[:,j],'.--',label=f'Prediction slot {j+1}')
    ax.axvspan(ts[0],.1,color='orange',alpha=.1,label='Historical accepted results');ax.set(xlabel='Solve input time relative to B [s]',ylabel='Original raw FRESH row index',title='Actual official nearest/+1 selections (no resampling)');ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=8);finish(fig,'selections')
    rows=[dict(window_steps=k,**{x:w.get(x) for x in ['available','observation_horizon_s','mean_distance_time_weighted_m','position_auc_m_s','position_excess_auc_m_s','yaw_auc_rad_s','yaw_excess_auc_rad_s','max_distance_m']}) for k,w in m['windows'].items()]
    write_csv(out/'windows.csv',rows)
    write_csv(out/'trace.csv',[dict(time_s=float(t[i]),x=float(p[i,0]),y=float(p[i,1]),yaw_rad=float(p[i,2]),distance_m=m['trace']['distance_m'][i],yaw_error_rad=m['trace']['yaw_error_rad'][i],fractional_progress=m['trace']['projection'][i]['progress'],arc_progress_m=m['trace']['arc_progress_m'][i]) for i in range(len(t))])
    save(out/'plot_manifest.json',manifest)
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>OSA03 Native continuation</h1>'
    body+='<p>One offline, phase-preserving continuation of REPEAT_00. Recorded .10 s prefix then new asynchronous official MPC. No new VLA. This is not complete obstacle traversal.</p>'
    body+='<p>'+html.escape(m['classification'])+'</p><p>Nominal 3 s observation cap, not LightNav waypoint timing.</p>'
    for x in manifest:body+=f'<h2>{x["name"]}</h2><img src="review/{x["name"]}.png"><p><a href="review/{x["name"]}.json">Numeric/hash sidecar</a></p>'
    with (run/'index.html').open('x') as h:h.write(body)
    save(run/'report_validation.json',validate_figures(run))
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for q in [run/'index.html',run/'metrics.json',run/'report_validation.json',*out.iterdir()]:z.write(q,q.relative_to(run))
    print(json.dumps(dict(validation=v,report=read(run/'report_validation.json'))))


def validate_figures(run):
    data=plot_data(run)
    for p in read(run/'review/plot_manifest.json'):
        key=p['name'];assert sha(run/'review'/f'{key}.png')==p['png_sha256'];assert sha(run/'review'/f'{key}.json')==p['sidecar_sha256']
        side=read(run/'review'/f'{key}.json');equal_record(data[key],side['data'])
        for f,h in side['inputs'].items():assert sha(f)==h
        assert side['processing_sha256']==sha(__file__)
    with (run/'review/windows.csv').open() as f:
        for row in csv.DictReader(f):
            w=read(run/'metrics.json')['windows'][row.pop('window_steps')]
            for key,val in row.items():assert val==str(w.get(key)) if w.get(key) is not None else val==''
    return dict(valid=True,six_numeric_sidecars_recomputed=True,windows_csv_parity=True,new_MPC_VLA_calls=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate-only',action='store_true');a=p.parse_args()
    print(validate_figures(a.run.resolve())) if a.validate_only else report(a.run.resolve())
