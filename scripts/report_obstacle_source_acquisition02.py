#!/usr/bin/env python3
"""Saved-only OSA02 completion/plots; never retries or changes frozen gates."""
import argparse
import csv
import html
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from analyze_join_online02 import csvread,pose_rows,jsonlines
from run_join_online02 import environments
from run_join_source05 import verify
from validate_robotless_online_handoffs import equal_record


def compact(row):
    b=row['B_state'];c=row['context'];d=row['raw_behavior']['difference'];f=row['FRESH'];o=row['OLD']
    return dict(episode_id=row['episode_id'],qualified=row['qualified'],gates=row['gates'],
        failure_reasons=row['failure_reasons'],raw_failure_reasons=row['raw_behavior']['failure_reasons'],
        OLD_clearance_before_reveal_m=o['geometry_off']['minimum_clearance_m'],OLD_hypothetical_cart_on_clearance_m=o['geometry_on']['minimum_clearance_m'],
        OLD_arc_m=o['arc_m'],OLD_max_lateral_m=o['max_lateral_m'],OLD_stop=o['stop'],
        FRESH_clearance_m=f['geometry_on']['minimum_clearance_m'],FRESH_arc_m=f['arc_m'],FRESH_stop=f['stop'],
        FRESH_lateral_m=f['max_lateral_m'],FRESH_max_yaw_deg=f['max_yaw_deg'],FRESH_forward_progress_m=f['forward_progress_m'],
        mismatch_interior_m=d['maximum_interior_separation_m'],mismatch_reliable_tangent_deg=d['maximum_reliable_tangent_difference_deg'],mismatch_reliable_yaw_deg=d['maximum_reliable_yaw_difference_deg'],
        observation_pose=f['observation']['pose_world'],B=b['pose_world'],u_minus=b['u_minus'],controller_previous_control=b['memory']['previous_control'],
        observation_B_travel_m=row['observation_B_travel_m'],B_clearance_m=row['B_clearance']['minimum_clearance_m'],
        future=row['remaining_future'],RTF=row['timing']['request_rtfs'][0],max_loop_stall_s=row['timing']['max_loop_stall_s'],
        client_RTT_s=f['client_RTT_s'],observation_sim_s=c['t_obs']['sim_time_s'],ready_seen_sim_s=c['t_ready_seen_sim']['sim_time_s'],
        install_sim_s=c['t_install']['sim_time_s'],switch_sim_s=c['t_switch']['sim_time_s'],
        observation_switch_sim_s=c['t_switch']['sim_time_s']-c['t_obs']['sim_time_s'],
        observation_request_host_s=c['t_request']['host_monotonic_s']-c['t_obs']['host_monotonic_s'],
        observation_switch_host_s=c['t_switch']['host_monotonic_s']-c['t_obs']['host_monotonic_s'],
        all_timing_fields={k:c[k] for k in ['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch']},
        cart_pixels=row['cart_pixels'],safety_abort=row['safety_abort'] is not None,termination=row['termination'],
        actual_prefix_clearance_lower_bound_m=row['actual_prefix_minimum_clearance_lower_bound_m'],
        complete_bypass=row['complete_bypass'],calls=row['calls'],
        OLD_raw_sha256=o['raw_local_ref']['sha256'],FRESH_raw_sha256=f['raw_local_ref']['sha256'])


def report(parent):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from PIL import Image
    verify(parent);verify(parent/'phaseB')
    v=read(parent/'phaseB/validation.json');p0=read(parent/'phase0/validation.json');old=read(parent/'diagnosis/diagnosis.json')
    assert v['valid'] and p0['valid']
    rows=[compact(x) for x in v['rows']]
    calls=dict(technical_predictions=p0['calls']['technical_predictions'],scientific_online_predictions=sum(x['calls']['terminal_predictions'] for x in rows),
        PhaseA_predictions=0,technical_buffers=p0['calls']['technical_buffers'],online_buffers=sum(x['calls']['buffers'] for x in rows),
        server_synthetic_warmup_calls=(parent/'logs/server.log').read_text().count('warmup done in'),
        MPC_accepted=p0['calls']['MPC_submissions']+sum(x['calls']['MPC_submissions'] for x in rows),
        MPC_saved=p0['calls']['MPC_saved_results']+sum(x['calls']['MPC_saved_results'] for x in rows),
        MPC_saved_wall_s=p0['calls']['saved_MPC_wall_s']+sum(x['calls']['saved_MPC_wall_s'] for x in rows),
        GP=0,rigid=0,graph=0,splice=0,reconciliation=0,report_and_validator_new_calls=0,regression_real_calls=0)
    summary=dict(classification=v['classification'],representative=v['representative'],qualified_count=v['qualified_count'],
        starting_sha=read(parent/'source.json')['starting_sha'],technical_execution_sha=read(parent/'phase0/execution_start.json')['sha'],
        PhaseB_execution_sha=read(parent/'phaseB/execution_start.json')['sha'],PhaseA_rerun=False,
        phase0=p0['qualification'],phase0_requests=[{k:r[k] for k in ['chunk_id','client_RTT_s','request_local_RTF','measured_burst_steps','maximum_consecutive_no_sleep_steps']} for r in p0['scheduler']['requests']],
        phase0_whole_RTF=p0['analysis']['summary']['episode_rtf'],phase0_min_wall_step_s=p0['scheduler']['minimum_wall_step_s'],
        phase0_integration=p0['saved_episode_validation']['integration'],phaseB=rows,calls=calls,
        phase0_wall_s_excluding_Isaac_startup=read(parent/'phase0/completion.json')['wall_s'],
        PhaseB_wall_s_excluding_Isaac_startup=read(parent/'phaseB/completion.json')['wall_s'],
        remaining_uncertainty='Whether a genuine OLD whose finite stored future is actually obstructed can coexist with this safe-FRESH moving-B response under separately frozen acquisition conditions.',
        scope='safe turning raw FRESH and moving B observed twice, but OLD obstruction gate fails; no accepted obstacle-induced representative source')
    save(parent/'aggregate/summary.json',summary)
    fields=['episode_id','qualified','OLD_hypothetical_cart_on_clearance_m','FRESH_clearance_m','FRESH_arc_m','FRESH_lateral_m','FRESH_max_yaw_deg','mismatch_interior_m','observation_B_travel_m','B_clearance_m','RTF','max_loop_stall_s','client_RTT_s','safety_abort']
    with (parent/'aggregate/episodes.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields+['failure_reasons']);w.writeheader()
        for r in rows:w.writerow({**{k:r[k] for k in fields},'failure_reasons':';'.join(r['raw_failure_reasons'])})
    out=parent/'review';out.mkdir(exist_ok=False);manifest=[]
    def finish(fig,name,data,sources):
        fig.savefig(out/(name+'.png'),dpi=150,bbox_inches='tight');plt.close(fig)
        save(out/(name+'.json'),dict(data=data,source_sha256={str(q):sha(q) for q in sources},processing_script_sha256=sha(__file__)))
        manifest.append(dict(name=name,png_sha256=sha(out/(name+'.png')),json_sha256=sha(out/(name+'.json'))))
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    before=[r['request_local_RTF'] for r in old['requests'][1:]];after=p0['qualification']['timing_gate']['request_rtfs']
    axes[0].plot([1,2,3],before,'o-',label='OSA01 absolute deadline');axes[0].plot([1,2,3],after,'o-',label='OSA02 no catch-up')
    axes[0].axhspan(.8,1.2,color='green',alpha=.1);axes[0].set(xticks=[1,2,3],xlabel='Post-bootstrap request',ylabel='Request-local RTF');axes[0].legend(fontsize=8)
    # Matching saved old C1 window; time is relative to first included state, never simulation time.
    steps=old['steps'];r=old['requests'][1];a=[s for s in steps if r['first_inflight_state']<=s['start_state_id']<r['last_inflight_state']]
    new=read(parent/'phase0/scheduler_analysis.json')['requests'][1];nr=jsonlines(parent/'phase0/episodes/PACING_OFF_00/scheduler.jsonl');b=[s for s in nr if new['first_inflight_state']<=s['start_state_id']<new['last_inflight_state']]
    axes[1].plot(range(len(a)),[s['wall_dt_s']*1000 for s in a],'o-',label='Before');axes[1].plot(range(len(b)),[s['wall_step_s']*1000 for s in b],'o-',label='After')
    axes[1].axhline(1000/60,color='black',linestyle=':',label='nominal 16.667 ms');axes[1].set(xlabel='Actual interval within C1 window',ylabel='Wall interval [ms]');axes[1].legend(fontsize=8)
    fig.suptitle('Saved pacing diagnosis | same simulation dt, different wall scheduling');fig.tight_layout()
    finish(fig,'pacing_before_after',dict(before=before,after=after,before_C1_wall_ms=[s['wall_dt_s']*1000 for s in a],after_C1_wall_ms=[s['wall_step_s']*1000 for s in b]),[parent/'diagnosis/diagnosis.json',parent/'phase0/validation.json'])
    _,_,cart,_=environments(parent/'phaseB')
    for row,full in zip(rows,v['rows']):
        ep=parent/'phaseB/episodes'/row['episode_id'];st=csvread(ep/'execution.csv');path=pose_rows(st);c=full['context'];i,j=c['obs_state_id'],c['switch_state_id']
        oldpath=np.asarray(full['OLD']['world']);fresh=np.asarray(full['FRESH']['world']);B=np.asarray(row['B']);obs=np.asarray(row['observation_pose'])
        fig,ax=plt.subplots(figsize=(8,7))
        polygons=list(cart.geoms) if hasattr(cart,'geoms') else [cart]
        for poly in polygons:
            x,y=poly.exterior.xy;ax.fill(x,y,color='saddlebrown',alpha=.5)
        for poly in ([cart.buffer(.25)] if not hasattr(cart.buffer(.25),'geoms') else cart.buffer(.25).geoms):ax.plot(*poly.exterior.xy,':',color='saddlebrown',lw=1.5)
        ax.plot(oldpath[:,0],oldpath[:,1],'o--',color='#2867ba',ms=3,label='Original OLD raw future')
        ax.plot(fresh[:,0],fresh[:,1],'o--',color='#bd3c97',ms=3,label='Post-reveal FRESH raw future')
        ax.plot(path[:i+1,0],path[:i+1,1],color='gray',lw=2,label='Actual OLD prefix')
        ax.plot(path[i:j+1,0],path[i:j+1,1],color='#13845d',lw=3,label='Actual OLD observation→B')
        ax.plot(path[j:,0],path[j:,1],color='#ef8d25',lw=3,label='Actual .10 s FRESH postroll')
        ax.plot(*obs[:2],marker='x',color='black',ms=8,label='Reveal / FRESH observation')
        ax.plot(*B[:2],marker='*',color='black',ms=12,label='Actual B')
        ax.add_patch(Circle(B[:2],.2,fill=False,color='black',alpha=.6));ax.arrow(*obs[:2],.15*np.cos(obs[2]),.15*np.sin(obs[2]),width=.004,color='black')
        ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]');ax.grid(alpha=.2)
        ax.set_title(row['episode_id']+' | raw FRESH safe; source NOT QUALIFIED\nOLD remains safe with cart: obstruction gate fails')
        ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);fig.tight_layout()
        finish(fig,row['episode_id']+'_world',dict(OLD=oldpath.tolist(),FRESH=fresh.tolist(),actual=path.tolist(),obs_state_id=i,B_state_id=j,metrics=row,cart_wkb_hex=cart.wkb_hex,cart_outline_radius_m=.25),[parent/'phaseB/validation.json',ep/'execution.csv',parent/'phaseB/scenario.json'])
        fig,axes=plt.subplots(1,2,figsize=(11,4))
        for ax,key in zip(axes,['OLD','FRESH']):
            rec=full[key];imgpath=ep/rec['observation']['path'];ax.imshow(Image.open(imgpath));ax.axis('off')
            ax.set_title(f'{key}: actual transmitted RGB\nframe {rec["observation"]["frame_id"]}, cart '+('OFF' if key=='OLD' else 'ON'))
        fig.suptitle(row['episode_id']+' | no annotations supplied to LightNav');fig.tight_layout()
        finish(fig,row['episode_id']+'_RGB',dict(OLD=full['OLD']['observation'],FRESH=full['FRESH']['observation'],cart_pixels=row['cart_pixels']),[ep/full['OLD']['observation']['path'],ep/full['FRESH']['observation']['path'],parent/'phaseB/validation.json'])
        labels=['observation','request sent','receipt','ready seen','install','first FRESH command']
        keys=['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch']
        host0=c['t_obs']['host_monotonic_s'];sim0=c['t_obs']['sim_time_s'];ht=[c[k]['host_monotonic_s']-host0 for k in keys]
        sv=[None if 'sim_time_s' not in c[k] else c[k]['sim_time_s']-sim0 for k in keys]
        fig,axes=plt.subplots(1,2,figsize=(11,4),sharey=True)
        axes[0].plot(ht,range(6),'o');axes[0].set(xlabel='Host time since observation [s]',yticks=range(6),yticklabels=labels,title='Host monotonic clock')
        for ii,x in enumerate(sv):
            if x is not None:axes[1].plot(x,ii,'o',color='#13845d')
        axes[1].set(xlabel='Simulation time since observation [s]',title='Simulation clock; send/receipt N/A')
        axes[0].invert_yaxis();fig.suptitle(row['episode_id']+' | clocks are separate');fig.tight_layout()
        finish(fig,row['episode_id']+'_timing',dict(labels=labels,host_relative_s=ht,simulation_relative_s=sv,absolute=c),[parent/'phaseB/validation.json'])
    matrix=[];labels=[]
    for row,full in zip(rows,v['rows']):
        flags={**full['raw_behavior']['gates'],**{k:x for k,x in row['gates'].items() if k!='raw_behavior'}}
        labels=list(flags);matrix.append([int(flags[k]) for k in labels])
    fig,ax=plt.subplots(figsize=(12,3.8));ax.imshow(matrix,cmap='RdYlGn',vmin=0,vmax=1,aspect='auto')
    ax.set(yticks=[0,1],yticklabels=[x['episode_id'] for x in rows],xticks=range(len(labels)),xticklabels=labels,title='Every source gate retained | ONLY OLD obstruction gate fails in both')
    plt.setp(ax.get_xticklabels(),rotation=45,ha='right',fontsize=8)
    for i,line in enumerate(matrix):
        for j,x in enumerate(line):ax.text(j,i,'PASS' if x else 'FAIL',ha='center',va='center',fontsize=7)
    fig.tight_layout();finish(fig,'source_gate_matrix',dict(labels=labels,values=matrix),[parent/'phaseB/validation.json'])
    save(out/'plot_manifest.json',manifest)
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}</style><h1>OSA02 — ONLINE_POSE11_SOURCE_NOT_QUALIFIED</h1>'
    body+='<p>Pacing qualifies. Safe turning raw FRESH and moving B are observed twice. Both sources fail ONLY the predeclared OLD obstruction gate: OLD hypothetical cart-ON edge clearance remains +0.273959 m. No accepted source, no retry, no optimization.</p>'
    body+='<p>Selected bank POSE11 is unchanged. Online OLD was generated 0.40 m behind it and ends before cart. We do not extrapolate or reanchor that finite OLD path.</p>'
    body+='<p><a href="aggregate/summary.json">Full numeric summary</a> · <a href="aggregate/episodes.csv">CSV</a> · <a href="phaseB/validation.json">Saved-record validation</a></p>'
    for m in manifest:body+=f'<h2>{html.escape(m["name"])}</h2><a href="review/{m["name"]}.json">Numeric/source sidecar</a><img src="review/{m["name"]}.png">'
    with (parent/'index.html').open('x') as f:f.write(body)
    with zipfile.ZipFile(parent/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for q in [parent/'index.html',parent/'aggregate/summary.json',parent/'aggregate/episodes.csv',parent/'protocol.json',parent/'phaseB/protocol.json',*sorted(out.glob('*'))]:z.write(q,q.relative_to(parent))
    # Saved outputs, numeric sidecars and CSV must agree; no second inference.
    for m in manifest:
        assert sha(out/(m['name']+'.png'))==m['png_sha256'] and sha(out/(m['name']+'.json'))==m['json_sha256']
    with (parent/'aggregate/episodes.csv').open() as f:
        for written,row in zip(csv.DictReader(f),rows,strict=True):
            for k in fields:
                assert written[k]==str(row[k]),(k,written[k],row[k])
    save(parent/'validation_final.json',dict(valid=True,primary_saved_validation_sha256=sha(parent/'phaseB/validation.json'),
        technical_validation_sha256=sha(parent/'phase0/validation.json'),summary_sha256=sha(parent/'aggregate/summary.json'),
        PNG_sidecar_hashes=manifest,CSV_JSON_parity=True,source_preserved=True,report_script_sha256=sha(__file__),
        no_new_model_MPC_optimizer_calls=True,classification=v['classification']))
    print(json.dumps(summary['calls'],indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run.resolve())
