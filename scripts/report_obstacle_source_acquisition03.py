#!/usr/bin/env python3
"""Saved-only OSA03 tables, plots and independently rechecked sealed source bundles."""
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
from report_obstacle_source_acquisition02 import compact
from validate_obstacle_source_acquisition03 import validate
from validate_robotless_online_handoffs import equal_record
from analyze_join_online02 import csvread,pose_rows
from run_join_online02 import environments


def summary(run,v):
    rows=[]
    for r in v['rows']:
        if not r['B_state'] or not r['FRESH'] or not r['raw_behavior']:
            rows.append(dict(episode_id=r['episode_id'],qualified=False,available=False,
                             failure_reasons=r['failure_reasons'],metrics=None,calls=r['calls']))
            continue
        x=compact(r)
        boot=r['target_OLD_application'];obs=r['OLD']['observation'];reveal=r['reveal']
        x.update(available=True,OLD_observation_pose=obs['pose_world'],OLD_N=r['OLD']['N'],FRESH_N=r['FRESH']['N'],
            OLD_obstruction_pass=r['raw_behavior']['gates']['B_OBSTACLE_RELEVANT'],
            OLD_observation_sim_s=obs['capture_sim_time_s'],OLD_application_sim_s=boot['t_switch']['sim_time_s'],
            OLD_timing={k:boot[k] for k in ['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch']},
            reveal_sim_s=reveal['start']['sim_time_s'],reveal_after_OLD_sim_s=reveal['start']['sim_time_s']-boot['t_switch']['sim_time_s'],
            reveal_host=reveal,whole_episode_RTF=read(run/'episodes'/r['episode_id']/'metadata.json')['episode_rtf'],
            integration=r['original_validation']['integration'],burst_steps=len(r['scheduler']['burst_state_ids']),
            minimum_wall_step_s=r['scheduler']['minimum_wall_step_s'],
            FRESH_history_count=r['context']['actual_history_count'],FRESH_endpoint_from_front_m=r['FRESH']['endpoint_from_front_m'])
        rows.append(x)
    callrecords=[r['calls'] for r in v['rows']]
    calls=dict(scientific_terminal_predictions=sum(x['terminal_predictions'] for x in callrecords),
        buffer_only_requests=sum(x['buffers'] for x in callrecords),
        server_synthetic_warmup_calls=(run/'logs/server.log').read_text().count('warmup done in'),
        MPC_submissions=sum(x['MPC_submissions'] for x in callrecords),MPC_saved_results=sum(x['MPC_saved_results'] for x in callrecords),
        MPC_saved_wall_s=sum(x['saved_MPC_wall_s'] for x in callrecords),
        terminal_RTT_sum_s=sum(x['terminal_RTT_sum_s'] for x in callrecords),
        PhaseA=0,technical_qualification=0,GP=0,rigid=0,graph=0,splice=0,reconciliation=0,report_validator_real_calls=0)
    return dict(classification=v['classification'],representative=v['representative'],qualified_count=v['qualified_count'],
        scientific_freeze_sha=read(run/'execution_start.json')['sha'],starting_sha=read(run/'source.json')['starting_sha'],
        protocol=read(run/'protocol.json'),episodes=rows,calls=calls,collector_wall_s_excluding_Isaac_startup=read(run/'completion.json')['wall_s'],
        source_validation_sha256=sha(run/'validation.json'),preserved_hash_count=len(read(run/'source.json')['preserved']),
        scope='genuine sudden-obstacle local-avoidance moving-B source only; .10 s postroll, no full bypass or reconciliation evaluation')


def presentation(run):
    v=read(run/'validation.json');s=summary(run,v)
    save(run/'aggregate/summary.json',s)
    fields=['episode_id','qualified','OLD_arc_m','OLD_clearance_before_reveal_m','OLD_hypothetical_cart_on_clearance_m','OLD_obstruction_pass',
            'FRESH_clearance_m','FRESH_lateral_m','FRESH_max_yaw_deg','FRESH_forward_progress_m','mismatch_interior_m',
            'observation_B_travel_m','B_clearance_m','RTF','max_loop_stall_s','client_RTT_s','reveal_after_OLD_sim_s']
    with (run/'aggregate/episodes.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in s['episodes']:w.writerow({k:r.get(k) for k in fields})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from PIL import Image
    out=run/'review';out.mkdir(exist_ok=False);manifest=[]
    def finish(fig,name,data,inputs):
        fig.savefig(out/(name+'.png'),dpi=150,bbox_inches='tight');plt.close(fig)
        save(out/(name+'.json'),dict(data=data,source_sha256={str(p):sha(p) for p in inputs},processing_script_sha256=sha(__file__)))
        manifest.append(dict(name=name,png_sha256=sha(out/(name+'.png')),json_sha256=sha(out/(name+'.json'))))
    _,_,cart,_=environments(run)
    for r,x in zip(v['rows'],s['episodes'],strict=True):
        if not x['available']:continue
        eid=r['episode_id'];ep=run/'episodes'/eid;actual=pose_rows(csvread(ep/'execution.csv'))
        old=np.asarray(r['OLD']['world']);fresh=np.asarray(r['FRESH']['world']);ctx=r['context'];i,j=ctx['obs_state_id'],ctx['switch_state_id']
        fig,ax=plt.subplots(figsize=(9,7));polys=list(cart.geoms) if hasattr(cart,'geoms') else [cart]
        for n,poly in enumerate(polys):ax.fill(*poly.exterior.xy,color='saddlebrown',alpha=.5,label='Revealed cart footprint' if n==0 else None)
        buf=cart.buffer(.25);polys=list(buf.geoms) if hasattr(buf,'geoms') else [buf]
        for n,poly in enumerate(polys):ax.plot(*poly.exterior.xy,':',color='saddlebrown',label='Nominal radius .20 + margin .05 m' if n==0 else None)
        ax.plot(old[:,0],old[:,1],'o--',color='#2765b0',ms=3,label='Original OLD future (obstructed after reveal)')
        ax.plot(fresh[:,0],fresh[:,1],'o--',color='#bc3295',ms=3,label='Original FRESH future (whole raw safe)')
        ax.plot(actual[:i+1,0],actual[:i+1,1],color='gray',lw=2,label='Actual OLD execution before reveal')
        ax.plot(actual[i:j+1,0],actual[i:j+1,1],color='#168656',lw=3,label='Actual OLD execution observation → B')
        ax.plot(actual[j:,0],actual[j:,1],color='#e8831c',lw=3,label='Actual FRESH postroll (.10 s only)')
        ax.plot(*x['OLD_observation_pose'][:2],marker='s',color='#2765b0',ms=6,label='Exact POSE11 / OLD observation')
        ax.plot(*x['observation_pose'][:2],marker='x',color='black',ms=9,label='Reveal / FRESH observation')
        ax.plot(*x['B'][:2],marker='*',color='black',ms=13,label='Actual B');ax.add_patch(Circle(x['B'][:2],.2,fill=False,color='black',alpha=.5))
        yaw=x['OLD_observation_pose'][2];ax.arrow(*x['OLD_observation_pose'][:2],.15*np.cos(yaw),.15*np.sin(yaw),width=.003,color='black')
        ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]',title=f'{eid} | source qualified: {x["qualified"]}\nOLD+cart edge {x["OLD_hypothetical_cart_on_clearance_m"]:.3f} m; FRESH edge {x["FRESH_clearance_m"]:.3f} m')
        ax.grid(alpha=.2);ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);fig.tight_layout()
        finish(fig,eid+'_world',dict(OLD=old.tolist(),FRESH=fresh.tolist(),actual=actual.tolist(),obs_state_id=i,B_state_id=j,metrics=x,cart_wkb_hex=cart.wkb_hex),[run/'validation.json',ep/'execution.csv',run/'scenario.json'])
        fig,axes=plt.subplots(1,2,figsize=(11,4))
        for ax,key in zip(axes,['OLD','FRESH']):
            imagepath=ep/r[key]['observation']['path'];ax.imshow(Image.open(imagepath));ax.axis('off')
            ax.set_title(f'{key}: new live input RGB, cart '+('OFF' if key=='OLD' else 'ON'))
        fig.suptitle(eid+' | annotations outside model RGB; no historical replay');fig.tight_layout()
        finish(fig,eid+'_RGB',dict(OLD=r['OLD']['observation'],FRESH=r['FRESH']['observation'],cart_pixels=r['cart_pixels']),[ep/r[k]['observation']['path'] for k in ['OLD','FRESH']])
        keys=['OLD observation','OLD receipt','OLD application','cart reveal','FRESH observation','FRESH sent','FRESH receipt','FRESH ready seen','FRESH install','B application']
        boot=r['target_OLD_application'];events=[boot['t_obs'],boot['t_ready_host'],boot['t_switch'],r['reveal']['start'],*[ctx[k] for k in ['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch']]]
        h0=events[0]['host_monotonic_s'];t0=events[0]['sim_time_s']
        hosts=[t['host_monotonic_s']-h0 for t in events];sims=[None if 'sim_time_s' not in t else t['sim_time_s']-t0 for t in events]
        fig,axes=plt.subplots(1,2,figsize=(12,5),sharey=True)
        axes[0].plot(hosts,range(len(keys)),'o');axes[0].set(yticks=range(len(keys)),yticklabels=keys,xlabel='Host seconds since OLD observation',title='Host monotonic clock')
        for n,t in enumerate(sims):
            if t is not None:axes[1].plot(t,n,'o',color='#168656')
        axes[1].set(xlabel='Simulation seconds since OLD observation',title='Simulation clock; absent fields remain N/A');axes[0].invert_yaxis()
        fig.suptitle(eid+' | actual OLD application precedes first scheduled reveal');fig.tight_layout()
        finish(fig,eid+'_timing',dict(labels=keys,host_relative_s=hosts,simulation_relative_s=sims,absolute=events),[run/'validation.json'])
    labels=list(v['rows'][0]['gates']);values=[[int(r['gates'][k]) for k in labels] for r in v['rows']]
    fig,ax=plt.subplots(figsize=(13,4));ax.imshow(values,cmap='RdYlGn',vmin=0,vmax=1,aspect='auto')
    ax.set(yticks=[0,1],yticklabels=[r['episode_id'] for r in v['rows']],xticks=range(len(labels)),xticklabels=labels,title='Frozen source gates | source qualification is not full obstacle traversal')
    plt.setp(ax.get_xticklabels(),rotation=45,ha='right',fontsize=8)
    for i,row in enumerate(values):
        for j,a in enumerate(row):ax.text(j,i,'PASS' if a else 'FAIL',ha='center',va='center',fontsize=7)
    fig.tight_layout();finish(fig,'qualification_matrix',dict(labels=labels,values=values),[run/'validation.json'])
    save(out/'plot_manifest.json',manifest)
    body='<meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1300px;margin:30px auto}img{max-width:100%}</style>'
    body+=f'<h1>OSA03 — {s["classification"]}</h1><p>Qualified {s["qualified_count"]}/2; representative {s["representative"]}. Genuine source only; no reconciliation, complete bypass or long navigation evaluation.</p>'
    body+='<p>Blue dashed = returned OLD future; magenta dashed = returned FRESH future; solid = actual execution. Negative hypothetical OLD clearance does not mean an executed collision.</p>'
    body+='<p><a href="aggregate/summary.json">Numeric summary</a> · <a href="aggregate/episodes.csv">CSV</a> · <a href="validation.json">Saved source validation</a> · <a href="source_bundle/manifest.json">Sealed source manifest</a></p>'
    for m in manifest:body+=f'<h2>{html.escape(m["name"])}</h2><a href="review/{m["name"]}.json">Numeric/hash sidecar</a><img src="review/{m["name"]}.png">'
    with (run/'index.html').open('x') as f:f.write(body)
    with zipfile.ZipFile(run/'review_bundle.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [run/'index.html',run/'aggregate/summary.json',run/'aggregate/episodes.csv',run/'protocol.json',run/'validation.json',run/'source_bundle/manifest.json',*sorted(out.glob('*'))]:z.write(p,p.relative_to(run))
    print(json.dumps(dict(classification=s['classification'],calls=s['calls']),indent=2))


def validate_report(run):
    from validate_obstacle_source02_report import csv_parity,zip_parity
    v=validate(run);equal_record(v,read(run/'validation.json'),'saved-only recomputation')
    s=summary(run,v);equal_record(s,read(run/'aggregate/summary.json'),'summary')
    with (run/'aggregate/episodes.csv').open() as f:
        written=list(csv.DictReader(f))
    assert len(written)==len(s['episodes'])
    for row,expected in zip(written,s['episodes'],strict=True):
        for k,a in row.items():assert a==('' if expected.get(k) is None else str(expected[k]))
    for m in read(run/'review/plot_manifest.json'):
        path=run/'review'/m['name'];assert sha(path.with_suffix('.png'))==m['png_sha256'];assert sha(path.with_suffix('.json'))==m['json_sha256']
        data=read(path.with_suffix('.json'))
        assert data['processing_script_sha256']==sha(__file__)
        for p,h in data['source_sha256'].items():assert sha(p)==h,p
    for r in v['rows']:
        if not r['B_state']:continue
        eid=r['episode_id'];data=read(run/'review'/(eid+'_world.json'))['data'];ep=run/'episodes'/eid
        for key in ['OLD','FRESH']:np.testing.assert_array_equal(data[key],np.load(ep/r[key]['world_ref']['path']))
        np.testing.assert_array_equal(data['actual'],pose_rows(csvread(ep/'execution.csv')))
    bundle=read(run/'source_bundle/manifest.json');assert bundle['representative']==v['representative'] and len(bundle['entries'])==v['qualified_count']
    for p in map(Path,bundle['entries']):
        for name,h in read(p/'hashes.json').items():assert sha(p/name)==h
        r=next(r for r in v['rows'] if r['episode_id']==p.name)
        assert r['qualified']
        for label,key in [('old','OLD'),('fresh','FRESH')]:
            assert (p/'raw'/(label+'_lightnav.npy')).read_bytes()==(run/'episodes'/p.name/r[key]['raw_local_ref']['path']).read_bytes()
            assert (p/'derived'/(label+'_world.npy')).read_bytes()==(run/'episodes'/p.name/r[key]['world_ref']['path']).read_bytes()
        equal_record(read(p/'state_and_timing.json')['B_state'],r['B_state'],'bundle B')
    zip_parity(run)
    return dict(valid=True,saved_recomputation=True,source_preservation=True,bundle_hashes_and_raw_parity=True,
        CSV_JSON_plot_ZIP_parity=True,real_model_MPC_optimizer_calls=0,classification=v['classification'],
        qualified_count=v['qualified_count'],representative=v['representative'],source_validation_sha256=sha(run/'validation.json'),
        summary_sha256=sha(run/'aggregate/summary.json'),report_script_sha256=sha(__file__))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args();run=a.run.resolve()
    if a.validate:
        v=validate_report(run)
        if a.output:save(a.output,v)
        print(json.dumps(v,indent=2))
    else:presentation(run)
