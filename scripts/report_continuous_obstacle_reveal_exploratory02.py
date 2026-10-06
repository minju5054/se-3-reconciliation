#!/usr/bin/env python3
"""Four compact saved-only PNGs with numeric and source-hash sidecars."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from reconciliation.continuous_obstacle_reveal_episode01 import FIGURES
from analyze_join_online02 import csvread, jsonlines, pose_rows
from run_join_online02 import environments
from validate_robotless_online_handoffs import equal_record


def inputs(run):
    v = read(run/'validation.json'); ep = Path(v['episode'])
    states = csvread(ep/'execution.csv')
    seen = jsonlines(ep/'requests/client_seen.jsonl')
    _, _, cart, _ = environments(run)
    return dict(validation=v, execution_world=pose_rows(states).tolist(),
        execution_sim_s=[float(s['sim_time_s']) for s in states],
        active_chunk=[s['active_chunk_id'] for s in states], request_seen=seen,
        cart_wkb_hex=cart.wkb_hex, validation_sha256=sha(run/'validation.json'),
        execution_sha256=sha(ep/'execution.csv'), scenario_sha256=sha(run/'scenario.json'))


def compact(v):
    from copy import deepcopy
    v=deepcopy(v)
    if v.get('first_response_geometry'):
        v['first_response_geometry']['difference'].pop('rows',None)
    keep = ['chunk_id','generated','applied','model_status','stop','A','B','P','ready_pose','install_pose',
        't_request_host','t_receipt_host','client_RTT_s','request_timing','t_ready_seen_sim','t_install','t_application',
        'cart_visible_pixels','raw_local_ref','world_ref','N','arc_m','max_lateral_m','max_yaw_deg',
        'max_reliable_tangent_deg','endpoint_local','net_yaw_rad','geometry_off','geometry_on',
        'physical_footprint_clearance_m','exploratory_overlap_free','legacy_5cm_margin_pass','history_provenance',
        'cart_only_edge_clearance_m','raw_reference_safe','B_clearance','observation_B_travel_m','remaining_future']
    return {**{k:v[k] for k in ('experiment','run','episode','scientific_freeze_sha','starting_sha',
        'exploratory_clearance','exploratory_first_reaction_valid','classification','first_response_gates','first_response_geometry','evolution','evolution_classification',
        'initial_pose','end_sim_time_s','status','termination_reason','actual_prefix_minimum_clearance_lower_bound_m','calls','interval_diagnostics')},
        'chunks':[{**{k:r.get(k) for k in keep},'observation_sim_s':r.get('observation',{}).get('capture_sim_time_s'),
            'u_minus':r.get('B_state',{}).get('u_minus'),
            'controller_memory':r.get('B_state',{}).get('memory'),
            'first_applied_command':r.get('first_applied_command')} for r in v['chunks']]}


def report(run, out):
    data = inputs(run); v = data['validation']; ep = Path(v['episode'])
    out.mkdir(parents=True, exist_ok=True); figures = out/'figures'; figures.mkdir(exist_ok=False)
    save(out/'result_summary.json', compact(v))
    rows = []
    for r in v['chunks']:
        rows.append(dict(chunk=r['chunk_id'], generated=r['generated'], applied=r['applied'],
            observation_sim_s=r.get('observation',{}).get('capture_sim_time_s'),
            ready_sim_s=r.get('t_ready_seen_sim',{}).get('sim_time_s') if r.get('t_ready_seen_sim') else None,
            application_sim_s=r.get('t_application',{}).get('sim_time_s'),
            cart_pixels=r.get('cart_visible_pixels'), N=r.get('N'), arc_m=r.get('arc_m'),
            raw_sha256=r.get('raw_local_ref',{}).get('sha256') if r.get('raw_local_ref') else None,
            exploratory_overlap_free=r.get('exploratory_overlap_free'),legacy_5cm_margin_pass=r.get('legacy_5cm_margin_pass'), clearance_m=(r.get('geometry_on') if r['chunk_id']!='chunk_000' else r.get('geometry_off') or {}).get('minimum_clearance_m') if r.get('world') else None))
    with (out/'chunks.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import shapely
    from PIL import Image
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.18})
    colors=['#2864b4','#bb3c86','#d48812','#008c80']; styles=['--','-.',':','--']
    chunks=v['chunks']; actual=np.asarray(data['execution_world']); cart=shapely.from_wkb(bytes.fromhex(data['cart_wkb_hex']))
    manifest={}
    def finish(fig,name):
        fig.savefig(figures/name,dpi=160,bbox_inches='tight');plt.close(fig)
        manifest[name]=sha(figures/name)
    fig,ax=plt.subplots(figsize=(9,6))
    for geom,kind in [(cart,'cart'),(cart.buffer(.20),'Footprint only: 0.20 m'),(cart.buffer(.25),'Legacy reserve: 0.20 + 0.05 m')]:
        polys=list(geom.geoms) if hasattr(geom,'geoms') else [geom]
        for j,g in enumerate(polys):
            if kind=='cart':ax.fill(*g.exterior.xy,color='#826a51',alpha=.45,label=kind if j==0 else None)
            else:ax.plot(*g.exterior.xy,'--' if 'Legacy' in kind else '-',color='#a33c2e' if 'Legacy' in kind else '#826a51',lw=1.5,label=kind if j==0 else None)
    for i,r in enumerate(chunks):
        if r.get('world'):
            a=np.asarray(r['world']); ax.plot(a[:,0],a[:,1],styles[i],marker=['o','s','^','d'][i],ms=3,color=colors[i],label=f'C{i} at its own A{i}')
            ax.scatter(*r['A'][:2],marker='x',s=55,color=colors[i]);ax.annotate(f'A{i}',r['A'][:2],xytext=(-27,7+i*2),textcoords='offset points',color=colors[i])
        if r['applied'] and i:
            ax.scatter(*r['B'][:2],marker='*',s=120,color=colors[i],zorder=8);ax.annotate(f'B{i}',r['B'][:2],xytext=(8,3),textcoords='offset points',color=colors[i])
    ax.plot(actual[:,0],actual[:,1],color='#202a32',lw=2.6,label='Actual continuous execution',zorder=7)
    ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]',title='Exploratory native episode: 0.20 m footprint, zero extra reserve')
    ax.text(1.02,.18,'Not generated: '+', '.join(f'C{i}' for i,r in enumerate(chunks) if not r['generated']),transform=ax.transAxes,fontsize=8)
    ax.legend(loc='upper left',bbox_to_anchor=(1.02,1),fontsize=8);fig.tight_layout();finish(fig,FIGURES[0])
    fig,axes=plt.subplots(1,2,figsize=(10,4.7))
    for i,r in enumerate(chunks):
        if not r.get('raw_local'):continue
        a=np.asarray(r['raw_local']);axes[0].plot(a[:,0],a[:,1],styles[i],marker=['o','s','^','d'][i],ms=4,color=colors[i],label=f'C{i}')
        arc=np.r_[0,np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1).cumsum()]
        axes[1].plot(arc,np.degrees(a[:,2]),styles[i],marker=['o','s','^','d'][i],ms=4,color=colors[i],label=f'C{i}')
    axes[0].set_aspect('equal');axes[0].set(xlabel='Own observation-local forward X [m]',ylabel='Left Y [m]',title='Raw geometry; no common world origin')
    axes[1].set(xlabel='Raw spatial arc [m] — untimed rows',ylabel='Local pose yaw [deg]',title='Raw pose yaw, not angular velocity')
    for ax in axes:
        if ax.get_legend_handles_labels()[0]:ax.legend()
    fig.text(.5,.01,'Missing chunks: '+(', '.join(f'C{i}: N/A' for i,r in enumerate(chunks) if not r['generated']) or 'none'),ha='center',fontsize=9)
    fig.suptitle(v['evolution_classification'] or 'Later chunk comparison: N/A',fontsize=10);fig.tight_layout();finish(fig,FIGURES[1])
    fig,axes=plt.subplots(1,3,figsize=(13,4.6),gridspec_kw={'width_ratios':[1.3,1.3,1]})
    origin=next((r['observation']['capture_monotonic_ns']/1e9 for r in chunks if r['generated']),0)
    for i,r in enumerate(chunks):
        if not r['generated']:
            for ax in axes[:2]:ax.text(.02,i,'N/A',transform=ax.get_yaxis_transform())
            continue
        sim_events=[('obs',r['observation']['capture_sim_time_s'],'o'),
            ('ready',r.get('t_ready_seen_sim',{}).get('sim_time_s') if r.get('t_ready_seen_sim') else None,'s'),
            ('apply',r.get('t_application',{}).get('sim_time_s'),'*')]
        sent=next((q for q in data['request_seen'] if q.get('chunk_id')==r['chunk_id']),None)
        sim_events.append(('send seen',None if sent is None else sent['seen_in_isaac']['sim_time_s'],'>'))
        for name,t,m in sim_events:
            if t is not None:axes[0].scatter(t,i,marker=m,color=colors[i],s=60,label=name if i==0 else None)
        for name,t,m in [('obs',r['observation']['capture_monotonic_ns']/1e9,'o'),('send',r['t_request_host']['monotonic_ns']/1e9,'>'),
            ('receipt',None if not r.get('t_receipt_host') else r['t_receipt_host']['monotonic_ns']/1e9,'s'),
            ('apply',r.get('t_application',{}).get('host_monotonic_s'),'*')]:
            if t is not None:axes[1].scatter(t-origin,i,marker=m,color=colors[i],s=60,label=name if i==0 else None)
    if v['reveal']:axes[0].axvline(v['reveal']['start']['sim_time_s'],color='brown',ls=':',label='cart ON')
    for ax in axes[:2]:
        ax.set(yticks=range(4),yticklabels=['C0','C1','C2','C3'],ylim=(3.4,-.4))
        if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8,loc='best')
    axes[0].set(xlabel='Simulation time [s]',title='Request marker = send seen by runtime')
    axes[1].set(xlabel='Host monotonic seconds after A0',title='Host clock; no mixed-clock latency')
    y=[-1 if not c or c=='None' else int(c.split('_')[-1]) for c in data['active_chunk']]
    axes[2].step(data['execution_sim_s'],y,where='post',color='#202a32');axes[2].set(yticks=[-1,0,1,2,3],yticklabels=['none','C0','C1','C2','C3'],xlabel='Simulation time [s]',title='Actual active reference')
    fig.tight_layout();finish(fig,FIGURES[2])
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for i,(ax,r) in enumerate(zip(axes.flat,chunks)):
        ax.axis('off')
        if r['generated']:
            path=ep/r['observation']['path'];assert sha(path)==r['observation']['sha256']
            ax.imshow(Image.open(path));ax.set_title(f'C{i} | exact request RGB | cart pixels {r["cart_visible_pixels"]}\nobs sim {r["observation"]["capture_sim_time_s"]:.6f} s | A{i}={tuple(round(x,4) for x in r['A'])}')
        else:ax.text(.5,.5,f'C{i}: N/A — not requested',ha='center',va='center',transform=ax.transAxes)
    fig.suptitle('Genuine live inputs; annotations outside image; original JPEG bytes preserved');fig.tight_layout();finish(fig,FIGURES[3])
    save(run/'plot_inputs.json',data)
    save(out/'plot_inputs_manifest.json',dict(path=str(run/'plot_inputs.json'),sha256=sha(run/'plot_inputs.json')))
    save(out/'figure_manifest.json',dict(PNG_sha256=manifest,plot_inputs_sha256=sha(run/'plot_inputs.json'),report_script_sha256=sha(__file__)))


def validate_report(run,out):
    equal_record(inputs(run),read(run/'plot_inputs.json'))
    equal_record(compact(read(run/'validation.json')),read(out/'result_summary.json'))
    assert sorted(p.name for p in (out/'figures').glob('*.png'))==sorted(FIGURES)
    manifest=read(out/'figure_manifest.json')
    assert manifest['plot_inputs_sha256']==sha(run/'plot_inputs.json') and manifest['report_script_sha256']==sha(__file__)
    from PIL import Image
    for name,h in manifest['PNG_sha256'].items():
        assert sha(out/'figures'/name)==h
        with Image.open(out/'figures'/name) as im:im.verify()
    return dict(valid=True,exactly_four_PNGs=True,numeric_and_hash_parity=True,new_model_MPC_optimizer_calls=0)

def blocked_report(run,out):
    v=read(run/'attempt_validation.json'); assert not v['scientific']
    out.mkdir(exist_ok=True,parents=True); d=out/'figures';d.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for name in FIGURES:
        fig,ax=plt.subplots(figsize=(9,3));ax.axis('off')
        ax.text(.05,.85,'TECHNICAL_EXECUTION_BLOCKED\nC0–C3 scientific panels: N/A\n'+str(v['scientific_validator_error']),va='top',wrap=True)
        fig.savefig(d/name,dpi=140,bbox_inches='tight');plt.close(fig)
    save(out/'result_summary.json',{k:x for k,x in v.items() if k!='scientific'})
    save(out/'figure_manifest.json',dict(PNG_sha256={n:sha(d/n) for n in FIGURES},technical_only=True))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate',action='store_true')
    a=p.parse_args()
    from run_continuous_obstacle_reveal_exploratory02 import namespace,OUT
    run=namespace(a.run);v=read(run/'attempt_validation.json')
    if a.validate:
        if v['scientific']: print(validate_report(run,OUT))
        else:
            assert sorted(p.name for p in (OUT/'figures').glob('*.png'))==sorted(FIGURES)
            for n,h in read(OUT/'figure_manifest.json')['PNG_sha256'].items(): assert sha(OUT/'figures'/n)==h
            print('technical N/A report hashes valid; zero scientific calls')
    elif v['scientific']:
        report(run,OUT);save(OUT/'report_validation.json',validate_report(run,OUT))
    else: blocked_report(run,OUT)
