#!/usr/bin/env python3
"""Saved-only continuous source figures; actual execution colored by active command."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.long_continuous_obstacle_reveal_source01 import figure_names,FIGURES,OPTIONAL_FIGURE
from report_continuous_obstacle_reveal_exploratory02 import inputs as old_inputs, compact as old_compact
from analyze_join_online02 import csvread
from validate_robotless_online_handoffs import equal_record
from run_long_continuous_obstacle_reveal_source01 import namespace,OUT


def inputs(run):
    data=old_inputs(run);ep=Path(data['validation']['episode'])
    data.update(commands=csvread(ep/'commands.csv'),commands_sha256=sha(ep/'commands.csv'))
    return data


def compact(v):
    result=old_compact(v)
    for k in ('consecutive_applied_count','longest_applied_prefix','usable_applied_handoffs',
              'usable_post_reveal_handoffs','speed_intervention','speed_audit','provenance_valid',
              'minimum_guard_lookahead_clearance_lower_bound_m','raw_unsafe_abort','safety_abort'):
        result[k]=v[k]
    for row,original in zip(result['chunks'],v['chunks']):
        for k in ('inference_active_chunk','inference_verified_interval_ids','intrinsic_waypoint_dt','history_ref','response_ref'):
            row[k]=original.get(k)
    return result


def segments(commands):
    """Command i moves state i to i+1; state-row identity lags a switch by one row."""
    start=0
    for i in range(1,len(commands)+1):
        if i==len(commands) or commands[i]['chunk_id']!=commands[start]['chunk_id']:
            yield commands[start]['chunk_id'],start,i
            start=i


def status(row):
    if row['applied']:return 'APPLIED'
    if row.get('stop'):return 'STOP / NOT EXECUTED'
    if row.get('raw_reference_safe') is False:return 'REJECTED / NOT EXECUTED'
    return 'GENERATED / NOT EXECUTED'


def report(run,out):
    data=inputs(run);v=data['validation'];ep=Path(v['episode'])
    out.mkdir(parents=True,exist_ok=True);figures=out/'figures';figures.mkdir(exist_ok=False)
    save(out/'result_summary.json',compact(v))
    generated=[r for r in v['chunks'] if r['generated']]
    rows=[dict(chunk=r['chunk_id'],status=status(r),A=r['A'],B=r.get('B'),
        observation_sim_s=r['observation']['capture_sim_time_s'],
        ready_sim_s=(r.get('t_ready_seen_sim') or {}).get('sim_time_s'),
        install_sim_s=(r.get('t_install') or {}).get('sim_time_s'),
        application_sim_s=(r.get('t_application') or {}).get('sim_time_s'),
        cart_pixels=r['cart_visible_pixels'],N=r.get('N'),arc_m=r.get('arc_m'),
        raw_sha256=(r.get('raw_local_ref') or {}).get('sha256'),
        clearance_m=r.get('physical_footprint_clearance_m'),legacy_5cm_pass=r.get('legacy_5cm_margin_pass'),
        inference_active_chunk=r.get('inference_active_chunk')) for r in generated]
    with (out/'chunks.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
    save(out/'evolution.json',v['evolution'])
    save(out/'handoff_index.json',read(run/'source_bundle/handoff_index.json'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    import shapely
    from PIL import Image
    plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':.18})
    colors=list(plt.get_cmap('tab10').colors);marks=['o','s','^','d','v','p','h','>','<','X']
    actual=np.asarray(data['execution_world']);times=np.asarray(data['execution_sim_s'])
    cart=shapely.from_wkb(bytes.fromhex(data['cart_wkb_hex']));manifest={}
    def finish(fig,name):
        fig.savefig(figures/name,dpi=160,bbox_inches='tight');plt.close(fig);manifest[name]=sha(figures/name)
    def obstacle(ax):
        for geom,label,style in [(cart,'Cart','-'),(cart.buffer(.20),'0.20 m footprint','-'),(cart.buffer(.25),'Legacy +0.05 m','--')]:
            for j,g in enumerate(list(geom.geoms) if hasattr(geom,'geoms') else [geom]):
                if geom is cart:ax.fill(*g.exterior.xy,color='#826a51',alpha=.4,label=label if j==0 else None)
                else:ax.plot(*g.exterior.xy,color='#826a51',ls=style,lw=1,label=label if j==0 else None)
    fig,ax=plt.subplots(figsize=(11,7));obstacle(ax)
    for i,r in enumerate(generated):
        if r.get('world'):
            a=np.asarray(r['world']);ax.plot(a[:,0],a[:,1],ls='--',marker=marks[i],ms=3,lw=1,color=colors[i],
                label=f'C{i} raw'+(' — '+status(r) if not r['applied'] else ''))
        ax.scatter(*r['A'][:2],marker='x',s=45,color=colors[i]);ax.annotate(f'A{i}',r['A'][:2],xytext=(-23,8),textcoords='offset points',color=colors[i],fontsize=8)
        if r['applied']:
            ax.scatter(*r['B'][:2],marker='*',s=105,color=colors[i],zorder=9)
            ax.annotate(f'B{i}',r['B'][:2],xytext=(7,-8),textcoords='offset points',color=colors[i],fontsize=8)
    for cid,start,end in segments(data['commands']):
        if not cid or cid=='None':continue
        i=int(cid.split('_')[-1]);a=actual[start:end+1]
        ax.plot(a[:,0],a[:,1],color=colors[i],lw=3,marker=marks[i],markevery=max(1,len(a)//4),ms=4,zorder=7,label=f'Actual under C{i}')
    ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]',title='Continuous native execution — solid colors identify the active chunk\nDashed: immutable observation-anchored raw references; stars: actual application')
    ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=8,ncol=1);fig.tight_layout();finish(fig,FIGURES[0])
    fig,axes=plt.subplots(1,2,figsize=(11,5))
    for i,r in enumerate(generated):
        if not r.get('raw_local'):continue
        a=np.asarray(r['raw_local']);arc=np.r_[0,np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1).cumsum()]
        axes[0].plot(a[:,0],a[:,1],marker=marks[i],ms=3,color=colors[i],label=f'C{i}')
        axes[1].plot(arc,np.degrees(a[:,2]),marker=marks[i],ms=3,color=colors[i],label=f'C{i}')
    axes[0].set_aspect('equal');axes[0].set(xlabel='Own observation-local forward X [m]',ylabel='Left Y [m]',title='All generated own-observation raw chunks')
    axes[1].set(xlabel='Raw XY arc [m] — untimed rows',ylabel='Local yaw [deg]',title='Spatial pose yaw; not angular velocity')
    for ax in axes:ax.legend(ncol=2,fontsize=8)
    fig.tight_layout();finish(fig,FIGURES[1])
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,gridspec_kw={'height_ratios':[1.6,1]})
    event_marks={'obs':'o','send seen':'>','ready seen':'s','install':'D','apply':'*'}
    for i,r in enumerate(generated):
        sent=next((q for q in data['request_seen'] if q.get('chunk_id')==r['chunk_id']),None)
        send=None if sent is None else sent['seen_in_isaac']['sim_time_s']
        ready=(r.get('t_ready_seen_sim') or {}).get('sim_time_s')
        events={'obs':r['observation']['capture_sim_time_s'],'send seen':send,'ready seen':ready,
            'install':(r.get('t_install') or {}).get('sim_time_s'),'apply':(r.get('t_application') or {}).get('sim_time_s')}
        if send is not None and ready is not None:axes[0].plot([send,ready],[i,i],color=colors[i],lw=6,alpha=.22)
        # Small vertical offsets distinguish coincident receipt/install without changing time.
        for j,(name,t) in enumerate(events.items()):
            if t is not None:axes[0].scatter(t,i+(j-2)*.07,marker=event_marks[name],s=45,color=colors[i],zorder=5)
        if not r['applied']:axes[0].text(times[-1]+.04,i,status(r),va='center',fontsize=8)
    handles=[Line2D([],[],marker=m,ls='',color='#34424e',label=k) for k,m in event_marks.items()]
    axes[0].legend(handles=handles,ncol=5,loc='upper left',bbox_to_anchor=(0,1.18),fontsize=8)
    axes[0].set(yticks=range(len(generated)),yticklabels=[f'C{i}' for i in range(len(generated))],ylim=(len(generated)-.5,-.5),title='Shaded interval: request-send seen → ready seen in simulation; exact host RTT in numeric sidecar')
    for cid,start,end in segments(data['commands']):
        i=-1 if not cid or cid=='None' else int(cid.split('_')[-1])
        axes[1].plot([times[start],times[end]],[i,i],color='#888' if i<0 else colors[i],lw=5)
        if start:axes[1].plot([times[start]]*2,[previous,i],color='#aaa',lw=1)
        previous=i
    axes[1].set(yticks=range(-1,len(generated)),yticklabels=['none']+[f'C{i}' for i in range(len(generated))],xlabel='Simulation time [s]',ylabel='Actual command reference',title='OLD remains active during each FRESH inference; application defines the step')
    if v['reveal']:
        for ax in axes:ax.axvline(v['reveal']['start']['sim_time_s'],color='#826a51',ls=':',lw=1)
    fig.tight_layout();finish(fig,FIGURES[2])
    nrows=(len(generated)+1)//2
    fig=plt.figure(figsize=(12,3.9*nrows),layout='constrained');grid=fig.add_gridspec(2*nrows,2,height_ratios=[.28,1]*nrows)
    for i,r in enumerate(generated):
        title=fig.add_subplot(grid[2*(i//2),i%2]);title.axis('off')
        ax=fig.add_subplot(grid[2*(i//2)+1,i%2]);ax.axis('off')
        title.text(.5,.5,f'C{i} | {status(r)} | cart pixels {r["cart_visible_pixels"]}\nobs sim {r["observation"]["capture_sim_time_s"]:.6f} s\nA{i} = ({r["A"][0]:.6f}, {r["A"][1]:.6f}, {r["A"][2]:.6f})',ha='center',va='center',fontsize=10)
        path=ep/r['observation']['path'];assert sha(path)==r['observation']['sha256']
        with Image.open(path) as im:ax.imshow(im)
    fig.suptitle('Exact terminal-request RGBs — metadata outside images; original JPEG bytes preserved');finish(fig,FIGURES[3])
    applied=[r for r in generated if r['applied']]
    if len(applied)>=6:
        n=len(applied)-1;fig,axes=plt.subplots((n+2)//3,3,figsize=(13,4*((n+2)//3)),squeeze=False)
        for i,ax in enumerate(axes.flat):
            if i>=n:ax.axis('off');continue
            old,fresh=applied[i:i+2];obstacle(ax)
            for row,ls in [(old,'--'),(fresh,':')]:
                a=np.asarray(row['world']);ax.plot(a[:,0],a[:,1],ls,color=colors[int(row['chunk_id'][-3:])],label=row['chunk_id'])
            a=actual[old['switch_state_id']:fresh['switch_state_id']+1]
            ax.plot(a[:,0],a[:,1],color=colors[i],lw=3,label='Actual OLD to B');ax.scatter(*fresh['B'][:2],marker='*',s=90,color=colors[i+1])
            ax.set_aspect('equal');ax.set(title=f'C{i} → C{i+1}',xlabel='World X [m]',ylabel='World Y [m]')
            ax.legend(fontsize=6)
        fig.suptitle('Saved applied-to-applied handoffs — descriptive source geometry only');fig.tight_layout();finish(fig,OPTIONAL_FIGURE)
    save(run/'plot_inputs.json',data)
    save(out/'plot_inputs_manifest.json',dict(path=str(run/'plot_inputs.json'),sha256=sha(run/'plot_inputs.json')))
    save(out/'figure_manifest.json',dict(PNG_sha256=manifest,plot_inputs_sha256=sha(run/'plot_inputs.json'),report_script_sha256=sha(__file__),
        actual_segment_identity='command i integrates state i to i+1',clock_display='simulation seen events; host send/receipt retained separately'))


def validate_report(run,out):
    equal_record(inputs(run),read(run/'plot_inputs.json'))
    v=read(run/'validation.json');equal_record(compact(v),read(out/'result_summary.json'))
    names=figure_names(v['consecutive_applied_count'])
    assert sorted(p.name for p in (out/'figures').glob('*.png'))==sorted(names)
    m=read(out/'figure_manifest.json');assert m['plot_inputs_sha256']==sha(run/'plot_inputs.json') and m['report_script_sha256']==sha(__file__)
    from PIL import Image
    for name,h in m['PNG_sha256'].items():
        assert sha(out/'figures'/name)==h
        with Image.open(out/'figures'/name) as im:im.verify()
    return dict(valid=True,PNG_count=len(names),numeric_and_hash_parity=True,new_scientific_calls=0)


def validate_blocked_report(out):
    assert sorted(p.name for p in (out/'figures').glob('*.png'))==sorted(FIGURES)
    for name,h in read(out/'figure_manifest.json')['PNG_sha256'].items():
        assert sha(out/'figures'/name)==h
    return dict(valid=True,technical_only=True,new_scientific_calls=0)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--validate',action='store_true')
    a=p.parse_args();run=namespace(a.run)
    if a.validate:
        print(validate_report(run,OUT) if read(run/'attempt_validation.json')['scientific'] else validate_blocked_report(OUT))
    elif read(run/'attempt_validation.json')['scientific']:
        report(run,OUT);save(OUT/'report_validation.json',validate_report(run,OUT))
    else:
        from report_continuous_obstacle_reveal_exploratory02 import blocked_report
        blocked_report(run,OUT)
