#!/usr/bin/env python3
"""REF-03 saved transfer evidence: every frozen event, including failures/N/A."""
from __future__ import annotations
import argparse
import hashlib
import html
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diagnostics import file_sha256
from reconciliation.gp_se2_ref03_evaluation import METHODS,STRATA,PATTERNS,OVERLAPPING_FLAGS
from reconciliation.se2 import wrap_angle
from plot_gp_se2_02 import base_xy,environment_path,past_execution,plain,read,write
from plot_gp_se2_ref02 import square_bounds,wrapped_curve_segments

LABEL='OFFLINE SOURCE-PROGRESS TRANSFER DIAGNOSTIC'
PLOT_NAMES=('reference_and_rollout_world','boundary_zoom','goal_yaw_and_position_error',
            'linear_command_vs_time','angular_command_vs_time','clearance_vs_time','selected_progress_and_goal_inclusion')
AGGREGATE_PLOT_NAMES=('strata_success_patterns','regressions_and_recoveries','paired_successful_quality','cohort_coverage')
COLORS=dict(zip(METHODS,('#333333','#d55e00','#0072b2')))
STYLES=dict(zip(METHODS,('-','--','-.')))
SHORT=dict(zip(METHODS,('A Native','B Dense / row step','C Dense / source progress')))
QUALITY_METRICS=('time_to_goal_s','terminal_position_error_m','terminal_yaw_error_rad',
                 'command_total_variation_v_mps','command_total_variation_omega_radps','minimum_clearance_m')


def load_methods(case):
    methods={}
    for name in METHODS:
        folder=Path(case)/'methods'/name
        metrics=read(folder/'metrics.json')
        methods[name]=dict(folder=folder,reference=np.load(folder/'reference_world.npy',allow_pickle=False),
            provenance=read(folder/'row_provenance.json'),metrics=metrics,
            rollout=read(folder/'rollout.json') if (folder/'rollout.json').exists() else None)
    identity=input_identity(methods)
    if not all(identity[key] for key in ('byte_identical','array_identical','lineage_identical')):
        raise ValueError('B/C source geometry or lineage differs')
    return methods


def input_identity(methods):
    b,c=[methods[m] for m in METHODS[1:]]
    paths=[r['folder']/'reference_world.npy' for r in (b,c)]
    return dict(file_sha256=dict(zip(METHODS[1:],map(file_sha256,paths))),
        byte_identical=paths[0].read_bytes()==paths[1].read_bytes(),
        array_identical=bool(b['reference'].shape==c['reference'].shape and b['reference'].dtype==c['reference'].dtype and np.array_equal(b['reference'],c['reference'])),
        lineage_identical=b['provenance']==c['provenance'],shared_curve_drawn_once=True,spatial_offsets=False)


def gate_geometry(route):
    result=[]
    for gate in route['gates']:
        center=np.asarray(gate['center_xy']);normal=np.asarray(gate['normal_xy']);tangent=np.array([-normal[1],normal[0]])
        half=gate['half_width_m']
        result.append(dict(gate_id=gate['gate_id'],center_world_xy=center.tolist(),normal_world_xy=normal.tolist(),
            segment_world_xy=[(center-half*tangent).tolist(),(center+half*tangent).tolist()],
            original_gate_definition=gate))
    return result


def expected_numeric(context,route,methods,past,config):
    """Exact saved quantities behind seven plots; missing results stay null."""
    gates=gate_geometry(route)
    geometry=dict(old_world=context['old_world'],fresh_world=context['fresh_world'],actual_past_world=past['poses_world'],
        actual_past_times_relative_to_B_s=past['times_relative_to_B_s'],B_world=context['B_world'],goal_world=route['goal_world'],
        original_route_status=route['route_status'],required_gates=gates,methods={})
    all_xy=[geometry[k] for k in ('old_world','fresh_world','actual_past_world')]+[[geometry['B_world'],geometry['goal_world']]]
    zoom_xy=[[geometry['B_world'],geometry['goal_world']]]
    outcome,commands,clearance,selections={},{},{},{}
    for name,item in methods.items():
        roll=item['rollout'];metrics=item['metrics'];summary=metrics['summary']
        states=[] if roll is None else roll.get('states',[])
        solves=[] if roll is None else roll.get('controller_reference_selections',[])
        poses=[s['pose_world'] for s in states];times=[s['time_s'] for s in states]
        reference=item['reference'].tolist()
        geometry['methods'][name]=dict(reference_world=reference,row_provenance=item['provenance'],
            actual_poses_world=poses or None,actual_times_s=times or None,completed=summary['completed'],
            original_success=summary['primary_success'],missing_reason=summary.get('missing_reason'),failure_reasons=summary.get('failure_reasons'))
        all_xy.append(reference);zoom_xy.append(reference)
        if poses:all_xy.append(poses);zoom_xy.append(poses)
        if metrics.get('dense_poses_world') is not None:
            dense=np.asarray(metrics['dense_poses_world']);dt=metrics['dense_times_s'];goal=np.asarray(route['goal_world'])
            yaw=wrap_angle(dense[:,2]-goal[2])
            outcome[name]=dict(times_s=dt,actual_wrapped_yaw_rad=dense[:,2].tolist(),
                actual_display_unwrapped_yaw_rad=np.unwrap(dense[:,2]).tolist(),goal_relative_wrapped_yaw_error_rad=yaw.tolist(),
                wrapped_error_display_segments=wrapped_curve_segments(dt,yaw),
                position_error_m=np.linalg.norm(dense[:,:2]-goal[:2],axis=1).tolist(),summary=summary)
            clearance[name]=dict(times_s=dt,clearance_m=metrics['environment']['clearance_samples_m'])
        else:outcome[name]=dict(times_s=None,summary=summary);clearance[name]=None
        if solves:
            # A hold extends only through the saved integration endpoint, never beyond evidence.
            end=times[-1] if times else solves[-1]['time_s']
            commands[name]=dict(times_s=[s['time_s'] for s in solves]+[end],
                applied=[s['command'] for s in solves]+[solves[-1]['command']],
                physical_u_minus=context['u_minus'],controller_previous_control=context['previous_control'])
            ds=[s['selection_diagnostic'] for s in solves]
            selections[name]=dict(times_s=[s['time_s'] for s in solves],nearest_original_progress=[d['nearest_original_fractional_row_coordinate'] for d in ds],
                selected_original_progress=[d['selected_original_fractional_row_coordinates'] for d in ds],
                final_goal_in_horizon=[d['final_goal_row_in_horizon'] for d in ds],requested_q_h=[d.get('q_h') for d in ds],
                progress_overshoot=[d.get('progress_overshoot') for d in ds],endpoint_repeated=[d['endpoint_repeated'] for d in ds],
                actual_sequential_unwrapped_reference_yaw_rad=[d['official_reference_unwrapped_yaw'] for d in ds])
        else:commands[name]=None;selections[name]=None
    all_xy += [g['segment_world_xy'] for g in gates]
    geometry['axes_world_m']=square_bounds(all_xy,padding=.35,minimum_span=1.)
    geometry['boundary_axes_world_m']=square_bounds(zoom_xy,padding=.25,minimum_span=.8)
    return dict(reference_and_rollout_world=geometry,boundary_zoom=geometry,
        goal_yaw_and_position_error=dict(methods=outcome,position_tolerance_m=config['formulation']['goal_position_tolerance'],
            yaw_tolerance_rad=config['formulation']['goal_yaw_tolerance'],required_terminal_dwell_s=config['evaluation']['terminal_goal_dwell_s']),
        linear_command_vs_time=dict(methods=commands,component=0),angular_command_vs_time=dict(methods=commands,component=1),
        clearance_vs_time=dict(methods=clearance,required_clearance_m=config['footprint']['required_clearance_m']),
        selected_progress_and_goal_inclusion=dict(methods=selections,units='original source row order, not time or distance'))


def representative_cases(manifest_rows,patterns,regressions):
    """Frozen all-regressions/controls plus hash-first recovery/unchanged policy."""
    by={r['case_id']:r for r in manifest_rows};events={r['case_id']:r for r in patterns};reasons={}
    order=lambda case:(hashlib.sha256(('REF03-v1:'+case).encode()).hexdigest(),case)
    priority=[]
    for row in regressions:
        if row['method']==METHODS[2] and row['baseline'] in METHODS[:2] and row.get('comparable') and (row.get('new_safety_reasons') or row.get('new_route_reasons')):
            case=row['case_id'];reasons.setdefault(case,[]).append('NEW_C_SAFETY_OR_ROUTE_VS_'+row['baseline'])
    priority.extend(sorted(reasons,key=order))
    for stratum in STRATA:
        candidates=[c for c,r in by.items() if r['cohort']=='ADDITIONAL_TRANSFER' and r['stratum']==stratum]
        recovered=[c for c in candidates if events[c]['overlapping_flags']['B_failure_C_success'] is True]
        unchanged=[c for c in candidates if events[c]['success_by_method'][METHODS[1]] is not None
                   and events[c]['success_by_method'][METHODS[1]]==events[c]['success_by_method'][METHODS[2]]]
        for pool,label in ((recovered,'FIRST_HASH_B_FAILURE_C_SUCCESS'),(unchanged,'FIRST_HASH_SAME_B_C_SUCCESS_STATUS')):
            if pool:
                case=min(pool,key=order);reasons.setdefault(case,[]).append(label);priority.append(case)
    for row in manifest_rows:
        if row['cohort']=='KNOWN_CONTROLS':
            reasons.setdefault(row['case_id'],[]).append('KNOWN_CONTROL_ALWAYS');priority.append(row['case_id'])
    seen=set();selected=[]
    for case in priority:
        if case not in seen:
            selected.append(dict(case_id=case,relative_directory=by[case]['relative_directory'],reasons=reasons[case],selection_hash=order(case)[0]))
            seen.add(case)
    return dict(selected=selected,count=len(selected),policy='all new C safety/route regressions first; hash-first B-failure/C-success and same B/C success status per transfer stratum; all controls',
        unchanged_means='same completed B/C original-success status, not identical path or commands',full_event_index_retained=True)


def expected_aggregate_numeric(summary,manifest_rows,coverage,patterns,regressions,paired_quality):
    strata=summary['transfer_strata']
    patterns_data=dict(transfer_strata=strata,known_controls=summary['known_controls'],known_control_by_role=summary['known_control_by_role'],
                       pattern_order=list(PATTERNS)+['UNAVAILABLE'],controls_not_pooled=True)
    flags=dict(transfer_strata={s:strata[s]['overlapping_flags'] for s in STRATA},known_controls=summary['known_controls']['overlapping_flags'],
               reason_comparisons=regressions,flags=list(OVERLAPPING_FLAGS),overlapping_counts_must_not_be_added=True)
    quality=dict(metrics=list(QUALITY_METRICS),rows=[r for r in paired_quality if r['cohort']=='ADDITIONAL_TRANSFER' and r['baseline']==METHODS[1] and r['method']==METHODS[2] and r['metric'] in QUALITY_METRICS],
                 scope='B/C pairs that both pass original full success only; unavailable/failed pairs excluded with explicit counts')
    coverage_data=dict(coverage=coverage,manifest_grouping=[{k:r.get(k) for k in ('case_id','cohort','stratum','episode_id','ordered_raw_pair','prior_selected_in','other_known_source_usage')} for r in manifest_rows],
                       additional_duplicate_weighting=summary['additional_transfer']['method_outcomes'],known_controls_separate=True)
    return dict(strata_success_patterns=patterns_data,regressions_and_recoveries=flags,paired_successful_quality=quality,cohort_coverage=coverage_data)


def _time(ax,ylabel):
    ax.set(xlim=(0.,3.),xlabel='simulation time after original B [s]',ylabel=ylabel);ax.grid(alpha=.2)


def _gates(ax,gates):
    for gate in gates:
        line=np.asarray(gate['segment_world_xy']);center=np.asarray(gate['center_world_xy']);normal=np.asarray(gate['normal_world_xy'])
        ax.plot(line[:,0],line[:,1],':',color='#a65628',lw=2,label='required gate '+gate['gate_id'])
        ax.arrow(*center,*(normal*.18),head_width=.04,length_includes_head=True,color='#a65628',zorder=8)


def _save(fig,target,numeric,sources,title,identity=None):
    target=Path(target)
    if target.exists() or target.with_suffix('.json').exists():raise FileExistsError('refusing plot overwrite')
    fig.suptitle(title,fontsize=13)
    fig.text(.012,.012,LABEL+' | saved results | original acceptance | source/numbers in adjacent JSON',fontsize=8)
    fig.tight_layout(rect=(0,.045,1,.93));fig.savefig(target,dpi=130);plt.close(fig)
    write(target.with_suffix('.json'),dict(image=target.name,image_sha256=file_sha256(target),label=LABEL,numeric_data=plain(numeric),
        input_identity=identity,source_hashes={str(Path(p).resolve()):file_sha256(p) for p in sources},plotter_sha256=file_sha256(__file__),
        new_inference=False,new_optimization=False,new_execution=False,gui_runtime_validated=False,
        primary_time_range_s=[0.,3.],missing_values='null; no fabricated command or execution trace'))
    return dict(path=str(target),sha256=file_sha256(target),sidecar_sha256=file_sha256(target.with_suffix('.json')))


def plot_case(run,row,env,config):
    case=run/row['relative_directory'];out=case/'plots'
    if out.exists():raise FileExistsError('refusing case plot directory overwrite')
    methods=load_methods(case);context=read(case/'input_context.json');route=read(case/'goal_route.json');past=past_execution(case)
    data=expected_numeric(context,route,methods,past,config);identity=input_identity(methods);xy=data['reference_and_rollout_world']
    sources=[run/p for p in ('source.json','protocol.json','config_snapshot.yaml','case_manifest.json','cohort_coverage.json')]
    sources += [case/p for p in ('input_context.json','goal_route.json','actual_past_execution.json')]
    for item in methods.values():
        sources += [item['folder']/p for p in ('reference_world.npy','row_provenance.json','metrics.json')]
        if (item['folder']/'rollout.json').exists():sources.append(item['folder']/'rollout.json')
    out.mkdir();images=[]
    def save(name,fig,title):images.append(_save(fig,out/(name+'.png'),data[name],sources,row['case_id']+' | '+title,identity))
    def draw(ax,bounds,references=True,rollouts=True):
        base_xy(ax,xy,env,bounds,config);_gates(ax,xy['required_gates'])
        if references:
            for name,label,color in ((METHODS[0],'A native reference','#555555'),(METHODS[1],'B/C same dense reference','#984ea3')):
                p=methods[name]['reference'];ax.plot(p[:,0],p[:,1],':',color=color,lw=2,marker='.',ms=3,label=label)
        if rollouts:
            for name,item in xy['methods'].items():
                if item['actual_poses_world'] is not None:
                    a=np.asarray(item['actual_poses_world']);ax.plot(a[:,0],a[:,1],STYLES[name],color=COLORS[name],lw=1.9,label=SHORT[name]+' actual')
                else:ax.plot([],[],color=COLORS[name],label=SHORT[name]+' N/A')
    fig,axes=plt.subplots(1,2,figsize=(16,7))
    draw(axes[0],xy['axes_world_m'],rollouts=False);draw(axes[1],xy['axes_world_m'])
    axes[0].set_title('Frozen input paths');axes[1].set_title('New independent actual rollouts')
    for ax in axes:ax.legend(fontsize=7,loc='best')
    save('reference_and_rollout_world',fig,'Shared dense input and actual execution')
    fig,ax=plt.subplots(figsize=(9,8));draw(ax,xy['boundary_axes_world_m']);ax.legend(fontsize=8,loc='best')
    save('boundary_zoom',fig,'Boundary/goal region; same geometry and scale for all methods')
    fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True)
    for name,record in data['goal_yaw_and_position_error']['methods'].items():
        if record['times_s'] is None:
            for ax in axes:ax.plot([],[],color=COLORS[name],label=SHORT[name]+' N/A')
            continue
        axes[0].plot(record['times_s'],np.rad2deg(record['actual_display_unwrapped_yaw_rad']),STYLES[name],color=COLORS[name],label=SHORT[name])
        for segment in record['wrapped_error_display_segments']:
            axes[1].plot(segment['times_s'],np.rad2deg(segment['angles_rad']),STYLES[name],color=COLORS[name])
        axes[2].plot(record['times_s'],record['position_error_m'],STYLES[name],color=COLORS[name])
    axes[1].axhline(np.rad2deg(config['formulation']['goal_yaw_tolerance']),color='red',ls=':');axes[1].axhline(-np.rad2deg(config['formulation']['goal_yaw_tolerance']),color='red',ls=':')
    axes[2].axhline(config['formulation']['goal_position_tolerance'],color='red',ls=':')
    for ax,label in zip(axes,('actual yaw, display unwrapped [deg]','original-goal wrapped yaw error [deg]','original-goal position error [m]')):_time(ax,label)
    axes[0].legend(fontsize=8,ncol=3);save('goal_yaw_and_position_error',fig,'Actual yaw and original-goal errors; unchanged terminal dwell')
    for component,name,label in ((0,'linear_command_vs_time','applied linear command [m/s]'),(1,'angular_command_vs_time','applied angular command [rad/s]')):
        fig,ax=plt.subplots(figsize=(11,5))
        for method,record in data[name]['methods'].items():
            if record is not None:ax.step(record['times_s'],np.asarray(record['applied'])[:,component],where='post',ls=STYLES[method],color=COLORS[method],label=SHORT[method])
            else:ax.plot([],[],color=COLORS[method],label=SHORT[method]+' N/A')
        ax.scatter([0],[context['u_minus'][component]],marker='o',facecolors='none',edgecolors='black',label='physical u_minus')
        ax.scatter([0],[context['previous_control'][component]],marker='x',color='#984ea3',label='controller memory')
        _time(ax,label);ax.legend(fontsize=8,ncol=3);save(name,fig,'Applied commands; memory and physical start kept distinct')
    fig,ax=plt.subplots(figsize=(11,5))
    for name,record in data['clearance_vs_time']['methods'].items():
        if record is not None:ax.plot(record['times_s'],record['clearance_m'],STYLES[name],color=COLORS[name],label=SHORT[name])
        else:ax.plot([],[],color=COLORS[name],label=SHORT[name]+' N/A')
    ax.axhline(config['footprint']['required_clearance_m'],color='red',ls=':',label='required edge clearance');ax.axhline(0.,color='#555555',lw=.8)
    _time(ax,'actual footprint edge clearance [m]');ax.legend(fontsize=8,ncol=4);save('clearance_vs_time',fig,'Original environment, footprint and clearance')
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for name,record in data['selected_progress_and_goal_inclusion']['methods'].items():
        if record is None:
            axes[0].plot([],[],color=COLORS[name],label=SHORT[name]+' N/A');continue
        s=np.asarray(record['selected_original_progress']);ts=record['times_s']
        axes[0].plot(ts,s[:,0],':',color=COLORS[name],alpha=.8,label=SHORT[name]+' first')
        axes[0].plot(ts,s[:,-1],STYLES[name],color=COLORS[name],label=SHORT[name]+' last')
        axes[1].step(ts,np.asarray(record['final_goal_in_horizon'],dtype=float),where='post',ls=STYLES[name],color=COLORS[name],label=SHORT[name])
    _time(axes[0],'selected original-row progress');_time(axes[1],'final source goal in horizon')
    axes[1].set(yticks=[0,1],yticklabels=['absent','included'],ylim=(-.12,1.12))
    axes[0].legend(fontsize=8,ncol=3);axes[1].legend(fontsize=8,ncol=3)
    save('selected_progress_and_goal_inclusion',fig,'Actual controller target progress and final-goal selection')
    return images


def plot_aggregate(run,manifest,summary,coverage,patterns,regressions,quality):
    out=run/'aggregate/plots'
    if out.exists():raise FileExistsError('refusing aggregate plot directory overwrite')
    out.mkdir();values=expected_aggregate_numeric(summary,manifest,coverage,patterns,regressions,quality)
    sources=[run/p for p in ('source.json','protocol.json','config_snapshot.yaml','case_manifest.json','cohort_coverage.json',
        'aggregate/summary.json','aggregate/success_patterns.json','aggregate/regressions.json','aggregate/paired_metrics.json')]
    images=[]
    def save(name,fig,title):images.append(_save(fig,out/(name+'.png'),values[name],sources,title))
    fig,axes=plt.subplots(1,2,figsize=(17,7));scopes=[summary['transfer_strata'][s] for s in STRATA]
    x=np.arange(4)
    for offset,method in enumerate(METHODS):
        counts=[s['method_outcomes'][method]['successes'] for s in scopes]
        axes[0].bar(x+(offset-1)*.25,counts,.25,color=COLORS[method],label=SHORT[method])
        for i,scope in enumerate(scopes):
            m=scope['method_outcomes'][method];axes[0].text(x[i]+(offset-1)*.25,counts[i]+.06,f"{counts[i]}/{m['completed']}",ha='center',fontsize=8)
    axes[0].set(xticks=x,xticklabels=['O','R','P','S'],ylabel='original full-success count',ylim=(0,max(7,max(s['event_count'] for s in scopes)+1)))
    axes[0].legend(fontsize=8);axes[0].grid(axis='y',alpha=.2)
    matrix=np.array([[scope['exclusive_patterns'][p] for p in (*PATTERNS,'UNAVAILABLE')] for scope in scopes])
    axes[1].imshow(matrix,cmap='Blues',aspect='auto');axes[1].set(yticks=x,yticklabels=['O','R','P','S'],xticks=np.arange(9),xticklabels=[*PATTERNS,'N/A'])
    axes[1].set_title('Exclusive patterns A/B/C; 1 = original success')
    for i in range(4):
        for j in range(9):axes[1].text(j,i,str(matrix[i,j]),ha='center',va='center',color='black')
    save('strata_success_patterns',fig,'Additional transfer strata; known controls are reported separately in tables')
    fig,axes=plt.subplots(2,1,figsize=(15,10),gridspec_kw={'height_ratios':[2,1]})
    labels=list(OVERLAPPING_FLAGS);matrix=np.array([[s['overlapping_flags'][k]['count'] for k in labels] for s in scopes])
    flag_labels=['B pass\nC fail','A pass\nC fail','A pass / B fail\nC pass','B fail\nC pass','A and B fail\nC pass','A, B and C\nall fail']
    axes[0].imshow(matrix,cmap='Oranges',aspect='auto');axes[0].set(yticks=x,yticklabels=['O','R','P','S'],xticks=np.arange(len(labels)),xticklabels=flag_labels)
    for i in range(4):
        for j in range(len(labels)):axes[0].text(j,i,str(matrix[i,j]),ha='center',va='center')
    axes[0].set_title('Overlapping event flags: these columns must not be added')
    rows=[]
    for s in STRATA:
        selected=[r for r in regressions if r['cohort']=='ADDITIONAL_TRANSFER' and r['stratum']==s and r['method']==METHODS[2]]
        rows.append([s[0]]+[str(len({r['case_id'] for r in selected if r.get(key)})) for key in ('new_safety_reasons','new_motion_reasons','new_route_reasons','new_goal_reasons')])
    axes[1].axis('off');table=axes[1].table(cellText=rows,colLabels=['Stratum','new safety','new motion','new route','new goal'],cellLoc='center',loc='center');table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.7)
    axes[1].set_title('New C failure reasons relative to A or B; distinct events within each column only')
    save('regressions_and_recoveries',fig,'Regressions and recoveries under original predicates; no combined score')
    fig,axes=plt.subplots(2,3,figsize=(16,9))
    for ax,metric in zip(axes.flat,QUALITY_METRICS):
        for i,stratum in enumerate(STRATA):
            selected=[r for r in values['paired_successful_quality']['rows'] if r['stratum']==stratum and r['metric']==metric]
            observed=[r['difference'] for r in selected if r['difference'] is not None]
            if observed:ax.scatter([i]*len(observed),observed,color='#0072b2',s=25,alpha=.65)
            ax.text(i,.99,f'n={len(observed)}',ha='center',va='top',transform=ax.get_xaxis_transform(),fontsize=9)
        ax.axhline(0.,color='#555555',lw=.8);ax.set(xticks=x,xticklabels=['O','R','P','S'],title=metric,ylabel='C minus B, original metric units');ax.grid(alpha=.2)
    save('paired_successful_quality',fig,'Execution-quality differences only where B and C both fully succeed')
    fig,axes=plt.subplots(1,2,figsize=(16,7),gridspec_kw={'width_ratios':[1,1.3]})
    axes[0].bar(x,[coverage['group_candidate_counts'][s] for s in ('O','R','P','S')],color='#dddddd',label='eligible source pool')
    axes[0].bar(x,[coverage['group_selected_counts'][s] for s in ('O','R','P','S')],color='#0072b2',label='frozen selected')
    axes[0].set(xticks=x,xticklabels=['O','R','P','S'],ylabel='event count');axes[0].legend(fontsize=9);axes[0].grid(axis='y',alpha=.2)
    axes[1].axis('off');rows=[['original source events',coverage['source_event_count']],['original GP01 eligible',coverage['original_gp01_eligible_count']],
        ['prior selected exclusion union',coverage['historical_exclusion_union_count']],['additional selected',coverage['additional_selected_count']],
        ['known controls (separate)',coverage['controls_count']],['additional unique episodes',coverage['additional_unique_episode_count']],
        ['additional unique ordered raw pairs',coverage['additional_unique_ordered_raw_pair_count']]]
    table=axes[1].table(cellText=rows,colLabels=['Frozen coverage / dependence','Count'],cellLoc='left',loc='center',colWidths=[.78,.22]);table.auto_set_font_size(False);table.set_fontsize(11);table.scale(1.,2.1)
    save('cohort_coverage',fig,'Source-only selection coverage; dependent records are not new independent episodes')
    return images


def main(run):
    run=Path(run).resolve()
    if (run/'plot_manifest.json').exists() or (run/'index.html').exists():raise FileExistsError('refusing plot/index overwrite')
    rows=read(run/'case_manifest.json')['selected'];summary=read(run/'aggregate/summary.json');coverage=read(run/'cohort_coverage.json')
    patterns=read(run/'aggregate/success_patterns.json');regressions=read(run/'aggregate/regressions.json');quality=read(run/'aggregate/paired_metrics.json')
    representatives=representative_cases(rows,patterns,regressions)
    config=yaml.safe_load((run/'config_snapshot.yaml').read_text());env=HospitalEnvironment.load(environment_path(run));images=[]
    for row in rows:images.extend(plot_case(run,row,env,config))
    images.extend(plot_aggregate(run,rows,summary,coverage,patterns,regressions,quality))
    for record in images:record['path']=str(Path(record['path']).relative_to(run))
    write(run/'representative_selection.json',representatives)
    write(run/'plot_manifest.json',dict(experiment='GP-SE2-REF-03',images=images,image_count=len(images),case_count=len(rows),
        per_case_image_count=7,aggregate_image_count=4,case_plot_names=list(PLOT_NAMES),aggregate_plot_names=list(AGGREGATE_PLOT_NAMES),
        gui_runtime_validated=False,plotter_sha256=file_sha256(__file__)))
    lines=['<!doctype html><html><head><meta charset="utf-8"><title>REF-03 transfer</title><style>body{font:16px system-ui;max-width:1550px;margin:25px auto;padding:18px}img{max-width:100%}article{margin:30px 0}table{border-collapse:collapse}td,th{padding:5px;border:1px solid #ccc}</style></head><body>',
        '<h1>'+LABEL+'</h1><p>Same fixed dense geometry, unchanged MPC calculation, different target stride. Controls and additional transfer strata remain separate. All events, failures and unavailable computation are retained. No online, held-out or general navigation claim.</p>',
        '<p><a href="aggregate/outcome_matrix.csv">Every method outcome</a> · <a href="aggregate/summary.json">Detailed summaries</a> · <a href="cohort_coverage.json">Coverage</a> · <a href="representative_selection.json">Frozen representative policy</a></p>']
    for name in AGGREGATE_PLOT_NAMES:lines.append(f'<article><h2>{name.replace("_"," ")}</h2><img src="aggregate/plots/{name}.png"><p><a href="aggregate/plots/{name}.json">Exact values and provenance</a></p></article>')
    reps={r['case_id']:r['reasons'] for r in representatives['selected']}
    for cohort in ('KNOWN_CONTROLS','ADDITIONAL_TRANSFER'):
        lines.append('<h2>'+cohort.replace('_',' ')+'</h2>')
        for row in rows:
            if row['cohort']!=cohort:continue
            base=row['relative_directory'];lines.append(f'<h3>{html.escape(row["case_id"])} · {html.escape(row["case_role"])}</h3>')
            if row['case_id'] in reps:lines.append('<p>Representative: '+html.escape('; '.join(reps[row['case_id']]))+'</p>')
            lines.append('<p>'+' · '.join(f'<a href="{base}/plots/{name}.png">{name.replace("_"," ")}</a>' for name in PLOT_NAMES)+'</p>')
            lines.append(f'<img loading="lazy" src="{base}/plots/reference_and_rollout_world.png">')
    (run/'index.html').write_text('\n'.join(lines+['</body></html>']))
    return dict(case_count=len(rows),image_count=len(images),representative_count=representatives['count'],gui_runtime_validated=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',required=True,type=Path)
    print(json.dumps(main(parser.parse_args().run),indent=2))
