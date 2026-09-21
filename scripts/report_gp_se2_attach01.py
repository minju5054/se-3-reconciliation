#!/usr/bin/env python3
"""Reporting-only ATTACH-01 tables and compact saved-execution presentation.

Never changes frozen planning/selection/acceptance, reruns a solver, or replaces
the original plots. Adds a separate review directory with complete legends.
"""
import argparse
import html
import json
import os
from pathlib import Path
import sys
import zipfile
os.environ.setdefault('MPLCONFIGDIR','/tmp/attach01-review-mpl')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from shapely.geometry import box
from run_gp_se2_attach01 import load_case,read,write,digest
from plot_gp_se2_attach01 import old_prefix,csv_file
from plot_gp_se2_join01 import draw_geometry


def report(run,output_name='review_v2',bundle_name='review_bundle_v2.zip'):
    case=load_case(run);outcomes=read(run/'outcomes.json');starts=read(run/'all_starts.json')
    if Path(output_name).name!=output_name or Path(bundle_name).name!=bundle_name:
        raise ValueError('exclusive one-level report names required')
    out=run/output_name;out.mkdir(exist_ok=False)
    rows=[]
    for s in starts:
        p=run/'methods'/s['method']/'starts'/s['initialization']/'solver_result.json'
        r=read(p);last=r['candidate_checks'][1];full=last['full_acceptance'];extra=full['additional_grid'];tube=s.get('latest_tube')
        rows.append(dict(method=s['method'],T_s=s['duration_s'],seed=s['initialization'],status=s['solver_status'],
            termination=s['status'],iterations=s['iterations'],latest_grid_valid=last['collocation']['feasible'],
            latest_original_full_valid=full['full_feasible'],latest_nominal_tube_valid=None if tube is None else tube['valid'],
            latest_tube_max_excess_m=None if tube is None else max(tube['distance_m'])-.1,
            latest_min_vx_m_s=extra['minimum_linear_speed_m_s'],latest_max_abs_vy_m_s=extra['maximum_absolute_lateral_velocity_m_s'],
            latest_max_abs_ax_m_s2=extra['maximum_absolute_linear_acceleration_m_s2'],
            latest_original_flags=extra['flags'],latest_diagnostic_attachment_s=s['latest_diagnostic_planned_attachment_s'],
            returned_candidate_source=r['selected_iterate'],condition_plan_valid=outcomes[s['method']]['plan_valid'],
            latest_failure_reasons=s['latest_failure_reasons'],solve_s=s['solve_s'],source=str(p),source_sha256=digest(p)))
    csv_file(out/'condition_details.csv',rows)
    native=outcomes['M0_NATIVE'];adapter=outcomes['M0_ADAPTER'];m4=outcomes['M4_FIXED_ATTACH_T14']
    if not m4['plan_valid']:raise ValueError('this actual-result report requires the recorded T1.4 candidate')
    deltas={}
    for label,baseline in [('Native',native),('Adapter',adapter),('M3',outcomes['M3_CURRENT_GP'])]:
        a=baseline['execution_metrics'];b=m4['execution_metrics']
        deltas[label]=dict(attachment_time_delta_s=m4['executed_attachment_s']-baseline['executed_attachment_s'],
            goal_time_delta_s=b['time_to_goal_s']-a['time_to_goal_s'],
            linear_TV_delta=b['command_total_variation_v_mps']-a['command_total_variation_v_mps'],
            angular_TV_delta=b['command_total_variation_omega_radps']-a['command_total_variation_omega_radps'],
            clearance_delta_m=m4['minimum_clearance_m']-baseline['minimum_clearance_m'])
    costs={k:sum(s.get(k,0.)for s in starts)for k in ['construction_s','warmup_s','solve_s','check_s','attachment_diagnostic_s']}
    costs.update(optimizer_phase_total_s=read(run/'comparison_freeze.json')['total_optimization_phase_s'],
        common_provider_construction_s=read(run/'comparison_freeze.json')['provider_construction_s'],
        source_scan_s=read(run/'source.json')['scan_wall_s'],derivative_preflight_s=read(run/'preflight/summary.json')['wall_s'],
        independent_validator_s=read(run/'validation.json')['wall_s'],
        primary_MPC_s=sum(r['mpc_solve_wall_s']or 0. for r in outcomes.values()))
    summary=dict(operational='GP_SE2_ATTACH_01_COMPLETED_WITH_LIMITATIONS',
        result_taxonomy=['POSITIVE_MECHANISM_EVIDENCE_FOR_ONE_VALID_TRANSITION','EXECUTION_TRADEOFF','NO_ATTACHMENT_SPEED_BENEFIT'],
        shortest_tested_full_valid_duration_s=1.4,not_optimal_T=True,
        primary_GP_starts=14,primary_rollouts=4,primary_MPC_solves=120,new_inference=0,new_GUI=0,
        M4_minus_baseline=deltas,timing=costs,
        interpretation='T1.4 supplies a valid safely executed transition. Attachment is later than all baselines while linear TV is lower. T1.0 rejection is nominal tube boundary roundoff, not macroscopic infeasibility. Latest and retained candidate differ.',
        source_limitation='B nearly stationary, straight/aligned world paths, identical raw local OLD/FRESH; e_perp is closest finite-polyline endpoint gap, not 14cm lateral displacement.',
        largest_remaining_uncertainty='Whether the same fixed-correspondence formulation remains full-valid and useful with appreciable lateral mismatch and nonzero boundary motion.')
    write(out/'reporting_summary.json',summary)
    f=case['native'];old=np.array(case['context']['old_world']);past=old_prefix(case['context']);b=np.array(case['context']['B_world'])
    executed=[m for m,r in outcomes.items()if r['rollout_performed']]
    points=np.vstack([f[:,:2],old[:,:2],past[:,:2]]);lo=points.min(0)-.30;hi=points.max(0)+.30;clip=box(*lo,*hi)
    fig,axes=plt.subplots(2,2,figsize=(12,9));numeric={}
    for ax,m in zip(axes.flat,executed):
        env=case['environment'];draw_geometry(ax,env.workspace.intersection(clip),facecolor='#f1f5ed',edgecolor='#b7c3aa')
        draw_geometry(ax,env.obstacles.intersection(clip),facecolor='#777',alpha=.8)
        draw_geometry(ax,env.obstacles.buffer(.25).boundary.intersection(clip),edgecolor='#b87b52')
        ax.plot(old[:,0],old[:,1],':',color='#4785ac');ax.plot(past[:,0],past[:,1],color='#777',lw=2)
        ax.plot(f[:,0],f[:,1],'--',color='#9d4391');r=np.load(run/'methods'/m/'reference.npy')
        ax.plot(r[:,0],r[:,1],color='#008a92',lw=1)
        e=read(run/'evaluation'/m/'execution.json');j=read(run/'evaluation'/m/'attachment.json');p=np.array(e['dense_poses_world'])
        ax.plot(p[:,0],p[:,1],color='#e47722',lw=2);ax.scatter(*b[:2],color='black',s=20)
        ax.scatter(*f[-1,:2],marker='*',color='#267947',s=80);ax.add_patch(Circle(b[:2],.20,fill=False,color='#555',alpha=.35))
        ax.scatter(*j['joined_pose_world'][:2],marker='o',facecolors='none',edgecolors='#e47722',s=70)
        planned=run/'methods'/m/'planned_attachment.json';plan=read(planned)if planned.exists()else None
        if plan is not None and plan['joined_pose_world']is not None:ax.scatter(*plan['joined_pose_world'][:2],marker='s',facecolors='none',edgecolors='#008a92',s=65)
        title=m.replace('M4_FIXED_ATTACH_T14','M4 — external T=1.4s')
        ax.set_title(title+f"\nexecuted attachment {j['sustained_join_time_s']:.3f}s",fontsize=11)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal')
        numeric[m]=dict(reference=r,execution=p,planned_attachment=plan,executed_attachment=j)
    handles=[Line2D([],[],color=c,ls=ls,label=label)for c,ls,label in [('#4785ac',':','original OLD'),('#777','-','executed OLD prefix'),('#9d4391','--','original FRESH'),('#008a92','-','method reference'),('#e47722','-','actual MPC execution'),('#b87b52','-','center exclusion: radius + .05m')]]
    handles += [Line2D([],[],linestyle='',marker=marker,color=c,markerfacecolor=face,label=label)for marker,c,face,label in [('s','#008a92','none','planned sustained attachment'),('o','#e47722','none','executed sustained attachment'),('o','black','black','B; circle is .20m footprint'),('*','#267947','#267947','original goal')]]
    fig.legend(handles=handles,loc='lower center',ncol=3,fontsize=8);fig.suptitle('OFFLINE SINGLE-HANDOFF COUNTERFACTUAL — valid executed methods\nPaths overlap without coordinate offsets; unavailable T conditions remain in the complete index',fontsize=12)
    fig.subplots_adjust(left=.07,right=.98,bottom=.18,top=.84,hspace=.65,wspace=.20)
    png=out/'valid_execution_comparison.png';fig.savefig(png,dpi=170);plt.close(fig)
    write(png.with_suffix('.json'),dict(image_sha256=digest(png),numeric=numeric,source_sha256=digest(run/'source.json'),
        config_sha256=digest(run/'config_snapshot.yaml'),common_sha256=digest(run/'inputs/common.npy'),outcomes_sha256=digest(run/'outcomes.json')))
    text=['<!doctype html><meta charset="utf-8"><title>ATTACH-01 review</title><style>body{font:16px system-ui;max-width:1400px;margin:30px auto}img{max-width:100%}td,th{padding:7px;border:1px solid #ccc}table{border-collapse:collapse}</style>',
        '<h1>ATTACH-01: fixed correspondence / external duration</h1>',
        '<p>T=1.4s produced a full-valid plan and safe sustained execution. Attachment was 0.170s vs Native/Adapter 0.155s: no speed benefit. Linear command TV decreased. T is a planning condition, not the measured geometric attachment time.</p>',
        '<p>All six durations were tested once with two unchanged initializations each. T=1.0s passes the original full checker but exceeds the separately frozen nominal tube by 5.55e-12–1.69e-11m. It was not executed or reclassified. This is not evidence of macroscopic physical infeasibility.</p>',
        '<p>Source limitation: nearly stationary B, aligned straight paths; the 14.12cm existing e_perp is endpoint separation, not lateral displacement. No new-obstacle response, automatic correspondence, optimal-time or general navigation claim.</p>',
        '<p><a href="../index.html">Complete frozen result index (all failed conditions)</a> · <a href="condition_details.csv">All 14 starts and exact rejection diagnostics</a> · <a href="reporting_summary.json">Interpretation and compute</a> · <a href="../validation.json">Independent validator</a></p>',
        '<img src="valid_execution_comparison.png">','<h2>All duration conditions</h2><table><tr><th>T [s]</th><th>SLSQP I0/I1</th><th>Plan</th><th>Planned attach</th><th>Executed attach</th><th>Latest failure</th></tr>']
    for m,r in outcomes.items():
        if r['duration_s']is None:continue
        pair=[s for s in starts if s['method']==m]
        vals=[r['duration_s'],' / '.join(str(s['solver_status'])for s in pair),r['plan_valid'],r['planned_attachment_s'],r['executed_attachment_s'],pair[0]['latest_failure_reasons']]
        text.append('<tr>'+''.join('<td>'+html.escape('N/A'if v is None else str(v))+'</td>'for v in vals)+'</tr>')
    text+=['</table>','<p>At T=1.4 the retained callback is full-valid; the later final iterate fails speed. Latest failures in the table do not override the retained candidate.</p>']
    for name in ['execution_to_fresh_distance','execution_to_fresh_yaw','duration_sweep_summary','duration_control_variation']:
        text += [f'<h2>{name}</h2>',f'<img src="../plots/{name}.png">',f'<a href="../plots/{name}.json">Numbers and provenance</a>']
    with (out/'index.html').open('x')as stream:stream.write('\n'.join(text))
    # Exact source-to-figure agreement for this reporting-only additional artifact.
    side=read(png.with_suffix('.json'))
    valid=all(side['numeric'][m]['execution']==read(run/'evaluation'/m/'execution.json')['dense_poses_world'] and
        side['numeric'][m]['reference']==np.load(run/'methods'/m/'reference.npy').tolist()for m in executed)
    write(out/'validation.json',dict(valid=valid,no_new_solve=True,primary_validation_sha256=digest(run/'validation.json'),
        image_sha256=digest(png),all_six_T_retained=True))
    if not valid:raise ValueError('report numeric mismatch')
    paths=[run/n for n in ['index.html','source.json','protocol.json','config_snapshot.yaml','outcomes.json','summary.json','validation.json']]
    paths+=list((run/'plots').glob('*'))+list((run/'aggregate').glob('*.csv'))+list(out.glob('*'))
    with zipfile.ZipFile(run/bundle_name,'x',zipfile.ZIP_DEFLATED)as archive:
        for p in paths:archive.write(p,p.relative_to(run))
        archive.writestr('README.txt',f'Open {output_name}/index.html. Static saved-evidence presentation. All failed T conditions are retained. No raw RGB, full environment, weights or external source. Original immutable primary plots and improved complete-legend review figure are separate. Presentation v2 only increases subplot spacing; no numerical record changed.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',default='review_v2');parser.add_argument('--bundle',default='review_bundle_v2.zip')
    args=parser.parse_args();report(args.run.resolve(),args.output,args.bundle)
