#!/usr/bin/env python3
"""Saved-record loss audit. No optimization, inference, MPC or new execution."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime,timezone
import html
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
from genuine_source_scan_gui import SavedSources,sha
from reconciliation.handoff_execution_loss import audit_case
from reconciliation.gp_se2_join01 import POSITION_M,YAW_RAD,DWELL_S
from reconciliation.gp_se2_environment import HospitalEnvironment


def plain(x):
    if isinstance(x,np.ndarray):return plain(x.tolist())
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [plain(v) for v in x]
    return x


def save(path,value):
    with Path(path).open('x') as f:json.dump(plain(value),f,indent=2,allow_nan=False)


def compute(config):
    if (config['position_threshold_m']!=POSITION_M or config['yaw_threshold_deg']!=15.
            or config['sampled_dwell_s']!=DWELL_S):raise ValueError('original attachment threshold mismatch')
    saved=SavedSources(ROOT/config['source_scan'],include_post_switch=True)
    path=Path(saved.source['environment_path']);env=HospitalEnvironment.load(path)
    # Direct geometry, not the optimization distance approximation.
    for p in (path/'geometry/projected').iterdir():
        if p.is_file():saved.record(p)
    rows=[]
    for c in saved.cases:
        meta=Path(c['row']['source_paths']['context']).parents[2]/'metadata.json'
        settings=json.loads(saved.record(meta).read_text())['mpc_worker']['provenance']['official_settings']
        expected={'CONTROL_RATE_HZ':config['command_grid_hz'],'OBJNAV_V_MAX':config['motion_limits']['v_max_mps'],
            'W_MAX':config['motion_limits']['omega_max_radps'],'A_MAX_V':config['motion_limits']['nominal_grid_a_v_max_mps2'],
            'A_MAX_W':config['motion_limits']['nominal_grid_a_omega_max_radps2']}
        if any(settings[k]!=v for k,v in expected.items()):raise ValueError('original controller settings mismatch')
        result=plain(audit_case(c,env,config))
        # Explain a flagged nominal-grid jump without declaring physical dynamics
        # failure or attributing it to the handoff optimizer.
        if not result['command']['nominal_command_grid_valid']:
            events=meta.parent/'controller/events.jsonl';saved.record(events)
            solves=[json.loads(line) for line in events.read_text().splitlines()]
            solves=[e for e in solves if e.get('type')=='solve_result']
            by_solve={e['solve_id']:e for e in solves};applied=c['post']['commands']
            u=np.asarray(result['command']['commands']);delta=np.diff(np.vstack([result['physical_u_minus'],u]),axis=0)
            limits=config['motion_limits'];bad=np.flatnonzero((abs(delta[:,0])*10>limits['nominal_grid_a_v_max_mps2']+limits['numeric_tolerance']) |
                (abs(delta[:,1])*10>limits['nominal_grid_a_omega_max_radps2']+limits['numeric_tolerance']))
            notes=[]
            for j in bad:
                earlier=np.flatnonzero(np.any(abs(delta[:j])>1e-12,axis=1));i=int(earlier[-1]) if len(earlier) else None
                elapsed=None if i is None else float(applied[j]['sim_time_s'])-float(applied[i]['sim_time_s'])
                current=by_solve.get(applied[j]['solve_id']);prior=None if i is None else by_solve.get(applied[i]['solve_id'])
                skipped=[] if current is None or prior is None else [e for e in solves if prior['input_sim_time_s']<e['input_sim_time_s']<current['input_sim_time_s'] and e['chunk_id']==current['chunk_id'] and not any(a['solve_id']==e['solve_id'] for a in applied)]
                notes.append(dict(saved_interval_index=int(j),application_command=applied[j],
                    preceding_held_command=None if j==0 else applied[j-1],
                    delta=delta[j].tolist(),elapsed_since_previous_value_change_s=elapsed,
                    delta_over_elapsed=None if elapsed is None else (delta[j]/elapsed).tolist(),
                    actual_solve_record=current,intermediate_unapplied_solve_records=skipped,
                    interpretation='Nominal 10Hz metric; asynchronous application spacing and controller memory retained. Not continuous physical acceleration or isolated optimizer fault.'))
            result['command']['nominal_grid_timing_context']=notes
        rows.append(result)
    for p,h in saved.hashes.items():
        if sha(p)!=h:raise ValueError('source changed during audit')
    return rows,saved.hashes


def flat(r):
    f,c,u,e=r['full'],r['common'],r['command'],r['environment']
    return dict(case_id=r['case_id'],group=r['projection_group'],observed_duration_s=f['observation_horizon_s'],
        B_distance_m=f['B_distance_m'],B_yaw_deg=f['B_yaw_error_deg'],join_observed=f['join_success'],
        join_time_s=f['join_time_s'],observation_status=f['observation_status'],first_tube_entry_s=f['first_tube_entry_time_s'],
        longest_inside_span_s=f['longest_observed_inside_span_s'],max_distance_m=f['max_distance_m'],
        max_distance_time_s=f['maximum_distance_time_s'],exits_after_join=f['exits_after_first_join'],end_distance_m=f['end_distance_m'],
        end_yaw_deg=f['end_yaw_error_deg'],position_auc_full_m_s=f['position_auc_m_s'],
        position_auc_common_m_s=c['position_auc_m_s'],position_excess_auc_common_m_s=c['position_excess_auc_m_s'],
        mean_distance_common_m=c['mean_distance_time_weighted_m'],yaw_auc_common_rad_s=c['yaw_auc_rad_s'],
        linear_TV_full_mps=u['linear_TV_mps'],angular_TV_full_radps=u['angular_TV_radps'],
        linear_TV_common_mps=r['common_command']['linear_TV_mps'],angular_TV_common_radps=r['common_command']['angular_TV_radps'],
        nominal_10Hz_max_a_v=u['nominal_10Hz_max_delta_v_over_dt_mps2'],nominal_10Hz_max_a_w=u['nominal_10Hz_max_delta_omega_over_dt_radps2'],
        nominal_command_grid_valid=u['nominal_command_grid_valid'],clearance_valid=e['clearance_valid'],
        minimum_clearance_lower_bound_m=e['minimum_clearance_lower_bound_m'],workspace_known=e['workspace_known'],
        physical_overlap_polyline=e['physical_overlap'],memory_discrepancy=r['memory_discrepancy'])


def aggregate(rows):
    result={}
    for group in ['ALL','INTERIOR','ENDPOINT_CAVEAT']:
        selected=rows if group=='ALL' else [r for r in rows if r['projection_group']==group]
        result[group]=dict(count=len(selected),join_observed=sum(r['full']['join_success'] for r in selected),
            no_join_observed=sum(not r['full']['join_success'] for r in selected),
            clearance_invalid=sum(not r['environment']['clearance_valid'] for r in selected),
            nominal_command_grid_invalid=sum(not r['command']['nominal_command_grid_valid'] for r in selected),
            episode_count=len({r['episode_id'] for r in selected}),ordered_pair_count=len({r['ordered_raw_pair'] for r in selected}))
        result[group]['observation_status_counts']={k:sum(r['full']['observation_status']==k for r in selected) for k in sorted({r['full']['observation_status'] for r in selected})}
    # Same fixed observation prefix for costs; do not average unavailable join times.
    for mode,key in [('event',None),('episode','episode_id'),('ordered_pair','ordered_raw_pair')]:
        groups={}
        for i,r in enumerate(rows):groups.setdefault(str(i) if key is None else r[key],[]).append(r)
        result[mode+'_weighted_common_position_auc_m_s']=float(np.mean([np.mean([r['common']['position_auc_m_s'] for r in g]) for g in groups.values()]))
    result.update(new_model_GP_MPC_rollout_calls=0,
        no_optimized_method_comparison=True,no_feasibility_or_improvement_claim=True,
        sampling='13 preselected mismatched development-corpus events; not population frequencies',
        censoring='join not observed before next actual reference application; not a fixed-3s failure')
    return result


def plots(out,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p=out/'plots';p.mkdir()
    binding=dict(records_sha256=sha(out/'records.json'),source_sha256=sha(out/'source.json'),protocol_sha256=sha(out/'protocol.yaml'))
    for field,threshold,unit,name in [('distance_m',POSITION_M,'m','actual_distance_to_original_fresh'),
                                    ('yaw_error_rad',float(np.degrees(YAW_RAD)),'deg','actual_yaw_to_original_fresh')]:
        fig,axes=plt.subplots(5,3,figsize=(13,13),sharex=True,sharey=True,layout='constrained');numbers=[]
        for ax,r in zip(axes.flat,rows):
            t=np.asarray(r['full']['trace']['times_s']);y=np.asarray(r['full']['trace'][field])
            if unit=='deg':y=np.degrees(y)
            ax.plot(t,y,color='tab:blue');ax.axhline(threshold,color='black',ls='--',lw=1)
            tau=r['full']['join_time_s']
            if tau is not None:ax.axvspan(tau,tau+DWELL_S,alpha=.2,color='green')
            ax.axvline(t[-1],ls=':',color='.5');ax.set_title(r['case_id'].replace('episode_','').replace('repeat_',''),fontsize=9)
            ax.set(xlim=(0,1.6),xlabel='Time after B [s]',ylabel=f'Error [{unit}]');ax.grid(alpha=.2)
            ax.text(.03,.96,'join N/A' if tau is None else f'join {tau:.3f}s',transform=ax.transAxes,va='top',fontsize=8)
            numbers.append(dict(case_id=r['case_id'],times_s=t.tolist(),values=y.tolist(),join_time_s=tau))
        for ax in list(axes.flat)[len(rows):]:ax.set_visible(False)
        fig.suptitle('Recorded native execution; dashed = original attachment threshold\nGreen = first sampled 0.30s join; trace stops at next chunk (no extrapolation)')
        fig.savefig(p/(name+'.png'),dpi=140);plt.close(fig)
        save(p/(name+'.json'),dict(**binding,numbers=numbers,png_sha256=sha(p/(name+'.png'))))
    fig,axes=plt.subplots(1,3,figsize=(15,6),layout='constrained');numeric=[flat(r) for r in rows]
    for ax,key,label in zip(axes,['position_auc_common_m_s','linear_TV_common_mps','angular_TV_common_radps'],
        ['Position-error AUC [m s]','Linear command TV [m/s]','Angular command TV [rad/s]']):
        ax.barh(range(len(rows)),[r[key] for r in numeric],color=['tab:orange' if r['group']=='ENDPOINT_CAVEAT' else 'tab:blue' for r in numeric])
        ax.set_yticks(range(len(rows)),[r['case_id'].replace('episode_','').replace('repeat_','') for r in rows],fontsize=8)
        ax.invert_yaxis();ax.set_xlabel(label);ax.grid(axis='x',alpha=.2)
    fig.suptitle('Same first 54 saved steps (~0.9s); separate costs, no weighted total\nOrange = endpoint caveat. Lower TV alone is not better attachment.')
    name='common_window_costs';fig.savefig(p/(name+'.png'),dpi=140);plt.close(fig)
    save(p/(name+'.json'),dict(**binding,numbers=numeric,png_sha256=sha(p/(name+'.png'))))
    cells=['case_id','group','join_time_s','observed_duration_s','mean_distance_common_m','position_auc_common_m_s','clearance_valid']
    body='<h1>Saved native handoff execution loss</h1><p>No new model, optimization, MPC, or rollout. N/A join is right-censored at the next chunk, not a 3s failure.</p><table border="1"><tr>'+''.join('<th>'+k+'</th>' for k in cells)+'</tr>'
    for r in numeric:body+='<tr>'+''.join('<td>'+html.escape('N/A' if r[k] is None else str(r[k]))+'</td>' for k in cells)+'</tr>'
    body+='</table><p>All source records, numeric traces and commands: records.json; summary: summary.json; metrics.csv.</p>'
    for name in ['actual_distance_to_original_fresh','actual_yaw_to_original_fresh','common_window_costs']:body+='<h2>'+name+'</h2><img width="1100" src="plots/'+name+'.png">'
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8">'+body)


def validate(out):
    source=json.loads((out/'source.json').read_text())
    for p,h in {**source['source_hashes'],**source['code_hashes']}.items():
        if sha(p)!=h:raise ValueError('immutable input/code changed: '+p)
    rows,_=compute(yaml.safe_load((out/'protocol.yaml').read_text()))
    if rows!=json.loads((out/'records.json').read_text()):raise ValueError('saved records do not reproduce')
    if aggregate(rows)!=json.loads((out/'summary.json').read_text()):raise ValueError('summary does not reproduce')
    with (out/'metrics.csv').open() as f:csvrows=list(csv.DictReader(f))
    expected=[{k:'' if v is None else str(v) for k,v in flat(r).items()} for r in rows]
    if csvrows!=expected:raise ValueError('CSV does not reproduce')
    for p in (out/'plots').glob('*.json'):
        side=json.loads(p.read_text())
        for k,name in [('records_sha256','records.json'),('source_sha256','source.json'),('protocol_sha256','protocol.yaml')]:
            if side[k]!=sha(out/name):raise ValueError('plot binding mismatch')
        if side['png_sha256']!=sha(p.with_suffix('.png')):raise ValueError('PNG hash changed')
        if p.stem=='common_window_costs':numbers=[flat(r) for r in rows]
        else:
            field='distance_m' if 'distance' in p.stem else 'yaw_error_rad';numbers=[]
            for r in rows:
                y=np.asarray(r['full']['trace'][field]);y=np.degrees(y) if field=='yaw_error_rad' else y
                numbers.append(dict(case_id=r['case_id'],times_s=r['full']['trace']['times_s'],values=y.tolist(),join_time_s=r['full']['join_time_s']))
        if side['numbers']!=numbers:raise ValueError('plot numbers differ')
    return dict(valid=True,case_count=len(rows),new_model_optimizer_MPC_rollout_calls=0,
        validation='saved-only full recomputation, source/code hashes, CSV and plotted-number equality')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/saved_handoff_execution_loss.yaml')
    parser.add_argument('--validate',action='store_true');args=parser.parse_args();out=args.output.resolve()
    if args.validate:print(json.dumps(validate(out)));return
    config=yaml.safe_load(args.config.read_text());out.mkdir(parents=True,exist_ok=False)
    (out/'protocol.yaml').write_bytes(args.config.read_bytes());started=time.perf_counter()
    rows,hashes=compute(config)
    code=['src/reconciliation/handoff_execution_loss.py','scripts/audit_saved_handoff_losses.py',
          'scripts/isaac/genuine_source_scan_gui.py','src/reconciliation/gp_se2_join01.py',
          'src/reconciliation/gp_se2_environment.py','src/reconciliation/robotless_online.py',
          'src/reconciliation/online_handoff_analysis.py','src/reconciliation/se2.py',
          'src/reconciliation/gp_se2_formulation.py','src/reconciliation/gp_se2_attach01.py',
          'configs/saved_handoff_execution_loss.yaml']
    save(out/'source.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),source_hashes=hashes,
        code_hashes={str(ROOT/p):sha(ROOT/p) for p in code},git_SHA=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        code_uncommitted_at_audit=True,source_scan=config['source_scan'],new_scientific_calls=0))
    save(out/'records.json',rows);save(out/'summary.json',aggregate(rows))
    with (out/'metrics.csv').open('x') as f:
        writer=csv.DictWriter(f,fieldnames=list(flat(rows[0])));writer.writeheader();writer.writerows(flat(r) for r in rows)
    plots(out,rows);save(out/'validation.json',validate(out))
    save(out/'timing.json',dict(total_audit_plot_validation_wall_s=time.perf_counter()-started,new_model_optimizer_MPC_rollout_calls=0))
    print(json.dumps(aggregate(rows),indent=2))


if __name__=='__main__':main()
