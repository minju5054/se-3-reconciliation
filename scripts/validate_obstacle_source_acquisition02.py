#!/usr/bin/env python3
"""Recompute technical qualification from saved records; no new model/solver."""
import argparse
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.obstacle_source_acquisition import timing_gate
from run_join_source05 import verify
from run_join_online02 import environments
from analyze_join_online02 import analyze_episode,csvread,jsonlines
from diagnose_obstacle_source_pacing import audit
from validate_robotless_online_handoffs import validate_episode,equal_record
from reconciliation.online_history import history_contract
from reconciliation.join_online02 import guard_check


def scheduler_audit(ep):
    states=csvread(ep/'execution.csv');rows=jsonlines(ep/'scheduler.jsonl');frames=jsonlines(ep/'capture.jsonl')
    meta=read(ep/'metadata.json');dt=meta['resolved_integration_dt_s']
    complete=[r for r in rows if r['sim_steps']==1]
    assert len(complete)==len(states)-1
    for i,r in enumerate(complete):
        assert r['start_state_id']==i and r['end_state_id']==i+1
        assert r['simulation_start_s']==float(states[i]['sim_time_s'])
        assert r['simulation_end_s']==float(states[i+1]['sim_time_s'])
        assert r['completion_host_s']==float(states[i+1]['host_monotonic_s'])
        assert abs(r['wall_step_s']-(float(states[i+1]['host_monotonic_s'])-float(states[i]['host_monotonic_s'])))<1e-9
        if i>0: assert abs(r['intended_wall_deadline_s']-(r['prior_completion_host_s']+dt))<1e-8
        assert r['sleep_requested_s']==max(0.,-r['deadline_error_before_sleep_s'])
    bursts=[r['start_state_id'] for r in complete if r['wall_step_s']<dt-1e-6]
    captures=[int(f['rendered_state_id']) for f in frames]
    capture_ticks_valid=captures==list(range(0,len(rows),15))
    mpc_ticks=[r['start_state_id'] for r in rows if r['mpc_submitted']]
    control_ticks_valid=all(x%6==0 for x in mpc_ticks) and all(b-a==6 for a,b in zip(mpc_ticks,mpc_ticks[1:]))
    requests=audit(ep)['requests']
    for r in requests:
        subset=[x for x in complete if r['first_inflight_state'] is not None and r['first_inflight_state']<=x['start_state_id']<r['last_inflight_state']]
        maximum=n=0
        for x in subset:
            n=n+1 if x['sleep_requested_s']==0 else 0;maximum=max(maximum,n)
        r.update(measured_no_sleep_steps=sum(x['sleep_requested_s']==0 for x in subset),maximum_consecutive_no_sleep_steps=maximum,
            measured_burst_steps=sum(x['wall_step_s']<dt-1e-6 for x in subset))
    return dict(valid=not bursts and capture_ticks_valid and control_ticks_valid,burst_state_ids=bursts,
        minimum_wall_step_s=min([r['wall_step_s'] for r in complete],default=None),
        capture_ticks_valid=capture_ticks_valid,capture_ticks=captures,control_ticks_valid=control_ticks_valid,control_ticks=mpc_ticks,
        simulation_tick_dt_s=dt,requests=requests,buffered_scheduler_records=len(rows),
        exact_one_step_per_completed_outer_loop=True,wall_rate_is_not_forced_to_simulation_rate=True)


def validate(run):
    verify(run)
    p0=run/'phase0';ep=p0/'episodes/PACING_OFF_00'
    config=yaml.safe_load((p0/'config_snapshot.yaml').read_text());source=Path(read(run/'source.json')['OSA01'])
    original=yaml.safe_load((source/'phase0/config_snapshot.yaml').read_text())
    cmp=__import__('copy').deepcopy(config);cmp['execution'].pop('pacing_policy');cmp['online'].pop('scheduler_diagnostics')
    assert cmp==original,'only scheduling policy/instrumentation may differ'
    assert read(run/'selected_pose11.json')==next(x for x in read(source/'candidate_manifest.json') if x['candidate']=='POSE11')
    contract,sampler=history_contract((ROOT/config['paths']['lightnav_checkout']).resolve(),(ROOT/config['paths']['checkpoint_path']).resolve())
    validation=validate_episode(ep,p0,contract,sampler,require_plots=False,integration_dt_s=read(ep/'metadata.json')['resolved_integration_dt_s'])
    assert validation['valid']
    base,_,_,_=environments(p0)
    for g in jsonlines(ep/'guard.jsonl'):
        expected=guard_check(base,g['start_pose'],g['command'],read(ep/'metadata.json')['resolved_integration_dt_s'])
        equal_record(expected,{k:g[k] for k in expected},'guard')
    analysis=analyze_episode(p0,'PACING_OFF_00');scheduler=scheduler_audit(ep)
    gate=timing_gate([r.get('inflight_rtf') for r in analysis['rows'][1:]],analysis['summary']['max_loop_stall_s'])
    checks=dict(requests_RTF_and_stall=gate['qualified'],scheduler=scheduler['valid'],state_reconstruction=validation['integration']['maximum_pose_reconstruction_error']==0.,
        no_safety_abort=not (ep/'guard_abort.json').exists(),normal_completion=read(ep/'metadata.json')['status']=='ATTEMPT_LIMIT',
        three_activated_handoffs=read(ep/'metadata.json')['activated_handoffs']>=3)
    qualified=all(checks.values())
    return dict(valid=True,source_preserved=True,source_only_inputs_not_rerun=True,config_semantics_equal_except_pacing=True,
        saved_episode_validation=validation,analysis=analysis,scheduler=scheduler,
        qualification=dict(qualified=qualified,checks=checks,timing_gate=gate,
            classification='PACING_QUALIFIED_PENDING_PHASEB_FREEZE' if qualified else 'PACING_FIX_NOT_QUALIFIED'),
        calls=dict(technical_predictions=analysis['summary']['predictions'],technical_buffers=analysis['summary']['buffer_requests'],
            MPC_submissions=analysis['summary']['MPC_accepted_submissions'],MPC_saved_results=analysis['summary']['MPC_solved_results'],
            saved_MPC_wall_s=analysis['summary']['MPC_time_s'],PhaseA_predictions=0,GP_rigid_reconciliation=0,validator_calls=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    r=validate(run);save(run/'phase0/validation.json',r);save(run/'phase0/qualification.json',r['qualification']);save(run/'phase0/analysis.json',r['analysis']);save(run/'phase0/scheduler_analysis.json',r['scheduler'])
    print(__import__('json').dumps(r['qualification'],indent=2))
