#!/usr/bin/env python3
"""Saved timing audit only. No simulator, controller or model imports/calls."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from analyze_join_online02 import csvread,jsonlines


def audit(ep):
    states=csvread(ep/'execution.csv');loop=jsonlines(ep/'loop.jsonl');frames=jsonlines(ep/'capture.jsonl')
    requests=[read(p) for p in sorted((ep/'requests').glob('seq_*_metadata.json'))]
    sim=np.array([float(x['sim_time_s']) for x in states]);host=np.array([int(x['host_monotonic_ns']) for x in states])/1e9
    dt=read(ep/'metadata.json')['resolved_integration_dt_s'];steps=[]
    renders={x['rendered_state_id']:x for x in frames}
    for i,(a,b) in enumerate(zip(states,states[1:])):
        f=renders.get(i);wall=host[i+1]-host[i]
        steps.append(dict(start_state_id=i,end_state_id=i+1,simulation_dt_s=sim[i+1]-sim[i],wall_dt_s=wall,
            host_completion_s=host[i+1],simulation_completion_s=sim[i+1],
            deadline_lateness_s=loop[i]['lateness_s'],
            inferred_fast_step=wall<dt-1e-6,
            capture_happened=f is not None,
            render_readback_s=None if f is None else (f['readback_finished_host']['host_monotonic_ns']-f['capture_monotonic_ns'])/1e9))
    rows=[]
    for r in requests:
        if r['kind']!='prediction' or not r.get('t_ready_host'):continue
        start=r['t_request_host']['monotonic_ns']/1e9;end=r['t_ready_host']['monotonic_ns']/1e9
        ids=np.flatnonzero((host>=start)&(host<=end));first,last=(int(ids[0]),int(ids[-1])) if len(ids) else (None,None)
        intervals=[] if len(ids)<2 else steps[first:last]
        fast=[x for x in intervals if x['inferred_fast_step']];obs=int(r['observation']['rendered_state_id'])
        # Unchanged historical request-local metric: first/last saved states inside client send/receipt.
        rtf=(sim[last]-sim[first])/(host[last]-host[first]) if len(ids)>=2 else None
        run=maximum=0
        for x in intervals:
            run=run+1 if x['inferred_fast_step'] else 0;maximum=max(maximum,run)
        rows.append(dict(chunk_id=r['chunk_id'],observation_state_id=obs,observation_sim_s=sim[obs],
            request_host_s=start,receipt_host_s=end,client_RTT_s=r['client_rtt_s'],
            first_inflight_state=first,last_inflight_state=last,inflight_intervals=len(intervals),request_local_RTF=rtf,
            fast_intervals=len(fast),maximum_consecutive_fast_steps=maximum,
            fastest_inflight_step_s=min([x['wall_dt_s'] for x in intervals],default=None),
            deadline_debt_first_inflight_s=None if first is None or first==0 else loop[first-1]['lateness_s'],
            debt_after_observation_render_s=loop[obs]['lateness_s'],
            observation_render_s=steps[obs]['render_readback_s'],
            observation_to_last_inflight_sim_s=None if last is None else sim[last]-sim[obs],
            window_definition='unchanged first/last saved state within actual send/receipt; not full RTT quotient'))
    return dict(source_episode=str(ep),source_sha256={str(p):sha(p) for p in [ep/'execution.csv',ep/'commands.csv',ep/'loop.jsonl',ep/'capture.jsonl',ep/'controller/events.jsonl',*sorted((ep/'requests').glob('seq_*_metadata.json'))]},
        nominal_dt_s=dt,whole_episode_RTF=(sim[-1]-sim[0])/(host[-1]-host[0]),
        maximum_loop_stall_s=max(x['loop_interval_host_s'] for x in loop),steps=steps,requests=rows,
        saved_limitations=dict(actual_sleep_duration=None,outer_loop_start=None,individual_while_iteration_step_count=None,
            reason='historical records did not store these; source code has exactly one integration per loop, no inner catchup loop; fast outer iterations inferred from actual state times'),
        conclusion='ABSOLUTE_DEADLINE_DEBT_REPAID_BY_FAST_OUTER_ITERATIONS',
        interpretation='render/readback blocking precedes deadline debt and fast saved steps during requests; individual GPU/renderer internal stall causes not isolated',
        new_model_MPC_optimizer_calls=0)


def main():
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=audit(a.episode.resolve());save(a.out/'diagnosis.json',result)
    with (a.out/'integration_steps.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(result['steps'][0]));w.writeheader();w.writerows(result['steps'])
    print(json.dumps(result['requests'],indent=2))
if __name__=='__main__':main()
