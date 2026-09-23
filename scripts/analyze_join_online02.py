#!/usr/bin/env python3
"""Saved-only chunk/activation geometry audit, never new inference or execution."""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
import yaml
from shapely.geometry import Point
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from run_join_online02 import read,save,sha,environments
from reconciliation.join_source03 import path_geometry
from reconciliation.join_online02 import hallway_geometry,classify_chunk,episode_outcome,candidate_source,ORDER
from reconciliation.join_source02_acquisition import held_interval_check
from reconciliation.se2 import local_trajectory_to_world,wrap_angle
from reconciliation.robotless_online import verify_resume

def csvread(p):
    if not p.exists():return []
    with p.open() as f:return list(csv.DictReader(f))
def jsonlines(p):return [json.loads(s) for s in p.read_text().splitlines() if s] if p.exists() else []
def pose_rows(rows):return np.asarray([[float(r[k]) for k in ['x','y','yaw']] for r in rows])

def analyze_episode(run,episode):
    ep=run/'episodes'/episode;metadata=read(ep/'metadata.json')
    assert verify_resume(ep)
    base,on,cart,source=environments(run);environment=on if metadata['cart_present'] else base
    scenario=read(run/'scenario.json');cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text())['join_online02']['classification']
    center=np.asarray(scenario['center_xy']);forward=np.asarray(scenario['forward_xy'])
    free_sides=[r['side'] for r in read(run/'side_passages.json') if r['check']['clearance_valid']]
    states=csvread(ep/'execution.csv');commands=csvread(ep/'commands.csv');poses=pose_rows(states)
    events=jsonlines(ep/'controller/events.jsonl');solves={r['solve_id']:r for r in events if r.get('type')=='solve_result' and r.get('episode_id')==episode}
    records=[read(p) for p in sorted((ep/'requests').glob('seq_*_metadata.json'))]
    vis={r['frame_id']:r for r in jsonlines(ep/'visibility.jsonl')}
    late=read(ep/'nonactivated_response.json')['result']['chunk_id'] if (ep/'nonactivated_response.json').exists() else None
    contexts=[read(p) for p in sorted((ep/'handoffs').glob('*/context.json'))]
    if (ep/'bootstrap.json').exists():contexts.append(read(ep/'bootstrap.json'))
    context_by={c['fresh_chunk_id']:c for c in contexts}
    first={}
    for c in commands:
        if c['chunk_id'] and c['chunk_id'] not in ('None','') and c['chunk_id'] not in first:first[c['chunk_id']]=c
    applied_order=sorted(first,key=lambda k:int(first[k]['application_state_id']))
    rows=[]
    for rec in records:
        if rec['kind']!='prediction':continue
        chunk=rec['chunk_id'];obs=rec['observation'];ctx=context_by.get(chunk,{})
        common=dict(episode=episode,chunk_id=chunk,display_id='C'+str(int(chunk.split('_')[-1])),
            native_status=rec['status'],cart_present=metadata['cart_present'],t_obs=obs['capture_sim_time_s'],
            observation=obs['pose_world'],observation_state_id=obs['rendered_state_id'],frame_id=obs['frame_id'],
            RGB_path=str(ep/obs['path']),RGB_sha256=obs['sha256'],
            t_request_host=rec['t_request_host'],t_receipt_host=rec.get('t_ready_host'),client_rtt_s=rec.get('client_rtt_s'),
            t_ready=ctx.get('t_ready_seen_sim',{}).get('sim_time_s'),t_install=(ctx.get('t_install') or {}).get('sim_time_s'),
            received_before_end=chunk!=late,cart_pixels=vis.get(obs['frame_id'],{}).get('instance',{}).get('visible_pixels'),
            terminal_mask_record=vis.get(obs['frame_id']),classification_basis='actual cart ON' if metadata['cart_present'] else 'hypothetical fixed cart ON; actual OFF safety separate')
        c=first.get(chunk);bindex=int(c['application_state_id']) if c else None
        b=poses[bindex] if c else None;current=applied_order.index(chunk) if c else None
        end=int(first[applied_order[current+1]]['application_state_id']) if c and current+1<len(applied_order) else len(states)-1 if c else None
        old=applied_order[current-1] if c and current else None
        if not c:old=ctx.get('old_chunk_id')
        previous=commands[bindex-1] if c and bindex>0 else None
        physical=[float(previous['v_mps']),float(previous['omega_radps'])] if previous else [0.,0.] if c else None
        activation=dict(t_apply=float(states[bindex]['sim_time_s']) if c else None,B=None if b is None else b.tolist(),
            B_state_id=bindex,B_cart_center_distance_m=None if b is None else float(np.linalg.norm(b[:2]-center)),
            B_cart_edge_clearance_m=None if b is None else float(cart.distance(Point(b[:2]))-.2),
            observation_to_B_travel_m=None if b is None else float(np.linalg.norm(np.diff(poses[int(obs['rendered_state_id']):bindex+1,:2],axis=0),axis=1).sum()),
            old_chunk_id=old,u_minus=physical,previous_control=None if not c else solves.get(c['solve_id'],{}).get('previous_command'),
            moving_B=bool(physical is not None and (abs(physical[0])>1e-6 or abs(physical[1])>1e-6)),
            reference_lifetime_s=None if not c else float(states[end]['sim_time_s'])-float(states[bindex]['sim_time_s']),
            reference_end_state_id=end,next_chunk_id=applied_order[current+1] if c and current+1<len(applied_order) else None)
        if rec['status'] not in ('PREDICTION_READY','MODEL_STOP'):
            rows.append(dict(**common,**activation,classification='TECHNICAL_UNAVAILABLE',raw_geometry=None));continue
        raw=np.load(ep/rec['raw_local_ref']['path']);world=np.load(ep/rec['world_ref']['path'])
        assert np.array_equal(raw,np.asarray(rec['response']['actions']['actions'],float))
        assert np.allclose(world,local_trajectory_to_world(obs['pose_world'],raw),rtol=0,atol=1e-10)
        geom=path_geometry(world,on);actual=geom if metadata['cart_present'] else path_geometry(world,base)
        motion=hallway_geometry(world,obs['pose_world'],center,forward,scenario['cart_extents'],cfg)
        label=classify_chunk(motion,geom['whole']['clearance_valid'],bool(rec['response'].get('stop')),free_sides,scenario['cart_extents'],cfg)
        if c:
            segment=poses[bindex:end+1];execution=environment.check_polyline(segment)
            bounds=[held_interval_check(environment,a,b)['minimum_clearance_lower_bound_m'] for a,b in zip(segment[:-1],segment[1:])]
            execution['minimum_curve_bounded_clearance_m']=min(bounds) if bounds else execution['minimum_clearance_m']
        else:execution=None
        inflight=[s for s in states if rec['t_request_host']['monotonic_ns']<=int(s['host_monotonic_ns'])<=rec['t_ready_host']['monotonic_ns']]
        rtf=None
        if len(inflight)>=2:rtf=(float(inflight[-1]['sim_time_s'])-float(inflight[0]['sim_time_s']))/((int(inflight[-1]['host_monotonic_ns'])-int(inflight[0]['host_monotonic_ns']))/1e9)
        rows.append(dict(**common,**activation,**motion,classification=label,raw_geometry=geom,actual_input_scene_geometry=actual,
            raw_safe=bool(geom['whole']['clearance_valid']),actual_scene_raw_safe=bool(actual['whole']['clearance_valid']),
            onset=label in ('SAFE_BYPASS_ONSET','SAFE_BYPASS'),full_bypass=label=='SAFE_BYPASS',
            observation_cart_edge_clearance_m=float(cart.distance(Point(obs['pose_world'][:2]))-.2),
            raw_local_ref=rec['raw_local_ref'],world_ref=rec['world_ref'],raw_local=raw.tolist(),world=world.tolist(),
            stop=bool(rec['response'].get('stop')),raw_text=rec['response'].get('raw_text'),pointing=rec['response'].get('pointing'),
            execution=execution,inflight_state_count=len(inflight),inflight_rtf=rtf,
            inflight_max_state_gap_s=max(np.diff([int(s['host_monotonic_ns']) for s in inflight])/1e9,default=0.) if inflight else None,
            inflight_timing_valid=rtf is not None and .8<=rtf<=1.2))
    whole=environment.check_polyline(poses)
    exact_checks=[held_interval_check(environment,a,b) for a,b in zip(poses[:-1],poses[1:])]
    first_event=lambda pred:next(({k:r.get(k) for k in ['chunk_id','t_obs','t_apply','observation_cart_center_distance_m','B','B_cart_edge_clearance_m',
        'observation_to_B_travel_m','old_chunk_id','classification']} for r in rows if r.get('received_before_end') and pred(r)),None)
    loop=jsonlines(ep/'loop.jsonl')
    summary=dict(episode=episode,termination=metadata['status'],termination_reason=metadata['termination_reason'],
        outcome=episode_outcome(rows,metadata['status']),predictions=len(rows),activated_chunks=len(first),
        activated_after_C0=metadata['activated_handoffs'],capture_count=metadata['captured_frames'],
        simulation_duration_s=float(states[-1]['sim_time_s'])-float(states[0]['sim_time_s']),
        actual_execution=whole,actual_steps_all_clearance_valid=all(x['clearance_valid'] for x in exact_checks),
        actual_minimum_curve_bounded_clearance_m=min([x['minimum_clearance_lower_bound_m'] for x in exact_checks],default=whole['minimum_clearance_m']),
        final_pose=poses[-1].tolist(),final_hallway_longitudinal_m=float((poses[-1,:2]-center)@forward),
        physically_past_rear=bool((poses[-1,:2]-center)@forward>=scenario['cart_extents'][1]+.25),
        first_visible_cart=first_event(lambda r:(r.get('cart_pixels') or 0)>=20),
        first_visible_capture=next((r for r in vis.values() if r['instance']['visible_pixels']>=20),None),
        first_raw_reaches_influence=first_event(lambda r:r.get('raw_reaches_influence',False)),
        first_safe_shorten=first_event(lambda r:r['classification']=='SAFE_SHORTEN'),
        first_onset=first_event(lambda r:r.get('onset',False)),
        first_bypass=first_event(lambda r:r.get('full_bypass',False)),
        episode_rtf=metadata['episode_rtf'],max_loop_stall_s=max([r['loop_interval_host_s'] for r in loop],default=0.),
        MPC_solved_results=len(solves),MPC_time_s=sum(x.get('official_solve_ms',0.) for x in solves.values())/1000,
        MPC_accepted_submissions=sum(e.get('type')=='reply' and e.get('status')=='submitted' and e.get('episode_id')==episode for e in events),
        model_terminal_RTT_sum_s=sum(r.get('client_rtt_s') or 0 for r in rows),
        buffer_requests=sum(r['kind']=='buffer_only' for r in records),
        raw_reference_unsafe_never_triggered_abort=True)
    return dict(rows=rows,summary=summary)

def analyze(run):
    data=[analyze_episode(run,e) for e in ORDER];rows=[r for x in data for r in x['rows']]
    match=[]
    for r in rows:
        if not r['episode'].startswith('ON_') or 'observation_longitudinal_m' not in r:continue
        off=[o for o in rows if o['episode']=='OFF_'+r['episode'][3:] and 'observation_longitudinal_m' in o]
        nearest=min(off,key=lambda o:abs(o['observation_longitudinal_m']-r['observation_longitudinal_m']),default=None)
        distance=None if nearest is None else abs(nearest['observation_longitudinal_m']-r['observation_longitudinal_m'])
        angle=None if nearest is None else abs(float(wrap_angle(nearest['observation'][2]-r['observation'][2])))
        valid=distance is not None and distance<=.25 and angle<=np.deg2rad(20.)
        match.append(dict(ON=r['episode']+'/'+r['chunk_id'],OFF=None if not valid else nearest['episode']+'/'+nearest['chunk_id'],
            longitudinal_gap_m=distance,yaw_gap_rad=angle,matched=valid,
            reason='descriptive nearest longitudinal match <=.25m and heading<=20deg' if valid else 'no comparable observed OFF pose; no forced index match'))
    chosen=candidate_source(rows)
    on_results=[x['summary']['outcome'] for x in data if x['summary']['episode'].startswith('ON_')]
    overall=on_results[0] if len(set(on_results))==1 else 'MIXED_ONLINE_RESULT'
    if overall=='NATURAL_MODEL_STOP_BEFORE_BYPASS':overall='PROGRESSIVE_SHORTENING_OR_STOP'
    if overall=='EPISODE_LIMIT_BEFORE_CONCLUSION':overall='INCONCLUSIVE'
    return dict(episodes=data,OFF_matches=match,selected_source=chosen,overall=overall)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    result=analyze(run);save(run/'aggregate/analysis.json',result)
    rows=[r for ep in result['episodes'] for r in ep['rows']]
    fields=['episode','chunk_id','classification','t_obs','t_ready','t_apply','observation_cart_center_distance_m','B_cart_edge_clearance_m',
        'observation_to_B_travel_m','raw_arc_m','max_lateral_m','final_yaw_relative_hallway_rad','endpoint_from_front_m','endpoint_from_rear_m',
        'raw_safe','reference_lifetime_s','old_chunk_id','next_chunk_id','stop']
    with (run/'aggregate/chunks.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields+['raw_min_clearance_m','execution_min_clearance_m']);w.writeheader()
        for r in rows:w.writerow(dict(**{k:r.get(k) for k in fields},
            raw_min_clearance_m=(r.get('raw_geometry') or {}).get('whole',{}).get('minimum_clearance_m'),
            execution_min_clearance_m=(r.get('execution') or {}).get('minimum_curve_bounded_clearance_m')))
    candidates=[r for r in rows if r['episode'].startswith('ON_') and r.get('onset') and r.get('received_before_end')]
    for r in candidates:
        r['preceding_OLD_record']=next((o for o in rows if o['episode']==r['episode'] and o['chunk_id']==r['old_chunk_id']),None)
    save(run/'source_bundle/manifest.json',dict(label='RECONCILIATION_CANDIDATE_SOURCE_ONLY',candidates=candidates,
        selected=result['selected_source'],moving_handoff_available=bool(result['selected_source'] and result['selected_source']['moving_B'] and result['selected_source']['old_chunk_id']),
        no_optimization=True,geometry=read(run/'scenario.json'),source_hashes={str(p):sha(p) for p in (run/'episodes').glob('*/completion.json')}))
    print(json.dumps([x['summary'] for x in result['episodes']],indent=2))
if __name__=='__main__':main()
