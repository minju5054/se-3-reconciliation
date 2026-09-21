#!/usr/bin/env python3
"""Read-only completion of a nonqualifying JOIN-01 source run.

The frozen root validator included stationary bootstrap commands in a predicate
labelled 'during FRESH inference'. Preserve it; independently audit the actual
client-inflight-to-B command interval. No source, threshold, or outcome changes.
"""
import argparse,csv,json,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
from run_gp_se2_join01 import read,write,digest,verify
from reconciliation.gp_se2_join01 import METHODS
from reconciliation.gp_se2_join01_qualification import qualification_from_episode
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_diag02_validation import plain


def old_inflight_check(commands, context):
    first=context['client_inflight_state_range']['start_state_id'];last=context['switch_state_id']
    records=[c for c in commands if first<=int(c['application_state_id'])<last]
    return dict(valid=bool(records and all(c['chunk_id']==context['old_chunk_id'] for c in records)),
                first_state_id=first,switch_state_id=last,command_count=len(records),
                state_interval='client-inflight first saved state through just before FRESH command application; bootstrap excluded',
                chunk_ids=sorted({c['chunk_id'] for c in records}))


def csv_new(path,rows):
    path.parent.mkdir(exist_ok=True,parents=True)
    with path.open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def complete(run):
    source=verify(run);collection=read(run/'collection_result.json');original=read(run/'validation.json')
    if collection['selected_episode'] is not None or collection['status']!='NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF':
        raise ValueError('This report is exclusively for a completed nonqualifying source schedule')
    attempts=collection['attempts'];expected={r['placement']+' OLD commands during FRESH inference' for r in attempts}
    if set(original['errors'])!=expected:raise ValueError('unexpected original validator failure; do not bypass')
    checks=[];rows=[];base=HospitalEnvironment.load(source['environment'])
    def check(name,value):checks.append(dict(name=name,passed=bool(value)))
    check('original validator only mis-scoped bootstrap predicates',all(r['passed'] for r in original['checks'] if r['name'] not in expected))
    for r in attempts:
        ep=r['placement'];folder=run/'source_event/episodes'/ep;q=read(run/'qualification'/(ep+'.json'));ctx=q['context']
        check(ep+' source-only qualification recomputed',q==plain(qualification_from_episode(run/'source_event',ep,base,read(run/'protocol.json'))))
        for f in read(folder/'completion.json')['raw_manifest']['files']:check(ep+'/'+f['path'],digest(folder/f['path'])==f['sha256'])
        raw=read(folder/'handoffs/handoff_000/context.json');cmds=list(csv.DictReader((folder/'commands.csv').open()))
        audit=old_inflight_check(cmds,raw);check(ep+' actual inflight OLD continuation',audit['valid'])
        write(run/'verification'/(ep+'_old_interval.json'),audit)
        meta=read(folder/'metadata.json');controller=[json.loads(s) for s in (folder/'controller/events.jsonl').read_text().splitlines()]
        solves=[s for s in controller if s.get('type')=='solve_result'];mpc_ms=[s['official_solve_ms'] for s in solves if s.get('official_solve_ms') is not None]
        b=np.array(ctx['B_world']);obs=np.array(ctx['original_capture_pose_world']);fresh=np.array(ctx['fresh_world'])
        rows.append(dict(placement=ep,qualified=q['qualified'],failure_reasons=';'.join(q['failure_reasons']),
            old_observation_sim_s=raw['old_observation_pose_time']['time'],old_activation_sim_s=meta['initial_activation_sim_time_s'],
            reveal_sim_s=q['timing']['reveal']['sim_time_s'],fresh_observation_sim_s=raw['t_obs']['sim_time_s'],
            request_host_monotonic_s=raw['t_request']['host_monotonic_s'],receipt_host_monotonic_s=raw['t_ready_host']['host_monotonic_s'],
            ready_seen_sim_s=raw['t_ready_seen_sim']['sim_time_s'],application_B_sim_s=raw['t_switch']['sim_time_s'],
            request_to_receipt_s=raw['t_ready_host']['host_monotonic_s']-raw['t_request']['host_monotonic_s'],
            observation_to_B_translation_m=float(np.linalg.norm(b[:2]-obs[:2])),B_to_original_fresh_first_m=float(np.linalg.norm(b[:2]-fresh[0,:2])),
            B_clearance_m=q['B_query']['clearance_m'],OLD_future_post_reveal_clearance_m=q['revealed_old']['minimum_clearance_m'],
            FRESH_suffix_clearance_m=q['fresh_suffix']['minimum_clearance_m'],maximum_cross_track_m=q['maximum_cross_track_m'],
            maximum_projected_pose_yaw_difference_deg=q['maximum_projected_pose_yaw_difference_deg'],
            semantic_obstacle_pixels=q['visibility']['obstacle_pixels'],inflight_rtf=q['timing']['inflight_rtf'],
            genuine_prediction_requests=meta['predictions_requested'],source_MPC_solve_results=len(solves),
            source_MPC_reported_solve_wall_s=sum(mpc_ms)/1000,source_MPC_timed_results=len(mpc_ms),
            stale_results=sum(s['status']=='stale_rejected' for s in solves)))
        check(ep+' essential geometry independently disqualifies',not q['flags']['fresh_suffix_valid'] and not q['flags']['nontrivial_disagreement'])
        side=read(run/'plots'/(ep+'_source_handoff.json'))
        check(ep+' plot numbers equal source qualification',side['numeric']==q)
        check(ep+' image hash intact',side['image_sha256']==digest(run/'plots'/(ep+'_source_handoff.png')))
    check('zero reconciliation or common-B rollout',not(run/'methods').exists() and not(run/'rollouts').exists())
    check('all frozen placements covered',[r['placement'] for r in rows]==[p['id'] for p in read(run/'protocol.json')['placements']])
    result=dict(valid=all(r['passed'] for r in checks),checks=checks,
        original_validator_path='validation.json',original_validator_sha256=digest(run/'validation.json'),
        correction='OLD observation precedes bootstrap response. Test the actual client-inflight-to-B interval, not all commands since OLD observation.',
        source_qualification_unchanged=True,visibility_threshold_unchanged=True,
        no_new_vla_gp_mpc_or_runtime_calls=True,report_source_sha256=digest(Path(__file__)))
    write(run/'verification/validation.json',result)
    csv_new(run/'aggregate/qualification.csv',rows)
    methods=[dict(method=m,status='NOT_RUN_UPSTREAM_QUALIFICATION_FAILED',candidate_available=None,planned_join_time_s=None,executed_join_time_s=None,safety_motion_goal=None,optimizer_wall_s=None,comparison_mpc_wall_s=None) for m in METHODS]
    csv_new(run/'aggregate/method_ledger.csv',methods)
    summary=dict(operational_status='GP_SE2_JOIN_01_COMPLETED_WITH_LIMITATIONS',research_result='NO_QUALIFYING_OBSTACLE_REVEAL_HANDOFF',
        result_class='UPSTREAM_QUALIFICATION_FAILURE',selected_handoff=None,execution_sha=source['execution_sha'],
        actual_placements=len(rows),qualifying_placements=0,genuine_lightnav_prediction_requests=sum(r['genuine_prediction_requests'] for r in rows),
        server_startup_warmup='one separate service warmup reported in server log; not an OLD/FRESH source request',
        reconciliation_gp_solves=0,rigid_solves=0,common_B_comparison_MPC_solves=0,common_B_rollouts=0,
        source_collection_MPC_solve_results=sum(r['source_MPC_solve_results'] for r in rows),
        source_collection_MPC_reported_solve_wall_s=sum(r['source_MPC_reported_solve_wall_s'] for r in rows),
        methods=methods,placements=rows,
        visibility_limitation='All saved masks contain only BACKGROUND/UNLABELLED; 0 labelled pixels is not evidence of invisible obstacle. Actual FRESH RGB was visually inspected and shows the box. No threshold/mask/source edit or new runtime retry.',
        source_motion_limitation='Placement01 inflight RTF exceeds frozen range; placements02/03 pass. All independently fail FRESH geometry and disagreement.',
        infrastructure_limitation='M4 and method runner have implementation/derivative tests but no real-event optimization/rollout validation, because upstream gate failed.',
        remaining_uncertainty='Can this official upstream pipeline produce a genuine clearance-valid obstacle-responsive FRESH under a separately declared observation intervention?',
        no_GP_performance_conclusion=True,authoritative_validation='verification/validation.json',
        historical_MPC_test_audits=[dict(path=str(p.relative_to(run)),sha256=digest(p)) for p in sorted((run/'test_audits').glob('*.json'))])
    write(run/'summary.json',summary)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);a=p.parse_args()
    r=complete(a.run.resolve());print({'valid':r['valid'],'checks':len(r['checks'])})
    if not r['valid']:raise SystemExit(1)
