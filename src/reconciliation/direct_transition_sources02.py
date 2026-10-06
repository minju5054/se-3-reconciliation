"""Source-only inventory. Reads identity manifests, geometry and acquisition clocks.

No historical method metrics, optimization results or execution outcomes are
opened. Existing acquisition streams are used only to authenticate the B state.
"""
import csv
from pathlib import Path
import numpy as np
from .gp_se2_attach01_source import read, digest
from .genuine_source_scan import motion_record
from .gp_se2_environment import HospitalEnvironment
from .spatial_entry_suffix import suffix_reference
from .boundary_row_ablation04 import staged_reference
from .se2 import local_trajectory_to_world, wrap_angle
from .direct_transition_hard_eval02 import severity, meaningful_transition, SelectionRecord, select


from .staging_eval_sources import authenticate, exclusion_registry as base_registry
from .staging_graph_necessity_eval01 import eligibility


def exclusion_registry(root,cfg):
    registry=base_registry(root,cfg)
    rel=cfg['previous_evaluation_manifest']
    assert digest(root/rel)==cfg['authority_hashes'][rel]
    previous=read(root/rel)
    assert {r['id'].split('/')[0] for r in previous}==set(cfg['excluded_previous_episodes'])
    for r in previous:
        cid=r['id'];episode=cid.split('/')[0]
        evidence=[dict(reason='previous STAGING_GRAPH_NECESSITY_EVAL_01 evaluation; identity only',evidence=rel,sha256=digest(root/rel))]
        registry['exact_source_membership'].setdefault(cid,[]).extend(evidence)
        registry['excluded_episodes'].setdefault(episode,[]).append(dict(case_id=cid,evidence=evidence))
    registry['previous_evaluation_episodes']=cfg['excluded_previous_episodes']
    return registry


def scan(root,cfg,registry):
    catalog,provenance,hashes=authenticate(root,cfg)
    env=HospitalEnvironment.load(provenance['environment_path']);cache={};rows=[]
    for source in sorted(catalog['all_candidates'],key=lambda r:r['case_id']):
        cid=source['case_id'];episode=cid.split('/')[0];paths=source['source_paths'];c=read(paths['context'])
        ep=Path(paths['context']).parents[2]
        if episode not in cache:
            completion=ep/'completion.json';manifest={r['path']:r for r in read(completion)['raw_manifest']['files']}
            hashes[str(completion)]=digest(completion);streams=[]
            for name in ['execution.csv','commands.csv','controller/events.jsonl','metadata.json']:
                p=ep/name;h=digest(p)
                assert manifest[name]['sha256']==h and manifest[name]['bytes']==p.stat().st_size
                hashes[str(p)]=h
                if name.endswith('.csv'):
                    with p.open() as f:streams.append(list(csv.DictReader(f)))
            cache[episode]=(streams[0],{int(r['command_id']):r for r in streams[1]})
        motion=motion_record(c,*cache[episode]);states=cache[episode][0]
        P=np.array([float(states[c['switch_state_id']-1][k]) for k in ('x','y','yaw')])
        np.testing.assert_array_equal(P,c['P'])
        arrays={k:np.load(paths[k],allow_pickle=False) for k in ['old_raw_local','old_world','fresh_raw_local','fresh_world']}
        finite=all(a.ndim==2 and a.shape[1]==3 and len(a)>=2 and np.isfinite(a).all() for a in arrays.values())
        for prefix,A in [('old',c['R_old_obs']),('fresh',c['R_obs'])]:
            computed=local_trajectory_to_world(A,arrays[prefix+'_raw_local']);stored=arrays[prefix+'_world']
            np.testing.assert_allclose(computed[:,:2],stored[:,:2],atol=1e-12,rtol=0)
            np.testing.assert_allclose(wrap_angle(computed[:,2]-stored[:,2]),0,atol=1e-12,rtol=0)
        for k in ['t_obs','t_request','t_ready_host','t_install','t_switch']:
            assert c[k] and np.isfinite(c[k]['host_monotonic_s'])
        raw_metrics=read(paths['metrics']);timing=raw_metrics['interval_timing']
        valid_timing=bool(timing['real_time_pacing_valid'] and timing['causal_execution_overlap_observed']
            and timing['actual_post_switch_execution_available'] and timing['inference_client_host_interval']['capture_count']>0)
        native,raw=arrays['fresh_world'],arrays['fresh_raw_local'];bits=raw.tobytes()
        entire=env.check_polyline(native);boundary=env.query(c['B'][:2])
        past=env.check_trajectory(motion['times'],motion['states'],curved_path_error_bound_m=motion['curve_bound_m'])
        entry=features=stage_check=None;error=None;suffix_rows=0;remaining=0.
        try:
            suffix,_,entry=suffix_reference(native,raw,c['R_obs'],c['B'])
            stage,_,_=staged_reference(native,raw,c['R_obs'],c['B'],entry,suffix)
            features=severity(P,c['B'],suffix);stage_check=env.check_polyline(stage)
            suffix_rows=len(suffix);remaining=entry['correspondence']['remaining_arc_m']
        except ValueError as exc:
            error=str(exc)
        assert raw.tobytes()==bits
        gates,reasons=eligibility(valid=finite and valid_timing,fresh_safe=entire['clearance_valid'],
            B_safe=boundary['status']=='CLEARANCE_VALID',history_safe=past['clearance_valid'],
            v_minus=motion['v_minus_mps'],travel=motion['observation_to_B_arc_m'],remaining=remaining,
            suffix_rows=suffix_rows,stage_safe=stage_check is not None and stage_check['clearance_valid'],
            geometry_defined=features is not None,excluded=episode in registry['excluded_episodes'],config=cfg['selection'])
        base_eligible=not reasons
        gates['meaningful_direct_transition']=meaningful_transition(features,cfg['selection'])
        if not gates['meaningful_direct_transition']:reasons.append('meaningful_direct_transition')
        rows.append(dict(base_eligible=base_eligible,case_id=cid,episode_id=episode,source_paths=paths,eligible=not reasons,gates=gates,
            rejection_reasons=reasons,geometry_error=error,severity=features,remaining_arc_m=remaining,suffix_rows=suffix_rows,
            entry=entry,entire_FRESH_safety=entire,B_safety=boundary,history_safety=past,B_ENTRY_safety=stage_check,
            v_minus_mps=motion['v_minus_mps'],physical_omega_minus_radps=motion['omega_minus_radps'],
            observation_to_B_travel_m=motion['observation_to_B_arc_m'],A=c['R_obs'],P=P.tolist(),B=c['B'],
            timing={k:c[k] for k in ['t_obs','t_request','t_ready_host','t_install','t_switch']},
            raw_value_sha256=__import__('hashlib').sha256(raw.tobytes()).hexdigest(),
            flags={'candidate':not reasons}))
    records=[SelectionRecord(r['case_id'],r['episode_id'],r['eligible'],
        *[None if r['severity'] is None else r['severity'][k] for k in
          ['delta_phi_rad','rho_jump','B_to_entry_chord_m','d_F_m']],r['remaining_arc_m']) for r in rows]
    decision=select(records)
    decision.update(eligible_before_meaningful_gate=sum(r['base_eligible'] for r in rows),
        meaningful_transition_candidates=sum(r['eligible'] for r in rows),
        excluded_episodes=len(registry['excluded_episodes']),saved_records_inspected=len(rows))
    return dict(rows=rows,selection=decision,source_hashes=hashes,environment_path=provenance['environment_path'])
