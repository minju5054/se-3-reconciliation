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
from .staging_graph_necessity_eval01 import severity, eligibility, SelectionRecord, select


def authenticate(root, cfg):
    for rel,h in cfg['authority_hashes'].items():
        if digest(root/rel) != h:
            raise ValueError('authority hash mismatch: '+rel)
    inv=root/cfg['inventory_run']; catalog=read(inv/'candidate_catalog.json')
    assert catalog['source_integrity_valid'] and catalog['source_event_count']==881
    assert digest(inv/'candidate_catalog.json')==read(inv/'execution_freeze.json')['input_sha256']['candidate_catalog.json']
    hashes=dict(catalog['source_hashes']); provenance=read(inv/'source.json')
    for p,h in hashes.items():
        if digest(p)!=h: raise ValueError('source hash mismatch: '+p)
    for p,h in provenance['authoritative_validations'].items():
        assert digest(p)==h and read(p)['valid']
        hashes[p]=h
    for item in provenance['environment_file_hashes']:
        p=str(Path(provenance['environment_path'])/item['path'])
        assert digest(p)==item['sha256'];hashes[p]=item['sha256']
    hashes.update({str(root/p):h for p,h in cfg['authority_hashes'].items()})
    return catalog,provenance,hashes


def exclusion_registry(root,cfg):
    membership={}
    def add(cid,reason,evidence):
        membership.setdefault(cid,[]).append(dict(reason=reason,evidence=evidence,sha256=digest(root/evidence)))
    for rel in cfg['development_manifests']:
        for row in read(root/rel)['selected']:
            add(row['case_id'],'prior GP/formulation/reference-selector development selection',rel)
    # All 13 also underwent saved execution-loss/delay-attribution development.
    # This inventory is source-only; no downstream outcome file is read.
    rel=cfg['moving_ledger']
    for row in read(root/rel):
        if row['flags']['candidate']:
            add(row['case_id'],'prior Genuine13 execution-loss/delay-attribution development',rel)
    for cid,rel in cfg['additional_development_sources'].items():
        assert cid in (root/rel).read_text()
        add(cid,'documented staging/Local-SE2 or GP ATTACH development',rel)
    episodes={}
    for cid,reasons in sorted(membership.items()):
        episode=cid.split('/')[0]
        episodes.setdefault(episode,[]).append(dict(case_id=cid,evidence=reasons))
    return dict(source_run=cfg['source_run'],excluded_episodes=episodes,exact_source_membership=membership,
        policy='exclude entire recorded episode including repeat; no method outcome used',
        outside_corpus=[dict(source='OSA03_R00/REPEAT_01',reason='separate OSA03 acquisition; all development, never candidates'),
            dict(source='historical EXP/DATA02',reason='different pre-cleanup acquisition lineage; absent from this 881-event corpus')],
        corpus_claim='development corpus; source/episode disjoint from registered prior use, not unseen population')


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
        rows.append(dict(case_id=cid,episode_id=episode,source_paths=paths,eligible=not reasons,gates=gates,
            rejection_reasons=reasons,geometry_error=error,severity=features,remaining_arc_m=remaining,suffix_rows=suffix_rows,
            entry=entry,entire_FRESH_safety=entire,B_safety=boundary,history_safety=past,B_ENTRY_safety=stage_check,
            v_minus_mps=motion['v_minus_mps'],physical_omega_minus_radps=motion['omega_minus_radps'],
            observation_to_B_travel_m=motion['observation_to_B_arc_m'],A=c['R_obs'],P=P.tolist(),B=c['B'],
            timing={k:c[k] for k in ['t_obs','t_request','t_ready_host','t_install','t_switch']},
            raw_value_sha256=__import__('hashlib').sha256(raw.tobytes()).hexdigest(),
            flags={'candidate':not reasons}))
    records=[SelectionRecord(r['case_id'],r['episode_id'],r['eligible'],
        None if r['severity'] is None else r['severity']['delta_phi_rad'],
        None if r['severity'] is None else r['severity']['spacing_severity_abs_log_rho']) for r in rows]
    decision=select(records)
    return dict(rows=rows,selection=decision,source_hashes=hashes,environment_path=provenance['environment_path'])
