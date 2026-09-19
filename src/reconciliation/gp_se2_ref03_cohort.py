"""Source-only deterministic REF-03 transfer cohort; no controller or GP solve.

Original GP-SE2-01 eligibility is authoritative. New grouping uses only preserved
raw suffix geometry, clearance and original mismatch diagnostics. Controls are
reported separately and do not initialize additional-cohort diversity priority.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .gp_se2_reference import geometric_group_flags, poses, prepare_reference
from .se2 import local_trajectory_to_world, wrap_angle

GROUPS = ('O', 'R', 'P', 'S')
STRATA = dict(zip(GROUPS, ('O_OBSTACLE_NEAR', 'R_LARGE_ROTATION', 'P_LARGE_MISMATCH', 'S_BENIGN_STRAIGHT')))
TARGET_PER_GROUP = 6
CONTROL_CASES = (
    ('REPRO_LARGE_TURN', 'episode_014_repeat_01/handoff_026'),
    ('REPRO_BENIGN', 'episode_001_repeat_01/handoff_002'),
    ('KNOWN_OBSTACLE_STRESS', 'episode_017_repeat_00/handoff_007'),
)
REQUIRED_PRIOR_MANIFESTS = ('GP01', 'GP02', 'REF01', 'REF02')
HASH_PREFIX = 'REF03-v1:'


def cohort_policy():
    return dict(version='REF03-v1', source_count=881, group_order=list(GROUPS), target_per_group=TARGET_PER_GROUP,
        controls=[dict(control_role=role, case_id=case) for role, case in CONTROL_CASES],
        eligibility='unchanged original GP-SE2-01 all_source_decisions; raw-source integrity is a separate prerequisite',
        exclusions='union of GP01, GP02, REF01, REF02 selected event IDs from their original manifests',
        O=dict(original_raw_future_suffix_clearance_valid=True, maximum_footprint_edge_clearance_m=.15,
               boundary_connector_included=False, original_required_clearance_m=.05, original_footprint_radius_m=.20),
        R=dict(minimum_internal_absolute_wrapped_yaw_deg=45., rotation_dominant_maximum_arc_m=.20,
               rotation_dominant_minimum_yaw_deg=30., rotation_dominant_is_alternative_eligibility=False),
        P=dict(minimum_existing_e_perp_m=.10, minimum_reliable_existing_window_difference_deg=30.),
        S='unchanged original GP-SE2-01 A_SMALL_STRAIGHT',
        greedy_priority=['episode not yet selected in additional cohort', 'ordered raw pair not yet selected in additional cohort',
                         'R only: rotation_dominant first', "lexical sha256('REF03-v1:'+case_id)", 'case_id'],
        diversity_sets_initialized_from='additional selections only; controls are separate and do not seed sets',
        duplicate_case_selection=False, group_replacement=False, threshold_relaxation=False,
        performance_or_rollout_results_used=False, synthetic_substitution=False,
        suffix_geometry='unchanged prepare_reference native[k:] internal segments only; no B connector; metres/radians',
        source_progress='source row order only, not physical time or distance')


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _read(path):
    return json.loads(Path(path).read_text())


def prior_exclusion_inventory(manifests):
    if set(manifests) != set(REQUIRED_PRIOR_MANIFESTS):
        raise ValueError('exact GP01/GP02/REF01/REF02 manifest coverage required')
    membership = {}
    counts = {}
    for label in REQUIRED_PRIOR_MANIFESTS:
        selected = manifests[label]['selected']
        ids = [row['case_id'] for row in selected]
        if len(ids) != len(set(ids)):
            raise ValueError('duplicate selected case in historical manifest: '+label)
        counts[label] = len(ids)
        for case in ids:
            membership.setdefault(case, []).append(label)
    return dict(case_ids=sorted(membership), membership=dict(sorted(membership.items())),
                selected_counts=counts, union_count=len(membership), exclusion_basis='event identity, not episode or raw-pair exclusion')


def suffix_features(native, boundary, metrics, original_row, config, suffix_environment):
    """Compute grouping on unchanged source suffix, never on a new rollout."""
    prepared = prepare_reference(native, boundary, [10., 10., 1.],
        first_time_s=config['reference']['first_time_s'], horizon_s=config['reference']['final_time_s'],
        output_dt_s=config['reference']['output_dt_s'])
    suffix = prepared['suffix_world']
    arc = float(np.linalg.norm(np.diff(suffix[:, :2], axis=0), axis=1).sum())
    yaw = float(np.rad2deg(np.abs(wrap_angle(np.diff(suffix[:, 2]))).sum()))
    flags = geometric_group_flags(metrics, original_row['raw_shape'], config['selection'])
    for key, value in flags.items():
        if original_row['groups'][key] != value:
            raise ValueError('original geometric grouping changed: '+key)
    e_perp = float(metrics['e_perp_m'])
    angle = metrics['abs_e_dir_window_deg']
    if not math.isfinite(e_perp) or e_perp < 0 or (angle is not None and (not math.isfinite(angle) or angle < 0)):
        raise ValueError('nonfinite/invalid original mismatch diagnostic')
    if original_row['e_perp_m'] != e_perp or original_row['direction_difference_deg'] != angle:
        raise ValueError('catalog mismatch diagnostics differ from authenticated source metrics')
    clearance = suffix_environment['minimum_clearance_m']
    if not math.isfinite(clearance):
        raise ValueError('nonfinite original suffix clearance')
    rotation_dominant = bool(arc <= .20 and yaw >= 30.)
    groups = dict(O=bool(suffix_environment['clearance_valid'] and clearance <= .15), R=bool(yaw >= 45.),
                  P=bool(e_perp >= .10 or (flags['reliable_direction'] and angle is not None and angle >= 30.)),
                  S=bool(flags['A_SMALL_STRAIGHT']))
    return dict(nearest_original_row_index=prepared['nearest_row_index'],
        first_future_original_row_index=prepared['first_future_row_index'],
        original_suffix_row_indices=prepared['original_row_indices'], native_row_count=len(native), suffix_row_count=len(suffix),
        suffix_xy_arc_m=arc, suffix_internal_accumulated_abs_wrapped_yaw_deg=yaw,
        suffix_minimum_footprint_edge_clearance_m=float(clearance), original_e_perp_m=e_perp,
        original_window_direction_difference_deg=angle, original_window_direction_reliable=flags['reliable_direction'],
        original_A_SMALL_STRAIGHT=flags['A_SMALL_STRAIGHT'], rotation_dominant=rotation_dominant,
        group_memberships=groups, boundary_connector_included=False,
        angular_geometry='sum abs(wrap(yaw[i+1]-yaw[i])) for retained original suffix rows only')


def select_additional(rows):
    """Exclusive greedy assignment, scarce groups first, with no shortfall refill."""
    ids = [row['case_id'] for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate source event IDs')
    blockers = [row['case_id'] for row in rows if not row['source_integrity_valid']]
    if blockers:
        raise ValueError('source-integrity blocker prevents cohort freeze; not a scientific shortfall')
    used, episodes, pairs = set(), set(), set()
    selected, trace, shortfalls = [], [], []
    for group in GROUPS:
        pool = [row for row in rows if row['additional_eligible'] and row['group_memberships'][group] and row['case_id'] not in used]
        available = len(pool)
        count = 0
        for _ in range(TARGET_PER_GROUP):
            if not pool:
                break
            def key(row):
                return (row['episode_id'] in episodes, row['ordered_raw_pair'] in pairs,
                        not row['rotation_dominant'] if group == 'R' else False,
                        hashlib.sha256((HASH_PREFIX+row['case_id']).encode('utf-8')).hexdigest(), row['case_id'])
            row = min(pool, key=key)
            priority = key(row)
            selected.append({**deepcopy(row), 'cohort_role':'ADDITIONAL_TRANSFER', 'control_role':None,
                             'cohort':'ADDITIONAL_TRANSFER', 'case_role':STRATA[group], 'stratum':STRATA[group],
                             'selected_group':group, 'assigned_group':group, 'selection_ordinal':len(selected)})
            trace.append(dict(selection_ordinal=len(trace), case_id=row['case_id'], selected_group=group,
                candidate_pool_size=len(pool), used_episode_before=row['episode_id'] in episodes,
                used_ordered_raw_pair_before=row['ordered_raw_pair'] in pairs, rotation_dominant=row['rotation_dominant'],
                priority_key=list(priority), episode_id=row['episode_id'], ordered_raw_pair=row['ordered_raw_pair']))
            used.add(row['case_id']); episodes.add(row['episode_id']); pairs.add(row['ordered_raw_pair'])
            pool.remove(row); count += 1
        shortfalls.append(dict(group=group, target=TARGET_PER_GROUP, available_after_previous_groups=available,
            selected_count=count, shortfall=TARGET_PER_GROUP-count,
            reason=None if count==TARGET_PER_GROUP else 'insufficient eligible unselected members under frozen thresholds; no replacement or relaxation'))
    return dict(selected=selected, selection_trace=trace, group_shortfalls=shortfalls,
                distinct_episode_count=len(episodes), distinct_ordered_raw_pair_count=len(pairs),
                diversity_scope='additional cohort only, globally updated across O/R/P/S')


def build_cohort(gp01_manifest_path, prior_manifest_paths, *, source_root, environment, original_config,
                 expected_source_hashes, other_known_source_usage=None):
    """Authenticate source-only evidence and return an auditable immutable plan.

All reads are authenticated against the caller's preserved absolute-path hash
inventory. Acquisition context and array hashes are also checked against each
episode's completion manifest. Metrics are derived records authenticated by the
preserved inventory. No controller instance, historical outcome, optimizer,
rendering operation or array write occurs here.
    """
    root=Path(source_root).resolve(); manifest_path=Path(gp01_manifest_path).resolve()
    expected={str(Path(p).resolve()):h for p,h in expected_source_hashes.items()}
    hashes={}; completions={}; blockers=[]
    def checked(path):
        path=Path(path).resolve(); key=str(path)
        if key not in expected:
            raise ValueError('source hash inventory has no authenticated entry: '+key)
        actual=digest(path)
        if actual!=expected[key]:
            raise ValueError('preserved source SHA256 mismatch: '+key)
        hashes[key]=actual
        return path
    try:
        manifest=_read(checked(manifest_path))
        prior={label:_read(checked(path)) for label,path in prior_manifest_paths.items()}
        if prior.get('GP01') != manifest:
            raise ValueError('GP01 exclusion manifest differs from authoritative catalog')
        excluded=prior_exclusion_inventory(prior)
    except (OSError,ValueError,KeyError,TypeError) as error:
        return dict(status='SOURCE_INTEGRITY_BLOCKED', source_integrity_valid=False,
            source_integrity_blockers=[dict(case_id=None,reason=f'{type(error).__name__}: {error}')],
            policy=cohort_policy(), all_candidates=[], controls=[], additional=[], selected=[], source_hashes=hashes)
    catalog=manifest['all_source_decisions']
    ids=[r['case_id'] for r in catalog]
    if len(catalog)!=881 or len(set(ids))!=881 or manifest['source_event_count']!=881:
        raise ValueError('original authoritative 881-event catalog coverage required')
    if sum(bool(r['eligible']) for r in catalog)!=manifest['eligible_count']:
        raise ValueError('original eligible count is inconsistent')
    fp=original_config['footprint']; ref=original_config['reference']
    if (fp['radius_m'],fp['required_clearance_m'],ref['first_time_s'],ref['final_time_s'],ref['output_dt_s'])!=(.2,.05,.1,3.,.1):
        raise ValueError('original footprint/clearance/reference configuration mismatch')
    if environment.numerical_tolerance_m != 1e-7:
        raise ValueError('original direct geometry numerical uncertainty changed')
    known=other_known_source_usage or {}
    candidates=[]
    for original in sorted(catalog,key=lambda row:row['case_id']):
        case=original['case_id']; episode_id,handoff_id=case.split('/')
        row=dict(case_id=case,case_directory=original['case_directory'],episode_id=episode_id,handoff_id=handoff_id,
            ordered_raw_pair=original['ordered_raw_pair'], original_gp01_eligible=bool(original['eligible']),
            original_gp01_rejection_reasons=list(original['rejection_reasons']), original_gp01_groups=deepcopy(original['groups']),
            original_route_status=original['route_status'], route_status=original['route_status'], prior_selected_in=excluded['membership'].get(case,[]),
            other_known_source_usage=list(known.get(case,[])), source_integrity_valid=True, source_integrity_error=None,
            group_memberships={g:False for g in GROUPS}, rotation_dominant=False, source_paths={},source_hashes={})
        try:
            episode=root/'episodes'/episode_id
            if episode_id not in completions:
                cp=checked(episode/'completion.json')
                completion=_read(cp)['raw_manifest']['files']
                completions[episode_id]={r['path']:r for r in completion}
            def acquisition(relative):
                path=(episode/relative).resolve()
                if not path.is_relative_to(episode.resolve()):
                    raise ValueError('acquisition path escapes original episode')
                checked(path)
                record=completions[episode_id].get(relative)
                if record is None or record['sha256']!=hashes[str(path)] or record['bytes']!=path.stat().st_size:
                    raise ValueError('acquisition completion-manifest mismatch: '+relative)
                row['source_hashes'][str(path)]=hashes[str(path)]
                return path
            context_path=acquisition(f'handoffs/{handoff_id}/context.json')
            metrics_path=checked(episode/f'handoffs/{handoff_id}/metrics.json')
            context,metrics=_read(context_path),_read(metrics_path)
            if context['episode_id']!=episode_id or metrics['episode_id']!=episode_id or metrics['event_id']!=handoff_id:
                raise ValueError('catalog/source event identity mismatch')
            row['source_paths'].update(context=str(context_path),metrics=str(metrics_path))
            row['source_hashes'][str(metrics_path)]=hashes[str(metrics_path)]
            arrays={}
            for kind in ('old','fresh'):
                for frame in ('raw_local','world'):
                    record=context[f'{kind}_{frame}_ref']; path=acquisition(record['path'])
                    if hashes[str(path)]!=record['sha256']:
                        raise ValueError('context array SHA256 differs from original acquisition')
                    key=kind+'_'+frame;row['source_paths'][key]=str(path)
                    arrays[key]=poses(np.load(path,allow_pickle=False))
                if context[f'{kind}_raw_local_ref']['sha256']!=metrics[f'{kind}_raw_sha256']:
                    raise ValueError('raw-pair metric differs from acquisition bytes')
            pair=metrics['old_raw_sha256']+':'+metrics['fresh_raw_sha256']
            if pair!=original['ordered_raw_pair']:
                raise ValueError('ordered OLD/FRESH raw pair changed')
            transformed=local_trajectory_to_world(context['R_obs'],arrays['fresh_raw_local'])
            if not np.allclose(transformed[:,:2],arrays['fresh_world'][:,:2],atol=1e-12,rtol=0) or not np.allclose(wrap_angle(transformed[:,2]-arrays['fresh_world'][:,2]),0.,atol=1e-12,rtol=0):
                raise ValueError('original FRESH capture transform mismatch')
            native=arrays['fresh_world'];prepared=prepare_reference(native,context['B'],[10.,10.,1.])
            checks=dict(B_environment=environment.query(context['B'][:2]),goal_environment=environment.query(native[-1,:2]),
                raw_suffix_environment=environment.check_polyline(prepared['suffix_world']))
            for key,value in checks.items():
                if original[key]!=value:
                    raise ValueError('original environment predicate changed: '+key)
            timing=metrics['interval_timing'];inflight=timing['inference_client_host_interval']
            timing_ok=bool(timing['real_time_pacing_valid'] and timing['causal_execution_overlap_observed'] and timing['actual_post_switch_execution_available'] and inflight['capture_count']>0)
            if timing_ok!=original['exact_timing_valid']:
                raise ValueError('original exact timing eligibility changed')
            features=suffix_features(native,context['B'],metrics,original,original_config,checks['raw_suffix_environment'])
            row.update(features)
            row['source_only_features']=deepcopy(features)
            row['original_suffix_environment']=checks['raw_suffix_environment']
            row['native_goal_world']=native[-1].tolist()
            row['original_route_source']='authoritative GP01 catalog route_status; root reconstructs selected exact gate records using unchanged route_for_source'
            row['selection_hash']=hashlib.sha256((HASH_PREFIX+case).encode()).hexdigest()
        except (OSError,ValueError,KeyError,TypeError) as error:
            row['source_integrity_valid']=False
            row['source_integrity_error']=f'{type(error).__name__}: {error}'
            blockers.append(dict(case_id=case,reason=row['source_integrity_error']))
        reasons=[]
        if not row['source_integrity_valid']: reasons.append('SOURCE_INTEGRITY_BLOCKER')
        if not row['original_gp01_eligible']: reasons.append('ORIGINAL_GP01_INELIGIBLE')
        if row['prior_selected_in']: reasons.append('PREVIOUSLY_SELECTED_EVENT')
        row['additional_eligible']=not reasons
        row['additional_exclusion_reasons']=reasons
        candidates.append(row)
    lookup={r['case_id']:r for r in candidates};controls=[]
    for ordinal,(role,case) in enumerate(CONTROL_CASES):
        if case not in lookup or not lookup[case]['original_gp01_eligible'] or not lookup[case]['source_integrity_valid']:
            blockers.append(dict(case_id=case,reason='required known control missing, originally ineligible or source-integrity blocked'))
        elif not blockers:
            controls.append({**deepcopy(lookup[case]),'cohort_role':'KNOWN_CONTROL','control_role':role,
                             'cohort':'KNOWN_CONTROLS','case_role':role,'stratum':None,
                             'selected_group':None,'assigned_group':None,'selection_ordinal':ordinal})
    selection=select_additional(candidates) if not blockers else dict(selected=[],selection_trace=[],group_shortfalls=[])
    for row in selection['selected']:
        row['additional_selection_ordinal']=row['selection_ordinal']
        row['selection_ordinal']+=len(controls)
    selected_ids={r['case_id']:r['selected_group'] for r in selection['selected']}
    for row in candidates:
        row['selected_additional']=row['case_id'] in selected_ids
        row['assigned_group']=selected_ids.get(row['case_id'])
        row['nonselection_reason']=None if row['selected_additional'] else (
            'excluded: '+', '.join(row['additional_exclusion_reasons']) if row['additional_exclusion_reasons'] else
            'outside frozen O/R/P/S groups' if not any(row['group_memberships'].values()) else
            'deterministic frozen capacity/diversity priority or earlier exclusive group assignment')
    eligible=[r for r in candidates if r['additional_eligible']]
    counts={g:sum(r['group_memberships'][g] for r in eligible) for g in GROUPS}
    overlaps={g:{h:sum(r['group_memberships'][g] and r['group_memberships'][h] for r in eligible) for h in GROUPS} for g in GROUPS}
    additional=selection['selected']
    episode_counts={episode:sum(r['episode_id']==episode for r in additional) for episode in sorted({r['episode_id'] for r in additional})}
    pair_counts={pair:sum(r['ordered_raw_pair']==pair for r in additional) for pair in sorted({r['ordered_raw_pair'] for r in additional})}
    coverage=dict(source_event_count=len(candidates),original_gp01_eligible_count=sum(r['original_gp01_eligible'] for r in candidates),
        original_gp01_ineligible_count=sum(not r['original_gp01_eligible'] for r in candidates),
        historical_exclusion_union_count=excluded['union_count'],prior_selected_eligible_count=sum(r['original_gp01_eligible'] and bool(r['prior_selected_in']) for r in candidates),
        additional_eligible_count=len(eligible),additional_selected_count=len(additional),controls_count=len(controls) if not blockers else 0,
        total_selected_count=len(controls)+len(additional) if not blockers else 0,target_additional_count=24,
        group_candidate_counts=counts,group_overlap_counts=overlaps,group_shortfalls=selection['group_shortfalls'],
        group_selected_counts={g:sum(r['selected_group']==g for r in additional) for g in GROUPS},
        additional_unique_episode_count=len(episode_counts),additional_unique_ordered_raw_pair_count=len(pair_counts),
        additional_episode_counts=episode_counts,additional_ordered_raw_pair_counts=pair_counts,
        additional_repeated_episodes={key:value for key,value in episode_counts.items() if value>1},
        additional_repeated_ordered_raw_pairs={key:value for key,value in pair_counts.items() if value>1},
        duplicate_selected_case_ids=[],source_integrity_blocker_count=len(blockers),
        controls_separate_from_additional=True,held_out_claim=False,shortfall_replacement=False)
    return dict(status='SOURCE_INTEGRITY_BLOCKED' if blockers else 'COHORT_SOURCE_AUDIT_READY',policy=cohort_policy(),
        source_integrity_valid=not blockers,source_integrity_blockers=blockers,source_event_count=len(candidates),
        original_gp01_eligible_count=sum(r['original_gp01_eligible'] for r in candidates),
        additional_eligible_count=len(eligible),exclusion_inventory=excluded,all_candidates=candidates,
        controls=controls if not blockers else [],additional=selection['selected'],selected=(controls+selection['selected']) if not blockers else [],coverage=coverage,
        controls_count=len(controls) if not blockers else 0,additional_selected_count=len(selection['selected']),
        selected_count=len(controls)+len(selection['selected']) if not blockers else 0,
        group_candidate_counts=counts,group_overlap_counts=overlaps,group_shortfalls=selection['group_shortfalls'],
        selection_trace=selection['selection_trace'],selection_used_rollout_or_optimizer_results=False,
        controls_seed_diversity=False,source_hashes=dict(sorted(hashes.items())),
        original_manifest_path=str(manifest_path),source_root=str(root),
        environment_numerical_uncertainty_m=environment.numerical_tolerance_m,
        other_known_source_usage_is_exclusion=False,
        limitations='development-corpus source-only diagnostic transfer; not held-out evaluation or prevalence estimation')
