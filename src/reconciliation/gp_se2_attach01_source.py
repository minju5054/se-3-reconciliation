"""ATTACH-01 source-only eligibility. No optimizer or controller is imported.

Historical GP01 source eligibility and REF03 acquisition hashes are reused;
historical method outcomes and historical case selections are not selection inputs.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from shapely.geometry import LineString

from .gp_se2_reference import prepare_reference
from .gp_se2_rollout import load_frozen_context
from .se2 import local_trajectory_to_world, wrap_angle


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def source_order(case_id):
    return hashlib.sha256(('ATTACH01-v1:' + case_id).encode()).hexdigest(), case_id


def geometry_eligibility(native, prepared, original, metrics, environment, policy):
    """Pure source predicates; original and prepared geometry are never moved."""
    native = np.asarray(native, dtype=np.float64)
    suffix = prepared['suffix_world']
    raw_check = environment.check_polyline(suffix)
    common_check = environment.check_polyline(prepared['common_world'])
    segments = np.linalg.norm(np.diff(native[:, :2], axis=0), axis=1)
    simple = bool(len(native) >= 2 and LineString(native[:, :2]).is_simple)
    nondegenerate = bool(len(segments) and np.all(segments > policy['minimum_original_xy_segment_m']))
    future_arc = float(np.linalg.norm(np.diff(suffix[:, :2], axis=0), axis=1).sum())
    direction = metrics['abs_e_dir_window_deg']
    yaw = metrics['abs_e_yaw_deg']
    reliable_direction = bool(original['groups']['reliable_direction'])
    reliable_yaw = bool(simple and nondegenerate and yaw is not None and np.isfinite(yaw))
    mismatch = bool(metrics['e_perp_m'] >= policy['minimum_e_perp_m'] or
        (reliable_direction and direction is not None and direction >= policy['minimum_direction_or_yaw_deg']) or
        (reliable_yaw and yaw >= policy['minimum_direction_or_yaw_deg']))
    predicates = dict(
        original_source_eligibility=bool(original['eligible']),
        no_required_gate_and_known_route=original['route_status'] == 'NOT_REQUIRED_CLEAR_SHORTCUT',
        boundary_clearance=original['B_environment']['status'] == 'CLEARANCE_VALID',
        raw_future_clearance=bool(raw_check['clearance_valid']),
        common_future_clearance=bool(common_check['clearance_valid']),
        obstacle_sensitive=bool(raw_check['clearance_valid'] and
            raw_check['minimum_clearance_m'] <= policy['maximum_fresh_edge_clearance_m']),
        enough_future=bool(len(suffix) >= policy['minimum_suffix_rows'] and
            future_arc > policy['minimum_suffix_xy_arc_m'] and len(prepared['common_world']) == 30),
        nontrivial_mismatch=mismatch,
        unambiguous_projection=bool(simple and nondegenerate))
    return dict(predicates=predicates, raw_suffix_environment=raw_check,
        prepared_environment=common_check, original_e_perp_m=float(metrics['e_perp_m']),
        original_window_direction_deg=direction, direction_reliable=reliable_direction,
        original_pose_yaw_deg=yaw, pose_yaw_reliable=reliable_yaw,
        original_row_count=len(native), suffix_row_count=len(suffix), suffix_arc_m=future_arc,
        original_XY_simple=simple, original_XY_nondegenerate=nondegenerate,
        nearest_original_index=prepared['nearest_row_index'], suffix_start=prepared['first_future_row_index'],
        common_shape=list(prepared['common_world'].shape),
        common_value_sha256=hashlib.sha256(prepared['common_world'].tobytes()).hexdigest())


def scan(root, protocol, environment):
    root = Path(root).resolve()
    inventory_root = root / protocol['source_inventory_run']
    inventory_path = inventory_root / 'candidate_catalog.json'
    freeze = read(inventory_root / 'execution_freeze.json')
    if digest(inventory_path) != freeze['input_sha256']['candidate_catalog.json']:
        raise ValueError('historical source-only catalog hash mismatch')
    inventory = read(inventory_path)
    if not inventory['source_integrity_valid'] or not read(inventory_root/'validation.json')['valid']:
        raise ValueError('historical source authority not valid')
    hashes = dict(inventory['source_hashes'])
    for path, expected in hashes.items():
        if digest(path) != expected:
            raise ValueError('SOURCE_INTEGRITY_BLOCKER: ' + path)
    provenance = read(inventory_root/'source.json')
    for path, expected in provenance['authoritative_validations'].items():
        if digest(path) != expected or not read(path)['valid']:
            raise ValueError('authoritative validation changed: ' + path)
    for item in provenance['environment_file_hashes']:
        path = Path(provenance['environment_path']) / item['path']
        if digest(path) != item['sha256']:
            raise ValueError('environment changed: ' + str(path))
        hashes[str(path)] = item['sha256']
    config_path = root / protocol['original_run'] / 'config_snapshot.yaml'
    if digest(config_path) != provenance['original_config_sha256']:
        raise ValueError('original config mismatch')
    hashes[str(config_path)] = digest(config_path)
    hashes[str(inventory_path)] = digest(inventory_path)
    for name in ['execution_freeze.json', 'validation.json', 'source.json']:
        p = inventory_root/name; hashes[str(p)] = digest(p)
    manifest = read(root/protocol['original_run']/'case_manifest.json')
    source_root = root / protocol['source_run']
    source_candidates = {r['case_id']:r for r in inventory['all_candidates']}
    rows, prepared_cache, contexts = [], {}, {}
    if len(manifest['all_source_decisions']) != 881:
        raise ValueError('expected complete original 881-event source catalog')
    for original in manifest['all_source_decisions']:
        case_id = original['case_id']; ep, handoff = case_id.split('/')
        previous = source_candidates[case_id]
        context = read(previous['source_paths']['context'])
        metrics = read(previous['source_paths']['metrics'])
        native = np.load(previous['source_paths']['fresh_world'], allow_pickle=False)
        raw = np.load(previous['source_paths']['fresh_raw_local'], allow_pickle=False)
        transformed = local_trajectory_to_world(context['R_obs'], raw)
        if not np.allclose(transformed[:, :2], native[:, :2], atol=1e-12, rtol=0) or not np.allclose(
                wrap_angle(transformed[:, 2]-native[:, 2]), 0, atol=1e-12, rtol=0):
            raise ValueError('SOURCE_INTEGRITY_BLOCKER: original observation transform ' + case_id)
        # One construction per source event. The selected result is saved from this cache.
        prepared = prepare_reference(native, context['B'], [10.,10.,1.])
        result = geometry_eligibility(native, prepared, original, metrics, environment, protocol['source_selection'])
        if result['raw_suffix_environment'] != original['raw_suffix_environment'] or environment.query(context['B'][:2]) != original['B_environment']:
            raise ValueError('original direct geometry disagreement: ' + case_id)
        result['predicates']['moving_handoff'] = context['status'] == 'VALID_HANDOFF_MOVING'
        timing = metrics['interval_timing']
        exact = bool(timing['real_time_pacing_valid'] and timing['causal_execution_overlap_observed'] and
            timing['actual_post_switch_execution_available'] and timing['inference_client_host_interval']['capture_count'] > 0)
        if exact != original['exact_timing_valid']:
            raise ValueError('original timing mismatch: ' + case_id)
        result['predicates']['exact_recorded_timing'] = exact
        # Source loader independently authenticates physical command, memory, B and frames.
        # All originally eligible events are checked, regardless of the new geometry result.
        frozen = load_frozen_context(source_root, ep, handoff) if original['eligible'] else None
        result['predicates']['physical_state_provenance'] = frozen is not None
        if frozen is not None:
            for item in frozen['source_files']:
                hashes[str(source_root/item['path'])] = item['sha256']
        failed = [key for key, passed in result['predicates'].items() if not passed]
        row = dict(case_id=case_id, source_hash_order=source_order(case_id)[0], eligible=not failed,
            rejection_reasons=failed, original_rejection_reasons=original['rejection_reasons'],
            route_status=original['route_status'], source_paths=previous['source_paths'],
            ordered_raw_pair=original['ordered_raw_pair'], **result)
        rows.append(row)
        if not failed:
            prepared_cache[case_id] = prepared; contexts[case_id] = frozen
    eligible = sorted([r for r in rows if r['eligible']], key=lambda r:source_order(r['case_id']))
    selected = eligible[0]['case_id'] if eligible else None
    predicates = list(rows[0]['predicates'])
    counts = {key:sum(r['predicates'][key] for r in rows) for key in predicates}
    cumulative = {}; survivors = rows
    for key in predicates:
        survivors = [r for r in survivors if r['predicates'][key]]; cumulative[key] = len(survivors)
    return dict(status='SOURCE_FROZEN' if selected else 'NO_EXISTING_OBSTACLE_SENSITIVE_FIXED_CORRESPONDENCE_EVENT',
        selected_case_id=selected, source_event_count=len(rows), eligible_count=len(eligible),
        eligible_hash_order=[r['case_id'] for r in eligible], all_candidates=rows,
        independent_predicate_pass_counts=counts, cumulative_pass_counts=cumulative,
        source_hashes=hashes, optimizer_or_MPC_outcomes_used=False,
        prepared_selected=None if selected is None else prepared_cache[selected],
        selected_context=None if selected is None else contexts[selected],
        future_duration_semantics='>=2 original suffix rows with positive XY arc; existing 30-row .1..3s planning convention, not intrinsic VLA timing')
