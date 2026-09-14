"""Predeclared screening and descriptive duplicate-aware summaries only."""
from collections import Counter, defaultdict
import hashlib
import re

import numpy as np

from reconciliation.robotless_controlled_staleness import characterize
from reconciliation.robotless_projection_handoff import characterize_projection
from reconciliation.robotless_successive_chunks import successive_observation_poses

MOTION = {'v_mps': .25, 'omega_radps': 0., 'tau_s': [0., .2, .5, 1.]}
DISPLACEMENT = [.30, 0., 0.]
CATEGORIES = ('straight', 'left_turn', 'right_turn', 'doorway', 'route_choice')
METRICS = ('e_perp_m', 'abs_e_dir_deg', 'abs_e_yaw_deg', 'normalized_progress')
STAT_NAMES = ('min', 'median', 'p75', 'p90', 'max')


def validate_bank(config: dict) -> None:
    if config.get('stage') != 'robotless-lightnav-handoff-screening' or config.get('schema_version') != 1:
        raise ValueError('wrong screening configuration')
    if config.get('motion') != MOTION or config.get('local_displacement') != DISPLACEMENT:
        raise ValueError('screening uses the fixed controlled-motion/displacement conventions')
    episodes = config.get('episodes')
    if not isinstance(episodes, list) or len(episodes) != 30:
        raise ValueError('exactly 30 predeclared episodes required')
    ids = [e['episode_id'] for e in episodes]
    if len(set(ids)) != len(ids):
        raise ValueError('duplicate episode ID')
    if ids != [f'episode_{i:03d}' for i in range(30)]:
        raise ValueError('episode IDs must be ordered episode_000 through episode_029')
    if Counter(e['category'] for e in episodes) != Counter(dict.fromkeys(CATEGORIES, 6)):
        raise ValueError('six episodes per verified Hospital context category required')
    for episode in episodes:
        if not isinstance(episode.get('instruction'), str) or not episode['instruction'].strip():
            raise ValueError('every episode needs a fixed nonempty instruction')
        if episode.get('local_displacement') != DISPLACEMENT:
            raise ValueError('episode displacement differs from bank convention')
        _, r1 = successive_observation_poses(episode['R0'], DISPLACEMENT)
        supplied = np.asarray(episode['R1'], dtype=float)
        if supplied.shape != (3,) or not np.all(np.isfinite(supplied)) or not np.allclose(supplied, r1, rtol=0, atol=1e-12):
            raise ValueError('R1 must equal R0 * local displacement')


def ordered_pair_hash(old_hash: str, fresh_hash: str) -> str:
    if any(not isinstance(h, str) or re.fullmatch('[0-9a-f]{64}', h) is None for h in (old_hash, fresh_hash)):
        raise ValueError('raw hashes must be lowercase SHA-256 hex')
    return hashlib.sha256(b'OLD\0' + bytes.fromhex(old_hash) + b'FRESH\0' + bytes.fromhex(fresh_hash)).hexdigest()


def transition_metrics(episode: dict, r_obs, fresh_world, old_hash: str, fresh_hash: str) -> list[dict]:
    """Reuse validated controlled B and projection geometry without modification."""
    boundaries, previous = characterize(r_obs, fresh_world, **MOTION)
    rows = characterize_projection(boundaries, fresh_world, previous, MOTION, distance_atol_m=1e-12)
    pair_hash = ordered_pair_hash(old_hash, fresh_hash)
    return [{**row, 'episode_id': episode['episode_id'], 'category': episode['category'],
        'OLD_raw_sha256': old_hash, 'FRESH_raw_sha256': fresh_hash, 'pair_sha256': pair_hash} for row in rows]


def percentiles(values) -> dict:
    defined = [x for x in values if x is not None]
    if not all(np.isfinite(x) for x in defined):
        raise ValueError('nonfinite defined metric')
    stats = np.percentile(defined, [0, 50, 75, 90, 100], method='linear').tolist() if defined else [None] * 5
    return {**dict(zip(STAT_NAMES, stats, strict=True)), 'n_available': len(defined),
            'n_unavailable': len(values) - len(defined)}


def metric_distributions(rows: list[dict]) -> list[dict]:
    return [{'tau_s': tau, 'metrics': {metric: percentiles([r[metric] for r in rows if r['tau_s'] == tau])
        for metric in METRICS}} for tau in MOTION['tau_s']]


def summarize(rows: list[dict], episode_statuses: list[dict]) -> dict:
    ids = [s['episode_id'] for s in episode_statuses]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate status IDs')
    valid = {s['episode_id'] for s in episode_statuses if s['status'] == 'VALID_PAIR'}
    if {r['episode_id'] for r in rows} != valid:
        raise ValueError('metrics and valid status IDs differ')
    for episode_id in valid:
        condition_rows = [r for r in rows if r['episode_id'] == episode_id]
        if sorted(r['tau_s'] for r in condition_rows) != MOTION['tau_s']:
            raise ValueError('each valid episode needs all four controlled delays')
        if len({(r['OLD_raw_sha256'], r['FRESH_raw_sha256'], r['pair_sha256']) for r in condition_rows}) != 1:
            raise ValueError('raw pair changes across controlled delay')
    base = [r for r in rows if r['tau_s'] == 0]
    groups = defaultdict(list)
    for row in base:
        if ordered_pair_hash(row['OLD_raw_sha256'], row['FRESH_raw_sha256']) != row['pair_sha256']:
            raise ValueError('ordered pair hash mismatch')
        groups[row['pair_sha256']].append(row['episode_id'])
    pair_rows = []
    for pair_hash, episode_ids in sorted(groups.items()):
        for tau in MOTION['tau_s']:
            subset = [r for r in rows if r['episode_id'] in episode_ids and r['tau_s'] == tau]
            pair_rows.append({'pair_sha256': pair_hash, 'tau_s': tau, 'count': len(episode_ids),
                'episode_ids': sorted(episode_ids), 'categories': sorted({r['category'] for r in subset}),
                **{metric: percentiles([r[metric] for r in subset])['median'] for metric in METRICS},
                'available_counts': {metric: sum(r[metric] is not None for r in subset) for metric in METRICS}})
    ranked = sorted(groups, key=lambda key: (-len(groups[key]), key))
    dominant = ranked[0] if ranked else None
    diversity = {'attempted': len(episode_statuses), 'valid': len(valid), 'invalid': len(ids)-len(valid),
        'status_counts': dict(sorted(Counter(s['status'] for s in episode_statuses).items())),
        'category_attempted_counts': dict(sorted(Counter(s['category'] for s in episode_statuses).items())),
        'category_valid_counts': {c: sum(s['category'] == c and s['status'] == 'VALID_PAIR' for s in episode_statuses) for c in CATEGORIES},
        'unique_OLD_count': len({r['OLD_raw_sha256'] for r in base}),
        'unique_FRESH_count': len({r['FRESH_raw_sha256'] for r in base}), 'unique_ordered_pair_count': len(groups),
        'most_frequent_pair_sha256': dominant, 'most_frequent_pair_count': len(groups[dominant]) if dominant else 0,
        'most_frequent_pair_fraction': len(groups[dominant])/len(valid) if dominant else None,
        'pair_groups': {key: sorted(value) for key, value in sorted(groups.items())},
        'raw_hash_convention': 'SHA-256 of immutable raw .npy file bytes; no rounding or tolerance grouping',
        'pair_hash_convention': 'SHA256(ASCII OLD + NUL + OLD digest bytes + ASCII FRESH + NUL + FRESH digest bytes)',
        'scope': 'valid screening episodes; reset-separated sessions, not a deployment-frequency estimate'}
    return {'diversity': diversity, 'unique_pairs': pair_rows,
        'statistics': {'percentile_method': 'numpy.percentile(method=linear), q=[0,50,75,90,100]',
            'episode_weighted': metric_distributions(rows), 'unique_pair_weighted': metric_distributions(pair_rows),
            'unique_pair_rule': 'one within-pair median per metric and tau; equal weight per distinct ordered raw pair',
            'unavailable_rule': 'null metrics excluded individually, with available/unavailable counts',
            'bootstrap_or_significance_test': False}}


def select_representatives(rows: list[dict]) -> list[dict]:
    subset = [r for r in rows if r['tau_s'] == 1.]
    rules = [('max_e_perp', 'e_perp_m'), ('max_abs_e_dir', 'abs_e_dir_deg'),
             ('max_abs_e_yaw', 'abs_e_yaw_deg'), ('nearest_median_e_perp', 'e_perp_m')]
    selected = []
    for rule, metric in rules:
        available = [r for r in subset if r[metric] is not None]
        target = float(np.median([r[metric] for r in available])) if available and rule.startswith('nearest') else None
        if available:
            row = min(available, key=lambda r: (abs(r[metric]-target) if target is not None else -r[metric], r['episode_id']))
            selected.append({'rule':rule, 'episode_id':row['episode_id'], 'metric':metric,
                'value':row[metric], 'median_target':target, 'tau_s':1., 'tie_rule':'lowest episode_id'})
        else:
            selected.append({'rule':rule, 'episode_id':None, 'metric':metric, 'reason':'no defined metric at tau=1'})
    return selected
