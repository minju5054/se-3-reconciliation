"""Exact initial OLD anchor and scheduled post-application reveal; no planner."""
import numpy as np
from .obstacle_source_online import REPETITIONS, first_post_reveal_allowed, representative


def target_pose(selected):
    if selected['candidate'] != 'POSE11':
        raise ValueError('frozen POSE11 required')
    p = np.asarray(selected['pose_world'], dtype=np.float64)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('finite SE(2) pose required')
    return p.tolist()


def reveal_due(state_id, sim_time, active, old_application_sim_time, already_revealed):
    return bool(not already_revealed and state_id % 15 == 0 and
                active is not None and active['chunk_id'] == 'chunk_000' and
                old_application_sim_time is not None and sim_time > old_application_sim_time)


def classify(rows):
    if [r['episode_id'] for r in rows] != list(REPETITIONS):
        raise ValueError('both predeclared repetitions must remain present')
    if any(r['qualified'] for r in rows):
        return 'QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE'
    if not any(r.get('valid_scientific_output', False) for r in rows):
        return 'TECHNICAL_EXECUTION_BLOCKED'
    return 'POSE11_OBSTRUCTED_OLD_SOURCE_NOT_QUALIFIED'
