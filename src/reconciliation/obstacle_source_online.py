"""Fixed POSE11 acquisition scheduling; no path correction or optimizer."""
import numpy as np

REPETITIONS=('REPEAT_00','REPEAT_01')


def initial_pose(selected_pose, forward):
    p=np.asarray(selected_pose,float).copy()
    p[:2]-=.40*np.asarray(forward,float)
    return p.tolist()


def reveal_due(pose, selected_pose, forward, old_active, already_revealed):
    return bool(not already_revealed and old_active and
                (np.asarray(pose)[:2]-np.asarray(selected_pose)[:2])@np.asarray(forward)>=0.)


def first_post_reveal_allowed(frame, reveal_state_id, eligible_frame_id):
    return bool(reveal_state_id is not None and frame['rendered_state_id']>=reveal_state_id
                and frame['frame_id']==eligible_frame_id)


def representative(rows):
    if [r['episode_id'] for r in rows]!=list(REPETITIONS):
        raise ValueError('both predeclared repetitions must remain in ledger')
    return next((r['episode_id'] for r in rows if r['qualified']),None)
