"""Ten-request acquisition policy; historical four-request defaults remain unchanged."""
from copy import deepcopy
import numpy as np
from .se2 import wrap_angle

EPISODES = ['EPISODE_00']
CHUNKS = [f'chunk_{i:03d}' for i in range(10)]
FIGURES = ['continuous_world_episode.png', 'observation_local_evolution.png',
           'episode_timeline.png', 'request_rgb_sequence.png']


class RequestPolicy:
    """Observe existing actual activations; never select a replacement frame."""
    def __init__(self):
        self.sent = []
        self.inflight = None
        self.eligible = {}

    def capture(self, frame, active, applied):
        if frame['rendered_state_id'] % 15:
            raise ValueError('not a scheduled capture')
        if active is None or applied is None or self.inflight is not None:
            return None
        i = CHUNKS.index(active['chunk_id']) + 1
        if i >= len(CHUNKS) or len(self.sent) != i or frame['capture_sim_time_s'] <= applied:
            return None
        cid = CHUNKS[i]
        if cid not in self.eligible:
            self.eligible[cid] = deepcopy(frame)
            return cid
        return None

    def allow(self, frame):
        if len(self.sent) >= 10 or self.inflight is not None:
            return False
        expected = self.eligible.get(CHUNKS[len(self.sent)])
        return expected is not None and frame['frame_id'] == expected['frame_id']

    def send(self, frame, predict, cid=None):
        if not predict:
            if self.allow(frame):
                raise ValueError('first eligible frame would be buffered; no queued substitution')
            return
        if len(self.sent) >= 10 or cid != CHUNKS[len(self.sent)] or self.inflight is not None:
            raise ValueError('terminal budget/order/one-inflight violation')
        if self.sent and not self.allow(frame):
            raise ValueError('not exact first eligible scheduled observation')
        if not self.sent and frame['rendered_state_id'] != 45:
            raise ValueError('C0 must use fourth scheduled live frame')
        self.sent.append(cid)
        self.inflight = cid

    def received(self, cid):
        if cid != self.inflight:
            raise ValueError('unexpected terminal response')
        self.inflight = None


OPTIONAL_FIGURE = 'handoff_sequence_overview.png'
QUALIFIED = {'LONG_CONTINUOUS_SOURCE_MAX_REACHED', 'LONG_CONTINUOUS_SOURCE_TARGET_REACHED',
             'LONG_CONTINUOUS_SOURCE_QUALIFIED'}


def classify(applied_count, first_reaction, provenance, technical=False):
    if not 0 <= applied_count <= 10:
        raise ValueError('invalid prefix length')
    if technical or not provenance:
        return 'TECHNICAL_EXECUTION_BLOCKED'
    if not first_reaction:
        return 'FIRST_REACTION_NOT_QUALIFIED'
    if applied_count == 10:
        return 'LONG_CONTINUOUS_SOURCE_MAX_REACHED'
    if applied_count >= 8:
        return 'LONG_CONTINUOUS_SOURCE_TARGET_REACHED'
    if applied_count >= 6:
        return 'LONG_CONTINUOUS_SOURCE_QUALIFIED'
    return 'PARTIAL_CONTINUOUS_SOURCE'


def figure_names(applied_count):
    return FIGURES + ([OPTIONAL_FIGURE] if applied_count >= 6 else [])
