"""Frozen acquisition policy and saved geometry descriptors; no scientific solvers."""
from copy import deepcopy
import numpy as np
from .se2 import wrap_angle

EPISODES = ['EPISODE_00']
CHUNKS = [f'chunk_{i:03d}' for i in range(4)]
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
        if len(self.sent) >= 4 or self.inflight is not None:
            return False
        expected = self.eligible.get(CHUNKS[len(self.sent)])
        return expected is not None and frame['frame_id'] == expected['frame_id']

    def send(self, frame, predict, cid=None):
        if not predict:
            if self.allow(frame):
                raise ValueError('first eligible frame would be buffered; no queued substitution')
            return
        if len(self.sent) >= 4 or cid != CHUNKS[len(self.sent)] or self.inflight is not None:
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


def separation(a, b):
    """Symmetric maximum vertex distance to the other continuous XY polyline."""
    from .data02_online_successive import validate_raw_chunk
    a, b = validate_raw_chunk(a), validate_raw_chunk(b)
    def directed(x, y):
        if len(y) == 1:
            return float(np.linalg.norm(x[:, :2] - y[0, :2], axis=1).max())
        p, d = y[:-1, :2], np.diff(y[:, :2], axis=0)
        den = np.sum(d*d, axis=1)
        t = np.clip(np.sum((x[:, None, :2]-p)*d, axis=2)/np.where(den > 0, den, 1), 0, 1)
        return float(np.linalg.norm(x[:, None, :2]-(p+t[..., None]*d), axis=2).min(axis=1).max())
    return max(directed(a, b), directed(b, a))


def evolution(a, b, thresholds):
    from .data02_online_successive import validate_raw_chunk
    a, b = validate_raw_chunk(a), validate_raw_chunk(b)
    sep = separation(a, b)
    endpoint = b[-1] - a[-1]
    endpoint[2] = wrap_angle(endpoint[2])
    net = float(wrap_angle((b[-1,2]-b[0,2])-(a[-1,2]-a[0,2])))
    def tangent(x):
        delta = np.diff(x[:,:2], axis=0)
        delta = delta[np.linalg.norm(delta, axis=1) >= .02]
        return None if not len(delta) else float(np.arctan2(delta[0,1], delta[0,0]))
    ta, tb = tangent(a), tangent(b)
    meaningful = (sep >= thresholds['symmetric_vertex_to_polyline_separation_m'] or
        abs(endpoint[1]) >= thresholds['endpoint_lateral_difference_m'] or
        max(abs(endpoint[2]), abs(net)) >= np.radians(thresholds['wrapped_endpoint_or_net_yaw_deg']))
    return dict(local_polyline_separation_m=sep, endpoint_local_delta=endpoint.tolist(),
        max_lateral_delta_m=float(np.max(np.abs(b[:,1]))-np.max(np.abs(a[:,1]))),
        wrapped_net_yaw_delta_rad=net, reliable_initial_tangent_delta_rad=None if ta is None or tb is None else float(wrap_angle(tb-ta)),
        meaningful=bool(meaningful))


def classification(first, successive, technical=False):
    if first:
        return ('CONTINUOUS_OBSTACLE_REVEAL_EPISODE_QUALIFIED' if successive else
                'FIRST_REACTION_ONLY_INSUFFICIENT_SUCCESSIVE_CHUNKS')
    return ('TECHNICAL_EXECUTION_BLOCKED' if technical else
            'CONTINUOUS_CHUNKS_NO_QUALIFIED_FIRST_REACTION')
