"""Isolated zero-extra-reserve policy; the original direct checker stays intact."""
from copy import deepcopy
import numpy as np

POLICY = dict(footprint_radius_m=.20, required_extra_clearance_m=.00,
              legacy_required_extra_clearance_m=.05, guard_enabled=True,
              physical_overlap_rejection=True)


def validate_policy(policy):
    if policy != POLICY:
        raise ValueError('only the frozen .20 m footprint / zero extra reserve policy is allowed')
    return deepcopy(policy)


class ExploratoryEnvironment:
    """Explicit adapter for historical callers which request .20/.05.

    Delegate both thresholds to the SAME direct checker. Workspace, uncertainty,
    numerical tolerance, curve bound and footprint stay unchanged. No optimistic
    boolean is inferred from distance alone; borderline/unknown remain rejected.
    This adapter is instantiated only by the EXPLORATORY_02 entry points.
    """
    def __init__(self, environment, policy):
        self.environment = environment
        self.policy = validate_policy(policy)

    def __getattr__(self, name):
        return getattr(self.environment, name)

    def _check(self, method, *args, radius=.20, required_clearance=.05, **kwargs):
        validate_policy(self.policy)
        if radius != .20 or required_clearance != .05:
            raise ValueError('unexpected historical radius/margin at isolated adapter')
        checker = getattr(self.environment, method)
        exploratory = checker(*args, radius=.20, required_clearance=0., **kwargs)
        legacy = checker(*args, radius=.20, required_clearance=.05, **kwargs)
        assert not (exploratory['clearance_valid'] and exploratory['physical_overlap'])
        exploratory.update(physical_footprint_clearance_m=exploratory['minimum_clearance_m'],
            exploratory_overlap_free=bool(exploratory['clearance_valid']),
            legacy_5cm_margin_pass=bool(legacy['clearance_valid']), legacy_5cm_check=legacy,
            exploratory_policy=deepcopy(self.policy))
        return exploratory

    def check_polyline(self, *args, **kwargs):
        return self._check('check_polyline', *args, **kwargs)

    def check_trajectory(self, *args, **kwargs):
        return self._check('check_trajectory', *args, **kwargs)


def post_reveal_classification(pairs):
    if any(p.get('meaningful') for p in pairs):
        return 'POST_REVEAL_INTENT_EVOLVING'
    if len(pairs) == 2 and all(p.get('available') for p in pairs):
        return 'POST_REVEAL_INTENT_STABLE'
    return None


def acquisition_classification(chunks, complete_valid, technical=False):
    if complete_valid:
        return 'CONTINUOUS_EXPLORATORY_C0_C3_ACQUIRED'
    if chunks[0]['applied'] and chunks[1]['applied'] and not chunks[3]['applied']:
        return 'PARTIAL_CONTINUOUS_EXPLORATORY_SEQUENCE'
    if chunks[1].get('geometry_on', {}).get('physical_overlap'):
        return 'FIRST_POST_REVEAL_REFERENCE_OVERLAPS'
    return 'TECHNICAL_EXECUTION_BLOCKED' if technical else None


def extra_evolution(a, b):
    from .se2 import wrap_angle
    delta = np.asarray(b['A']) - a['A']
    delta[2] = wrap_angle(delta[2])
    return dict(observation_translation_m=float(np.linalg.norm(delta[:2])),
                observation_yaw_change_rad=float(delta[2]),
                max_absolute_local_yaw_difference_deg=b['max_yaw_deg']-a['max_yaw_deg'])
