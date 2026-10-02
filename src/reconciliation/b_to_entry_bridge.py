"""Untimed spatial bridges with fixed B, C3 entry and original downstream rows."""
import math
import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq
from shapely.geometry import LineString

from .se2 import relative_pose, se2_log, wrap_angle
from .spatial_entry_suffix import NATIVE, ENTRY, execution_metrics, endpoint_dwell
from .spatial_correspondence_selector import SOURCE_IDS

HERMITE = 'HERMITE_BRIDGE'
GRAPH = 'SE2_GRAPH_BRIDGE'
NEW = [HERMITE, GRAPH]
ORDER = [NATIVE, ENTRY, *NEW]
PNGS = ['world_execution_overview.png', 'bridge_geometry.png',
        'transition_and_completion_metrics.png', 'attachment_vs_completion.png']
EPS = 1e-12


def unit(v):
    v = np.asarray(v, float)
    d = np.linalg.norm(v)
    if not np.isfinite(d) or d <= EPS:
        raise ValueError('undefined spatial direction')
    return v / d


def hermite_curve(P, B, E, next_pose):
    """Derivative magnitudes equal the B-to-entry chord, never P-to-B length."""
    P, B, E, next_pose = map(lambda p: np.asarray(p, float), (P, B, E, next_pose))
    chord = float(np.linalg.norm(E[:2]-B[:2]))
    if chord <= EPS:
        raise ValueError('zero B-to-entry chord')
    m0, m1 = chord*unit(B[:2]-P[:2]), chord*unit(next_pose[:2]-E[:2])
    def xy(t):
        return (2*t**3-3*t**2+1)*B[:2]+(t**3-2*t**2+t)*m0+(-2*t**3+3*t**2)*E[:2]+(t**3-t**2)*m1
    def derivative(t):
        return (6*t*t-6*t)*B[:2]+(3*t*t-4*t+1)*m0+(-6*t*t+6*t)*E[:2]+(3*t*t-2*t)*m1
    def arc(t):
        value, error = quad(lambda u: float(np.linalg.norm(derivative(u))), 0., t,
                            epsabs=1e-12, epsrel=1e-12, limit=200)
        if error > 1e-10:
            raise ValueError('Hermite arc quadrature did not meet frozen tolerance')
        return value
    return xy, derivative, arc, m0, m1


def hermite_bridge(P, B, suffix):
    """Suffix is [exact E, exact Native-installed downstream poses]."""
    B, suffix = np.asarray(B, float), np.asarray(suffix, float)
    if suffix.ndim != 2 or suffix.shape[1] != 3 or len(suffix) < 2:
        raise ValueError('entry and downstream row required')
    segments = np.linalg.norm(np.diff(suffix[:, :2], axis=0), axis=1)
    if np.any(segments <= EPS):
        raise ValueError('degenerate retained suffix edge')
    d_F = float(np.median(segments))
    E = suffix[0]
    xy, derivative, arc, m0, m1 = hermite_curve(P, B, E, suffix[1])
    length = arc(1.)
    M = max(2, math.ceil(length/d_F))
    fractions = np.linspace(0., 1., M+1)
    parameters = np.array([0., *[brentq(lambda u: arc(u)-s*length, 0., 1., xtol=1e-14,
                       rtol=4*np.finfo(float).eps) for s in fractions[1:-1]], 1.])
    bridge = np.array([[*xy(t), wrap_angle(B[2]+s*wrap_angle(E[2]-B[2]))]
                       for t,s in zip(parameters, fractions)])
    # Copy fixed boundary bits, including yaw representation, after interpolation.
    bridge[0], bridge[-1] = B, E
    return bridge, dict(M=M, d_F_m=d_F, Hermite_continuous_arc_m=length,
        Hermite_parameters=parameters.tolist(), arc_fractions=fractions.tolist(),
        m0=m0.tolist(), m1=m1.tolist(), P=np.asarray(P).tolist(), B=B.tolist(), E=E.tolist(),
        next_original_pose=suffix[1].tolist(), suffix_segment_lengths_m=segments.tolist(),
        density_definition='median segments of [E*, exact original downstream suffix]; spatial metres only')


class BridgeProblem:
    def __init__(self, P, B, suffix, d_F, M):
        self.P, self.B, self.suffix = (np.array(x, float, copy=True) for x in (P, B, suffix))
        self.E = self.suffix[0].copy()
        self.d_F, self.M = float(d_F), int(M)
        if M < 2 or self.d_F <= EPS:
            raise ValueError('M >= 2 and positive spatial row scale required')
        self.phi_in = math.atan2(*unit(self.B[:2]-self.P[:2])[::-1])
        self.phi_out = math.atan2(*unit(self.suffix[1,:2]-self.E[:2])[::-1])

    def bridge(self, interior):
        x = np.asarray(interior, float)
        if x.shape != (self.M-1, 3) or not np.isfinite(x).all():
            raise ValueError('finite interior SE(2) poses required')
        return np.vstack([self.B, x, self.E])

    def reference(self, interior):
        return np.vstack([self.bridge(interior), self.suffix[1:]])

    def factors(self, interior):
        x = self.bridge(interior)
        edges = np.diff(x[:, :2], axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        # atan2(0,0)=0 is finite for proposal costing. The feasibility gate
        # rejects collapsed edges; no collapse penalty is added to the objective.
        phi = np.arctan2(edges[:,1], edges[:,0])
        D = relative_pose(x[:-1], x[1:])
        smooth = se2_log(relative_pose(D[:-1], D[1:])) / [self.d_F, self.d_F, np.deg2rad(10)]
        return dict(E_in=np.atleast_1d(wrap_angle(phi[0]-self.phi_in)/np.deg2rad(15)),
            E_out=np.atleast_1d(wrap_angle(phi[-1]-self.phi_out)/np.deg2rad(15)),
            E_smooth=smooth.ravel(), E_space=np.diff(lengths)/self.d_F)

    def residual(self, interior):
        return np.concatenate(list(self.factors(interior).values()))

    def costs(self, interior):
        c = {k: float(v@v) for k,v in self.factors(interior).items()}
        return dict(**c, total=sum(c.values()))

    def nondegenerate(self, interior):
        return bool(np.all(np.linalg.norm(np.diff(self.bridge(interior)[:, :2], axis=0), axis=1) > EPS))


def reference_arrays(problem, bridge, A, raw, original_ids):
    """Observation-local representation; raw original downstream bytes preserved."""
    bridge = np.asarray(bridge)
    np.testing.assert_array_equal(bridge[0], problem.B)
    np.testing.assert_array_equal(bridge[-1], problem.E)
    world = problem.reference(bridge[1:-1])
    local = np.vstack([relative_pose(A, bridge), np.asarray(raw)[original_ids]])
    assert len(world) == len(local)
    labels = ['B', *[f'bridge_{j}' for j in range(1, problem.M)], 'E*',
              *[f'F_{j}' for j in original_ids]]
    return world, local, labels


def bridge_geometry(problem, interior, safety):
    x = problem.bridge(interior)
    delta = np.diff(x[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    turn = wrap_angle(np.diff(np.arctan2(delta[:,1], delta[:,0])))
    chord = float(np.linalg.norm(x[-1,:2]-x[0,:2]))
    return dict(M=problem.M, d_F_m=problem.d_F, bridge_XY_length_m=float(sum(lengths)),
        B_to_entry_chord_m=chord, length_chord_ratio=float(sum(lengths)/chord),
        maximum_turning_angle_rad=float(max(abs(turn))), RMS_turning_angle_rad=float(np.sqrt(np.mean(turn**2))),
        segment_length_min_m=float(min(lengths)), segment_length_max_m=float(max(lengths)),
        turning_angles_rad=turn.tolist(), segment_lengths_m=lengths.tolist(),
        factor_costs=problem.costs(interior), reference_clearance_m=safety['minimum_clearance_m'],
        reference_safe=safety['clearance_valid'],
        bridge_self_intersection=not LineString(x[:,:2]).is_simple,
        full_reference_self_intersection=not LineString(problem.reference(interior)[:,:2]).is_simple)


def classify(sources, tolerance=1e-9, clear_delay=.10):
    """Frozen precedence and separate pairwise outcomes; no aggregate score."""
    hard = sources[SOURCE_IDS[2]]
    comparisons, regressions = {}, []
    for sid,q in sources.items():
        comparisons[sid] = {}
        base = q['primary_metrics'][ENTRY]
        for name in NEW:
            r = q['primary_metrics'][name]
            if r is None:
                comparisons[sid][name] = None
                continue
            ta,te = r['sustained_attachment_s'],r['original_FRESH_endpoint_dwell_s']
            ba,be = base['sustained_attachment_s'],base['original_FRESH_endpoint_dwell_s']
            delta = None if te is None or be is None else te-be
            lost = be is not None and te is None
            row = dict(AUC_09_delta_vs_C3=r['position_auc_09_m_s']-base['position_auc_09_m_s'] if r['position_auc_09_m_s'] is not None else None,
                attachment_recovered=ta is not None and ba is None, endpoint_recovered=te is not None and be is None,
                attachment_delta_s=None if ta is None or ba is None else ta-ba, endpoint_delta_s=delta,
                endpoint_lost=lost, clearly_later_endpoint=delta is not None and delta>clear_delay+tolerance,
                any_later_endpoint=delta is not None and delta>tolerance)
            comparisons[sid][name] = row
            if sid != SOURCE_IDS[2] and (lost or row['clearly_later_endpoint']):
                regressions.append(dict(source_id=sid,method=name,**row))
    recovered = {n: bool(comparisons[SOURCE_IDS[2]][n] and
        (comparisons[SOURCE_IDS[2]][n]['attachment_recovered'] or comparisons[SOURCE_IDS[2]][n]['endpoint_recovered'])) for n in NEW}
    partial = {n: bool(comparisons[SOURCE_IDS[2]][n] and comparisons[SOURCE_IDS[2]][n]['AUC_09_delta_vs_C3'] is not None and
        comparisons[SOURCE_IDS[2]][n]['AUC_09_delta_vs_C3'] < -tolerance) for n in NEW}
    if any(not q['technical_valid'] for q in sources.values()):
        label = 'TECHNICAL_BLOCKED'
    elif any(q['reference_failure'] for q in sources.values()):
        label = 'BRIDGE_REFERENCE_FAILURE'
    elif (any(recovered.values()) or any(partial.values())) and regressions:
        label = 'BRIDGE_TRADEOFF'
    elif recovered[GRAPH] and (not recovered[HERMITE] or
            (hard['primary_metrics'][GRAPH]['original_FRESH_endpoint_dwell_s'] is not None and
             hard['primary_metrics'][HERMITE]['original_FRESH_endpoint_dwell_s'] is None)):
        label = 'GRAPH_BRIDGE_SUPPORTED'
    elif recovered[HERMITE] and not any(r['method']==HERMITE for r in regressions) and not any(
            comparisons[s][HERMITE]['any_later_endpoint'] for s in SOURCE_IDS if s!=SOURCE_IDS[2]):
        label = 'SIMPLE_BRIDGE_SUFFICIENT'
    elif any(partial.values()) and not any(recovered.values()) and not regressions:
        label = 'BRIDGE_PARTIAL_ONLY'
    else:
        label = 'BRIDGE_INSUFFICIENT'
    return dict(classification=label,pairwise=comparisons,easy_source_regressions=regressions,
                S3_dwell_recovered=recovered,S3_AUC_lower=partial)
