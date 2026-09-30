"""Untimed B-to-original-FRESH diagnostics; no reconciliation or controller runtime."""
from __future__ import annotations

import ast
from contextlib import contextmanager
import hashlib
import math
from pathlib import Path
import sys

import numpy as np

from .join_source03 import literal_settings, sha
from .online_mpc_adapter import PINNED_MPC_SHA256, selection_audit

RULES = ['C0_ROW0', 'C1_XY_PROJECTION', 'C2_SE2_PROJECTION', 'C3_FORWARD_SE2']
METHODS = ['M0_NATIVE', 'HALF_TRANSPORT_LOCAL_SE2', 'FULL_LOCAL_SE2']
SOURCE_IDS = ['OSA03_R00', 'episode_001_repeat_01/handoff_013',
              'episode_008_repeat_01/handoff_023', 'episode_013_repeat_00/handoff_020']
PNGS = ['correspondence_world_overview.png', 'selector_progress_reset.png',
        'correspondence_rule_summary.png']
POSITION_SCALE = .10
YAW_SCALE = math.radians(15)
TIE_ATOL = 1e-12
SIGN_ATOL = 1e-12
DENSE_SCORE_ATOL = 1e-7


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


class SpatialCurve:
    """Original XY arc parameter; shortest yaw. Duplicate XY rows fail closed.

    Zero XY arc with different yaw has no single-valued arc parameterization.
    Reject every zero-length segment rather than silently remove raw identities.
    At an interior vertex, the containing segment is the earlier segment.
    """
    def __init__(self, fresh):
        self.fresh = np.array(fresh, dtype=float, copy=True)
        if (self.fresh.ndim != 2 or self.fresh.shape[1] != 3 or len(self.fresh) < 2
                or not np.isfinite(self.fresh).all()):
            raise ValueError('finite original FRESH Nx3, N>=2 required')
        self.delta = np.diff(self.fresh[:, :2], axis=0)
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if np.any(self.lengths <= 0):
            raise ValueError('zero XY segment: original arc is not a unique pose parameter')
        self.arc = np.r_[0., np.cumsum(self.lengths)]
        self.yaw_delta = wrap(np.diff(self.fresh[:, 2]))
        self.fresh.setflags(write=False)

    def locate(self, arc):
        if not 0 <= arc <= self.arc[-1]:
            raise ValueError('arc outside original path')
        j = max(0, min(len(self.lengths)-1, int(np.searchsorted(self.arc, arc, side='left'))-1))
        return j, float((arc-self.arc[j])/self.lengths[j])

    def pose(self, arc):
        j, t = self.locate(arc)
        return np.r_[self.fresh[j, :2]+t*self.delta[j],
                     wrap(self.fresh[j, 2]+t*self.yaw_delta[j])]

    def score(self, B, arc):
        pose = self.pose(arc)
        return float(np.sum((pose[:2]-B[:2])**2)/POSITION_SCALE**2
                     + (wrap(B[2]-pose[2])/YAW_SCALE)**2)

    def minimize(self, B, *, pose_score, lower=0.):
        """Solve each yaw branch's quadratic exactly, with all endpoints included."""
        B = np.asarray(B, dtype=float)
        if B.shape != (3,) or not np.isfinite(B).all():
            raise ValueError('finite B pose required')
        candidates = []
        for j, length in enumerate(self.lengths):
            if self.arc[j+1] < lower:
                continue
            lo = max(0., float((lower-self.arc[j])/length))
            d = self.delta[j]
            offset = self.fresh[j, :2]-B[:2]
            if not pose_score:
                ts = [lo, 1., float(np.clip(-np.dot(offset, d)/np.dot(d, d), lo, 1.))]
            else:
                dyaw = float(self.yaw_delta[j])
                error0 = float(wrap(B[2]-self.fresh[j, 2]))
                cuts = [lo, 1.]
                if dyaw != 0:
                    # error0 and dyaw are in [-pi,pi], so these cover every branch.
                    for k in range(-2, 3):
                        t = (error0-(2*k+1)*math.pi)/dyaw
                        if lo < t < 1.:
                            cuts.append(t)
                cuts = sorted(set(cuts))
                ts = cuts.copy()
                for a, b in zip(cuts[:-1], cuts[1:]):
                    mid = (a+b)/2
                    branch0 = float(wrap(error0-dyaw*mid))+dyaw*mid
                    denom = np.dot(d, d)/POSITION_SCALE**2 + dyaw**2/YAW_SCALE**2
                    t = (dyaw*branch0/YAW_SCALE**2 - np.dot(offset, d)/POSITION_SCALE**2)/denom
                    ts.append(float(np.clip(t, a, b)))
            for t in ts:
                arc = float(np.clip(self.arc[j]+length*t, lower, self.arc[-1]))
                value = (self.score(B, arc) if pose_score else
                         float(np.sum((self.pose(arc)[:2]-B[:2])**2)))
                candidates.append((value, arc))
        if not candidates:
            raise ValueError('no feasible arc')
        best = min(v for v, _ in candidates)
        return min(a for v, a in candidates if v <= best+TIE_ATOL)

    def describe(self, B, arc):
        B = np.asarray(B)
        pose = self.pose(arc)
        j, t = self.locate(arc)
        tangent = math.atan2(self.delta[j, 1], self.delta[j, 0])
        nearest = int(np.argmin(np.abs(self.arc-arc)))
        return dict(arc_m=float(arc), normalized_progress=float(arc/self.arc[-1]),
                    segment=j, alpha=t, nearest_raw_row_diagnostic=nearest,
                    target_world=pose.tolist(), position_gap_m=float(np.linalg.norm(pose[:2]-B[:2])),
                    yaw_gap_rad=float(abs(wrap(B[2]-pose[2]))),
                    tangent_gap_rad=float(abs(wrap(B[2]-tangent))),
                    remaining_arc_m=float(self.arc[-1]-arc), total_arc_m=float(self.arc[-1]),
                    pose_score=self.score(B, arc))

    def correspondences(self, B):
        xy = self.minimize(B, pose_score=False)
        arcs = [0., xy, self.minimize(B, pose_score=True),
                self.minimize(B, pose_score=True, lower=xy)]
        return {rule: self.describe(B, arc) for rule, arc in zip(RULES, arcs)}

    def safety(self, B, target, environment):
        j, _ = self.locate(target['arc_m'])
        pose = self.pose(target['arc_m'])
        suffix = np.vstack([pose, self.fresh[j+1:]])
        connector = environment.check_polyline(np.vstack([B, pose]), radius=.20, required_clearance=.05)
        suffix_check = environment.check_polyline(suffix, radius=.20, required_clearance=.05)
        return dict(label='hypothetical straight connector diagnostic only',
                    connector=connector, suffix=suffix_check,
                    connector_geometric_threshold_met=connector['minimum_clearance_m'] >= .05,
                    selection_rejected=False, suffix_is_executed=False)


def official_selector(path):
    """Compile ONLY two authenticated, unchanged function ASTs; no tracker/CasADi."""
    path = Path(path)
    if sha(path) != PINNED_MPC_SHA256:
        raise ValueError('official MPC source hash mismatch')
    source = path.read_text()
    names = ['wrap_angle', 'build_pose_aligned_reference']
    nodes = [n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name in names]
    if [n.name for n in nodes] != names:
        raise ValueError('selector extraction failed')
    settings = literal_settings(source)
    assert settings['HORIZON'] == 5 and tuple(settings['Q_WEIGHTS']) == (10., 10., 1.)
    namespace = dict(np=np, math=math)
    code = compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec')
    exec(code, namespace)
    meta = dict(source=str(path), source_sha256=sha(path), horizon=settings['HORIZON'],
                weights=list(settings['Q_WEIGHTS']), functions={n.name: dict(
                    first_line=n.lineno, last_line=n.end_lineno,
                    source_sha256=hashlib.sha256(ast.get_source_segment(source, n).encode()).hexdigest(),
                    ast_sha256=hashlib.sha256(ast.dump(n, include_attributes=False).encode()).hexdigest())
                    for n in nodes})
    return namespace['build_pose_aligned_reference'], meta


@contextmanager
def diagnostic_guard():
    """Fail before known solver/runtime bodies; count pure official selector calls.

    The extracted AST contains no controller class, memory or integration. This
    additional Python-call guard catches accidental imports/calls of repo solvers
    or the official runtime. Historical saved validators run outside this guard.
    """
    counts = dict(selector_queries=0, optimizer=0, MPC=0, rollouts=0,
                  integration=0, memory_updates=0, command_applications=0, LightNav=0, RGB=0, Isaac=0)
    previous = sys.getprofile()

    def profile(frame, event, arg):
        if event != 'call':
            return
        name = frame.f_code.co_name
        file = frame.f_code.co_filename
        if name == 'build_pose_aligned_reference' and file.endswith('/vln_mujoco/mpc.py'):
            counts['selector_queries'] += 1
        forbidden = (name in {'solve_least_squares', 'solve_graph', 'integrate_unicycle',
                              '_rollout_unicycle', 'load_official', 'solve_half', 'solve_condition',
                              'run_method', 'apply_command'}
                     or '/scipy/optimize/' in file or '/casadi/' in file
                     or file.endswith('/vln_mujoco/mpc.py') and name not in
                     {'build_pose_aligned_reference', 'wrap_angle', '<listcomp>', '<module>'}
                     or file.endswith('/online_mpc_adapter.py') and name in
                     {'__init__', 'submit', 'poll', 'install', 'restore', 'command'})
        if forbidden:
            raise RuntimeError('diagnostic forbids solver/controller/integration: '+file+':'+name)
    sys.setprofile(profile)
    try:
        yield counts
    finally:
        sys.setprofile(previous)


def selector_record(curve, reference, pose, selector, *, method, tick, time_s):
    """Map selected physical rows to unchanged original node identities."""
    pose = np.array(pose, dtype=float, copy=True)
    reference = np.array(reference, dtype=float, copy=True)
    before = pose.copy(), reference.copy()
    output = selector(reference, pose, horizon=5, weights=(10., 10., 1.))
    np.testing.assert_array_equal(pose, before[0])
    np.testing.assert_array_equal(reference, before[1])
    audit = selection_audit(reference, pose, output, horizon=5, weights=(10., 10., 1.))
    ids = audit['indices']
    return dict(method=method, tick=tick, time_after_B_s=time_s, matched_pose_world=pose.tolist(),
                nearest_row=audit['nearest_index'], H5_rows=ids, original_node_identities=ids,
                physical_H5_world=output.tolist(), physical_H5_wrapped_world=reference[ids].tolist(),
                nearest_physical_world=reference[audit['nearest_index']].tolist(),
                first_row=ids[0], mean_row=float(np.mean(ids)), min_row=min(ids), max_row=max(ids),
                original_H5_arcs_m=curve.arc[ids].tolist(),
                nearest_arc_m=float(curve.arc[audit['nearest_index']]),
                first_arc_m=float(curve.arc[ids[0]]), mean_arc_m=float(np.mean(curve.arc[ids])),
                min_arc_m=float(curve.arc[min(ids)]), max_arc_m=float(curve.arc[max(ids)]),
                endpoint_repeated=audit['endpoint_repeated'])


def matched_queries(curve, references, states, selector):
    rows = []
    for state in states:
        for method in METHODS:
            rows.append(selector_record(curve, references[method], state['pose_world'], selector,
                                        method=method, tick=state['tick'], time_s=state['time_after_B_s']))
    return rows


def reset_summary(rows):
    by = {n: [r for r in rows if r['method'] == n] for n in METHODS}
    native = by[METHODS[0]]
    result = dict(matched_ticks=len(native), methods={})
    for method in METHODS[1:]:
        differences = []
        for n, m in zip(native, by[method]):
            assert n['tick'] == m['tick'] and n['matched_pose_world'] == m['matched_pose_world']
            d = dict(tick=n['tick'])
            for metric in ['nearest_row', 'first_row', 'mean_row', 'nearest_arc_m', 'first_arc_m', 'mean_arc_m']:
                d['delta_'+metric] = m[metric]-n[metric]
            differences.append(d)
        negative = [d['delta_first_arc_m'] < -SIGN_ATOL for d in differences]
        persistent = any(a and b for a, b in zip(negative, negative[1:]))
        def first_diff(a, b):
            return next((x['tick'] for x, y in zip(a, b) if x['H5_rows'] != y['H5_rows']), None)
        result['methods'][method] = dict(differences=differences,
            negative_first_count=sum(negative), negative_first_fraction=sum(negative)/len(negative),
            source_reset=any(negative), persistent_two_ticks=persistent,
            max_backward_first_rows=max(0., -min(d['delta_first_row'] for d in differences)),
            max_backward_first_arc_m=max(0., -min(d['delta_first_arc_m'] for d in differences)),
            mean_delta_first_arc_m=float(np.mean([d['delta_first_arc_m'] for d in differences])),
            negative_counts={k:sum(d['delta_'+k] < -SIGN_ATOL for d in differences)
                             for k in ['nearest_row', 'first_row', 'mean_row', 'nearest_arc_m', 'first_arc_m', 'mean_arc_m']},
            negative_fractions={k:sum(d['delta_'+k] < -SIGN_ATOL for d in differences)/len(differences)
                                for k in ['nearest_row', 'first_row', 'mean_row', 'nearest_arc_m', 'first_arc_m', 'mean_arc_m']},
            max_backward={k:max(0., -min(d['delta_'+k] for d in differences))
                          for k in ['nearest_row', 'first_row', 'mean_row', 'nearest_arc_m', 'first_arc_m', 'mean_arc_m']},
            first_different_tick=first_diff(native, by[method]))
    result['first_Half_Full_different_tick'] = next((h['tick'] for h, f in zip(by[METHODS[1]], by[METHODS[2]])
                                                   if h['H5_rows'] != f['H5_rows']), None)
    return result


def classification(sources):
    """Frozen operational definition; full facts are kept even when category fits."""
    full = METHODS[2]
    reset_sources = sum(q['progress_reset']['methods'][full]['source_reset'] for q in sources.values())
    coexist = []
    for sid, q in sources.items():
        native, transported = (q['saved_execution_context'][n] for n in [METHODS[0], full])
        worse = (transported['position_auc_09_m_s'] > native['position_auc_09_m_s']+1e-9
                 or native['sustained_attachment_s'] is not None and
                 (transported['sustained_attachment_s'] is None or
                  transported['sustained_attachment_s'] > native['sustained_attachment_s']+1e-9))
        if worse and q['progress_reset']['methods'][full]['persistent_two_ticks']:
            coexist.append(sid)
    all_future = all(q['correspondences'][r]['arc_m'] > SIGN_ATOL for q in sources.values() for r in RULES[1:])
    any_systematic = any(sum(q['progress_reset']['methods'][n]['source_reset'] for q in sources.values()) >= 3
                         for n in METHODS[1:])
    if reset_sources >= 3 and coexist:
        label = 'TRANSPORT_PROGRESS_RESET_SUPPORTED'
    elif all_future and not any_systematic:
        label = 'CORRESPONDENCE_WITHOUT_RESET'
    else:
        label = 'MIXED_CORRESPONDENCE_EFFECT'
    return dict(classification=label, Full_reset_sources=reset_sources,
                persistent_and_worse_sources=coexist, all_continuous_rules_future=all_future,
                causal_claim=False)
