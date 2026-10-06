"""Planning-only geometry diagnostics and predeclared interpretation predicates."""
from contextlib import contextmanager
import sys
import numpy as np
from .local_se2_reconciliation import planning_diagnostics
from .se2 import wrap_angle

PNGS = ['formulation_geometry.png', 'canonical_correction_profile.png',
        'relative_motion_nonrigidity.png', 'v2_hermite_geometry.png']
BASELINES = {'B_ENTRY': 'B_ENTRY_STAGE', 'HERMITE': 'HERMITE_BRIDGE', 'V2_SINGLE_NODE': 'V2_GRAPH_BRIDGE'}


@contextmanager
def planning_guard(allow_optimizer=False, optimizer_budget=None):
    """Fail before any execution/model/acquisition entrypoint can run.

    Also forbid child processes during science: a subprocess cannot bypass the
    Python profile guard. Git authentication happens before this context.
    """
    previous = sys.getprofile()
    counts = dict(canonical_optimizer_calls=0)
    def profile(frame, event, arg):
        if event != 'call': return
        name, module = frame.f_code.co_name, frame.f_globals.get('__name__', '')
        if name == 'solve_least_squares':
            counts['canonical_optimizer_calls'] += 1
            if optimizer_budget is not None and counts['canonical_optimizer_calls'] > optimizer_budget:
                raise RuntimeError('canonical optimizer budget exceeded')
        forbidden = (name in {'run_method', 'run_rollout', 'execute_native', 'integrate_unicycle',
            'solve_mpc', '_solve', 'infer', 'inference', 'predict', 'capture_rgb',
            'collect_source', 'collect', 'scan', 'preflight', 'Popen', '_execute_child'}
            or (name == 'get_rgb' and not module.startswith('matplotlib.'))
            or module.startswith(('isaacsim', 'omni.', 'lightnav', 'torch', 'casadi'))
            or ('mpc' in module.lower() and name in {'solve', 'step', 'submit', 'start'})
            or (not allow_optimizer and name in {'solve_least_squares', 'solve_graph', 'solve_variant'}))
        if forbidden: raise RuntimeError('planning-only guard: '+module+'.'+name)
    sys.setprofile(profile)
    try: yield counts
    finally: sys.setprofile(previous)


def pose_difference(left, right):
    a, b = np.asarray(left), np.asarray(right)
    if a.shape != b.shape: raise ValueError('identity-matched arrays required')
    d = np.linalg.norm(a[:, :2]-b[:, :2], axis=1)
    yaw = abs(wrap_angle(a[:, 2]-b[:, 2]))
    return dict(XY_max_m=float(d.max()), XY_RMS_m=float(np.sqrt(np.mean(d*d))),
                yaw_max_rad=float(yaw.max()), yaw_RMS_rad=float(np.sqrt(np.mean(yaw*yaw))))


def directed_projection(samples, reference):
    """Each node -> closest continuous XY segment; exact distance tie earliest arc.

    Yaw is shortest interpolation at that XY projection. Diagnostic only, not a
    graph correspondence factor or continuous Hausdorff/Frechet distance.
    Zero-length segments are allowed and choose their first endpoint.
    """
    a, b = np.asarray(samples), np.asarray(reference)
    if len(b) < 2: raise ValueError('N>=2 reference required')
    matched = []
    for pose in a:
        choices = []
        for j, (p, q) in enumerate(zip(b[:-1], b[1:])):
            edge = q[:2]-p[:2]; d2 = edge@edge
            t = float(np.clip((pose[:2]-p[:2])@edge/d2, 0, 1)) if d2 else 0.
            xy = p[:2]+t*edge
            choices.append((float(np.sum((pose[:2]-xy)**2)), j, t,
                            np.r_[xy, wrap_angle(p[2]+t*wrap_angle(q[2]-p[2]))]))
        matched.append(min(choices, key=lambda c: c[:3])[3])
    return pose_difference(a, np.asarray(matched))


def curve_difference(a, b):
    forward, reverse = directed_projection(a, b), directed_projection(b, a)
    return dict(canonical_nodes_to_baseline=forward, baseline_nodes_to_canonical=reverse,
        symmetric_vertex_XY_max_m=max(forward['XY_max_m'], reverse['XY_max_m']),
        symmetric_vertex_yaw_max_rad=max(forward['yaw_max_rad'], reverse['yaw_max_rad']))


def v2_hermite_similarity(h, v, installed_h, installed_v):
    h, v = np.asarray(h), np.asarray(v)
    if h.shape != v.shape or len(h) < 3: raise ValueError('same saved identities required')
    eh, ev = np.diff(h[:3, :2], axis=0), np.diff(v[:3, :2], axis=0)
    lh, lv = np.linalg.norm(eh, axis=1), np.linalg.norm(ev, axis=1)
    directions = wrap_angle(np.arctan2(ev[:, 1], ev[:, 0])-np.arctan2(eh[:, 1], eh[:, 0]))
    return dict(Hermite_X1=h[1].tolist(), V2_X1=v[1].tolist(), B=h[0].tolist(), entry=h[2].tolist(),
        X1_XY_difference_m=float(np.linalg.norm(v[1, :2]-h[1, :2])),
        X1_yaw_difference_rad=float(wrap_angle(v[1, 2]-h[1, 2])),
        bridge_arc_difference_m=float(lv.sum()-lh.sum()),
        first_edge_direction_difference_rad=float(directions[0]),
        second_edge_direction_difference_rad=float(directions[1]),
        segment_length_difference_m=(lv-lh).tolist(),
        full_planned=pose_difference(v, h), full_installed=pose_difference(installed_v, installed_h))


def diagnostics(problem, x, baselines, safety):
    p = problem; d = planning_diagnostics(p, x)
    raw = pose_difference(x, p.fresh); target = pose_difference(x, p.target)
    for j, node in enumerate(d['nodes']):
        node['target_world_XY_displacement_m'] = float(np.linalg.norm(x[j, :2]-p.target[j, :2]))
    first = float(np.linalg.norm(x[0, :2]-p.fresh[0, :2])); end = float(np.linalg.norm(x[-1, :2]-p.fresh[-1, :2]))
    initial_target = float(np.linalg.norm(p.target[0, :2]-p.fresh[0, :2]))
    return dict(**d, raw_difference=raw, transported_difference=target,
        first_node_correction_m=first, endpoint_correction_m=end,
        first_target_distance_reduction_m=initial_target-d['nodes'][0]['target_world_XY_displacement_m'],
        endpoint_over_first=None if first == 0 else end/first,
        comparisons={k: curve_difference(x, b) for k,b in baselines.items()},
        reference_clearance_m=safety['minimum_clearance_m'], reference_safe=safety['clearance_valid'])


def classify(sources, cfg):
    rules = cfg['interpretation']; flags = {}
    for label, s in sources.items():
        d = s['diagnostics']; xy, yaw = rules['distinct_XY_m'], rules['distinct_yaw_rad']
        different = all(c['symmetric_vertex_XY_max_m'] > xy or c['symmetric_vertex_yaw_max_rad'] > yaw
                        for c in d['comparisons'].values())
        nonrigid = d['rigid_fit']['translation_RMS_m'] > xy or d['rigid_fit']['yaw_RMS_rad'] > yaw
        bounded = (d['relative_translation_RMS_m'] <= rules['relative_translation_RMS_bound_m']
                   and d['relative_yaw_RMS_rad'] <= rules['relative_yaw_RMS_bound_rad']
                   and d['relative_translation_max_m'] <= rules['relative_translation_max_bound_m']
                   and d['relative_yaw_max_rad'] <= rules['relative_yaw_max_bound_rad'])
        valid = s['solver']['converged'] and d['reference_safe']
        recovery = d['endpoint_over_first'] is not None and d['endpoint_over_first'] <= rules['endpoint_first_ratio_max']
        early = d['first_node_correction_m'] > xy and d['first_target_distance_reduction_m'] > xy
        flags[label] = dict(valid=bool(valid), distinct=bool(different), nonrigid=bool(nonrigid),
                            bounded=bool(bounded), recovery=bool(recovery), early=bool(early),
                            plausible=bool(valid and different and nonrigid and bounded and recovery and early))
    if any(s.get('technical_failure') for s in sources.values()): category='TECHNICAL_BLOCKED'
    elif sum(not f['valid'] for f in flags.values()) >= rules['repeated_invalid_count']: category='CANONICAL_FORMULATION_INVALID'
    elif all(not f['distinct'] or not f['nonrigid'] for f in flags.values()): category='CANONICAL_COLLAPSES_TO_RIGID_OR_EXISTING'
    elif sum(f['plausible'] for f in flags.values()) >= 2: category='CANONICAL_DISTINCT_AND_STRUCTURALLY_PLAUSIBLE'
    elif all(f['valid'] and f['distinct'] and f['nonrigid'] for f in flags.values()): category='CANONICAL_DISTINCT_BUT_NO_CLEAR_STRUCTURAL_GAIN'
    else: category='MIXED_FORMULATION_EVIDENCE'
    direction = 'A' if category=='CANONICAL_DISTINCT_AND_STRUCTURALLY_PLAUSIBLE' else 'C' if category in {'TECHNICAL_BLOCKED','CANONICAL_FORMULATION_INVALID'} else 'B'
    return dict(classification=category, direction=direction, predicates=flags, execution_benefit_demonstrated=False)
