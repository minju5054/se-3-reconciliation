"""Model-free blind-corner bank. Probes are geometry, never VLA/execution evidence.

No model/controller/planner calls. All coordinates are world metres after an
explicit canonical-corner rigid transform; raw model data are never edited.
"""
from pathlib import Path
import hashlib
import numpy as np
from shapely.geometry import LineString, Point

from .blind_corner_source import INSTRUCTION, left_corner, old_turning


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def historical_scale(corpus):
    records = []
    for path in sorted(Path(corpus).glob('episodes/*/chunks/*/raw_local.npy')):
        raw = np.load(path, allow_pickle=False)
        turn = old_turning(raw) if len(raw) >= 2 else None
        records.append(dict(path=str(path.resolve()), sha256=digest(path), N=len(raw), turn=turn))
    arcs = [r['turn']['arc_m'] for r in records if r['turn'] and r['turn']['qualified']]
    if not arcs:
        raise ValueError('No authentic historical turning chunks')
    return dict(records=records, count=len(records), turning_count=len(arcs),
                quantile=.75, finite_arc_m=float(np.quantile(arcs, .75)),
                rule='unchanged arc>=.50/yaw>=30/tangent>=30/reliable segment>=.02; np.quantile linear')


def canonical_to_world(xy, candidate):
    xy = np.asarray(xy, dtype=float)
    basis = np.column_stack([candidate['outgoing_xy'], -np.asarray(candidate['incoming_xy'])])
    if not left_corner(candidate['incoming_xy'], candidate['outgoing_xy']):
        raise ValueError('Only orthogonal left corridor corners')
    return xy @ basis.T + candidate['corner_xy']


def nominal_poses(arcs, radius, candidate, bank):
    """Straight/quarter-circle/straight diagnostic centerline; spatial arc only."""
    arcs = np.asarray(arcs, dtype=float)
    d, a = bank['approach_offset_m'], bank['approach_lead_m']
    lead = a + d - radius
    if lead < 0 or np.any(arcs < 0):
        raise ValueError('Invalid nominal spatial construction')
    theta = np.clip((arcs-lead)/radius, 0, np.pi/2)
    x = -d + radius*(1-np.cos(theta)) + np.maximum(arcs-lead-radius*np.pi/2, 0)
    y = radius-d-radius*np.sin(theta) + np.maximum(lead-arcs, 0)
    xy = canonical_to_world(np.column_stack([x, y]), candidate)
    yaw = np.arctan2(candidate['incoming_xy'][1], candidate['incoming_xy'][0]) + theta
    return np.column_stack([xy, yaw])


def construct_bank(config, finite_arc_m):
    if config['instruction'] != INSTRUCTION or config['retry']:
        raise ValueError('Frozen instruction/no-retry contract')
    ids = [c['id'] for c in config['candidates']]
    if ids != config['candidate_ids'] or ids != ['C03', 'C04', 'C05'] or len(ids) > 3:
        raise ValueError('Only the predeclared new bank; no C02 retry or extra candidate')
    b = config['geometry_bank']; rows = []
    arcs = np.unique(np.r_[np.arange(0, finite_arc_m, b['nominal_sample_step_m']), finite_arc_m])
    probe_arcs = np.arange(0, b['visibility_probe_max_arc_m']+1e-9, b['visibility_probe_step_m'])
    for candidate in config['candidates']:
        c = dict(candidate)
        yaw_out = np.arctan2(c['outgoing_xy'][1], c['outgoing_xy'][0])
        yaw_cart = yaw_out + np.arctan2(-(b['approach_lead_m']-b['camera_forward_offset_m']), b['approach_offset_m'])
        c.update(approach_pose=nominal_poses([0], b['visibility_probe_radius_m'], c, b)[0].tolist(),
                 cart_center_xy=canonical_to_world(b['cart_canonical_xy_m'], c).tolist(), cart_yaw=float(yaw_cart),
                 fill_translation=[*c['corner_xy'], 2.6],
                 probe_arcs_m=probe_arcs.tolist(),
                 probe_poses=nominal_poses(probe_arcs, b['visibility_probe_radius_m'], c, b).tolist(),
                 bypass_probe=canonical_to_world(b['bypass_canonical_xy_m'], c).tolist(),
                 outgoing_probe=canonical_to_world([[1.35, -1.55], [2.4, -1.55]], c).tolist(),
                 nominal_arcs_m=arcs.tolist(),
                 nominal_paths=[dict(radius_m=r, poses=nominal_poses(arcs, r, c, b).tolist()) for r in b['nominal_radii_m']])
        rows.append(c)
    return rows


def attributed_clearance(points, base, cart):
    """Exact swept finite polyline; no endpoint extension, connector or AABB."""
    xy = np.asarray(points, float)
    if xy.ndim != 2 or not len(xy) or xy.shape[1] < 2:
        raise ValueError('Empty/invalid path is unavailable, not safe')
    xy = xy[:, :2]
    path = LineString(xy) if len(xy) > 1 else Point(xy[0])
    hospital = base.check_polyline(xy)
    cart_edge = float(path.distance(cart)-.20)
    h = hospital['minimum_clearance_m']
    return dict(Hospital_only_m=h, cart_only_m=cart_edge, combined_m=min(h, cart_edge),
                workspace_known=hospital['workspace_known'],
                limiting_geometry='cart' if cart_edge < h else 'Hospital',
                Hospital_check=hospital, whole_finite_polyline=True, connector_included=False)


def geometric_probes(candidate, base, cart, config, finite_arc_m):
    b = config['geometry_bank']; paths = []
    # Influence slab uses the inherited cart longitudinal extents +/- .50 m.
    origin = np.asarray(candidate['cart_center_xy']); forward = np.asarray(candidate['outgoing_xy'])
    coords = np.asarray(cart.envelope.exterior.coords)  # extent only; never clearance oracle
    extents = (coords-origin) @ forward
    lo, hi = float(extents.min()-.5), float(extents.max()+.5)
    for p in candidate['nominal_paths']:
        poses = np.asarray(p['poses']); clear = attributed_clearance(poses, base, cart)
        seg = [float(LineString(poses[i:i+2, :2]).distance(cart)-.20) for i in range(len(poses)-1)]
        first = next((i for i, v in enumerate(seg) if v < .05), None)
        progress = (poses[:, :2]-origin) @ forward
        inside = np.flatnonzero((progress >= lo) & (progress <= hi))
        entry = float(candidate['nominal_arcs_m'][inside[0]]) if len(inside) else None
        paths.append(dict(radius_m=p['radius_m'], clearance=clear, first_cart_conflict_segment=first,
                          conflict_arc_m=None if first is None else candidate['nominal_arcs_m'][first],
                          influence_entry_arc_m=entry,
                          qualifies=bool(clear['Hospital_check']['clearance_valid'] and first is not None and
                                         entry is not None and entry <= finite_arc_m-b['interaction_entry_reserve_m'])))
    bypass = attributed_clearance(candidate['bypass_probe'], base, cart)
    outgoing = attributed_clearance(candidate['outgoing_probe'], base, cart)
    return dict(nominal=paths, nominal_conflict=any(p['qualifies'] for p in paths),
                bypass=bypass, outgoing=outgoing, influence_progress_m=[lo, hi],
                interaction_bound_m=finite_arc_m,
                bypass_pass=bool(bypass['workspace_known'] and bypass['combined_m'] >= b['bypass_edge_reserve_m']),
                outgoing_pass=bool(outgoing['workspace_known'] and outgoing['combined_m'] >= b['bypass_edge_reserve_m']))


def geometry_order(rows):
    if len(rows) > 3 or len({r['candidate_id'] for r in rows}) != len(rows):
        raise ValueError('Undeclared or duplicate candidates')
    return [r['candidate_id'] for r in sorted((r for r in rows if r['qualified']),
            key=lambda r: (-r['probes']['bypass']['combined_m'], r['candidate_id']))]
