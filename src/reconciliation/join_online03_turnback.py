"""Saved visual-history/trajectory measurements. No model, controller or renderer.

Image coordinates: continuous pixels, u right / v down. Masks use integer raster
indices. World geometry is metres, Z-up; hallway lateral is positive left.
Projected mesh pixels are an offline, pixel-centre silhouette WITHOUT scene
occlusion. Original same-render-product instance pixels are visible evidence.
"""
from collections import Counter

import numpy as np

from .se2 import wrap_angle


def mask_statistics(mask, projected=None, minimum_pixels=20):
    mask = np.asarray(mask, bool)
    if mask.ndim != 2 or min(mask.shape) < 1:
        raise ValueError('nonempty 2D image required')
    h, w = mask.shape
    ys, xs = np.nonzero(mask)
    n = len(xs)
    bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())] if n else None
    center = [(bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2] if n else None
    edge = bool(n and (bbox[0] == 0 or bbox[1] == 0 or bbox[2] == w-1 or bbox[3] == h-1))
    region = ('partial-edge' if edge else ('left' if center[0] < w/3 else
              'right' if center[0] >= 2*w/3 else 'center')) if n else 'no-visible-pixels'
    pcount = None
    if projected is not None:
        projected = np.asarray(projected, bool)
        if projected.shape != mask.shape:
            raise ValueError('projection/mask resolution mismatch')
        pcount = int(projected.sum())
        if not n and not pcount:
            region = 'out-of-frame-or-clipped'
    return dict(width=w, height=h, visible_pixels=n, any_visible=n > 0,
                visibility_gate_pass=n >= minimum_pixels, visible_image_fraction=n/(w*h),
                bbox_xyxy_inclusive=bbox, bbox_center_px=center,
                centroid_px=[float(xs.mean()), float(ys.mean())] if n else None,
                touches_image_boundary=edge, region=region, projected_pixels=pcount,
                projected_image_fraction=None if pcount is None else pcount/(w*h),
                visibility_source='saved same-product instance raster',
                occlusion_status='ORIGINAL_RENDERER_VISIBLE_PIXELS_AVAILABLE',
                occlusion_fraction=None,
                occlusion_fraction_reason='offline silhouette and renderer have different raster rules; no exact occlusion ratio claimed')


def project_mesh(triangles, camera):
    """Frustum-clipped mesh union at pixel centres, no external raster/runtime.

    Clip camera-CV xyz against near/far and four image planes, then rasterize
    triangle interiors by barycentric coordinates. No texture, depth or scene
    occlusion is inferred. This diagnostic never creates a model observation.
    """
    K = np.asarray(camera['actual_intrinsics']['K'], float)
    T = np.asarray(camera['T_world_camera'], float)
    w, h = camera['actual_intrinsics']['resolution_width_height']
    near, far = camera['actual_intrinsics']['clipping_range_m']
    tri = np.asarray(triangles, float)
    if tri.ndim != 3 or tri.shape[1:] != (3, 3) or not np.isfinite(tri).all():
        raise ValueError('finite world triangles required')
    cv = ((tri-T[:3, 3]) @ T[:3, :3]) * [1., -1., -1.]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    planes = [(np.array(n), c) for n, c in [
        ([0, 0, 1], -near), ([0, 0, -1], far),
        ([fx, 0, cx], 0), ([-fx, 0, w-cx], 0),
        ([0, fy, cy], 0), ([0, -fy, h-cy], 0)]]
    mask = np.zeros((h, w), bool)
    distances = np.stack([cv @ n+c for n, c in planes], axis=-1)
    keep = ~np.any(np.all(distances < 0, axis=1), axis=1)
    for polygon, ds in zip(cv[keep], distances[keep]):
        if np.any(ds < 0):
            polygon = polygon.tolist()
            for normal, constant in planes:
                clipped = []
                for a, b in zip(polygon, polygon[1:]+polygon[:1]):
                    a, b = np.asarray(a), np.asarray(b)
                    da, db = a@normal+constant, b@normal+constant
                    if da >= 0:
                        clipped.append(a.tolist())
                    if (da >= 0) != (db >= 0):
                        clipped.append((a+(b-a)*da/(da-db)).tolist())
                polygon = clipped
                if not polygon:
                    break
            if len(polygon) < 3:
                continue
        polygon = np.asarray(polygon)
        uv = np.column_stack((fx*polygon[:, 0]/polygon[:, 2]+cx,
                              fy*polygon[:, 1]/polygon[:, 2]+cy))
        for i in range(1, len(uv)-1):
            a, b, c = uv[[0, i, i+1]]
            lo = np.maximum(np.ceil(np.min([a, b, c], axis=0)-.5).astype(int), [0, 0])
            hi = np.minimum(np.floor(np.max([a, b, c], axis=0)-.5).astype(int), [w-1, h-1])
            if np.any(hi < lo):
                continue
            den = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(den) < 1e-12:
                continue
            x, y = np.meshgrid(np.arange(lo[0], hi[0]+1)+.5, np.arange(lo[1], hi[1]+1)+.5)
            aa = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den
            bb = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den
            mask[lo[1]:hi[1]+1, lo[0]:hi[0]+1] |= (aa >= 0) & (bb >= 0) & (aa+bb <= 1)
    return mask


def point_bbox_relation(pixel, cart_mask):
    s = mask_statistics(cart_mask)
    out = dict(inside_cart_bbox=None, distance_to_cart_bbox_px=None,
               relative_to_cart_bbox=None, relative_to_cart_centroid_px=None)
    if pixel is None or s['bbox_xyxy_inclusive'] is None:
        return out
    u, v = pixel
    x0, y0, x1, y1 = s['bbox_xyxy_inclusive']
    out.update(inside_cart_bbox=bool(x0 <= u <= x1 and y0 <= v <= y1),
               distance_to_cart_bbox_px=float(np.hypot(max(x0-u, 0, u-x1), max(y0-v, 0, v-y1))),
               relative_to_cart_bbox='left' if u < x0 else 'right' if u > x1 else 'within-horizontal-span',
               relative_to_cart_centroid_px=(np.asarray(pixel)-s['centroid_px']).tolist())
    return out


def sampled_slots(snapshot, sampler):
    """Recompute official tier/order; padding is a repeated slot, never a frame."""
    frames = snapshot['frames']
    ids = [f['frame_id'] for f in frames]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate delivered frame ID')
    if any(b['capture_sim_time_s'] < a['capture_sim_time_s'] or
           b['capture_monotonic_ns'] <= a['capture_monotonic_ns'] for a, b in zip(frames, frames[1:])):
        raise ValueError('nonchronological history')
    n = len(frames)
    if snapshot['history_contract']['history_storage'] != 'full_episode_slowfast':
        raise ValueError('this audit requires the saved full-episode sampler contract')
    segments = sampler(n-1, n, snapshot['history_contract']['slowfast_tiers'])
    expected = []
    for segment in segments:
        indices = list(segment['frame_ids'])
        if any(i < 0 or i >= n for i in indices):
            raise ValueError('future/unavailable sampled frame')
        expected.append({**segment, 'server_frame_indices_reconstructed': indices,
                         'frame_ids': [ids[i] for i in indices]})
    if expected != snapshot['model_input_segments_reconstructed']:
        raise ValueError('saved/pinned-source sampler order mismatch')
    current_time = frames[-1]['capture_sim_time_s']
    slots, seen = [], Counter()
    for si, segment in enumerate(expected):
        for within, fi in enumerate(segment['server_frame_indices_reconstructed']):
            f = frames[fi]
            slots.append(dict(slot=len(slots), segment=si, segment_position=within,
                server_frame_index=fi, frame_id=f['frame_id'], tier=segment['tier'],
                pool_spatial=segment['pool_spatial'], pool_mode=segment['pool_mode'],
                age_sim_s=current_time-f['capture_sim_time_s'], repeated_processor_slot=seen[fi] > 0))
            seen[fi] += 1
    return slots


def history_aggregate(rows, *, include_current):
    selected = [r for r in rows if include_current or not r['is_current']]
    unique = {r['frame_id']: r for r in selected}
    def stats(values):
        visible = [r for r in values if r['visibility_gate_pass']]
        return dict(count=len(values), cart_visible_count=len(visible),
            total_cart_pixels=sum(r['visible_pixels'] for r in values),
            max_cart_pixels=max((r['visible_pixels'] for r in values), default=None),
            most_recent_visible_age_s=min((r['age_sim_s'] for r in visible), default=None),
            oldest_visible_age_s=max((r['age_sim_s'] for r in visible), default=None),
            visible_fraction=len(visible)/len(values) if values else None)
    return dict(slots=stats(selected), unique=stats(list(unique.values())),
                includes_current=include_current, pixel_sum_is_not_attention_weight=True)


def trajectory_metrics(world, scenario, tangent_chord_m=.10):
    p = np.asarray(world, float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('finite raw N x 3 required')
    f, center = np.asarray(scenario['forward_xy']), np.asarray(scenario['center_xy'])
    left = np.array([-f[1], f[0]])
    lat, progress = (p[:, :2]-center)@left, (p[:, :2]-center)@f
    d = np.diff(p[:, :2], axis=0)
    signed = d@left
    tangent = np.arctan2(signed, d@f)
    lengths = np.linalg.norm(d, axis=1)
    usable = lengths > 1e-12
    inward = usable & (np.abs(lat[1:]) < np.abs(lat[:-1]))
    init = next((p[i, :2]-p[0, :2] for i in range(1, len(p))
                 if np.linalg.norm(p[i, :2]-p[0, :2]) >= tangent_chord_m), None)
    final = next((p[-1, :2]-p[i, :2] for i in range(len(p)-2, -1, -1)
                  if np.linalg.norm(p[-1, :2]-p[i, :2]) >= tangent_chord_m), None)
    angle = lambda v: None if v is None else float(np.arctan2(v@left, v@f))
    return dict(N=len(p), raw_arc_m=float(lengths.sum()), lateral_nodes_m=lat.tolist(),
        longitudinal_nodes_m=progress.tolist(), min_signed_lateral_m=float(lat.min()),
        max_signed_lateral_m=float(lat.max()), major_signed_lateral_m=float(lat[np.argmax(np.abs(lat))]),
        max_abs_lateral_m=float(np.max(np.abs(lat))), endpoint_lateral_m=float(lat[-1]),
        initial_tangent_rad=angle(init), final_tangent_rad=angle(final),
        final_yaw_hallway_rad=float(wrap_angle(p[-1, 2]-np.arctan2(f[1], f[0]))),
        segment_tangents_rad=[float(x) if ok else None for x, ok in zip(tangent, usable)],
        segment_lateral_change_m=signed.tolist(), inward_segments=np.flatnonzero(inward).tolist(),
        inward_interior_segments=[int(i) for i in np.flatnonzero(inward) if i < len(d)-1],
        inward_arc_m=float(lengths[inward].sum()),
        inward_lateral_travel_m=float((np.abs(lat[:-1])-np.abs(lat[1:]))[inward].sum()),
        endpoint_front_m=float(progress[-1]-scenario['cart_extents'][0]),
        endpoint_rear_m=float(progress[-1]-scenario['cart_extents'][1]),
        connector_included=False, waypoint_intrinsic_dt=None)


def transition(a, b):
    ga, gb = a['trajectory'], b['trajectory']
    return dict(episode=a['episode'], source=a['chunk'], target=b['chunk'],
        delta_endpoint_lateral_m=gb['endpoint_lateral_m']-ga['endpoint_lateral_m'],
        delta_major_signed_lateral_m=gb['major_signed_lateral_m']-ga['major_signed_lateral_m'],
        delta_final_yaw_rad=float(wrap_angle(gb['final_yaw_hallway_rad']-ga['final_yaw_hallway_rad'])),
        delta_final_tangent_rad=None if ga['final_tangent_rad'] is None or gb['final_tangent_rad'] is None else
            float(wrap_angle(gb['final_tangent_rad']-ga['final_tangent_rad'])),
        delta_clearance_m=b['original_geometry']['minimum_clearance_m']-a['original_geometry']['minimum_clearance_m'],
        delta_cart_clearance_m=b['cart_geometry']['minimum_clearance_m']-a['cart_geometry']['minimum_clearance_m'],
        delta_current_pixels=b['current']['visible_pixels']-a['current']['visible_pixels'],
        current_pixel_ratio=b['current']['visible_pixels']/a['current']['visible_pixels'] if a['current']['visible_pixels'] else None,
        before_history=a['history'], after_history=b['history'],
        before_prior_history=a['prior_history'], after_prior_history=b['prior_history'],
        after_inward_interior_segments=gb['inward_interior_segments'])
