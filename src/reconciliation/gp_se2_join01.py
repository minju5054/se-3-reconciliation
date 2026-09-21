"""JOIN-01 frozen geometry, forward projection and sampled attachment metrics.

No controller, optimization or inference is imported here. Original FRESH is an
observation-anchored spatial polyline, never a time-labelled upstream output.
"""
from __future__ import annotations

import numpy as np

from .gp_se2_reference import poses, interpolate_rows
from .se2 import wrap_angle

METHODS = ('M0_NATIVE', 'M0_ADAPTER', 'M1_RIGID', 'M3_CURRENT_GP', 'M4_JOIN_GP')
JOIN_TIMES = (.4, .6, .8, 1., 1.2, 1.4)
POSITION_M = .10
YAW_RAD = np.pi / 12
DWELL_S = .30


def nearest_index(fresh, boundary, weights=(10., 10., 1.)):
    f = poses(fresh)
    b, w = np.asarray(boundary, float), np.asarray(weights, float)
    if b.shape != (3,) or w.shape != (3,) or not np.isfinite([b, w]).all() or np.any(w <= 0):
        raise ValueError('finite pose and official positive weights required')
    d = f - b
    d[:, 2] = np.arctan2(np.sin(d[:, 2]), np.cos(d[:, 2]))
    return int(np.argmin(np.sum(d*d*w, axis=1)))


def join_candidates(fresh, boundary, *, weights=(10., 10., 1.), dt=.1, horizon=3.):
    f = poses(fresh)
    j0 = nearest_index(f, boundary, weights)
    indices = sorted({min(j0+i, len(f)-1) for i in range(5)})
    result = []
    for tau in JOIN_TIMES:
        k = round(tau / dt)
        if not np.isclose(k*dt, tau, atol=1e-12, rtol=0):
            raise ValueError('join time must coincide with original support grid')
        if horizon-tau < DWELL_S-1e-12:
            continue
        for j in indices:
            result.append(dict(join_time_s=tau, support_index=k, join_index=j,
                               nearest_original_index=j0, label=f't{k:02d}_j{j:03d}'))
    return result


def post_join_reference(fresh, candidate, support_times):
    f = poses(fresh)
    t = np.asarray(support_times, float)
    k, j = candidate['support_index'], candidate['join_index']
    if not 1 <= k < len(t) or not 0 <= j < len(f):
        raise ValueError('invalid support/original row index')
    if not np.isclose(t[k], candidate['join_time_s'], atol=1e-12, rtol=0) or t[-1]-t[k] < DWELL_S-1e-12:
        raise ValueError('insufficient or inconsistent join interval')
    suffix = f[j:].copy()
    clock = np.linspace(t[k], t[-1], len(suffix)) if len(suffix)>1 else np.array([t[k]])
    return interpolate_rows(suffix, clock, t[k:])


def tube_margins(actual, reference):
    a, r = poses(actual), poses(reference)
    if a.shape != r.shape:
        raise ValueError('one corresponding original-FRESH target per support required')
    yaw = wrap_angle(a[:, 2]-r[:, 2])
    return np.r_[POSITION_M**2-np.sum((a[:, :2]-r[:, :2])**2, axis=1),
                 YAW_RAD-yaw, YAW_RAD+yaw]


def select_candidate(records):
    valid = [r for r in records if r.get('full_valid') is True]
    return min(valid, key=lambda r: (r['join_time_s'], r['objective'], r['join_index'],
                                    r['initialization'], r['source_label'])) if valid else None


def forward_projection(fresh, samples, *, initial_progress=0.):
    """Closest point on the remaining continuous polyline, exact lexical ties.

    Progress is original fractional row coordinate, not time/distance. The
    current segment is clipped at previous progress before minimization. Thus
    choosing a farther future point cannot later jump back to an earlier row.
    Zero-length segments retain source identity and first-endpoint yaw.
    """
    f, x = poses(fresh), poses(samples)
    progress = float(initial_progress)
    if not 0 <= progress <= len(f)-1:
        raise ValueError('initial progress outside original path')
    out = []
    for pose in x:
        candidates = []
        for j in range(max(0, int(np.floor(progress))), len(f)-1):
            lo = max(0., progress-j)
            if lo > 1:
                continue
            delta = f[j+1, :2]-f[j, :2]
            den = float(delta@delta)
            alpha = float(np.clip((pose[:2]-f[j, :2])@delta/den, lo, 1.)) if den>0 else lo
            xy = f[j, :2]+alpha*delta
            candidates.append((float(np.sum((xy-pose[:2])**2)), j, alpha, xy))
        if not candidates:
            candidates = [(float(np.sum((f[-1, :2]-pose[:2])**2)), len(f)-1, 0., f[-1, :2])]
        distance2, j, alpha, xy = min(candidates, key=lambda a: (a[0], a[1], a[2]))
        yaw = f[j, 2] if j == len(f)-1 else wrap_angle(f[j, 2]+alpha*wrap_angle(f[j+1, 2]-f[j, 2]))
        progress = max(progress, j+alpha)
        out.append(dict(progress=progress, segment_index=j, alpha=alpha,
                        projected_pose=[*xy, float(yaw)], distance_m=float(np.sqrt(distance2)),
                        yaw_error_rad=float(abs(wrap_angle(pose[2]-yaw)))))
    return out


def sustained_join(times, samples, fresh):
    t, x = np.asarray(times, float), poses(samples)
    if t.shape != (len(x),) or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
        raise ValueError('finite strictly increasing sample times required')
    projection = forward_projection(fresh, x)
    d = np.array([r['distance_m'] for r in projection])
    yaw = np.array([r['yaw_error_rad'] for r in projection])
    good = (d <= POSITION_M) & (yaw <= YAW_RAD)
    joined = None
    for i, start in enumerate(t):
        end = start+DWELL_S
        if end > t[-1]+1e-12:
            break
        stop = min(int(np.searchsorted(t, end-1e-12, side='left')), len(t)-1)
        if good[i:stop+1].all():
            joined = i
            break
    return dict(join_success=joined is not None,
                sustained_join_time_s=None if joined is None else float(t[joined]),
                joined_pose_world=None if joined is None else x[joined].tolist(),
                position_threshold_m=POSITION_M, yaw_threshold_rad=YAW_RAD,
                required_sustained_duration_s=DWELL_S,
                post_join_mean_distance_m=None if joined is None else float(np.mean(d[joined:])),
                post_join_max_distance_m=None if joined is None else float(np.max(d[joined:])),
                pre_join_position_auc_m_s=None if joined is None else float(np.trapezoid(d[:joined+1], t[:joined+1])),
                full_window_position_auc_m_s=float(np.trapezoid(d, t)),
                times_s=t, distance_m=d, yaw_error_rad=yaw, projection=projection,
                maximum_sample_spacing_s=float(np.max(np.diff(t))) if len(t)>1 else None,
                sampled_sustained_only=True, continuous_time_proof=False,
                failure_auc_policy='pre-join N/A without join; full-window AUC separately available')


def box_polygon(obstacle):
    from shapely.geometry import Polygon
    p, size = np.asarray(obstacle['pose_world'], float), np.asarray(obstacle['dimensions_m'], float)
    if p.shape != (3,) or size.shape != (3,) or not np.isfinite(np.r_[p, size]).all() or np.any(size<=0):
        raise ValueError('finite SE2 box pose and positive xyz dimensions required')
    a = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*size[:2]/2
    c, s = np.cos(p[2]), np.sin(p[2])
    return Polygon(a@np.array([[c,s],[-s,c]])+p[:2])


def obstacle_on_old(old_observation, old_world, distance_m, dimensions):
    """Freeze a primitive on OLD's observation-to-path arc, without extending it."""
    old = poses(np.vstack([old_observation, poses(old_world)]))
    lengths = np.linalg.norm(np.diff(old[:, :2], axis=0), axis=1)
    arc = np.r_[0., np.cumsum(lengths)]
    if distance_m<=0 or distance_m>arc[-1]:
        raise ValueError('PLACEMENT_OUTSIDE_ACTUAL_OLD_FUTURE')
    j = min(int(np.searchsorted(arc, distance_m, side='right')-1), len(old)-2)
    if lengths[j]<=0:
        raise ValueError('degenerate OLD placement segment')
    alpha = (distance_m-arc[j])/lengths[j]
    xy = old[j,:2]+alpha*(old[j+1,:2]-old[j,:2])
    yaw = np.arctan2(*(old[j+1,:2]-old[j,:2])[::-1])
    return dict(pose_world=[*xy,float(yaw)], dimensions_m=list(dimensions),
                center_z_m=float(dimensions[2])/2, distance_from_old_observation_along_old_m=distance_m,
                old_segment_index=j, alpha=float(alpha), prim_path='/World/JOIN01Obstacle',
                representation='runtime static box; unchanged Hospital asset; world metres')
