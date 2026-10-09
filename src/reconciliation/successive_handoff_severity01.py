"""Saved-only geometry screening. Selection accepts geometry fields only."""
from contextlib import contextmanager
from dataclasses import dataclass
import sys
import numpy as np
from .se2 import relative_pose, wrap_angle
from .spatial_correspondence_selector import SpatialCurve, diagnostic_guard, wrap as continuous_wrap
from .spatial_entry_suffix import suffix_reference
from .gp_se2_join01 import forward_projection
from .successive_isaac_four_method01 import auc
from .join_source02 import whole_raw_polyline_check

HANDOFFS = [f'C{i}_to_C{i+1}' for i in range(1, 11)]
CONTROLS = ['C8_to_C9', 'C10_to_C11']
EPS = 1e-12
WINDOW = .30
DIMENSIONS = dict(progress='removed_prefix_fraction', gap='B_to_E_gap_m',
    turn='abs_transition_turn_demand_rad', fresh_turn='fresh_suffix_abs_net_yaw_rad',
    revision='local_polyline_separation_m')
PNGS = ['handoff_severity_overview.png', 'hard_candidate_geometry.png']


@contextmanager
def saved_only_guard():
    """Fail before any known scientific/runtime construction, even via an import."""
    with diagnostic_guard() as counts:
        prior = sys.getprofile()
        def profile(frame, event, arg):
            prior(frame, event, arg)
            if event != 'call':
                return
            name, path = frame.f_code.co_name, frame.f_code.co_filename
            if (name in {'hermite_bridge', 'staged_reference', 'SimulationApp',
                         'capture_rgb', 'capture_observation', 'generate', 'Popen'}
                    or name == '__init__' and any(x in path for x in
                        ['simulation_app', 'robotless_online_handoffs', 'subprocess.py'])):
                raise RuntimeError('saved-only severity audit forbids '+path+':'+name)
        sys.setprofile(profile)
        try:
            yield counts
        finally:
            sys.setprofile(prior)


def incoming_tangent(P, B):
    d = np.asarray(B)[:2]-np.asarray(P)[:2]
    if not np.isfinite(d).all() or np.linalg.norm(d) <= EPS:
        return None, 'P_TO_B_DISPLACEMENT_UNDEFINED; no B-yaw substitution'
    return float(np.arctan2(d[1], d[0])), None


def geometry_descriptors(F, raw, A, B, P, env):
    """No optimization, bridge construction or native trajectory input."""
    F, raw, B = np.asarray(F), np.asarray(raw), np.asarray(B)
    before = F.tobytes(), raw.tobytes()
    suffix, _, entry = suffix_reference(F, raw, A, B)
    c = entry['correspondence']; E = np.array(c['target_world'])
    k = c['segment']; edge = F[k+1,:2]-F[k,:2]
    fresh_tangent = float(np.arctan2(edge[1], edge[0]))
    incoming, reason = incoming_tangent(P, B)
    turn = None if incoming is None else float(wrap_angle(fresh_tangent-incoming))
    local = relative_pose(E, suffix)
    net = float(wrap_angle(F[-1,2]-E[2]))
    edges = np.diff(suffix[:,:2], axis=0)
    tangent_change = None if np.any(np.linalg.norm(edges,axis=1)<=EPS) else float(
        abs(wrap_angle(np.arctan2(edges[:,1],edges[:,0])-fresh_tangent)).max())
    checks = {n: whole_raw_polyline_check(w, env) for n,w in
        [('original',F),('suffix',suffix),('hypothetical_connector',np.vstack([B,E]))]}
    assert before == (F.tobytes(),raw.tobytes())
    row = dict(E_star_arc_m=c['arc_m'], E_star_progress=c['normalized_progress'],
        E_star_segment=k,E_star_alpha=c['alpha'],E_star_world=E.tolist(),
        E_star_nearest_raw_row=c['nearest_raw_row_diagnostic'],remaining_original_arc_m=c['remaining_arc_m'],
        removed_prefix_arc_m=c['arc_m'],removed_prefix_fraction=c['normalized_progress'],
        B_to_E_gap_m=c['position_gap_m'],B_to_E_yaw_gap_rad=c['yaw_gap_rad'],
        incoming_old_tangent_rad=incoming,fresh_entry_tangent_rad=fresh_tangent,
        signed_transition_turn_demand_rad=turn,abs_transition_turn_demand_rad=None if turn is None else abs(turn),
        incoming_descriptor_failure=reason,
        fresh_suffix_final_lateral_m=float(local[-1,1]),fresh_suffix_max_abs_lateral_m=float(abs(local[:,1]).max()),
        fresh_suffix_net_yaw_rad=net,fresh_suffix_abs_net_yaw_rad=abs(net),
        fresh_suffix_max_abs_tangent_change_rad=tangent_change,
        A_to_B_translation_m=float(np.linalg.norm(B[:2]-np.asarray(A)[:2])),
        A_to_B_yaw_change_rad=float(wrap_angle(B[2]-A[2])),
        original_FRESH_clearance_m=checks['original']['minimum_clearance_m'],
        suffix_clearance_m=checks['suffix']['minimum_clearance_m'],
        hypothetical_connector_clearance_m=checks['hypothetical_connector']['minimum_clearance_m'],
        connector_physical_overlap=checks['hypothetical_connector']['physical_overlap'])
    for n,q in checks.items():
        row[n+'_physical_overlap'] = q['physical_overlap']
        row[n+'_legacy_05_pass'] = q['legacy_5cm_margin_pass']
    return row, dict(entry=entry,suffix_world=suffix.tolist(),suffix_E_local=local.tolist(),
        safety=checks,connector_label='HYPOTHETICAL_DIAGNOSTIC_ONLY_NOT_EXECUTED',
        frame='world XY metres, +Z up, CCW yaw radians; suffix local = E*^-1 F; F=A*raw')


def turn50(times, yaws, theta_B, psi_fresh, horizon):
    """First qualifying sample with one following qualifying saved sample."""
    t, y = np.asarray(times,float), np.asarray(yaws,float)
    if t.ndim!=1 or y.shape!=t.shape or not len(t) or not np.isfinite(t).all() or not np.isfinite(y).all() or t[0]!=0 or np.any(np.diff(t)<=0):
        raise ValueError('finite increasing actual execution times starting at B required')
    delta = float(continuous_wrap(psi_fresh-theta_B))
    if abs(delta)<=EPS:
        return dict(time_s=None,status='N/A_NEAR_ZERO_TURN',delta_psi_F_rad=delta,confirmation_time_s=None)
    if t[-1]<horizon-EPS:
        return dict(time_s=None,status='INCOMPLETE_WINDOW',delta_psi_F_rad=delta,confirmation_time_s=None)
    use=t<=horizon; tt=t[use]
    q=np.sign(delta)*continuous_wrap(y[use]-theta_B)/abs(delta)
    i=next((i for i in range(len(tt)-1) if q[i]>=.5 and q[i+1]>=.5),None)
    return dict(time_s=None if i is None else float(tt[i]),
        status='CENSORED_NOT_REACHED' if i is None else 'OBSERVED',delta_psi_F_rad=delta,
        confirmation_time_s=None if i is None else float(tt[i+1]))


def native_context(F, times, poses, B, tangent):
    t, p = np.asarray(times,float), np.asarray(poses,float)
    duration = float(t[-1]); complete=duration>=WINDOW-EPS
    row=dict(native_position_auc_03_m_s=None,native_yaw_auc_03_rad_s=None,native_T_turn50_s=None,
        native_position_auc_full_m_s=None,native_yaw_auc_full_rad_s=None,native_T_turn50_full_s=None,
        native_metric_coverage='COMPLETE_0.30' if complete else 'INCOMPLETE_0.30_NO_EXTRAPOLATION',
        native_coverage_s=duration,native_T_turn50_status='INCOMPLETE_0.30',native_T_turn50_full_status='INCOMPLETE_0.30')
    if not complete:
        return row, dict(coverage_s=duration)
    projection=forward_projection(F,p)
    d=np.array([r['distance_m'] for r in projection]); y=np.array([r['yaw_error_rad'] for r in projection])
    short=turn50(t,p[:,2],B[2],tangent,WINDOW);full=turn50(t,p[:,2],B[2],tangent,duration)
    row.update(native_position_auc_03_m_s=auc(t,d,WINDOW),native_yaw_auc_03_rad_s=auc(t,y,WINDOW),
        native_position_auc_full_m_s=auc(t,d,duration),native_yaw_auc_full_rad_s=auc(t,y,duration),
        native_T_turn50_s=short['time_s'],native_T_turn50_full_s=full['time_s'],
        native_T_turn50_status=short['status'],native_T_turn50_full_status=full['status'])
    return row, dict(short_turn50=short,full_turn50=full,projection=projection,time_s=t.tolist())


@dataclass(frozen=True)
class GeometryCandidate:
    handoff: str
    index: int
    progress: float
    gap: float
    turn: float
    fresh_turn: float
    revision: float

    @classmethod
    def from_row(cls, row):
        values={name:row[key] for name,key in DIMENSIONS.items()}
        if any(v is None or not np.isfinite(v) for v in values.values()):
            raise ValueError('TECHNICAL_BLOCKED: undefined ranking geometry')
        return cls(row['handoff'],row['handoff_index'],**values)


def select_geometry(rows):
    """Explicit field whitelist excludes native metrics and every method outcome."""
    candidates=[GeometryCandidate.from_row(r) for r in rows if r['evolution']=='EVOLVING']
    def order(dimension, pool):
        return sorted(pool,key=lambda r:(-getattr(r,dimension),-r.gap,-r.fresh_turn,r.index))
    rankings={k:[r.handoff for r in order(k,candidates)] for k in DIMENSIONS}
    pareto=[]
    for r in candidates:
        dominated=any(all(getattr(z,k)>=getattr(r,k) for k in DIMENSIONS) and
            any(getattr(z,k)>getattr(r,k) for k in DIMENSIONS) for z in candidates)
        if not dominated:pareto.append(r.handoff)
    roles={}; remaining=list(candidates)
    for role,dim in [('TURN','turn'),('PROGRESS','progress'),('REVISION','revision')]:
        if remaining:
            chosen=order(dim,remaining)[0];roles[role]=chosen.handoff;remaining.remove(chosen)
    return dict(rankings=rankings,top3={k:v[:3] for k,v in rankings.items()},
        pareto_hard=pareto,recommended=roles,
        selection_inputs=[*DIMENSIONS.values(),'handoff_index'],no_weighted_scalar=True)
