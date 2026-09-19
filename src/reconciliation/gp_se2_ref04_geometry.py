"""Saved-geometry diagnostics with unchanged original clearance/gate predicates.

No trajectory optimization or rollout occurs here. Refinement queries an optional
saved-trajectory interpolant, or the supplied linear polyline, and never replaces
the original acceptance result. All distances are world XY metres.
"""
from __future__ import annotations

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from .gp_se2_reference import directed_gate_crossings

REFINEMENT_TIME_WIDTH_S = .001


def _inputs(points, times):
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] not in (2, 3) or not len(p) or not np.isfinite(p).all():
        raise ValueError('nonempty finite world Nx2 or Nx3 points required')
    t = None if times is None else np.asarray(times, dtype=np.float64)
    if t is not None and (t.shape != (len(p),) or not np.isfinite(t).all() or np.any(np.diff(t) <= 0)):
        raise ValueError('finite strictly increasing times matching points required')
    return p, t


def _geometry(points):
    xy = np.asarray(points)[:, :2]
    return Point(xy[0]) if len(xy) == 1 or np.all(xy == xy[0]) else LineString(xy)


def _check(points, env, radius, required, bound):
    result = env.check_polyline(points, radius=radius, required_clearance=required+bound)
    if bound > 0:
        geometry = _geometry(points)
        known = bool(env.workspace.covers(geometry) and
                     geometry.distance(env.workspace.boundary) >= radius+bound+env.numerical_tolerance_m)
        result.update(workspace_known=known, unknown=not known)
        if not known:
            result.update(status='UNKNOWN_WORKSPACE', clearance_valid=False)
    result.update(curved_path_error_bound_m=bound,
                  minimum_clearance_lower_bound_m=result['minimum_clearance_m']-bound)
    return result


def _minimum(points, env):
    geometry = _geometry(points)
    obstacle = env.parts[env.tree.nearest(geometry)]
    on_path, on_obstacle = nearest_points(geometry, obstacle)
    return np.asarray(on_path.coords[0], float), np.asarray(on_obstacle.coords[0], float)


def _query_pose(time, lo, hi, left, right, pose_at):
    value = (np.asarray(left)+(time-lo)/(hi-lo)*(np.asarray(right)-left)
             if pose_at is None else np.asarray(pose_at(float(time)), dtype=float))
    if value.ndim != 1 or value.size not in (2, 3) or not np.isfinite(value).all():
        raise ValueError('pose_at must return finite world XY or world pose')
    return value[:2]


def _refine_sweep(lo, hi, left, right, env, radius, required, bound, pose_at):
    left,right=np.asarray(left)[:2],np.asarray(right)[:2]
    original = [float(lo), float(hi)]
    queries = 0
    while hi-lo > REFINEMENT_TIME_WIDTH_S:
        mid = (lo+hi)/2
        if mid == lo or mid == hi:
            return dict(bracket_s=[float(lo),float(hi)],original_bracket_s=original,
                        status='FLOAT_RESOLUTION_LIMIT',half_sweep_queries=queries)
        point = _query_pose(mid, lo, hi, left, right, pose_at)
        lcheck = _check(np.vstack([left,point]),env,radius,required,bound)
        rcheck = _check(np.vstack([point,right]),env,radius,required,bound)
        queries += 2
        if not lcheck['clearance_valid']:
            hi,right = mid,point
        elif not rcheck['clearance_valid']:
            lo,left = mid,point
        else:
            return dict(bracket_s=[float(lo),float(hi)],original_bracket_s=original,
                status='PARENT_BRACKET_RETAINED_BOTH_HALF_SWEEPS_VALID',half_sweep_queries=queries,
                explanation='The original failing chord is retained; supplemental half chords do not prove its earliest violation time.')
    return dict(bracket_s=[float(lo),float(hi)],original_bracket_s=original,
                status='REFINED_TO_AT_MOST_1_MS',half_sweep_queries=queries)


def polyline_audit(points, env, config, *, times=None, curve_bound=0., pose_at=None, refine=True):
    """Check original swept geometry, then supplement its first failure bracket.

    Minimum location/time refer to a linear polyline segment. With ``pose_at``,
    refinement evaluates only that supplied saved-trajectory interpolant; the
    coarse original acceptance and conservative global curve bound stay fixed.
    Unknown workspace counts as an invalid segment, never free space.
    """
    p,t = _inputs(points,times)
    bound=float(curve_bound)
    if not np.isfinite(bound) or bound < 0:raise ValueError('finite nonnegative curve bound required')
    radius=float(config['footprint']['radius_m']);required=float(config['footprint']['required_clearance_m'])
    eps=float(env.numerical_tolerance_m);threshold=required+bound+eps
    report=_check(p,env,radius,required,bound)
    node=np.asarray(env.clearance(p[:,:2],radius),float)
    workspace=np.asarray(env.workspace_margin(p[:,:2],radius),float) >= bound+eps
    if not np.isfinite(node).all():raise ValueError('nonfinite direct environment query')
    segments=[]
    for i in range(len(p)-1):
        checked=_check(p[i:i+2],env,radius,required,bound)
        location,obstacle=_minimum(p[i:i+2],env)
        delta=p[i+1,:2]-p[i,:2];length2=float(delta@delta)
        fraction=0. if length2==0 else float(np.clip((location-p[i,:2])@delta/length2,0.,1.))
        at=None if t is None else float(t[i]+fraction*(t[i+1]-t[i]))
        segments.append(dict(index=i,start_time_s=None if t is None else float(t[i]),
            end_time_s=None if t is None else float(t[i+1]),minimum_clearance_m=checked['minimum_clearance_m'],
            minimum_location_world=location.tolist(),nearest_obstacle_location_world=obstacle.tolist(),
            minimum_time_s=at,segment_fraction=fraction,clearance_valid=checked['clearance_valid'],
            physical_overlap=checked['physical_overlap'],workspace_known=checked['workspace_known'],status=checked['status']))
    if segments:
        minimum=segments[int(np.argmin([s['minimum_clearance_m'] for s in segments]))]
        location,minimum_time=minimum['minimum_location_world'],minimum['minimum_time_s']
    else:
        location=p[0,:2].tolist();minimum_time=None if t is None else float(t[0])
    invalid_nodes=np.flatnonzero((node<threshold)|~workspace)
    invalid_segments=[s for s in segments if not s['clearance_valid']]
    first=invalid_segments[0] if invalid_segments else None
    bracket=None if first is None or t is None else [first['start_time_s'],first['end_time_s']]
    refinement=None
    if refine and bracket is not None:
        i=first['index'];refinement=_refine_sweep(t[i],t[i+1],p[i],p[i+1],env,radius,required,bound,pose_at)
    result=dict(points_world=p.tolist(),times_s=None if t is None else t.tolist(),node_clearance_m=node.tolist(),
        node_workspace_known=workspace.tolist(),minimum_clearance_m=report['minimum_clearance_m'],
        effective_threshold_m=threshold,physical_overlap=report['physical_overlap'],clearance_valid=report['clearance_valid'],
        workspace_known=report['workspace_known'],first_sampled_violation_time_s=None if t is None or not len(invalid_nodes) else float(t[invalid_nodes[0]]),
        first_sampled_violation_index=None if not len(invalid_nodes) else int(invalid_nodes[0]),
        first_swept_violation_bracket_s=bracket,first_swept_violation_segment_index=None if first is None else first['index'],
        first_refined_violation_bracket_s=None if refinement is None else refinement['bracket_s'],
        minimum_location_world=location,minimum_time_s=minimum_time,segments=segments,
        original_environment_check=report,refinement=refinement,curve_bound_m=bound,geometry_epsilon_m=eps,
        physical_overlap_threshold_m=-eps,nominal_required_clearance_m=required,
        minimum_location_time_semantics='closest point and linear time coordinate on the supplied XY polyline, not an exact curved-trajectory extremum',
        refinement_semantics='supplemental saved-pose queries' if pose_at is not None else 'supplemental linear-polyline interpolation',
        first_violation_semantics='clearance below effective acceptance threshold or unknown workspace; original status also distinguishes BORDERLINE',
        original_acceptance_unchanged_by_refinement=True,continuous_time_collision_proof=False)
    return result


def _refine_gate(lo,hi,left,right,center,normal,pose_at):
    left,right=np.asarray(left)[:2],np.asarray(right)[:2]
    original=[float(lo),float(hi)];start=float((left[:2]-center)@normal)
    while hi-lo>REFINEMENT_TIME_WIDTH_S:
        mid=(lo+hi)/2
        if mid==lo or mid==hi:break
        point=_query_pose(mid,lo,hi,left,right,pose_at);value=float((point[:2]-center)@normal)
        if (value<0)==(start<0):lo,left=mid,point
        else:hi,right=mid,point
    return dict(bracket_s=[float(lo),float(hi)],original_bracket_s=original,
        width_s=float(hi-lo),semantics='supplemental zero-plane bracket; original tolerance-band/gate decision remains unchanged')


def _interval_relation(offset, half, tolerance):
    margin=half-abs(offset)
    relation='INSIDE' if abs(offset)<half-tolerance else 'OUTSIDE' if abs(offset)>half+tolerance else 'ENDPOINT'
    return dict(finite_interval_relation=relation,outside_distance_m=float(max(0.,-margin)),
        outside_acceptance_distance_m=float(max(0.,tolerance-margin)),
        endpoint_tolerance_band=relation=='ENDPOINT',
        finite_interval_relation_semantics='ENDPOINT means within original gate tolerance of either interval endpoint; outside_distance is distance beyond the nominal finite center interval')


def gate_audit(points,times,gates,env,config,*,pose_at=None,scope='FULL_EXECUTION'):
    """Describe finite directed gate events; original route result is separate.

    Validity of one geometric crossing does not assert ordered-route completion.
    A NO_CROSSING in a prediction or prefix is not automatically a rollout failure.
    The saved half-width is already a robot-center interval, never shrunk again.
    """
    p,t=_inputs(points,times)
    if t is None:raise ValueError('gate audit requires declared sample times')
    tolerance=float(config['evaluation']['gate_crossing_tolerance_m'])
    authoritative=directed_gate_crossings(p[:,:2],t,gates,tolerance=tolerance)
    records=[]
    for gate,original in zip(gates,authoritative['gates']):
        center=np.asarray(gate['center_xy'],float);normal=np.asarray(gate['normal_xy'],float)
        tangent=np.array([-normal[1],normal[0]]);half=float(gate['half_width_m'])
        signed=(p[:,:2]-center)@normal;sign=np.where(signed < -tolerance,-1,np.where(signed > tolerance,1,0))
        common=dict(gate_id=gate['gate_id'],scope=scope,gate_center_world=center.tolist(),gate_normal_world=normal.tolist(),
            gate_tangent_world=tangent.tolist(),half_width_m=half,tolerance_m=tolerance,
            center_interval_semantics='saved footprint/clearance-safe center interval; no second radius or clearance subtraction',
            original_gate_result=original,original_route_valid=authoritative['valid'],original_route_result=authoritative,
            validity_semantics='local directed finite crossing; original ordered-route result is reported separately')
        event_count=0;last=None;covered=set()
        for i in range(len(p)):
            if sign[i]==0:continue
            if last is not None and sign[last]!=sign[i]:
                a,z=last,i;middle=p[a+1:z,:2]
                grazing=bool(len(middle) and np.max(np.linalg.norm(middle-middle[0],axis=1))>tolerance)
                if len(middle):location=middle[0];at=float(t[a+1])
                else:
                    fraction=-signed[a]/(signed[z]-signed[a]);location=p[a,:2]+fraction*(p[z,:2]-p[a,:2]);at=float(t[a]+fraction*(t[z]-t[a]))
                offset=float((location-center)@tangent);inside=abs(offset)<half-tolerance;forward=sign[a]<sign[z]
                label='GRAZING' if grazing else 'REVERSE_CROSSING' if not forward else 'VALID_CROSSING' if inside else 'INVALID_INTERVAL_CROSSING'
                query=pose_at if pose_at is not None else lambda time: np.array([np.interp(time,t,p[:,axis]) for axis in (0,1)])
                refined=None if grazing else _refine_gate(t[a],t[z],p[a],p[z],center,normal,query)
                records.append(dict(**common,classification=label,valid=bool(forward and inside and not grazing),
                    status=label,**_interval_relation(offset,half,tolerance),
                    direction='FORWARD' if forward else 'REVERSE',crossing_time_s=at,bracket_s=[float(t[a]),float(t[z])],
                    refined_bracket_s=None if refined is None else refined['bracket_s'],refinement=refined,
                    location_world=location.tolist(),tangent_offset_m=offset,interval_margin_m=half-abs(offset),
                    strict_interval_margin_m=half-tolerance-abs(offset),within_finite_interval=bool(inside),grazing=grazing,
                    start_index=a,end_index=z,geometry_query=env.query(location,config['footprint']['radius_m'],config['footprint']['required_clearance_m'])))
                event_count+=1;covered.update(range(a+1,z))
            last=i
        # Same-side touch, initial/final tolerance-band contact, or boundary slide
        # must remain visible even if no completed directed crossing exists.
        band=np.flatnonzero(sign==0)
        groups=np.split(band,np.flatnonzero(np.diff(band)>1)+1) if len(band) else []
        for group in groups:
            if all(int(i) in covered for i in group):continue
            a,z=int(group[0]),int(group[-1]);location=p[a,:2];offset=float((location-center)@tangent)
            records.append(dict(**common,classification='GRAZING',valid=False,direction=None,crossing_time_s=None,
                status='GRAZING',**_interval_relation(offset,half,tolerance),
                contact_time_s=float(t[a]),bracket_s=[float(t[a]),float(t[z])],refined_bracket_s=None,refinement=None,
                location_world=location.tolist(),tangent_offset_m=offset,interval_margin_m=half-abs(offset),
                strict_interval_margin_m=half-tolerance-abs(offset),within_finite_interval=bool(abs(offset)<half-tolerance),
                grazing=True,start_index=a,end_index=z,geometry_query=env.query(location,config['footprint']['radius_m'],config['footprint']['required_clearance_m'])))
            event_count+=1
        if not event_count:
            records.append(dict(**common,classification='NO_CROSSING',valid=None,direction=None,crossing_time_s=None,
                status='NO_CROSSING_IN_HORIZON' if scope=='PREDICTION' else 'NO_CROSSING',
                finite_interval_relation=None,outside_distance_m=None,outside_acceptance_distance_m=None,endpoint_tolerance_band=None,
                bracket_s=None,refined_bracket_s=None,refinement=None,location_world=None,tangent_offset_m=None,
                interval_margin_m=None,strict_interval_margin_m=None,within_finite_interval=None,grazing=False,
                no_crossing_reason='ALL_AFTER_GATE' if np.all(sign>0) else 'ALL_BEFORE_GATE' if np.all(sign<0) else 'NO_COMPLETED_DIRECTIONAL_EVENT',
                interpretation='absence within this supplied window; do not call a short prediction/prefix a full-route failure'))
    return records


def spatial_gate_audit(points,gates,env,config):
    """Inspect row-ordered straight connectors without inventing physical times.

    Parameter i is input vertex i; fractional values only identify locations on
    the connector i→i+1. The original geometric crossing algorithm is reused,
    but every temporal key is renamed and no execution/forecast claim is made.
    """
    p,_=_inputs(points,None)
    rows=gate_audit(p,np.arange(len(p),dtype=float),gates,env,config,scope='SPATIAL_ROW_PARAMETER')
    rename={'crossing_time_s':'crossing_parameter','contact_time_s':'contact_parameter',
        'bracket_s':'bracket_parameter','refined_bracket_s':'refined_bracket_parameter',
        'original_bracket_s':'original_bracket_parameter','width_s':'width_parameter',
        'first_directed_crossing_s':'first_directed_crossing_parameter','crossings_s':'crossings_parameter',
        'original_route_valid':'ordered_spatial_route_valid','original_route_result':'ordered_spatial_route_result',
        'original_gate_result':'ordered_spatial_gate_result'}
    def changed(value):
        if isinstance(value,dict):return {rename.get(k,k):changed(v) for k,v in value.items()}
        if isinstance(value,list):return [changed(v) for v in value]
        return value
    result=changed(rows)
    for row in result:
        row.update(parameter_semantics='input row index plus linear segment fraction; not physical time, distance, prediction or execution',
            actual_motion_or_prediction=False,validity_semantics='ordered diagnostic connector geometry only; not actual route completion')
        if row['classification']=='NO_CROSSING':
            row['interpretation']='absence of crossing on these spatial connectors; no temporal or execution claim'
        if row.get('refinement') is not None:
            row['refinement']['semantics']='linear connector parameter localization, maximum parameter width 0.001; no seconds or physical timing'
    return result
