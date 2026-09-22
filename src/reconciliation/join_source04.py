"""Frozen persistence and evaluator-only affordance diagnostics. No inference/solve.

APOS remains a coarse image token; clamped output is censored. Raster first hit
and z=0 ground proxy are distinct and never become a model metric waypoint.
"""
from copy import deepcopy
import re

import numpy as np
from shapely.geometry import Point

from .join_source03 import trajectory_equal
from .se2 import wrap_angle

ORDER = ['K0_OFF', 'K0_OFF_SHAM', 'K1', 'K1_SHAM', 'K2', 'K4', 'K8']
KS = dict(zip(ORDER, [0, 0, 1, 1, 2, 4, 8]))
LABEL = 'COUNTERFACTUAL MOVING-POSE HISTORY; not an executed obstacle-reveal episode'


def construct_frames(bank, k):
    if len(bank) != 16 or k not in (0, 1, 2, 4, 8):
        raise ValueError('exact H16 and frozen K set required')
    frames = [deepcopy(row['ON' if i >= 16-k else 'OFF']['frame']) for i, row in enumerate(bank)]
    if len({f['frame_id'] for f in frames}) != 16 or len({f['path'] for f in frames}) != 16:
        raise ValueError('no duplicate frame padding')
    if any(b['capture_sim_time_s'] < a['capture_sim_time_s'] or
           b['capture_monotonic_ns'] <= a['capture_monotonic_ns'] for a,b in zip(frames,frames[1:])):
        raise ValueError('nonchronological rendered poses')
    if not all(f['all_bright'] and f['counterfactual_moving_pose_history'] for f in frames):
        raise ValueError('all frames must be explicitly rerendered BRIGHT counterfactuals')
    return frames


def premodel(bank, k, *, minimum_pixels=20, pose_atol=1e-12):
    construct_frames(bank, k)
    checks = {}
    transform = bank[0]['ON']['cart_transform']
    for i, row in enumerate(bank):
        off,on = row['OFF'],row['ON']
        checks[f'{i}:pair'] = (off['frame']['frame_id']==on['frame']['frame_id'] and
            off['camera']==on['camera'] and off['scene_signature']==on['scene_signature'] and
            off['lighting_sha256']==on['lighting_sha256'] and on['cart_transform']==transform and
            off['cart_pixels']==0 and off['present'] is False and on['present'] is True and
            off['stable'] and on['stable'] and off['same_product'] and on['same_product'])
        checks[f'{i}:source_pose'] = all(np.allclose(x['frame']['pose_world'],row['source_pose_world'],atol=pose_atol,rtol=0)
                                                 for x in (off,on))
        if i >= 16-k:
            checks[f'{i}:visible'] = on['cart_pixels'] >= minimum_pixels
            checks[f'{i}:nonoverlap'] = on['cart_edge_clearance_m'] >= 0
    return dict(valid=all(checks.values()), checks=checks,
                status='PREMODEL_VALID' if all(checks.values()) else 'PREMODEL_K_UNAVAILABLE')


def pixel_ray(pixel, camera):
    u,v=np.asarray(pixel,float)
    if not np.isfinite([u,v]).all(): raise ValueError('finite pixel required')
    K=np.asarray(camera['actual_intrinsics']['K'],float)
    T=np.asarray(camera['T_world_camera'],float)
    local=np.array([(u-K[0,2])/K[0,0],-(v-K[1,2])/K[1,1],-1.])
    direction=T[:3,:3]@local;direction/=np.linalg.norm(direction)
    return T[:3,3].copy(),direction


def ground_intersection(origin, direction, z=0.):
    origin,direction=np.asarray(origin,float),np.asarray(direction,float)
    if abs(direction[2]) < 1e-12: return None
    t=(z-origin[2])/direction[2]
    return None if t<=0 else origin+t*direction


def hit_class(path, cart_prefix, target_prefix):
    if not path or path in ('BACKGROUND','UNLABELLED','None'): return 'none'
    if path.startswith(cart_prefix): return 'cart'
    if path.startswith(target_prefix): return 'target'
    name=path.lower()
    if 'floor' in name or 'ground' in name: return 'floor'
    if 'wall' in name: return 'wall'
    return 'other'


def mask_relation(pixel, mask, ids):
    selected=np.isin(mask,ids);ys,xs=np.nonzero(selected)
    out=dict(cart_pixels=int(len(xs)),inside_cart_mask=None,distance_to_cart_mask_px=None,
             relative_to_cart_bbox=None,relative_to_centroid_px=None,bbox=None)
    if pixel is None or not len(xs): return out
    u,v=map(float,pixel);x,y=int(np.floor(u)),int(np.floor(v))
    out.update(inside_cart_mask=bool(selected[y,x]) if 0<=y<mask.shape[0] and 0<=x<mask.shape[1] else None,
        distance_to_cart_mask_px=float(np.min(np.hypot(xs-u,ys-v))),
        relative_to_cart_bbox='left' if u<xs.min() else 'right' if u>xs.max() else 'within-horizontal-span',
        relative_to_centroid_px=[float(u-xs.mean()),float(v-ys.mean())],
        bbox=[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())])
    return out


def affordance(response, terminal, mask, depth, environment, cart_polygon, center_xy, target_prefix):
    p=response.get('pointing') or {};pixel=p.get('apos_px');clamped=bool(p.get('apos_clamped'))
    out=dict(pointing=p,raw_pointing_tokens=re.findall(r'<(?:apos|opos)[^>]*>',response.get('raw_text','')),
        target_visible=response.get('visible'),stop=response.get('stop'),
        interpretation='CENSORED_BY_CLAMPING' if clamped else 'COARSE_UNCLAMPED_POINT',
        ray_kind='BOUNDARY_RAY_PROXY' if clamped else 'COARSE_PIXEL_RAY',
        mask=mask_relation(pixel,mask,terminal['instance']['matched_instance_ids']),
        first_hit_valid=False,first_hit_world_xyz=None,first_hit_prim=None,first_hit_class='none',
        first_hit_distance_m=None,ground_intersection_valid=False,ground_xy=None,
        ground_local_forward_left_m=None,ground_bearing_rad=None,ground_edge_clearance_m=None,
        ground_cart_edge_clearance_m=None,ground_side='ambiguous',clear_side=False,
        target_pixels=terminal['target_pixels'],metric_model_waypoint=False)
    if pixel is None or p.get('apos_state')!='point':
        out['unavailable_reason']='no point token';return out
    camera=terminal['camera'];origin,direction=pixel_ray(pixel,camera)
    out.update(ray_world_origin=origin.tolist(),ray_world_direction=direction.tolist())
    u,v=map(float,pixel);x,y=int(np.floor(u)),int(np.floor(v))
    if 0<=y<depth.shape[0] and 0<=x<depth.shape[1]:
        distance=float(depth[y,x]);path=terminal['instance']['idToLabels'].get(str(int(mask[y,x])))
        valid=np.isfinite(distance) and distance>0 and hit_class(path,terminal['cart_prefix'],target_prefix)!='none'
        out.update(first_hit_valid=bool(valid),first_hit_prim=path,
                   first_hit_class=hit_class(path,terminal['cart_prefix'],target_prefix),
                   first_hit_source='same-product Isaac distance_to_camera + instance raster; pixel-sampled first rendered surface',
                   first_hit_sample_pixel_xy=[x,y],subpixel_offset=[u-x,v-y])
        if valid:
            sample_origin,sample_direction=pixel_ray([x+.5,y+.5],camera)
            out.update(first_hit_distance_m=distance,first_hit_world_xyz=(sample_origin+distance*sample_direction).tolist(),
                       first_hit_raster_center=[x+.5,y+.5],first_hit_raster_ray_world_direction=sample_direction.tolist())
    ground=ground_intersection(origin,direction)
    if ground is None:
        out['ground_unavailable_reason']='no forward ray intersection with nominal z=0';return out
    pose=np.asarray(terminal['frame']['pose_world']);f=np.array([np.cos(pose[2]),np.sin(pose[2])]);left=np.array([-f[1],f[0]])
    rel=ground[:2]-pose[:2];local=np.array([rel@f,rel@left]);bearing=float(np.arctan2(local[1],local[0]))
    # Side must clear the actual lateral cart extent plus footprint + required margin.
    vertices=np.asarray(cart_polygon.convex_hull.exterior.coords) # extent only, never collision oracle
    side_coords=(vertices-center_xy)@left;offset=float((ground[:2]-center_xy)@left)
    side='left' if offset>side_coords.max()+.25 else 'right' if offset<side_coords.min()-.25 else 'ambiguous'
    q=environment.query(ground[:2]);cart_clear=float(cart_polygon.distance(Point(ground[:2]))-.20)
    out.update(ground_intersection_valid=True,ground_xy=ground[:2].tolist(),ground_local_forward_left_m=local.tolist(),
        ground_bearing_rad=bearing,ground_edge_clearance_m=float(q['clearance_m']),ground_status=q['status'],
        ground_cart_edge_clearance_m=cart_clear,ground_side=side,
        clear_side=bool(not clamped and out['first_hit_valid'] and out['first_hit_class']=='floor' and
                       q['status']=='CLEARANCE_VALID' and side!='ambiguous' and abs(bearing)>=np.deg2rad(20)))
    return out


def motion_geometry(local,world,pose,target_xy,center_xy,forward,cart_bounds_progress):
    a=np.asarray(local,float);w=np.asarray(world,float)
    if a.ndim!=2 or a.shape[1]!=3 or not len(a) or not np.isfinite(a).all(): raise ValueError('finite raw Nx3')
    delta=np.diff(a[:,:2],axis=0);chords=np.linalg.norm(delta,axis=1)
    reliable=np.flatnonzero(chords>=.02)
    initial=None if not len(reliable) else float(np.arctan2(*delta[reliable[0]][::-1]))
    i=int(np.argmax(np.abs(a[:,1])));major=float(a[i,1])
    side='left' if major>=.20 else 'right' if major<=-.20 else ('left' if initial is not None and initial>=np.deg2rad(20) else 'right' if initial is not None and initial<=-np.deg2rad(20) else 'ambiguous')
    progress=float((w[-1,:2]-center_xy)@forward)
    return dict(N=len(a),max_abs_lateral_m=float(abs(major)),major_lateral_m=major,final_lateral_m=float(a[-1,1]),
        accumulated_abs_yaw_rad=float(np.abs(wrap_angle(np.diff(a[:,2]))).sum()),
        max_abs_yaw_increment_rad=float(np.max(np.abs(wrap_angle(np.diff(a[:,2]))))) if len(a)>1 else 0.,
        max_abs_yaw_rad=float(np.max(np.abs(wrap_angle(a[:,2])))),final_yaw_rad=float(a[-1,2]),
        initial_reliable_segment=None if not len(reliable) else int(reliable[0]),initial_tangent_rad=initial,
        broad_side=side,target_directed_progress_m=float(np.linalg.norm(np.asarray(pose)[:2]-target_xy)-np.linalg.norm(w[-1,:2]-target_xy)),
        endpoint_cart_progress_m=progress,endpoint_beyond_front_m=progress-cart_bounds_progress[0],
        endpoint_beyond_rear_m=progress-cart_bounds_progress[1],connector_included=False)


def agreement(apos,motion,safe):
    if not apos['clear_side']:
        return 'APOS_OBSTACLE_DIRECTED_OR_AMBIGUOUS'
    if motion['broad_side']=='ambiguous': return 'APOS_OBSTACLE_DIRECTED_OR_AMBIGUOUS'
    if apos['ground_side']!=motion['broad_side']: return 'APOS_TRAJECTORY_DIRECTION_DISAGREEMENT'
    return 'APOS_CLEAR_SIDE_TRAJ_SAFE' if safe else 'APOS_CLEAR_SIDE_TRAJ_UNSAFE'


def persistence(results):
    required=['K0_OFF','K0_OFF_SHAM','K1','K1_SHAM']
    if not all(k in results for k in required): return dict(classification='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE')
    sham={a:trajectory_equal(results[a]['raw_local'],results[b]['raw_local']) for a,b in [('K0_OFF','K0_OFF_SHAM'),('K1','K1_SHAM')]}
    if not all(sham.values()):return dict(classification='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE',sham=sham)
    base=results['K1'];comparisons={}
    if base['geometry_on']['whole']['clearance_valid']:
        return dict(classification='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE',sham=sham,
                    reason='all-BRIGHT K1 already safe; sudden-reveal failure not reproduced')
    for key in ('K2','K4','K8'):
        if key not in results:continue
        r=results[key];g=r['geometry_on'];bg=base['geometry_on']
        gain=g['whole']['minimum_clearance_m']-bg['whole']['minimum_clearance_m']
        a,b=bg['first_unsafe_segment_zero_based'],g['first_unsafe_segment_zero_based']
        onset=(a is not None and b is not None and b>=a+1 and all(
            g['segment_checks'][i]['minimum_clearance_m']>=bg['segment_checks'][i]['minimum_clearance_m']
            for i in range(min(a,len(g['segment_checks']),len(bg['segment_checks'])))))
        comparisons[key]=dict(clearance_gain_m=gain,material=gain>=.02,unsafe_onset_later=onset,
            equivalent=trajectory_equal(base['raw_local'],r['raw_local']),
            safe_raw_future=g['whole']['clearance_valid'] and not r['stop'] and r['motion']['target_directed_progress_m']>0)
    if not comparisons:label='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE'
    elif not base['geometry_on']['whole']['clearance_valid'] and any(c['safe_raw_future'] for c in comparisons.values()):label='PERSISTENCE_RECOVERS_SAFE_FRESH'
    elif any(c['material'] or c['unsafe_onset_later'] for c in comparisons.values()):label='PERSISTENCE_IMPROVES_BUT_REMAINS_UNSAFE'
    elif any(not c['equivalent'] for c in comparisons.values()):label='PERSISTENCE_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN'
    else:label='PERSISTENCE_HAS_NO_MATERIAL_EFFECT'
    return dict(classification=label,sham=sham,comparisons=comparisons,complete_K_coverage=len(comparisons)==3)
