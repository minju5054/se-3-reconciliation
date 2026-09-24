"""Static-occlusion source protocol; no model, controller or planner implementation."""
import numpy as np
from .exp02d_turning_search import path_turning
from .obstacle_source_acquisition import future_at_pose, timing_gate
from .join_source02 import response_geometry

INSTRUCTION='Turn left at the corner and continue down the hallway. Avoid obstacles and keep going.'

class FirstCrossing:
    """Latch the first scheduled visibility crossing; never substitute a later frame."""
    def __init__(self, threshold=20):
        self.threshold=threshold;self.previous=None;self.crossing=None;self.ids=[]
    def observe(self, frame_id, pixels, state_id):
        if frame_id in self.ids or (self.previous and state_id<=self.previous['state_id']):
            raise ValueError('duplicate/nonchronological frame')
        self.ids.append(frame_id)
        row=dict(frame_id=frame_id,pixels=int(pixels),state_id=int(state_id))
        if self.crossing is None and self.previous and self.previous['pixels']<self.threshold<=pixels:
            self.crossing=dict(previous=self.previous,current=row)
        self.previous=row
        return self.crossing
    def allows(self, frame_id):
        return self.crossing is not None and frame_id==self.crossing['current']['frame_id']


def ray_first_hit(origin, target, triangles):
    """Evaluator-only finite segment / exact exported mesh intersection."""
    o=np.asarray(origin,float);end=np.asarray(target,float);t=np.asarray(triangles,float)
    d=end-o;e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0]
    p=np.cross(np.broadcast_to(d,e2.shape),e2);det=np.einsum('ij,ij->i',e1,p)
    good=np.abs(det)>1e-12;inv=np.zeros(len(t));inv[good]=1/det[good]
    s=o-t[:,0];u=np.einsum('ij,ij->i',s,p)*inv;q=np.cross(s,e1)
    v=q@d*inv;alpha=np.einsum('ij,ij->i',e2,q)*inv
    valid=good&(u>=-1e-10)&(v>=-1e-10)&(u+v<=1+1e-10)&(alpha>1e-8)&(alpha<1-1e-8)
    if not valid.any():return None
    i=int(np.argmin(np.where(valid,alpha,np.inf)))
    return dict(triangle_index=i,alpha=float(alpha[i]),point_world=(o+alpha[i]*d).tolist(),distance_m=float(np.linalg.norm(d)*alpha[i]))


def project_camera(point,camera):
    T=np.array(camera['T_world_camera']);p=np.linalg.inv(T)@np.r_[point,1.]
    K=np.array(camera['actual_intrinsics']['K']);q=K@np.array([p[0],-p[1],-p[2]])
    if q[2]<=0:return dict(in_frustum=False,pixel=None)
    uv=q[:2]/q[2];w,h=camera['actual_intrinsics']['resolution_width_height']
    return dict(in_frustum=bool(0<=uv[0]<w and 0<=uv[1]<h),pixel=uv.tolist())


def left_corner(incoming,outgoing):
    a=np.array(incoming,float);b=np.array(outgoing,float)
    return bool(np.isclose(np.dot(a,b),0) and a[0]*b[1]-a[1]*b[0]>0)


def old_turning(raw):
    r=path_turning(np.asarray(raw,float),.02)
    r['qualified']=r['arc_m']>=.5 and r['yaw_excursion_deg']>=30 and r['tangent_excursion_deg']>=30
    return r


def motion_gate(u_minus,travel,clearance):
    return bool(u_minus[0]>.20 and abs(u_minus[1])>=.10 and travel>=.02 and clearance>=.05)


def mismatch(old,fresh,scenario):
    influence=dict(origin_xy=scenario['center_xy'],forward_xy=scenario['forward_xy'],
        progress_min_m=scenario['cart_extents'][0]-.5,progress_max_m=scenario['cart_extents'][1]+.5)
    return response_geometry(old,fresh,influence,minimum_separation_m=.10,minimum_angle_deg=20,minimum_chord_m=.02)


def turn_descriptor(old,fresh):
    def info(a):
        a=np.asarray(a);d=np.diff(a[:,:2],axis=0);d=d[np.linalg.norm(d,axis=1)>=.02]
        theta=np.unwrap(np.arctan2(d[:,1],d[:,0]))
        net=float(theta[-1]-theta[0]) if len(theta)>1 else None
        arc=float(np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1).sum())
        return dict(net_tangent_turn_rad=net,arc_m=arc,arc_over_abs_turn_m=None if net is None or abs(net)<1e-8 else arc/abs(net))
    a,b=info(old),info(fresh);v=b['net_tangent_turn_rad']
    if v is None:label='UNAVAILABLE'
    elif abs(v)<np.deg2rad(20):label='STRAIGHTENS'
    elif v<0:label='REVERSES_RIGHT'
    elif a['arc_over_abs_turn_m'] is None:label='CONTINUES_LEFT_RADIUS_UNRESOLVED'
    else:label='CONTINUES_LEFT_LARGER_RADIUS_PROXY' if b['arc_over_abs_turn_m']>a['arc_over_abs_turn_m'] else 'CONTINUES_LEFT_SMALLER_RADIUS_PROXY'
    return dict(label=label,OLD=a,FRESH=b,note='arc/net tangent-turn is descriptive, not fitted curvature or intrinsic waypoint timing')


def select_representative(rows,order):
    if [r['candidate_id'] for r in rows]!=order:raise ValueError('frozen bank/order mismatch')
    return next((r['candidate_id'] for r in rows if r['qualified']),None)
