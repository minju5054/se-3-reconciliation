"""One fixed-source, pre-next-install comparison. Pure geometry and saved metrics."""
from copy import deepcopy
import numpy as np
from .se2 import compose_poses, inverse_pose, relative_pose, se2_exp, se2_log, wrap_angle
from .spatial_entry_suffix import suffix_reference
from .boundary_row_ablation04 import staged_reference
from .b_to_entry_bridge import hermite_bridge, EPS
from .gp_se2_join01 import forward_projection
from .robotless_online import integrate_unicycle
from .isaac_clock_restore01 import prime_clock

ORDER = ['RAW', 'B_ENTRY', 'HERMITE', 'GRAPH']
PNGS = ['four_method_world_execution.png', 'transition_metrics.png']
SCALES = np.array([.10, .10, np.deg2rad(10.)])
DIRECTION_SCALE = np.deg2rad(15.)


def isolated_schedule(full, fresh_id):
    """End at first next-reference INSTALL, including its incoming state only."""
    later = [r for r in full['installs'] if r['chunk_id'] != fresh_id]
    if not later:
        raise ValueError('no authenticated next install boundary')
    stop = min(r['install_tick'] for r in later)
    start = full['start_tick']
    submits = [r for r in full['submissions'] if start <= r['submit_tick'] < stop]
    assert all(r['chunk_id'] == fresh_id and r['generation'] == full['initial_generation']
               and r['result_generation'] == full['initial_generation'] and r['result_status'] == 'command'
               and r['submit_tick'] < r['application_tick'] < stop for r in submits)
    assert not [r for r in full['installs'] if start <= r['install_tick'] < stop]
    attempted = [t for t in full['attempted_submit_ticks'] if start <= t < stop]
    assert attempted == [r['submit_tick'] for r in submits]
    pairs = [dict(submit_tick=r['submit_tick'], application_tick=r['application_tick']) for r in submits]
    assert all(a['application_tick'] < b['submit_tick'] for a,b in zip(pairs,pairs[1:]))
    return dict(start_tick=start, stop_tick=stop, integration_steps=stop-start,
        attempted_submit_ticks=attempted, accepted_submit_ticks=attempted,
        application_ticks=[r['application_tick'] for r in submits], pairs=pairs,
        excluded_next_install=deepcopy(later[0]), original_generation=full['initial_generation'],
        window_rule='[B application, next reference installation); terminal state before install')


class ProgressGraph:
    """X0=B, XM=S[-1]; only internal SE(2) poses enter right-local LM."""
    def __init__(self, P, B, suffix):
        self.P, self.B, self.S = [np.array(a, float, copy=True) for a in (P,B,suffix)]
        if self.S.ndim != 2 or self.S.shape[1] != 3 or len(self.S) < 3:
            raise ValueError('Graph requires at least one editable internal pose')
        if not all(np.isfinite(a).all() for a in [self.P,self.B,self.S]):
            raise ValueError('nonfinite graph input')
        delta = self.B[:2]-self.P[:2]
        if np.linalg.norm(delta) <= EPS:
            raise ValueError('undefined incoming OLD direction')
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.S[:,:2],axis=0),axis=1))]
        if np.any(np.diff(arc) <= EPS):
            raise ValueError('collapsed original suffix')
        self.s = arc/arc[-1]
        self.phi = np.arctan2(delta[1],delta[0])
        self.D = relative_pose(self.S[:-1],self.S[1:])
        for a in [self.P,self.B,self.S,self.s,self.D]: a.setflags(write=False)

    def unpack(self, interior):
        a = np.asarray(interior,float)
        if a.shape != (len(self.S)-2,3) or not np.isfinite(a).all():
            raise ValueError('finite internal poses required')
        return np.vstack([self.B,a,self.S[-1]])

    def pack(self, world):
        x = np.asarray(world,float)
        assert x.shape == self.S.shape and x[0].tobytes() == self.B.tobytes()
        assert x[-1].tobytes() == self.S[-1].tobytes()
        return x[1:-1].copy()

    def initial(self):
        delta = compose_poses(self.B,inverse_pose(self.S[0]))
        x = compose_poses(se2_exp((1-self.s)[:,None]*se2_log(delta)),self.S)
        x[0],x[-1] = self.B,self.S[-1]
        return x

    def factors(self, interior):
        x = self.unpack(interior)
        edge = x[1,:2]-x[0,:2]
        # Finite proposal residual; noncollapsed edges are enforced by acceptance.
        t = np.atleast_1d(wrap_angle(np.arctan2(edge[1],edge[0])-self.phi)/DIRECTION_SCALE)
        r = se2_log(relative_pose(self.D,relative_pose(x[:-1],x[1:]))) / SCALES
        a = self.s[1:-1,None]*se2_log(relative_pose(self.S[1:-1],x[1:-1])) / SCALES
        return dict(T=t,R=r.ravel(),A=a.ravel())

    def residual(self, interior):
        return np.concatenate(list(self.factors(interior).values()))

    def costs(self, interior):
        c = {k:float(v@v) for k,v in self.factors(interior).items()}
        return dict(**c,total=sum(c.values()))

    def noncollapsed(self, interior):
        return bool(np.all(np.linalg.norm(np.diff(self.unpack(interior)[:,:2],axis=0),axis=1)>EPS))


def fixed_references(F, raw, A, B, P):
    suffix, suffix_local, entry = suffix_reference(F,raw,A,B)
    # Use EXACT prior boundary-row convention, including vertex deduplication.
    w,l,labels = staged_reference(F,raw,A,B,entry,suffix)
    h,meta = hermite_bridge(P,B,suffix)
    ids = entry['row_original_identities'][1:]
    hw = np.vstack([h,np.asarray(F)[ids]])
    hl = np.vstack([relative_pose(A,h),np.asarray(raw)[ids]])
    graph = ProgressGraph(P,B,suffix)
    refs = dict(RAW=(np.array(F,copy=True),np.array(raw,copy=True),[f'F_{i}' for i in range(len(F))]),
        B_ENTRY=(w,l,labels),HERMITE=(hw,hl,['B',*[f'H_{i}' for i in range(1,len(h)-1)],'E*',*[f'F_{i}' for i in ids]]))
    return refs, suffix, entry, meta, graph


def auc(t, values, horizon):
    if len(t)<2 or t[-1]<horizon-1e-12: return None
    mask = t < horizon
    tt = np.r_[t[mask],horizon]
    vv = np.r_[values[mask],np.interp(horizon,t,values)]
    return float(np.trapezoid(vv,tt))


def metrics(rollout, F, environment, duration):
    t = np.array([r['time_s'] for r in rollout['states']])
    p = np.array([r['pose_world'] for r in rollout['states']])
    u = np.array([[r['v_mps'],r['omega_radps']] for r in rollout['commands']]).reshape(-1,2)
    proj = forward_projection(F,p)
    d = np.array([r['distance_m'] for r in proj]); yaw = np.array([r['yaw_error_rad'] for r in proj])
    dt = rollout['phase']['integration_dt_s']
    bounds=[]
    for x,v in zip(p,u):
        curve = abs(v[0]*v[1])*dt**2/8
        y=integrate_unicycle(x,v,dt)
        q=environment.check_trajectory([0.,dt],np.array([x,y]),radius=.20,required_clearance=.05,curved_path_error_bound_m=curve)
        bounds.append(q['minimum_clearance_lower_bound_m'])
    first=next((c for c in rollout['commands'] if c['application_tick']>rollout['phase']['B_tick'] and c['reason']=='new_solve'),None)
    uv=None if first is None else [first['v_mps'],first['omega_radps']]
    delta=None if uv is None else np.asarray(uv)-rollout['phase']['u_B_plus']
    tv=np.sum(abs(np.diff(u,axis=0)),axis=0) if len(u)>1 else [0.,0.]
    return dict(position_auc_03_m_s=auc(t,d,.3),position_auc_full_m_s=auc(t,d,duration),
        yaw_auc_03_rad_s=auc(t,yaw,.3),yaw_auc_full_rad_s=auc(t,yaw,duration),
        initial_position_error_m=float(d[0]),initial_yaw_error_rad=float(yaw[0]),
        first_new_command=uv,first_new_application_tick=None if first is None else first['application_tick'],
        abs_delta_v_from_u_B_plus=None if delta is None else float(abs(delta[0])),
        abs_delta_omega_from_u_B_plus=None if delta is None else float(abs(delta[1])),
        linear_TV=float(tv[0]),angular_TV=float(tv[1]),
        max_abs_v=None if not len(u) else float(abs(u[:,0]).max()),
        max_abs_omega=None if not len(u) else float(abs(u[:,1]).max()),
        swept_clearance_lower_bound_m=min(bounds) if bounds else None,
        physical_overlap=any(x<0 for x in bounds),guard_abort=rollout['abort'] is not None,
        termination=rollout['termination'],integration_steps=len(u),
        trace=dict(time_s=t.tolist(),position_error_m=d.tolist(),yaw_error_rad=yaw.tolist(),projection=proj,
                   executed_interval_clearance_lower_bounds_m=bounds))


def parity(rollout, historical, schedule, tolerances):
    """Saved-only RAW gate. No replacement of missing observations with zeros."""
    if len(rollout['states'])!=len(historical['states']) or len(rollout['commands'])!=schedule['integration_steps']:
        return dict(passed=False,reason='incomplete RAW window',pose_max_error=None,command_max_error=None)
    a=np.array([r['pose_world'] for r in rollout['states']]);b=np.array([r['pose_world'] for r in historical['states']])
    pd=a-b;pd[:,2]=wrap_angle(pd[:,2])
    u=np.array([[r['v_mps'],r['omega_radps']] for r in rollout['commands']]);v=np.asarray(historical['controls'])
    sols=[e for e in rollout['events'] if e.get('type')=='solve_result']
    memory=max((float(np.max(abs(np.asarray(e['previous_command'])-h['previous_command']))) for e,h in zip(sols,historical['solves'])),default=0.)
    check=dict(initial_B_exact=a[0].tobytes()==b[0].tobytes(),pose_max_error=float(abs(pd).max()),
        command_max_error=float(abs(u-v).max()),memory_max_error=memory,
        solve_count_equal=len(sols)==len(historical['solves']),
        attempts_equal=[r['tick'] for r in rollout['submit_requests']]==schedule['attempted_submit_ticks'],
        accepted_equal=[r['input_state_id'] for r in rollout['events'] if r.get('status')=='submitted']==schedule['accepted_submit_ticks'],
        applications_equal=[r['application_tick'] for r in rollout['commands'] if r['reason']=='new_solve' and r['application_tick']>schedule['start_tick']]==schedule['application_ticks'],
        reference_identity_equal=all(c['chunk_id']==h['chunk_id'] and int(c['reference_version'])==int(h['reference_version']) for c,h in zip(rollout['commands'],historical['commands'])),
        generations_equal=all(e['official_generation']==e['result_generation']==schedule['original_generation'] for e in sols),
        guard_decisions_equal=[g['safe'] for g in rollout['guards']]==[g['safe'] for g in historical['guards']],
        guard_clearance_max_error=max(abs(g['check']['minimum_clearance_lower_bound_m']-h['check']['minimum_clearance_lower_bound_m']) for g,h in zip(rollout['guards'],historical['guards'])),
        horizon_completed=rollout['termination']=='PRE_NEXT_INSTALL_CAP')
    numeric={'pose_max_error':tolerances['pose_atol'],'command_max_error':tolerances['command_atol'],
             'memory_max_error':tolerances['command_atol'],'guard_clearance_max_error':tolerances['pose_atol']}
    check['passed']=all(v<=numeric[k] if k in numeric else v is True for k,v in check.items())
    return check
