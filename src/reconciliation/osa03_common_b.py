"""Frozen common-B spatial references and saved diagnostics, without optimization."""
from copy import deepcopy
from pathlib import Path
import hashlib
import numpy as np
from .se2 import compose_poses, inverse_pose, relative_pose, se2_exp, se2_log, wrap_angle
from .trajectory import validate_se2_trajectory
from .local_se2_reconciliation import rigid_fit_diagnostic
from .robotless_online import CommandActivation

ORDER = ['M0_NATIVE','M1_SE2_TAPER','M2_RIGID_TRANSPORT','M3_LOCAL_SE2']
M3_HASH = 'bd459b0343fac1e78e4ac80831f4f99af9b86dd4876a3b0df1b45f41c21ce120'


def authenticated_m3(path, expected=M3_HASH):
    p=Path(path)
    if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
        raise ValueError('frozen M3 hash mismatch; no re-optimization or fallback')
    return validate_se2_trajectory(np.load(p,allow_pickle=False))


def references(A,B,fresh,m3):
    f=validate_se2_trajectory(fresh);m=validate_se2_trajectory(m3)
    if len(f)<2 or f.shape!=m.shape:raise ValueError('same original generic N>=2 required')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(f[:,:2],axis=0),axis=1))]
    if arc[-1]<=0:raise ValueError('positive original XY arc required')
    s=arc/arc[-1];g=compose_poses(B,inverse_pose(A));xi=se2_log(g)
    return dict(zip(ORDER,[f.copy(),compose_poses(se2_exp((1-s)[:,None]*xi),f),compose_poses(g,f),m.copy()]))


def distortion(fresh,reference):
    f=validate_se2_trajectory(fresh);x=validate_se2_trajectory(reference)
    if f.shape!=x.shape:raise ValueError('no crop/resampling allowed')
    corr=se2_log(relative_pose(f,x));edges=se2_log(relative_pose(relative_pose(f[:-1],f[1:]),relative_pose(x[:-1],x[1:])))
    dis=np.linalg.norm(x[:,:2]-f[:,:2],axis=1);et=np.linalg.norm(edges[:,:2],axis=1);ey=abs(edges[:,2])
    return dict(first_node_displacement_m=float(dis[0]),endpoint_displacement_m=float(dis[-1]),
        per_node_displacement_m=dis.tolist(),per_node_log_correction=corr.tolist(),relative_edge_log=edges.tolist(),
        relative_translation_RMS_m=float(np.sqrt(np.mean(et**2))),relative_yaw_RMS_rad=float(np.sqrt(np.mean(ey**2))),
        relative_translation_max_m=float(et.max()),relative_yaw_max_rad=float(ey.max()),rigid_fit=rigid_fit_diagnostic(f,x))


def common_state(phase):
    first=phase['first_FRESH_solve'];app=phase['applications'][0]
    assert app['application_tick']==phase['B_tick'] and app['result']['solve_id']==first['solve_id']
    assert phase['next_submit_after_B']>phase['B_tick'] and phase['next_submit_after_B']%6==0
    np.testing.assert_array_equal(first['command'],phase['u_B_plus'])
    # Physical incoming command and advanced worker memory are separate fields.
    return {k:deepcopy(phase[k]) for k in ['B','B_tick','B_sim_s','u_minus','u_B_plus','u_mem_B',
        'delta_u_B','previous_command_application_sim_s','fresh_capture_pose','fresh_chunk_id','fresh_version',
        'original_generation','integration_dt_s','next_submit_after_B','first_FRESH_solve']}


def activation_at_B(common):
    """Restore the application already occurring at B, no new installation event."""
    a=CommandActivation(.5)
    a.installed=dict(chunk_id=common['fresh_chunk_id'],reference_version=common['fresh_version'],context={})
    a.active=a.installed
    assert a.accept(deepcopy(common['first_FRESH_solve']),common['B_sim_s'])
    return a


def may_execute(reference_check):
    return bool(reference_check['clearance_valid'])


def schedule(rollout, primary_intervals=54):
    if rollout is None:return None
    end=rollout['phase']['B_tick']+primary_intervals
    events=rollout['events'];solved={e['solve_id']:e for e in events if e.get('type')=='solve_result'}
    applications=[]
    for c in rollout['commands']:
        if c['application_tick']>end or c['reason']!='new_solve':continue
        if c['solve_id']==rollout['phase']['first_FRESH_solve']['solve_id']:continue
        e=solved[c['solve_id']]
        applications.append(dict(submit_tick=e['input_state_id'],application_tick=c['application_tick']))
    return dict(attempted_submit_ticks=[r['tick'] for r in rollout['submit_requests'] if r['tick']<=end],
                accepted_submit_ticks=[e['input_state_id'] for e in events if e.get('status')=='submitted' and e['input_state_id']<=end],
                applications=applications,complete_primary_exposure=len(rollout['commands'])>=primary_intervals,
                endpoint_inclusive_absolute_tick=end)


def timing_audit(rollouts):
    rows={name:schedule(r) for name,r in rollouts.items()};valid=[v for v in rows.values() if v is not None]
    complete=bool(valid) and all(v['complete_primary_exposure'] for v in valid)
    same=complete and all(v==valid[0] for v in valid[1:])
    return dict(comparable=bool(same),complete_primary_exposure=complete,schedules=rows,
                rule='same attempted/accepted submit ticks and corresponding successful application ticks through inclusive B+54')


def metric_row(metric):
    if metric is None:return None
    w=metric['windows']['54'];full=metric['full'];trace=metric['trace'];j=None
    if full['join_time_s'] is not None:j=int(np.searchsorted(trace['times_s'],full['join_time_s']))
    return dict(position_auc_09_m_s=w.get('position_auc_m_s'),yaw_auc_09_rad_s=w.get('yaw_auc_rad_s'),
                sustained_attachment_s=full['join_time_s'],
                attachment_fractional_row=None if j is None else trace['projection'][j]['progress'],
                remaining_arc_at_attachment_m=None if j is None else float(trace['remaining_arc_m'][j]),
                execution_clearance_lower_bound_m=metric['clearance']['execution']['minimum_clearance_lower_bound_m'])


def signed_gaps(rows):
    native=rows.get('M0_NATIVE')
    return {m:None if row is None or native is None else
            {k:None if row[k] is None or native[k] is None else float(row[k]-native[k]) for k in native}
            for m,row in rows.items()}


def classify(references_safe,rollouts,timing,validated=True):
    flags=[]
    if any(not s for s in references_safe.values()):flags.append('REFERENCE_LIMITED_COMMON_B_COMPARISON')
    technical=not validated or any(references_safe[m] and (r is None or r['termination'] in ['CONTROLLER_ERROR','HOLD_TIMEOUT']) for m,r in rollouts.items())
    if technical:return dict(primary='TECHNICAL_EXECUTION_BLOCKED',flags=flags)
    if not timing['comparable']:
        return dict(primary='TIMING_CONFOUNDED_COMMON_B_COMPARISON',flags=flags+['TIMING_CONFOUNDED_COMMON_B_COMPARISON'])
    return dict(primary=flags[0] if flags else 'COMMON_B_COMPARISON_VALID',flags=flags)
