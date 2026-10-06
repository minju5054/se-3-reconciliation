"""Frozen B wrappers, command diagnostics and decision rules; no planning solves."""
from contextlib import contextmanager
import sys
import numpy as np
from .se2 import relative_pose, local_trajectory_to_world
from .boundary_row_ablation04 import identity_mapping, selector_exposure

RAW='B_FULL_RAW'
ENTRY='B_ENTRY_STAGE'
CANONICAL='B_CANONICAL'
CENTRAL=[RAW,ENTRY,CANONICAL]
NEW=[RAW,CANONICAL]
CONTEXT=['C3_ENTRY_SUFFIX','HERMITE_BRIDGE','V2_GRAPH_BRIDGE']
SOURCE_IDS=['episode_009_repeat_00/handoff_001','episode_017_repeat_01/handoff_008',
            'episode_007_repeat_01/handoff_001','episode_016_repeat_00/handoff_008']
PNGS=['reference_geometry.png','execution_comparison.png','transition_control_behavior.png']
ATTACH='sustained_attachment_s'
END='original_FRESH_endpoint_dwell_s'
AUC='position_auc_09_m_s'


@contextmanager
def execution_guard(saved_only=False):
    """Forbid all graph solves/acquisition; saved validation also forbids MPC calls."""
    old=sys.getprofile()
    def profile(frame,event,arg):
        if event!='call':return
        name=frame.f_code.co_name;module=frame.f_globals.get('__name__','')
        forbidden=(name in {'solve_least_squares','solve_graph','solve_variant','solve_half','solve_condition',
                            'infer','inference','capture_rgb','collect_source','scan'}
            or module.startswith(('isaacsim','omni.','lightnav','torch','casadi'))
            or (saved_only and (name in {'run_method','preflight','_execute_child','Popen','solve_mpc','_solve'}
                or ('mpc' in module.lower() and name in {'solve','step','submit','start'}))))
        if forbidden:raise RuntimeError('canonical execution guard: '+module+'.'+name)
    sys.setprofile(profile)
    try:yield
    finally:sys.setprofile(old)


def boundary_wrapper(A,B,world,local,prefix):
    w,l=np.asarray(world),np.asarray(local)
    if w.shape!=l.shape or w.ndim!=2 or w.shape[1]!=3 or len(w)<2 or not np.isfinite(w).all() or not np.isfinite(l).all():
        raise ValueError('finite matching N>=2 world and original-A-local arrays required')
    b=np.asarray(B,dtype=float)
    if b.shape!=(3,) or not np.isfinite(b).all():raise ValueError('finite B pose required')
    if prefix not in ['F','X']:raise ValueError('F or X counterpart identities required')
    rw=np.vstack([b,w]);rl=np.vstack([relative_pose(A,b),l])
    np.testing.assert_allclose(local_trajectory_to_world(A,rl),rw,atol=1e-12,rtol=0)
    assert rw[0].tobytes()==b.tobytes() and rw[1:].tobytes()==w.tobytes() and rl[1:].tobytes()==l.tobytes()
    return rw,rl,['B',*[f'{prefix}_{i}' for i in range(len(w))]]


def identities(labels,entry,original):
    rows=identity_mapping([('F_'+s[2:]) if s.startswith('X_') else s for s in labels],entry,original)
    for r,label in zip(rows,labels):r['label']=label
    return rows


def selectors(rollout,mapping):
    result=selector_exposure(rollout,mapping)
    for r in result['rows']:
        originals=[x for x in r['H5_original_metadata'] if x['original_arc_m'] is not None]
        r['first_original_identity']=originals[0] if originals else None
    result['first']=result['rows'][0] if result['rows'] else None
    first=next((r for r in result['rows'] if r['first_H5_identity'].startswith(('F_','X_'))),None)
    result['first_original_H5_submit_tick']=None if first is None else first['submit_tick']
    result['identity_scope']='B none; X_j retains F_j identity and ORIGINAL arc; E* frozen fractional identity'
    return result


def command_metrics(r):
    c=r['phase'];successful={e['solve_id'] for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command'}
    first=next((u for u in r['commands'] if u.get('solve_id') in successful),None)
    value=None if first is None else np.array([first['v_mps'],first['omega_radps']])
    early=[u for u in r['commands'] if (u['application_tick']-c['B_tick'])*c['integration_dt_s']<.3]
    u=np.array([[x['v_mps'],x['omega_radps']] for x in early]).reshape(-1,2)
    complete=len(r['commands'])*c['integration_dt_s']>=.3
    tv=None if not len(u) else np.abs(np.diff(u,axis=0)).sum(axis=0).tolist()
    return dict(first_new_applied_command=None if value is None else value.tolist(),
        first_new_application_tick=None if first is None else first['application_tick'],
        delta_from_physical_u_minus=None if value is None else (value-c['u_minus']).tolist(),
        delta_from_previous_control=None if value is None else (value-c['u_mem_B']).tolist(),
        physical_u_minus=c['u_minus'],previous_control=c['u_mem_B'],already_applied_u_B_plus=c['u_B_plus'],
        early_03_complete=complete,early_03_max_abs_v_mps=float(abs(u[:,0]).max()) if complete else None,
        early_03_max_abs_omega_radps=float(abs(u[:,1]).max()) if complete else None,
        early_03_linear_TV=tv[0] if complete else None,early_03_angular_TV=tv[1] if complete else None,
        observed_early_prefix_TV=tv,
        convention='held command intervals starting in [0,.3); TV between held interval commands, includes initial u_B_plus, excludes u_minus-to-u_B_plus jump; incomplete window null')


def delta(a,b,key):
    return None if a is None or b is None or a.get(key) is None or b.get(key) is None else a[key]-b[key]


def source_outcome(q,rules):
    b,x=q['methods'][ENTRY],q['methods'][CANONICAL];bm,xm=b['metrics'],x['metrics']
    dt=q['integration_dt_s'];tol=rules['scalar_atol'];tick=dt-tol
    def both(m):return m is not None and m[ATTACH] is not None and m[END] is not None
    safe=x['valid'];bb=both(bm) and b['valid'];xb=both(xm) and safe
    da,de,dp=delta(xm,bm,ATTACH),delta(xm,bm,END),delta(xm,bm,AUC)
    endpoint_gate=xb and (bm is None or bm[END] is None or de<=dt+tol)
    meaningful_auc=(dp is not None and dp<=-rules['early_auc_absolute_m_s'] and bm[AUC]>0
                    and -dp/bm[AUC]>=rules['early_auc_relative'])
    zero=(bm is not None and xm is not None and bm[ATTACH]==0 and xm[ATTACH]==0)
    timing=bb and xb and not zero and da<=-tick
    recovery=xb and not bb
    early=bb and xb and zero and meaningful_auc
    compensating=recovery or timing or early
    safety_regression=b['valid'] and (x['unsafe_reference'] or x['safety_abort'])
    loses_attach=bm is not None and bm[ATTACH] is not None and (xm is None or xm[ATTACH] is None)
    loses_endpoint=bm is not None and bm[END] is not None and (xm is None or xm[END] is None)
    later=de is not None and de>dt+tol
    regression=b['valid'] and (safety_regression or loses_attach or loses_endpoint or (later and not compensating))
    command_worse=[]
    for key in ['linear_command_TV','angular_command_TV']:
        gap=delta(xm,bm,key)
        if gap is not None and gap>=rules['command_TV_absolute'] and gap>=rules['command_TV_relative']*bm[key]:command_worse.append(key)
    transition=compensating or meaningful_auc
    adverse=bool(later or loses_attach or loses_endpoint or safety_regression or command_worse)
    positive=('CANONICAL_RECOVERY_GAIN' if recovery else 'CANONICAL_TIMING_GAIN' if timing else
              'CANONICAL_EARLY_TRACKING_GAIN' if early else None) if endpoint_gate else None
    return dict(positive=positive,regression=bool(regression),B_ENTRY_both=bb,canonical_both=xb,
        endpoint_preserved=bool(endpoint_gate),safety_regression=bool(safety_regression),
        unsafe_canonical_reference=x['unsafe_reference'],meaningful_early_AUC=bool(meaningful_auc),
        transition_improved=bool(transition),repeated_tradeoff_candidate=bool(transition and adverse),
        endpoint_later_more_than_tick=bool(later),attachment_lost=bool(loses_attach),endpoint_lost=bool(loses_endpoint),
        command_TV_worse=command_worse,attachment_delta_s=da,endpoint_delta_s=de,AUC09_delta_m_s=dp)


def classify(sources,rules):
    rows={s:source_outcome(q,rules) for s,q in sources.items()}
    positives=sum(v['positive'] is not None for v in rows.values());regressions=sum(v['regression'] for v in rows.values())
    tradeoffs=sum(v['repeated_tradeoff_candidate'] for v in rows.values())
    if not sources or any(not q['technical_valid'] for q in sources.values()):category='TECHNICAL_BLOCKED'
    elif sum(v['unsafe_canonical_reference'] for v in rows.values())>=2:category='CANONICAL_INTERFACE_NOT_EXECUTABLE'
    elif positives>=2 and not any(v['safety_regression'] for v in rows.values()):category='CANONICAL_EXECUTION_GAIN_SUPPORTED'
    elif tradeoffs>=2:category='CANONICAL_EXECUTION_TRADEOFF'
    elif regressions>=2:category='CANONICAL_EXECUTION_REGRESSION'
    elif all(v['B_ENTRY_both'] for v in rows.values()) and positives<2:category='STAGING_REMAINS_SUFFICIENT'
    else:category='MIXED_EXECUTION_EVIDENCE'
    return dict(classification=category,positive_sources=positives,regression_sources=regressions,
                tradeoff_sources=tradeoffs,sources=rows)
