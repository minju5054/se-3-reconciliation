"""Frozen transport-scale diagnostic; existing LocalSE2Problem is unchanged."""
from dataclasses import dataclass
from copy import deepcopy
import numpy as np
from .local_se2_reconciliation import LocalSE2Problem
from .se2 import compose_poses, se2_exp, se2_log, relative_pose
from .osa03_common_b import schedule
from .gp_se2_join01 import forward_projection

NATIVE='M0_NATIVE'
HALF='HALF_TRANSPORT_LOCAL_SE2'
FULL='FULL_LOCAL_SE2'
ORDER=[NATIVE,HALF,FULL]
ALPHAS=[0.,.5,1.]
SOURCE_IDS=['OSA03_R00','episode_001_repeat_01/handoff_013',
            'episode_008_repeat_01/handoff_023','episode_013_repeat_00/handoff_020']
PNGS=['world_execution_overview.png','transport_scale_primary_metrics.png',
      'transport_scale_deltas.png','reference_transport_overview.png']
PRIMARY_KEYS=['position_auc_09_m_s','sustained_attachment_s',
              'execution_clearance_lower_bound_m','endpoint_error_m']


@dataclass(frozen=True)
class TransportScaleProblem(LocalSE2Problem):
    alpha: float = 1.

    def __post_init__(self):
        super().__post_init__()
        if self.alpha not in ALPHAS:
            raise ValueError('only frozen alpha values 0, .5, 1 are supported')

    @property
    def scaled_transport(self):
        if self.alpha==1.:return self.transport
        return se2_exp(self.alpha*se2_log(self.transport))

    @property
    def target(self):
        if self.alpha==1.:return super().target
        if self.alpha==0.:return self.fresh.copy()
        return compose_poses(self.scaled_transport,self.fresh)


def transport_diagnostics(p):
    body=se2_log(relative_pose(p.A,p.B));world=se2_log(p.transport)
    half=se2_exp(.5*world)
    return dict(measured_A_inverse_B_log=body.tolist(),
        body_log_translation_m=float(np.linalg.norm(body[:2])),
        measured_A_to_B_pose=relative_pose(p.A,p.B).tolist(),
        world_B_A_inverse_log=world.tolist(),
        full_world_transport_pose=p.transport.tolist(),half_world_transport_pose=half.tolist(),
        full_world_transform_translation_m=float(np.linalg.norm(p.transport[:2])),
        half_world_transform_translation_m=float(np.linalg.norm(half[:2])),
        full_body_motion_translation_m=float(np.linalg.norm(relative_pose(p.A,p.B)[:2])),
        half_body_motion_translation_m=float(np.linalg.norm(se2_exp(.5*body)[:2])),
        full_yaw_shift_rad=float(world[2]),half_yaw_shift_rad=float(.5*world[2]),
        original_FRESH_B_projection=forward_projection(p.fresh,[p.B])[0],
        units='metres and radians; body Log(A^-1 B) differs from world Log(B A^-1) translation')


def schedule_gate(rollouts,frozen,common):
    checks={}
    for name in ORDER:
        r=rollouts.get(name)
        if r is None:checks[name]=dict(passed=False,available=False);continue
        a=schedule(r,54);b=schedule(r,180)
        initial=r['phase']==common and r['states'][0]['pose_world']==common['B']
        checks[name]=dict(available=True,primary=a,full=b,initial_provenance=initial,
            steps=len(r['commands']),passed=initial and len(r['commands'])==180 and
            a==frozen['primary'] and b==frozen['full'])
    return dict(passed=all(x['passed'] for x in checks.values()),methods=checks,
                rule='identical authenticated source schedule, 54 and 180 intervals, B/held command/memory/generation')


def selectors(rollouts,primary):
    rows={}
    for name in ORDER:
        r=rollouts.get(name)
        rows[name]=[] if r is None else [dict(submit_tick=e['input_state_id'],
            input_pose=deepcopy(e['input_pose']),nearest_row=e['selection']['nearest_index'],
            selected_H5_rows=deepcopy(e['selection']['indices']),command=deepcopy(e['command']),
            application_tick=e.get('continuation_seen',{}).get('tick'),
            withheld=bool(e.get('withheld_by_logical_scheduler',False)))
            for e in r['events'] if e.get('type')=='solve_result' and e.get('status')=='command']
    comparisons={};half={r['submit_tick']:r for r in rows[HALF]}
    for name in [NATIVE,FULL]:
        other={r['submit_tick']:r for r in rows[name]}
        ticks=sorted(t for t in half.keys()&other.keys()
                     if half[t]['selected_H5_rows']!=other[t]['selected_H5_rows'])
        a=primary.get(HALF);b=primary.get(name)
        changed=None if a is None or b is None else a['sustained_attachment_s']!=b['sustained_attachment_s']
        comparisons[name]=dict(first_differing_submit_tick=min(ticks,default=None),
            differing_submit_ticks=ticks,attachment_changed=changed,
            selector_difference_and_attachment_change_observed=bool(ticks) and changed is True)
    return dict(methods=rows,comparisons=comparisons,diagnostic_only=True,
                interpretation='co-occurrence only, not selector causality')


def cross_source(sources,tol=1e-9):
    deltas={};better=[];worse=[];native_best=[];safe=[];attachment={}
    for sid,q in sources.items():
        n,h,f=[q['primary_metrics'][x] for x in ORDER]
        deltas[sid]={k:None if h is None or f is None or h[k] is None or f[k] is None else h[k]-f[k]
                     for k in PRIMARY_KEYS}
        d=deltas[sid]['position_auc_09_m_s']
        better.append(d is not None and d < -tol);worse.append(d is not None and d > tol)
        native_best.append(all(x is not None for x in [n,h,f]) and n['position_auc_09_m_s']<min(h['position_auc_09_m_s'],f['position_auc_09_m_s'])-tol)
        safe.append(all(q['reference_safety'][x]['clearance_valid'] and q['primary_metrics'][x] is not None and
                        q['primary_metrics'][x]['execution_clearance_lower_bound_m']>=.05 and
                        q['primary_metrics'][x]['termination_reason']=='OBSERVATION_CAP' for x in ORDER))
        attachment[sid]={x:None if q['primary_metrics'][x] is None else q['primary_metrics'][x]['sustained_attachment_s'] for x in ORDER}
    technical=any(not q['schedule_gate']['passed'] or any(t in ['CONTROLLER_ERROR','HOLD_TIMEOUT'] for t in q['termination'].values()) for q in sources.values())
    # Rules are ordered before execution. Count directions; never construct a winner score.
    if technical:classification='TECHNICAL_BLOCKED'
    elif any(better) and any(worse):classification='SOURCE_DEPENDENT_TRANSPORT'
    elif not all(better[1:]) or any(native_best[1:]) or not all(safe):classification='TRANSPORT_MAGNITUDE_INSUFFICIENT'
    else:classification='FULL_TRANSPORT_OVERCOMPENSATES'
    def median(key):
        values=[r[key] for r in deltas.values() if r[key] is not None]
        return None if not values else float(np.median(values))
    return dict(classification=classification,deltas=deltas,half_better_Full_AUC_count=sum(better),
        half_worse_Full_AUC_count=sum(worse),native_better_both_AUC_count=sum(native_best),
        safety_preserved_count=sum(safe),attachment=attachment,
        median_Half_minus_Full_AUC=median('position_auc_09_m_s'),
        median_Half_minus_Full_endpoint=median('endpoint_error_m'),
        H1_safe_AUC_improvement_sources=[sid for sid,b,s in zip(sources,better,safe) if b and s],
        sign_tolerance=tol)
