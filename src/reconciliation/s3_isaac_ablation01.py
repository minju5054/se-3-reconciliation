"""Frozen S3 relative-factor ablation: shared graph algebra and saved metrics."""
from copy import deepcopy
import numpy as np
from .successive_isaac_four_method01 import ProgressGraph, SCALES, auc, metrics, parity
from .successive_handoff_severity01 import turn50
from .spatial_entry_suffix import endpoint_dwell
from .se2 import relative_pose, se2_log, wrap_angle
from .isaac_clock_restore01 import prime_clock

ORDER = ['RAW', 'B_ENTRY', 'FULL_GRAPH', 'GRAPH_NO_R']
GRAPHS = ORDER[2:]
PNGS = ['s3_isaac_execution_comparison.png', 's3_response_and_deformation.png']
ATOL = 1e-9  # Existing boundary-row comparison scalar equality tolerance.


class AblationGraph(ProgressGraph):
    """Historical default algebra untouched; experiment-local R inclusion only."""
    def __init__(self, P, B, suffix, *, include_relative=True):
        super().__init__(P, B, suffix)
        self.include_relative = include_relative

    def optimized_factors(self, interior):
        factors = super().factors(interior)
        return factors if self.include_relative else {k:factors[k] for k in ['T','A']}

    def residual(self, interior):
        return np.concatenate(list(self.optimized_factors(interior).values()))

    def costs(self, interior):
        c = {k:float(v@v) for k,v in self.optimized_factors(interior).items()}
        return dict(**c, total=sum(c.values()))

    def relative_diagnostic(self, interior):
        r = super().factors(interior)['R'].reshape(-1,3)
        e = r*SCALES
        tr = np.linalg.norm(e[:,:2],axis=1)
        return dict(per_edge_log=e.tolist(), translation_RMS_m=float(np.sqrt(np.mean(tr**2))),
            translation_max_m=float(tr.max()), yaw_RMS_rad=float(np.sqrt(np.mean(e[:,2]**2))),
            yaw_max_rad=float(abs(e[:,2]).max()), normalized_squared_residual=float(r.ravel()@r.ravel()),
            optimized=self.include_relative,
            label='optimized R cost' if self.include_relative else 'diagnostic relative-edge distortion, not optimized cost')


def deformation(problem, world):
    x=np.asarray(world);tr=np.linalg.norm(x[:,:2]-problem.S[:,:2],axis=1)
    yaw=abs(wrap_angle(x[:,2]-problem.S[:,2]));edges=np.diff(x[:,:2],axis=0);length=np.linalg.norm(edges,axis=1)
    return dict(per_node_translation_m=tr.tolist(),translation_RMS_m=float(np.sqrt(np.mean(tr**2))),translation_max_m=float(tr.max()),
        per_node_yaw_correction_rad=yaw.tolist(),yaw_RMS_rad=float(np.sqrt(np.mean(yaw*yaw))),yaw_max_rad=float(yaw.max()),
        endpoint_displacement_m=float(tr[-1]),endpoint_yaw_correction_rad=float(yaw[-1]),
        first_edge_direction_rad=float(np.arctan2(edges[0,1],edges[0,0])),first_edge_length_m=float(length[0]),
        total_XY_arc_m=float(length.sum()),minimum_edge_length_m=float(length.min()),
        relative_distortion=problem.relative_diagnostic(problem.pack(x)),factor_costs=problem.costs(problem.pack(x)))


def prime_s3_clock(world, B_sim_s, dt, maximum_steps):
    """Reuse unchanged <=1000-step prime_clock in bounded pre-clock batches."""
    start=float(world.current_time);n=int(round((B_sim_s-start)/dt))
    if n<0 or n>maximum_steps or abs(start+n*dt-B_sim_s)>1e-10:
        raise ValueError('saved S3 clock is not exactly reachable within frozen initialization budget')
    records=[]
    remaining=n
    while remaining:
        count=min(1000,remaining)
        target=B_sim_s if count==remaining else float(world.current_time)+count*dt
        records.append(prime_clock(world,target,dt));remaining-=count
    if not records: records.append(prime_clock(world,B_sim_s,dt))
    assert abs(float(world.current_time)-B_sim_s)<=1e-10
    return dict(initial_clock_s=start,zero_motion_initialization_steps=n,restored_clock_s=float(world.current_time),batches=records)


def raw_parity(rollout, historical, schedule, tolerances):
    """Reuse the previous Isaac parity gate and add clock/selection/state coverage."""
    native=historical
    h=dict(states=native['states'],controls=[[c['v_mps'],c['omega_radps']] for c in native['commands']],
        commands=native['commands'],solves=[e for e in native['events'] if e.get('type')=='solve_result'],guards=native['guards'])
    adapted=dict(rollout,termination='PRE_NEXT_INSTALL_CAP' if rollout['termination']=='OBSERVATION_CAP' else rollout['termination'])
    gate=parity(adapted,h,schedule,tolerances)
    gate['state_ticks_equal']=[s['absolute_tick'] for s in rollout['states']]==[s['absolute_tick'] for s in native['states']]
    gate['clock_max_error']=None if len(rollout['states'])!=len(native['states']) else max(abs(a['sim_time_s']-b['sim_time_s']) for a,b in zip(rollout['states'],native['states']))
    gate['clock_equal']=gate['clock_max_error'] is not None and gate['clock_max_error']<=tolerances['pose_atol']
    new=[e for e in rollout['events'] if e.get('type')=='solve_result']
    gate['selection_equal']=len(new)==len(h['solves']) and all(a['selection']['indices']==b['selection']['indices'] and
        np.allclose(a['selection']['reference_world'],b['selection']['reference_world'],rtol=0,atol=tolerances['selection_atol']) for a,b in zip(new,h['solves']))
    gate['termination_equal']=rollout['termination']==native['termination']=='OBSERVATION_CAP'
    gate['initial_controller_memory_equal']=rollout['phase']==native['phase']
    gate['passed'] = gate['passed'] and all(gate[k] for k in ['clock_equal','state_ticks_equal','selection_equal','termination_equal','initial_controller_memory_equal'])
    return gate


def evaluate(rollout, fresh, env, entry, duration):
    result=metrics(rollout,fresh,env,duration)
    trace=result['trace'];t=np.asarray(trace['time_s']);p=np.array([s['pose_world'] for s in rollout['states']])
    k=entry['correspondence']['segment'];edge=fresh[k+1,:2]-fresh[k,:2];phi=float(np.arctan2(edge[1],edge[0]))
    for horizon,label in [(.3,'03'),(.9,'09'),(duration,'full')]:
        result['position_auc_'+label+'_m_s']=auc(t,np.asarray(trace['position_error_m']),horizon)
        result['yaw_auc_'+label+'_rad_s']=auc(t,np.asarray(trace['yaw_error_rad']),horizon)
        response=turn50(t,p[:,2],rollout['phase']['B'][2],phi,horizon)
        result['turn50_'+label]=response
    commands=[[c['v_mps'],c['omega_radps']] for c in rollout['commands']]
    endpoint=endpoint_dwell(t,p,np.asarray(fresh)[-1],commands)
    result.update({k:v for k,v in endpoint.items() if k!='endpoint_dwell_trace'})
    result['endpoint_error_at_3s_m']=None if t[-1]<3. else float(np.linalg.norm(p[-1,:2]-fresh[-1,:2]))
    result['endpoint_error_at_3s_definition']='historical 180-interval nominal 3 s cap'
    result['safety_abort_tick']=None if rollout['abort'] is None else rollout['abort']['tick']
    return result


def latency_difference(left, right, dt):
    a,b=left['time_s'],right['time_s']
    if 'INCOMPLETE_WINDOW' in [left['status'],right['status']]:
        return dict(delta_s=None,ordering='INCOMPLETE_WINDOW_NO_ORDER')
    if left['status']=='N/A_NEAR_ZERO_TURN' or right['status']=='N/A_NEAR_ZERO_TURN':
        return dict(delta_s=None,ordering='N/A_NEAR_ZERO_TURN')
    if a is None and b is None:return dict(delta_s=None,ordering='BOTH_CENSORED_NO_ORDER')
    if a is None:return dict(delta_s=None,ordering='RIGHT_OBSERVED_LEFT_CENSORED')
    if b is None:return dict(delta_s=None,ordering='LEFT_OBSERVED_RIGHT_CENSORED')
    delta=a-b
    return dict(delta_s=delta,ordering='LEFT_EARLIER' if delta<=-(dt-ATOL) else 'RIGHT_EARLIER' if delta>=dt-ATOL else 'SAME_OBSERVABLE_TICK')


def comparisons(values, dt):
    result={}
    for a,b in [('FULL_GRAPH','RAW'),('FULL_GRAPH','B_ENTRY'),('GRAPH_NO_R','FULL_GRAPH'),('GRAPH_NO_R','B_ENTRY')]:
        ma,mb=values.get(a),values.get(b)
        result[a+' - '+b]=None if ma is None or mb is None else dict(
            position_auc_03_m_s=None if ma['position_auc_03_m_s'] is None or mb['position_auc_03_m_s'] is None else ma['position_auc_03_m_s']-mb['position_auc_03_m_s'],
            yaw_auc_03_rad_s=None if ma['yaw_auc_03_rad_s'] is None or mb['yaw_auc_03_rad_s'] is None else ma['yaw_auc_03_rad_s']-mb['yaw_auc_03_rad_s'],
            turn50_03=latency_difference(ma['turn50_03'],mb['turn50_03'],dt),turn50_09=latency_difference(ma['turn50_09'],mb['turn50_09'],dt))
    return result


def classify(values, planning, raw_gate, dt, technical=False):
    if technical:return 'TECHNICAL_BLOCKED'
    if raw_gate is not None and not raw_gate['passed']:return 'RAW_ISAAC_PARITY_FAIL'
    if any(not p['valid'] for p in planning.values()):return 'GRAPH_PLANNING_FAILED'
    if len(values)!=4 or any(m is None for m in values.values()):return 'TECHNICAL_BLOCKED'
    def improves(a,b):
        order=latency_difference(values[a]['turn50_03'],values[b]['turn50_03'],dt)['ordering']
        x,y=values[a]['position_auc_03_m_s'],values[b]['position_auc_03_m_s']
        return order in ['LEFT_EARLIER','LEFT_OBSERVED_RIGHT_CENSORED'] and x is not None and y is not None and x<y-ATOL
    safe=all(not m['guard_abort'] and not m['physical_overlap'] for m in values.values())
    if safe and improves('GRAPH_NO_R','FULL_GRAPH'):return 'RELATIVE_FACTOR_LIMITING_EVIDENCE'
    if safe and improves('FULL_GRAPH','GRAPH_NO_R'):return 'FULL_RELATIVE_FACTOR_RESPONSE_SUPPORTED'
    if not any(improves(n,'B_ENTRY') for n in GRAPHS):return 'B_ENTRY_REMAINS_SUFFICIENT'
    # No post-hoc deformation magnitude cutoff. Conflicting yaw/safety is explicit.
    if not safe or any(improves(n,'B_ENTRY') and values[n]['yaw_auc_03_rad_s']>values['B_ENTRY']['yaw_auc_03_rad_s']+ATOL for n in GRAPHS):
        return 'RESPONSE_DEFORMATION_TRADEOFF'
    return 'MIXED_MECHANISM_EVIDENCE'
