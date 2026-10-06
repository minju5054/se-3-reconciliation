"""Meaningful direct-transition selection and frozen graph-necessity decisions.

Selection accepts a closed, geometry-only record type. Execution outcomes never
enter that type. Synthetic classification examples are not scientific evidence.
"""
from dataclasses import dataclass
import math
import numpy as np
from .se2 import wrap_angle

METHODS = ['C3_ENTRY_SUFFIX', 'B_ENTRY_STAGE', 'HERMITE_BRIDGE', 'V2_GRAPH_BRIDGE']
ATTACH = 'sustained_attachment_s'
ENDPOINT = 'original_FRESH_endpoint_dwell_s'
ATOL = 1e-9
EPS = 1e-12
PNGS = ['direct_transition_geometry.png', 'execution_outcome.png', 'severity_vs_method_difference.png']


@dataclass(frozen=True, slots=True)
class SelectionRecord:
    case_id: str
    episode_id: str
    eligibility: bool
    delta_phi_rad: float | None
    rho_jump: float | None
    B_to_entry_chord_m: float | None
    d_F_m: float | None
    remaining_arc_m: float


def severity(P, B, suffix):
    P, B, suffix = map(lambda a: np.asarray(a, float), (P, B, suffix))
    if P.shape != (3,) or B.shape != (3,) or suffix.ndim != 2 or suffix.shape[1] != 3 or len(suffix) < 2:
        raise ValueError('finite P/B and at least two suffix poses required')
    if not all(np.isfinite(a).all() for a in (P, B, suffix)):
        raise ValueError('nonfinite spatial geometry')
    incoming, chord = B[:2]-P[:2], suffix[0,:2]-B[:2]
    lengths = np.linalg.norm(np.diff(suffix[:,:2], axis=0), axis=1)
    if min(np.linalg.norm(incoming), np.linalg.norm(chord)) <= EPS:
        raise ValueError('undefined incoming or B-to-entry direction')
    if np.any(lengths <= EPS):
        raise ValueError('degenerate retained suffix spacing')
    d_F = float(np.median(lengths))
    rho = float(np.linalg.norm(chord)/d_F)
    delta = float(abs(wrap_angle(math.atan2(chord[1],chord[0])-math.atan2(incoming[1],incoming[0]))))
    return dict(delta_phi_rad=delta, rho_jump=rho,
                d_F_m=d_F, B_to_entry_chord_m=float(np.linalg.norm(chord)))


def meaningful_transition(features, config):
    return bool(features is not None
        and features['B_to_entry_chord_m'] >= config['minimum_B_to_entry_chord_m']
        and features['rho_jump'] >= config['minimum_rho_jump'])


def select(records, target=6, minimum=4):
    if target != 6 or minimum != 4:
        raise ValueError('frozen target/minimum are six/four')
    if not all(type(r) is SelectionRecord for r in records):
        raise TypeError('selection requires closed geometry-only SelectionRecord')
    eligible = [r for r in records if r.eligibility]
    if len({r.case_id for r in records}) != len(records):
        raise ValueError('duplicate case identity')
    chosen, episodes, trace = [], set(), []
    for axis in ('delta_phi_rad', 'rho_jump'):
        ranked = sorted(eligible, key=lambda r:(-getattr(r,axis), r.case_id))
        n = 0
        for r in ranked:
            if r.episode_id in episodes:
                continue
            chosen.append(r.case_id); episodes.add(r.episode_id); n += 1
            trace.append(dict(case_id=r.case_id, axis=axis, value=getattr(r,axis), rank_in_axis=n))
            if n == 3:
                break
    return dict(selected_ids=chosen, selected_count=len(chosen), selection_trace=trace,
        eligible_candidates=len(eligible), eligible_episodes=len({r.episode_id for r in eligible}),
        status='READY_FOR_FREEZE' if len(chosen)>=minimum else 'INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES',
        outcome_fields_used=False, episode_unit='exact recorded episode_id, including repeat')


def source_decision(q):
    methods=q['methods']; b,h,v=[methods[m] for m in METHODS[1:]]
    def observed(r,k): return r['valid'] and r['metrics'] is not None and r['metrics'][k] is not None
    def both(r): return observed(r,ATTACH) and observed(r,ENDPOINT)
    all_both=all(both(r) for r in (b,h,v)); tick=q['integration_dt_s']-ATOL
    strong_recovery=both(v) and not both(b) and not both(h)
    strong_timing=all_both and all(v['metrics'][ATTACH] <= r['metrics'][ATTACH]-tick
        and v['metrics'][ENDPOINT] <= r['metrics'][ENDPOINT]+ATOL for r in (b,h))
    shared=all(r['safety_abort'] for r in methods.values()) and len(methods)==4
    regression=dict(
        unsafe_reference=not shared and v['unsafe_reference'] and all(r['planning_valid'] for r in (b,h)),
        safety_abort=not shared and v['safety_abort'] and all(not r['safety_abort'] for r in (b,h)),
        loses_endpoint=not shared and all(observed(r,ENDPOINT) for r in (b,h)) and not observed(v,ENDPOINT),
        loses_attachment=not shared and all(observed(r,ATTACH) for r in (b,h)) and not observed(v,ATTACH))
    positive=bool(v['planning_valid'] and both(v) and (strong_recovery or strong_timing))
    intermediate=not both(b) and both(h) and both(v)
    return dict(shared_execution_failure=shared,
        shared_failure_status='SHARED_EXECUTION_FEASIBILITY_FAILURE' if shared else None,
        BOTH_DWELLS={m:both(r) for m,r in methods.items()}, GRAPH_SPECIFIC_POSITIVE=positive,
        INTERMEDIATE_GEOMETRY_POSITIVE=intermediate, strong_recovery=strong_recovery, strong_timing_gain=strong_timing,
        V2_regression=any(regression.values()), V2_regression_reasons=regression,
        V2_nonconvergence=v['nonconvergence'], V2_invalid_reference=not v['planning_valid'],
        simpler_valid=all(r['valid'] for r in (b,h)))


def classify(sources, selected_count, technical_valid=True):
    decisions={sid:source_decision(q) for sid,q in sources.items()}
    graph=sum(d['GRAPH_SPECIFIC_POSITIVE'] for d in decisions.values())
    intermediate=sum(d['INTERMEDIATE_GEOMETRY_POSITIVE'] for d in decisions.values())
    invalid=sum((d['V2_invalid_reference'] or d['V2_nonconvergence']) and d['simpler_valid'] for d in decisions.values())
    evaluable={s:d for s,d in decisions.items() if not d['shared_execution_failure']}
    b_success=sum(d['BOTH_DWELLS'][METHODS[1]] for d in decisions.values())
    safety_failure=any(d['V2_regression_reasons']['unsafe_reference'] or d['V2_regression_reasons']['safety_abort'] for d in decisions.values())
    loses_endpoint=any(d['V2_regression_reasons']['loses_endpoint'] for d in decisions.values())
    if not technical_valid or any(not q['technical_valid'] for q in sources.values()):
        label='TECHNICAL_BLOCKED'
    elif selected_count < 4:
        label='INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES'
    elif len(sources) != selected_count:
        label='TECHNICAL_BLOCKED'
    elif graph >= 2 and not safety_failure and not loses_endpoint:
        label='GRAPH_SPECIFIC_GAIN_SUPPORTED'
    elif intermediate >= 2:
        label='INTERMEDIATE_GEOMETRY_NEEDED'
    elif evaluable and graph == 0 and all(d['BOTH_DWELLS'][METHODS[1]] for d in evaluable.values()):
        label='STAGING_HARD_TRANSITION_SUPPORTED'
    elif invalid >= 2:
        label='GRAPH_FORMULATION_NOT_ROBUST'
    else:
        label='MIXED_EVIDENCE'
    return dict(classification=label, sources=decisions, selected_source_count=selected_count,
        shared_execution_failure_count=len(decisions)-len(evaluable),evaluable_source_count=len(evaluable),
        B_ENTRY_BOTH_DWELLS_count=b_success,
        B_ENTRY_success_selected=dict(numerator=b_success,denominator=selected_count),
        B_ENTRY_success_evaluable=dict(numerator=b_success,denominator=len(evaluable)), graph_specific_positive_count=graph,
        intermediate_geometry_positive_count=intermediate, V2_regression_count=sum(d['V2_regression'] for d in decisions.values()),
        V2_nonconvergence_count=sum(d['V2_nonconvergence'] for d in decisions.values()),
        V2_invalid_reference_count=sum(d['V2_invalid_reference'] for d in decisions.values()))


def application_accounting(rollout):
    """Count new results actually carried by applied intervals, not just releases."""
    results=[e for e in rollout['events'] if e.get('type')=='solve_result']
    successful={e['solve_id']:e for e in results if e.get('status')=='command'}
    released={i for i,e in successful.items() if 'continuation_seen' in e}
    applied={c.get('solve_id') for c in rollout['commands']} & set(successful)
    withheld={i for i,e in successful.items() if e.get('withheld_by_logical_scheduler')}
    blocked=released-applied
    assert applied <= released and not withheld & released
    assert set(successful)==released|withheld
    if blocked:
        assert rollout['termination']=='SAFETY_ABORT_BEFORE_UNSAFE_COMMAND'
        assert not rollout['safety_abort']['command_applied']
        assert blocked=={rollout['safety_abort']['proposed_command']['solve_id']}
    return dict(MPC_solves=len(results),MPC_logical_result_releases=len(released),
        MPC_physical_applications=len(applied),withheld_results=len(withheld),
        guard_blocked_applications=len(blocked),executed_intervals=len(rollout['commands']),
        controller_numerical_failures=len(results)-len(successful))


def schedule_prefix_audit(rollout, frozen):
    """Authenticate every attempted/accepted tick, including an abort tick.

    The guard can censor the rollout before a released result's first physical
    interval. Thus releases and applied commands have separate prefix checks.
    """
    stop=rollout['states'][-1]['absolute_tick']
    inclusive=rollout['termination']!='OBSERVATION_CAP'
    pairs=frozen['pairs']
    expected=[p['submit_tick'] for p in pairs if p['submit_tick']<stop or inclusive and p['submit_tick']==stop]
    assert [r['tick'] for r in rollout['submit_requests']]==expected
    assert [e['input_state_id'] for e in rollout['events'] if e.get('status')=='submitted']==expected
    successful={e['solve_id']:e for e in rollout['events'] if e.get('type')=='solve_result' and e.get('status')=='command'}
    release={p['submit_tick']:p['application_tick'] for p in pairs}
    for e in successful.values():
        tick=release[e['input_state_id']]
        if 'continuation_seen' in e:assert e['continuation_seen']['tick']==tick and (tick<stop or inclusive and tick==stop)
        else:assert e['withheld_by_logical_scheduler'] and tick>=stop
    actual=[(successful[c['solve_id']]['input_state_id'],c['application_tick']) for c in rollout['commands']
        if c['reason']=='new_solve' and c['solve_id'] in successful]
    expected_app=[(e['input_state_id'],release[e['input_state_id']]) for e in successful.values() if release[e['input_state_id']]<stop]
    assert actual==expected_app
    return True
