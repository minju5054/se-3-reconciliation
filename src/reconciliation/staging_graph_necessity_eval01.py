"""Outcome-blind source selection and frozen graph-necessity decisions.

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
PNGS = ['source_reference_geometry.png', 'execution_outcome.png', 'severity_vs_benefit.png']


@dataclass(frozen=True, slots=True)
class SelectionRecord:
    case_id: str
    episode_id: str
    eligible: bool
    delta_phi_rad: float | None
    spacing_severity_abs_log_rho: float | None


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
    return dict(delta_phi_rad=delta, rho_d=rho, spacing_severity_abs_log_rho=abs(math.log(rho)),
                d_F_m=d_F, B_to_entry_chord_m=float(np.linalg.norm(chord)))


def eligibility(*, valid, fresh_safe, B_safe, history_safe, v_minus, travel, remaining,
                suffix_rows, stage_safe, geometry_defined, excluded, config):
    gates = dict(source_valid=bool(valid), entire_FRESH_safe=bool(fresh_safe), B_safe=bool(B_safe),
        recorded_history_safe=bool(history_safe), physically_moving=bool(v_minus > config['minimum_v_minus_strict_mps']
        and travel >= config['minimum_history_travel_m']), sufficient_future=bool(remaining >= config['minimum_remaining_arc_m']
        and suffix_rows >= config['minimum_suffix_rows']), B_ENTRY_safe=bool(stage_safe),
        severity_defined=bool(geometry_defined), development_disjoint=not excluded)
    return gates, [k for k,v in gates.items() if not v]


def select(records, target=6, minimum=4):
    if target != 6 or minimum != 4:
        raise ValueError('frozen target/minimum are six/four')
    if not all(type(r) is SelectionRecord for r in records):
        raise TypeError('selection requires closed geometry-only SelectionRecord')
    eligible = [r for r in records if r.eligible]
    if len({r.case_id for r in records}) != len(records):
        raise ValueError('duplicate case identity')
    chosen, episodes, trace = [], set(), []
    for axis in ('delta_phi_rad', 'spacing_severity_abs_log_rho'):
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
        status='READY_FOR_FREEZE' if len(chosen)>=minimum else 'INSUFFICIENT_SOURCE_DIVERSITY',
        outcome_fields_used=False, episode_unit='exact recorded episode_id, including repeat')


def source_decision(q):
    methods=q['methods']; b,h,v=[methods[m] for m in METHODS[1:]]
    def observed(r,k): return r['valid'] and r['metrics'] is not None and r['metrics'][k] is not None
    def both(r): return observed(r,ATTACH) and observed(r,ENDPOINT)
    all_both=all(both(r) for r in (b,h,v)); tick=q['integration_dt_s']-ATOL
    strong_recovery=both(v) and not both(b) and not both(h)
    strong_timing=all_both and all(v['metrics'][ATTACH] <= r['metrics'][ATTACH]-tick
        and v['metrics'][ENDPOINT] <= r['metrics'][ENDPOINT]+ATOL for r in (b,h))
    regression=dict(unsafe_reference=v['unsafe_reference'], safety_abort=v['safety_abort'],
        loses_endpoint=all(observed(r,ENDPOINT) for r in (b,h)) and not observed(v,ENDPOINT),
        loses_attachment=all(observed(r,ATTACH) for r in (b,h)) and not observed(v,ATTACH))
    positive=bool(v['planning_valid'] and both(v) and (strong_recovery or strong_timing))
    intermediate=not both(b) and both(h) and both(v)
    return dict(BOTH_DWELLS={m:both(r) for m,r in methods.items()}, GRAPH_SPECIFIC_POSITIVE=positive,
        INTERMEDIATE_GEOMETRY_POSITIVE=intermediate, strong_recovery=strong_recovery, strong_timing_gain=strong_timing,
        V2_regression=any(regression.values()), V2_regression_reasons=regression,
        V2_nonconvergence=v['nonconvergence'], V2_invalid_reference=not v['planning_valid'],
        simpler_technically_valid=all(r['technical_valid'] for r in (b,h)))


def classify(sources, selected_count, technical_valid=True):
    decisions={sid:source_decision(q) for sid,q in sources.items()}
    graph=sum(d['GRAPH_SPECIFIC_POSITIVE'] for d in decisions.values())
    intermediate=sum(d['INTERMEDIATE_GEOMETRY_POSITIVE'] for d in decisions.values())
    invalid=sum(d['V2_invalid_reference'] and d['simpler_technically_valid'] for d in decisions.values())
    safety_failure=any(d['V2_regression_reasons']['unsafe_reference'] or d['V2_regression_reasons']['safety_abort'] for d in decisions.values())
    loses_endpoint=any(d['V2_regression_reasons']['loses_endpoint'] for d in decisions.values())
    if not technical_valid or any(not q['technical_valid'] for q in sources.values()):
        label='TECHNICAL_BLOCKED'
    elif selected_count < 4:
        label='INSUFFICIENT_SOURCE_DIVERSITY'
    elif len(sources) != selected_count:
        label='TECHNICAL_BLOCKED'
    elif graph >= 2 and not safety_failure and not loses_endpoint:
        label='GRAPH_SPECIFIC_GAIN_SUPPORTED'
    elif intermediate >= 2:
        label='INTERMEDIATE_GEOMETRY_NEEDED'
    elif all(d['BOTH_DWELLS'][METHODS[1]] for d in decisions.values()):
        label='STAGING_EVALUATION_SUPPORTED'
    elif invalid >= 2:
        label='GRAPH_FORMULATION_NOT_ROBUST'
    else:
        label='MIXED_EVIDENCE'
    return dict(classification=label, sources=decisions, graph_specific_positive_count=graph,
        intermediate_geometry_positive_count=intermediate, V2_regression_count=sum(d['V2_regression'] for d in decisions.values()),
        V2_nonconvergence_count=sum(d['V2_nonconvergence'] for d in decisions.values()),
        V2_invalid_reference_count=sum(d['V2_invalid_reference'] for d in decisions.values()))
