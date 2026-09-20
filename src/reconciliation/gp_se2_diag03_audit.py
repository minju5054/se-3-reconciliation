"""Bounded read-only audit of saved GP-SE2-02 vectors; no optimization entry point."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np

from .gp_se2_diag_acceptance import check_full_candidate
from .gp_se2_diag03_derivatives import DERIVATIVE_PROTOCOL, check_interpolation_consistency
from .gp_se2_diag03_intervals import audit_intervals
from .gp_se2_formulation import _constraint_report as constraint_report
from .se2 import wrap_angle

HARD = 'episode_013_repeat_01/handoff_024'
BENIGN = 'episode_001_repeat_01/handoff_002'
METHODS = ('M2_GP_NO_OBSTACLE', 'M3_GP_CONSTRAINED')
INITIALIZATIONS = ('I0_FRESH', 'I1_DECEL')
PROTOCOL = dict(experiment='GP-SE2-DIAG-03', planned_records=9,
    primary_final_records=4, context_initial_records=4, benign_reference_records=1,
    new_VLA_inference=0, new_GP_optimization=0, new_rigid_optimization=0,
    new_MPC_solve=0, new_closed_loop_rollout=0, new_Isaac_GUI_runtime=0,
    original_reproduction_absolute_tolerance=1e-8, original_reproduction_relative_tolerance=1e-10,
    support_reconstruction_absolute_tolerance=1e-10, vector_equality='literal float64 bytes',
    derivative_diagnostic=DERIVATIVE_PROTOCOL,
    supplemental_spacing_s=[.010, .005, .001], finest_offset_fraction=.371,
    violation_bracket_maximum_width_s=.0001,
    grid_detection='G0 original; G1 quarter points; G2 post-hoc per-interval quantity witnesses; no constraints added',
    representative_rule='maximum observed tolerance excess per quantity, earliest interval on ties',
    source_frame='X=T_world_agent; right-local X=anchor Exp(delta); body twist=vee(X^-1 dX/dt)',
    units=dict(pose=['m','m','rad'],twist=['m/s','m/s','rad/s'],acceleration=['m/s^2','m/s^2','rad/s^2']),
    acceleration='time derivative of body-twist components, not world or centripetal acceleration',
    acceptance_modified=False, finite_sampling_continuous_proof=False,
    fixed_initials='load saved vectors only; no new seed construction',
    missing_saved_vector='MISSING_SAVED_VECTOR, never reconstruct or optimize a substitute')


def record_specs():
    rows = []
    for method in METHODS:
        for initialization in INITIALIZATIONS:
            for phase in ('initial', 'final'):
                rows.append(dict(record_id=f'hard__{method}__{initialization}__{phase}',
                    case_id=HARD, role='hard_'+phase, method=method,
                    initialization=initialization, phase=phase))
    rows.append(dict(record_id='benign__M3_GP_CONSTRAINED__I1_DECEL__final', case_id=BENIGN,
        role='benign_final', method=METHODS[1], initialization='I1_DECEL', phase='final'))
    return rows


def vector_hash(vector):
    return hashlib.sha256(np.asarray(vector, dtype=np.float64).tobytes()).hexdigest()


def numeric_comparison(actual, expected):
    """Recursive saved-value comparison, with no outcome-dependent tolerance."""
    a, b = np.asarray(actual), np.asarray(expected)
    if a.shape != b.shape:
        return dict(literal_equal=False, numerical_agreement=False, shape_equal=False)
    if a.dtype.kind in 'biuf' and b.dtype.kind in 'biuf':
        finite = bool(np.all(np.isfinite(a)) and np.all(np.isfinite(b)))
        return dict(literal_equal=bool(np.array_equal(a,b)), shape_equal=True,
            numerical_agreement=bool(finite and np.allclose(a,b,atol=1e-8,rtol=1e-10)),
            maximum_absolute_error=float(np.max(np.abs(a.astype(float)-b.astype(float)))) if a.size else 0.)
    return dict(literal_equal=bool(actual==expected), numerical_agreement=bool(actual==expected), shape_equal=True)


def compare_tree(actual, expected, prefix=''):
    rows=[]
    if isinstance(expected,dict):
        for key,value in expected.items():
            name=prefix+'.'+key if prefix else key
            if key not in actual:
                rows.append(dict(field=name,literal_equal=False,numerical_agreement=False,reason='missing'))
            else: rows.extend(compare_tree(actual[key],value,name))
    else:
        rows.append(dict(field=prefix,**numeric_comparison(actual,expected)))
    return rows


def restore_record(spec, case, solver, saved_initial, saved_initial_metadata):
    """Candidate availability never controls access to a rejected latest iterate."""
    key='initial_vector' if spec['phase']=='initial' else 'latest_iterate'
    value=solver.get(key)
    if value is None:
        return dict(spec,available=False,status='MISSING_SAVED_VECTOR',reason=f'solver_result.{key} is missing')
    x=np.asarray(value,dtype=np.float64)
    if x.shape!=(150,) or not np.all(np.isfinite(x)):
        raise ValueError('saved vector has invalid shape/nonfinite values')
    problem=case['problem']; p,v=problem.unpack(x)
    initial=np.asarray(saved_initial,dtype=np.float64)
    if initial.shape!=(150,) or not np.all(np.isfinite(initial)):
        raise ValueError('saved initialization shape/nonfinite values')
    if initial.tobytes()!=np.asarray(solver['initial_vector'],np.float64).tobytes():
        raise ValueError('saved initialization file differs from solver initial_vector')
    expected_p=solver.get('latest_support_poses') if spec['phase']=='final' else saved_initial_metadata.get('chart_reconstructed_poses')
    expected_v=solver.get('latest_support_twists') if spec['phase']=='final' else saved_initial_metadata.get('chart_reconstructed_twists')
    reconstruction=dict(saved_support_available=expected_p is not None and expected_v is not None)
    if reconstruction['saved_support_available']:
        ep,ev=np.asarray(expected_p),np.asarray(expected_v)
        if ep.shape!=p.shape or ev.shape!=v.shape or not np.all(np.isfinite(ep)) or not np.all(np.isfinite(ev)):
            raise ValueError('saved support arrays have invalid shape/nonfinite values')
        periodic=p-ep;periodic[:,2]=wrap_angle(periodic[:,2])
        reconstruction.update(pose_literal_equal=bool(np.array_equal(p,ep)),twist_literal_equal=bool(np.array_equal(v,ev)),
            pose_periodic_max_error=float(np.max(np.abs(periodic))),twist_max_error=float(np.max(np.abs(v-ev))),
            consistent=bool(np.max(np.abs(periodic))<=1e-10 and np.max(np.abs(v-ev))<=1e-10))
    else: reconstruction.update(consistent=False,reason='MISSING_SAVED_SUPPORT_ARRAYS')
    if not np.array_equal(p[0],problem.boundary_pose) or not np.array_equal(v[0],problem.initial_twist):
        raise ValueError('boundary/physical initial twist changed')
    return dict(spec,available=True,vector=x,vector_sha256=vector_hash(x),support_poses=p,support_twists=v,
        reconstruction=reconstruction,solver_termination=solver['termination'],
        solver_converged=bool(solver['solver_success']),candidate_available=bool(solver['candidate_found']),
        selected_iterate=solver.get('selected_iterate'),source_vector_key=key)


def audit_record(record, case, solver, post_checks, saved_initial_full):
    if not record['available']: return record
    x=record['vector']; before=x.copy(); problem=case['problem']
    evaluation=problem.evaluate(x)
    collocation=constraint_report(evaluation['equality'],evaluation['inequality'],problem.config)
    full=check_full_candidate(problem,x,case)
    dense=full['original_dense_report']
    name='initial' if record['phase']=='initial' else 'latest_iterate'
    historical=next((r for r in solver['candidate_checks'] if r['iterate']==name),None)
    comparisons=[]
    if historical is not None:
        comparisons += compare_tree(evaluation['objective'],historical['objective'],'objective')
        comparisons += compare_tree(collocation,historical['collocation'],'collocation')
        comparisons += compare_tree(dense,historical['constraint_report'],'dense_report')
    historical_post=next((r for r in post_checks['rows'] if r['iterate']==name),None)
    if historical_post:
        comparisons += compare_tree(full['full_feasible'],historical_post['full_feasible'],'full_feasible')
    if record['phase']=='initial':
        comparisons += compare_tree(full,saved_initial_full,'initial_full_checker')
    intervals=audit_intervals(problem,x)
    witnesses=[dict(interval_index=w['interval_index'],local_fraction=w['u'],label=w['quantity'])
               for w in intervals['diagnostic_witnesses']]
    derivatives=check_interpolation_consistency(problem.times,record['support_poses'],record['support_twists'],witness_points=witnesses)
    failed=[k for k,v in full['additional_grid']['flags'].items() if not v]
    samples=dict(intervals['finest_trace']);samples['local_u']=samples['local_fraction']
    final=dict(record,objective=float(evaluation['objective']),
        factor_costs={k:evaluation[k] for k in ('gp_factor_costs','fresh_cost') if k in evaluation},
        factor_cost_historical_comparison='Not saved per rejected iterate; recomputed with unchanged evaluator; objective compared',
        acceptance=dict(collocation_feasible=collocation['feasible'],dense_feasible=dense['feasible'],
            full_feasible=full['full_feasible'],plan_valid=full['original_plan_valid'],failure_families=failed),
        original_reproduction=dict(comparisons=comparisons,historical_check_available=historical is not None,
            reproduced=bool(historical is not None and historical_post is not None and all(r['numerical_agreement'] for r in comparisons)),
            all_literal_equal=all(r['literal_equal'] for r in comparisons),
            historical_full_check_status=historical_post,
            note='Hard rejected final was screened out by original dense check; this audit additionally executes unchanged full checker.'),
        original_collocation=collocation, original_full=full, intervals=intervals, derivatives=derivatives,
        samples=samples,interval_extrema=intervals['interval_extrema'],grid_detection=intervals['grid_detection_comparison'])
    assert np.array_equal(x,before)
    return final


def summarize(records):
    available=[r for r in records if r['available']]
    finals=[r for r in available if r['role']=='hard_final']
    knots=[]
    for r in finals:
        c=r['original_full']['original_config']['formulation']
        for knot in r['intervals']['knot_limits']:
            for side in ('left','right'):
                a=knot['body_acceleration_'+side]
                if abs(a[0])>c['a_v_max']+c['inequality_tolerance'] or abs(a[2])>c['a_w_max']+c['inequality_tolerance']:
                    knots.append((r['record_id'],knot['time_s'],side))
    return dict(planned_records=9,available_records=len(available),unique_vectors=len({r['vector_sha256'] for r in available}),
        hard_final_count=len(finals),benign_reference_count=sum(r['role']=='benign_final' for r in available),
        original_results_reproduced=len(available)==9 and all(r['original_reproduction']['reproduced'] and r['reconstruction']['consistent'] for r in available),
        interpolation_derivative_consistent=all(r['derivatives']['summary']['interpolation_derivative_consistent'] for r in available),
        coefficient_cross_check_consistent=all(r['derivatives']['summary']['coefficient_cross_check_consistent'] for r in available),
        collocation_pass_interior_fail_observed=any(r['acceptance']['collocation_feasible'] and not r['acceptance']['dense_feasible'] for r in finals),
        one_sided_knot_violation_observed=bool(knots),
        supplemental_grid_detects_missed_violations=any(
            row['grid']=='G0_ORIGINAL' and row['finest_observed_intervals_missed'] for r in finals for row in r['grid_detection']),
        implementation_inconsistency_found=any(not r['derivatives']['summary']['coefficient_cross_check_consistent'] or not r['derivatives']['summary']['interpolation_derivative_consistent'] for r in available),
        insufficient_saved_evidence=len(available)!=9,
        new_VLA_inference=0,new_GP_optimization=0,new_rigid_optimization=0,new_MPC_solve=0,new_closed_loop_rollout=0,new_Isaac_GUI_runtime=0,
        primary_acceptance_modified=False,continuous_time_feasibility_proven=False,
        operational_status='GP_SE2_DIAG_03_COMPLETED_WITH_LIMITATIONS',
        limitations=['Finite sampling and FD consistency do not prove continuous feasibility.',
            'Hard final full feasibility was historically false after dense rejection; no historical full-query arrays exist for those finals.',
            'Algebraic coefficient check shares frozen Exp/Log/Jr primitives; independent time derivatives provide a separate check.',
            'Extra detection points were not added to an optimizer. Feasible recovery and controller execution remain untested.'])
