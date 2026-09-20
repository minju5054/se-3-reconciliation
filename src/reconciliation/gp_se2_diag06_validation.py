"""Frozen gate and descriptive G3 taxonomy; original acceptance is untouched."""
from __future__ import annotations
import numpy as np
from .gp_se2_diag05_validation import PROTOCOL as P05, verify_point as verify_g2, equality_diagnostics as rank_g2
from .gp_se2_diag05_validation import schedule as schedule_g2
from .gp_se2_diag02_validation import error_report
from .gp_se2_diag06_constraints import G3
from .gp_se2_diag06_witnesses import extract_runs

PROTOCOL=dict(P05,experiment='GP-SE2-DIAG-06',grid_order=[G3],
    historical_baselines='saved G0/G1 DIAG04 and G2 DIAG05; no reoptimization',
    extra_inequalities='480 quarters + frozen union count',
    point_policy='five original seeds, five G2 latest, five unchanged DIAG04 seed perturbations',
    witness_selection='one minimum signed margin per connected violating run on canonical interval-side ordered 1 ms + .371 offset trace; earliest time ties',
    witness_identity=['family','interval_index','canonical_grid_sample_index'],
    witness_union='single common union from four hard G2 latest vectors, applied to all five starts',
    new_witness_row_policy='only the signed motion family generating each record; never lateral',
    adaptive_refinement=False,additional_refinement_rounds=0,search_uses_exact_continuous_extrema=False,
    classification_relocation='all frozen witness margins >= -original allowance, but a different full-sampled location violates a motion inequality; original-run-region overlap is separately reported')


def schedule():
    rows=schedule_g2()
    for r in rows:r['grid']=G3;r['solve_id']=f"{r['case_role']}__{r['method']}__{r['initialization']}__{G3}"
    return rows


def verify_point(name,z,base,geometry,original,g2,p2,g3,p3,directions):
    old,arrays=verify_g2(name,z,base,geometry,original,g2,p2,directions)
    p3.warmup(z);a=g2.evaluate(z);b=g3.evaluate(z);n=len(a['inequality'])
    parity=dict(objective=b['objective']==a['objective'],equality=np.array_equal(b['equality'],a['equality']),
        inequality_prefix=np.array_equal(b['inequality'][:n],a['inequality']),
        objective_gradient=np.array_equal(p3.objective_gradient(z),p2.objective_gradient(z)),
        equality_jacobian=np.array_equal(p3.equality_jacobian(z),p2.equality_jacobian(z)),
        inequality_jacobian_prefix=np.array_equal(p3.inequality_jacobian(z)[:n],p2.inequality_jacobian(z)))
    w=len(g3.rows);j=p3.inequality_jacobian(z)[n:];values=p3.values(z)['witness_inequality']
    primal=error_report(values[:,None],b['witness_inequality'][:,None],PROTOCOL['primal_tolerance']) if w else dict(passed=True)
    arrays.update(witness_jacobian=j,witness_primal_AD=values,witness_primal_numpy=b['witness_inequality'])
    directional=[]
    for number,h in enumerate(PROTOCOL['directional_fd_steps']):
        fd=[]
        for d in directions:
            original._guard_wrap_cuts(np.asarray(z)+h*d,'runtime');original._guard_wrap_cuts(np.asarray(z)-h*d,'runtime')
            fd.append((g3.evaluate(np.asarray(z)+h*d)['witness_inequality']-g3.evaluate(np.asarray(z)-h*d)['witness_inequality'])/(2*h))
        fd=np.stack(fd,axis=1);predicted=j@directions.T
        reports=[dict(witness_row_index=k,family=row['family'],unit=row['unit'],
            **error_report(predicted[k:k+1],fd[k:k+1],PROTOCOL['constraint_derivative_tolerance'],row_offset=k)) for k,row in enumerate(g3.rows)]
        directional.append(dict(step=h,reports=reports,passed=all(r['passed'] for r in reports)))
        arrays[f'witness_{number}_AD']=predicted;arrays[f'witness_{number}_FD']=fd
    shape=p3.equality_jacobian(z).shape==(30,150) and j.shape==(w,150)
    valid=old['valid'] and all(parity.values()) and shape and primal['passed'] and all(r['passed'] for r in directional[-2:])
    return dict(name=name,valid=bool(valid),base_G2_verification=old,parity=parity,dimensions=g3.dimensions(z),
        witness_count=w,witness_primal=primal,witness_directional=directional,shapes_valid=shape,
        status='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if valid else 'DERIVATIVE_VALIDATION_FAILED',no_FD_fallback=True),arrays


def equality_diagnostics(original,provider,vectors):
    out=rank_g2(original,provider,vectors)
    for r in out['records']:
        if r.get('grid')=='G2_QUARTER_INEQUALITY_ONLY':r['grid']=G3
    out['meaning']='G0 and G3 at identical vectors; G2 equalities delegate to G0 literally'
    return out


def witness_diagnostics(view,vector,trace):
    margins=view.evaluate(vector)['witness_inequality'];tol=view.config['inequality_tolerance'];rows=[]
    from .gp_se2_diag06_witnesses import signed_margins
    all_values=signed_margins(trace,view.config)
    for row,margin in zip(view.rows,margins):
        values,all_margins=all_values[row['family']];k=row['canonical_grid_sample_index']
        worst=int(np.argmin(all_margins))
        rows.append(dict(witness_row_index=row['witness_row_index'],family=row['family'],unit=row['unit'],
            interval_index=row['interval_index'],local_fraction=row['local_fraction'],time_s=row['time_s'],
            canonical_grid_sample_index=k,new_margin=float(margin),new_tolerance_excess=float(max(0.,-margin-tol)),
            feasible=bool(margin>=-tol),no_longer_worst_sample=bool(k!=worst),
            new_worst_sample_index=worst,new_worst_time_s=float(trace['times_s'][worst]),
            new_worst_margin=float(all_margins[worst]),
            old_sources=[dict(source=s['source'],old_margin=s['signed_margin'],old_tolerance_excess=s['tolerance_excess']) for s in row.get('source_violations',[])]))
    return rows


def classify(result, remaining, witness_rows, *, benign=False):
    latest=result['candidate_checks'][1];full=latest['full_acceptance'];flags=full.get('additional_grid',{}).get('flags',{})
    failed=sorted(k for k,v in flags.items() if not v)
    seed=result['candidate_checks'][0]['objective'];selected=result['candidate_objective']
    threshold=max(10*result['config']['ftol'],1e-8*max(1,abs(seed)))
    improved=bool(result['selected_full_feasible'] and selected is not None and seed-selected>threshold)
    repaired=all(w['feasible'] for w in witness_rows)
    dense=latest['constraint_report'];ineq_ok=dense.get('maximum_inequality_violation',np.inf)<=result['config']['inequality_tolerance']
    if benign:outcome='BENIGN_OPTIMIZATION_PRESERVED' if improved else 'BENIGN_FEASIBILITY_ONLY' if result['selected_full_feasible'] else 'BENIGN_REGRESSION'
    elif result['selected_full_feasible']:outcome='FULL_FEASIBILITY_RECOVERED'
    elif not result['solver_success']:outcome='NUMERICAL_REGRESSION'
    elif failed==['lateral_velocity'] and not remaining and ineq_ok and dense.get('body_derivative_identity_valid',False):
        outcome='INEQUALITY_GAP_CLOSED_LATERAL_ONLY_REMAINS'
    elif remaining and repaired:outcome='VIOLATION_RELOCATED'
    elif remaining:outcome='TARGETED_VIOLATION_REMAINS'
    else:outcome='OTHER_FULL_FAILURE'
    return dict(outcome=outcome,full_failure_flags=failed,frozen_witnesses_repaired=repaired,
        supplemental_motion_violating_runs=len(remaining),feasible_seed_objective_improved=improved,
        numerical_improvement_threshold=threshold,objective_delta=None if selected is None else selected-seed,
        lateral_is_only_full_failure=failed==['lateral_velocity'] and ineq_ok)
