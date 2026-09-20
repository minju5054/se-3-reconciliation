"""Fixed DIAG-05 verification, equality parity and outcome classification."""
from __future__ import annotations
import numpy as np
from .gp_se2_diag04_validation import PROTOCOL as P04, PAIR_SPECS, vector_hash
from .gp_se2_diag02_validation import validate_point, error_report
from .gp_se2_diag04_solver import equality_rank_diagnostics
from .gp_se2_diag04_constraints import MOTION_ROWS
from .gp_se2_diag05_constraints import G2

PROTOCOL=dict(P04,experiment='GP-SE2-DIAG-05',planned_GP_solves=5,
    grid_order=[G2],extra_equalities=0,extra_inequalities=480,
    historical_baselines='DIAG-04 saved G0/G1; no reoptimization',
    point_policy='exact DIAG-04 saved 15 points and directions; no new random probes',
    near_active_threshold='max(1e-6,10*original inequality_tolerance)',
    active_threshold='abs(margin)<=original inequality_tolerance',
    equality_parity_failure='SOURCE_OR_IMPLEMENTATION_MISMATCH',
    outcome_threshold='benign objective improvement > max(10*ftol,1e-8*max(1,abs(seed objective)))')


def schedule():
    return [dict(solve_index=i,pair_index=i,case_id=cid,case_role=role,method=method,
        initialization=seed,grid=G2,solve_id=f'{role}__{method}__{seed}__{G2}')
        for i,(cid,role,method,seed) in enumerate(PAIR_SPECS)]


def family_reports(actual, expected, tolerance):
    return [dict(family=row[0],unit=row[2],**error_report(actual[k*60:(k+1)*60],
        expected[k*60:(k+1)*60],tolerance,row_offset=k*60)) for k,row in enumerate(MOTION_ROWS)]


def verify_point(name, vector, base, geometry, original, view, provider, directions):
    z=np.asarray(vector,np.float64); before=z.copy(); vector_hash(z)
    base_check, base_arrays=validate_point(dict(name=name,problem=base,geometry=geometry,vector=z,
        mode='smooth_with_geometry_classification',full_coordinate=False),original,directions)
    provider.warmup(z); b=base.evaluate(z); g=view.evaluate(z); ad=provider.values(z); n=len(b['inequality'])
    parity=dict(objective=g['objective']==b['objective'],equality=np.array_equal(g['equality'],b['equality']),
        inequality_prefix=np.array_equal(g['inequality'][:n],b['inequality']),
        objective_gradient=np.array_equal(provider.objective_gradient(z),original.objective_gradient(z)),
        equality_jacobian=np.array_equal(provider.equality_jacobian(z),original.equality_jacobian(z)),
        inequality_jacobian_prefix=np.array_equal(provider.inequality_jacobian(z)[:n],original.inequality_jacobian(z)))
    j=provider.inequality_jacobian(z)[n:]; primal=family_reports(ad['quarter_inequality'][:,None],
        g['quarter_inequality'][:,None],PROTOCOL['primal_tolerance'])
    arrays=dict(vector=z,directions=directions,quarter_jacobian=j,
        **{'base_'+k:v for k,v in base_arrays.items()}); directional=[]
    for number,h in enumerate(PROTOCOL['directional_fd_steps']):
        fd=[]
        for d in directions:
            original._guard_wrap_cuts(z+h*d,'runtime');original._guard_wrap_cuts(z-h*d,'runtime')
            fd.append((view.evaluate(z+h*d)['quarter_inequality']-view.evaluate(z-h*d)['quarter_inequality'])/(2*h))
        fd=np.stack(fd,axis=1); predicted=j@directions.T
        reports=family_reports(predicted,fd,PROTOCOL['constraint_derivative_tolerance'])
        directional.append(dict(step=h,reports=reports,passed=all(r['passed'] for r in reports)))
        arrays[f'quarter_{number}_AD']=predicted;arrays[f'quarter_{number}_FD']=fd
    shape=provider.equality_jacobian(z).shape==(30,150) and j.shape==(480,150)
    valid=base_check['valid'] and all(parity.values()) and shape and all(r['passed'] for r in primal) and all(r['passed'] for r in directional[-2:])
    assert np.array_equal(before,z)
    return dict(name=name,vector_sha256=vector_hash(z),valid=bool(valid),parity=parity,
        dimensions=view.dimensions(z),shapes_valid=shape,base_verification=base_check,
        quarter_primal=primal,quarter_directional=directional,no_FD_fallback=True,
        status='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if valid else 'DERIVATIVE_VALIDATION_FAILED'),arrays


def equality_diagnostics(original, provider, vectors):
    parity=[]
    for source,z in vectors.items():
        if z is None:
            parity.append(dict(source=source,available=False,literal_equal=None));continue
        a,b=original.equality_jacobian(z),provider.equality_jacobian(z)
        equal=a.shape==b.shape==(30,150) and np.array_equal(a,b)
        if not equal:raise ValueError('SOURCE_OR_IMPLEMENTATION_MISMATCH: G2 equality Jacobian')
        parity.append(dict(source=source,available=True,literal_equal=True,shape=list(a.shape)))
    rank=equality_rank_diagnostics(original,provider,vectors)
    for record in rank['records']:
        if record.get('grid')=='G1_QUARTER':record['grid']=G2
    rank.update(equality_parity=parity,meaning='G0 and G2 evaluated at the SAME vector; not different optimized vectors')
    return rank


def classify(result, *, benign=False):
    latest=result['candidate_checks'][1]; full=latest.get('full_acceptance',{})
    flags=full.get('additional_grid',{}).get('flags')
    failed=None if flags is None else sorted(k for k,v in flags.items() if not v)
    if flags is None:layer='UNAVAILABLE'
    elif not latest['collocation']['feasible']:layer='A_GRID_FAIL'
    elif latest['grid_and_full_feasible']:layer='D_FULL_VALID'
    elif (failed==['lateral_velocity']
          and latest['constraint_report'].get('maximum_inequality_violation',np.inf)<=result['config']['inequality_tolerance']
          and latest['constraint_report'].get('body_derivative_identity_valid',False)):
        layer='B_LATERAL_ONLY'
    else:layer='C_OTHER_FULL_FAILURE'
    seed= result['candidate_checks'][0]['objective']; selected=result['candidate_objective']
    threshold=max(10*result['config']['ftol'],1e-8*max(1,abs(seed))) if seed is not None else None
    improved=bool(result['selected_full_feasible'] and selected is not None and seed is not None and seed-selected>threshold)
    if flags is None:outcome='SOURCE_OR_DERIVATIVE_BLOCKER'
    elif benign:
        outcome='BENIGN_OPTIMIZATION_RECOVERED' if improved else 'BENIGN_FEASIBILITY_ONLY' if result['selected_full_feasible'] else 'BENIGN_REGRESSION'
    elif result['selected_full_feasible']:outcome='FULL_FEASIBILITY_RECOVERY'
    elif result['solver_success']:outcome='NUMERICAL_RECOVERY_LATERAL_REMAINS' if layer=='B_LATERAL_ONLY' else 'NUMERICAL_RECOVERY_OTHER_FAILURE'
    else:outcome='NO_NUMERICAL_RECOVERY'
    return dict(layer=layer,outcome=outcome,full_failure_flags=failed,
        feasible_seed_objective_improved=improved,numerical_improvement_threshold=threshold,
        objective_delta=None if selected is None or seed is None else selected-seed)
