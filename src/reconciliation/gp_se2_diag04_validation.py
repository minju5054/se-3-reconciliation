"""Frozen DIAG-04 point inventory and derivative checks, never optimization."""
from __future__ import annotations
import copy
import hashlib
import numpy as np

from .gp_se2_diag02_validation import PROTOCOL as ORIGINAL_PROTOCOL, validate_point, error_report

HARD='episode_013_repeat_01/handoff_024'
BENIGN='episode_001_repeat_01/handoff_002'
GRIDS=('G0_ORIGINAL','G1_QUARTER')
PAIR_SPECS=[(HARD,'HARD',m,s) for m in ('M2_GP_NO_OBSTACLE','M3_GP_CONSTRAINED')
            for s in ('I0_FRESH','I1_DECEL')]+[(BENIGN,'BENIGN_CONTROL','M3_GP_CONSTRAINED','I1_DECEL')]
PROTOCOL=dict(experiment='GP-SE2-DIAG-04',planned_GP_solves=10,
    grid_order=list(GRIDS),quarter_fractions=[.25,.75],extra_equalities=60,extra_inequalities=480,
    original_rows_prefix=True,environment_and_goal_rows_unchanged=True,
    support_states=31,variables=150,horizon_s=3.,support_dt_s=.1,
    max_iterations=200,prepared_solve_budget_s=30.,ftol=1e-7,
    derivative_mode='SUPPLIED_JAC for both grids; unchanged base provider plus quarter-only AD extension',
    solver_scaling_changed=False,physical_acceptance_changed=False,
    initialization_source='original GP-SE2-02 initialization .npy, never historical optimized final',
    point_policy='five original seeds + five historical latest vectors + five fixed perturbations of seeds',
    point_count=15,random_seed=ORIGINAL_PROTOCOL['random_seed'],
    directional_fd_steps=ORIGINAL_PROTOCOL['directional_fd_steps'],direction_count=3,
    primal_tolerance=ORIGINAL_PROTOCOL['primal_tolerance'],
    objective_derivative_tolerance=ORIGINAL_PROTOCOL['objective_derivative_tolerance'],
    constraint_derivative_tolerance=ORIGINAL_PROTOCOL['constraint_derivative_tolerance'],
    required_directional_steps='both two finest; all errors preserved',
    perturbation_scales=[1e-4]*5,perturbation_used_as_seed=False,
    derivative_failure='no primary solves on failed verification; no threshold relaxation or FD fallback',
    full_checker='unchanged check_full_candidate(base_problem,...), plus separately recomputed quarter rows',
    rank_column_scales=[1.,1.,1.,.8,3.],rank_relative_cutoffs=[1e-12,1e-10,1e-8,1e-6],
    rank_machine_threshold='max(matrix.shape)*float64_epsilon*largest_singular_value',
    rank_near_zero_row_norm=1e-12,rank_implies_infeasibility=False,
    candidate_tie_break='minimum original objective, then lexical source label',
    new_VLA_inference=0,new_MPC_solve=0,new_rollout=0,new_GUI_runtime=0)


def schedule():
    rows=[]
    for pair,(case_id,role,method,seed) in enumerate(PAIR_SPECS):
        for grid in GRIDS:
            rows.append(dict(solve_index=len(rows),pair_index=pair,case_id=case_id,case_role=role,
                method=method,initialization=seed,grid=grid,
                solve_id=f'{role}__{method}__{seed}__{grid}'))
    return rows


def vector_hash(z):
    x=np.asarray(z,dtype=np.float64)
    if x.shape!=(150,) or not np.all(np.isfinite(x)):raise ValueError('finite 150-variable saved chart required')
    return hashlib.sha256(x.tobytes()).hexdigest()


def directions_and_perturbation():
    rng=np.random.default_rng(PROTOCOL['random_seed'])
    directions=rng.normal(size=(3,150));directions/=np.linalg.norm(directions,axis=1,keepdims=True)
    # Independent RNG makes the perturbation fixed independently of direction consumption.
    perturbation=np.random.default_rng(PROTOCOL['random_seed']).normal(size=(30,5))*np.array(PROTOCOL['perturbation_scales'])
    return directions,perturbation.ravel()


def _quarter_reports(actual, expected, tolerance):
    output=[]
    families=[('lateral','equality',0,60,'m/s')]
    for i,(name,unit) in enumerate((('linear_speed_lower','m/s'),('linear_speed_upper','m/s'),
        ('angular_speed_upper','rad/s'),('angular_speed_lower','rad/s'),
        ('linear_acceleration_upper','m/s^2'),('linear_acceleration_lower','m/s^2'),
        ('angular_acceleration_upper','rad/s^2'),('angular_acceleration_lower','rad/s^2'))):
        families.append((name,'inequality',60*i,60*(i+1),unit))
    for family,field,start,end,unit in families:
        output.append(dict(family='quarter_'+family,field=field,unit=unit,
            **error_report(np.asarray(actual[field])[start:end],np.asarray(expected[field])[start:end],tolerance,row_offset=start)))
    return output


def verify_point(name, vector, base, geometry, base_provider, g0, g1, p0, p1, directions):
    """Reuse original full base derivative validation, check only appended AD separately."""
    z=np.asarray(vector,np.float64);vector_hash(z);before=z.copy()
    original,base_arrays=validate_point(dict(name=name,problem=base,geometry=geometry,vector=z,
        mode='smooth_with_geometry_classification',full_coordinate=False),base_provider,directions)
    p0.warmup(z);p1.warmup(z)
    bv=base.evaluate(z);v0=g0.evaluate(z);v1=g1.evaluate(z)
    ne,ni=len(bv['equality']),len(bv['inequality'])
    parity=dict(g0_objective_exact=v0['objective']==bv['objective'],
        g0_equality_exact=np.array_equal(v0['equality'],bv['equality']),
        g0_inequality_exact=np.array_equal(v0['inequality'],bv['inequality']),
        g1_objective_exact=v1['objective']==bv['objective'],
        g1_base_equality_exact=np.array_equal(v1['equality'][:ne],bv['equality']),
        g1_base_inequality_exact=np.array_equal(v1['inequality'][:ni],bv['inequality']))
    jac_names={'objective':'objective_gradient','equality':'equality_jacobian','inequality':'inequality_jacobian'}
    j1={}
    for field,callback in jac_names.items():
        jbase=np.asarray(getattr(base_provider,callback)(z));j0=np.asarray(getattr(p0,callback)(z));j1[field]=np.asarray(getattr(p1,callback)(z))
        parity['g0_'+field+'_derivative_exact']=np.array_equal(jbase,j0)
        prefix=j1[field] if field=='objective' else j1[field][:len(jbase)]
        parity['g1_base_'+field+'_derivative_exact']=np.array_equal(jbase,prefix)
    av=p1.values(z)
    extra=dict(equality=v1['equality'][ne:],inequality=v1['inequality'][ni:])
    ad=dict(equality=av['equality'][ne:],inequality=av['inequality'][ni:])
    primal=_quarter_reports({k:v[:,None] for k,v in ad.items()},
        {k:v[:,None] for k,v in extra.items()},PROTOCOL['primal_tolerance'])
    ej=dict(equality=j1['equality'][ne:],inequality=j1['inequality'][ni:])
    shapes=ej['equality'].shape==(60,150) and ej['inequality'].shape==(480,150) and all(np.all(np.isfinite(v)) for v in ej.values())
    arrays=dict(vector=z,directions=directions,quarter_equality_jacobian=ej['equality'],quarter_inequality_jacobian=ej['inequality'])
    for key,value in base_arrays.items():arrays['base_'+key]=value
    directional=[]
    for index,h in enumerate(PROTOCOL['directional_fd_steps']):
        predicted={k:j@directions.T for k,j in ej.items()};fd={k:np.empty_like(v) for k,v in predicted.items()}
        for col,d in enumerate(directions):
            # Same original relative-Log/goal-cut guard, no silent perturbation of a cut.
            base_provider._guard_wrap_cuts(z+h*d,'runtime');base_provider._guard_wrap_cuts(z-h*d,'runtime')
            plus,minus=g1.evaluate(z+h*d),g1.evaluate(z-h*d)
            fd['equality'][:,col]=(plus['equality'][ne:]-minus['equality'][ne:])/(2*h)
            fd['inequality'][:,col]=(plus['inequality'][ni:]-minus['inequality'][ni:])/(2*h)
        reports=_quarter_reports(predicted,fd,PROTOCOL['constraint_derivative_tolerance'])
        directional.append(dict(step=h,reports=reports,passed=all(x['passed'] for x in reports)))
        for field in fd:
            arrays[f'quarter_{index}_{field}_AD_directional']=predicted[field]
            arrays[f'quarter_{index}_{field}_central_FD']=fd[field]
    result=dict(name=name,vector_sha256=vector_hash(z),base_verification=original,
        parity={k:bool(v) for k,v in parity.items()},quarter_primal=primal,quarter_shapes_valid=bool(shapes),quarter_directional=directional,
        original_geometry_queries_only=True,base_environment_metadata=original['geometry_metadata'],
        no_numerical_fallback=True,optimizer_executed=False)
    result['valid']=bool(original['valid'] and all(parity.values()) and shapes and all(x['passed'] for x in primal) and all(x['passed'] for x in directional[-2:]))
    result['status']='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS' if result['valid'] else 'DERIVATIVE_VALIDATION_FAILED'
    assert np.array_equal(z,before)
    return result,arrays
