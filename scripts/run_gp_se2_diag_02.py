#!/usr/bin/env python3
"""Frozen sequential derivative comparison; no inference or controller execution."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[_key]='1'
os.environ['JAX_PLATFORMS']='cpu'
os.environ['JAX_ENABLE_X64']='true'
os.environ.setdefault('XLA_FLAGS','--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from run_gp_se2_01 import digest,read,write,array,table
from reconciliation.gp_se2_diagnostics import load_frozen_case


def utc(): return datetime.now(timezone.utc).isoformat()


def snapshot(run,phase):
    paths=sorted(set(list((ROOT/'src/reconciliation').glob('gp_se2*.py'))+
        list((ROOT/'scripts').glob('*gp_se2_diag*'))+list((ROOT/'tests').glob('test_gp_se2_diag*'))+
        [ROOT/'src/reconciliation/se2.py',ROOT/'configs/gp_se2_diag_02.yaml',ROOT/'pyproject.toml']))
    base=run/'implementation_sources'/phase;base.mkdir(parents=True,exist_ok=False)
    hashes={}
    for p in paths:
        if not p.is_file(): continue
        rel=p.relative_to(ROOT);target=base/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,target);hashes[str(rel)]=digest(p)
    write(base/'manifest.json',dict(created_utc=utc(),files=hashes))
    return hashes


def preservation(cfg):
    from run_gp_se2_diag_01 import preservation_snapshot
    previous=ROOT/cfg['previous_diagnostic']
    previous_cfg=yaml.safe_load((previous/'config_snapshot.yaml').read_text())
    out=preservation_snapshot(previous_cfg)
    out['previous_diagnostic_files']={str(p.relative_to(ROOT)):digest(p)
        for p in sorted(previous.rglob('*')) if p.is_file()}
    return out


def settings(run):
    cfg=yaml.safe_load((run/'config_snapshot.yaml').read_text());src=read(run/'source.json')
    assert digest(run/'config_snapshot.yaml')==src['diagnostic_config_sha256']
    for p,h in src['preserved_core_sha256'].items():
        if digest(ROOT/p)!=h: raise ValueError('preserved source drift: '+p)
    for p,h in src['input_sha256'].items():
        if digest(p)!=h: raise ValueError('preserved input drift: '+p)
    return cfg,src


def prepare(run,config):
    from reconciliation.gp_se2_diag_initialization import same_curvature_deceleration_seed
    from run_gp_se2_diag_01 import freeze_actual_inputs
    cfg=yaml.safe_load(Path(config).read_text())
    if not run.is_relative_to(ROOT/'data/robotless_gp_se2_diag_02'):
        raise ValueError('new run below DIAG-02 required')
    existing={str(p.relative_to(run)) for p in run.rglob('*') if p.is_file()} if run.exists() else set()
    allowed={'derivative_checks/protocol.json','config_snapshot.yaml','preservation_before.json',
        'preparation_failure_01.json','preparation_failure_02.json','preparation_failure_03.json',
        'actual_event/input_snapshot.json'}
    if existing-allowed:
        raise ValueError('only frozen protocol and preserved incomplete preparation may preexist')
    previous=ROOT/cfg['previous_diagnostic'];primary=ROOT/cfg['primary_run']
    validation=read(previous/'verification/validation.json')
    if not validation['valid']:raise ValueError('authoritative previous validation failed')
    current_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    if subprocess.run(['git','merge-base','--is-ancestor',cfg['starting_git_sha'],current_sha],check=False).returncode:
        raise ValueError('reviewed starting revision must be an ancestor')
    run.mkdir(parents=True,exist_ok=True)
    if (run/'config_snapshot.yaml').exists():
        if digest(config)!=digest(run/'config_snapshot.yaml'):raise ValueError('preparation config changed')
    else:shutil.copy2(config,run/'config_snapshot.yaml')
    if (run/'preservation_before.json').exists():before=read(run/'preservation_before.json')
    else:
        before=preservation(cfg);write(run/'preservation_before.json',before)
    if not before['valid']:raise ValueError('historical preservation failed')
    case=load_frozen_case(primary,cfg['actual_case_id'])
    original_source=read(primary/'source.json')
    export=Path(original_source['environment_export_source']).resolve()
    expected_export=(ROOT/yaml.safe_load((previous/'config_snapshot.yaml').read_text())['environment_export']).resolve()
    if export!=expected_export:raise ValueError('environment provenance mismatch')
    for record in original_source['environment_file_hashes']:
        if digest(primary/'environment'/record['path'])!=record['sha256']:
            raise ValueError('environment copy differs from historical hash')
        # Historical GUI added this derived mesh index to the primary copy.
        # It is covered by its original hash, but is not an export input.
        if record['path']=='geometry/render_geometry.json':continue
        if digest(export/record['path'])!=record['sha256']:
            raise ValueError('environment copy differs from validated export')
    if (run/'actual_event/input_snapshot.json').exists():
        if digest(run/'actual_event/input_snapshot.json')!=digest(previous/'actual_event/input_snapshot.json'):
            raise ValueError('partial input snapshot differs from original')
    else:freeze_actual_inputs(run,case)
    source_inputs=dict(case['hashes'])
    source_inputs[str(previous/'verification/validation.json')]=digest(previous/'verification/validation.json')
    source_inputs[str(previous/'actual_event/input_snapshot.json')]=digest(previous/'actual_event/input_snapshot.json')
    seeds={}
    for method in cfg['methods']:
        cc=load_frozen_case(primary,cfg['actual_case_id'],method,environment=case['environment']);p=cc['problem']
        init=p.initializations()[0];d=same_curvature_deceleration_seed(p)
        vectors=[p.vector(init['poses'],init['twists']),p.vector(d['poses'],d['twists'])]
        for i,name in enumerate(cfg['initializations']):
            oldfile=previous/'actual_event/single_change'/method/f'start_{i:02d}.json'
            old=read(oldfile);oldz=np.asarray(old['initial_vector'],np.float64)
            if not np.array_equal(oldz,vectors[i]):raise ValueError('saved seed vector mismatch')
            poses,twists=p.unpack(vectors[i])
            assert np.array_equal(poses[0],p.boundary_pose) and np.array_equal(twists[0],p.initial_twist)
            if name in seeds:assert np.array_equal(seeds[name],vectors[i])
            else:seeds[name]=vectors[i]
            source_inputs[str(oldfile)]=digest(oldfile)
    (run/'seeds').mkdir(exist_ok=False)
    for name,z in seeds.items():array(run/'seeds'/f'{name}.npy',z)
    core_names=['se2.py','gp_se2.py','gp_se2_formulation.py','gp_se2_environment.py','gp_se2_evaluation.py',
        'gp_se2_reference.py','gp_se2_rollout.py','gp_se2_diag_solver.py','gp_se2_diag_initialization.py',
        'gp_se2_diag_acceptance.py','gp_se2_diagnostics.py']
    core={f'src/reconciliation/{n}':digest(ROOT/'src/reconciliation'/n) for n in core_names}
    write(run/'source.json',dict(created_utc=utc(),starting_git_sha=current_sha,reviewed_git_sha=cfg['starting_git_sha'],
        primary_run=str(primary),previous_diagnostic=str(previous),environment=str((primary/'environment').resolve()),
        validated_environment_export=str(export),environment_copy_hashes_match=True,
        authoritative_previous_validation=str(previous/'verification/validation.json'),
        previous_validation_check_count=validation['check_count'],preserved_core_sha256=core,
        input_sha256=source_inputs,diagnostic_config_sha256=digest(run/'config_snapshot.yaml'),
        seed_sha256={n:digest(run/'seeds'/f'{n}.npy') for n in seeds},
        seed_reconstruction_bitwise_matches_saved=True,physical_acceptance_unchanged=True))
    write(run/'protocol.json',dict(created_utc=utc(),experiment='GP-SE2-DIAG-02',
        case_id=cfg['actual_case_id'],solve_order=cfg['solve_order'],
        original_formulation=case['config']['formulation'],primary_timing=cfg['primary_timing'],
        derivative_strategy=cfg['strategy'],single_blas_thread=True,sequential=True,retry=False,
        independent_acceptance='unchanged gp_se2_diag_acceptance.check_full_candidate',
        candidate_selection='original min objective among dense-feasible initial/latest/actual collocation-feasible callbacks; full check separately',
        numerical_improvement_threshold='max(10*ftol,1e-8*max(1,abs(J_seed)))',
        new_inference=False,new_mpc_rollout=False,navigation_performance_claim=False,
        seeds_bitwise_identical_across_all_modes=True,preparation_source_sha256=snapshot(run,'prepare')))
    print(json.dumps(dict(run=str(run),status='PREPARED',historical_files=before['historical_file_count'],
        previous_diagnostic_files=len(before['previous_diagnostic_files']))),flush=True)


def freeze(run):
    import inspect
    import importlib.metadata
    import scipy
    import scipy.optimize._slsqp_py as slsqp
    import jax,jaxlib,shapely
    cfg,src=settings(run)
    validation=read(run/'derivative_checks/authoritative_validation.json')
    if not validation.get('valid',False):raise ValueError('full derivative verification must pass before freeze')
    versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,jax=jax.__version__,
        jaxlib=jaxlib.__version__,shapely=shapely.__version__,devices=[str(d) for d in jax.devices()],
        jax_enable_x64=bool(jax.config.jax_enable_x64),slsqp_source_path=inspect.getfile(slsqp),
        slsqp_source_sha256=digest(inspect.getfile(slsqp)),
        default_jacobian='two-point finite differences with absolute epsilon; jac=None',
        supplied_interface='objective jac callable + equality/inequality dict jac callables',
        constraint_convention='h=0; g>=0; rows retain original order',
        multiplier_convention='final QP m[:meq] equality, m[meq:] inequality; no variable bounds supplied',
        official_docs=['https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html',
            'https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html',
            'https://docs.jax.dev/en/latest/automatic-differentiation.html'],
        web_scipy_documentation_version='1.18.0; installed source 1.18.1 inspected separately; no upgrade')
    versions['added_dependency_versions']={name:importlib.metadata.version(name)
        for name in ('jax','jaxlib','ml-dtypes','opt-einsum')}
    versions['thread_environment']={name:os.environ.get(name) for name in
        ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','XLA_FLAGS','JAX_PLATFORMS')}
    write(run/'runtime_versions.json',versions)
    write(run/'derivative_design.json',dict(strategy='one CPU float64 JAX forward-AD transcription plus analytic original geometry',
        primal_source='unchanged GPProblem.evaluate for both modes',
        chart='30 rows [right pose delta x,y,yaw, body vx,omega], fixed original first pose/twist',
        interfaces=dict(objective_gradient=[150],equality_jacobian=[30,150],
            inequality_jacobian_M2=[813,150],inequality_jacobian_M3=[903,150]),
        row_order='exact original lateral midpoint; motion vx,+/-omega,+/-acceleration; squared goal position,+/-yaw;workspace;obstacle',
        ad_coverage=['nonzero right-retraction chain','Exp/Log exact original branches','GP residual+Cholesky whitening',
            'original FRESH mean normalization','body velocity','complete acceleration including J_r directional chain','original goal'],
        geometry_coverage=['original bilinear grid world derivative and constant conservative offsets',
            'dynamic actual nearest polygon boundary including holes and signed distance'],
        invalid_domain='original constant -1e6 query penalty, zero derivative of constant branch only; not safe space',
        branch_limits=['selected active limiting workspace gradients at true ties',
            'selected floor-cell one-sided bilinear gradient at cell boundaries',
            'possible domain discontinuity, selected-branch only',
            'relative Log and goal-yaw cuts within1e-12 rad explicitly unsupported and rejected'],
        unsupported_gates='current event has no gates; provider rejects other gated problems',
        derivative_finite_difference_fallback=False,solver_constraint_rescaling=False,
        derivative_cache='one exact-vector cache shared by all three jac callbacks; cleared after warmup',
        source_sha256={str(p.relative_to(ROOT)):digest(p) for p in sorted((ROOT/'src/reconciliation').glob('gp_se2_diag02_*.py'))},
        verification_source=str(run/'derivative_checks/authoritative_validation.json')))
    sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    write(run/'comparison_freeze.json',dict(created_utc=utc(),implementation_git_sha=sha,
        code_sha256=snapshot(run,'comparison'),derivative_validation_sha256=digest(run/'derivative_checks/authoritative_validation.json'),
        protocol_sha256=digest(run/'protocol.json'),config_sha256=src['diagnostic_config_sha256'],
        runtime_versions_sha256=digest(run/'runtime_versions.json'),seeds=src['seed_sha256'],
        exact_order=cfg['solve_order'],ready=True))
    print(json.dumps(dict(status='COMPARISON_FROZEN',implementation_git_sha=sha)),flush=True)


def brief_full(full):
    return dict(full_feasible=full['full_feasible'],original_dense_feasible=full['original_dense_feasible'],
        original_plan_valid=full['original_plan_valid'],additional_grid_valid=full['additional_grid']['valid'],
        maximum_lateral_m_s=full['additional_grid']['maximum_absolute_lateral_velocity_m_s'],
        max_linear_acceleration_m_s2=full['additional_grid']['maximum_absolute_linear_acceleration_m_s2'],
        flags=full['additional_grid']['flags'])


def postprocess(case,result,out):
    from hashlib import sha256
    from reconciliation.gp_se2_diag_acceptance import check_full_candidate
    p=case['problem'];start=time.perf_counter();cache={};rows=[]
    callback_times={f'callback_{s["iteration"]:04d}':s['elapsed_s'] for s in result['callback_snapshots']}
    for entry in result['candidate_checks']:
        z=np.asarray(entry['vector'],np.float64);key=sha256(z.tobytes()).hexdigest();label=entry['iterate']
        row=dict(iterate=label,objective=entry['objective'],vector_sha256=key,
            discovery_time_s=0. if label=='initial' else callback_times.get(label,result['solve_wall_time_s']),
            certification='post-solve; not certified at recorded callback time',
            dense_feasible=bool(entry['constraint_report']['feasible']))
        if row['dense_feasible']:
            if key not in cache:cache[key]=check_full_candidate(p,z,case)
            row.update(brief_full(cache[key]))
        else:row.update(full_feasible=False,full_check_skipped_reason='original dense acceptance already fails')
        rows.append(row)
    full=None
    if result['candidate_vector'] is not None:
        key=sha256(np.asarray(result['candidate_vector'],np.float64).tobytes()).hexdigest()
        full=cache[key];write(out/'full_acceptance.json',full)
    else:write(out/'full_acceptance.json',dict(full_feasible=False,status='NO_CANDIDATE',metrics=None))
    write(out/'post_full_checks.json',dict(rows=rows,unique_full_checks=len(cache),
        full_validation_wall_time_s=time.perf_counter()-start,selection_rule_unchanged=True,
        selected_invalid_not_replaced=bool(full is not None and not full['full_feasible'])))
    full_seconds=time.perf_counter()-start
    initial=next(r for r in rows if r['iterate']=='initial');final=next(r for r in rows if r['iterate']=='latest_iterate')
    delta=None if result['candidate_objective'] is None else initial['objective']-result['candidate_objective']
    threshold=max(10*p.config['ftol'],1e-8*max(1,abs(initial['objective'])))
    selected_full=bool(full is not None and full['full_feasible'])
    changed={}
    if result['candidate_vector'] is not None:
        a,b=p.unpack(np.asarray(result['initial_vector']));c,d=p.unpack(np.asarray(result['candidate_vector']))
        from reconciliation.se2 import wrap_angle
        changed=dict(max_position_change_m=float(np.max(np.linalg.norm(c[:,:2]-a[:,:2],axis=1))),
            max_yaw_change_rad=float(np.max(np.abs(wrap_angle(c[:,2]-a[:,2])))),
            max_linear_velocity_change_m_s=float(np.max(np.abs(d[:,0]-b[:,0]))),
            max_angular_velocity_change_rad_s=float(np.max(np.abs(d[:,2]-b[:,2]))))
    return dict(termination=result['termination'],solver_success=result['solver_success'],iterations=result['iterations'],
        initial_full_feasible=initial['full_feasible'],final_full_feasible=final['full_feasible'],
        best_retained_dense_feasible=result['candidate_found'],selected_full_feasible=selected_full,
        selected_source=result['selected_iterate'],returned_initial_unchanged=result['returned_initial_unchanged'],
        infeasible_start_recovered=bool(not initial['full_feasible'] and selected_full),
        feasible_seed_objective_improved=bool(initial['full_feasible'] and selected_full and delta>threshold),
        solver_converged_with_full_feasibility=bool(result['solver_success'] and final['full_feasible']),
        initial_objective=initial['objective'],final_objective=final['objective'],selected_objective=result['candidate_objective'],
        objective_reduction=delta,relative_objective_reduction=None if delta is None else delta/max(1,abs(initial['objective'])),
        numerical_improvement_threshold=threshold,candidate_change=changed,
        objective_calls=result['objective_evaluations'],equality_calls=result['equality_evaluations'],
        inequality_calls=result['inequality_evaluations'],derivative_calls=result['derivative_calls'],
        primal_cache_misses=result['profiling']['evaluator_cache_misses'],
        solve_wall_time_s=result['solve_wall_time_s'],post_solve_validation_time_s=result['post_solve_validation_time_s'],
        independent_full_validation_time_s=full_seconds,optimality_diagnostic_time_s=None,
        navigation_improvement_claim=False)


def solve(run):
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
    from reconciliation.gp_se2_diag02_solver import run_instrumented
    cfg,src=settings(run);frozen=read(run/'comparison_freeze.json')
    for path,h in frozen['code_sha256'].items():
        if digest(ROOT/path)!=h:raise ValueError('implementation changed after comparison freeze: '+path)
    if digest(run/'derivative_checks/authoritative_validation.json')!=frozen['derivative_validation_sha256']:
        raise ValueError('derivative validation changed')
    if not read(run/'derivative_checks/authoritative_validation.json').get('valid',False):raise ValueError('unverified derivatives')
    write(run/'execution_started.json',dict(utc=utc(),order=cfg['solve_order'],retry_allowed=False))
    start=time.perf_counter();env=HospitalEnvironment.load(src['environment']);env_seconds=time.perf_counter()-start
    summaries=[]
    for index,(method,mode,initialization) in enumerate(cfg['solve_order']):
        out=run/'actual_event'/method/mode/initialization;out.mkdir(parents=True,exist_ok=False)
        begin=time.perf_counter();case=load_frozen_case(src['primary_run'],cfg['actual_case_id'],method,environment=env)
        z=np.load(run/'seeds'/f'{initialization}.npy',allow_pickle=False)
        case_seed_seconds=time.perf_counter()-begin
        p=case['problem'];value=p.evaluate(z)
        assert len(z)==150 and len(value['equality'])==30 and len(value['inequality'])==(903 if p.include_obstacles else 813)
        provider=None;graph_seconds=warmup_seconds=0.
        if mode=='SUPPLIED_JAC':
            begin=time.perf_counter();geometry=EnvironmentDerivatives(env,case['config']['footprint']['radius_m'])
            provider=DerivativeProvider(p,geometry);graph_seconds=time.perf_counter()-begin
            begin=time.perf_counter();provider.warmup(z);warmup_seconds=time.perf_counter()-begin
            provider.reset_stats(clear_cache=True)
        write(out/'setup.json',dict(execution_index=index,method=method,mode=mode,initialization=initialization,
            shared_environment_loading_s=env_seconds,case_loading_seed_preparation_s=case_seed_seconds,
            derivative_graph_construction_s=graph_seconds,compilation_first_call_warmup_s=warmup_seconds,
            seed_sha256=digest(run/'seeds'/f'{initialization}.npy'),
            primary_timing='prepared functions, 30s solve; cold costs separate',
            derivative_cache='per provider, one exact float64 vector, shared across 3 callbacks; cleared after warmup',
            original_primal_cache='same GPProblem last-vector cache in both modes'))
        solve_started_utc=utc();solve_started_monotonic=time.monotonic()
        result=run_instrumented(p,z,initialization_name=initialization,derivative_provider=provider)
        write(out/'execution_timing.json',dict(start_utc=solve_started_utc,end_utc=utc(),
            start_monotonic_s=solve_started_monotonic,end_monotonic_s=time.monotonic(),
            scope='harness setup + prepared solve + post-solve dense checks; independent full checks follow'))
        write(out/'solver_result.json',result)
        summary=postprocess(case,result,out)
        summary.update(method=method,derivative_mode=mode,initialization=initialization,execution_index=index,
            derivative_graph_construction_s=graph_seconds,compilation_first_call_warmup_s=warmup_seconds,
            case_loading_seed_preparation_s=case_seed_seconds,optimality_provider_setup_s=None,
            shared_environment_loading_s=env_seconds,
            cold_setup_solve_validation_s=case_seed_seconds+graph_seconds+warmup_seconds+result['setup_wall_time_s']+
                result['solve_wall_time_s']+result['post_solve_validation_time_s']+summary['independent_full_validation_time_s'],
            cold_total_excludes='shared environment loading (reported once) and optional post-solve optimality diagnostics')
        write(out/'result_summary.json',summary);summaries.append(summary)
        print(json.dumps(summary),flush=True)
    write(run/'execution_completed.json',dict(utc=utc(),solve_count=len(summaries),shared_environment_loading_s=env_seconds,
        exact_order=cfg['solve_order'],solves=summaries))


def aggregate(run):
    cfg,src=settings(run);completed=read(run/'execution_completed.json')
    if completed['solve_count']!=8:raise ValueError('all eight prescribed starts required')
    rows=completed['solves'];paired=[]
    # Local optimality is evaluated only after all eight primary solves finish.
    from reconciliation.gp_se2_environment import HospitalEnvironment
    from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
    from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider
    from reconciliation.gp_se2_diag02_optimality import first_order_diagnostic
    env=HospitalEnvironment.load(src['environment'])
    for row in rows:
        out=run/'actual_event'/row['method']/row['derivative_mode']/row['initialization']
        result=read(out/'solver_result.json')
        case=load_frozen_case(src['primary_run'],cfg['actual_case_id'],row['method'],environment=env)
        p=case['problem'];begin=time.perf_counter()
        provider=DerivativeProvider(p,EnvironmentDerivatives(env,case['config']['footprint']['radius_m']))
        provider.warmup(result['initial_vector']);setup_seconds=time.perf_counter()-begin
        diagnostics={}
        for label,z in [('initial',result['initial_vector']),('final',result['latest_iterate'])]:
            try:diagnostics[label]=first_order_diagnostic(p,provider,z,
                qp_multipliers=result['scipy_qp_multipliers'] if label=='final' else None)
            except (ValueError,FloatingPointError,np.linalg.LinAlgError,RuntimeError) as exc:
                diagnostics[label]=dict(status='UNSUPPORTED',error=f'{type(exc).__name__}: {exc}')
        write(out/'optimality.json',dict(**diagnostics,provider_setup_s=setup_seconds,
            timing_scope='optional local optimality diagnostics after all eight primary solves'))
        row['optimality_provider_setup_s']=setup_seconds
        row['optimality_diagnostic_time_s']=sum(v.get('wall_time_s',0.) for v in diagnostics.values())
    for method in cfg['methods']:
        for init in cfg['initializations']:
            a,b=[next(r for r in rows if r['method']==method and r['initialization']==init and r['derivative_mode']==mode)
                for mode in cfg['derivative_modes']]
            pair=dict(method=method,initialization=init)
            for k in ('termination','selected_objective','selected_full_feasible','returned_initial_unchanged',
                      'infeasible_start_recovered','feasible_seed_objective_improved','solver_converged_with_full_feasibility',
                      'solve_wall_time_s','objective_calls','primal_cache_misses','cold_setup_solve_validation_s'):
                pair['fd_'+k]=a[k];pair['supplied_'+k]=b[k]
            pair.update(solve_time_ratio_supplied_over_fd=b['solve_wall_time_s']/a['solve_wall_time_s'],
                objective_call_ratio_supplied_over_fd=b['objective_calls']/max(1,a['objective_calls']),
                primal_miss_ratio_supplied_over_fd=b['primal_cache_misses']/max(1,a['primal_cache_misses']),
                compute_cost_reduced=bool(b['solve_wall_time_s']<a['solve_wall_time_s'] and b['primal_cache_misses']<a['primal_cache_misses']),
                compute_cost_reduced_definition='both observed prepared solve wall time and original primal cache misses decrease; one run, not statistical claim')
            paired.append(pair)
    folder=run/'aggregate';folder.mkdir(exist_ok=False)
    table(folder/'all_solves.csv',[{k:v for k,v in r.items() if not isinstance(v,(dict,list))} for r in rows])
    table(folder/'paired_comparison.csv',paired)
    table(folder/'candidate_acceptance.csv',[{k:r[k] for k in ('method','derivative_mode','initialization','initial_full_feasible',
        'final_full_feasible','best_retained_dense_feasible','selected_full_feasible','selected_source','returned_initial_unchanged',
        'infeasible_start_recovered','feasible_seed_objective_improved','solver_converged_with_full_feasibility')} for r in rows])
    derivative=read(run/'derivative_checks/authoritative_validation.json')
    coverage=[]
    attempt=run/'derivative_checks'/derivative['authoritative_attempt']
    for path in sorted(attempt.glob('*.json')):
        item=read(path)
        if 'name' not in item:continue
        if item.get('expected_unsupported_rejected') or item.get('unsupported_expected'):
            coverage.append(dict(point=item['name'],kind='explicit unsupported cut rejection',family='relative_log_wrap_cut',passed=item['valid']))
        for kind in ('primal_reports','coordinate_reports'):
            for r in item.get(kind,[]):coverage.append(dict(point=item['name'],kind=kind,**r))
        for block in item.get('directional',[]):
            for r in block['smooth_reports']:coverage.append(dict(point=item['name'],kind='directional',step=block['step'],**r))
    table(folder/'derivative_coverage.csv',coverage)
    supplied=[r for r in rows if r['derivative_mode']=='SUPPLIED_JAC']
    summary=dict(experiment='GP-SE2-DIAG-02',operational_status='GP_SE2_DIAG_02_COMPLETED_WITH_LIMITATIONS',
        derivative_validation='VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS',actual_case_id=cfg['actual_case_id'],
        actual_solve_count=len(rows),all_solves=rows,paired_comparison=paired,
        numerical_outcome_flags=dict(infeasible_start_recovered=any(r['infeasible_start_recovered'] for r in supplied),
            feasible_seed_objective_improved=any(r['feasible_seed_objective_improved'] for r in supplied),
            solver_converged_with_full_feasibility=any(r['solver_converged_with_full_feasibility'] for r in supplied),
            returned_initial_unchanged=any(r['returned_initial_unchanged'] for r in supplied),
            compute_cost_reduced=any(r['compute_cost_reduced'] for r in paired)),
        flag_aggregation='any SUPPLIED_JAC start (compute flag any matched pair); inspect all per-start flags, not universal outcome',
        derivative_gate=derivative,shared_environment_loading_s=completed['shared_environment_loading_s'],
        input_source_sha256=digest(run/'source.json'),comparison_freeze_sha256=digest(run/'comparison_freeze.json'),
        navigation_improvement_claim=False,new_mpc_rollouts=0,new_inference=0,
        limitations=['one fixed benign event, one start per matrix entry',
            'piecewise/generalized derivative branches; gates outside this no-gate event unsupported',
            'finite dense/offset acceptance is not a continuous-time safety proof',
            'prepared solve budget excludes disclosed cold compilation/setup',
            'internal QP multipliers and local stationarity are not global optimality proofs'])
    write(run/'summary.json',summary)
    print(json.dumps(dict(status=summary['operational_status'],flags=summary['numerical_outcome_flags'])),flush=True)


def finalize(run):
    cfg,src=settings(run);before=read(run/'preservation_before.json');after=preservation(cfg)
    if not after['valid']:raise ValueError('historical files changed')
    for key in ('historical_files','external_files','previous_diagnostic_files','user_config_hashes','external_git_sha','external_git_status'):
        if after[key]!=before[key]:raise ValueError('preservation difference: '+key)
    write(run/'preservation_after.json',after)
    write(run/'implementation_final.json',dict(created_utc=utc(),source_sha256=snapshot(run,'final')))
    files={str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file()
        and p.name not in ('artifact_manifest.json','validation.json','git_completion.json')}
    write(run/'artifact_manifest.json',dict(created_utc=utc(),files=files,generated_only=True))
    print(json.dumps(dict(status='ARTIFACTS_FROZEN',files=len(files))),flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('phase',choices=['prepare','freeze','solve','aggregate','finalize'])
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/gp_se2_diag_02.yaml')
    args=parser.parse_args();run=args.run.resolve()
    if args.phase=='prepare':prepare(run,args.config)
    elif args.phase=='freeze':freeze(run)
    elif args.phase=='solve':solve(run)
    elif args.phase=='aggregate':aggregate(run)
    elif args.phase=='finalize':finalize(run)


if __name__=='__main__':main()
