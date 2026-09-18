"""Supplied AD/analytic derivatives for the unchanged GP-SE2-01 chart.

JAX forward AD differentiates the CPU float64 GP objective, equality, motion,
goal, and world-XY interpolation. Explicit analytic environment gradients are
chained through that XY Jacobian. All three SciPy callbacks are supplied; no
finite-difference fallback exists. The source GPProblem is still the primal
solver evaluator. Gates are outside this fixed-event diagnostic and rejected.
"""
from __future__ import annotations

import copy
import hashlib
import json
import time

import numpy as np

from .gp_se2_formulation import GPProblem
from .se2 import relative_pose,wrap_angle


class DerivativeError(RuntimeError):
    """Unsupported, nonfinite, or failed derivative evaluation (never FD fallback)."""

    def __init__(self,message,*,reason_code='DERIVATIVE_EVALUATION_ERROR',diagnostics=None):
        super().__init__(message)
        self.reason_code=reason_code
        self.diagnostics=diagnostics


class DerivativeProvider:
    def __init__(self,problem:GPProblem,environment_derivatives):
        start=time.perf_counter()
        if not isinstance(problem,GPProblem):
            raise TypeError('DerivativeProvider requires the original GPProblem')
        if problem.gates:
            raise DerivativeError('gate derivatives are unsupported in this fixed no-gate event diagnostic')
        if len(problem.times)!=31 or problem.config['horizon_s']!=3. or problem.config['support_dt_s']!=.1:
            raise DerivativeError('provider scope is the original 150-variable, 31-support, 3-second problem')
        for family,applicable in [('workspace',problem.workspace_margin is not None),('obstacle',problem.include_obstacles)]:
            if applicable and (environment_derivatives is None or not callable(getattr(environment_derivatives,family,None))):
                raise DerivativeError(f'analytic {family} derivatives are required')
        try:
            import jax
            from .gp_se2_diag02_ad import make_core
        except ImportError as exc:
            raise DerivativeError('CPU JAX is required; no numerical fallback is available') from exc
        if not jax.config.jax_enable_x64 or jax.default_backend()!='cpu':
            raise DerivativeError('provider requires JAX float64 and CPU backend')
        self.problem=problem
        self.environment_derivatives=environment_derivatives
        self.variable_count=150
        self.equality_count=30
        self.core_inequality_count=8*90+3
        self.collocation_count=90
        self.inequality_count=self.core_inequality_count+90*int(problem.workspace_margin is not None)+90*int(problem.include_obstacles)
        self._jax=jax
        self._jitted=jax.jit(jax.jacfwd(make_core(problem),has_aux=True))
        self._compiled=None
        self._cache_x=None
        self._cache=None
        self._compile_seconds=0.
        self._warmup_seconds=0.
        self._compile_count=0
        self._warmup_count=0
        self._runtime_reset_count=0
        self._unsupported_wrap_cut_events=[]
        self._buckets={name:self._empty_bucket() for name in ('warmup','runtime')}
        self._branch_data={name:self._empty_branches() for name in ('warmup','runtime')}
        self._branch_seen={name:set() for name in ('warmup','runtime')}
        self._callback_calls={'values':0,'objective_gradient':0,'equality_jacobian':0,'inequality_jacobian':0,'geometry_metadata':0}
        self._callback_seconds={name:0. for name in self._callback_calls}
        self._setup_seconds=time.perf_counter()-start

    @staticmethod
    def _empty_bucket():
        return dict(requests=0,cache_hits=0,cache_misses=0,core_ad_evaluations=0,
                    core_ad_seconds=0.,environment_calls=0,environment_seconds=0.,
                    geometry_chain_seconds=0.,branch_recording_seconds=0.,wrap_cut_guard_seconds=0.,
                    derivative_cache_seconds=0.,total_seconds=0.)

    def _guard_wrap_cuts(self,vector,phase):
        """Reject unsupported Log/goal branch cuts before using any Jacobian.

        World pose yaw can cross its representation cut smoothly. Only relative
        logarithm and wrapped-goal residual cuts are excluded here. The 1e-12 rad
        band identifies roundoff around a nondifferentiable cut; it never alters
        a physical acceptance tolerance or original primal calculation.
        """
        start=time.perf_counter();poses,_=self.problem.unpack(vector);cuts=[];tolerance=1e-12
        groups=[('adjacent_relative_log',relative_pose(poses[:-1],poses[1:])[:,2]),
                ('fresh_relative_log',relative_pose(self.problem.common_reference,poses[1:])[:,2]),
                ('goal_yaw',np.atleast_1d(wrap_angle(poses[-1,2]-self.problem.goal_pose[2])))]
        for family,yaw in groups:
            distance=np.abs(np.abs(yaw)-np.pi)
            for index in np.flatnonzero(distance<=tolerance):
                entry=dict(family=family,yaw_rad=float(yaw[index]),distance_to_cut_rad=float(distance[index]))
                if family=='adjacent_relative_log':entry['interval_index']=int(index)
                else:entry['support_index']=30 if family=='goal_yaw' else int(index)+1
                cuts.append(entry)
        self._buckets[phase]['wrap_cut_guard_seconds']+=time.perf_counter()-start
        if cuts:
            details=dict(cuts=cuts,cut_tolerance_rad=tolerance,phase=phase,
                         no_classical_or_limiting_derivative_asserted=True)
            self._unsupported_wrap_cut_events.append(copy.deepcopy(details))
            raise DerivativeError('unsupported relative/goal yaw wrap cut; no supplied Jacobian is valid here',
                                  reason_code='UNSUPPORTED_WRAP_CUT',diagnostics=details)

    @staticmethod
    def _empty_branches():
        return dict(families={},samples=[],sample_limit=128,distinct_flagged_locations=0,
                    omitted_distinct_locations=0,scope='actual analytic derivative evaluations; cache hits do not query geometry')

    def _record_geometry(self,phase,family,xy,metadata):
        """Aggregate actual nonsmooth/penalty branches in memory; no per-call I/O."""
        begin=time.perf_counter();data=self._branch_data[phase]
        counts=data['families'].setdefault(family,dict(query_calls=0,point_evaluations=0,
            nonsmooth_points=0,invalid_points=0,points_without_metadata=0,branch_counts={}))
        counts['query_calls']+=1;counts['point_evaluations']+=len(xy)
        points=metadata.get('points',[]) if isinstance(metadata,dict) else []
        counts['points_without_metadata']+=max(0,len(xy)-len(points))
        for coordinate,record in zip(xy,points):
            nonsmooth=record.get('smooth') is False
            invalid=bool(record.get('invalid_query',False))
            kind=str(record.get('kind','UNSPECIFIED'))
            counts['branch_counts'][kind]=counts['branch_counts'].get(kind,0)+1
            counts['nonsmooth_points']+=int(nonsmooth);counts['invalid_points']+=int(invalid)
            if not (nonsmooth or invalid):
                continue
            sample=dict(family=family,world_xy_m=np.asarray(coordinate).tolist(),
                        nonsmooth=nonsmooth,invalid_query=invalid,metadata=record)
            identity=hashlib.blake2b(json.dumps(sample,sort_keys=True,allow_nan=False).encode(),digest_size=16).digest()
            if identity not in self._branch_seen[phase]:
                self._branch_seen[phase].add(identity)
                if len(data['samples'])<data['sample_limit']:
                    data['samples'].append(copy.deepcopy(sample))
        data['distinct_flagged_locations']=len(self._branch_seen[phase])
        data['omitted_distinct_locations']=max(0,len(self._branch_seen[phase])-data['sample_limit'])
        self._buckets[phase]['branch_recording_seconds']+=time.perf_counter()-begin

    def _vector(self,z):
        vector=np.asarray(z,dtype=np.float64)
        if vector.shape!=(150,) or not np.all(np.isfinite(vector)):
            raise DerivativeError('derivative vector must be finite original-chart shape (150,)')
        return vector

    def _compile(self,z):
        if self._compiled is None:
            started=time.perf_counter()
            try:
                self._compiled=self._jitted.lower(z).compile()
            except Exception as exc:
                raise DerivativeError(f'JAX compilation failed: {type(exc).__name__}: {exc}') from exc
            self._compile_seconds+=time.perf_counter()-started
            self._compile_count+=1

    def clear_cache(self):
        self._cache_x=None
        self._cache=None

    def reset_stats(self,*,clear_cache=True):
        """Reset solve counters while retaining independently reported setup costs."""
        self._buckets['runtime']=self._empty_bucket()
        self._branch_data['runtime']=self._empty_branches()
        self._branch_seen['runtime']=set()
        self._callback_calls={name:0 for name in self._callback_calls}
        self._callback_seconds={name:0. for name in self._callback_seconds}
        self._runtime_reset_count+=1
        self._unsupported_wrap_cut_events=[]
        if clear_cache:
            self.clear_cache()

    def warmup(self,z):
        """Compile and synchronize one evaluation; discard its numerical cache.

        Setup/warmup is separately charged by the caller. Clearing the cache
        prevents an initial solve Jacobian from being obtained for free.
        """
        started=time.perf_counter()
        vector=self._vector(z)
        self._guard_wrap_cuts(vector,'warmup')
        self._compile(vector)
        self._ensure(vector,phase='warmup')
        self.clear_cache()
        self._warmup_seconds+=time.perf_counter()-started
        self._warmup_count+=1
        return self.stats()

    def _ensure(self,z,*,phase='runtime'):
        vector=self._vector(z)
        bucket=self._buckets[phase]
        started=time.perf_counter();bucket['requests']+=1
        check_start=time.perf_counter()
        hit=self._cache_x is not None and np.array_equal(vector,self._cache_x)
        bucket['derivative_cache_seconds']+=time.perf_counter()-check_start
        if hit:
            bucket['cache_hits']+=1
            bucket['total_seconds']+=time.perf_counter()-started
            return self._cache
        bucket['cache_misses']+=1
        self._guard_wrap_cuts(vector,phase)
        self._compile(vector)
        core_start=time.perf_counter()
        try:
            jacobian,(packed,aux)=self._compiled(vector)
            # NumPy conversion synchronizes CPU execution before timing stops.
            jacobian=np.asarray(jacobian,dtype=np.float64)
            packed=np.asarray(packed,dtype=np.float64)
            aux={key:np.asarray(value,dtype=np.float64) for key,value in aux.items()}
        except Exception as exc:
            raise DerivativeError(f'JAX derivative evaluation failed: {type(exc).__name__}: {exc}') from exc
        bucket['core_ad_seconds']+=time.perf_counter()-core_start
        bucket['core_ad_evaluations']+=1
        if not np.all(np.isfinite(packed)) or not np.all(np.isfinite(jacobian)):
            raise DerivativeError('nonfinite JAX primal or Jacobian; no fallback')
        eq_end=1+self.equality_count
        iq_end=eq_end+self.core_inequality_count
        xy=packed[iq_end:].reshape((90,2))
        xy_jac=jacobian[iq_end:].reshape((90,2,150))
        inequalities=[packed[eq_end:iq_end]]
        inequality_jacobians=[jacobian[eq_end:iq_end]]
        metadata={}
        for family,applicable in [('workspace',self.problem.workspace_margin is not None),
                                  ('obstacle',self.problem.include_obstacles)]:
            if not applicable:
                continue
            env_start=time.perf_counter()
            try:
                values,gradients,details=getattr(self.environment_derivatives,family)(xy)
            except Exception as exc:
                raise DerivativeError(f'analytic {family} derivative failed: {type(exc).__name__}: {exc}') from exc
            bucket['environment_seconds']+=time.perf_counter()-env_start
            bucket['environment_calls']+=1
            values=np.asarray(values,dtype=np.float64);gradients=np.asarray(gradients,dtype=np.float64)
            if values.shape!=(90,) or gradients.shape!=(90,2) or not np.all(np.isfinite(values)) or not np.all(np.isfinite(gradients)):
                raise DerivativeError(f'analytic {family} output must be finite values (90,) and gradients (90,2)')
            self._record_geometry(phase,family,xy,details)
            chain_start=time.perf_counter()
            if family=='obstacle':
                values=values-self.problem.config['required_clearance']
            inequalities.append(values)
            inequality_jacobians.append(np.einsum('ni,nij->nj',gradients,xy_jac))
            metadata[family]=details
            bucket['geometry_chain_seconds']+=time.perf_counter()-chain_start
        values=dict(aux,objective=float(packed[0]),equality=packed[1:eq_end].copy(),
                    inequality=np.concatenate(inequalities),collocation_xy=xy.copy())
        cache=dict(values=values,objective_gradient=jacobian[0].copy(),
                   equality_jacobian=jacobian[1:eq_end].copy(),
                   inequality_jacobian=np.concatenate(inequality_jacobians,axis=0),
                   geometry_metadata=metadata)
        if not np.all(np.isfinite(cache['inequality_jacobian'])):
            raise DerivativeError('nonfinite chained environment Jacobian; no fallback')
        cache_start=time.perf_counter()
        self._cache_x=vector.copy();self._cache=cache
        bucket['derivative_cache_seconds']+=time.perf_counter()-cache_start
        bucket['total_seconds']+=time.perf_counter()-started
        return cache

    def _callback(self,z,name):
        started=time.perf_counter();self._callback_calls[name]+=1
        try:
            value=self._ensure(z)[name]
            return copy.deepcopy(value) if isinstance(value,dict) else value.copy()
        finally:
            self._callback_seconds[name]+=time.perf_counter()-started

    def values(self,z):
        return self._callback(z,'values')

    def objective_gradient(self,z):
        return self._callback(z,'objective_gradient')

    def equality_jacobian(self,z):
        return self._callback(z,'equality_jacobian')

    def inequality_jacobian(self,z):
        return self._callback(z,'inequality_jacobian')

    def geometry_metadata(self,z):
        return self._callback(z,'geometry_metadata')

    def stats(self):
        return dict(backend='JAX CPU float64 forward automatic differentiation + analytic environment chain',
                    jax_version=self._jax.__version__,float64_enabled=bool(self._jax.config.jax_enable_x64),
                    device_backend=self._jax.default_backend(),variable_count=self.variable_count,
                    equality_count=self.equality_count,inequality_count=self.inequality_count,
                    setup_wall_s=self._setup_seconds,compile_wall_s=self._compile_seconds,
                    compile_count=self._compile_count,warmup_wall_s=self._warmup_seconds,warmup_count=self._warmup_count,
                    runtime_reset_count=self._runtime_reset_count,
                    callback_calls=dict(self._callback_calls),callback_seconds=dict(self._callback_seconds),
                    phases=copy.deepcopy(self._buckets),
                    geometry_branches=copy.deepcopy(self._branch_data),
                    unsupported_wrap_cut_events=copy.deepcopy(self._unsupported_wrap_cut_events),
                    numerical_finite_difference_calls=0,numerical_fallback_used=False,
                    cache_scope='one complete exact float64 vector, shared by all three supplied derivative callbacks',
                    timing_scope='phase total includes core, environment, chaining, cache; callback times are inclusive; warmup includes compilation',
                    unsupported_constraints=['gates'],geometry_branch_scope='selected analytic piecewise branch, metadata retained')
