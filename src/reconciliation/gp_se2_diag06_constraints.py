"""G3 = unchanged G2 plus only the frozen signed witness inequality suffix."""
from __future__ import annotations
import copy
import time
import numpy as np
from .gp_se2 import interpolate_interval
from .gp_se2_diag04_constraints import MOTION_ROWS, _vector
from .gp_se2_diag05_constraints import InequalityOnlyView
from .gp_se2_diag02_derivatives import DerivativeError

G3='G3_EXTREMUM_WITNESS_INEQUALITY'
COMPONENT={'v_x':0,'omega':2,'a_x':3,'alpha':5}


def specifications(rows, config):
    families={r[0]:r for r in MOTION_ROWS}
    specs=[families[r['family']] for r in rows]
    return (np.array([r['interval_index'] for r in rows],int),
        np.array([r['local_fraction'] for r in rows],np.float64),
        np.array([COMPONENT[s[1]] for s in specs],int),
        np.array([s[3] for s in specs],np.float64),
        np.array([0. if s[5] is None else config[s[5]] for s in specs],np.float64))


def witness_values(problem, vector, rows):
    z=_vector(problem,vector)
    if not rows:return np.empty(0,np.float64)
    p,v=problem.unpack(z);i,u,component,sign,offset=specifications(rows,problem.config)
    _,vv,aa=interpolate_interval(p[i],v[i],p[i+1],v[i+1],problem.config['support_dt_s'],u)
    result=np.concatenate((vv,aa),axis=1)[np.arange(len(i)),component]*sign+offset
    if not np.isfinite(result).all():raise FloatingPointError('nonfinite witness margin')
    return result


class WitnessView(InequalityOnlyView):
    def __init__(self, base, rows):
        super().__init__(base)
        self.g2=InequalityOnlyView(base)
        self.rows=copy.deepcopy(rows)
        self.grid=self.grid_name=G3

    def evaluate(self, vector):
        z=_vector(self.base,vector)
        if self._view_last_x is not None and np.array_equal(z,self._view_last_x):return self._view_last_eval
        original=self.g2.evaluate(z);started=time.perf_counter()
        values=witness_values(self.base,z,self.rows)
        self._witness_seconds+=time.perf_counter()-started;self._witness_calls+=1
        result=dict(original,witness_inequality=values,
            inequality=np.concatenate((original['inequality'],values)))
        self._view_last_x,self._view_last_eval=z.copy(),result
        return result

    def clear_cache(self):
        super().clear_cache();self.g2.clear_cache()

    def reset_stats(self):
        super().reset_stats();self._witness_seconds=0.;self._witness_calls=0
        if hasattr(self,'g2'):self.g2.reset_stats()

    def stats(self):
        return dict(self.g2.stats(),witness_primal_interpolation_calls=self._witness_calls,
            witness_primal_interpolation_wall_s=self._witness_seconds,witness_count=len(self.rows))

    def appended_row_metadata(self, vector):
        result=self.g2.appended_row_metadata(vector);offset=len(self.g2.evaluate(vector)['inequality'])
        values=self.evaluate(vector)['witness_inequality'];families={r[0]:r for r in MOTION_ROWS}
        for row,value in zip(self.rows,values):
            family,quantity,unit,sign,expression,limit=families[row['family']]
            result.append(dict(row,constraint_kind='inequality',row_index=offset+row['witness_row_index'],
                source='frozen_extremum_witness',physical_quantity=quantity,physical_unit=unit,sign=sign,
                expression=expression,actual_numerical_tolerance=self.config['inequality_tolerance'],
                signed_residual_or_margin=float(value),nominal_limit=0. if limit is None else
                (-1 if family.endswith('_lower') else 1)*self.config[limit]))
        return result


def make_witness_core(problem, rows):
    from . import gp_se2_diag02_ad as ad
    import jax.numpy as jnp
    boundary=jnp.asarray(problem.boundary_pose);initial=jnp.asarray(problem.initial_twist)
    anchors=jnp.asarray(problem._anchors[1:]);h=problem.config['support_dt_s']
    i,u,component,sign,offset=map(jnp.asarray,specifications(rows,problem.config))
    def core(vector):
        z=vector.reshape((-1,5))
        poses=jnp.concatenate((boundary[None],ad.retract_pose(anchors,z[:,:3])))
        twists=jnp.concatenate((initial[None],jnp.stack((z[:,3],jnp.zeros_like(z[:,3]),z[:,4]),axis=-1)))
        _,v,a=ad.interpolate_interval(poses[i],twists[i],poses[i+1],twists[i+1],h,u)
        values=jnp.concatenate((v,a),axis=1)[jnp.arange(len(rows)),component]*sign+offset
        return values,values
    return core


class WitnessDerivatives:
    def __init__(self, view, g2_provider):
        if g2_provider.problem is not view.base:raise ValueError('identical base required')
        begin=time.perf_counter();self.view=view;self.g2=g2_provider
        self.base_provider=g2_provider.base_provider;self.problem=view.base
        self.variable_count=g2_provider.variable_count;self.equality_count=g2_provider.equality_count
        self.inequality_count=g2_provider.inequality_count+len(view.rows)
        self._compiled=None;self._compile_s=0.;self._compile_count=0;self._warmup_s=0.
        self._x=self._cache=None;self._calls=0;self._misses=0;self._ad_s=0.
        self._jitted=self.base_provider._jax.jit(self.base_provider._jax.jacfwd(make_witness_core(view.base,view.rows),has_aux=True)) if view.rows else None
        self._construction_s=time.perf_counter()-begin

    def _ensure(self, vector, phase='runtime'):
        z=self.base_provider._vector(vector);self._calls+=1
        self.base_provider._guard_wrap_cuts(z,phase)
        if self._x is not None and np.array_equal(z,self._x):return self._cache
        self._misses+=1
        if not self.view.rows:return np.empty(0),np.empty((0,self.variable_count))
        if self._compiled is None:
            begin=time.perf_counter()
            try:self._compiled=self._jitted.lower(z).compile()
            except Exception as exc:raise DerivativeError('witness AD compile failed: '+str(exc)) from exc
            self._compile_s+=time.perf_counter()-begin;self._compile_count+=1
        begin=time.perf_counter()
        try:
            jac,values=self._compiled(z);jac=np.asarray(jac,np.float64);values=np.asarray(values,np.float64)
        except Exception as exc:raise DerivativeError('witness AD failed: '+str(exc)) from exc
        self._ad_s+=time.perf_counter()-begin
        if (values.shape!=(len(self.view.rows),) or jac.shape!=(len(self.view.rows),self.variable_count)
            or not np.isfinite(values).all() or not np.isfinite(jac).all()):
            raise DerivativeError('nonfinite or wrong-shaped witness AD; no FD fallback')
        self._x=z.copy();self._cache=(values,jac)
        return self._cache

    def values(self,vector):
        original=self.g2.values(vector);values,_=self._ensure(vector)
        return dict(original,witness_inequality=values,inequality=np.concatenate((original['inequality'],values)))

    def objective_gradient(self,vector):return self.g2.objective_gradient(vector)
    def equality_jacobian(self,vector):return self.g2.equality_jacobian(vector)
    def inequality_jacobian(self,vector):
        return np.concatenate((self.g2.inequality_jacobian(vector),self._ensure(vector)[1]))
    def geometry_metadata(self,vector):return self.g2.geometry_metadata(vector)
    def warmup(self,vector):
        self.g2.warmup(vector);t=time.perf_counter();self._ensure(vector,'warmup');self._warmup_s+=time.perf_counter()-t
        self._x=self._cache=None
        return self.stats()
    def reset_stats(self,*,clear_cache=True):
        self.g2.reset_stats(clear_cache=clear_cache);self._calls=self._misses=0;self._ad_s=0.
        if clear_cache:self._x=self._cache=None
    def stats(self):
        b=self.g2.stats()
        return dict(b,grid=G3,equality_count=self.equality_count,inequality_count=self.inequality_count,
            setup_wall_s=b['setup_wall_s']+self._construction_s,compile_wall_s=b['compile_wall_s']+self._compile_s,
            warmup_wall_s=b['warmup_wall_s']+self._warmup_s,compile_count=b['compile_count']+self._compile_count,
            witness_provider=dict(requests=self._calls,cache_misses=self._misses,ad_wall_s=self._ad_s,
                compile_wall_s=self._compile_s,construction_wall_s=self._construction_s,warmup_wall_s=self._warmup_s,
                cache_scope='one exact float64 vector; witness values/Jacobian',numerical_FD_calls=0),
            timing_scope='inclusive base+quarter+witness; warmup includes compile; do not sum nested costs')
