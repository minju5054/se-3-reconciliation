"""M4: unchanged GP chart/motion/environment plus explicit forward attachment.

Original evaluator remains source of all original constraints and GP prior.
Only FRESH cost masking/correspondence and four support tube sites are changed.
No event-specific motion witness or endpoint reserve is imported.
"""
import time
import numpy as np
from .se2 import relative_pose, se2_log, wrap_angle
from .gp_se2_join01 import post_join_reference, tube_margins, POSITION_M, YAW_RAD, DWELL_S
from .gp_se2_diag02_derivatives import DerivativeError


class JoinView:
    def __init__(self, base, fresh, candidate):
        self.base = self.base_problem = base
        self.config, self.times = base.config, base.times
        self.grid_name = 'M4_JOIN_GP'
        self.candidate = dict(candidate)
        self.k = candidate['support_index']
        self.reference = post_join_reference(fresh, candidate, self.times)
        self.tube_count = round(DWELL_S/base.config['support_dt_s'])+1
        self.clear_cache()

    def clear_cache(self):
        self.base._last_x = self.base._last_eval = None
        self._last_x = self._last_eval = None

    def unpack(self, z):
        return self.base.unpack(z)

    def evaluate(self, z):
        if self._last_x is not None and np.array_equal(z, self._last_x):
            return self._last_eval
        original = self.base.evaluate(z)
        p = original['poses'][self.k:]
        residual = se2_log(relative_pose(self.reference, p))/self.config['fresh_std']
        fresh_cost = float(np.mean(np.sum(residual**2, axis=1)))
        tube = tube_margins(p[:self.tube_count], self.reference[:self.tube_count])
        out = dict(original, fresh_cost=fresh_cost,
                   objective=float(np.sum(original['gp_factor_costs'])+self.config['lambda_fresh']*fresh_cost),
                   inequality=np.r_[original['inequality'],tube], join_margins=tube)
        self._last_x, self._last_eval = np.array(z,copy=True), out
        return out

    def independent_tube_check(self, z):
        p,_ = self.base.unpack(z)
        a = p[self.k:self.k+self.tube_count]; r = self.reference[:self.tube_count]
        d = np.linalg.norm(a[:,:2]-r[:,:2],axis=1); yaw = np.abs(wrap_angle(a[:,2]-r[:,2]))
        return dict(valid=bool(np.all(d<=POSITION_M) and np.all(yaw<=YAW_RAD)),
                    distance_m=d, yaw_error_rad=yaw,
                    times_s=self.times[self.k:self.k+self.tube_count],
                    numerical_margin_policy='nominal direct tube check; original solver residual tolerance separately recorded',
                    continuous_time_proof=False)


class JoinDerivatives:
    """Original full Jacobians + CPU AD of changed cost and appended tube rows."""
    def __init__(self, view, base_provider):
        from . import gp_se2_diag02_ad as ad
        import jax
        import jax.numpy as jnp
        self.view, self.base_provider = view, base_provider
        self._elapsed = 0.; self._calls = 0
        b = view.base
        anchors=jnp.asarray(b._anchors[1:]); initial=jnp.asarray(b.boundary_pose)
        old=jnp.asarray(b.common_reference); ref=jnp.asarray(view.reference)
        std=jnp.asarray(b.config['fresh_std']); k=view.k; count=view.tube_count
        def extra(z):
            p=jnp.concatenate((initial[None,:],ad.retract_pose(anchors,z.reshape((-1,5))[:,:3])))
            old_r=ad.se2_log(ad.relative_pose(old,p[1:]))/std
            new_r=ad.se2_log(ad.relative_pose(ref,p[k:]))/std
            change=b.config['lambda_fresh']*(jnp.mean(jnp.sum(new_r**2,axis=1))-jnp.mean(jnp.sum(old_r**2,axis=1)))
            yaw=ad.wrap_angle(p[k:k+count,2]-ref[:count,2])
            margins=jnp.concatenate((POSITION_M**2-jnp.sum((p[k:k+count,:2]-ref[:count,:2])**2,axis=1),YAW_RAD-yaw,YAW_RAD+yaw))
            values=jnp.concatenate((change[None],margins))
            return values,values
        self._f=jax.jit(jax.jacfwd(extra,has_aux=True))
        self._z=self._data=None

    def _ensure(self,z):
        if self._z is None or not np.array_equal(z,self._z):
            p,_=self.view.base.unpack(z)
            angles=wrap_angle(p[self.view.k:,2]-self.view.reference[:,2])
            if np.any(np.abs(np.abs(angles)-np.pi)<=1e-12):
                raise DerivativeError('unsupported JOIN relative yaw cut')
            start=time.perf_counter();jac,values=self._f(np.asarray(z,float))
            self._data=(np.asarray(jac),np.asarray(values));self._z=np.array(z,copy=True)
            if not all(np.isfinite(a).all() for a in self._data):
                raise DerivativeError('nonfinite JOIN derivatives')
            self._elapsed+=time.perf_counter()-start;self._calls+=1
        return self._data

    def warmup(self,z):
        self.base_provider.warmup(z);self._ensure(z);self.clear_cache()
        return self.stats()

    def clear_cache(self):
        self.base_provider.clear_cache();self._z=self._data=None

    def objective_gradient(self,z):
        return self.base_provider.objective_gradient(z)+self._ensure(z)[0][0]

    def equality_jacobian(self,z):
        return self.base_provider.equality_jacobian(z)

    def inequality_jacobian(self,z):
        return np.vstack((self.base_provider.inequality_jacobian(z),self._ensure(z)[0][1:]))

    def values(self,z):
        base=self.base_provider.values(z);_,extra=self._ensure(z)
        return dict(base,objective=base['objective']+extra[0],inequality=np.r_[base['inequality'],extra[1:]])

    def stats(self):
        return dict(base=self.base_provider.stats(),join_ad_calls=self._calls,join_ad_inclusive_s=self._elapsed,
                    finite_difference_fallback=False,additional_equality_rows=0,additional_inequality_rows=12)


def verify_derivatives(view, provider, vector):
    """Frozen three-direction, three-step gate; no solver or adaptive tolerance."""
    from .gp_se2_diag02_validation import PROTOCOL,error_report
    z=np.asarray(vector,float); primal=view.evaluate(z); supplied=provider.values(z)
    reports={f:error_report(np.atleast_1d(supplied[f]),np.atleast_1d(primal[f]),PROTOCOL['primal_tolerance']) for f in ('objective','equality','inequality')}
    rng=np.random.default_rng(PROTOCOL['random_seed']);rows=[]
    for di in range(PROTOCOL['direction_count']):
        d=rng.normal(size=len(z));d/=np.linalg.norm(d)
        for step in PROTOCOL['directional_fd_steps']:
            plus=view.evaluate(z+step*d);minus=view.evaluate(z-step*d)
            for field,callback in [('objective','objective_gradient'),('equality','equality_jacobian'),('inequality','inequality_jacobian')]:
                actual=getattr(provider,callback)(z)@d
                fd=(np.asarray(plus[field])-np.asarray(minus[field]))/(2*step)
                tolerance=PROTOCOL['objective_derivative_tolerance' if field=='objective' else 'constraint_derivative_tolerance']
                rows.append(dict(direction=di,step=step,field=field,**error_report(np.atleast_1d(actual),np.atleast_1d(fd),tolerance)))
    passed=all(r['passed'] for r in reports.values()) and all(r['passed'] for r in rows if r['step'] in PROTOCOL['directional_fd_steps'][-2:])
    return dict(passed=passed,primal=reports,directional=rows,protocol=PROTOCOL,
                nonsmooth_policy='any failed smooth FD gate blocks that start; no fallback or threshold changes')
