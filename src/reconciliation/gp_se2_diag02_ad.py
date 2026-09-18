"""Float64 CPU JAX transcription of the frozen GP-SE2-01 mathematics.

This module supplies derivatives only for GP-SE2-DIAG-02. The original NumPy
GPProblem remains the solver's primal evaluator. Arithmetic, wrapping, Taylor
thresholds, residual ordering, and both terms of body acceleration follow the
original modules. No finite differences, alternate trajectory model, or relaxed
physical condition is introduced here. Derivatives at piecewise branch cuts are
the selected branch derivatives, not a claim of global differentiability.
"""
from __future__ import annotations

import jax
jax.config.update('jax_enable_x64', True)
import jax.numpy as jnp
import jax.scipy.linalg as jsl


def wrap_angle(angle):
    return (angle+jnp.pi) % (2*jnp.pi)-jnp.pi


def compose_poses(left, right):
    yaw=left[...,2]
    cosine,sine=jnp.cos(yaw),jnp.sin(yaw)
    x=left[...,0]+cosine*right[...,0]-sine*right[...,1]
    y=left[...,1]+sine*right[...,0]+cosine*right[...,1]
    return jnp.stack((x,y,wrap_angle(yaw+right[...,2])),axis=-1)


def inverse_pose(pose):
    yaw=pose[...,2]
    cosine,sine=jnp.cos(yaw),jnp.sin(yaw)
    x=-cosine*pose[...,0]-sine*pose[...,1]
    y=sine*pose[...,0]-cosine*pose[...,1]
    return jnp.stack((x,y,wrap_angle(-yaw)),axis=-1)


def relative_pose(reference,target):
    return compose_poses(inverse_pose(reference),target)


def _exp_log_coefficients(omega):
    small=jnp.abs(omega)<1e-4
    omega2=omega*omega
    # Inactive divisions have safe denominators, including at omega=0. This
    # matches the original values and prevents 0/0 poisoning AD's inactive path.
    safe=jnp.where(small,1.,omega)
    a=jnp.where(small,1.-omega2/6.+omega2*omega2/120.,jnp.sin(omega)/safe)
    b=jnp.where(small,omega/2.-omega*omega2/24.+omega*omega2*omega2/720.,
                (1.-jnp.cos(omega))/safe)
    return a,b


def se2_exp(tangent):
    omega=tangent[...,2]
    a,b=_exp_log_coefficients(omega)
    x=a*tangent[...,0]-b*tangent[...,1]
    y=b*tangent[...,0]+a*tangent[...,1]
    return jnp.stack((x,y,wrap_angle(omega)),axis=-1)


def se2_log(pose):
    omega=wrap_angle(pose[...,2])
    a,b=_exp_log_coefficients(omega)
    determinant=a*a+b*b
    vx=(a*pose[...,0]+b*pose[...,1])/determinant
    vy=(-b*pose[...,0]+a*pose[...,1])/determinant
    return jnp.stack((vx,vy,omega),axis=-1)


def retract_pose(pose,delta):
    return compose_poses(pose,se2_exp(delta))


def adjoint_algebra(xi):
    zero=jnp.zeros_like(xi[...,0])
    row0=jnp.stack((zero,-xi[...,2],xi[...,1]),axis=-1)
    row1=jnp.stack((xi[...,2],zero,-xi[...,0]),axis=-1)
    row2=jnp.stack((zero,zero,zero),axis=-1)
    return jnp.stack((row0,row1,row2),axis=-2)


def _coefficients(w):
    small=jnp.abs(w)<1e-3
    safe=jnp.where(small,1.,w)
    w2=w*w
    a=jnp.where(small,.5-w2/24+w2*w2/720-w2**3/40320,
                (1-jnp.cos(safe))/safe**2)
    b=jnp.where(small,1/6-w2/120+w2*w2/5040-w2**3/362880,
                (safe-jnp.sin(safe))/safe**3)
    da=jnp.where(small,-w/12+w*w2/180-w*w2*w2/6720,
                 (safe*jnp.sin(safe)-2*(1-jnp.cos(safe)))/safe**3)
    db=jnp.where(small,-w/60+w*w2/1260-w*w2*w2/60480,
                 (safe*(1-jnp.cos(safe))-3*(safe-jnp.sin(safe)))/safe**4)
    return a,b,da,db


def right_jacobian(xi):
    ad=adjoint_algebra(xi)
    a,b,_,_=_coefficients(xi[...,2])
    return jnp.eye(3,dtype=jnp.float64)-a[...,None,None]*ad+b[...,None,None]*(ad@ad)


def right_jacobian_inverse_apply(xi,body_twist):
    return jnp.linalg.solve(right_jacobian(xi),body_twist[...,None])[...,0]


def right_jacobian_directional(xi,direction):
    x,d=jnp.broadcast_arrays(xi,direction)
    ad,dad=adjoint_algebra(x),adjoint_algebra(d)
    a,b,da,db=_coefficients(x[...,2])
    return (-da[...,None,None]*d[...,2,None,None]*ad
            -a[...,None,None]*dad
            +db[...,None,None]*d[...,2,None,None]*(ad@ad)
            +b[...,None,None]*(dad@ad+ad@dad))


def gp_residual(pose0,twist0,pose1,twist1,h):
    xi=se2_log(relative_pose(pose0,pose1))
    d1=right_jacobian_inverse_apply(xi,twist1)
    return jnp.concatenate((xi-h*twist0,d1-twist0),axis=-1)


def interpolate_interval(pose0,twist0,pose1,twist1,h,fraction):
    u=jnp.clip(jnp.asarray(fraction,dtype=jnp.float64),0,1)[...,None]
    x1=se2_log(relative_pose(pose0,pose1))
    d0=twist0
    d1=right_jacobian_inverse_apply(x1,twist1)
    xi=(u**3-2*u**2+u)*h*d0+(-2*u**3+3*u**2)*x1+(u**3-u**2)*h*d1
    dxi=(3*u**2-4*u+1)*d0+(-6*u**2+6*u)*x1/h+(3*u**2-2*u)*d1
    ddxi=(6*u-4)*d0/h+(-12*u+6)*x1/h**2+(6*u-2)*d1/h
    jr=right_jacobian(xi)
    velocity=(jr@dxi[...,None])[...,0]
    acceleration=((right_jacobian_directional(xi,dxi)@dxi[...,None])+jr@ddxi[...,None])[...,0]
    return compose_poses(pose0,se2_exp(xi)),velocity,acceleration


def make_core(problem):
    """Return (packed differentiable outputs, nondifferentiated primal aux).

    Packed order: objective; lateral midpoint equalities; original motion/goal
    inequalities; 90 world XY collocation samples for analytic geometry chaining.
    Gates are rejected by DerivativeProvider before this function is used.
    """
    c=dict(problem.config)
    boundary=jnp.asarray(problem.boundary_pose,dtype=jnp.float64)
    initial_twist=jnp.asarray(problem.initial_twist,dtype=jnp.float64)
    anchors=jnp.asarray(problem._anchors[1:],dtype=jnp.float64)
    reference=jnp.asarray(problem.common_reference,dtype=jnp.float64)
    goal=jnp.asarray(problem.goal_pose,dtype=jnp.float64)
    chol=jnp.asarray(problem.chol,dtype=jnp.float64)
    fresh_std=jnp.asarray(c['fresh_std'],dtype=jnp.float64)
    h=c['support_dt_s']

    def core(vector):
        z=vector.reshape((-1,5))
        p=jnp.concatenate((boundary[None,:],retract_pose(anchors,z[:,:3])),axis=0)
        future_v=jnp.stack((z[:,3],jnp.zeros_like(z[:,3]),z[:,4]),axis=-1)
        v=jnp.concatenate((initial_twist[None,:],future_v),axis=0)
        residual=gp_residual(p[:-1],v[:-1],p[1:],v[1:],h)
        white=jsl.solve_triangular(chol,residual.T,lower=True).T
        gp_by_factor=.5*jnp.sum(white**2,axis=1)
        fresh=se2_log(relative_pose(reference,p[1:]))/fresh_std
        fresh_cost=jnp.mean(jnp.sum(fresh**2,axis=1))
        objective=jnp.sum(gp_by_factor)+c['lambda_fresh']*fresh_cost
        pp,vv,aa=interpolate_interval(p[:-1,None],v[:-1,None],p[1:,None],v[1:,None],h,
                                      jnp.array([0.,.5,1.],dtype=jnp.float64))
        equality=vv[:,1,1]
        speed,acceleration=vv.reshape((-1,3)),aa.reshape((-1,3))
        motion=jnp.concatenate((speed[:,0],c['v_max']-speed[:,0],c['w_max']-speed[:,2],c['w_max']+speed[:,2],
                                c['a_v_max']-acceleration[:,0],c['a_v_max']+acceleration[:,0],
                                c['a_w_max']-acceleration[:,2],c['a_w_max']+acceleration[:,2]))
        yaw=wrap_angle(p[-1,2]-goal[2])
        goal_inequality=jnp.stack((c['goal_position_tolerance']**2-jnp.sum((p[-1,:2]-goal[:2])**2),
                                   c['goal_yaw_tolerance']-yaw,c['goal_yaw_tolerance']+yaw))
        inequality=jnp.concatenate((motion,goal_inequality))
        xy=pp.reshape((-1,3))[:,:2]
        packed=jnp.concatenate((objective[None],equality,inequality,xy.ravel()))
        aux=dict(poses=p,twists=v,gp_factor_costs=gp_by_factor,fresh_cost=fresh_cost,
                 collocation_body_twists=vv.reshape((-1,3)),
                 collocation_body_accelerations=aa.reshape((-1,3)))
        return packed,(packed,aux)
    return core
