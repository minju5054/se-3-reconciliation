"""Actual equality/inequality constrained rigid and locally linear GP pilot.

SLSQP is used as a constrained numerical solver, not as the unconstrained factor
solver from Dong et al. GP factor sparsity and constraint solving are distinct.
All positions are world metres; angles are radians; velocities are body-frame
metres/s and radians/s. Inputs are copied, and raw files are never touched.

M2/M3 share objective, support chart, two deterministic starts and solver budget;
only obstacle inequalities differ. Known-workspace inequalities apply to both.
The output is a finite candidate only after independent-in-time dense motion
checks, not a proof of continuous feasibility. Environment callbacks used here
are the optimizer approximation; a separate geometry checker must assess the
saved candidate and actual MPC rollout. A failed numerical solve is never a
proof of infeasibility and is never replaced with a RAW fallback.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Any

import numpy as np
from scipy import __version__ as scipy_version
from scipy.linalg import solve_triangular
from scipy.optimize import minimize

from .gp_se2 import gp_residual, interpolate_interval, process_covariance, sample_gp
from .se2 import compose_poses, inverse_pose, relative_pose, retract_pose, se2_exp, se2_log, wrap_angle

DistanceQuery = Callable[[np.ndarray], np.ndarray]

DEFAULT_CONFIG = {
    "horizon_s": 3.0, "support_dt_s": .1,
    "qc_diag": [1., 1., 1.], "fresh_std": [.1, .1, float(np.deg2rad(10.))],
    "lambda_fresh": 1., "entry_weight": 1., "v_max": .8, "w_max": 3.,
    "a_v_max": 2., "a_w_max": 5.,
    "goal_position_tolerance": .15, "goal_yaw_tolerance": float(np.deg2rad(15.)),
    "required_clearance": .05, "max_iterations": 200, "wall_time_s": 30.,
    "ftol": 1e-7, "equality_tolerance": 1e-5, "inequality_tolerance": 1e-5,
    "dense_dt_s": .01, "initializations": 2, "random_seed": 0,
}


def resolve_config(config: dict | None = None) -> dict:
    c = dict(DEFAULT_CONFIG)
    if config:
        unknown = set(config)-set(c)
        if unknown:
            raise ValueError(f"unknown formulation configuration: {sorted(unknown)}")
        c.update(config)
    for name in ("horizon_s", "support_dt_s", "wall_time_s", "ftol", "dense_dt_s",
                 "v_max", "w_max", "a_v_max", "a_w_max", "goal_position_tolerance",
                 "goal_yaw_tolerance", "equality_tolerance", "inequality_tolerance"):
        if not np.isfinite(c[name]) or c[name] <= 0:
            raise ValueError(f"{name} must be positive")
    if c["initializations"] not in (1, 2) or int(c["max_iterations"]) < 1:
        raise ValueError("one or two deterministic initializations and positive iterations required")
    if c["dense_dt_s"] > .01+1e-12:
        raise ValueError("dense validation interval must be <= .01 s")
    n = round(c["horizon_s"]/c["support_dt_s"])
    if n < 1 or not np.isclose(n*c["support_dt_s"],c["horizon_s"]):
        raise ValueError("horizon must be an integer number of support intervals")
    if (np.asarray(c["fresh_std"]).shape != (3,) or not np.all(np.isfinite(c["fresh_std"]))
            or np.any(np.asarray(c["fresh_std"]) <= 0)):
        raise ValueError("fresh_std must contain three positive scales")
    process_covariance(c["support_dt_s"],c["qc_diag"])
    if (not np.all(np.isfinite([c["lambda_fresh"],c["entry_weight"],c["required_clearance"]]))
            or c["lambda_fresh"] < 0 or c["entry_weight"] < 0 or c["required_clearance"] < 0):
        raise ValueError("objective weights and clearance must be nonnegative")
    return c


def _input_pose(x: Any, label: str) -> np.ndarray:
    p=np.array(x,dtype=float,copy=True)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError(f"{label} must be finite shape (3,)")
    return p


def _query(query: DistanceQuery | None, poses: np.ndarray, name: str) -> np.ndarray:
    if query is None:
        raise ValueError(f"{name} callback is required")
    result=np.asarray(query(np.asarray(poses)[...,:2]),float)
    if result.shape != np.asarray(poses).shape[:-1]:
        raise ValueError(f"{name} callback shape mismatch")
    # NaN/unknown cannot create free space. A finite negative constraint is
    # returned to SLSQP; the independent checker retains unknown provenance.
    return np.where(np.isfinite(result),result,-1e6)


def _pose_reference(common: np.ndarray, sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """XY linear / shortest-yaw reference, valid only on the future row domain."""
    t=np.asarray(query_times,float)
    if np.any(t<sample_times[0]-1e-12) or np.any(t>sample_times[-1]+1e-12):
        raise ValueError("rigid/native interpolation cannot invent a boundary connector")
    yaw=np.unwrap(common[:,2]); result=np.stack([np.interp(t,sample_times,common[:,0]),
        np.interp(t,sample_times,common[:,1]),wrap_angle(np.interp(t,sample_times,yaw))],axis=-1)
    return result


def _gate_rows(poses_at: Callable[[np.ndarray], np.ndarray], gates: list[dict],
               start: float, end: float) -> tuple[np.ndarray,np.ndarray]:
    eq=[]; inequalities=[]
    previous=-np.inf
    for gate in gates:
        center=np.asarray(gate["center_xy"],float); normal=np.asarray(gate["normal_xy"],float)
        if center.shape != (2,) or normal.shape != (2,) or not np.all(np.isfinite([center,normal])):
            raise ValueError("gate center and normal must be finite 2-vectors")
        if not np.isclose(np.linalg.norm(normal),1.,atol=1e-8):
            raise ValueError("gate normal must be a unit vector")
        gt=float(gate["time_s"]); half=float(gate["half_width_m"])
        offset=float(gate.get("crossing_offset_s",.05)); progress=float(gate.get("minimum_progress_m",1e-4))
        if not start < gt < end or gt<=previous or half<=0 or offset<=0 or progress<=0:
            raise ValueError("gates require ordered interior times, positive width/crossing settings")
        previous=gt
        ts=np.array([max(start,gt-offset),gt,min(end,gt+offset)])
        p=poses_at(ts)[:,:2]-center
        signed=p@normal; lateral=p[1]@np.array([-normal[1],normal[0]])
        eq.append(signed[1]); inequalities.extend([half-lateral,half+lateral,-signed[0]-progress,signed[2]-progress])
    return np.asarray(eq),np.asarray(inequalities)


def _goal_inequalities(pose: np.ndarray, goal: np.ndarray, c: dict) -> np.ndarray:
    yaw=float(wrap_angle(pose[2]-goal[2]))
    return np.array([c["goal_position_tolerance"]**2-np.sum((pose[:2]-goal[:2])**2),
        c["goal_yaw_tolerance"]-yaw,c["goal_yaw_tolerance"]+yaw])


def _motion_inequalities(v: np.ndarray, a: np.ndarray,c: dict) -> np.ndarray:
    return np.concatenate([v[:,0],c["v_max"]-v[:,0],c["w_max"]-v[:,2],c["w_max"]+v[:,2],
        c["a_v_max"]-a[:,0],c["a_v_max"]+a[:,0],c["a_w_max"]-a[:,2],c["a_w_max"]+a[:,2]])


def _constraint_report(eq: np.ndarray,ineq: np.ndarray,c:dict) -> dict:
    finite=bool(np.all(np.isfinite(eq)) and np.all(np.isfinite(ineq)))
    maxeq=float(np.max(np.abs(eq),initial=0.)); minineq=float(np.min(ineq,initial=0.))
    return {"finite":finite,"maximum_equality_residual":maxeq,"maximum_inequality_violation":max(0.,-minineq),
        "equality_count":len(eq),"inequality_count":len(ineq),
        "feasible":bool(finite and maxeq<=c["equality_tolerance"] and minineq>=-c["inequality_tolerance"]),
        "equality_tolerance":c["equality_tolerance"],"inequality_tolerance":c["inequality_tolerance"]}


@dataclass
class GPProblem:
    """An inspectable objective/constraint object for one frozen GP formulation."""
    boundary_pose: np.ndarray
    initial_twist: np.ndarray
    common_reference: np.ndarray
    goal_pose: np.ndarray
    config: dict
    obstacle_clearance: DistanceQuery | None = None
    workspace_margin: DistanceQuery | None = None
    gates: list[dict] | None = None
    include_obstacles: bool = True

    def __post_init__(self):
        self.config=resolve_config(self.config)
        self.boundary_pose=_input_pose(self.boundary_pose,"boundary")
        self.initial_twist=_input_pose(self.initial_twist,"initial_twist")
        if abs(self.initial_twist[1]) > 1e-12:
            raise ValueError("boundary body lateral twist must be zero")
        self.goal_pose=_input_pose(self.goal_pose,"goal")
        self.common_reference=np.array(self.common_reference,float,copy=True)
        n=round(self.config["horizon_s"]/self.config["support_dt_s"])
        if self.common_reference.shape != (n,3) or not np.all(np.isfinite(self.common_reference)):
            raise ValueError("common reference must have one finite pose per future support")
        self.times=np.linspace(0,self.config["horizon_s"],n+1)
        self.gates=list(self.gates or [])
        self.chol=np.linalg.cholesky(process_covariance(self.config["support_dt_s"],self.config["qc_diag"]))
        self._anchors=np.vstack([self.boundary_pose,self.common_reference])
        self._last_x=None; self._last_eval=None

    def initializations(self) -> list[dict]:
        h=self.config["support_dt_s"]
        p=self._anchors.copy()
        interval=se2_log(relative_pose(p[:-1],p[1:]))/h
        # Initialization only. Boundary conditions remain exactly the recorded values.
        v=np.zeros_like(p); v[0]=self.initial_twist
        v[1:,0]=np.clip(interval[:,0],0,self.config["v_max"])
        v[1:,2]=np.clip(interval[:,2],-self.config["w_max"],self.config["w_max"])
        cp=compose_poses(self.boundary_pose,se2_exp(self.times[:,None]*self.initial_twist))
        cv=np.tile(self.initial_twist,(len(p),1))
        return [{"name":"fresh_pose_finite_log_twist","poses":p,"twists":v},
                {"name":"boundary_constant_body_twist","poses":cp,"twists":cv}][:self.config["initializations"]]

    def vector(self,poses:np.ndarray,twists:np.ndarray)->np.ndarray:
        d=se2_log(relative_pose(self._anchors[1:],poses[1:]))
        return np.column_stack([d,twists[1:,0],twists[1:,2]]).ravel()

    def unpack(self,x:np.ndarray)->tuple[np.ndarray,np.ndarray]:
        z=np.asarray(x).reshape(-1,5)
        p=np.vstack([self.boundary_pose,retract_pose(self._anchors[1:],z[:,:3])])
        v=np.zeros_like(p);v[0]=self.initial_twist;v[1:,0]=z[:,3];v[1:,2]=z[:,4]
        return p,v

    def evaluate(self,x:np.ndarray)->dict:
        if self._last_x is not None and np.array_equal(x,self._last_x):
            return self._last_eval
        p,v=self.unpack(x);h=self.config["support_dt_s"]
        r=gp_residual(p[:-1],v[:-1],p[1:],v[1:],h)
        white=solve_triangular(self.chol,r.T,lower=True,check_finite=False).T
        gp_by_factor=.5*np.sum(white**2,axis=1)
        fresh=se2_log(relative_pose(self.common_reference,p[1:]))/np.asarray(self.config["fresh_std"])
        fresh_cost=float(np.mean(np.sum(fresh**2,axis=1)))
        pp,vv,aa=interpolate_interval(p[:-1,None],v[:-1,None],p[1:,None],v[1:,None],h,np.array([0.,.5,1.]))
        gateeq,gateineq=_gate_rows(lambda q:sample_gp(self.times,p,v,q)[0],self.gates,0,self.times[-1])
        eq=np.concatenate([vv[:,1,1],gateeq])
        inequalities=[_motion_inequalities(vv.reshape(-1,3),aa.reshape(-1,3),self.config),
                       _goal_inequalities(p[-1],self.goal_pose,self.config),gateineq]
        if self.workspace_margin is not None:
            inequalities.append(_query(self.workspace_margin,pp.reshape(-1,3),"workspace_margin"))
        if self.include_obstacles:
            inequalities.append(_query(self.obstacle_clearance,pp.reshape(-1,3),"obstacle_clearance")-self.config["required_clearance"])
        result={"poses":p,"twists":v,"equality":eq,"inequality":np.concatenate(inequalities),
                "objective":float(np.sum(gp_by_factor)+self.config["lambda_fresh"]*fresh_cost),
                "gp_factor_costs":gp_by_factor,"fresh_cost":fresh_cost}
        self._last_x=np.array(x,copy=True);self._last_eval=result
        return result

    def dense_report(self,x:np.ndarray)->dict:
        e=self.evaluate(x);p,v=e["poses"],e["twists"]
        nq=int(np.ceil(self.times[-1]/self.config["dense_dt_s"]))
        qt=np.linspace(0,self.times[-1],nq+1)
        pp,vv,aa=sample_gp(self.times,p,v,qt)
        # Both one-sided accelerations at knots are necessary; the trajectory is C1, not C2.
        _,sv,sa=interpolate_interval(p[:-1,None],v[:-1,None],p[1:,None],v[1:,None],
                                     self.config["support_dt_s"],np.array([0.,1.]))
        eq=np.concatenate([e["equality"],vv[:,1],sv[:,:,1].ravel()])
        ineq=[e["inequality"],_motion_inequalities(vv,aa,self.config),
              _motion_inequalities(sv.reshape(-1,3),sa.reshape(-1,3),self.config)]
        if self.workspace_margin is not None:ineq.append(_query(self.workspace_margin,pp,"workspace_margin"))
        if self.include_obstacles:ineq.append(_query(self.obstacle_clearance,pp,"obstacle_clearance")-self.config["required_clearance"])
        report=_constraint_report(eq,np.concatenate(ineq),self.config)
        # Independently differentiate the returned poses inside every support interval.
        check_t=(self.times[:-1]+self.times[1:])/2
        eps=1e-5
        mid,midv,_=sample_gp(self.times,p,v,check_t)
        before=sample_gp(self.times,p,v,check_t-eps)[0];after=sample_gp(self.times,p,v,check_t+eps)[0]
        fd=(se2_log(relative_pose(mid,after))-se2_log(relative_pose(mid,before)))/(2*eps)
        derivative_error=float(np.max(np.abs(fd-midv),initial=0.))
        report.update({"validation_level":"SAMPLED_MOTION_AND_APPROXIMATE_CLEARANCE_VALID" if report["feasible"] else "SAMPLED_CONSTRAINT_FAILURE",
            "dense_dt_max_s":float(qt[1]-qt[0]),"maximum_absolute_lateral_velocity_m_s":float(np.max(np.abs(vv[:,1]))),
            "pose_derivative_body_twist_error_max":derivative_error,
            "body_derivative_identity_valid":derivative_error<=self.config["equality_tolerance"],
            "times":qt,"poses":pp,"body_twists":vv,"body_accelerations":aa,
            "equality_residuals":eq,"inequality_margins":np.concatenate(ineq),
            "independent_environment_check_required":True})
        report["feasible"] = bool(report["feasible"] and report["body_derivative_identity_valid"])
        return report


class _WallTimeExceeded(RuntimeError):
    pass


def _minimize_budgeted(initial:np.ndarray,evaluate:Callable[[np.ndarray],dict],config:dict,
                       dense_check:Callable[[np.ndarray],dict]|None=None)->dict:
    start=time.monotonic();latest=np.array(initial,copy=True); evaluations=0;iterations=0
    best_feasible=None;best_objective=np.inf;callback_checks=[]
    def check():
        if time.monotonic()-start>=config["wall_time_s"]:raise _WallTimeExceeded("frozen per-initialization wall-time budget reached")
    def ev(x):
        nonlocal evaluations
        check();evaluations+=1
        return evaluate(x)
    def callback(x):
        nonlocal latest,iterations,best_feasible,best_objective
        if not np.all(np.isfinite(x)):
            raise FloatingPointError("solver produced a nonfinite iterate")
        latest=np.array(x,copy=True);iterations+=1;check()
        value=evaluate(x)
        collocation=_constraint_report(value["equality"],value["inequality"],config)
        if collocation["feasible"] and value["objective"] < best_objective and dense_check is not None:
            report=dense_check(x)
            callback_checks.append({"iteration":iterations,"objective":value["objective"],
                "dense_feasible":report["feasible"],
                "maximum_equality_residual":report.get("maximum_equality_residual"),
                "maximum_inequality_violation":report.get("maximum_inequality_violation")})
            if report["feasible"]:
                best_feasible=np.array(x,copy=True);best_objective=value["objective"]
        check()
    initial_e=evaluate(initial)
    constraints=[]
    if len(initial_e["equality"]):constraints.append({"type":"eq","fun":lambda x:ev(x)["equality"]})
    if len(initial_e["inequality"]):constraints.append({"type":"ineq","fun":lambda x:ev(x)["inequality"]})
    try:
        result=minimize(lambda x:ev(x)["objective"],initial,method="SLSQP",constraints=constraints,
                        callback=callback,options={"maxiter":int(config["max_iterations"]),"ftol":config["ftol"],"disp":False})
        if np.all(np.isfinite(result.x)):
            latest=np.array(result.x,copy=True)
        status={"termination":"CONVERGED" if result.success else "SOLVER_FAILURE",
                "solver_success":bool(result.success),"solver_status":int(result.status),"message":str(result.message),
                "iterations":int(result.nit),"objective_evaluations":int(result.nfev)}
    except _WallTimeExceeded as exc:
        status={"termination":"TIMEOUT","solver_success":False,"solver_status":None,"message":str(exc),"iterations":iterations,"objective_evaluations":None}
    except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc:
        status={"termination":"NUMERICAL_FAILURE","solver_success":False,"solver_status":None,"message":f"{type(exc).__name__}: {exc}","iterations":iterations,"objective_evaluations":None}
    status.update({"wall_time_s":time.monotonic()-start,"all_callback_evaluations":evaluations,
                   "latest_iterate":latest,"initial_vector":np.array(initial,copy=True),
                   "best_feasible_iterate":best_feasible,"callback_candidate_checks":callback_checks,
                   "random_seed":config["random_seed"],"random_generator_used":False,
                   "infeasibility_proven":False})
    return status


def solve_gp(boundary_pose:Any,initial_twist:Any,common_reference:Any,goal_pose:Any,
             config:dict|None=None,obstacle_clearance:DistanceQuery|None=None,
             gates:list[dict]|None=None,include_obstacles:bool=True,
             workspace_margin:DistanceQuery|None=None)->dict:
    """Solve M2 or M3; no accepted candidate is represented by ``None``, never RAW."""
    problem=GPProblem(boundary_pose,initial_twist,common_reference,goal_pose,config or {},
                      obstacle_clearance,workspace_margin,gates,include_obstacles)
    attempts=[];choices=[]
    for init_index,init in enumerate(problem.initializations()):
        initial=problem.vector(init["poses"],init["twists"])
        attempt=_minimize_budgeted(initial,problem.evaluate,problem.config,problem.dense_report)
        # Check initial and latest accepted iterate, recording either as such. No extra restart.
        checks=[]
        iterate_checks=[("initial",initial),("latest_iterate",attempt["latest_iterate"])]
        if attempt["best_feasible_iterate"] is not None:
            iterate_checks.append(("best_feasible_intermediate",attempt["best_feasible_iterate"]))
        for label,x in iterate_checks:
            try:
                value=problem.evaluate(x); report=problem.dense_report(x)
                checks.append({"iterate":label,"objective":value["objective"],"constraint_report":report})
                if report["feasible"]:
                    choices.append((value["objective"],init_index,label,np.array(x,copy=True),report))
            except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc:
                checks.append({"iterate":label,"constraint_report":{"feasible":False,"error":str(exc)}})
        latest_poses, latest_twists = problem.unpack(attempt["latest_iterate"])
        attempt.update({"latest_support_poses":latest_poses,"latest_support_twists":latest_twists,
                        "initialization":init["name"],"initial_support_poses":init["poses"],
                        "initial_support_twists":init["twists"],"candidate_checks":checks})
        attempts.append(attempt)
    selected=min(choices,key=lambda row:(row[0],row[1],row[2])) if choices else None
    out={"method":"M3_GP_CONSTRAINED" if include_obstacles else "M2_GP_NO_OBSTACLE",
         "status":"CANDIDATE_FOUND" if selected else "NO_FEASIBLE_CANDIDATE_FOUND",
         "solver_backend":"scipy.optimize.SLSQP","solver_version":scipy_version,
         "config":problem.config,"attempts":attempts,"sample_times":problem.times[1:],
         "support_times":problem.times,"candidate_world":None,"support_poses":None,"support_twists":None,
         "constraint_report":None,"factor_costs":None,"selected_initialization":None,
         "diagnostic_candidate_world":attempts[-1]["latest_support_poses"][1:].copy(),
         "diagnostic_support_poses":attempts[-1]["latest_support_poses"],
         "diagnostic_support_twists":attempts[-1]["latest_support_twists"],
         "source_frame":"world","pose_units":["m","m","rad"],
         "infeasibility_proven":False,"independent_environment_check_required":True}
    if selected:
        _,index,label,x,report=selected;value=problem.evaluate(x)
        out.update({"candidate_world":value["poses"][1:].copy(),"support_poses":value["poses"],
                    "support_twists":value["twists"],"constraint_report":report,
                    "factor_costs":{"gp":value["gp_factor_costs"],"fresh_uniform":value["fresh_cost"],"total":value["objective"]},
                    "selected_initialization":index,"selected_iterate":label})
    return out


def solve_rigid(boundary_pose:Any,initial_twist:Any,common_reference:Any,goal_pose:Any,
                config:dict|None=None,obstacle_clearance:DistanceQuery|None=None,
                gates:list[dict]|None=None,workspace_margin:DistanceQuery|None=None)->dict:
    """Optimize one world-left G with finite-lookahead entry and uniform FRESH cost.

    No hard equality forces a rigid reference to the actual initial state, and
    no artificial B connector is prepended. Geometry begins at the first future
    reference time. Actual initial motion feasibility belongs to the MPC rollout.
    """
    c=resolve_config(config); b=_input_pose(boundary_pose,"boundary");v=_input_pose(initial_twist,"initial_twist")
    goal=_input_pose(goal_pose,"goal");f=np.array(common_reference,float,copy=True)
    n=round(c["horizon_s"]/c["support_dt_s"]);ts=np.arange(1,n+1)*c["support_dt_s"]
    if f.shape!=(n,3) or not np.all(np.isfinite(f)):raise ValueError("common reference shape mismatch")
    gates=list(gates or []);entry=compose_poses(b,se2_exp(ts[0]*v));std=np.asarray(c["fresh_std"])
    ct=np.sort(np.unique(np.r_[ts,(ts[:-1]+ts[1:])/2]))
    dense_t=np.linspace(ts[0],ts[-1],int(np.ceil((ts[-1]-ts[0])/c["dense_dt_s"]))+1)
    def evaluate(x,qt=ct):
        g=se2_exp(x);candidate=compose_poses(g,f)
        entry_r=se2_log(relative_pose(entry,candidate[0]))/std
        fresh=se2_log(relative_pose(f,candidate))/std
        def at(q):return compose_poses(g,_pose_reference(f,ts,q))
        eq,gateineq=_gate_rows(at,gates,ts[0],ts[-1]);qp=at(qt)
        inequalities=[_goal_inequalities(candidate[-1],goal,c),gateineq,
                      _query(obstacle_clearance,qp,"obstacle_clearance")-c["required_clearance"]]
        if workspace_margin is not None:inequalities.append(_query(workspace_margin,qp,"workspace_margin"))
        entrycost=float(entry_r@entry_r);freshcost=float(np.mean(np.sum(fresh**2,axis=1)))
        return {"objective":c["entry_weight"]*entrycost+c["lambda_fresh"]*freshcost,
                "equality":eq,"inequality":np.concatenate(inequalities),"candidate":candidate,
                "entry_cost":entrycost,"fresh_cost":freshcost,"poses":qp,"transform_pose":g}
    initial=[("identity",np.zeros(3)),("finite_lookahead_alignment",se2_log(compose_poses(entry,inverse_pose(f[0]))))][:c["initializations"]]
    attempts=[];choices=[]
    for index,(name,x0) in enumerate(initial):
        def dense_check(x):
            value=evaluate(x,dense_t)
            return _constraint_report(value["equality"],value["inequality"],c)
        attempt=_minimize_budgeted(x0,evaluate,c,dense_check);checks=[]
        iterate_checks=[("initial",x0),("latest_iterate",attempt["latest_iterate"])]
        if attempt["best_feasible_iterate"] is not None:
            iterate_checks.append(("best_feasible_intermediate",attempt["best_feasible_iterate"]))
        for label,x in iterate_checks:
            try:
                val=evaluate(x,dense_t); report=_constraint_report(val["equality"],val["inequality"],c)
                report.update({"validation_level":"SAMPLED_APPROXIMATE_REFERENCE_CLEARANCE_VALID" if report["feasible"] else "SAMPLED_CONSTRAINT_FAILURE",
                    "times":dense_t,"poses":val["poses"],"dense_dt_max_s":float(np.max(np.diff(dense_t),initial=0.)),
                    "independent_environment_check_required":True,"initial_connector":"UNDEFINED_EVALUATED_ONLY_BY_MPC"})
                checks.append({"iterate":label,"objective":val["objective"],"constraint_report":report})
                if report["feasible"]:choices.append((val["objective"],index,label,x.copy(),report))
            except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc:
                checks.append({"iterate":label,"constraint_report":{"feasible":False,"error":str(exc)}})
        latest_candidate=compose_poses(se2_exp(attempt["latest_iterate"]),f)
        attempt.update({"initialization":name,"candidate_checks":checks,"latest_candidate_world":latest_candidate});attempts.append(attempt)
    selected=min(choices,key=lambda row:(row[0],row[1],row[2])) if choices else None
    out={"method":"M1_RIGID","status":"CANDIDATE_FOUND" if selected else "NO_FEASIBLE_CANDIDATE_FOUND",
         "solver_backend":"scipy.optimize.SLSQP","solver_version":scipy_version,"config":c,
         "sample_times":ts,"attempts":attempts,"candidate_world":None,"support_poses":None,
         "support_twists":None,"constraint_report":None,"transform_pose":None,"factor_costs":None,
         "infeasibility_proven":False,"independent_environment_check_required":True,
         "diagnostic_candidate_world":attempts[-1]["latest_candidate_world"]}
    if selected:
        _,index,label,x,report=selected;val=evaluate(x)
        out.update({"candidate_world":val["candidate"],"constraint_report":report,"transform_pose":val["transform_pose"],
            "selected_initialization":index,"selected_iterate":label,
            "factor_costs":{"entry":val["entry_cost"],"fresh_uniform":val["fresh_cost"],"total":val["objective"]}})
    return out
