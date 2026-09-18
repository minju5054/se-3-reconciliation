"""Synthetic failure-first evaluation checks, never dataset performance evidence."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml
from shapely.geometry import LineString, box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_evaluation import METHODS, comparisons, evaluate_plan, evaluate_rollout
from reconciliation.gp_se2_reference import directed_gate_crossings
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.se2 import compose_poses, se2_exp


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/gp_se2_01.yaml').read_text())


def environment(obstacle=None):
    return HospitalEnvironment(LineString([(8,-8),(8,8)]) if obstacle is None else obstacle,box(-10,-10,10,10))


def route(goal):
    return {'route_status':'KNOWN','gates':[],'goal_world':np.asarray(goal).tolist()}


def synthetic_rollout(command=(.2,0.),initial=(0.,0.,0.)):
    """New analytic fixture, not a recorded or reused experimental execution."""
    command=np.asarray(command,float);pose=np.asarray(initial,float)
    states=[{'tick':0,'time_s':0.,'pose_world':pose.tolist()}];commands=[];solves=[]
    for tick in range(180):
        if tick%6==0:solves.append({'time_s':tick/60,'command':command.tolist(),'success':True})
        commands.append({'time_s':tick/60,'end_time_s':(tick+1)/60,'command':command.tolist()})
        pose=integrate_unicycle(pose,command,1/60)
        states.append({'tick':tick+1,'time_s':(tick+1)/60,'pose_world':pose.tolist()})
    return {'horizon_s':3.,'control_hz':10.,'integration_hz':60.,'states':states,'commands':commands,
        'controller_reference_selections':solves,'controller_failure_count':0,
        'initial_physical_command':command.tolist(),
        'resolved_limits':{'v_max':.8,'omega_max':3.,'a_v_max':2.,'a_omega_max':5.}}


def test_stationary_smooth_trivial_solution_fails_goal(config):
    result=evaluate_rollout(synthetic_rollout((0.,0.)),route([.6,0,0]),environment(),config)
    assert not result['primary_success']
    assert result['failure_reasons']==['goal_failure']
    assert result['execution']['command_total_variation_v_mps']==0
    assert result['execution']['time_to_goal_s'] is None


def test_safe_reference_and_unsafe_actual_rollout_are_separate(config):
    env=environment(LineString([(.3,-1),(.3,1)]))
    candidate=np.column_stack([np.linspace(.02,.6,30),np.full(30,2.),np.zeros(30)])
    goal=route(candidate[-1]);boundary=np.zeros(3)
    plan=evaluate_plan('M0_ADAPTER',candidate,None,candidate,boundary,[.2,0,0],goal,env,config)
    actual=evaluate_rollout(synthetic_rollout(),goal,env,config)
    assert plan['plan_valid']
    assert not actual['primary_success']
    assert actual['environment']['physical_overlap']
    assert 'new_collision' in actual['failure_reasons']
    assert actual['environment']['first_overlap_time_bracket_s'] is not None
    assert not actual['environment']['continuous_time_collision_proof']


def test_no_candidate_does_not_become_raw_fallback(config):
    out=evaluate_plan('M3_GP_CONSTRAINED',None,{},np.zeros((30,3)),[0,0,0],[0,0,0],route([1,0,0]),environment(),config)
    assert out['status']=='NO_CANDIDATE'
    assert out['plan_valid'] is False
    assert out['environment'] is None and out['deformation'] is None


def test_unknown_route_and_unknown_workspace_never_pass(config):
    rollout=synthetic_rollout();goal=route([.6,0,0]);goal['route_status']='UNKNOWN'
    out=evaluate_rollout(rollout,goal,environment(),config)
    assert not out['primary_success'] and 'unknown_route' in out['failure_reasons']
    narrow=HospitalEnvironment(LineString([(8,-8),(8,8)]),box(-.1,-.1,1.,1.))
    out=evaluate_rollout(rollout,route([.6,0,0]),narrow,config)
    assert not out['primary_success'] and 'unknown_workspace' in out['failure_reasons']


def test_curve_checker_inserts_command_boundaries_and_records_sagitta_bound(config):
    rollout=synthetic_rollout((.2,.4))
    out=evaluate_rollout(rollout,route(rollout['states'][-1]['pose_world']),environment(),config)
    env=out['environment'];t=np.asarray(out['dense_times_s'])
    assert env['curved_path_error_bound_m']==pytest.approx(.2*.4*np.max(np.diff(t))**2/8)
    assert np.max(np.diff(t))<=.005+1e-12
    for row in rollout['states']:assert np.any(t==row['time_s'])
    assert env['continuous_time_collision_proof'] is False


def test_gp_runtime_pose_derivative_identity_across_world_yaw_wrap(config):
    boundary=np.array([1.,2.,3.1]);velocity=np.array([.2,0,.4]);times=np.arange(31)*.1
    support=compose_poses(boundary,se2_exp(times[:,None]*velocity));twists=np.tile(velocity,(31,1))
    solver={'support_poses':support,'support_twists':twists,'support_times':times}
    out=evaluate_plan('M3_GP_CONSTRAINED',support[1:],solver,support[1:],boundary,velocity,
                      route(support[-1]),environment(),config)
    assert out['plan_valid']
    assert out['motion']['valid']
    assert out['motion']['pose_derivative_max_error']<1e-8
    assert out['motion']['sampled_only']
    assert not out['environment']['continuous_time_collision_proof']


def test_gp_acceleration_includes_both_one_sided_support_limits(config):
    times=np.arange(31)*.1;support=np.zeros((31,3));twists=np.zeros((31,3))
    support[:,0]=.2*times;twists[:,0]=.2
    # On [1.0,1.1], analytic Hermite acceleration rises 0 -> 2.02.
    # At 1.1 the right interval acceleration is zero. A right-only knot query
    # and .005s intermediate samples miss this isolated left-sided maximum.
    endpoint=.2+.2*.1+2.02*.1**2/6
    support[11:,0]=endpoint+.301*(times[11:]-1.1);twists[11:,0]=.301
    solver={'support_poses':support,'support_twists':twists,'support_times':times}
    out=evaluate_plan('M3_GP_CONSTRAINED',support[1:],solver,support[1:],support[0],twists[0],
                      route(support[-1]),environment(),config)
    assert not out['motion']['valid']
    assert out['motion']['violations']['linear_acceleration']
    assert not out['plan_valid']


def test_plane_grazing_does_not_fabricate_directed_gate_crossing():
    gate={'gate_id':'synthetic_gate','center_xy':[0.,0.],'normal_xy':[1.,0.],'half_width_m':.1}
    path=np.array([[-1.,0.],[0.,2.],[0.,-2.],[1.,0.]])
    assert not directed_gate_crossings(path,np.arange(4),[gate])['valid']
    direct=np.array([[-1.,0.],[0.,0.],[1.,0.]])
    assert directed_gate_crossings(direct,np.arange(3),[gate])['valid']


def test_failure_first_regressions_and_secondary_metrics_only_for_paired_success(config):
    evaluation=evaluate_rollout(synthetic_rollout(),route([.6,0,0]),environment(),config)
    assert evaluation['primary_success']
    rows=[]
    for case in ['all_success','gp_no_candidate']:
        for method in METHODS:
            failure=case=='gp_no_candidate' and method=='M3_GP_CONSTRAINED'
            rows.append({'case_id':case,'method':method,'candidate_found':not failure,'plan_valid':not failure,
                'primary_success':not failure,'failure_reasons':['no_candidate'] if failure else [],
                'rollout_metrics':None if failure else deepcopy(evaluation)})
    result=comparisons(rows)
    assert result['native_success_to_gp_failure']==1
    assert result['adapter_success_to_gp_failure']==1
    assert len(result['paired_metrics'])==6  # 5 successful pairs + native/adapter control.
    assert result['methods']['M3_GP_CONSTRAINED']['attempts']==2
    assert result['methods']['M3_GP_CONSTRAINED']['candidates']==1
    assert result['methods']['M3_GP_CONSTRAINED']['primary_success']==1
    assert all(row['reasons']==['no_candidate'] for row in result['regressions'])
    assert all(row['both_primary_success'] for row in result['paired_metrics'])
    assert not any(row['case_id']=='gp_no_candidate' and row['method']=='M3_GP_CONSTRAINED' for row in result['paired_metrics'])
