"""Geometry fixtures are synthetic numerical checks, not experimental evidence."""
from copy import deepcopy

import numpy as np
import pytest
from shapely.geometry import LineString, Point, box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_ref04_geometry import polyline_audit, gate_audit, spatial_gate_audit
from reconciliation.gp_se2_reference import directed_gate_crossings


def config():
    return dict(footprint=dict(radius_m=.2,required_clearance_m=.05),
                evaluation=dict(gate_crossing_tolerance_m=1e-6))


def environment(obstacle=None,workspace=None):
    return HospitalEnvironment(LineString([(0.,-4.),(0.,4.)]) if obstacle is None else obstacle,
                               box(-5.,-5.,5.,5.) if workspace is None else workspace)


def gate(**updates):
    value=dict(gate_id='synthetic',center_xy=[0.,0.],normal_xy=[1.,0.],half_width_m=1.)
    value.update(updates);return value


def test_swept_violation_between_safe_nodes_refines_earliest_segment():
    env=environment();points=np.array([[-1.,0.,0.],[1.,0.,0.]])
    result=polyline_audit(points,env,config(),times=[0.,1.])
    assert result['node_clearance_m']==pytest.approx([.8,.8])
    assert result['first_sampled_violation_time_s'] is None
    assert result['physical_overlap'] and not result['clearance_valid']
    assert result['first_swept_violation_bracket_s']==[0.,1.]
    lo,hi=result['first_refined_violation_bracket_s']
    crossing=(1-(.25+env.numerical_tolerance_m))/2
    assert lo<=crossing<=hi and hi-lo<=.001
    assert result['minimum_location_world']==pytest.approx([0.,0.])
    assert result['minimum_time_s']==pytest.approx(.5)
    assert result['minimum_clearance_m']==pytest.approx(-.2)
    assert result['refinement']['half_sweep_queries']>0


def test_clearance_failure_is_not_physical_overlap():
    result=polyline_audit([[.23,-1.],[.23,1.]],environment(),config(),times=[0.,1.])
    assert result['minimum_clearance_m']==pytest.approx(.03)
    assert not result['clearance_valid'] and not result['physical_overlap']
    assert result['first_sampled_violation_time_s']==0.
    assert result['physical_overlap_threshold_m']==-1e-7


def test_original_curve_bound_uncertainty_and_workspace_predicates_match():
    env=environment();points=np.array([[.3,-1.,0.],[.3,1.,0.]])
    before=points.copy();bound=.001
    result=polyline_audit(points,env,config(),times=[0.,1.],curve_bound=bound)
    original=env.check_trajectory([0.,1.],points,radius=.2,required_clearance=.05,curved_path_error_bound_m=bound)
    for key in ('minimum_clearance_m','physical_overlap','clearance_valid','workspace_known'):
        assert result[key]==original[key]
    assert result['effective_threshold_m']==pytest.approx(.0510001)
    assert result['first_refined_violation_bracket_s'] is None
    np.testing.assert_array_equal(points,before)


def test_borderline_is_invalid_without_being_definite_violation_or_overlap():
    result=polyline_audit([[.25,0.],[.25,.1]],environment(),config(),times=[0.,.1])
    assert result['original_environment_check']['status']=='BORDERLINE'
    assert not result['clearance_valid'] and not result['physical_overlap']


def test_workspace_unknown_never_free_and_curve_allowance_applied_once():
    env=environment(LineString([(4.,-3.),(4.,3.)]),box(0.,-2.,3.,2.))
    points=[[.2005,0.],[.2005,.1]]
    valid=polyline_audit(points,env,config(),times=[0.,.1])
    invalid=polyline_audit(points,env,config(),times=[0.,.1],curve_bound=.001)
    assert valid['workspace_known'] and valid['clearance_valid']
    assert not invalid['workspace_known'] and not invalid['clearance_valid']
    assert invalid['minimum_clearance_m']>.05 and invalid['first_sampled_violation_time_s']==0.


def test_refinement_retains_parent_when_saved_curve_half_sweeps_are_valid():
    # Parent chord intersects the point obstacle, whereas saved curved midpoints
    # make both half chords safe. Preserve original rejection and its bracket.
    env=environment(Point(0.,0.));points=[[-1.,0.,0.],[1.,0.,0.]]
    def saved_pose(time):return [2*time-1.,np.sin(np.pi*time),0.]
    result=polyline_audit(points,env,config(),times=[0.,1.],pose_at=saved_pose)
    assert not result['clearance_valid'] and result['physical_overlap']
    assert result['first_refined_violation_bracket_s']==[0.,1.]
    assert result['refinement']['status']=='PARENT_BRACKET_RETAINED_BOTH_HALF_SWEEPS_VALID'
    assert result['original_acceptance_unchanged_by_refinement']


def test_refinement_uses_pose_callback_without_replacing_global_bound():
    env=environment();seen=[]
    def saved_pose(time):seen.append(time);return [-1.+time,0.,0.]
    result=polyline_audit([[-1.,0.],[0.,0.]],env,config(),times=[0.,1.],curve_bound=.01,pose_at=saved_pose)
    lo,hi=result['first_refined_violation_bracket_s']
    assert seen and lo<=.7399999<=hi and hi-lo<=.001
    assert result['effective_threshold_m']==pytest.approx(.0600001)
    assert result['minimum_clearance_m']==-.2


def test_reference_polyline_without_times_has_spatial_index_not_fabricated_time():
    result=polyline_audit([[-1.,0.],[1.,0.]],environment(),config())
    assert result['first_swept_violation_segment_index']==0
    for key in ('times_s','minimum_time_s','first_sampled_violation_time_s','first_swept_violation_bracket_s','first_refined_violation_bracket_s'):
        assert result[key] is None
    assert all(s['start_time_s'] is s['end_time_s'] is None for s in result['segments'])


def test_stationary_duplicates_and_single_reference_point_have_finite_minimum():
    for points,times in [([[1.,0.]], [2.]),([[1.,0.],[1.,0.]], [2.,3.])]:
        result=polyline_audit(points,environment(),config(),times=times)
        assert result['minimum_location_world']==[1.,0.]
        assert result['minimum_time_s']==2. and result['minimum_clearance_m']==.8
        assert result['clearance_valid']


@pytest.mark.parametrize('points,times,bound',[
    ([],None,0.),([[0.,float('nan')]],None,0.),([[0.,0.],[1.,0.]],[0.,0.],0.),
    ([[0.,0.],[1.,0.]],[0.],0.),([[0.,0.]],None,-.1)])
def test_invalid_geometry_times_and_bound_are_errors(points,times,bound):
    with pytest.raises(ValueError):polyline_audit(points,environment(),config(),times=times,curve_bound=bound)


def test_nan_pose_callback_fails_without_substituting_safe_values():
    with pytest.raises(ValueError,match='pose_at'):
        polyline_audit([[-1.,0.],[1.,0.]],environment(),config(),times=[0.,1.],pose_at=lambda _: [float('nan'),0.])


def test_valid_center_interval_is_not_shrunk_again_by_footprint():
    env=environment(LineString([(4.,-4.),(4.,4.)]));points=[[-1.,.9],[1.,.9]]
    record=gate_audit(points,[0.,1.],[gate()],env,config())[0]
    assert record['classification']=='VALID_CROSSING' and record['valid']
    assert record['interval_margin_m']==pytest.approx(.1)
    assert record['crossing_time_s']==.5
    assert record['refined_bracket_s'][1]-record['refined_bracket_s'][0]<=.001
    assert record['original_route_result']==directed_gate_crossings(points,[0.,1.],[gate()],tolerance=1e-6)


@pytest.mark.parametrize('y',[1.,1.01,1.-.5e-6])
def test_endpoint_outside_and_tolerance_band_are_not_valid_gate_crossings(y):
    result=gate_audit([[-1.,y],[1.,y]],[0.,1.],[gate()],environment(),config())[0]
    assert result['classification']=='INVALID_INTERVAL_CROSSING'
    assert not result['valid'] and not result['original_route_valid']
    assert result['original_gate_result']['endpoint_or_outside_crossings']==1
    assert result['finite_interval_relation']==('OUTSIDE' if y>1.+1e-6 else 'ENDPOINT')
    assert result['outside_distance_m']==pytest.approx(max(0.,y-1.))


def test_reverse_grazing_and_no_crossing_remain_distinct():
    env=environment();gates=[gate()]
    reverse=gate_audit([[1.,0.],[-1.,0.]],[0.,1.],gates,env,config())[0]
    assert reverse['classification']=='REVERSE_CROSSING' and reverse['direction']=='REVERSE'
    grazing=gate_audit([[-1.,0.],[0.,0.],[0.,.5],[1.,.5]],[0.,1.,2.,3.],gates,env,config())[0]
    assert grazing['classification']=='GRAZING' and not grazing['original_route_valid']
    touch=gate_audit([[-1.,0.],[0.,0.],[-1.,0.]],[0.,1.,2.],gates,env,config())[0]
    assert touch['classification']=='GRAZING' and touch['crossing_time_s'] is None
    absent=gate_audit([[-2.,0.],[-1.,0.]],[0.,.1],gates,env,config(),scope='PREDICTION')[0]
    assert absent['classification']=='NO_CROSSING' and absent['valid'] is None
    assert absent['status']=='NO_CROSSING_IN_HORIZON'
    assert absent['scope']=='PREDICTION' and absent['crossing_time_s'] is None


def test_stationary_on_plane_pause_is_valid_and_refinement_uses_piecewise_nodes():
    points=[[-1.,0.],[0.,0.],[0.,0.],[1.,0.]];times=[0.,1.,2.,3.]
    record=gate_audit(points,times,[gate()],environment(),config())[0]
    assert record['valid'] and not record['grazing'] and record['crossing_time_s']==1.
    lo,hi=record['refined_bracket_s']
    assert lo<=1.<=hi and hi-lo<=.001


def test_original_ordered_route_failure_is_not_erased_by_valid_local_crossing():
    points=[[-2.,0.],[2.,0.]];times=[0.,1.]
    gates=[gate(gate_id='later',center_xy=[1.,0.]),gate(gate_id='earlier',center_xy=[-1.,0.])]
    records=gate_audit(points,times,gates,environment(),config())
    assert all(r['valid'] for r in records)
    assert not any(r['original_route_valid'] for r in records)
    assert not records[1]['original_gate_result']['ordered']


def test_gate_refinement_uses_saved_pose_and_preserves_authoritative_coarse_result():
    points=[[-1.,0.,0.],[1.,0.,0.]]
    def saved_pose(t):return [2*t*t-1.,0.,0.]
    result=gate_audit(points,[0.,1.],[gate()],environment(),config(),pose_at=saved_pose)[0]
    assert result['crossing_time_s']==.5
    lo,hi=result['refined_bracket_s']
    assert lo<=np.sqrt(.5)<=hi and hi-lo<=.001
    assert result['original_gate_result']['first_directed_crossing_s']==.5


def test_empty_gate_list_requires_no_invented_gate_and_inputs_stay_unchanged():
    points=[[0.,0.],[1.,0.]];original=deepcopy(points)
    assert gate_audit(points,[0.,1.],[],environment(),config())==[]
    assert points==original


@pytest.mark.parametrize('y,relation,valid',[(.5,'INSIDE',True),(1.-1e-6,'ENDPOINT',False),
    (1.,'ENDPOINT',False),(1.+.5e-6,'ENDPOINT',False),(1.01,'OUTSIDE',False)])
def test_finite_interval_relation_preserves_exact_strict_comparison(y,relation,valid):
    result=gate_audit([[-1.,y],[1.,y]],[0.,1.],[gate()],environment(),config())[0]
    assert result['finite_interval_relation']==relation and result['valid']==valid
    assert result['outside_distance_m']==pytest.approx(max(0.,abs(y)-1.))
    assert result['outside_acceptance_distance_m']==pytest.approx(max(0.,abs(y)-(1.-1e-6)),abs=1e-15)


def test_spatial_gate_audit_never_exposes_artificial_seconds():
    result=spatial_gate_audit([[-2.,0.],[2.,0.]],[gate()],environment(),config())[0]
    assert result['classification']=='VALID_CROSSING' and result['crossing_parameter']==.5
    assert result['bracket_parameter']==[0.,1.]
    assert result['ordered_spatial_gate_result']['first_directed_crossing_parameter']==.5
    assert result['actual_motion_or_prediction'] is False
    def keys(value):
        if isinstance(value,dict):
            for key,child in value.items():yield key;yield from keys(child)
        elif isinstance(value,list):
            for child in value:yield from keys(child)
    assert not any(key.endswith('_s') or 'time' in key for key in keys(result))
    assert 'not physical time' in result['parameter_semantics']


def test_spatial_entry_connector_and_no_crossing_do_not_claim_execution():
    points=[[-1.,1.01],[1.,1.01]]
    result=spatial_gate_audit(points,[gate()],environment(),config())[0]
    assert result['classification']=='INVALID_INTERVAL_CROSSING'
    assert result['outside_distance_m']==pytest.approx(.01)
    absent=spatial_gate_audit([[-2.,0.],[-1.,0.]],[gate()],environment(),config())[0]
    assert absent['classification']=='NO_CROSSING' and absent['crossing_parameter'] is None
    assert absent['status']=='NO_CROSSING'
