"""Synthetic contract tests only; these are not pilot research evidence."""
import copy
import io
import hashlib
import numpy as np
import pytest
from reconciliation.robotless_old_consistent_observation import (
    plan_observation, check_old_reproduction, immediate_metrics, raw_array_hash, validate_config, CASE_IDS,
)
from reconciliation.robotless_single_chunk import observation_to_world
from reconciliation.robotless_projection_handoff import projection_geometry
from reconciliation.se2 import compose_poses, relative_pose, wrap_angle


def straight():
    return np.array([[.1,0,0],[.4,0,0],[1.,0,0]])


def test_augmented_connector_once_and_source_immutable():
    old=straight(); before=old.copy(); r0=np.zeros(3)
    p=plan_observation(r0,[.3,0,0],old)
    assert p['augmented_path_world']==[r0.tolist(),*old.tolist()]
    assert p['connector_length_m']==.1
    assert p['model_old_arc_length_m']==.9
    assert p['total_augmented_old_arc_length_m']==1.
    np.testing.assert_allclose(p['R1_old'],[.3,0,0],atol=1e-15)
    assert p['sample']['segment_index']==1
    assert p['sample']['alpha']==pytest.approx(2/3)
    assert p['target_progress_m']==.30 and p['remaining_arc_length_m']==.7
    np.testing.assert_array_equal(old,before)
    assert p['actual_execution_time'] is None and p['waypoint_time_base'] is None


def test_curved_path_and_relative_transform():
    p=plan_observation([0,0,0],[.3,0,0],[[.1,0,0],[.1,.4,np.pi/2]])
    np.testing.assert_allclose(p['R1_old'],[.1,.2,np.pi/4],atol=1e-14)
    np.testing.assert_allclose(compose_poses(p['fixed_R1'],p['fixed_R1_to_R1_old']),p['R1_old'],atol=1e-14)


def test_shortest_yaw_across_pi():
    p=plan_observation([0,0,3.1],[.3,0,0],[[.6,0,-3.1]])
    assert abs(abs(p['R1_old'][2])-np.pi)<1e-14


@pytest.mark.parametrize('yaw',[-2.7,-.3,.8,2.2])
def test_arbitrary_world_transform(yaw):
    world=[4,-2,yaw]; old=observation_to_world(world,straight())
    p=plan_observation(world,compose_poses(world,[.3,0,0]),old)
    np.testing.assert_allclose(p['R1_old'],compose_poses(world,[.3,0,0]),atol=1e-14)
    assert wrap_angle(p['sample']['phi_local_rad']-yaw)==pytest.approx(0,abs=1e-14)


def test_zero_length_segments_have_no_arc_time():
    p=plan_observation([0,0,0],[.3,0,0],[[0,0,1],[.3,0,2],[.3,0,-2],[.5,0,-1]])
    assert p['sample']['segment_index']==1 and p['sample']['alpha']==1
    np.testing.assert_allclose(p['R1_old'],[.3,0,2])


def test_exact_endpoint_allowed():
    p=plan_observation([0,0,0],[.3,0,0],[[.3,0,0]])
    assert p['status']=='OLD_OBSERVATION_PLANNED' and p['sample']['alpha']==1


@pytest.mark.parametrize('length',[0,.1,np.nextafter(.3,0)])
def test_insufficient_never_clamps(length):
    p=plan_observation([0,0,0],[.3,0,0],[[length,0,0]])
    assert p['status']=='OLD_ARC_INSUFFICIENT' and p['R1_old'] is None and p['sample'] is None
    assert p['available_arc_length_m']==length and p['reason']
    with pytest.raises(ValueError): immediate_metrics(p,straight(),{'tau_s':0})


def test_exact_repro_and_deterministic_npy_hash():
    old=straight(); result=check_old_reproduction(old,old.copy())
    assert result['status']=='PLANNING_OLD_REPRODUCED'
    b=io.BytesIO(); np.save(b,old,allow_pickle=False)
    assert raw_array_hash(old)==hashlib.sha256(b.getvalue()).hexdigest()


@pytest.mark.parametrize('change',['shape','dtype','value','signed_zero'])
def test_reproduction_mismatch(change):
    old=straight(); final=old.copy()
    if change=='shape': final=final[:2]
    if change=='dtype': final=final.astype('float32')
    if change=='value': final[0,0]=np.nextafter(final[0,0],1.)
    if change=='signed_zero': final[0,1]=-0.
    assert check_old_reproduction(old,final)['status']=='PLANNING_OLD_MISMATCH'


def test_metrics_fresh_anchor_direction_window_and_paired_arithmetic():
    old=straight(); p=plan_observation([0,0,0],[.3,0,0],old)
    raw=np.array([[.1,.05,.2],[.5,.05,.4]])
    fresh=observation_to_world(p['R1_old'],raw); before=fresh.copy()
    prev=projection_geometry([.3,0,0],fresh); prev['tau_s']=0.
    m=immediate_metrics(p,fresh,prev)
    assert m['B_world']==p['R1_old'] and m['primary_tau_s']==0
    assert m['e_perp_m']==pytest.approx(np.hypot(.1,.05))
    assert m['fresh_projection']['alpha']==0
    for key in ('phi_old_local_rad','phi_old_window_rad','phi_fresh_local_rad','phi_fresh_window_rad'):
        assert m[key]==pytest.approx(0,abs=1e-14)
    assert m['abs_e_yaw_deg']==pytest.approx(np.degrees(.2))
    for key in ('delta_e_perp_m','delta_abs_e_dir_local_deg','delta_abs_e_yaw_deg'): assert m[key]==pytest.approx(0)
    np.testing.assert_array_equal(fresh,before)
    np.testing.assert_array_equal(raw,[[.1,.05,.2],[.5,.05,.4]])


def test_curved_window_is_separate_and_local_tangent_not_pose_yaw():
    p=plan_observation([0,0,1],[.3,0,1],[[.3,0,1],[.3,.4,1]])
    fresh=np.array([[.3,0,1],[.3,.4,1]])
    prev=projection_geometry([.3,0,1],fresh); prev['tau_s']=0
    m=immediate_metrics(p,fresh,prev)
    assert m['phi_old_local_rad']==0
    assert m['phi_old_window_rad']==pytest.approx(np.pi/4)
    assert m['abs_e_dir_local_deg']==pytest.approx(90)
    assert m['abs_e_dir_window_deg']==pytest.approx(45)
    assert m['abs_e_yaw_deg']==0


def test_fresh_degenerate_local_and_insufficient_window_explicit():
    p=plan_observation([0,0,0],[.3,0,0],straight())
    fresh=np.array([[.3,0,0],[.3,0,1],[.3+1e-13,0,1]])
    prev=projection_geometry([.3,0,0],fresh); prev['tau_s']=0
    m=immediate_metrics(p,fresh,prev)
    assert m['abs_e_dir_local_deg'] is None and m['local_unavailable_reason']
    assert m['abs_e_dir_window_deg'] is None and m['window_unavailable_reason']


@pytest.mark.parametrize('tau',[.2,1.,float('nan')])
def test_no_extra_latency_or_tau1_pairing(tau):
    p=plan_observation([0,0,0],[.3,0,0],straight())
    with pytest.raises(ValueError): immediate_metrics(p,straight(),{'tau_s':0},tau_s=tau)
    with pytest.raises(ValueError): immediate_metrics(p,straight(),{'tau_s':tau})


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_rejected(bad):
    old=straight(); old[0,0]=bad
    with pytest.raises(ValueError): plan_observation([0,0,0],[.3,0,0],old)
    with pytest.raises(ValueError): check_old_reproduction(straight(),old)
    with pytest.raises(ValueError): plan_observation([bad,0,0],[.3,0,0],straight())


def test_config_freezes_cases_and_protocol():
    c={'stage':'robotless-old-consistent-observation','schema_version':1,'episode_ids':list(CASE_IDS),
        'delta_s_obs_m':.3,'primary_tau_s':0.,'tangent_window_m':.1}
    validate_config(c)
    for k,v in [('episode_ids',list(CASE_IDS)[1:]),('delta_s_obs_m',.25),('primary_tau_s',1.),('tangent_window_m',.2)]:
        bad=copy.deepcopy(c);bad[k]=v
        with pytest.raises(ValueError): validate_config(bad)
