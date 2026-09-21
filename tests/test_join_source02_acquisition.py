import numpy as np
import pytest
from shapely.geometry import box
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_acquisition import (
    ObstacleState, held_interval_check, history_suffix, placement_on_future,
)


def environment():
    return HospitalEnvironment(box(9,9,10,10), box(-5,-5,12,12))


def test_presence_is_one_explicit_render_oracle_version():
    rendered=[]
    state=ObstacleState(environment(),box(.8,-.3,1.2,.3),rendered.append)
    assert state.environment.check_polyline([[0,0],[2,0]])['clearance_valid']
    event=state.set(True,{'sim_time_s':1.25})
    assert event['render_visible'] and event['evaluator_occupied']
    assert not state.environment.check_polyline([[0,0],[2,0]])['clearance_valid']
    state.set(False,{'sim_time_s':2.})
    assert rendered==[False,True,False]


def test_held_interval_guard_checks_segment_not_only_endpoints():
    env=HospitalEnvironment(box(.9,-.1,1.1,.1),box(-5,-5,5,5))
    assert env.query([0,0])['status']=='CLEARANCE_VALID'
    assert env.query([2,0])['status']=='CLEARANCE_VALID'
    assert not held_interval_check(env,[0,0,0],[2,0,0])['clearance_valid']
    curved=held_interval_check(environment(),[0,0,0],[.01,.0001,.02])
    assert curved['curve_allowance_m']>0
    assert not curved['avoidance_command_generated']


def test_history_has_no_padding_reversal_or_duplicate_id():
    frames=[dict(frame_id=str(i),capture_monotonic_ns=i+1,capture_sim_time_s=i*.25) for i in range(32)]
    r=history_suffix(frames,16)
    assert r['history'][0]['frame_id']=='16' and r['terminal']['frame_id']=='31'
    assert history_suffix(frames,64)['status']=='HISTORY_UNAVAILABLE'
    with pytest.raises(ValueError):history_suffix(frames[::-1],16)
    with pytest.raises(ValueError):history_suffix([frames[0]]*16,16)


def test_placement_is_geometry_only_and_never_extrapolates():
    p=placement_on_future([0,0,0],np.array([[0,0,0],[3,0,0]]),fraction=.35,half_forward_m=.25)
    assert p['status']=='AVAILABLE' and 0<p['center_xy'][0]<3
    assert p['guarantee'] is False
    q=placement_on_future([0,0,0],np.array([[0,0,0],[.5,0,0]]),fraction=.65,half_forward_m=.25)
    assert q['status']=='PLACEMENT_UNAVAILABLE'
    with pytest.raises(ValueError):placement_on_future([0,0,0],[[0,0,0],[3,0,0]],fraction=.5,half_forward_m=.25)
