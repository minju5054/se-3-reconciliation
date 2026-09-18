import numpy as np
import pytest

from reconciliation.gp_se2_reference import prepare_reference, directed_gate_crossings, choose_cases
from reconciliation.se2 import wrap_angle


def test_official_weighted_next_row_preparation_has_no_boundary_connector():
    native=np.array([[0.,0.,0.],[.2,0.,.5],[.4,0.,.7]])
    saved=native.copy()
    r=prepare_reference(native,[.19,0.,0.],[1,1,10])
    assert r['nearest_row_index']==0  # yaw matters, not XY-only correspondence
    assert r['first_future_row_index']==1
    assert r['common_world'].shape==(30,3)
    assert np.allclose(r['common_world'][0],native[1])
    assert np.allclose(r['common_world'][-1],native[-1])
    assert r['sample_times_s'][0]==.1
    assert np.array_equal(native,saved)
    assert r['intrinsic_model_waypoint_dt_s'] is None


def test_rotation_only_rows_and_shortest_angle_are_preserved():
    native=np.array([[0,0,3.0],[0,0,3.1],[0,0,-3.0]])
    r=prepare_reference(native,[0,0,3.0],[1,1,1])
    assert r['original_row_indices']==[1,2]
    assert np.max(np.abs(wrap_angle(np.diff(r['common_world'][:,2])))) < .01
    assert np.array_equal(r['common_world'][:,:2],np.zeros((30,2)))


def test_single_last_row_constant_future_and_grid_validation():
    r=prepare_reference([[1,2,.3]],[0,0,0],[1,1,1])
    assert np.allclose(r['common_world'],np.tile([1,2,.3],(30,1)))
    with pytest.raises(ValueError):prepare_reference([[0,0,0]],[0,0,0],[1,1,1],horizon_s=3.01)


GATE={'gate_id':'door','center_xy':[0,0],'normal_xy':[1,0],'half_width_m':.5}


@pytest.mark.parametrize('xy,valid',[
    ([[-1,0],[0,0],[1,0]],True),
    ([[1,0],[0,0],[-1,0]],False),
    ([[-1,0],[0,0],[-1,0]],False),
    ([[-1,.5],[0,.5],[1,.5]],False),
    ([[-1,.6],[0,.6],[1,.6]],False),
])
def test_gate_requires_actual_directed_crossing_not_touch(xy,valid):
    assert directed_gate_crossings(xy,[0,1,2],[GATE])['valid']==valid


def test_gate_order_rejects_reverse_order_even_if_both_crossed():
    far={**GATE,'gate_id':'far','center_xy':[.5,0]}
    assert not directed_gate_crossings([[-1,0],[1,0]],[0,1],[far,GATE])['valid']


def test_selection_deduplicates_and_does_not_fill_missing_groups():
    config={'group_order':['D','A'],'priority_events':[],'target_per_group':3}
    rows=[dict(case_id='e0/h0',episode_id='e0',ordered_raw_pair='p0',groups={'D':True,'A':True}),
          dict(case_id='e0/h1',episode_id='e0',ordered_raw_pair='p0',groups={'A':True}),
          dict(case_id='e1/h0',episode_id='e1',ordered_raw_pair='p1',groups={'A':True})]
    selected=choose_cases(rows,config)
    assert [r['case_id'] for r in selected]==['e0/h0','e1/h0','e0/h1']
    assert [r['selected_group'] for r in selected]==['D','A','A']
