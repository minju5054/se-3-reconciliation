"""Synthetic audit checker fixtures, never scientific model evidence."""
from copy import deepcopy
import ast
import csv
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_join_online02_stop import point_relation, write_new
from audit_join_online03_turnback import write_csv
from reconciliation.join_online03_turnback import (
    mask_statistics, project_mesh, point_bbox_relation, sampled_slots,
    history_aggregate, trajectory_metrics, transition,
)


def camera(w=100,h=80):
    return dict(T_world_camera=np.eye(4).tolist(),actual_intrinsics=dict(
        K=[[50,0,w/2],[0,50,h/2],[0,0,1]],resolution_width_height=[w,h],clipping_range_m=[.1,10]))


@pytest.mark.parametrize('w,h',[(100,80),(480,270),(33,21)])
def test_projection_visible_and_resolution(w,h):
    triangle=np.array([[[-.2,-.2,-2],[.2,-.2,-2],[0,.2,-2]]])
    projected=project_mesh(triangle,camera(w,h));assert projected.shape==(h,w) and projected.any()
    stats=mask_statistics(projected,projected)
    assert stats['visible_pixels']==stats['projected_pixels'] and stats['region']=='center'
    assert 0<stats['visible_image_fraction']<1


def test_clipping_out_of_view_near_and_far():
    tri=np.array([[[-3,-1,-1],[0,-1,-1],[0,1,-1]]],float)
    partial=project_mesh(tri,camera());s=mask_statistics(partial,partial)
    assert s['region']=='partial-edge' and s['bbox_xyxy_inclusive'][0]==0
    for delta in [[100,0,0],[0,0,20],[0,0,-20]]:
        p=project_mesh(tri+delta,camera());assert not p.any()
        assert mask_statistics(p,p)['region']=='out-of-frame-or-clipped'
    cross=np.array([[[-.1,-.1,-.05],[.1,-.1,-1],[0,.1,-1]]])
    assert project_mesh(cross,camera()).any()


def test_visible_and_projected_pixels_distinct():
    projected=np.ones((10,20),bool);visible=np.zeros_like(projected);visible[4:6,4:6]=True
    s=mask_statistics(visible,projected)
    assert s['visible_pixels']==4 and s['projected_pixels']==200
    assert s['occlusion_fraction'] is None and not s['visibility_gate_pass']
    assert mask_statistics(np.zeros_like(visible))['bbox_xyxy_inclusive'] is None


@pytest.mark.parametrize('channel',['apos','opos'])
def test_point_cart_mask_bbox_and_empty(channel):
    mask=np.zeros((27,48),np.uint32);mask[10:15,20:25]=7
    instance=dict(matched_instance_ids=[7],idToLabels={'7':'/cart','0':'/floor'})
    for pixel,inside in [([22.,12.],True),([5.,5.],False)]:
        p=dict(mode='grid',frame_size=[48,27],**{channel+'_px':pixel,channel+'_clamped':False,channel+'_state':'point'})
        r=point_relation(p,channel,mask,instance);assert r['raster_on_cart']==inside
        assert point_bbox_relation(pixel,mask==7)['inside_cart_bbox']==inside
        p[channel+'_clamped']=True
        assert point_relation(p,channel,mask,instance)['interpretation']=='CENSORED_BY_CLAMPING'
    assert point_bbox_relation([0,0],np.zeros_like(mask))['distance_to_cart_bbox_px'] is None


def snapshot():
    frames=[dict(frame_id=f'f{i}',capture_sim_time_s=i*.25,capture_monotonic_ns=i+1) for i in range(3)]
    segments=[dict(frame_ids=[0,1,2,2],tier=0,pool_mode='avg',pool_spatial=1)]
    h=dict(frames=frames,history_contract=dict(history_storage='full_episode_slowfast',slowfast_tiers=[]),
        model_input_segments_reconstructed=[dict(frame_ids=['f0','f1','f2','f2'],
            server_frame_indices_reconstructed=[0,1,2,2],tier=0,pool_mode='avg',pool_spatial=1)])
    return h,lambda *args:deepcopy(segments)


def test_sampler_order_age_and_padding_not_new_capture():
    h,s=snapshot();rows=sampled_slots(h,s)
    assert [r['frame_id'] for r in rows]==['f0','f1','f2','f2']
    assert [r['age_sim_s'] for r in rows]==[.5,.25,0,0]
    assert [r['repeated_processor_slot'] for r in rows]==[False,False,False,True]
    for r in rows:r.update(is_current=r['frame_id']=='f2',visible_pixels=30,visibility_gate_pass=True)
    agg=history_aggregate(rows,include_current=True)
    assert agg['slots']['count']==4 and agg['unique']['count']==3
    assert history_aggregate(rows,include_current=False)['unique']['most_recent_visible_age_s']==.25


@pytest.mark.parametrize('bad',['duplicate','future','reorder'])
def test_invalid_history_rejected(bad):
    h,s=snapshot()
    if bad=='duplicate':h['frames'][1]['frame_id']='f0'
    elif bad=='future':h['frames'][1]['capture_sim_time_s']=99
    else:h['model_input_segments_reconstructed'][0]['frame_ids'].reverse()
    with pytest.raises(ValueError):sampled_slots(h,s)


def test_empty_history_visibility_not_zero_age():
    rows=[dict(frame_id='f',visible_pixels=0,visibility_gate_pass=False,age_sim_s=1.,is_current=False)]
    assert history_aggregate(rows,include_current=False)['unique']['most_recent_visible_age_s'] is None


def test_full_polyline_interior_not_only_endpoint_and_generic_N():
    scenario=dict(forward_xy=[1,0],center_xy=[0,0],cart_extents=[-.2,.2])
    for n in [1,4,17]:
        p=np.column_stack((np.arange(n)*.2,np.linspace(-1,-.2,n),np.zeros(n)))
        before=p.copy();m=trajectory_metrics(p,scenario)
        np.testing.assert_array_equal(p,before);assert m['N']==n
        if n>2:assert m['inward_interior_segments'] and m['inward_lateral_travel_m']==pytest.approx(.8)
    assert m['major_signed_lateral_m']==-1 and m['max_signed_lateral_m']==-.2


def test_transition_wrap_is_not_a_large_rotation():
    g=dict(endpoint_lateral_m=-.5,major_signed_lateral_m=-.6,final_yaw_hallway_rad=np.pi-.01,
           final_tangent_rad=np.pi-.01,inward_interior_segments=[2])
    a=dict(episode='test',chunk='C4',trajectory=g,original_geometry=dict(minimum_clearance_m=.2),
           cart_geometry=dict(minimum_clearance_m=.2),current=dict(visible_pixels=20),history={},prior_history={})
    b=deepcopy(a);b['chunk']='C5';b['trajectory']['final_yaw_hallway_rad']=-np.pi+.01
    b['trajectory']['final_tangent_rad']=-np.pi+.01
    assert transition(a,b)['delta_final_yaw_rad']==pytest.approx(.02)


def test_deterministic_projection_and_no_source_overwrite(tmp_path):
    p=np.array([[[-.2,-.2,-2],[.2,-.2,-2],[0,.2,-2]]]);before=p.copy()
    np.testing.assert_array_equal(project_mesh(p,camera()),project_mesh(p,camera()))
    np.testing.assert_array_equal(p,before)
    file=tmp_path/'saved.json';write_new(file,{'raw':1})
    with pytest.raises(FileExistsError):write_new(file,{'raw':2})


def test_optional_pointing_fields_have_explicit_na_csv(tmp_path):
    path=tmp_path/'points.csv'
    write_csv(path,[dict(channel='apos',interpretation='point'),dict(channel='opos',pixel=None)])
    with path.open() as f:rows=list(csv.DictReader(f))
    assert rows==[dict(channel='apos',interpretation='point',pixel='N/A'),
                  dict(channel='opos',interpretation='N/A',pixel='N/A')]
    with pytest.raises(FileExistsError):write_csv(path,[])


def test_audit_has_no_scientific_execution_calls():
    files=['src/reconciliation/join_online03_turnback.py','scripts/audit_join_online03_turnback.py',
           'scripts/plot_join_online03_turnback.py','scripts/validate_join_online03_turnback.py']
    forbidden={'solve','solve_gp','run_instrumented','minimize','predict','connect','SimulationApp','step','render'}
    for name in files:
        tree=ast.parse((ROOT/name).read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Call):
                called=node.func.id if isinstance(node.func,ast.Name) else node.func.attr if isinstance(node.func,ast.Attribute) else ''
                assert called not in forbidden
