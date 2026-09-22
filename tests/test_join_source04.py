"""Synthetic implementation tests only; never LightNav experiment evidence."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import box

from reconciliation.join_source04 import (construct_frames,premodel,pixel_ray,ground_intersection,hit_class,
    mask_relation,affordance,motion_geometry,agreement,persistence,KS,ORDER)
from reconciliation.join_source03 import path_geometry,save
from reconciliation.join_source02 import observation_anchored_world,jpeg_wire_parity
from reconciliation.gp_se2_environment import HospitalEnvironment


def bank():
    rows=[]
    for i in range(16):
        row=dict(source_pose_world=[i*.01,0,0])
        for name in ('OFF','ON'):
            frame=dict(frame_id=str(i),path=f'{i}_{name}.jpg',pose_world=row['source_pose_world'],
                capture_sim_time_s=i*.25,capture_monotonic_ns=i*10+(1 if name=='OFF' else 2),
                all_bright=True,counterfactual_moving_pose_history=True)
            row[name]=dict(frame=frame,camera={'pose':i},scene_signature=str(i),lighting_sha256='same',
                cart_transform=np.eye(4).tolist(),cart_pixels=0 if name=='OFF' else 30,
                present=name=='ON',stable=True,same_product=True,cart_edge_clearance_m=.1)
        rows.append(row)
    return rows


@pytest.mark.parametrize('k',[0,1,2,4,8])
def test_exact_K_H_pose_order_and_bright(k):
    b=bank();f=construct_frames(b,k)
    assert len(f)==16 and [x['frame_id'] for x in f]==list(map(str,range(16)))
    assert [x['path'].endswith('_ON.jpg') for x in f]==[False]*(16-k)+[True]*k
    assert [x['pose_world'] for x in f]==[x['source_pose_world'] for x in b]
    assert premodel(b,k)['valid']
    assert f==construct_frames(b,k) # byte-identical same bank for sham


def test_unavailable_no_padding_no_movement_no_relaxation():
    b=bank();b[8]['ON']['cart_edge_clearance_m']=-.01
    assert not premodel(b,8)['valid'] and premodel(b,4)['valid']
    b=bank();b[15]['ON']['cart_pixels']=19;assert not premodel(b,1)['valid']
    b=bank();b[3]['ON']['cart_transform'][0][3]=1;assert not premodel(b,1)['valid']
    b=bank();b[0]['OFF']['frame']['path']=b[1]['OFF']['frame']['path']
    with pytest.raises(ValueError):construct_frames(b,1)
    with pytest.raises(ValueError):construct_frames(bank(),3)
    b=bank();b[0]['OFF']['frame']['all_bright']=False
    with pytest.raises(ValueError):construct_frames(b,1)


def test_pair_camera_scene_presence_identity():
    for key,value in [('camera',{'different':1}),('scene_signature','bad'),('same_product',False),('stable',False),('present',False)]:
        b=bank();b[-1]['ON'][key]=value;assert not premodel(b,1)['valid']


def camera():
    return dict(actual_intrinsics=dict(K=[[100,0,100],[0,100,50],[0,0,1]]),
                T_world_camera=[[0,0,-1,0],[-1,0,0,0],[0,1,0,1],[0,0,0,1]])


def test_pixel_ray_and_invalid_ground():
    o,d=pixel_ray([100,50],camera());assert np.allclose(o,[0,0,1]) and np.allclose(d,[1,0,0])
    assert ground_intersection(o,d) is None
    o,d=pixel_ray([100,150],camera());assert np.allclose(ground_intersection(o,d),[1,0,0])
    o,d=pixel_ray([150,150],camera());assert ground_intersection(o,d)[1]<0 # image right = agent right
    assert ground_intersection([0,0,1],[0,0,1]) is None


@pytest.mark.parametrize('path,expected',[('/cart/a','cart'),('/target/mesh','target'),('/World/Floor','floor'),('/World/Wall','wall'),('/other','other'),(None,'none')])
def test_first_hit_identity(path,expected):
    assert hit_class(path,'/cart','/target')==expected


def test_mask_censoring_and_no_point():
    mask=np.zeros((200,200),np.uint32);mask[30:80,90:120]=1
    assert mask_relation([100,50],mask,[1])['inside_cart_mask']
    assert mask_relation([50,50],mask,[1])['relative_to_cart_bbox']=='left'
    assert mask_relation(None,mask,[1])['inside_cart_mask'] is None
    env=HospitalEnvironment(box(4,-.5,5,.5),box(-10,-10,10,10))
    t=dict(camera=camera(),instance=dict(matched_instance_ids=[1],idToLabels={'0':'/World/Floor','1':'/cart/a'}),
        target_pixels=20,cart_prefix='/cart',frame=dict(pose_world=[0,0,0]))
    response=dict(pointing=dict(apos_px=[100,190],apos_clamped=True,apos_state='point'))
    a=affordance(response,t,mask,np.ones_like(mask,dtype=float),env,box(4,-.5,5,.5),np.array([4.5,0]),'/target')
    assert a['ray_kind']=='BOUNDARY_RAY_PROXY' and not a['clear_side'] and not a['metric_model_waypoint']
    assert agreement(a,dict(broad_side='right'),False)=='APOS_OBSTACLE_DIRECTED_OR_AMBIGUOUS'


@pytest.mark.parametrize('n',[1,3,10,17])
def test_generic_N_raw_preservation_and_observation_anchor(n):
    raw=np.column_stack([np.arange(n)*.1,np.zeros(n),np.zeros(n)]);before=raw.copy()
    world=observation_anchored_world(raw,[1,2,np.pi/2])
    assert np.allclose(world[0,:2],[1,2])
    m=motion_geometry(raw,world,[1,2,np.pi/2],np.array([1,5]),np.array([1,3]),np.array([0,1]),[-.2,.2])
    assert m['N']==n and np.array_equal(raw,before)


def test_entire_polyline_no_suffix_connector_and_crossing():
    env=HospitalEnvironment(box(.49,-.1,.51,.1),box(-5,-5,5,5))
    result=path_geometry(np.array([[0,0,0],[1,0,0]]),env)
    assert result['first_unsafe_waypoint_zero_based'] is None
    assert result['first_unsafe_segment_zero_based']==0 and not result['connector_included']
    assert result['safe_prefix_boundary']['distance_from_first_row_m'][1]<.5


def result(clearance=-.1,safe=False,raw=None):
    return dict(raw_local=[[0,0,0],[1,0,0]] if raw is None else raw,
        geometry_on=dict(whole=dict(clearance_valid=safe,minimum_clearance_m=clearance),
            first_unsafe_segment_zero_based=0,segment_checks=[dict(minimum_clearance_m=clearance)]),
        stop=False,motion=dict(target_directed_progress_m=1))


def test_predeclared_persistence_no_forced_success():
    r={k:result() for k in ORDER};assert persistence(r)['classification']=='PERSISTENCE_HAS_NO_MATERIAL_EFFECT'
    r['K4']=result(clearance=.1,safe=True);assert persistence(r)['classification']=='PERSISTENCE_RECOVERS_SAFE_FRESH'
    r['K4']=result(clearance=-.07);assert persistence(r)['classification']=='PERSISTENCE_IMPROVES_BUT_REMAINS_UNSAFE'
    r['K1_SHAM']=result(raw=[[0,0,0],[2,0,0]]);assert persistence(r)['classification']=='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE'
    assert persistence({})['classification']=='PERSISTENCE_DIAGNOSTIC_INCONCLUSIVE'


def test_agreement_is_not_decoder_causality():
    a=dict(clear_side=True,ground_side='right')
    assert agreement(a,dict(broad_side='right'),False)=='APOS_CLEAR_SIDE_TRAJ_UNSAFE'
    assert agreement(a,dict(broad_side='left'),False)=='APOS_TRAJECTORY_DIRECTION_DISAGREEMENT'


def test_exclusive_files_and_exact_wire_bytes(tmp_path):
    p=tmp_path/'one.json';save(p,{'raw':1})
    with pytest.raises(FileExistsError):save(p,{'raw':2})
    assert jpeg_wire_parity(b'jpeg',b'jpeg')['valid']
    assert not jpeg_wire_parity(b'jpeg',b'altered')['valid']


def test_no_model_controller_optimization_imports_in_evaluator():
    import ast
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'src/reconciliation/join_source04.py').read_text())
    forbidden={'minimize','solve_gp','MpcTracker','integrate_unicycle','Session','predict'}
    names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
    assert not names.intersection(forbidden)


def test_prediction_worker_does_not_import_evaluator_dependencies():
    import ast
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'scripts/lightnav/join_source04_predict.py').read_text())
    imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert 'reconciliation.join_source04' not in imports # Shapely belongs to research venv only
    assert 'online_lightnav_worker' in imports
