"""Synthetic implementation fixtures only; no experimental model evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source03 import (compare_mpc, literal_settings, luminance_statistics,
    lighting_gate, lighting_classification, trajectory_equal, history_suffix, path_geometry, save)
from reconciliation.online_mpc_adapter import PINNED_LIGHTNAV_SHA, PINNED_MPC_SHA256
from reconciliation.join_source02 import observation_anchored_world, jpeg_wire_parity, pointing_diagnostics

ROOT=Path(__file__).resolve().parents[1]


def mpc():
    return dict(lightnav_sha=PINNED_LIGHTNAV_SHA,mpc_source_sha256=PINNED_MPC_SHA256,external_git_status='',
        official_settings=dict(HORIZON=5,MPC_DT_S=.1,CONTROL_RATE_HZ=10,Q_WEIGHTS=[10,10,1],
                              R_WEIGHTS=[.1,.1],OBJNAV_V_MAX=.8,W_MAX=3,A_MAX_V=2,A_MAX_W=5))


def test_saved_mpc_comparison():
    a=mpc()
    assert compare_mpc(a,[deepcopy(a)])=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    assert compare_mpc(a,[])=='MPC_AUDIT_INCONCLUSIVE'
    for key,value in [('lightnav_sha','wrong'),('mpc_source_sha256','wrong'),('external_git_status','dirty')]:
        b=deepcopy(a);b[key]=value
        assert compare_mpc(a,[b])=='MPC_PROVENANCE_MISMATCH'
    b=deepcopy(a);b['official_settings']['HORIZON']=6
    assert compare_mpc(a,[b])=='MPC_PROVENANCE_MISMATCH'


def test_read_constants_does_not_execute_module():
    assert literal_settings('HORIZON=5\nraise RuntimeError("no execution")')['HORIZON']==5


def test_luminance_is_read_only_and_missing_mask_is_none():
    a=np.full((4,5,3),80,dtype=np.uint8);before=a.copy()
    s=luminance_statistics(a)
    assert s['mean']==pytest.approx(80)
    assert np.array_equal(a,before)
    assert luminance_statistics(a,np.zeros((4,5),bool)) is None


def test_brightness_gate_is_not_model_based():
    import yaml
    cfg=yaml.safe_load((ROOT/'configs/join_source_03_bright_cause.yaml').read_text())
    def stats(v):
        s=luminance_statistics(np.full((10,10,3),v,dtype=np.uint8))
        return dict(whole=s,cart=s,cart_pixels=100,target_pixels=30)
    assert lighting_gate(stats(10),stats(80),cfg['brightness_acceptance'])['valid']
    assert not lighting_gate(stats(10),stats(255),cfg['brightness_acceptance'])['valid']


@pytest.mark.parametrize('safe,gain,equivalent,label',[
    (True,.30,False,'BRIGHT_RECOVERS_SAFE_RESPONSE'),
    (False,.03,False,'BRIGHT_IMPROVES_CLEARANCE_BUT_UNSAFE'),
    (False,0.,False,'BRIGHT_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN'),
    (False,0.,True,'DARK_AND_BRIGHT_BEHAVIOR_EQUIVALENT')])
def test_predeclared_lighting_classes(safe,gain,equivalent,label):
    d=dict(safe=False,clearance_m=-.2,meaningful_response=True,stop=False)
    b=dict(d,safe=safe,clearance_m=-.2+gain)
    assert lighting_classification(d,b,equivalent=equivalent)==label


def test_stop_not_reclassified_as_bypass():
    d=dict(safe=False,clearance_m=-.2,meaningful_response=True,stop=False)
    b=dict(d,safe=True,clearance_m=.1,stop=True)
    assert lighting_classification(d,b,equivalent=False)!='BRIGHT_RECOVERS_SAFE_RESPONSE'


def test_wrapped_yaw_and_generic_horizon():
    for n in (1,3,10,17):
        a=np.zeros((n,3));b=a.copy();b[:,2]=2*np.pi
        assert trajectory_equal(a,b)
        b[0,0]=1e-3
        assert not trajectory_equal(a,b)


def test_source_history_is_causal_and_not_padded():
    f=[dict(frame_id=str(i),capture_sim_time_s=i*.25,path=f'{i}.jpg') for i in range(40)]
    assert history_suffix(f,32,'30') is None
    assert history_suffix(f,16,'32')==f[17:33]
    assert history_suffix(f,32,'32')==f[1:33]
    bad=deepcopy(f);bad[31]['path']=bad[30]['path']
    with pytest.raises(ValueError):history_suffix(bad,16,'32')
    bad=deepcopy(f);bad[31]['capture_sim_time_s']=0
    with pytest.raises(ValueError):history_suffix(bad,16,'32')


def env():
    obstacle=box(.99,-.1,1.01,.1)
    return HospitalEnvironment(obstacle,box(-4,-4,4,4),parts=[obstacle])


def test_point_safe_segment_unsafe_is_not_endpoint_error():
    r=path_geometry([[0,0,0],[2,0,0]],env())
    assert min(r['node_clearance_m'])>.05
    assert r['first_unsafe_waypoint_zero_based'] is None
    assert r['first_unsafe_segment_zero_based']==0
    assert r['whole']['physical_overlap']
    assert r['safe_prefix_boundary']['distance_from_first_row_m'][1] < 1
    assert r['connector_included'] is False and r['row_timestamps'] is None


def test_entire_original_path_checked_no_suffix_trim():
    a=np.array([[0,0,0],[1,0,0],[2,0,0]])
    before=a.copy();r=path_geometry(a,env())
    assert r['row_count']==3 and r['first_unsafe_waypoint_zero_based']==1
    assert np.array_equal(a,before)


def test_safe_start_is_not_collision_start():
    r=path_geometry([[0,0,0],[.2,0,0],[.8,0,0]],env())
    assert r['node_clearance_m'][0]>.05
    assert r['first_unsafe_waypoint_zero_based']==2


def test_no_missing_geometry_as_zero():
    with pytest.raises(ValueError):path_geometry([],env())
    with pytest.raises(ValueError):path_geometry([[np.nan,0,0]],env())
    assert path_geometry([[0,1,0]],env())['first_unsafe_segment_zero_based'] is None


def test_observation_transform_not_boundary_anchor():
    a=np.array([[1,0,0],[2,0,.2]]);before=a.copy()
    w=observation_anchored_world(a,[10,20,np.pi/2])
    np.testing.assert_allclose(w[:,:2],[[10,21],[10,22]],atol=1e-12)
    np.testing.assert_array_equal(a,before)


def test_exact_wire_input_and_optional_target_visibility():
    assert jpeg_wire_parity(b'original',b'original')['valid']
    assert not jpeg_wire_parity(b'original',b'brightened')['valid']
    r=pointing_diagnostics({})
    assert r['visible'] is None


def test_exclusive_artifacts(tmp_path):
    save(tmp_path/'result.json',{'missing':None})
    assert json.loads((tmp_path/'result.json').read_text())['missing'] is None
    with pytest.raises(FileExistsError):save(tmp_path/'result.json',{})


def test_frozen_scope_and_no_hidden_controller():
    import yaml
    c=yaml.safe_load((ROOT/'configs/join_source_03_bright_cause.yaml').read_text())
    assert c['budget']['maximum_terminal_predictions']==21
    assert c['budget']['new_MPC_solves']==0
    assert c['receding_horizon']['enabled'] is False
    assert len(c['distance']['center_arc_m'])==3
    worker=(ROOT/'scripts/lightnav/join_source03_paired.py').read_text()
    assert 'run_branches' in worker and 'MpcTracker(' not in worker


def test_frozen_scene_mutation_restricted_to_new_fill():
    source=(ROOT/'scripts/isaac/join_source03_render.py').read_text()
    assert 'UsdLux.RectLight.Define' in source
    assert 'SetIntensity' not in source
    assert 'ImageEnhance' not in source and 'equalizeHist' not in source
