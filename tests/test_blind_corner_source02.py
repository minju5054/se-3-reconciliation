"""Synthetic implementation fixtures only; never scientific source evidence."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from shapely.geometry import box
from reconciliation.blind_corner_source02 import construct_bank, canonical_to_world, nominal_poses, attributed_clearance, geometric_probes, geometry_order
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.blind_corner_source import FirstCrossing, old_turning, motion_gate, mismatch
ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def cfg():
    return yaml.safe_load((ROOT/'configs/blind_corner_source_acquisition_02.yaml').read_text())

def test_fixed_three_new_corners_and_no_retry(cfg):
    a=construct_bank(cfg,1.325);assert a==construct_bank(cfg,1.325)
    assert [c['id'] for c in a]==['C03','C04','C05']
    assert not any(np.allclose(c['corner_xy'],[-23.56444,12.82462]) for c in a)
    for key,value in [('retry',True),('candidate_ids',['C02','C04','C05'])]:
        bad=deepcopy(cfg);bad[key]=value
        with pytest.raises(ValueError):construct_bank(bad,1.325)

def test_left_turn_transform_and_finite_spatial_clock(cfg):
    c=construct_bank(cfg,1.325)[0];b=cfg['geometry_bank']
    np.testing.assert_allclose(c['approach_pose'],[4.46-.75,12.348226013183607-.43,0])
    np.testing.assert_allclose(canonical_to_world([.66,-.55],c),[5.01,13.008226013183607])
    for p in c['nominal_paths']:
        xy=np.array(p['poses']);arc=np.linalg.norm(np.diff(xy[:,:2],axis=0),axis=1).sum()
        assert 1.324<arc<=1.325+1e-10
        assert np.all(np.diff(xy[:,2])>=0)
    r=.59;end=b['approach_lead_m']+b['approach_offset_m']-r+np.pi*r/2
    assert nominal_poses([end],r,c,b)[0,2]==pytest.approx(np.pi/2)
    assert len(c['probe_poses'])==11

def test_finite_mesh_clearance_attribution_no_connector():
    base=HospitalEnvironment(box(3,-1,4,1),box(-10,-10,10,10))
    cart=box(.8,-.2,1.2,.2)
    r=attributed_clearance([[0,0],[2,0]],base,cart)
    assert r['Hospital_only_m']==pytest.approx(.8)
    assert r['cart_only_m']==pytest.approx(-.2)
    assert r['limiting_geometry']=='cart'
    assert attributed_clearance([[0,0],[.2,0]],base,cart)['combined_m']==pytest.approx(.4)
    assert not r['connector_included']
    with pytest.raises(ValueError):attributed_clearance([],base,cart)

def test_model_free_nominal_conflict_bypass_and_distance(cfg):
    c=construct_bank(cfg,1.325)[0]
    base=HospitalEnvironment(box(50,50,51,51),box(-100,-100,100,100))
    p=np.array(c['nominal_paths'][1]['poses'])
    cart=box(p[-10,0]-.05,p[-10,1]-.05,p[-10,0]+.05,p[-10,1]+.05)
    r=geometric_probes(c,base,cart,cfg,1.325)
    assert r['nominal_conflict'];assert r['bypass_pass']
    assert not geometric_probes(c,base,box(40,40,41,41),cfg,1.325)['nominal_conflict']
    blocked=HospitalEnvironment(box(2,10,6,13),box(-100,-100,100,100))
    assert not geometric_probes(c,blocked,cart,cfg,1.325)['bypass_pass']

def test_geometry_only_deterministic_order():
    rows=[dict(candidate_id=c,qualified=q,probes={'bypass':{'combined_m':v}}) for c,q,v in [('C04',True,.2),('C03',True,.2),('C05',False,.9)]]
    assert geometry_order(rows)==['C03','C04']
    assert geometry_order([])==[]
    with pytest.raises(ValueError):geometry_order(rows+rows)

def test_first_crossing_no_later_substitution_and_turn_boundary():
    latch=FirstCrossing()
    for i,p in enumerate([0,12,237,500]):latch.observe(i,p,i*15)
    assert latch.allows(2) and not latch.allows(3)
    assert motion_gate([.8,.10],.02,.05)
    assert not motion_gate([.8,.099],.28,.3)
    assert not motion_gate([.20,.5],.28,.3)

def test_arbitrary_N_and_empty_obstacle_region():
    for n in [6,10,17]:
        t=np.linspace(0,np.pi/2,n);raw=np.c_[np.sin(t),1-np.cos(t),t]
        assert old_turning(raw)['qualified']
        scenario=dict(center_xy=[100,100],forward_xy=[1,0],cart_extents=[-.4,.4])
        assert not mismatch(raw,raw+[0,.2,0],scenario)['meaningful']

def test_preflight_has_no_model_controller_optimization_entrypoints():
    source=(ROOT/'scripts/isaac/blind_corner_preflight02.py').read_text()
    for forbidden in ['online_lightnav_worker','online_mpc_worker','MpcTracker','collect_episode','scipy.optimize']:
        assert forbidden not in source
    assert "open('xb')" in source

def test_static_runtime_reuses_first_crossing_guard_and_fixed_order():
    import ast
    source=(ROOT/'scripts/isaac/blind_corner_online02.py').read_text()
    assert 'from blind_corner_online import StaticOcclusion' in source
    assert "protocol['eligible_order'][:protocol['eligible_order'].index(a.candidate)]" in source
    tree=ast.parse(source)
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    activate=[n for n in calls if n.func.id=='set_present'];execute=[n for n in calls if n.func.id=='collect_episode']
    assert len(activate)==len(execute)==1 and activate[0].lineno<execute[0].lineno
    assert activate[0].args[1].value is True
    assert not any(n.func.id in ['rollout','minimize','least_squares','solve_gp'] for n in calls)

def test_historical_C02_negative_facts_remain_preserved():
    # Read-only regression against authoritative saved evidence, not a new experiment.
    import json
    p=ROOT/'data/blind_corner_source_acquisition_01/primary_20260924T054500Z/validation.json'
    if not p.exists():pytest.skip('local immutable corpus not installed')
    v=json.loads(p.read_text());r=v['rows'][0]
    assert v['classification']=='BLIND_CORNER_LIGHTNAV_SOURCE_NOT_QUALIFIED'
    assert not r['qualified'] and r['candidate_id']=='C02'
    assert r['OLD']['geometry_on']['minimum_clearance_m']==pytest.approx(.160775,abs=1e-6)
    assert r['FRESH']['geometry_on']['minimum_clearance_m']==pytest.approx(.015055,abs=1e-6)
    assert r['failure_reasons']==['finite_OLD_cart_obstruction','FRESH_whole_safe','meaningful_change']

def test_preflight_margin_is_stronger_than_scientific_gate(cfg):
    assert cfg['geometry_bank']['bypass_edge_reserve_m']==.10
    assert cfg['scientific_thresholds']['required_edge_clearance_m']==.05
    assert cfg['geometry_bank']['historical_quantile']==.75
    assert cfg['execution']['maximum_terminal_predictions_per_candidate']==2
    assert cfg['execution']['rates_hz']=={'capture':4,'MPC':10,'integration':60}
    assert cfg['execution']['pacing_policy']=='minimum_wall_step_no_catchup_v1'
