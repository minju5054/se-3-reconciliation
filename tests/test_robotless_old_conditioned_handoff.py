"""Synthetic math fixtures; not experimental evidence."""
from pathlib import Path
import copy

import numpy as np
import pytest
import yaml

from reconciliation import robotless_old_conditioned_handoff as m
from reconciliation.robotless_handoff_screening import transition_metrics
from reconciliation.se2 import compose_poses, wrap_angle


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).parents[1]/'configs/robotless_old_conditioned_handoff.yaml').read_text())


def characterize(config, old, fresh=None, r_obs=(.2, .1, 0), episode_id='episode_000'):
    if fresh is None: fresh=[[.1,0,0],[1,0,0],[2,0,0]]
    ep={'episode_id':episode_id,'category':'straight'}
    previous=transition_metrics(ep,r_obs,fresh,'a'*64,'b'*64)
    return m.characterize_episode(ep,r_obs,old,fresh,previous,config)


def test_interior_anchor_alignment_and_no_snap(config):
    old=np.array([[0,0,0],[1,0,0],[2,0,0]])
    result=characterize(config,old)
    ref=result['reference'];assert ref['segment_index']==0 and ref['alpha_obs']==pytest.approx(.2)
    assert ref['Q_old_obs_xy_world_m']==pytest.approx([.2,0])
    assert ref['s_old_obs_m']==pytest.approx(.2) and ref['d_obs_to_old_m']==pytest.approx(.1)
    assert compose_poses(ref['T_align_pose'],ref['A_obs'])==pytest.approx(ref['R_obs'])
    assert result['conditions'][0]['B_old']==[.2,.1,0]
    assert result['conditions'][-1]['B_old']==pytest.approx([.45,.1,0])


@pytest.mark.parametrize('r_obs,expected,remaining',[([-1,.2,0],[0,0],1.),([2,.2,0],[1,0],0.)])
def test_endpoint_projection_and_exhaustion(config,r_obs,expected,remaining):
    result=characterize(config,[[0,0,0],[1,0,0]],r_obs=r_obs)
    assert result['reference']['Q_old_obs_xy_world_m']==expected
    assert result['reference']['available_remaining_m']==remaining
    assert result['conditions'][0]['B_old']==r_obs
    if remaining==0:
        for row in result['conditions'][1:]:
            assert row['status']=='OLD_CONTINUATION_EXHAUSTED' and row['B_old'] is None
            assert row['requested_delta_s_m']>row['available_remaining_m']
            assert all(row[k] is None for k in m.OLD_METRICS+m.PAIRED_METRICS)


def test_shortest_angle_yaw_interpolation_and_exact_vertex():
    path=[[0,0,np.deg2rad(170)],[1,0,np.deg2rad(-170)],[1,1,0]]
    assert m.interpolate_arc(path,.5)['pose'][2]==pytest.approx(-np.pi)
    sample=m.interpolate_arc(path,1)
    assert sample['segment_index']==0 and sample['alpha']==1
    assert m.interpolate_arc(path,1.5)['pose'][:2]==pytest.approx([1,.5])


def test_old_anchor_uses_projected_shortest_yaw(config):
    result=characterize(config,[[0,0,3.],[1,0,-3.]],r_obs=[.5,0,.7])
    assert result['reference']['A_obs'][2]==pytest.approx(-np.pi)
    assert result['conditions'][0]['B_old']==[.5,0,.7]


def test_curved_old_arc_advance_and_tangent_separate_from_yaw(config):
    result=characterize(config,[[0,0,0],[.3,0,0],[.3,1,0]],r_obs=[.2,0,0])
    row=result['conditions'][-1]
    assert row['old_arc_sample']['s_m']==pytest.approx(.45)
    assert row['B_old']==pytest.approx([.3,.15,0])
    assert row['phi_old_local_rad']==pytest.approx(np.pi/2)
    assert row['B_old'][2]==0  # Incoming geometry is deliberately not pose yaw.


@pytest.mark.parametrize('yaw',[-np.pi,-2.,.5,np.pi/2])
def test_arbitrary_world_yaw_equivariance_and_source_immutability(config,yaw):
    origin=np.array([4.,-2.,yaw]);old=compose_poses(origin,[[0,0,.2],[.3,0,.3],[.3,1,.4]])
    fresh=compose_poses(origin,[[.1,0,0],[1,0,0]]);obs=compose_poses(origin,[.15,.04,.25])
    old_before=old.copy();fresh_before=fresh.copy();old.setflags(write=False);fresh.setflags(write=False)
    # Avoid an equidistant segment bisector: exact ties can change under roundoff.
    local=characterize(config,[[0,0,.2],[.3,0,.3],[.3,1,.4]],[[.1,0,0],[1,0,0]],[.15,.04,.25])
    world=characterize(config,old,fresh,obs)
    for a,b in zip(local['conditions'],world['conditions'],strict=True):
        assert b['B_old']==pytest.approx(compose_poses(origin,a['B_old']))
        for key in m.OLD_METRICS+m.PAIRED_METRICS: assert b[key]==pytest.approx(a[key],abs=1e-10)
    assert np.array_equal(old,old_before) and np.array_equal(fresh,fresh_before)


def test_exhaustion_strict_and_equal_remaining_is_available(config):
    result=characterize(config,[[0,0,0],[.25,0,0]],r_obs=[0,0,0])
    assert result['conditions'][-1]['status']=='AVAILABLE'
    short=characterize(config,[[0,0,0],[np.nextafter(.25,0),0,0]],r_obs=[0,0,0])
    assert short['conditions'][-1]['status']=='OLD_CONTINUATION_EXHAUSTED'
    with pytest.raises(ValueError,match='outside'):m.interpolate_arc([[0,0,0],[.2,0,0]],.25)


@pytest.mark.parametrize('s,angle,span',[(0,0,.05),(.5,0,.1),(1,0,.05)])
def test_window_straight_endpoints(s,angle,span):
    w=m.window_tangent([[0,0,0],[1,0,0]],s)
    assert w['available'] and w['phi_rad']==angle and w['span_m']==pytest.approx(span)


def test_window_across_corner_retains_local_difference():
    path=[[0,0,0],[1,0,0],[1,1,0]]
    assert m.interpolate_arc(path,1)['phi_local_rad']==0
    assert m.window_tangent(path,1)['phi_rad']==pytest.approx(np.pi/4)


def test_short_segment_tangent_preserved_and_window_separate(config):
    old=[[0,0,0],[0,1e-8,0],[1,1e-8,0]]
    result=characterize(config,old,r_obs=[0,0,0]);row=result['conditions'][0]
    assert row['phi_old_local_rad']==pytest.approx(np.pi/2)
    assert row['phi_old_window_rad']==pytest.approx(2e-7,abs=1e-12)
    assert row['old_arc_sample']['segment_length_m']==1e-8


def test_degenerate_winner_local_unavailable_no_invented_tangent(config):
    result=characterize(config,[[0,0,0],[0,0,.2],[1,0,.2]],r_obs=[0,0,0])
    row=result['conditions'][0]
    assert row['phi_old_local_rad'] is None and row['abs_e_dir_old_deg'] is None
    assert row['old_window']['available'] and row['B_old']==[0,0,0]


@pytest.mark.parametrize('path,s,reason',[
    ([[0,0,0],[1e-13,0,0]],0,'span'),
    ([[0,0,0],[.05,0,0],[0,0,0]],.05,'chord')])
def test_unavailable_window_explicit_reason(path,s,reason):
    result=m.window_tangent(path,s)
    assert not result['available'] and result['phi_rad'] is None and reason in result['reason']


def test_fresh_is_fixed_world_and_paired_difference_arithmetic(config):
    fresh=np.array([[1,1,.1],[2,1,.2]])
    result=characterize(config,[[0,0,0],[1,0,0]],fresh,r_obs=[0,0,0])
    for row in result['conditions']:
        assert row['fresh_projection']['Q_xy_world_m']==[1,1]
        for delta,old,straight in zip(m.PAIRED_METRICS,('e_perp_old_m','abs_e_dir_old_deg','abs_e_yaw_old_deg'),m.STRAIGHT_METRICS):
            assert row[delta]==pytest.approx(row[old]-row[straight])


def test_duplicate_groups_individual_availability_and_representatives(config):
    a=characterize(config,[[0,0,0],[1,0,0]],episode_id='episode_006')
    b=copy.deepcopy(a);b['episode_id']='episode_016'
    for row in b['conditions']:
        row['episode_id']='episode_016';row['e_perp_old_m']+=2
    summary=m.summarize([a,b])
    assert summary['episode_count']==2 and summary['unique_pair_count']==1
    assert summary['unique_pairs'][0]['count']==2
    assert summary['unique_pairs'][0]['e_perp_old_m']==pytest.approx(a['conditions'][0]['e_perp_old_m']+1)
    assert m.select_representatives([a,b])==m.select_representatives([b,a])
    reps=m.select_representatives([a,b]);assert reps[2]['episode_id']=='episode_016' and reps[3]['episode_id']=='episode_006'
    b['conditions'][1]['abs_e_dir_old_deg']=None
    assert m.summarize([a,b])['statistics']['episode_weighted'][1]['metrics']['abs_e_dir_old_deg']['n_unavailable']==1
    with pytest.raises(ValueError):m.summarize([a,a])


@pytest.mark.parametrize('n',[2,7,31])
def test_arbitrary_old_fresh_lengths(config,n):
    old=np.column_stack([np.linspace(0,2,n),np.zeros(n),np.zeros(n)])
    assert len(characterize(config,old)['conditions'])==4


@pytest.mark.parametrize('bad',[np.nan,np.inf,-np.inf])
def test_nonfinite_rejected(config,bad):
    with pytest.raises(ValueError):characterize(config,[[0,0,0],[bad,0,0]])
    with pytest.raises(ValueError):m.interpolate_arc([[0,0,0],[1,0,0]],bad)
    with pytest.raises(ValueError):m.window_tangent([[0,0,0],[1,0,0]],bad)


def test_zero_total_length_rejected(config):
    with pytest.raises(ValueError,match='arc length'):characterize(config,[[0,0,0],[0,0,.1]])
