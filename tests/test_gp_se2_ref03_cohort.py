"""Source-only cohort protocol tests; synthetic cases are not performance evidence."""
from copy import deepcopy
import hashlib
from pathlib import Path
import random

import numpy as np
import pytest
import yaml

from reconciliation.gp_se2_reference import geometric_group_flags
from reconciliation.gp_se2_ref03_cohort import (
    CONTROL_CASES, GROUPS, STRATA, build_cohort, cohort_policy,
    prior_exclusion_inventory, select_additional, suffix_features,
)


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/gp_se2_01.yaml').read_text())


def record(case,groups,episode=None,pair=None,rotation=False,eligible=True):
    return dict(case_id=case,episode_id=episode or case.split('/')[0],ordered_raw_pair=pair or case,
        group_memberships={g:g in groups for g in GROUPS},rotation_dominant=rotation,
        source_integrity_valid=True,additional_eligible=eligible)


def feature_fixture(config,suffix,*,eperp=0.,angle=0.,reliable=True,clearance=.15,valid=True,shape=None):
    path=np.vstack(([-99.,0.,2.8],np.asarray(suffix,float)))
    metric=dict(e_perp_m=eperp,abs_e_dir_window_deg=angle,abs_e_yaw_deg=0.,
        incoming_execution=dict(available=reliable,chord_m=.1),
        fresh_projection={'window_0_10m':dict(available=True,chord_m=.1)})
    shape=shape or dict(arc_m=1.,chord_over_arc=1.,yaw_travel_from_observation_deg=0.,max_forward_direction_deviation_deg=0.)
    original=dict(raw_shape=shape,groups=geometric_group_flags(metric,shape,config['selection']),
        e_perp_m=eperp,direction_difference_deg=angle)
    env=dict(clearance_valid=valid,minimum_clearance_m=clearance)
    return path,metric,original,env


def features(config,suffix,**kwargs):
    path,metric,original,env=feature_fixture(config,suffix,**kwargs)
    return suffix_features(path,path[0],metric,original,config,env)


def test_policy_fixed_counts_order_controls_diversity_and_no_tuning():
    p=cohort_policy()
    assert p['source_count']==881 and p['target_per_group']==6
    assert p['group_order']==['O','R','P','S']
    assert [case for _,case in CONTROL_CASES]==['episode_014_repeat_01/handoff_026','episode_001_repeat_01/handoff_002','episode_017_repeat_00/handoff_007']
    assert p['diversity_sets_initialized_from'].startswith('additional selections only')
    assert p['O']['boundary_connector_included'] is False
    assert p['R']['rotation_dominant_is_alternative_eligibility'] is False
    assert all(p[k] is False for k in ('duplicate_case_selection','group_replacement','threshold_relaxation','performance_or_rollout_results_used','synthetic_substitution'))
    p['group_order'].reverse()
    assert cohort_policy()['group_order']==list(GROUPS)


def test_prior_exclusions_union_events_only_with_multiple_usage_labels():
    manifests={k:dict(selected=[dict(case_id='ep/h0')]) for k in ('GP01','GP02','REF01','REF02')}
    manifests['GP01']['selected'].append(dict(case_id='ep/h1'))
    out=prior_exclusion_inventory(manifests)
    assert out['case_ids']==['ep/h0','ep/h1'] and out['union_count']==2
    assert out['membership']['ep/h0']==['GP01','GP02','REF01','REF02']
    with pytest.raises(ValueError,match='coverage'):
        prior_exclusion_inventory({k:v for k,v in manifests.items() if k!='REF02'})
    manifests['GP02']['selected'].append(dict(case_id='ep/h0'))
    with pytest.raises(ValueError,match='duplicate'):
        prior_exclusion_inventory(manifests)


def test_suffix_geometry_excludes_removed_prefix_and_boundary_connector(config):
    out=features(config,[[0.,0.,0.],[0.,0.,np.deg2rad(20.)]])
    assert out['first_future_original_row_index']==1
    assert out['original_suffix_row_indices']==[1,2]
    assert out['suffix_xy_arc_m']==0.
    assert out['suffix_internal_accumulated_abs_wrapped_yaw_deg']==pytest.approx(20.)
    assert not out['group_memberships']['R']
    assert out['boundary_connector_included'] is False


def test_R_requires45_even_when_rotation_dominant_above30(config):
    out=features(config,[[0.,0.,0.],[.1,0.,np.deg2rad(35.)]])
    assert out['rotation_dominant'] and not out['group_memberships']['R']
    out=features(config,[[0.,0.,0.],[.1,0.,np.deg2rad(45.)]])
    assert out['rotation_dominant'] and out['group_memberships']['R']
    out=features(config,[[0.,0.,0.],[.3,0.,np.deg2rad(60.)]])
    assert not out['rotation_dominant'] and out['group_memberships']['R']


def test_yaw_is_internal_absolute_wrapped_variation_not_net_turn_or_capture_yaw(config):
    out=features(config,[[0.,0.,np.deg2rad(170.)],[0.,0.,np.deg2rad(-170.)],[0.,0.,np.deg2rad(150.)]])
    assert out['suffix_internal_accumulated_abs_wrapped_yaw_deg']==pytest.approx(60.)
    assert out['group_memberships']['R'] and out['rotation_dominant']
    out=features(config,[[0.,0.,2.5]])
    assert out['suffix_xy_arc_m']==0 and out['suffix_internal_accumulated_abs_wrapped_yaw_deg']==0
    assert not out['group_memberships']['R']


@pytest.mark.parametrize('clearance,valid,wanted',[(.15,True,True),(.15000001,True,False),(.051,True,True),(.01,False,False)])
def test_O_requires_original_suffix_clearance_valid_and_fixed_proximity(config,clearance,valid,wanted):
    out=features(config,[[0.,0.,0.]],clearance=clearance,valid=valid)
    assert out['group_memberships']['O'] is wanted


@pytest.mark.parametrize('eperp,angle,reliable,wanted',[(.10,0.,False,True),(.0999,30.,True,True),(.0999,45.,False,False),(.0999,None,True,False)])
def test_P_uses_saved_position_or_reliable_window_direction_only(config,eperp,angle,reliable,wanted):
    out=features(config,[[0.,0.,0.]],eperp=eperp,angle=angle,reliable=reliable)
    assert out['group_memberships']['P'] is wanted


def test_S_is_exact_original_flag_and_source_catalog_disagreement_is_error(config):
    path,metric,original,env=feature_fixture(config,[[0.,0.,0.],[1.,0.,0.]])
    out=suffix_features(path,path[0],metric,original,config,env)
    assert out['group_memberships']['S']==original['groups']['A_SMALL_STRAIGHT']
    original['groups']['A_SMALL_STRAIGHT']=False
    with pytest.raises(ValueError,match='original geometric'):
        suffix_features(path,path[0],metric,original,config,env)


def test_selection_deterministic_under_source_iteration_order_and_ignores_outcomes():
    rows=[record(f'ep_{i}/h0',GROUPS) for i in range(35)]
    expected=select_additional(rows)
    assert len(expected['selected'])==24
    assert [r['selected_group'] for r in expected['selected']]==sum(([g]*6 for g in GROUPS),[])
    assert len({r['case_id'] for r in expected['selected']})==24
    shuffled=deepcopy(rows);random.Random(876).shuffle(shuffled)
    for row in shuffled:
        row['hypothetical_future_success']=False
        row['selection_hash']='f'*64
    actual=select_additional(shuffled)
    assert [r['case_id'] for r in actual['selected']]==[r['case_id'] for r in expected['selected']]
    first=min(rows,key=lambda r:(hashlib.sha256(('REF03-v1:'+r['case_id']).encode()).hexdigest(),r['case_id']))
    assert expected['selected'][0]['case_id']==first['case_id']
    for row in expected['selected']:
        assert row['cohort']=='ADDITIONAL_TRANSFER'
        assert row['stratum']==STRATA[row['selected_group']]


def test_group_assignment_global_diversity_episode_then_pair_then_Rrotation():
    rows=[record('used/h0','O',episode='used',pair='usedpair'),
          record('a/h0','R',episode='used',pair='newpair1',rotation=True),
          record('b/h0','R',episode='b',pair='usedpair',rotation=True),
          record('c/h0','R',episode='c',pair='newpair2',rotation=False),
          record('d/h0','R',episode='d',pair='newpair3',rotation=True)]
    chosen=select_additional(rows)['selected']
    assert [r['case_id'] for r in chosen]==['used/h0','d/h0','c/h0','b/h0','a/h0']


def test_R_rotation_priority_never_overrides_episode_or_pair_preference():
    rows=[record('o/h0','O',episode='episode',pair='pair'),
          record('r_used/h0','R',episode='episode',pair='other',rotation=True),
          record('r_new/h0','R',episode='different',pair='other2',rotation=False)]
    selected=select_additional(rows)['selected']
    assert selected[1]['case_id']=='r_new/h0'


def test_overlap_assigned_once_shortfalls_not_filled_from_other_groups():
    rows=[record('a/h0','OR'),record('b/h0','P'),record('c/h0','S'),record('d/h0','',eligible=True)]
    result=select_additional(rows)
    assert [(r['case_id'],r['selected_group']) for r in result['selected']]==[('a/h0','O'),('b/h0','P'),('c/h0','S')]
    assert [r['shortfall'] for r in result['group_shortfalls']]==[5,6,5,5]
    assert result['group_shortfalls'][1]['available_after_previous_groups']==0


def test_integrity_blocker_is_not_ordinary_eligibility_exclusion_or_shortfall():
    good=record('a/h0','O');bad=record('b/h0','S',eligible=False);bad['source_integrity_valid']=False
    with pytest.raises(ValueError,match='source-integrity blocker'):
        select_additional([good,bad])
    bad['source_integrity_valid']=True
    selected=select_additional([good,bad])
    assert [r['case_id'] for r in selected['selected']]==['a/h0']
    with pytest.raises(ValueError,match='duplicate'):
        select_additional([good,good])


def test_source_integrity_failure_preserved_without_synthetic_or_selection(tmp_path,config):
    manifest=tmp_path/'missing_manifest.json'
    result=build_cohort(manifest,{},source_root=tmp_path,environment=None,original_config=config,expected_source_hashes={})
    assert result['status']=='SOURCE_INTEGRITY_BLOCKED'
    assert not result['source_integrity_valid'] and result['source_integrity_blockers']
    assert result['selected']==[] and result['all_candidates']==[]
