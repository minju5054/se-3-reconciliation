"""Synthetic implementation fixtures only; no live model, MPC or experiment evidence."""
import ast
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from reconciliation.obstacle_source03 import target_pose,reveal_due,first_post_reveal_allowed,REPETITIONS,representative,classify
from reconciliation.obstacle_source_acquisition import geometry,qualify
from test_obstacle_source_acquisition import envs,paths,SCENARIO,GATES
ROOT=Path(__file__).resolve().parents[1]
POSE=[19.20312073159454,24.423334915767065,-1.5689742328041627]


def test_exact_authoritative_pose_no_backward_shift_or_reanchor():
    selected=dict(candidate='POSE11',pose_world=POSE.copy())
    result=target_pose(selected)
    assert result==POSE and result is not selected['pose_world']
    np.testing.assert_array_equal(np.array(result,dtype=np.float64).view('u8'),np.array(POSE,dtype=np.float64).view('u8'))
    with pytest.raises(ValueError):target_pose(dict(candidate='different',pose_world=POSE))


@pytest.mark.parametrize('tick,time,active,applied,revealed,want',[
    (15,.25,None,None,False,False),(15,.25,dict(chunk_id='chunk_000'),None,False,False),
    (15,.25,dict(chunk_id='chunk_000'),.25,False,False),
    (15,.25,dict(chunk_id='chunk_000'),.2,False,True),
    (16,.266,dict(chunk_id='chunk_000'),.2,False,False),
    (30,.5,dict(chunk_id='chunk_000'),.2,True,False),
    (30,.5,dict(chunk_id='chunk_001'),.2,False,False)])
def test_strictly_after_actual_OLD_only_scheduled_capture(tick,time,active,applied,revealed,want):
    assert reveal_due(tick,time,active,applied,revealed)==want


def test_queued_or_later_frame_never_substitutes_first_post_reveal():
    assert first_post_reveal_allowed(dict(frame_id='first',rendered_state_id=75),75,'first')
    assert not first_post_reveal_allowed(dict(frame_id='pre',rendered_state_id=60),75,'first')
    assert not first_post_reveal_allowed(dict(frame_id='later',rendered_state_id=90),75,'first')


def test_finite_raw_OLD_never_extrapolated():
    base,on=envs()
    # A straight continuation would hit obstacle, but returned OLD ends before it.
    short=np.array([[.1,0,0],[.2,0,0],[.3,0,0]])
    r=geometry(short,[0,0,0],base,on,SCENARIO);r['stop']=False
    a,b=paths()
    assert not qualify(r,b,SCENARIO,GATES)['gates']['B_OBSTACLE_RELEVANT']
    assert r['geometry_on']['checked_row_count']==3
    assert not r['geometry_on']['connector_included']
    assert qualify(a,b,SCENARIO,GATES)['qualified']


def test_two_repeats_no_positive_reclassification():
    rows=[dict(episode_id=e,qualified=False,valid_scientific_output=True) for e in REPETITIONS]
    assert classify(rows)=='POSE11_OBSTRUCTED_OLD_SOURCE_NOT_QUALIFIED' and representative(rows) is None
    rows[1]['qualified']=True
    assert classify(rows)=='QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE' and representative(rows)=='REPEAT_01'
    rows[0]['qualified']=True;assert representative(rows)=='REPEAT_00'
    with pytest.raises(ValueError):classify(rows[:1])


def test_frozen_request_eligibility_not_pacing_change():
    cfg=yaml.safe_load((ROOT/'configs/obstacle_source_acquisition_03.yaml').read_text())
    assert cfg['repetitions']==list(REPETITIONS) and cfg['retry'] is False
    assert cfg['pacing_policy']=='minimum_wall_step_no_catchup_v1'
    assert cfg['minimum_active_before_prediction_sim_s']==0.
    collector=(ROOT/'scripts/isaac/obstacle_source03_online.py').read_text()
    assert 'from robotless_online_handoffs import Worker,collect_episode' in collector
    assert 'command_guard=geo.guard' in collector
    assert "set_present(prop,False);geo.set(False)" in collector
    assert 'time.sleep' not in collector
    tree=ast.parse(collector)
    loop=next(n for n in ast.walk(tree) if isinstance(n,ast.For) and isinstance(n.iter,ast.Name) and n.iter.id=='specs')
    assert not any(isinstance(n,ast.Break) for n in ast.walk(loop))
    for path in ['scripts/run_obstacle_source_acquisition03.py','scripts/isaac/obstacle_source03_online.py','scripts/validate_obstacle_source_acquisition03.py','src/reconciliation/obstacle_source03.py']:
        text=(ROOT/path).read_text();names={n.id for n in ast.walk(ast.parse(text)) if isinstance(n,ast.Name)}
        assert not names.intersection({'solve_gp','solve_rigid','minimize','GPProblem'})
        assert 'POSE12' not in text


def test_same_legacy_bootstrap_genuine_target_OLD():
    s=(ROOT/'scripts/isaac/robotless_online_handoffs.py').read_text()
    assert "predict=(predictions==0 and history_count==3)" in s
    assert "model.send('frame',frame=frame,predict=True,chunk_id=chunk_id)" in s
    assert "first_activation=sim" in s
    assert "last_activation=sim" in s
    assert s.index('intervention.before_capture') < s.index('activation=proposal')
