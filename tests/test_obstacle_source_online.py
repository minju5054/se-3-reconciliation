"""Synthetic scheduling/qualification tests, never model evidence."""
import ast
from pathlib import Path
import numpy as np
import pytest
from reconciliation.obstacle_source_online import initial_pose,reveal_due,first_post_reveal_allowed,representative,REPETITIONS


def test_fixed_backwards_pose_no_anchor_change():
    p=[19.20312073159454,24.423334915767065,-1.5689742328041627];orig=p.copy();f=[.00223269010175,-.999997507544]
    actual=initial_pose(p,f)
    np.testing.assert_allclose(np.asarray(actual)[:2],np.asarray(p)[:2]-.4*np.asarray(f))
    assert actual[2]==p[2] and p==orig


def test_reveal_needs_active_old_and_plane():
    assert not reveal_due([0,0,0],[1,0,0],[1,0],True,False)
    assert not reveal_due([1,0,0],[1,0,0],[1,0],False,False)
    assert reveal_due([1,0,0],[1,0,0],[1,0],True,False)
    assert not reveal_due([2,0,0],[1,0,0],[1,0],True,True)


def test_first_post_reveal_only_not_queue_or_later_retry():
    for state,fid,wanted in [(29,'pre',False),(30,'first',True),(45,'later',False)]:
        assert first_post_reveal_allowed(dict(rendered_state_id=state,frame_id=fid),30,'first')==wanted
    assert not first_post_reveal_allowed(dict(rendered_state_id=30,frame_id='first'),None,None)


def test_both_repetitions_first_qualified_selection():
    a=[dict(episode_id=e,qualified=True) for e in REPETITIONS]
    assert representative(a)=='REPEAT_00'
    a[0]['qualified']=False;assert representative(a)=='REPEAT_01'
    a[1]['qualified']=False;assert representative(a) is None
    with pytest.raises(ValueError):representative(a[:1])
    with pytest.raises(ValueError):representative(a[::-1])


def test_no_phaseA_fallback_no_optimization():
    root=Path(__file__).resolve().parents[1]
    for f in ('src/reconciliation/obstacle_source_online.py','scripts/isaac/obstacle_source02_online.py','scripts/validate_obstacle_source02_online.py'):
        s=(root/f).read_text();names={n.id for n in ast.walk(ast.parse(s)) if isinstance(n,ast.Name)}
        assert not names.intersection({'solve_gp','solve_rigid','minimize','GPProblem'})
        assert 'POSE12' not in s
    s=(root/'scripts/isaac/obstacle_source02_online.py').read_text()
    loop=next(n for n in ast.walk(ast.parse(s)) if isinstance(n,ast.For) and isinstance(n.iter,ast.Name) and n.iter.id=='specs')
    assert not any(isinstance(n,ast.Break) for n in ast.walk(loop))
