"""Synthetic pure/mock contract tests only; not experimental evidence."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import old_fresh_problem_gui_artifacts as a
from reconciliation.old_fresh_problem_gui import (
    CASE_IDS, ReplayState, reconstruct, arrow_segments, agree,
)
from reconciliation.robotless_old_consistent_observation import plan_observation, immediate_metrics
from reconciliation.robotless_single_chunk import observation_to_world
from reconciliation.se2 import wrap_angle


def fixture(yaw=0.):
    raw_old = np.array([[.1,0,0],[.5,0,0],[.8,0,0]])
    raw_fresh = np.array([[0,-.2,.2],[0,.2,.6]])
    r0 = [2.,3.,yaw]
    old = observation_to_world(r0, raw_old)
    plan = plan_observation(r0, r0, old)
    fresh = observation_to_world(plan['R1_old'], raw_fresh)
    previous = {'tau_s':0., 'e_perp_m':0., 'abs_e_dir_deg':0., 'abs_e_yaw_deg':0.}
    saved = {'episode_id':CASE_IDS[0], 'status':'VALID_PAIR', 'plan':plan,
             'metrics':immediate_metrics(plan, fresh, previous)}
    meta = {'episode_id':CASE_IDS[0], 'R0':r0, 'R1':plan['R1_old'], 'planned_R1_old':plan['R1_old']}
    return [CASE_IDS[0], meta, raw_old, raw_fresh, old, fresh, saved]


@pytest.mark.parametrize('case_id', CASE_IDS)
def test_allowed_cases(case_id):
    assert ReplayState(case_id).case_id == case_id


@pytest.mark.parametrize('case_id', ['episode_007','episode_013','006','../episode_006',''])
def test_other_cases_fail_closed(case_id):
    with pytest.raises(ValueError): ReplayState(case_id)
    state = ReplayState()
    with pytest.raises(ValueError): state.select(case_id)
    assert state.case_id == CASE_IDS[0]


def test_augmented_path_connector_once_and_exact_endpoints():
    c = reconstruct(*fixture())
    np.testing.assert_array_equal(c.augmented, np.vstack([c.r0, c.old]))
    np.testing.assert_array_equal(c.marker(0.), c.r0)
    np.testing.assert_array_equal(c.marker(.30), c.b)
    np.testing.assert_allclose(c.marker(.15), [2.15,3.,0.])
    np.testing.assert_allclose(c.b, [2.3,3.,0.])


@pytest.mark.parametrize('s', [-.1,.30000001,np.nan,np.inf])
def test_marker_progress_never_clamps(s):
    with pytest.raises(ValueError): reconstruct(*fixture()).marker(s)


def test_local_window_projection_and_yaw_have_independent_meanings():
    c = reconstruct(*fixture())
    m = c.reconstructed
    assert m['phi_old_local_rad'] == pytest.approx(0.)
    assert m['phi_fresh_local_rad'] == pytest.approx(np.pi/2)
    assert m['phi_old_window_rad'] == pytest.approx(0.)
    assert m['phi_fresh_window_rad'] == pytest.approx(np.pi/2)
    assert m['e_perp_m'] == 0.
    assert m['abs_e_dir_local_deg'] == pytest.approx(90.)
    assert m['abs_e_dir_window_deg'] == pytest.approx(90.)
    assert m['e_yaw_rad'] == pytest.approx(.4)
    assert m['fresh_projection']['segment_index'] == 0
    assert m['fresh_projection']['alpha'] == pytest.approx(.5)
    np.testing.assert_allclose(m['fresh_projection']['Q_xy_world_m'], c.b[:2])


@pytest.mark.parametrize('yaw', [-3.1,-.7,1.3,3.1])
def test_own_observation_frames_and_true_arrow_direction(yaw):
    args = fixture(yaw)
    c = reconstruct(*args)
    np.testing.assert_array_equal(c.fresh, args[5])
    assert c.reconstructed['abs_e_dir_local_deg'] == pytest.approx(90.)
    assert c.reconstructed['e_yaw_rad'] == pytest.approx(.4)
    phi = c.reconstructed['phi_fresh_local_rad']
    segments = arrow_segments(c.reconstructed['fresh_projection']['Q_xy_world_m'], phi)
    vector = np.subtract(segments[0][1], segments[0][0])
    assert np.linalg.norm(vector) == pytest.approx(.24)
    assert wrap_angle(np.arctan2(vector[1],vector[0])-phi) == pytest.approx(0.,abs=1e-14)


@pytest.mark.parametrize('key', ['e_perp_m','abs_e_dir_local_deg','phi_old_window_rad','e_yaw_rad'])
def test_saved_metric_mismatch_fails_closed(key):
    args = fixture()
    args[-1]['metrics'][key] += .001
    with pytest.raises(ValueError, match='saved metric'): reconstruct(*args)


def test_geometry_is_not_built_from_saved_metric_lists():
    args = fixture()
    args[-1]['plan']['augmented_path_world'] = [[999,999,999]]
    c = reconstruct(*args)
    np.testing.assert_array_equal(c.augmented, np.vstack([args[1]['R0'],args[4]]))
    args[-1]['metrics']['fresh_projection']['Q_xy_world_m'][0] += .01
    with pytest.raises(ValueError): reconstruct(*args)


@pytest.mark.parametrize('change', ['raw','world','anchor','boundary','status'])
def test_source_geometry_mismatch_fails_closed(change):
    args = fixture()
    if change == 'raw': args[3][0,0] += .02
    if change == 'world': args[5][0,0] += .02
    if change == 'anchor': args[1]['R1'] = [3,3,0]
    if change == 'boundary': args[1]['planned_R1_old'] = [3,3,0]
    if change == 'status': args[-1]['status'] = 'INVALID'
    with pytest.raises(ValueError): reconstruct(*args)


def test_visualization_leaves_geometry_immutable():
    args = fixture()
    before = [x.copy() for x in args[2:6]]
    c = reconstruct(*args)
    for s in np.linspace(0,.3,31):
        marker = c.marker(s); marker[0] = 999
        arrow_segments(c.b, c.reconstructed['phi_old_local_rad'])
    for original, expected in zip(args[2:6],before): np.testing.assert_array_equal(original,expected)
    for array in (c.r0,c.old,c.fresh,c.augmented,c.b):
        with pytest.raises(ValueError): array.flat[0] = 999


def test_reset_and_case_switch_remove_reveal_and_window():
    s = ReplayState(show_rgb=True)
    s.reveal(); s.show_window = True; s.select(CASE_IDS[1])
    assert s.case_id == CASE_IDS[1] and s.phase == 'READY'
    assert s.progress_m == 0 and not s.revealed and not s.show_window and not s.playing
    assert s.show_rgb  # User presentation preference survives case switches.


def test_reveal_and_pause_are_exact_handoff_actions():
    s = ReplayState(); s.replay(); s.advance(1); s.handoff()
    assert s.progress_m == .30 and s.phase == 'HANDOFF_PAUSE' and not s.revealed and not s.playing
    s.reveal()
    assert s.progress_m == .30 and s.revealed and not s.playing
    s.handoff()
    assert not s.revealed
    s.reset()
    assert s.progress_m == 0 and s.elapsed_s == 0


def test_old_only_stops_at_three_seconds_exactly():
    s = ReplayState(); s.replay(); s.advance(1.5)
    assert s.progress_m == .15 and not s.revealed
    s.advance(1.5)
    assert s.progress_m == .30 and s.elapsed_s == 3 and not s.playing and not s.revealed
    s.advance(100)
    assert s.phase == 'HANDOFF_PAUSE' and s.elapsed_s == 3


def test_full_display_timeline_boundaries():
    s = ReplayState(); s.replay(full=True)
    s.advance(3)
    assert s.phase == 'HANDOFF_PAUSE' and s.playing and not s.revealed
    s.advance(.999)
    assert not s.revealed
    s.advance(.001)
    assert s.phase == 'FRESH_REVEALED' and s.playing and s.revealed
    s.advance(4)
    assert not s.playing and s.revealed and s.elapsed_s == 8 and s.progress_m == .3


@pytest.mark.parametrize('dt', [-1,np.inf,np.nan])
def test_invalid_display_time(dt):
    with pytest.raises(ValueError): ReplayState().advance(dt)


def test_large_clock_step_has_exact_endpoint_and_finite_finish():
    s = ReplayState(); s.replay(full=True); s.advance(100)
    assert s.elapsed_s == 8 and s.progress_m == .3 and s.revealed and not s.playing


def test_source_hash_inventory_addition_mutation_and_removal(tmp_path):
    path = tmp_path/'raw.npy'; path.write_bytes(b'fixture only')
    expected = a.inventory(tmp_path)
    a.verify_hashes(tmp_path,expected)
    path.write_bytes(b'modified fixture')
    with pytest.raises(ValueError): a.verify_hashes(tmp_path,expected)
    path.write_bytes(b'fixture only'); extra = tmp_path/'extra'; extra.touch()
    with pytest.raises(ValueError): a.verify_hashes(tmp_path,expected)
    extra.unlink(); path.unlink()
    with pytest.raises(ValueError): a.verify_hashes(tmp_path,expected)


def test_missing_source_never_falls_back(tmp_path):
    with pytest.raises(ValueError,match='missing'): a.verify_source(tmp_path/'absent')


def test_invalid_source_validator_fails_closed(tmp_path,monkeypatch):
    import validate_robotless_old_consistent as validator
    monkeypatch.setattr(validator,'validate_run',lambda _: {'status':'INVALID'})
    with pytest.raises(ValueError,match='validation failed'): a.verify_source(tmp_path)


def test_validator_cannot_mutate_source(tmp_path,monkeypatch):
    import validate_robotless_old_consistent as validator
    def mutate(_):
        (tmp_path/'unexpected').touch()
        return {'status':a.SOURCE_VALIDATED}
    monkeypatch.setattr(validator,'validate_run',mutate)
    with pytest.raises(ValueError,match='changed during'): a.verify_source(tmp_path)


def test_output_cannot_overwrite_or_nest_with_source(tmp_path):
    source = tmp_path/'source'; source.mkdir()
    for output in (source,source/'child',tmp_path):
        with pytest.raises(ValueError): a.guard_output(output,source)
    output = tmp_path/'new'
    assert a.guard_output(output,source) == output
    output.mkdir()
    with pytest.raises(FileExistsError): a.guard_output(output,source)


def test_compare_missing_and_nonfinite_metrics():
    with pytest.raises(ValueError): agree({'alpha':.1},{})
    with pytest.raises(ValueError): agree(np.nan,np.nan)
