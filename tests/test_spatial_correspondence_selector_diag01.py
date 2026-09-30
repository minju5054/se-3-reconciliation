"""Synthetic numerical/selector fixtures plus read-only historical authentication."""
import ast
import copy
import inspect
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from reconciliation.spatial_correspondence_selector import (
    SpatialCurve, POSITION_SCALE, YAW_SCALE, TIE_ATOL, DENSE_SCORE_ATOL, RULES,
    METHODS, SOURCE_IDS, PNGS, official_selector, diagnostic_guard, matched_queries,
    reset_summary, classification, wrap)
from run_spatial_correspondence_selector_diag01 import CONFIG, authenticate
from reconciliation.join_source03 import read, sha

OFFICIAL = ROOT.parent/'external/LightNav-0-official-demo/mujoco_demo/vln_mujoco/mpc.py'


def line():
    return SpatialCurve([[0, 0, 0], [1, 0, 0], [2, 0, 0]])


def test_known_continuous_XY_and_no_snap():
    result = line().correspondences([.37, .2, 0])['C1_XY_PROJECTION']
    assert result['arc_m'] == pytest.approx(.37)
    assert result['alpha'] == pytest.approx(.37)
    assert result['position_gap_m'] == pytest.approx(.2)
    assert result['nearest_raw_row_diagnostic'] == 0


def test_shortest_yaw_interpolation():
    c = SpatialCurve([[0, 0, math.radians(179)], [1, 0, math.radians(-179)]])
    assert abs(c.pose(.5)[2]) == pytest.approx(math.pi)
    assert c.yaw_delta[0] == pytest.approx(math.radians(2))


def test_score_exact_attachment_normalizations():
    c = line()
    assert POSITION_SCALE == .10 and YAW_SCALE == math.radians(15)
    assert c.score([0, .10, YAW_SCALE], 0) == pytest.approx(2)


def test_forward_constraint_has_effect():
    c = SpatialCurve([[0, 0, 0], [1, 0, 2]])
    r = c.correspondences([.8, 0, 0])
    assert r[RULES[2]]['arc_m'] < r[RULES[1]]['arc_m']
    assert r[RULES[3]]['arc_m'] >= r[RULES[1]]['arc_m']


def test_tie_earliest_arc():
    c = SpatialCurve([[0, 0, 0], [1, 0, 0], [0, 0, 0]])
    for r in c.correspondences([0, .1, 0]).values():
        assert r['arc_m'] == 0


@pytest.mark.parametrize('B_yaw', [-math.pi+1e-10, math.pi-1e-10, 0.])
def test_branch_boundaries_with_dense_oracle(B_yaw):
    c = SpatialCurve([[0, 0, -1.8], [.5, .1, 1.2]])
    B = np.array([.25, -.03, B_yaw])
    arc = c.minimize(B, pose_score=True)
    t = np.linspace(0, 1, 200001)
    xy = c.fresh[0, :2]+t[:, None]*c.delta[0]
    theta = c.fresh[0, 2]+t*c.yaw_delta[0]
    scores = np.sum((xy-B[:2])**2, axis=1)/.1**2+(wrap(B[2]-theta)/YAW_SCALE)**2
    assert abs(c.score(B, arc)-float(scores.min())) < DENSE_SCORE_ATOL


def test_random_dense_oracle_repeatability():
    rng = np.random.default_rng(731)
    for _ in range(20):
        fresh = np.c_[np.cumsum(rng.normal(size=(5, 2))*.2, axis=0), rng.uniform(-math.pi, math.pi, 5)]
        c = SpatialCurve(fresh)
        B = np.r_[rng.normal(size=2)*.2, rng.uniform(-math.pi, math.pi)]
        result = c.correspondences(B)
        assert result == c.correspondences(B)
        for rule, lower in [(RULES[2], 0.), (RULES[3], result[RULES[1]]['arc_m'])]:
            values = []
            for j in range(len(c.lengths)):
                if c.arc[j+1] < lower:
                    continue
                ts = np.linspace(max(0., (lower-c.arc[j])/c.lengths[j]), 1, 100001)
                xy = fresh[j, :2]+ts[:, None]*c.delta[j]
                theta = fresh[j, 2]+ts*c.yaw_delta[j]
                scores = np.sum((xy-B[:2])**2, axis=1)/.1**2+(wrap(B[2]-theta)/YAW_SCALE)**2
                values.append(float(scores.min()))
            assert abs(result[rule]['pose_score']-min(values)) < DENSE_SCORE_ATOL


@pytest.mark.parametrize('n', [2, 3, 10, 25])
def test_generic_nodes_immutable_untimed(n):
    fresh = np.c_[np.linspace(0, 2, n), np.zeros(n), np.zeros(n)]
    before = fresh.copy()
    c = SpatialCurve(fresh)
    result = c.correspondences([.4, .1, .2])
    np.testing.assert_array_equal(fresh, before)
    assert 'time' not in inspect.signature(SpatialCurve).parameters
    assert all('time' not in k and 'dt' not in k for r in result.values() for k in r)
    with pytest.raises(ValueError):
        c.fresh[0, 0] = 5


@pytest.mark.parametrize('fresh', [[[0, 0, 0]], [[0, 0, 0], [0, 0, 1]], [[0, 0, 0], [0, 0, 0]]])
def test_degenerate_arc_fails_without_removing_rows(fresh):
    with pytest.raises(ValueError):
        SpatialCurve(fresh)


def test_vertex_earlier_segment_and_endpoint():
    c = line()
    assert c.locate(1) == (0, 1.)
    assert c.locate(2) == (1, 1.)
    assert c.describe([3, 0, 0], 2)['remaining_arc_m'] == 0


def test_pure_exact_official_extraction():
    function, meta = official_selector(OFFICIAL)
    assert meta['horizon'] == 5 and meta['weights'] == [10., 10., 1.]
    assert set(meta['functions']) == {'wrap_angle', 'build_pose_aligned_reference'}
    assert set(function.__globals__) == {'np', 'math', '__builtins__', 'wrap_angle', 'build_pose_aligned_reference'}
    assert function.__code__.co_filename == str(OFFICIAL)


def test_selector_authentication_fails_closed(tmp_path):
    source = tmp_path/'mpc.py'
    source.write_text(OFFICIAL.read_text()+'\n# modified\n')
    with pytest.raises(ValueError, match='hash mismatch'):
        official_selector(source)


def synthetic_matched():
    fresh = np.c_[np.arange(10)*.1, np.zeros(10), np.zeros(10)]
    refs = {n:fresh.copy() for n in METHODS}
    refs[METHODS[1]][:, 0] += .1
    refs[METHODS[2]][:, 0] += .2
    states = [dict(tick=i, time_after_B_s=(i-10)*.1, pose_world=[.31+(i-10)*.05, 0, 0]) for i in [10, 11, 12]]
    return SpatialCurve(fresh), refs, states


def test_matched_identical_poses_identity_and_no_mutation():
    curve, refs, states = synthetic_matched()
    before = copy.deepcopy((refs, states))
    selector, _ = official_selector(OFFICIAL)
    with diagnostic_guard() as calls:
        rows = matched_queries(curve, refs, states, selector)
    assert calls['selector_queries'] == 9
    assert all(v == 0 for k, v in calls.items() if k != 'selector_queries')
    for i, state in enumerate(states):
        for r in rows[3*i:3*i+3]:
            assert r['matched_pose_world'] == state['pose_world']
            assert r['original_node_identities'] == r['H5_rows']
            assert r['original_H5_arcs_m'] == curve.arc[r['H5_rows']].tolist()
    for n in METHODS:
        np.testing.assert_array_equal(refs[n], before[0][n])
    assert states == before[1]


def test_reset_negative_means_backward_original_progress():
    curve, refs, states = synthetic_matched()
    selector, _ = official_selector(OFFICIAL)
    result = reset_summary(matched_queries(curve, refs, states, selector))
    half, full = [result['methods'][n] for n in METHODS[1:]]
    assert full['negative_first_count'] == 3 and full['persistent_two_ticks']
    assert full['max_backward_first_arc_m'] > half['max_backward_first_arc_m']
    assert full['first_different_tick'] == 10
    assert result['first_Half_Full_different_tick'] == 10


def test_endpoint_repetition_and_wrapped_physical_rows():
    c = line()
    selector, _ = official_selector(OFFICIAL)
    states = [dict(tick=1, time_after_B_s=0., pose_world=[2, 0, math.pi*2])]
    rows = matched_queries(c, {n:c.fresh for n in METHODS}, states, selector)
    assert rows[0]['H5_rows'] == [2]*5 and rows[0]['endpoint_repeated']
    assert rows[0]['physical_H5_world'][0][2] == pytest.approx(math.pi*2)
    assert rows[0]['physical_H5_wrapped_world'][0][2] == 0


def test_guard_stops_real_optimizer_before_numerical_body():
    from reconciliation.graph_optimizer import solve_least_squares
    with pytest.raises(RuntimeError, match='forbids'):
        with diagnostic_guard():
            solve_least_squares(None, None, None)


@pytest.mark.parametrize('name,filename', [('_solve', str(OFFICIAL)), ('__init__', str(OFFICIAL)),
    ('submit', '/tmp/online_mpc_adapter.py'), ('poll', '/tmp/online_mpc_adapter.py'),
    ('minimize', '/tmp/scipy/optimize/solver.py'), ('integrate_unicycle', '/tmp/runtime.py'), ('apply_command', '/tmp/runtime.py')])
def test_guard_stops_MPC_memory_integration_and_commands(name, filename):
    namespace = {}
    exec(compile(f'def {name}():\n    raise AssertionError("body must never run")', filename, 'exec'), namespace)
    with pytest.raises(RuntimeError, match='forbids'):
        with diagnostic_guard():
            namespace[name]()


def test_safety_diagnostic_only_does_not_reject_correspondence():
    c = line()
    B = [.5, .3, 0]
    target = c.correspondences(B)[RULES[1]]
    checks = []
    class Unsafe:
        def check_polyline(self, points, **kwargs):
            checks.append((points.copy(), kwargs))
            return dict(minimum_clearance_m=-.02, clearance_valid=False)
    safety = c.safety(B, target, Unsafe())
    assert not safety['selection_rejected'] and not safety['suffix_is_executed']
    assert not safety['connector_geometric_threshold_met']
    assert target['arc_m'] == .5 and safety['label'] == 'hypothetical straight connector diagnostic only'
    np.testing.assert_array_equal(checks[1][0], [[.5, 0, 0], [1, 0, 0], [2, 0, 0]])
    assert all(k == dict(radius=.20, required_clearance=.05) for _, k in checks)


def test_frozen_protocol_constants():
    cfg = yaml.safe_load(CONFIG.read_text())
    assert cfg['position_scale_m'] == POSITION_SCALE and math.radians(cfg['yaw_scale_deg']) == YAW_SCALE
    assert cfg['candidate_score_tie_absolute_tolerance'] == TIE_ATOL
    assert cfg['dense_synthetic_score_tolerance'] == DENSE_SCORE_ATOL
    assert cfg['final_pngs'] == PNGS and cfg['expected_scientific_selector_queries'] == 111
    assert cfg['sources'] == SOURCE_IDS


def test_historical_references_frames_and_saved_validators():
    cfg = yaml.safe_load(CONFIG.read_text())
    result = authenticate(cfg, deep=True)
    from validate_state_shift_transport_scale01 import method_folder
    from reconciliation.se2 import local_trajectory_to_world
    for spec in result['summary']['selected_sources']:
        folder = Path(spec['folder'])
        common = read(folder/'common_state.json')
        before = copy.deepcopy(common)
        refs = read(folder/'references.json')
        for method in METHODS:
            r = refs[method]
            for frame in ['world', 'local']:
                assert sha(r[frame+'_path']) == r[frame+'_sha256']
            world, local = np.load(r['world_path']), np.load(r['local_path'])
            np.testing.assert_allclose(local_trajectory_to_world(common['fresh_capture_pose'], local), world, atol=1e-12, rtol=0)
            assert not np.allclose(local_trajectory_to_world(common['B'], local), world, atol=1e-3, rtol=0)
            restoration = read(method_folder(folder, method)/'restoration.json')
            assert restoration['B'] == common['B'] and restoration['capture_pose'] == common['fresh_capture_pose']
            np.testing.assert_allclose(world, restoration['installed_world'], atol=1e-12, rtol=0)
        assert common == before


def test_classification_predeclared_thresholds():
    c, refs, states = synthetic_matched()
    selector, _ = official_selector(OFFICIAL)
    reset = reset_summary(matched_queries(c, refs, states, selector))
    q = dict(progress_reset=reset, correspondences=c.correspondences([.4, .1, 0]),
        saved_execution_context={n:dict(position_auc_09_m_s=.1+i*.1, sustained_attachment_s=None) for i, n in enumerate(METHODS)})
    sources = {sid:copy.deepcopy(q) for sid in SOURCE_IDS}
    assert classification(sources)['classification'] == 'TRANSPORT_PROGRESS_RESET_SUPPORTED'
    for source in sources.values():
        for r in source['progress_reset']['methods'].values():
            r['source_reset'] = False
            r['persistent_two_ticks'] = False
    assert classification(sources)['classification'] == 'CORRESPONDENCE_WITHOUT_RESET'
    sources[SOURCE_IDS[0]]['correspondences'][RULES[2]]['arc_m'] = 0
    assert classification(sources)['classification'] == 'MIXED_CORRESPONDENCE_EFFECT'


def test_three_final_PNGs_from_synthetic_fixture(tmp_path):
    from report_spatial_correspondence_selector_diag01 import compact_figures
    from shapely.geometry import box
    curve, refs, states = synthetic_matched()
    selector, _ = official_selector(OFFICIAL)
    rows = matched_queries(curve, refs, states, selector)
    fresh_path, old_path = tmp_path/'fresh.npy', tmp_path/'old.npy'
    np.save(fresh_path, curve.fresh)
    np.save(old_path, [[0, -.2, 0], [0, 0, 0]])
    specs, sources = [], {}
    for sid in SOURCE_IDS:
        q = dict(B=[.31, 0, 0], correspondences=curve.correspondences([.31, 0, 0]),
                 matched_selector=rows, progress_reset=reset_summary(rows),
                 saved_execution_context={n:dict(sustained_attachment_s=None) for n in METHODS}, first_official_selection={})
        for n, row in zip(METHODS, rows[:3]):
            q['first_official_selection'][n] = dict(query_tick=row['tick'], H5_start=dict(
                physical_target_world=row['physical_H5_world'][0], original_row_identity=row['first_row']))
        specs.append(dict(source_id=sid, original_world=str(fresh_path), old_path=str(old_path)))
        sources[sid] = q
    environments = [dict(base=SimpleNamespace(obstacles=box(4, 4, 5, 5)), cart=None)]*4
    output = tmp_path/'figures'
    manifest = compact_figures(dict(sources=sources), dict(sources=specs), output, environments)
    assert [m['file'] for m in manifest] == PNGS
    assert sorted(p.name for p in output.iterdir()) == sorted(PNGS)
    from PIL import Image
    for name in PNGS:
        with Image.open(output/name) as image:
            assert image.format == 'PNG' and min(image.size) >= 1500


def test_validator_does_not_invoke_official_selector_or_solver():
    import validate_spatial_correspondence_selector_diag01 as validator
    tree = ast.parse(inspect.getsource(validator))
    calls = {node.func.id for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert not calls & {'official_selector', 'solve_least_squares', 'load_official', 'integrate_unicycle', 'execute'}
