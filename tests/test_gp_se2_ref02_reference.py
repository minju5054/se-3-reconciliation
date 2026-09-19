"""Synthetic selector correctness fixtures only; no rollout or recovery claims."""
import ast
from copy import deepcopy
import math
from pathlib import Path

import numpy as np
import pytest

import reconciliation.gp_se2_ref02_reference as module
from reconciliation.gp_se2_ref02_reference import (
    METHODS, audit_source_progress, enrich_selector_result, select_source_progress, validate_lineage,
)
from reconciliation.online_mpc_adapter import PINNED_MPC_SHA256, sha256

W = (10., 10., 1.)
PINNED_SOURCE = Path.home()/"Workspace/external/LightNav-0-official-demo/mujoco_demo/vln_mujoco/mpc.py"


@pytest.fixture(scope="module")
def official_selector():
    """Execute only the actual pinned pure function AST; import no MPC/runtime."""
    if not PINNED_SOURCE.is_file():
        pytest.skip("pinned external MPC source unavailable for read-only selector parity")
    assert sha256(PINNED_SOURCE) == PINNED_MPC_SHA256
    parsed = ast.parse(PINNED_SOURCE.read_text())
    functions = [node for node in parsed.body if isinstance(node, ast.FunctionDef)
                 and node.name in ("wrap_angle", "build_pose_aligned_reference")]
    assert len(functions) == 2
    namespace = {"np": np, "math": math}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(PINNED_SOURCE), "exec"), namespace)
    return namespace["build_pose_aligned_reference"]


def lineage(path, progress):
    rows = []
    for i, (point, s) in enumerate(zip(path, progress)):
        left, right = math.floor(s), math.ceil(s)
        alpha = 0. if left == right else s-left
        rows.append({"derived_row_index": i, "original_left_row_index": left,
                     "original_right_row_index": right, "interpolation_alpha": alpha,
                     "original_fractional_row_coordinate": float(s), "world_xy": point[:2].tolist(),
                     "stored_yaw": float(point[2]), "wrapped_yaw": math.atan2(math.sin(point[2]), math.cos(point[2])),
                     "unwrapped_yaw": float(point[2])})
    return rows


@pytest.mark.parametrize("length", [1, 2, 7, 23])
@pytest.mark.parametrize("horizon", [1, 5, 9])
def test_identity_progress_equals_actual_official_selector_for_arbitrary_length(official_selector, length, horizon):
    rng = np.random.default_rng(435)
    path = rng.normal(size=(length, 3))
    path[:, 2] *= 4.
    saved = path.copy()
    progress = np.arange(length, dtype=np.float64)
    for state in np.vstack((path, rng.normal(size=(5, 3)))):
        expected = official_selector(path, state, horizon=horizon, weights=W)
        actual, details = select_source_progress(path, state, progress, horizon=horizon, weights=W)
        np.testing.assert_array_equal(actual, expected)
        audited = audit_source_progress(path, state, progress, actual, horizon=horizon, weights=W,
                                        actual_indices=details["indices"])
        assert details["indices"] == audited["indices"]
        assert not np.shares_memory(path, actual)
    np.testing.assert_array_equal(path, saved)


def test_nonuniform_lower_bound_uses_same_fractional_base_without_overshoot_accumulation():
    progress = np.array([.2, .7, 1.4, 2.25, 3.45, 4.7])
    path = np.column_stack((np.arange(6), np.zeros((6, 2))))
    actual, details = select_source_progress(path, path[0], progress, horizon=5, weights=W)
    assert details["indices"] == [2, 3, 4, 5, 5]
    assert details["q_h"] == [1.2, 2.2, 3.2, 4.2, 4.7]
    np.testing.assert_allclose(details["progress_overshoot"], [.2, .05, .25, .5, 0.], atol=1e-15)
    np.testing.assert_array_equal(actual, path[[2, 3, 4, 5, 5]])
    assert details["endpoint_clamped"] == [False, False, False, False, True]
    assert details["endpoint_repeated_per_target"] == [False, False, False, False, True]


def test_strict_float64_lower_bound_has_no_search_tolerance():
    s = np.array([0., np.nextafter(1., 0.), 1., np.nextafter(1., np.inf), 2.1])
    path = np.column_stack((np.arange(5), np.zeros((5, 2))))
    _, details = select_source_progress(path, path[0], s, horizon=2, weights=W)
    assert details["indices"] == [2, 4]
    assert details["q_h"] == [1., 2.]
    assert details["progress_overshoot"][0] == 0.


def test_fractional_nearest_is_not_rounded_or_searched_on_another_path():
    path = np.array([[0., 0., 0.], [2., 0., 0.], [3., 0., 0.], [4., 0., 0.]])
    _, details = select_source_progress(path, path[1], [1.2, 1.8, 2.7, 2.9], horizon=1, weights=W)
    assert details["nearest_index"] == 1
    assert details["q_h"] == [2.8]
    assert details["indices"] == [3]


def test_endpoint_exact_target_distinguished_from_clamp_and_repeat():
    path = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
    _, details = select_source_progress(path, path[1], [0., 1., 2.], horizon=3, weights=W)
    assert details["indices"] == [2, 2, 2]
    assert details["q_h"] == [2., 2., 2.]
    assert details["endpoint_clamped"] == [False, True, True]
    assert details["endpoint_selected"] == [True, True, True]
    assert details["endpoint_repeated_per_target"] == [False, True, True]


def test_duplicate_xy_rotation_only_and_source_identity_are_preserved():
    path = np.array([[0., 0., 0.], [0., 0., .1], [0., 0., .1], [0., 0., .8], [0., 0., 1.]])
    s = [0., .5, 1., 1.5, 2.]
    rows = lineage(path, s)
    validated = validate_lineage(path, rows)
    actual, details = select_source_progress(path, path[0], validated, horizon=2, weights=W)
    assert details["indices"] == [2, 4]
    np.testing.assert_array_equal(actual[:, :2], path[[2, 4], :2])
    np.testing.assert_allclose(actual[:, 2], path[[2, 4], 2], atol=1e-15, rtol=0.)
    assert len(rows) == 5


@pytest.mark.parametrize("yaw", [math.pi-1e-8, -math.pi+1e-8, math.pi, -math.pi, 0.])
def test_sequential_yaw_unwrap_matches_official_at_pi_cut(official_selector, yaw):
    path = np.array([[0., 0., yaw], [1., 0., -math.pi+.01], [2., 0., math.pi-.02], [3., 0., -2.8]])
    expected = official_selector(path, path[0], horizon=5, weights=W)
    actual, details = select_source_progress(path, path[0], np.arange(4.), horizon=5, weights=W)
    np.testing.assert_array_equal(actual, expected)
    audit_source_progress(path, path[0], np.arange(4.), actual, horizon=5, weights=W)
    with pytest.raises(ValueError, match="literal sequential"):
        audit_source_progress(path, path[0], np.arange(4.), actual+[0., 0., 2*math.pi], horizon=5, weights=W)


def test_nearest_tie_and_near_tie_are_official_first_argmin(official_selector):
    path = np.array([[0., 0., -.1], [0., 0., .1], [1., 0., .2]])
    pose = np.zeros(3)
    s = np.arange(3.)
    actual, details = select_source_progress(path, pose, s, horizon=2, weights=W)
    full = enrich_selector_result(path, pose, actual, lineage(path, s), method=METHODS[2],
                                  weights=W, horizon=2, original_goal_row_index=2, selection_details=details)
    assert details["nearest_index"] == 0
    assert full["exact_tied_nearest_indices"] == [0, 1]
    assert full["nearest_cost_margin"] == 0.
    path[0, 2] -= 1e-10
    actual, details = select_source_progress(path, pose, s, horizon=2, weights=W)
    full = enrich_selector_result(path, pose, actual, lineage(path, s), method=METHODS[2],
                                  weights=W, horizon=2, original_goal_row_index=2, selection_details=details)
    assert details["nearest_index"] == 1
    assert full["near_tied_nearest_indices"] == [0, 1]
    np.testing.assert_array_equal(actual, official_selector(path, pose, horizon=2, weights=W))


def test_constant_reference_needs_metadata_and_repeats_final_installed_pose():
    path = np.tile([1., 2., .3], (30, 1))
    rows = lineage(path, [7.]*30)
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_lineage(path, rows)
    metadata = {"constant_reference": True, "retained_source_row_count": 1, "original_source_row_index": 7}
    s = validate_lineage(path, rows, constant_reference_metadata=metadata)
    actual, details = select_source_progress(path, [0., 0., 0.], s, horizon=5, weights=W, constant_reference=True)
    assert details["indices"] == [29]*5 and details["nearest_index"] == 0
    assert details["q_h"] == [7.]*5
    assert details["progress_overshoot"] == [0.]*5
    full = enrich_selector_result(path, [0., 0., 0.], actual, rows, method=METHODS[2], weights=W,
                                  horizon=5, original_goal_row_index=7, selection_details=details,
                                  constant_reference_metadata=metadata)
    assert full["final_goal_row_in_horizon"]
    for key, bad in [("constant_reference", False), ("retained_source_row_count", 2), ("original_source_row_index", 6)]:
        with pytest.raises(ValueError, match="metadata"):
            validate_lineage(path, rows, constant_reference_metadata={**metadata, key: bad})
    changed = path.copy(); changed[2, 2] += .01
    with pytest.raises(ValueError, match="identical pose"):
        validate_lineage(changed, lineage(changed, [7.]*30), constant_reference_metadata=metadata)


@pytest.mark.parametrize("mutation", ["order", "left", "alpha", "progress", "xy", "yaw", "unwrapped", "nan"])
def test_invalid_lineage_is_explicit_error(mutation):
    path = np.array([[0., 0., 0.], [1., 0., .1], [2., 0., .2]])
    rows = lineage(path, [1., 1.5, 2.])
    if mutation == "order": rows[1]["derived_row_index"] = 2
    if mutation == "left": rows[1]["original_left_row_index"] = 0
    if mutation == "alpha": rows[1]["interpolation_alpha"] = 1.1
    if mutation == "progress": rows[1]["original_fractional_row_coordinate"] = 1.6
    if mutation == "xy": rows[1]["world_xy"][0] += .01
    if mutation == "yaw": rows[1]["wrapped_yaw"] += .01
    if mutation == "unwrapped": rows[1]["unwrapped_yaw"] += .01
    if mutation == "nan": rows[1]["original_fractional_row_coordinate"] = float("nan")
    with pytest.raises(ValueError, match="lineage"):
        validate_lineage(path, rows)


@pytest.mark.parametrize("progress", [[0., 0., 1.], [0., 2., 1.], [0., math.nan, 1.], [-1., 0., 1.], [0., 1.]])
def test_invalid_progress_never_silently_sorts_projects_or_falls_back(progress):
    path = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]])
    with pytest.raises(ValueError, match="progress"):
        select_source_progress(path, path[0], progress, horizon=5, weights=W)


def test_same_state_dense_nearest_preserved_but_targets_use_existing_rows_only(official_selector):
    path = np.column_stack((np.arange(8.), np.zeros(8), np.arange(8.)*.1))
    original = path.copy(); progress = np.linspace(1., 5., len(path)); rows = lineage(path, progress)
    pose = np.array([1.1, .01, .12])
    baseline = official_selector(path, pose, horizon=5, weights=W)
    selected, details = select_source_progress(path, pose, progress, horizon=5, weights=W)
    b = enrich_selector_result(path, pose, baseline, rows, method=METHODS[1], weights=W,
                               horizon=5, original_goal_row_index=5)
    c = enrich_selector_result(path, pose, selected, rows, method=METHODS[2], weights=W,
                               horizon=5, original_goal_row_index=5, selection_details=details)
    for key in ("nearest_index", "nearest_original_fractional_row_coordinate", "nearest_cost", "weighted_total_pose_distances"):
        assert b[key] == c[key]
    assert b["indices"] != c["indices"]
    assert b["q_h"] is None and b["progress_overshoot"] is None
    np.testing.assert_array_equal(selected[:, :2], path[c["indices"], :2])
    np.testing.assert_array_equal(path, original)
    assert not np.shares_memory(selected, path)
    assert c["poses_interpolated_by_selector"] is False
    assert c["official_reference_unwrapped_yaw"] == selected[:, 2].tolist()


def test_independent_audit_scans_without_invoking_selector_or_searchsorted(monkeypatch):
    path = np.array([[0., 0., 0.], [1., 0., .1], [2., 0., .2]])
    s = [0., .6, 2.]
    actual, details = select_source_progress(path, path[0], s, horizon=5, weights=W)
    def prohibited(*args, **kwargs):
        raise AssertionError("independent audit invoked primary search")
    monkeypatch.setattr(module, "select_source_progress", prohibited)
    monkeypatch.setattr(np, "searchsorted", prohibited)
    audit = audit_source_progress(path, path[0], s, actual, horizon=5, weights=W,
                                  actual_indices=details["indices"])
    assert audit["indices"] == [2]*5


def test_independent_audit_rejects_fabricated_reference_and_wrong_recorded_indices():
    path = np.array([[0., 0., 0.], [1., 0., .1], [2., 0., .2]])
    s = [0., .6, 2.]
    actual, details = select_source_progress(path, path[0], s, horizon=3, weights=W)
    with pytest.raises(ValueError, match="indices"):
        audit_source_progress(path, path[0], s, actual, horizon=3, weights=W, actual_indices=[1, 2, 2])
    with pytest.raises(ValueError, match="XY"):
        audit_source_progress(path, path[0], s, actual+[.01, 0., 0.], horizon=3, weights=W)
    with pytest.raises(ValueError, match="yaw"):
        audit_source_progress(path, path[0], s, actual+[0., 0., .01], horizon=3, weights=W)
    rows = lineage(path, s)
    bad = deepcopy(details); bad["q_h"][0] += .1
    with pytest.raises(ValueError, match="q_h"):
        enrich_selector_result(path, path[0], actual, rows, method=METHODS[2], weights=W,
                               horizon=3, original_goal_row_index=2, selection_details=bad)


@pytest.mark.parametrize("horizon,weights", [(0, W), (1.5, W), (True, W), (5, [10., 10., 0.]), (5, [10., math.nan, 1.])])
def test_invalid_configuration_rejected(horizon, weights):
    with pytest.raises(ValueError):
        select_source_progress([[0., 0., 0.]], [0., 0., 0.], [0.], horizon=horizon, weights=weights)


def test_baseline_audit_is_not_disabled_or_forced_to_accept_progress_selection(official_selector):
    path = np.column_stack((np.arange(8.), np.zeros((8, 2))))
    s = np.linspace(0., 2., 8); rows = lineage(path, s)
    actual, _ = select_source_progress(path, path[0], s, horizon=5, weights=W)
    with pytest.raises(ValueError, match="XY"):
        enrich_selector_result(path, path[0], actual, rows, method=METHODS[1], weights=W,
                               horizon=5, original_goal_row_index=2)
    baseline = official_selector(path, path[0], horizon=5, weights=W)
    with pytest.raises(ValueError, match="XY"):
        enrich_selector_result(path, path[0], baseline, rows, method=METHODS[2], weights=W,
                               horizon=5, original_goal_row_index=2)
