"""Frozen validation policy and synthetic derivative checks; no event solves."""
import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("jax")

from reconciliation import gp_se2_diag04_validation as validation
from reconciliation.gp_se2_diag02_derivatives import DerivativeError, DerivativeProvider
from reconciliation.gp_se2_diag02_validation import (
    ConstantEnvironmentDerivatives, PROTOCOL as ORIGINAL_PROTOCOL,
)
from reconciliation.gp_se2_diag04_constraints import ConstraintView, RefinedDerivativeProvider
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem


def test_fixed_ten_start_order_and_five_paired_seed_roles():
    rows = validation.schedule()
    assert len(rows) == 10 and len({r["solve_id"] for r in rows}) == 10
    assert [r["solve_index"] for r in rows] == list(range(10))
    expected_pairs = [
        (validation.HARD, "M2_GP_NO_OBSTACLE", "I0_FRESH"),
        (validation.HARD, "M2_GP_NO_OBSTACLE", "I1_DECEL"),
        (validation.HARD, "M3_GP_CONSTRAINED", "I0_FRESH"),
        (validation.HARD, "M3_GP_CONSTRAINED", "I1_DECEL"),
        (validation.BENIGN, "M3_GP_CONSTRAINED", "I1_DECEL"),
    ]
    for pair, expected in enumerate(expected_pairs):
        a, b = rows[2*pair:2*pair+2]
        assert (a["case_id"], a["method"], a["initialization"]) == expected
        assert (b["case_id"], b["method"], b["initialization"]) == expected
        assert a["pair_index"] == b["pair_index"] == pair
        assert (a["grid"], b["grid"]) == validation.GRIDS
    rows[0]["grid"] = "changed output"
    assert validation.schedule()[0]["grid"] == "G0_ORIGINAL"


def test_protocol_preserves_original_derivative_thresholds_and_budget():
    p = validation.PROTOCOL
    assert p["point_count"] == 3*len(validation.PAIR_SPECS) == 15
    assert p["planned_GP_solves"] == 2*len(validation.PAIR_SPECS) == 10
    assert p["quarter_fractions"] == [.25, .75]
    assert p["extra_equalities"] == 60 and p["extra_inequalities"] == 480
    assert (p["variables"], p["support_states"], p["horizon_s"], p["support_dt_s"]) == (150, 31, 3., .1)
    assert (p["max_iterations"], p["prepared_solve_budget_s"], p["ftol"]) == (200, 30., 1e-7)
    for key in ("directional_fd_steps", "primal_tolerance", "objective_derivative_tolerance", "constraint_derivative_tolerance"):
        assert p[key] == ORIGINAL_PROTOCOL[key]
    assert p["directional_fd_steps"] == [2e-4, 2e-5, 2e-6]
    assert p["required_directional_steps"] == "both two finest; all errors preserved"
    assert not p["perturbation_used_as_seed"] and not p["solver_scaling_changed"]
    assert not p["physical_acceptance_changed"] and not p["rank_implies_infeasibility"]
    assert all(p[key] == 0 for key in ("new_VLA_inference", "new_MPC_solve", "new_rollout", "new_GUI_runtime"))


def test_frozen_directions_and_perturbation_are_reproducible_and_not_aliases():
    a, p = validation.directions_and_perturbation()
    b, q = validation.directions_and_perturbation()
    assert a.shape == (3, 150) and p.shape == (150,)
    np.testing.assert_array_equal(a, b); np.testing.assert_array_equal(p, q)
    np.testing.assert_allclose(np.linalg.norm(a, axis=1), 1., atol=1e-15)
    expected = np.random.default_rng(20260919).normal(size=(30, 5))*1e-4
    np.testing.assert_array_equal(p, expected.ravel())
    p[:] = 0.; a[:] = 0.
    c, r = validation.directions_and_perturbation()
    np.testing.assert_array_equal(c, b); np.testing.assert_array_equal(r, q)


def test_vector_identity_is_exact_float64_bytes_and_invalid_is_not_zero_filled():
    a = np.arange(150, dtype=np.float64)
    assert validation.vector_hash(a) == validation.vector_hash(a.tolist())
    changed = a.copy(); changed[13] = np.nextafter(changed[13], np.inf)
    assert validation.vector_hash(a) != validation.vector_hash(changed)
    for invalid in (np.zeros(149), np.full(150, np.nan), np.full(150, np.inf)):
        with pytest.raises(ValueError, match="finite 150-variable"):
            validation.vector_hash(invalid)


@pytest.fixture(scope="module")
def synthetic():
    problem, fixture = make_fixture_problem("S1")
    directions, perturbation = validation.directions_and_perturbation()
    vector = fixture["known_vector"]+perturbation
    geometry = ConstantEnvironmentDerivatives()
    provider = DerivativeProvider(problem, geometry)
    g0, g1 = [ConstraintView(problem, grid) for grid in validation.GRIDS]
    p0, p1 = [RefinedDerivativeProvider(view, provider) for view in (g0, g1)]
    return vector, problem, geometry, provider, g0, g1, p0, p1, directions


def test_actual_verifier_on_synthetic_nonzero_chart_without_any_optimizer(synthetic, monkeypatch):
    import scipy.optimize
    from reconciliation import gp_se2_formulation
    def forbidden(*args, **kwargs):
        pytest.fail("derivative validation invoked an optimizer")
    monkeypatch.setattr(scipy.optimize, "minimize", forbidden)
    monkeypatch.setattr(gp_se2_formulation, "minimize", forbidden)
    before = synthetic[0].copy()
    report, arrays = validation.verify_point("SYNTHETIC_NONZERO_CHART", *synthetic)
    assert report["valid"] and report["status"] == "VERIFIED_WITH_DECLARED_BRANCH_LIMITATIONS"
    assert all(report["parity"].values()) and report["quarter_shapes_valid"]
    assert report["base_verification"]["valid"] and report["no_numerical_fallback"]
    assert not report["optimizer_executed"]
    assert len(report["quarter_primal"]) == 9
    assert [r["step"] for r in report["quarter_directional"]] == validation.PROTOCOL["directional_fd_steps"]
    assert all(r["passed"] for r in report["quarter_directional"][-2:])
    assert arrays["quarter_equality_jacobian"].shape == (60, 150)
    assert arrays["quarter_inequality_jacobian"].shape == (480, 150)
    assert arrays["quarter_2_equality_central_FD"].shape == (60, 3)
    np.testing.assert_array_equal(synthetic[0], before)


def test_wrong_added_acceleration_jacobian_fails_without_threshold_relaxation(synthetic, monkeypatch):
    p1 = synthetic[7]
    original = p1.inequality_jacobian
    frozen = copy.deepcopy(validation.PROTOCOL)
    def wrong(vector):
        value = original(vector)
        value[903+240, 3] += 1.  # First appended a_x upper row, not a base row.
        return value
    monkeypatch.setattr(p1, "inequality_jacobian", wrong)
    report, _ = validation.verify_point("SYNTHETIC_BAD_ACCELERATION_DERIVATIVE", *synthetic)
    assert not report["valid"] and report["status"] == "DERIVATIVE_VALIDATION_FAILED"
    assert all(report["parity"].values())  # Base parity alone cannot hide extra-row error.
    assert all(not r["passed"] for r in report["quarter_directional"][-2:])
    assert any(not r["passed"] and r["family"] == "quarter_linear_acceleration_upper"
               for step in report["quarter_directional"][-2:] for r in step["reports"])
    assert validation.PROTOCOL == frozen


def test_nonfinite_added_primal_is_failed_evidence_not_safe_value(synthetic, monkeypatch):
    p1 = synthetic[7]
    original = p1.values
    def wrong(vector):
        value = original(vector)
        value["equality"][30] = np.nan
        return value
    monkeypatch.setattr(p1, "values", wrong)
    report, _ = validation.verify_point("SYNTHETIC_BAD_AD_PRIMAL", *synthetic)
    assert not report["valid"] and not report["quarter_primal"][0]["finite"]
    assert report["quarter_primal"][0]["maximum_absolute_error"] is None


def test_unsupported_relative_cut_is_explicit_and_not_perturbed_to_pass(synthetic):
    values = list(synthetic)
    bad = values[0].copy(); bad[12] = np.pi; values[0] = bad
    before = bad.copy()
    with pytest.raises(DerivativeError) as caught:
        validation.verify_point("SYNTHETIC_UNSUPPORTED_CUT", *values)
    assert caught.value.reason_code == "UNSUPPORTED_WRAP_CUT"
    np.testing.assert_array_equal(bad, before)


def load_runner(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/"scripts"))
    spec = importlib.util.spec_from_file_location("diag04_runner_validation_test", root/"scripts/run_gp_se2_diag04.py")
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    return runner


def test_runner_refuses_overwrite_and_outside_root_before_reading_sources(tmp_path, monkeypatch):
    runner = load_runner(monkeypatch)
    def forbidden(*args, **kwargs):
        pytest.fail("invalid output must be rejected before source reads or numerical work")
    monkeypatch.setattr(runner, "read", forbidden)
    monkeypatch.setattr(runner, "run_refined", forbidden)
    monkeypatch.setattr(runner, "OUTPUT", tmp_path/"allowed")
    existing = tmp_path/"allowed"/"existing"; existing.mkdir(parents=True)
    with pytest.raises(FileExistsError, match="exclusive"):
        runner.prepare(existing)
    with pytest.raises(FileExistsError, match="exclusive"):
        runner.prepare(tmp_path/"outside")
    assert not (tmp_path/"outside").exists()


@pytest.mark.parametrize("changed", ["protocol.json", "experiment_manifest.json", "config_snapshot.yaml",
                                     "derivative_checks/point_manifest.json", "initializations/pair_0.npy"])
def test_frozen_new_input_mutation_blocks_execution(tmp_path, monkeypatch, changed):
    runner = load_runner(monkeypatch)
    paths = ["protocol.json", "experiment_manifest.json", "config_snapshot.yaml",
             "derivative_checks/point_manifest.json", "initializations/pair_0.npy"]
    for path in paths:
        target = tmp_path/path; target.parent.mkdir(exist_ok=True, parents=True)
        target.write_bytes(b"synthetic frozen fixture, not actual-event data")
    source = dict(preserved_hashes={}, user_config_hashes={}, experiment_code_sha256={},
                  frozen_input_sha256={path: runner.digest(tmp_path/path) for path in paths})
    (tmp_path/"source.json").write_text(json.dumps(source))
    assert runner.verify_frozen(tmp_path) == source
    (tmp_path/changed).write_bytes(b"changed after freeze")
    with pytest.raises(ValueError, match="SOURCE_OR_IMPLEMENTATION_MISMATCH") as caught:
        runner.verify_frozen(tmp_path)
    assert changed in str(caught.value)


def test_source_core_and_unrelated_user_hashes_are_checked_without_modification(tmp_path, monkeypatch):
    runner = load_runner(monkeypatch)
    code = tmp_path/"code.py"; historical = tmp_path/"historical.json"; user = tmp_path/"user.yaml"
    for path in (code, historical, user):
        path.write_bytes(b"synthetic original")
    monkeypatch.setattr(runner, "ROOT", tmp_path)
    source = dict(preserved_hashes={str(historical): runner.digest(historical)},
                  user_config_hashes={str(user): runner.digest(user)},
                  experiment_code_sha256={"code.py": runner.digest(code)}, frozen_input_sha256={})
    (tmp_path/"source.json").write_text(json.dumps(source))
    for path in (historical, user, code):
        assert runner.verify_frozen(tmp_path) == source
        path.write_bytes(b"edited")
        with pytest.raises(ValueError, match="SOURCE_OR_IMPLEMENTATION_MISMATCH"):
            runner.verify_frozen(tmp_path)
        assert path.read_bytes() == b"edited"  # Guard must not repair or erase an unrelated edit.
        path.write_bytes(b"synthetic original")
    historical.unlink()
    assert runner.check_hashes(source["preserved_hashes"]) == [str(historical)]


def test_failed_derivatives_block_solve_and_started_phases_cannot_retry(tmp_path, monkeypatch):
    runner = load_runner(monkeypatch)
    monkeypatch.setattr(runner, "verify_frozen", lambda run: {})
    def forbidden(*args, **kwargs):
        pytest.fail("failed or already-started phase invoked numerical work")
    monkeypatch.setattr(runner, "run_refined", forbidden)
    monkeypatch.setattr(runner.HospitalEnvironment, "load", forbidden)
    folder = tmp_path/"derivative_checks"; folder.mkdir()
    gate = folder/"validation.json"; gate.write_text(json.dumps(dict(valid=False)))
    with pytest.raises(ValueError, match="verification not passed"):
        runner.solve(tmp_path)
    assert not (tmp_path/"optimization_started.json").exists()
    gate.write_text(json.dumps(dict(valid=True)))
    (tmp_path/"optimization_started.json").write_text("{}")
    with pytest.raises(FileExistsError, match="already started"):
        runner.solve(tmp_path)
    (folder/"started.json").write_text("{}")
    with pytest.raises(FileExistsError, match="already started"):
        runner.derivatives(tmp_path)
