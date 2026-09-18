#!/usr/bin/env python3
"""Reproduce DIAG-02 analytic environment derivative qualification.

Read-only numerical diagnostics of the original ten-case environment, never a
solve, inference, rollout, or source-data transformation. The fixed protocol
matches the original environment_derivatives.json: both original GP seeds,
90 collocation XY points per seed, RNG1902, central step1e-6m, absolute error
threshold1e-7, exact original value agreement, and four synthetic one-sided
checks. Existing reports are never overwritten.

Timings and source hashes necessarily reflect the new diagnostic invocation.
This command does not rerun or claim a pytest result.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
from shapely.geometry import LineString, box

from reconciliation.gp_se2 import interpolate_interval
from reconciliation.gp_se2_diagnostics import load_frozen_case
from reconciliation.gp_se2_diag02_environment import EnvironmentDerivatives
from reconciliation.gp_se2_environment import HospitalEnvironment


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def qualify(primary):
    """Return the fixed qualification report without writing or optimizing."""
    primary = Path(primary).resolve()
    rows = json.loads((primary / "case_manifest.json").read_text())["selected"]
    if len(rows) != 10:
        raise ValueError("qualification requires the original ten frozen cases")
    environment = derivatives = None
    random = np.random.default_rng(1902)
    report = dict(
        valid=True,
        scope="analytic environment derivative qualification only; no solve or execution",
        actual_source=str(primary), random_seed=1902, central_step_m=1e-6,
        derivative_absolute_error_threshold=1e-7,
        original_values_requirement="exact equality",
        families={family: dict(point_count=0, value_max_abs_error=0.,
            branch_preserving_count=0, finite_step_branch_crossing_count=0,
            max_branch_preserving_directional_abs_error=0., batch_query_seconds=0.)
            for family in ("workspace", "obstacle")},
        per_case=[])
    for row in rows:
        loaded = load_frozen_case(primary, row["case_id"], environment=environment)
        environment, problem = loaded["environment"], loaded["problem"]
        if derivatives is None:
            derivatives = EnvironmentDerivatives(environment, loaded["config"]["footprint"]["radius_m"])
        initializations = problem.initializations()
        if len(initializations) != 2:
            raise ValueError("qualification requires both original initializations")
        for initial in initializations:
            poses, twists = initial["poses"], initial["twists"]
            collocation, _, _ = interpolate_interval(
                poses[:-1, None], twists[:-1, None], poses[1:, None], twists[1:, None],
                problem.config["support_dt_s"], np.array([0., .5, 1.]))
            xy = collocation.reshape(-1, 3)[:, :2]
            if len(xy) != 90:
                raise ValueError("qualification requires 90 original collocation XY points per seed")
            direction = random.normal(size=xy.shape)
            direction /= np.linalg.norm(direction, axis=1)[:, None]
            for family, query in (("workspace", environment.workspace_margin),
                                  ("obstacle", environment.optimizer_clearance)):
                started = time.perf_counter()
                values, gradient, metadata = derivatives.evaluate(xy, family)
                elapsed = time.perf_counter() - started
                plus, _, plus_metadata = derivatives.evaluate(xy + 1e-6 * direction, family)
                minus, _, minus_metadata = derivatives.evaluate(xy - 1e-6 * direction, family)

                def signature(entry):
                    return entry["selected_segment_id"] if family == "workspace" else entry["cell_xy"]

                stable = np.array([
                    center["smooth"] and signature(center) == signature(a) == signature(b)
                    for center, a, b in zip(metadata["points"], plus_metadata["points"], minus_metadata["points"])])
                original = np.asarray(query(xy))
                original = np.where(np.isfinite(original), original, -1e6)
                value_error = float(np.max(np.abs(values - original)))
                errors = np.abs((plus - minus) / (2e-6) - np.sum(gradient * direction, axis=1))
                error = float(np.max(errors[stable], initial=0.))
                totals = report["families"][family]
                totals["point_count"] += len(xy)
                totals["value_max_abs_error"] = max(totals["value_max_abs_error"], value_error)
                totals["branch_preserving_count"] += int(stable.sum())
                totals["finite_step_branch_crossing_count"] += int((~stable).sum())
                totals["max_branch_preserving_directional_abs_error"] = max(
                    totals["max_branch_preserving_directional_abs_error"], error)
                totals["batch_query_seconds"] += elapsed
                report["per_case"].append(dict(
                    case_id=row["case_id"], initialization=initial["name"], family=family,
                    value_max_abs_error=value_error, branch_preserving_points=int(stable.sum()),
                    branch_crossing_points=int((~stable).sum()),
                    max_branch_preserving_directional_abs_error=error))
                report["valid"] &= value_error == 0. and error < 1e-7

    # Synthetic branch checks are tests of local formulas, not event evidence.
    grid = dict(origin=np.array([0., 0.]), resolution=np.array(1.),
                distances=np.array([[0., 1., 3.], [0., 1., 4.], [0., 1., 5.]]))
    synthetic_environment = HospitalEnvironment(
        LineString([(0., 0.), (0., 2.)]), box(-4., -4., 4., 4.), grid=grid)
    derivative = EnvironmentDerivatives(synthetic_environment)
    synthetic = []
    for name, family, point in (
        ("medial_tie", "workspace", np.array([0., 0.])),
        ("polygon_corner", "workspace", np.array([4., 4.])),
        ("internal_grid_boundary", "obstacle", np.array([1., .5])),
        ("grid_domain_boundary", "obstacle", np.array([0., .5])),
    ):
        value, gradient, metadata = derivative.evaluate(point, family)
        direction = -gradient if family == "workspace" else np.array([1., 0.])
        query = (synthetic_environment.workspace_margin if family == "workspace"
                 else synthetic_environment.optimizer_clearance)
        numeric = float((query(point + 1e-7 * direction) - value) / 1e-7)
        analytic = float(gradient @ direction)
        error = abs(numeric - analytic)
        synthetic.append(dict(
            name=name, synthetic_only=True, point=point.tolist(), direction=direction.tolist(),
            selected_gradient=gradient.tolist(), metadata=metadata["points"][0],
            one_sided_numeric=numeric, analytic=analytic, abs_error=error, valid=error < 1e-7))
        report["valid"] &= error < 1e-7
    report["synthetic_branch_checks"] = synthetic
    sources = [
        "src/reconciliation/gp_se2_diag02_environment.py", "tests/test_gp_se2_diag02_environment.py",
        "src/reconciliation/gp_se2_environment.py", "src/reconciliation/gp_se2_formulation.py",
        "src/reconciliation/gp_se2_diagnostics.py", "scripts/check_gp_se2_diag02_environment.py",
    ]
    report["source_sha256"] = {path: digest(ROOT / path) for path in sources}
    report["input_sha256"] = {
        str(primary / "case_manifest.json"): digest(primary / "case_manifest.json"),
        str(primary / "config_snapshot.yaml"): digest(primary / "config_snapshot.yaml"),
        str(primary / "environment/validation.json"): digest(primary / "environment/validation.json"),
    }
    report["test_verification"] = dict(
        executed_by_this_command=False,
        separate_command="PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider tests/test_gp_se2_diag02_environment.py tests/test_gp_se2_environment.py",
        note="This report qualifies the numeric derivative probes above; pytest is a separate check.")
    report["limitations"] = [
        "Classical derivatives hold within the selected smooth branch; exact ties/vertices use the recorded active limiting gradient.",
        "Bilinear grid-domain or invalid-cell penalty can be discontinuous; derivative there describes only the selected branch.",
        "Finite-step probes crossing feature/cell boundaries are reported separately, never treated as central derivative failures.",
        "No finite-difference fallback; unsupported missing grid/invalid workspace raises.",
        "Actual checks validate derivatives, not trajectory feasibility, optimizer improvement or physical safety.",
    ]
    return report


def run_check(run, output=None):
    run = Path(run).resolve()
    destination = Path(output).resolve() if output is not None else run / "derivative_checks/environment_derivatives.json"
    # Refuse before any expensive qualification or output-directory mutation.
    if destination.exists():
        raise FileExistsError(destination)
    source_path = run / "source.json"
    source = json.loads(source_path.read_text())
    primary = Path(source["primary_run"])
    if not primary.is_absolute():
        primary = ROOT / primary
    report = qualify(primary)
    report["run_source_sha256"] = digest(source_path)
    report["created_utc"] = datetime.now(timezone.utc).isoformat()
    report["reproduction"] = "same fixed probes and thresholds as original environment_derivatives.json; timing/source metadata are invocation-specific"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return dict(valid=report["valid"], report=str(destination), families=report["families"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True, help="prepared DIAG-02 run containing source.json")
    parser.add_argument("--output", type=Path, help="new report path; default RUN/derivative_checks/environment_derivatives.json")
    args = parser.parse_args()
    result = run_check(args.run, args.output)
    print(json.dumps(result, indent=2))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
