"""Synthetic saved-result plot qualification, never actual-event evidence."""
import copy
import importlib.util
import json
from pathlib import Path
import re
import zipfile

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import LineString, box

from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.gp_se2_formulation import GPProblem

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("diag02_plots", ROOT / "scripts/plot_gp_se2_diag_02.py")
plots = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plots)


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(plots.plain(value)))


@pytest.fixture
def fixture(tmp_path):
    p = GPProblem([0., 0., 0.], [.2, 0., 0.], [[.02, 0., 0.], [.04, 0., 0.]], [.04, 0., 0.],
                  dict(horizon_s=.2, support_dt_s=.1), include_obstacles=False)
    init = p.initializations()[0]; vector = p.vector(init["poses"], init["twists"])
    collocation = dict(maximum_equality_residual=0., maximum_inequality_violation=0., feasible=True)
    result = dict(variable_count=10, config=p.config, initial_vector=vector,
        latest_support_poses=init["poses"], latest_support_twists=init["twists"],
        support_poses=init["poses"], support_twists=init["twists"], candidate_found=True,
        termination="CONVERGED", iterations=1,
        candidate_checks=[dict(iterate="initial", objective=9., collocation=collocation,
                               constraint_report=dict(equality_residuals=[0., 0.], inequality_margins=[1.] * 48))],
        callback_snapshots=[dict(iteration=1, elapsed_s=.5, objective=5., collocation=collocation,
                                 equality_residuals=[0., 0.], inequality_margins=[1.] * 48,
                                 source="synthetic test callback")],
        solve_wall_time_s=1., setup_wall_time_s=.01, post_solve_validation_time_s=.02,
        derivative_callback_seconds=dict(objective_gradient=.02, equality_jacobian=.03, inequality_jacobian=.04),
        derivative_calls=dict(objective_gradient=1, equality_jacobian=1, inequality_jacobian=1),
        derivative_provider_stats=dict(scope="synthetic"),
        objective_evaluations=8, equality_evaluations=10, inequality_evaluations=10,
        profiling=dict(evaluate_inclusive_seconds=.25, unique_vector_evaluations=5,
                       numerical_component_seconds={"environment_query":.05}))
    save(tmp_path / "execution_completed.json", dict(solve_count=8))
    save(tmp_path / "source.json", dict(primary_run="synthetic injected fixture"))
    save(tmp_path / "protocol.json", dict(synthetic_only=True))
    (tmp_path / "config_snapshot.yaml").write_text("synthetic_only: true\n")
    save(tmp_path / "actual_event/input_snapshot.json", dict(case_id="SYNTHETIC/TEST", F_native=p.common_reference,
         context=dict(B_world=p.boundary_pose), goal_route=dict(goal_world=p.goal_pose)))
    for method in plots.METHODS:
        for seed in plots.SEEDS:
            for mode in plots.MODES:
                r = copy.deepcopy(result)
                full = mode == "SUPPLIED_JAC"
                initial_full = seed == "I1_DECEL"
                if mode == "FD_BASELINE":
                    r.update(termination="TIMEOUT", support_poses=None, support_twists=None, candidate_found=False,
                             derivative_callback_seconds={k:0. for k in r["derivative_callback_seconds"]},
                             derivative_calls={k:0 for k in r["derivative_calls"]})
                if mode == "FD_BASELINE" and seed == "I0_FRESH":
                    r.update(latest_support_poses=None, latest_support_twists=None)
                    r["callback_snapshots"] = [dict(iteration=1, elapsed_s=.5, objective=None, collocation=None,
                        equality_residuals=None, inequality_margins=None, source="synthetic incomplete callback")]
                summary = dict(method=method, derivative_mode=mode, initialization=seed,
                    termination=r["termination"], initial_full_feasible=initial_full,
                    final_full_feasible=full, selected_full_feasible=full, iterations=1,
                    initial_objective=9., final_objective=5., selected_objective=5. if full else None,
                    solve_wall_time_s=1., independent_full_validation_time_s=.1,
                    optimality_diagnostic_time_s=None, optimality_provider_setup_s=None)
                setup = dict(case_loading_seed_preparation_s=.005, derivative_graph_construction_s=.01,
                    compilation_first_call_warmup_s=.1, shared_environment_loading_s=.2)
                post = dict(rows=[dict(iterate="initial", objective=9., full_feasible=initial_full, discovery_time_s=0., vector_sha256="test_seed"),
                                 dict(iterate="latest_iterate", objective=5., full_feasible=full, discovery_time_s=1., vector_sha256="test_final")])
                directory = tmp_path / "actual_event" / method / mode / seed
                for name, data in (("solver_result", r), ("result_summary", summary), ("full_acceptance", dict(full_feasible=full)),
                                   ("setup", setup), ("post_full_checks", post)):
                    save(directory / f"{name}.json", data)
    return tmp_path, dict(problem=p, environment=HospitalEnvironment(LineString([(.3, -.5), (.3, .5)]), box(-1., -1., 1., 1.)))


def test_all_eighteen_plots_provenance_dpi_eight_cells_links_bundle_and_overwrite(fixture):
    run, loaded = fixture
    result = plots.generate(run, loaded=loaded)
    assert result["plot_count"] == 18 and result["comparison_cell_count"] == 8
    manifest = plots.read(run / "plot_manifest.json")
    assert {Path(row["path"]).stem for row in manifest["records"]} == set(plots.PLOT_NAMES)
    hashes = {}
    for row in manifest["records"]:
        png, sidecar = run / row["path"], run / row["sidecar"]
        assert plots.file_sha256(png) == row["sha256"]
        hashes[png] = row["sha256"]
        assert plots.file_sha256(sidecar) == row["sidecar_sha256"]
        with Image.open(png) as image:
            assert min(image.info["dpi"]) >= 159.9
        meta = plots.read(sidecar)
        assert meta["image_sha256"] == row["sha256"]
        assert meta["no_new_execution"]
        for path, digest in meta["source_sha256"].items():
            assert plots.file_sha256(path) == digest
        if png.stem == "world_xy":
            assert meta["numeric_data"]["aspect"] == "equal"
        if png.stem == "objective_iteration":
            assert meta["numeric_data"]["I0_FRESH/FD_BASELINE"]["objective"] == [9., None]
        if png.stem == "best_full_feasible_objective":
            assert [r["best_full_objective"] for r in meta["numeric_data"]["I0_FRESH/FD_BASELINE"]["points"]] == [None, None]
            assert meta["numeric_data"]["I1_DECEL/SUPPLIED_JAC"]["seed_full_feasible"]
    for directory in (run, run / "review_bundle"):
        index = (directory / "index.html").read_text()
        for link in re.findall(r'(?:href|src)="([^"]+)"', index):
            assert (directory / link).is_file()
        assert index.count("<tr><td>") == 8
        assert "TIMEOUT" in index
    with zipfile.ZipFile(run / "review_bundle.zip") as archive:
        assert archive.testzip() is None
        names = archive.namelist()
        assert len([name for name in names if name.endswith(".png")]) == 18
        assert not any(Path(name).suffix in (".npy", ".npz", ".wkb", ".usd", ".pt") for name in names)
        assert not any(name.endswith("solver_result.json") for name in names)
        inventory = json.loads(archive.read("manifest.json"))
        assert set(names) == {r["path"] for r in inventory["allowlisted_files"]} | {"manifest.json"}
    with pytest.raises(FileExistsError):
        plots.generate(run, loaded=loaded)
    assert all(plots.file_sha256(path) == value for path, value in hashes.items())


def test_incomplete_or_low_dpi_refused_before_output(fixture):
    run, loaded = fixture
    with pytest.raises(ValueError, match="160 dpi"):
        plots.generate(run, loaded=loaded, dpi=100)
    assert not (run / "plots").exists()
    (run / "actual_event/M3_GP_CONSTRAINED/SUPPLIED_JAC/I1_DECEL/result_summary.json").unlink()
    with pytest.raises(FileNotFoundError):
        plots.generate(run, loaded=loaded)
    assert not (run / "plots").exists()


def test_full_feasible_history_is_postcertified_and_never_fabricates_zero():
    record = {"post_full_checks":{"rows":[
        dict(iterate="initial", objective=100., full_feasible=False, discovery_time_s=0.),
        dict(iterate="callback_0002", objective=8., full_feasible=True, discovery_time_s=2.),
        dict(iterate="callback_0003", objective=9., full_feasible=True, discovery_time_s=3.),
        dict(iterate="latest_iterate", objective=2., full_feasible=False, discovery_time_s=4.)]}}
    history = plots.best_full_history(record)
    assert [p["best_full_objective"] for p in history["points"]] == [None, 8., 8., 8.]
    assert not history["seed_full_feasible"]
    assert "post-solve" in history["certification"]


def test_disjoint_profile_excludes_nested_and_separate_clocks(fixture):
    run, _ = fixture
    record = plots.records(run)[plots.METHODS[0], plots.SEEDS[0], "SUPPLIED_JAC"]
    profile = plots.disjoint_profile(record)
    parts = profile["disjoint_wall_seconds"]
    assert parts["primal_evaluator"] + parts["supplied_derivative_callbacks"] + parts["solver_and_bookkeeping_outside_callbacks"] == pytest.approx(1.)
    assert profile["total_seconds"] == pytest.approx(1.245)
    assert "shared_environment_loading_s" not in parts
    assert "optional_optimality_diagnostic_seconds" not in parts
    record["solver_result"]["profiling"]["evaluate_inclusive_seconds"] = 2.
    with pytest.raises(ValueError, match="overlap"):
        plots.disjoint_profile(record)
