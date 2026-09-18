"""Synthetic plotting qualification; these fixtures are not research evidence."""
from __future__ import annotations

import hashlib
from html.parser import HTMLParser
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import box
import yaml

from reconciliation.gp_se2_environment import HospitalEnvironment


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/plot_gp_se2_01.py"
SPEC = importlib.util.spec_from_file_location("gp_se2_plots_qualification", SCRIPT)
plots = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plots)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def fixture_run(tmp_path):
    run = tmp_path / "synthetic_plot_fixture"
    run.mkdir()
    config = {"plots": {"dpi": 160}, "footprint": {"radius_m": .20, "required_clearance_m": .05},
              "formulation": {"goal_position_tolerance": .15}}
    (run / "config_snapshot.yaml").write_text(yaml.safe_dump(config))
    write(run / "source.json", {"synthetic": True, "warning": "implementation fixture only"})
    write(run / "protocol.json", {"synthetic": True})
    write(run / "aggregate/summary.json", {"native_success_to_gp_failure": 1, "adapter_success_to_gp_failure": 1})
    selected = {"case_id": "synthetic_001/handoff_001", "case_directory": "synthetic_001__handoff_001", "selected_group": "SYNTHETIC_ONLY"}
    write(run / "case_manifest.json", {"selected": [selected]})
    case = run / "cases" / selected["case_directory"]
    world = np.column_stack([np.linspace(.03, .9, 30), np.zeros((30, 2))])
    write(case / "input_context.json", {"B_world": [0., 0., 0.], "u_minus": [.2, .1],
          "old_world": [[-.3, 0., 0.], [0., 0., 0.], [.3, 0., 0.]], "fresh_world": world.tolist()})
    write(case / "goal_route.json", {"goal_world": [.9, 0., 0.], "gates": [
          {"center_xy": [.45, 0.], "normal_xy": [1., 0.], "half_width_m": .5}]})
    times = np.linspace(0., 3., 301)
    poses = np.column_stack([.3 * times, np.zeros((len(times), 2))])
    knots = np.linspace(0., 3., 31)
    support = np.column_stack([.3 * knots, np.zeros((31, 2))])
    twists = np.tile([.3, 0., 0.], (31, 1))
    for method in plots.METHODS:
        target = case / "methods" / method
        failed = method in ("M1_RIGID", "M3_GP_CONSTRAINED")
        solver = {"status": "NO_FEASIBLE_CANDIDATE_FOUND" if failed else "FEASIBLE_CANDIDATE_FOUND",
                  "candidate_world": None if failed else world.tolist(), "factor_costs": None,
                  "support_poses": None, "support_twists": None}
        if method == "M2_GP_NO_OBSTACLE":
            solver.update(support_times=knots.tolist(), support_poses=support.tolist(), support_twists=twists.tolist(),
                          factor_costs={"gp": [0.] * 30, "fresh_uniform": .125})
        if method == "M3_GP_CONSTRAINED":
            solver.update(support_times=knots.tolist(), diagnostic_candidate_world=(world + [0., .3, 0.]).tolist(),
                          diagnostic_support_poses=(support + [0., .3, 0.]).tolist(), diagnostic_support_twists=twists.tolist())
        write(target / "solver_result.json", solver)
        metrics = {"primary_success": not failed, "failure_reasons": ["no_candidate"] if failed else [],
                   "environment": None, "dense_poses_world": None, "dense_times_s": None}
        plan = {"status": "NO_CANDIDATE" if failed else "SAMPLED_PLAN_VALID", "environment": None}
        if not failed:
            np.save(target / "candidate_world.npy", world)
            metrics.update(dense_times_s=times.tolist(), dense_poses_world=poses.tolist(),
                           environment={"clearance_samples_m": (1.8 - poses[:, 0]).tolist()})
            plan.update(dense_times_s=times.tolist(), environment={"clearance_samples_m": (1.8 - poses[:, 0]).tolist()})
            rollout = {"controller_reference_selections": [{"time_s": i / 10, "command": [.3, 0.]} for i in range(30)]}
            write(target / "rollout/rollout.json", rollout)
        write(target / "metrics.json", metrics)
        write(target / "plan_validation.json", plan)
    return run, case


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []
    def handle_starttag(self, tag, attrs):
        self.links.extend(value for key, value in attrs if key in ("src", "href"))


def test_every_method_png_provenance_failure_visibility_index_and_overwrite(fixture_run, monkeypatch):
    run, case = fixture_run
    environment = HospitalEnvironment(box(2., -2., 2.2, 2.), box(-3., -3., 3., 3.))
    monkeypatch.setattr(plots.HospitalEnvironment, "load", lambda path: environment)
    snapshots = {}
    original_save = plots.save
    def inspected_save(figure, path, title, dpi):
        snapshots[path] = {"title": title, "texts": [text.get_text() for axis in figure.axes for text in axis.texts],
                           "lines": [(line.get_label(), np.asarray(line.get_ydata()).tolist()) for axis in figure.axes for line in axis.lines],
                           "axes_aspects": [axis.get_aspect() for axis in figure.axes]}
        original_save(figure, path, title, dpi)
    monkeypatch.setattr(plots, "save", inspected_save)
    plots.main(run)
    completion = json.loads((run / "plot_completion.json").read_text())
    assert completion["method_image_count"] == 46
    assert completion["common_overlay_count"] == 1
    assert completion["index_sha256"] == digest(run / "index.html")
    assert len(snapshots) == 47
    html = (run / "index.html").read_text()
    parser = Links()
    parser.feed(html)
    assert parser.links and all((run / path).is_file() for path in parser.links)
    assert "Native success → M3 failure: 1" in html
    for method in plots.METHODS:
        directory = case / "methods" / method / "plots"
        expected = set(plots.REQUIRED + (plots.GP_EXTRA if method.startswith(("M2", "M3")) else ()))
        assert {path.name for path in directory.glob("*.png")} == expected
        provenance = json.loads((directory / "plot_provenance.json").read_text())
        assert {record["path"] for record in provenance["images"]} == expected
        assert provenance["dpi"] >= 160
        assert provenance["config_sha256"] == digest(run / "config_snapshot.yaml")
        assert provenance["source_sha256"] == digest(run / "source.json")
        assert provenance["metrics_sha256"] == digest(directory.parent / "metrics.json")
        assert provenance["solver_sha256"] == digest(directory.parent / "solver_result.json")
        for record in provenance["images"]:
            path = directory / record["path"]
            assert record["sha256"] == digest(path)
            assert str(path.relative_to(run)) in parser.links
            with Image.open(path) as image:
                assert image.format == "PNG"
                assert min(image.info["dpi"]) >= 159.9
                assert image.width >= 1000 and image.height >= 700
                image.verify()
        for filename in plots.REQUIRED[:4]:
            assert snapshots[directory / filename]["axes_aspects"] == [1.]
        if method in ("M1_RIGID", "M3_GP_CONSTRAINED"):
            for filename in ("linear_command_vs_time.png", "angular_command_vs_time.png"):
                snap = snapshots[directory / filename]
                assert "NO_FEASIBLE_CANDIDATE_FOUND" in snap["title"]
                assert any("No zero-filled command trace" in text for text in snap["texts"])
                assert not any(label == "applied command" for label, _ in snap["lines"])
            snap = snapshots[directory / "clearance_vs_time.png"]
            assert any("No clearance curve is fabricated" in text for text in snap["texts"])
            assert not any(label in ("plan / reference", "actual MPC rollout") for label, _ in snap["lines"])
    failed_gp = case / "methods/M3_GP_CONSTRAINED/plots"
    assert any(label == "REJECTED last iterate (no candidate)" for label, _ in snapshots[failed_gp / "candidate_overlay.png"]["lines"])
    assert any("Accepted factor costs: N/A" in text for text in snapshots[failed_gp / "factor_costs.png"]["texts"])
    hashes = {str(path.relative_to(run)): digest(path) for path in run.rglob("*") if path.is_file()}
    with pytest.raises(FileExistsError, match="overwrite"):
        plots.main(run)
    assert hashes == {str(path.relative_to(run)): digest(path) for path in run.rglob("*") if path.is_file()}


def test_partial_existing_method_directory_refuses_overwrite(fixture_run, monkeypatch):
    run, case = fixture_run
    monkeypatch.setattr(plots.HospitalEnvironment, "load", lambda path: HospitalEnvironment(box(2., -2., 2.2, 2.), box(-3., -3., 3., 3.)))
    existing = case / "methods/M0_NATIVE/plots"
    existing.mkdir()
    sentinel = existing / "candidate_overlay.png"
    sentinel.write_bytes(b"existing artifact must not change")
    with pytest.raises(FileExistsError):
        plots.main(run)
    assert sentinel.read_bytes() == b"existing artifact must not change"


def test_low_dpi_rejected_before_writing_plots(fixture_run, monkeypatch):
    run, case = fixture_run
    config = yaml.safe_load((run / "config_snapshot.yaml").read_text())
    config["plots"]["dpi"] = 120
    (run / "config_snapshot.yaml").write_text(yaml.safe_dump(config))
    monkeypatch.setattr(plots.HospitalEnvironment, "load", lambda path: HospitalEnvironment(box(2., -2., 2.2, 2.), box(-3., -3., 3., 3.)))
    with pytest.raises(ValueError, match="160"):
        plots.main(run)
    assert not (case / "plots").exists()


def test_empty_controller_log_has_no_fabricated_command_curve(fixture_run, monkeypatch):
    run, case = fixture_run
    method = "M1_RIGID"
    target = case / "methods" / method
    write(target / "rollout/rollout.json", {"status": "CONTROLLER_RUNTIME_FAILED", "controller_reference_selections": []})
    loaded = plots.load_methods(case)
    seen = {}
    def inspect_only(figure, path, title, dpi):
        seen[path.name] = {"labels": [line.get_label() for axis in figure.axes for line in axis.lines],
                           "texts": [text.get_text() for axis in figure.axes for text in axis.texts]}
        plots.plt.close(figure)
    monkeypatch.setattr(plots, "save", inspect_only)
    config = yaml.safe_load((run / "config_snapshot.yaml").read_text())
    row = json.loads((run / "case_manifest.json").read_text())["selected"][0]
    env = HospitalEnvironment(box(2., -2., 2.2, 2.), box(-3., -3., 3., 3.))
    plots.plot_method(run, case, row, method, loaded[method], env, plots.limits_for(case, loaded), config)
    for filename in ("linear_command_vs_time.png", "angular_command_vs_time.png"):
        assert "applied command" not in seen[filename]["labels"]
        assert any("NO" in text or "unavailable" in text.lower() for text in seen[filename]["texts"])
