"""Synthetic file plumbing, with explicit mocked source validation."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import characterize_robotless_controlled_staleness as controlled
import characterize_robotless_projection_handoff as pipeline
import robotless_projection_artifacts as artifacts
import validate_robotless_projection_handoff as validator
from reconciliation.robotless_single_chunk import sha256_file
from test_robotless_controlled_staleness_artifacts import fixture_source


@pytest.fixture
def projection_source(fixture_source, monkeypatch):
    successive, cfg, source = fixture_source
    controlled.build_run(cfg, source)
    (source / "git_completion.json").write_text(json.dumps({"final_git_sha": "b"*40}))
    (source / "validation.json").write_text(json.dumps({"status": artifacts.SOURCE_VALIDATED, "synthetic_only": True}))
    monkeypatch.setattr(artifacts, "validate_source", lambda _: {"status": artifacts.SOURCE_VALIDATED, "synthetic_only": True})
    config = yaml.safe_load((ROOT / "configs/robotless_projection_handoff.yaml").read_text())
    config["source_run"] = str(source)
    config_path = source.parent / "projection.yaml"
    config_path.write_text(yaml.safe_dump(config))
    return successive, source, config_path, source.parent / "projection_output"


@pytest.fixture
def built(projection_source):
    successive, source, config, output = projection_source
    before = {str(p): sha256_file(p) for root in (successive, source) for p in root.rglob('*') if p.is_file()}
    pipeline.build_run(config, output)
    assert all(sha256_file(Path(p)) == digest for p, digest in before.items())
    return successive, source, output


def test_analysis_and_plots_do_not_establish_actual_runtime_pass(built):
    _, _, output = built
    details = validator.validate_analysis(output)
    assert len(details["conditions"]) == 4
    assert details["max_abs_previous_d_poly_difference_m"] == 0
    assert validator.validate_run(output)["status"] == validator.NOT_VALIDATED
    assert not (output / "raw").exists()


def test_overwrite_rejected(projection_source):
    _, _, config, output = projection_source
    output.mkdir()
    with pytest.raises(FileExistsError):
        pipeline.build_run(config, output)


def test_invalid_source_stops_before_outputs(projection_source, monkeypatch):
    _, _, config, output = projection_source
    monkeypatch.setattr(artifacts, "validate_source", lambda _: {"status": "FAILED"})
    with pytest.raises(ValueError, match="validator"):
        pipeline.build_run(config, output)
    assert not output.exists()


@pytest.mark.parametrize("which,path", [("controlled", "derived/boundaries.npy"), ("controlled", "derived/metrics.json"),
                                       ("successive", "raw/chunk_001.npy"), ("successive", "derived/chunk_001_world.npy")])
def test_source_changes_rejected(built, which, path):
    successive, source, output = built
    with ((source if which == "controlled" else successive) / path).open("ab") as f:
        f.write(b"changed")
    assert validator.validate_run(output)["status"] == validator.FAILED


@pytest.mark.parametrize("key", ["alpha", "e_perp_m", "s_Q_m", "e_dir_rad", "e_yaw_rad", "theta_Q_rad"])
def test_metric_changes_rejected(built, key):
    _, _, output = built
    path = output / "derived/projection_metrics.json"
    metrics = artifacts.read_json(path)
    metrics["conditions"][3][key] += .1
    path.write_text(json.dumps(metrics))
    assert validator.validate_run(output)["status"] == validator.FAILED


def test_unavailable_direction_can_be_plotted(tmp_path):
    (tmp_path / "derived").mkdir()
    (tmp_path / "evidence").mkdir()
    config = yaml.safe_load((ROOT / "configs/robotless_projection_handoff.yaml").read_text())
    (tmp_path / "config_snapshot.yaml").write_text(yaml.safe_dump(config))
    rows = [{"tau_s": t, "e_perp_m": 1., "abs_e_dir_deg": None, "abs_e_yaw_deg": 20., "s_Q_m": 0.} for t in [0, 1]]
    (tmp_path / "derived/projection_metrics.json").write_text(json.dumps({"conditions": rows}))
    pipeline.make_plots(tmp_path, config)
    assert len(artifacts.read_json(tmp_path / "plot_manifest.json")["plots"]) == 4


@pytest.fixture
def synthetic_reviewed(built):
    """Mocked renderer schema only; these images are NOT runtime evidence."""
    import shutil
    _, _, run = built
    _, record, loaded = artifacts.load_inputs(run)
    _, _, _, _, _, arrays, boundaries, _ = loaded
    rows = artifacts.read_json(run / "derived/projection_metrics.json")["conditions"]
    images = ["evidence/isaac_projection_overview.png"] + [f"evidence/tau_{i:03d}_detail.png" for i in range(len(rows))]
    for path in images:
        shutil.copyfile(run / "evidence/tau_vs_cross_track.png", run / path)
    visual = {
        "synthetic_only": True, "source_json_sha256": sha256_file(run / "source.json"),
        "metrics_sha256": sha256_file(run / "derived/projection_metrics.json"),
        "config_sha256": sha256_file(run / "config_snapshot.yaml"),
        "isaac_viewport_rendered": True, "new_lightnav_inference_count": 0,
        "timeline_time_s": 0, "geometry_scaling": 1, "scene_visibility_modifications": [], "fresh_reanchored": False,
        "scene": {"runtime_inventory": {"no_robot_model": True, "articulation_roots": [], "robot_named_paths": [],
            "rigid_body_count": 0, "physics_scene_count": 0, "dynamics_advanced": False,
            "authored_asset_references": ["synthetic.usd"]}},
        "agent_pose_before": record["R_obs"], "agent_pose_after": record["R_obs"],
        "fresh_world_drawn": arrays["FRESH_world"].tolist(), "boundaries_world": boundaries.tolist(),
        "created_time": record["created_time"], "details": rows,
        "screenshots": [artifacts.file_record(run / p, run) for p in images],
    }
    (run / "visualization.json").write_text(json.dumps(visual))
    images += [f"evidence/tau_vs_{name}.png" for _, name, _ in pipeline.PLOTS]
    (run / "visual_review.json").write_text(json.dumps({"synthetic_only": True, "reviewed": True,
        "checks": dict.fromkeys(validator.REVIEW_FLAGS, True), "image_sha256": {p: sha256_file(run / p) for p in images}}))
    return run


def test_complete_schema_with_mocked_source(synthetic_reviewed):
    assert validator.validate_run(synthetic_reviewed)["status"] == validator.VALIDATED


@pytest.mark.parametrize("key,value", [("isaac_viewport_rendered", False), ("new_lightnav_inference_count", 1),
    ("fresh_reanchored", True), ("geometry_scaling", 2), ("timeline_time_s", 1), ("agent_pose_after", [1, 0, 0])])
def test_visual_interpretation_changes_rejected(synthetic_reviewed, key, value):
    path = synthetic_reviewed / "visualization.json"
    visual = artifacts.read_json(path)
    visual[key] = value
    path.write_text(json.dumps(visual))
    assert validator.validate_run(synthetic_reviewed)["status"] == validator.FAILED


def test_missing_review_cannot_pass(synthetic_reviewed):
    (synthetic_reviewed / "visual_review.json").unlink()
    assert validator.validate_run(synthetic_reviewed)["status"] == validator.NOT_VALIDATED


def test_modified_rendered_projection_rejected(synthetic_reviewed):
    path = synthetic_reviewed / "visualization.json"
    visual = artifacts.read_json(path)
    visual["details"][3]["Q_xy_world_m"][0] += .1
    path.write_text(json.dumps(visual))
    assert validator.validate_run(synthetic_reviewed)["status"] == validator.FAILED


def test_reviewed_image_mutation_rejected(synthetic_reviewed):
    with (synthetic_reviewed / "evidence/tau_003_detail.png").open("ab") as stream:
        stream.write(b"modified")
    assert validator.validate_run(synthetic_reviewed)["status"] == validator.FAILED
