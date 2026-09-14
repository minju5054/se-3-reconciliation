"""Synthetic artifact tests. Mocked source validation is not runtime evidence."""

import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import characterize_robotless_controlled_staleness as pipeline
import robotless_controlled_staleness_artifacts as artifacts
import validate_robotless_controlled_staleness as validator
from reconciliation.robotless_single_chunk import sha256_file


@pytest.fixture
def fixture_source(tmp_path, monkeypatch):
    source = tmp_path / "synthetic_source"
    (source / "raw").mkdir(parents=True)
    (source / "derived").mkdir()
    for relative in artifacts.SOURCE_ARRAYS.values():
        np.save(source / relative, np.array([[.15, 0., 0.], [.5, 0., 0.], [1., 0., 0.]]))
    observation_time = {"utc": "2026-01-01T00:00:00Z", "monotonic_ns": 1, "simulation_time_s": 0.0}
    metadata = {"R1": [0., 0., 0.], "research_git_sha": "a" * 40,
                "observations": [{}, {"agent_pose_world": [0., 0., 0.], "observation_time": observation_time}]}
    (source / "metadata.json").write_text(json.dumps(metadata))
    (source / "git_completion.json").write_text(json.dumps({"final_git_sha": "b" * 40}))
    (source / "config_snapshot.yaml").write_text("synthetic_only: true\nscene:\n  asset_relative_path: synthetic.usd\n")
    result = {"status": artifacts.SOURCE_VALIDATED, "synthetic_only": True}
    (source / "validation.json").write_text(json.dumps(result))
    monkeypatch.setattr(artifacts, "validate_source", lambda _: copy.deepcopy(result))
    config = yaml.safe_load((ROOT / "configs/robotless_controlled_staleness.yaml").read_text())
    config["source_run"] = str(source)
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(config))
    return source, config_path, tmp_path / "output"


def test_build_preserves_source_and_has_one_fixed_fresh_reference(fixture_source):
    source, config, output = fixture_source
    hashes = {str(p): sha256_file(p) for p in source.rglob("*") if p.is_file()}
    result = pipeline.build_run(config, output)
    assert all(sha256_file(Path(path)) == value for path, value in hashes.items())
    assert not (output / "raw").exists()
    refs = [row["fixed_fresh_world_input"] for row in result["conditions"]]
    assert all(ref == refs[0] for ref in refs)
    analysis = validator.validate_analysis(output)
    assert len(analysis["conditions"]) == 4
    # Offline plots/metrics alone do not establish actual Isaac runtime success.
    assert validator.validate_run(output)["status"] == validator.NOT_VALIDATED


def test_output_overwrite_rejected(fixture_source):
    _, config, output = fixture_source
    output.mkdir()
    with pytest.raises(FileExistsError):
        pipeline.build_run(config, output)


def test_failed_source_validation_produces_no_output(fixture_source, monkeypatch):
    _, config, output = fixture_source
    monkeypatch.setattr(artifacts, "validate_source", lambda _: {"status": "FAILED"})
    with pytest.raises(ValueError, match="source successive validation failed"):
        pipeline.build_run(config, output)
    assert not output.exists()


def test_output_cannot_be_in_source(fixture_source):
    source, config, _ = fixture_source
    with pytest.raises(ValueError, match="separate"):
        pipeline.build_run(config, source / "new")


@pytest.fixture
def built_run(fixture_source):
    source, config, output = fixture_source
    pipeline.build_run(config, output)
    return source, output


@pytest.mark.parametrize("relative", list(artifacts.SOURCE_ARRAYS.values()) + ["metadata.json"])
def test_modified_source_artifact_rejected(built_run, relative):
    source, output = built_run
    with (source / relative).open("ab") as stream:
        stream.write(b"changed")
    assert validator.validate_run(output)["status"] == validator.FAILED


@pytest.mark.parametrize("relative", ["derived/boundaries.npy", "derived/metrics.json", "derived/metrics.csv", "plot_manifest.json"])
def test_mutated_derived_artifact_rejected(built_run, relative):
    _, output = built_run
    if relative.endswith(".npy"):
        data = np.load(output / relative)
        data[1, 0] += .1
        np.save(output / relative, data)
    elif relative.endswith(".json"):
        data = json.loads((output / relative).read_text())
        if relative.endswith("metrics.json"):
            data["conditions"][1]["d_poly_m"] += .1
        else:
            data["plots"][0]["distance_m"][1] += .1
        (output / relative).write_text(json.dumps(data))
    else:
        with (output / relative).open("a") as stream:
            stream.write("bad,row\n")
    assert validator.validate_run(output)["status"] == validator.FAILED


def test_metrics_use_true_log_and_csv_matches(built_run):
    _, output = built_run
    details = validator.validate_analysis(output)
    assert details["conditions"][3]["d_entry_m"] == pytest.approx(.1)
    assert details["conditions"][3]["d_poly_m"] == 0


def test_visual_input_loader_does_not_read_metrics(built_run):
    _, output = built_run
    (output / "derived/metrics.json").unlink()
    (output / "derived/metrics.csv").unlink()
    _, _, _, _, arrays, boundaries = artifacts.load_inputs(output)
    assert boundaries.shape == (4, 3)
    assert not arrays["FRESH_world"].flags.writeable


@pytest.fixture
def synthetic_reviewed_run(built_run):
    """Fake renderer records only inside a source-validator-mocked unit test."""
    import shutil
    source, output = built_run
    record = artifacts.read_json(output / "source.json")
    b = np.load(output / "derived/boundaries.npy")
    screenshot = "evidence/isaac_handoff_overview.png"
    shutil.copyfile(output / "evidence/latency_vs_entry_gap.png", output / screenshot)
    visual = {
        "synthetic_only": True, "source_json_sha256": sha256_file(output / "source.json"),
        "config_sha256": sha256_file(output / "config_snapshot.yaml"),
        "boundaries_input": artifacts.file_record(output / "derived/boundaries.npy", output),
        "isaac_viewport_rendered": True, "metrics_file_used": False, "new_lightnav_inference_count": 0,
        "timeline_time_s": 0.0, "geometry_scaling": 1, "scene_visibility_modifications": [],
        "created_time": record["created_time"], "R_obs_world": record["R_obs"],
        "agent_pose_before": record["R_obs"], "agent_pose_after": record["R_obs"],
        "boundaries_world": b.tolist(), "tau_s": [0, .2, .5, 1],
        "fresh_world_instances": 1, "observation_to_entry_connector": False,
        "trajectory_inputs": {k: record["array_inputs"][k] for k in ("OLD_world", "FRESH_world")},
        "drawn_trajectories": {k: np.load(source / artifacts.SOURCE_ARRAYS[k]).tolist() for k in ("OLD_world", "FRESH_world")},
        "screenshot": artifacts.file_record(output / screenshot, output),
        "scene": {"runtime_inventory": {"no_robot_model": True, "articulation_roots": [], "robot_named_paths": [],
                  "rigid_body_count": 0, "physics_scene_count": 0, "dynamics_advanced": False,
                  "authored_asset_references": ["synthetic.usd"]}},
    }
    (output / "visualization.json").write_text(json.dumps(visual))
    review = {"synthetic_only": True, "reviewed": True, "checks": dict.fromkeys(validator.REVIEW_CHECKS, True),
              "image_sha256": {p: sha256_file(output / p) for p in validator.IMAGES}}
    (output / "visual_review.json").write_text(json.dumps(review))
    return output


def test_complete_validation_schema_with_mocked_source(synthetic_reviewed_run):
    # Schema plumbing only; the fixture explicitly carries synthetic_only.
    assert validator.validate_run(synthetic_reviewed_run)["status"] == validator.VALIDATED


@pytest.mark.parametrize("key,value", [("metrics_file_used", True), ("fresh_world_instances", 4),
    ("observation_to_entry_connector", True), ("new_lightnav_inference_count", 1),
    ("geometry_scaling", 2), ("timeline_time_s", 1), ("R_obs_world", [1, 0, 0])])
def test_invalid_visual_interpretation_rejected(synthetic_reviewed_run, key, value):
    path = synthetic_reviewed_run / "visualization.json"
    visual = artifacts.read_json(path)
    visual[key] = value
    path.write_text(json.dumps(visual))
    assert validator.validate_run(synthetic_reviewed_run)["status"] == validator.FAILED


def test_unreviewed_images_cannot_establish_pass(synthetic_reviewed_run):
    path = synthetic_reviewed_run / "visual_review.json"
    review = artifacts.read_json(path)
    review["reviewed"] = False
    path.write_text(json.dumps(review))
    assert validator.validate_run(synthetic_reviewed_run)["status"] == validator.FAILED
