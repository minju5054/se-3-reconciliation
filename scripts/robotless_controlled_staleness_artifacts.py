"""Read-only frozen-source access shared by analysis, validation and Isaac view."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.robotless_single_chunk import load_config, sha256_file, validate_waypoints
from reconciliation.robotless_controlled_staleness import validate_config
from validate_robotless_successive_chunks import VALIDATED as SOURCE_VALIDATED, validate_run as validate_source


SOURCE_ARRAYS = {
    "OLD_raw": "raw/chunk_000.npy", "FRESH_raw": "raw/chunk_001.npy",
    "OLD_world": "derived/chunk_000_world.npy", "FRESH_world": "derived/chunk_001_world.npy",
}


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def resolve_source(value: str) -> Path:
    path = Path(value)
    return (path if path.is_absolute() else ROOT / path).resolve()


def git(*args: str) -> str:
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def file_record(path: Path, relative_to: Path) -> dict:
    return {"path": str(path.relative_to(relative_to)), "sha256": sha256_file(path), "bytes": path.stat().st_size}


def snapshot_source(source: Path) -> dict:
    """Validate first; hash all existing source-run files without writing to it."""
    result = validate_source(source)
    if result["status"] != SOURCE_VALIDATED:
        raise ValueError(f"source successive validation failed: {result}")
    return {"source_run": str(source), "source_validation": result,
            "files": [file_record(path, source) for path in sorted(source.rglob("*")) if path.is_file()]}


def verify_source(record: dict) -> Path:
    source = Path(record["source_run"])
    if not source.is_absolute() or record["source_validation"]["status"] != SOURCE_VALIDATED:
        raise ValueError("source record needs an absolute path and validated source")
    for item in record["files"]:
        path = (source / item["path"]).resolve()
        if not path.is_relative_to(source) or sha256_file(path) != item["sha256"] or path.stat().st_size != item["bytes"]:
            raise ValueError(f"source artifact changed: {item['path']}")
    required = set(SOURCE_ARRAYS.values()) | {"metadata.json", "config_snapshot.yaml", "validation.json"}
    if not required.issubset({entry["path"] for entry in record["files"]}):
        raise ValueError("source manifest omits required inputs")
    return source


def source_arrays(source: Path) -> dict:
    arrays = {name: validate_waypoints(np.load(source / rel, allow_pickle=False)) for name, rel in SOURCE_ARRAYS.items()}
    for array in arrays.values():
        array.setflags(write=False)
    return arrays


def load_inputs(run: Path) -> tuple[dict, dict, Path, dict, dict, np.ndarray]:
    """No metrics file is read: visualization can independently inspect geometry."""
    record = read_json(run / "source.json")
    source = verify_source(record)
    config = load_config(run / "config_snapshot.yaml")
    validate_config(config)
    if source != resolve_source(config["source_run"]):
        raise ValueError("source path differs from configuration")
    if sha256_file(run / "config_snapshot.yaml") != record["configuration"]["sha256"]:
        raise ValueError("configuration snapshot changed")
    metadata = read_json(source / "metadata.json")
    if record["R_obs"] != metadata["R1"] or record["R_obs"] != metadata["observations"][1]["agent_pose_world"]:
        raise ValueError("R_obs must be the FRESH observation pose R1")
    arrays = source_arrays(source)
    boundaries = validate_waypoints(np.load(run / "derived/boundaries.npy", allow_pickle=False))
    return config, record, source, metadata, arrays, boundaries


def processing_sources() -> dict:
    paths = ["src/reconciliation/robotless_controlled_staleness.py", "src/reconciliation/se2.py",
             "src/reconciliation/robotless_single_chunk.py", "scripts/robotless_controlled_staleness_artifacts.py",
             "scripts/characterize_robotless_controlled_staleness.py", "scripts/validate_robotless_successive_chunks.py",
             "scripts/validate_robotless_single_chunk.py"]
    return {rel: sha256_file(ROOT / rel) for rel in paths}
