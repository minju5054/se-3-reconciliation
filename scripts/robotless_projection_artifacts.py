"""Frozen controlled-source access; no inference or source writes."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import robotless_controlled_staleness_artifacts as previous
from reconciliation.robotless_single_chunk import load_config, sha256_file
from reconciliation.robotless_controlled_staleness import finite_scalar

file_record, read_json, git = previous.file_record, previous.read_json, previous.git
SOURCE_VALIDATED = "ROBOTLESS_CONTROLLED_STALENESS_VALIDATED"


def validate_source(path: Path) -> dict:
    # Lazy import: the Isaac viewer never imports plotting or metric validation.
    from validate_robotless_controlled_staleness import validate_run
    return validate_run(path)


def config_read(path: Path) -> dict:
    config = load_config(path)
    if config.get("stage") != "robotless-projection-handoff-geometry" or config.get("schema_version") != 1:
        raise ValueError("wrong projection configuration")
    if not isinstance(config.get("source_run"), str):
        raise ValueError("source_run must be a path string")
    finite_scalar(config["numerical_validation"]["previous_d_poly_atol_m"], "distance tolerance", nonnegative=True)
    return config


def verify_files(root: Path, records: list[dict]) -> None:
    for record in records:
        path = (root / record["path"]).resolve()
        if not path.is_relative_to(root.resolve()) or sha256_file(path) != record["sha256"] or path.stat().st_size != record["bytes"]:
            raise ValueError(f"frozen source changed: {record['path']}")


def snapshot_source(path: Path) -> dict:
    result = validate_source(path)
    if result["status"] != SOURCE_VALIDATED:
        raise ValueError(f"controlled source validator did not pass: {result}")
    return {"source_run": str(path), "source_validation": result,
            "files": [file_record(p, path) for p in sorted(path.rglob("*")) if p.is_file()]}


def load_source(record: dict):
    path = Path(record["source_run"])
    if not path.is_absolute() or record["source_validation"]["status"] != SOURCE_VALIDATED:
        raise ValueError("validated absolute controlled source path required")
    required = {"source.json", "config_snapshot.yaml", "derived/boundaries.npy", "derived/metrics.json", "validation.json"}
    if not required.issubset({r["path"] for r in record["files"]}):
        raise ValueError("source manifest omits required controlled inputs")
    verify_files(path, record["files"])
    # Reuses the previous loader and its complete transitive successive hash audit.
    config, source_record, successive, metadata, arrays, boundaries = previous.load_inputs(path)
    metrics = read_json(path / "derived/metrics.json")
    return path, config, source_record, successive, metadata, arrays, boundaries, metrics


def load_inputs(run: Path):
    record = read_json(run / "source.json")
    config = config_read(run / "config_snapshot.yaml")
    if sha256_file(run / "config_snapshot.yaml") != record["configuration"]["sha256"]:
        raise ValueError("projection configuration snapshot changed")
    if previous.resolve_source(config["source_run"]) != Path(record["source_run"]):
        raise ValueError("controlled source path differs from configuration")
    return config, record, load_source(record)


def source_hashes() -> dict:
    files = ["src/reconciliation/robotless_projection_handoff.py", "src/reconciliation/robotless_controlled_staleness.py",
             "src/reconciliation/se2.py", "src/reconciliation/robotless_single_chunk.py",
             "scripts/robotless_projection_artifacts.py", "scripts/robotless_controlled_staleness_artifacts.py",
             "scripts/characterize_robotless_projection_handoff.py", "scripts/validate_robotless_controlled_staleness.py"]
    return {p: sha256_file(ROOT / p) for p in files}
