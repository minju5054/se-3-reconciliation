"""Validated, versioned oracle annotations for EXP-02."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Mapping

import yaml

from reconciliation.se2_graph import OracleCorrespondence, validate_correspondences


MEASUREMENT_CONVENTION = "Z_ij = inverse(O_i) * desired_X_j"


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_oracle_annotation(
    path: str | Path, *, old_count: int, new_count: int
) -> tuple[dict[str, Any], tuple[OracleCorrespondence, ...]]:
    annotation_path = Path(path)
    with annotation_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("oracle annotation must be a schema-version-1 mapping")
    convention = document.get("measurement_convention", {})
    if not isinstance(convention, Mapping) or convention.get("Z_ij") != MEASUREMENT_CONVENTION:
        raise ValueError("oracle measurement convention is missing or unsupported")
    raw = document.get("correspondences")
    if not isinstance(raw, list):
        raise ValueError("oracle correspondences must be a list")
    correspondences = tuple(
        OracleCorrespondence(
            old_index=item["old_index"],
            new_index=item["new_index"],
            relative_transform=item["relative_transform"],
            rationale=item.get("rationale", ""),
            confidence=item.get("confidence"),
        )
        for item in raw
        if isinstance(item, Mapping)
    )
    if len(correspondences) != len(raw):
        raise ValueError("each oracle correspondence must be a mapping")
    return document, validate_correspondences(
        correspondences, old_count=old_count, new_count=new_count
    )


def validate_source_hashes(annotation: Mapping[str, Any], trial_directory: str | Path) -> None:
    source = annotation.get("source_experiment")
    if not isinstance(source, Mapping):
        raise ValueError("oracle source_experiment is missing")
    root = Path(trial_directory)
    expected = {
        "raw_old_sha256": sha256_file(root / "raw/old_actions.npy"),
        "raw_new_sha256": sha256_file(root / "raw/new_actions.npy"),
    }
    for field, actual in expected.items():
        if source.get(field) != actual:
            raise ValueError(f"oracle {field} does not match immutable source")
