#!/usr/bin/env python3
"""Strict read-only validator for a reference-only combined DATA-02 corpus."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.data02_v2 import combined_isolated_split, combined_readiness_decision
from reconciliation.online_switch import sha256_file


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("combined_directory", type=Path)
    return parser.parse_args()


def strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def fail(errors: list[str], condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)


def main() -> None:
    combined = arguments().combined_directory.resolve()
    manifest = strict_json(combined / "combined_manifest.json")
    errors: list[str] = []
    fail(errors, manifest.get("schema") == "DATA02_CombinedReferenceCorpus_v1", "wrong combined schema")
    fail(errors, int(manifest.get("source_count", 0)) == 2, "combined source count is not two")
    source_runs: dict[str, Path] = {}
    for source in manifest.get("sources", []):
        cohort = str(source.get("cohort_id")); run = ROOT / str(source.get("run_path"))
        source_runs[cohort] = run
        for key, relative in (
            ("metadata_sha256", "metadata.json"),
            ("config_snapshot_sha256", "config_snapshot.yaml"),
            ("protocol_sha256", "protocol.json"),
            ("collection_manifest_sha256", "collection_manifest.json"),
            ("transition_index_sha256", "summary/transition_index.json"),
            ("strict_validation_sha256", "summary/validation.json"),
        ):
            path = run / relative
            fail(errors, path.is_file() and sha256_file(path) == source.get(key), f"{cohort} source hash mismatch: {relative}")
        validation = strict_json(run / "summary/validation.json")
        fail(errors, validation.get("valid") is True and source.get("strict_validation_valid") is True, f"{cohort} source validation is not valid")
    fail(errors, set(source_runs) == {"v1", "v2"}, "combined cohorts are not exactly v1 and v2")

    for relative, expected in manifest.get("artifact_sha256", {}).items():
        path = combined / str(relative)
        fail(errors, path.is_file() and sha256_file(path) == expected, f"combined artifact hash mismatch: {relative}")
    forbidden = [path for path in combined.rglob("*") if path.is_file() and path.suffix == ".npy"]
    fail(errors, not forbidden, "combined corpus contains copied NPY/raw arrays")
    fail(errors, not any((combined / name).exists() for name in ("episodes", "rgb", "chunks", "transitions")), "combined corpus contains copied source-data directories")

    records = strict_json(combined / "all_transition_index.json")["records"]
    fail(errors, len(records) == int(manifest.get("attempt_count", -1)), "combined attempt count mismatch")
    identifiers = [str(row["corpus_transition_id"]) for row in records]
    fail(errors, len(set(identifiers)) == len(identifiers), "duplicate corpus transition identifier")
    for row in records:
        source_path = ROOT / str(row["source_transition_relative_path"])
        fail(errors, source_path.is_file() and sha256_file(source_path) == row["source_transition_sha256"], f"source transition hash mismatch: {row['corpus_transition_id']}")
        fail(errors, str(row["corpus_episode_id"]).startswith(str(row["cohort_id"]) + ":"), f"episode namespace mismatch: {row['corpus_transition_id']}")
    with (combined / "eligible_transition_index.csv").open(encoding="utf-8", newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    eligible_ids = {str(row["corpus_transition_id"]) for row in records if row["status"] == "ELIGIBLE_MOVING"}
    fail(errors, {row["corpus_transition_id"] for row in csv_rows} == eligible_ids, "eligible CSV does not equal eligible union")
    fail(errors, len(csv_rows) == int(manifest.get("eligible_count", -1)), "eligible count mismatch")

    v1_config = yaml.safe_load((source_runs["v1"] / "config_snapshot.yaml").read_text(encoding="utf-8")) if "v1" in source_runs else {}
    criteria = v1_config.get("readiness", {})
    if criteria:
        recomputed = combined_isolated_split(
            records, heldout_fraction=float(criteria["heldout_fraction"]),
            required_families=("straight", "left", "right", "doorway", "detour", "compound"),
            required_geometry=("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING"),
            required_difficulty=("BENIGN", "CHALLENGING"),
        )
        stored = strict_json(combined / "split/split_summary.json")
        fail(errors, recomputed["development_transition_ids"] == stored.get("development_transition_ids"), "development split does not deterministically reconstruct")
        fail(errors, recomputed["heldout_transition_ids"] == stored.get("heldout_transition_ids"), "held-out split does not deterministically reconstruct")
        fail(errors, bool(stored.get("no_episode_leakage")), "episode leakage")
        fail(errors, bool(stored.get("no_ordered_raw_pair_leakage")), "ordered raw-pair leakage")
        template_manifest = strict_json(source_runs["v2"] / "template_bank/manifest.json")
        expected_decision = combined_readiness_decision(
            records, recomputed, criteria, artifacts_valid=True,
            template_bank_valid=bool(template_manifest.get("template_bank_valid")),
        )
        readiness = strict_json(combined / "readiness.json")
        fail(errors, readiness == expected_decision, "readiness does not reconstruct from original A-H criteria")
    development = strict_json(combined / "split/development.json")
    heldout = strict_json(combined / "split/heldout.json")
    fail(errors, development.get("access") == "OPEN_FOR_EXP02D_FORMULATION_DEVELOPMENT", "development access marker missing")
    fail(errors, heldout.get("access") == "LOCKED_FOR_FINAL_EVALUATION", "held-out lock marker missing")
    fail(errors, not ({row["corpus_transition_id"] for row in development.get("records", [])} & {row["corpus_transition_id"] for row in heldout.get("records", [])}), "split transition overlap")

    result = {
        "valid": not errors, "errors": errors,
        "attempt_count": len(records), "eligible_count": len(eligible_ids),
        "source_transition_hashes_verified": len(records),
        "combined_artifact_hashes_verified": len(manifest.get("artifact_sha256", {})),
        "no_raw_data_copied": not forbidden,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
