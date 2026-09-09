#!/usr/bin/env python3
"""Build a reference-only combined DATA-02 v1+v2 corpus and frozen split."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any, Mapping

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.data02_online_successive import percentile_summary
from reconciliation.data02_v2 import (
    combined_isolated_split,
    combined_readiness_decision,
    cross_cohort_duplicates,
    deterministic_combined_representatives,
)
from reconciliation.online_switch import save_json_exclusive, sha256_file


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v1-run", type=Path, required=True)
    parser.add_argument("--v2-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def strict_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def relative(path: Path) -> str:
    return str(path.resolve().relative_to(ROOT))


def source_description(cohort: str, run: Path) -> dict[str, Any]:
    metadata = strict_json(run / "metadata.json")
    validation = strict_json(run / "summary/validation.json")
    if not validation.get("valid"):
        raise ValueError(f"{cohort} strict validation is not valid")
    return {
        "cohort_id": cohort,
        "run_path": relative(run),
        "run_id": run.name,
        "collector_git_sha": metadata["collector_git_sha"],
        "metadata_sha256": sha256_file(run / "metadata.json"),
        "config_snapshot_sha256": sha256_file(run / "config_snapshot.yaml"),
        "protocol_sha256": sha256_file(run / "protocol.json"),
        "collection_manifest_sha256": sha256_file(run / "collection_manifest.json"),
        "transition_index_sha256": sha256_file(run / "summary/transition_index.json"),
        "strict_validation_sha256": sha256_file(run / "summary/validation.json"),
        "strict_validation_valid": True,
    }


def load_records(cohort: str, run: Path) -> list[dict[str, Any]]:
    config = yaml.safe_load((run / "config_snapshot.yaml").read_text(encoding="utf-8"))
    templates = {str(item["id"]): item for item in config["episode_templates"]}
    source_records = strict_json(run / "summary/transition_index.json")["records"]
    records = []
    for source in source_records:
        template = templates[str(source["template_id"])]
        source_transition = run / str(source["artifact_relative_path"]) / "transition.json"
        if not source_transition.is_file():
            raise ValueError(f"missing source transition: {source_transition}")
        row = dict(source)
        row.update({
            "cohort_id": cohort,
            "source_run_path": relative(run),
            "source_transition_relative_path": relative(source_transition),
            "source_transition_sha256": sha256_file(source_transition),
            "corpus_episode_id": f"{cohort}:{source['episode_id']}",
            "corpus_transition_id": f"{cohort}:{source['transition_id']}",
            "semantic_family": str(template["class"]),
            "physical_region": str(template["region"]),
        })
        records.append(row)
    return records


def write_csv_exclusive(path: Path, rows: list[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError("combined eligible index cannot be empty")
    with path.open("x", encoding="utf-8", newline="") as stream:
        columns = list(rows[0])
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="raise")
        writer.writeheader(); writer.writerows(rows)


def timing_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for label, subset in (
        ("v1", [row for row in records if row["cohort_id"] == "v1"]),
        ("v2", [row for row in records if row["cohort_id"] == "v2"]),
        ("combined", records),
    ):
        eligible = [row for row in subset if row["status"] == "ELIGIBLE_MOVING"]
        invalid = [row for row in subset if row["status"] == "TIMING_INVALID"]
        result[label] = {
            "attempt_count": len(subset), "eligible_count": len(eligible),
            "timing_invalid_count": len(invalid),
            "timing_invalid_fraction_of_attempts": len(invalid) / len(subset) if subset else None,
            "eligible_distributions": {
                field: percentile_summary(float(row[field]) for row in eligible)
                for field in ("host_latency_s", "model_reported_s", "tau_effective_s", "rtf")
            },
        }
    result["claim_limit"] = "v1/v2 collection-runtime comparison is descriptive; it is not a scientific latency-treatment effect"
    return result


def coverage_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for label, subset in (
        ("v1", [row for row in records if row["cohort_id"] == "v1"]),
        ("v2", [row for row in records if row["cohort_id"] == "v2"]),
        ("combined", records),
    ):
        eligible = [row for row in subset if row["status"] == "ELIGIBLE_MOVING"]
        pair = Counter(str(row["ordered_raw_pair_sha256"]) for row in eligible)
        result[label] = {
            "attempt_count": len(subset), "status_counts": dict(sorted(Counter(str(row["status"]) for row in subset).items())),
            "eligible_count": len(eligible),
            "unique_old_raw": len({row["old_raw_sha256"] for row in eligible}),
            "unique_fresh_raw": len({row["fresh_raw_sha256"] for row in eligible}),
            "unique_ordered_raw_pairs": len(pair),
            "largest_pair_count": max(pair.values(), default=0),
            "largest_pair_fraction": max(pair.values(), default=0) / len(eligible) if eligible else None,
            "geometry_counts": dict(sorted(Counter(str(row["fresh_geometry_bin"]) for row in eligible).items())),
            "difficulty_counts": dict(sorted(Counter(str(row["difficulty_bin"]) for row in eligible).items())),
            "semantic_family_counts": dict(sorted(Counter(str(row["semantic_family"]) for row in eligible).items())),
            "physical_region_counts": dict(sorted(Counter(str(row["physical_region"]) for row in eligible).items())),
        }
    return result


def main() -> None:
    options = arguments()
    v1 = options.v1_run.resolve(); v2 = options.v2_run.resolve(); output = options.output.resolve()
    if output.exists():
        raise FileExistsError(f"combined output already exists: {output}")
    output.mkdir(parents=True)
    (output / "split").mkdir()
    (output / "plots").mkdir()
    sources = [source_description("v1", v1), source_description("v2", v2)]
    records = load_records("v1", v1) + load_records("v2", v2)
    records.sort(key=lambda row: str(row["corpus_transition_id"]))
    eligible = [row for row in records if row["status"] == "ELIGIBLE_MOVING"]
    v1_config = yaml.safe_load((v1 / "config_snapshot.yaml").read_text(encoding="utf-8"))
    v2_template_manifest = strict_json(v2 / "template_bank/manifest.json")
    criteria = v1_config["readiness"]
    split = combined_isolated_split(
        records, heldout_fraction=float(criteria["heldout_fraction"]),
        required_families=v1_config.get("readiness", {}).get("required_split_semantic_families", ["straight", "left", "right", "doorway", "detour", "compound"]),
        required_geometry=("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING"),
        required_difficulty=("BENIGN", "CHALLENGING"),
    )
    decision = combined_readiness_decision(
        records, split, criteria, artifacts_valid=True,
        template_bank_valid=bool(v2_template_manifest.get("template_bank_valid")),
    )
    save_json_exclusive(output / "all_transition_index.json", {"records": records})
    write_csv_exclusive(output / "eligible_transition_index.csv", eligible)
    save_json_exclusive(output / "duplicate_groups.json", {
        "v2": cross_cohort_duplicates([row for row in records if row["cohort_id"] == "v2"]),
        "combined": cross_cohort_duplicates(records),
    })
    save_json_exclusive(output / "coverage_summary.json", coverage_summary(records))
    save_json_exclusive(output / "timing_summary.json", timing_summary(records))
    by_id = {row["corpus_transition_id"]: row for row in eligible}
    save_json_exclusive(output / "split/development.json", {
        "access": "OPEN_FOR_EXP02D_FORMULATION_DEVELOPMENT",
        "records": [by_id[identifier] for identifier in split["development_transition_ids"]],
    })
    save_json_exclusive(output / "split/heldout.json", {
        "access": "LOCKED_FOR_FINAL_EVALUATION",
        "records": [by_id[identifier] for identifier in split["heldout_transition_ids"]],
    })
    save_json_exclusive(output / "split/components.json", {"components": split["components"]})
    save_json_exclusive(output / "split/split_summary.json", {key: value for key, value in split.items() if key != "components"})
    representatives = deterministic_combined_representatives(records)
    save_json_exclusive(output / "representatives.json", {
        "selection_rule": "predeclared deterministic strata/extrema/pair-identity union plus ID-ordered fill",
        "records": [{**item, "source_transition_relative_path": by_id[item["corpus_transition_id"]]["source_transition_relative_path"]} for item in representatives],
    })
    save_json_exclusive(output / "readiness.json", decision)
    artifacts = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "combined_manifest.json" and "plots" not in path.parts:
            artifacts[str(path.relative_to(output))] = sha256_file(path)
    manifest = {
        "schema": "DATA02_CombinedReferenceCorpus_v1", "combined_id": output.name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "reference-only immutable v1+v2 index; no raw artifacts copied; no reconciliation evaluation",
        "sources": sources, "source_count": 2,
        "attempt_count": len(records), "eligible_count": len(eligible),
        "source_transition_hash_checked": True,
        "split_assignment": "connected components of shared episode or exact ordered raw-pair identity",
        "development_access": "OPEN_FOR_EXP02D_FORMULATION_DEVELOPMENT",
        "heldout_access": "LOCKED_FOR_FINAL_EVALUATION",
        "artifact_sha256": artifacts,
    }
    save_json_exclusive(output / "combined_manifest.json", manifest)
    print(json.dumps({"output": relative(output), "readiness": decision, "split": {"development": len(split["development_transition_ids"]), "heldout": len(split["heldout_transition_ids"]), "components": split["component_count"]}}, indent=2))


if __name__ == "__main__":
    main()
