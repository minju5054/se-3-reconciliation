from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import pytest
import yaml

from reconciliation.data02_v2 import (
    combined_episode_pair_components,
    combined_isolated_split,
    combined_readiness_decision,
    cross_cohort_duplicates,
    semantic_goal_equivalent,
    template_near_duplicate,
    timing_invalid_direction,
    validate_independent_template_bank,
)
from reconciliation.online_switch import sha256_file


ROOT = Path(__file__).resolve().parents[1]


def configs():
    v1 = yaml.safe_load((ROOT / "configs/data02_online_successive_v1.yaml").read_text())
    v2 = yaml.safe_load((ROOT / "configs/data02_online_successive_v2.yaml").read_text())
    return v1, v2


def test_v2_bank_is_new_balanced_and_predeclared_independent() -> None:
    v1, v2 = configs()
    result = validate_independent_template_bank(v2["episode_templates"], v1["episode_templates"])
    assert result["valid"]
    assert result["template_count"] == 24
    assert result["family_counts"] == {name: 4 for name in ("compound", "detour", "doorway", "left", "right", "straight")}
    assert v2["template_independence"]["lightnav_outputs_used"] is False


def test_near_duplicate_requires_pose_heading_and_semantic_equivalence() -> None:
    base = {"id": "old", "class": "left", "initial_pose_se2": [1, 2, 0], "expected_route": "turn left at junction"}
    same = {"id": "new", "class": "left", "initial_pose_se2": [1.1, 2.1, .1], "expected_route": "Turn left at junction."}
    different_goal = {**same, "id": "different", "expected_route": "turn right at junction"}
    assert semantic_goal_equivalent(base, same)
    assert template_near_duplicate(same, base)["flagged"]
    assert not template_near_duplicate(different_goal, base)["flagged"]


def test_v1_v2_scientific_and_successive_contracts_match() -> None:
    v1, v2 = configs()
    for section in ("baseline", "lightnav", "simulation", "robot", "environment", "camera", "follower", "variants"):
        assert v2[section] == v1[section]
    for key in (
        "initial_history_frames", "fresh_trigger_delay_sim_s", "maximum_transitions_per_episode",
        "maximum_fresh_wait_sim_s", "maximum_episode_sim_s", "no_reset_between_successive_chunks",
        "one_fresh_request_in_flight", "raw_fresh_observation_anchored", "prepend_switch_boundary", "waypoint_dt",
    ):
        assert v2["protocol"][key] == v1["protocol"][key]
    assert v2["protocol"]["rgb_persistence_mode"] == "buffered_episode_end"


def row(identifier: str, episode: str, pair: str, *, cohort: str = "v1", family: str = "straight", geometry: str = "STRAIGHT_LIKE", difficulty: str = "BENIGN", status: str = "ELIGIBLE_MOVING") -> dict:
    return {
        "corpus_transition_id": f"{cohort}:{identifier}", "corpus_episode_id": f"{cohort}:{episode}",
        "cohort_id": cohort, "ordered_raw_pair_sha256": pair, "status": status,
        "semantic_family": family, "fresh_geometry_bin": geometry, "difficulty_bin": difficulty,
        "old_raw_sha256": "old-" + pair, "fresh_raw_sha256": "fresh-" + pair,
        "host_latency_s": .5,
    }


def synthetic_records(count: int = 400) -> list[dict]:
    families = ("straight", "left", "right", "doorway", "detour", "compound")
    geometry = ("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING")
    difficulty = ("BENIGN", "CHALLENGING")
    return [
        row(
            f"t{index:04d}", f"e{index:04d}", f"pair{index % 100:03d}",
            cohort="v1" if index % 2 == 0 else "v2", family=families[index % len(families)],
            geometry=geometry[index % len(geometry)], difficulty=difficulty[index % 2],
        )
        for index in range(count)
    ]


def criteria() -> dict:
    return {
        "minimum_eligible_moving": 300, "minimum_unique_ordered_raw_pairs": 75,
        "maximum_largest_pair_fraction": .2, "minimum_unique_old_chunks": 25,
        "minimum_unique_fresh_chunks": 25, "minimum_straight_like": 30,
        "minimum_positive_turning": 20, "minimum_negative_turning": 20,
        "minimum_benign": 20, "minimum_challenging": 20,
        "minimum_heldout_eligible": 60,
    }


def split(records: list[dict]) -> dict:
    return combined_isolated_split(
        records, heldout_fraction=.25,
        required_families=("straight", "left", "right", "doorway", "detour", "compound"),
        required_geometry=("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING"),
        required_difficulty=("BENIGN", "CHALLENGING"),
    )


def test_cross_cohort_exact_pair_connects_whole_components_and_never_leaks() -> None:
    records = [
        row("a", "e0", "shared", cohort="v1"), row("b", "e1", "shared", cohort="v2"),
        row("c", "e1", "other", cohort="v2"), row("d", "e2", "unique", cohort="v1"),
    ]
    components = combined_episode_pair_components(records)
    assert any({item["corpus_transition_id"] for item in group} == {"v1:a", "v2:b", "v2:c"} for group in components)
    result = combined_isolated_split(records, heldout_fraction=.25, required_families=(), required_geometry=(), required_difficulty=())
    assert result["no_episode_leakage"] and result["no_ordered_raw_pair_leakage"]
    assert result == combined_isolated_split(records, heldout_fraction=.25, required_families=(), required_geometry=(), required_difficulty=())


def test_cross_cohort_duplicate_report() -> None:
    result = cross_cohort_duplicates([row("a", "e0", "same", cohort="v1"), row("b", "e1", "same", cohort="v2")])
    assert result["shared_pair_identity_count"] == 1
    assert result["groups"][0]["cohorts"] == ["v1", "v2"]


def test_combined_readiness_success_and_original_gate_failures() -> None:
    records = synthetic_records()
    frozen_split = split(records)
    result = combined_readiness_decision(records, frozen_split, criteria(), artifacts_valid=True, template_bank_valid=True)
    assert result["status"] == "DATA02_COMBINED_READY_FOR_EXP02D"
    assert all(result["checks"].values())
    sample_failure = combined_readiness_decision(records[:200], split(records[:200]), criteria(), artifacts_valid=True, template_bank_valid=True)
    assert not sample_failure["checks"]["A_minimum_eligible_moving"]
    dominated = [dict(item, ordered_raw_pair_sha256="dominant") for item in records]
    duplicate_failure = combined_readiness_decision(dominated, split(dominated), criteria(), artifacts_valid=True, template_bank_valid=True)
    assert not duplicate_failure["checks"]["C_largest_pair_fraction"]
    technical = combined_readiness_decision(records, frozen_split, criteria(), artifacts_valid=False, template_bank_valid=True)
    assert technical["status"] == "DATA02V2_TECHNICAL_INVALID"


def test_timing_invalid_direction_rejects_nonfinite() -> None:
    assert timing_invalid_direction(.8, [.9, 1.1]) == "BELOW_LOWER_RTF_BOUND"
    assert timing_invalid_direction(1.2, [.9, 1.1]) == "ABOVE_UPPER_RTF_BOUND"
    with pytest.raises(ValueError):
        timing_invalid_direction(float("nan"), [.9, 1.1])


def test_immutable_v1_run_level_hashes_when_dataset_is_present() -> None:
    run = ROOT / "data/data02_online_successive_v1/data02-online-successive-primary-v1"
    if not run.is_dir():
        pytest.skip("immutable generated v1 corpus is not present")
    expected = {
        "metadata.json": "dc30cfd27ce53da69b0b4445df4d692fdb5d184e889fa50e2494ab8f134a2320",
        "config_snapshot.yaml": "b92f4ddc1ba7e80d5544573ffedd8b8175e6b92ae79c8d79637cbb3e17c5a313",
        "protocol.json": "883099b35d0cec8724b08367057cd6b008b7d3eb3a96190a8ba36e14199e1868",
        "collection_manifest.json": "e3418cde107a4c644787cec45d3099bc856daa4dbcfbe2fa35d4da43125ad227",
        "summary/validation.json": "3d19b9829467cae1e78cfe44752eeecf55d466e2824120f8f574a99a43078fa2",
        "summary/transition_index.json": "b2c24b3cf510e1884369b81efc31c0caf8f9f249a93bd3d75c1e2124578dcf66",
    }
    assert {name: sha256_file(run / name) for name in expected} == expected
    assert json.loads((run / "summary/validation.json").read_text())["valid"] is True
