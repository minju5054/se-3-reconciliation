"""Pure DATA-02 v2 extension, diagnosis, and combined-corpus contracts.

All trajectory rows remain untimed spatial SE(2) waypoints.  The helpers in
this module operate on metadata and immutable identities; they do not modify
LightNav outputs or implement reconciliation.
"""

from __future__ import annotations

from collections import Counter, defaultdict, deque
import hashlib
import math
import re
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from reconciliation.data02_online_successive import percentile_summary
from reconciliation.se2 import wrap_angle


SEMANTIC_FAMILIES = ("straight", "left", "right", "doorway", "detour", "compound")
COMBINED_FINAL_STATUSES = (
    "DATA02_COMBINED_READY_FOR_EXP02D",
    "DATA02_COMBINED_DIVERSITY_INSUFFICIENT",
    "DATA02V2_TECHNICAL_INVALID",
    "DATA02V2_TEMPLATE_BANK_INSUFFICIENT",
)


def _normal_text(value: object) -> str:
    return " ".join(re.findall(r"[a-z0-9]+", str(value).lower()))


def semantic_goal_equivalent(first: Mapping[str, Any], second: Mapping[str, Any]) -> bool:
    """Conservative declared-semantics equality without consulting model output."""

    if str(first.get("class")) != str(second.get("class")):
        return False
    first_key = first.get("semantic_goal_key", first.get("expected_route", first.get("instruction", "")))
    second_key = second.get("semantic_goal_key", second.get("expected_route", second.get("instruction", "")))
    return bool(_normal_text(first_key)) and _normal_text(first_key) == _normal_text(second_key)


def template_near_duplicate(
    candidate: Mapping[str, Any],
    reference: Mapping[str, Any],
    *,
    translation_threshold_m: float = 0.4,
    yaw_threshold_rad: float = 0.15,
) -> dict[str, Any]:
    candidate_pose = np.asarray(candidate["initial_pose_se2"], dtype=np.float64)
    reference_pose = np.asarray(reference["initial_pose_se2"], dtype=np.float64)
    if candidate_pose.shape != (3,) or reference_pose.shape != (3,):
        raise ValueError("template poses must be finite SE(2) triples")
    if not np.all(np.isfinite(candidate_pose)) or not np.all(np.isfinite(reference_pose)):
        raise ValueError("template poses must be finite SE(2) triples")
    translation = float(np.linalg.norm(candidate_pose[:2] - reference_pose[:2]))
    yaw = abs(float(wrap_angle(candidate_pose[2] - reference_pose[2])))
    semantics = semantic_goal_equivalent(candidate, reference)
    return {
        "candidate_id": str(candidate["id"]),
        "reference_id": str(reference["id"]),
        "translation_m": translation,
        "wrapped_yaw_difference_rad": yaw,
        "semantically_equivalent": semantics,
        "flagged": translation < translation_threshold_m and yaw < yaw_threshold_rad and semantics,
    }


def validate_independent_template_bank(
    candidates: Sequence[Mapping[str, Any]],
    v1_templates: Sequence[Mapping[str, Any]],
    *,
    required_per_family: int = 4,
    translation_threshold_m: float = 0.4,
    yaw_threshold_rad: float = 0.15,
) -> dict[str, Any]:
    identifiers = [str(item["id"]) for item in candidates]
    old_identifiers = {str(item["id"]) for item in v1_templates}
    family_counts = Counter(str(item.get("class")) for item in candidates)
    comparisons = [
        template_near_duplicate(
            candidate,
            reference,
            translation_threshold_m=translation_threshold_m,
            yaw_threshold_rad=yaw_threshold_rad,
        )
        for candidate in candidates
        for reference in v1_templates
    ]
    flagged = [item for item in comparisons if item["flagged"]]
    checks = {
        "template_count_24": len(candidates) == 24,
        "unique_new_ids": len(set(identifiers)) == len(identifiers),
        "no_v1_id_reuse": not bool(set(identifiers) & old_identifiers),
        "balanced_semantic_families": all(family_counts[name] == required_per_family for name in SEMANTIC_FAMILIES),
        "no_flagged_v1_near_duplicate": not flagged,
    }
    return {
        "valid": all(checks.values()),
        "checks": checks,
        "template_count": len(candidates),
        "family_counts": dict(sorted(family_counts.items())),
        "near_duplicate_rule": {
            "translation_strictly_below_m": float(translation_threshold_m),
            "wrapped_yaw_strictly_below_rad": float(yaw_threshold_rad),
            "semantic_equivalence_required": True,
        },
        "flagged_v1_near_duplicates": flagged,
        "nearest_v1_by_candidate": [
            min(
                (item for item in comparisons if item["candidate_id"] == identifier),
                key=lambda item: (item["translation_m"], item["wrapped_yaw_difference_rad"], item["reference_id"]),
            )
            for identifier in identifiers
        ],
    }


def timing_invalid_direction(rtf: float, bounds: Sequence[float]) -> str:
    value = float(rtf)
    lower, upper = map(float, bounds)
    if not math.isfinite(value) or not lower < upper:
        raise ValueError("RTF and bounds must be finite and ordered")
    if value < lower:
        return "BELOW_LOWER_RTF_BOUND"
    if value > upper:
        return "ABOVE_UPPER_RTF_BOUND"
    return "WITHIN_RTF_BOUNDS"


def timing_group_summary(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    fields = (
        "rtf",
        "host_latency_s",
        "model_reported_s",
        "simulation_ready_latency_s",
        "effective_latency_s",
        "server_prediction_s",
        "main_loop_detection_delay_s",
        "inferred_frames_queued_during_inference",
    )
    return {name: percentile_summary(float(item[name]) for item in records) for name in fields}


def combined_episode_pair_components(records: Sequence[Mapping[str, Any]]) -> list[list[Mapping[str, Any]]]:
    eligible = [item for item in records if item.get("status") == "ELIGIBLE_MOVING"]
    by_episode: dict[str, list[int]] = defaultdict(list)
    by_pair: dict[str, list[int]] = defaultdict(list)
    for index, item in enumerate(eligible):
        by_episode[str(item["corpus_episode_id"])].append(index)
        by_pair[str(item["ordered_raw_pair_sha256"])].append(index)
    unseen = set(range(len(eligible)))
    components: list[list[Mapping[str, Any]]] = []
    while unseen:
        seed = min(unseen, key=lambda index: str(eligible[index]["corpus_transition_id"]))
        queue = deque([seed])
        members: set[int] = set()
        while queue:
            current = queue.popleft()
            if current in members:
                continue
            members.add(current)
            item = eligible[current]
            neighbours = by_episode[str(item["corpus_episode_id"])] + by_pair[str(item["ordered_raw_pair_sha256"])]
            queue.extend(index for index in neighbours if index not in members)
        unseen -= members
        components.append(sorted((eligible[index] for index in members), key=lambda item: str(item["corpus_transition_id"])))
    return sorted(components, key=lambda group: str(group[0]["corpus_transition_id"]))


def _coverage(group: Iterable[Mapping[str, Any]], field: str) -> set[str]:
    return {str(item[field]) for item in group}


def combined_isolated_split(
    records: Sequence[Mapping[str, Any]],
    *,
    heldout_fraction: float,
    required_families: Sequence[str],
    required_geometry: Sequence[str],
    required_difficulty: Sequence[str],
) -> dict[str, Any]:
    """Deterministically assign whole episode/raw-pair components."""

    if not 0.0 < heldout_fraction < 1.0:
        raise ValueError("heldout_fraction must be between zero and one")
    components = combined_episode_pair_components(records)
    eligible = [item for group in components for item in group]
    if not eligible:
        raise ValueError("combined split requires eligible transitions")
    target = round(len(eligible) * heldout_fraction)
    required = {
        "semantic_family": set(map(str, required_families)),
        "fresh_geometry_bin": set(map(str, required_geometry)),
        "difficulty_bin": set(map(str, required_difficulty)),
    }

    def score(selected: frozenset[int]) -> tuple[Any, ...]:
        held = [item for index in selected for item in components[index]]
        dev = [item for index, group in enumerate(components) if index not in selected for item in group]
        missing = []
        for field in ("semantic_family", "fresh_geometry_bin", "difficulty_bin"):
            missing.append(len(required[field] - _coverage(held, field)) + len(required[field] - _coverage(dev, field)))
        signature = tuple(sorted(selected))
        return (*missing, abs(len(held) - target), abs(len(selected) - max(1, round(len(components) * heldout_fraction))), signature)

    orders = [
        list(range(len(components))),
        sorted(range(len(components)), key=lambda index: (-len(components[index]), str(components[index][0]["corpus_transition_id"]))),
        sorted(range(len(components)), key=lambda index: (len(components[index]), str(components[index][0]["corpus_transition_id"]))),
    ]
    starts: set[frozenset[int]] = {frozenset()}
    for order in orders:
        for offset in range(min(len(order), 12)):
            rotated = order[offset:] + order[:offset]
            chosen: set[int] = set()
            while len([item for index in chosen for item in components[index]]) < target:
                candidate = min((index for index in rotated if index not in chosen), key=lambda index: score(frozenset(chosen | {index})))
                chosen.add(candidate)
            starts.add(frozenset(chosen))

    def improve(initial: frozenset[int]) -> frozenset[int]:
        current = initial
        while True:
            candidates = {current}
            for index in range(len(components)):
                candidates.add(frozenset(set(current) ^ {index}))
            for outgoing in current:
                for incoming in range(len(components)):
                    if incoming not in current:
                        candidates.add(frozenset((set(current) - {outgoing}) | {incoming}))
            best = min(candidates, key=score)
            if score(best) >= score(current):
                return current
            current = best

    selected = min((improve(start) for start in starts), key=score)
    heldout = [item for index in sorted(selected) for item in components[index]]
    development = [item for index, group in enumerate(components) if index not in selected for item in group]
    dev_episodes = _coverage(development, "corpus_episode_id")
    held_episodes = _coverage(heldout, "corpus_episode_id")
    dev_pairs = _coverage(development, "ordered_raw_pair_sha256")
    held_pairs = _coverage(heldout, "ordered_raw_pair_sha256")
    component_summaries = []
    for index, group in enumerate(components):
        component_summaries.append({
            "component_id": f"component_{index:04d}",
            "split": "heldout" if index in selected else "development",
            "transition_count": len(group),
            "episode_ids": sorted(_coverage(group, "corpus_episode_id")),
            "ordered_raw_pair_sha256": sorted(_coverage(group, "ordered_raw_pair_sha256")),
            "semantic_families": sorted(_coverage(group, "semantic_family")),
            "geometry_bins": sorted(_coverage(group, "fresh_geometry_bin")),
            "difficulty_bins": sorted(_coverage(group, "difficulty_bin")),
        })
    return {
        "assignment_method": "deterministic whole-component local search; size then semantic/geometry/difficulty coverage; no component split",
        "target_heldout_fraction": float(heldout_fraction),
        "actual_heldout_fraction": len(heldout) / len(eligible),
        "development_transition_ids": sorted(str(item["corpus_transition_id"]) for item in development),
        "heldout_transition_ids": sorted(str(item["corpus_transition_id"]) for item in heldout),
        "development_episode_ids": sorted(dev_episodes),
        "heldout_episode_ids": sorted(held_episodes),
        "component_count": len(components),
        "largest_component_transition_count": max(map(len, components), default=0),
        "components": component_summaries,
        "no_episode_leakage": not bool(dev_episodes & held_episodes),
        "no_ordered_raw_pair_leakage": not bool(dev_pairs & held_pairs),
        "semantic_family_coverage_both_splits": all(
            required[field] <= _coverage(group, field)
            for field, group in (("semantic_family", development), ("semantic_family", heldout))
        ),
        "geometry_coverage_both_splits": all(
            required["fresh_geometry_bin"] <= _coverage(group, "fresh_geometry_bin")
            for group in (development, heldout)
        ),
        "difficulty_coverage_both_splits": all(
            required["difficulty_bin"] <= _coverage(group, "difficulty_bin")
            for group in (development, heldout)
        ),
        "development_semantic_families": sorted(_coverage(development, "semantic_family")),
        "heldout_semantic_families": sorted(_coverage(heldout, "semantic_family")),
        "development_geometry_bins": sorted(_coverage(development, "fresh_geometry_bin")),
        "heldout_geometry_bins": sorted(_coverage(heldout, "fresh_geometry_bin")),
        "development_difficulty_bins": sorted(_coverage(development, "difficulty_bin")),
        "heldout_difficulty_bins": sorted(_coverage(heldout, "difficulty_bin")),
    }


def combined_readiness_decision(
    records: Sequence[Mapping[str, Any]],
    split: Mapping[str, Any],
    criteria: Mapping[str, Any],
    *,
    artifacts_valid: bool,
    template_bank_valid: bool,
) -> dict[str, Any]:
    if not template_bank_valid:
        return {"status": "DATA02V2_TEMPLATE_BANK_INSUFFICIENT", "checks": {"template_bank": False}}
    if not artifacts_valid:
        return {"status": "DATA02V2_TECHNICAL_INVALID", "checks": {"H_artifact_validity": False}}
    eligible = [item for item in records if item.get("status") == "ELIGIBLE_MOVING"]
    pairs = Counter(str(item["ordered_raw_pair_sha256"]) for item in eligible)
    geometry = Counter(str(item["fresh_geometry_bin"]) for item in eligible)
    difficulty = Counter(str(item["difficulty_bin"]) for item in eligible)
    largest_fraction = max(pairs.values(), default=0) / len(eligible) if eligible else 1.0
    checks = {
        "A_minimum_eligible_moving": len(eligible) >= int(criteria["minimum_eligible_moving"]),
        "B_minimum_unique_ordered_raw_pairs": len(pairs) >= int(criteria["minimum_unique_ordered_raw_pairs"]),
        "C_largest_pair_fraction": largest_fraction <= float(criteria["maximum_largest_pair_fraction"]),
        "D_unique_old_and_fresh": len(_coverage(eligible, "old_raw_sha256")) >= int(criteria["minimum_unique_old_chunks"]) and len(_coverage(eligible, "fresh_raw_sha256")) >= int(criteria["minimum_unique_fresh_chunks"]),
        "E_geometry_coverage": geometry["STRAIGHT_LIKE"] >= int(criteria["minimum_straight_like"]) and geometry["POSITIVE_TURNING"] >= int(criteria["minimum_positive_turning"]) and geometry["NEGATIVE_TURNING"] >= int(criteria["minimum_negative_turning"]),
        "F_difficulty_coverage": difficulty["BENIGN"] >= int(criteria["minimum_benign"]) and difficulty["CHALLENGING"] >= int(criteria["minimum_challenging"]),
        "G_isolated_split": bool(split["no_episode_leakage"]) and bool(split["no_ordered_raw_pair_leakage"]) and bool(split["semantic_family_coverage_both_splits"]) and len(split["heldout_transition_ids"]) >= int(criteria["minimum_heldout_eligible"]),
        "H_artifact_validity": True,
    }
    return {
        "status": "DATA02_COMBINED_READY_FOR_EXP02D" if all(checks.values()) else "DATA02_COMBINED_DIVERSITY_INSUFFICIENT",
        "checks": checks,
        "counts": {
            "eligible_moving": len(eligible),
            "unique_old_raw": len(_coverage(eligible, "old_raw_sha256")),
            "unique_fresh_raw": len(_coverage(eligible, "fresh_raw_sha256")),
            "unique_ordered_raw_pairs": len(pairs),
            "largest_pair_count": max(pairs.values(), default=0),
            "largest_pair_fraction": largest_fraction,
            "geometry": dict(sorted(geometry.items())),
            "difficulty": dict(sorted(difficulty.items())),
            "heldout_eligible": len(split["heldout_transition_ids"]),
        },
    }


def cross_cohort_duplicates(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    eligible = [item for item in records if item.get("status") == "ELIGIBLE_MOVING"]
    pair_rows: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for item in eligible:
        pair_rows[str(item["ordered_raw_pair_sha256"])].append(item)
    groups = []
    for pair, items in pair_rows.items():
        cohorts = sorted(_coverage(items, "cohort_id"))
        groups.append({
            "ordered_raw_pair_sha256": pair,
            "count": len(items),
            "cohorts": cohorts,
            "shared_across_cohorts": len(cohorts) > 1,
            "transition_ids": sorted(str(item["corpus_transition_id"]) for item in items),
        })
    groups.sort(key=lambda item: (-item["count"], item["ordered_raw_pair_sha256"]))
    return {
        "unique_ordered_raw_pairs": len(groups),
        "shared_pair_identity_count": sum(item["shared_across_cohorts"] for item in groups),
        "largest_pair_count": groups[0]["count"] if groups else 0,
        "largest_pair_fraction": groups[0]["count"] / len(eligible) if groups else None,
        "top_20_pair_frequencies": groups[:20],
        "groups": groups,
    }


def deterministic_combined_representatives(records: Sequence[Mapping[str, Any]], limit: int = 40) -> list[dict[str, Any]]:
    eligible = [item for item in records if item.get("status") == "ELIGIBLE_MOVING"]
    if not eligible:
        return []
    frequency = Counter(str(item["ordered_raw_pair_sha256"]) for item in eligible)
    cohorts_by_pair: dict[str, set[str]] = defaultdict(set)
    for item in eligible:
        cohorts_by_pair[str(item["ordered_raw_pair_sha256"])].add(str(item["cohort_id"]))
    selected: dict[str, set[str]] = defaultdict(set)

    def choose(label: str, candidates: Iterable[Mapping[str, Any]], key=None) -> None:
        values = list(candidates)
        if not values:
            return
        item = min(values, key=key or (lambda row: str(row["corpus_transition_id"])))
        selected[str(item["corpus_transition_id"])].add(label)

    largest = max(frequency.values())
    choose("largest_duplicate_group", (item for item in eligible if frequency[str(item["ordered_raw_pair_sha256"])] == largest))
    choose("rare_pair_group", (item for item in eligible if frequency[str(item["ordered_raw_pair_sha256"])] == 1))
    for value in ("STRAIGHT_LIKE", "POSITIVE_TURNING", "NEGATIVE_TURNING"):
        choose(value.lower(), (item for item in eligible if item["fresh_geometry_bin"] == value))
    for value in ("BENIGN", "CHALLENGING"):
        choose(value.lower(), (item for item in eligible if item["difficulty_bin"] == value))
    choose("low_latency", eligible, key=lambda item: (float(item["host_latency_s"]), str(item["corpus_transition_id"])))
    choose("high_latency", eligible, key=lambda item: (-float(item["host_latency_s"]), str(item["corpus_transition_id"])))
    for cohort, other in (("v1", "v2"), ("v2", "v1")):
        choose(f"{cohort}_only_pair", (
            item for item in eligible
            if item["cohort_id"] == cohort and other not in cohorts_by_pair[str(item["ordered_raw_pair_sha256"])]
        ))
    choose("shared_v1_v2_pair", (
        item for item in eligible if len(cohorts_by_pair[str(item["ordered_raw_pair_sha256"])] ) > 1
    ))
    for item in sorted(eligible, key=lambda row: str(row["corpus_transition_id"])):
        if len(selected) >= min(36, len(eligible), limit):
            break
        selected[str(item["corpus_transition_id"])].add("deterministic_coverage_fill")
    return [
        {"corpus_transition_id": identifier, "selection_reasons": sorted(reasons)}
        for identifier, reasons in sorted(selected.items())[:limit]
    ]
