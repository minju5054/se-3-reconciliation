from __future__ import annotations

import csv
import hashlib
from pathlib import Path

import numpy as np
import pytest
import yaml

from reconciliation.frozen_transition_bank import (
    OVERVIEW_PLOTS,
    PAIR_INDEX_FIELDS,
    PAIR_PLOTS,
    _copy_file_exclusive,
    add_deterministic_ranks,
    boundary_canonicalize,
    canonical_json_sha256,
    classify_attempt_for_bank,
    create_bank_root,
    deterministic_grouped_split,
    duplicate_pair_identity,
    pair_identity,
    plot_bank,
    select_visualization_representatives,
    transition_descriptor,
)
from reconciliation.lightnav_execution_envelope import geometry_descriptor
from reconciliation.online_switch import sha256_file
from reconciliation.se2 import relative_pose


def straight(offset: float = 0.0) -> np.ndarray:
    return np.array(
        [[offset, 0.0, 0.0], [offset + 0.5, 0.0, 0.0], [offset + 1.0, 0.0, 0.0]],
        dtype=np.float64,
    )


@pytest.mark.parametrize(
    ("classification", "timing_valid", "stop", "expected"),
    [
        ("VALID_MOVING", True, False, (True, "ELIGIBLE_VALID_MOVING")),
        ("MODEL_STOP_OUTPUT", True, True, (False, "EXCLUDED_MODEL_STOP_OUTPUT")),
        ("OLD_EXHAUSTED", True, True, (False, "EXCLUDED_OLD_EXHAUSTED")),
        ("TIMING_INVALID", False, False, (False, "EXCLUDED_TIMING_INVALID")),
    ],
)
def test_source_classification(
    classification: str, timing_valid: bool, stop: bool, expected: tuple[bool, str]
) -> None:
    assert classify_attempt_for_bank(
        {"classification": classification, "timing_valid": timing_valid},
        fresh_is_stop=stop,
    ) == expected


def test_pair_identity_is_deterministic_and_context_sensitive() -> None:
    first = {"source": "trial", "B": [1.0, 2.0, 0.3], "hash": "a" * 64}
    reordered = {"hash": "a" * 64, "B": [1.0, 2.0, 0.3], "source": "trial"}
    assert pair_identity(first) == pair_identity(reordered)
    assert pair_identity(first) != pair_identity({**first, "B": [1.1, 2.0, 0.3]})
    assert canonical_json_sha256(first) == canonical_json_sha256(reordered)


def test_duplicate_raw_and_world_identity_are_explicit() -> None:
    first = duplicate_pair_identity("a" * 64, "b" * 64, kind="raw")
    second = duplicate_pair_identity("a" * 64, "b" * 64, kind="raw")
    assert first == second
    assert first != duplicate_pair_identity("a" * 64, "c" * 64, kind="raw")
    assert first != duplicate_pair_identity("a" * 64, "b" * 64, kind="world")


def test_byte_copy_preserves_source_hash_mtime_and_rejects_overwrite(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.npy"
    np.save(source, np.array([[1.0, 2.0, 3.0]], dtype=np.float32))
    before_hash = sha256_file(source)
    before_mtime = source.stat().st_mtime_ns
    destination = tmp_path / "copy.npy"
    _copy_file_exclusive(source, destination)
    assert sha256_file(destination) == before_hash
    assert sha256_file(source) == before_hash
    assert source.stat().st_mtime_ns == before_mtime
    with pytest.raises(FileExistsError):
        _copy_file_exclusive(source, destination)


def test_boundary_canonical_transform_maps_B_and_preserves_geometry() -> None:
    boundary = np.array([3.0, -2.0, 1.1])
    old = np.array([[2.5, -2.0, 0.8], [3.0, -2.0, 1.1], [3.2, -1.7, 1.3]])
    fresh = np.array([[3.1, -1.9, 1.2], [3.4, -1.5, 1.4]])
    previous = np.array([2.8, -2.1, 1.0])
    old_before = old.copy()
    old_c, fresh_c, previous_c, boundary_c = boundary_canonicalize(
        boundary, old, fresh, previous, boundary
    )
    assert boundary_c == pytest.approx(np.zeros(3), abs=1e-12)
    assert old_c == pytest.approx(relative_pose(boundary, old))
    assert fresh_c == pytest.approx(relative_pose(boundary, fresh))
    assert previous_c == pytest.approx(relative_pose(boundary, previous))
    assert np.array_equal(old, old_before)
    assert np.linalg.norm(old_c[-1, :2] - old_c[0, :2]) == pytest.approx(
        np.linalg.norm(old[-1, :2] - old[0, :2])
    )


def test_transition_descriptor_straight_and_wrapped_yaw() -> None:
    descriptor = transition_descriptor(
        previous_pose=[-0.1, 0.0, np.pi - 0.02],
        boundary_pose=[0.0, 0.0, -np.pi + 0.02],
        old_world=straight(-1.0),
        fresh_world=np.array([[0.2, 0.0, np.pi - 0.01], [1.0, 0.0, 0.0]]),
        direction_epsilon_m=1e-3,
    )
    assert descriptor["boundary_to_fresh0_distance_m"] == pytest.approx(0.2)
    assert descriptor["previous_to_boundary_world_direction_rad"] == pytest.approx(0.0)
    assert descriptor[
        "incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"
    ] == pytest.approx(0.0)
    assert descriptor["boundary_to_fresh0_wrapped_yaw_rad"] == pytest.approx(-0.03)
    assert descriptor["intrinsic_waypoint_time_base"] is False


def test_transition_descriptor_marks_near_zero_direction_undefined() -> None:
    descriptor = transition_descriptor(
        previous_pose=[0.0, 0.0, 0.0],
        boundary_pose=[0.0001, 0.0, 0.0],
        old_world=straight(),
        fresh_world=np.array([[0.0002, 0.0, 0.0], [1.0, 0.0, 0.0]]),
        direction_epsilon_m=1e-3,
    )
    assert descriptor["previous_to_boundary_world_direction_defined"] is False
    assert descriptor["boundary_to_fresh0_world_direction_defined"] is False
    assert descriptor[
        "incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad"
    ] is None


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_descriptor_rejects_nonfinite_values(bad: float) -> None:
    path = straight()
    path[1, 0] = bad
    with pytest.raises(ValueError, match="finite"):
        geometry_descriptor(path)
    with pytest.raises(ValueError, match="finite"):
        transition_descriptor(
            previous_pose=[0.0, 0.0, 0.0],
            boundary_pose=[0.1, 0.0, 0.0],
            old_world=straight(),
            fresh_world=path,
        )


def split_rows() -> list[dict]:
    rows = []
    index = 0
    for geometry in ("G0", "G1", "G2"):
        for latency in ("L0", "L1"):
            for repeat in range(3):
                group = f"raw_{geometry}_{latency}_{repeat // 2}"
                rows.append(
                    {
                        "pair_id": f"pair_{index:012x}",
                        "duplicate_raw_pair_group": group,
                        "geometry_group": geometry,
                        "latency_group": latency,
                    }
                )
                index += 1
    return rows


def test_grouped_split_is_deterministic_disjoint_complete_and_no_leakage() -> None:
    rows = split_rows()
    first = deterministic_grouped_split(rows, seed="fixed")
    second = deterministic_grouped_split(list(reversed(rows)), seed="fixed")
    assert first == second
    development = set(first["development_pair_ids"])
    heldout = set(first["heldout_pair_ids"])
    assert development.isdisjoint(heldout)
    assert development | heldout == {row["pair_id"] for row in rows}
    for group in {row["duplicate_raw_pair_group"] for row in rows}:
        splits = {
            first["assignment"][row["pair_id"]]
            for row in rows
            if row["duplicate_raw_pair_group"] == group
        }
        assert len(splits) == 1
    assert first["leakage_check"]["passed"] is True
    assert all(
        value["development"] > 0 and value["heldout"] > 0
        for value in first["stratum_counts"].values()
    )


def rank_rows() -> list[dict]:
    return [
        {
            "pair_id": f"pair_{index:012x}",
            "historical_raw_switch": {
                "delta_v_raw_abs_mps": float(index + 1),
                "delta_omega_raw_abs_rps": float(6 - index),
            },
            "transition": {
                "incoming_vs_boundary_to_fresh0_direction_disagreement_abs_rad": float(index) / 10,
                "yaw_increment_disagreement_abs_rad": float(index) / 20,
            },
            "timing": {
                "effective_latency_sim_s": float(index + 1),
                "robot_translation_observation_to_switch_m": float(index + 1) / 10,
            },
        }
        for index in range(6)
    ]


def test_ranks_and_representatives_are_deterministic_raw_only() -> None:
    ranked = add_deterministic_ranks(rank_rows())
    assert sorted(row["rank_abs_delta_v"] for row in ranked) == list(range(1, 7))
    assert {row["quantile_abs_delta_v"] for row in ranked} == {"Q1", "Q2", "Q3", "Q4"}
    first = select_visualization_representatives(ranked)
    second = select_visualization_representatives(list(reversed(ranked)))
    assert first == second
    assert first["selection_uses_graph_or_reconciliation_results"] is False


def _plot_fixture(root: Path) -> None:
    config = {"plotting": {"dpi": 75}}
    (root / "config_snapshot.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
    fields = list(PAIR_INDEX_FIELDS)
    rows = []
    for index, (geometry, latency, split) in enumerate(
        [
            ("G0_straight", "L0_natural", "development"),
            ("G0_straight", "L1_added_050", "heldout"),
            ("G1_turn", "L0_natural", "development"),
            ("G1_turn", "L1_added_050", "heldout"),
            ("G2_route_change", "L0_natural", "development"),
            ("G2_route_change", "L1_added_050", "heldout"),
        ]
    ):
        pair_id_value = f"pair_{index:012x}"
        pair_root = root / "pairs" / pair_id_value
        (pair_root / "derived").mkdir(parents=True)
        old = straight(float(index))
        fresh = straight(float(index) + 0.2)
        np.save(pair_root / "derived/old_world.npy", old)
        np.save(pair_root / "derived/fresh_world.npy", fresh)
        context = {
            "execution_context": {
                "previous_pose_P_world_se2": [float(index) - 0.1, 0.0, 0.0],
                "boundary_pose_B_world_se2": [float(index), 0.0, 0.0],
            },
            "old": {"observation_pose_world_se2": [float(index), 0.0, 0.0]},
            "fresh": {"observation_pose_world_se2": [float(index) + 0.1, 0.0, 0.0]},
        }
        import json

        (pair_root / "context.json").write_text(json.dumps(context), encoding="utf-8")
        row = {field: "0" for field in fields}
        row.update(
            {
                "pair_id": pair_id_value,
                "display_index": f"{index:03d}",
                "geometry_group": geometry,
                "latency_group": latency,
                "split": split,
                "effective_latency_s": "0.5",
                "raw_delta_v_abs_mps": "0.1",
                "raw_delta_omega_abs_rps": "0.2",
            }
        )
        rows.append(row)
    with (root / "dataset_index.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_plotting_creates_all_nonempty_pair_and_overview_pngs(tmp_path: Path) -> None:
    root = tmp_path / "bank"
    root.mkdir()
    _plot_fixture(root)
    result = plot_bank(root)
    assert result["pair_count"] == 6
    assert result["per_pair_plot_count"] == 6 * len(PAIR_PLOTS)
    assert result["overview_plot_count"] == len(OVERVIEW_PLOTS)
    assert len(result["plots_sha256"]) == 6 * len(PAIR_PLOTS) + len(OVERVIEW_PLOTS)
    for relative, digest in result["plots_sha256"].items():
        path = root / relative
        assert path.stat().st_size > 1024
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    with pytest.raises(FileExistsError):
        plot_bank(root)


def test_existing_bank_root_cannot_be_overwritten(tmp_path: Path) -> None:
    root = tmp_path / "lightnav_exp01b_v1"
    create_bank_root(root)
    with pytest.raises(FileExistsError):
        create_bank_root(root)
