"""Read frozen arbitrary EXP02D transitions and measure explicit turning criteria."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from reconciliation.exp02d_gui import _verify_result_file
from reconciliation.data02_active_old import load_active_old_interval
from reconciliation.online_switch import sha256_file
from reconciliation.data02_online_successive import strict_json
from reconciliation.trajectory import validate_se2_trajectory

METHODS = ("M0_RAW", "M1_HISTORICAL_M4", "M3_LOOKAHEAD")


def path_turning(trajectory, minimum_segment_m=.02):
    """World XY metres, +Z CCW yaw radians; unwrap only for measuring excursion."""
    path = validate_se2_trajectory(trajectory)
    delta = np.diff(path[:, :2], axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    directions = delta[lengths >= minimum_segment_m]
    tangent = np.unwrap(np.arctan2(directions[:, 1], directions[:, 0])) if len(directions) else np.zeros(1)
    return {"arc_m": float(lengths.sum()),
            "yaw_excursion_deg": float(np.rad2deg(np.ptp(np.unwrap(path[:, 2])))),
            "tangent_excursion_deg": float(np.rad2deg(np.ptp(tangent)))}


def reference_is_turning(features, config):
    return (features["arc_m"] >= config["minimum_reference_arc_m"] and
            features["yaw_excursion_deg"] >= config["minimum_reference_yaw_excursion_deg"] and
            features["tangent_excursion_deg"] >= config["minimum_reference_tangent_excursion_deg"])


def actual_is_turning(features, config):
    return (features["arc_m"] >= config["minimum_actual_arc_m"] and
            features["yaw_excursion_deg"] >= config["minimum_actual_yaw_excursion_deg"])


@dataclass
class SavedTransition:
    case: str
    corpus_transition_id: str
    input_reference: dict
    active_old: object
    method_candidates: dict
    source_sha256: dict
    result_sha256: dict


class FrozenArchive:
    def __init__(self, run):
        self.run = Path(run).resolve()
        self.manifest = strict_json(self.run / "result_manifest.json")
        if self.manifest.get("schema") != "EXP02D_ResultManifest_v1":
            raise ValueError("requires frozen EXP02D result manifest")
        self.base_hashes = {}
        for name in ("metadata.json", "corpus_manifest.json", "config_snapshot.yaml"):
            _, digest = self.verify(name)
            self.base_hashes[name] = digest
        self.corpus = strict_json(self.run / "corpus_manifest.json")["transition_artifacts"]
        metadata = strict_json(self.run / "metadata.json")
        if metadata["technical_status"] != "EXP02D_RUN_COMPLETE" or metadata["candidate_methods_physically_executed"]:
            raise ValueError("source must be completed candidate-only primary")

    def verify(self, relative):
        return _verify_result_file(self.run, self.manifest, relative, "frozen search input")

    def candidates(self, identifier):
        record = self.corpus[identifier]
        paths, hashes = {}, dict(self.base_hashes)
        for method in METHODS:
            relative = record["artifact_relative_path"] + f"/{method}/candidate.npy"
            path, digest = self.verify(relative)
            if digest != record["methods"][method]["candidate_sha256"]:
                raise ValueError("candidate hash disagrees with corpus")
            paths[method] = validate_se2_trajectory(np.load(path, allow_pickle=False))
            hashes[relative] = digest
        return paths, hashes

    def load(self, identifier, case=None):
        record = self.corpus[identifier]
        paths, hashes = self.candidates(identifier)
        relative = record["artifact_relative_path"] + "/input_reference.json"
        path, digest = self.verify(relative)
        if digest != record["input_reference_sha256"]:
            raise ValueError("input reference hash disagrees with corpus")
        hashes[relative] = digest
        info = strict_json(path)
        if info["corpus_transition_id"] != identifier:
            raise ValueError("transition identity mismatch")
        active = load_active_old_interval(info["source_run"], info["episode_id"], info["transition_index"])
        if active.source_sha256["transition.json"] != info["source_transition_sha256"]:
            raise ValueError("source transition changed")
        for name in ("derived/old_world.npy", "derived/fresh_world.npy", "actual.npy", "telemetry.csv"):
            key = "transition_telemetry.csv" if name == "telemetry.csv" else name
            if active.source_sha256[key] != info["source_artifact_sha256"][name]:
                raise ValueError("source telemetry or path changed")
        relative = record["artifact_relative_path"] + "/oracle_indices.json"
        path, digest = self.verify(relative)
        if digest != record["oracle_indices_sha256"]:
            raise ValueError("oracle hash mismatch")
        hashes[relative] = digest
        k = int(strict_json(path)["k_fresh"])
        if not np.array_equal(paths["M0_RAW"], active.fresh_world[k:]):
            raise ValueError("RAW differs from saved selected FRESH")
        if not np.array_equal(np.asarray(info["B_world_se2"]), active.boundary_pose_world_se2):
            raise ValueError("saved initial pose mismatch")
        source_hashes = dict(active.source_sha256)
        config_hash = sha256_file(active.run / "config_snapshot.yaml")
        if config_hash != strict_json(active.run / "collection_manifest.json")["config_snapshot_sha256"]:
            raise ValueError("DATA02 source configuration changed")
        source_hashes["config_snapshot.yaml"] = config_hash
        return SavedTransition(case or identifier.replace(":", "__"), identifier, info, active, paths, source_hashes, hashes)
