#!/usr/bin/env python3
"""Saved-record JOIN-SOURCE-02 audit; never infer, solve MPC, or render.

Scientific source failure is allowed. Corrupt hashes, missing ledger coverage,
fabricated unavailable data, changed geometry, or figure/record disagreement
fail artifact validation. This post-primary checker is separate from frozen
acquisition/selection code. It does not repair or overwrite historical output.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"src"), str(ROOT/"scripts"), str(ROOT/"scripts/lightnav")]
from reconciliation.join_source02 import validate_development_order
from reconciliation.join_source02_acquisition import history_suffix, placement_on_future


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            result.update(chunk)
    return result.hexdigest()


def resolve(path, base=ROOT):
    path = Path(path)
    return path if path.is_absolute() else base/path


def _load_runner():
    spec = importlib.util.spec_from_file_location("_join_source02_saved_validation_runner", ROOT/"scripts/run_join_source02.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Audit:
    def __init__(self):
        self.checks = {}
        self.details = {}
        self.hash_cache = {}

    def check(self, name, value, detail=None):
        if name in self.checks:
            raise ValueError(f"duplicate validation key: {name}")
        self.checks[name] = bool(value)
        if detail is not None:
            self.details[name] = detail
        return self.checks[name]

    def sha(self, path):
        path = Path(path)
        if path not in self.hash_cache:
            self.hash_cache[path] = digest(path)
        return self.hash_cache[path]

    def hashes(self, prefix, values, base=ROOT):
        for name, expected in values.items():
            path = resolve(name, base)
            self.check(prefix+":"+name, path.is_file() and self.sha(path) == expected)

    def attempt(self, name, function):
        try:
            return function()
        except Exception as exc:
            self.check(name, False, f"{type(exc).__name__}: {exc}")
            return None


def validate_capture(path, audit, *, root_prim="/World/JOINSource02Obstacle"):
    """Recount saved raw instance mask; invisible is not a technical failure."""
    path = Path(path)
    record = read(path)
    folder = path.parent
    jpeg, mask_file = folder/record["rgb_file"], folder/record["mask_file"]
    key = "capture:"+str(path)
    audit.check(key+":jpeg_hash", audit.sha(jpeg) == record["rgb_jpeg_sha256"])
    audit.check(key+":mask_hash", audit.sha(mask_file) == record["mask_sha256"])
    with np.load(mask_file, allow_pickle=False) as archive:
        mask = archive["mask"].copy()
    with Image.open(jpeg) as im:
        image_size = list(im.size)
    resolution = record["camera"]["actual_intrinsics"]["resolution_width_height"]
    audit.check(key+":same_pixel_resolution", mask.ndim == 2 and mask.size > 0 and
                [mask.shape[1], mask.shape[0]] == image_size == resolution)
    labels = record["instance"]["idToLabels"]
    actual_ids = sorted(int(i) for i, prim in labels.items()
                        if prim == root_prim or prim.startswith(root_prim+"/"))
    saved_ids = sorted(record["instance"]["matched_instance_ids"])
    audit.check(key+":instance_identity", actual_ids == saved_ids)
    selected = np.isin(mask, actual_ids)
    count = int(selected.sum())
    where = np.argwhere(selected)
    # Frozen Isaac instance_data stores inclusive pixel maxima (not crop-style
    # half-open extents). Match that declared producer convention exactly.
    bbox = None if not len(where) else [int(where[:, 1].min()), int(where[:, 0].min()),
                                      int(where[:, 1].max()), int(where[:, 0].max())]
    audit.check(key+":pixel_count", count == record["instance"]["visible_pixels"])
    audit.check(key+":image_fraction", float(selected.mean()) == record["instance"]["image_fraction"])
    audit.check(key+":bbox", bbox == record["instance"]["bbox_xyxy"])
    audit.check(key+":render_state_contract", record["stable_camera_pose_and_simulation_time"] is True
                and record["same_render_product"] is True and record["model_input_annotations"] is False)
    return dict(visible_pixels=count, bbox_convention="inclusive pixel-index maxima",
                pose=record["agent_pose_world"], camera=record["camera"],
                simulation_time_s=record["simulation_time_s"], actual_instance_ids=actual_ids)


def validate_placement(path, runner, audit):
    path = Path(path)
    placement = read(path)
    if placement.get("status") != "AVAILABLE":
        audit.check("placement_unavailable_reason:"+str(path), bool(placement.get("reason")))
        return placement
    # Reprojects actual saved triangles and checks literal WKB/hash equality.
    off, on = runner.load_environments(placement)
    audit.check("placement_geometry_reconstructed:"+str(path), off is not on)
    audit.check("placement_not_bbox_oracle:"+str(path),
                placement["projection"]["metadata"]["bbox_is_oracle"] is False)
    captures = {branch: validate_capture(path.parent/f"{branch}.json", audit) for branch in ("A", "B")}
    actual_visibility = captures["A"]["visible_pixels"] == 0 and captures["B"]["visible_pixels"] >= 20
    audit.check("placement_visibility_flag:"+str(path),
                actual_visibility == placement["technical_checks"]["visibility"])
    audit.check("placement_same_camera_pose_time:"+str(path),
                all(captures["A"][k] == captures["B"][k] for k in ("pose", "camera", "simulation_time_s")))
    activation = placement["activation"]
    audit.check("placement_off_on_activation:"+str(path), len(activation) == 2 and
                [r["present"] for r in activation] == [False, True] and
                all(r["oracle_present"] == r["present"] and
                    (r["render"] != "invisible") == r["present"] for r in activation))
    return placement


def validate_bank(path, audit):
    """Prove the truncated history is an authentic prefix ending at OLD capture."""
    path = Path(path)
    bank = read(path)
    actual = [json.loads(line) for line in (path.parent/"capture.jsonl").read_text().splitlines()]
    frames = bank["frames"]
    key = "history_bank:"+str(path)
    audit.check(key+":count", len(frames) == bank["available_frames"] and len(actual) == bank["acquired_frames"])
    audit.check(key+":prefix", [f["frame_id"] for f in frames] == [f["frame_id"] for f in actual[:len(frames)]])
    audit.check(key+":unique", len({f["frame_id"] for f in frames}) == len(frames))
    for i, (frame, original) in enumerate(zip(frames, actual)):
        audit.check(key+f":capture_identity:{i}", all(frame[k] == original[k] for k in
                    ("sha256", "capture_monotonic_ns", "capture_sim_time_s", "pose_world", "rendered_state_id")))
        audit.check(key+f":jpeg:{i}", audit.sha(Path(frame["path"])) == frame["sha256"])
    if frames:
        context = bank["actual_old_context"]["context"]
        audit.check(key+":ends_at_actual_OLD_observation", frames[-1]["frame_id"] == context["frame_id"])
        audit.check(key+":chronological", all(b["capture_monotonic_ns"] > a["capture_monotonic_ns"]
                    and b["capture_sim_time_s"] > a["capture_sim_time_s"] for a, b in zip(frames[:-1], frames[1:])))
        reference = context["world_ref"]
        oldpath = path.parent/reference["path"]
        audit.check(key+":actual_OLD_world_hash", audit.sha(oldpath) == reference["sha256"])
        audit.check(key+":actual_OLD_world_array", np.array_equal(np.load(oldpath, allow_pickle=False), context["world"]))
    return bank


def validate_placement_rule(run, manifest, bank, protocol, audit):
    """Recompute frozen placement from actual OLD and separately captured prop."""
    placement = read(resolve(manifest["placement_path"]))
    prop_path = run/"technical_preflight/visibility/prop_geometry.json"
    triangles_path = prop_path.parent/"actual_prop_triangles.npz"
    prop = read(prop_path)
    with np.load(triangles_path, allow_pickle=False) as archive:
        actual = archive["triangles"].reshape((-1, 3))
    transform = np.asarray(prop["source_to_actual_world_matrix_column"])
    original = (np.c_[actual, np.ones(len(actual))] @ np.linalg.inv(transform).T)[:, :3]
    theta = prop["source_root_yaw_rad"]
    c, s = np.cos(theta), np.sin(theta)
    local = original @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    half = float(np.ptp(local, axis=0)[1]/2)
    scenario = protocol.get("immutable_scenario", protocol)
    rules = {r["id"]: r for r in scenario["placements"]}
    design = protocol["placement_design"]
    fresh = bank["actual_old_context"]["context"]
    out = placement_on_future(bank["frames"][-1]["pose_world"], fresh["world"],
        fraction=rules[manifest["placement_id"]]["usable_remaining_arc_fraction"], half_forward_m=half,
        design_latency_s=design["latency_estimate_s"], design_speed_mps=design["speed_estimate_mps"])
    key = "placement_rule:"+manifest["condition_id"]
    audit.check(key+":same_availability", out["status"] == placement["status"])
    # Independent round-trip asset transforms can differ by numerical ulps;
    # this is a geometry reconstruction tolerance, not a safety relaxation.
    for field, value in out.items():
        literal = isinstance(value, (str, bool)) or value is None
        same = placement.get(field) == value if literal else np.allclose(placement.get(field), value, atol=1e-10, rtol=0)
        audit.check(key+":"+field, same)
    return out


def validate_figures(run, review, records, audit):
    if not review.is_dir():
        audit.check("review_exists", False)
        return
    by_id = {row["condition_id"]: row for row in records}
    summary = read(review/"reporting_summary.json")
    audit.check("review_condition_count", summary["development_conditions_recorded"] == len(records))
    plotted_ids = set()
    for path in review.rglob("presentation.json"):
        value = read(path)
        cid = value["condition_id"]
        plotted_ids.add(cid)
        audit.check("review_classification:"+cid, cid in by_id and value["classification"] == by_id[cid]["label"])
    audit.check("review_all_conditions", plotted_ids == set(by_id))
    for path in review.rglob("*.json"):
        side = read(path)
        if "png_sha256" not in side:
            continue
        key = "figure:"+str(path.relative_to(review))
        audit.check(key+":png_hash", audit.sha(path.with_suffix(".png")) == side["png_sha256"])
        audit.hashes(key+":source", side["source_sha256"])
        numbers = side["plotted_numbers"]
        if path.name == "paired_world.json":
            cid = path.parent.name
            audit.check(key+":evaluation", numbers["evaluation"] == by_id[cid])
            branch_results = read(run/"development"/cid/"summary.json")["branches"]
            for branch, world in numbers["world"].items():
                original = branch_results[branch]["terminal_prediction"]["world"]
                audit.check(key+":world:"+branch, np.array_equal(world, original))
            audit.check(key+":not_execution", numbers["no_execution_trace"] is True)
        if path.name == "outcome_summary.json":
            audit.check(key+":condition_order", [x["condition_id"] for x in numbers] == list(by_id))
            for row in numbers:
                cid = row["condition_id"]
                audit.check(key+":classification:"+cid, row["label"] == by_id[cid]["label"])
                for branch in ("A_OFF", "B_ON", "A_SHAM"):
                    for state in ("off", "on"):
                        expected = by_id[cid].get("checks", {}).get(branch, {}).get(state, {}).get("minimum_clearance_m")
                        audit.check(key+f":clearance:{cid}:{branch}:{state}",
                                    row.get(f"{branch}_{state}_clearance_m") == expected)
    with (review/"development_outcomes.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    audit.check("review_csv_coverage", [r["condition_id"] for r in rows] == list(by_id))
    for row in rows:
        cid = row["condition_id"]
        audit.check("review_csv_label:"+cid, row["label"] == by_id[cid]["label"])
        for branch in ("A_OFF", "B_ON", "A_SHAM"):
            for state in ("off", "on"):
                expected = by_id[cid].get("checks", {}).get(branch, {}).get(state, {}).get("minimum_clearance_m")
                actual = row[f"{branch}_{state}_clearance_m"]
                audit.check(f"review_csv_clearance:{cid}:{branch}:{state}",
                            actual == "N/A" if expected is None else float(actual) == expected)


def validate_call_counts(run, terminal, buffered, audit):
    """Count actual recorded sends/submissions, not successful responses only."""
    path = run/"aggregate/actual_call_counts.json"
    if not path.is_file():
        return None
    counts = read(path)
    audit.hashes("call_count_sources", counts.get("source_sha256", {}))
    audit.check("call_scope_development", counts["development"]["terminal_predictions"] == terminal
                and counts["development"]["buffer_only_requests"] == buffered)
    history_predict, history_buffer = 0, 0
    submissions, completions = set(), set()
    for ep in sorted((run/"development/history_acquisition/episodes").glob("*")):
        log = ep/"requests/wire.jsonl"
        if not log.is_file():
            continue
        for line in log.read_text().splitlines():
            row = json.loads(line)
            raw = row["raw"]
            payload_path = ep/raw["path"]
            audit.check("history_wire_hash:"+str(payload_path), audit.sha(payload_path) == raw["sha256"])
            if row["direction"] == "request" and row["action"] == "next":
                payload = read(payload_path)
                if payload["data"]["instruction"]:
                    history_predict += 1
                else:
                    history_buffer += 1
        for line in (ep/"controller/events.jsonl").read_text().splitlines():
            row = json.loads(line)
            key = (row.get("episode_id"), row.get("solve_id"))
            if row.get("type") == "reply" and row.get("op") == "submit" and row.get("status") == "submitted":
                submissions.add(key)
            elif row.get("type") == "solve_result":
                completions.add(key)
    live = counts["live_history_acquisition"]
    audit.check("call_scope_history_prediction", history_predict == live["history_prediction_requests"])
    audit.check("call_scope_history_buffer", history_buffer == live["history_buffer_only_requests"])
    audit.check("call_scope_MPC_submissions", len(submissions) == live["MPC_submitted_attempts"])
    audit.check("call_scope_MPC_observed_results", len(completions) == live["MPC_saved_completed_result_records"])
    audit.check("call_scope_MPC_unobserved", completions <= submissions and
                len(submissions-completions) == live["MPC_unobserved_results"])
    audit.check("call_source_total", history_predict+terminal == counts["totals"]["model_prediction_requests_source"])
    audit.check("call_buffer_total", history_buffer+buffered == counts["totals"]["buffer_only_requests"])
    log = run/"logs/server.log"
    if log.is_file():
        warmups = [line for line in log.read_text().splitlines() if "warmup done in" in line]
        audit.check("warmup_separate_count", len(warmups) == counts["startup_warmup"]["warmup_prediction_attempts"])
        audit.check("warmup_separate_log", counts["startup_warmup"]["warmup_log_line"] in warmups)
    return dict(history_predictions=history_predict, history_buffer_only=history_buffer,
                MPC_submitted=len(submissions), MPC_observed_results=len(completions),
                MPC_unobserved_results=len(submissions-completions))


def validate(run, *, review="review", output="validation.json"):
    run = Path(run).resolve()
    output_path = run/output
    if output_path.exists():
        raise FileExistsError(output_path)
    started = time.monotonic()
    audit = Audit()
    runner = _load_runner()
    freeze = audit.attempt("frozen_source_read", lambda: read(run/"development_freeze.json"))
    if freeze is not None:
        audit.hashes("frozen_core", freeze["source_sha256"])
        audit.hashes("frozen_environment", freeze.get("environment_source_sha256", {}))
        audit.hashes("frozen_config", {str(run/"protocol.json"): freeze["protocol_sha256"],
                                      str(run/"config_snapshot.yaml"): freeze["config_snapshot_sha256"]})
    original = read(run/"initial_source.json") if (run/"initial_source.json").is_file() else {}
    audit.hashes("historical_and_user_preserved", original.get("preserved", {}))
    protocol = audit.attempt("protocol_read", lambda: read(run/"protocol.json"))
    ledger = audit.attempt("ledger_read", lambda: read(run/"aggregate/development_ledger.json"))
    records, terminal_count, buffer_count = [], 0, 0
    coverage_complete = False
    inspected_placements = set()
    inspected_banks = {}
    if protocol is not None and ledger is not None:
        records = ledger["records"]
        declaration = [dict(condition_id=row) if isinstance(row, str) else row
                       for row in protocol["development"]["condition_order"]]
        checked = audit.attempt("deterministic_schedule", lambda: validate_development_order(declaration, records))
        if checked is not None:
            audit.check("selected_first_qualifier", checked["selected_condition_id"] == ledger["selected_condition_id"])
            coverage_complete = bool(checked["selected_condition_id"] or len(records) == len(declaration))
            audit.check("complete_declared_coverage", coverage_complete)
        for row in records:
            cid = row["condition_id"]
            manifest_path = run/"development/input_manifests"/(cid+".json")
            manifest = audit.attempt("manifest:"+cid, lambda p=manifest_path: read(p))
            if manifest is None:
                continue
            audit.check("manifest_hash:"+cid, audit.sha(manifest_path) == row["input_manifest_sha256"])
            audit.check("ledger_vs_condition_record:"+cid, read(run/"development"/cid/"evaluation.json") == row)
            placement_path = resolve(manifest["placement_path"]) if manifest.get("placement_path") else None
            if manifest.get("bank_path"):
                bank_path = resolve(manifest["bank_path"])
                if bank_path not in inspected_banks:
                    inspected_banks[bank_path] = audit.attempt("history_bank:"+cid,
                        lambda p=bank_path: validate_bank(p, audit))
                bank = inspected_banks[bank_path]
                if bank is not None:
                    suffix = audit.attempt("authentic_history_suffix:"+cid,
                        lambda b=bank, m=manifest: history_suffix(b["frames"], m["history_count"]))
                    if suffix is not None:
                        if manifest.get("availability") == "HISTORY_UNAVAILABLE":
                            audit.check("history_unavailable_reproduced:"+cid, suffix["status"] == "HISTORY_UNAVAILABLE")
                        elif manifest.get("availability") == "AVAILABLE":
                            audit.check("same_frozen_history_suffix:"+cid,
                                        suffix["status"] == "AVAILABLE" and suffix["history"] == manifest["history"])
                    if placement_path is not None and placement_path not in inspected_placements:
                        audit.attempt("frozen_placement_rule:"+cid,
                            lambda m=manifest, b=bank: validate_placement_rule(run, m, b, protocol, audit))
            if placement_path is not None and placement_path not in inspected_placements:
                inspected_placements.add(placement_path)
                audit.attempt("placement:"+cid, lambda p=placement_path: validate_placement(p, runner, audit))
            availability = manifest.get("availability", "AVAILABLE")
            if availability != "AVAILABLE":
                audit.check("unavailable_not_success:"+cid, row["qualified"] is False and row["label"] == availability)
                audit.check("unavailable_no_calls:"+cid, row["call_counts"] == {"terminal_predictions": 0, "buffer_only": 0}
                            and row["completed_branches"] == [])
                audit.check("unavailable_no_fabricated_branches:"+cid,
                            not (run/"development"/cid/"branches").exists())
                continue
            manifest["_manifest_path"] = str(manifest_path)
            recomputed = audit.attempt("geometry_input_recompute:"+cid, lambda m=manifest: runner.evaluate_saved(run, m))
            if recomputed is not None:
                audit.check("original_source_outcome_reproduced:"+cid,
                            all(row.get(key) == value for key, value in recomputed.items()))
                terminal_count += recomputed["call_counts"]["terminal_predictions"]
                buffer_count += recomputed["call_counts"]["buffer_only"]
                if recomputed["label"] == "TECHNICAL_INVALID":
                    audit.check("technical_failure_not_qualified:"+cid, not row["qualified"])
        audit.check("terminal_prediction_count", terminal_count == ledger["actual_terminal_prediction_requests"])
        audit.check("buffer_only_count", buffer_count == ledger["actual_buffer_only_requests"])
        audit.check("declared_budget", len(records) <= protocol["development"]["maximum_conditions"] <= 12 and
                    terminal_count <= protocol["development"]["maximum_terminal_predictions"] <= 36)
        known_ids = {r["condition_id"] for r in declaration}
        actual_ids = {p.parent.name for p in (run/"development").glob("*/evaluation.json")}
        audit.check("no_hidden_or_undeclared_conditions", actual_ids == {r["condition_id"] for r in records} and actual_ids <= known_ids)
        audit.attempt("figures_revalidation", lambda: validate_figures(run, run/review, records, audit))
    confirmation_path = run/"aggregate/confirmation_ledger.json"
    limitations = []
    confirmation = read(confirmation_path) if confirmation_path.is_file() else None
    if confirmation is None:
        audit.check("confirmation_absent_not_claimed", not list((run/"confirmation").glob("episode_*/qualification.json")))
        if ledger is not None and ledger.get("selected_condition_id") is None:
            audit.check("no_confirmations_without_development_qualifier", not list((run/"confirmation").glob("episode_*")))
    else:
        # This entry point deliberately does not pretend a presence/hash check
        # reconstructs future schemas. Extend saved-confirmation checks only
        # after its fixed capture schema is available; never call a solver here.
        confirmation_records = confirmation.get("records", confirmation.get("episodes", []))
        attempts = [r for r in confirmation_records if r.get("attempted") is True]
        if not attempts:
            audit.check("confirmation_not_attempted_is_NA", all(
                r.get("attempted") is False and r.get("qualified") is None
                for r in confirmation_records) and confirmation.get("attempted", 0) == 0
                and confirmation.get("qualification_rate") is None)
            audit.check("no_actual_confirmation_directories", not list((run/"confirmation").glob("episode_*")))
        else:
            audit.check("confirmation_independent_schema_supported", False,
                        "Confirmation schema requires dedicated saved-record reconstruction; no runtime fallback")
            limitations.append("ONLINE_CONFIRMATION_VALIDATION_NOT_IMPLEMENTED_FOR_THIS_SCHEMA")
    calls = audit.attempt("call_counts_saved_reconstruction", lambda: validate_call_counts(run, terminal_count, buffer_count, audit))
    authoritative_geometry_path = run/"technical_preflight/post_execution_geometry_audit_v2.json"
    if authoritative_geometry_path.is_file():
        geometry_audit = read(authoritative_geometry_path)
        audit.check("authoritative_resolved_dt_geometry_audit", geometry_audit["valid"] is True
                    and all(geometry_audit["checks"].values()))
        audit.hashes("authoritative_geometry_source", geometry_audit.get("source_sha256", {}))
        audit.check("authoritative_geometry_no_new_runtime", all(geometry_audit[k] == 0 for k in
            ("new_model_calls", "new_MPC_calls", "new_Isaac_calls", "new_optimizer_calls")))
        limitations.append("Earlier post_execution_geometry_audit.json is superseded only for its ideal-1/60 reconstruction; authoritative v2 uses original saved resolved dt.")
    result = dict(valid=bool(audit.checks) and all(audit.checks.values()),
                  artifact_validation_only=True, scientific_failure_is_not_corruption=True,
                  source_outcomes=[dict(condition_id=r["condition_id"], label=r["label"], qualified=r["qualified"]) for r in records],
                  complete_declared_coverage=coverage_complete,
                  recomputed_terminal_predictions=terminal_count, recomputed_buffer_only_requests=buffer_count,
                  checked_runtime_placements=len(inspected_placements), checks=audit.checks, details=audit.details,
                  reconstructed_acquisition_call_counts=calls,
                  authoritative_saved_execution_geometry_audit=str(authoritative_geometry_path) if authoritative_geometry_path.is_file() else None,
                  failures=[key for key, value in audit.checks.items() if not value], limitations=limitations,
                  new_model_inference=0, new_GP_rigid_optimization=0, new_MPC_solve=0, new_rollout=0,
                  new_Isaac_runtime=0, wall_time_s=time.monotonic()-started,
                  validator_sha256=digest(Path(__file__)))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--review", default="review")
    parser.add_argument("--output", default="validation.json")
    args = parser.parse_args()
    result = validate(args.run, review=args.review, output=args.output)
    print(json.dumps({key: result[key] for key in ("valid", "complete_declared_coverage", "failures", "wall_time_s")}, indent=2))
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
