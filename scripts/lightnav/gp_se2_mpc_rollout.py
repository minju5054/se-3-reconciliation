#!/usr/bin/env python3
"""GP-SE2-01 pinned official MPC batch audit / frozen-state rollout.

Run using EXTERNAL/mujoco_demo/.venv/bin/python, not system Python. Example
request JSON: {"source_root":"...", "audit_only":true,
"cases":[{"episode_id":"episode_008_repeat_01","handoff_id":"handoff_013"}]}.
For rollouts set audit_only=false and provide each case's methods object mapping
method names to candidate world .npy paths (or null for no candidate), plus goal.
Output must be new. Source files and external source are never modified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

import numpy as np

from reconciliation.gp_se2_rollout import (
    FAILURE_POLICY, PREVIOUS_CONTROL_POLICY, counterfactual_rollout,
    execution_metrics, historical_solve_audit, load_frozen_context, save_rollout,
    verify_source_records,
)
from reconciliation.online_mpc_adapter import load_official, sha256


def write_new(path, payload):
    with path.open("x") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lightnav-checkout", type=Path, default=Path("/home/gpuadmin/Workspace/external/LightNav-0-official-demo"))
    args = parser.parse_args()
    checkout = args.lightnav_checkout.resolve()
    if Path(sys.prefix).resolve() != (checkout/"mujoco_demo/.venv").resolve():
        raise ValueError("use the existing official-demo isolated MPC environment")
    request = json.loads(args.request.read_text())
    output = args.output.resolve()
    source = Path(request["source_root"]).resolve()
    if output.is_relative_to(source):
        raise ValueError("derived output must be outside the immutable source run")
    cases = request["cases"]
    keys = [(c["episode_id"], c["handoff_id"]) for c in cases]
    if len(keys) != len(set(keys)):
        raise ValueError("duplicate case in batch")
    output.mkdir(parents=True, exist_ok=False)
    module, provenance = load_official(checkout)
    settings_hash = hashlib.sha256(json.dumps(provenance["official_settings"], sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    limits = {"v_max": module.OBJNAV_V_MAX, "omega_max": module.W_MAX,
              "a_v_max": module.A_MAX_V, "a_omega_max": module.A_MAX_W}
    provenance.update(official_settings_sha256=settings_hash, resolved_limits=limits,
        previous_control_policy=PREVIOUS_CONTROL_POLICY, failure_policy=FAILURE_POLICY,
        request_sha256=sha256(args.request),
        implementation_sha256={str(p.relative_to(Path(__file__).resolve().parents[2])):sha256(p) for p in
            [Path(__file__).resolve(), Path(__file__).resolve().parents[2]/"src/reconciliation/gp_se2_rollout.py"]})
    write_new(output/"provenance.json", provenance)
    write_new(output/"request.json", request)
    summaries = []
    for case in cases:
        frozen = load_frozen_context(source, *[case[k] for k in ("episode_id", "handoff_id")])
        case_dir = output/(case["episode_id"]+"__"+case["handoff_id"])
        case_dir.mkdir()
        write_new(case_dir/"input_context.json", frozen)
        audit = historical_solve_audit(module, frozen)
        write_new(case_dir/"historical_solve_audit.json", audit)
        if not audit["passed"]:
            raise RuntimeError(f"historical solve audit failed: {frozen['case_id']}")
        methods = {}
        if not request.get("audit_only", False):
            for method, candidate_path in case["methods"].items():
                if Path(method).name != method:
                    raise ValueError("method name must be a path component")
                if candidate_path is None:
                    methods[method] = {"status": "NO_CANDIDATE", "rollout_performed": False,
                        "candidate": None, "rollout_states": None, "rollout_commands": None,
                        "metrics": None, "missing_reason": "solver provided no validated candidate; no RAW fallback"}
                    (case_dir/method).mkdir()
                    write_new(case_dir/method/"status.json", methods[method])
                    continue
                candidate_file = Path(candidate_path).resolve()
                candidate_hash = sha256(candidate_file)
                candidate = np.load(candidate_file, allow_pickle=False)
                rollout = counterfactual_rollout(module, frozen, candidate,
                    horizon_s=request.get("horizon_s", 3.), control_hz=request.get("control_hz", 10.),
                    integration_hz=request.get("integration_hz", 60.))
                if sha256(candidate_file) != candidate_hash:
                    raise ValueError("candidate changed during rollout")
                rollout["candidate_source"] = {"path": str(candidate_file), "sha256": candidate_hash}
                rollout["official_settings_sha256"] = settings_hash
                method_dir = save_rollout(case_dir/method, rollout)
                metrics = None
                if "goal" in case:
                    metrics = execution_metrics(rollout, case["goal"], limits=limits,
                        position_tolerance_m=request.get("goal_position_tolerance_m", .15),
                        yaw_tolerance_rad=request.get("goal_yaw_tolerance_rad", np.pi/12),
                        dwell_s=request.get("goal_dwell_s", .2))
                    write_new(method_dir/"execution_metrics.json", metrics)
                methods[method] = {"status": "ROLLED_OUT" if not rollout["controller_failure_count"] else "ROLLED_OUT_WITH_CONTROLLER_FAILURE",
                    "rollout_performed": True, "controller_failure_count": rollout["controller_failure_count"], "metrics": metrics}
        verify_source_records(source, frozen["source_files"])
        summaries.append({"case_id": frozen["case_id"], "historical_audit_passed": audit["passed"], "methods": methods})
    if sha256(Path(provenance["mpc_source"])) != provenance["mpc_source_sha256"]:
        raise ValueError("official MPC source changed during audit/rollout")
    write_new(output/"summary.json", {"audit_only": request.get("audit_only", False), "cases": summaries,
        "source_files_preserved": True, "official_source_unchanged": True})
    write_new(output/"output_hashes.json", {"files": [
        {"path": str(p.relative_to(output)), "sha256": sha256(p), "bytes": p.stat().st_size}
        for p in sorted(output.rglob("*")) if p.is_file()]})
    print(json.dumps({"output": str(output), "cases": len(summaries), "historical_audits_passed": True}))


if __name__ == "__main__":
    main()
