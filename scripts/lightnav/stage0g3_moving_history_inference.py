#!/usr/bin/env python3
"""Run frozen LightNav once per Stage 0-G3 moving-history case."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation import stage0g3_moving_history_qualification as g3


spec = importlib.util.spec_from_file_location("stage0g_inference_impl", ROOT / "scripts/lightnav/stage0g_lightnav_inference.py")
if spec is None or spec.loader is None:
    raise RuntimeError("cannot load frozen Stage 0-G inference implementation")
implementation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(implementation)
implementation.SCENARIO_IDS = g3.SCENARIO_IDS
implementation.VARIANT_IDS = g3.VARIANT_IDS


def load_config(run: Path) -> dict:
    config = g3.load_yaml(run / "config_snapshot.yaml")
    g3.validate_config(config, ROOT)
    freeze = json.loads((run / "config_freeze.json").read_text(encoding="utf-8"))
    if g3.sha256_file(run / "config_snapshot.yaml") != freeze.get("config_sha256"):
        raise ValueError("frozen Stage 0-G3 config hash mismatch")
    if g3.sha256_file(run / "approach_feasibility.json") != freeze.get("approach_feasibility_sha256"):
        raise ValueError("frozen Stage 0-G3 approach hash mismatch")
    return g3.resolved_scientific_config(config, ROOT)


implementation.load_config = load_config


def main() -> None:
    if len(sys.argv) < 2:
        raise ValueError("run directory is required")
    run = Path(sys.argv[1]).resolve()
    before = {path.parents[1].relative_to(run).as_posix() for path in run.glob("qualification/G2_Q*/V*/raw/lightnav.npy")}
    implementation.main()
    control = json.loads((run / "stage0g2_control_reference.json").read_text(encoding="utf-8"))
    after_paths = sorted(run.glob("qualification/G2_Q*/V*/raw/lightnav.npy"))
    for raw_path in after_paths:
        case = raw_path.parents[1]
        relative = case.relative_to(run).as_posix()
        if relative in before:
            continue
        scenario_id, variant_id = case.parent.name, case.name
        g2_case = Path(control["primary_run"]) / "qualification" / scenario_id / variant_id
        save_json_exclusive(case / "paired_control_provenance.json", {
            "stage0g2_control_run": control["primary_run"],
            "stage0g2_config_sha256": control["config_sha256"],
            "stage0g2_raw_lightnav_sha256": control["cases"][f"{scenario_id}/{variant_id}"]["raw_sha256"],
            "stage0g2_capture_metadata_sha256": g3.sha256_file(g2_case / "input/capture_metadata.json"),
            "stage0g3_capture_metadata_sha256": g3.sha256_file(case / "input/capture_metadata.json"),
            "paired_final_observation_pose": control["cases"][f"{scenario_id}/{variant_id}"]["final_pose_se2"],
            "changed_variable": "64-frame stationary visual history versus 64-frame scripted moving egocentric visual history",
            "no_controller_execution": True,
        })


if __name__ == "__main__":
    main()
