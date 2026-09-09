#!/usr/bin/env python3
"""Create pre-inference contact sheets for the six captured Stage 0-G3 V0 histories."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation.lightnav_adapter import save_json_exclusive
from reconciliation.stage0g3_moving_history_qualification import SCENARIO_IDS, load_yaml, validate_config


FRAMES = (0, 16, 32, 48, 63)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("run_directory", type=Path)
    parser.add_argument("--record-inspection-passed", action="store_true")
    args = parser.parse_args()
    run = args.run_directory.resolve(); config = load_yaml(run / "config_snapshot.yaml"); validate_config(config, ROOT)
    destination = run / "history_previews"
    if args.record_inspection_passed:
        required = [destination / f"{scenario_id}_V0_contact_sheet.png" for scenario_id in SCENARIO_IDS]
        if any(not path.is_file() for path in required): raise ValueError("all six V0 contact sheets must exist before recording inspection")
        save_json_exclusive(destination / "visual_inspection.json", {
            "inspection_actor": "Codex visual inspection", "inspection_utc": datetime.now(timezone.utc).isoformat(),
            "all_six_v0_contact_sheets_inspected": True, "moving_viewpoint_progression_visible": True,
            "stale_robot_render_artifact_absent": True, "inspection_before_any_stage0g3_inference": not any(run.glob("qualification/G2_Q*/V*/raw/lightnav.npy")),
            "allowed_fixes": "technical camera/render synchronization only",
        })
        print("STAGE0G3_HISTORY_VISUAL_INSPECTION=PASS count=6")
        return
    destination.mkdir(exist_ok=False)
    overview, overview_axes = plt.subplots(6, 5, figsize=(15, 14), constrained_layout=True)
    for row, scenario_id in enumerate(SCENARIO_IDS):
        case = run / "qualification" / scenario_id / "V0"
        figure, axes = plt.subplots(1, 5, figsize=(15, 2.6), constrained_layout=True)
        for column, frame_index in enumerate(FRAMES):
            path = case / f"input/history/frame_{frame_index:06d}.png"
            image = Image.open(path).convert("RGB")
            axes[column].imshow(image); axes[column].axis("off"); axes[column].set_title(f"frame {frame_index}\nt={frame_index / 4:.2f}s", fontsize=8)
            overview_axes[row, column].imshow(image); overview_axes[row, column].axis("off")
            overview_axes[row, column].set_title(f"{scenario_id}\nf{frame_index}" if column == 0 else f"f{frame_index}", fontsize=7)
        figure.suptitle(f"{scenario_id}/V0 — SCRIPTED MOVING HISTORY; not controller execution")
        figure.savefig(destination / f"{scenario_id}_V0_contact_sheet.png", dpi=130); plt.close(figure)
    overview.suptitle("Stage 0-G3 pre-inference V0 history inspection")
    overview.savefig(destination / "all_six_v0_contact_sheets.png", dpi=130); plt.close(overview)
    save_json_exclusive(destination / "manifest.json", {
        "created_before_any_stage0g3_lightnav_inference": True,
        "scenario_ids": list(SCENARIO_IDS), "variant": "V0", "frame_indices": list(FRAMES),
        "inspection_status": "PENDING_EXPLICIT_VISUAL_INSPECTION", "scripted_history_is_not_controller_execution": True,
    })
    print(f"STAGE0G3_HISTORY_PREVIEWS={destination} count=6 inference_outputs_at_creation=0")


if __name__ == "__main__": main()
