#!/usr/bin/env python3
"""Derive and validate one Stage 0-C LightNav single-chunk run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from reconciliation.lightnav_adapter import (
    derive_single_chunk,
    save_json_exclusive,
    trajectory_sanity,
    validate_single_chunk_run,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPOSITORY_ROOT / "configs/stage0_lightnav_single_chunk.yaml",
    )
    parser.add_argument("--require-execution", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("config must contain a mapping")
    derived = args.run_directory / "derived"
    if not (derived / "lightnav_local_path.npy").exists():
        derive_single_chunk(args.run_directory)
    validation = validate_single_chunk_run(
        args.run_directory,
        require_execution=args.require_execution,
    )
    local = np.load(derived / "lightnav_local_path.npy", allow_pickle=False)
    sanity = trajectory_sanity(local)
    limits = config["execution_safety"]
    checks = {
        "translation_within_limit": (
            sanity["translation_magnitude_max_m"]
            <= float(limits["maximum_translation_from_observation_m"])
        ),
        "yaw_within_limit": (
            sanity["absolute_yaw_max_rad"] <= float(limits["maximum_absolute_local_yaw_rad"])
        ),
        "spacing_within_limit": (
            sanity["consecutive_spacing_max_m"]
            <= float(limits["maximum_consecutive_spacing_m"])
        ),
    }
    validation["execution_safety_checks"] = checks
    validation["safe_for_execution"] = all(checks.values())
    validation_path = args.run_directory / "results/validation.json"
    if not validation_path.exists():
        save_json_exclusive(validation_path, validation)
    print(json.dumps(validation, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
