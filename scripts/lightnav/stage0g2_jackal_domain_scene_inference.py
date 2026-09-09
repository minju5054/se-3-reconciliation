#!/usr/bin/env python3
"""Run the frozen Stage 0-G inference implementation with the G2 identity contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from reconciliation import stage0g2_jackal_domain_scene_qualification as g2


spec = importlib.util.spec_from_file_location("stage0g_inference_impl", ROOT / "scripts/lightnav/stage0g_lightnav_inference.py")
if spec is None or spec.loader is None: raise RuntimeError("cannot load frozen Stage 0-G inference implementation")
implementation = importlib.util.module_from_spec(spec); spec.loader.exec_module(implementation)
implementation.SCENARIO_IDS = g2.SCENARIO_IDS
implementation.VARIANT_IDS = g2.VARIANT_IDS
implementation.validate_config = g2.validate_config


if __name__ == "__main__": implementation.main()
