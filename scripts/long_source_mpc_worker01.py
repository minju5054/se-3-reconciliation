#!/usr/bin/env python3
"""Isolated bound configuration followed by the byte-unchanged official worker."""
import argparse
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read
from reconciliation.long_source_mpc01 import configure_before_tracker
import online_mpc_worker as worker


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--lightnav-checkout', type=Path, required=True)
    args = p.parse_args()
    frozen = read(args.run/'speed_intervention.json')
    original_loader = worker.load_official
    def load(checkout):
        module, provenance = original_loader(checkout)
        return configure_before_tracker(module, provenance, frozen)
    worker.load_official = load
    sys.argv = [sys.argv[0], '--lightnav-checkout', str(args.lightnav_checkout)]
    return worker.main()


if __name__ == '__main__':
    raise SystemExit(main())
