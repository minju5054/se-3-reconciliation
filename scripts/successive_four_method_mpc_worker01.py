#!/usr/bin/env python3
"""Existing common-B restoration and official worker, with saved native .4 bound."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src')]
import osa03_common_b_mpc_worker as common
from reconciliation.join_source03 import read
from reconciliation.long_source_mpc01 import configure_before_tracker


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--lightnav-checkout',required=True);p.add_argument('--preflight',action='store_true')
    args=p.parse_args();frozen=read(args.run/'speed_intervention.json')
    original=common.load_official
    def load(checkout):
        module,provenance=original(checkout)
        return configure_before_tracker(module,provenance,frozen)
    common.load_official=load
    common.original.load_official=load
    if args.preflight:
        print(json.dumps(common.preflight(args.lightnav_checkout,read(args.run/'common_state.json'),
            read(args.run/'preflight_references.json')),allow_nan=False))
    else:
        sys.argv=[sys.argv[0],'--lightnav-checkout',args.lightnav_checkout]
        return common.original.main()


if __name__=='__main__':
    raise SystemExit(main())
