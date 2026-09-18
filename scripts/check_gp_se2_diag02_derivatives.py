#!/usr/bin/env python3
"""Freeze and check supplied derivatives; this script never optimizes."""
import argparse
import json
import os
from pathlib import Path
import sys

for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
os.environ['JAX_PLATFORM_NAME'] = 'cpu'
os.environ['JAX_ENABLE_X64'] = 'true'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))

from reconciliation.gp_se2_diag02_validation import (
    build_validation_points, freeze_points, freeze_protocol, run_validation, publish_validation_gate,
    freeze_runtime_cut_addendum,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=['freeze', 'cut-policy', 'check'])
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--primary', type=Path, default=ROOT/'data/robotless_gp_se2_01/primary_20260918T054000Z')
    parser.add_argument('--diag01', type=Path, default=ROOT/'data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z')
    parser.add_argument('--attempt', default='attempt_01')
    parser.add_argument('--geometry-report', type=Path)
    parser.add_argument('--cut-evidence', type=Path,
                        help='preserved pre-guard attempt directory containing exact-cut JSON and NPZ')
    args = parser.parse_args()
    directory = args.run/'derivative_checks'
    if args.phase == 'cut-policy':
        if args.cut_evidence is None: parser.error('cut-policy requires --cut-evidence')
        print(json.dumps(freeze_runtime_cut_addendum(directory, args.cut_evidence)))
        return
    points = build_validation_points(args.primary, args.diag01, seeds=args.run/'seeds')
    if args.phase == 'freeze':
        if not (directory/'protocol.json').exists():
            freeze_protocol(directory)
        result = freeze_points(directory, points)
        print(json.dumps(dict(point_count=len(result['points']), protocol_sha256=result['protocol_sha256'])))
    else:
        result = run_validation(directory, points, attempt=args.attempt, geometry_report=args.geometry_report)
        if result['valid']:
            result = publish_validation_gate(directory, args.attempt)
        print(json.dumps(result))


if __name__ == '__main__':
    main()
