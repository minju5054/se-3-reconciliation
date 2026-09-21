#!/usr/bin/env python3
"""Validate additional saved-only paired figures and ZIP, never run a model."""
import argparse
import csv
import hashlib
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from reconciliation.join_source03 import read, save, sha


def validate(run):
    checks = []
    def check(name, passed):
        checks.append({'name': name, 'passed': bool(passed)})

    authority = read(run / 'validation_v2.json')
    check('original_saved_record_validation', authority['valid'])
    folder = run / 'review_comparisons'
    figures = read(folder / 'figure_manifest.json')
    for figure in figures:
        side = read(folder / figure['sidecar'])
        check(figure['png'] + ':png_hash', sha(folder / figure['png']) == figure['png_sha256'])
        check(figure['png'] + ':config', side['config_sha256'] == sha(run / 'config_snapshot.yaml'))
        check(figure['png'] + ':coverage', len(side['conditions']) == len(side['records']) == len(side['inputs']))
        check(figure['png'] + ':renderer', side['renderer_sha256'] == sha(ROOT / 'scripts/review_join_source03_comparisons.py'))
        for cid, record, source in zip(side['conditions'], side['records'], side['inputs']):
            path = run / 'paired_diagnostics' / cid / 'evaluation.json'
            check(figure['png'] + ':' + cid + ':numbers', record == read(path))
            check(figure['png'] + ':' + cid + ':input', source == read(run / 'input_manifests' / (cid + '.json')))
            check(figure['png'] + ':' + cid + ':hash', side['source_sha256'][str(path)] == sha(path))

    expected = {}
    for row in read(run / 'aggregate/ledger.json'):
        if row['status'] != 'COMPLETED':
            continue
        cid = row['condition_id']
        for alias, d in read(run / 'paired_diagnostics' / cid / 'evaluation.json')['details'].items():
            for i, (local, world, clearance) in enumerate(zip(d['raw_local'], d['world'], d['geometry_on']['node_clearance_m'])):
                expected[(cid, alias, i)] = (list(local) + list(world) + [clearance], d['world_sha256'])
    fields = ['local_forward_m', 'local_left_m', 'local_yaw_rad', 'world_x_m', 'world_y_m', 'world_yaw_rad', 'edge_clearance_m']
    with (run / 'aggregate/row_geometry.csv').open() as f:
        rows = list(csv.DictReader(f))
    keys = [(r['condition'], r['branch'], int(r['row_index'])) for r in rows]
    check('row_coverage_no_duplicates', len(keys) == len(set(keys)) and set(keys) == set(expected))
    for row, key in zip(rows, keys):
        if key not in expected:
            check(str(key), False)
            continue
        values, world_hash = expected[key]
        check(str(key), [float(row[f]) for f in fields] == values and row['source_world_sha256'] == world_hash)
    with zipfile.ZipFile(run / 'review_bundle_final.zip') as archive:
        names = archive.namelist()
        check('zip_unique_members', len(names) == len(set(names)))
        for name in names:
            check('zip:' + name, hashlib.sha256(archive.read(name)).hexdigest() == sha(run / name))
    return {'valid': all(c['passed'] for c in checks), 'checks': checks,
            'failed': [c for c in checks if not c['passed']],
            'figure_count': len(figures), 'row_count': len(rows),
            'new_model_calls': 0, 'new_MPC_calls': 0,
            'validator_sha256': sha(__file__), 'authoritative_validation_sha256': sha(run / 'validation_v2.json')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    result = validate(run)
    save(run / 'validation_review.json', result)
    print({k: result[k] for k in ['valid', 'failed', 'figure_count', 'row_count']})
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
