#!/usr/bin/env python3
"""Saved-only validation; zero official selector invocations and no solver work."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from run_spatial_correspondence_selector_diag01 import (ROOT, RESULTS, read, save, sha, verify,
    authenticate, compute_sources, METHODS, RULES)
from reconciliation.spatial_correspondence_selector import classification, diagnostic_guard, PNGS
from reconciliation.online_mpc_adapter import selection_audit
from validate_robotless_online_handoffs import equal_record


def validate(run, deep=True):
    verify(run)
    cfg = read(run/'protocol.json')
    authenticate(cfg, deep=deep)
    summary = read(run/'summary.json')
    prepared = read(run/'prepared.json')
    saved = iter(row for q in summary['sources'].values() for row in q['matched_selector'])
    count = 0

    def audit_saved_selection(reference, pose, *, horizon, weights):
        nonlocal count
        record = next(saved)
        assert record['matched_pose_world'] == pose.tolist()
        result = np.array(record['physical_H5_world'])
        audit = selection_audit(reference, pose, result, horizon=horizon, weights=weights)
        assert audit['indices'] == record['H5_rows'] == record['original_node_identities']
        # Verify the actual sequential unwrapped yaw, in addition to modulo equality.
        previous = float(pose[2])
        expected = []
        for identity in audit['indices']:
            difference = float(reference[identity, 2]-previous)
            previous += math.atan2(math.sin(difference), math.cos(difference))
            expected.append(previous)
        np.testing.assert_allclose(expected, result[:, 2], rtol=0, atol=1e-12)
        count += 1
        return result

    with diagnostic_guard() as calls:
        rebuilt = compute_sources(prepared, audit_saved_selection)
    assert all(v == 0 for v in calls.values())
    assert next(saved, None) is None and count == cfg['expected_scientific_selector_queries']
    historical = read(ROOT/next(iter(cfg['historical_results'])))['summary']['sources']
    for sid, q in rebuilt.items():
        old = summary['sources'][sid]
        for key in q:
            equal_record(q[key], old[key])
        for method in METHODS:
            for key, value in old['saved_execution_context'][method].items():
                assert value == historical[sid]['primary_metrics'][method][key]
        for key, value in old['saved_Half_minus_Full'].items():
            h, f = [old['saved_execution_context'][n][key] for n in METHODS[1:]]
            assert value == (None if h is None or f is None else h-f)
    assert summary['classification'] == classification(summary['sources'])
    assert summary['counts']['selector_queries'] == count
    assert all(v == 0 for k, v in summary['counts'].items() if k != 'selector_queries')
    assert summary['scientific_freeze_sha'] == read(run/'execution_start.json')['sha']
    if (RESULTS/'figure_manifest.json').exists():
        manifest = read(RESULTS/'figure_manifest.json')
        assert sorted(p.name for p in (RESULTS/'figures').iterdir()) == sorted(PNGS)
        assert manifest['summary_sha256'] == sha(run/'summary.json')
        assert [r['file'] for r in manifest['figures']] == PNGS
        for record in manifest['figures']:
            assert sha(RESULTS/'figures'/record['file']) == record['sha256']
    if (run/'result_hashes.json').exists():
        for p, h in read(run/'result_hashes.json').items():
            assert sha(run/p) == h, p
        tracked = read(RESULTS/'result_summary.json')
        assert tracked['summary'] == summary and tracked['result_hashes_sha256'] == sha(run/'result_hashes.json')
        for p, h in tracked['table_hashes'].items():
            assert sha(RESULTS/p) == h
    return summary, dict(valid=True, old_transport_multisource_R00_R01_validators=deep,
        independently_checked_saved_selections=count, new_official_selector_queries=0,
        new_optimizer_MPC_rollout_calls=0, raw_sources_unchanged=True)


def csv_write(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fields, lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v) if isinstance(v, (list, dict)) else v for k, v in row.items()})


def write(run):
    summary, valid = validate(run)
    save(run/'validation.json', valid)
    b_rows, selector_rows, reset_rows = [], [], []
    for sid, q in summary['sources'].items():
        for rule, row in q['correspondences'].items():
            b_rows.append(dict(source_id=sid, rule=rule, target_basis='continuous original FRESH at B', **row))
        for method, item in q['first_official_selection'].items():
            for target in ['nearest', 'H5_start']:
                b_rows.append(dict(source_id=sid, rule=method+'_'+target,
                    target_basis='original identity at first matched query; gaps measured from B',
                    query_tick=item['query_tick'], query_time_after_B_s=item['query_time_after_B_s'],
                    query_pose_world=item['query_pose_world'], **item[target]))
        selector_rows.extend(dict(source_id=sid, **r) for r in q['matched_selector'])
        for method, row in q['progress_reset']['methods'].items():
            reset_rows.append(dict(source_id=sid, method=method,
                matched_ticks=q['progress_reset']['matched_ticks'],
                first_Half_Full_different_tick=q['progress_reset']['first_Half_Full_different_tick'], **row))
    csv_write(RESULTS/'b_correspondence.csv', b_rows)
    csv_write(RESULTS/'matched_selector.csv', selector_rows)
    csv_write(RESULTS/'progress_reset_summary.csv', reset_rows)
    print(json.dumps(valid))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--check-only', action='store_true')
    args = p.parse_args()
    if args.check_only:
        print(json.dumps(validate(args.run.resolve())[1]))
    else:
        write(args.run.resolve())
