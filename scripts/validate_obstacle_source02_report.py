#!/usr/bin/env python3
"""Independently recheck OSA02 saved episodes and presentation, with no live calls."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
from reconciliation.join_source03 import read, save, sha
from validate_robotless_online_handoffs import equal_record
from validate_obstacle_source_acquisition02 import validate as technical
from validate_obstacle_source02_online import validate as online
from report_obstacle_source_acquisition02 import compact
from analyze_join_online02 import csvread, pose_rows


def csv_parity(path, records):
    with path.open() as f:
        saved = list(csv.DictReader(f))
    assert len(saved) == len(records), 'CSV record count'
    for actual, expected in zip(saved, records, strict=True):
        for key, value in actual.items():
            wanted = ';'.join(expected['raw_failure_reasons']) if key == 'failure_reasons' else str(expected[key])
            assert value == wanted, (key, value, wanted)


def zip_parity(parent):
    with zipfile.ZipFile(parent / 'review_bundle.zip') as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert z.read(name) == (parent / name).read_bytes(), name


def validate(parent):
    t = technical(parent)
    o = online(parent)
    equal_record(t, read(parent / 'phase0/validation.json'), 'technical')
    equal_record(o, read(parent / 'phaseB/validation.json'), 'online')
    s = read(parent / 'aggregate/summary.json')
    equal_record([compact(r) for r in o['rows']], s['phaseB'], 'summary')
    equal_record(t['qualification'], s['phase0'], 'qualification')
    for key in ('classification', 'representative', 'qualified_count'):
        assert s[key] == o[key] == read(parent / 'source_bundle/manifest.json')[key]
    csv_parity(parent / 'aggregate/episodes.csv', s['phaseB'])
    plots = read(parent / 'review/plot_manifest.json')
    for p in plots:
        q = parent / 'review' / p['name']
        assert sha(q.with_suffix('.png')) == p['png_sha256']
        assert sha(q.with_suffix('.json')) == p['json_sha256']
        sidecar = read(q.with_suffix('.json'))
        assert sidecar['processing_script_sha256'] == sha(ROOT / 'scripts/report_obstacle_source_acquisition02.py')
        for name, h in sidecar['source_sha256'].items():
            assert sha(name) == h, name
    for row in o['rows']:
        eid = row['episode_id']
        d = read(parent / 'review' / (eid + '_world.json'))['data']
        ep = parent / 'phaseB/episodes' / eid
        for k in ('OLD', 'FRESH'):
            np.testing.assert_array_equal(d[k], np.load(ep / row[k]['world_ref']['path']))
        np.testing.assert_array_equal(d['actual'], pose_rows(csvread(ep / 'execution.csv')))
        equal_record(d['metrics'], compact(row), 'world sidecar')
        c = row['context']; timing = read(parent / 'review' / (eid + '_timing.json'))['data']
        for i, key in enumerate(('t_obs', 't_request', 't_ready_host', 't_ready_seen_sim', 't_install', 't_switch')):
            assert timing['host_relative_s'][i] == c[key]['host_monotonic_s'] - c['t_obs']['host_monotonic_s']
            expected = c[key]['sim_time_s'] - c['t_obs']['sim_time_s'] if 'sim_time_s' in c[key] else None
            assert timing['simulation_relative_s'][i] == expected
    zip_parity(parent)
    manifest = read(parent / 'source_bundle/manifest.json')
    assert len(manifest['entries']) == o['qualified_count']
    final = read(parent / 'validation_final.json')
    assert final['summary_sha256'] == sha(parent / 'aggregate/summary.json')
    assert final['primary_saved_validation_sha256'] == sha(parent / 'phaseB/validation.json')
    calls = s['calls']
    assert calls['technical_predictions'] == t['calls']['technical_predictions']
    assert calls['scientific_online_predictions'] == sum(r['calls']['terminal_predictions'] for r in o['rows'])
    assert calls['MPC_accepted'] == t['calls']['MPC_submissions'] + sum(r['calls']['MPC_submissions'] for r in o['rows'])
    assert calls['MPC_saved'] == t['calls']['MPC_saved_results'] + sum(r['calls']['MPC_saved_results'] for r in o['rows'])
    return dict(valid=True, saved_episode_recomputation=True, raw_world_plot_parity=True,
                CSV_JSON_parity=True, source_hashes_preserved=True, ZIP_byte_parity=True,
                classification=o['classification'], qualified_count=o['qualified_count'],
                real_model_MPC_optimizer_calls=0, validator_sha256=sha(__file__),
                source_validation_sha256=sha(parent / 'phaseB/validation.json'),
                summary_sha256=sha(parent / 'aggregate/summary.json'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = validate(args.run.resolve())
    if args.output:
        save(args.output, result)  # Exclusive creation; existing evidence is immutable.
    print(json.dumps(result, indent=2))
