#!/usr/bin/env python3
"""Prepare, freeze and evaluate saved-only data; never start a controller."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
import yaml
from reconciliation.join_source03 import read, save, sha
from reconciliation.spatial_correspondence_selector import (
    METHODS, RULES, SOURCE_IDS, SpatialCurve, official_selector, diagnostic_guard,
    matched_queries, reset_summary, classification)

CONFIG = ROOT/'configs/spatial_correspondence_selector_diag_01.yaml'
RESULTS = ROOT/'results/spatial_correspondence_selector_diag_01'
DOC = ROOT/'docs/SPATIAL_CORRESPONDENCE_SELECTOR_DIAG_01.md'
NEW_FILES = [CONFIG, ROOT/'src/reconciliation/spatial_correspondence_selector.py',
             *[ROOT/'scripts'/f'{s}_spatial_correspondence_selector_diag01.py' for s in ['run', 'validate', 'report']],
             ROOT/'tests/test_spatial_correspondence_selector_diag01.py']


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def authenticate(cfg, deep=False):
    """Complete historical result ledgers and frozen source/code chains, read-only."""
    from run_state_shift_transport_scale01 import verify as verify_transport
    results = []
    for file, expected in cfg['historical_results'].items():
        assert sha(ROOT/file) == expected, file
        r = read(ROOT/file)
        run = Path(r['run'])
        assert sha(run/'result_hashes.json') == r['result_hashes_sha256']
        for path, checksum in read(run/'result_hashes.json').items():
            assert sha(run/path) == checksum, path
        assert r['validation']['valid'] and r['summary'] == read(run/'summary.json')
        results.append(r)
    transport = results[0]
    verify_transport(Path(transport['run']))
    if deep:
        from validate_state_shift_transport_scale01 import validate
        summary, validation = validate(Path(transport['run']))
        assert validation['valid'] and summary == transport['summary']
    assert cfg['sources'] == SOURCE_IDS and cfg['methods'] == METHODS and cfg['rules'] == RULES
    assert [s['id'] for s in transport['summary']['selected_sources']] == SOURCE_IDS
    return transport


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text())
    transport = authenticate(cfg, deep=True)
    from validate_state_shift_transport_scale01 import method_folder
    selector, metadata = official_selector(cfg['official_source'])  # compile only; zero queries
    del selector
    selected = []
    inputs = {}
    def register(path):
        path = Path(path).resolve()
        inputs[str(path)] = sha(path)
        return str(path)
    for result_path in cfg['historical_results']:
        r = read(register(ROOT/result_path))
        register(Path(r['run'])/'result_hashes.json')
    for spec in transport['summary']['selected_sources']:
        folder = Path(spec['folder'])
        common = read(register(folder/'common_state.json'))
        refs = read(register(folder/'references.json'))
        manifest = read(register(folder/'source_manifest.json'))
        register(folder/'recorded_old_to_B.npy')
        register(folder/'schedule.json')
        register(folder/'scenario.json')
        saved_refs = {}
        for method in METHODS:
            ref = refs[method]
            for frame in ['world', 'local']:
                assert sha(register(ref[frame+'_path'])) == ref[frame+'_sha256']
            out = method_folder(folder, method)
            restoration = read(register(out/'restoration.json'))
            register(out/'rollout.json')
            register(out/'metrics.json')
            world = np.load(ref['world_path'])
            installed = np.asarray(restoration['installed_world'])
            np.testing.assert_allclose(installed, world, rtol=0, atol=cfg['saved_numeric_tolerance'])
            assert restoration['B'] == common['B'] and restoration['capture_pose'] == common['fresh_capture_pose']
            saved_refs[method] = dict(record=ref, method_folder=str(out), installed_world=installed.tolist(),
                                     installed_vs_saved_max_abs=float(np.max(np.abs(installed-world))),
                                     restoration_path=str(out/'restoration.json'))
        native = read(Path(saved_refs[METHODS[0]]['method_folder'])/'rollout.json')
        states = {s['absolute_tick']: s for s in native['states']}
        matches = []
        for request in native['submit_requests']:
            tick = request['tick']
            if common['B_tick'] <= tick <= common['B_tick']+cfg['primary_intervals']:
                state = states[tick]
                assert request['pose_world'] == state['pose_world'] and request['sim_time_s'] == state['sim_time_s']
                matches.append(dict(request, time_after_B_s=state['time_s']))
        assert [s['tick'] for s in matches] == read(folder/'schedule.json')['primary']['attempted_submit_ticks']
        assert refs[METHODS[0]]['local_sha256'] == manifest['FRESH_sha256']
        selected.append(dict(source_id=spec['id'], folder=str(folder), common=common,
                             original_FRESH_sha256=manifest['FRESH_sha256'], references=saved_refs,
                             matched_states=matches, original_world=refs[METHODS[0]]['world_path'],
                             old_path=str(folder/'recorded_old_to_B.npy')))
    assert sum(len(s['matched_states']) for s in selected) == cfg['expected_matched_states']
    run.mkdir(parents=True, exist_ok=False)
    save(run/'protocol.json', cfg)
    save(run/'prepared.json', dict(sources=selected, selector=metadata,
                                  historical_inputs=inputs, historical_validators_passed=True))
    print(json.dumps(dict(prepared=str(run), matched_states=37, scientific_queries=0,
                          historical_validators='transport, multisource, R00, R01 passed')))


def freeze(run):
    from run_state_shift_transport_scale01 import FILES as historical_code
    with (run/'protocol_document.md').open('xb') as f:
        f.write(DOC.read_bytes())
    prepared = read(run/'prepared.json')
    frozen = dict(files={str(p): sha(p) for p in dict.fromkeys(list(historical_code)+NEW_FILES)},
                  inputs={str(p): sha(p) for p in run.rglob('*') if p.is_file()},
                  historical_inputs=prepared['historical_inputs'])
    save(run/'freeze.json', frozen)
    save(RESULTS/'freeze_summary.json', dict(run=str(run), freeze=frozen,
        selector=prepared['selector'], matched_states={s['source_id']: s['matched_states'] for s in prepared['sources']},
        references={s['source_id']: {n:dict(world_sha256=r['record']['world_sha256'],
                     local_sha256=r['record']['local_sha256'], restoration_sha256=sha(r['restoration_path']),
                     installed_vs_saved_max_abs=r['installed_vs_saved_max_abs'])
                     for n, r in s['references'].items()} for s in prepared['sources']},
        historical_validators_passed=True, expected_selector_queries=111))


def verify(run, pushed=False):
    frozen = read(run/'freeze.json')
    for group in ['files', 'inputs', 'historical_inputs']:
        for path, checksum in frozen[group].items():
            assert sha(path) == checksum, path
    cfg = read(run/'protocol.json')
    assert sha(cfg['official_source']) == read(run/'prepared.json')['selector']['source_sha256']
    if pushed:
        assert git('rev-parse', 'HEAD') == git('ls-remote', 'origin', 'refs/heads/main').split()[0]
        for path in frozen['files']:
            assert git('rev-parse', 'HEAD:'+str(Path(path).relative_to(ROOT))) == git('hash-object', path)
        assert git('rev-parse', 'HEAD:'+str(DOC.relative_to(ROOT))) == git('hash-object', str(run/'protocol_document.md'))
        assert not git('diff', 'HEAD', '--', str(RESULTS/'freeze_summary.json'))


def compute_sources(prepared, selector):
    from reconciliation.relative_factor_multisource import geometry
    sources = {}
    for spec in prepared['sources']:
        fresh = np.load(spec['original_world'])
        curve = SpatialCurve(fresh)
        B = spec['common']['B']
        correspondences = curve.correspondences(B)
        environment = geometry(Path(spec['folder']))['on']
        for rule in RULES[1:]:
            correspondences[rule]['safety'] = curve.safety(B, correspondences[rule], environment)
        refs = {n: np.asarray(r['installed_world']) for n, r in spec['references'].items()}
        rows = matched_queries(curve, refs, spec['matched_states'], selector)
        for row in rows:
            provenance = spec['references'][row['method']]
            row['stored_reference_world_sha256'] = provenance['record']['world_sha256']
            row['installed_reference_restoration_sha256'] = sha(provenance['restoration_path'])
        first = {}
        for row in rows[:3]:
            name = row['method']
            record = dict(query_tick=row['tick'], query_time_after_B_s=row['time_after_B_s'],
                          query_pose_world=row['matched_pose_world'], B_pose_world=B)
            for key, identity, physical in [('nearest', row['nearest_row'], row['nearest_physical_world']),
                                            ('H5_start', row['first_row'], row['physical_H5_world'][0])]:
                original = curve.describe(B, curve.arc[identity])
                original.update(original_row_identity=identity, physical_target_world=physical,
                    physical_B_position_gap_m=float(np.linalg.norm(np.asarray(physical)[:2]-np.asarray(B)[:2])),
                    physical_B_yaw_gap_rad=float(abs(np.arctan2(np.sin(physical[2]-B[2]), np.cos(physical[2]-B[2])))))
                record[key] = original
            first[name] = record
        sources[spec['source_id']] = dict(correspondences=correspondences, matched_selector=rows,
            progress_reset=reset_summary(rows), first_official_selection=first,
            A=spec['common']['fresh_capture_pose'], B=B,
            original_arc_m=curve.arc.tolist(), original_FRESH_sha256=spec['original_FRESH_sha256'])
        np.testing.assert_array_equal(fresh, np.load(spec['original_world']))
    return sources


def execute(run):
    verify(run, pushed=True)
    save(run/'execution_start.json', dict(sha=git('rev-parse', 'HEAD'),
        started_UTC=datetime.now(timezone.utc).isoformat(), retries=0))
    cfg = read(run/'protocol.json')
    try:
        prepared = read(run/'prepared.json')
        selector, metadata = official_selector(cfg['official_source'])
        assert metadata == prepared['selector']
        with diagnostic_guard() as counts:
            sources = compute_sources(prepared, selector)
        assert counts['selector_queries'] == cfg['expected_scientific_selector_queries']
        # Deliberately join execution outcomes only after ALL matched-state queries.
        historical = read(ROOT/next(iter(cfg['historical_results'])))['summary']['sources']
        for sid, q in sources.items():
            q['saved_execution_context'] = {n: {k: historical[sid]['primary_metrics'][n][k] for k in
                ['position_auc_09_m_s', 'sustained_attachment_s', 'endpoint_error_m']} for n in METHODS}
            q['saved_Half_minus_Full'] = {k: (None if q['saved_execution_context'][METHODS[1]][k] is None or
                q['saved_execution_context'][METHODS[2]][k] is None else
                q['saved_execution_context'][METHODS[1]][k]-q['saved_execution_context'][METHODS[2]][k])
                for k in q['saved_execution_context'][METHODS[0]]}
        summary = dict(experiment=cfg['experiment'], label=cfg['label'], starting_sha=cfg['starting_sha'],
            scientific_freeze_sha=read(run/'execution_start.json')['sha'], sources=sources,
            counts=counts, classification=classification(sources), optional_Genuine13='not performed')
        verify(run)
        save(run/'summary.json', summary)
        print(json.dumps(dict(classification=summary['classification'], counts=counts)))
    except Exception as exc:
        save(run/'technical_failure.json', dict(classification='TECHNICAL_BLOCKED', error=repr(exc), retries=0))
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True, type=Path)
    p.add_argument('--mode', required=True, choices=['prepare', 'freeze', 'execute'])
    args = p.parse_args()
    globals()[args.mode](args.run.resolve())
