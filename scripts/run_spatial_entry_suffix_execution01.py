#!/usr/bin/env python3
"""One C3 suffix rollout/source; unchanged historical executor and official MPC."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_relative_factor_multisource01 as legacy
from run_osa03_native_continuation import git, plain
from reconciliation.join_source03 import read, save, sha
from reconciliation.robotless_online import stamp
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.spatial_entry_suffix import *

CONFIG = ROOT/'configs/spatial_entry_suffix_execution_01.yaml'
RESULTS = ROOT/'results/spatial_entry_suffix_execution_01'
DOC = ROOT/'docs/SPATIAL_ENTRY_SUFFIX_EXECUTION_01.md'
NEW_FILES = [CONFIG, ROOT/'src/reconciliation/spatial_entry_suffix.py',
    *[ROOT/'scripts'/f'{s}_spatial_entry_suffix_execution01.py' for s in ['run', 'validate', 'report']],
    ROOT/'tests/test_spatial_entry_suffix_execution01.py']


def authenticate(cfg, deep=False):
    records = []
    for path, digest in cfg['historical_results'].items():
        assert sha(ROOT/path) == digest, path
        r = read(ROOT/path)
        old = Path(r['run'])
        assert sha(old/'result_hashes.json') == r['result_hashes_sha256']
        for p, h in read(old/'result_hashes.json').items():
            assert sha(old/p) == h, p
        assert r['summary'] == read(old/'summary.json') and r['validation']['valid']
        records.append(r)
    from run_spatial_correspondence_selector_diag01 import verify as verify_diagnostic
    verify_diagnostic(Path(records[0]['run']))
    if deep:
        from validate_spatial_correspondence_selector_diag01 import validate
        s, v = validate(Path(records[0]['run']))
        assert v['valid'] and s == records[0]['summary']
    assert cfg['sources'] == SOURCE_IDS and cfg['order'] == ORDER
    assert cfg['integration_steps'] == 180 and cfg['primary_intervals'] == 54
    assert cfg['vertex_duplicate_tolerance'] == DUPLICATE_ATOL
    assert cfg['sign_tolerance'] == SIGN_ATOL and cfg['substantially_more_remaining_arc_m'] == MORE_REMAINING_M
    return records[0], records[1]


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text())
    diagnostic, transport = authenticate(cfg, deep=True)
    from validate_state_shift_transport_scale01 import method_folder
    run.mkdir(parents=True, exist_ok=False)
    save(run/'protocol.json', cfg)
    selected = []
    for spec in transport['summary']['selected_sources']:
        old = Path(spec['folder'])
        folder = run/'sources'/spec['id'].replace('/', '__')
        (folder/'references').mkdir(parents=True)
        copies = {}
        for file in ['common_state.json', 'source_manifest.json', 'schedule.json', 'scenario.json',
                     'metric_protocol.json', 'recorded_old_to_B.npy', 'source_audit.json']:
            shutil.copyfile(old/file, folder/file)
            copies[file] = dict(path=str(old/file), sha256=sha(old/file))
        save(folder/'protocol.json', cfg)
        c = read(folder/'common_state.json')
        before = read(old/'references.json')
        fresh = np.load(before[NATIVE]['world_path'])
        raw = np.load(before[NATIVE]['local_path'])
        frozen_c3 = diagnostic['summary']['sources'][spec['id']]['correspondences']['C3_FORWARD_SE2']
        world, local, descriptor = suffix_reference(fresh, raw, c['fresh_capture_pose'], c['B'], frozen_c3)
        ref = {}
        for frame, array in [('world', world), ('local', local)]:
            path = folder/'references'/f'{ENTRY}_{frame}.npy'
            with path.open('xb') as f:
                np.save(f, array, allow_pickle=False)
            ref.update({frame+'_path':str(path), frame+'_sha256':sha(path)})
        safety = reference_safety(world, geometry(folder))
        ref.update(safety=safety, descriptor=descriptor,
                   status='REFERENCE_GEOMETRY_SAFE' if safety['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')
        refs = {NATIVE:before[NATIVE], FULL:before[FULL], ENTRY:ref}
        save(folder/'references.json', refs)
        # Only the new suffix is installed in the zero-solve preflight.
        save(folder/'prepared_references.json', {ENTRY:ref})
        shutil.copyfile(before[NATIVE]['world_path'], folder/'references/M0_NATIVE_world.npy')
        save(folder/'entry.json', dict(**descriptor, historical_C3=frozen_c3, historical_C3_exact=True,
            B_connector_is_not_reference=True, safety=safety,
            eligibility=dict(N_at_least_two=len(world)>=2, positive_remaining_arc=True,
                             well_defined_C3=True, geometry_valid=safety['clearance_valid'])))
        reuse = dict(source_folder=str(old), copies=copies,
            methods={n:dict(folder=str(method_folder(old,n)), reference=before[n],
                            hashes_sha256=sha(method_folder(old,n)/'hashes.json')) for n in [NATIVE, FULL]})
        save(folder/'reuse.json', reuse)
        legacy.prior.preflight(folder)
        pre = read(folder/'restoration_preflight.json')
        assert pre['provenance']['official_settings'] == read(folder/'source_manifest.json')['mpc']['official_settings']
        restored = pre['rows'][0]
        native_installed = np.asarray(read(method_folder(old,NATIVE)/'restoration.json')['installed_world'])
        for j, identity in enumerate(descriptor['row_original_identities']):
            if identity is not None:
                np.testing.assert_array_equal(restored['installed_world'][j], native_installed[identity])
        selected.append(dict(id=spec['id'], folder=str(folder), historical_folder=str(old),
                             raw_sha256=spec['raw_sha256']))
    save(run/'selected_sources.json', selected)
    save(run/'authentication.json', dict(all_five_saved_validators=True, new_numerical_MPC_solves=0,
        new_optimizer_solves=0, exact_C3_matches=4, reused_Native_Full_references=8, reused_rollouts=8,
        arbitrary_N_official_restore_preflight=True))
    print(json.dumps(dict(prepared=str(run), source_ids=SOURCE_IDS, new_scientific_solves=0)))


def freeze(run):
    cfg = yaml.safe_load(CONFIG.read_text())
    assert read(run/'protocol.json') == cfg
    for spec in read(run/'selected_sources.json'):
        assert read(Path(spec['folder'])/'protocol.json') == cfg
    from run_state_shift_transport_scale01 import FILES as transport_files
    from run_spatial_correspondence_selector_diag01 import NEW_FILES as diag_files
    files = list(dict.fromkeys(list(transport_files)+diag_files+NEW_FILES))
    with (run/'protocol_document.md').open('xb') as f:
        f.write(DOC.read_bytes())
    save(run/'freeze.json', dict(files={str(p):sha(p) for p in files},
        inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}, new_rollouts=4, optimizer_calls=0, retries=0))
    save(RESULTS/'freeze_summary.json', dict(run=str(run), freeze=read(run/'freeze.json'),
        authentication=read(run/'authentication.json'), selected_sources=read(run/'selected_sources.json'),
        sources={s['id']:{k:read(Path(s['folder'])/(k+'.json')) for k in ['entry','common_state','schedule','reuse']}
                 for s in read(run/'selected_sources.json')}))


def verify(run, pushed=False):
    authenticate(read(run/'protocol.json'))
    for group in ['files', 'inputs']:
        for p, h in read(run/'freeze.json')[group].items():
            assert sha(p) == h, p
    if pushed:
        assert git('rev-parse','HEAD') == git('ls-remote','origin','refs/heads/main').split()[0]
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT))) == git('hash-object',p)
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT))) == git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def execute(run):
    verify(run, pushed=True)
    save(run/'execution_start.json', dict(sha=git('rev-parse','HEAD'), started=stamp(), retries=0))
    with no_reconciliation_optimizer():
        for spec in read(run/'selected_sources.json'):
            folder = Path(spec['folder'])
            ref = read(folder/'references.json')[ENTRY]
            if ref['safety']['clearance_valid']:
                legacy.run_method(folder, ENTRY)
            else:
                save(folder/'methods'/ENTRY/'skipped.json', dict(status=ref['status'], metrics=None, rollouts=0))
    verify(run)
    save(run/'completion.json', dict(completed=stamp(), new_optimizer_calls=0, no_historical_rollout_rerun=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--mode', choices=['prepare', 'freeze', 'execute'], required=True)
    args = parser.parse_args()
    globals()[args.mode](args.run.resolve())
